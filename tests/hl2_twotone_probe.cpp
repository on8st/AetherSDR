// TWO-TONE IMD / COMB probe — v26.9.4, TXA build.
//
// WHY THIS EXISTS. hl2_lsbam_probe measured a SINGLE tone and found the
// modulator clean (image -320 dBc, carrier -334 dBc on LSB). A single tone
// CANNOT produce intermodulation: it takes two. The operator's symptom —
// recognisable speech with a metallic, "boxy" quality — is the signature of
// either intermodulation or comb filtering, and the single-tone figure is
// blind to both.
//
// WHAT IT REPORTS, per case:
//   f1, f2       the two wanted tones, on the sideband the mode names
//   img1, img2   the same tones on the OPPOSITE sideband (sideband suppression)
//   carrier      DC of the emitted IQ
//   IMD3         2f1-f2 and 2f2-f1, dBc relative to ONE tone (the convention
//                every SSB datasheet uses; PEP is 6 dB above one tone)
//   IMD5         3f1-2f2 and 3f2-2f1
//   residual     EVERYTHING that is not the two wanted tones, as one number.
//                Parseval over the same window. This is the catch-all: it sees
//                spurs at frequencies nobody thought to name, and it sees
//                envelope-rate sidebands from a block-stepped gain.
//   blockmod     the largest sideband at a multiple of the ALC BLOCK RATE
//                (48000/1024 = 46.875 Hz) around either tone. The ALC computes
//                ONE scalar per 512-sample input block and applies it to the
//                whole block, so a gain that moves block-to-block amplitude-
//                modulates the audio at the block rate.
//
// EXACT BINS, NO WINDOW, NO LEAKAGE. Every frequency above is an exact
// multiple of 3.125 Hz, and the analysis window is 15360 output samples =
// lcm(480, 1024) — 480 makes every 100 Hz tone/IMD bin whole-cycle, 1024 makes
// every ALC-block-rate bin whole-cycle. A rectangular window is therefore
// leakage-free for all of them simultaneously and no figure below is a window
// skirt.
//
// WIRE-FACING SIGN, per hl2_txdsp_test's header: USB -> energy at NEGATIVE
// audio offset, LSB -> energy at POSITIVE. The HPSDR wire order has the
// opposite handedness to the analytic convention.
//
// THE INSTRUMENT MUST BE ABLE TO FAIL. Two deliberate defects are injected
// into the AUDIO before it is handed to Hl2TxDsp, and their rows are printed
// beside the real ones:
//   * softclip  tanh() saturation — must move IMD3.
//   * comb      x[n] + x[n-24] (1 ms at 24 kHz) — must notch the frequency
//               response at 500/1500/2500 Hz.
// A clean row is only believable next to those.
//
// NOTHING HERE TOUCHES A RADIO.

#include "core/backends/hl2/Hl2TxDsp.h"
#include "TxTestAuthority.h"

#include <QCoreApplication>
#include <QObject>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

using namespace AetherSDR::hl2;
using AetherSDR::TxAudioSource;

static constexpr double kFsOut = 48000.0;
static constexpr double kFsIn  = 24000.0;
static constexpr double kF1 = 700.0;
static constexpr double kF2 = 1900.0;

// Skip the mute ramp create_slews puts on the front of a TXA channel, plus
// enough for the ALC release (0.5 s) to settle on a steady signal.
static constexpr std::size_t kSettleSamples = 36864;   // 768 ms at 48 kHz
// lcm(480, 1024). 480 = whole cycles of every 100 Hz component at 48 kHz;
// 1024 = one ALC input block (512 at 24 kHz) expressed at the output rate.
static constexpr std::size_t kWindow = 15360;          // 320 ms, 3.125 Hz bins
static constexpr double kBinHz = kFsOut / static_cast<double>(kWindow); // 3.125
// One ALC decision per dspBlockSize=512 input samples = 1024 output samples.
static constexpr double kAlcBlockHz = kFsOut / 1024.0;                  // 46.875

struct Window { std::size_t from; std::size_t count; };

static Window analysisWindow(const std::vector<std::complex<float>>& iq)
{
    if (iq.size() < kSettleSamples + kWindow) return {0, 0};
    // Take the LAST whole window, so anything still settling is behind us.
    return {iq.size() - kWindow, kWindow};
}

// Complex bin amplitude over an explicit window. For a pure complex exponential
// of amplitude A at exactly this frequency this returns A.
static double binAmp(const std::vector<std::complex<float>>& iq,
                     double hz, const Window& w)
{
    if (w.count == 0) return 0.0;
    std::complex<double> acc{0.0, 0.0};
    const double om = -2.0 * M_PI * hz / kFsOut;
    for (std::size_t n = 0; n < w.count; ++n) {
        const double ph = om * static_cast<double>(n);
        acc += std::complex<double>(iq[w.from + n].real(), iq[w.from + n].imag())
             * std::complex<double>(std::cos(ph), std::sin(ph));
    }
    return std::abs(acc) / static_cast<double>(w.count);
}

// BLIND SCAN. binAmp() answers "how much is at the frequency I named", which
// is only as good as the naming — the first cut of this probe named the IMD3
// bins with the wrong sign and read the floor while the soft-clip control was
// visibly destroying the spectrum. This function names nothing: it walks every
// bin of the leakage-free grid over +/-6 kHz and returns the largest ones that
// are not a wanted tone. Incremental phasor rather than a cos/sin per sample,
// renormalised every 1024 steps so the rotation cannot drift into the figures.
struct Spur { double hz; double amp; };

static std::vector<Spur> blindScan(const std::vector<std::complex<float>>& iq,
                                   const Window& w,
                                   const std::vector<double>& excludeHz,
                                   double excludeWidthHz,
                                   std::size_t topN)
{
    std::vector<Spur> found;
    if (w.count == 0) return found;
    const int kLo = -1920;   // -6000 Hz at 3.125 Hz bins
    const int kHi = +1920;
    for (int k = kLo; k <= kHi; ++k) {
        const double hz = k * kBinHz;
        bool skip = false;
        for (double e : excludeHz)
            if (std::fabs(hz - e) <= excludeWidthHz) { skip = true; break; }
        if (skip) continue;
        const double om = -2.0 * M_PI * hz / kFsOut;
        const std::complex<double> step(std::cos(om), std::sin(om));
        std::complex<double> p(1.0, 0.0);
        std::complex<double> acc(0.0, 0.0);
        for (std::size_t n = 0; n < w.count; ++n) {
            const std::complex<float>& z = iq[w.from + n];
            acc += std::complex<double>(z.real(), z.imag()) * p;
            p *= step;
            if ((n & 1023u) == 1023u) p /= std::abs(p);
        }
        found.push_back({hz, std::abs(acc) / static_cast<double>(w.count)});
    }
    std::sort(found.begin(), found.end(),
              [](const Spur& a, const Spur& b) { return a.amp > b.amp; });
    if (found.size() > topN) found.resize(topN);
    return found;
}

static double meanSquare(const std::vector<std::complex<float>>& iq, const Window& w)
{
    if (w.count == 0) return 0.0;
    double acc = 0.0;
    for (std::size_t n = 0; n < w.count; ++n) {
        const std::complex<float>& z = iq[w.from + n];
        acc += static_cast<double>(z.real()) * z.real()
             + static_cast<double>(z.imag()) * z.imag();
    }
    return acc / static_cast<double>(w.count);
}

static double db(double num, double den)
{
    const double eps = 1e-30;
    return 20.0 * std::log10((num + eps) / (den + eps));
}
static double dbPow(double num, double den)
{
    const double eps = 1e-30;
    return 10.0 * std::log10((num + eps) / (den + eps));
}

// ── audio generators and the two injected defects ───────────────────────────

struct AudioSpec {
    std::vector<double> tones;      // Hz
    double amplitude = 0.2;         // per tone
    double envelopeHz = 0.0;        // 0 = steady; >0 = syllabic gate
    double envelopeLow = 0.1;       // depth of the gate's quiet half
    bool softclip = false;          // INJECTED DEFECT: tanh saturation
    double softclipK = 3.0;
    bool comb = false;              // INJECTED DEFECT: x[n] + x[n-combDelay]
    std::size_t combDelay = 24;     // 1 ms at 24 kHz -> nulls at 500/1500/2500
    // IMPULSE mode: silence everywhere except one sample. Measures the
    // modulator's impulse response directly -- a linear-phase FIR rings
    // SYMMETRICALLY, half of it BEFORE the event, which is the textbook
    // mechanism for "metallic" on consonants and is invisible to any steady
    // tone. Overrides tones/envelope when impulseAt != 0.
    long long impulseAt = 0;
    double impulseAmp = 0.5;
};

class AudioGen {
public:
    explicit AudioGen(const AudioSpec& s) : m_s(s), m_hist(4096, 0.0f) {}

    float next()
    {
        if (m_s.impulseAt != 0) {
            const double v = (m_n == m_s.impulseAt) ? m_s.impulseAmp : 0.0;
            ++m_n;
            return static_cast<float>(v);
        }
        double x = 0.0;
        for (double f : m_s.tones)
            x += m_s.amplitude * std::sin(2.0 * M_PI * f * m_n / kFsIn);
        if (m_s.envelopeHz > 0.0) {
            // A RAISED-COSINE syllabic envelope, not a square gate.
            //
            // The first cut gated hard, and a hard gate's OWN spectrum is a
            // 1/n skirt reaching kHz away — every gated row read residual
            // -3 dB including the rows with no nonlinearity in them at all, so
            // the measurement could not tell the gate from the chain. A raised
            // cosine confines the envelope's spectrum to a few multiples of
            // envelopeHz, which leaves everything further out attributable.
            const double ph = 2.0 * M_PI * m_s.envelopeHz * m_n / kFsIn;
            const double lo = m_s.envelopeLow;
            x *= lo + (1.0 - lo) * 0.5 * (1.0 - std::cos(ph));
        }
        if (m_s.softclip)
            x = std::tanh(m_s.softclipK * x) / std::tanh(m_s.softclipK);
        float y = static_cast<float>(x);
        if (m_s.comb) {
            const std::size_t d = m_s.combDelay;
            const float old = m_hist[(m_pos + m_hist.size() - d) % m_hist.size()];
            m_hist[m_pos] = y;
            m_pos = (m_pos + 1) % m_hist.size();
            y = 0.5f * (y + old);
        }
        ++m_n;
        return y;
    }

private:
    AudioSpec m_s;
    std::vector<float> m_hist;
    std::size_t m_pos = 0;
    long long m_n = 0;
};

// The TXA channel is opened with blockForOutput=false and STARVES if fed
// faster than real time; every fault block is counted and reported.
static void feed(Hl2TxDsp& tx, const std::vector<float>& chunk,
                 const AetherSDR::TxCoordinator::Context& ctx)
{
    tx.processAudioBlock(chunk, TxAudioSource::Microphone, ctx);
    std::this_thread::sleep_for(std::chrono::duration<double>(
        static_cast<double>(chunk.size()) / tx.config().inputSampleRateHz));
}

// ── the case ────────────────────────────────────────────────────────────────

struct Case {
    const char* label;
    WdspChannel::Mode mode = WdspChannel::Mode::Lsb;
    double lowHz = 300.0;
    double highHz = 2700.0;
    bool wantedIsUpper = false;     // LSB -> wanted at POSITIVE wire offset
    bool alc = true;
    double micGain = 1.0;
    AudioSpec audio{};
    double seconds = 1.6;
    bool sweepResponse = false;     // print per-tone magnitudes (comb detector)
};

static void runCase(const Case& c)
{
    TxTestAuthority authority;
    Hl2TxDsp tx;
    Hl2TxDsp::Config cfg;
    cfg.mode = c.mode;
    cfg.filterLowHz = c.lowHz;
    cfg.filterHighHz = c.highHz;
    cfg.alcEnabled = c.alc;
    std::string err;
    if (!tx.configure(cfg, &err)) {
        std::fprintf(stdout, "%-52s CONFIGURE REFUSED: %s\n", c.label, err.c_str());
        return;
    }
    tx.setMicGain(c.micGain);

    std::vector<std::complex<float>> cap;
    double lastAlcDb = 0.0;
    QObject::connect(&tx, &Hl2TxDsp::iqReady, &tx,
                     [&cap](const std::vector<std::complex<float>>& iq) {
        cap.insert(cap.end(), iq.begin(), iq.end());
    });
    QObject::connect(&tx, &Hl2TxDsp::alcGain, &tx,
                     [&lastAlcDb](float d) { lastAlcDb = d; });

    AudioGen gen(c.audio);
    constexpr std::size_t kChunk = 240;      // 10 ms at 24 kHz
    const std::size_t total = static_cast<std::size_t>(kFsIn * c.seconds);
    std::vector<float> chunk(kChunk);
    for (std::size_t off = 0; off + kChunk <= total; off += kChunk) {
        for (std::size_t n = 0; n < kChunk; ++n) chunk[n] = gen.next();
        feed(tx, chunk, authority.context);
    }

    const Window w = analysisWindow(cap);
    if (w.count == 0) {
        std::fprintf(stdout, "%-52s NO CAPTURE (%zu samples, need %zu)\n",
                     c.label, cap.size(), kSettleSamples + kWindow);
        return;
    }

    const double sgn = c.wantedIsUpper ? -1.0 : +1.0;   // wire sign of the wanted side
    const double f1 = c.audio.tones.size() > 0 ? c.audio.tones[0] : kF1;
    const double f2 = c.audio.tones.size() > 1 ? c.audio.tones[1] : kF2;

    const double a1 = binAmp(cap, sgn * f1, w);
    const double a2 = binAmp(cap, sgn * f2, w);
    const double i1 = binAmp(cap, -sgn * f1, w);
    const double i2 = binAmp(cap, -sgn * f2, w);
    const double dc = binAmp(cap, 0.0, w);
    const double ref = std::max(a1, a2);                 // ONE tone

    // IMD products, in the AUDIO domain, carried onto the wire with the
    // wanted side's sign. 2f1-f2 is NEGATIVE for 700/1900, which puts it on
    // the far side of the suppressed carrier — exactly where it lands on a
    // real analyser, and exactly where a passband filter cannot remove it.
    // AUDIO-DOMAIN products. Every nonlinearity in Hl2TxDsp -- mic gain into
    // the hard clamp, and the ALC's block-stepped multiplier -- acts on the
    // REAL audio BEFORE modulate(). A real signal has no negative frequencies,
    // so the product at 2f1-f2 = -500 Hz IS a 500 Hz audio tone and lands on
    // the WANTED side at +500, not at -500. The first cut of this probe
    // measured the signed RF-convention bins and read the floor while the
    // residual moved 108 dB on the soft-clip control -- the bins were empty,
    // not the spectrum.
    const double d3lo = binAmp(cap, sgn * std::fabs(2.0 * f1 - f2), w);
    const double d3hi = binAmp(cap, sgn * std::fabs(2.0 * f2 - f1), w);
    const double d5lo = binAmp(cap, sgn * std::fabs(3.0 * f1 - 2.0 * f2), w);
    const double d5hi = binAmp(cap, sgn * std::fabs(3.0 * f2 - 2.0 * f1), w);
    const double d7lo = binAmp(cap, sgn * std::fabs(4.0 * f1 - 3.0 * f2), w);
    const double d7hi = binAmp(cap, sgn * std::fabs(4.0 * f2 - 3.0 * f1), w);

    // EVERYTHING that is not the two wanted tones, by Parseval.
    const double tot = meanSquare(cap, w);
    const double toneP = a1 * a1 + a2 * a2;
    const double residual = std::max(0.0, tot - toneP);

    // The ALC decides once per 512-sample input block = 1024 output samples.
    // A gain that moves block-to-block is an AM at 46.875 Hz and its harmonics.
    double blockmod = 0.0;
    double blockmodHz = 0.0;
    for (int k = 1; k <= 6; ++k) {
        for (double side : {-1.0, +1.0}) {
            for (double tone : {f1, f2}) {
                const double hz = sgn * tone + side * k * kAlcBlockHz;
                const double v = binAmp(cap, hz, w);
                if (v > blockmod) { blockmod = v; blockmodHz = hz; }
            }
        }
    }

    if (!c.sweepResponse)
    std::fprintf(stdout,
        "%-52s | tones %+7.2f/%+7.2f dB | img %+8.2f/%+8.2f | carr %+8.2f | "
        "IMD3 %+8.2f/%+8.2f | IMD5 %+8.2f/%+8.2f | IMD7 %+8.2f/%+8.2f | "
        "resid %+8.2f | blockmod %+8.2f @%+8.1fHz | alc %+6.2f dB | faults %llu/%llu\n",
        c.label,
        db(a1, ref), db(a2, ref),
        db(i1, ref), db(i2, ref), db(dc, ref),
        db(d3lo, ref), db(d3hi, ref),
        db(d5lo, ref), db(d5hi, ref),
        db(d7lo, ref), db(d7hi, ref),
        dbPow(residual, toneP),
        db(blockmod, ref), blockmodHz,
        lastAlcDb,
        tx.modulatorFaultBlocks(), tx.modulatorBlocks());

    // THE BLIND SCAN, printed for every case. Excludes the wanted tones and
    // +/-1 bin of them; everything else in +/-6 kHz is fair game.
    {
        std::vector<double> excl;
        for (double f : c.audio.tones) excl.push_back(sgn * f);
        excl.push_back(0.0);            // DC is reported separately as `carr`
        const std::vector<Spur> spurs = blindScan(cap, w, excl, 1.5 * kBinHz, 6);
        std::fprintf(stdout, "%-52s   worst unnamed bins:", "");
        for (const Spur& sp : spurs)
            std::fprintf(stdout, " %+.1fHz:%+.1f", sp.hz, db(sp.amp, ref));
        std::fprintf(stdout, "\n");
    }

    if (c.sweepResponse) {
        std::fprintf(stdout, "%-52s | multitone sweep: the per-tone response "
                     "below IS the result; the single-row IMD figures above are "
                     "meaningless for a 25/42-tone stimulus and are suppressed. "
                     "faults %llu/%llu\n",
                     c.label, tx.modulatorFaultBlocks(), tx.modulatorBlocks());
        // COMB DETECTOR. Print the magnitude of every fed tone, normalised to
        // the largest. A comb filter from a delayed copy shows PERIODIC deep
        // notches; a plain bandpass shows a monotonic skirt and a flat middle.
        double peak = 0.0;
        std::vector<double> mags;
        mags.reserve(c.audio.tones.size());
        for (double f : c.audio.tones) {
            const double m = binAmp(cap, sgn * f, w);
            mags.push_back(m);
            peak = std::max(peak, m);
        }
        std::fprintf(stdout, "%-52s   response:", "");
        for (std::size_t k = 0; k < mags.size(); ++k)
            std::fprintf(stdout, " %.0f:%+.2f", c.audio.tones[k], db(mags[k], peak));
        std::fprintf(stdout, "\n");
        // One number for the flat middle only (600..2400 Hz), so the filter
        // skirts do not masquerade as ripple.
        double lo = 1e30, hi = -1e30;
        for (std::size_t k = 0; k < mags.size(); ++k) {
            if (c.audio.tones[k] < 600.0 || c.audio.tones[k] > 2400.0) continue;
            lo = std::min(lo, mags[k]);
            hi = std::max(hi, mags[k]);
        }
        if (hi > 0.0)
            std::fprintf(stdout, "%-52s   passband ripple 600..2400 Hz = %.2f dB\n",
                         "", db(hi, lo));
    }
}

// ONSET / RINGING. A single-sample impulse was tried first and read EXACTLY
// zero: a unit impulse is spectrally flat, the shipped bp0 rejects everything
// outside 300..2700 Hz by more than 160 dB (measured in 8a), and what little
// survives is spread over the filter's whole length. The instrument was asking
// the filter to pass the one signal it is built to destroy.
//
// A 1500 Hz TONE THAT STARTS ABRUPTLY asks the same question in-band. What the
// filter does to that onset IS the consonant smearing: the length of the
// transition is the length of the impulse response. v26.9.3's phasing modulator
// is 255 Blackman taps at 48 kHz (~5 ms); TXA's bp0 is 2048 linear-phase
// coefficients (~43 ms). Eight times longer, and LINEAR phase means half of it
// arrives BEFORE the onset that caused it.
//
// Reported: |IQ| in 1 ms RMS buckets around the 50% point, and the -40 dB to
// -3 dB rise time, which is one number that can be compared between builds.
static void runOnset(const char* label, WdspChannel::Mode mode,
                     double lowHz, double highHz, double toneHz)
{
    TxTestAuthority authority;
    Hl2TxDsp tx;
    Hl2TxDsp::Config cfg;
    cfg.mode = mode; cfg.filterLowHz = lowHz; cfg.filterHighHz = highHz;
    cfg.alcEnabled = false;         // a limiter would reshape the very thing
    std::string err;
    if (!tx.configure(cfg, &err)) {
        std::fprintf(stdout, "%-52s CONFIGURE REFUSED: %s\n", label, err.c_str());
        return;
    }
    std::vector<std::complex<float>> cap;
    QObject::connect(&tx, &Hl2TxDsp::iqReady, &tx,
                     [&cap](const std::vector<std::complex<float>>& iq) {
        cap.insert(cap.end(), iq.begin(), iq.end());
    });
    constexpr std::size_t kChunk = 240;
    const std::size_t total = 24000 * 3;                 // 3 s
    const long long onset = 24000 * 2;                   // tone starts at 2.0 s
    std::vector<float> chunk(kChunk);
    long long n = 0;
    for (std::size_t off = 0; off + kChunk <= total; off += kChunk) {
        for (std::size_t k = 0; k < kChunk; ++k, ++n)
            chunk[k] = (n < onset) ? 0.0f
                     : static_cast<float>(0.4 * std::sin(2.0 * M_PI * toneHz * (n - onset) / kFsIn));
        feed(tx, chunk, authority.context);
    }
    if (cap.size() < 120000) {
        std::fprintf(stdout, "%-52s NO CAPTURE (%zu)\n", label, cap.size());
        return;
    }
    // 1 ms RMS buckets over the whole capture.
    const std::size_t bucket = 48;
    const std::size_t nb = cap.size() / bucket;
    std::vector<double> env(nb, 0.0);
    for (std::size_t b = 0; b < nb; ++b) {
        double acc = 0.0;
        for (std::size_t k = 0; k < bucket; ++k) {
            const std::complex<float>& z = cap[b * bucket + k];
            acc += static_cast<double>(z.real()) * z.real()
                 + static_cast<double>(z.imag()) * z.imag();
        }
        env[b] = std::sqrt(acc / static_cast<double>(bucket));
    }
    // Steady level = median of the last 200 ms.
    std::vector<double> tail(env.end() - 200, env.end() - 20);
    std::sort(tail.begin(), tail.end());
    const double steady = tail[tail.size() / 2];
    if (steady <= 0.0) {
        std::fprintf(stdout, "%-52s NO SIGNAL (steady=0)\n", label);
        return;
    }
    // 50% (-6 dB) crossing, searching forward from 1.5 s.
    std::size_t half = 0;
    for (std::size_t b = 1500; b + 1 < nb; ++b)
        if (env[b] >= 0.5 * steady) { half = b; break; }
    // -40 dB and -3 dB crossings around it, for the rise time.
    std::size_t lo40 = half, hi3 = half;
    while (lo40 > 1 && env[lo40 - 1] > 0.01 * steady) --lo40;
    while (hi3 + 1 < nb && env[hi3] < 0.7079 * steady) ++hi3;
    std::fprintf(stdout,
        "%-52s steady=%.4e  50%% at %zu ms  RISE -40dB..-3dB = %zu ms  "
        "(pre-onset lead: 50%% point is %+lld ms after the input edge)\n",
        label, steady, half, hi3 - lo40,
        static_cast<long long>(half) - 2000);
    std::fprintf(stdout, "%-52s   env dB re steady, ms around the 50%% point:\n", "");
    std::fprintf(stdout, "%-52s  ", "");
    int printed = 0;
    for (int ms = -60; ms <= 40; ms += 2) {
        const long long b = static_cast<long long>(half) + ms;
        if (b < 0 || b >= static_cast<long long>(nb)) continue;
        std::fprintf(stdout, " %+d:%.1f", ms, db(env[static_cast<std::size_t>(b)], steady));
        if (++printed % 12 == 0) std::fprintf(stdout, "\n%-52s  ", "");
    }
    std::fprintf(stdout, "\n");
}

static AudioSpec twoTone(double amp)
{
    AudioSpec s;
    s.tones = {kF1, kF2};
    s.amplitude = amp;
    return s;
}

static AudioSpec multiTone()
{
    AudioSpec s;
    for (double f = 300.0; f <= 2700.0; f += 100.0) s.tones.push_back(f);
    s.amplitude = 0.01;     // 25 tones -> worst-case peak 0.25
    return s;
}

// The SKIRTS, not just the passband. The symptom words are "boxy" (too much
// 300-900 Hz) and "metallic" (a hard shoulder at the top), and both are
// statements about the filter EDGES, which the 300..2700 sweep above deliberately
// does not reach past. This one runs 100..4200 Hz so the transition width is a
// measured number rather than an inference from the taps.
static AudioSpec skirtTone()
{
    AudioSpec s;
    for (double f = 100.0; f <= 4200.0; f += 100.0) s.tones.push_back(f);
    s.amplitude = 0.004;    // 42 tones
    return s;
}

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    std::fprintf(stdout, "modulator: %s (AETHER_HL2_TX_TXA=%d)\n",
                 Hl2TxDsp::modulatorName(), AETHER_HL2_TX_TXA);
    std::fprintf(stdout,
        "two-tone %.0f + %.0f Hz, LSB {300,2700}, IQ %.0f Hz, window %zu samples "
        "(%.3f Hz bins), settle %zu, ALC block rate %.3f Hz\n",
        kF1, kF2, kFsOut, kWindow, kBinHz, kSettleSamples, kAlcBlockHz);
    std::fprintf(stdout,
        "all dB are relative to ONE tone. PEP is 6 dB above one tone; a clean "
        "SSB exciter is <= -30 dBc IMD3 and a good one -35..-40 dBc.\n\n");

    // ══ 1. DRIVE SWEEP, ALC ENGAGED vs BYPASSED, same drive ═══════════════
    std::fprintf(stdout, "── 1. DRIVE SWEEP — ALC on vs off at the SAME drive ──\n");
    for (double amp : {0.02, 0.05, 0.1, 0.2, 0.35, 0.425, 0.5}) {
        char lon[96], loff[96];
        std::snprintf(lon,  sizeof lon,  "LSB amp=%.3f (peak %.2f) ALC ON",  amp, 2 * amp);
        std::snprintf(loff, sizeof loff, "LSB amp=%.3f (peak %.2f) ALC off", amp, 2 * amp);
        { Case c{lon};  c.alc = true;  c.audio = twoTone(amp); runCase(c); }
        { Case c{loff}; c.alc = false; c.audio = twoTone(amp); runCase(c); }
    }

    // ══ 2. MIC GAIN SWEEP — the slider now reaches +40 dB (100x) ══════════
    std::fprintf(stdout, "\n── 2. MIC GAIN SWEEP at a fixed quiet drive (amp=0.02) ──\n");
    for (double g : {1.0, 3.16, 10.0, 31.6, 100.0}) {
        char lon[96], loff[96];
        std::snprintf(lon,  sizeof lon,  "LSB micGain=%.2f (%+.0f dB) ALC ON",  g, 20 * std::log10(g));
        std::snprintf(loff, sizeof loff, "LSB micGain=%.2f (%+.0f dB) ALC off", g, 20 * std::log10(g));
        { Case c{lon};  c.alc = true;  c.micGain = g; c.audio = twoTone(0.02); runCase(c); }
        { Case c{loff}; c.alc = false; c.micGain = g; c.audio = twoTone(0.02); runCase(c); }
    }

    // ══ 3. SYLLABIC ENVELOPE — the ALC actually MOVING, which is the only
    //      state in which a block-stepped gain can distort. A steady two-tone
    //      settles to a constant multiplier and hides it.
    std::fprintf(stdout, "\n── 3. SYLLABIC GATE (4 Hz, 20 dB depth) — ALC moving ──\n");
    for (double g : {1.0, 10.0, 100.0}) {
        AudioSpec s = twoTone(0.05);
        s.envelopeHz = 4.0;
        s.envelopeLow = 0.1;
        char lon[96], loff[96];
        std::snprintf(lon,  sizeof lon,  "LSB gated micGain=%.0f ALC ON",  g);
        std::snprintf(loff, sizeof loff, "LSB gated micGain=%.0f ALC off", g);
        { Case c{lon};  c.alc = true;  c.micGain = g; c.audio = s; c.seconds = 2.4; runCase(c); }
        { Case c{loff}; c.alc = false; c.micGain = g; c.audio = s; c.seconds = 2.4; runCase(c); }
    }

    // 3b. A TONE PAIR WHOSE IMD3 *AND* IMD5 LAND INSIDE THE PASSBAND.
    // 700/1900 puts IMD3-high at 3100 Hz, outside {300,2700}, so the TX filter
    // removes it before it can be measured. 1300/1700 puts IMD3 at 900/2100
    // and IMD5 at 500/2500 - all four inside the passband, nothing filtered.
    std::fprintf(stdout, "\n-- 3b. 1300 + 1700 Hz (IMD3 900/2100, IMD5 500/2500, all in band) --\n");
    for (double amp : {0.05, 0.25, 0.5}) {
        for (bool alcOn : {true, false}) {
            AudioSpec s2; s2.tones = {1300.0, 1700.0}; s2.amplitude = amp;
            char lbl[96];
            std::snprintf(lbl, sizeof lbl, "LSB 1300/1700 amp=%.2f ALC %s",
                          amp, alcOn ? "ON" : "off");
            Case c{lbl}; c.alc = alcOn; c.audio = s2; runCase(c);
        }
    }
    { AudioSpec s2; s2.tones = {1300.0, 1700.0}; s2.amplitude = 0.05;
      s2.softclip = true; s2.softclipK = 3.0;
      Case c{"CONTROL 1300/1700 softclip tanh(3x) ALC off"};
      c.alc = false; c.audio = s2; runCase(c); }
    { AudioSpec s2; s2.tones = {1300.0, 1700.0}; s2.amplitude = 0.02;
      Case c{"CONTROL 1300/1700 micGain=100 ALC off (clamp)"};
      c.alc = false; c.micGain = 100.0; c.audio = s2; runCase(c); }

    // ══ 4. USB and DIGL, so the answer is not an LSB-only artefact ════════
    std::fprintf(stdout, "\n── 4. OTHER MODES at amp=0.2 ──\n");
    { Case c{"USB {300,2700} amp=0.2 ALC ON"};
      c.mode = WdspChannel::Mode::Usb; c.wantedIsUpper = true;
      c.audio = twoTone(0.2); runCase(c); }
    { Case c{"DIGL {150,3000} amp=0.2 ALC ON"};
      c.mode = WdspChannel::Mode::Digl; c.lowHz = 150; c.highHz = 3000;
      c.audio = twoTone(0.2); runCase(c); }

    // ══ 5. FREQUENCY RESPONSE — the comb detector ═════════════════════════
    std::fprintf(stdout, "\n── 5. FREQUENCY RESPONSE (25 tones, 300..2700 Hz) ──\n");
    { Case c{"LSB multitone ALC off (response)"};
      c.alc = false; c.audio = multiTone(); c.sweepResponse = true; c.seconds = 2.0;
      runCase(c); }

    // ══ 6. CONTROLS — THE INSTRUMENT MUST MOVE ════════════════════════════
    std::fprintf(stdout, "\n── 6. CONTROLS: injected defects in the AUDIO, ALC off ──\n");
    { AudioSpec s = twoTone(0.2); s.softclip = true; s.softclipK = 3.0;
      Case c{"CONTROL softclip tanh(3x) amp=0.2 ALC off"};
      c.alc = false; c.audio = s; runCase(c); }
    { AudioSpec s = twoTone(0.2); s.softclip = true; s.softclipK = 8.0;
      Case c{"CONTROL softclip tanh(8x) amp=0.2 ALC off"};
      c.alc = false; c.audio = s; runCase(c); }
    { AudioSpec s = multiTone(); s.comb = true; s.combDelay = 24;
      Case c{"CONTROL comb x[n]+x[n-24] (1 ms) multitone"};
      c.alc = false; c.audio = s; c.sweepResponse = true; c.seconds = 2.0;
      runCase(c); }
    { AudioSpec s = twoTone(0.2); s.comb = true; s.combDelay = 24;
      Case c{"CONTROL comb x[n]+x[n-24] (1 ms) two-tone"};
      c.alc = false; c.audio = s; runCase(c); }

    // ══ 7. THE ALC ACTUALLY MOVING, on a LEAKAGE-FREE envelope ═══════
    //
    // Section 3's envelope ran at 4 Hz, which is NOT a multiple of the 3.125 Hz
    // bin spacing, so the envelope's own two sidebands smeared across the grid
    // and read -15.9 dBc in rows that contain no nonlinearity at all (micGain=1,
    // ALC off). That measured the window, not the chain.
    //
    // 3.125 Hz is EXACTLY one bin. A raised-cosine envelope at that rate puts
    // its entire spectrum in tone+/-1 bin and tone+/-2 bins, and everything
    // else in the window is attributable to the chain. This is the state the
    // symptom needs: a leveller that is linear when idle and compressing on
    // speech peaks. `blockmod` is the number to watch -- the ALC recomputes one
    // scalar per 512-sample block and holds it, so a gain that MOVES between
    // blocks amplitude-modulates the audio at 46.875 Hz and its harmonics.
    std::fprintf(stdout, "\n-- 7. ALC MOVING: raised-cosine envelope at 3.125 Hz (exactly 1 bin) --\n");
    for (double g : {1.0, 10.0, 31.6, 100.0}) {
        for (bool alcOn : {true, false}) {
            AudioSpec s7; s7.tones = {kF1, kF2}; s7.amplitude = 0.05;
            s7.envelopeHz = 3.125; s7.envelopeLow = 0.1;
            char lbl[96];
            std::snprintf(lbl, sizeof lbl, "LSB env3.125 micGain=%.1f ALC %s",
                          g, alcOn ? "ON" : "off");
            Case c{lbl}; c.alc = alcOn; c.micGain = g; c.audio = s7;
            c.seconds = 2.8; runCase(c);
        }
    }
    // The same envelope on the in-band tone pair, so IMD3 and IMD5 are both
    // measurable rather than half-filtered.
    for (double g : {1.0, 100.0}) {
        for (bool alcOn : {true, false}) {
            AudioSpec s7; s7.tones = {1300.0, 1700.0}; s7.amplitude = 0.05;
            s7.envelopeHz = 3.125; s7.envelopeLow = 0.1;
            char lbl[96];
            std::snprintf(lbl, sizeof lbl, "LSB 1300/1700 env3.125 micGain=%.1f ALC %s",
                          g, alcOn ? "ON" : "off");
            Case c{lbl}; c.alc = alcOn; c.micGain = g; c.audio = s7;
            c.seconds = 2.8; runCase(c);
        }
    }

    // ══ 8. THE FILTER EDGES AND THE IMPULSE RESPONSE ══════════════════
    //
    // v26.9.3 shipped ONLY the in-tree phasing modulator: 255 Blackman taps at
    // 48 kHz. v26.9.4 ships WDSP TXA, whose bp0 is 2048 linear-phase
    // coefficients. Those are different filters by a factor of eight in length,
    // and the passband sweep above cannot see it because both are flat in the
    // middle. The EDGES and the RINGING are where an eightfold change in length
    // lives, and both are exactly what "boxy" and "metallic" describe.
    std::fprintf(stdout, "\n-- 8a. SKIRTS: 42 tones, 100..4200 Hz, LSB {300,2700}, ALC off --\n");
    { Case c{"LSB skirt sweep ALC off"};
      c.alc = false; c.audio = skirtTone(); c.sweepResponse = true; c.seconds = 2.4;
      runCase(c); }
    std::fprintf(stdout, "\n-- 8b. ONSET / RINGING (consonant smearing is the metallic mechanism) --\n");
    runOnset("LSB {300,2700} onset 1500 Hz", WdspChannel::Mode::Lsb, 300, 2700, 1500.0);
    runOnset("LSB {300,2700} onset 2500 Hz", WdspChannel::Mode::Lsb, 300, 2700, 2500.0);
    runOnset("DIGL {150,3000} onset 1500 Hz", WdspChannel::Mode::Digl, 150, 3000, 1500.0);

    std::fprintf(stdout, "\nprobe done\n");
    return 0;
}
