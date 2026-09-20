// aetherd HL2 Phase 1a — Hl2RxDsp integration test. Pushes a synthetic +1 kHz
// IQ tone through the full RX DSP as 126-sample EP6-shaped blocks and checks the
// three outputs wire up: the panadapter peaks on the positive-frequency side
// (not DC), the WdspChannel demod produces non-silent audio at the right block
// size, and the S-meter tracks it. Exercises the 126 -> 1024 block buffering.

#include "core/backends/hl2/Hl2RxDsp.h"
#include "core/backends/hl2/MetisProtocol.h"   // kEp6BlockSamples (EP6 block size)

#include <QCoreApplication>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <span>
#include <vector>

using namespace AetherSDR::hl2;

static int g_failures = 0;
static void check(bool cond, const char* what)
{
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++g_failures;
    }
}

static constexpr double kPi = 3.14159265358979323846;

// ── The measured offset between what is DRAWN and what is HEARD ──────────
//
// Hl2RxDsp::processIqBlock() feeds Hl2Spectrum from the conjugated wire and
// WdspChannel from the raw wire, and only the second passes through the RX
// filter chain — so a signal is on the panadapter before it is in the
// headphones, by the chain's group delay. Hl2RxDsp::setPanadapterAudioAlignment()
// is the mechanism that can close that; this measures whether it DOES.
//
// MEASURED, not configured. An alignment that is armed and not achieved is the
// failure this exists to catch, so nothing below reads a delay out of the
// object and calls that the answer: the tone's arrival is found independently
// on each path and the two positions are subtracted. The one number taken from
// the object is the delay it claims to hold, and it is used only to check that
// the display moved by exactly that much — never as the definition of aligned.
struct Skew {
    bool ok = false;
    long panOnsetInput = -1;     // input samples fed when the tone first drew
    long audioOnsetInput = -1;   // ... and when it was first audible, same units
    std::size_t delaySamples = 0;    // what the object says it is holding back
    double reportedSkewMs = 0.0;     // what panadapterAudioSkewMs() claims
    [[nodiscard]] long measuredSkewInput() const   // + means audio lags the display
    {
        return audioOnsetInput - panOnsetInput;
    }
};

static constexpr int kSkewRateHz = 48000;
static constexpr int kSkewAudioHz = 24000;
static constexpr int kSkewFftSize = 1024;
static constexpr int kSkewSilence = 24000;   // 0.5 s of nothing, then the tone
static constexpr int kSkewTone = 36000;      // 0.75 s of it
static constexpr double kSkewToneHz = 1000.0;

// ── Is the group delay the mechanism is parameterised on the REAL one? ──
//
// WdspChannel::filterGroupDelaySamples() is arithmetic — (taps - 1) / 2 per
// core in series, from fir_bandpass()'s linear-phase centre. The alignment is
// only as good as that number, so it is measured here rather than believed.
//
// THE METHOD, and why it is a difference rather than an absolute. A tone burst
// through a linear-phase FIR has a step-shaped envelope whose 50 % point IS the
// group delay, because the impulse response is symmetric about its centre. The
// ABSOLUTE onset also carries every other delay in the chain — this class's
// input buffering, WDSP's i/o buffering, the output resampler — none of which
// WdspChannel reports. Measuring the SAME chain at two tap counts and
// subtracting cancels all of them: the only thing that differs is the filter.
//
// AGC OFF for this one. Under AGC the envelope overshoots at onset and is then
// pulled back, so there is no settled level for a 50 % point to be 50 % OF, and
// the crossing moves with the input amplitude instead of with the filter.
// 48 kHz in, DSP and out, so one output sample is one input sample and the
// result is in the same units as the group delay itself.
static long measureFilterOnsetSamples(int taps, std::string* err)
{
    WdspChannel::Config wc;
    wc.direction = WdspChannel::Direction::Receive;
    wc.inputBlockSize = 1024;
    wc.dspBlockSize = 1024;
    wc.inputSampleRate = 48000;
    wc.dspSampleRate = 48000;
    wc.outputSampleRate = 48000;
    wc.mode = WdspChannel::Mode::Usb;
    wc.filterLowHz = 150.0;
    wc.filterHighHz = 3000.0;
    wc.agcMode = 0;            // off — see above
    wc.filterTaps = taps;
    wc.blockForOutput = true;
    auto ch = WdspChannel::create(wc, err);
    if (!ch)
        return -1;

    const std::size_t inN = ch->config().inputBlockSize;
    const std::size_t outN = ch->outputBlockSize();
    std::vector<float> bi(inN), bq(inN), bl(outN), br(outN);
    std::vector<float> env;                    // |left| against input position
    const long silence = 24000;
    const long total = silence + 48000;        // 1.0 s of tone: plenty to settle
    long n = 0;
    for (long b = 0; n < total; ++b) {
        for (std::size_t k = 0; k < inN; ++k, ++n) {
            if (n < silence) { bi[k] = 0.0f; bq[k] = 0.0f; continue; }
            const double ph = 2.0 * kPi * 1000.0 * static_cast<double>(n - silence) / 48000.0;
            bi[k] = 0.1f * static_cast<float>(std::cos(ph));
            bq[k] = 0.1f * static_cast<float>(-std::sin(ph));
        }
        // Position is the count of blocks CONSUMED, including the ones WDSP
        // produced no output for while its pipeline filled — an index into the
        // emitted audio would silently drop those and shift the whole ruler.
        const auto res = ch->processIq(bi, bq, bl, br);
        env.resize(static_cast<std::size_t>(b + 1) * outN, 0.0f);
        if (res != WdspChannel::ProcessResult::Ok)
            continue;
        for (std::size_t k = 0; k < outN; ++k)
            env[static_cast<std::size_t>(b) * outN + k] = std::abs(bl[k]);
    }

    // Settled level from the last 4096 output samples, which are a long way
    // past any filter of this length.
    double settled = 0.0;
    const std::size_t tail = std::min<std::size_t>(4096, env.size());
    for (std::size_t k = env.size() - tail; k < env.size(); ++k)
        settled = std::max(settled, static_cast<double>(env[k]));
    if (!(settled > 0.0)) {
        if (err) *err = "no settled audio at this tap count";
        return -1;
    }
    for (std::size_t k = 0; k < env.size(); ++k) {
        if (env[k] >= 0.5 * settled)
            return static_cast<long>(k) - silence;
    }
    if (err) *err = "envelope never reached half the settled level";
    return -1;
}

static Skew measureSkew(bool align, WdspChannel::Mode mode)
{
    Skew s;
    Hl2RxDsp dsp;
    Hl2RxDsp::Config cfg;
    cfg.inputSampleRateHz = kSkewRateHz;
    cfg.audioSampleRateHz = kSkewAudioHz;   // production: AudioEngine's native rate
    cfg.dspBlockSize = 1024;
    cfg.fftSize = kSkewFftSize;
    cfg.mode = mode;
    cfg.filterLowHz = 150.0;
    cfg.filterHighHz = 3000.0;
    cfg.blockForOutput = true;   // deterministic for an offline burst feed
    std::string err;
    if (!dsp.configure(cfg, &err)) {
        std::fprintf(stderr, "FAIL: skew fixture did not configure: %s\n", err.c_str());
        return s;
    }
    if (align)
        dsp.setPanadapterAudioAlignment(align);
    s.delaySamples = dsp.panadapterAlignmentDelaySamples();
    s.reportedSkewMs = dsp.panadapterAudioSkewMs();

    // ONE RULER FOR BOTH PATHS: input samples fed. Every emit below happens
    // synchronously inside processIqBlock(), so "fed" at the moment of an emit
    // is the input position that produced it — for the spectrum and the audio
    // alike.
    //
    // NOT the emitted audio-frame count, which was the first ruler here and is
    // wrong: processIqBlock() `continue`s past a non-Ok processIq() without
    // emitting, so the startup underruns while WDSP's pipeline fills shift the
    // audio stream against the input stream by an amount nothing reports. An
    // index into the emitted audio is therefore not a position in the input.
    long fed = 0;

    QObject::connect(&dsp, &Hl2RxDsp::spectrumReady, &dsp,
                     [&](const std::vector<float>& bins) {
        if (s.panOnsetInput >= 0 || bins.empty())
            return;
        // Hl2Spectrum fftshifts, so DC is the middle bin and the tone — which
        // is ABOVE centre and reaches the spectrum conjugated into the analytic
        // convention — sits that many bins above it.
        const int n = static_cast<int>(bins.size());
        const int k = n / 2 + static_cast<int>(kSkewToneHz) * n / kSkewRateHz;
        float peak = -400.0f;
        for (int i = k - 2; i <= k + 2; ++i) {
            if (i >= 0 && i < n)
                peak = std::max(peak, bins[i]);
        }
        // Silence transforms to about -240 dBFS and the tone to about -11, so
        // this threshold is nowhere near either edge.
        if (peak > -60.0f)
            s.panOnsetInput = fed;
    });
    QObject::connect(&dsp, &Hl2RxDsp::audioReady, &dsp,
                     [&](const std::vector<float>& pcm) {
        if (s.audioOnsetInput >= 0)
            return;
        for (std::size_t k = 0; k < pcm.size(); k += 2) {
            if (std::abs(pcm[k]) > 0.02f) {
                s.audioOnsetInput = fed;
                return;
            }
        }
    });

    std::vector<std::complex<float>> blk;
    blk.reserve(kEp6BlockSamples);
    const int total = kSkewSilence + kSkewTone;
    for (int n = 0; n < total; ++n) {
        std::complex<float> v {0.0f, 0.0f};
        if (n >= kSkewSilence) {
            // WIRE ORDER — the negative sine. See hl2_rxdsp_rate_test.cpp: the
            // HPSDR wire is the conjugate of the analytic convention, so a tone
            // above centre arrives as exp(-j.2.pi.f.t) and the textbook exp(+j)
            // is out of a USB passband.
            const double ph = 2.0 * kPi * kSkewToneHz * (n - kSkewSilence) / kSkewRateHz;
            v = {0.3f * static_cast<float>(std::cos(ph)),
                 0.3f * static_cast<float>(-std::sin(ph))};
        }
        blk.push_back(v);
        if (static_cast<int>(blk.size()) == kEp6BlockSamples) {
            // Counted BEFORE the call: the signals fire inside it, and "fed"
            // must already include the block that carried the tone.
            fed += static_cast<long>(blk.size());
            dsp.processIqBlock(blk);
            blk.clear();
        }
    }
    if (!blk.empty()) {
        fed += static_cast<long>(blk.size());
        dsp.processIqBlock(blk);
    }
    s.ok = (s.panOnsetInput >= 0 && s.audioOnsetInput >= 0);
    if (!s.ok) {
        std::fprintf(stderr,
                     "FAIL: skew fixture saw no onset (pan %ld, audio %ld)\n",
                     s.panOnsetInput, s.audioOnsetInput);
    }
    return s;
}

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);

    Hl2RxDsp dsp;
    Hl2RxDsp::Config cfg;
    cfg.inputSampleRateHz = 48000;
    cfg.audioSampleRateHz = 48000;
    cfg.dspBlockSize = 1024;
    cfg.fftSize = 256;
    cfg.mode = WdspChannel::Mode::Usb;
    cfg.blockForOutput = true;   // deterministic audio for this offline burst feed
    std::string err;

    // A zero input sample rate — as a missing/malformed "sampleRateHz" params
    // override decodes to (QVariant::toInt) — must be rejected at the boundary,
    // not divide-by-zero in the dsp_size computation (#4448). Guard runs before
    // any state is touched, so the good configure below still succeeds.
    {
        Hl2RxDsp guardDsp;
        Hl2RxDsp::Config bad = cfg;
        bad.inputSampleRateHz = 0;
        std::string guardErr;
        check(!guardDsp.configure(bad, &guardErr),
              "zero inputSampleRateHz is rejected, not a divide-by-zero");
        check(!guardErr.empty(), "rejected configure reports an error string");
    }

    check(dsp.configure(cfg, &err), err.empty() ? "Hl2RxDsp configures" : err.c_str());

    int audioCount = 0, specCount = 0;
    std::size_t lastAudioSize = 0;
    float maxMeter = -300.0f;
    std::vector<float> lastBins;
    QObject::connect(&dsp, &Hl2RxDsp::audioReady, &dsp,
                     [&](const std::vector<float>& pcm) { ++audioCount; lastAudioSize = pcm.size(); });
    float lastMeter = -300.0f;
    QObject::connect(&dsp, &Hl2RxDsp::meterUpdate, &dsp,
                     [&](float dbfs) { if (dbfs > maxMeter) maxMeter = dbfs; lastMeter = dbfs; });
    QObject::connect(&dsp, &Hl2RxDsp::spectrumReady, &dsp,
                     [&](const std::vector<float>& bins) { ++specCount; lastBins = bins; });

    // Feed a signal 1 kHz ABOVE centre, IN WIRE ORDER — note the NEGATIVE sine.
    //
    // The HPSDR wire is the conjugate of the analytic convention, so a signal
    // above centre arrives as exp(-j.2.pi.f.t). This generator emitted the
    // textbook exp(+j...) — synthetic IQ no HL2 ever sends — and both assertions
    // below passed against a chain that mirrored the panadapter and inverted
    // every demodulated sideband.
    const int fs = 48000;
    const double f = 1000.0;
    // ~0.5 s, and NOT SHORTER: Hl2RxDsp holds the S-meter tap for a settle
    // window after a channel install (~15 blocks of 1024 at 48 kHz, ~0.3 s), so
    // the first blocks of this feed publish nothing and the meter assertions
    // below rest on the ~8 emissions that follow. Trim this and they fail with
    // maxMeter still at its sentinel, looking like a dead demodulator.
    const int total = fs / 2;
    std::vector<std::complex<float>> stream(static_cast<std::size_t>(total));
    for (int n = 0; n < total; ++n) {
        const double ph = 2.0 * kPi * f * n / fs;
        stream[static_cast<std::size_t>(n)] =
            0.3f * std::complex<float>(static_cast<float>(std::cos(ph)),
                                       static_cast<float>(-std::sin(ph)));
    }
    std::span<const std::complex<float>> s(stream);
    for (std::size_t off = 0; off < s.size(); off += kEp6BlockSamples) {
        const std::size_t n = std::min<std::size_t>(kEp6BlockSamples, s.size() - off);
        const auto sub = s.subspan(off, n);
        dsp.processIqBlock(std::vector<std::complex<float>>(sub.begin(), sub.end()));
    }

    // ---- panadapter ----
    check(specCount > 0, "spectrumReady fired");
    check(lastBins.size() == 256, "spectrum has fftSize bins");
    if (lastBins.size() == 256) {
        int peak = 0;
        for (int i = 1; i < 256; ++i)
            if (lastBins[static_cast<std::size_t>(i)] > lastBins[static_cast<std::size_t>(peak)]) peak = i;
        check(peak > 128, "+1 kHz tone peaks on the positive-frequency side (above centre)");
        check(lastBins[static_cast<std::size_t>(peak)] > lastBins[128] + 15.0f,
              "tone peak stands well above the DC (centre) bin");
    }

    // ---- audio ----
    check(audioCount >= 5, "audioReady fired for the buffered DSP blocks");
    check(lastAudioSize == static_cast<std::size_t>(cfg.dspBlockSize) * 2,
          "audio block is interleaved stereo of outputBlockSize");
    check(maxMeter > -60.0f, "demod produced non-silent audio (S-meter above floor)");

    // ---- the S-meter must track SIGNAL STRENGTH, not the AGC's output ----
    //
    // This is the assertion an audio-RMS meter cannot pass. Holding the audio
    // level constant is exactly what the AGC does, so a meter derived from
    // demodulated audio barely moves between a strong signal and a weak one —
    // it deflects, which is why it looked like it worked. Feed the same tone
    // 40 dB down and require the meter to follow it down.
    {
        // RXA_S_PK is a PEAK detector with decay, so compare settled values —
        // the maximum during the weak run is just the decay from the strong one.
        const float strongMeter = lastMeter;
        const double weakAmp = 0.3 * 0.01;          // -40 dB
        for (int n = 0; n < total; ++n) {
            const double ph = 2.0 * kPi * f * n / fs;
            // Wire order, same convention as the strong tone above.
            stream[static_cast<std::size_t>(n)] =
                std::complex<float>(static_cast<float>(weakAmp * std::cos(ph)),
                                    static_cast<float>(-weakAmp * std::sin(ph)));
        }
        // Feed several seconds: RXA_S_PK decays rather than jumping, so a short
        // burst measures the decay slope instead of the settled level.
        std::span<const std::complex<float>> w(stream);
        for (int pass = 0; pass < 6; ++pass) {
            for (std::size_t off = 0; off < w.size(); off += kEp6BlockSamples) {
                const std::size_t n = std::min<std::size_t>(kEp6BlockSamples, w.size() - off);
                const auto sub = w.subspan(off, n);
                dsp.processIqBlock(std::vector<std::complex<float>>(sub.begin(), sub.end()));
            }
        }
        const float drop = strongMeter - lastMeter;
        std::fprintf(stderr, "S-meter: strong %.1f, weak %.1f, drop %.1f dB (input -40 dB)\n",
                     strongMeter, lastMeter, drop);
        // A 40 dB input drop must move the meter by at least 30 dB. An
        // audio-RMS meter under AGC moves by ~0 dB, which is the point.
        check(drop > 30.0f,
              "S-meter follows a 40 dB signal drop (audio-RMS meter would not)");
    }

    // ---- mode change doesn't break the pipeline ----
    const int before = audioCount;
    dsp.setMode(WdspChannel::Mode::Am);
    for (int b = 0; b < 40; ++b) {
        std::vector<std::complex<float>> blk(kEp6BlockSamples, std::complex<float>(0.2f, 0.0f));
        dsp.processIqBlock(blk);
    }
    check(audioCount > before, "audio continues after a mode change");

    // ---- 4.6: panadapter/audio latency matching, measured on both paths ----
    {
        const Skew usbOff = measureSkew(false, WdspChannel::Mode::Usb);
        const Skew usbOn  = measureSkew(true,  WdspChannel::Mode::Usb);
        const Skew amOff  = measureSkew(false, WdspChannel::Mode::Am);
        const Skew amOn   = measureSkew(true,  WdspChannel::Mode::Am);
        check(usbOff.ok && usbOn.ok && amOff.ok && amOn.ok,
              "every skew fixture saw the tone on both paths");
        auto show = [](const char* what, const Skew& k) {
            std::fprintf(stderr,
                "pan/audio %s: pan@%ld audio@%ld skew %+ld samples (%+.2f ms), "
                "delay %zu, reported %+.2f ms\n",
                what, k.panOnsetInput, k.audioOnsetInput, k.measuredSkewInput(),
                1000.0 * k.measuredSkewInput() / kSkewRateHz,
                k.delaySamples, k.reportedSkewMs);
        };
        show("USB off", usbOff); show("USB on ", usbOn);
        show(" AM off", amOff);  show(" AM on ", amOn);

        if (usbOff.ok && usbOn.ok && amOff.ok && amOn.ok) {
            // THE DEFECT, measured: the display leads the audio by most of a
            // filter delay and nothing in the shipped path does anything about
            // it. Bounded well under the 4095.5-sample group delay so this does
            // not quietly become a tap-count assertion.
            check(usbOff.delaySamples == 0,
                  "the default holds nothing back on the panadapter path");
            check(usbOff.measuredSkewInput() > 3000,
                  "by default the audio lags the display by most of a filter delay");

            // THE ANCHOR, AND THE ONLY ASSERTION HERE THAT THE DELAY LINE
            // CANNOT SATISFY BY AGREEING WITH ITSELF.
            //
            // Everything else in this block is stated in terms of the delay the
            // object REPORTS, so a mechanism armed with half the group delay
            // would still move the display by exactly what it claims and still
            // close most of the skew, and pass all of them. This one holds the
            // armed delay to a group delay MEASURED off the chain — the 50 %
            // envelope point at 8192 taps minus the same point at 2048, which
            // cancels every delay term the filter does not own.
            std::string tapErr;
            const long onset8192 = measureFilterOnsetSamples(8192, &tapErr);
            const long onset2048 = measureFilterOnsetSamples(2048, &tapErr);
            check(onset8192 > 0 && onset2048 > 0,
                  tapErr.empty() ? "both tap counts produced a settled envelope"
                                 : tapErr.c_str());
            if (onset8192 > 0 && onset2048 > 0) {
                const long measuredDelta = onset8192 - onset2048;
                // What the accessor says the same difference is. Read off two
                // real channels rather than retyped: (8192-1)/2 - (2048-1)/2.
                const double expectedDelta = (8192.0 - 1.0) / 2.0 - (2048.0 - 1.0) / 2.0;
                std::fprintf(stderr,
                    "filter group delay, measured: 50%% envelope at 8192 taps "
                    "%ld samples, at 2048 taps %ld, difference %ld "
                    "(arithmetic says %.1f)\n",
                    onset8192, onset2048, measuredDelta, expectedDelta);
                // One output block of tolerance: the envelope is sampled at the
                // output rate and processIq() delivers it a block at a time, so
                // the crossing is located to within that block.
                check(std::abs(static_cast<double>(measuredDelta) - expectedDelta) <= 64.0,
                      "the measured group-delay difference matches "
                      "filterGroupDelaySamples()'s model");
                // ...and the delay the alignment actually armed is that model
                // evaluated at the shipped tap count. Ties the mechanism to the
                // measurement rather than to its own arithmetic.
                check(std::abs(static_cast<long>(usbOn.delaySamples)
                               - (onset8192 - onset2048)
                               - static_cast<long>((2048 - 1) / 2)) <= 64,
                      "the armed delay is the measured group delay at 8192 taps");
            }

            // The mechanism moved the DISPLAY, by what it claims to hold, and
            // left the AUDIO alone. Within one EP6 block, not exactly: a frame
            // completes every kSkewFftSize (1024) input samples, the feed
            // arrives in kEp6BlockSamples (126), and 126 does not divide 1024 —
            // so the block that CARRIES a crossing lands up to 125 samples past
            // it, and differently for two crossings 4096 apart.
            check(usbOn.delaySamples > 0, "the alignment armed a delay");
            check(std::abs((usbOn.panOnsetInput - usbOff.panOnsetInput)
                           - static_cast<long>(usbOn.delaySamples))
                      < kEp6BlockSamples,
                  "the panadapter onset moved later by the armed delay");
            check(usbOn.audioOnsetInput == usbOff.audioOnsetInput,
                  "the audio path is untouched by the alignment");

            // AND THE MEASURED OFFSET BETWEEN THE TWO PATHS ACTUALLY FELL, by
            // the armed amount. Stated as a drop rather than as a residual
            // because it does not reach zero and must not be asserted to: what
            // is left is every delay term WdspChannel does not know about —
            // this class's own input-block buffering, WDSP's i/o buffering and
            // the output resampler — plus the frame quantisation above. See the
            // header: this mechanism removes the term that MOVES with the tap
            // count, which is the term a matching mechanism has to track, and
            // it does not claim to remove the rest.
            check(usbOn.measuredSkewInput() < usbOff.measuredSkewInput(),
                  "the alignment reduced the measured offset between the paths");
            check(std::abs((usbOff.measuredSkewInput() - usbOn.measuredSkewInput())
                           - static_cast<long>(usbOn.delaySamples))
                      < kEp6BlockSamples,
                  "the measured offset fell by the armed delay");

            // The reported number is derived from the chain, not a restatement
            // of the request: a read-back that simply echoed "aligned" would
            // pass every check above and tell an operator nothing.
            check(usbOff.reportedSkewMs > 1.0,
                  "the skew is reported as non-zero while nothing is aligned");
            check(std::abs(usbOn.reportedSkewMs) < 0.1,
                  "the reported skew collapses when the alignment is armed");

            // And the mode is part of the geometry, not a detail: AM puts bp1
            // in series behind nbp0, so an alignment computed once at open and
            // never re-derived would be exactly half right here. One sample of
            // tolerance because a single core's delay is (8192-1)/2 = 4095.5
            // and each mode rounds its own total once.
            check(std::abs(static_cast<long>(amOn.delaySamples)
                           - 2 * static_cast<long>(usbOn.delaySamples)) <= 1,
                  "AM arms twice the delay of USB (nbp0 + bp1 in series)");
        }
    }

    if (g_failures == 0)
        std::fprintf(stderr, "hl2_rxdsp_test: all checks passed\n");
    return g_failures == 0 ? 0 : 1;
}
