// The RX DSP must demodulate at EVERY IQ rate the HL2 can run, in the exact
// configuration Hl2Backend uses in production.
//
// hl2_rxdsp_test only ever ran 48 kHz in / 48 kHz audio out. Production runs
// 24 kHz audio (AudioEngine's native rate) and, since the panadapter span became
// operator-controllable, any of 48/96/192/384 kHz in. Nothing exercised that
// grid, so a rate at which the demodulator goes silent was invisible: the
// channel opens without error (validateConfig only checks integral ratios) and
// the panadapter keeps working because it does its own FFT and never touches
// WdspChannel.
//
// The failure this pins is silent by construction. Assert on AUDIO, at each
// rate, against the same tone.

#include "core/backends/hl2/Hl2RxDsp.h"
#include "core/backends/hl2/MetisProtocol.h"   // kEp6BlockSamples

#include <QCoreApplication>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <iterator>
#include <set>
#include <string>
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

// Demodulate a tone 1 kHz above centre at `rateHz` and return the peak |sample|
// of the audio that came out.
static float demodPeakAt(int rateHz, std::size_t* outBlockSamples,
                         int* audioBlocks, std::string* err)
{
    Hl2RxDsp dsp;
    Hl2RxDsp::Config cfg;
    // EXACTLY what Hl2Backend::connectRadio and setPanBandwidth build.
    cfg.inputSampleRateHz = rateHz;
    cfg.audioSampleRateHz = 24000;    // AudioEngine's native RX rate
    cfg.dspBlockSize = 1024;
    cfg.fftSize = 1024;
    cfg.mode = WdspChannel::Mode::Usb;
    cfg.filterLowHz = 150.0;
    cfg.filterHighHz = 3000.0;
    cfg.blockForOutput = true;        // deterministic for an offline burst feed

    if (!dsp.configure(cfg, err))
        return -1.0f;

    float peak = 0.0f;
    int blocks = 0;
    std::size_t blockSamples = 0;
    QObject::connect(&dsp, &Hl2RxDsp::audioReady, &dsp,
                     [&](const std::vector<float>& pcm) {
        ++blocks;
        blockSamples = pcm.size();
        for (const float v : pcm)
            peak = std::max(peak, std::abs(v));
    });

    // 0.5 s of a tone 1 kHz above centre, delivered as 126-sample EP6-shaped
    // blocks — IN WIRE ORDER, note the NEGATIVE sine. The HPSDR wire is the
    // conjugate of the analytic convention, so a signal above centre arrives as
    // exp(-j.2.pi.f.t), and the demodulator is fed the raw wire (HERMES §16).
    // The textbook exp(+j...) is synthetic IQ no HL2 ever sends, and against
    // USB [150,3000] it is out of passband: this same generator produced
    // audible audio only while the chain inverted every sideband.
    const double f = 1000.0;
    const int total = rateHz / 2;
    std::vector<std::complex<float>> blk;
    blk.reserve(kEp6BlockSamples);
    for (int n = 0; n < total; ++n) {
        const double ph = 2.0 * kPi * f * n / rateHz;
        blk.emplace_back(0.3f * static_cast<float>(std::cos(ph)),
                         0.3f * static_cast<float>(-std::sin(ph)));
        if (static_cast<int>(blk.size()) == kEp6BlockSamples) {
            dsp.processIqBlock(blk);
            blk.clear();
        }
    }
    if (!blk.empty())
        dsp.processIqBlock(blk);

    if (outBlockSamples) *outBlockSamples = blockSamples;
    if (audioBlocks) *audioBlocks = blocks;
    return peak;
}

// ── How many FFTW transform lengths the four rates plan ─────────────────
//
// Every partitioned-convolution filter core WDSP builds on an RX channel is
// planned at 2 * ch[].dsp_size under FFTW_PATIENT (firmin.c's plan_fircore,
// through RXA.c which passes ch[].dsp_size as every core's `size`), and
// ch[].dsp_size is the third argument WdspChannel::open() hands OpenChannel()
// — i.e. WdspChannel::Config::dspBlockSize. FFTW wisdom is keyed on the
// transform, so two channels share their planning cost only when that length
// matches.
//
// Hl2RxDsp::buildChannel() holds the INPUT block at 1024 and scales the DSP
// block DOWN with the rate, which gives each of the four rates its own
// transform length and therefore its own first-ever planning cost. Holding the
// DSP block and scaling the INPUT block UP instead — the same wall-clock span
// per DSP pass either way — would give all four ONE length.
//
// COUNTED, NEVER TIMED. Wall-clock planning figures are machine- and
// load-dependent (docs/HERMES.md §22.3's are 5-19x out against this lab's own
// bench, upstream #5456), so a seconds assertion measures the machine. The
// number of distinct transform lengths is a property of the geometry and is
// the same on every host.
namespace {

// The geometry Hl2RxDsp::buildChannel() actually builds, read back off the
// channel it opened rather than recomputed here.
std::size_t shippedTransformLengthAt(int rateHz, std::string* err)
{
    Hl2RxDsp::Config cfg;
    cfg.inputSampleRateHz = rateHz;
    cfg.audioSampleRateHz = 24000;
    cfg.dspBlockSize = 1024;
    cfg.fftSize = 1024;
    cfg.mode = WdspChannel::Mode::Usb;
    cfg.blockForOutput = true;
    Hl2RxDsp::RebuildResult r = Hl2RxDsp::buildChannel(cfg, false, 50);
    if (!r.channel) {
        if (err) *err = r.error;
        return 0;
    }
    return r.channel->filterTransformLength();
}

// The inversion: dspBlockSize held at 1024 for every rate, inputBlockSize
// scaled up so one DSP pass still consumes exactly one input block.
//     dsp_insize = dsp_size * in_rate / dsp_rate   [WDSP channel.c]
// which is 1024 * rate/48000 — so in_size must be the same, and the wall-clock
// span per pass stays 21.3 ms as it is today.
WdspChannel::Config invertedConfigAt(int rateHz)
{
    WdspChannel::Config wc;
    wc.direction = WdspChannel::Direction::Receive;
    wc.dspBlockSize = 1024;
    wc.inputBlockSize = static_cast<std::size_t>(1024) *
                        static_cast<std::size_t>(rateHz) / 48000u;
    wc.inputSampleRate = rateHz;
    wc.dspSampleRate = Hl2RxDsp::kWdspDspSampleRateHz;
    wc.outputSampleRate = 24000;
    wc.mode = WdspChannel::Mode::Usb;
    wc.filterLowHz = 150.0;
    wc.filterHighHz = 3000.0;
    wc.agcMode = 3;
    wc.maximumAgcGainDb = 39.0;
    wc.filterTaps = Hl2RxDsp::kRxFilterTaps;
    wc.blockForOutput = true;
    return wc;
}

}  // namespace

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);

    // The four rates the gateware can run — the same list Hl2Backend advertises
    // and snaps zoom requests to.
    struct Row { int rateHz; float peak; std::size_t blockSamples; int blocks; };
    Row rows[] = {{48000, 0, 0, 0}, {96000, 0, 0, 0},
                  {192000, 0, 0, 0}, {384000, 0, 0, 0}};

    for (auto& r : rows) {
        std::string err;
        r.peak = demodPeakAt(r.rateHz, &r.blockSamples, &r.blocks, &err);
        if (r.peak < 0.0f) {
            std::fprintf(stderr, "FAIL: %d Hz did not configure: %s\n",
                         r.rateHz, err.c_str());
            ++g_failures;
            continue;
        }
        // outputBlockSize = dspBlockSize * 24000 / rate, interleaved stereo.
        const std::size_t expect =
            static_cast<std::size_t>(1024) * 24000 / static_cast<std::size_t>(r.rateHz) * 2;
        std::fprintf(stderr,
                     "%6d Hz in -> audio peak %.5f, %d blocks of %zu samples "
                     "(expect %zu)\n",
                     r.rateHz, static_cast<double>(r.peak), r.blocks,
                     r.blockSamples, expect);
        check(r.blockSamples == expect, "audio block is the rate-scaled size");
        check(r.blocks > 0, "audio blocks were produced at this rate");
        // THE assertion. A silent demodulator at a rate the operator can select
        // by zooming is the bug; everything above is diagnosis for when it trips.
        check(r.peak > 0.01f, "demodulator produces audible audio at this rate");
    }

    // And the levels must AGREE across rates. The same tone at the same
    // amplitude has to come out at the same level whatever the DDC is doing —
    // otherwise zooming would change how loud the radio is.
    float lo = 1e9f, hi = 0.0f;
    for (const auto& r : rows) {
        if (r.peak <= 0.0f) continue;
        lo = std::min(lo, r.peak);
        hi = std::max(hi, r.peak);
    }
    if (hi > 0.0f && lo < 1e9f) {
        const double spreadDb = 20.0 * std::log10(hi / lo);
        std::fprintf(stderr, "level spread across rates: %.1f dB\n", spreadDb);
        check(spreadDb < 6.0, "audio level is consistent across every IQ rate");
    }

    // ---- 5.4: one plan set per rate, or one plan set full stop ----
    {
        std::set<std::size_t> shipped;
        for (const auto& r : rows) {
            std::string err;
            const std::size_t len = shippedTransformLengthAt(r.rateHz, &err);
            if (len == 0) {
                std::fprintf(stderr, "FAIL: %d Hz did not build: %s\n",
                             r.rateHz, err.c_str());
                ++g_failures;
                continue;
            }
            std::fprintf(stderr, "%6d Hz in -> WDSP filter transform length %zu\n",
                         r.rateHz, len);
            shipped.insert(len);
        }
        // THE DEFECT: nothing is shared. Stated as "one per rate" rather than
        // as the literal 4, so it still says the right thing if the rate list
        // ever changes.
        check(shipped.size() == std::size(rows),
              "shipped geometry plans a DISTINCT transform length for every rate");

        // And the inversion. Opened for real, not reasoned about: if WDSP's
        // half-band input resampler could not take an 8192-sample input block
        // against a 1024-sample DSP block, this is where it would say so.
        std::set<std::size_t> inverted;
        for (const auto& r : rows) {
            std::string err;
            auto ch = WdspChannel::create(invertedConfigAt(r.rateHz), &err);
            if (!ch) {
                std::fprintf(stderr, "FAIL: %d Hz inverted geometry: %s\n",
                             r.rateHz, err.c_str());
                ++g_failures;
                continue;
            }
            // It must also DEMODULATE, or "one plan set" is bookkeeping about a
            // chain that does not work. Same tone, same wire order, same
            // threshold as the rate grid above.
            const std::size_t inN = ch->config().inputBlockSize;
            const std::size_t outN = ch->outputBlockSize();
            std::vector<float> bi(inN), bq(inN), bl(outN), br(outN);
            float peak = 0.0f;
            const int blocks = r.rateHz / static_cast<int>(inN) / 2;   // ~0.5 s
            long n = 0;
            for (int b = 0; b < blocks; ++b) {
                for (std::size_t k = 0; k < inN; ++k, ++n) {
                    const double ph = 2.0 * kPi * 1000.0 * static_cast<double>(n)
                                      / r.rateHz;
                    bi[k] = 0.3f * static_cast<float>(std::cos(ph));
                    bq[k] = 0.3f * static_cast<float>(-std::sin(ph));
                }
                if (ch->processIq(bi, bq, bl, br) != WdspChannel::ProcessResult::Ok)
                    continue;
                for (const float v : bl)
                    peak = std::max(peak, std::abs(v));
            }
            std::fprintf(stderr,
                         "%6d Hz inverted -> in_size %zu, dsp_size %zu, "
                         "transform length %zu, audio peak %.5f\n",
                         r.rateHz, inN, ch->config().dspBlockSize,
                         ch->filterTransformLength(), static_cast<double>(peak));
            check(peak > 0.01f,
                  "the inverted geometry still demodulates at this rate");
            inverted.insert(ch->filterTransformLength());
        }
        // WHAT INVERTING BUYS, measured the same way the defect was: one plan
        // set for all four rates, so a crossing never meets an unplanned
        // transform and the per-rate cold cost stops existing.
        check(inverted.size() == 1,
              "the inverted geometry plans ONE transform length for all rates");
    }

    if (g_failures == 0)
        std::fprintf(stderr, "hl2_rxdsp_rate_test: all checks passed\n");
    return g_failures == 0 ? 0 : 1;
}
