// LSB-sounds-like-AM probe — v26.9.4, TXA build.
//
// The existing hl2_txdsp_test measures the SIDEBAND RATIO and nothing else.
// The reported symptom is AM, which is a CARRIER and/or BOTH SIDEBANDS, so
// this probe reports three numbers per case and never collapses them:
//
//   wanted   energy at the sideband the mode names
//   image    energy at the opposite sideband
//   carrier  energy at DC of the emitted IQ  <-- nothing in the tree reads this
//
// Everything about the instrument (pacing, settle skip, whole-cycle trim,
// Goertzel bin) is copied from tests/hl2_txdsp_test.cpp so the figures are
// comparable with the published rows. The carrier bin is the one addition, and
// it reuses the SAME trimmed window as the tone bins so the three numbers come
// from one capture of one length.
//
// NOTHING HERE TOUCHES A RADIO.

#include "core/backends/hl2/Hl2TxDsp.h"
#include "TxTestAuthority.h"

#include <QCoreApplication>
#include <QObject>

#include <chrono>
#include <cmath>
#include <complex>
#include <cstdio>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

using namespace AetherSDR::hl2;
using AetherSDR::TxAudioSource;

static constexpr double kFsOut = 48000.0;
static constexpr double kTone = 1000.0;
// Identical to hl2_txdsp_test's kSettleSamples: clears create_slews' mute ramp.
static constexpr std::size_t kSettleSamples = 12288;

static std::size_t wholeCycles(std::size_t have, double hz, double fs)
{
    const long f = std::lround(hz);
    const long r = std::lround(fs);
    if (f <= 0 || r <= 0) return 0;
    const std::size_t period = static_cast<std::size_t>(r / std::gcd(f, r));
    return (have / period) * period;
}

// Goertzel-style complex bin, over an EXPLICIT window, so DC (hz = 0) can be
// probed over exactly the same samples as the tone bins.
static double binPowerWindow(const std::vector<std::complex<float>>& iq,
                             double hz, double fs,
                             std::size_t from, std::size_t count)
{
    if (count == 0 || from + count > iq.size()) return 0.0;
    std::complex<double> acc{0.0, 0.0};
    const double w = -2.0 * M_PI * hz / fs;
    for (std::size_t n = 0; n < count; ++n) {
        const double ph = w * static_cast<double>(n);
        acc += std::complex<double>(iq[from + n].real(), iq[from + n].imag())
             * std::complex<double>(std::cos(ph), std::sin(ph));
    }
    return std::abs(acc) / static_cast<double>(count);
}

struct Window { std::size_t from; std::size_t count; };

static Window trimmedWindow(const std::vector<std::complex<float>>& iq, double toneHz)
{
    if (iq.size() <= kSettleSamples) return {0, 0};
    return {kSettleSamples, wholeCycles(iq.size() - kSettleSamples, toneHz, kFsOut)};
}

static double db(double num, double den)
{
    const double eps = 1e-30;
    return 20.0 * std::log10((num + eps) / (den + eps));
}

// Feed paced at the configured audio rate — the TXA channel is opened with
// blockForOutput = false and STARVES if fed faster than real time.
static void feed(Hl2TxDsp& tx, const std::vector<float>& chunk,
                 const AetherSDR::TxCoordinator::Context& ctx)
{
    tx.processAudioBlock(chunk, TxAudioSource::Microphone, ctx);
    std::this_thread::sleep_for(std::chrono::duration<double>(
        static_cast<double>(chunk.size()) / tx.config().inputSampleRateHz));
}

static void runTone(Hl2TxDsp& tx, const AetherSDR::TxCoordinator::Context& ctx,
                    double seconds, double amplitude, double dcOffset,
                    std::vector<std::complex<float>>& cap)
{
    const int fs = tx.config().inputSampleRateHz;
    constexpr std::size_t kChunk = 240;
    const int total = static_cast<int>(fs * seconds);
    std::vector<float> chunk(kChunk);
    for (int off = 0; off + static_cast<int>(kChunk) <= total;
         off += static_cast<int>(kChunk)) {
        for (std::size_t n = 0; n < kChunk; ++n)
            chunk[n] = static_cast<float>(
                dcOffset + amplitude * std::sin(2.0 * M_PI * kTone
                                                * (off + static_cast<int>(n)) / fs));
        feed(tx, chunk, ctx);
    }
    (void)cap;
}

// Report one capture. `wantedIsUpper` says which wire bin the mode should use.
//
// WIRE-FACING SIGN, per hl2_txdsp_test's header: USB -> energy at -1 kHz,
// LSB -> energy at +1 kHz. That is not a typo; the HPSDR wire order has the
// opposite handedness to the analytic convention.
static void report(const char* label, const std::vector<std::complex<float>>& iq,
                   bool wantedIsUpper, unsigned long long faults,
                   unsigned long long blocks)
{
    const Window w = trimmedWindow(iq, kTone);
    if (w.count == 0) {
        std::fprintf(stdout, "%-46s NO CAPTURE (%zu samples)\n", label, iq.size());
        return;
    }
    const double up = binPowerWindow(iq, +kTone, kFsOut, w.from, w.count);
    const double lo = binPowerWindow(iq, -kTone, kFsOut, w.from, w.count);
    const double dc = binPowerWindow(iq, 0.0,    kFsOut, w.from, w.count);
    const double wanted = wantedIsUpper ? up : lo;
    const double image  = wantedIsUpper ? lo : up;

    // Harmonics on the WANTED side: asymmetric/hard clipping shows here.
    const double sgn = wantedIsUpper ? +1.0 : -1.0;
    const double h2 = binPowerWindow(iq, sgn * 2.0 * kTone, kFsOut, w.from, w.count);
    const double h3 = binPowerWindow(iq, sgn * 3.0 * kTone, kFsOut, w.from, w.count);
    std::fprintf(stdout,
        "%-46s wanted=%.6e image=%.6e carrier=%.6e | "
        "image %+8.2f dBc | carrier %+8.2f dBc | 2f %+7.2f dBc | 3f %+7.2f dBc | n=%zu faults=%llu/%llu\n",
        label, wanted, image, dc,
        db(image, wanted), db(dc, wanted), db(h2, wanted), db(h3, wanted),
        w.count, faults, blocks);
}

// One self-contained case: build a modulator, optionally mutate it mid-run,
// capture, report.
struct Case {
    const char* label;
    WdspChannel::Mode mode;
    double lowHz;
    double highHz;
    bool wantedIsUpper;         // expected wire bin for THIS mode
    double dcOffset = 0.0;
    // mid-run mutation, applied after `preSeconds` of audio
    WdspChannel::Mode switchTo = WdspChannel::Mode::Usb;
    bool doSwitchMode = false;
    bool doRefusedFilter = false;   // push an inverted/degenerate pair
    double refuseLow = 0.0;
    double refuseHigh = 0.0;
    double micGain = 1.0;
    bool alc = false;
    double amplitude = 0.5;
    WdspChannel::Mode switchTo2 = WdspChannel::Mode::Usb;
    bool doSwitchMode2 = false;
};

static void runCase(const Case& c)
{
    TxTestAuthority authority;
    Hl2TxDsp tx;
    Hl2TxDsp::Config cfg;
    cfg.mode = c.mode;
    cfg.filterLowHz = c.lowHz;
    cfg.filterHighHz = c.highHz;
    cfg.alcEnabled = c.alc;     // an ALC breaks fixed input/output relationships
    std::string err;
    if (!tx.configure(cfg, &err)) {
        std::fprintf(stdout, "%-46s CONFIGURE REFUSED: %s\n", c.label, err.c_str());
        return;
    }
    tx.setMicGain(c.micGain);

    std::vector<std::complex<float>> cap;
    QObject::connect(&tx, &Hl2TxDsp::iqReady, &tx,
                     [&cap](const std::vector<std::complex<float>>& iq) {
        cap.insert(cap.end(), iq.begin(), iq.end());
    });

    if (c.doSwitchMode || c.doRefusedFilter) {
        // Run the channel first so the mutation lands on an OPEN, RUNNING
        // channel — the applyModeAndFilter() path, which buildModulator() never
        // reaches.
        runTone(tx, authority.context, 1.0, c.amplitude, c.dcOffset, cap);
        if (c.doSwitchMode)      tx.setMode(c.switchTo);
        if (c.doRefusedFilter)   tx.setFilter(c.refuseLow, c.refuseHigh);
        if (c.doSwitchMode2) {
            // What Hl2Backend::setSliceMode does on the way back: mode first,
            // then the passband, both onto the same open channel.
            cap.clear();
            runTone(tx, authority.context, 0.5, c.amplitude, c.dcOffset, cap);
            tx.setMode(c.switchTo2);
            tx.setFilter(300.0, 2700.0);
        }
        cap.clear();
    }
    runTone(tx, authority.context, 1.0, c.amplitude, c.dcOffset, cap);
    report(c.label, cap, c.wantedIsUpper,
           tx.modulatorFaultBlocks(), tx.modulatorBlocks());

    // What the channel BELIEVES its passband is, after whatever happened above.
    if (const WdspChannel::Config* cc = tx.channelConfig()) {
        std::fprintf(stdout, "%-46s   channel passband now %.1f .. %.1f Hz, mode=%d, id=%d\n",
                     "", cc->filterLowHz, cc->filterHighHz,
                     static_cast<int>(cc->mode), tx.wdspChannelId());
    }
}

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    std::fprintf(stdout, "modulator: %s (AETHER_HL2_TX_TXA=%d)\n",
                 Hl2TxDsp::modulatorName(), AETHER_HL2_TX_TXA);
    std::fprintf(stdout, "tone %.0f Hz, amplitude 0.5, ALC off, %0.f Hz IQ, "
                 "settle skip %zu samples\n\n", kTone, kFsOut, kSettleSamples);

    // ── SHIPPED DEFAULTS ────────────────────────────────────────────────
    runCase({"USB {300,2700} (control)",  WdspChannel::Mode::Usb, 300, 2700, false});
    runCase({"LSB {300,2700}",            WdspChannel::Mode::Lsb, 300, 2700, true});
    runCase({"DIGU {150,3000}",           WdspChannel::Mode::Digu, 150, 3000, false});
    runCase({"DIGL {150,3000}",           WdspChannel::Mode::Digl, 150, 3000, true});

    // ── POSITIVE CONTROLS: what a REAL both-sidebands / carrier looks like ──
    // If the instrument cannot see these, it cannot see the fault either.
    runCase({"DSB {300,2700} (control: both sidebands)",
             WdspChannel::Mode::Dsb, 300, 2700, false});
    runCase({"AM {300,2700} (control: what AM looks like)",
             WdspChannel::Mode::Am, 300, 2700, false});
    runCase({"SAM {300,2700} (control)",
             WdspChannel::Mode::Sam, 300, 2700, false});

    // ── THE MODE-CHANGE PATH (applyModeAndFilter on an open channel) ────
    {
        Case c{"USB -> setMode(LSB) on a running channel",
               WdspChannel::Mode::Usb, 300, 2700, true};
        c.doSwitchMode = true; c.switchTo = WdspChannel::Mode::Lsb;
        runCase(c);
    }
    {
        Case c{"AM -> setMode(LSB) on a running channel",
               WdspChannel::Mode::Am, 300, 2700, true};
        c.doSwitchMode = true; c.switchTo = WdspChannel::Mode::Lsb;
        runCase(c);
    }
    {
        Case c{"DSB -> setMode(LSB) on a running channel",
               WdspChannel::Mode::Dsb, 300, 2700, true};
        c.doSwitchMode = true; c.switchTo = WdspChannel::Mode::Lsb;
        runCase(c);
    }
    {
        Case c{"FM -> setMode(LSB) on a running channel",
               WdspChannel::Mode::Fm, 300, 2700, true};
        c.doSwitchMode = true; c.switchTo = WdspChannel::Mode::Lsb;
        runCase(c);
    }

    // ── A REFUSED setFilter: what passband is the transmitter running? ──
    {
        Case c{"LSB then REFUSED setFilter(2700,2700)",
               WdspChannel::Mode::Lsb, 300, 2700, true};
        c.doRefusedFilter = true; c.refuseLow = 2700; c.refuseHigh = 2700;
        runCase(c);
    }
    {
        Case c{"USB then REFUSED setFilter(2700,2700)",
               WdspChannel::Mode::Usb, 300, 2700, false};
        c.doRefusedFilter = true; c.refuseLow = 2700; c.refuseHigh = 2700;
        runCase(c);
    }

    // ── OPERATOR eSSB EDGES that reach effectiveTxPassband verbatim ─────
    // setTxFilter clamps low to [0, max-50] and high to [low+50, max], so a
    // ZERO low-cut is reachable and a degenerate pair is not.
    runCase({"LSB {0,2700}   (eSSB zero low-cut)",   WdspChannel::Mode::Lsb, 0, 2700, true});
    runCase({"USB {0,2700}   (eSSB zero low-cut)",   WdspChannel::Mode::Usb, 0, 2700, false});
    runCase({"LSB {50,100}   (narrowest legal)",     WdspChannel::Mode::Lsb, 50, 100, true});

    // ── DC ON THE AUDIO: can a carrier get through at all? ──────────────
    {
        Case c{"LSB {300,2700} + 0.05 audio DC",
               WdspChannel::Mode::Lsb, 300, 2700, true};
        c.dcOffset = 0.05;
        runCase(c);
    }
    {
        Case c{"LSB {0,2700} + 0.05 audio DC",
               WdspChannel::Mode::Lsb, 0, 2700, true};
        c.dcOffset = 0.05;
        runCase(c);
    }

    // ── THE LEVEL CHAIN: +40 dB slider, ALC reduction-only (both v26.9.4) ──
    // Asymmetric clipping is a DC generator, and DC inside the passband is a
    // carrier. These push the shared level chain into the hard clamp.
    { Case c{"LSB {300,2700} micGain=1   ALC on",  WdspChannel::Mode::Lsb, 300, 2700, true};
      c.alc = true; runCase(c); }
    { Case c{"LSB {300,2700} micGain=100 ALC on",  WdspChannel::Mode::Lsb, 300, 2700, true};
      c.alc = true; c.micGain = 100.0; runCase(c); }
    { Case c{"LSB {300,2700} micGain=100 ALC off", WdspChannel::Mode::Lsb, 300, 2700, true};
      c.micGain = 100.0; runCase(c); }
    { Case c{"LSB {0,2700}   micGain=100 ALC off", WdspChannel::Mode::Lsb, 0, 2700, true};
      c.micGain = 100.0; runCase(c); }

    // ── ORDINAL 12: RXA_WBFM on receive, TXA_AM_LSB on transmit ────────
    // WdspChannel::create REFUSES Transmit+Wbfm. WdspChannel::setMode does
    // not, and wdspMode() is a bare static_cast shared by both directions.
    {
        Case c{"USB -> setMode(WBFM) [ordinal 12 on a TX channel]",
               WdspChannel::Mode::Usb, 300, 2700, false};
        c.doSwitchMode = true; c.switchTo = WdspChannel::Mode::Wbfm;
        runCase(c);
    }
    {
        Case c{"LSB -> setMode(WBFM) [ordinal 12 on a TX channel]",
               WdspChannel::Mode::Lsb, 300, 2700, true};
        c.doSwitchMode = true; c.switchTo = WdspChannel::Mode::Wbfm;
        runCase(c);
    }
    // THE DECISIVE ONE: setSliceMode pushes setMode to m_txDsp even for a
    // receive-only mode, so the channel reaches ordinal 12 without keying.
    // Does moving the slice back to LSB undo it?
    {
        Case c{"LSB -> WBFM -> back to LSB (does it recover?)",
               WdspChannel::Mode::Lsb, 300, 2700, true};
        c.doSwitchMode = true;  c.switchTo  = WdspChannel::Mode::Wbfm;
        c.doSwitchMode2 = true; c.switchTo2 = WdspChannel::Mode::Lsb;
        runCase(c);
    }
    {
        Case c{"USB -> WBFM -> back to USB (does it recover?)",
               WdspChannel::Mode::Usb, 300, 2700, false};
        c.doSwitchMode = true;  c.switchTo  = WdspChannel::Mode::Wbfm;
        c.doSwitchMode2 = true; c.switchTo2 = WdspChannel::Mode::Usb;
        runCase(c);
    }
    // configure() straight into Wbfm on transmit: create's validator should refuse.
    runCase({"WBFM at configure() (create must refuse)",
             WdspChannel::Mode::Wbfm, 300, 2700, false});

    // Does a Wbfm excursion poison the pooled WDSP channel for the NEXT open?
    runCase({"LSB {300,2700} on a FRESH channel after all that",
             WdspChannel::Mode::Lsb, 300, 2700, true});

    std::fprintf(stdout, "\nprobe done\n");
    return 0;
}
