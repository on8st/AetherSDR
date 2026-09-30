// The operator's MON on the Hermes-Lite 2: the post-ALC transmit audio, mixed
// into this computer's receive output at the MON level, ONLY while keyed.
//
// Socket-free and radio-free. Three layers, each asserting what only it can:
//
//   1. Hl2TxMonitor, the pure gate and gain: every refusal term (MON off, not
//      keyed, diagnostic monitor on, CW, level 0) yields NOTHING, and an
//      admitted block comes out as L=R at exactly level/100.
//   2. Hl2TxDsp's tap: off by default and silent; on, it hands over the block
//      the modulator was given -- post mic gain, post ALC -- which is checked
//      against the chain's own alcPeak() reading rather than a retyped target.
//   3. Hl2Backend: setTxMonitor() reaches the tap, and a delivered block is
//      PUBLISHED on the receive-audio path while keyed and dropped once
//      unkeyed, judged at delivery.

#include "TestSettingsProfile.h"
#include "TxTestAuthority.h"
#include "core/backends/hl2/Hl2Backend.h"
#include "core/backends/hl2/Hl2TxDsp.h"
#include "core/backends/hl2/Hl2TxMonitor.h"

#include <QCoreApplication>

#include <cmath>
#include <cstdio>
#include <vector>

namespace AetherSDR::hl2 {
struct Hl2HostTxTestAccess {
    static void start(Hl2Backend& backend, const QString& txMode)
    {
        backend.m_rx.resize(1);
        backend.m_rx[0].mode = txMode;
        backend.m_txDdc = 0;
        backend.m_connected = true;
        emit backend.connected();   // opens the speaker PCM session
    }
    static void setMode(Hl2Backend& backend, const QString& txMode) { backend.m_rx[0].mode = txMode; }
    static void setKeyed(Hl2Backend& backend, bool keyed) { backend.m_keyed = keyed; }
    static void setDiagnosticMonitor(Hl2Backend& backend, bool on) { backend.m_txMonitor = on; }
    static Hl2TxDsp* txDsp(Hl2Backend& backend) { return backend.m_txDsp; }
    static void deliver(Hl2Backend& backend, const std::vector<float>& block)
    {
        backend.deliverTxMonitorAudio(block);
    }
    static void finish(Hl2Backend& backend)
    {
        backend.m_rx.clear();
        backend.m_keyed = false;
        backend.m_connected = false;
        backend.retirePcmStreams();
    }
};
} // namespace AetherSDR::hl2

using namespace AetherSDR;
using namespace AetherSDR::hl2;

namespace {
int failures = 0;
void check(bool ok, const char* what)
{
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    failures += ok ? 0 : 1;
}

std::vector<float> ramp(std::size_t n)
{
    std::vector<float> v(n);
    for (std::size_t i = 0; i < n; ++i) {
        v[i] = -0.9f + 1.8f * static_cast<float>(i) / static_cast<float>(n - 1);
    }
    return v;
}

void pureGateAndGain()
{
    Hl2TxMonitor mon;
    const std::vector<float> in = ramp(64);
    check(!mon.enabled(), "MON starts off");
    check(mon.render(in, true, false, false).empty(), "MON off: nothing while keyed");

    mon.setMonitor(true, 40);
    const std::vector<float> out = mon.render(in, true, false, false);
    check(out.size() == in.size() * 2, "admitted block becomes interleaved stereo");
    bool exact = true;
    for (std::size_t i = 0; i < in.size(); ++i) {
        const float want = in[i] * 0.40f;
        exact = exact && out[2 * i] == want && out[2 * i + 1] == want;
    }
    check(exact, "level 40 honoured exactly as 0.40 linear, L == R");

    check(mon.render(in, false, false, false).empty(), "not keyed: nothing");
    check(mon.render(in, true, true, false).empty(),
          "diagnostic TX-audio monitor on: MON stays out of the capture");
    check(mon.render(in, true, false, true).empty(), "CW: nothing (sidetone's job)");
    mon.setMonitor(true, 0);
    check(mon.render(in, true, false, false).empty(), "level 0: nothing, not a block of zeros");
    mon.setMonitor(true, 250);
    check(mon.levelPercent() == 100 && mon.gain() == 1.0f, "level clamps to 100 = unity");
    mon.setMonitor(false, 100);
    check(mon.render(in, true, false, false).empty(), "switched off again: nothing");
}

void txDspTapIsThePostAlcBlock()
{
    TxTestAuthority authority;
    Hl2TxDsp tx;
    Hl2TxDsp::Config cfg;
    cfg.alcEnabled = true;
    std::string err;
    if (!tx.configure(cfg, &err)) {
        check(false, "Hl2TxDsp configures");
        return;
    }
    // +12 dB of mic gain on a 0.5 tone: 2.0 into the ALC, which must pull it
    // back to its target. The raw mic would be 0.5; the pre-ALC level 2.0.
    tx.setMicGain(4.0);
    std::vector<std::vector<float>> tapped;
    float lastAlcPeakDb = -999.0f;
    QObject::connect(&tx, &Hl2TxDsp::monitorAudio, &tx,
                     [&](const std::vector<float>& b) { tapped.push_back(b); });
    QObject::connect(&tx, &Hl2TxDsp::alcPeak, &tx, [&](float db) { lastAlcPeakDb = db; });

    const int block = cfg.dspBlockSize;
    std::vector<float> tone(static_cast<std::size_t>(block));
    for (int n = 0; n < block; ++n) {
        tone[static_cast<std::size_t>(n)] =
            0.5f * static_cast<float>(std::sin(2.0 * M_PI * 1000.0 * n / cfg.inputSampleRateHz));
    }
    tx.processAudioBlock(tone, TxAudioSource::Microphone, authority.context);
    check(tapped.empty() && !tx.monitorTap(), "tap off by default: no block leaves the I/O thread");

    tx.setMonitorTap(true);
    tx.processAudioBlock(tone, TxAudioSource::Microphone, authority.context);
    check(tapped.size() == 1, "tap on: one monitor block per modulated block");
    if (tapped.size() != 1) {
        return;
    }
    check(tapped[0].size() == static_cast<std::size_t>(block), "the whole consumed block");
    float peak = 0.0f;
    for (float s : tapped[0]) {
        peak = std::max(peak, std::fabs(s));
    }
    const float peakDb = 20.0f * std::log10(peak);
    check(std::fabs(peakDb - lastAlcPeakDb) < 1e-4f,
          "monitored block peaks exactly where alcPeak() says the modulator input does");
    check(peak <= static_cast<float>(cfg.alcTargetPeak) + 1e-4f && peak > 0.5f,
          "it is the post-ALC level: above the raw mic, at or below the ALC target");

    tx.setMonitorTap(false);
    tx.processAudioBlock(tone, TxAudioSource::Microphone, authority.context);
    check(tapped.size() == 1, "tap off again: nothing further");
}

void backendPublishesOnlyWhileKeyed()
{
    Hl2Backend backend;
    Hl2HostTxTestAccess::start(backend, QStringLiteral("USB"));
    int frames = 0;
    QByteArray last;
    QObject::connect(&backend, &IRadioBackend::audioFrameReady, &backend,
                     [&](const PcmFrame& f) { ++frames; last = f.legacyStereo24(); });

    Hl2TxDsp* dsp = Hl2HostTxTestAccess::txDsp(backend);
    check(dsp && !dsp->monitorTap(), "backend's TX DSP starts with the tap off");
    backend.setTxMonitor(true, 25);
    check(dsp && dsp->monitorTap(), "setTxMonitor(true) reaches the TX DSP's tap");

    const std::vector<float> block = ramp(32);
    Hl2HostTxTestAccess::deliver(backend, block);
    check(frames == 0, "unkeyed: a delivered block is dropped, not played");

    Hl2HostTxTestAccess::setKeyed(backend, true);
    Hl2HostTxTestAccess::deliver(backend, block);
    check(frames == 1, "keyed with MON on: the block is published on the receive-audio path");
    const auto* s = reinterpret_cast<const float*>(last.constData());
    const bool levelled = last.size() == static_cast<qsizetype>(block.size() * 2 * sizeof(float))
        && s[0] == block[0] * 0.25f && s[1] == block[0] * 0.25f;
    check(levelled, "published at the MON level (25 -> 0.25), stereo");

    Hl2HostTxTestAccess::setDiagnosticMonitor(backend, true);
    Hl2HostTxTestAccess::deliver(backend, block);
    check(frames == 1, "diagnostic monitor on: MON withheld");
    Hl2HostTxTestAccess::setDiagnosticMonitor(backend, false);

    Hl2HostTxTestAccess::setMode(backend, QStringLiteral("CW"));
    Hl2HostTxTestAccess::deliver(backend, block);
    check(frames == 1, "CW transmit mode: MON withheld");
    Hl2HostTxTestAccess::setMode(backend, QStringLiteral("LSB"));

    backend.setTxMonitor(false, 25);
    Hl2HostTxTestAccess::deliver(backend, block);
    check(frames == 1 && !dsp->monitorTap(), "MON off while keyed: silent at once, tap closed");

    backend.setTxMonitor(true, 25);
    Hl2HostTxTestAccess::setKeyed(backend, false);
    Hl2HostTxTestAccess::deliver(backend, block);
    check(frames == 1, "a block arriving after the unkey is judged by the unkey");
    Hl2HostTxTestAccess::finish(backend);
}
} // namespace

int main(int argc, char** argv)
{
    TestSettingsProfile profile(QStringLiteral("hl2-tx-monitor"));
    if (!profile.isValid()) {
        return 1;
    }
    QCoreApplication app(argc, argv);
    pureGateAndGain();
    txDspTapIsThePostAlcBlock();
    backendPublishesOnlyWhileKeyed();
    std::printf("%d failure(s)\n", failures);
    return failures == 0 ? 0 : 1;
}
