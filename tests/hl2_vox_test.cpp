// Host VOX on the Hermes-Lite 2, with synthetic mic frames. No radio, no
// socket, no transmission: the "keying" asserted in part C is a recording
// backend's setKeying() count, reached through the real RadioModel TX path.
//
//   A. Hl2VoxDetector: keys above the threshold, HOLDS for the delay counted
//      in mic samples, releases after it, resets its hang on renewed speech,
//      and never keys when VOX is off or keying is not permitted.
//   B. Hl2Backend: the level it compares is micPeak()'s (mic x HL2 mic gain),
//      and every refusal the backend knows about -- transmit not allowed (the
//      automation bridge without ALLOW_TX), CW, TUNE -- keeps it silent; VOX
//      off mid-hold releases; a mic that stops arriving is released by the
//      wall-clock backstop.
//   C. RadioModel: a VOX request becomes an ordinary PTT press. Permitted, it
//      keys once and releases once. VOX off in the model, a pan TX inhibit, a
//      receive-only mode or a receive-only backend: NOTHING reaches setKeying.
//      And it never stacks on, or unkeys, an operator's own MOX.
//   D. Anti-VOX (d167 D-vox chatter, hl2-lab d167-vox-chatter.md): the
//      reference input to Hl2VoxDetector::feed gates NEW keys only; Hl2AntiVox
//      holds VOX off for kRestartHoldMs after our output restarts, and carries
//      an opt-in level term. Through the real backend: after our receive audio
//      resumes, d167's re-key levels are held off, while speech at the
//      operator's own measured -21.1 dBFS still keys once the hold ends.

#include "TestSettingsProfile.h"
#include "core/backends/hl2/Hl2Backend.h"
#include "core/backends/hl2/Hl2TxLevelPolicy.h"
#include "core/backends/hl2/Hl2Vox.h"
#include "models/RadioModel.h"
#include "models/SliceModel.h"
#include "models/TransmitModel.h"
#include "models/TxController.h"

#include <QCoreApplication>
#include <QElapsedTimer>

#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <limits>
#include <memory>
#include <vector>

namespace AetherSDR::hl2 {
struct Hl2HostTxTestAccess {
    static void start(Hl2Backend& backend, const QString& txMode)
    {
        backend.m_rx.resize(1);
        backend.m_rx[0].mode = txMode;
        backend.m_txDdc = 0;
        backend.m_connected = true;
        backend.m_txAllowed = true;
    }
    static void setMode(Hl2Backend& backend, const QString& txMode) { backend.m_rx[0].mode = txMode; }
    static void setTxAllowed(Hl2Backend& backend, bool allowed) { backend.m_txAllowed = allowed; }
    static void setTuning(Hl2Backend& backend, bool tuning) { backend.m_tuning = tuning; }
    // The receive-audio hold's own writer: true at key-down, false when the
    // unkey hold runs out and our receive output resumes.
    static void muteRx(Hl2Backend& backend, bool muted) { backend.applyRxAudioMute(muted); }
    static std::uint64_t antiVoxSuppressed(const Hl2Backend& backend) { return backend.m_vox.antiVoxSuppressed(); }
    static void finish(Hl2Backend& backend)
    {
        backend.m_rx.clear();
        backend.m_connected = false;
    }
};
} // namespace AetherSDR::hl2

using namespace AetherSDR;
using namespace AetherSDR::hl2;
using Edge = Hl2VoxDetector::Edge;

namespace {
int failures = 0;
void check(bool ok, const char* what)
{
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    failures += ok ? 0 : 1;
}

constexpr int kRate = 24000;
constexpr int kBlock = 480;   // 20 ms at 24 kHz

double fromDb(double db) { return std::pow(10.0, db / 20.0); }

// A block of int16 stereo holding a 1 kHz tone at `dbfs` peak, L == R.
QByteArray micBlock(double dbfs)
{
    QByteArray b(kBlock * 2 * static_cast<int>(sizeof(qint16)), Qt::Uninitialized);
    auto* p = reinterpret_cast<qint16*>(b.data());
    const double a = fromDb(dbfs) * 32767.0;
    for (int n = 0; n < kBlock; ++n) {
        const auto s = static_cast<qint16>(std::lround(a * std::sin(2.0 * M_PI * 1000.0 * n / kRate)));
        p[2 * n] = s;
        p[2 * n + 1] = s;
    }
    return b;
}

// ── A. The detector ─────────────────────────────────────────────────────────
void detector()
{
    check(Hl2VoxDetector::thresholdDbfs(0) == 0.0 && Hl2VoxDetector::thresholdDbfs(50) == -30.0
              && Hl2VoxDetector::thresholdDbfs(100) == -60.0,
          "sensitivity law: 0 -> 0 dBFS, 50 -> -30, 100 -> -60");
    check(Hl2VoxDetector::hangMsForDelay(25) == 500 && Hl2VoxDetector::hangMsForDelay(100) == 2000,
          "delay law: TransmitModel raw x 20 ms");

    Hl2VoxDetector v;
    check(v.feed(1.0, kBlock, kRate, true) == Edge::None, "VOX off: full scale never keys");

    v.configure(true, 50, 25);   // -30 dBFS, 500 ms
    check(v.feed(fromDb(-40), kBlock, kRate, true) == Edge::None, "below threshold: no key");
    check(v.feed(fromDb(-20), kBlock, kRate, true) == Edge::Key, "above threshold: key");
    check(v.feed(fromDb(-20), kBlock, kRate, true) == Edge::None, "continued speech: no second key");

    // 500 ms of silence is 25 blocks of 20 ms: held through 24, released on 25.
    int releasedAt = -1;
    for (int i = 1; i <= 30 && releasedAt < 0; ++i) {
        if (v.feed(fromDb(-50), kBlock, kRate, true) == Edge::Release) {
            releasedAt = i;
        }
    }
    check(releasedAt == 25, "holds for exactly the delay (25 x 20 ms), then releases");
    check(!v.holding(), "not holding after release");

    // A pause shorter than the hang is bridged, and speech renews the hang.
    check(v.feed(fromDb(-10), kBlock, kRate, true) == Edge::Key, "speech again: key");
    bool releasedEarly = false;
    for (int i = 0; i < 20; ++i) {
        releasedEarly |= v.feed(fromDb(-50), kBlock, kRate, true) == Edge::Release;
    }
    check(v.feed(fromDb(-10), kBlock, kRate, true) == Edge::None && !releasedEarly,
          "a 400 ms pause inside a 500 ms hang does not release");
    int renewedAt = -1;
    for (int i = 1; i <= 30 && renewedAt < 0; ++i) {
        if (v.feed(fromDb(-50), kBlock, kRate, true) == Edge::Release) {
            renewedAt = i;
        }
    }
    check(renewedAt == 25, "the hang restarts from the last loud block");

    // Not permitted: never keys; a running hold is released at once.
    check(v.feed(1.0, kBlock, kRate, false) == Edge::None, "not permitted: full scale never keys");
    check(v.feed(1.0, kBlock, kRate, true) == Edge::Key, "permitted again: keys");
    check(v.feed(1.0, kBlock, kRate, false) == Edge::Release,
          "permission lost mid-hold: released at once, not after the hang");

    // Switching off mid-hold releases through drop().
    check(v.feed(1.0, kBlock, kRate, true) == Edge::Key, "key");
    v.configure(false, 50, 25);
    check(v.feed(1.0, kBlock, kRate, true) == Edge::Release, "VOX off mid-hold: release");
    check(v.feed(1.0, kBlock, kRate, true) == Edge::None, "VOX off: stays silent");

    // Sensitivity: 100 keys on -55 dBFS, 0 does not key on -1 dBFS.
    Hl2VoxDetector sens;
    sens.configure(true, 100, 0);
    check(sens.feed(fromDb(-55), kBlock, kRate, true) == Edge::Key, "level 100 keys at -55 dBFS");
    check(sens.feed(0.0, kBlock, kRate, true) == Edge::Release, "delay 0: releases on the first quiet block");
    sens.configure(true, 0, 0);
    check(sens.feed(fromDb(-1), kBlock, kRate, true) == Edge::None, "level 0 ignores -1 dBFS");
}

// ── B. The backend ──────────────────────────────────────────────────────────
struct Requests {
    std::vector<bool> edges;
    void attach(Hl2Backend& b)
    {
        QObject::connect(&b, &IRadioBackend::voxKeyingRequested, &b,
                         [this](bool key) { edges.push_back(key); });
    }
};

void backend()
{
    Hl2Backend b;
    Hl2HostTxTestAccess::start(b, QStringLiteral("USB"));
    Requests r;
    r.attach(b);

    check(micSliderToLinear(50) == 1.0, "precondition: mic level 50 is unity gain");
    b.observeTxMicAudio(micBlock(-6), kRate);
    check(r.edges.empty(), "VOX never enabled: loud mic asks nothing");

    b.setVox(true, 50, 5);   // -30 dBFS, 100 ms = 5 blocks
    b.observeTxMicAudio(micBlock(-40), kRate);
    check(r.edges.empty(), "below threshold: nothing");
    b.observeTxMicAudio(micBlock(-20), kRate);
    check(r.edges == std::vector<bool>{true}, "above threshold: exactly one key request");
    for (int i = 0; i < 4; ++i) {
        b.observeTxMicAudio(micBlock(-60), kRate);
    }
    check(r.edges.size() == 1, "held through 4 quiet blocks of a 5-block hang");
    b.observeTxMicAudio(micBlock(-60), kRate);
    check(r.edges == std::vector<bool>({true, false}), "released on the fifth");

    // THE MIC GAIN IS PART OF THE LEVEL: +20 dB of HL2 mic gain lifts a -40
    // dBFS mic over a -30 threshold, which is what micPeak() would read.
    b.setMicGain(75);   // (75 - 50) * 0.8 = +20 dB
    b.observeTxMicAudio(micBlock(-40), kRate);
    check(r.edges.size() == 3 && r.edges.back(), "HL2 mic gain counts: -40 dBFS + 20 dB keys");
    b.setVox(false, 50, 5);
    check(r.edges.size() == 4 && !r.edges.back(), "VOX switched off mid-hold: released at once");
    b.setMicGain(50);

    // Every refusal the backend knows about keeps it silent.
    b.setVox(true, 50, 5);
    r.edges.clear();
    Hl2HostTxTestAccess::setTxAllowed(b, false);
    b.observeTxMicAudio(micBlock(0), kRate);
    check(r.edges.empty(), "transmit not allowed (bridge without ALLOW_TX): never asks");
    Hl2HostTxTestAccess::setTxAllowed(b, true);
    Hl2HostTxTestAccess::setMode(b, QStringLiteral("CW"));
    b.observeTxMicAudio(micBlock(0), kRate);
    check(r.edges.empty(), "CW transmit mode: never asks");
    Hl2HostTxTestAccess::setMode(b, QStringLiteral("LSB"));
    Hl2HostTxTestAccess::setTuning(b, true);
    b.observeTxMicAudio(micBlock(0), kRate);
    check(r.edges.empty(), "TUNE running: never asks");
    Hl2HostTxTestAccess::setTuning(b, false);

    // Permission lost while holding: released at once.
    b.observeTxMicAudio(micBlock(0), kRate);
    Hl2HostTxTestAccess::setTxAllowed(b, false);
    b.observeTxMicAudio(micBlock(0), kRate);
    check(r.edges == std::vector<bool>({true, false}), "transmit gate closes mid-hold: released");
    Hl2HostTxTestAccess::setTxAllowed(b, true);

    // THE BACKSTOP: key, then the mic stops. The audio clock cannot run the
    // hang out, so the wall-clock backstop must (hang 20 ms + 250 ms slack).
    r.edges.clear();
    b.setVox(true, 50, 1);
    b.observeTxMicAudio(micBlock(0), kRate);
    QElapsedTimer t;
    t.start();
    while (r.edges.size() < 2 && t.elapsed() < 5000) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    }
    check(r.edges == std::vector<bool>({true, false}),
          "a mic that stops arriving is released by the backstop, not held forever");

    // Disconnect releases.
    b.observeTxMicAudio(micBlock(0), kRate);
    Hl2HostTxTestAccess::finish(b);
    b.disconnectRadio();
    check(r.edges.size() == 4 && !r.edges.back(), "disconnect releases a running hold");
}

// ── C. The model: the same TX gate a PTT press passes ───────────────────────
class RecordingBackend final : public IRadioBackend {
public:
    RadioCapabilities caps;
    int keyDown{0};
    int keyUp{0};
    RadioCapabilities capabilities() const override { return caps; }
    // FALSE, as atu_seam_gate_test's: the socket-free slice fixture installs
    // only on a disconnected model, and the TX path under test does not ask.
    bool isConnected() const override { return false; }
    void connectRadio(const RadioConnectRequest&) override {}
    void disconnectRadio() override {}
    void setSliceFrequency(int, double) override {}
    void setSliceMode(int, const QString&) override {}
    void setSliceFilter(int, int, int) override {}
    void setSliceAgc(int, const QString&, int) override {}
    void setPanCenter(const QString&, double, PanCenterIntent) override {}
    void setKeying(bool key, const TxCoordinator::Operation&, const TxCoordinator::Completion& done) override
    {
        key ? ++keyDown : ++keyUp;
        done.finish();   // no transport: the queue is empty at once
    }
    void invokeExtension(const QString&, const QString&, quint64, const QVariant&) override {}
};

struct Fixture {
    RadioModel radio;
    RecordingBackend* backend{nullptr};
    SliceModel* slice{nullptr};

    explicit Fixture(bool canTransmit, const QStringList& receiveOnlyModes = {})
    {
        auto owned = std::make_unique<RecordingBackend>();
        backend = owned.get();
        backend->caps.family = QStringLiteral("hl2");
        backend->caps.canTransmit = canTransmit;
        backend->caps.receiveOnlyModes = receiveOnlyModes;
        radio.setBackendForTest(std::move(owned), QStringLiteral("hl2"));
        slice = radio.automationApplySliceFixture(0, QStringLiteral("A")) ? radio.slice(0) : nullptr;
        if (!slice) {
            // Every leg below drives this slice; stop rather than dereference.
            std::fprintf(stderr, "FAIL: slice fixture not installed; cannot continue\n");
            std::exit(1);
        }
        SliceDelta delta;
        delta.txSlice = true;
        delta.panId = QStringLiteral("0x40000000");
        delta.mode = QStringLiteral("USB");
        slice->applyChanges(delta);
    }
    void vox(bool key)
    {
        emit backend->voxKeyingRequested(key);
        QCoreApplication::processEvents();
    }
    void setMode(const QString& mode)
    {
        SliceDelta delta;
        delta.mode = mode;
        slice->applyChanges(delta);
    }
};

void model()
{
    {
        Fixture f(true);
        f.vox(true);
        check(f.backend->keyDown == 0, "VOX off in the transmit model: a backend request keys nothing");
        f.radio.transmitModel().setVoxEnable(true);
        f.vox(true);
        check(f.backend->keyDown == 1, "VOX on, permitted: one key-down through the PTT path");
        check(f.radio.transmitModel().isTransmitting(), "the model shows the over");
        f.vox(true);
        check(f.backend->keyDown == 1, "a repeated request inside the hold does not key again");
        f.vox(false);
        check(f.backend->keyUp >= 1 && !f.radio.transmitModel().isTransmitting(),
              "release unkeys through the same path");
        f.vox(true);
        check(f.backend->keyDown == 2, "the next utterance is a fresh press");
        f.vox(false);
    }
    {
        Fixture f(true);
        f.radio.transmitModel().setVoxEnable(true);
        f.radio.setPanTransmitInhibited(f.slice->panId(), true, QStringLiteral("inhibited"));
        f.vox(true);
        check(f.backend->keyDown == 0 && !f.radio.transmitModel().isTransmitting(),
              "pan TX inhibit: VOX keys nothing");
        f.radio.setPanTransmitInhibited(f.slice->panId(), false);
        f.vox(false);
        f.vox(true);
        check(f.backend->keyDown == 1, "inhibit lifted: VOX keys (positive control)");
        f.vox(false);
    }
    {
        Fixture f(true, {QStringLiteral("WFM")});
        f.radio.transmitModel().setVoxEnable(true);
        f.setMode(QStringLiteral("WFM"));
        f.vox(true);
        check(f.backend->keyDown == 0, "receive-only mode: VOX keys nothing");
    }
    {
        Fixture f(false);
        f.radio.transmitModel().setVoxEnable(true);
        f.vox(true);
        check(f.backend->keyDown == 0, "receive-only backend: VOX keys nothing");
    }
    {
        Fixture f(true);
        f.radio.transmitModel().setVoxEnable(true);
        const auto operatorPtt = f.radio.localTxController();
        check(operatorPtt && operatorPtt->capture(TxController::Activity::Mox).start(),
              "operator MOX keys");
        const int downs = f.backend->keyDown;
        f.vox(true);
        check(f.backend->keyDown == downs, "VOX does not stack a hold on the operator's MOX");
        f.vox(false);
        check(f.radio.transmitModel().isTransmitting(), "a VOX release never unkeys the operator's MOX");
        operatorPtt->current(TxController::Activity::Mox).stop();
        QCoreApplication::processEvents();
        check(!f.radio.transmitModel().isTransmitting(), "operator releases their own MOX");
    }
}
} // namespace

// ── D. Anti-VOX ─────────────────────────────────────────────────────────────
constexpr std::int64_t kMs = 1'000'000;   // ns

void antiVoxDetector()
{
    const double inf = std::numeric_limits<double>::infinity();
    Hl2VoxDetector v;
    v.configure(true, 50, 5);   // -30 dBFS, 100 ms = 5 blocks
    check(v.feed(fromDb(-20), kBlock, kRate, true, inf) == Edge::None && v.antiVoxSuppressed() == 1,
          "reference +inf: a block 10 dB over the threshold does not key, and is counted");
    check(!v.holding(), "a suppressed block starts no hold");
    check(v.feed(fromDb(-20), kBlock, kRate, true, fromDb(-20)) == Edge::None,
          "a block EQUAL to the reference does not key (it must exceed it)");
    check(v.feed(fromDb(-20), kBlock, kRate, true, fromDb(-25)) == Edge::Key,
          "a block over both threshold and reference keys");
    // THE HOLD IS UNTOUCHED: renewal and release ignore the reference.
    check(v.feed(fromDb(-20), kBlock, kRate, true, inf) == Edge::None && v.holding(),
          "inside a hold, reference +inf neither releases nor re-keys");
    int releasedAt = -1;
    for (int i = 1; i <= 10 && releasedAt < 0; ++i) {
        if (v.feed(fromDb(-60), kBlock, kRate, true, inf) == Edge::Release) {
            releasedAt = i;
        }
    }
    check(releasedAt == 5, "the hang runs exactly as without anti-VOX: it can neither lengthen nor cut an over");
    check(v.feed(fromDb(-40), kBlock, kRate, true, 0.0) == Edge::None && v.antiVoxSuppressed() == 2,
          "below the threshold is not an anti-VOX suppression (count unchanged)");
}

void antiVoxClass()
{
    const double inf = std::numeric_limits<double>::infinity();
    Hl2AntiVox a;
    check(a.referenceLinear(0) == 0.0 && !a.levelOn(), "fresh: no reference, level term off");
    const std::int64_t t0 = 1'000 * kMs;
    a.noteOutputRestart(t0);
    check(a.referenceLinear(t0) == inf, "restart: +inf at once");
    check(a.referenceLinear(t0 + (Hl2AntiVox::kRestartHoldMs - 1) * kMs) == inf,
          "still +inf 1 ms before the hold ends");
    check(a.referenceLinear(t0 + Hl2AntiVox::kRestartHoldMs * kMs) == 0.0, "0 when the hold ends");
    // The unkey edge, then the resume 70 ms later: the later one governs.
    Hl2AntiVox b;
    b.noteOutputRestart(t0);
    b.noteOutputRestart(t0 + 70 * kMs);
    check(b.referenceLinear(t0 + (70 + Hl2AntiVox::kRestartHoldMs - 1) * kMs) == inf,
          "a second restart (the resume after the unkey hold) extends the hold");
    b.noteOutputRestart(t0 + 10 * kMs);
    check(b.referenceLinear(t0 + (70 + Hl2AntiVox::kRestartHoldMs - 1) * kMs) == inf,
          "an earlier restart never shortens a hold already running");
    check(Hl2AntiVox::kRestartHoldMs >= 356 - 70 + 530,
          "the hold covers d167's latest re-key after resume (286 ms) plus its 0.53 s tail");

    // LEVEL TERM: off by default -- output is not even remembered.
    Hl2AntiVox c;
    c.noteOutputPeak(0.5, t0);
    check(c.referenceLinear(t0 + kMs) == 0.0, "level term off: our output sets no reference");
    c.setLevelGainDb(0.0);
    c.noteOutputPeak(0.1, t0);
    c.noteOutputPeak(0.05, t0 + 400 * kMs);
    check(std::fabs(c.referenceLinear(t0 + 500 * kMs) - 0.1) < 1e-12,
          "level term at 0 dB: the loudest output of the window");
    check(std::fabs(c.referenceLinear(t0 + 900 * kMs) - 0.05) < 1e-12,
          "the loud block ages out after kLevelWindowMs; the quieter one is still live");
    check(c.referenceLinear(t0 + 1300 * kMs) == 0.0, "all aged out: no reference");
    c.setLevelGainDb(20.0);
    c.noteOutputPeak(0.01, t0 + 2000 * kMs);
    check(std::fabs(c.referenceLinear(t0 + 2000 * kMs) - 0.1) < 1e-9, "+20 dB gain scales the reference x10");
    c.setLevelOff();
    check(c.referenceLinear(t0 + 2000 * kMs) == 0.0 && !c.levelOn(), "level term off again forgets");
}

void antiVoxBackend()
{
    Hl2Backend b;
    Hl2HostTxTestAccess::start(b, QStringLiteral("USB"));
    Requests r;
    r.attach(b);
    b.setVox(true, 50, 5);   // -30 dBFS, 100 ms = 5 blocks

    // POSITIVE CONTROL: with no output restart, d167's loudest re-key level
    // keys at once. Whatever holds it off below is anti-VOX, not the level.
    b.observeTxMicAudio(micBlock(-17.9), kRate);
    check(r.edges == std::vector<bool>{true}, "control: -17.9 dBFS keys when our output has not restarted");
    for (int i = 0; i < 5; ++i) {
        b.observeTxMicAudio(micBlock(-60), kRate);
    }
    check(r.edges == std::vector<bool>({true, false}), "control: released after the hang");

    // Our own over: the receive audio is muted while keyed, and resumes when
    // the unkey hold runs out. That resume is the edge d167 re-keyed on.
    r.edges.clear();
    Hl2HostTxTestAccess::muteRx(b, true);
    QElapsedTimer sinceResume;
    Hl2HostTxTestAccess::muteRx(b, false);
    sinceResume.start();
    const auto before = Hl2HostTxTestAccess::antiVoxSuppressed(b);
    // d167's chatter spanned -17.9 .. -27.1 dBFS at the key ask; feed 300 ms
    // of each extreme -- the whole 246-356 ms re-key window and more.
    for (int i = 0; i < 15; ++i) {
        b.observeTxMicAudio(micBlock(-17.9), kRate);
    }
    for (int i = 0; i < 15; ++i) {
        b.observeTxMicAudio(micBlock(-27.1), kRate);
    }
    const bool inside = sinceResume.elapsed() < Hl2AntiVox::kRestartHoldMs;
    check(inside && r.edges.empty(),
          "after our output restarts, d167's re-key levels (-17.9 and -27.1 dBFS) do NOT key");
    check(Hl2HostTxTestAccess::antiVoxSuppressed(b) - before == 30,
          "every one of those blocks was held off by anti-VOX, not missed by the threshold");

    // SPEECH STILL KEYS. The operator's own d167 key asks were -21.1 and -22.5
    // dBFS -- inside the chatter's range, which is why no level floor could
    // do this. Speech that is still going when the hold ends keys then.
    while (sinceResume.elapsed() <= Hl2AntiVox::kRestartHoldMs + 20) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
    }
    b.observeTxMicAudio(micBlock(-21.1), kRate);
    check(r.edges == std::vector<bool>{true},
          "speech at the operator's measured -21.1 dBFS keys once the hold has ended");
    for (int i = 0; i < 5; ++i) {
        b.observeTxMicAudio(micBlock(-60), kRate);
    }
    check(r.edges == std::vector<bool>({true, false}), "and releases after the ordinary hang");

    // ANTI-VOX FOLLOWS OUR OUTPUT, NOT THE DETECTOR'S OWN RELEASE: that release
    // restarted nothing, so the next sentence keys at once.
    b.observeTxMicAudio(micBlock(-22.5), kRate);
    check(r.edges == std::vector<bool>({true, false, true}),
          "a VOX release with no output restart does not hold the next key off");

    // A restart DURING a hold (MON's unkey, say) cannot cut the over short.
    Hl2HostTxTestAccess::muteRx(b, true);
    Hl2HostTxTestAccess::muteRx(b, false);
    for (int i = 0; i < 4; ++i) {
        b.observeTxMicAudio(micBlock(-60), kRate);
    }
    check(r.edges.size() == 3, "a restart inside a hold leaves the hang running (4 of 5 quiet blocks)");
    b.observeTxMicAudio(micBlock(-60), kRate);
    check(r.edges == std::vector<bool>({true, false, true, false}), "released on the fifth, as ever");

    Hl2HostTxTestAccess::finish(b);
}

int main(int argc, char** argv)
{
    TestSettingsProfile profile(QStringLiteral("hl2-vox"));
    if (!profile.isValid()) {
        return 1;
    }
    QCoreApplication app(argc, argv);
    detector();
    backend();
    model();
    antiVoxDetector();
    antiVoxClass();
    antiVoxBackend();
    std::printf("%d failure(s)\n", failures);
    return failures == 0 ? 0 : 1;
}
