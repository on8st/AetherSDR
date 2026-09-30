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
    std::printf("%d failure(s)\n", failures);
    return failures == 0 ? 0 : 1;
}
