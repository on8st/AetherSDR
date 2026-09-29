// A dropped Flex verb whose typed twin was APPLIED is not a dead control
// (#5263 follow-up). Socket-free: an injected backend on RadioModel's real
// wiring, no transport, no radio.
//
// Before this, RadioModel emitted commandDropped for every no-command-plane
// drop, and MainWindow turned the first one of a session into "This radio
// doesn't support that control". On a Hermes-Lite 2 the first drop is usually
// RF power or mic gain — both applied through the seam a line later — so the
// operator was told a working control was unsupported and the session's one
// notice was spent before a genuinely dead control could use it.
//
// What is pinned, and why each case exists:
//   * an implemented seam verb makes its twin's drop quiet (the bug);
//   * a verb with no twin is still loud, synchronously (unchanged #5263 path);
//   * a twinned verb the backend did NOT implement stays loud — the case a
//     table of "verbs with seam equivalents" would have got wrong;
//   * an override that declines stays loud;
//   * a host-side receipt covers its own turn only, never a later drop;
//   * one untwinned key makes a multi-key line loud;
//   * host-read TX state (TUNE power) is quiet only on a host-modulating radio.
#include "TestSettingsProfile.h"
#include "core/backends/IRadioBackend.h"
#include "models/CommandDropAccounting.h"
#include "models/RadioModel.h"
#include "models/TransmitModel.h"

#include <QCoreApplication>
#include <QSignalSpy>

#include <cstdio>
#include <memory>

using namespace AetherSDR;

namespace {
int failures = 0;
void check(bool condition, const char* description)
{
    std::printf("[%s] %s\n", condition ? "PASS" : "FAIL", description);
    if (!condition)
        ++failures;
}

// The shape of Hl2Backend as far as this concern goes: owns a drive register
// and a mic gain, has no VOX, and declines a TX passband it cannot write.
class Hl2LikeBackend : public IRadioBackend
{
public:
    RadioCapabilities caps;
    int txPowerCalls = 0;
    int lastTxPower = -1;
    int micGainCalls = 0;
    RadioCapabilities capabilities() const override { return caps; }
    void connectRadio(const RadioConnectRequest&) override {}
    void disconnectRadio() override {}
    bool isConnected() const override { return true; }
    void setSliceFrequency(int, double) override {}
    void setSliceMode(int, const QString&) override {}
    void setSliceFilter(int, int, int) override {}
    void setSliceAgc(int, const QString&, int) override {}
    void setPanCenter(const QString&, double, PanCenterIntent) override {}
    void setKeying(bool, const TxCoordinator::Operation&,
                   const TxCoordinator::Completion&) override {}
    void invokeExtension(const QString&, const QString&, quint64, const QVariant&) override {}

    void setTxPower(int percent) override { ++txPowerCalls; lastTxPower = percent; }
    void setMicGain(int) override { ++micGainCalls; }
    // Present but declining, the IcomCivBackend::setTxFilter shape.
    void setTxFilter(int, int) override { declineIntent(); }
};

Hl2LikeBackend* install(RadioModel& radio, bool hostModulates = false)
{
    auto backend = std::make_unique<Hl2LikeBackend>();
    backend->caps.hostModulates = hostModulates;
    Hl2LikeBackend* pointer = backend.get();
    radio.setBackendForTest(std::move(backend), QStringLiteral("hl2"));
    return pointer;
}

// End the turn: the twinned drops are reconciled by a zero-timer.
void endTurn()
{
    QCoreApplication::processEvents();
    QCoreApplication::processEvents();
}

bool droppedContains(const QSignalSpy& spy, const QString& command)
{
    for (const QList<QVariant>& args : spy) {
        if (args.value(0).toString() == command)
            return true;
    }
    return false;
}

void testAppliedTwinIsQuiet()
{
    RadioModel radio;
    Hl2LikeBackend* backend = install(radio);
    check(!radio.hasCommandPlane(), "fixture has no command plane (the HL2 shape)");
    QSignalSpy dropped(&radio, &RadioModel::commandDropped);

    radio.transmitModel().setRfPower(40);
    endTurn();
    check(backend->txPowerCalls == 1 && backend->lastTxPower == 40,
          "RF power reached the backend through the seam");
    check(dropped.isEmpty(), "RF power applied through the seam raises no drop notice");

    radio.transmitModel().setMicLevel(62);
    endTurn();
    check(backend->micGainCalls == 1, "mic gain reached the backend through the seam");
    check(dropped.isEmpty(), "mic gain (intent emitted BEFORE the wire text) is quiet too");
}

void testDeadControlsStayLoud()
{
    RadioModel radio;
    install(radio);
    QSignalSpy dropped(&radio, &RadioModel::commandDropped);

    radio.transmitModel().setMicBoost(true);
    check(dropped.size() == 1 && droppedContains(dropped, QStringLiteral("mic boost 1")),
          "a verb with no typed twin is dropped loudly and synchronously");

    dropped.clear();
    radio.transmitModel().setVoxEnable(true);
    endTurn();
    check(droppedContains(dropped, QStringLiteral("transmit set vox_enable=1")),
          "a twinned verb the backend never implemented stays loud "
          "(the default setVox declines; a verb table would have hidden this)");

    dropped.clear();
    radio.transmitModel().setTxFilter(100, 2900);
    endTurn();
    check(dropped.size() == 1,
          "an override that declines (writes nothing) leaves its drop loud");
}

void testHostReceiptIsScopedToItsTurn()
{
    RadioModel radio;
    install(radio);
    QSignalSpy dropped(&radio, &RadioModel::commandDropped);

    // The speech processor: no backend override here, so the seam declines —
    // loud unless a host applier (MainWindow's client compressor) receipts it.
    radio.transmitModel().setSpeechProcessorEnable(true);
    endTurn();
    check(dropped.size() == 1, "PROC with no applier anywhere is announced");

    dropped.clear();
    const auto receipt = QObject::connect(
        &radio.transmitModel(), &TransmitModel::speechProcessorCommandIssued, &radio,
        [&radio](bool, int) { radio.noteIntentApplied(ControlIntent::SpeechProcessor); });
    radio.transmitModel().setSpeechProcessorEnable(false);
    endTurn();
    check(dropped.isEmpty(), "PROC receipted by a host applier is quiet");
    QObject::disconnect(receipt);

    // A receipt earned in one turn must not excuse a drop in the next.
    dropped.clear();
    radio.noteIntentApplied(ControlIntent::TxPower);
    endTurn();
    radio.sendCommand(QStringLiteral("transmit set rfpower=10"));
    endTurn();
    check(dropped.size() == 1, "a receipt does not outlive the turn that earned it");

    // One untwinned key makes the whole line loud even with a receipt in hand.
    dropped.clear();
    radio.noteIntentApplied(ControlIntent::TxPower);
    radio.sendCommand(QStringLiteral("transmit set rfpower=10 am_carrier=5"));
    endTurn();
    check(dropped.size() == 1, "a multi-key line with an untwinned key stays loud");
}

void testHostReadStateNeedsHostModulation()
{
    {
        RadioModel radio;
        install(radio, /*hostModulates=*/true);
        QSignalSpy dropped(&radio, &RadioModel::commandDropped);
        radio.transmitModel().setTunePower(15);
        endTurn();
        check(dropped.isEmpty(),
              "TUNE power on a host-modulating radio is read at key-down, not dropped");
    }
    {
        RadioModel radio;
        install(radio, /*hostModulates=*/false);
        QSignalSpy dropped(&radio, &RadioModel::commandDropped);
        radio.transmitModel().setTunePower(15);
        endTurn();
        check(dropped.size() == 1,
              "TUNE power on a radio that neither receives it nor reads it is announced");
    }
}

void testClassifier()
{
    check(controlIntentForCommand(QStringLiteral("transmit set rfpower=40"))
              == ControlIntent::TxPower, "rfpower classifies as TX power");
    check(controlIntentForCommand(QStringLiteral("transmit set filter_low=100 filter_high=2900"))
              == ControlIntent::TxFilter, "both passband keys classify as one TX filter");
    check(!controlIntentForCommand(QStringLiteral("transmit set rfpower=40 miclevel=3")),
          "two different intents on one line have no single twin");
    check(controlIntentForCommand(QStringLiteral("cw break_in 1")) == ControlIntent::CwBreakIn
              && controlIntentForCommand(QStringLiteral("cw break_in_delay 20"))
                     == ControlIntent::CwBreakInDelay,
          "break_in and break_in_delay are distinct");
    check(!controlIntentForCommand(QStringLiteral("mic boost 1")), "mic boost has no twin");
}
} // namespace

int main(int argc, char** argv)
{
    TestSettingsProfile profile(QStringLiteral("aether-command-drop-receipt-test"));
    QCoreApplication app(argc, argv);
    check(profile.isValid(), "isolated settings profile is available");
    testClassifier();
    testAppliedTwinIsQuiet();
    testDeadControlsStayLoud();
    testHostReceiptIsScopedToItsTurn();
    testHostReadStateNeedsHostModulation();
    std::printf("%s\n", failures == 0 ? "ALL PASS" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
