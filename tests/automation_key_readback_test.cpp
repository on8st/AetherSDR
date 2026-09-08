// `key ptt/mox off` must not report success for an unkey that did not happen
// (#5252), and the read-back has to consult a flag that CAN disagree.
//
// WHY THIS TEST EXISTS IN THIS SHAPE. The issue and its triage both propose
// reading `TransmitModel::isTransmitting()` back after `setTransmit(false)`.
// That flag is written synchronously by RadioModel::setTransmit() itself --
// "Optimistic edge gating: TX off: stop immediately to avoid 'stuck TX tail'
// during UNKEY_REQUESTED" -- and on a backend with no radio PTT readback
// (Hl2Backend sets RadioCapabilities::hasRadioPttReadback = false) nothing ever
// writes it from radio state. So a read-back on isTransmitting() asserts its own
// assignment: it is false the instant the unkey is issued, for ever, whatever the
// transmitter is doing. testIsTransmittingIsBlindToAStuckTransmitter() below is
// the positive control that says so out loud, and it passes BEFORE the fix as
// well as after -- it characterises the flag, it does not test the repair.
//
// The flag that can disagree is TransmitModel::isMox(), which
// TransmitModel::applyChanges() assigns from the backend's TransmitDelta and
// which its own comment calls "observed radio state, not this client's transmit
// intent". AutomationServer::txBridgeOwnsCurrentTransmit() already joins it with
// isTuning(), and AutomationServer::releaseEdgeHandsBackPolicing() already
// applies exactly the discipline this verb is missing -- for `keyevent`, whose
// release edge hands policing back ONLY when the transmitter is actually down.
// `doKey`'s keyOff lambda is the path that does not use it.
//
// A STUCK TRANSMITTER, modelled honestly: the backend has reported mox=true and
// goes on reporting it, because that is what "the unkey did not take effect"
// looks like from inside this process. setTransmit(false) clears the local
// optimistic flag and leaves the observed one alone, which is the exact state
// nigelfenton reported on 2026-08-13 -- the bridge saying "off" while every
// other observation said otherwise.

#include "core/AutomationServer.h"
#include "core/backends/IRadioBackend.h"
#include "models/RadioModel.h"

#include <QCoreApplication>
#include <QJsonObject>
#include <QString>

#include <cstdio>

using AetherSDR::AutomationServer;
using AetherSDR::RadioModel;
using AetherSDR::TransmitDelta;

namespace AetherSDR {

// Each automation test builds its own accessor; they are separate executables,
// so there is no ODR conflict with the one in automation_tx_watchdog_test.
class AutomationServerTestAccess
{
public:
    static QJsonObject key(AutomationServer& server, const QString& name,
                           const QString& arg)
    {
        return server.doKey(name, arg);
    }
    static bool bridgeInitiated(const AutomationServer& server)
    {
        return server.m_txBridgeInitiated;
    }
    static qint64 keyedSinceMs(const AutomationServer& server)
    {
        return server.m_txKeyedSinceMs;
    }
    static void setKeyedAtRequestStart(AutomationServer& server, bool keyed)
    {
        server.m_txKeyedAtRequestStart = keyed;
    }
    static void markTxBridgeInitiated(AutomationServer& server)
    {
        server.markTxBridgeInitiated();
    }
};

} // namespace AetherSDR

namespace {

int failures = 0;

void check(bool condition, const char* message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

bool replyOk(const QJsonObject& r)
{
    return r.value(QStringLiteral("ok")).toBool(false);
}

QString replyError(const QJsonObject& r)
{
    return r.value(QStringLiteral("error")).toString();
}

// The backend has told us the radio is keyed. Nothing here goes through
// setTransmit(), so this is observed state and not our own intent -- the same
// route Hl2Backend::setKeying and the Flex interlock decoder use.
void radioReportsMox(RadioModel& radio, bool mox)
{
    TransmitDelta delta;
    delta.mox = mox;
    radio.transmitModel().applyChanges(delta);
}

// Put the bridge in the state a successful `key ptt on` leaves behind: the
// transmitter up by both intent and observation, and the watchdog armed and
// owning it.
//
// Staged through markTxBridgeInitiated() rather than by calling the `key ptt on`
// verb, deliberately: a bare RadioModel has no backend, so a real keyOn is
// refused and rolled back, and these cases are about the UNKEY. Same staging as
// automation_tx_watchdog_test.
void bridgeHasKeyed(AutomationServer& server, RadioModel& radio)
{
    AetherSDR::AutomationServerTestAccess::setKeyedAtRequestStart(server, false);
    radio.transmitModel().setTransmitting(true);
    radioReportsMox(radio, true);
    AetherSDR::AutomationServerTestAccess::markTxBridgeInitiated(server);
}

// ── The positive control: the flag the issue proposes reading is blind ──────
//
// This does NOT test the fix. It tests the instrument, and it is the whole
// argument for not implementing the read-back as written in #5252. It must pass
// both before and after the repair.
void testIsTransmittingIsBlindToAStuckTransmitter()
{
    RadioModel radio;
    // The radio is keyed and says so.
    radio.transmitModel().setTransmitting(true);
    radioReportsMox(radio, true);
    check(radio.transmitModel().isTransmitting(),
          "precondition: the transmitter is up");

    // The unkey is issued and does not take effect: the backend goes on
    // reporting mox.
    radio.setTransmit(false);

    check(!radio.transmitModel().isTransmitting(),
          "isTransmitting() reads false IMMEDIATELY after setTransmit(false) -- "
          "it is the optimistic local flag, not an observation of the radio");
    check(radio.transmitModel().isMox(),
          "isMox() still reports the stuck transmitter -- this is the flag a "
          "read-back has to consult");
}

// ── The finding: an unkey that did not take must not report ok:true ─────────
void testUnkeyThatDidNotTakeEffectIsAnError()
{
    RadioModel radio;
    AutomationServer server;
    server.setRadioModel(&radio);
    server.setTxAllowed(true);

    bridgeHasKeyed(server, radio);
    // The unkey will not take: the backend keeps reporting mox.
    const QJsonObject off =
        AetherSDR::AutomationServerTestAccess::key(server, QStringLiteral("ptt"),
                                                   QStringLiteral("off"));

    check(!replyOk(off),
          "key ptt off does not report ok:true while the radio still reports "
          "transmitting");
    check(replyError(off).contains(QStringLiteral("transmit")),
          "the error names what the radio still reports");
}

// ── The ordering point: the watchdog must survive a failed unkey ────────────
//
// keyOff clears m_txKeyedSinceMs / m_txBridgeInitiated before any read-back, so
// on a failed unkey the backstop that would force-unkey at m_txMaxKeyMs has
// already been disarmed -- precisely when it is most needed. Constitution VI:
// fail closed.
void testFailedUnkeyLeavesTheWatchdogArmed()
{
    RadioModel radio;
    AutomationServer server;
    server.setRadioModel(&radio);
    server.setTxAllowed(true);

    bridgeHasKeyed(server, radio);
    check(AetherSDR::AutomationServerTestAccess::bridgeInitiated(server),
          "precondition: the bridge owns this transmission");

    AetherSDR::AutomationServerTestAccess::key(server, QStringLiteral("ptt"),
                                               QStringLiteral("off"));

    check(AetherSDR::AutomationServerTestAccess::bridgeInitiated(server),
          "a failed unkey leaves the watchdog ARMED -- it is the only remaining "
          "backstop for a transmitter that would not stop");
    check(AetherSDR::AutomationServerTestAccess::keyedSinceMs(server) != 0,
          "a failed unkey keeps the arming timestamp, so m_txMaxKeyMs still "
          "measures from the original key");
}

// ── The control leg, whose correct answer is known in advance ──────────────
void testCleanUnkeyStillReportsOkAndHandsPolicingBack()
{
    RadioModel radio;
    AutomationServer server;
    server.setRadioModel(&radio);
    server.setTxAllowed(true);

    bridgeHasKeyed(server, radio);
    // This time the radio does stop, and says so.
    radioReportsMox(radio, false);

    const QJsonObject off =
        AetherSDR::AutomationServerTestAccess::key(server, QStringLiteral("ptt"),
                                                   QStringLiteral("off"));

    check(replyOk(off), "a clean unkey still reports ok:true");
    check(off.value(QStringLiteral("state")).toString() == QStringLiteral("off"),
          "a clean unkey still reports state:off");
    check(!AetherSDR::AutomationServerTestAccess::bridgeInitiated(server),
          "a clean unkey hands policing back");
    check(AetherSDR::AutomationServerTestAccess::keyedSinceMs(server) == 0,
          "a clean unkey clears the arming timestamp");
}

// mox_toggle while keyed takes the same unkey path and must report the same way.
void testMoxToggleUnkeyThatDidNotTakeEffectIsAnError()
{
    RadioModel radio;
    AutomationServer server;
    server.setRadioModel(&radio);
    server.setTxAllowed(true);

    bridgeHasKeyed(server, radio);
    const QJsonObject off =
        AetherSDR::AutomationServerTestAccess::key(server, QStringLiteral("mox"),
                                                   QString());

    check(!replyOk(off),
          "mox_toggle unkeying a stuck transmitter reports an error, not ok:true");
}

// ── keyOn's asymmetry, which is sharper than the triage states ─────────────
//
// A bare RadioModel has no backend, so RadioCapabilities::canTransmit is false
// and setTransmit(true) refuses through refuseKeyWithInterlock() -- which
// ROLLS BACK the optimistic flag. So on the key-on side the read-back is not
// merely symmetric decoration: it is the side where isTransmitting() actually
// carries information, because the refusal paths clear it. keyOn reporting
// ok:true here is a key that provably never happened.
void testKeyOnThatWasRefusedIsNotReportedAsOk()
{
    RadioModel radio;
    AutomationServer server;
    server.setRadioModel(&radio);
    server.setTxAllowed(true);

    const QJsonObject on =
        AetherSDR::AutomationServerTestAccess::key(server, QStringLiteral("ptt"),
                                                   QStringLiteral("on"));

    check(!radio.transmitModel().isTransmitting(),
          "precondition: a receive-only model refused the key and rolled the "
          "optimistic flag back");
    check(!replyOk(on),
          "key ptt on does not report ok:true for a key the radio refused");
}

// The phantom watchdog: a key that never took still arms the backstop, which
// then tries to unkey a transmitter that was never on.
void testRefusedKeyOnDoesNotArmThePhantomWatchdog()
{
    RadioModel radio;
    AutomationServer server;
    server.setRadioModel(&radio);
    server.setTxAllowed(true);

    AetherSDR::AutomationServerTestAccess::key(server, QStringLiteral("ptt"),
                                               QStringLiteral("on"));

    check(!AetherSDR::AutomationServerTestAccess::bridgeInitiated(server),
          "a refused key does not arm the watchdog on a transmitter that was "
          "never on");
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);

    testIsTransmittingIsBlindToAStuckTransmitter();
    testUnkeyThatDidNotTakeEffectIsAnError();
    testFailedUnkeyLeavesTheWatchdogArmed();
    testCleanUnkeyStillReportsOkAndHandsPolicingBack();
    testMoxToggleUnkeyThatDidNotTakeEffectIsAnError();
    testKeyOnThatWasRefusedIsNotReportedAsOk();
    testRefusedKeyOnDoesNotArmThePhantomWatchdog();

    if (failures == 0) {
        std::puts("Automation key read-back tests passed");
    }
    return failures == 0 ? 0 : 1;
}
