// The automation bridge may switch VOX OFF, and only off.
//
// d167 D-vox (hl2-lab, 2026-09-30): VOX chattered 15 overs against an approval
// of 4, and the runner's `invoke "VOX voice-operated transmit" setChecked
// false` was refused -- the Phone applet's VOX button reached the TX guard
// through its NAME ("...transmit"), a name-matched control has no scoped
// action, and invokeTxAction() refused the safe direction. The applet now
// registers a disarm-only scoped action on the button. This test pins both
// halves: the disarm is accepted and lands in the model; every arming verb is
// still refused; and without bridge TX permission nothing changes (the guard
// still blocks the invoke outright -- upstream behaviour, not widened here).
//
// No radio, no socket, no transmission: a bare RadioModel's TransmitModel.

#include "TestEventLoop.h"
#include "TestSettingsProfile.h"
#include "core/AutomationServer.h"
#include "core/TxKeyingMarker.h"
#include "gui/PhoneApplet.h"
#include "models/RadioModel.h"
#include "models/TransmitModel.h"

#include <QApplication>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPushButton>
#include <QSignalBlocker>

#include <atomic>
#include <functional>
#include <cstdio>

namespace AetherSDR {
class AutomationServerTestAccess {
public:
    static QJsonObject request(AutomationServer& server, const QJsonObject& object)
    {
        return server.handleLine(QJsonDocument(object).toJson(QJsonDocument::Compact), nullptr);
    }
};
} // namespace AetherSDR

using namespace AetherSDR;

namespace {

int failures = 0;
std::atomic<int> nameMatchWarnings{0};
QtMessageHandler previousHandler = nullptr;

void check(bool ok, const char* what)
{
    std::printf("%s %s\n", ok ? "[ OK ]" : "[FAIL]", what);
    failures += ok ? 0 : 1;
}

void captureNameMatch(QtMsgType type, const QMessageLogContext& ctx, const QString& msg)
{
    if (msg.contains(QStringLiteral("TX guard fell back to name match on VOX"))) {
        ++nameMatchWarnings;
    }
    if (previousHandler) {
        previousHandler(type, ctx, msg);
    }
}

const QString kVox = QStringLiteral("VOX voice-operated transmit");

QJsonObject invoke(AutomationServer& server, const QString& action, const QString& value = {})
{
    QJsonObject o{{QStringLiteral("cmd"), QStringLiteral("invoke")},
                  {QStringLiteral("target"), kVox},
                  {QStringLiteral("action"), action}};
    if (!value.isEmpty()) {
        o[QStringLiteral("value")] = value;
    }
    return AutomationServerTestAccess::request(server, o);
}

bool ok(const QJsonObject& reply) { return reply.value(QStringLiteral("ok")).toBool(); }

// Let deferred invokes run; true once `cond` holds.
bool settle(const std::function<bool()>& cond) { return AetherTest::waitFor(cond, 2000); }

// Run the event loop briefly so a (wrongly) deferred arm would have landed.
void drain()
{
    AetherTest::waitFor([] { return false; }, 100);
}

} // namespace

int main(int argc, char** argv)
{
    TestSettingsProfile profile(QStringLiteral("aether-phone-applet-vox-disarm-test"));
    if (!profile.isValid()) {
        std::fprintf(stderr, "FAIL could not create isolated settings profile\n");
        return 1;
    }
    qunsetenv("AETHER_AUTOMATION_ALLOW_TX");
    previousHandler = qInstallMessageHandler(captureNameMatch);
    QApplication app(argc, argv);

    RadioModel radio;
    TransmitModel& tx = radio.transmitModel();
    PhoneApplet applet;
    applet.setTransmitModel(&tx);
    applet.show();
    AetherTest::waitFor([&] { return applet.isVisible(); }, 2000);

    auto* button = [&]() -> QPushButton* {
        for (auto* b : applet.findChildren<QPushButton*>()) {
            if (b->accessibleName() == kVox) {
                return b;
            }
        }
        return nullptr;
    }();
    check(button != nullptr, "the Phone applet has the VOX button the runner targets");
    if (!button) {
        return 1;
    }
    check(button->property(kTxKeyingProperty).toBool(),
          "the VOX button carries the positive TX marker, not only a keyword in its name");

    AutomationServer server;
    server.setRadioModel(&radio);
    server.setTxAllowed(true);

    // ── Disarm: accepted, and it lands in the model ─────────────────────────
    tx.setVoxEnable(true);
    check(tx.voxEnable() && button->isChecked(), "precondition: VOX armed, button shows it");
    nameMatchWarnings = 0;
    const QJsonObject off = invoke(server, QStringLiteral("setChecked"), QStringLiteral("false"));
    check(ok(off), "invoke setChecked false is ACCEPTED (d167: refused, 'no scoped action')");
    check(settle([&] { return !tx.voxEnable() && !button->isChecked(); }),
          "the disarm reaches TransmitModel: voxEnable reads false, the button follows");
    check(nameMatchWarnings == 0,
          "the guard no longer falls back to a name match on VOX (the d167 log line)");

    // Whatever the button shows: model armed while the button reads off.
    tx.setVoxEnable(true);
    {
        const QSignalBlocker block(button);
        button->setChecked(false);
    }
    check(tx.voxEnable() && !button->isChecked(), "precondition: model armed, button reads off");
    check(ok(invoke(server, QStringLiteral("setChecked"), QStringLiteral("0"))),
          "disarm accepted with the button already unchecked");
    check(settle([&] { return !tx.voxEnable(); }),
          "a disarm switches the MODEL off even when the button disagrees");

    // Idempotent: disarming a VOX that is off is accepted and leaves it off.
    check(ok(invoke(server, QStringLiteral("setChecked"), QStringLiteral("false"))),
          "disarming an already-off VOX is accepted");
    drain();
    check(!tx.voxEnable(), "and VOX stays off");

    // ── Arming: every verb still refused ────────────────────────────────────
    const QJsonObject on = invoke(server, QStringLiteral("setChecked"), QStringLiteral("true"));
    const QJsonObject click = invoke(server, QStringLiteral("click"));
    const QJsonObject toggle = invoke(server, QStringLiteral("toggle"));
    drain();
    check(!ok(on) && !ok(click) && !ok(toggle),
          "setChecked true, click and toggle are all refused: the bridge cannot arm VOX here");
    check(!tx.voxEnable() && !button->isChecked(), "and nothing armed VOX behind the refusal");

    // ── Without bridge TX permission: unchanged ─────────────────────────────
    // The guard blocks a keying control outright when TX is not allowed. That
    // includes this disarm; it is upstream's rule and is NOT widened here.
    server.setTxAllowed(false);
    tx.setVoxEnable(true);
    const QJsonObject offNoTx = invoke(server, QStringLiteral("setChecked"), QStringLiteral("false"));
    const QJsonObject onNoTx = invoke(server, QStringLiteral("setChecked"), QStringLiteral("true"));
    drain();
    check(!ok(offNoTx) && !ok(onNoTx),
          "without TX permission the invoke is still blocked, both directions (unchanged)");
    check(tx.voxEnable(), "and the model was not touched");
    tx.setVoxEnable(false);

    qInstallMessageHandler(previousHandler);
    std::printf("%d failure(s)\n", failures);
    return failures == 0 ? 0 : 1;
}
