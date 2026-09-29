// Radio Setup controls a radio cannot honour are DIMMED WITH A REASON, not
// left live and silent (gui/RadioSetupControlGate.h).
//
// THE DEFECT. A traced inventory of the Radio Setup dialog (2026-09-29, at
// 20d022d5) found that the Transmit, Phone & CW, Receive, Audio and
// Transverters pages carried ~37 controls whose only effect is Flex wire text.
// RadioModel::sendCmd drops that text on a backend with no command plane, so on
// a Hermes-Lite 2 Max Power, the interlock timings, Mic Bias/+20 dB, CWU/CWL,
// Freq Offset, the 10 MHz reference, Binaural, the line-out/headphone mixer and
// the rest moved and did nothing.
//
// WHAT IS PINNED, in three layers so each can fail on its own:
//
//   1. DECLARATIONS, read from the real backends: the HL2 does NOT engage
//      RadioCapabilities::radioHeldSettings and Flex does; ANAN, Icom, RTL and
//      Sim do not either. Every assertion reads a capability — never
//      caps.family — so this cannot pass against the anti-pattern the
//      capability struct exists to prevent.
//   2. THE GATE: against the HL2's own declared capabilities, EVERY Radio Setup
//      control in kAllRadioSetupControls is unavailable and has a non-empty
//      reason; against Flex's, every one is available. The list is the one the
//      dialog registers from, not a copy.
//   3. THE MECHANISM: a widget registered the way RadioSetupDialog::gateOnRadio
//      registers it is disabled, still shown, and carries the reason on BOTH
//      its tooltip and its accessibleDescription on an HL2 — and on a Flex it
//      is enabled and keeps the help tooltip it was built with.
//
// Socket-free: backends are constructed and asked for capabilities(), never
// connected; the widget layer uses an injected backend double.

#include "TestSettingsProfile.h"
#include "core/AppSettings.h"
#include "core/backends/IRadioBackend.h"
#include "core/backends/anan/AnanBackend.h"
#include "core/backends/flex/FlexBackend.h"
#include "core/backends/hl2/Hl2Backend.h"
#include "core/backends/icom/IcomCivBackend.h"
#include "core/backends/rtl/RtlSdrBackend.h"
#include "core/backends/sim/SimBackend.h"
#include "gui/ControlAvailabilityRegistry.h"
#include "gui/RadioSetupControlGate.h"
#include "models/RadioModel.h"

#include <QApplication>
#include <QLineEdit>
#include <QPushButton>

#include <cstdio>
#include <memory>

using namespace AetherSDR;

static int g_failures = 0;
static void check(bool ok, const char* what)
{
    std::printf("%s %s\n", ok ? "[ OK ]" : "[FAIL]", what);
    if (!ok) {
        ++g_failures;
    }
}

// An injected state source, not a peer: no transport, timers, or firmware
// model. It reports whatever capability payload the test hands it — here, a
// payload copied from a REAL backend's capabilities().
class DeclaringBackend final : public IRadioBackend {
public:
    explicit DeclaringBackend(RadioCapabilities caps) : m_caps(std::move(caps)) {}
    RadioCapabilities capabilities() const override { return m_caps; }
    bool isConnected() const override { return true; }
    void connectRadio(const RadioConnectRequest&) override {}
    void disconnectRadio() override {}
    void setSliceFrequency(int, double) override {}
    void setSliceMode(int, const QString&) override {}
    void setSliceFilter(int, int, int) override {}
    void setSliceAgc(int, const QString&, int) override {}
    void setPanCenter(const QString&, double, PanCenterIntent) override {}
    void setKeying(bool, const TxCoordinator::Operation&,
                   const TxCoordinator::Completion&) override {}
    void invokeExtension(const QString&, const QString&, quint64, const QVariant&) override {}

private:
    RadioCapabilities m_caps;
};

// Registered exactly as RadioSetupDialog::gateOnRadio registers a control.
static void gate(ControlAvailabilityRegistry& registry, QWidget* widget,
                 RadioSetupControl control)
{
    registry.registerSetting(
        widget, radioSetupControlUnavailableReason(control),
        [control](bool, const RadioCapabilities& caps) {
            return radioSetupControlAvailable(control, caps);
        });
}

int main(int argc, char** argv)
{
    TestSettingsProfile profile(QStringLiteral("radio-setup-capability-gating"));
    QApplication app(argc, argv);
    AppSettings::instance().load();

    RadioCapabilities hl2Caps;
    RadioCapabilities flexCaps;
    {
        hl2::Hl2Backend hl2;
        hl2Caps = hl2.capabilities();
        FlexBackend flex;
        flexCaps = flex.capabilities();
    }

    // ---- 1. declarations ----
    check(!hl2Caps.radioHeldSettings.has_value(),
          "HL2 declares no radio-held station settings");
    check(flexCaps.radioHeldSettings.has_value()
              && !flexCaps.radioHeldSettings->setCommands.isEmpty(),
          "Flex engages radioHeldSettings and names the verbs it grants");
    {
        anan::AnanBackend anan;
        check(!anan.capabilities().radioHeldSettings.has_value(),
              "ANAN declares no radio-held station settings");
    }
    {
        icom::IcomCivBackend icom;
        check(!icom.capabilities().radioHeldSettings.has_value(),
              "Icom declares no radio-held station settings");
    }
    {
        rtl::RtlSdrBackend rtl;
        check(!rtl.capabilities().radioHeldSettings.has_value(),
              "RTL declares no radio-held station settings");
    }
    {
        // Demo mode owns a command connection that acks these verbs with
        // success and models no effect — the reason this is a record rather
        // than RadioModel::hasCommandPlane().
        SimBackend sim;
        check(!sim.capabilities().radioHeldSettings.has_value(),
              "Sim declares no radio-held station settings although it acks the verbs");
    }

    // ---- 2. the gate, against the real declarations ----
    {
        bool allDimmedOnHl2 = true;
        bool allReasoned = true;
        bool allLiveOnFlex = true;
        for (const RadioSetupControl control : kAllRadioSetupControls) {
            if (radioSetupControlAvailable(control, hl2Caps)) {
                allDimmedOnHl2 = false;
                std::printf("       still live on HL2: control #%d\n",
                            static_cast<int>(control));
            }
            if (radioSetupControlUnavailableReason(control).trimmed().isEmpty()) {
                allReasoned = false;
                std::printf("       no reason: control #%d\n", static_cast<int>(control));
            }
            if (!radioSetupControlAvailable(control, flexCaps)) {
                allLiveOnFlex = false;
                std::printf("       dimmed on Flex: control #%d\n",
                            static_cast<int>(control));
            }
        }
        check(allDimmedOnHl2, "every gated Radio Setup control is unavailable on the HL2");
        check(allReasoned, "and every one carries a stated reason");
        check(allLiveOnFlex, "every gated Radio Setup control stays live on a Flex");
    }
    // The three rows that gate on a narrower existing capability must follow
    // THAT capability, not the record — otherwise a radio that has multiFLEX
    // or profiles but no other radio-held settings would lose them.
    {
        RadioCapabilities narrow;  // no radioHeldSettings
        narrow.hasMultiClientSessions = true;
        narrow.hasProfiles = true;
        narrow.usesVita49Transport = true;
        check(radioSetupControlAvailable(RadioSetupControl::StationName, narrow)
                  && radioSetupControlAvailable(RadioSetupControl::TxProfile, narrow)
                  && radioSetupControlAvailable(RadioSetupControl::PacketLossConcealment,
                                                narrow),
              "Station Name / TX Profile / PLC follow their own capability");
        check(!radioSetupControlAvailable(RadioSetupControl::MaxPower, narrow),
              "while Max Power still follows radioHeldSettings");
    }

    // ---- 3a. offline: a setting is left exactly as built ----
    //
    // Not Inactive. Greying every setting "not currently active" with no radio
    // attached would re-tint the whole Radio Setup dialog offline, and it is
    // the regression radio_setup_label_theme_token_test caught when these
    // controls were first registered with registerWidget().
    {
        RadioModel model;  // not connected
        ControlAvailabilityRegistry registry(model);
        QLineEdit edit;
        const QString sheet = QStringLiteral("QLineEdit { font-size: 11px; }");
        edit.setStyleSheet(sheet);
        edit.setToolTip(QStringLiteral("help"));
        gate(registry, &edit, RadioSetupControl::MaxPower);
        check(registry.stateOf(&edit) == ControlAvailability::Active,
              "offline: a gated setting is Active, not Inactive");
        check(edit.isEnabled() && edit.styleSheet() == sheet
                  && edit.toolTip() == QStringLiteral("help")
                  && edit.accessibleDescription().isEmpty(),
              "offline: its stylesheet, tooltip and description are untouched");
    }

    // ---- 3b. the mechanism: dimmed, shown, announced ----
    {
        RadioModel model;
        ControlAvailabilityRegistry registry(model);
        model.setBackendForTest(std::make_unique<DeclaringBackend>(hl2Caps),
                                QStringLiteral("hl2"));

        QWidget page;
        auto* maxPower = new QLineEdit(&page);
        maxPower->setToolTip(QStringLiteral("Maximum transmit power, percent"));
        auto* cwl = new QPushButton(QStringLiteral("CWL"), &page);
        page.show();
        gate(registry, maxPower, RadioSetupControl::MaxPower);
        gate(registry, cwl, RadioSetupControl::CwSideband);

        const QString reason = radioSetupControlUnavailableReason(RadioSetupControl::MaxPower);
        check(registry.stateOf(maxPower) == ControlAvailability::Unavailable,
              "HL2: Max Power is Unavailable");
        check(!maxPower->isEnabled() && maxPower->isVisible(),
              "HL2: Max Power is dimmed, not hidden");
        check(maxPower->accessibleDescription() == reason,
              "HL2: the reason reaches a screen reader (accessibleDescription)");
        check(maxPower->toolTip() == reason, "HL2: and the tooltip says the same");
        check(!cwl->isEnabled() && cwl->isVisible()
                  && cwl->accessibleDescription()
                         == radioSetupControlUnavailableReason(RadioSetupControl::CwSideband),
              "HL2: CWL is dimmed with its own reason");

        // Same widgets, a radio that honours them: live again, help restored.
        model.setBackendForTest(std::make_unique<DeclaringBackend>(flexCaps),
                                QStringLiteral("flex"));
        emit model.capabilitiesChanged(true, flexCaps);
        check(registry.stateOf(maxPower) == ControlAvailability::Active,
              "Flex: Max Power is Active again, not greyed as Inactive");
        check(maxPower->isEnabled(), "Flex: Max Power is enabled");
        check(maxPower->toolTip() == QStringLiteral("Maximum transmit power, percent"),
              "Flex: the control keeps the help tooltip it was built with");
        check(maxPower->accessibleDescription().isEmpty(),
              "Flex: no stale reason left on the accessible description");
    }

    std::printf("\n%s (%d failure%s)\n", g_failures ? "FAILED" : "PASSED", g_failures,
                g_failures == 1 ? "" : "s");
    return g_failures ? 1 : 0;
}
