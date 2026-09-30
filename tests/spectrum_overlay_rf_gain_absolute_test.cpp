// THE RF GAIN SLIDER ON THE RADIO'S OWN SCALE, IN MANUAL AND IN AUTO (d168).
//
// ON8ST, 2026-09-30, on the integration build: "I want the slider to show the
// real gain (from -12 to +48?)" and "i expect to see the upper RF gain to auto
// adapt when i set the auto switch". The backend already echoes the EFFECTIVE
// gain to every pan (Hl2Backend::pushEffectiveLnaGain, pinned in
// hl2_gain_split_test); what this file pins is the widget half:
//
//   * the slider sits on the range the radio published (-12..+48 on the HL2)
//     and shows whatever gain it is handed, in Auto as in manual;
//   * it stays LIVE while Auto is on -- moving it is the operator taking the
//     gain, RFC #5535's "manual override ... not the same as switching it off"
//     -- and a move is emitted as the value asked for;
//   * its description and tooltip carry THAT range. The unarmed text used to
//     be Flex's "-8 to +32 dB in 8 dB steps" on every Auto toggle, over a
//     radio that had just said -12..+48 in 1 dB steps.
//
// Offscreen and widget-only, like spectrum_overlay_auto_rf_gain_refusal_test.

#include "gui/SpectrumOverlayMenu.h"

#include <QApplication>
#include <QCheckBox>
#include <QLabel>
#include <QSlider>
#include <QWidget>

#include <cstdio>

using namespace AetherSDR;

namespace {

int g_failed = 0;

void report(const char* name, bool ok)
{
    std::printf("%s %s\n", ok ? "[ OK ]" : "[FAIL]", name);
    if (!ok) {
        ++g_failed;
    }
}

bool someLabelReads(const QWidget& root, const QString& text)
{
    for (const auto* l : root.findChildren<QLabel*>()) {
        if (l->text() == text) {
            return true;
        }
    }
    return false;
}

} // namespace

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    QWidget parent;
    SpectrumOverlayMenu menu(&parent);

    auto* slider = parent.findChild<QSlider*>(QStringLiteral("antennaRfGainSlider"));
    report("the RF Gain slider is reachable by object name", slider != nullptr);
    if (!slider) {
        return 1;
    }
    int emitted = 0;
    int lastEmitted = 999;
    QObject::connect(&menu, &SpectrumOverlayMenu::rfGainChanged,
                     [&](int g) { ++emitted; lastEmitted = g; });

    // What Hl2Backend publishes on connect: the AD9866's native -12..+48 dB in
    // 1 dB steps (MetisProtocol.h: C4 = 0x40 | (dB + 12)).
    menu.setRfGainRange(-12, 48, 1, QStringLiteral(" dB"));
    menu.setAutoRfGainAvailable(true);
    report("the slider spans the radio's own -12..+48",
           slider->minimum() == -12 && slider->maximum() == 48);

    // ---- MANUAL: the operator's gain, on the absolute scale ----
    menu.setRfGain(8);
    report("manual: +8 sits at +8 and the number beside it reads 8 dB",
           slider->value() == 8 && someLabelReads(parent, QStringLiteral("8 dB")));
    const QString manualDesc = slider->accessibleDescription();
    report("manual: the description names the published range, -12 to +48 dB",
           manualDesc.contains(QStringLiteral("−12 to +48 dB")));
    report("manual: and not Flex's -8..+32 in 8 dB steps",
           !manualDesc.contains(QStringLiteral("8 dB steps"))
               && !manualDesc.contains(QStringLiteral("32")));
    report("manual: the tooltip carries the same range",
           slider->toolTip().contains(QStringLiteral("−12 to +48 dB")));

    // ---- AUTO: the slider shows what the loop is running, and moves with it ----
    //
    // SHOWN, as it is when the operator ticks Auto: the checkbox lives in a
    // popup panel, and the old read-only rule keyed on isVisible(), so with the
    // panel closed it never fired and this test could not have seen it.
    auto* box = parent.findChild<QCheckBox*>(QStringLiteral("antennaAutoRfGainCheck"));
    report("the Auto checkbox is reachable by object name", box != nullptr);
    if (!box) {
        return 1;
    }
    parent.show();
    for (QWidget* w = box; w && w != &parent; w = w->parentWidget()) {
        w->show();
    }
    report("precondition: the Auto checkbox is on screen", box->isVisible());
    menu.setAutoRfGainEnabled(true);
    menu.setRfGain(22);   // the backend's echo of +48 baseline, 26 held (d168)
    report("auto: the slider shows the running gain, +22, on the same scale",
           slider->value() == 22 && someLabelReads(parent, QStringLiteral("22 dB")));
    menu.setRfGain(16);
    report("auto: and follows the loop when it moves",
           slider->value() == 16 && someLabelReads(parent, QStringLiteral("16 dB")));
    report("auto: an echo is not an operator request -- nothing is emitted",
           emitted == 0);

    report("auto: the slider stays LIVE -- RFC #5535's manual override is not "
           "switching Auto off",
           slider->isEnabled());
    const QString autoDesc = slider->accessibleDescription();
    report("auto: the description says it shows the running gain and that "
           "moving it keeps Auto on",
           autoDesc.contains(QStringLiteral("running"))
               && autoDesc.contains(QStringLiteral("stays on"))
               && autoDesc.contains(QStringLiteral("−12 to +48 dB")));
    report("auto: no offset figure and no 'read-only' in it",
           !autoDesc.contains(QStringLiteral("minus whatever"))
               && !autoDesc.contains(QStringLiteral("ead-only")));

    // The operator takes the gain with Auto on.
    slider->setValue(8);
    report("auto: moving the slider asks for exactly the gain it now shows",
           emitted == 1 && lastEmitted == 8);

    // ---- AND BACK TO MANUAL: the range text survives the toggle ----
    menu.setAutoRfGainEnabled(false);
    const QString offDesc = slider->accessibleDescription();
    report("unticking Auto does not write Flex's -8..+32 over the radio's range",
           offDesc.contains(QStringLiteral("−12 to +48 dB"))
               && !offDesc.contains(QStringLiteral("minus 8")));

    return g_failed == 0 ? 0 : 1;
}
