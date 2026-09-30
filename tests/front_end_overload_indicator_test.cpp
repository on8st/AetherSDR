// The one rule the indicator widget owns that the pure header cannot: the red
// latch.
//
// Everything else about how this widget speaks is in
// FrontEndOverloadPresentation.h and is asserted by
// front_end_overload_presentation_test without a GUI. The latch needs a clock
// and a widget, so it is here.
//
// WHY THERE IS A LATCH AT ALL. #5535 measured the clean-to-clipped transition at
// 3-5 dB wide on an axis with about 18 dB of usable range. A converter that
// rails for 200 ms and recovers is therefore not noise -- it is the warning that
// the next excursion will not be brief -- and it is exactly the event an
// operator glancing at a panadapter misses. The lamp holds red past the
// recovery so the glance still lands on it.

#include "gui/FrontEndOverloadIndicator.h"

#include <QApplication>
#include <QLabel>

#include <cstdio>

using AetherSDR::FrontEndLevel;
using AetherSDR::FrontEndOverload;
using AetherSDR::gui::LampColour;

static int g_failures = 0;
static void check(bool cond, const char* what)
{
    std::printf("[%s] %s\n", cond ? " OK " : "FAIL", what);
    if (!cond)
        ++g_failures;
}

static FrontEndOverload at(FrontEndLevel l, int offset = 0)
{
    FrontEndOverload s;
    s.level = l;
    s.autoArmed = offset > 0;
    s.autoOffsetDb = offset;
    return s;
}

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    FrontEndOverloadIndicator ind;

    check(ind.shownLamp() == LampColour::Dark,
          "born dark -- nothing has been observed yet");

    ind.setState(at(FrontEndLevel::Clean));
    check(ind.shownLamp() == LampColour::Green, "a clean window is green");

    ind.setState(at(FrontEndLevel::Hot, 6));
    check(ind.shownLamp() == LampColour::Red, "a hot window is red");

    // THE LATCH. The level recovers; the lamp must not.
    ind.setState(at(FrontEndLevel::Clean));
    check(ind.shownLamp() == LampColour::Red,
          "the lamp HOLDS red after the level recovers -- a 200 ms excursion "
          "must still be visible to someone who looked a moment later");

    // ...but the TEXT tells the truth immediately, which is the other half of
    // the same decision: the lamp says what just happened, the line says what
    // is true now.
    check(ind.state().level == FrontEndLevel::Clean,
          "while the state underneath is the recovered one");
    check(AetherSDR::gui::shortText(ind.state())
              .contains(QStringLiteral("Clean")),
          "and the word reads clean, not the latched warning");

    // A marginal window arriving during the latch does not clear it either.
    ind.setState(at(FrontEndLevel::Marginal));
    check(ind.shownLamp() == LampColour::Red,
          "and an amber level during the latch does not downgrade the lamp");

    // Going straight to the floor is red on its own merits, latch or no latch.
    FrontEndOverloadIndicator fresh;
    fresh.setState(at(FrontEndLevel::AtFloor, 12));
    check(fresh.shownLamp() == LampColour::Red, "at floor is red immediately");

    // Unobserved is dark, not green: losing the reading must not look like good
    // news. Checked on a fresh widget so no latch is in play.
    FrontEndOverloadIndicator blank;
    blank.setState(at(FrontEndLevel::Clean));
    blank.setState(at(FrontEndLevel::Unobserved));
    check(blank.shownLamp() == LampColour::Dark,
          "losing the reading goes dark rather than staying green");

    // NO VISIBLE DETAIL (ON8ST, 2026-09-30: "the indicator pill is enough").
    // The widget's one visible label shows the state word only; the running
    // gain and the operator's setting are in the tooltip AND the accessible
    // description, on the widget and on the label, because a tooltip alone is
    // never announced.
    {
        FrontEndOverloadIndicator w;
        FrontEndOverload s;
        s.level = FrontEndLevel::AtFloor;
        s.autoArmed = true;
        s.autoOffsetDb = 26;
        s.gainScaleKnown = true;
        s.settingDb = 48;
        s.effectiveDb = 22;
        s.minGainDb = -12;
        w.setState(s);
        const auto labels = w.findChildren<QLabel*>();
        check(labels.size() == 1,
              "one label and no second detail label beside it");
        const QLabel* word = w.findChild<QLabel*>(QStringLiteral("frontEndStateWord"));
        check(word && word->text() == QStringLiteral("At limit"),
              "the visible text is \"At limit\" alone -- no \"+22 dB (set +48 dB)\"");
        const auto carries = [](const QString& t) {
            return t.contains(QStringLiteral("+22 dB"))
                && t.contains(QStringLiteral("+48 dB"));
        };
        check(carries(w.toolTip()), "the tooltip carries both levels");
        check(carries(w.accessibleDescription()),
              "and so does the accessible description, for a screen reader");
        check(word && carries(word->toolTip()) && carries(word->accessibleDescription()),
              "on the word itself too, so hovering or reading it finds them");

        s.level = FrontEndLevel::Clean;
        w.setState(s);
        check(word && word->text() == QStringLiteral("Clean"),
              "a clean window with gain held reads \"Clean\" alone");
        check(carries(w.toolTip()) && carries(w.accessibleDescription()),
              "with the held gain still in tooltip and description");
    }

    if (g_failures == 0)
        std::printf("front_end_overload_indicator_test: all checks passed\n");
    else
        std::printf("front_end_overload_indicator_test: %d failure(s)\n", g_failures);
    return g_failures == 0 ? 0 : 1;
}
