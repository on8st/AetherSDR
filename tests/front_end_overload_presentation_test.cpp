// The visible half of RFC #5535, asserted without a GUI.
//
// #5535 approved the automatic RF-gain loop ON THE CONDITION that it is visible,
// and named two things that must be: the clipping, and the regulator's own
// action. This file pins both, plus the rule that keeps the second one from
// making the radio unusable with a screen reader.
//
// Socket-free and widget-free: everything under test is a pure function of
// AetherSDR::FrontEndOverload. The widget draws what these return and owns no
// rules of its own except the red latch, which needs a clock.

#include "gui/FrontEndOverloadPresentation.h"

#include <QCoreApplication>

#include <cstdio>

using AetherSDR::FrontEndLevel;
using AetherSDR::FrontEndOverload;
using namespace AetherSDR::gui;

static int g_failures = 0;
static void check(bool cond, const char* what)
{
    std::printf("[%s] %s\n", cond ? " OK " : "FAIL", what);
    if (!cond)
        ++g_failures;
}

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);

    // ── The lamp, modelled on the radio's own LEDs ───────────────────────
    check(lampFor(FrontEndLevel::Unobserved) == LampColour::Dark,
          "no reading is DARK, not green -- an indicator must not reassure "
          "about a measurement nobody took");
    check(lampFor(FrontEndLevel::Clean) == LampColour::Green, "clean is green");
    check(lampFor(FrontEndLevel::Marginal) == LampColour::Amber,
          "clipping occasionally is amber");
    check(lampFor(FrontEndLevel::Hot) == LampColour::Red,
          "clipping most of the time is red");
    check(lampFor(FrontEndLevel::AtFloor) == LampColour::Red,
          "and at the floor is red too");

    // ── THE REGULATOR'S OWN ACTION, which is #5535's second condition ────
    //
    // It must be visible to the operator on demand -- but NOT as text beside
    // the lamp (ON8ST, 2026-09-30: "the indicator pill is enough"). It lives in
    // accessibleText(), which the widget sets as tooltip AND accessible
    // description.
    {
        FrontEndOverload s;
        s.level = FrontEndLevel::Clean;
        s.autoArmed = true;
        s.autoOffsetDb = 6;
        check(accessibleText(s).contains(QStringLiteral("6 dB below")),
              "an armed loop holding 6 dB down SAYS SO in the tooltip and "
              "description -- without this, \"my noise floor moved and I "
              "touched nothing\" comes back");
        check(shortText(s) == QStringLiteral("Clean"),
              "and the visible word stays the bare state, with no offset");

        s.autoArmed = false;
        check(!accessibleText(s).contains(QStringLiteral("Automatic gain")),
              "a disarmed loop reports no offset even with a stale number");
    }

    // ── The spoken form is not the written one ───────────────────────────
    {
        FrontEndOverload s;
        s.level = FrontEndLevel::AtFloor;
        s.autoArmed = true;
        s.autoOffsetDb = 12;
        const QString spoken = accessibleText(s);
        check(spoken.length() > shortText(s).length(),
              "the screen reader gets more than the abbreviated line");
        check(spoken.contains(QStringLiteral("12")),
              "including the regulator's action");
        check(spoken.contains(QStringLiteral("ttenuation")),
              "and at the floor it says what the operator must actually do, "
              "because no amount of gain management fixes this one");

        FrontEndOverload u;
        check(accessibleText(u).contains(QStringLiteral("no converter reading")),
              "no reading is spoken as absent, not as clean");
    }

    {
        FrontEndOverload s;
        s.level = FrontEndLevel::Clean;
        s.autoArmed = true;
        check(accessibleText(s).contains(QStringLiteral("armed")),
              "an armed loop at zero offset is still announced as armed -- "
              "\"no offset\" and \"no loop\" are different states");
    }

    {
        FrontEndOverload s;
        s.level = FrontEndLevel::Marginal;
        s.reason = QStringLiteral("reducing gain — clipping occasionally");
        check(accessibleText(s).contains(s.reason),
              "the backend's own words are carried through, not re-invented "
              "above the seam");
    }

    // ── THE GAIN IN LEVELS, NEVER AS AN OFFSET (d168, 2026-09-30) ─────────
    //
    // The operator's bench case in its own numbers: a +48 baseline from his
    // profile, the loop 26 dB down at +22 and still clipping, and the radio's
    // own lowest gain -12. The indicator read "At floor  -26 dB" beside a
    // slider reading "RF Gain: 22 dB" -- two numbers that read as two levels.
    // Every string the operator can see or hear now carries LEVELS and no
    // offset figure.
    {
        FrontEndOverload s;
        s.level = FrontEndLevel::AtFloor;
        s.autoArmed = true;
        s.autoOffsetDb = 26;
        s.gainScaleKnown = true;
        s.settingDb = 48;
        s.effectiveDb = 22;
        s.minGainDb = -12;
        s.reason = QStringLiteral("AT FLOOR and still clipping");

        const QString line = shortText(s);
        check(line == QStringLiteral("At limit"),
              "the visible word is \"At limit\" and nothing else -- no "
              "\"+22 dB (set +48 dB)\" beside the pill");

        const QString spoken = accessibleText(s);
        check(spoken.contains(QStringLiteral("+22 dB"))
                  && spoken.contains(QStringLiteral("+48 dB")),
              "the spoken form names both levels too");
        check(!spoken.contains(QStringLiteral("26")),
              "and no offset figure either -- the tooltip is this same text");
        check(spoken.contains(QStringLiteral("−12 dB"))
                  && spoken.contains(QStringLiteral("by hand")),
              "with native range still below the loop's limit it says the "
              "radio can go lower and that RF gain is the remedy");
        check(!spoken.contains(QStringLiteral("ttenuation")),
              "and does NOT send the operator to buy an attenuator for a "
              "problem the slider solves (a manual +8 was clean on d168)");

        // At the radio's own bottom the old advice is the true one.
        s.settingDb = 14;
        s.effectiveDb = -12;
        const QString bottom = accessibleText(s);
        check(bottom.contains(QStringLiteral("ttenuation"))
                  && bottom.contains(QStringLiteral("−12 dB")),
              "at the radio's lowest gain it still says attenuation or a "
              "filter ahead of the radio is needed");
    }
    {
        FrontEndOverload s;
        s.level = FrontEndLevel::Clean;
        s.autoArmed = true;
        s.autoOffsetDb = 26;
        s.gainScaleKnown = true;
        s.settingDb = 48;
        s.effectiveDb = 22;
        s.minGainDb = -12;
        const QString line = shortText(s);
        check(line == QStringLiteral("Clean"),
              "ON8ST: \"that text should not be there, the indicator pill is "
              "enough\" -- a clean window with gain held shows only \"Clean\"");
        const QString spoken = accessibleText(s);
        check(spoken.contains(QStringLiteral("+22 dB"))
                  && spoken.contains(QStringLiteral("+48 dB"))
                  && !spoken.contains(QStringLiteral("26")),
              "tooltip/description: running gain and setting, in levels, no "
              "offset");

        s.settingDb = 0;
        s.effectiveDb = -6;
        s.autoOffsetDb = 6;
        check(shortText(s) == QStringLiteral("Clean"),
              "a negative running gain adds nothing to the visible word either");
        check(accessibleText(s).contains(QStringLiteral("−6 dB"))
                  && accessibleText(s).contains(QStringLiteral("setting is 0 dB")),
              "and in the description a negative LEVEL carries its sign and "
              "unit, zero is plain 0");
    }

    // ── NO VISIBLE DETAIL, IN ANY STATE ──────────────────────────────────
    //
    // Every level, with the loop armed and holding and a reason attached: the
    // visible word is exactly the state word, and every detail is in the
    // description instead.
    {
        const struct { FrontEndLevel level; const char* word; } words[] = {
            {FrontEndLevel::Unobserved, "No ADC reading"},
            {FrontEndLevel::Clean,      "Clean"},
            {FrontEndLevel::Marginal,   "Clipping"},
            {FrontEndLevel::Hot,        "Clipping hard"},
            {FrontEndLevel::AtFloor,    "At limit"},
        };
        bool wordsOnly = true;
        bool detailsInDescription = true;
        for (const auto& w : words) {
            FrontEndOverload s;
            s.level = w.level;
            s.autoArmed = true;
            s.autoOffsetDb = 26;
            s.gainScaleKnown = true;
            s.settingDb = 48;
            s.effectiveDb = 22;
            s.minGainDb = -12;
            s.reason = QStringLiteral("backend reason");
            if (shortText(s) != QString::fromUtf8(w.word)) {
                std::printf("  shortText(%s) = \"%s\"\n", w.word,
                            shortText(s).toUtf8().constData());
                wordsOnly = false;
            }
            const QString d = accessibleText(s);
            if (!d.contains(QStringLiteral("+22 dB"))
                || !d.contains(QStringLiteral("+48 dB"))
                || !d.contains(s.reason)) {
                detailsInDescription = false;
            }
        }
        check(wordsOnly,
              "every state shows its word alone -- no gain, offset or advice "
              "beside the pill");
        check(detailsInDescription,
              "and every state's description (= tooltip) carries both levels "
              "and the backend's reason");

        FrontEndOverload noScale;
        noScale.level = FrontEndLevel::AtFloor;
        noScale.autoArmed = true;
        noScale.autoOffsetDb = 12;
        check(shortText(noScale) == QStringLiteral("At floor"),
              "a family with no gain scale: the word alone as well");
    }

    // ── WHAT IS WORTH INTERRUPTING A SCREEN READER FOR ───────────────────
    //
    // The inputs move at 10 Hz. A polite announcement on every window would
    // make the radio unusable with a screen reader, which is a WORSE a11y
    // outcome than saying nothing -- so only the transitions that matter speak.
    check(!shouldAnnounce(FrontEndLevel::Clean, FrontEndLevel::Clean),
          "no change says nothing");
    check(shouldAnnounce(FrontEndLevel::Clean, FrontEndLevel::Marginal),
          "it started clipping -- speak");
    check(shouldAnnounce(FrontEndLevel::Clean, FrontEndLevel::Hot),
          "it started clipping hard -- speak");
    check(shouldAnnounce(FrontEndLevel::Hot, FrontEndLevel::Clean),
          "it stopped -- speak, so the operator is not left believing the "
          "warning still stands");
    check(shouldAnnounce(FrontEndLevel::Hot, FrontEndLevel::AtFloor),
          "reaching the floor always speaks: it is the state software cannot fix");
    check(shouldAnnounce(FrontEndLevel::Marginal, FrontEndLevel::AtFloor),
          "from either direction");

    check(!shouldAnnounce(FrontEndLevel::Marginal, FrontEndLevel::Hot),
          "movement WITHIN clipping does not re-announce -- it is the same "
          "news, and at 10 Hz it would be chatter");
    check(!shouldAnnounce(FrontEndLevel::Hot, FrontEndLevel::Marginal),
          "nor does it on the way back down");
    check(!shouldAnnounce(FrontEndLevel::Unobserved, FrontEndLevel::Clean),
          "a first reading arriving is not an event worth speaking over");
    check(!shouldAnnounce(FrontEndLevel::Clean, FrontEndLevel::Unobserved),
          "and losing the reading is not either -- the lamp goes dark, which "
          "is visible, and the radio is probably just gone");

    if (g_failures == 0)
        std::printf("front_end_overload_presentation_test: all checks passed\n");
    else
        std::printf("front_end_overload_presentation_test: %d failure(s)\n", g_failures);
    return g_failures == 0 ? 0 : 1;
}
