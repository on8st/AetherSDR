#pragma once

// How front-end state is presented: lamp colour, one state word and the
// screen-reader/tooltip text, as pure functions of FrontEndOverload so
// front_end_overload_presentation_test covers RFC #5535 without a GUI. Modelled
// on the radio's own LEDs: dark (nothing to say), green (clean), amber
// (starting), red (bad); red latches briefly so a 200 ms rail is not missed.

#include "core/backends/FrontEndOverload.h"

#include <QCoreApplication>
#include <QString>

namespace AetherSDR::gui {

enum class LampColour { Dark, Green, Amber, Red };

[[nodiscard]] inline LampColour lampFor(FrontEndLevel level)
{
    switch (level) {
    case FrontEndLevel::Unobserved: return LampColour::Dark;
    case FrontEndLevel::Clean:      return LampColour::Green;
    case FrontEndLevel::Marginal:   return LampColour::Amber;
    case FrontEndLevel::Hot:        return LampColour::Red;
    case FrontEndLevel::AtFloor:    return LampColour::Red;
    }
    return LampColour::Dark;
}

// A gain level as the operator reads it: "+48", "−6", "0". The sign is explicit
// on purpose, so that a LEVEL can never be mistaken for an OFFSET.
[[nodiscard]] inline QString signedNumberText(int db)
{
    if (db > 0) {
        return QStringLiteral("+%1").arg(db);
    }
    if (db < 0) {
        return QStringLiteral("−%1").arg(-db);
    }
    return QStringLiteral("0");
}

[[nodiscard]] inline QString signedDbText(int db)
{
    return signedNumberText(db) + QStringLiteral(" dB");
}

// Whether the loop's limit is also the radio's. At the floor with native range
// still below it, the fix is the operator's hand on RF Gain, not a filter, and
// saying "attenuation ahead of the radio" there sends them the wrong way.
[[nodiscard]] inline bool radioHasGainBelow(const FrontEndOverload& s)
{
    return s.gainScaleKnown && s.effectiveDb > s.minGainDb;
}

// The word beside the lamp, and only the word ("Clean", "Clipping", "At
// limit"). The regulator's action is in accessibleText(), set as tooltip and
// accessible description; a level beside the gain slider reads as a second one.
[[nodiscard]] inline QString shortText(const FrontEndOverload& s)
{
    const auto tr_ = [](const char* k) {
        return QCoreApplication::translate("FrontEndOverload", k);
    };
    QString head;
    switch (s.level) {
    case FrontEndLevel::Unobserved: head = tr_("No ADC reading"); break;
    case FrontEndLevel::Clean:      head = tr_("Clean");          break;
    case FrontEndLevel::Marginal:   head = tr_("Clipping");       break;
    case FrontEndLevel::Hot:        head = tr_("Clipping hard");  break;
    // "limit", not "floor": the floor is an offset below the operator's
    // setting, and beside a gain readout the word reads as a level.
    case FrontEndLevel::AtFloor:
        head = s.gainScaleKnown ? tr_("At limit") : tr_("At floor");
        break;
    }
    return head;
}

// What a screen reader and the tooltip are told: the state, the regulator's
// action in levels and the backend's reason, in one sentence. It is the only
// place those details appear, so it is set as accessible description as well
// as tooltip (docs/a11y.md).
[[nodiscard]] inline QString accessibleText(const FrontEndOverload& s)
{
    const auto tr_ = [](const char* k) {
        return QCoreApplication::translate("FrontEndOverload", k);
    };
    QString out;
    switch (s.level) {
    case FrontEndLevel::Unobserved:
        out = tr_("Front end: no converter reading available");
        break;
    case FrontEndLevel::Clean:
        out = tr_("Front end clean");
        break;
    case FrontEndLevel::Marginal:
        out = tr_("Front end clipping occasionally");
        break;
    case FrontEndLevel::Hot:
        out = tr_("Front end clipping most of the time");
        break;
    case FrontEndLevel::AtFloor:
        if (!s.gainScaleKnown) {
            out = tr_("Front end still clipping at the automatic gain floor. "
                      "Attenuation or a filter ahead of the radio is needed.");
        } else if (radioHasGainBelow(s)) {
            out = tr_("Front end still clipping at RF gain %1, the lowest the "
                      "automatic gain may go from your setting of %2. The radio "
                      "can go down to %3: lower the RF gain by hand")
                      .arg(signedDbText(s.effectiveDb), signedDbText(s.settingDb),
                           signedDbText(s.minGainDb));
        } else {
            out = tr_("Front end still clipping at RF gain %1, the lowest this "
                      "radio has. Attenuation or a filter ahead of the radio is "
                      "needed")
                      .arg(signedDbText(s.effectiveDb));
        }
        break;
    }
    if (s.gainScaleKnown && s.autoArmed && s.autoOffsetDb > 0) {
        // In levels. At the limit the sentence above has already said both.
        if (s.level != FrontEndLevel::AtFloor) {
            out += QStringLiteral(". ")
                + tr_("Automatic gain is running the RF gain at %1; your own "
                      "setting is %2")
                      .arg(signedDbText(s.effectiveDb), signedDbText(s.settingDb));
        }
    } else if (s.autoArmed && s.autoOffsetDb > 0) {
        out += QStringLiteral(". ")
            + tr_("Automatic gain is holding %1 dB below your setting")
                  .arg(s.autoOffsetDb);
    } else if (s.autoArmed) {
        out += QStringLiteral(". ") + tr_("Automatic gain is armed and holding");
    }
    if (!s.reason.isEmpty()) {
        out += QStringLiteral(". ") + s.reason;
    }
    return out;
}

// WHETHER A CHANGE IS WORTH INTERRUPTING A SCREEN READER FOR.
//
// A polite announcement on every 10 Hz telemetry window would make the radio
// unusable with a screen reader, which is a worse a11y outcome than saying
// nothing. So only TRANSITIONS THAT MATTER speak: becoming un-clean, reaching
// the floor, and recovering to clean. Movement within clipping (Marginal to Hot
// and back) does not re-announce, and neither does the offset changing on its
// own -- both stay readable on demand through accessibleText().
[[nodiscard]] inline bool shouldAnnounce(FrontEndLevel before, FrontEndLevel after)
{
    if (before == after) {
        return false;
    }
    const auto clipping = [](FrontEndLevel l) {
        return l == FrontEndLevel::Marginal || l == FrontEndLevel::Hot
            || l == FrontEndLevel::AtFloor;
    };
    if (after == FrontEndLevel::AtFloor) {
        return true;   // the state software cannot fix
    }
    if (!clipping(before) && clipping(after)) {
        return true;   // it started
    }
    if (clipping(before) && after == FrontEndLevel::Clean) {
        return true;   // it stopped
    }
    return false;
}

}  // namespace AetherSDR::gui
