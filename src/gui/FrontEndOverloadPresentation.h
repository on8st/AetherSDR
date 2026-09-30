#pragma once

// HOW THE FRONT-END STATE IS SAID, in three registers: a lamp, a line of text,
// and what a screen reader is told.
//
// SEPARATED FROM THE WIDGET SO IT CAN BE TESTED. Everything here is a pure
// function of AetherSDR::FrontEndOverload -- no Qt widgets, no painting, no
// clock. The widget draws what these return and owns no rules of its own. That
// division is what lets front_end_overload_presentation_test assert the
// behaviour RFC #5535 made a merge condition without instantiating a GUI.
//
// MODELLED ON THE RADIO'S OWN LEDS, which is #5535's wording and not a
// decoration: an operator watching an HL2 already reads clipping off the board,
// so the indicator that replaces that glance should not require learning a new
// vocabulary. Dark when there is nothing to say, green when clean, amber when it
// is starting, red when it is bad -- and red LATCHES BRIEFLY, because a
// converter that rails for 200 ms and recovers is exactly the event a glance
// would miss.

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

// THE REGULATOR'S ACTION, as a short suffix, or empty when there is none to
// report. The operator must be able to see that something moved their gain --
// the second half of #5535's condition.
//
// WITH A GAIN SCALE IT IS SAID IN LEVELS ONLY: "+22 dB (set +48 dB)", where the
// gain the radio is running comes first and the operator's own setting second.
// No offset figure. A bare "−26 dB" beside "RF Gain: 22 dB" reads as a second
// level that disagrees with the first; on 2026-09-30 the operator read it that
// way and could not see that his own setting was +48, which the slider -- it
// shows the running gain -- no longer showed anywhere.
//
// Without a gain scale the family has given nothing to say it in, and the
// offset is all there is.
[[nodiscard]] inline QString offsetText(const FrontEndOverload& s)
{
    if (!s.autoArmed || s.autoOffsetDb <= 0) {
        return {};
    }
    if (s.gainScaleKnown) {
        return QCoreApplication::translate("FrontEndOverload", "%1 (set %2)")
            .arg(signedDbText(s.effectiveDb), signedDbText(s.settingDb));
    }
    return QCoreApplication::translate("FrontEndOverload", "−%1 dB")
        .arg(s.autoOffsetDb);
}

// Whether the loop's limit is also the radio's. At the floor with native range
// still below it, the fix is the operator's hand on RF Gain, not a filter, and
// saying "attenuation ahead of the radio" there sends them the wrong way.
[[nodiscard]] inline bool radioHasGainBelow(const FrontEndOverload& s)
{
    return s.gainScaleKnown && s.effectiveDb > s.minGainDb;
}

// The line beside the lamp. Deliberately short -- it sits next to the RF Gain
// slider, not in a dialog.
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
    const QString off = offsetText(s);
    return off.isEmpty() ? head : QStringLiteral("%1  %2").arg(head, off);
}

// WHAT A SCREEN READER IS TOLD, which is not the same string. The lamp carries
// colour and the line is abbreviated for space; neither survives being read
// aloud, so this spells out the state, the regulator's action and the backend's
// own reason in one sentence. docs/a11y.md asks for exactly this rather than a
// terse label that happens to be technically present.
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
