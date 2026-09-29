#pragma once

// Which Radio Setup controls a connected radio can honour, and what the dialog
// says when it cannot.
//
// THE DEFECT THIS CLOSES. A traced inventory of the Radio Setup dialog
// (2026-09-29, at 20d022d5) found ~37 rows whose only effect is Flex wire text —
// `interlock set`, `transmit set max_power_level=`, `mic bias`, `cw cwl_enabled`,
// `radio set freq_error_ppb=`, `radio oscillator`, `mixer lineout gain`, … —
// and RadioModel::sendCmd drops that text when the backend owns no command
// plane. On a Hermes-Lite 2 (and ANAN, Icom, RTL) every one of those controls
// moved and did nothing: the HERMES.md §17 dead-control shape. At most one
// generic status-bar notice per session hinted that something had been dropped.
//
// THE FIX IS THE THREE-STATE DOCTRINE, NOT A HIDE. Each control stays where it
// is, dimmed, with a reason that reaches a screen reader
// (docs/style/theme-style-guide.md §4a). The dialog registers each one with
// ControlAvailabilityRegistry using the predicate and reason below, so the
// mechanism is the one the doctrine names rather than a parallel one.
//
// DECLARED CAPABILITIES ONLY. Every predicate reads RadioCapabilities — never
// the family string (#5554) — so ANAN, Icom, RTL and demo mode get the same
// honest answer as the HL2 without an edit here. Most rows ask
// RadioCapabilities::radioHeldSettings; three rows have a narrower capability
// that already said the same thing and are gated on that instead:
//   * Station Name       — hasMultiClientSessions (it names this client to
//                          OTHER multiFLEX stations, per its own tooltip);
//   * TX Profile         — hasProfiles (the list is empty without it);
//   * Packet-loss conc.  — usesVita49Transport (only the VITA-49
//                          PanadapterStream reads the setting).
//
// Not here: TX Band Settings. That button fires MainWindow's m_txBandAction,
// whose enable state MainWindow already decides from RadioModel::
// hasCommandPlane(); the dialog mirrors that exact condition so the button and
// the menu entry cannot disagree (RadioSetupDialog::buildTxTab).
//
// WHAT IT IS NOT. A dimmed control is not the safety mechanism (Principle VI).
// None of these rows keys the transmitter; they were dead, not dangerous.

#include "core/backends/RadioCapabilities.h"

#include <QCoreApplication>
#include <QString>

#include <array>

namespace AetherSDR {

enum class RadioSetupControl {
    StationName,            // Radio page: Station Name
    InterlockTimings,       // Transmit: ACC TX / TX Delay / RCA TX1-3 / Timeout
    TxProfile,              // Transmit: TX profile combo (Timings group)
    InterlockPolarity,      // Transmit: Interlocks - TX REQ, RCA / Accessory
    MaxPower,               // Transmit: Max Power %
    ShowTxInWaterfall,      // Transmit: Show TX in Waterfall
    MicBiasBoost,           // Phone & CW: Mic Bias Voltage, Mic +20 dB Boost
    MeterInReceive,         // Phone & CW: Level Meter During Receive
    CwSideband,             // Phone & CW: Sideband CWU / CWL
    CwxSync,                // Phone & CW: CWX Sync
    RttyMarkDefault,        // Phone & CW: RTTY Mark Default
    FrequencyOffset,        // Receive: Cal Frequency / Start / Freq Offset (ppb)
    OscillatorSource,       // Receive: 10 MHz Reference source
    MuteLocalWhenRemote,    // Receive: Mute local audio when remote
    BinauralReceive,        // Receive: Binaural audio
    RadioAudioOutputs,      // Audio: Line Out / Headphone gain and mute
    PacketLossConcealment,  // Audio: Smooth packet loss
    RadioSideRecording,     // Audio: Record Mode "Radio Side"
    TransverterCreate,      // Transverters: Create New Transverter
};

inline constexpr std::array<RadioSetupControl, 19> kAllRadioSetupControls{
    RadioSetupControl::StationName,
    RadioSetupControl::InterlockTimings,
    RadioSetupControl::TxProfile,
    RadioSetupControl::InterlockPolarity,
    RadioSetupControl::MaxPower,
    RadioSetupControl::ShowTxInWaterfall,
    RadioSetupControl::MicBiasBoost,
    RadioSetupControl::MeterInReceive,
    RadioSetupControl::CwSideband,
    RadioSetupControl::CwxSync,
    RadioSetupControl::RttyMarkDefault,
    RadioSetupControl::FrequencyOffset,
    RadioSetupControl::OscillatorSource,
    RadioSetupControl::MuteLocalWhenRemote,
    RadioSetupControl::BinauralReceive,
    RadioSetupControl::RadioAudioOutputs,
    RadioSetupControl::PacketLossConcealment,
    RadioSetupControl::RadioSideRecording,
    RadioSetupControl::TransverterCreate,
};

// Can the connected radio honour this control? Takes the capability payload,
// matching ControlAvailabilityRegistry::AvailabilityPredicate; the registry
// supplies the permissive-on-disconnect rule itself.
[[nodiscard]] inline bool radioSetupControlAvailable(RadioSetupControl control,
                                                     const RadioCapabilities& caps)
{
    switch (control) {
        case RadioSetupControl::StationName:
            return caps.hasMultiClientSessions;
        case RadioSetupControl::TxProfile:
            return caps.hasProfiles;
        case RadioSetupControl::PacketLossConcealment:
            return caps.usesVita49Transport;
        case RadioSetupControl::InterlockTimings:
        case RadioSetupControl::InterlockPolarity:
        case RadioSetupControl::MaxPower:
        case RadioSetupControl::ShowTxInWaterfall:
        case RadioSetupControl::MicBiasBoost:
        case RadioSetupControl::MeterInReceive:
        case RadioSetupControl::CwSideband:
        case RadioSetupControl::CwxSync:
        case RadioSetupControl::RttyMarkDefault:
        case RadioSetupControl::FrequencyOffset:
        case RadioSetupControl::OscillatorSource:
        case RadioSetupControl::MuteLocalWhenRemote:
        case RadioSetupControl::BinauralReceive:
        case RadioSetupControl::RadioAudioOutputs:
        case RadioSetupControl::RadioSideRecording:
        case RadioSetupControl::TransverterCreate:
            return caps.radioHeldSettings.has_value();
    }
    return false;
}

// What the dimmed control says, verbatim, as its tooltip and accessible
// description. Plain words about WHY — the radio lacks the thing — and, where
// there is one, where the operator goes instead. Never empty: an unavailable
// control with no reason is the #4896 defect.
[[nodiscard]] inline QString radioSetupControlUnavailableReason(RadioSetupControl control)
{
    const char* text = "";
    switch (control) {
        case RadioSetupControl::StationName:
            text = QT_TRANSLATE_NOOP("RadioSetupControlGate",
                "This radio has no multi-client sessions, so there are no other "
                "stations to identify this client to — nothing to set.");
            break;
        case RadioSetupControl::InterlockTimings:
            text = QT_TRANSLATE_NOOP("RadioSetupControlGate",
                "This radio has no radio-side transmit interlock timings for this "
                "client to set — nothing to set.");
            break;
        case RadioSetupControl::TxProfile:
            text = QT_TRANSLATE_NOOP("RadioSetupControlGate",
                "This radio has no radio-side transmit profiles — nothing to select.");
            break;
        case RadioSetupControl::InterlockPolarity:
            text = QT_TRANSLATE_NOOP("RadioSetupControlGate",
                "This radio has no radio-side TX REQ inputs whose polarity this "
                "client can set — nothing to set.");
            break;
        case RadioSetupControl::MaxPower:
            text = QT_TRANSLATE_NOOP("RadioSetupControlGate",
                "This radio has no radio-side maximum power limit — nothing to set. "
                "Transmit drive is set with the RF Power control.");
            break;
        case RadioSetupControl::ShowTxInWaterfall:
            text = QT_TRANSLATE_NOOP("RadioSetupControlGate",
                "This radio has no radio-side setting for showing transmit in the "
                "waterfall — nothing to set.");
            break;
        case RadioSetupControl::MicBiasBoost:
            text = QT_TRANSLATE_NOOP("RadioSetupControlGate",
                "This radio has no radio-side microphone bias supply or +20 dB "
                "preamp for this client to switch — nothing to set.");
            break;
        case RadioSetupControl::MeterInReceive:
            text = QT_TRANSLATE_NOOP("RadioSetupControlGate",
                "This radio has no radio-side setting for metering the microphone "
                "during receive — nothing to set.");
            break;
        case RadioSetupControl::CwSideband:
            text = QT_TRANSLATE_NOOP("RadioSetupControlGate",
                "This radio takes the CW sideband from the slice mode (CWU or CWL), "
                "so there is no separate sideband setting — choose CWL as the mode.");
            break;
        case RadioSetupControl::CwxSync:
            text = QT_TRANSLATE_NOOP("RadioSetupControlGate",
                "This radio has no radio-side CWX keyer to synchronise with the "
                "paddle keyer — nothing to set.");
            break;
        case RadioSetupControl::RttyMarkDefault:
            text = QT_TRANSLATE_NOOP("RadioSetupControlGate",
                "This radio has no radio-side RTTY mark default for this client to "
                "set — nothing to set.");
            break;
        case RadioSetupControl::FrequencyOffset:
            text = QT_TRANSLATE_NOOP("RadioSetupControlGate",
                "This radio has no radio-side frequency calibration for this client "
                "to set. Where this client corrects the frequency itself, that is on "
                "the Calibration page.");
            break;
        case RadioSetupControl::OscillatorSource:
            text = QT_TRANSLATE_NOOP("RadioSetupControlGate",
                "This radio has no selectable 10 MHz reference for this client to "
                "switch — nothing to set.");
            break;
        case RadioSetupControl::MuteLocalWhenRemote:
            text = QT_TRANSLATE_NOOP("RadioSetupControlGate",
                "This radio has no radio-side local audio to mute during remote "
                "operation — nothing to set.");
            break;
        case RadioSetupControl::BinauralReceive:
            text = QT_TRANSLATE_NOOP("RadioSetupControlGate",
                "This radio has no radio-side binaural receive setting — nothing "
                "to set.");
            break;
        case RadioSetupControl::RadioAudioOutputs:
            text = QT_TRANSLATE_NOOP("RadioSetupControlGate",
                "This radio has no line-out or headphone mixer of its own for this "
                "client to set — nothing to set.");
            break;
        case RadioSetupControl::PacketLossConcealment:
            text = QT_TRANSLATE_NOOP("RadioSetupControlGate",
                "This radio does not stream its audio as VITA-49 packets, so there "
                "is no packet loss for this to conceal — nothing to set.");
            break;
        case RadioSetupControl::RadioSideRecording:
            text = QT_TRANSLATE_NOOP("RadioSetupControlGate",
                "This radio has no recorder of its own — use Client Side recording.");
            break;
        case RadioSetupControl::TransverterCreate:
            text = QT_TRANSLATE_NOOP("RadioSetupControlGate",
                "This radio keeps no transverter table of its own for this client to "
                "add to — nothing to set.");
            break;
    }
    return QCoreApplication::translate("RadioSetupControlGate", text);
}

// The TX Band Settings button's reason. Its enable state mirrors MainWindow's
// m_txBandAction (RadioModel::hasCommandPlane()), not a capability — see the
// header comment.
[[nodiscard]] inline QString txBandSettingsUnavailableReason()
{
    return QCoreApplication::translate("RadioSetupControlGate",
        "This radio has no radio-side per-band transmit settings for this client "
        "to set — nothing to set.");
}

}  // namespace AetherSDR
