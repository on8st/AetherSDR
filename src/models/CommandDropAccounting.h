#pragma once

// Which dropped Flex wire commands have a typed twin, and so may have reached
// the radio after all (#5263 follow-up).
//
// RadioModel's no-command-plane drop used to treat every dropped line as a dead
// control: qCWarning plus commandDropped, and MainWindow turned the first one
// of a session into "This radio doesn't support that control". But a converted
// setter emits BOTH its old wire text and a typed intent — setRfPower() emits
// `transmit set rfpower=` and rfPowerCommandIssued — and on a Hermes-Lite 2 the
// intent is what sets the drive. The first drop of a session was therefore very
// often a control that had just worked, and it used up the session's one notice.
//
// This header only CLASSIFIES. Whether the twin was actually applied is decided
// at run time, per call, by the intent receipt (IRadioBackend::intentDeclined)
// or by a host-side applier that says so (RadioModel::noteIntentApplied). A
// command that is not listed here, or that carries any key not listed here, has
// no twin and stays loud.

#include <QString>
#include <QStringList>

#include <optional>

namespace AetherSDR {

enum class ControlIntent : quint8 {
    TxPower,
    TunePower,
    MicGain,
    TxFilter,
    SpeechProcessor,
    Vox,
    TxMonitor,
    CwPitch,
    CwSpeed,
    CwBreakIn,
    CwBreakInDelay,
    TxEq,
    RxEq,
};

constexpr quint64 controlIntentBit(ControlIntent intent)
{
    return quint64(1) << static_cast<quint64>(intent);
}

// State the host reads at the moment it is USED rather than when it is set,
// so there is no per-call receipt to wait for: TUNE power is passed to
// IRadioBackend::setTune() at key-down (RadioModel::dispatchTuneIntent), CW
// break-in and its delay to setCwKeying() at every key edge, and CW speed is
// read by the host iambic keyer and AudioEngine::setCwWpm(). On a backend that
// modulates on this host (RadioCapabilities::hostModulates) the model holding
// the value IS the delivery. Deliberately short: each entry names its reader.
inline bool controlIntentHeldForHostUse(ControlIntent intent)
{
    switch (intent) {
    case ControlIntent::TunePower:
    case ControlIntent::CwSpeed:
    case ControlIntent::CwBreakIn:
    case ControlIntent::CwBreakInDelay:
        return true;
    default:
        return false;
    }
}

namespace detail {
inline std::optional<ControlIntent> transmitKeyIntent(const QString& key)
{
    if (key == QLatin1String("rfpower")) return ControlIntent::TxPower;
    if (key == QLatin1String("tunepower")) return ControlIntent::TunePower;
    if (key == QLatin1String("miclevel")) return ControlIntent::MicGain;
    if (key == QLatin1String("filter_low") || key == QLatin1String("filter_high"))
        return ControlIntent::TxFilter;
    if (key == QLatin1String("speech_processor_enable")
        || key == QLatin1String("speech_processor_level"))
        return ControlIntent::SpeechProcessor;
    if (key == QLatin1String("vox_enable") || key == QLatin1String("vox_level")
        || key == QLatin1String("vox_delay"))
        return ControlIntent::Vox;
    if (key == QLatin1String("mon") || key == QLatin1String("mon_gain_sb"))
        return ControlIntent::TxMonitor;
    return std::nullopt;
}
} // namespace detail

// The twin of a dropped command, or nullopt when it has none. A multi-key
// `transmit set` line qualifies only when EVERY key maps to the SAME intent
// (filter_low + filter_high): one untwinned key makes the whole line loud,
// because staying quiet about it would hide that key's drop.
inline std::optional<ControlIntent> controlIntentForCommand(const QString& command)
{
    const QStringList words = command.simplified().split(QLatin1Char(' '));
    if (words.size() >= 3 && words[0] == QLatin1String("transmit")
        && words[1] == QLatin1String("set")) {
        std::optional<ControlIntent> intent;
        for (int i = 2; i < words.size(); ++i) {
            const int eq = words[i].indexOf(QLatin1Char('='));
            if (eq <= 0)
                return std::nullopt;
            const auto keyIntent = detail::transmitKeyIntent(words[i].left(eq));
            if (!keyIntent || (intent && *intent != *keyIntent))
                return std::nullopt;
            intent = keyIntent;
        }
        return intent;
    }
    if (words.size() >= 3 && words[0] == QLatin1String("eq")) {
        if (words[1] == QLatin1String("TXsc")) return ControlIntent::TxEq;
        if (words[1] == QLatin1String("RXsc")) return ControlIntent::RxEq;
    }
    if (words.size() == 3 && words[0] == QLatin1String("cw")) {
        if (words[1] == QLatin1String("pitch")) return ControlIntent::CwPitch;
        if (words[1] == QLatin1String("wpm")) return ControlIntent::CwSpeed;
        if (words[1] == QLatin1String("break_in")) return ControlIntent::CwBreakIn;
        if (words[1] == QLatin1String("break_in_delay")) return ControlIntent::CwBreakInDelay;
    }
    return std::nullopt;
}

} // namespace AetherSDR
