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
// This header CLASSIFIES and NAMES; it decides nothing. Whether the twin was actually applied is decided
// at run time, per call, by the intent receipt (IRadioBackend::intentDeclined)
// or by a host-side applier that says so (RadioModel::noteIntentApplied). A
// command that is not listed here, or that carries any key not listed here, has
// no twin and stays loud.

#include <QCoreApplication>
#include <QHash>
#include <QSet>
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
    // Slice-level intents (the SliceModel *CommandIssued family).
    SliceFrequency,
    SliceFilter,
    SliceAgc,
    SliceNoiseReduction,
    SliceNoiseBlanker,
    SliceAutoNotch,
    SliceSquelch,
    SliceRit,
    SliceXit,
    SliceRxAntenna,
    SliceLock,
    SliceTxSlice,
    SliceActive,
    SliceAudioMute,
    SliceAudioGain,
    SliceAudioPan,
    SliceFmToneMode,
    SliceFmToneValue,
    SliceRepeaterOffsetDir,
    SliceRepeaterOffset,
};
static_assert(static_cast<int>(ControlIntent::SliceRepeaterOffset) < 64,
              "controlIntentBit() packs intents into a quint64");

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

inline std::optional<ControlIntent> sliceKeyIntent(const QString& key)
{
    static const QHash<QString, ControlIntent> intents = {
        {QStringLiteral("agc_mode"), ControlIntent::SliceAgc},
        {QStringLiteral("agc_threshold"), ControlIntent::SliceAgc},
        {QStringLiteral("nr"), ControlIntent::SliceNoiseReduction},
        {QStringLiteral("nr_level"), ControlIntent::SliceNoiseReduction},
        {QStringLiteral("nb"), ControlIntent::SliceNoiseBlanker},
        {QStringLiteral("nb_level"), ControlIntent::SliceNoiseBlanker},
        {QStringLiteral("anf"), ControlIntent::SliceAutoNotch},
        {QStringLiteral("squelch"), ControlIntent::SliceSquelch},
        {QStringLiteral("squelch_level"), ControlIntent::SliceSquelch},
        {QStringLiteral("rit_on"), ControlIntent::SliceRit},
        {QStringLiteral("rit_freq"), ControlIntent::SliceRit},
        {QStringLiteral("xit_on"), ControlIntent::SliceXit},
        {QStringLiteral("xit_freq"), ControlIntent::SliceXit},
        {QStringLiteral("rxant"), ControlIntent::SliceRxAntenna},
        {QStringLiteral("tx"), ControlIntent::SliceTxSlice},
        {QStringLiteral("active"), ControlIntent::SliceActive},
        {QStringLiteral("audio_mute"), ControlIntent::SliceAudioMute},
        {QStringLiteral("audio_level"), ControlIntent::SliceAudioGain},
        {QStringLiteral("audio_pan"), ControlIntent::SliceAudioPan},
        {QStringLiteral("fm_tone_mode"), ControlIntent::SliceFmToneMode},
        {QStringLiteral("fm_tone_value"), ControlIntent::SliceFmToneValue},
        {QStringLiteral("repeater_offset_dir"), ControlIntent::SliceRepeaterOffsetDir},
        {QStringLiteral("fm_repeater_offset_freq"), ControlIntent::SliceRepeaterOffset},
    };
    const auto it = intents.constFind(key);
    if (it == intents.constEnd())
        return std::nullopt;
    return *it;
}

// Every key=value word from `from` on must map to ONE intent, or nullopt.
template <typename KeyIntent>
std::optional<ControlIntent> singleKeyIntent(const QStringList& words, int from,
                                             KeyIntent keyIntent)
{
    std::optional<ControlIntent> intent;
    for (int i = from; i < words.size(); ++i) {
        const int eq = words[i].indexOf(QLatin1Char('='));
        if (eq <= 0)
            return std::nullopt;
        const auto one = keyIntent(words[i].left(eq));
        if (!one || (intent && *intent != *one))
            return std::nullopt;
        intent = one;
    }
    return intent;
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
    // Slice verbs. `slice tune <id> <MHz> [autopan=0]` and `filt <id> <lo> <hi>`
    // are positional; `slice set <id> k=v …` follows the transmit rule above.
    if (words.size() >= 4 && words[0] == QLatin1String("slice")
        && words[1] == QLatin1String("tune"))
        return ControlIntent::SliceFrequency;
    if (words.size() == 4 && words[0] == QLatin1String("filt"))
        return ControlIntent::SliceFilter;
    if (words.size() == 3 && words[0] == QLatin1String("slice")
        && (words[1] == QLatin1String("lock") || words[1] == QLatin1String("unlock")))
        return ControlIntent::SliceLock;
    if (words.size() >= 4 && words[0] == QLatin1String("slice")
        && words[1] == QLatin1String("set"))
        return detail::singleKeyIntent(words, 3, detail::sliceKeyIntent);
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

// ── Naming a dropped control, and announcing each one once ──────────────────
//
// The operator notice used to be one generic sentence per session. With one
// sentence, the first dropped control silenced every later one, and the
// sentence could not say WHICH control it meant — an operator with ten
// controls in view had to guess. The notice now names the control and fires
// once per distinct control per connect session.
//
// The name is derived from the wire text, which is the one thing every drop
// has. Known verbs and keys get the label the operator sees; anything else
// falls back to the verb itself in quotes, so a control nobody has named yet
// is still announced rather than folded into a generic line.

namespace detail {
inline QString sliceKeyLabel(const QString& key)
{
    static const QHash<QString, QString> labels = {
        {QStringLiteral("apf"), QStringLiteral("APF")},
        {QStringLiteral("apf_level"), QStringLiteral("APF level")},
        {QStringLiteral("nb"), QStringLiteral("NB")},
        {QStringLiteral("nb_level"), QStringLiteral("NB level")},
        {QStringLiteral("nr"), QStringLiteral("NR")},
        {QStringLiteral("nr_level"), QStringLiteral("NR level")},
        {QStringLiteral("anf"), QStringLiteral("ANF")},
        {QStringLiteral("anf_level"), QStringLiteral("ANF level")},
        {QStringLiteral("lms_nr"), QStringLiteral("NRL")},
        {QStringLiteral("lms_nr_level"), QStringLiteral("NRL level")},
        {QStringLiteral("speex_nr"), QStringLiteral("NRS")},
        {QStringLiteral("speex_nr_level"), QStringLiteral("NRS level")},
        {QStringLiteral("rnnoise"), QStringLiteral("RNN")},
        {QStringLiteral("nrf"), QStringLiteral("NRF")},
        {QStringLiteral("nrf_level"), QStringLiteral("NRF level")},
        {QStringLiteral("lms_anf"), QStringLiteral("ANFL")},
        {QStringLiteral("lms_anf_level"), QStringLiteral("ANFL level")},
        {QStringLiteral("anft"), QStringLiteral("ANFT")},
        {QStringLiteral("agc_mode"), QStringLiteral("AGC mode")},
        {QStringLiteral("agc_threshold"), QStringLiteral("AGC threshold")},
        {QStringLiteral("agc_off_level"), QStringLiteral("AGC off level")},
        {QStringLiteral("squelch"), QStringLiteral("Squelch")},
        {QStringLiteral("squelch_level"), QStringLiteral("Squelch level")},
        {QStringLiteral("rit_on"), QStringLiteral("RIT")},
        {QStringLiteral("rit_freq"), QStringLiteral("RIT")},
        {QStringLiteral("xit_on"), QStringLiteral("XIT")},
        {QStringLiteral("xit_freq"), QStringLiteral("XIT")},
        {QStringLiteral("dax"), QStringLiteral("DAX channel")},
        {QStringLiteral("rtty_mark"), QStringLiteral("RTTY mark")},
        {QStringLiteral("rtty_shift"), QStringLiteral("RTTY shift")},
        {QStringLiteral("digl_offset"), QStringLiteral("DIGL offset")},
        {QStringLiteral("digu_offset"), QStringLiteral("DIGU offset")},
        {QStringLiteral("tx"), QStringLiteral("TX slice")},
        {QStringLiteral("active"), QStringLiteral("Active slice")},
        {QStringLiteral("record"), QStringLiteral("Record")},
        {QStringLiteral("play"), QStringLiteral("Play")},
        {QStringLiteral("fm_tone_mode"), QStringLiteral("FM tone mode")},
        {QStringLiteral("fm_tone_value"), QStringLiteral("FM tone")},
        {QStringLiteral("repeater_offset_dir"), QStringLiteral("Repeater offset direction")},
        {QStringLiteral("fm_repeater_offset_freq"), QStringLiteral("Repeater offset")},
        {QStringLiteral("tx_offset_freq"), QStringLiteral("TX offset")},
        {QStringLiteral("fm_deviation"), QStringLiteral("FM deviation")},
        {QStringLiteral("audio_level"), QStringLiteral("AF gain")},
        {QStringLiteral("audio_mute"), QStringLiteral("Mute")},
        {QStringLiteral("audio_pan"), QStringLiteral("Audio pan")},
        {QStringLiteral("rfgain"), QStringLiteral("RF gain")},
        {QStringLiteral("diversity"), QStringLiteral("Diversity")},
        {QStringLiteral("esc"), QStringLiteral("ESC")},
        {QStringLiteral("esc_gain"), QStringLiteral("ESC gain")},
        {QStringLiteral("esc_phase_shift"), QStringLiteral("ESC phase")},
        {QStringLiteral("rxant"), QStringLiteral("RX antenna")},
        {QStringLiteral("txant"), QStringLiteral("TX antenna")},
        {QStringLiteral("mode"), QStringLiteral("Mode")},
    };
    return labels.value(key);
}

inline QString transmitKeyLabel(const QString& key)
{
    static const QHash<QString, QString> labels = {
        {QStringLiteral("rfpower"), QStringLiteral("RF power")},
        {QStringLiteral("tunepower"), QStringLiteral("Tune power")},
        {QStringLiteral("tune_mode"), QStringLiteral("Tune mode")},
        {QStringLiteral("miclevel"), QStringLiteral("Mic gain")},
        {QStringLiteral("speech_processor_enable"), QStringLiteral("PROC")},
        {QStringLiteral("speech_processor_level"), QStringLiteral("PROC level")},
        {QStringLiteral("dax"), QStringLiteral("TX DAX")},
        {QStringLiteral("mon"), QStringLiteral("MON")},
        {QStringLiteral("mon_gain_sb"), QStringLiteral("MON level")},
        {QStringLiteral("mon_gain_cw"), QStringLiteral("CW sidetone level")},
        {QStringLiteral("mon_pan_cw"), QStringLiteral("CW sidetone pan")},
        {QStringLiteral("vox_enable"), QStringLiteral("VOX")},
        {QStringLiteral("vox_level"), QStringLiteral("VOX level")},
        {QStringLiteral("vox_delay"), QStringLiteral("VOX delay")},
        {QStringLiteral("am_carrier"), QStringLiteral("AM carrier")},
        {QStringLiteral("compander"), QStringLiteral("DEXP")},
        {QStringLiteral("compander_level"), QStringLiteral("DEXP level")},
        {QStringLiteral("filter_low"), QStringLiteral("TX filter")},
        {QStringLiteral("filter_high"), QStringLiteral("TX filter")},
    };
    return labels.value(key);
}

inline QString verbLabel(const QStringList& words)
{
    static const QHash<QString, QString> labels = {
        {QStringLiteral("cw pitch"), QStringLiteral("CW pitch")},
        {QStringLiteral("cw wpm"), QStringLiteral("CW speed")},
        {QStringLiteral("cw break_in"), QStringLiteral("Break-in")},
        {QStringLiteral("cw break_in_delay"), QStringLiteral("Break-in delay")},
        {QStringLiteral("cw sidetone"), QStringLiteral("Sidetone")},
        {QStringLiteral("cw iambic"), QStringLiteral("Iambic")},
        {QStringLiteral("cw mode"), QStringLiteral("Iambic mode")},
        {QStringLiteral("cw swap"), QStringLiteral("Paddle swap")},
        {QStringLiteral("cw cwl_enabled"), QStringLiteral("CWL")},
        {QStringLiteral("mic boost"), QStringLiteral("Mic boost")},
        {QStringLiteral("mic bias"), QStringLiteral("Mic bias")},
        {QStringLiteral("mic acc"), QStringLiteral("ACC mic")},
        {QStringLiteral("mic input"), QStringLiteral("Mic input")},
        {QStringLiteral("slice tune"), QStringLiteral("Tuning")},
        {QStringLiteral("slice lock"), QStringLiteral("Lock")},
        {QStringLiteral("slice unlock"), QStringLiteral("Lock")},
        {QStringLiteral("atu start"), QStringLiteral("ATU")},
        {QStringLiteral("atu bypass"), QStringLiteral("ATU")},
        {QStringLiteral("atu set"), QStringLiteral("ATU memories")},
        {QStringLiteral("atu clear"), QStringLiteral("ATU memories")},
        {QStringLiteral("apd enable"), QStringLiteral("APD")},
        {QStringLiteral("apd sampler"), QStringLiteral("APD sampler")},
        {QStringLiteral("apd reset"), QStringLiteral("APD reset")},
        {QStringLiteral("eq TXsc"), QStringLiteral("TX EQ")},
        {QStringLiteral("eq RXsc"), QStringLiteral("RX EQ")},
    };
    if (words.size() >= 2) {
        const QString two = words[0] + QLatin1Char(' ') + words[1];
        const QString hit = labels.value(two);
        if (!hit.isEmpty())
            return hit;
    }
    if (!words.isEmpty() && words[0] == QLatin1String("filt"))
        return QStringLiteral("Filter");
    if (words.size() >= 3 && words[0] == QLatin1String("profile")
        && words[2] == QLatin1String("load"))
        return words[1] == QLatin1String("mic") ? QStringLiteral("Mic profile")
             : words[1] == QLatin1String("tx")  ? QStringLiteral("TX profile")
                                                : QStringLiteral("Profile");
    return {};
}

// A token that identifies an object or carries a value, never part of a name.
inline bool isOperandToken(const QString& word)
{
    if (word.contains(QLatin1Char('=')) || word.startsWith(QLatin1Char('"')))
        return true;
    bool numeric = false;
    word.toDouble(&numeric);
    if (numeric)
        return true;
    return word.startsWith(QLatin1String("0x"), Qt::CaseInsensitive);
}
} // namespace detail

// The operator-facing name of the control a dropped command belongs to. Never
// empty: an unknown verb falls back to its own words in quotes.
inline QString controlNameForCommand(const QString& command)
{
    const QStringList words = command.simplified().split(QLatin1Char(' '), Qt::SkipEmptyParts);
    if (words.isEmpty())
        return QStringLiteral("“”");

    // `slice set <id> key=value …` and `transmit set key=value …`: the FIRST
    // key names the control. A multi-key line (rit_on + rit_freq, filter_low
    // + filter_high) is one control whose keys share a label.
    auto firstKey = [&words](int from) -> QString {
        for (int i = from; i < words.size(); ++i) {
            const int eq = words[i].indexOf(QLatin1Char('='));
            if (eq > 0)
                return words[i].left(eq);
        }
        return {};
    };
    if (words.size() >= 3 && words[0] == QLatin1String("slice")
        && words[1] == QLatin1String("set")) {
        const QString key = firstKey(2);
        const QString label = detail::sliceKeyLabel(key);
        if (!label.isEmpty())
            return label;
        if (!key.isEmpty())
            return QStringLiteral("“%1”").arg(key);
    }
    if (words.size() >= 3 && words[0] == QLatin1String("transmit")
        && words[1] == QLatin1String("set")) {
        const QString key = firstKey(2);
        const QString label = detail::transmitKeyLabel(key);
        if (!label.isEmpty())
            return label;
        if (!key.isEmpty())
            return QStringLiteral("“%1”").arg(key);
    }
    const QString verb = detail::verbLabel(words);
    if (!verb.isEmpty())
        return verb;

    // Fallback: the verb's own words up to the first operand, at most three.
    QStringList name;
    for (const QString& word : words) {
        if (detail::isOperandToken(word) || name.size() == 3)
            break;
        name << word;
    }
    if (name.isEmpty())
        name << words.first();
    return QStringLiteral("“%1”").arg(name.join(QLatin1Char(' ')));
}

// Per-connect-session memory of which controls have been announced. Pure
// logic so it can be pinned headless; MainWindow owns one and resets it on the
// connect edge.
class DroppedControlAnnouncements
{
public:
    // True the first time this control is seen in the session.
    bool firstThisSession(const QString& controlName)
    {
        if (m_announced.contains(controlName))
            return false;
        m_announced.insert(controlName);
        return true;
    }
    void reset() { m_announced.clear(); }
    int size() const { return int(m_announced.size()); }

private:
    QSet<QString> m_announced;
};

// One status-bar line for the controls first seen in one event-loop turn. A
// profile load or a band change can drop several controls at once; one merged
// line is readable where a burst of replacements is not.
inline QString droppedControlNotice(const QStringList& names)
{
    const char* ctx = "DroppedControlNotice";
    if (names.isEmpty())
        return {};
    if (names.size() == 1)
        return QCoreApplication::translate(
                   ctx, "%1 isn't available on this radio — nothing was sent.")
            .arg(names[0]);
    if (names.size() == 2)
        return QCoreApplication::translate(
                   ctx, "%1 and %2 aren't available on this radio — nothing was sent.")
            .arg(names[0], names[1]);
    if (names.size() == 3)
        return QCoreApplication::translate(
                   ctx, "%1, %2 and %3 aren't available on this radio — nothing was sent.")
            .arg(names[0], names[1], names[2]);
    return QCoreApplication::translate(
               ctx, "%1, %2 and %3 more controls aren't available on this radio "
                    "— nothing was sent.")
        .arg(names[0], names[1]).arg(names.size() - 2);
}

} // namespace AetherSDR
