// An operator-intent verb a backend does not implement must refuse VISIBLY
// (HERMES §17's dead-control shape; #5263 M0 item 1 carried one layer down,
// from RadioModel::sendCmd to the IRadioBackend defaults).
//
// Socket-free: fake backends override only the pure virtuals (or, for the
// negative case, every loud verb), and the test calls the verbs directly the
// way RadioModel's handlers do. What is pinned:
//   1. every loud verb on a backend that overrides nothing emits exactly ONE
//      intentUnsupported per session however often it is called (a drag);
//   2. the latch resets on connected(), and nothing is emitted disconnected;
//   3. a backend that owns a command plane (Flex, Sim) emits nothing;
//   4. a backend that overrides the verbs emits nothing;
//   5. the verbs classified as deliberately silent stay silent;
//   6. RadioModel relays the refusal as backendIntentUnsupported.

#include "TestSettingsProfile.h"
#include "core/backends/IRadioBackend.h"
#include "models/RadioModel.h"
#include "models/TransmitModel.h"

#include <QCoreApplication>
#include <QStringList>

#include <cstdio>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

using namespace AetherSDR;

namespace {
int failures = 0;
void check(bool ok, const QString& message)
{
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", qPrintable(message));
    failures += !ok;
}

// Overrides the pure virtuals and nothing else.
class BareBackend : public IRadioBackend {
public:
    bool live{true};
    bool commandPlane{false};
    QStringList refusals;
    QStringList messages;

    BareBackend()
    {
        connect(this, &IRadioBackend::intentUnsupported, this,
                [this](const QString& intent, const QString& message) {
            refusals << intent;
            messages << message;
        });
    }
    RadioCapabilities capabilities() const override { return {}; }
    bool ownsCommandPlane() const override { return commandPlane; }
    bool isConnected() const override { return live; }
    void connectRadio(const RadioConnectRequest&) override { live = true; }
    void disconnectRadio() override { live = false; }
    void setSliceFrequency(int, double) override {}
    void setSliceMode(int, const QString&) override {}
    void setSliceFilter(int, int, int) override {}
    void setSliceAgc(int, const QString&, int) override {}
    void setPanCenter(const QString&, double, PanCenterIntent) override {}
    void setKeying(bool, const TxCoordinator::Operation&, const TxCoordinator::Completion&) override {}
    void invokeExtension(const QString&, const QString&, quint64, const QVariant&) override {}
    void reannounce() { emit connected(); }
};

// Implements every loud verb (as a no-op standing in for a real encode).
class ImplementingBackend final : public BareBackend {
public:
    void setSliceFilterPreset(int, int) override {}
    void setPanBandwidth(const QString&, double) override {}
    void setPanRfGain(const QString&, int) override {}
    void setPanPreamp(const QString&, int) override {}
    void setPanAttenuator(const QString&, int) override {}
    void setSliceRxAntenna(int, const QString&) override {}
    void setRadioDialLock(bool) override {}
    void createNotch(double, double) override {}
    void setNotch(int, const NotchDelta&) override {}
    void removeNotch(int) override {}
    void setNotchesEnabled(bool) override {}
    void setTxMonitor(bool, int) override {}
    void setTxPower(int) override {}
    void setCwPitch(int) override {}
    void setCwSpeed(int) override {}
    void setCwBreakIn(bool) override {}
    void setVox(bool, int, int) override {}
    void setSliceNoiseReduction(int, bool, int) override {}
    void setSliceNoiseBlanker(int, bool, int) override {}
    void setSliceAutoNotch(int, bool) override {}
    void setSliceManualNotch(int, bool, int) override {}
    void setSliceSquelch(int, bool, int) override {}
    void setSliceFmToneMode(int, const QString&) override {}
    void setSliceFmToneValue(int, double) override {}
    void setSliceFmToneRxValue(int, double) override {}
    void setSliceFmDtcs(int, int, bool, bool) override {}
    void setSliceRepeaterOffsetDir(int, const QString&) override {}
    void setSliceFmRepeaterOffset(int, double) override {}
    void setRitEnabled(bool) override {}
    void setXitEnabled(bool) override {}
    void setRitOffset(int) override {}
    void setTxFilter(int, int) override {}
    void setMicGain(int) override {}
};

struct LoudVerb {
    const char* name;
    const char* intent;
    std::function<void(IRadioBackend&)> call;
};

std::vector<LoudVerb> loudVerbs()
{
    const QString pan = QStringLiteral("0x40000000");
    return {
        {"setSliceFilterPreset", "filter-preset", [](IRadioBackend& b) { b.setSliceFilterPreset(0, 1); }},
        {"setPanBandwidth", "pan-bandwidth", [pan](IRadioBackend& b) { b.setPanBandwidth(pan, 48000.0); }},
        {"setPanRfGain", "rf-gain", [pan](IRadioBackend& b) { b.setPanRfGain(pan, 10); }},
        {"setPanPreamp", "preamp", [pan](IRadioBackend& b) { b.setPanPreamp(pan, 1); }},
        {"setPanAttenuator", "attenuator", [pan](IRadioBackend& b) { b.setPanAttenuator(pan, 1); }},
        {"setSliceRxAntenna", "rx-antenna", [](IRadioBackend& b) { b.setSliceRxAntenna(0, QStringLiteral("ANT2")); }},
        {"setRadioDialLock", "dial-lock", [](IRadioBackend& b) { b.setRadioDialLock(true); }},
        {"createNotch", "notch", [](IRadioBackend& b) { b.createNotch(7.1e6, 100.0); }},
        {"setNotch", "notch", [](IRadioBackend& b) { b.setNotch(1, NotchDelta{}); }},
        {"removeNotch", "notch", [](IRadioBackend& b) { b.removeNotch(1); }},
        {"setNotchesEnabled", "notch", [](IRadioBackend& b) { b.setNotchesEnabled(true); }},
        {"setTxMonitor", "tx-monitor", [](IRadioBackend& b) { b.setTxMonitor(true, 50); }},
        {"setTxPower", "tx-power", [](IRadioBackend& b) { b.setTxPower(40); }},
        {"setCwPitch", "cw-pitch", [](IRadioBackend& b) { b.setCwPitch(600); }},
        {"setCwSpeed", "cw-speed", [](IRadioBackend& b) { b.setCwSpeed(20); }},
        {"setCwBreakIn", "cw-break-in", [](IRadioBackend& b) { b.setCwBreakIn(true); }},
        {"setVox", "vox", [](IRadioBackend& b) { b.setVox(true, 50, 300); }},
        {"setSliceNoiseReduction", "noise-reduction", [](IRadioBackend& b) { b.setSliceNoiseReduction(0, true, 50); }},
        {"setSliceNoiseBlanker", "noise-blanker", [](IRadioBackend& b) { b.setSliceNoiseBlanker(0, true, 50); }},
        {"setSliceAutoNotch", "auto-notch", [](IRadioBackend& b) { b.setSliceAutoNotch(0, true); }},
        {"setSliceManualNotch", "manual-notch", [](IRadioBackend& b) { b.setSliceManualNotch(0, true, 50); }},
        {"setSliceSquelch", "squelch", [](IRadioBackend& b) { b.setSliceSquelch(0, true, 20); }},
        {"setSliceFmToneMode", "fm-repeater", [](IRadioBackend& b) { b.setSliceFmToneMode(0, QStringLiteral("ctcss_tx")); }},
        {"setSliceFmToneValue", "fm-repeater", [](IRadioBackend& b) { b.setSliceFmToneValue(0, 88.5); }},
        {"setSliceFmToneRxValue", "fm-repeater", [](IRadioBackend& b) { b.setSliceFmToneRxValue(0, 88.5); }},
        {"setSliceFmDtcs", "fm-repeater", [](IRadioBackend& b) { b.setSliceFmDtcs(0, 23, false, false); }},
        {"setSliceRepeaterOffsetDir", "fm-repeater", [](IRadioBackend& b) { b.setSliceRepeaterOffsetDir(0, QStringLiteral("up")); }},
        {"setSliceFmRepeaterOffset", "fm-repeater", [](IRadioBackend& b) { b.setSliceFmRepeaterOffset(0, 600000.0); }},
        // The composites: defaults that reach the FM leaves above.
        {"setSliceFmRepeater", "fm-repeater", [](IRadioBackend& b) {
            b.setSliceFmRepeater(0, QStringLiteral("up"), 600000.0, QStringLiteral("ctcss_tx"), 88.5); }},
        {"applyMemoryRecallDetails", "fm-repeater", [](IRadioBackend& b) {
            MemoryRecallDetails d; d.direction = QStringLiteral("up"); d.offsetHz = 600000.0;
            d.toneMode = QStringLiteral("ctcss_tx"); d.txToneHz = 88.5;
            b.applyMemoryRecallDetails(d); }},
        {"setRitEnabled", "rit-xit", [](IRadioBackend& b) { b.setRitEnabled(true); }},
        {"setXitEnabled", "rit-xit", [](IRadioBackend& b) { b.setXitEnabled(true); }},
        {"setRitOffset", "rit-xit", [](IRadioBackend& b) { b.setRitOffset(100); }},
        {"setXitOffset", "rit-xit", [](IRadioBackend& b) { b.setXitOffset(100); }},
        {"setTxFilter", "tx-filter", [](IRadioBackend& b) { b.setTxFilter(100, 2900); }},
        {"setMicGain", "mic-gain", [](IRadioBackend& b) { b.setMicGain(50); }},
    };
}

void eachLoudVerbRefusesExactlyOnce()
{
    for (const LoudVerb& v : loudVerbs()) {
        BareBackend b;
        for (int i = 0; i < 5; ++i) {   // a drag, not a click
            v.call(b);
        }
        check(b.refusals == QStringList{QString::fromLatin1(v.intent)},
              QStringLiteral("%1: exactly one '%2' refusal for five calls (got [%3])")
                  .arg(QLatin1String(v.name), QLatin1String(v.intent), b.refusals.join(',')));
        check(!b.messages.isEmpty()
                  && b.messages.first().contains(QStringLiteral("isn't available on this radio")),
              QStringLiteral("%1: the notice is an operator sentence").arg(QLatin1String(v.name)));
    }
}

void latchIsPerIntentPerSession()
{
    BareBackend b;
    b.setVox(true, 50, 300);
    b.setVox(true, 60, 300);
    b.setTxMonitor(true, 40);
    check(b.refusals == QStringList({QStringLiteral("vox"), QStringLiteral("tx-monitor")}),
          QStringLiteral("distinct intents each refuse once, repeats do not"));

    // RadioModel's RIT press calls two verbs; the operator sees one notice.
    BareBackend rit;
    rit.setRitEnabled(true);
    rit.setRitOffset(200);
    rit.setXitEnabled(true);
    rit.setXitOffset(200);
    check(rit.refusals.size() == 1, QStringLiteral("RIT/XIT cluster is one notice"));

    b.reannounce();
    b.setVox(false, 50, 300);
    check(b.refusals.count(QStringLiteral("vox")) == 2,
          QStringLiteral("a new session re-arms the notice"));

    BareBackend offline;
    offline.live = false;
    offline.setVox(true, 50, 300);
    offline.setSliceSquelch(0, true, 10);
    check(offline.refusals.isEmpty(), QStringLiteral("nothing is refused while disconnected"));
    offline.live = true;
    offline.setVox(true, 50, 300);
    check(offline.refusals == QStringList{QStringLiteral("vox")},
          QStringLiteral("a disconnected call does not consume the session's notice"));
}

void commandPlaneBackendIsSilent()
{
    BareBackend b;
    b.commandPlane = true;
    for (const LoudVerb& v : loudVerbs()) {
        v.call(b);
    }
    check(b.refusals.isEmpty(),
          QStringLiteral("a backend that owns a command plane refuses nothing (got [%1])")
              .arg(b.refusals.join(',')));
}

void implementingBackendIsSilent()
{
    ImplementingBackend b;
    for (const LoudVerb& v : loudVerbs()) {
        v.call(b);
    }
    check(b.refusals.isEmpty(),
          QStringLiteral("a backend that overrides the verbs refuses nothing (got [%1])")
              .arg(b.refusals.join(',')));
}

// The deliberately silent defaults. Pinned so a later edit that makes one loud
// is a visible decision, not a drive-by.
void silentDefaultsStaySilent()
{
    BareBackend b;
    const QString pan = QStringLiteral("0x40000000");
    b.applyRestoredState(RestoredRadioState{});
    b.setPanFrameRate(pan, 25);
    b.setPanAverage(pan, 50);
    b.setPanWeightedAverage(pan, true);
    b.setPanPixelWidth(pan, 1024);
    b.setSliceAudioMute(0, true);
    b.setSliceAudioGain(0, 50);
    b.setSliceAudioPan(0, 50);
    b.setTxSlice(0);
    b.setActiveSlice(0);
    b.createSlice(pan, 7.1e6);
    b.removeSlice(0);
    b.createPanadapter();
    b.removePanadapter(pan);
    b.setTxAudioMonitor(true);
    b.setSpeechProcessor(true, 5);
    b.refreshMemories(QString());
    b.setTransmitFrequencyCheck(false);
    b.submitTxAudio(QByteArray(), 48000, TxAudioSource{}, TxCoordinator::Context{});
    b.finishTxAudio(TxCoordinator::Context{});
    check(b.refusals.isEmpty(),
          QStringLiteral("silent-by-design defaults emit nothing (got [%1])").arg(b.refusals.join(',')));
}

void radioModelRelaysTheRefusal()
{
    RadioModel radio;
    auto owned = std::make_unique<BareBackend>();
    radio.setBackendForTest(std::move(owned), QStringLiteral("hl2"));
    QStringList relayed;
    QObject::connect(&radio, &RadioModel::backendIntentUnsupported, &radio,
                     [&relayed](const QString& intent, const QString&) { relayed << intent; });
    TransmitModel& tx = radio.transmitModel();
    tx.setVoxEnable(true);
    tx.setVoxLevel(30);
    tx.setVoxLevel(31);
    tx.setVoxLevel(32);
    check(relayed == QStringList{QStringLiteral("vox")},
          QStringLiteral("RadioModel relays one VOX notice for a toggle and a drag (got [%1])")
              .arg(relayed.join(',')));
}
} // namespace

int main(int argc, char** argv)
{
    TestSettingsProfile profile(QStringLiteral("backend-intent-refusal"));
    if (!profile.isValid()) { return 1; }
    QCoreApplication app(argc, argv);
    eachLoudVerbRefusesExactlyOnce();
    latchIsPerIntentPerSession();
    commandPlaneBackendIsSilent();
    implementingBackendIsSilent();
    silentDefaultsStaySilent();
    radioModelRelaysTheRefusal();
    std::printf("%d failure(s)\n", failures);
    return failures == 0 ? 0 : 1;
}
