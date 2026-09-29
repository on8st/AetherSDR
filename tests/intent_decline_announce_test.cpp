// Every operator-intent verb a backend does not implement DECLINES, and a
// decline nothing else applied is announced by name through the one notice
// (#5263; HERMES §17's dead-control shape carried from sendCmd down to the
// IRadioBackend defaults). Socket-free: fake backends on RadioModel's real
// wiring, no transport, no radio.
//
// The receipt mechanism (command_drop_receipt_test) decides whether a DROPPED
// wire command's control was applied. Some intents have no wire twin at all —
// manual notch, DTCS, FM RX tone, preamp, attenuator, the notch filters, a
// filter preset — so nothing was dropped and an un-overridden verb was dead in
// silence. Here the decline itself is the evidence. What is pinned:
//   1. every verb classified loud declines on a backend that overrides nothing,
//      however often it is called;
//   2. a backend that implements the verbs declines none of them;
//   3. the verbs classified as deliberately silent do not decline;
//   4. a decline with no wire twin reaches controlUnavailable, named, once per
//      turn — and not at all while disconnected, nor when a command plane
//      carried the wire text (Flex, the demo);
//   5. a decline and its own dropped twin are ONE announcement, and a host
//      applier's receipt in the same turn silences the decline;
//   6. one operator control is one name: VOX's toggle and a level drag, and
//      RIT's two verbs per press, each give the ledger a single entry.
#include "TestSettingsProfile.h"
#include "core/backends/IRadioBackend.h"
#include "models/CommandDropAccounting.h"
#include "models/RadioModel.h"
#include "models/SliceModel.h"
#include "models/TnfModel.h"
#include "models/TransmitModel.h"

#include <QCoreApplication>
#include <QSignalSpy>
#include <QStringList>

#include <cstdio>
#include <functional>
#include <memory>
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
class BareBackend : public IRadioBackend
{
public:
    bool live{true};
    RadioCapabilities caps;
    RadioCapabilities capabilities() const override { return caps; }
    bool isConnected() const override { return live; }
    void connectRadio(const RadioConnectRequest&) override { live = true; }
    void disconnectRadio() override { live = false; }
    void setSliceFrequency(int, double) override {}
    void setSliceMode(int, const QString&) override {}
    void setSliceFilter(int, int, int) override {}
    void setSliceAgc(int, const QString&, int) override {}
    void setPanCenter(const QString&, double, PanCenterIntent) override {}
    void setKeying(bool, const TxCoordinator::Operation&,
                   const TxCoordinator::Completion&) override {}
    void invokeExtension(const QString&, const QString&, quint64, const QVariant&) override {}
};

// Implements every loud verb (as a no-op standing in for a real encode).
class ImplementingBackend final : public BareBackend
{
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
    void setSpeechProcessor(bool, int) override {}
    void setVox(bool, int, int) override {}
    void setSliceAudioMute(int, bool) override {}
    void setSliceAudioGain(int, int) override {}
    void setSliceAudioPan(int, int) override {}
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
    std::function<void(IRadioBackend&)> call;
};

std::vector<LoudVerb> loudVerbs()
{
    const QString pan = QStringLiteral("0x40000000");
    return {
        {"setSliceFilterPreset", [](IRadioBackend& b) { b.setSliceFilterPreset(0, 1); }},
        {"setPanBandwidth", [pan](IRadioBackend& b) { b.setPanBandwidth(pan, 48000.0); }},
        {"setPanRfGain", [pan](IRadioBackend& b) { b.setPanRfGain(pan, 10); }},
        {"setPanPreamp", [pan](IRadioBackend& b) { b.setPanPreamp(pan, 1); }},
        {"setPanAttenuator", [pan](IRadioBackend& b) { b.setPanAttenuator(pan, 1); }},
        {"setSliceRxAntenna", [](IRadioBackend& b) { b.setSliceRxAntenna(0, QStringLiteral("ANT2")); }},
        {"setRadioDialLock", [](IRadioBackend& b) { b.setRadioDialLock(true); }},
        {"createNotch", [](IRadioBackend& b) { b.createNotch(7.1e6, 100.0); }},
        {"setNotch", [](IRadioBackend& b) { b.setNotch(1, NotchDelta{}); }},
        {"removeNotch", [](IRadioBackend& b) { b.removeNotch(1); }},
        {"setNotchesEnabled", [](IRadioBackend& b) { b.setNotchesEnabled(true); }},
        {"setTxMonitor", [](IRadioBackend& b) { b.setTxMonitor(true, 50); }},
        {"setTxPower", [](IRadioBackend& b) { b.setTxPower(40); }},
        {"setCwPitch", [](IRadioBackend& b) { b.setCwPitch(600); }},
        {"setCwSpeed", [](IRadioBackend& b) { b.setCwSpeed(20); }},
        {"setCwBreakIn", [](IRadioBackend& b) { b.setCwBreakIn(true); }},
        {"setSpeechProcessor", [](IRadioBackend& b) { b.setSpeechProcessor(true, 1); }},
        {"setVox", [](IRadioBackend& b) { b.setVox(true, 50, 300); }},
        {"setSliceAudioMute", [](IRadioBackend& b) { b.setSliceAudioMute(0, true); }},
        {"setSliceAudioGain", [](IRadioBackend& b) { b.setSliceAudioGain(0, 50); }},
        {"setSliceAudioPan", [](IRadioBackend& b) { b.setSliceAudioPan(0, 50); }},
        {"setSliceNoiseReduction", [](IRadioBackend& b) { b.setSliceNoiseReduction(0, true, 50); }},
        {"setSliceNoiseBlanker", [](IRadioBackend& b) { b.setSliceNoiseBlanker(0, true, 50); }},
        {"setSliceAutoNotch", [](IRadioBackend& b) { b.setSliceAutoNotch(0, true); }},
        {"setSliceManualNotch", [](IRadioBackend& b) { b.setSliceManualNotch(0, true, 50); }},
        {"setSliceSquelch", [](IRadioBackend& b) { b.setSliceSquelch(0, true, 20); }},
        {"setSliceFmToneMode", [](IRadioBackend& b) { b.setSliceFmToneMode(0, QStringLiteral("ctcss_tx")); }},
        {"setSliceFmToneValue", [](IRadioBackend& b) { b.setSliceFmToneValue(0, 88.5); }},
        {"setSliceFmToneRxValue", [](IRadioBackend& b) { b.setSliceFmToneRxValue(0, 88.5); }},
        {"setSliceFmDtcs", [](IRadioBackend& b) { b.setSliceFmDtcs(0, 23, false, false); }},
        {"setSliceRepeaterOffsetDir", [](IRadioBackend& b) { b.setSliceRepeaterOffsetDir(0, QStringLiteral("up")); }},
        {"setSliceFmRepeaterOffset", [](IRadioBackend& b) { b.setSliceFmRepeaterOffset(0, 600000.0); }},
        // The composites: defaults that reach the FM leaves above.
        {"setSliceFmRepeater", [](IRadioBackend& b) {
            b.setSliceFmRepeater(0, QStringLiteral("up"), 600000.0, QStringLiteral("ctcss_tx"), 88.5); }},
        {"applyMemoryRecallDetails", [](IRadioBackend& b) {
            MemoryRecallDetails d; d.direction = QStringLiteral("up"); d.offsetHz = 600000.0;
            d.toneMode = QStringLiteral("ctcss_tx"); d.txToneHz = 88.5;
            b.applyMemoryRecallDetails(d); }},
        {"setRitEnabled", [](IRadioBackend& b) { b.setRitEnabled(true); }},
        {"setXitEnabled", [](IRadioBackend& b) { b.setXitEnabled(true); }},
        {"setRitOffset", [](IRadioBackend& b) { b.setRitOffset(100); }},
        {"setXitOffset", [](IRadioBackend& b) { b.setXitOffset(100); }},
        {"setTxFilter", [](IRadioBackend& b) { b.setTxFilter(100, 2900); }},
        {"setMicGain", [](IRadioBackend& b) { b.setMicGain(50); }},
    };
}

bool declines(IRadioBackend& b, const std::function<void(IRadioBackend&)>& call, int times = 1)
{
    b.beginIntentReceipt();
    for (int i = 0; i < times; ++i)
        call(b);
    return b.intentDeclined();
}

void eachLoudVerbDeclines()
{
    for (const LoudVerb& v : loudVerbs()) {
        BareBackend b;
        check(declines(b, v.call, 5),   // a drag, not a click
              QStringLiteral("%1: un-overridden default declines").arg(QLatin1String(v.name)));
    }
}

void implementingBackendDeclinesNothing()
{
    for (const LoudVerb& v : loudVerbs()) {
        ImplementingBackend b;
        check(!declines(b, v.call),
              QStringLiteral("%1: an override that applies does not decline")
                  .arg(QLatin1String(v.name)));
    }
}

// The deliberately silent defaults (see IRadioBackend::beginIntentReceipt()).
// Pinned so a later edit that makes one loud is a visible decision.
void silentDefaultsDoNotDecline()
{
    const QString pan = QStringLiteral("0x40000000");
    const std::vector<LoudVerb> silent = {
        {"applyRestoredState", [](IRadioBackend& b) { b.applyRestoredState(RestoredRadioState{}); }},
        {"setPanFrameRate", [pan](IRadioBackend& b) { b.setPanFrameRate(pan, 25); }},
        {"setPanAverage", [pan](IRadioBackend& b) { b.setPanAverage(pan, 50); }},
        {"setPanWeightedAverage", [pan](IRadioBackend& b) { b.setPanWeightedAverage(pan, true); }},
        {"setPanPixelWidth", [pan](IRadioBackend& b) { b.setPanPixelWidth(pan, 1024); }},
        {"setTxSlice", [](IRadioBackend& b) { b.setTxSlice(0); }},
        {"setActiveSlice", [](IRadioBackend& b) { b.setActiveSlice(0); }},
        {"createSlice", [pan](IRadioBackend& b) { b.createSlice(pan, 7.1e6); }},
        {"removeSlice", [](IRadioBackend& b) { b.removeSlice(0); }},
        {"createPanadapter", [](IRadioBackend& b) { b.createPanadapter(); }},
        {"removePanadapter", [pan](IRadioBackend& b) { b.removePanadapter(pan); }},
        {"setTxAudioMonitor", [](IRadioBackend& b) { b.setTxAudioMonitor(true); }},
        {"refreshMemories", [](IRadioBackend& b) { b.refreshMemories(QString()); }},
        {"setTransmitFrequencyCheck", [](IRadioBackend& b) { b.setTransmitFrequencyCheck(false); }},
        {"submitTxAudio", [](IRadioBackend& b) {
            b.submitTxAudio(QByteArray(), 48000, TxAudioSource{}, TxCoordinator::Context{}); }},
        {"finishTxAudio", [](IRadioBackend& b) { b.finishTxAudio(TxCoordinator::Context{}); }},
    };
    for (const LoudVerb& v : silent) {
        BareBackend b;
        check(!declines(b, v.call),
              QStringLiteral("%1: silent by classification, does not decline")
                  .arg(QLatin1String(v.name)));
    }
}

void endTurn()
{
    QCoreApplication::processEvents();
    QCoreApplication::processEvents();
}

QStringList names(const QSignalSpy& spy)
{
    QStringList out;
    for (const QList<QVariant>& args : spy)
        out << args.value(0).toString();
    return out;
}

template <typename Backend>
Backend* install(RadioModel& radio)
{
    auto owned = std::make_unique<Backend>();
    Backend* backend = owned.get();
    radio.setBackendForTest(std::move(owned), QStringLiteral("hl2"));
    return backend;
}

SliceModel* materializeSlice(RadioModel& radio, IRadioBackend& backend)
{
    emit backend.panCenterBandwidthChanged(QStringLiteral("hl2"), 145.5, 0.1);
    SliceDelta delta;
    delta.panId = QStringLiteral("hl2");
    delta.frequency = 145.5;
    delta.mode = QStringLiteral("FM");
    delta.filterLow = -6000;
    delta.filterHigh = 6000;
    delta.inUse = true;
    emit backend.sliceChanged(0, delta);
    return radio.slice(0);
}

void intentOnlyDeclinesAreAnnounced()
{
    RadioModel radio;
    BareBackend* backend = install<BareBackend>(radio);
    check(!radio.hasCommandPlane() && radio.isConnected(),
          QStringLiteral("fixture: connected, no command plane (the HL2 shape)"));
    SliceModel* slice = materializeSlice(radio, *backend);
    check(slice != nullptr, QStringLiteral("the non-Flex materializer built slice 0"));
    if (!slice)
        return;

    QSignalSpy unavailable(&radio, &RadioModel::controlUnavailable);
    QSignalSpy dropped(&radio, &RadioModel::commandDropped);

    radio.setPanPreampFor(QStringLiteral("hl2"), 1);
    endTurn();
    check(names(unavailable) == QStringList{QStringLiteral("Preamp")},
          QStringLiteral("preamp (no wire twin) declined -> announced by name (got [%1])")
              .arg(names(unavailable).join(',')));
    check(dropped.isEmpty(), QStringLiteral("and no wire command was invented for it"));

    unavailable.clear();
    slice->setFmDtcs(23, false, false);
    endTurn();
    check(names(unavailable) == QStringList{QStringLiteral("DTCS")},
          QStringLiteral("DTCS (no wire twin) declined -> announced"));

    unavailable.clear();
    slice->setMn(true);
    endTurn();
    check(names(unavailable) == QStringList{QStringLiteral("Manual notch")},
          QStringLiteral("manual notch (no wire twin) declined -> announced"));

    unavailable.clear();
    radio.tnfModel().requestGlobalTnfEnabled(true);
    endTurn();
    check(names(unavailable) == QStringList{QStringLiteral("Notch filters")},
          QStringLiteral("notch bypass on a backend with no notch engine -> announced"));

    // A drag within one turn is one announcement, not five.
    unavailable.clear();
    for (int step = 0; step < 5; ++step)
        radio.setPanAttenuatorFor(QStringLiteral("hl2"), step);
    endTurn();
    check(names(unavailable) == QStringList{QStringLiteral("Attenuator")},
          QStringLiteral("five declines in one turn are one announcement"));
}

void appliedIntentOnlyIsQuiet()
{
    RadioModel radio;
    install<ImplementingBackend>(radio);
    QSignalSpy unavailable(&radio, &RadioModel::controlUnavailable);
    radio.setPanPreampFor(QStringLiteral("hl2"), 1);
    radio.setPanAttenuatorFor(QStringLiteral("hl2"), 1);
    radio.tnfModel().requestGlobalTnfEnabled(true);
    endTurn();
    check(unavailable.isEmpty(),
          QStringLiteral("an implemented intent-only verb is not announced (got [%1])")
              .arg(names(unavailable).join(',')));
}

void disconnectedDeclinesAreSilent()
{
    RadioModel radio;
    BareBackend* backend = install<BareBackend>(radio);
    backend->live = false;
    QSignalSpy unavailable(&radio, &RadioModel::controlUnavailable);
    radio.setPanPreampFor(QStringLiteral("hl2"), 1);
    endTurn();
    check(unavailable.isEmpty(), QStringLiteral("nothing is announced while disconnected"));

    backend->live = true;
    radio.setPanPreampFor(QStringLiteral("hl2"), 2);
    endTurn();
    check(names(unavailable) == QStringList{QStringLiteral("Preamp")},
          QStringLiteral("a disconnected call did not consume the announcement"));
}

void commandPlaneGate()
{
    check(declinedIntentIsADrop(true, false),
          QStringLiteral("connected, no command plane: a decline is a drop"));
    check(!declinedIntentIsADrop(true, true),
          QStringLiteral("a command plane carried the wire text: a decline is not a drop "
                         "(Flex, the demo — their 'Flex takes this as text' defaults)"));
    check(!declinedIntentIsADrop(false, false),
          QStringLiteral("disconnected: nothing is a drop"));
}

void declineAndItsTwinAreOneAnnouncement()
{
    RadioModel radio;
    install<BareBackend>(radio);
    QSignalSpy unavailable(&radio, &RadioModel::controlUnavailable);

    // setVox declines AND `transmit set vox_enable=1` is dropped beside it.
    radio.transmitModel().setVoxEnable(true);
    endTurn();
    check(names(unavailable) == QStringList{QStringLiteral("VOX")},
          QStringLiteral("a decline with a dropped twin is announced once (got [%1])")
              .arg(names(unavailable).join(',')));

    // Toggle plus a level drag: every emission names the same control, so
    // the per-session ledger MainWindow keeps announces VOX exactly once.
    unavailable.clear();
    radio.transmitModel().setVoxEnable(false);
    radio.transmitModel().setVoxLevel(30);
    radio.transmitModel().setVoxLevel(31);
    radio.transmitModel().setVoxLevel(32);
    endTurn();
    DroppedControlAnnouncements ledger;
    QStringList announced;
    for (const QString& name : names(unavailable)) {
        if (ledger.firstThisSession(name))
            announced << name;
    }
    check(announced == QStringList{QStringLiteral("VOX")},
          QStringLiteral("a VOX toggle and a level drag are one control to the operator "
                         "(got [%1])").arg(announced.join(',')));
}

void hostReceiptSilencesTheDecline()
{
    RadioModel radio;
    install<BareBackend>(radio);
    QSignalSpy unavailable(&radio, &RadioModel::controlUnavailable);
    // The client compressor that IS PROC on a host-modulating radio.
    QObject::connect(&radio.transmitModel(), &TransmitModel::speechProcessorCommandIssued,
                     &radio, [&radio](bool, int) {
        radio.noteIntentApplied(ControlIntent::SpeechProcessor);
    });
    radio.transmitModel().setSpeechProcessorEnable(true);
    endTurn();
    check(unavailable.isEmpty(),
          QStringLiteral("setSpeechProcessor declines, but a host receipt in the same turn "
                         "wins (the HL2 PROC case)"));
}

void ritPressIsOneControl()
{
    RadioModel radio;
    BareBackend* backend = install<BareBackend>(radio);
    SliceModel* slice = materializeSlice(radio, *backend);
    if (!slice) {
        check(false, QStringLiteral("slice materialized"));
        return;
    }
    QSignalSpy unavailable(&radio, &RadioModel::controlUnavailable);
    slice->setRit(true, 50);   // setRitEnabled + setRitOffset, both declining
    endTurn();
    check(names(unavailable) == QStringList{QStringLiteral("RIT")},
          QStringLiteral("a RIT press (two verbs) is one announcement (got [%1])")
              .arg(names(unavailable).join(',')));

    unavailable.clear();
    slice->setTxSlice(true);   // bookkeeping: deliberately silent
    endTurn();
    check(unavailable.isEmpty(),
          QStringLiteral("taking TX on a slice is bookkeeping, not announced (got [%1])")
              .arg(names(unavailable).join(',')));
}
} // namespace

int main(int argc, char** argv)
{
    TestSettingsProfile profile(QStringLiteral("aether-intent-decline-announce-test"));
    QCoreApplication app(argc, argv);
    check(profile.isValid(), QStringLiteral("isolated settings profile is available"));
    eachLoudVerbDeclines();
    implementingBackendDeclinesNothing();
    silentDefaultsDoNotDecline();
    commandPlaneGate();
    intentOnlyDeclinesAreAnnounced();
    appliedIntentOnlyIsQuiet();
    disconnectedDeclinesAreSilent();
    declineAndItsTwinAreOneAnnouncement();
    hostReceiptSilencesTheDecline();
    ritPressIsOneControl();
    std::printf("%s\n", failures == 0 ? "ALL PASS" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
