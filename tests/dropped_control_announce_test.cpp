// No drop may be silent, and each dead control is announced by name, once
// (#5263 follow-up). Socket-free: an injected backend on RadioModel's real
// wiring, plus the pure naming/ledger logic MainWindow's notice uses.
//
// Two things were silent before:
//   * a slice built by the non-Flex materializer never had its commandReady
//     connected, so APF, NRL, DAX channel and every other slice control
//     without a typed intent moved, changed the model and reached nothing —
//     no notice, not even a log line;
//   * the operator notice was once per SESSION and unnamed, so the first
//     dropped control hid every later one and the operator could not tell
//     which control the sentence meant.
#include "TestSettingsProfile.h"
#include "core/backends/IRadioBackend.h"
#include "models/CommandDropAccounting.h"
#include "models/RadioModel.h"
#include "models/SliceModel.h"

#include <QCoreApplication>
#include <QSignalSpy>

#include <cstdio>
#include <memory>

using namespace AetherSDR;

namespace {
int failures = 0;
void check(bool condition, const char* description)
{
    std::printf("[%s] %s\n", condition ? "PASS" : "FAIL", description);
    if (!condition)
        ++failures;
}

// Tunes, filters and mixes slice audio on this host; has no radio NR and no
// dial lock — the HL2 shape for the controls exercised here.
class HostSliceBackend : public IRadioBackend
{
public:
    RadioCapabilities caps;
    int frequencyCalls = 0;
    int audioGainCalls = 0;
    RadioCapabilities capabilities() const override { return caps; }
    void connectRadio(const RadioConnectRequest&) override {}
    void disconnectRadio() override {}
    bool isConnected() const override { return true; }
    void setSliceFrequency(int, double) override { ++frequencyCalls; }
    void setSliceMode(int, const QString&) override {}
    void setSliceFilter(int, int, int) override {}
    void setSliceAgc(int, const QString&, int) override {}
    void setPanCenter(const QString&, double, PanCenterIntent) override {}
    void setKeying(bool, const TxCoordinator::Operation&,
                   const TxCoordinator::Completion&) override {}
    void invokeExtension(const QString&, const QString&, quint64, const QVariant&) override {}
    void setSliceAudioGain(int, int) override { ++audioGainCalls; }
};

void endTurn()
{
    QCoreApplication::processEvents();
    QCoreApplication::processEvents();
}

bool droppedContains(const QSignalSpy& spy, const QString& command)
{
    for (const QList<QVariant>& args : spy) {
        if (args.value(0).toString() == command)
            return true;
    }
    return false;
}

void testSliceDropsAreAccounted()
{
    RadioModel radio;
    auto owned = std::make_unique<HostSliceBackend>();
    HostSliceBackend* backend = owned.get();
    radio.setBackendForTest(std::move(owned), QStringLiteral("hl2"));

    emit backend->panCenterBandwidthChanged(QStringLiteral("hl2"), 14.2, 0.1);
    SliceDelta delta;
    delta.panId = QStringLiteral("hl2");
    delta.frequency = 14.2;
    delta.mode = QStringLiteral("USB");
    delta.filterLow = 150;
    delta.filterHigh = 2700;
    delta.inUse = true;
    emit backend->sliceChanged(0, delta);
    SliceModel* slice = radio.slice(0);
    check(slice != nullptr, "the non-Flex materializer built slice 0");
    if (!slice)
        return;

    QSignalSpy dropped(&radio, &RadioModel::commandDropped);

    slice->setApf(true);
    check(droppedContains(dropped, QStringLiteral("slice set 0 apf=1")),
          "APF on a non-Flex slice (no typed intent) is announced, synchronously");

    dropped.clear();
    slice->setNr(true);
    endTurn();
    check(droppedContains(dropped, QStringLiteral("slice set 0 nr=1")),
          "NR whose intent the backend does not implement is announced");

    dropped.clear();
    slice->setFrequency(14.25);
    slice->setAudioGain(30.0f);
    slice->setLocked(true);
    endTurn();
    check(backend->frequencyCalls >= 1 && backend->audioGainCalls >= 1,
          "tune and AF gain reached the backend through the seam");
    check(dropped.isEmpty(),
          "tune, AF gain and a model-enforced lock are applied, so not announced");
}

void testNaming()
{
    check(controlNameForCommand(QStringLiteral("slice set 0 apf=1")) == QLatin1String("APF"),
          "slice key names the control: APF");
    check(controlNameForCommand(QStringLiteral("slice set 2 rit_on=1 rit_freq=50"))
              == QLatin1String("RIT"), "a multi-key slice line is one control: RIT");
    check(controlNameForCommand(QStringLiteral("transmit set rfpower=40"))
              == QLatin1String("RF power"), "transmit key names the control: RF power");
    check(controlNameForCommand(QStringLiteral("transmit set filter_low=100 filter_high=2900"))
              == QLatin1String("TX filter"), "passband pair is TX filter");
    check(controlNameForCommand(QStringLiteral("cw wpm 22")) == QLatin1String("CW speed"),
          "two-word verb names the control: CW speed");
    check(controlNameForCommand(QStringLiteral("filt 0 100 2900")) == QLatin1String("Filter"),
          "positional filt is Filter");
    check(controlNameForCommand(QStringLiteral("profile mic load \"Default\""))
              == QLatin1String("Mic profile"), "profile load names its kind");
    check(controlNameForCommand(QStringLiteral("slice set 0 brand_new_key=3"))
              == QStringLiteral("“brand_new_key”"),
          "an unnamed slice key falls back to the key itself, quoted");
    check(controlNameForCommand(QStringLiteral("sub radio all"))
              == QStringLiteral("“sub radio all”"),
          "an unknown verb falls back to its own words, quoted");
    check(controlNameForCommand(QStringLiteral("display pan set 0x40000000 rfgain_info"))
              == QStringLiteral("“display pan set”"),
          "the fallback stops at the first operand (a pan id)");
}

void testLedgerAndMessage()
{
    DroppedControlAnnouncements ledger;
    check(ledger.firstThisSession(QStringLiteral("APF")), "first APF is announced");
    check(!ledger.firstThisSession(QStringLiteral("APF")), "APF again is not re-announced");
    check(ledger.firstThisSession(QStringLiteral("NRL")),
          "a DIFFERENT control is still announced after the first");
    ledger.reset();
    check(ledger.firstThisSession(QStringLiteral("APF")),
          "a new connect session announces APF again");

    const QString one = droppedControlNotice({QStringLiteral("APF")});
    check(one.startsWith(QLatin1String("APF isn't available on this radio")),
          "one control: named in the sentence");
    const QString two = droppedControlNotice({QStringLiteral("APF"), QStringLiteral("NRL")});
    check(two.startsWith(QLatin1String("APF and NRL aren't available")),
          "two controls: both named");
    const QString many = droppedControlNotice({QStringLiteral("A"), QStringLiteral("B"),
                                               QStringLiteral("C"), QStringLiteral("D"),
                                               QStringLiteral("E")});
    check(many.startsWith(QLatin1String("A, B and 3 more controls aren't available")),
          "a burst collapses to one line with a count");
    check(droppedControlNotice({}).isEmpty(), "nothing to announce, no message");
}
} // namespace

int main(int argc, char** argv)
{
    TestSettingsProfile profile(QStringLiteral("aether-dropped-control-announce-test"));
    QCoreApplication app(argc, argv);
    check(profile.isValid(), "isolated settings profile is available");
    testNaming();
    testLedgerAndMessage();
    testSliceDropsAreAccounted();
    std::printf("%s\n", failures == 0 ? "ALL PASS" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
