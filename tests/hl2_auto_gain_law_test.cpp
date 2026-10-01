// WHICH AUTOMATIC RF GAIN LAW A CONNECT INSTALLS.
//
// RFC #5535 approved the loop driving on the wideband bandscope with a floor of
// 24 dB, and the constructor installs that law. applyRestoredState() -- which
// RadioModel calls on every engaged connect, also with an empty state -- reset
// the law to the clip-counter "ramp" with a floor of 26. So a constructed
// backend and a connected one ran different laws, and no test read the law
// after a restore: hl2_auto_gain_policy_test pins the configurations
// themselves, not which one the backend installs.
//
// WHAT THIS PROVES, AND WHAT IT DOES NOT. Socket-free and radio-free. The link
// edges are fired through MetisClient's own signals, so the backend's real
// handlers run; telemetry and bandscope blocks arrive through the same signals
// the I/O thread emits. That proves which law is installed, that the backend
// ASKS for the bandscope gate when that law is armed on a connect, and that
// the law's decisions read the block that arrives. It does not prove a radio
// answers: MetisClient ignores the enable with no stream behind it, and the
// gate itself is hl2_ep4_gate_test's subject. The positive path on hardware is
// not certified here.
//
// AND WHETHER THE LOOP IS ARMED AT A CONNECT NOBODY ASKED FOR (sections 6-10).
// RFC #5535 approved the loop armed by default and its amendment shipped it
// off, because a +20 dB LNA default above a +19 dB arming ceiling meant the
// connect edge would ask, be refused, and log a warning about a control the
// operator never touched. Those sections drive that same connect edge and
// count the three things the amendment named: the refusal, the warning and
// the settled verdict. A positive control forces the refusal through the test
// seam, so that "none seen" is read off instruments shown to see one.

#include "TestSettingsProfile.h"
#include "core/AppSettings.h"
#include "core/backends/hl2/Hl2AutoGainPolicy.h"
#include "core/backends/hl2/Hl2Backend.h"
#include "core/backends/hl2/Hl2BandscopeHeadroom.h"
#include "core/backends/hl2/MetisClient.h"
#include "core/backends/hl2/MetisProtocol.h"

#include <QCoreApplication>
#include <QEvent>
#include <QJsonObject>
#include <QMutex>
#include <QMutexLocker>
#include <QStringList>
#include <cstdio>

namespace AetherSDR::hl2 {

struct Hl2AutoGainLawTestAccess {
    // The handlers under test are queued connections onto the backend's
    // thread, and this test never returns to an event loop to service them.
    static void drain(Hl2Backend& backend)
    {
        QCoreApplication::sendPostedEvents(&backend, QEvent::MetaCall);
    }
    static void linkUp(Hl2Backend& backend)
    {
        QMetaObject::invokeMethod(backend.m_metis, "linkUp",
                                  Qt::BlockingQueuedConnection);
        drain(backend);
    }
    static void linkDown(Hl2Backend& backend)
    {
        QMetaObject::invokeMethod(backend.m_metis, "linkDown",
                                  Qt::BlockingQueuedConnection);
        drain(backend);
    }
    // WHAT connectRadio() DOES WITH THE BAND'S REMEMBERED GAIN, done by hand.
    // connectRadio() itself hands the DSP opens to the I/O thread and its
    // completion would start the wire, which is what socket-free rules out.
    static void seedConnectBaseline(Hl2Backend& backend, int gainDb)
    {
        backend.m_lnaGainDb = gainDb;
    }
    static void telemetry(Hl2Backend& backend, const Hl2Telemetry& t)
    {
        MetisClient* metis = backend.m_metis;
        QMetaObject::invokeMethod(metis, [metis, t] {
            emit metis->telemetryUpdated(t);
        }, Qt::BlockingQueuedConnection);
        drain(backend);
    }
    static void deliverBlock(Hl2Backend& backend, const Ep4Stats& block)
    {
        MetisClient* metis = backend.m_metis;
        QMetaObject::invokeMethod(metis, [metis, block] {
            emit metis->bandscopeBlockReady(block);
        }, Qt::BlockingQueuedConnection);
        drain(backend);
    }
    static const AutoGainConfig& config(const Hl2Backend& backend)
    {
        return backend.m_autoGainConfig;
    }
    static AutoGainReason reason(const Hl2Backend& backend)
    {
        return backend.m_autoGainReason;
    }
    // TRUE when the backend has asked MetisClient for the bandscope gate on
    // the loop's behalf. The request, not the stream: see the file header.
    static bool gateAskedFor(const Hl2Backend& backend)
    {
        return backend.m_bandscopeOwnedByAutoGain;
    }
};

}  // namespace AetherSDR::hl2

using namespace AetherSDR;
using Access = AetherSDR::hl2::Hl2AutoGainLawTestAccess;

namespace {
int failures = 0;
void check(bool condition, const char* label)
{
    std::printf("%s %s\n", condition ? "[ OK ]" : "[FAIL]", label);
    if (!condition) {
        ++failures;
    }
}

// THE RULING'S OWN NUMBERS, written here and not read from the code under
// test: #5535 point 3 approved the bandscope as the input, point 2 holds
// maxOffsetDb at 24 dB.
const QString kApprovedLaw = QStringLiteral("bandscope");
constexpr int kApprovedFloorDb = 24;
// Inside the range the loop arms from on any ceiling this branch can meet.
constexpr int kTrustedBaselineDb = 10;

QVariant row(const hl2::Hl2Backend& backend, const char* key)
{
    return backend.healthSnapshot().values.value(QString::fromLatin1(key));
}

int autoOffsetDb(const hl2::Hl2Backend& backend)
{
    return row(backend, "lnaAutoOffsetDb").toInt();
}

// A profile as currentOperatingState() writes it: the operator's switch and
// one band's gain. Those are the only automatic-gain facts that are persisted.
RestoredRadioState savedProfile(bool autoWanted)
{
    RestoredRadioState state;
    state.rfFrequencyHz = 14'074'000.0;
    state.sampleRateHz = 48'000;
    state.extensionSchemaVersion = 1;
    QJsonObject rfGain{
        {QStringLiteral("defaultDb"), 20},
        {QStringLiteral("lnaDbByBand"),
         QJsonObject{{QStringLiteral("20m"), kTrustedBaselineDb}}}};
    if (autoWanted) {
        rfGain.insert(QStringLiteral("autoEnabled"), true);
    }
    state.extension = QJsonObject{{QStringLiteral("rfGain"), rfGain}};
    return state;
}

QJsonObject storedRfGain(const hl2::Hl2Backend& backend)
{
    return backend.currentOperatingState().extension
        .value(QStringLiteral("rfGain")).toObject();
}

// One telemetry window of converter observations.
hl2::Hl2Telemetry window(int overloadSamples, int windowMs)
{
    hl2::Hl2Telemetry t;
    t.adcSamples = 100;
    t.adcOverloadSamples = overloadSamples;
    t.adcWindowMs = windowMs;
    return t;
}

// A bandscope block far from the rail: peak code 8 of 2048 is 48 dB of
// headroom, more than any release step plus its bias and margin.
hl2::Ep4Stats quietBlock()
{
    hl2::Ep4Stats block;
    block.samples = 2048;
    block.peakAbs = 8;
    block.sumSquares = 2048.0 * 16.0;
    return block;
}

// applyRestoredState, the hand-seeded connect baseline, then the link edge.
void connectWith(hl2::Hl2Backend& backend, const RestoredRadioState& state)
{
    backend.applyRestoredState(state);
    Access::seedConnectBaseline(backend, kTrustedBaselineDb);
    Access::linkUp(backend);
}

// THE AMENDMENT'S OWN NUMBER, written here and not read from the code under
// test: "This radio's constructed LNA default is +20 dB".
constexpr int kShippedLnaDefaultDb = 20;

// A profile that has never said anything about the switch, in the two forms it
// reaches the backend: no memory at all (a first connect of this radio), and a
// document with gains in it and no `autoEnabled` key.
RestoredRadioState noMemory()
{
    return RestoredRadioState{};
}
RestoredRadioState gainsButNoSwitch()
{
    RestoredRadioState state;
    state.rfFrequencyHz = 14'074'000.0;
    state.sampleRateHz = 48'000;
    state.extensionSchemaVersion = 1;
    state.extension = QJsonObject{
        {QStringLiteral("rfGain"), QJsonObject{
            {QStringLiteral("lnaDbByBand"),
             QJsonObject{{QStringLiteral("40m"), 7}}}}}};
    return state;
}
// The same document with the operator's switch written in it, either way.
RestoredRadioState withSwitch(const QJsonValue& value)
{
    RestoredRadioState state = gainsButNoSwitch();
    QJsonObject rfGain = state.extension.value(QStringLiteral("rfGain")).toObject();
    rfGain.insert(QStringLiteral("autoEnabled"), value);
    state.extension.insert(QStringLiteral("rfGain"), rfGain);
    return state;
}

// applyRestoredState and the link edge, with the baseline applyRestoredState
// itself leaves: nothing is seeded by hand.
void connectUnseeded(hl2::Hl2Backend& backend, const RestoredRadioState& state)
{
    backend.applyRestoredState(state);
    Access::linkUp(backend);
}

// EVERY SETTLED VERDICT, and the refusal reason as a handler would have read
// it when the verdict arrived. MainWindow::onAutoRfGainArmSettled shows the
// refusal card from exactly this pair.
class SettledLog {
public:
    explicit SettledLog(hl2::Hl2Backend& backend)
    {
        m_conn = QObject::connect(&backend, &IRadioBackend::autoRfGainArmSettled,
                                  &backend, [this, &backend](bool armed) {
            ++m_settles;
            armed ? ++m_armed : ++m_notArmed;
            m_reasonAtEmit = backend.lastArmRefusalReason();
        });
    }
    ~SettledLog() { QObject::disconnect(m_conn); }
    SettledLog(const SettledLog&) = delete;
    SettledLog& operator=(const SettledLog&) = delete;

    int settles() const { return m_settles; }
    int armed() const { return m_armed; }
    int notArmed() const { return m_notArmed; }
    QString reasonAtEmit() const { return m_reasonAtEmit; }

private:
    QMetaObject::Connection m_conn;
    int m_settles = 0;
    int m_armed = 0;
    int m_notArmed = 0;
    QString m_reasonAtEmit;
};

// HOW MANY TIMES THE BACKEND SAID ITS DOCUMENT MOVED, and what the document
// said at that moment. RadioModel saves on this signal and on nothing else.
class SaveLog {
public:
    explicit SaveLog(hl2::Hl2Backend& backend)
    {
        m_conn = QObject::connect(&backend, &IRadioBackend::operatingStateChanged,
                                  &backend, [this, &backend] {
            ++m_saves;
            m_stored = backend.currentOperatingState().extension
                           .value(QStringLiteral("rfGain")).toObject();
        });
    }
    ~SaveLog() { QObject::disconnect(m_conn); }
    SaveLog(const SaveLog&) = delete;
    SaveLog& operator=(const SaveLog&) = delete;

    int saves() const { return m_saves; }
    QJsonValue storedSwitch() const { return m_stored.value(QStringLiteral("autoEnabled")); }

private:
    QMetaObject::Connection m_conn;
    int m_saves = 0;
    QJsonObject m_stored;
};

// EVERY WARNING THE PROCESS LOGS WHILE THIS IS ALIVE. The amendment's harm was
// "a warning in the log about a control they never asked for", so the log is
// an instrument here and not noise. Warnings from any thread are kept; the
// ones about this control are the ones that name it.
class WarningLog {
public:
    WarningLog()
    {
        s_self = this;
        m_previous = qInstallMessageHandler(&WarningLog::handle);
    }
    ~WarningLog()
    {
        qInstallMessageHandler(m_previous);
        s_self = nullptr;
    }
    WarningLog(const WarningLog&) = delete;
    WarningLog& operator=(const WarningLog&) = delete;

    int aboutAutoGain() const
    {
        QMutexLocker lock(&m_mutex);
        int n = 0;
        for (const QString& line : m_warnings) {
            if (line.contains(QLatin1String("Auto RF gain"), Qt::CaseInsensitive)) {
                ++n;
            }
        }
        return n;
    }
    int total() const
    {
        QMutexLocker lock(&m_mutex);
        return static_cast<int>(m_warnings.size());
    }

private:
    static void handle(QtMsgType type, const QMessageLogContext& context,
                       const QString& message)
    {
        if (!s_self) {
            return;
        }
        if (type == QtWarningMsg || type == QtCriticalMsg) {
            QMutexLocker lock(&s_self->m_mutex);
            s_self->m_warnings.append(message);
        }
        if (s_self->m_previous) {
            s_self->m_previous(type, context, message);
        } else {
            // Nothing was installed before: keep the line visible in the
            // test's own output, which is where a failure will be read.
            std::fprintf(stderr, "%s\n", qPrintable(message));
        }
    }
    static inline WarningLog* s_self = nullptr;
    QtMessageHandler m_previous = nullptr;
    mutable QMutex m_mutex;
    QStringList m_warnings;
};
}  // namespace

int main(int argc, char** argv)
{
    TestSettingsProfile profile(QStringLiteral("aether-hl2-auto-gain-law"));
    if (!profile.isValid()) {
        return 1;
    }
    QCoreApplication app(argc, argv);
    AppSettings::instance().load();

    // ---- 1. A CONSTRUCTED BACKEND RUNS THE APPROVED LAW --------------------
    // The reference every later section compares against, so it is pinned to
    // the ruling first: two backends agreeing with each other proves nothing
    // if both are wrong.
    {
        hl2::Hl2Backend fresh;
        check(fresh.law() == kApprovedLaw,
              "a constructed backend names the bandscope law");
        check(fresh.laws().value(0) == fresh.law(),
              "and it is the first law listed, which is the documented default");
        check(fresh.floorDb() == kApprovedFloorDb,
              "its floor is the 24 dB the ruling holds");
        check(Access::config(fresh).requireHeadroomToRelease,
              "and its release needs a measured headroom reading");
        check(row(fresh, "autoRfGainMode").toString() == kApprovedLaw,
              "the health row reports the same law");
        check(!fresh.isArmed(), "installing a law does not arm the loop");
    }

    // ---- 2. applyRestoredState({}) INSTALLS WHAT THE CONSTRUCTOR INSTALLS --
    // "This radio has no memory", which is what RadioModel hands over on a
    // first connect. Fails on main: law "ramp", floor 26.
    {
        hl2::Hl2Backend fresh;
        hl2::Hl2Backend restored;
        restored.applyRestoredState(RestoredRadioState{});
        check(restored.law() == fresh.law(),
              "after applyRestoredState({}) the law is the constructed one");
        check(restored.floorDb() == fresh.floorDb(),
              "after applyRestoredState({}) the floor is the constructed one");
        check(Access::config(restored) == Access::config(fresh),
              "after applyRestoredState({}) every field of the config matches");
        check(row(restored, "autoRfGainMode") == row(fresh, "autoRfGainMode")
                  && row(restored, "autoRfGainFloorDb") == row(fresh, "autoRfGainFloorDb")
                  && row(restored, "autoRfGainReleaseNeedsDb")
                         == row(fresh, "autoRfGainReleaseNeedsDb"),
              "and the health rows an operator reads agree with it");
        check(row(restored, "autoRfGainReleaseNeedsDb").isValid(),
              "positive control: the release-needs row exists, so the "
              "comparison above is not two absent values agreeing");
    }

    // ---- 3. A RESTORE WITH SAVED STATE: PERSISTED MEMBERS COME BACK, THE ----
    // ---- REST RETURN TO THE CONSTRUCTED DEFAULT ----------------------------
    // The law and the floor are session state. A session that selected
    // another law and a deeper floor must not leak them into the next
    // connect, and the operator's saved switch and gain must survive.
    {
        hl2::Hl2Backend fresh;
        hl2::Hl2Backend backend;
        check(backend.setLaw(QStringLiteral("ramp")) && backend.law() == QLatin1String("ramp"),
              "positive control: the session selected another law");
        backend.setFloorDb(hl2::Hl2Backend::kAutoRfGainFloorMaxDb);
        check(backend.floorDb() == hl2::Hl2Backend::kAutoRfGainFloorMaxDb,
              "positive control: and a deeper floor");

        backend.applyRestoredState(savedProfile(true));
        check(backend.law() == fresh.law(),
              "a restore with saved state installs the constructed law");
        check(Access::config(backend) == Access::config(fresh),
              "and the constructed config, floor included");
        const QJsonObject stored = storedRfGain(backend);
        check(stored.value(QStringLiteral("autoEnabled")).toBool(false),
              "the operator's saved switch is restored");
        check(stored.value(QStringLiteral("lnaDbByBand")).toObject()
                  .value(QStringLiteral("20m")).toInt(999) == kTrustedBaselineDb,
              "and so is the saved per-band gain");
        check(!backend.isArmed(),
              "the wish alone does not run the loop before the link is up");
    }

    // ---- 4. "default" NAMES THE DEFAULT ------------------------------------
    {
        hl2::Hl2Backend fresh;
        hl2::Hl2Backend backend;
        check(backend.setLaw(QStringLiteral("ramp")), "positive control: leave the default");
        check(backend.setLaw(QStringLiteral("default")), "\"default\" is accepted");
        check(backend.law() == fresh.law()
                  && Access::config(backend) == Access::config(fresh),
              "and installs the law a constructed backend has");
    }

    // ---- 5. AFTER A CONNECT, THE BANDSCOPE LAW IS WHAT STEPS ---------------
    {
        hl2::Hl2Backend backend;
        connectWith(backend, savedProfile(true));
        check(backend.isConnected(), "positive control: the link edge was delivered");
        check(backend.isArmed(),
              "a saved switch arms the loop at the connect edge");
        check(backend.law() == kApprovedLaw
                  && row(backend, "autoRfGainMode").toString() == kApprovedLaw,
              "the armed loop runs the bandscope law");
        check(Access::gateAskedFor(backend),
              "and the backend asked for the bandscope gate that law reads");

        // Clean windows first: nothing to do, nothing held.
        for (int i = 0; i < 3; ++i) {
            Access::telemetry(backend, window(0, 100));
        }
        check(autoOffsetDb(backend) == 0, "clean windows hold no offset");

        // An occasional clip. The ramp's first step out of a quiet period is
        // 3 dB for this rate; the bandscope law steps one 6 dB quantum.
        Access::telemetry(backend, window(10, 100));
        check(autoOffsetDb(backend) == 6,
              "an occasional clip is answered with the bandscope law's 6 dB "
              "step, not the ramp's 3");

        // Clean for longer than any dwell, with no bandscope block. The ramp
        // and the probing law would both have given gain back by now.
        for (int i = 0; i < 40; ++i) {
            Access::telemetry(backend, window(0, 1000));
        }
        check(autoOffsetDb(backend) == 6,
              "with no bandscope reading the law gives nothing back");
        check(Access::reason(backend) == hl2::AutoGainReason::HeadroomAbsent,
              "and says the reading is what it is waiting for");

        // The reading arrives and has room for the step.
        const double needsDb = row(backend, "autoRfGainReleaseNeedsDb").toDouble();
        const hl2::HeadroomObservation seen = hl2::bandscopeHeadroom(quietBlock(), 0);
        check(seen.isMeasurement() && needsDb > 0.0 && seen.headroomDb >= needsDb,
              "positive control: the block carries more headroom than a release needs");
        Access::deliverBlock(backend, quietBlock());
        Access::telemetry(backend, window(0, 1000));
        check(autoOffsetDb(backend) == 0,
              "a measured reading with room licenses the release");
        check(Access::reason(backend) == hl2::AutoGainReason::Release,
              "and the decision is recorded as a release");

        // ---- the link drops and resumes on the same session ----
        // MetisClient re-emits linkUp when EP6 resumes after a silence; no
        // connectRadio() and no applyRestoredState() run in between.
        Access::linkDown(backend);
        check(!Access::gateAskedFor(backend) && backend.isArmed(),
              "a link drop ends the claim on the gate and leaves the loop armed");
        Access::linkUp(backend);
        check(Access::gateAskedFor(backend),
              "the resumed link asks for the gate again");

        // ---- and a law that does not read the bandscope lets it go ----
        check(backend.setLaw(QStringLiteral("ramp")), "select the ramp");
        check(!Access::gateAskedFor(backend),
              "the ramp does not read the bandscope, so the gate is released");
        backend.disconnectRadio();
    }

    // ---- 6. THE INSTRUMENTS SEE A REFUSAL WHEN THERE IS ONE ----------------
    // The positive control for sections 7 and 9, and it comes first. This is
    // the connect the amendment described: the wish is on, the baseline is
    // above the arming ceiling, the connect edge asks and is refused. No
    // public path produces a baseline above the ceiling any more (the setter
    // and the restore both clamp to the native range), so it is forced through
    // the test seam. What matters is that the three instruments below register
    // it, because sections 7 and 9 read "none" off the same three.
    {
        hl2::Hl2Backend backend;
        WarningLog warnings;
        SettledLog settled(backend);
        backend.applyRestoredState(noMemory());
        Access::seedConnectBaseline(backend,
                                    hl2::Hl2Backend::kAutoRfGainMaxBaselineDb + 1);
        Access::linkUp(backend);
        check(backend.isConnected(), "positive control: the link edge was delivered");
        check(!backend.isArmed(),
              "positive control: a baseline above the ceiling is refused at the connect edge");
        check(settled.settles() == 1 && settled.notArmed() == 1,
              "positive control: the refusal settles once, as not armed");
        check(!settled.reasonAtEmit().isEmpty() && !backend.lastArmRefusalReason().isEmpty(),
              "positive control: with a reason, which is what the GUI shows as a card");
        check(warnings.aboutAutoGain() == 1,
              "positive control: and with one warning in the log that names the control");
        backend.disconnectRadio();
    }

    // ---- 7. NO SAVED PREFERENCE: ARMED AT +20, ON THE BANDSCOPE LAW, --------
    // ---- AND NOTHING IS REFUSED --------------------------------------------
    // The first connect of a radio this installation has no memory of. Not
    // seeded by hand: the baseline is the one applyRestoredState() leaves.
    {
        check(hl2::kLnaDefaultGainDb == kShippedLnaDefaultDb,
              "the shipped LNA default is still the +20 dB the amendment names");

        hl2::Hl2Backend backend;
        WarningLog warnings;
        SettledLog settled(backend);
        SaveLog saves(backend);
        connectUnseeded(backend, noMemory());

        check(backend.isConnected(), "positive control: the link edge was delivered");
        check(backend.isArmed(),
              "with no saved preference the loop is armed at the connect edge");
        check(backend.lnaBaselineDb() == kShippedLnaDefaultDb,
              "from the shipped +20 dB baseline");
        check(row(backend, "lnaGainDb").toInt() == kShippedLnaDefaultDb
                  && autoOffsetDb(backend) == 0,
              "and arming moves nothing: the gain on the wire is still +20, offset 0");
        check(backend.law() == kApprovedLaw
                  && row(backend, "autoRfGainMode").toString() == kApprovedLaw,
              "the armed loop runs the bandscope law");
        check(backend.floorDb() == kApprovedFloorDb,
              "with the 24 dB floor the ruling holds");
        check(Access::gateAskedFor(backend),
              "and the backend asked for the bandscope gate that law reads");

        // WHAT THE AMENDMENT SAID WOULD BE WRONG WITH DEFAULT-ON, item by item.
        check(settled.settles() == 1 && settled.armed() == 1 && settled.notArmed() == 0,
              "the connect edge settles once, as ARMED: nothing was refused");
        check(settled.reasonAtEmit().isEmpty() && backend.lastArmRefusalReason().isEmpty(),
              "there is no refusal reason, so no card and no announcement");
        check(warnings.aboutAutoGain() == 0,
              "and no warning in the log about a control the operator never touched");

        // The default is recorded as the wish, and the profile is told.
        check(storedRfGain(backend).value(QStringLiteral("autoEnabled")).toBool(false),
              "the document now says the loop is wanted");
        check(saves.saves() > 0 && saves.storedSwitch().toBool(false),
              "and the backend announced it, so the profile is written");

        // Armed is not attenuating. Without clip evidence nothing moves.
        for (int i = 0; i < 40; ++i) {
            Access::telemetry(backend, window(0, 1000));
        }
        check(autoOffsetDb(backend) == 0
                  && row(backend, "lnaGainDb").toInt() == kShippedLnaDefaultDb,
              "40 s of clean windows: no attenuation, the wire still carries +20");
        // And the loop is live: clip evidence is answered.
        Access::telemetry(backend, window(10, 100));
        check(autoOffsetDb(backend) == 6
                  && row(backend, "lnaGainDb").toInt() == kShippedLnaDefaultDb - 6
                  && backend.lnaBaselineDb() == kShippedLnaDefaultDb,
              "positive control: a clip is answered with one 6 dB step, and the "
              "operator's own number is not touched");
        check(warnings.aboutAutoGain() == 0,
              "still without a warning");

        // ---- the operator switches it off, and it stays off ----
        backend.setAutoRfGain(false);
        check(!backend.isArmed() && autoOffsetDb(backend) == 0
                  && row(backend, "lnaGainDb").toInt() == kShippedLnaDefaultDb,
              "an explicit off disarms and restores +20 in one action");
        check(!Access::gateAskedFor(backend),
              "and releases the bandscope gate the loop started");
        const QJsonValue switchAfterOff =
            storedRfGain(backend).value(QStringLiteral("autoEnabled"));
        check(switchAfterOff.isBool() && !switchAfterOff.toBool(),
              "the off is written as an explicit false, not as a missing key");
        check(saves.storedSwitch().isBool() && !saves.storedSwitch().toBool(),
              "and it reaches the profile");

        // The next connect reads that document. The default must not undo it.
        const RestoredRadioState saved = backend.currentOperatingState();
        backend.disconnectRadio();
        hl2::Hl2Backend next;
        SettledLog nextSettled(next);
        connectUnseeded(next, saved);
        check(next.isConnected(), "positive control: the next connect was delivered");
        check(!next.isArmed() && nextSettled.settles() == 0,
              "on the next connect the saved off holds: the loop is not armed "
              "and nothing is even asked");
        check(!Access::gateAskedFor(next),
              "and no bandscope gate is asked for");
        next.disconnectRadio();
    }

    // ---- 8. THE THREE THINGS A DOCUMENT CAN SAY ----------------------------
    // A document with gains in it, differing only in the switch.
    {
        // No key: armed. The other form of "never said".
        hl2::Hl2Backend absent;
        SettledLog absentSettled(absent);
        connectUnseeded(absent, gainsButNoSwitch());
        check(absent.isConnected() && absent.isArmed(),
              "a document with gains and no switch arms at the connect edge");
        check(absentSettled.settles() == 1 && absentSettled.armed() == 1
                  && absent.lastArmRefusalReason().isEmpty(),
              "once, as armed, with no refusal");
        check(Access::gateAskedFor(absent) && absent.law() == kApprovedLaw,
              "on the bandscope law, with its gate asked for");
        absent.disconnectRadio();

        // An explicit false: off, and it stays off under clip evidence.
        hl2::Hl2Backend off;
        SettledLog offSettled(off);
        SaveLog offSaves(off);
        connectUnseeded(off, withSwitch(false));
        check(off.isConnected(), "positive control: the link edge was delivered");
        check(!off.isArmed(), "an explicit saved off does not arm");
        check(offSettled.settles() == 0,
              "nothing is asked of the control, so nothing settles");
        check(off.law() == kApprovedLaw, "the law is installed all the same");
        check(!Access::gateAskedFor(off),
              "and no bandscope gate is asked for while the loop is off");
        for (int i = 0; i < 10; ++i) {
            Access::telemetry(off, window(100, 100));
        }
        check(autoOffsetDb(off) == 0 && !off.isArmed(),
              "ten windows of solid clipping: the switched-off loop takes no gain");
        const QJsonValue stillOff = storedRfGain(off).value(QStringLiteral("autoEnabled"));
        check(stillOff.isBool() && !stillOff.toBool(),
              "and the document still says off");
        off.disconnectRadio();

        // An explicit true: on, as before this change.
        hl2::Hl2Backend on;
        SettledLog onSettled(on);
        connectUnseeded(on, withSwitch(true));
        check(on.isConnected() && on.isArmed(), "an explicit saved on arms");
        check(onSettled.settles() == 1 && onSettled.armed() == 1,
              "once, as armed");
        check(storedRfGain(on).value(QStringLiteral("autoEnabled")).toBool(false),
              "and the document still says on");
        on.disconnectRadio();

        // A value that is not a boolean is not an operator's off. The restore
        // boundary drops a field that fails validation, and the default
        // decides. Stated so that it is a decision and not an accident.
        hl2::Hl2Backend garbled;
        connectUnseeded(garbled, withSwitch(QStringLiteral("off")));
        check(garbled.isArmed(),
              "a switch that is not a boolean is dropped, and the default arms");
        garbled.disconnectRadio();
    }

    // ---- 9. NO BASELINE A PROFILE CAN HOLD IS REFUSED ----------------------
    // The amendment's failure was a refusal at the connect edge. A fresh
    // profile comes up on +20, and an old one can hold any gain in the native
    // range for the band it comes up on. Every one of them, with no saved
    // switch: armed, one settled verdict, no reason, no warning.
    {
        int tried = 0;
        int armedCount = 0;
        int cleanCount = 0;
        WarningLog warnings;
        for (int gainDb = hl2::kLnaGainMinDb; gainDb <= hl2::kLnaGainMaxDb; ++gainDb) {
            hl2::Hl2Backend backend;
            SettledLog settled(backend);
            backend.applyRestoredState(noMemory());
            Access::seedConnectBaseline(backend, gainDb);
            Access::linkUp(backend);
            ++tried;
            if (backend.isConnected() && backend.isArmed()
                && backend.lnaBaselineDb() == gainDb) {
                ++armedCount;
            }
            if (settled.settles() == 1 && settled.armed() == 1
                && backend.lastArmRefusalReason().isEmpty()) {
                ++cleanCount;
            }
            backend.disconnectRadio();
        }
        check(tried == 61 && hl2::kLnaGainMinDb == -12 && hl2::kLnaGainMaxDb == 48,
              "positive control: all 61 baselines of the native -12..+48 dB range were tried");
        check(armedCount == tried,
              "every one of them arms at the connect edge, baseline untouched");
        check(cleanCount == tried,
              "every one settles once as armed, with no refusal reason");
        check(warnings.aboutAutoGain() == 0,
              "and none of the 61 connects logs a warning about the control");
    }

    // ---- 10. THE CONSTRUCTED DEFAULT AND THE RESTORED ONE AGREE -------------
    // The law's default had three copies and one was moved without the others.
    // The switch's default has one constant; this is the check that it is the
    // only one.
    {
        hl2::Hl2Backend fresh;
        hl2::Hl2Backend restored;
        restored.applyRestoredState(noMemory());
        const QJsonValue constructed =
            storedRfGain(fresh).value(QStringLiteral("autoEnabled"));
        const QJsonValue afterRestore =
            storedRfGain(restored).value(QStringLiteral("autoEnabled"));
        check(constructed.isBool() && constructed.toBool(),
              "a constructed backend's document says the loop is wanted");
        check(afterRestore.isBool() && afterRestore.toBool() == constructed.toBool(),
              "and applyRestoredState({}) leaves the same answer");
        check(hl2::Hl2Backend::kAutoRfGainArmedByDefault == constructed.toBool(),
              "which is the one constant both read");
        check(!fresh.isArmed() && !restored.isArmed(),
              "wanted is not running: neither is armed before a link is up");
    }

    return failures ? 1 : 0;
}
