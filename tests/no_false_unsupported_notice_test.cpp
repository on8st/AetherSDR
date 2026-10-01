// A control that WORKED must not be reported as unsupported.
//
// On a radio with no Flex command plane, RadioModel::sendCmd() drops Flex wire
// text and emits commandDropped, which MainWindow turns into the once-per-
// session notice "This radio doesn't support that control" (#5263). That is
// how the controls that really do nothing are found. Some controls act through
// another path and STILL sent their Flex text, so the notice was raised for a
// control that had just worked, and the one-shot latch was then spent:
//
//   PROC and NOR/DX/DX+        served by the ClientComp in the host's TX chain
//                              (MainWindow::applySpeechProcessorToClientComp)
//   WtrFall Gain               rendered by the client (SpectrumWidget)
//   the dBm scale              a local scale on a radio that takes no range
//   Clone to all Pans          copies client-rendered values between widgets
//   Reset display defaults     resets those values on the widget
//
// NOT in that list, and pinned here as still loud: the manual Black Level
// slider. By source reading it changes nothing on a backend whose waterfall
// rows are dBm (SpectrumWidget::intensityToWaterfallLevel() compares the row
// with 160 - level, a tile-intensity threshold), so its notice stays until the
// renderer is fixed.
//
// WHAT THIS TEST PROVES, AND HOW.
//
// 1. PROC, by execution. An injected backend with no command plane stands in
//    for the radio. The operator intent that drives the compressor
//    (TransmitModel::speechProcessorCommandIssued) is observed leaving the
//    model, and no commandDropped follows. The alarm is narrowed, not
//    silenced: an unrouted verb, a line that also carries another key, a
//    receive-only host-DSP radio and a radio that modulates itself all still
//    raise it. On a Flex-shaped model (a radio-modulated backend plus an
//    unopened RadioConnection with a command sink; no socket) the wire text is
//    compared byte for byte.
//
// 2. The panadapter sites, by SOURCE TEXT. They are lambdas inside
//    MainWindow::wirePanadapter(), and MainWindow is not a test target, so
//    they cannot be run here. The test reads MainWindow_Wiring.cpp with its
//    comments stripped and requires, at each site, that the local effect comes
//    first, that hasCommandPlane() gates the send, and that the wire text
//    literal is the one a Flex has always been sent. That proves the code is
//    WRITTEN this way and fails on a reversal. It does not prove the code
//    runs. The same reading pins MainWindow's half of PROC: the binding of the
//    intent to the compressor, behind the same predicate RadioModel asks.
//
// 3. The guard is necessary, by execution: the same display text handed
//    straight to RadioModel::sendCommand() on the no-command-plane model is
//    still dropped loudly.
//
// Socket-free. No radio, no peer, no application window.

#include "TestSettingsProfile.h"
#include "core/HostVoiceChainPolicy.h"
#include "core/backends/flex/RadioConnection.h"
#include "models/RadioModel.h"
#include "models/TransmitModel.h"

#include <QCoreApplication>
#include <QFile>
#include <QPair>
#include <QStringList>

#include <cstdio>
#include <memory>
#include <utility>

namespace AetherSDR {
// RadioModel and RadioConnection both befriend a class of this name for
// socket-free tests; this is this file's own definition of it. It gives the
// model a command plane (an unopened connection) and captures what is written.
class TxOperationIntegrationTestAccess {
public:
    static void useCommandPlane(RadioModel& radio, RadioConnection& connection,
                                QStringList& written)
    {
        connection.m_commandSinkForTest =
            [&written](quint32, const QString& command) { written << command; };
        radio.m_connection = &connection;
    }
    static void setFamily(RadioModel& radio, const QString& family)
    {
        radio.m_family = family;
    }
    static void releaseCommandPlane(RadioModel& radio) { radio.m_connection = nullptr; }
};
} // namespace AetherSDR

using namespace AetherSDR;

namespace {
int failures = 0;
void check(bool ok, const QString& message)
{
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", qPrintable(message));
    failures += !ok;
}

// The predicate, at compile time: the host serves PROC only when it both runs
// the modulator and may transmit.
static_assert(hostRunsTxVoiceChain(true, true));
static_assert(!hostRunsTxVoiceChain(true, false));
static_assert(!hostRunsTxVoiceChain(false, true));
static_assert(!hostRunsTxVoiceChain(false, false));

// A backend that implements NO transmit setter beyond the pure virtuals: like
// the Hermes-Lite 2, it inherits IRadioBackend's do-nothing setSpeechProcessor.
class StubBackend final : public IRadioBackend {
public:
    RadioCapabilities caps;
    RadioCapabilities capabilities() const override { return caps; }
    bool isConnected() const override { return true; }
    void connectRadio(const RadioConnectRequest&) override {}
    void disconnectRadio() override {}
    void setSliceFrequency(int, double) override {}
    void setSliceMode(int, const QString&) override {}
    void setSliceFilter(int, int, int) override {}
    void setSliceAgc(int, const QString&, int) override {}
    void setPanCenter(const QString&, double, PanCenterIntent) override {}
    void setKeying(bool, const TxCoordinator::Operation&,
                   const TxCoordinator::Completion&) override {}
    void invokeExtension(const QString&, const QString&, quint64, const QVariant&) override {}
};

// The Hermes-Lite 2's answers to the two questions the gate asks.
RadioCapabilities hostModulatingTransmitter()
{
    RadioCapabilities c;
    c.family = QStringLiteral("hl2");
    c.canTransmit = true;
    c.hostModulates = true;
    return c;
}

// A Flex's answers: it may transmit and the radio modulates. The Flex-shaped
// fixtures add the command plane with useCommandPlane().
RadioCapabilities radioModulatedTransmitter()
{
    RadioCapabilities c;
    c.family = QStringLiteral("flex");
    c.canTransmit = true;
    c.hostModulates = false;
    return c;
}

struct Fixture {
    RadioModel radio;
    QStringList dropped;
    QList<QPair<bool, int>> procIntents;

    explicit Fixture(const RadioCapabilities& caps)
    {
        auto owned = std::make_unique<StubBackend>();
        owned->caps = caps;
        radio.setBackendForTest(std::move(owned), caps.family);
        QObject::connect(&radio, &RadioModel::commandDropped, &radio,
                         [this](const QString& cmd) { dropped << cmd; });
        QObject::connect(&radio.transmitModel(),
                         &TransmitModel::speechProcessorCommandIssued, &radio,
                         [this](bool on, int level) { procIntents << qMakePair(on, level); });
    }

    // After setBackendForTest() the model holds no connection; a Flex-shaped
    // fixture is handed an unopened one that records what is written to it.
    void useCommandPlane(RadioConnection& connection, QStringList& written)
    {
        TxOperationIntegrationTestAccess::setFamily(radio, QStringLiteral("flex"));
        TxOperationIntegrationTestAccess::useCommandPlane(radio, connection, written);
    }
    ~Fixture() { TxOperationIntegrationTestAccess::releaseCommandPlane(radio); }

    bool droppedContaining(const QString& fragment) const
    {
        for (const QString& cmd : dropped) {
            if (cmd.contains(fragment)) {
                return true;
            }
        }
        return false;
    }
};

// ── 1. PROC, by execution ────────────────────────────────────────────────────

void procActsWithoutNoticeOnHostModulatingRadio()
{
    Fixture f(hostModulatingTransmitter());
    check(!f.radio.hasCommandPlane(),
          "premise: the injected backend has no command plane");

    f.radio.transmitModel().setSpeechProcessorEnable(true);
    check(f.procIntents == QList<QPair<bool, int>>{qMakePair(true, 0)},
          "PROC on: the intent that drives the host compressor left the model");
    check(f.radio.transmitModel().speechProcessorEnable(),
          "PROC on: the model shows it on");
    check(f.dropped.isEmpty(),
          "PROC on: no commandDropped for a control the host chain serves");

    f.radio.transmitModel().setSpeechProcessorLevel(2);
    check(f.procIntents.size() == 2 && f.procIntents.last() == qMakePair(true, 2),
          "DX+: the intent carries the new level");
    check(f.dropped.isEmpty(),
          "DX+: no commandDropped for a level the host chain serves");

    f.radio.transmitModel().setSpeechProcessorEnable(false);
    check(f.procIntents.size() == 3 && f.procIntents.last() == qMakePair(false, 2),
          "PROC off: the intent left the model");
    check(f.dropped.isEmpty(), "PROC off: no commandDropped");
}

// The negative controls. Same backend, same model: what the gate must NOT
// swallow. A blanket gate on the commandReady forward turns these red.
void theNoticeStillFiresForWhatDoesNothing()
{
    Fixture f(hostModulatingTransmitter());
    f.radio.transmitModel().setVoxEnable(true);
    check(f.droppedContaining(QStringLiteral("transmit set vox_enable=")),
          "vox: a verb nothing behind this backend implements still raises the notice");

    // A line that carries a speech processor key AND something else is not
    // "served by the host chain" as a whole, so it keeps its drop.
    f.dropped.clear();
    emit f.radio.transmitModel().commandReady(
        QStringLiteral("transmit set speech_processor_enable=1 vox_enable=1"));
    check(f.dropped == QStringList{QStringLiteral(
              "transmit set speech_processor_enable=1 vox_enable=1")},
          "a line with a second, unrouted key still raises the notice");

    // Not a `transmit set` line at all.
    f.dropped.clear();
    f.radio.transmitModel().setMicAcc(true);
    check(f.droppedContaining(QStringLiteral("mic acc 1")),
          "mic acc: an unrelated transmit verb still raises the notice");
}

// A receive-only host-DSP radio (the ANAN's shape today): the host modulates
// nothing that is transmitted, so PROC serves nothing and the notice is true.
void procKeepsNoticeOnReceiveOnlyRadio()
{
    RadioCapabilities caps = hostModulatingTransmitter();
    caps.canTransmit = false;
    Fixture f(caps);
    f.radio.transmitModel().setSpeechProcessorEnable(true);
    check(f.droppedContaining(QStringLiteral("transmit set speech_processor_enable=1")),
          "receive-only host-DSP radio: PROC still raises the notice");
    f.radio.transmitModel().setSpeechProcessorLevel(1);
    check(f.droppedContaining(QStringLiteral("transmit set speech_processor_level=1")),
          "receive-only host-DSP radio: the level still raises the notice");
}

// A radio that modulates itself: the host chain is not in the transmit path.
// This stub implements no setSpeechProcessor, so here the notice is true. (A
// backend that does implement it is a different case, not decided here.)
void procKeepsNoticeOnRadioThatModulatesItself()
{
    RadioCapabilities caps = hostModulatingTransmitter();
    caps.hostModulates = false;
    Fixture f(caps);
    f.radio.transmitModel().setSpeechProcessorEnable(true);
    check(f.droppedContaining(QStringLiteral("transmit set speech_processor_enable=1")),
          "radio-modulated transmitter: PROC still raises the notice");
}

// Flex-shaped: a command plane and no host modulation. Byte for byte.
void flexWireTextIsUnchanged()
{
    RadioConnection connection;   // unopened: no socket, no thread
    QStringList written;
    {
        Fixture f(radioModulatedTransmitter());
        f.useCommandPlane(connection, written);
        check(f.radio.hasCommandPlane(), "premise: the Flex-shaped model has a command plane");

        f.radio.transmitModel().setSpeechProcessorEnable(true);
        f.radio.transmitModel().setSpeechProcessorLevel(1);
        f.radio.transmitModel().setSpeechProcessorLevel(2);
        f.radio.transmitModel().setSpeechProcessorEnable(false);
        const QStringList expected{
            QStringLiteral("transmit set speech_processor_enable=1"),
            QStringLiteral("transmit set speech_processor_level=1"),
            QStringLiteral("transmit set speech_processor_level=2"),
            QStringLiteral("transmit set speech_processor_enable=0"),
        };
        check(written == expected,
              QStringLiteral("Flex: the speech processor wire text is unchanged (got: %1)")
                  .arg(written.join(QStringLiteral(" | "))));
        check(f.dropped.isEmpty(), "Flex: nothing is dropped");
    }
}

// The command-plane term is what decides, not the capability: a backend that
// modulates on the host AND has a command plane still gets the text.
void commandPlaneAlwaysGetsTheText()
{
    RadioConnection connection;
    QStringList written;
    {
        Fixture f(hostModulatingTransmitter());
        f.useCommandPlane(connection, written);
        f.radio.transmitModel().setSpeechProcessorEnable(true);
        check(written == QStringList{QStringLiteral("transmit set speech_processor_enable=1")},
              "a command plane takes the text even where the host modulates");
        check(f.dropped.isEmpty(), "...and nothing is dropped");
    }
}

// ── 3. The GUI guard is necessary ────────────────────────────────────────────

void ungatedDisplayTextIsStillDroppedLoudly()
{
    Fixture f(hostModulatingTransmitter());
    const QStringList texts{
        QStringLiteral("display panafall set 0x42000000 color_gain=60"),
        QStringLiteral("display panafall set 0x42000000 black_level=20"),
        QStringLiteral("display pan set 0x40000000 min_dbm=-130.00 max_dbm=-40.00"),
    };
    for (const QString& text : texts) {
        f.dropped.clear();
        f.radio.sendCommand(text);
        check(f.dropped == QStringList{text},
              QStringLiteral("sent ungated, \"%1\" is dropped loudly").arg(text));
    }
}

void flexDisplayTextReachesTheWire()
{
    RadioConnection connection;
    QStringList written;
    {
        Fixture f(radioModulatedTransmitter());
        f.useCommandPlane(connection, written);
        f.radio.sendCommand(QStringLiteral("display panafall set 0x42000000 color_gain=60"));
        check(written == QStringList{QStringLiteral(
                  "display panafall set 0x42000000 color_gain=60")},
              "Flex: the display text reaches the wire as written");
    }
}

// ── 2. The GUI sites, by source text ─────────────────────────────────────────

QString readSource(const QString& relative)
{
    QFile file(QStringLiteral(AETHER_SOURCE_DIR) + QLatin1Char('/') + relative);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return {};
    }
    return QString::fromUtf8(file.readAll());
}

// Drops `//` comments, so a comment that merely mentions hasCommandPlane()
// cannot satisfy a check meant for code. String literals in the regions read
// here contain no `//`.
QString withoutLineComments(const QString& text)
{
    QStringList kept;
    const QStringList lines = text.split(QLatin1Char('\n'));
    for (const QString& line : lines) {
        const int at = line.indexOf(QStringLiteral("//"));
        kept << (at < 0 ? line : line.left(at));
    }
    return kept.join(QLatin1Char('\n'));
}

// The code between two anchors, each of which must occur exactly once.
QString region(const QString& code, const QString& from, const QString& to,
               const QString& label)
{
    const int start = code.indexOf(from);
    const int end = start < 0 ? -1 : code.indexOf(to, start + from.size());
    const bool unique = code.count(from) == 1;
    check(start >= 0 && end > start && unique,
          QStringLiteral("%1: the site is found in the source").arg(label));
    return (start >= 0 && end > start) ? code.mid(start, end - start) : QString();
}

// In `body`: `effect` (the local, client-side action) comes first, then the
// hasCommandPlane() gate, then the send carrying `wireText` exactly.
void requireGatedSend(const QString& body, const QString& label,
                      const QString& effect, const QStringList& wireTexts)
{
    const QString gate = QStringLiteral("m_radioModel.hasCommandPlane()");
    const int effectAt = effect.isEmpty() ? 0 : body.indexOf(effect);
    const int gateAt = body.indexOf(gate);
    const int sendAt = body.indexOf(QStringLiteral("m_radioModel.sendCommand("));
    check(effectAt >= 0,
          QStringLiteral("%1: the local effect is applied").arg(label));
    check(gateAt >= 0 && sendAt > gateAt,
          QStringLiteral("%1: the send is gated on hasCommandPlane()").arg(label));
    check(effectAt >= 0 && gateAt > effectAt,
          QStringLiteral("%1: the local effect is not behind the gate").arg(label));
    for (const QString& wireText : wireTexts) {
        check(sendAt >= 0 && body.indexOf(wireText, sendAt) > sendAt,
              QStringLiteral("%1: the Flex wire text is unchanged (%2)").arg(label, wireText));
    }
}

void panadapterSitesAreGatedInSource()
{
    const QString raw = readSource(QStringLiteral("src/gui/MainWindow_Wiring.cpp"));
    check(!raw.isEmpty(), "MainWindow_Wiring.cpp is readable");
    const QString code = withoutLineComments(raw);

    requireGatedSend(
        region(code, QStringLiteral("&SpectrumOverlayMenu::wfColorGainChanged"),
               QStringLiteral("&SpectrumOverlayMenu::wfBlackLevelChanged"),
               QStringLiteral("WtrFall Gain")),
        QStringLiteral("WtrFall Gain"), QStringLiteral("sw->setWfColorGain(v);"),
        {QStringLiteral("QString(\"display panafall set %1 color_gain=%2\")"
                        ".arg(pan->waterfallId()).arg(v)")});

    // The negative control among the GUI sites. The manual Black Level slider
    // is not a working control on dBm rows (see the header), so its send stays
    // ungated and keeps raising the notice there. Gating it is a change to
    // make together with the renderer, not a tidy-up.
    const QString black = region(code,
                                 QStringLiteral("&SpectrumOverlayMenu::wfBlackLevelChanged"),
                                 QStringLiteral("&SpectrumOverlayMenu::wfAutoBlackChanged"),
                                 QStringLiteral("Black Level"));
    check(black.contains(QStringLiteral("m_radioModel.sendCommand("))
              && !black.contains(QStringLiteral("hasCommandPlane()")),
          "Black Level: the send is NOT gated, so a slider that does nothing stays loud");
    check(black.contains(QStringLiteral("QString(\"display panafall set %1 black_level=%2\")"
                                        ".arg(pan->waterfallId()).arg(v)")),
          "Black Level: the Flex wire text is unchanged");

    requireGatedSend(
        region(code, QStringLiteral("dst->setWfColorGain(src->wfColorGain());"),
               QStringLiteral("dst->setWfLineDuration("),
               QStringLiteral("Clone to all Pans")),
        QStringLiteral("Clone to all Pans"),
        QStringLiteral("dst->setWfBlackLevel(src->wfBlackLevel());"),
        {QStringLiteral("QString(\"display panafall set %1 color_gain=%2\")"),
         QStringLiteral("QString(\"display panafall set %1 black_level=%2\")")});

    // The reset applies both values on the widget well above its radio
    // commands, with an unrelated send (weighted_average) in between, so the
    // effect and the gated send are read from two regions.
    const QString reset = region(code, QStringLiteral("sw->setWfColorGain(50);"),
                                 QStringLiteral("auto& s = AppSettings::instance();"),
                                 QStringLiteral("Reset display defaults"));
    check(reset.contains(QStringLiteral("sw->setWfBlackLevel(15);")),
          "Reset display defaults: the local effect is applied");
    requireGatedSend(
        reset.mid(qMax(0, reset.indexOf(QStringLiteral("requestPanDisplayRates(")))),
        QStringLiteral("Reset display defaults"), QString(),
        {QStringLiteral("QString(\"display panafall set %1 color_gain=50\")"
                        ".arg(pan->waterfallId())"),
         QStringLiteral("QString(\"display panafall set %1 black_level=15\")"
                        ".arg(pan->waterfallId())")});

    // The dBm scale has one outlet for every caller (drag, arrows, headroom
    // recovery); the local scale is moved by SpectrumWidget before any of them
    // gets here, so there is no effect to order against inside this lambda.
    const QString dbm = region(code, QStringLiteral("auto sendDbmRangeCommand ="),
                               QStringLiteral("sw->disconnect(this);"),
                               QStringLiteral("dBm scale"));
    requireGatedSend(dbm, QStringLiteral("dBm scale"), QString(),
                     {QStringLiteral("QString(\"display pan set %1 min_dbm=%2 max_dbm=%3\")"),
                      QStringLiteral(".arg(static_cast<double>(minDbm), 0, 'f', 2)"),
                      QStringLiteral(".arg(static_cast<double>(maxDbm), 0, 'f', 2)")});
    check(dbm.contains(QStringLiteral("backendCapabilities().radioOwnsDbmScale")),
          "dBm scale: the radioOwnsDbmScale backstop is still there");
}

void procBindingIsWrittenInSource()
{
    const QString raw = readSource(QStringLiteral("src/gui/MainWindow_DspApplets.cpp"));
    check(!raw.isEmpty(), "MainWindow_DspApplets.cpp is readable");
    const QString code = withoutLineComments(raw);

    const QString predicate = region(
        code, QStringLiteral("bool MainWindow::hostModulatesTxAudio() const"),
        QStringLiteral("void MainWindow::applySpeechProcessorToClientComp("),
        QStringLiteral("PROC predicate"));
    check(predicate.contains(QStringLiteral(
              "return hostRunsTxVoiceChain(caps.hostModulates, caps.canTransmit);")),
          "PROC: MainWindow asks the predicate RadioModel asks");

    const QString applier = region(
        code, QStringLiteral("void MainWindow::applySpeechProcessorToClientComp("),
        QStringLiteral("m_lastAppliedProcEnable = on;"),
        QStringLiteral("PROC applier"));
    check(applier.contains(QStringLiteral("!hostModulatesTxAudio()")),
          "PROC: the applier acts exactly where that predicate holds");
    check(applier.contains(QStringLiteral("comp->setEnabled(on);"))
              && applier.contains(QStringLiteral("comp->setThresholdDb(p.thresholdDb);")),
          "PROC: the applier drives the TX compressor's enable and preset");

    const QString binding = region(
        code, QStringLiteral("&TransmitModel::speechProcessorCommandIssued, this,"),
        QStringLiteral("&TransmitModel::micStateChanged"),
        QStringLiteral("PROC binding"));
    check(binding.contains(QStringLiteral("applySpeechProcessorToClientComp(true);")),
          "PROC: the operator intent is bound to the applier");
}
} // namespace

int main(int argc, char** argv)
{
    TestSettingsProfile profile(QStringLiteral("no-false-unsupported-notice"));
    if (!profile.isValid()) {
        return 1;
    }
    QCoreApplication app(argc, argv);

    procActsWithoutNoticeOnHostModulatingRadio();
    theNoticeStillFiresForWhatDoesNothing();
    procKeepsNoticeOnReceiveOnlyRadio();
    procKeepsNoticeOnRadioThatModulatesItself();
    flexWireTextIsUnchanged();
    commandPlaneAlwaysGetsTheText();
    ungatedDisplayTextIsStillDroppedLoudly();
    flexDisplayTextReachesTheWire();
    panadapterSitesAreGatedInSource();
    procBindingIsWrittenInSource();

    std::printf("%d failure(s)\n", failures);
    return failures == 0 ? 0 : 1;
}
