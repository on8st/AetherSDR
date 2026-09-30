// Two-tone TUNE on the Hermes-Lite 2, measured OFFLINE on the EP2 bytes the
// packet builder would put on the wire. No socket, no radio, no transmission.
//
//   A. MetisClient::setTxTwoTone: the decoded IQ carries two equal tones at
//      exactly the requested offsets, on the side of the carrier their signs
//      name, with nothing on the mirror side; the envelope PEAKS at the
//      requested level (PEP = single-carrier TUNE) and averages half of it;
//      a clear or a fenced operation leaves silence, and a single tone after a
//      two-tone carries no leftover second tone.
//   B. Hl2Backend: declares twoToneGenerator (so `txtest twotone` passes the
//      AutomationServer capability check), echoes the selected tune mode, and
//      setTune() raises +700/+1900 Hz on USB, -700/-1900 on LSB, a single
//      carrier again once Mono Tone is selected -- and nothing at all while
//      transmit is not allowed.
//   C. RadioModel: startTwoToneTune() selects the waveform on the seam BEFORE
//      the TUNE key is dispatched.

#include "TestSettingsProfile.h"
#include "TxTestAuthority.h"
#include "core/backends/hl2/Hl2Backend.h"
#include "core/backends/hl2/MetisClient.h"
#include "core/backends/hl2/MetisProtocol.h"
#include "models/RadioModel.h"
#include "models/SliceModel.h"
#include "models/TransmitModel.h"

#include <QCoreApplication>

#include <array>
#include <cmath>
#include <complex>
#include <cstdio>
#include <memory>
#include <vector>

namespace AetherSDR::hl2 {
struct Hl2HostTxTestAccess {
    static void start(Hl2Backend& backend, const QString& txMode)
    {
        backend.m_rx.resize(1);
        backend.m_rx[0].mode = txMode;
        backend.m_txDdc = 0;
        backend.m_txAllowed = true;
        QMetaObject::invokeMethod(backend.m_metis, [metis = backend.m_metis] {
            metis->enableTransmit(true);
        }, Qt::BlockingQueuedConnection);
    }
    static void setMode(Hl2Backend& backend, const QString& txMode) { backend.m_rx[0].mode = txMode; }
    static void setTxAllowed(Hl2Backend& backend, bool allowed) { backend.m_txAllowed = allowed; }
    static std::array<std::uint8_t, kUsbPacketSize> packet(Hl2Backend& backend)
    {
        std::array<std::uint8_t, kUsbPacketSize> p{};
        QMetaObject::invokeMethod(backend.m_metis, [&p, metis = backend.m_metis] {
            p = metis->buildNextControlPacket();
        }, Qt::BlockingQueuedConnection);
        return p;
    }
    static void finish(Hl2Backend& backend) { backend.m_rx.clear(); }
};
} // namespace AetherSDR::hl2

using namespace AetherSDR;
using namespace AetherSDR::hl2;

namespace {
int failures = 0;
void check(bool ok, const char* what)
{
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    failures += ok ? 0 : 1;
}

constexpr double kEp2Rate = 48000.0;
constexpr int kSamplesPerFrame = 63;

// The wire layout MetisProtocol.h documents at ep2WriteTxIq: per frame, 8
// bytes of SYNC + C&C, then 63 slots of [4 audio/EADDR][I be16][Q be16]. The
// wire is the CONJUGATE of the analytic signal, so undo that here: a positive
// offset must decode as a positive frequency.
void appendIq(const std::array<std::uint8_t, kUsbPacketSize>& pkt, std::vector<std::complex<double>>& out)
{
    for (std::size_t frame = 0; frame < 2; ++frame) {
        const std::size_t base = 8 + frame * kFrameSize + 8;
        for (int s = 0; s < kSamplesPerFrame; ++s) {
            const std::uint8_t* p = pkt.data() + base + static_cast<std::size_t>(s) * 8;
            const auto i = static_cast<std::int16_t>((p[4] << 8) | p[5]);
            const auto q = static_cast<std::int16_t>((p[6] << 8) | p[7]);
            out.emplace_back(i / 32767.0, -q / 32767.0);
        }
    }
}

// Amplitude of the component at `hz` over a whole number of its periods.
double amplitudeAt(const std::vector<std::complex<double>>& z, double hz, std::size_t n)
{
    std::complex<double> acc{};
    for (std::size_t k = 0; k < n; ++k) {
        acc += z[k] * std::polar(1.0, -2.0 * M_PI * hz * static_cast<double>(k) / kEp2Rate);
    }
    return std::abs(acc) / static_cast<double>(n);
}

double peakOf(const std::vector<std::complex<double>>& z, std::size_t n)
{
    double m = 0.0;
    for (std::size_t k = 0; k < n; ++k) {
        m = std::max(m, std::abs(z[k]));
    }
    return m;
}

double meanPower(const std::vector<std::complex<double>>& z, std::size_t n)
{
    double p = 0.0;
    for (std::size_t k = 0; k < n; ++k) {
        p += std::norm(z[k]);
    }
    return p / static_cast<double>(n);
}

bool payloadSilent(const std::array<std::uint8_t, kUsbPacketSize>& pkt)
{
    std::vector<std::complex<double>> z;
    appendIq(pkt, z);
    return peakOf(z, z.size()) == 0.0;
}

// 700 and 1900 share a 100 Hz period: 480 samples at 48 kHz. 4800 samples is
// ten of them, so both tones and every IMD product sit on exact bins.
constexpr std::size_t kWindow = 4800;

std::vector<std::complex<double>> capture(MetisClient& c, std::size_t samples)
{
    std::vector<std::complex<double>> z;
    while (z.size() < samples) {
        appendIq(c.buildNextControlPacket(), z);
    }
    return z;
}

void metisTwoTone()
{
    TxTestAuthority tx;
    MetisClient c;
    c.enableTransmit(true);
    c.setMox(true, tx.operation);
    c.setTxTwoTone(700.0, 1900.0, 1.0, tx.operation);
    check(c.txTwoToneEnabled(), "two-tone armed");
    const auto z = capture(c, kWindow);
    const double a700 = amplitudeAt(z, 700.0, kWindow);
    const double a1900 = amplitudeAt(z, 1900.0, kWindow);
    std::printf("    +700 %.4f  +1900 %.4f  -700 %.2e  -1900 %.2e  +500/3100 IMD %.2e/%.2e\n",
                a700, a1900, amplitudeAt(z, -700.0, kWindow), amplitudeAt(z, -1900.0, kWindow),
                amplitudeAt(z, -500.0, kWindow), amplitudeAt(z, 3100.0, kWindow));
    check(std::fabs(a700 - 0.5) < 1e-3 && std::fabs(a1900 - 0.5) < 1e-3,
          "two tones of amplitude 0.5 at +700 and +1900 Hz");
    check(std::fabs(20.0 * std::log10(a700 / a1900)) < 0.01, "equal amplitude within 0.01 dB");
    check(amplitudeAt(z, -700.0, kWindow) < 1e-4 && amplitudeAt(z, -1900.0, kWindow) < 1e-4,
          "nothing on the mirror side (handedness matches the voice path)");
    check(amplitudeAt(z, 0.0, kWindow) < 1e-4, "no carrier leak at 0 Hz");
    check(amplitudeAt(z, -500.0, kWindow) < 1e-4 && amplitudeAt(z, 3100.0, kWindow) < 1e-4,
          "no third-order product generated in the host (quantisation only)");
    const double pep = peakOf(z, kWindow);
    check(pep > 0.999 && pep <= 1.0 + 1e-4, "envelope peaks at the requested PEP (1.0 = TUNE carrier)");
    check(std::fabs(10.0 * std::log10(meanPower(z, kWindow)) - 10.0 * std::log10(0.5)) < 0.01,
          "average power is half the PEP (-3.01 dB)");

    c.setTxTwoTone(-700.0, -1900.0, 0.5, tx.operation);
    const auto lsb = capture(c, kWindow);
    check(std::fabs(amplitudeAt(lsb, -700.0, kWindow) - 0.25) < 1e-3
              && std::fabs(amplitudeAt(lsb, -1900.0, kWindow) - 0.25) < 1e-3
              && amplitudeAt(lsb, 700.0, kWindow) < 1e-4,
          "negative offsets land below the carrier; PEP 0.5 gives 0.25 per tone");
    check(std::fabs(peakOf(lsb, kWindow) - 0.5) < 1e-3, "PEP follows the requested peak");

    c.setTxTestTone(0.0, 1.0, tx.operation);
    const auto single = capture(c, kWindow);
    check(!c.txTwoToneEnabled() && std::fabs(amplitudeAt(single, 0.0, kWindow) - 1.0) < 1e-3
              && amplitudeAt(single, 700.0, kWindow) < 1e-4 && amplitudeAt(single, 1900.0, kWindow) < 1e-4,
          "a single tone after a two-tone carries no leftover second tone");

    c.setTxTwoTone(700.0, 1900.0, 1.0, tx.operation);
    c.setTxTestTone(0.0, 0.0, tx.operation);
    check(payloadSilent(c.buildNextControlPacket()), "the TUNE clear silences a two-tone");

    c.setTxTwoTone(700.0, 1900.0, std::nan(""), tx.operation);
    check(payloadSilent(c.buildNextControlPacket()), "a NaN peak is refused, not transmitted");

    c.setTxTwoTone(700.0, 1900.0, 1.0, tx.operation);
    (void)tx.coordinator.cancel(tx.actor, tx.operation);
    check(payloadSilent(c.buildNextControlPacket()),
          "a cancelled operation fences the two-tone at the packet builder");
}

void backendTwoTone()
{
    TxTestAuthority authority;
    Hl2Backend b;
    check(b.capabilities().twoToneGenerator.has_value(),
          "HL2 declares twoToneGenerator, so `txtest twotone` is no longer refused");
    Hl2HostTxTestAccess::start(b, QStringLiteral("USB"));
    QString echoed;
    QObject::connect(&b, &IRadioBackend::transmitChanged, &b, [&](const TransmitDelta& d) {
        if (d.tuneMode) {
            echoed = *d.tuneMode;
        }
    });
    b.setTuneTwoTone(true);
    check(echoed == QStringLiteral("two_tone"), "the selection is echoed for the right-click check marks");

    const auto measure = [&](double hz) {
        std::vector<std::complex<double>> z;
        while (z.size() < kWindow) {
            appendIq(Hl2HostTxTestAccess::packet(b), z);
        }
        return amplitudeAt(z, hz, kWindow);
    };

    b.setTune(true, 10, authority.operation, {});
    QCoreApplication::processEvents();
    check(measure(700.0) > 0.49 && measure(1900.0) > 0.49 && measure(0.0) < 1e-3,
          "USB: TUNE in two-tone mode raises +700 and +1900 Hz, no carrier");
    b.setTune(false, 10, authority.operation, {});
    QCoreApplication::processEvents();
    check(payloadSilent(Hl2HostTxTestAccess::packet(b)), "TUNE released: silence");

    Hl2HostTxTestAccess::setMode(b, QStringLiteral("LSB"));
    b.setTune(true, 10, authority.operation, {});
    QCoreApplication::processEvents();
    check(measure(-700.0) > 0.49 && measure(-1900.0) > 0.49 && measure(700.0) < 1e-3,
          "LSB: the tones sit below the carrier");
    b.setTune(false, 10, authority.operation, {});
    QCoreApplication::processEvents();

    b.setTuneTwoTone(false);
    check(echoed == QStringLiteral("single_tone"), "Mono Tone echoed");
    b.setTune(true, 10, authority.operation, {});
    QCoreApplication::processEvents();
    check(measure(0.0) > 0.99 && measure(-700.0) < 1e-3, "Mono Tone: the single TUNE carrier again");
    b.setTune(false, 10, authority.operation, {});
    QCoreApplication::processEvents();

    b.setTuneTwoTone(true);
    Hl2HostTxTestAccess::setTxAllowed(b, false);
    b.setTune(true, 10, authority.operation, {});
    QCoreApplication::processEvents();
    check(payloadSilent(Hl2HostTxTestAccess::packet(b)),
          "transmit not allowed: a two-tone TUNE generates nothing");
    b.setTune(false, 10, authority.operation, {});
    QCoreApplication::processEvents();
    Hl2HostTxTestAccess::finish(b);
}

class RecordingBackend final : public IRadioBackend {
public:
    RadioCapabilities caps;
    QStringList calls;
    RadioCapabilities capabilities() const override { return caps; }
    // FALSE, as atu_seam_gate_test's: the socket-free slice fixture installs
    // only on a disconnected model, and the TX path under test does not ask.
    bool isConnected() const override { return false; }
    void connectRadio(const RadioConnectRequest&) override {}
    void disconnectRadio() override {}
    void setSliceFrequency(int, double) override {}
    void setSliceMode(int, const QString&) override {}
    void setSliceFilter(int, int, int) override {}
    void setSliceAgc(int, const QString&, int) override {}
    void setPanCenter(const QString&, double, PanCenterIntent) override {}
    void setKeying(bool, const TxCoordinator::Operation&, const TxCoordinator::Completion& done) override { done.finish(); }
    void setTuneTwoTone(bool twoTone) override { calls << (twoTone ? QStringLiteral("two") : QStringLiteral("mono")); }
    void setTune(bool on, int, const TxCoordinator::Operation&, const TxCoordinator::Completion& done) override
    {
        calls << (on ? QStringLiteral("tune-on") : QStringLiteral("tune-off"));
        done.finish();
    }
    void invokeExtension(const QString&, const QString&, quint64, const QVariant&) override {}
};

void modelOrder()
{
    RadioModel radio;
    auto owned = std::make_unique<RecordingBackend>();
    RecordingBackend* backend = owned.get();
    backend->caps.family = QStringLiteral("hl2");
    backend->caps.canTransmit = true;
    backend->caps.twoToneGenerator = RadioCapabilities::TwoToneGenerator{QStringLiteral("test")};
    radio.setBackendForTest(std::move(owned), QStringLiteral("hl2"));
    if (!radio.automationApplySliceFixture(0, QStringLiteral("A"))) {
        check(false, "slice fixture installed");
        return;
    }
    SliceDelta delta;
    delta.txSlice = true;
    delta.panId = QStringLiteral("0x40000000");
    delta.mode = QStringLiteral("USB");
    radio.slice(0)->applyChanges(delta);

    radio.transmitModel().startTwoToneTune();
    const qsizetype two = backend->calls.indexOf(QStringLiteral("two"));
    const qsizetype on = backend->calls.indexOf(QStringLiteral("tune-on"));
    check(two >= 0 && on > two, "two-tone is selected on the seam BEFORE TUNE keys");
    radio.transmitModel().toggleTwoToneTune();
    check(backend->calls.contains(QStringLiteral("tune-off"))
              && backend->calls.last() == QStringLiteral("mono"),
          "the two-tone shortcut's stop reverts the seam to Mono Tone");
}
} // namespace

int main(int argc, char** argv)
{
    TestSettingsProfile profile(QStringLiteral("hl2-two-tone"));
    if (!profile.isValid()) {
        return 1;
    }
    QCoreApplication app(argc, argv);
    metisTwoTone();
    backendTwoTone();
    modelOrder();
    std::printf("%d failure(s)\n", failures);
    return failures == 0 ? 0 : 1;
}
