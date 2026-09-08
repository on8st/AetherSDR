// The arm / disarm / fire path of the signal-handler emergency stop.
// (aethersdr/AetherSDR#4581)
//
// WHAT THIS PROVES, exactly: that a target published by armEmergencyStop() is
// the one fireEmergencyStop() sends to, that a RE-ARM moves the descriptor, the
// address and the packet together, and that a disarm — explicit, or by arming
// with an fd or an address that cannot be used — silences it. It is a guard on
// the data structure, which #4581's fix rewrites: the payload now lives in two
// alternating slots published by one atomic pointer, so getting the slot
// bookkeeping wrong would send a PREVIOUS session's stop to a PREVIOUS session's
// address, and that mistake is what these assertions catch.
//
// WHAT IT DOES NOT PROVE, said plainly because the issue is about a race:
//
//   - It does not exercise the data race. The race needs a signal delivered to
//     another thread inside a re-arm; nothing here is concurrent, and every
//     assertion below passes on the UNFIXED tree too. The argument for the fix
//     is the one written in Hl2EmergencyStop.cpp, not this file.
//   - It does not exercise the Win64 SOCKET narrowing. That needs a descriptor
//     above INT_MAX, which this platform does not hand out.
//   - No signal is raised, so the handler path itself is not run. The fixture
//     that did that (hl2_signal_stop_test) is retired; see tests.cmake.
//
// Its positive control is an off-by-one in the published slot index, which turns
// "a re-arm moves the whole target" red.

#include "core/backends/hl2/Hl2EmergencyStop.h"

#include <QCoreApplication>
#include <QHostAddress>
#include <QNetworkDatagram>
#include <QUdpSocket>

#include <array>
#include <cstdio>
#include <vector>

using namespace AetherSDR::hl2;

static int g_failures = 0;
static void check(bool ok, const char* what)
{
    if (!ok) { std::fprintf(stderr, "FAIL: %s\n", what); ++g_failures; }
    else     { std::fprintf(stderr, "[ OK ] %s\n", what); }
}

static std::array<std::uint8_t, 64> markedPacket(std::uint8_t marker)
{
    std::array<std::uint8_t, 64> p{};
    p.fill(marker);
    return p;
}

// Collect whatever arrived, waiting only as long as it takes. Loopback UDP is
// not guaranteed, so the count is asserted as "at least one of the three
// repeats", never as exactly three — the repeats exist because the datagram may
// be lost, and a test that demanded all three would be asserting the opposite.
static std::vector<QByteArray> drain(QUdpSocket& sock, int budgetMs)
{
    std::vector<QByteArray> out;
    while (sock.waitForReadyRead(budgetMs)) {
        while (sock.hasPendingDatagrams())
            out.push_back(sock.receiveDatagram().data());
        budgetMs = 20;   // the first wait pays the latency; the rest are drains
    }
    return out;
}

static bool allAre(const std::vector<QByteArray>& got, std::uint8_t marker)
{
    if (got.empty())
        return false;
    for (const QByteArray& d : got) {
        if (d.size() != 64)
            return false;
        for (char c : d) {
            if (static_cast<std::uint8_t>(c) != marker)
                return false;
        }
    }
    return true;
}

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);

    QUdpSocket first;
    QUdpSocket second;
    check(first.bind(QHostAddress::LocalHost, 0), "first destination binds");
    check(second.bind(QHostAddress::LocalHost, 0), "second destination binds");
    const quint16 firstPort = first.localPort();
    const quint16 secondPort = second.localPort();

    QUdpSocket sender;
    check(sender.bind(QHostAddress::LocalHost, 0), "sending socket binds");
    const qintptr fd = sender.socketDescriptor();
    check(fd >= 0, "sending socket has a descriptor to arm with");

    // ---- armed target is the one that receives ----
    armEmergencyStop(fd, QHostAddress::LocalHost, firstPort, markedPacket(0xA1));
    fireEmergencyStop();
    check(allAre(drain(first, 500), 0xA1), "the armed target receives the stop datagram");
    check(drain(second, 20).empty(), "nothing reaches an address that was never armed");

    // ---- a re-arm moves the WHOLE target, not part of it ----
    //
    // Different port AND different payload, so a slot published while another
    // slot was filled shows up as either the old address or the old bytes.
    armEmergencyStop(fd, QHostAddress::LocalHost, secondPort, markedPacket(0xB2));
    fireEmergencyStop();
    check(allAre(drain(second, 500), 0xB2), "a re-arm moves the address and the packet together");
    check(drain(first, 20).empty(), "the previous target stops receiving");

    // ---- a third arm reuses the first slot ----
    armEmergencyStop(fd, QHostAddress::LocalHost, firstPort, markedPacket(0xC3));
    fireEmergencyStop();
    check(allAre(drain(first, 500), 0xC3), "the slots alternate without carrying stale bytes");
    check(drain(second, 20).empty(), "the second target stops receiving");

    // ---- disarm silences it ----
    disarmEmergencyStop();
    fireEmergencyStop();
    check(drain(first, 100).empty() && drain(second, 20).empty(),
          "a disarmed stop sends nothing");

    // ---- arming with an unusable fd disarms rather than leaving the old one ----
    armEmergencyStop(fd, QHostAddress::LocalHost, firstPort, markedPacket(0xD4));
    armEmergencyStop(-1, QHostAddress::LocalHost, firstPort, markedPacket(0xD4));
    fireEmergencyStop();
    check(drain(first, 100).empty(), "an invalid descriptor disarms instead of leaving the last target");

    // ---- and so does a non-IPv4 destination: Metis is IPv4-only ----
    armEmergencyStop(fd, QHostAddress::LocalHost, firstPort, markedPacket(0xE5));
    armEmergencyStop(fd, QHostAddress(QStringLiteral("::1")), firstPort, markedPacket(0xE5));
    fireEmergencyStop();
    check(drain(first, 100).empty(), "a non-IPv4 destination disarms");

    disarmEmergencyStop();

    if (g_failures == 0)
        std::fprintf(stderr, "hl2_emergency_stop_test: all checks passed\n");
    return g_failures == 0 ? 0 : 1;
}
