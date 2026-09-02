// What does the envelope look like at the EP2 wire boundary?
//
// hl2-lab's tx-dynamics stream has measured every stage of the transmit chain
// and found each one healthy -- microphone, capture path, and Hl2TxDsp -- while
// the transmitted envelope stays flat at peak/mean 1.09-1.14. Each of those
// measurements was a RATIO. None of them measured absolute LEVEL.
//
// That distinction is the reason this exists. ep2WriteTxIq() hard-clamps every
// sample to +-1.0 before packing it:
//
//     if (v >  1.0f) v =  1.0f;
//     if (v < -1.0f) v = -1.0f;
//
// So a modulator whose output preserves dynamics perfectly, but whose absolute
// amplitude sits well above full scale, produces a flat envelope ON THE WIRE
// while every ratio measured upstream of the clamp looks correct. That is
// consistent with every observation this stream has: the mic is fine, the
// capture path is fine, Hl2TxDsp "preserves dynamics", and the envelope is
// still flat when it reaches the PA.
//
// This probe drives the real Hl2TxDsp with real speech and reports what nobody
// has looked at: how far above or below full scale its IQ actually sits, how
// much of it the wire clamp removes, and what the envelope's peak/mean is on
// each side of that clamp.
//
// Offline. No radio, no simulator, no network, no keying.
//
// Usage:
//   hl2_wire_envelope_probe <mono-24k-16bit.wav> [--committed] [--ep2 out.bin]
//
//   --committed  use the pre-#5198 ALC settings (target 0.85, makeup 40 dB)
//                instead of whatever the working tree currently has.
//   --ep2 FILE   write the packed EP2 payloads so tools/ep2_envelope.py can
//                read the same bytes independently.

#include "core/backends/hl2/Hl2TxDsp.h"
#include "core/backends/hl2/MetisProtocol.h"

#include <QCoreApplication>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using AetherSDR::hl2::Hl2TxDsp;
namespace hl2 = AetherSDR::hl2;

namespace {

bool readWav(const char* path, std::vector<float>& mono, int& rate)
{
    FILE* f = std::fopen(path, "rb");
    if (!f) { std::fprintf(stderr, "cannot open %s\n", path); return false; }
    std::vector<uint8_t> all;
    uint8_t buf[65536];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0)
        all.insert(all.end(), buf, buf + n);
    std::fclose(f);
    if (all.size() < 44 || std::memcmp(all.data(), "RIFF", 4) != 0) return false;
    int channels = 1, bits = 16;
    size_t pos = 12, dataOff = 0, dataLen = 0;
    while (pos + 8 <= all.size()) {
        const char* id = reinterpret_cast<const char*>(all.data() + pos);
        uint32_t sz; std::memcpy(&sz, all.data() + pos + 4, 4);
        if (std::memcmp(id, "fmt ", 4) == 0 && pos + 24 <= all.size()) {
            uint16_t ch, bps; uint32_t sr;
            std::memcpy(&ch,  all.data() + pos + 10, 2);
            std::memcpy(&sr,  all.data() + pos + 12, 4);
            std::memcpy(&bps, all.data() + pos + 22, 2);
            channels = ch; rate = int(sr); bits = bps;
        } else if (std::memcmp(id, "data", 4) == 0) {
            dataOff = pos + 8; dataLen = std::min<size_t>(sz, all.size() - dataOff);
        }
        pos += 8 + sz + (sz & 1);
    }
    if (!dataOff || bits != 16) return false;
    const auto* s = reinterpret_cast<const int16_t*>(all.data() + dataOff);
    const size_t frames = (dataLen / 2) / size_t(channels);
    mono.resize(frames);
    for (size_t i = 0; i < frames; ++i)
        mono[i] = s[i * size_t(channels)] / 32768.0f;
    return true;
}

double ratio(const std::vector<double>& v)
{
    if (v.empty()) return 0.0;
    double sum = 0, peak = 0;
    for (double t : v) { sum += t; peak = std::max(peak, t); }
    const double mean = sum / double(v.size());
    return mean > 0 ? peak / mean : 0.0;
}

// 100 ms RMS frames on the envelope -- the window that compares to the 10 Hz
// forward-power telemetry, stated because peak/mean without one is not a number.
double envRatio(const std::vector<double>& env, int rate)
{
    const int n = rate / 10;
    if (n <= 0 || int(env.size()) < n * 2) return 0.0;
    std::vector<double> frames;
    for (size_t i = 0; i + n <= env.size(); i += n) {
        double acc = 0;
        for (int j = 0; j < n; ++j) acc += env[i + j] * env[i + j];
        frames.push_back(std::sqrt(acc / n));
    }
    return ratio(frames);
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <mono-24k-16bit.wav> [--committed] "
                             "[--ep2 out.bin]\n", argv[0]);
        return 2;
    }
    bool committed = false;
    const char* ep2Path = nullptr;
    for (int i = 2; i < argc; ++i) {
        if (std::string(argv[i]) == "--committed") committed = true;
        else if (std::string(argv[i]) == "--ep2" && i + 1 < argc) ep2Path = argv[++i];
    }

    std::vector<float> audio;
    int rate = 24000;
    if (!readWav(argv[1], audio, rate)) { std::fprintf(stderr, "bad wav\n"); return 2; }

    Hl2TxDsp dsp;
    Hl2TxDsp::Config cfg;
    cfg.inputSampleRateHz = rate;
    if (committed) { cfg.alcTargetPeak = 0.85; cfg.alcMaxGainDb = 40.0; }
    std::string err;
    if (!dsp.configure(cfg, &err)) {
        std::fprintf(stderr, "configure failed: %s\n", err.c_str());
        return 2;
    }

    std::vector<std::complex<float>> iq;
    QObject::connect(&dsp, &Hl2TxDsp::iqReady,
                     [&iq](const std::vector<std::complex<float>>& block) {
                         iq.insert(iq.end(), block.begin(), block.end());
                     });

    const int block = cfg.dspBlockSize;
    for (size_t i = 0; i + block <= audio.size(); i += block) {
        std::vector<float> mono(audio.begin() + i, audio.begin() + i + block);
        dsp.processAudioBlock(mono, /*clientLeveled=*/false);
    }
    if (iq.empty()) { std::fprintf(stderr, "modulator produced nothing\n"); return 2; }

    // ---- what the wire clamp does to it -------------------------------
    double peakAbs = 0;
    size_t overI = 0, overQ = 0;
    std::vector<double> envBefore, envAfter;
    envBefore.reserve(iq.size());
    envAfter.reserve(iq.size());
    const auto clamp1 = [](float v) { return v > 1.0f ? 1.0f : (v < -1.0f ? -1.0f : v); };
    for (const auto& z : iq) {
        peakAbs = std::max(peakAbs, double(std::abs(z)));
        if (std::fabs(z.real()) > 1.0f) ++overI;
        if (std::fabs(z.imag()) > 1.0f) ++overQ;
        envBefore.push_back(std::abs(z));
        envAfter.push_back(std::abs(std::complex<float>(clamp1(z.real()),
                                                        clamp1(z.imag()))));
    }
    const size_t clamped = std::max(overI, overQ);

    std::printf("\n  ALC config: target %.2f, makeup %.1f dB  (%s)\n",
                cfg.alcTargetPeak, cfg.alcMaxGainDb,
                committed ? "pre-#5198 committed values"
                          : "working tree, i.e. the #5198 patch");
    std::printf("  %zu audio samples @ %d Hz -> %zu IQ samples @ %d Hz\n\n",
                audio.size(), rate, iq.size(), cfg.outputSampleRateHz);

    std::printf("  modulator output peak |z|      %8.3f  of full scale\n", peakAbs);
    std::printf("  samples the wire clamp alters  %8.2f%%  (I %.2f%%, Q %.2f%%)\n",
                100.0 * double(clamped) / double(iq.size()),
                100.0 * double(overI) / double(iq.size()),
                100.0 * double(overQ) / double(iq.size()));
    std::printf("\n  envelope peak/mean (100 ms frames)\n");
    std::printf("    before the wire clamp        %8.2f\n",
                envRatio(envBefore, cfg.outputSampleRateHz));
    std::printf("    after  the wire clamp        %8.2f\n",
                envRatio(envAfter, cfg.outputSampleRateHz));

    if (ep2Path) {
        FILE* f = std::fopen(ep2Path, "wb");
        if (f) {
            size_t written = 0;
            for (size_t i = 0; i < iq.size(); i += hl2::kTxSamplesPerPacket) {
                auto pkt = hl2::ep2Packet(uint32_t(i / hl2::kTxSamplesPerPacket),
                                          {0x01, 0, 0, 0, 0}, {0x01, 0, 0, 0, 0});
                const size_t n = std::min<size_t>(hl2::kTxSamplesPerPacket,
                                                  iq.size() - i);
                hl2::ep2WriteTxIq(pkt, std::span<const std::complex<float>>(
                                           iq.data() + i, n));
                std::fwrite(pkt.data(), 1, pkt.size(), f);
                ++written;
            }
            std::fclose(f);
            std::printf("\n  wrote %zu EP2 packets to %s\n", written, ep2Path);
            std::printf("  cross-check: ep2_envelope.py --raw %s\n", ep2Path);
        }
    }

    std::printf("\n  The transmitted envelope was measured at 1.09-1.14.\n");
    const double after = envRatio(envAfter, cfg.outputSampleRateHz);
    if (peakAbs > 1.0 && after < 1.35)
        std::printf("  THE WIRE CLAMP IS THE FLATTENER. The modulator overdrives\n"
                    "  full scale and ep2WriteTxIq clips it into a flat envelope.\n\n");
    else if (peakAbs > 1.0)
        std::printf("  The modulator overdrives full scale and the clamp is\n"
                    "  removing signal, but not enough to explain the symptom.\n\n");
    else
        std::printf("  The modulator stays inside full scale; the wire clamp is\n"
                    "  not touching it. Not the flattener.\n\n");
    return 0;
}
