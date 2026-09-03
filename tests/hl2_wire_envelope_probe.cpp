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

    // --tone measures the ANALYTIC SENSE of the modulator's output instead of
    // its envelope: which side of DC a single audio tone lands on. That turns a
    // derivation about filter kernel signs into a measurement, which is what it
    // has to be before anyone acts on it.
    double toneHz = 0.0;
    const char* wavOut = nullptr;
    bool noAlc = false;
    // --check turns this from a diagnostic that prints and exits 0 into a test
    // with a verdict. The criteria are the PROPERTIES FACTS established, not
    // the exact figures it recorded: those were measured on a different
    // stimulus, so asserting them against this fixture would pin a coincidence.
    bool check = false;
    for (int i = 2; i < argc; ++i) {
        const std::string a2(argv[i]);
        if (a2 == "--tone" && i + 1 < argc) toneHz = std::atof(argv[++i]);
        // --wav-out writes Re(IQ): for USB the modulator emits z = bi - j*bq,
        // so the real part IS the audio after Hl2TxDsp's own 255-tap
        // Blackman-windowed bandpass. That is the point -- it is the KERNEL
        // itself, not a reimplementation of its nominal corners, and two
        // implementations of "300-2700 Hz" have already been measured 7% apart.
        //
        // Written at the modulator's 48 kHz output rate deliberately. Resampling
        // to 24 kHz would put ANOTHER filter in the path and contaminate exactly
        // the property this file exists to carry.
        else if (a2 == "--wav-out" && i + 1 < argc) wavOut = argv[++i];
        // --no-alc isolates the FILTER. The ALC is a dynamics processor, so
        // leaving it in measures filter+ALC and cannot answer "what does the
        // filter alone do to the crest".
        else if (a2 == "--no-alc") noAlc = true;
        else if (a2 == "--check") check = true;
    }

    std::vector<float> audio;
    int rate = 24000;
    if (toneHz > 0.0) {
        rate = 24000;
        audio.resize(size_t(rate * 3));
        for (size_t n = 0; n < audio.size(); ++n)
            audio[n] = 0.3f * float(std::sin(2.0 * M_PI * toneHz * double(n) / rate));
    } else if (!readWav(argv[1], audio, rate)) {
        std::fprintf(stderr, "bad wav\n"); return 2;
    }

    Hl2TxDsp dsp;
    Hl2TxDsp::Config cfg;
    cfg.inputSampleRateHz = rate;
    if (committed) { cfg.alcTargetPeak = 0.85; cfg.alcMaxGainDb = 40.0; }
    if (noAlc) cfg.alcEnabled = false;
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

    if (toneHz > 0.0) {
        // Correlate against e^-jwt and e^+jwt. Whichever is larger says which
        // side of the carrier this IQ puts the tone on, with no FFT and no
        // assumption about the filter kernels.
        const double w = 2.0 * M_PI * toneHz / double(cfg.outputSampleRateHz);
        std::complex<double> pos{}, neg{};
        const size_t skip = std::min<size_t>(iq.size() / 4, 8192);   // settle
        for (size_t n = skip; n < iq.size(); ++n) {
            const std::complex<double> z(iq[n].real(), iq[n].imag());
            pos += z * std::exp(std::complex<double>(0, -w * double(n)));
            neg += z * std::exp(std::complex<double>(0, +w * double(n)));
        }
        const double mp = std::abs(pos), mn = std::abs(neg);
        const bool above = mp > mn;
        std::printf("\n  TONE TEST: %.0f Hz audio, mode %s\n", toneHz,
                    committed ? "USB (committed ALC)" : "USB");
        std::printf("    energy at +%.0f Hz  %12.1f\n", toneHz, mp);
        std::printf("    energy at -%.0f Hz  %12.1f\n", toneHz, mn);
        std::printf("    ratio             %12.1f dB\n",
                    20.0 * std::log10(std::max(mp, mn) / std::max(1e-9, std::min(mp, mn))));
        if (check) {
            // Row C-13. Measured 115.6 dB of rejection; 40 dB is that property
            // with a wide margin, and the SIDE is the assertion that matters --
            // whether "below" is correct depends on the gateware TX mixer sign,
            // which is a different stream's read. This pins what AetherSDR
            // does, so a silent change of convention cannot pass unnoticed.
            const double rejectDb =
                20.0 * std::log10(std::max(mp, mn) / std::max(1e-9, std::min(mp, mn)));
            int failures = 0;
            const auto ck = [&](bool ok, const char* what) {
                std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
                if (!ok) ++failures;
            };
            std::printf("\n");
            ck(!above, "a USB tone leaves the modulator BELOW the carrier "
                       "(the conjugate convention this code applies on purpose)");
            ck(rejectDb >= 40.0,
               "the opposite side is rejected by at least 40 dB");
            std::printf("\n  rejection %.1f dB\n\n", rejectDb);
            return failures == 0 ? 0 : 1;
        }
        std::printf("\n    The modulator puts a USB tone %s the carrier.\n",
                    above ? "ABOVE" : "BELOW");
        std::printf("    %s\n\n", above
            ? "That is the STANDARD analytic convention on the wire."
            : "That is the CONJUGATE of the standard analytic convention:\n"
              "    what reaches the wire is mirrored, and the radio's mixer sign\n"
              "    is what decides whether that is correct.");
        return 0;
    }

    if (wavOut) {
        // 48 kHz mono 16-bit PCM, no further processing of any kind.
        std::vector<int16_t> pcm(iq.size());
        double pk = 0;
        for (size_t n = 0; n < iq.size(); ++n) pk = std::max(pk, double(std::fabs(iq[n].real())));
        for (size_t n = 0; n < iq.size(); ++n)
            pcm[n] = int16_t(std::clamp(iq[n].real(), -1.0f, 1.0f) * 32767.0f);
        FILE* f = std::fopen(wavOut, "wb");
        if (!f) { std::fprintf(stderr, "cannot write %s\n", wavOut); return 2; }
        const uint32_t rate = uint32_t(cfg.outputSampleRateHz);
        const uint32_t dataBytes = uint32_t(pcm.size() * 2);
        const uint32_t riff = 36 + dataBytes;
        const uint16_t one = 1, bits = 16, blockAlign = 2;
        const uint32_t fmtLen = 16, byteRate = rate * 2;
        std::fwrite("RIFF", 1, 4, f); std::fwrite(&riff, 4, 1, f);
        std::fwrite("WAVEfmt ", 1, 8, f); std::fwrite(&fmtLen, 4, 1, f);
        std::fwrite(&one, 2, 1, f); std::fwrite(&one, 2, 1, f);
        std::fwrite(&rate, 4, 1, f); std::fwrite(&byteRate, 4, 1, f);
        std::fwrite(&blockAlign, 2, 1, f); std::fwrite(&bits, 2, 1, f);
        std::fwrite("data", 1, 4, f); std::fwrite(&dataBytes, 4, 1, f);
        std::fwrite(pcm.data(), 2, pcm.size(), f);
        std::fclose(f);
        std::printf("\n  wrote %s\n", wavOut);
        std::printf("    Re(IQ) at %u Hz, %zu samples, peak %.4f\n",
                    rate, pcm.size(), pk);
        std::printf("    filter: Hl2TxDsp 255-tap Blackman analytic bandpass,"
                    " %.0f-%.0f Hz\n", cfg.filterLowHz, cfg.filterHighHz);
        std::printf("    ALC: %s\n", cfg.alcEnabled
                    ? (committed ? "ENABLED, target 0.85 / makeup 40 dB"
                                 : "ENABLED, working-tree values")
                    : "DISABLED -- this isolates the filter");
    }

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

    const double before = envRatio(envBefore, cfg.outputSampleRateHz);
    const double after  = envRatio(envAfter,  cfg.outputSampleRateHz);
    if (check) {
        // Row C-04. Crest is peak/mean of the analytic envelope over 100 ms RMS
        // frames, amplitude basis, on IQ produced by Hl2TxDsp's own 255-tap
        // Blackman analytic bandpass at 300-2700 Hz.
        int failures = 0;
        const auto ck = [&](bool ok, const char* what) {
            std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
            if (!ok) ++failures;
        };
        std::printf("\n");
        ck(clamped == 0,
           "no sample is altered by ep2WriteTxIq's +-1.0 clamp");
        ck(peakAbs < 1.0,
           "the modulator stays inside full scale");
        ck(std::fabs(after - before) < 1e-6,
           "the wire clamp does not change the envelope crest");
        ck(after >= 2.5,
           "crest stays above the speech floor (>= 2.5 at 100 ms frames)");
        std::printf("\n  peak |z| %.3f   crest %.2f   clamped %zu samples\n\n",
                    peakAbs, after, clamped);
        return failures == 0 ? 0 : 1;
    }

    std::printf("\n  The transmitted envelope was measured at 1.09-1.14.\n");
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
