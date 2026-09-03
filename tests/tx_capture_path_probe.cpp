// Does AetherSDR's TX capture path flatten the speech envelope?
//
// The hl2-lab tx-dynamics stream has measured both ENDS of the transmit chain
// and found both healthy: the microphone delivers normal speech dynamics, and
// an offline probe showed Hl2TxDsp preserves them at every ALC ceiling. The
// transmitted envelope is nevertheless flat (peak/mean 1.09-1.14). By
// elimination the flattening should live in the span between them -- the
// capture path, whose DSP island is TxVoiceProcessor.
//
// This measures that span directly, with the operator's real configuration,
// offline. No radio, no operator, no microphone.
//
// REGISTERED AS A TEST as of the acceptance suite (row C-01). It was previously
// a diagnostic that printed and exited 0, which is right for a probe and wrong
// for a test.
//
// The pass criteria are the properties FACTS established, not the exact numbers
// it recorded. The numbers there -- 3.34 in, 3.41 out, 102% retained -- were
// measured on a different stimulus, so asserting them against this fixture
// would be pinning a coincidence. What transfers is the PROPERTY: the span
// retains the envelope. Every threshold below carries its basis, window and
// frame period, because a crest figure without them names nothing.
//
// The positive control is part of the assertion, not a separate mode. A test
// that cannot detect flattening would pass on a chain that flattens everything,
// so this runs the aggressive-ClientComp configuration too and REQUIRES it to
// collapse. If both legs pass, the clean leg means something.
//
// Usage:
//   tx_capture_path_probe <mono-48k-16bit.wav> [--comp]
//
// --comp is the POSITIVE CONTROL and it matters. A probe that reports "no
// flattening" is worthless unless it can be shown to detect flattening when
// flattening is present, so --comp switches in an aggressive ClientComp and the
// same measurement should collapse. If it does not, disbelieve the clean run.

#include "core/TxVoiceProcessor.h"
#include "core/ClientComp.h"

#include <QByteArray>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using AetherSDR::TxVoiceProcessor;
using AetherSDR::ClientComp;

namespace {

// Minimal RIFF reader: 16-bit PCM, mono, any rate. Enough for a `say` capture.
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
    if (all.size() < 44 || std::memcmp(all.data(), "RIFF", 4) != 0) {
        std::fprintf(stderr, "not a RIFF file\n"); return false;
    }
    int channels = 1, bits = 16;
    size_t pos = 12, dataOff = 0, dataLen = 0;
    while (pos + 8 <= all.size()) {
        const char* id = reinterpret_cast<const char*>(all.data() + pos);
        uint32_t sz;
        std::memcpy(&sz, all.data() + pos + 4, 4);
        if (std::memcmp(id, "fmt ", 4) == 0 && pos + 8 + 16 <= all.size()) {
            uint16_t ch, bps; uint32_t sr;
            std::memcpy(&ch,  all.data() + pos + 10, 2);
            std::memcpy(&sr,  all.data() + pos + 12, 4);
            std::memcpy(&bps, all.data() + pos + 22, 2);
            channels = ch; rate = static_cast<int>(sr); bits = bps;
        } else if (std::memcmp(id, "data", 4) == 0) {
            dataOff = pos + 8;
            dataLen = std::min<size_t>(sz, all.size() - dataOff);
        }
        pos += 8 + sz + (sz & 1);
    }
    if (!dataOff || bits != 16) {
        std::fprintf(stderr, "need 16-bit PCM with a data chunk\n"); return false;
    }
    const auto* s = reinterpret_cast<const int16_t*>(all.data() + dataOff);
    const size_t frames = (dataLen / 2) / static_cast<size_t>(channels);
    mono.resize(frames);
    for (size_t i = 0; i < frames; ++i)          // first channel only
        mono[i] = s[i * static_cast<size_t>(channels)] / 32768.0f;
    return true;
}

struct Stats { double rms100; double point10; };

// Both windows the stream reports, on one signal. Frame length is stated
// because peak/mean without one is not a number: the same speech reads ~12 at
// 20 ms, ~8 at 100 ms and ~1.9 at 1 s.
Stats measure(const std::vector<float>& x, int rate)
{
    const int step = rate / 10;                 // 100 ms
    if (step <= 0 || static_cast<int>(x.size()) < step * 2) return {0, 0};

    std::vector<double> frames;
    for (size_t i = 0; i + step <= x.size(); i += step) {
        double acc = 0;
        for (int j = 0; j < step; ++j) acc += double(x[i + j]) * x[i + j];
        frames.push_back(std::sqrt(acc / step));
    }
    auto ratio = [](const std::vector<double>& v) {
        if (v.empty()) return 0.0;
        double sum = 0, peak = 0;
        for (double t : v) { sum += t; peak = std::max(peak, t); }
        const double mean = sum / double(v.size());
        return mean > 0 ? peak / mean : 0.0;
    };

    // A 10 Hz point sample lands at an arbitrary instant, so average the ratio
    // over every phase rather than trusting one alignment.
    double acc = 0; int phases = 0;
    for (int p = 0; p < step; p += std::max(1, step / 50)) {
        std::vector<double> pts;
        for (size_t i = p; i < x.size(); i += step) pts.push_back(std::fabs(x[i]));
        acc += ratio(pts); ++phases;
    }
    return {ratio(frames), phases ? acc / phases : 0.0};
}

} // namespace

// One pass of the capture path. Returns the 100 ms-frame crest in and out.
static bool runOnce(const std::vector<float>& in, int rate,
                    bool positiveControl, Stats& a, Stats& b)
{

    TxVoiceProcessor proc;
    const int block = 1024;
    if (!proc.prepare(rate, block)) {
        std::fprintf(stderr, "prepare(%d) failed\n", rate); return false;
    }

    // The operator's real configuration, read from AetherSDR.db app_settings
    // and from the default each stage falls back to when no key is stored:
    // ClientGateTxEnabled=False, ClientCompTxEnabled defaults False,
    // ClientFinalLimiterTxEnabled defaults False, ClientRn2Enabled=False, and
    // PcMicGain defaults to full. So: no stages, no limiter, unity gain.
    ClientComp comp;
    TxVoiceProcessor::Processors procs{};
    uint64_t stages = 0;
    if (positiveControl) {
        comp.prepare(TxVoiceProcessor::kDspRate);
        comp.setEnabled(true);
        comp.setThresholdDb(-40.0f);
        comp.setRatio(20.0f);
        comp.setAttackMs(1.0f);
        comp.setReleaseMs(50.0f);
        comp.setMakeupDb(0.0f);
        comp.setLimiterEnabled(true);
        comp.setLimiterCeilingDb(-1.0f);
        procs.comp = &comp;
        stages = static_cast<uint64_t>(TxVoiceProcessor::Stage::Comp);
    }
    proc.setProcessors(procs);
    proc.setStageOrder(stages);
    proc.setMicGain(1.0f);
    proc.setRnnoiseEnabled(false);

    // Feed it the way AudioEngine does: canonical duplicated-stereo int16 at
    // the negotiated device rate, one capture block at a time, so the streaming
    // resampler and every stage sees the same block partitioning it sees live.
    std::vector<float> out;
    QByteArray chunk;
    for (size_t i = 0; i + block <= in.size(); i += block) {
        chunk.resize(block * 2 * int(sizeof(int16_t)));
        auto* d = reinterpret_cast<int16_t*>(chunk.data());
        for (int j = 0; j < block; ++j) {
            const auto v = static_cast<int16_t>(
                std::clamp(in[i + j] * 32768.0f, -32768.0f, 32767.0f));
            d[j * 2] = v; d[j * 2 + 1] = v;
        }
        if (!proc.processCapturedInt16(chunk)) continue;
        const auto* o = reinterpret_cast<const int16_t*>(chunk.constData());
        const int frames = chunk.size() / int(2 * sizeof(int16_t));
        for (int j = 0; j < frames; ++j) out.push_back(o[j * 2] / 32768.0f);
    }

    a = measure(in, rate);
    b = measure(out, TxVoiceProcessor::kTransportRate);
    std::printf("  %-34s in %6.2f  out %6.2f  retained %5.0f%%\n",
                positiveControl ? "aggressive ClientComp (control)"
                                : "operator settings, all stages off",
                a.rms100, b.rms100,
                a.rms100 > 0 ? 100.0 * b.rms100 / a.rms100 : 0.0);
    return true;
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <mono-16bit.wav>\n", argv[0]);
        return 2;
    }
    std::vector<float> in;
    int rate = 48000;
    if (!readWav(argv[1], in, rate)) return 2;

    std::printf("\n  fixture: %s  (%zu samples @ %d Hz, %.2f s)\n",
                argv[1], in.size(), rate, double(in.size()) / rate);
    std::printf("  crest is peak/mean of the analytic envelope over 100 ms RMS\n"
                "  frames, amplitude basis, window trimmed to whole frames.\n\n");

    Stats cleanIn{}, cleanOut{}, compIn{}, compOut{};
    if (!runOnce(in, rate, /*positiveControl=*/false, cleanIn, cleanOut)) {
        std::fprintf(stderr, "clean pass produced nothing\n");
        return 1;
    }
    if (!runOnce(in, rate, /*positiveControl=*/true, compIn, compOut)) {
        std::fprintf(stderr, "control pass produced nothing\n");
        return 1;
    }

    const double retained = cleanIn.rms100 > 0 ? cleanOut.rms100 / cleanIn.rms100 : 0.0;
    const double control  = compIn.rms100  > 0 ? compOut.rms100  / compIn.rms100  : 0.0;

    // Row C-01. FACTS measured 102% retained on a different stimulus; 0.95 is
    // that property with margin, not that number re-asserted.
    constexpr double kMinRetained = 0.95;
    // Catalogue B1's floor for speech that still has its dynamics.
    constexpr double kMinCrest = 2.5;
    // The control must visibly collapse, or the clean leg proves nothing.
    constexpr double kMaxControl = 0.80;

    int failures = 0;
    const auto check = [&](bool ok, const char* what) {
        std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
        if (!ok) ++failures;
    };
    std::printf("\n");
    check(retained >= kMinRetained,
          "the capture path retains the envelope (>= 95% of input crest)");
    check(cleanOut.rms100 >= kMinCrest,
          "output crest stays above the speech floor (>= 2.5 at 100 ms)");
    check(control <= kMaxControl,
          "POSITIVE CONTROL: aggressive ClientComp collapses the crest (<= 80%)");
    check(control < retained,
          "the control is measurably worse than the clean path");

    std::printf("\n  retained %.0f%%   control %.0f%%\n\n",
                100.0 * retained, 100.0 * control);
    return failures == 0 ? 0 : 1;
}
