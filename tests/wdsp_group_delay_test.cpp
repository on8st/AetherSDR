// Measured RX group delay vs bandpass FIR length, and what minimum phase does
// to it and to notch depth.
//
// WHY THIS EXISTS. The S4 latency claim is arithmetic, not measurement:
// Hl2RxDsp::kRxFilterTaps == 8192 is asserted to cost (8192-1)/2 = 4095.5
// samples = 85.3 ms at 48 kHz, against 21.3 ms for WDSP's 2048 default, so a
// 64 ms delta. Nobody has ever put a signal through the channel and watched
// when it came out. (taps-1)/2 is the delay of the FIR ALONE; the channel the
// operator actually hears is an overlap-save FFT filter with its own block
// latency, a resampler, an AGC and a mute envelope in front of it. This file
// measures the whole thing, end to end, offline.
//
// It is a MEASUREMENT harness, not a regression gate: it prints numbers and
// fails only when a measurement could not be taken at all. Pinning a latency
// figure as an assertion is a separate decision and needs a number first.
//
// METHOD. Open a channel, clock it with silence until the startup mute ramp
// has finished, then switch to a continuous in-band complex tone and find the
// first output sample that reaches half of the eventual steady amplitude. For
// a linear-phase FIR the step response of the envelope crosses 50 % at the
// centre tap, so that index is the delay we are after plus whatever fixed
// pipeline latency the channel adds. The mute ramp and the pipeline are
// identical for every configuration measured here, so they cancel in the
// deltas; the absolute figures carry them and are reported as such.
//
// PRIOR ART. Closed PR #5579 (W5TSU) wrote the same onset measurement inside
// tests/wdsp_channel_test.cpp's runFilterTapsTest. Its lambda is sound and the
// shape of it is reused here. Two things are changed deliberately:
//
//   * #5579 asserted only the DELTA, into a band (2700 < delta < 3400
//     samples), and printed the numbers only on failure. A passing run
//     therefore produced no measurement at all. Here every number is printed
//     on success, which is the entire point.
//   * #5579 switched tap length in place with a setFilterTaps() method that PR
//     added to WdspChannel. That method is not in main and this file adds no
//     production code, so each configuration gets its own create()/destroy.
//     That is also the cleaner measurement: no residual state from the
//     previous length, and open() is the path the operator's channel really
//     takes.
//   * #5579 took the threshold from the peak over the WHOLE captured run. A
//     linear-phase bandpass step response overshoots, so that peak is the
//     Gibbs overshoot, not the steady level. Here the steady amplitude is the
//     peak over the final quarter of the capture.

#include "core/dsp/WdspChannel.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <span>
#include <string>
#include <vector>

namespace {

constexpr int kSampleRate = 48000;
constexpr std::size_t kBlock = 256;
// Baseband pitch of the probe tone. NEGATIVE because RXA as configured passes
// the opposite sign to its passband bounds (see Hl2RxDsp::onIqBlock and the
// note in runNotchAttenuationTest) — this is the geometry the HL2 runs in.
constexpr double kToneBasebandHz = -1500.0;

bool require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

double rms(std::span<const float> samples)
{
    double sum = 0.0;
    for (const float sample : samples) {
        sum += static_cast<double>(sample) * static_cast<double>(sample);
    }
    return std::sqrt(sum / static_cast<double>(samples.size()));
}

void fillComplexTone(std::span<float> i, std::span<float> q,
                     double frequencyHz, std::size_t offset)
{
    for (std::size_t sample = 0; sample < i.size(); ++sample) {
        const double phase = 2.0 * std::numbers::pi * frequencyHz *
                             static_cast<double>(offset + sample) /
                             static_cast<double>(kSampleRate);
        i[sample] = static_cast<float>(0.1 * std::cos(phase));
        q[sample] = static_cast<float>(0.1 * std::sin(phase));
    }
}

WdspChannel::Config baseConfig()
{
    WdspChannel::Config config;
    config.inputBlockSize = kBlock;
    config.dspBlockSize = kBlock;
    config.inputSampleRate = kSampleRate;
    config.dspSampleRate = kSampleRate;
    config.outputSampleRate = kSampleRate;
    config.mode = WdspChannel::Mode::Usb;
    config.filterLowHz = 150.0;
    config.filterHighHz = 3000.0;
    // AGC off at unity. An AGC would ramp the level up on its own schedule and
    // the "half of steady" crossing would then be measuring the AGC attack.
    config.agcMode = 0;
    config.agcFixedGainDb = 0.0;
    // Nothing here paces the loop; without this the caller outruns WDSP's
    // worker and every block underruns.
    config.blockForOutput = true;
    return config;
}

struct OnsetResult
{
    bool ok = false;
    long rawCrossingSamples = -1;       // first |x| >= half steady
    long envelopeCrossingSamples = -1;  // centred-window envelope crossing
    double steadyPeak = 0.0;
};

// Clock `silenceBlocks` of zeros to settle the startup mute ramp, then a
// continuous tone for `toneBlocks`, and report when the output came up.
OnsetResult measureOnset(WdspChannel& channel,
                         int silenceBlocks = 120,
                         int toneBlocks = 240)
{
    OnsetResult result;
    std::vector<float> inI(kBlock, 0.0f);
    std::vector<float> inQ(kBlock, 0.0f);
    std::vector<float> outL(channel.outputBlockSize());
    std::vector<float> outR(channel.outputBlockSize());

    for (int block = 0; block < silenceBlocks; ++block) {
        if (channel.processIq(inI, inQ, outL, outR) != WdspChannel::ProcessResult::Ok) {
            return result;
        }
    }

    std::vector<float> captured;
    captured.reserve(static_cast<std::size_t>(toneBlocks) * channel.outputBlockSize());
    for (int block = 0; block < toneBlocks; ++block) {
        fillComplexTone(inI, inQ, kToneBasebandHz,
                        static_cast<std::size_t>(block) * kBlock);
        if (channel.processIq(inI, inQ, outL, outR) != WdspChannel::ProcessResult::Ok) {
            return result;
        }
        captured.insert(captured.end(), outL.begin(), outL.end());
    }
    if (captured.empty()) {
        return result;
    }

    // Steady amplitude from the final quarter of the capture: past the filter
    // transient and past its overshoot.
    const std::size_t tailStart = captured.size() * 3 / 4;
    double steady = 0.0;
    for (std::size_t n = tailStart; n < captured.size(); ++n) {
        steady = std::max(steady, std::fabs(static_cast<double>(captured[n])));
    }
    if (steady <= 0.0) {
        return result;
    }
    const double threshold = steady * 0.5;

    long raw = -1;
    for (std::size_t n = 0; n < captured.size(); ++n) {
        if (std::fabs(static_cast<double>(captured[n])) >= threshold) {
            raw = static_cast<long>(n);
            break;
        }
    }

    // The raw crossing can only land on a peak of the 1500 Hz carrier, so it
    // is quantised to a half period (16 samples, 0.33 ms) and biased late.
    // A centred max over one tone period removes that bias.
    const long half = static_cast<long>(kSampleRate / std::fabs(kToneBasebandHz) / 2.0);
    long enveloped = -1;
    for (long n = 0; n < static_cast<long>(captured.size()); ++n) {
        const long lo = std::max<long>(0, n - half);
        const long hi = std::min<long>(static_cast<long>(captured.size()) - 1, n + half);
        double localPeak = 0.0;
        for (long k = lo; k <= hi; ++k) {
            localPeak = std::max(localPeak, std::fabs(static_cast<double>(captured[k])));
        }
        if (localPeak >= threshold) {
            enveloped = n;
            break;
        }
    }

    result.ok = raw >= 0 && enveloped >= 0;
    result.rawCrossingSamples = raw;
    result.envelopeCrossingSamples = enveloped;
    result.steadyPeak = steady;
    return result;
}

double samplesToMs(long samples)
{
    return 1000.0 * static_cast<double>(samples) / static_cast<double>(kSampleRate);
}

// `notchHz` != 0 places a notch (relative to a 7 MHz tune, away from the probe
// tone) before measuring. That is the POSITIVE CONTROL for (c): adding a notch
// rebuilds the NBP mask, and if that rebuild dropped the minimum-phase flag,
// (c)'s "minimum phase" rows would silently be measuring a linear-phase
// channel and would agree with the linear-phase rows for the wrong reason.
// A notched mp=1 channel must still show the collapsed delay from (b).
OnsetResult measureConfiguration(const char* label, int taps, bool minimumPhase,
                                 double notchOffsetHz = 0.0)
{
    WdspChannel::Config config = baseConfig();
    config.filterTaps = taps;
    config.minimumPhase = minimumPhase;

    std::string error;
    std::unique_ptr<WdspChannel> channel = WdspChannel::create(config, &error);
    if (channel && notchOffsetHz != 0.0) {
        constexpr double kTuneHz = 7'000'000.0;
        if (!channel->setNotchTuneFrequency(kTuneHz) ||
            !channel->addNotch(0, kTuneHz + notchOffsetHz, 50.0, true) ||
            !channel->setNotchesEnabled(true)) {
            std::cerr << "FAIL: could not seed the control notch for " << label << '\n';
            return {};
        }
    }
    if (!channel) {
        std::cerr << "FAIL: could not open channel for " << label << ": "
                  << error << '\n';
        return {};
    }
    const OnsetResult onset = measureOnset(*channel);
    if (!onset.ok) {
        std::cerr << "FAIL: onset measurement produced no crossing for "
                  << label << '\n';
        return onset;
    }
    std::printf("  %-34s taps=%-5d mp=%d  onset raw=%5ld smp (%7.2f ms)"
                "  envelope=%5ld smp (%7.2f ms)  steady=%.6f\n",
                label, taps, minimumPhase ? 1 : 0,
                onset.rawCrossingSamples, samplesToMs(onset.rawCrossingSamples),
                onset.envelopeCrossingSamples, samplesToMs(onset.envelopeCrossingSamples),
                onset.steadyPeak);
    return onset;
}

// ── (c) notch depth ─────────────────────────────────────────────────────────
//
// The same measurement runNotchAttenuationTest makes, reported in dB instead
// of compared to a ratio, and run twice: linear phase and minimum phase. The
// risk being probed is WDSP's cepstral minimum-phase path, which floors its
// log at log(1.0e-300) — exactly where a notch null sits. If the null cannot
// survive the log floor, minimum phase and a deep notch are mutually
// exclusive.
//
// The mirror notch is the positive control. A notch as far BELOW the tuned
// frequency as the tone is above must leave the tone alone; if both notches
// attenuate, or neither does, the measurement is broken and the on-tone number
// means nothing.
double notchEnergy(int taps, bool minimumPhase, double notchRfHz, double tuneHz,
                   double widthHz, double* minimumWidthOut = nullptr)
{
    WdspChannel::Config config = baseConfig();
    config.filterTaps = taps;
    config.minimumPhase = minimumPhase;

    std::string error;
    std::unique_ptr<WdspChannel> channel = WdspChannel::create(config, &error);
    if (!channel || !channel->setNotchTuneFrequency(tuneHz)) {
        return -1.0;
    }
    if (minimumWidthOut != nullptr) {
        *minimumWidthOut = channel->minimumNotchWidthHz();
    }
    if (notchRfHz != 0.0) {
        if (!channel->addNotch(0, notchRfHz, widthHz, true) ||
            !channel->setNotchesEnabled(true)) {
            return -1.0;
        }
    }
    std::vector<float> inI(kBlock);
    std::vector<float> inQ(kBlock);
    std::vector<float> outL(channel->outputBlockSize());
    std::vector<float> outR(channel->outputBlockSize());
    double energy = 0.0;
    // Long settle: at 8192 taps the mask takes a while to fill and the group
    // delay alone is ~85 ms by the arithmetic this file is checking.
    constexpr std::size_t kSettleBlocks = 60;
    constexpr std::size_t kTotalBlocks = 140;
    for (std::size_t block = 0; block < kTotalBlocks; ++block) {
        fillComplexTone(inI, inQ, kToneBasebandHz, block * kBlock);
        if (channel->processIq(inI, inQ, outL, outR) != WdspChannel::ProcessResult::Ok) {
            return -1.0;
        }
        if (block >= kSettleBlocks) {
            energy += rms(outL);
        }
    }
    return energy;
}

double toDb(double ratio)
{
    if (ratio <= 0.0) {
        return -std::numeric_limits<double>::infinity();
    }
    return 20.0 * std::log10(ratio);
}

bool runNotchDepth(int taps, bool minimumPhase, double widthHz)
{
    constexpr double kTuneHz = 7'000'000.0;
    const double toneRfHz = kTuneHz + 1500.0;
    const double mirrorRfHz = kTuneHz - 1500.0;

    double minimumWidth = 0.0;
    const double unnotched =
        notchEnergy(taps, minimumPhase, 0.0, kTuneHz, widthHz, &minimumWidth);
    const double notched = notchEnergy(taps, minimumPhase, toneRfHz, kTuneHz, widthHz);
    const double mirrored = notchEnergy(taps, minimumPhase, mirrorRfHz, kTuneHz, widthHz);

    if (!require(unnotched > 0.0 && notched >= 0.0 && mirrored >= 0.0,
                 "notch depth measurement failed to run") ||
        !require(unnotched > 1.0e-4, "the unnotched tone produced no audio")) {
        return false;
    }
    std::printf("  taps=%-5d mp=%d width=%5.0f Hz (floor %5.1f Hz)  "
                "on-tone=%+8.2f dB   mirror(control)=%+6.2f dB\n",
                taps, minimumPhase ? 1 : 0, widthHz, minimumWidth,
                toDb(notched / unnotched), toDb(mirrored / unnotched));
    return true;
}

} // namespace

int main()
{
    std::printf("wdsp_group_delay_test: RX latency and notch depth, measured\n");
    std::printf("conditions: %d Hz, block %zu, USB 150-3000 Hz, AGC off at 0 dB, "
                "blockForOutput, probe tone %.0f Hz baseband\n\n",
                kSampleRate, kBlock, kToneBasebandHz);

    std::printf("(a) group delay vs taps, linear phase\n");
    const OnsetResult shortTaps = measureConfiguration("2048 taps (WDSP default)", 2048, false);
    const OnsetResult longTaps = measureConfiguration("8192 taps (Hl2RxDsp)", 8192, false);
    if (!require(shortTaps.ok && longTaps.ok, "(a) did not measure")) {
        return 1;
    }
    const long deltaRaw = longTaps.rawCrossingSamples - shortTaps.rawCrossingSamples;
    const long deltaEnv = longTaps.envelopeCrossingSamples - shortTaps.envelopeCrossingSamples;
    std::printf("  DELTA 8192 - 2048: raw=%ld smp (%.2f ms)  envelope=%ld smp (%.2f ms)\n",
                deltaRaw, samplesToMs(deltaRaw), deltaEnv, samplesToMs(deltaEnv));
    std::printf("  arithmetic predicts (8192-2048)/2 = 3072 smp = %.2f ms\n\n",
                samplesToMs(3072));

    std::printf("(b) 8192 taps with minimum phase\n");
    const OnsetResult minPhase = measureConfiguration("8192 taps, RXASetMP(1)", 8192, true);
    if (!require(minPhase.ok, "(b) did not measure")) {
        return 1;
    }
    std::printf("  vs 8192 linear phase: raw=%ld smp (%.2f ms)  envelope=%ld smp (%.2f ms)\n",
                minPhase.rawCrossingSamples - longTaps.rawCrossingSamples,
                samplesToMs(minPhase.rawCrossingSamples - longTaps.rawCrossingSamples),
                minPhase.envelopeCrossingSamples - longTaps.envelopeCrossingSamples,
                samplesToMs(minPhase.envelopeCrossingSamples - longTaps.envelopeCrossingSamples));
    std::printf("  vs 2048 linear phase: raw=%ld smp (%.2f ms)  envelope=%ld smp (%.2f ms)\n\n",
                minPhase.rawCrossingSamples - shortTaps.rawCrossingSamples,
                samplesToMs(minPhase.rawCrossingSamples - shortTaps.rawCrossingSamples),
                minPhase.envelopeCrossingSamples - shortTaps.envelopeCrossingSamples,
                samplesToMs(minPhase.envelopeCrossingSamples - shortTaps.envelopeCrossingSamples));

    // 400 Hz is what runNotchAttenuationTest already uses, so the linear-phase
    // number is comparable to the existing suite. 50 Hz is the width that
    // 8192 taps exist to make possible at all, and therefore the one option C
    // has to keep: a minimum-phase channel that holds a 400 Hz notch but
    // collapses at 50 Hz has not kept the thing that was being paid for.
    // Positive control, run before (c) so a broken one invalidates it loudly:
    // 50 Hz notch at +2500 Hz, clear of the -1500 Hz probe tone.
    std::printf("(c-control) minimum phase survives notch placement\n");
    const OnsetResult minPhaseNotched =
        measureConfiguration("8192 mp, one 50 Hz notch", 8192, true, 2500.0);
    if (!require(minPhaseNotched.ok, "(c-control) did not measure")) {
        return 1;
    }
    std::printf("  vs un-notched mp: %ld smp; vs 8192 linear phase: %ld smp"
                "  (must track the former, not the latter)\n\n",
                minPhaseNotched.envelopeCrossingSamples - minPhase.envelopeCrossingSamples,
                minPhaseNotched.envelopeCrossingSamples - longTaps.envelopeCrossingSamples);

    std::printf("(c) notch depth, notch parked on the tone\n");
    if (!runNotchDepth(8192, false, 400.0) ||
        !runNotchDepth(8192, true, 400.0) ||
        !runNotchDepth(8192, false, 50.0) ||
        !runNotchDepth(8192, true, 50.0)) {
        return 1;
    }

    std::printf("\nwdsp_group_delay_test: measurements taken\n");
    return 0;
}
