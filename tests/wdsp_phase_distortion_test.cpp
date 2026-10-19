// What minimum phase costs in the passband, and where the RX channel's
// constant 33 ms of latency comes from.
//
// WHY THIS EXISTS. d107 (tests/wdsp_group_delay_test.cpp) measured that
// RXASetMP(1) at 8192 taps brings the channel's onset to 34.27 ms -- 20 ms
// FASTER than WDSP's 2048-tap linear-phase default -- while holding the 50 Hz
// notch floor that 8192 taps were bought for. That makes minimum phase better
// than the tap-switching in upstream #5578/#5579 on both axes, and the only
// thing in the way is that nobody has measured what the phase response costs.
// #5578 carries the CW label, so the keying envelope is the named risk; data
// demodulators are the other. d107 closed with "passband phase distortion is
// not established". Section (A) establishes it.
//
// d107 also closed with a second open item: measured onset fits
// 1583.5 + (taps-1)/2 at BOTH tap lengths, so ~33.0 ms of the channel's
// latency is not the bandpass FIR and is invisible in every delta. Section (B)
// decomposes it.
//
// A MEASUREMENT HARNESS, NOT A GATE -- it prints numbers and fails only when a
// measurement could not be taken. Same reason d107 is not wired into ctest.
//
// ── THE ARTEFACT THAT HAD TO BE REMOVED FIRST ──────────────────────────────
//
// d107 clocks 640 ms of SILENCE to settle the startup mute ramp, and states
// that the ramp is therefore spent before the probe tone starts. IT IS NOT, and
// this file's first run is what caught it. WDSP's up-slew state machine
// (third_party/wdsp/upstream/iobuffs.c, upslew2) leaves its BEGIN state only
// `if ((I != 0.0) || (Q != 0.0))`. Silence is the one input that pins it in
// BEGIN forever. The ramp therefore fires on the FIRST TONE SAMPLE, and a
// silent settle is a no-op for the thing it was written to do.
//
// It costs 482 samples of hard zero plus a 1201-sample raised cosine
// (Config::muteDelayUpSec 0.010 and muteSlewUpSec 0.025 at 48 kHz), 50 % at
// +1082. That lands inside every envelope d107 measured. The visible damage in
// the first run here: a 20 ms CW dot came out at 12 % of full amplitude, which
// looks exactly like an 8192-tap filter destroying fast keying and is in fact
// the dot sitting inside the mute ramp's dead zone.
//
// So the settle here is PRIMED, not silent: deterministic pseudo-random noise
// at amplitude 1e-5 (100 dB below the probe tone, and below every threshold
// used) that drives upslew2 out of BEGIN and lets the ramp complete. That is
// also the production-faithful condition -- a real receiver's input is never
// bit-exact zero, so the operator's channel has finished its ramp long before
// any signal of interest arrives. Section (B) measures the unprimed case too,
// because the size of the difference is the evidence for the attribution.
//
// ── METHOD ──────────────────────────────────────────────────────────────────
//
// ENVELOPE. d107's onset estimator takes a centred max over one period of the
// probe tone, so its window is a function of frequency (16 samples at 1500 Hz,
// 120 at 200 Hz). Comparing onsets ACROSS frequency with it would compare
// estimators as much as channels. Here the output is demodulated by
// exp(-j*2*pi*f*n/fs) and smoothed with a fixed-length centred boxcar: the
// demodulation removes the carrier so the smoothing no longer has to track it,
// and the window is symmetric so a 50 %-of-steady crossing on a step is
// unbiased rather than late. Onset figures here are therefore NOT
// bit-comparable with d107's.
//
// GROUP DELAY is measured from steady-state PHASE, not from an onset. An onset
// is a perceptual arrival time; group delay is -dphi/domega, and it is the
// quantity that governs whether a data demodulator's symbols smear. It is also
// immune to the mute ramp and to the filter's ring-up, which is why section (B)
// leans on it.
//
// A comb of exact-DFT-bin tones goes in -- the channel is linear here, AGC off
// at fixed gain -- and one capture returns the steady-state phase of every
// tone. Bins are exact multiples of fs/W and the analysis window is RECTANGULAR
// and exactly W samples, which makes neighbouring bins exactly orthogonal; a
// Hann window would have smeared the 1.95 Hz pair into one mainlobe and
// produced a confident wrong number. The window also starts on an exact
// multiple of W from the tone's origin, because a window starting `from`
// samples in measures (from - D) instead of -D, and `from` is far outside the
// range the one-bin pair can resolve -- that aliases the answer rather than
// merely offsetting it.
//
// Each probe gets THREE bins: k, k+1, k+8. The (k, k+1) pair is unambiguous
// over W/2 = 12288 samples, which covers every delay this channel produces, but
// resolves only ~0.8 ms. The (k, k+8) pair is 8x finer and ambiguous every 3072
// samples, so it is unwrapped against the coarse one. The reported delay is the
// refined one, over a 15.6 Hz baseline -- about the bandwidth of a 60 WPM CW
// dot and wider than a PSK31 signal, so it is the local group delay a
// narrowband mode actually sees.
//
// CONTROLS, because a phase measurement can agree for the wrong reason:
//   * LINEAR PHASE MUST READ FLAT. A linear-phase FIR has constant group delay
//     by construction. If the mp=0 sweep is not flat the method is broken and
//     the mp=1 sweep means nothing. This is the load-bearing control.
//   * A DEAD BIN, read out alongside the others with no tone injected. It
//     reports the intermodulation and numerical floor; if it is not far below
//     the tone bins, superposition is false and every group delay is void.
//   * COMB vs SINGLE TONE at two frequencies, which must agree.
//   * THE INPUT GATE ITSELF is pushed through the keying estimator with no
//     channel at all, so every envelope figure has its estimator floor printed
//     next to it. A 2.9 ms "rise time" means nothing until you know the
//     estimator reads 2.5 ms on an undistorted gate.
//
// Cited by file and symbol throughout, never by line number.

#include "core/dsp/WdspChannel.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr int kSampleRate = 48000;
constexpr std::size_t kBlock = 256;
constexpr double kPi = std::numbers::pi;

// d107's sign note, carried over because it is easy to get wrong: RXA as
// configured passes the OPPOSITE sign to its passband bounds (see
// Hl2RxDsp::onIqBlock and the handedness note on WdspChannel::addNotch), so an
// audio tone at +f Hz is produced by a baseband input at -f Hz.
constexpr double kBasebandSign = -1.0;

// The settle-period prime. Non-zero is the whole requirement (upslew2 tests
// `I != 0.0 || Q != 0.0`); 1e-5 is 100 dB below the 0.1 probe tone, so it
// cannot reach any threshold used below.
constexpr double kPrimeAmplitude = 1.0e-5;

bool require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

double samplesToMs(double samples, int rate = kSampleRate)
{
    return 1000.0 * samples / static_cast<double>(rate);
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
    // AGC off at unity, exactly as d107. An AGC would ramp the level on its own
    // schedule and every envelope figure below would be measuring the attack.
    // NOTE, and it matters for anyone quoting these numbers: wcpagc.c's
    // xwcpagc early-returns at mode 0, so the AGC's lookahead buffer is not in
    // this measurement at all. The production default is agcMode 3, and (B9)
    // prices it.
    config.agcMode = 0;
    config.agcFixedGainDb = 0.0;
    config.blockForOutput = true;
    // Mute envelope left at the production default. It is primed away rather
    // than switched off, so these runs stay comparable with the shipping
    // channel; (B5) switches it off in one row to price it.
    return config;
}

// Deterministic, so a re-run is bit-identical. xorshift32 rather than
// std::mt19937 only because it is one line and the distribution does not
// matter -- non-zero does.
struct Prime
{
    std::uint32_t state = 0x1234567u;
    double next()
    {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return kPrimeAmplitude *
               (2.0 * (static_cast<double>(state) / 4294967296.0) - 1.0);
    }
};

// ── capture ─────────────────────────────────────────────────────────────────

struct Capture
{
    bool ok = false;
    std::vector<float> samples;   // output left channel, from the first tone block
};

// Clock `settleBlocks` of PRIMED noise (or true silence if `primed` is false,
// which is d107's condition and is kept only so (B5) can price the difference),
// then `toneBlocks` of whatever `fill` writes, capturing the output.
template <typename Fill>
Capture captureChannel(WdspChannel& channel, std::size_t inBlock, bool primed,
                       int settleBlocks, int toneBlocks, Fill fill)
{
    Capture capture;
    const std::size_t outBlock = channel.outputBlockSize();
    std::vector<float> inI(inBlock, 0.0f);
    std::vector<float> inQ(inBlock, 0.0f);
    std::vector<float> outL(outBlock);
    std::vector<float> outR(outBlock);

    Prime prime;
    for (int block = 0; block < settleBlocks; ++block) {
        for (std::size_t sample = 0; sample < inBlock; ++sample) {
            inI[sample] = primed ? static_cast<float>(prime.next()) : 0.0f;
            inQ[sample] = primed ? static_cast<float>(prime.next()) : 0.0f;
        }
        if (channel.processIq(inI, inQ, outL, outR) != WdspChannel::ProcessResult::Ok) {
            return capture;
        }
    }

    capture.samples.reserve(static_cast<std::size_t>(toneBlocks) * outBlock);
    for (int block = 0; block < toneBlocks; ++block) {
        fill(inI, inQ, static_cast<std::size_t>(block) * inBlock);
        if (channel.processIq(inI, inQ, outL, outR) != WdspChannel::ProcessResult::Ok) {
            return capture;
        }
        capture.samples.insert(capture.samples.end(), outL.begin(), outL.end());
    }
    capture.ok = !capture.samples.empty();
    return capture;
}

std::unique_ptr<WdspChannel> openChannel(const WdspChannel::Config& config,
                                         const char* label)
{
    std::string error;
    std::unique_ptr<WdspChannel> channel = WdspChannel::create(config, &error);
    if (!channel) {
        std::cerr << "FAIL: could not open channel for " << label << ": "
                  << error << '\n';
    }
    return channel;
}

// ── envelope ────────────────────────────────────────────────────────────────
//
// |boxcar(x[n] * exp(-j 2 pi f n / fs))|, one output per input sample, O(N).
// `halfWidth` is chosen per section: wide enough to reject the 2f image term
// the demodulation leaves behind, narrow enough not to smear the edge being
// measured. Both choices are stated where they are made.
std::vector<double> demodEnvelope(std::span<const float> x, double toneHz,
                                  long halfWidth, int rate)
{
    const std::size_t n = x.size();
    std::vector<std::complex<double>> prefix(n + 1, {0.0, 0.0});
    for (std::size_t i = 0; i < n; ++i) {
        const double phase = -2.0 * kPi * toneHz * static_cast<double>(i) /
                             static_cast<double>(rate);
        prefix[i + 1] = prefix[i] +
                        static_cast<double>(x[i]) *
                            std::complex<double>(std::cos(phase), std::sin(phase));
    }
    std::vector<double> envelope(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        const long lo = std::max<long>(0, static_cast<long>(i) - halfWidth);
        const long hi = std::min<long>(static_cast<long>(n),
                                       static_cast<long>(i) + halfWidth + 1);
        envelope[i] = std::abs(prefix[static_cast<std::size_t>(hi)] -
                               prefix[static_cast<std::size_t>(lo)]) /
                      static_cast<double>(hi - lo);
    }
    return envelope;
}

// Steady level from the final quarter, as d107 does and for the same reason: a
// linear-phase bandpass step response overshoots, so the peak over the whole
// capture is Gibbs overshoot rather than the settled level.
//
// THE LAST `halfWidth` SAMPLES ARE EXCLUDED, and they have to be. There the
// boxcar is truncated against the end of the buffer, so the 2f image term the
// demodulation leaves behind stops cancelling and the envelope climbs: measured
// on a pure 700 Hz tone with halfWidth 68, the interior reads 0.049948 and the
// final sample reads 0.053349 -- 6.8 % high, and "max over the final quarter"
// picks exactly that. It read back as every gated pulse failing to reach 94 %
// of the continuous level through a filter that is flat at the probe frequency.
double steadyLevel(std::span<const double> envelope, long halfWidth)
{
    const std::size_t last = envelope.size() > static_cast<std::size_t>(halfWidth)
                                 ? envelope.size() - static_cast<std::size_t>(halfWidth)
                                 : envelope.size();
    double steady = 0.0;
    for (std::size_t i = envelope.size() * 3 / 4; i < last; ++i) {
        steady = std::max(steady, envelope[i]);
    }
    return steady;
}

double risingCrossing(std::span<const double> envelope, double level)
{
    for (std::size_t i = 1; i < envelope.size(); ++i) {
        if (envelope[i] >= level) {
            const double previous = envelope[i - 1];
            if (envelope[i] == previous) {
                return static_cast<double>(i);
            }
            return static_cast<double>(i - 1) +
                   (level - previous) / (envelope[i] - previous);
        }
    }
    return -1.0;
}

// ── group delay from steady-state phase ─────────────────────────────────────

constexpr std::size_t kAnalysisWindow = 24576;   // at 48 kHz: 0.512 s
constexpr double kBinHz = static_cast<double>(kSampleRate) /
                          static_cast<double>(kAnalysisWindow);
constexpr int kFineOffset = 1;     // unambiguous over W/2 = 12288 smp, ~0.8 ms
constexpr int kCoarseOffset = 8;   // 8x finer, ambiguous every 3072 smp

// 150-3000 Hz is the passband, so 200 and 2900 are deliberately near the
// skirts: minimum phase concentrates its delay variation at the band edges and
// a sweep stopping at 2400 would not see it.
constexpr double kProbeCentres[] = {200.0,  300.0,  400.0,  600.0,
                                    800.0,  1000.0, 1200.0, 1500.0,
                                    2000.0, 2400.0, 2700.0, 2900.0};
constexpr std::size_t kProbeCount = std::size(kProbeCentres);

// A bin with no tone in it: the intermodulation and numerical floor.
constexpr int kDeadBin = 350;   // 683.6 Hz, clear of every probe triple

// Per-tone amplitude. All tones start at phase 0 so they add coherently once
// per W samples; 36 at 0.005 peak at 0.18, which the dead-bin control then
// confirms is inside the linear range.
constexpr double kCombAmplitude = 0.005;

struct CombTone
{
    int bin = 0;
    double hz = 0.0;
};

double wrapToPi(double radians)
{
    while (radians > kPi) {
        radians -= 2.0 * kPi;
    }
    while (radians <= -kPi) {
        radians += 2.0 * kPi;
    }
    return radians;
}

// A pure delay D gives phase -2*pi*bin*D/W, so d(phase)/d(bin) = -2*pi*D/W and
// D = -dphi*W/(2*pi*binSpan). Ambiguous every W/binSpan; `near` resolves it by
// taking the alias closest to a coarser estimate.
double delayFromPhase(double phaseDifference, int binSpan, std::size_t window,
                      double near)
{
    const double period = static_cast<double>(window) / static_cast<double>(binSpan);
    double delay = -wrapToPi(phaseDifference) * static_cast<double>(window) /
                   (2.0 * kPi * static_cast<double>(binSpan));
    if (std::isfinite(near)) {
        delay += period * std::round((near - delay) / period);
    }
    return delay;
}

std::complex<double> binPhasor(std::span<const float> x, std::size_t from,
                               int bin, std::size_t window)
{
    std::complex<double> acc {0.0, 0.0};
    for (std::size_t n = 0; n < window; ++n) {
        const double phase = -2.0 * kPi * static_cast<double>(bin) *
                             static_cast<double>(n) / static_cast<double>(window);
        acc += static_cast<double>(x[from + n]) *
               std::complex<double>(std::cos(phase), std::sin(phase));
    }
    return acc * (2.0 / static_cast<double>(window));
}

void fillComb(std::span<float> i, std::span<float> q,
              const std::vector<CombTone>& tones, std::size_t offset, int rate,
              double amplitude)
{
    std::fill(i.begin(), i.end(), 0.0f);
    std::fill(q.begin(), q.end(), 0.0f);
    for (const CombTone& tone : tones) {
        for (std::size_t sample = 0; sample < i.size(); ++sample) {
            const double phase = kBasebandSign * 2.0 * kPi * tone.hz *
                                 static_cast<double>(offset + sample) /
                                 static_cast<double>(rate);
            i[sample] += static_cast<float>(amplitude * std::cos(phase));
            q[sample] += static_cast<float>(amplitude * std::sin(phase));
        }
    }
}

struct PhaseDelay
{
    bool ok = false;
    double fineSamples = 0.0;      // (k, k+1), unambiguous, coarse
    double refinedSamples = 0.0;   // (k, k+8), unwrapped against it
    double levelDb = 0.0;
};

// The general form: arbitrary block size and arbitrary in/dsp/out rates. The
// analysis window scales with the rate so the probe stays on an exact bin on
// both sides, and the answer is converted back to INPUT samples so rows with
// different output rates are comparable.
PhaseDelay measurePhaseDelay(const WdspChannel::Config& config, double centreHz,
                             bool primed, const char* label)
{
    PhaseDelay result;
    std::unique_ptr<WdspChannel> channel = openChannel(config, label);
    if (!channel) {
        return result;
    }
    const std::size_t inBlock = config.inputBlockSize;
    const std::size_t outBlock = channel->outputBlockSize();
    const std::size_t windowIn = kAnalysisWindow *
                                 static_cast<std::size_t>(config.inputSampleRate) /
                                 static_cast<std::size_t>(kSampleRate);
    const std::size_t windowOut = windowIn *
                                  static_cast<std::size_t>(config.outputSampleRate) /
                                  static_cast<std::size_t>(config.inputSampleRate);
    const int base = static_cast<int>(std::lround(
        centreHz * static_cast<double>(windowIn) / config.inputSampleRate));
    const double binHz = static_cast<double>(config.inputSampleRate) /
                         static_cast<double>(windowIn);
    std::vector<CombTone> tones {
        {base, base * binHz},
        {base + kFineOffset, (base + kFineOffset) * binHz},
        {base + kCoarseOffset, (base + kCoarseOffset) * binHz}};

    // 0.4 s of settle, then enough tone for two whole analysis windows plus the
    // longest transient the filter here can produce.
    const int settleBlocks = static_cast<int>(
        static_cast<std::size_t>(config.inputSampleRate) * 2 / 5 / inBlock) + 1;
    const int toneBlocks =
        static_cast<int>((2 * windowOut + 32768 + outBlock - 1) / outBlock);
    const int rate = config.inputSampleRate;
    const Capture capture = captureChannel(*channel, inBlock, primed, settleBlocks,
        toneBlocks,
        [&tones, rate](std::span<float> i, std::span<float> q, std::size_t offset) {
            fillComb(i, q, tones, offset, rate, 0.05);
        });
    if (!capture.ok || capture.samples.size() < 2 * windowOut) {
        std::cerr << "FAIL: phase capture failed for " << label << '\n';
        return result;
    }
    const std::size_t from = windowOut;   // an exact multiple of W: see the header
    const std::complex<double> atBase =
        binPhasor(capture.samples, from, base, windowOut);
    const std::complex<double> atFine =
        binPhasor(capture.samples, from, base + kFineOffset, windowOut);
    const std::complex<double> atCoarse =
        binPhasor(capture.samples, from, base + kCoarseOffset, windowOut);
    const double toInput = static_cast<double>(config.inputSampleRate) /
                           static_cast<double>(config.outputSampleRate);
    const double fine = delayFromPhase(std::arg(atFine) - std::arg(atBase),
                                       kFineOffset, windowOut,
                                       std::numeric_limits<double>::quiet_NaN());
    result.fineSamples = fine * toInput;
    result.refinedSamples = delayFromPhase(std::arg(atCoarse) - std::arg(atBase),
                                           kCoarseOffset, windowOut, fine) * toInput;
    result.levelDb = 20.0 * std::log10(std::max(std::abs(atBase), 1.0e-30));
    result.ok = true;
    return result;
}

// The whole-band sweep, from one capture. Only ever run at 48 kHz / 256.
struct CombResult
{
    bool ok = false;
    std::vector<PhaseDelay> points;
    std::vector<double> centreHz;
    double deadBinDb = 0.0;
    double referenceLevelDb = 0.0;
};

CombResult measureComb(const WdspChannel::Config& config, const char* label)
{
    CombResult result;
    std::unique_ptr<WdspChannel> channel = openChannel(config, label);
    if (!channel) {
        return result;
    }
    std::vector<CombTone> tones;
    for (const double centre : kProbeCentres) {
        const int base = static_cast<int>(std::lround(centre / kBinHz));
        for (const int offset : {0, kFineOffset, kCoarseOffset}) {
            tones.push_back({base + offset, (base + offset) * kBinHz});
        }
    }
    const Capture capture = captureChannel(*channel, kBlock, true, 120, 220,
        [&tones](std::span<float> i, std::span<float> q, std::size_t offset) {
            fillComb(i, q, tones, offset, kSampleRate, kCombAmplitude);
        });
    if (!capture.ok || capture.samples.size() < 2 * kAnalysisWindow) {
        std::cerr << "FAIL: comb capture failed for " << label << '\n';
        return result;
    }
    const std::size_t from = kAnalysisWindow;
    double referenceLevel = 0.0;
    for (const double centre : kProbeCentres) {
        const int base = static_cast<int>(std::lround(centre / kBinHz));
        const std::complex<double> atBase =
            binPhasor(capture.samples, from, base, kAnalysisWindow);
        const std::complex<double> atFine =
            binPhasor(capture.samples, from, base + kFineOffset, kAnalysisWindow);
        const std::complex<double> atCoarse =
            binPhasor(capture.samples, from, base + kCoarseOffset, kAnalysisWindow);
        PhaseDelay point;
        const double fine = delayFromPhase(std::arg(atFine) - std::arg(atBase),
                                           kFineOffset, kAnalysisWindow,
                                           std::numeric_limits<double>::quiet_NaN());
        point.fineSamples = fine;
        point.refinedSamples = delayFromPhase(std::arg(atCoarse) - std::arg(atBase),
                                              kCoarseOffset, kAnalysisWindow, fine);
        point.levelDb = 20.0 * std::log10(std::max(std::abs(atBase), 1.0e-30));
        point.ok = true;
        referenceLevel = std::max(referenceLevel, std::abs(atBase));
        result.points.push_back(point);
        result.centreHz.push_back(base * kBinHz);
    }
    const std::complex<double> dead =
        binPhasor(capture.samples, from, kDeadBin, kAnalysisWindow);
    result.deadBinDb = 20.0 * std::log10(std::max(std::abs(dead), 1.0e-30));
    result.referenceLevelDb = 20.0 * std::log10(std::max(referenceLevel, 1.0e-30));
    result.ok = true;
    return result;
}

// ── onset ───────────────────────────────────────────────────────────────────

// 481 samples, 10.0 ms. Wide enough that the 2f image the demodulation leaves
// is suppressed even at the 200 Hz probe (two full periods), and the SAME width
// at every frequency, which is the point.
constexpr long kOnsetHalfWidth = 240;

void fillTone(std::span<float> i, std::span<float> q, double toneHz,
              std::size_t offset, double amplitude, int rate)
{
    for (std::size_t sample = 0; sample < i.size(); ++sample) {
        const double phase = kBasebandSign * 2.0 * kPi * toneHz *
                             static_cast<double>(offset + sample) /
                             static_cast<double>(rate);
        i[sample] = static_cast<float>(amplitude * std::cos(phase));
        q[sample] = static_cast<float>(amplitude * std::sin(phase));
    }
}

struct OnsetPoint
{
    bool ok = false;
    double onsetSamples = 0.0;
    double steady = 0.0;
};

OnsetPoint measureOnset(const WdspChannel::Config& config, double toneHz,
                        bool primed, const char* label)
{
    OnsetPoint point;
    std::unique_ptr<WdspChannel> channel = openChannel(config, label);
    if (!channel) {
        return point;
    }
    const Capture capture = captureChannel(*channel, kBlock, primed, 120, 240,
        [toneHz](std::span<float> i, std::span<float> q, std::size_t offset) {
            fillTone(i, q, toneHz, offset, 0.1, kSampleRate);
        });
    if (!capture.ok) {
        return point;
    }
    const std::vector<double> envelope =
        demodEnvelope(capture.samples, toneHz, kOnsetHalfWidth, kSampleRate);
    const double steady = steadyLevel(envelope, kOnsetHalfWidth);
    if (steady <= 0.0) {
        return point;
    }
    const double onset = risingCrossing(envelope, steady * 0.5);
    if (onset < 0.0) {
        return point;
    }
    point.ok = true;
    point.onsetSamples = onset;
    point.steady = steady;
    return point;
}

// ── keying envelope ─────────────────────────────────────────────────────────
//
// A single isolated key-down with a long silence after it, so the filter's
// whole impulse response plays out before anything else happens. 8192 taps is
// 170 ms of impulse response; a repeated dot pattern at any realistic speed
// would overlap successive responses and every edge measured would be the sum
// of two events.

constexpr double kCwToneHz = 700.0;
// 137 samples, 2.85 ms. Almost exactly two periods of 700 Hz, which nulls the
// 2f image the demodulation leaves, and short enough that the estimator's own
// smear on a CW edge is small -- the INPUT row prices what is left.
constexpr long kKeyHalfWidth = 68;
// Silence before the key-down. Without it the centred envelope window straddles
// the start of the buffer, envelope[0] is already above every threshold, and
// the leading-edge crossings come back negative -- which is exactly how the
// INPUT control row failed on the first run of this section. It also gives the
// pre-ringing somewhere to be.
constexpr double kKeyLeadInMs = 100.0;

struct KeyingShape
{
    const char* name = "";
    double keyDownMs = 0.0;
    double rampMs = 0.0;   // 0 = hard gate
};

void fillGatedTone(std::span<float> i, std::span<float> q, std::size_t offset,
                   const KeyingShape& shape)
{
    const double keyDownSamples = shape.keyDownMs * kSampleRate / 1000.0;
    const double rampSamples = shape.rampMs * kSampleRate / 1000.0;
    const double leadIn = kKeyLeadInMs * kSampleRate / 1000.0;
    fillTone(i, q, kCwToneHz, offset, 0.1, kSampleRate);
    for (std::size_t sample = 0; sample < i.size(); ++sample) {
        const double n = static_cast<double>(offset + sample) - leadIn;
        double gate = 0.0;
        if (n >= 0.0 && n < keyDownSamples) {
            gate = 1.0;
            if (rampSamples > 0.0) {
                if (n < rampSamples) {
                    gate = 0.5 - 0.5 * std::cos(kPi * n / rampSamples);
                } else if (n > keyDownSamples - rampSamples) {
                    gate = 0.5 - 0.5 *
                           std::cos(kPi * (keyDownSamples - n) / rampSamples);
                }
            }
        }
        i[sample] = static_cast<float>(i[sample] * gate);
        q[sample] = static_cast<float>(q[sample] * gate);
    }
}

struct KeyingResult
{
    bool ok = false;
    double peakOverSteady = 0.0;      // global peak / continuous key-down level
    double plateauOverSteady = 0.0;   // settled level / continuous key-down level
    double riseMs = 0.0;            // 10 % -> 90 % of the pulse's own peak
    double fallMs = 0.0;            // 90 % -> 10 %
    double footMs = 0.0;            // 1 % -> 50 % on the leading edge (pre-ring)
    double tailMs = 0.0;            // 50 % -> 1 % on the trailing edge
    double overshootPct = 0.0;
    // Envelope at a FIXED time after the 50 % fall crossing, in dB below the
    // pulse peak. The first version of this took the largest envelope after the
    // 10 % crossing, which can only ever be the sample immediately after that
    // crossing -- it read 9.5-9.8 % for every configuration, including ones
    // with no filter at all, because it was reporting its own threshold back.
    // A fixed offset measures ringing; a threshold-relative maximum measures
    // the threshold.
    double ring10Db = 0.0;
    double ring30Db = 0.0;
    // Set when the key-down is too short for a settled level to exist at all --
    // the middle half of it is narrower than two envelope windows. rise, fall
    // and over are then fitted to a transient rather than to a plateau and mean
    // nothing; the row is printed with a * rather than silently trusted.
    bool noPlateau = false;
};

constexpr double kRingProbeMs[] = {10.0, 30.0};

// EDGES ARE MEASURED AGAINST THE PLATEAU, NOT AGAINST THE GLOBAL PEAK, and the
// first version of this got it wrong in a way that produced confident nonsense.
// A narrow filter overshoots hard on the leading edge -- 19 % above the settled
// level for a 300 Hz CW filter in minimum phase -- so "first drop below 90 % of
// the global peak" fires a few ms into the dot, when the overshoot decays into
// the plateau, and the 90 %-to-10 % "fall time" then spans the whole dot. It
// printed 55 ms of fall on a 60 ms dot while the 50 %-to-1 % tail printed 3 ms,
// which is the contradiction that gave it away.
//
// So: find the settled level over the middle half of the key-down as it appears
// at the output, and take every 1/10/50/90 % crossing against THAT. The global
// peak is still reported, as overshoot.
KeyingResult analyseKeying(std::span<const float> audio, double steadyReference,
                           const KeyingShape& shape)
{
    KeyingResult result;
    const std::vector<double> envelope =
        demodEnvelope(audio, kCwToneHz, kKeyHalfWidth, kSampleRate);
    const auto peakIt = std::max_element(envelope.begin(), envelope.end());
    const double globalPeak = *peakIt;
    if (globalPeak <= 0.0 || steadyReference <= 0.0) {
        return result;
    }
    const double keyDownSamples = shape.keyDownMs * kSampleRate / 1000.0;
    // A first, rough leading edge, only to locate the key-down in the output.
    const double roughEdge = risingCrossing(envelope, 0.5 * globalPeak);
    if (roughEdge < 0.0) {
        return result;
    }
    const std::size_t plateauFrom = static_cast<std::size_t>(
        roughEdge + 0.25 * keyDownSamples);
    const std::size_t plateauTo = std::min<std::size_t>(
        envelope.size(), static_cast<std::size_t>(roughEdge + 0.75 * keyDownSamples) + 1);
    if (plateauFrom >= plateauTo) {
        return result;
    }
    std::vector<double> middle(envelope.begin() + static_cast<long>(plateauFrom),
                               envelope.begin() + static_cast<long>(plateauTo));
    std::sort(middle.begin(), middle.end());
    // Median, so a ripple peak or a single overshoot sample inside the window
    // cannot become "the plateau".
    const double plateau = middle[middle.size() / 2];
    if (plateau <= 0.0) {
        return result;
    }
    result.noPlateau = (plateauTo - plateauFrom) <
                       2 * static_cast<std::size_t>(2 * kKeyHalfWidth + 1);

    const double t01 = risingCrossing(envelope, 0.01 * plateau);
    const double t10 = risingCrossing(envelope, 0.10 * plateau);
    const double t50 = risingCrossing(envelope, 0.50 * plateau);
    const double t90 = risingCrossing(envelope, 0.90 * plateau);
    // The trailing edge is searched from the END of the plateau window, so a
    // leading-edge overshoot cannot be mistaken for it.
    std::size_t scan = plateauTo;
    auto fallingCrossing = [&](double level) -> double {
        for (std::size_t i = scan + 1; i < envelope.size(); ++i) {
            if (envelope[i] <= level) {
                scan = i;
                const double previous = envelope[i - 1];
                if (previous == envelope[i]) {
                    return static_cast<double>(i);
                }
                return static_cast<double>(i - 1) +
                       (previous - level) / (previous - envelope[i]);
            }
        }
        return -1.0;
    };
    const double f90 = fallingCrossing(0.90 * plateau);
    const double f50 = fallingCrossing(0.50 * plateau);
    const double f10 = fallingCrossing(0.10 * plateau);
    const double f01 = fallingCrossing(0.01 * plateau);
    if (t01 < 0.0 || t10 < 0.0 || t50 < 0.0 || t90 < 0.0 ||
        f90 < 0.0 || f50 < 0.0 || f10 < 0.0 || f01 < 0.0) {
        return result;
    }
    result.peakOverSteady = globalPeak / steadyReference;
    result.plateauOverSteady = plateau / steadyReference;
    result.riseMs = samplesToMs(t90 - t10);
    result.fallMs = samplesToMs(f10 - f90);
    result.footMs = samplesToMs(t50 - t01);
    result.tailMs = samplesToMs(f01 - f50);
    result.overshootPct = 100.0 * (globalPeak / plateau - 1.0);
    auto ringAt = [&](double afterMs) {
        const std::size_t at = static_cast<std::size_t>(
            f50 + afterMs * kSampleRate / 1000.0);
        const double level = at < envelope.size() ? envelope[at] : 0.0;
        return 20.0 * std::log10(std::max(level, 1.0e-30) / plateau);
    };
    result.ring10Db = ringAt(kRingProbeMs[0]);
    result.ring30Db = ringAt(kRingProbeMs[1]);
    result.ok = true;
    return result;
}

KeyingResult measureKeying(const WdspChannel::Config& config,
                           const KeyingShape& shape, double steadyReference,
                           const char* label)
{
    std::unique_ptr<WdspChannel> channel = openChannel(config, label);
    if (!channel) {
        return {};
    }
    const Capture capture = captureChannel(*channel, kBlock, true, 120, 240,
        [&shape](std::span<float> i, std::span<float> q, std::size_t offset) {
            fillGatedTone(i, q, offset, shape);
        });
    if (!capture.ok) {
        return {};
    }
    return analyseKeying(capture.samples, steadyReference, shape);
}

// THE ESTIMATOR FLOOR. The same gate, the same analysis, no channel at all: the
// real part of the complex gated tone is what a perfect wire would deliver.
// Every figure in the (A3) table has to be read against this row, because a
// 2.9 ms rise time means nothing until you know an undistorted gate also reads
// 2.9 ms through the same 137-sample window.
KeyingResult measureKeyingFloor(const KeyingShape& shape, double& steadyOut)
{
    constexpr int kBlocks = 240;
    std::vector<float> audio;
    std::vector<float> continuous;
    std::vector<float> i(kBlock);
    std::vector<float> q(kBlock);
    for (int block = 0; block < kBlocks; ++block) {
        fillGatedTone(i, q, static_cast<std::size_t>(block) * kBlock, shape);
        audio.insert(audio.end(), i.begin(), i.end());
    }
    for (int block = 0; block < kBlocks; ++block) {
        fillTone(i, q, kCwToneHz, static_cast<std::size_t>(block) * kBlock, 0.1,
                 kSampleRate);
        continuous.insert(continuous.end(), i.begin(), i.end());
    }
    steadyOut = steadyLevel(
        demodEnvelope(continuous, kCwToneHz, kKeyHalfWidth, kSampleRate),
        kKeyHalfWidth);
    return analyseKeying(audio, steadyOut, shape);
}

double measureSteadyLevel(const WdspChannel::Config& config, double toneHz,
                          long halfWidth, const char* label)
{
    std::unique_ptr<WdspChannel> channel = openChannel(config, label);
    if (!channel) {
        return 0.0;
    }
    const Capture capture = captureChannel(*channel, kBlock, true, 120, 240,
        [toneHz](std::span<float> i, std::span<float> q, std::size_t offset) {
            fillTone(i, q, toneHz, offset, 0.1, kSampleRate);
        });
    if (!capture.ok) {
        return 0.0;
    }
    return steadyLevel(demodEnvelope(capture.samples, toneHz, halfWidth, kSampleRate),
                       halfWidth);
}

// ── (B) the residue ─────────────────────────────────────────────────────────

struct ResidueRow
{
    bool ok = false;
    double onsetSamples = 0.0;
    double residueSamples = 0.0;
};

// The onset-based residue, generalised over block size and rates, reported in
// INPUT samples. This one CARRIES the mute ramp (when unprimed) and the
// filter's own ring-up, so it is an upper bound; the phase-based residue beside
// it is the clean number.
ResidueRow measureOnsetResidue(const WdspChannel::Config& config, double toneHz,
                               bool primed, const char* label)
{
    ResidueRow row;
    std::string error;
    std::unique_ptr<WdspChannel> channel = WdspChannel::create(config, &error);
    if (!channel) {
        std::cerr << "SKIP: " << label << " would not open: " << error << '\n';
        return row;
    }
    const std::size_t inBlock = config.inputBlockSize;
    // Fixed WALL time, not a fixed block count, so changing the block size does
    // not change how long anything gets to settle.
    const int settleBlocks = static_cast<int>(
        (static_cast<std::size_t>(config.inputSampleRate) * 3 / 4) / inBlock) + 1;
    const int toneBlocks = static_cast<int>(
        (static_cast<std::size_t>(config.inputSampleRate) * 5 / 4) / inBlock) + 1;
    const int rate = config.inputSampleRate;
    const Capture capture = captureChannel(*channel, inBlock, primed, settleBlocks,
        toneBlocks,
        [toneHz, rate](std::span<float> i, std::span<float> q, std::size_t offset) {
            fillTone(i, q, toneHz, offset, 0.1, rate);
        });
    if (!capture.ok) {
        return row;
    }
    // The boxcar is scaled to the OUTPUT rate so it always spans the same
    // 10 ms: an estimator whose smoothing length changed with the rate would
    // move the crossing on its own.
    const long half = static_cast<long>(std::lround(
        static_cast<double>(kOnsetHalfWidth) * config.outputSampleRate / kSampleRate));
    const std::vector<double> envelope =
        demodEnvelope(capture.samples, toneHz, half, config.outputSampleRate);
    const double steady = steadyLevel(envelope, half);
    if (steady <= 0.0) {
        return row;
    }
    const double crossing = risingCrossing(envelope, steady * 0.5);
    if (crossing < 0.0) {
        return row;
    }
    const double inputSamples = crossing * config.inputSampleRate /
                                static_cast<double>(config.outputSampleRate);
    const double firTerm = 0.5 * static_cast<double>(config.filterTaps - 1) *
                           config.inputSampleRate /
                           static_cast<double>(config.dspSampleRate);
    row.ok = true;
    row.onsetSamples = inputSamples;
    row.residueSamples = inputSamples - firTerm;
    return row;
}

} // namespace

int main()
{
    std::printf("wdsp_phase_distortion_test (d109): what minimum phase costs in\n"
                "the passband, and what composes the constant channel latency\n");
    std::printf("conditions: %d Hz in/dsp/out, block %zu, USB 150-3000 Hz, AGC OFF\n"
                "at 0 dB, blockForOutput, mute envelope at the production default\n"
                "but PRIMED -- see the header, a silent settle does not advance\n"
                "upslew2 and d107's does not settle anything\n", kSampleRate, kBlock);
    std::printf("analysis window %zu smp, bin spacing %.6f Hz, group-delay\n"
                "baselines %.3f Hz (coarse) / %.3f Hz (refined)\n\n",
                kAnalysisWindow, kBinHz, kBinHz * kFineOffset, kBinHz * kCoarseOffset);

    WdspChannel::Config linear = baseConfig();
    linear.filterTaps = 8192;
    WdspChannel::Config minimum = baseConfig();
    minimum.filterTaps = 8192;
    minimum.minimumPhase = true;
    WdspChannel::Config shortLinear = baseConfig();
    shortLinear.filterTaps = 2048;

    // ── (A1) ────────────────────────────────────────────────────────────────
    std::printf("(A1) GROUP DELAY vs FREQUENCY, from steady-state phase\n");
    const CombResult linearComb = measureComb(linear, "8192 taps, linear phase");
    const CombResult minimumComb = measureComb(minimum, "8192 taps, minimum phase");
    const CombResult shortComb = measureComb(shortLinear, "2048 taps, linear phase");
    if (!require(linearComb.ok && minimumComb.ok && shortComb.ok,
                 "(A1) comb measurement did not run")) {
        return 1;
    }
    std::printf("  CONTROL, dead bin %d (%.2f Hz, no tone injected):\n"
                "    8192 lp %+8.2f dB   8192 mp %+8.2f dB   2048 lp %+8.2f dB\n"
                "    strongest tone bin  %+8.2f       %+8.2f          %+8.2f dB\n"
                "    A small gap would mean the comb is driving something nonlinear,\n"
                "    superposition is false, and every row below is void.\n\n",
                kDeadBin, kDeadBin * kBinHz, linearComb.deadBinDb,
                minimumComb.deadBinDb, shortComb.deadBinDb,
                linearComb.referenceLevelDb, minimumComb.referenceLevelDb,
                shortComb.referenceLevelDb);
    std::printf("  %9s | %-26s | %-26s | %-20s\n", "freq Hz",
                "8192 linear phase", "8192 minimum phase", "2048 linear phase");
    std::printf("  %9s | %11s %8s %5s | %11s %8s %5s | %11s %8s\n", "",
                "delay smp", "ms", "dB", "delay smp", "ms", "dB", "delay smp", "ms");
    for (std::size_t i = 0; i < kProbeCount; ++i) {
        std::printf("  %9.2f | %11.1f %8.3f %5.1f | %11.1f %8.3f %5.1f | "
                    "%11.1f %8.3f\n",
                    linearComb.centreHz[i],
                    linearComb.points[i].refinedSamples,
                    samplesToMs(linearComb.points[i].refinedSamples),
                    linearComb.points[i].levelDb,
                    minimumComb.points[i].refinedSamples,
                    samplesToMs(minimumComb.points[i].refinedSamples),
                    minimumComb.points[i].levelDb,
                    shortComb.points[i].refinedSamples,
                    samplesToMs(shortComb.points[i].refinedSamples));
    }
    auto spread = [](const CombResult& comb, double loHz, double hiHz) {
        double lo = std::numeric_limits<double>::max();
        double hi = std::numeric_limits<double>::lowest();
        for (std::size_t i = 0; i < comb.points.size(); ++i) {
            if (comb.centreHz[i] < loHz || comb.centreHz[i] > hiHz) {
                continue;
            }
            lo = std::min(lo, comb.points[i].refinedSamples);
            hi = std::max(hi, comb.points[i].refinedSamples);
        }
        return hi - lo;
    };
    struct Band { const char* name; double lo; double hi; };
    constexpr Band kBands[] = {
        {"whole measured band 200-2900 Hz", 0.0, 1.0e9},
        {"SSB/data core     400-2400 Hz", 400.0, 2400.0},
        {"CW window          600-1000 Hz", 600.0, 1000.0},
    };
    std::printf("\n  GROUP DELAY SPREAD (max - min across the band)\n");
    for (const Band& band : kBands) {
        std::printf("    %-32s 8192 lp %7.1f smp %7.3f ms |"
                    " 8192 mp %7.1f smp %7.3f ms | 2048 lp %7.1f smp %7.3f ms\n",
                    band.name,
                    spread(linearComb, band.lo, band.hi),
                    samplesToMs(spread(linearComb, band.lo, band.hi)),
                    spread(minimumComb, band.lo, band.hi),
                    samplesToMs(spread(minimumComb, band.lo, band.hi)),
                    spread(shortComb, band.lo, band.hi),
                    samplesToMs(spread(shortComb, band.lo, band.hi)));
    }
    std::printf("  The two linear-phase columns are the CONTROL and must read 0.\n");

    std::printf("\n  CONTROL, comb vs a single isolated tone (must agree)\n");
    for (const double check : {800.0, 1500.0}) {
        const PhaseDelay singleLinear =
            measurePhaseDelay(linear, check, true, "single tone, linear");
        const PhaseDelay singleMinimum =
            measurePhaseDelay(minimum, check, true, "single tone, minimum");
        double combLinear = 0.0;
        double combMinimum = 0.0;
        for (std::size_t i = 0; i < kProbeCount; ++i) {
            if (std::fabs(linearComb.centreHz[i] - check) < 2.0) {
                combLinear = linearComb.points[i].refinedSamples;
                combMinimum = minimumComb.points[i].refinedSamples;
            }
        }
        std::printf("    %4.0f Hz   lp comb %8.1f  single %8.1f  (diff %5.1f)"
                    "   mp comb %8.1f  single %8.1f  (diff %5.1f)\n",
                    check, combLinear, singleLinear.refinedSamples,
                    singleLinear.refinedSamples - combLinear,
                    combMinimum, singleMinimum.refinedSamples,
                    singleMinimum.refinedSamples - combMinimum);
    }

    // ── (A2) ────────────────────────────────────────────────────────────────
    std::printf("\n(A2) ONSET PER TONE (50%% of steady, demodulated envelope, the\n"
                "     same %ld-sample centred window at every frequency)\n",
                2 * kOnsetHalfWidth + 1);
    std::printf("  %8s | %-24s | %-24s | %9s\n", "freq Hz",
                "8192 linear phase", "8192 minimum phase", "mp - lp");
    constexpr double kOnsetTones[] = {200.0, 400.0, 800.0, 1500.0, 2400.0};
    bool onsetOk = true;
    for (const double tone : kOnsetTones) {
        const OnsetPoint lp = measureOnset(linear, tone, true, "onset, linear");
        const OnsetPoint mp = measureOnset(minimum, tone, true, "onset, minimum");
        if (!lp.ok || !mp.ok) {
            onsetOk = false;
            std::printf("  %8.0f | measurement failed\n", tone);
            continue;
        }
        std::printf("  %8.0f | %10.1f smp %8.2f ms | %10.1f smp %8.2f ms | "
                    "%7.2f ms\n",
                    tone, lp.onsetSamples, samplesToMs(lp.onsetSamples),
                    mp.onsetSamples, samplesToMs(mp.onsetSamples),
                    samplesToMs(mp.onsetSamples - lp.onsetSamples));
    }
    if (!require(onsetOk, "(A2) at least one onset did not measure")) {
        return 1;
    }

    // ── (A3) ────────────────────────────────────────────────────────────────
    std::printf("\n(A3) KEYING ENVELOPE, single isolated key-down at %.0f Hz,\n"
                "     envelope window %ld smp (%.2f ms)\n",
                kCwToneHz, 2 * kKeyHalfWidth + 1,
                samplesToMs(static_cast<double>(2 * kKeyHalfWidth + 1)));
    const double steadyLinear =
        measureSteadyLevel(linear, kCwToneHz, kKeyHalfWidth, "cw steady lp");
    const double steadyMinimum =
        measureSteadyLevel(minimum, kCwToneHz, kKeyHalfWidth, "cw steady mp");
    const double steadyShort =
        measureSteadyLevel(shortLinear, kCwToneHz, kKeyHalfWidth, "cw steady 2048");
    if (!require(steadyLinear > 0.0 && steadyMinimum > 0.0 && steadyShort > 0.0,
                 "(A3) continuous-key-down reference did not measure")) {
        return 1;
    }
    std::printf("  continuous key-down reference: 8192 lp %.6f   8192 mp %.6f   "
                "2048 lp %.6f\n", steadyLinear, steadyMinimum, steadyShort);
    constexpr KeyingShape kShapes[] = {
        {"60 ms dot, 5 ms raised cosine", 60.0, 5.0},
        {"60 ms dot, hard gate",          60.0, 0.0},
        {"20 ms dot (60 WPM), hard gate", 20.0, 0.0},
        {"6 ms dot, hard gate",            6.0, 0.0},
    };
    struct KeyRow { const char* name; const WdspChannel::Config* config; double steady; };
    bool keyingOk = true;
    auto keyingTable = [&keyingOk](std::span<const KeyingShape> shapes,
                                   std::span<const KeyRow> rows) {
        std::printf("  %-31s %-8s %8s %8s %8s %8s %8s %7s %9s %9s\n",
                    "shape", "config", "plat/ss", "rise ms", "fall ms", "foot ms",
                    "tail ms", "over %", "ring+10dB", "ring+30dB");
        for (const KeyingShape& shape : shapes) {
            double floorSteady = 0.0;
            const KeyingResult floorResult = measureKeyingFloor(shape, floorSteady);
            if (floorResult.ok) {
                std::printf("  %-31s %-8s %8.4f %8.3f %8.3f %8.3f %8.3f %7.2f %9.1f %9.1f\n",
                            shape.name, floorResult.noPlateau ? "INPUT *" : "INPUT",
                            floorResult.plateauOverSteady,
                            floorResult.riseMs, floorResult.fallMs, floorResult.footMs,
                            floorResult.tailMs, floorResult.overshootPct,
                            floorResult.ring10Db, floorResult.ring30Db);
            } else {
                keyingOk = false;
                std::printf("  %-31s %-8s  estimator floor failed\n", shape.name, "INPUT");
            }
            for (const KeyRow& row : rows) {
                const KeyingResult result =
                    measureKeying(*row.config, shape, row.steady, row.name);
                if (!result.ok) {
                    keyingOk = false;
                    std::printf("  %-31s %-8s  measurement failed\n", shape.name, row.name);
                    continue;
                }
                std::printf("  %-31s %-8s %8.4f %8.3f %8.3f %8.3f %8.3f %7.2f %9.1f %9.1f\n",
                            shape.name,
                            (std::string(row.name) + (result.noPlateau ? " *" : "")).c_str(),
                            result.plateauOverSteady, result.riseMs,
                            result.fallMs, result.footMs, result.tailMs,
                            result.overshootPct, result.ring10Db, result.ring30Db);
            }
        }
    };
    const KeyRow kWideRows[] = {{"2048 lp", &shortLinear, steadyShort},
                                {"8192 lp", &linear, steadyLinear},
                                {"8192 mp", &minimum, steadyMinimum}};
    keyingTable(kShapes, kWideRows);
    if (!require(keyingOk, "(A3) at least one keying envelope did not measure")) {
        return 1;
    }
    std::printf("\n  INPUT   = the gate itself through the same estimator, no channel.\n"
                "            Everything below it is the channel's EXCESS over this.\n"
                "  plat/ss = the dot's SETTLED level / the continuous key-down level. Below\n"
                "            1 means the dot never reaches full amplitude. Measured as the\n"
                "            median over the middle half of the key-down, so a leading-edge\n"
                "            overshoot cannot masquerade as the plateau -- that mistake is\n"
                "            what produced a 55 ms fall time on a 60 ms dot in the first\n"
                "            version of this section.\n"
                "  rise    = 10%%-90%%, fall = 90%%-10%%, of the pulse's OWN peak.\n"
                "  foot    = 1%% -> 50%% on the leading edge, i.e. PRE-RINGING. A\n"
                "            linear-phase step response is symmetric, so energy arrives\n"
                "            BEFORE the key-down; minimum phase rings only causally and\n"
                "            should show a shorter foot. This is the one figure a CW\n"
                "            operator would describe as a soft or mushy edge.\n"
                "  tail    = 50%% -> 1%% after key-up (post-ringing).\n"
                "  over    = global peak over the plateau, per cent (leading-edge overshoot).\n"
                "  ring+N  = envelope N ms after the 50%% fall crossing, dB below the\n"
                "            pulse peak: what is still ringing when the next dot starts.\n"
                "  Every ratio is against EACH configuration's own continuous key-down\n"
                "  level, printed above the table.\n"
                "  *       = the key-down is too short for a settled level to exist through\n"
                "            this filter, so rise, fall and over are fitted to a transient.\n"
                "            Read plat/ss and the ring columns on those rows and nothing else.\n");

    // ── (A4) ────────────────────────────────────────────────────────────────
    //
    // THE CASE (A3) CANNOT SEE. (A3) runs the SSB passband d107 used, 150-3000
    // Hz, and a 700 Hz CW tone sits dead centre of it where the magnitude
    // response is flat and both phase modes are nearly transparent. That is not
    // the filter a CW operator uses. In a 300 Hz CW filter the dot's own
    // spectrum overlaps the skirts, which is precisely where linear and minimum
    // phase diverge -- a linear-phase skirt rings symmetrically about the edge
    // (energy BEFORE the key-down), a minimum-phase one only after it. #5578
    // carries the CW label, so this is the row that answers it.
    std::printf("\n(A4) NARROW CW FILTER: 550-850 Hz (a 300 Hz CW filter centred\n"
                "     on the %.0f Hz probe). This is where the two phase modes\n"
                "     are supposed to differ.\n", kCwToneHz);
    WdspChannel::Config cwShort = baseConfig();
    cwShort.filterTaps = 2048;
    cwShort.filterLowHz = 550.0;
    cwShort.filterHighHz = 850.0;
    WdspChannel::Config cwLinear = cwShort;
    cwLinear.filterTaps = 8192;
    WdspChannel::Config cwMinimum = cwLinear;
    cwMinimum.minimumPhase = true;

    std::printf("  group delay inside the CW filter, from steady-state phase\n");
    std::printf("  %8s | %11s %8s | %11s %8s | %11s %8s\n", "freq Hz",
                "2048 lp smp", "ms", "8192 lp smp", "ms", "8192 mp smp", "ms");
    constexpr double kCwProbes[] = {600.0, 650.0, 700.0, 750.0, 800.0};
    double cwSpread[3] = {0.0, 0.0, 0.0};
    double cwLo[3] = {1.0e18, 1.0e18, 1.0e18};
    double cwHi[3] = {-1.0e18, -1.0e18, -1.0e18};
    bool cwOk = true;
    for (const double probe : kCwProbes) {
        const PhaseDelay a = measurePhaseDelay(cwShort, probe, true, "cw 2048 lp");
        const PhaseDelay b = measurePhaseDelay(cwLinear, probe, true, "cw 8192 lp");
        const PhaseDelay c = measurePhaseDelay(cwMinimum, probe, true, "cw 8192 mp");
        if (!a.ok || !b.ok || !c.ok) {
            cwOk = false;
            std::printf("  %8.0f | measurement failed\n", probe);
            continue;
        }
        const double values[3] = {a.refinedSamples, b.refinedSamples, c.refinedSamples};
        for (int k = 0; k < 3; ++k) {
            cwLo[k] = std::min(cwLo[k], values[k]);
            cwHi[k] = std::max(cwHi[k], values[k]);
        }
        std::printf("  %8.0f | %11.1f %8.3f | %11.1f %8.3f | %11.1f %8.3f\n",
                    probe, values[0], samplesToMs(values[0]),
                    values[1], samplesToMs(values[1]),
                    values[2], samplesToMs(values[2]));
    }
    if (!require(cwOk, "(A4) a group-delay point did not measure")) {
        return 1;
    }
    for (int k = 0; k < 3; ++k) {
        cwSpread[k] = cwHi[k] - cwLo[k];
    }
    std::printf("  SPREAD 600-800 Hz:  2048 lp %7.1f smp %7.3f ms | "
                "8192 lp %7.1f smp %7.3f ms | 8192 mp %7.1f smp %7.3f ms\n",
                cwSpread[0], samplesToMs(cwSpread[0]),
                cwSpread[1], samplesToMs(cwSpread[1]),
                cwSpread[2], samplesToMs(cwSpread[2]));
    std::printf("  The linear-phase columns are again the CONTROL and must read 0.\n");

    const double cwSteadyShort =
        measureSteadyLevel(cwShort, kCwToneHz, kKeyHalfWidth, "cw narrow steady 2048");
    const double cwSteadyLinear =
        measureSteadyLevel(cwLinear, kCwToneHz, kKeyHalfWidth, "cw narrow steady lp");
    const double cwSteadyMinimum =
        measureSteadyLevel(cwMinimum, kCwToneHz, kKeyHalfWidth, "cw narrow steady mp");
    if (!require(cwSteadyShort > 0.0 && cwSteadyLinear > 0.0 && cwSteadyMinimum > 0.0,
                 "(A4) continuous reference did not measure")) {
        return 1;
    }
    std::printf("\n  keying envelope through the 300 Hz CW filter\n"
                "  continuous key-down reference: 2048 lp %.6f   8192 lp %.6f   "
                "8192 mp %.6f\n", cwSteadyShort, cwSteadyLinear, cwSteadyMinimum);
    const KeyRow kCwRows[] = {{"2048 lp", &cwShort, cwSteadyShort},
                              {"8192 lp", &cwLinear, cwSteadyLinear},
                              {"8192 mp", &cwMinimum, cwSteadyMinimum}};
    keyingTable(kShapes, kCwRows);
    if (!require(keyingOk, "(A4) at least one keying envelope did not measure")) {
        return 1;
    }

    // ── (B) ─────────────────────────────────────────────────────────────────
    std::printf("\n(B) WHAT COMPOSES THE CONSTANT RESIDUE\n");
    std::printf("  Two independent estimators of the same quantity, both in INPUT\n"
                "  samples, both with the FIR term (taps-1)/2 subtracted:\n"
                "    phase = steady-state group delay. Carries NO mute ramp and NO\n"
                "            filter ring-up. This is the channel's true pipeline latency.\n"
                "    onset = 50%%-of-steady arrival. Carries both, so it is an UPPER\n"
                "            bound, and the gap between the columns is itself a\n"
                "            measurement.\n\n");

    struct Variant
    {
        std::string label;
        WdspChannel::Config config;
        bool primed = true;
    };
    std::vector<Variant> variants;
    auto add = [&variants](std::string label, const WdspChannel::Config& config,
                           bool primed = true) {
        variants.push_back({std::move(label), config, primed});
    };

    // B0: five tap lengths instead of d107's two, so the "constant" is fitted
    // rather than assumed.
    for (const int taps : {1024, 2048, 4096, 8192, 16384}) {
        WdspChannel::Config config = baseConfig();
        config.filterTaps = taps;
        add("B0 taps=" + std::to_string(taps), config);
    }
    // B1: block size. Ring buffering in iobuffs.c is the prime suspect, and a
    // term of the form k*blockSize moves linearly with this.
    for (const std::size_t block : {std::size_t {64}, std::size_t {128},
                                    std::size_t {512}, std::size_t {1024}}) {
        WdspChannel::Config config = baseConfig();
        config.filterTaps = 8192;
        config.inputBlockSize = block;
        config.dspBlockSize = block;
        add("B1 block=" + std::to_string(block), config);
    }
    // B2: the host-facing block and WDSP's internal one moved INDEPENDENTLY,
    // which is what separates the r2 pre-fill from the worker's input lag.
    for (const auto pair : {std::pair<std::size_t, std::size_t> {256, 512},
                            std::pair<std::size_t, std::size_t> {512, 256},
                            std::pair<std::size_t, std::size_t> {256, 1024},
                            std::pair<std::size_t, std::size_t> {1024, 256}}) {
        WdspChannel::Config config = baseConfig();
        config.filterTaps = 8192;
        config.inputBlockSize = pair.first;
        config.dspBlockSize = pair.second;
        add("B2 in=" + std::to_string(pair.first) +
            " dsp=" + std::to_string(pair.second), config);
    }
    // B3: output rate, which is what would engage RXA_RSMP. Reported in input
    // samples, so a term that is really a fixed count of OUTPUT samples shows.
    for (const int outRate : {24000, 96000}) {
        WdspChannel::Config config = baseConfig();
        config.filterTaps = 8192;
        config.outputSampleRate = outRate;
        add("B3 outRate=" + std::to_string(outRate), config);
    }
    // B4: the whole channel at double rate. A term that is a fixed number of
    // SAMPLES stays put; a term that is a fixed TIME doubles.
    {
        WdspChannel::Config config = baseConfig();
        config.filterTaps = 8192;
        config.inputSampleRate = 96000;
        config.dspSampleRate = 96000;
        config.outputSampleRate = 96000;
        add("B4 all rates 96 kHz", config);
    }
    // B5: the mute envelope, four ways. The unprimed row is d107's own
    // condition; the difference between it and B0 taps=8192 is the ramp, and
    // the two scaled rows say whether it scales the way create_slews says.
    {
        WdspChannel::Config config = baseConfig();
        config.filterTaps = 8192;
        add("B5 UNPRIMED (d107 method)", config, false);
    }
    {
        WdspChannel::Config config = baseConfig();
        config.filterTaps = 8192;
        config.muteDelayUpSec = 0.0;
        config.muteSlewUpSec = 0.0;
        add("B5 ramp off, unprimed", config, false);
    }
    {
        WdspChannel::Config config = baseConfig();
        config.filterTaps = 8192;
        config.muteDelayUpSec = 0.020;
        config.muteSlewUpSec = 0.025;
        add("B5 delayUp 20 ms, unprimed", config, false);
    }
    {
        WdspChannel::Config config = baseConfig();
        config.filterTaps = 8192;
        config.muteDelayUpSec = 0.010;
        config.muteSlewUpSec = 0.050;
        add("B5 slewUp 50 ms, unprimed", config, false);
    }
    // B6: passband width. The FIR term does not depend on it, so a residue that
    // moves is telling us the estimator is reading ring-up, not latency -- and
    // the phase column, which cannot read ring-up, is the referee.
    {
        WdspChannel::Config config = baseConfig();
        config.filterTaps = 8192;
        config.filterLowHz = 300.0;
        config.filterHighHz = 2700.0;
        add("B6 passband 300-2700", config);
    }
    {
        WdspChannel::Config config = baseConfig();
        config.filterTaps = 8192;
        config.filterLowHz = 100.0;
        config.filterHighHz = 3900.0;
        add("B6 passband 100-3900", config);
    }
    // B7: minimum phase. The residue must be phase-mode independent for d107's
    // fit to hold at all.
    {
        WdspChannel::Config config = baseConfig();
        config.filterTaps = 8192;
        config.minimumPhase = true;
        add("B7 8192 minimum phase", config);
    }
    // B8: mode, which selects a different RXA stage set.
    {
        WdspChannel::Config config = baseConfig();
        config.filterTaps = 8192;
        config.mode = WdspChannel::Mode::Digu;
        add("B8 mode DIGU", config);
    }
    {
        WdspChannel::Config config = baseConfig();
        config.filterTaps = 8192;
        config.mode = WdspChannel::Mode::Cwu;
        add("B8 mode CWU", config);
    }
    // B9: the AGC this measurement normally switches off. wcpagc.c's xwcpagc
    // early-returns at mode 0, so mode 0 never touches the lookahead buffer --
    // but the SHIPPING default is mode 3, so whatever this row adds is latency
    // the operator pays and every figure above hides.
    {
        WdspChannel::Config config = baseConfig();
        config.filterTaps = 8192;
        config.agcMode = 3;
        add("B9 agcMode=3 (production)", config);
    }

    std::printf("  %-28s %9s | %10s %9s | %10s %9s %10s\n", "variant", "(taps-1)/2",
                "phase smp", "residue", "onset smp", "residue", "onset-phase");
    for (const Variant& variant : variants) {
        const double firTerm = 0.5 * static_cast<double>(variant.config.filterTaps - 1) *
                               variant.config.inputSampleRate /
                               static_cast<double>(variant.config.dspSampleRate);
        const PhaseDelay phase = measurePhaseDelay(variant.config, 1500.0,
                                                   variant.primed,
                                                   variant.label.c_str());
        const ResidueRow onset = measureOnsetResidue(variant.config, 1500.0,
                                                     variant.primed,
                                                     variant.label.c_str());
        std::printf("  %-28s %9.1f |", variant.label.c_str(), firTerm);
        if (phase.ok) {
            std::printf(" %10.1f %9.1f |", phase.refinedSamples,
                        phase.refinedSamples - firTerm);
        } else {
            std::printf(" %10s %9s |", "-", "-");
        }
        if (onset.ok) {
            std::printf(" %10.1f %9.1f", onset.onsetSamples, onset.residueSamples);
            if (phase.ok) {
                std::printf(" %10.1f", onset.onsetSamples - phase.refinedSamples);
            }
        } else {
            std::printf(" %10s %9s", "-", "-");
        }
        std::printf("\n");
    }
    std::printf("\n  All figures in INPUT samples at inputSampleRate. The phase\n"
                "  residue is the channel's fixed pipeline latency; if it tracks\n"
                "  2 x blockSize across B1 and B2 then iobuffs.c's r1/r2 pipeline is\n"
                "  the whole of it.\n");

    std::printf("\nwdsp_phase_distortion_test: measurements taken\n");
    return 0;
}
