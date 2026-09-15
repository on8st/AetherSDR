#include "core/dsp/WdspChannel.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <numbers>
#include <string>
#include <thread>
#include <vector>

namespace {

bool require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

template<typename Test>
bool runLeakChecked(const char* name, Test&& test)
{
    const uint64_t baseline = WdspChannel::outstandingAllocationsForTest();
    if (!test()) {
        return false;
    }
    const uint64_t outstanding = WdspChannel::outstandingAllocationsForTest();
    if (outstanding != baseline) {
        std::cerr << "FAIL: " << name << " leaked "
                  << (outstanding - baseline) << " WDSP allocations\n";
        return false;
    }
    return true;
}

double rms(std::span<const float> samples)
{
    double sum = 0.0;
    for (const float sample : samples) {
        sum += static_cast<double>(sample) * static_cast<double>(sample);
    }
    return std::sqrt(sum / static_cast<double>(samples.size()));
}

double maximumDifference(std::span<const float> left, std::span<const float> right)
{
    double difference = 0.0;
    for (std::size_t sample = 0; sample < left.size(); ++sample) {
        difference = std::max(difference,
                              std::abs(static_cast<double>(left[sample]) -
                                       static_cast<double>(right[sample])));
    }
    return difference;
}

void fillComplexTone(std::span<float> i, std::span<float> q,
                     int sampleRate, double frequencyHz, std::size_t offset)
{
    for (std::size_t sample = 0; sample < i.size(); ++sample) {
        const double phase = 2.0 * std::numbers::pi * frequencyHz *
                             static_cast<double>(offset + sample) /
                             static_cast<double>(sampleRate);
        i[sample] = static_cast<float>(0.1 * std::cos(phase));
        q[sample] = static_cast<float>(0.1 * std::sin(phase));
    }
}

void fillAudioTone(std::span<float> left, std::span<float> right,
                   int sampleRate, double frequencyHz, std::size_t offset)
{
    for (std::size_t sample = 0; sample < left.size(); ++sample) {
        const double phase = 2.0 * std::numbers::pi * frequencyHz *
                             static_cast<double>(offset + sample) /
                             static_cast<double>(sampleRate);
        const float value = static_cast<float>(0.1 * std::cos(phase));
        left[sample] = value;
        right[sample] = value;
    }
}

bool runVector(WdspChannel::Direction direction)
{
    WdspChannel::Config config;
    config.direction = direction;
    config.inputBlockSize = 256;
    config.dspBlockSize = 256;
    config.mode = WdspChannel::Mode::Usb;
    config.blockForOutput = true;

    std::string error;
    std::unique_ptr<WdspChannel> channel = WdspChannel::create(config, &error);
    if (!require(channel != nullptr, error.c_str())) {
        return false;
    }
    std::unique_ptr<WdspChannel> reference = WdspChannel::create(config, &error);
    if (!require(reference != nullptr, error.c_str())) {
        return false;
    }

    std::vector<float> inputI(config.inputBlockSize);
    std::vector<float> inputQ(config.inputBlockSize);
    std::vector<float> outputLeft(channel->outputBlockSize());
    std::vector<float> outputRight(channel->outputBlockSize());
    std::vector<float> referenceLeft(reference->outputBlockSize());
    std::vector<float> referenceRight(reference->outputBlockSize());
    double accumulatedEnergy = 0.0;

    for (std::size_t block = 0; block < 24; ++block) {
        if (direction == WdspChannel::Direction::Receive) {
            fillComplexTone(inputI, inputQ, config.inputSampleRate, 1000.0,
                            block * config.inputBlockSize);
        } else {
            fillAudioTone(inputI, inputQ, config.inputSampleRate, 1000.0,
                          block * config.inputBlockSize);
        }

        const uint64_t allocationsBefore = WdspChannel::allocationSequenceForTest();
        const WdspChannel::ProcessResult result =
            channel->processIq(inputI, inputQ, outputLeft, outputRight);
        const WdspChannel::ProcessResult referenceResult =
            reference->processIq(inputI, inputQ, referenceLeft, referenceRight);
        const uint64_t allocationsAfter = WdspChannel::allocationSequenceForTest();
        if (!require(result == WdspChannel::ProcessResult::Ok,
                     "blocking vector processing failed") ||
            !require(referenceResult == WdspChannel::ProcessResult::Ok,
                     "reference vector processing failed") ||
            !require(allocationsAfter == allocationsBefore,
                     "WDSP allocated inside processIq") ||
            !require(maximumDifference(outputLeft, referenceLeft) < 1.0e-6 &&
                         maximumDifference(outputRight, referenceRight) < 1.0e-6,
                     "identical WDSP channels produced different vectors")) {
            return false;
        }
        if (block >= 8) {
            accumulatedEnergy += rms(outputLeft) + rms(outputRight);
        }
        if (!require(std::ranges::all_of(outputLeft, [](float value) {
                         return std::isfinite(value);
                     }), "WDSP produced non-finite output")) {
            return false;
        }
    }

    return require(accumulatedEnergy > 0.01,
                   direction == WdspChannel::Direction::Receive
                       ? "RX vector produced no demodulated audio"
                       : "TX vector produced no IQ output");
}

bool runUnderrunTest()
{
    WdspChannel::Config config;
    config.inputBlockSize = 64;
    config.dspBlockSize = 2048;
    config.blockForOutput = false;

    std::unique_ptr<WdspChannel> channel = WdspChannel::create(config);
    if (!require(channel != nullptr, "could not create underrun channel")) {
        return false;
    }

    std::vector<float> inputI(config.inputBlockSize, 0.0f);
    std::vector<float> inputQ(config.inputBlockSize, 0.0f);
    std::vector<float> outputLeft(channel->outputBlockSize());
    std::vector<float> outputRight(channel->outputBlockSize());
    int underruns = 0;
    for (int block = 0; block < 512; ++block) {
        const WdspChannel::ProcessResult result =
            channel->processIq(inputI, inputQ, outputLeft, outputRight);
        if (result == WdspChannel::ProcessResult::Underrun) {
            ++underruns;
        } else if (!require(result == WdspChannel::ProcessResult::Ok,
                            "unexpected result in underrun test")) {
            return false;
        }
    }
    return require(underruns > 0, "nonblocking test did not report an underrun");
}

bool runReconfigurationTest()
{
    WdspChannel::Config config;
    config.inputBlockSize = 256;
    config.dspBlockSize = 256;
    config.blockForOutput = true;

    std::string error;
    std::unique_ptr<WdspChannel> channel = WdspChannel::create(config, &error);
    if (!require(channel != nullptr, error.c_str())) {
        return false;
    }

    const uint64_t allocationsBefore = WdspChannel::allocationSequenceForTest();
    config.inputBlockSize = 512;
    config.inputSampleRate = 96000;
    config.dspSampleRate = 48000;
    config.outputSampleRate = 48000;
    if (!require(channel->reconfigure(config, &error), error.c_str()) ||
        !require(channel->outputBlockSize() == 256,
                 "reconfiguration calculated the wrong output block size") ||
        !require(WdspChannel::allocationSequenceForTest() > allocationsBefore,
                 "reconfiguration did not rebuild WDSP resources")) {
        return false;
    }

    std::vector<float> inputI(config.inputBlockSize);
    std::vector<float> inputQ(config.inputBlockSize);
    std::vector<float> outputLeft(channel->outputBlockSize());
    std::vector<float> outputRight(channel->outputBlockSize());
    fillComplexTone(inputI, inputQ, config.inputSampleRate, 1000.0, 0);
    const uint64_t processAllocations = WdspChannel::allocationSequenceForTest();
    const WdspChannel::ProcessResult result =
        channel->processIq(inputI, inputQ, outputLeft, outputRight);
    return require(result == WdspChannel::ProcessResult::Ok,
                   "processing after reconfiguration failed") &&
           require(WdspChannel::allocationSequenceForTest() == processAllocations,
                   "processing after reconfiguration allocated memory");
}

// WDSP notch handles are POSITIONAL, and the whole stable-id mapping in
// Hl2Backend is built on exactly how they shift. Pin that behaviour here rather
// than inferring it from nbp.c, so a WDSP refresh that changes it fails loudly.
bool runNotchIndexTest()
{
    WdspChannel::Config config;
    config.inputBlockSize = 256;
    config.dspBlockSize = 256;
    config.blockForOutput = true;

    std::string error;
    std::unique_ptr<WdspChannel> channel = WdspChannel::create(config, &error);
    if (!require(channel != nullptr, error.c_str())) {
        return false;
    }

    const double tuneHz = 7'000'000.0;
    if (!require(channel->setNotchTuneFrequency(tuneHz),
                 "could not set the notch tune frequency") ||
        !require(channel->notchCount() == 0,
                 "a fresh channel reported existing notches")) {
        return false;
    }

    // Append three, then confirm they read back in the order they went in.
    for (int index = 0; index < 3; ++index) {
        if (!require(channel->addNotch(index, tuneHz + 1000.0 * (index + 1),
                                       400.0, true),
                     "could not add a notch")) {
            return false;
        }
    }
    if (!require(channel->notchCount() == 3, "notch count did not reach 3")) {
        return false;
    }

    // Deleting the middle notch must close the gap: what was index 2 becomes
    // index 1. A caller holding stable ids has to remap here or it edits the
    // wrong notch — this is the exact behaviour Hl2Backend::notchIndexFor()
    // and the ordered m_notches vector exist to absorb.
    if (!require(channel->removeNotch(1), "could not remove the middle notch") ||
        !require(channel->notchCount() == 2, "notch count did not fall to 2")) {
        return false;
    }
    double center = 0.0;
    double width = 0.0;
    bool active = false;
    if (!require(channel->notchAt(0, &center, &width, &active),
                 "could not read notch 0 back") ||
        !require(std::abs(center - (tuneHz + 1000.0)) < 1.0,
                 "notch 0 moved when a later notch was deleted") ||
        !require(channel->notchAt(1, &center, &width, &active),
                 "could not read notch 1 back") ||
        !require(std::abs(center - (tuneHz + 3000.0)) < 1.0,
                 "deleting a notch did not shift the ones above it down")) {
        return false;
    }

    // Centres round-trip as ABSOLUTE Hz, unmodified. If a sign correction ever
    // creeps back into WdspChannel, this is what catches it.
    if (!require(channel->editNotch(0, tuneHz - 1500.0, 250.0, false),
                 "could not edit a notch") ||
        !require(channel->notchAt(0, &center, &width, &active),
                 "could not read an edited notch back") ||
        !require(std::abs(center - (tuneHz - 1500.0)) < 1.0,
                 "an edited notch centre did not round-trip") ||
        !require(std::abs(width - 250.0) < 1.0,
                 "an edited notch width did not round-trip") ||
        !require(!active, "an edited notch kept the wrong active flag")) {
        return false;
    }

    // Out-of-range handles are refused rather than silently clamped.
    if (!require(!channel->removeNotch(9), "removing a nonexistent notch succeeded") ||
        !require(!channel->editNotch(9, tuneHz, 200.0, true),
                 "editing a nonexistent notch succeeded") ||
        !require(!channel->notchAt(9, &center, &width, &active),
                 "reading a nonexistent notch succeeded")) {
        return false;
    }

    // The documented width floor, which the UI's width presets are built on.
    if (!require(std::abs(channel->minimumNotchWidthHz() - 200.0) < 1.0,
                 "the 2048-tap minimum notch width is no longer 200 Hz")) {
        return false;
    }
    return require(channel->setNotchesEnabled(false) &&
                       channel->setNotchesEnabled(true),
                   "could not toggle the global notch run flag");
}

// The functional half: a notch parked on a tone must actually remove it, and a
// notch parked on that tone's MIRROR must not. Index bookkeeping can be right
// while the audio is untouched, and an inverted axis passes every API check.
bool runNotchAttenuationTest()
{
    WdspChannel::Config config;
    config.inputBlockSize = 256;
    config.dspBlockSize = 256;
    config.inputSampleRate = 48000;
    config.dspSampleRate = 48000;
    config.outputSampleRate = 48000;
    config.mode = WdspChannel::Mode::Usb;
    config.filterLowHz = 150.0;
    config.filterHighHz = 3000.0;
    // AGC off with a fixed gain: an AGC would claw the level back after the
    // notch and mask exactly the attenuation being measured.
    config.agcMode = 0;
    config.agcFixedGainDb = 0.0;
    config.blockForOutput = true;
    // The tap count HL2 actually runs (Hl2RxDsp::kRxFilterTaps), so the null is
    // measured in the configuration the operator hears rather than in WDSP's
    // 2048 default — which is also the one whose 200 Hz notch floor this PR
    // exists to get away from.
    config.filterTaps = 8192;

    const double tuneHz = 7'000'000.0;
    // The tone's audio pitch, and therefore the RF frequency it corresponds to.
    // Negative baseband because RXA as configured passes the opposite sign to
    // its passband bounds (see Hl2RxDsp::onIqBlock) — this is the geometry the
    // HL2 actually runs in, so the notch has to work in it.
    const double toneBasebandHz = -1500.0;
    const double toneRfHz = tuneHz + 1500.0;

    const auto measure = [&](double notchRfHz) -> double {
        std::string error;
        std::unique_ptr<WdspChannel> channel = WdspChannel::create(config, &error);
        if (!channel) {
            return -1.0;
        }
        if (!channel->setNotchTuneFrequency(tuneHz)) {
            return -1.0;
        }
        if (notchRfHz != 0.0) {
            if (!channel->addNotch(0, notchRfHz, 400.0, true) ||
                !channel->setNotchesEnabled(true)) {
                return -1.0;
            }
        }
        std::vector<float> inputI(config.inputBlockSize);
        std::vector<float> inputQ(config.inputBlockSize);
        std::vector<float> outputLeft(channel->outputBlockSize());
        std::vector<float> outputRight(channel->outputBlockSize());
        double energy = 0.0;
        // Discard the first blocks: the channel's mute ramp and the filter's
        // own group delay both suppress output that has nothing to do with the
        // notch, and 8192-tap masks take a while to fill.
        constexpr std::size_t kSettleBlocks = 40;
        constexpr std::size_t kTotalBlocks = 80;
        for (std::size_t block = 0; block < kTotalBlocks; ++block) {
            fillComplexTone(inputI, inputQ, config.inputSampleRate,
                            toneBasebandHz, block * config.inputBlockSize);
            if (channel->processIq(inputI, inputQ, outputLeft, outputRight) !=
                WdspChannel::ProcessResult::Ok) {
                return -1.0;
            }
            if (block >= kSettleBlocks) {
                energy += rms(outputLeft);
            }
        }
        return energy;
    };

    const double unnotched = measure(0.0);
    const double notched = measure(toneRfHz);
    // The mirror image: as far below the tuned frequency as the tone is above.
    // A notch here must leave the tone alone. If this attenuates instead of the
    // one above, the notch axis is inverted — the failure an API-only test
    // cannot see, because both notches are equally well-formed.
    const double mirrorNotched = measure(tuneHz - 1500.0);

    if (!require(unnotched > 0.0 && notched >= 0.0 && mirrorNotched >= 0.0,
                 "notch attenuation measurement failed to run")) {
        return false;
    }
    if (!require(unnotched > 1.0e-4, "the unnotched tone produced no audio")) {
        return false;
    }
    return require(notched < unnotched * 0.25,
                   "a notch on the tone did not attenuate it") &&
           require(mirrorNotched > unnotched * 0.75,
                   "a notch on the tone's mirror image attenuated the tone — "
                   "the notch frequency axis is inverted");
}

bool runLifecycleTest()
{
    const uint64_t baseline = WdspChannel::outstandingAllocationsForTest();
    for (int iteration = 0; iteration < 3; ++iteration) {
        std::unique_ptr<WdspChannel> first = WdspChannel::create({});
        std::unique_ptr<WdspChannel> second = WdspChannel::create({});
        if (!require(first != nullptr && second != nullptr,
                     "could not create concurrent WDSP channels") ||
            !require(first->channelIdForTest() != second->channelIdForTest(),
                     "RAII owners received the same WDSP channel ID")) {
            return false;
        }
        second.reset();
        first.reset();
        const uint64_t outstanding = WdspChannel::outstandingAllocationsForTest();
        if (outstanding != baseline) {
            std::cerr << "FAIL: WDSP teardown allocation baseline=" << baseline
                      << " outstanding=" << outstanding
                      << " iteration=" << iteration << '\n';
            return false;
        }
    }
    return true;
}

// The wisdom cache must be on disk when open() RETURNS — not at process exit.
//
// What this pins: export used to be a std::atexit handler alone, and the app's
// own signal path (Hl2EmergencyStop restores SIG_DFL and re-raises) turns every
// SIGTERM, crash and Force Quit into an exit that never runs one. The plans were
// measured and then thrown away, so the next launch paid full first-run cost
// again — for anyone who had ever force-quit, on every run.
//
// ── TXA driven the way the HL2 backend drives a transmit chain ────────────
//
// runVector(Direction::Transmit) above proves a TXA channel produces IQ, but it
// proves it in a configuration the live path cannot use: equal rates, equal
// block sizes, and blockForOutput = true. With bfo set, fexchange2()'s
// `*error += -2` branch is unreachable (iobuffs.c: `if (a->bfo)
// WaitForSingleObject(...INFINITE); if (a->bfo || doit)`), so that vector is
// structurally incapable of reporting an underrun, and every buffer
// relationship pre_main_build() computes differs from the live one.
//
// This case runs a transmit channel at the rates and block sizes
// Hl2TxDsp::Config and MetisClient actually use -- 24 kHz audio in, 48 kHz DSP,
// 48 kHz EP2 out -- with blockForOutput = false, and measures what comes out.
//
// Three things it pins that nothing else in the tree does:
//
//  * THE CALLER MUST BE PACED. A non-blocking channel has exactly one DSP
//    buffer of slack (create_iobuffs: r2_havesamps = (DSP_MULT - 1) * r2_size,
//    DSP_MULT = 2) and fexchange2 CLAMPS r2_havesamps at zero on a miss while
//    still advancing r2_outidx, so credit is destroyed rather than banked and a
//    caller that outruns the worker never recovers. A tight loop underruns
//    almost every block; at the live block period it underruns none after the
//    first.
//
//  * THE AUDIO MUST BE IN I. xpanel runs with inselect = 2, which evaluates
//    I = in[2i] * (inselect >> 1) and Q = in[2i+1] * (inselect & 1), so Q is
//    multiplied by zero. A caller that fills inputQ and leaves inputI empty
//    gets exact zeros out forever, with no error. fillAudioTone() writes the
//    same value into both planes, so runVector cannot distinguish the two.
//
//  * POSITIVE PASSBAND EDGES ALREADY GIVE THE HPSDR WIRE'S HANDEDNESS.
//    fir_bandpass builds the complex impulse as coef * (cos, -sin), i.e.
//    exp(-j*w_osc*pos), so a positive signed band selects the NEGATIVE
//    baseband half. TXA therefore emits, with no conjugation, the same
//    handedness Hl2TxDsp reaches by conjugating -- which is what
//    hl2_txdsp_test's "USB puts energy on the LOWER wire bin" asserts.

struct TransmitRun
{
    bool created = false;
    int okBlocks = 0;
    int underrunBlocks = 0;
    int otherBlocks = 0;
    int firstOkBlock = -1;
    int lastUnderrunBlock = -1;
    int firstNonZeroBlock = -1;
    double peakMagnitude = 0.0;
    // Wire-facing IQ from Ok blocks at or after `discardBlocks`, with each
    // sample's ABSOLUTE index in the output stream. The index matters: a
    // dropped block is a phase discontinuity, and correlating a concatenation
    // of non-adjacent blocks against a fixed tone measures nothing.
    std::vector<std::complex<float>> iq;
    std::vector<std::size_t> index;
    // Per-Ok-block correlation phase at the tone frequency, in degrees, taken
    // against each block's ABSOLUTE position in the output stream. A channel
    // whose r2 read pointer is aligned with its write pointer returns the same
    // phase for every block (the chain's fixed group delay). fexchange2
    // advances r2_outidx on a MISS as well as on a hit, so a run of underruns
    // de-phases the two pointers and a later successful read can return a
    // buffer the worker has not refreshed -- which shows up here, and only
    // here, as a block whose phase differs from its neighbours'.
    std::vector<double> blockPhaseDeg;
    std::vector<double> blockPhasePeak;   // that block's peak |sample|, for gating
};

// Live geometry. Hl2TxDsp::Config's 24 kHz audio in and 48 kHz EP2 out, with
// dspBlockSize in DSP-rate samples -- twice the input block, so the channel
// consumes exactly one input block per DSP pass. That is the mirror of the
// arithmetic Hl2RxDsp::configure already does for receive, and nothing in the
// tree does it for transmit today.
WdspChannel::Config liveTransmitConfig(WdspChannel::Mode mode,
                                       double lowHz, double highHz)
{
    WdspChannel::Config config;
    config.direction = WdspChannel::Direction::Transmit;
    config.inputSampleRate = 24000;
    config.dspSampleRate = 48000;
    config.outputSampleRate = 48000;
    config.inputBlockSize = 512;
    config.dspBlockSize = 1024;
    config.mode = mode;
    config.filterLowHz = lowHz;
    config.filterHighHz = highHz;
    config.blockForOutput = false;   // the LIVE setting, deliberately
    return config;
}

// `plane`: 0 = tone in I only (what a backend feeding mono audio would do),
// 1 = tone in Q only, 2 = both (what fillAudioTone does).
// `paceUs`: wall-clock delay between calls. 0 is a tight loop; the live value
// is inputBlockSize / inputSampleRate = 21333 us.
TransmitRun runTransmitChannel(int plane, double toneHz, WdspChannel::Mode mode,
                               double lowHz, double highHz,
                               std::size_t blocks, std::size_t discardBlocks,
                               int paceUs)
{
    TransmitRun run;
    const WdspChannel::Config config = liveTransmitConfig(mode, lowHz, highHz);

    std::string error;
    std::unique_ptr<WdspChannel> channel = WdspChannel::create(config, &error);
    if (!require(channel != nullptr, error.c_str())) {
        return run;
    }
    run.created = true;

    std::vector<float> inputI(config.inputBlockSize, 0.0f);
    std::vector<float> inputQ(config.inputBlockSize, 0.0f);
    std::vector<float> outputLeft(channel->outputBlockSize());
    std::vector<float> outputRight(channel->outputBlockSize());
    const std::size_t outBlock = channel->outputBlockSize();

    for (std::size_t block = 0; block < blocks; ++block) {
        for (std::size_t sample = 0; sample < inputI.size(); ++sample) {
            const double phase = 2.0 * std::numbers::pi * toneHz *
                                 static_cast<double>(block * inputI.size() + sample) /
                                 static_cast<double>(config.inputSampleRate);
            const float value = static_cast<float>(0.1 * std::cos(phase));
            inputI[sample] = (plane == 1) ? 0.0f : value;
            inputQ[sample] = (plane == 0) ? 0.0f : value;
        }
        const WdspChannel::ProcessResult result =
            channel->processIq(inputI, inputQ, outputLeft, outputRight);
        switch (result) {
        case WdspChannel::ProcessResult::Ok:
            ++run.okBlocks;
            if (run.firstOkBlock < 0) {
                run.firstOkBlock = static_cast<int>(block);
            }
            break;
        case WdspChannel::ProcessResult::Underrun:
            ++run.underrunBlocks;
            run.lastUnderrunBlock = static_cast<int>(block);
            break;
        default:
            ++run.otherBlocks;
            break;
        }
        double blockPeak = 0.0;
        for (std::size_t k = 0; k < outBlock; ++k) {
            blockPeak = std::max(blockPeak,
                                 std::abs(static_cast<double>(outputLeft[k])));
            blockPeak = std::max(blockPeak,
                                 std::abs(static_cast<double>(outputRight[k])));
        }
        if (blockPeak > 0.0 && run.firstNonZeroBlock < 0) {
            run.firstNonZeroBlock = static_cast<int>(block);
        }
        run.peakMagnitude = std::max(run.peakMagnitude, blockPeak);
        if (result == WdspChannel::ProcessResult::Ok && blockPeak > 0.0) {
            std::complex<double> acc {0.0, 0.0};
            const double w = 2.0 * std::numbers::pi * toneHz / 48000.0;
            for (std::size_t k = 0; k < outBlock; ++k) {
                const double ph = w * static_cast<double>(block * outBlock + k);
                acc += std::complex<double>(outputLeft[k], outputRight[k]) *
                       std::complex<double>(std::cos(ph), std::sin(ph));
            }
            run.blockPhaseDeg.push_back(std::arg(acc) * 180.0 /
                                        std::numbers::pi);
            run.blockPhasePeak.push_back(blockPeak);
        }
        if (result != WdspChannel::ProcessResult::Ok) {
            // An underrun slips the output stream by one whole DSP buffer for
            // good (see the census in runTransmitLiveGeometryTest), so anything
            // collected before it is in a different phase frame from anything
            // collected after. Start again rather than correlate across the
            // seam: the alternative is a suppression figure that silently
            // averages two time origins.
            run.iq.clear();
            run.index.clear();
        } else if (block >= discardBlocks) {
            for (std::size_t k = 0; k < outBlock; ++k) {
                run.iq.emplace_back(outputLeft[k], outputRight[k]);
                run.index.push_back(block * outBlock + k);
            }
        }
        if (paceUs > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(paceUs));
        }
    }
    return run;
}

// Goertzel-style complex-bin correlation. The same instrument hl2_txdsp_test
// runs on the phasing modulator, extended only to take each sample's absolute
// index so a dropped block does not silently become a phase step.
double binPower(const std::vector<std::complex<float>>& iq,
                const std::vector<std::size_t>& index, double hz, double fs)
{
    if (iq.empty()) {
        return 0.0;
    }
    std::complex<double> acc {0.0, 0.0};
    const double w = -2.0 * std::numbers::pi * hz / fs;
    for (std::size_t n = 0; n < iq.size(); ++n) {
        const double ph = w * static_cast<double>(index[n]);
        acc += std::complex<double>(iq[n].real(), iq[n].imag()) *
               std::complex<double>(std::cos(ph), std::sin(ph));
    }
    return std::abs(acc) / static_cast<double>(iq.size());
}

double suppressionDb(double wanted, double unwanted)
{
    // Floor well below float32 quantization so a bin that measures at the
    // arithmetic floor prints its real value rather than a clamp artefact.
    return 20.0 * std::log10(std::max(1.0e-20, unwanted) /
                             std::max(1.0e-20, wanted));
}

// Input block period at the live rates, in microseconds: what
// Hl2TxDsp::processAudioBlock's accumulator hands the channel, paced by
// AudioEngine's 5 ms TX poll timer.
constexpr int kLivePaceUs = 512 * 1000000 / 24000;   // 21333
// Pace for the spectral runs. Four times FASTER than the live cadence, and
// still five times slower than the point the census below shows the worker
// keeping up, so these runs are not measuring a race. Keeping them off the live
// cadence keeps the case's wall-clock cost to a few seconds.
constexpr int kSpectralPaceUs = 5000;

bool runTransmitLiveGeometryTest()
{
    // 1. Underrun census against caller pacing. This is the measurement the
    //    original TXA attempt needed and the one runVector cannot make.
    std::cout << "  TX underrun census at the live geometry"
                 " (512 in / 1024 dsp, 24k->48k, bfo=0):\n";
    bool pacedClean = false;
    for (const int paceUs : {0, 1000, 5000, kLivePaceUs}) {
        // The unpaced leg costs no wall clock, so run it long enough to show
        // that the state does not clear itself.
        const std::size_t censusBlocks = (paceUs == 0) ? 256 : 64;
        const TransmitRun census =
            runTransmitChannel(0, 1000.0, WdspChannel::Mode::Usb, 300.0, 2700.0,
                               censusBlocks, censusBlocks, paceUs);
        if (!require(census.created, "live-geometry transmit channel was refused")) {
            return false;
        }
        // Only blocks at full amplitude: a block still inside the mute ramp or
        // bp0's fill has a meaningless phase, and including it would report a
        // priming transient as a pointer fault.
        double phaseSpread = 0.0;
        std::size_t settledBlocks = 0;
        {
            double reference = 0.0;
            bool haveReference = false;
            for (std::size_t n = 0; n < census.blockPhaseDeg.size(); ++n) {
                if (census.blockPhasePeak[n] < 0.5 * census.peakMagnitude) {
                    continue;
                }
                ++settledBlocks;
                if (!haveReference) {
                    reference = census.blockPhaseDeg[n];
                    haveReference = true;
                    continue;
                }
                double delta = census.blockPhaseDeg[n] - reference;
                while (delta > 180.0) { delta -= 360.0; }
                while (delta < -180.0) { delta += 360.0; }
                phaseSpread = std::max(phaseSpread, std::abs(delta));
            }
        }
        std::cout << "    pace " << paceUs << " us over " << censusBlocks
                  << " blocks: ok=" << census.okBlocks
                  << " underrun=" << census.underrunBlocks
                  << " other=" << census.otherBlocks
                  << " lastUnderrun=" << census.lastUnderrunBlock
                  << " signalBlocks=" << census.blockPhaseDeg.size()
                  << " settledBlocks=" << settledBlocks
                  << " maxPhaseSpread=" << phaseSpread << " deg\n";
        if (!require(census.otherBlocks == 0,
                     "a live-geometry transmit channel reported a hard error")) {
            return false;
        }
        if (paceUs == kLivePaceUs) {
            // At the live cadence the pipeline primes and stays primed. The
            // margin is deliberately loose -- what this falsifies is the
            // existing note's "Underrun on most blocks", not a tight bound.
            pacedClean = census.underrunBlocks * 20 <=
                         static_cast<int>(censusBlocks);
            // AND the output must not have SLIPPED. An underrun does not merely
            // drop a block: fexchange2 advances r2_outidx on the miss without
            // consuming, so the read pointer catches the write pointer up and
            // the stream thereafter runs one whole DSP buffer ahead -- measured
            // here as a 120 degree step at 1 kHz, which is exactly 1024 samples
            // at 48 kHz. A block of audio is discarded on top of the block of
            // silence, permanently, with nothing reported. Only asserted on a
            // leg that underran nothing, so a loaded machine reports the slip
            // rather than failing twice for one cause.
            if (census.underrunBlocks == 0 &&
                !require(phaseSpread < 1.0,
                         "a clean transmit channel's output slipped against "
                         "its input")) {
                return false;
            }
        }
    }
    if (!require(pacedClean,
                 "a transmit channel paced at the live block period underran "
                 "more than 5% of blocks")) {
        return false;
    }

    constexpr std::size_t kBlocks = 96;
    constexpr std::size_t kDiscard = 24;   // past the mute ramp and bp0's fill

    // 2. The backend's arrangement: mono audio in I, Q empty.
    const TransmitRun iOnly =
        runTransmitChannel(0, 1000.0, WdspChannel::Mode::Usb, 300.0, 2700.0,
                           kBlocks, kDiscard, kSpectralPaceUs);
    if (!require(iOnly.created, "live-geometry transmit channel was refused")) {
        return false;
    }
    std::cout << "  TX live geometry (I only): ok=" << iOnly.okBlocks
              << " underrun=" << iOnly.underrunBlocks
              << " firstNonZero=" << iOnly.firstNonZeroBlock
              << " peak=" << iOnly.peakMagnitude << '\n';

    // 3. The same feed in the other plane. xpanel's inselect = 2 discards Q.
    const TransmitRun qOnly =
        runTransmitChannel(1, 1000.0, WdspChannel::Mode::Usb, 300.0, 2700.0,
                           24, 24, kSpectralPaceUs);
    if (!require(qOnly.created, "live-geometry transmit channel was refused")) {
        return false;
    }
    std::cout << "  TX live geometry (Q only): ok=" << qOnly.okBlocks
              << " peak=" << qOnly.peakMagnitude << '\n';

    if (!require(iOnly.iq.size() >= 4096,
                 "live-geometry transmit channel produced too few contiguous "
                 "Ok blocks to correlate") ||
        !require(iOnly.peakMagnitude > 0.0,
                 "audio in I produced no transmit IQ at the live geometry") ||
        !require(qOnly.peakMagnitude == 0.0,
                 "audio in Q produced transmit IQ; xpanel's inselect changed")) {
        return false;
    }

    // 4. Sideband and opposite-sideband suppression, measured with the same
    //    instrument and on the same convention hl2_txdsp_test uses: the
    //    wire-facing plane pair, against an audio tone, with nothing
    //    downstream. Two conjugations cannot cancel here.
    const double upper = binPower(iOnly.iq, iOnly.index, 1000.0, 48000.0);
    const double lower = binPower(iOnly.iq, iOnly.index, -1000.0, 48000.0);
    std::cout << "  TX USB {300,2700} 1 kHz: +1 kHz " << upper
              << "  -1 kHz " << lower << "  suppression "
              << suppressionDb(std::max(upper, lower), std::min(upper, lower))
              << " dB over " << iOnly.iq.size() << " samples\n";

    // hl2_txdsp_test asserts exactly this of Hl2TxDsp's CONJUGATED output. TXA
    // reaches it with no conjugation, because fir_bandpass's exp(-j*w_osc*pos)
    // impulse makes a positive signed band select the negative baseband half.
    // So Hl2Backend::defaultTxPassbandForMode's positive-for-every-mode table
    // is already right for a TXA channel, and ADDING the conjugation would
    // transmit on the wrong sideband.
    if (!require(lower > upper,
                 "TXA with positive passband edges did not put the tone on the "
                 "LOWER wire bin")) {
        return false;
    }
    if (!require(suppressionDb(lower, upper) < -40.0,
                 "TXA opposite-sideband suppression below 40 dB")) {
        return false;
    }

    // 5. Negative passband edges mirror it. TXASetupBPFilters handles TXA_LSB
    //    and TXA_USB with the identical CalcBandpassFilter call, so the MODE
    //    does not select the sideband in TXA -- the passband sign does, exactly
    //    as it does in RXA.
    const TransmitRun mirrored =
        runTransmitChannel(0, 1000.0, WdspChannel::Mode::Usb, -2700.0, -300.0,
                           kBlocks, kDiscard, kSpectralPaceUs);
    if (!require(mirrored.created && !mirrored.iq.empty(),
                 "mirrored-passband transmit channel produced nothing")) {
        return false;
    }
    const double mirroredUpper = binPower(mirrored.iq, mirrored.index, 1000.0, 48000.0);
    const double mirroredLower = binPower(mirrored.iq, mirrored.index, -1000.0, 48000.0);
    std::cout << "  TX USB {-2700,-300} 1 kHz: +1 kHz " << mirroredUpper
              << "  -1 kHz " << mirroredLower << "  suppression "
              << suppressionDb(std::max(mirroredUpper, mirroredLower),
                               std::min(mirroredUpper, mirroredLower)) << " dB\n";
    if (!require(mirroredUpper > mirroredLower,
                 "negative passband edges did not mirror the sideband")) {
        return false;
    }

    // 6. The DIGU/DIGL passband at its low edge. Hl2TxDsp's 255 Blackman taps
    //    at 48 kHz give a derived 22 dB at 150 Hz and 30.6 dB at 200 Hz on this
    //    passband; TXA's bp0 is max(2048, dsp_size) taps at the same rate.
    for (const double toneHz : {150.0, 200.0, 300.0, 1000.0}) {
        const TransmitRun dig =
            runTransmitChannel(0, toneHz, WdspChannel::Mode::Digu, 150.0, 3000.0,
                               kBlocks, kDiscard, kSpectralPaceUs);
        if (!require(dig.created && !dig.iq.empty(),
                     "DIGU transmit channel produced nothing")) {
            return false;
        }
        const double wanted = binPower(dig.iq, dig.index, -toneHz, 48000.0);
        const double image = binPower(dig.iq, dig.index, toneHz, 48000.0);
        std::cout << "  TX DIGU {150,3000} tone " << toneHz
                  << " Hz: wanted " << wanted << "  image " << image
                  << "  suppression " << suppressionDb(wanted, image) << " dB\n";
    }

    // 7. Out-of-passband rejection -- the assertion that caught the wideband
    //    Hilbert bug on the phasing modulator.
    const TransmitRun outOfBand =
        runTransmitChannel(0, 5000.0, WdspChannel::Mode::Usb, 300.0, 2700.0,
                           kBlocks, kDiscard, kSpectralPaceUs);
    if (!require(outOfBand.created && !outOfBand.iq.empty(),
                 "out-of-band transmit channel produced nothing")) {
        return false;
    }
    const double leak =
        std::max(binPower(outOfBand.iq, outOfBand.index, 5000.0, 48000.0),
                 binPower(outOfBand.iq, outOfBand.index, -5000.0, 48000.0));
    std::cout << "  TX out-of-band 5 kHz against a 2700 Hz edge: "
              << suppressionDb(lower, leak) << " dB below an in-band tone\n";
    if (!require(suppressionDb(lower, leak) < -60.0,
                 "a 5 kHz tone leaked through a 2700 Hz transmit filter")) {
        return false;
    }

    return true;
}

// ── "Underrun on most blocks and zeros on the rest" ───────────────────────
//
// Hl2TxDsp.cpp's note records a TXA attempt that "returned Underrun on most
// blocks and zeros on the rest". S6 §3.6 ranks four candidates for the ZEROS
// half and says none of them is established. This case is the experiment §3.6
// names: a transmit channel at the live rates and block sizes, fed with the
// tone in ONE plane only, printing per-block RMS and ProcessResult for the
// first 200 blocks.
//
// It is deliberately a SIBLING of runTransmitLiveGeometryTest, not a
// replacement. That case establishes the Underrun half (caller cadence) and
// measures the channel's spectra; this one is about the zeros, and about which
// of §3.6's four candidates can and cannot produce them.
//
// The four candidates, and what each leg below does to it:
//
//  1. WRONG PLANE. xpanel runs with inselect = 2 -- create_panel's ninth
//     argument in txa.c's create_txa, commented "1 to use Q, 2 to use I for
//     input" -- and evaluates I = in[2i] * (inselect >> 1),
//     Q = in[2i+1] * (inselect & 1). Q is multiplied by zero. Leg Q feeds the
//     tone into inputQ and nothing into inputI.
//
//  2. PRIMING: the iobuffs mute ramp plus bp0's fill. create_slews sizes
//     ndelup = tdelayup * in_rate and ntup = tslewup * in_rate, and upslew2
//     sits in BEGIN emitting zeros until a non-zero sample arrives. Leg I
//     measures how long the priming actually lasts, in blocks and in samples,
//     against that arithmetic.
//
//  3. A REFUSED CONTROL CALL. WdspChannel::setFilter returns false on
//     lowHz >= highHz and on a control operation already in flight. Leg 3
//     refuses a call on a RUNNING channel and keeps feeding it.
//
//  4. xuslew. Not reachable for SSB: TXAuSlewCheck leaves runmode 0 and the
//     stage is a pass-through. Leg AM drives the same geometry in AM, where
//     ammod.run is 1, and reports what that does to the priming stretch.
//
// Note what upslew2 tests: `(I != 0.0) || (Q != 0.0)`, on the RAW input pair,
// BEFORE xpanel. So candidates 1 and 2 are separable rather than confounded --
// a Q-only feed opens the mute ramp exactly as an I-only feed does, and is
// then zeroed one stage later.

struct BlockRecord
{
    WdspChannel::ProcessResult result = WdspChannel::ProcessResult::Ok;
    double rms = 0.0;
    bool exactlyZero = true;
};

const char* resultName(WdspChannel::ProcessResult result)
{
    switch (result) {
    case WdspChannel::ProcessResult::Ok:                  return "Ok";
    case WdspChannel::ProcessResult::Underrun:            return "Underrun";
    case WdspChannel::ProcessResult::Busy:                return "Busy";
    case WdspChannel::ProcessResult::InvalidBuffer:       return "InvalidBuffer";
    case WdspChannel::ProcessResult::AllocationViolation: return "AllocationViolation";
    case WdspChannel::ProcessResult::EngineError:         return "EngineError";
    }
    return "?";
}

struct ZerosCensus
{
    bool created = false;
    std::vector<BlockRecord> blocks;
    int okBlocks = 0;
    int underrunBlocks = 0;
    int otherBlocks = 0;
    int firstNonZeroBlock = -1;
    // Index of the first non-zero sample in the OUTPUT stream, at the output
    // rate. Converted to input samples this is the quantity §3.6's mute-ramp
    // arithmetic predicts.
    long long firstNonZeroOutputSample = -1;
    int okBlocksThatWereZero = 0;
    int okBlocksAfterFirstNonZeroThatWereZero = 0;
    double peakRms = 0.0;
    // RMS of the Ok blocks only, in the order they were returned. On a starved
    // caller this is the OUTPUT STREAM's own opening sequence sampled one block
    // at a time, which is the whole point: it is what the caller sees, and it
    // is not the same thing as the first N calls.
    std::vector<double> okRms;
};

// plane: 0 = tone in I only, 1 = tone in Q only.
ZerosCensus runZerosCensus(int plane, double toneHz, WdspChannel::Mode mode,
                           double lowHz, double highHz, std::size_t blocks,
                           int paceUs)
{
    ZerosCensus census;
    const WdspChannel::Config config = liveTransmitConfig(mode, lowHz, highHz);
    std::string error;
    std::unique_ptr<WdspChannel> channel = WdspChannel::create(config, &error);
    if (!require(channel != nullptr, error.c_str())) {
        return census;
    }
    census.created = true;

    std::vector<float> inputI(config.inputBlockSize, 0.0f);
    std::vector<float> inputQ(config.inputBlockSize, 0.0f);
    const std::size_t outBlock = channel->outputBlockSize();
    std::vector<float> outputLeft(outBlock);
    std::vector<float> outputRight(outBlock);

    for (std::size_t block = 0; block < blocks; ++block) {
        for (std::size_t sample = 0; sample < inputI.size(); ++sample) {
            const double phase = 2.0 * std::numbers::pi * toneHz *
                                 static_cast<double>(block * inputI.size() + sample) /
                                 static_cast<double>(config.inputSampleRate);
            const float value = static_cast<float>(0.1 * std::cos(phase));
            inputI[sample] = (plane == 1) ? 0.0f : value;
            inputQ[sample] = (plane == 0) ? 0.0f : value;
        }
        const WdspChannel::ProcessResult result =
            channel->processIq(inputI, inputQ, outputLeft, outputRight);

        BlockRecord record;
        record.result = result;
        double sum = 0.0;
        for (std::size_t k = 0; k < outBlock; ++k) {
            const double l = outputLeft[k];
            const double r = outputRight[k];
            sum += l * l + r * r;
            if (l != 0.0 || r != 0.0) {
                if (record.exactlyZero) {
                    record.exactlyZero = false;
                    if (census.firstNonZeroOutputSample < 0) {
                        census.firstNonZeroOutputSample =
                            static_cast<long long>(block * outBlock + k);
                    }
                }
            }
        }
        record.rms = std::sqrt(sum / static_cast<double>(2 * outBlock));
        census.peakRms = std::max(census.peakRms, record.rms);
        if (!record.exactlyZero && census.firstNonZeroBlock < 0) {
            census.firstNonZeroBlock = static_cast<int>(block);
        }
        switch (result) {
        case WdspChannel::ProcessResult::Ok:
            ++census.okBlocks;
            census.okRms.push_back(record.rms);
            if (record.exactlyZero) {
                ++census.okBlocksThatWereZero;
                if (census.firstNonZeroBlock >= 0) {
                    ++census.okBlocksAfterFirstNonZeroThatWereZero;
                }
            }
            break;
        case WdspChannel::ProcessResult::Underrun:
            ++census.underrunBlocks;
            break;
        default:
            ++census.otherBlocks;
            break;
        }
        census.blocks.push_back(record);
        if (paceUs > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(paceUs));
        }
    }
    return census;
}

// Per-block print. Every block of the first `verbatim`, then only blocks that
// are not Ok or that change the zero/non-zero state, then a run-length line for
// each collapsed stretch -- so a 200-block leg that says the same thing 190
// times says it once, and nothing that CHANGES is ever hidden.
void printCensus(const char* label, const ZerosCensus& census,
                 std::size_t verbatim)
{
    std::cout << "    " << label << ":\n";
    std::size_t runStart = 0;
    for (std::size_t n = 0; n < census.blocks.size(); ++n) {
        const BlockRecord& record = census.blocks[n];
        const bool changed =
            n == 0 ||
            record.result != census.blocks[n - 1].result ||
            record.exactlyZero != census.blocks[n - 1].exactlyZero;
        if (changed && n > 0 && n > verbatim && n - runStart > 1) {
            std::cout << "      b" << runStart << "..b" << (n - 1) << ": "
                      << resultName(census.blocks[runStart].result)
                      << (census.blocks[runStart].exactlyZero
                              ? "  rms=0 (exactly)" : "  rms>0")
                      << "  (" << (n - runStart) << " blocks, same)\n";
        }
        if (changed) {
            runStart = n;
        }
        if (n < verbatim || changed) {
            std::cout << "      b" << n << ": " << resultName(record.result)
                      << "  rms=" << record.rms
                      << (record.exactlyZero ? "  (exactly zero)" : "") << '\n';
        }
    }
    const std::size_t last = census.blocks.size();
    if (last > verbatim && last - runStart > 1) {
        std::cout << "      b" << runStart << "..b" << (last - 1) << ": "
                  << resultName(census.blocks[runStart].result)
                  << (census.blocks[runStart].exactlyZero
                          ? "  rms=0 (exactly)" : "  rms>0")
                  << "  (" << (last - runStart) << " blocks, same)\n";
    }
    std::cout << "      totals: ok=" << census.okBlocks
              << " underrun=" << census.underrunBlocks
              << " other=" << census.otherBlocks
              << " firstNonZeroBlock=" << census.firstNonZeroBlock
              << " firstNonZeroOutputSample=" << census.firstNonZeroOutputSample
              << " okButZero=" << census.okBlocksThatWereZero
              << " okButZeroAfterSignalStarted="
              << census.okBlocksAfterFirstNonZeroThatWereZero
              << " peakRms=" << census.peakRms << '\n';
}

bool runTransmitZerosCensusTest()
{
    constexpr std::size_t kBlocks = 200;     // §3.6's own number
    constexpr std::size_t kVerbatim = 12;

    // The mute-ramp arithmetic §3.6 predicts, printed so the measurement below
    // can be read against it rather than against a remembered number.
    const WdspChannel::Config probe =
        liveTransmitConfig(WdspChannel::Mode::Usb, 300.0, 2700.0);
    const int ndelup = static_cast<int>(probe.muteDelayUpSec *
                                        probe.inputSampleRate);
    const int ntup = static_cast<int>(probe.muteSlewUpSec *
                                      probe.inputSampleRate);
    std::cout << "  TX zeros census at the live geometry (512 in / 1024 dsp, "
                 "24k->48k, bfo=0)\n";
    std::cout << "    create_slews arithmetic: ndelup=" << ndelup
              << " + ntup=" << ntup << " = " << (ndelup + ntup)
              << " input samples = "
              << static_cast<double>(ndelup + ntup) /
                     static_cast<double>(probe.inputBlockSize)
              << " input blocks; bp0 = " << probe.filterTaps << " taps over "
              << probe.dspBlockSize << "-sample DSP passes\n";

    // ── Leg I: the tone in inputI, paced at the live block period. ────────
    const ZerosCensus legI =
        runZerosCensus(0, 1000.0, WdspChannel::Mode::Usb, 300.0, 2700.0,
                       kBlocks, kLivePaceUs);
    if (!require(legI.created, "live-geometry transmit channel was refused")) {
        return false;
    }
    printCensus("leg I  (tone in inputI, inputQ zero, live pace)", legI,
                kVerbatim);

    // ── Leg Q: the same tone in inputQ. ───────────────────────────────────
    const ZerosCensus legQ =
        runZerosCensus(1, 1000.0, WdspChannel::Mode::Usb, 300.0, 2700.0,
                       kBlocks, kLivePaceUs);
    if (!require(legQ.created, "live-geometry transmit channel was refused")) {
        return false;
    }
    printCensus("leg Q  (tone in inputQ, inputI zero, live pace)", legQ,
                kVerbatim);

    // ── Leg U: I-only, unpaced. The historical caller's cadence. ──────────
    const ZerosCensus legU =
        runZerosCensus(0, 1000.0, WdspChannel::Mode::Usb, 300.0, 2700.0,
                       kBlocks, 0);
    if (!require(legU.created, "live-geometry transmit channel was refused")) {
        return false;
    }
    printCensus("leg U  (tone in inputI, unpaced tight loop)", legU, kVerbatim);

    // ── Leg AM: candidate 4's stage, reachable. ───────────────────────────
    const ZerosCensus legAm =
        runZerosCensus(0, 1000.0, WdspChannel::Mode::Am, -3000.0, 3000.0,
                       kBlocks, kLivePaceUs);
    if (!require(legAm.created, "AM live-geometry transmit channel was refused")) {
        return false;
    }
    printCensus("leg AM (tone in inputI, AM, live pace -- xuslew reachable)",
                legAm, kVerbatim);

    // ── What the four legs settle ─────────────────────────────────────────

    // CANDIDATE 1 is a real, silent, permanent zeros mechanism. Not "zeros for
    // a while": zeros for every block of the run, with every ProcessResult Ok,
    // so a caller watching the return value sees a healthy channel.
    if (!require(legQ.peakRms == 0.0,
                 "audio in Q produced transmit output; xpanel's inselect "
                 "changed")) {
        return false;
    }
    if (!require(legQ.otherBlocks == 0 && legQ.underrunBlocks == 0,
                 "the Q-only leg reported an error, so the wrong-plane fault "
                 "is not silent after all")) {
        return false;
    }
    if (!require(static_cast<std::size_t>(legQ.okBlocksThatWereZero) == kBlocks,
                 "the Q-only leg did not report Ok-and-zero on every block")) {
        return false;
    }

    // CANDIDATE 2 is real but BOUNDED, and the bound is what excludes it as an
    // explanation for zeros that persist. Output starts, and once started it
    // never returns to exact zero on an Ok block.
    if (!require(legI.firstNonZeroBlock >= 0,
                 "audio in I produced no transmit output at all")) {
        return false;
    }
    if (!require(legI.firstNonZeroBlock < 8,
                 "the priming stretch on an I-only feed ran past 8 blocks")) {
        return false;
    }
    if (!require(legI.okBlocksAfterFirstNonZeroThatWereZero == 0,
                 "an Ok block returned exact zeros AFTER the channel had "
                 "started producing output")) {
        return false;
    }

    // THE RECORDED SYMPTOM IS LEG U, AND ITS ZEROS ARE CANDIDATE 2's.
    // Hl2TxDsp.cpp's note says "Underrun on most blocks AND ZEROS ON THE REST"
    // -- the rest of the BLOCKS, not the rest of the run. Leg U reproduces both
    // halves at once with the audio in the RIGHT plane. The caller outruns the
    // worker, fexchange2 misses on nearly every call, and the few reads that do
    // land walk the OUTPUT STREAM's opening blocks one at a time -- so what the
    // caller gets back is leg I's priming stretch, spread over hundreds of
    // calls instead of five. The okRms sequence printed below is that walk.
    //
    // How far it walks is load-dependent, and deliberately not asserted: on a
    // busier machine fewer reads land and every one of them is zero (which is
    // the record verbatim); on a quieter one a late read reaches full
    // amplitude. What is asserted is the part that does not move -- most calls
    // underrun, and the first read a starved caller gets is exact zeros.
    //
    // So candidate 1 is NOT what the note describes. It cannot be: leg Q shows
    // a wrong-plane channel returns Ok on every block and underruns NOTHING,
    // which contradicts the "Underrun on most blocks" half of the record. It
    // remains a real, silent, permanent zeros mechanism -- just not this one.
    if (!require(legU.underrunBlocks * 2 >= static_cast<int>(kBlocks),
                 "an unpaced caller at the live geometry did not underrun; the "
                 "recorded symptom was not reproduced")) {
        return false;
    }
    if (!require(!legU.okRms.empty() && legU.okRms.front() == 0.0,
                 "the first block a starved transmit caller got back was not "
                 "exact zeros")) {
        return false;
    }
    std::cout << "    leg U Ok-block rms in order:";
    for (const double value : legU.okRms) {
        std::cout << ' ' << value;
    }
    std::cout << "\n    leg I block rms in order (first 8):";
    for (std::size_t n = 0; n < 8 && n < legI.blocks.size(); ++n) {
        std::cout << ' ' << legI.blocks[n].rms;
    }
    std::cout << '\n';
    std::cout << "    verdict:\n"
                 "      candidate 1 (leg Q): " << legQ.okBlocksThatWereZero
              << "/" << kBlocks << " Ok-and-exactly-zero, "
              << legQ.underrunBlocks << " underruns. A real, silent, permanent"
                 " zeros mechanism -- but it underruns NOTHING, so it does not"
                 " fit the recorded \"Underrun on most blocks\".\n"
                 "      candidate 2 (leg I): priming ends at output sample "
              << legI.firstNonZeroOutputSample << " ("
              << 1000.0 * static_cast<double>(legI.firstNonZeroOutputSample) /
                     static_cast<double>(probe.outputSampleRate)
              << " ms), of which create_slews accounts for "
              << 1000.0 * static_cast<double>(ndelup + ntup) /
                     static_cast<double>(probe.inputSampleRate)
              << " ms. Bounded: output never returns to exact zero after it"
                 " starts.\n"
                 "      leg U = the record: underrun=" << legU.underrunBlocks
              << "/" << kBlocks << ", ok=" << legU.okBlocks
              << ". The output stream advanced " << legU.okBlocks
              << " blocks in " << kBlocks << " calls, so candidate 2's bounded"
                 " priming is what a starved caller sees for the whole run.\n";

    // CANDIDATE 3 cannot be the mechanism either, and this is the leg that
    // shows why: a refused setFilter leaves a RUNNING channel running. The
    // channel is fed across the refusal, so "the caller stopped feeding" --
    // §3.6's own caveat -- is the only way a false becomes zeros, and that is a
    // property of the caller, not of the channel.
    {
        const WdspChannel::Config config =
            liveTransmitConfig(WdspChannel::Mode::Usb, 300.0, 2700.0);
        std::string error;
        std::unique_ptr<WdspChannel> channel = WdspChannel::create(config, &error);
        if (!require(channel != nullptr, error.c_str())) {
            return false;
        }
        std::vector<float> inputI(config.inputBlockSize, 0.0f);
        std::vector<float> inputQ(config.inputBlockSize, 0.0f);
        const std::size_t outBlock = channel->outputBlockSize();
        std::vector<float> outputLeft(outBlock);
        std::vector<float> outputRight(outBlock);
        double rmsBefore = 0.0;
        double rmsAfter = 0.0;
        bool refused = false;
        bool accepted = true;
        for (std::size_t block = 0; block < 32; ++block) {
            for (std::size_t sample = 0; sample < inputI.size(); ++sample) {
                const double phase = 2.0 * std::numbers::pi * 1000.0 *
                    static_cast<double>(block * inputI.size() + sample) /
                    static_cast<double>(config.inputSampleRate);
                inputI[sample] = static_cast<float>(0.1 * std::cos(phase));
            }
            if (block == 16) {
                rmsBefore = rms(outputLeft);
                // Inverted edges: setFilter's own guard, on a live channel.
                refused = !channel->setFilter(2700.0, 300.0);
                accepted = channel->setFilter(400.0, 2600.0);
            }
            channel->processIq(inputI, inputQ, outputLeft, outputRight);
            if (block == 31) {
                rmsAfter = rms(outputLeft);
            }
            std::this_thread::sleep_for(
                std::chrono::microseconds(kLivePaceUs));
        }
        std::cout << "    leg 3  (setFilter refused mid-run): "
                     "setFilter(2700,300) refused=" << (refused ? "yes" : "no")
                  << "  setFilter(400,2600) accepted=" << (accepted ? "yes" : "no")
                  << "  rms before=" << rmsBefore << " after=" << rmsAfter << '\n';
        if (!require(refused, "setFilter accepted inverted passband edges") ||
            !require(accepted, "setFilter refused a valid passband on a "
                               "running channel") ||
            !require(rmsBefore > 0.0 && rmsAfter > 0.0,
                     "a channel that refused a setFilter stopped producing "
                     "output")) {
            return false;
        }
    }

    // CANDIDATE 4 -- the AM leg -- is reported, not asserted into a bound. The
    // point is only that the stage being reachable does not turn the priming
    // stretch into a permanent one.
    if (!require(legAm.firstNonZeroBlock >= 0,
                 "an AM transmit channel produced no output at all")) {
        return false;
    }
    if (!require(legAm.okBlocksAfterFirstNonZeroThatWereZero == 0,
                 "an AM Ok block returned exact zeros after output started")) {
        return false;
    }

    return true;
}

} // namespace

int main()
{
    const uint64_t allocationBaseline = WdspChannel::outstandingAllocationsForTest();
    WdspChannel::Config invalid;
    invalid.inputSampleRate = 44100;
    invalid.dspSampleRate = 48000;
    std::string validationError;
    if (!require(WdspChannel::create(invalid, &validationError) == nullptr,
                 "invalid non-integral rate configuration was accepted") ||
        !runLeakChecked("lifecycle test", runLifecycleTest) ||
        !runLeakChecked("RX vector", [] {
            return runVector(WdspChannel::Direction::Receive);
        }) ||
        !runLeakChecked("TX vector", [] {
            return runVector(WdspChannel::Direction::Transmit);
        }) ||
        !runLeakChecked("TX live geometry", runTransmitLiveGeometryTest) ||
        !runLeakChecked("TX zeros census", runTransmitZerosCensusTest) ||
        !runLeakChecked("underrun test", runUnderrunTest) ||
        !runLeakChecked("reconfiguration test", runReconfigurationTest) ||
        !runLeakChecked("notch index test", runNotchIndexTest) ||
        !runLeakChecked("notch attenuation test", runNotchAttenuationTest) ||
        !require(WdspChannel::outstandingAllocationsForTest() == allocationBaseline,
                 "WDSP test suite left allocations outstanding")) {
        return 1;
    }

    std::cout << "WDSP channel tests passed\n";
    return 0;
}
