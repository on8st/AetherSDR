#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace AetherSDR::hl2 {

// Host VOX for the Hermes-Lite 2: the mic level against a threshold, with a
// hang time. It DECIDES NOTHING ABOUT TRANSMITTING. Its only output is an
// edge -- "the operator started talking" / "the hang ran out" -- which
// Hl2Backend raises as IRadioBackend::voxKeyingRequested, and RadioModel turns
// into an ordinary PTT press through TxController, the TX coordinator and
// every preflight a MOX press passes. A refused press is simply refused.
//
// WHAT IT LISTENS TO is the level Hl2TxDsp's micPeak() measures: the processed
// mic block (AudioEngine's chain, exactly what submitTxAudio() would carry)
// times this radio's own mic gain (micSliderToLinear), BEFORE the ALC. That is
// the level the operator's MIC slider and meter describe, so the VOX threshold
// and the mic meter speak the same dBFS. Measured by the caller; this class
// takes the block peak.
//
// THE CLOCK IS THE AUDIO. The hang counts SAMPLES of mic audio, not wall-clock
// milliseconds: deterministic, testable with synthetic frames, and immune to
// the timer stretching this machine applies to a background session. The one
// thing an audio clock cannot see is audio that STOPS arriving; Hl2Backend
// covers that with a wall-clock backstop that releases the hold, so a lost mic
// device cannot leave the transmitter keyed.
//
// THE LEVEL AND DELAY LAWS are TransmitModel's controls, not measurements:
//   * level 0..100 is sensitivity, as on a Flex (vox_level) and an Icom (VOX
//     GAIN): higher keys on quieter audio. threshold = -0.6 * level dBFS, so 0
//     needs full scale, 50 (the model default) is -30 dBFS, and 100 is -60.
//     A chosen, linear-in-dB law, stated as such -- not fitted to anything.
//   * delay is TransmitModel's raw 0..100, which the model documents as
//     "actual ms = value x 20", so 0..2000 ms of hang.
class Hl2VoxDetector {
public:
    enum class Edge : std::uint8_t { None, Key, Release };

    static constexpr double kDbPerLevelStep = 0.6;
    static constexpr int kMsPerDelayStep = 20;

    [[nodiscard]] static constexpr double thresholdDbfs(int levelPercent) noexcept
    {
        return -kDbPerLevelStep * static_cast<double>(std::clamp(levelPercent, 0, 100));
    }
    [[nodiscard]] static constexpr int hangMsForDelay(int delayRaw) noexcept
    {
        return std::clamp(delayRaw, 0, 100) * kMsPerDelayStep;
    }

    void configure(bool enabled, int levelPercent, int delayRaw) noexcept
    {
        m_enabled = enabled;
        m_levelPercent = std::clamp(levelPercent, 0, 100);
        m_hangMs = hangMsForDelay(delayRaw);
    }

    [[nodiscard]] bool enabled() const noexcept { return m_enabled; }
    [[nodiscard]] bool holding() const noexcept { return m_holding; }
    [[nodiscard]] int levelPercent() const noexcept { return m_levelPercent; }
    [[nodiscard]] int hangMs() const noexcept { return m_hangMs; }

    // One block of mic audio: its peak (linear, post mic gain, pre ALC), how
    // many frames it spans and at what rate, and whether keying is PERMITTED
    // at all right now (connected, transmit allowed, a voice mode, not tuning).
    //
    // `permitted` false, or VOX off, never produces Key; if a hold is running
    // it produces Release at once -- the hang is for speech pauses, not for
    // outliving a reason to stop.
    [[nodiscard]] Edge feed(double peakLinear, int frames, int sampleRateHz, bool permitted) noexcept
    {
        if (!m_enabled || !permitted) {
            return drop();
        }
        if (frames <= 0 || sampleRateHz <= 0) {
            return Edge::None;
        }
        const bool above = peakLinear > 0.0
            && 20.0 * std::log10(peakLinear) >= thresholdDbfs(m_levelPercent);
        if (above) {
            m_hangRemainingFrames =
                static_cast<std::int64_t>(m_hangMs) * sampleRateHz / 1000;
            if (!m_holding) {
                m_holding = true;
                return Edge::Key;
            }
            return Edge::None;
        }
        if (!m_holding) {
            return Edge::None;
        }
        m_hangRemainingFrames -= frames;
        if (m_hangRemainingFrames <= 0) {
            m_holding = false;
            m_hangRemainingFrames = 0;
            return Edge::Release;
        }
        return Edge::None;
    }

    // Forget any hold. Release if one was running, so the caller unkeys it.
    [[nodiscard]] Edge drop() noexcept
    {
        m_hangRemainingFrames = 0;
        if (m_holding) {
            m_holding = false;
            return Edge::Release;
        }
        return Edge::None;
    }

private:
    bool m_enabled = false;     // off until the operator switches it on
    int m_levelPercent = 50;    // TransmitModel defaults
    int m_hangMs = 50 * kMsPerDelayStep;
    bool m_holding = false;
    std::int64_t m_hangRemainingFrames = 0;
};

}  // namespace AetherSDR::hl2
