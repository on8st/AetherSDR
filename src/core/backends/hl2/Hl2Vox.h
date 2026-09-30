#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <utility>

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
    // Blocks that cleared the threshold but were refused a Key by anti-VOX.
    [[nodiscard]] std::uint64_t antiVoxSuppressed() const noexcept { return m_antiVoxSuppressed; }

    // One block of mic audio: its peak (linear, post mic gain, pre ALC), how
    // many frames it spans and at what rate, and whether keying is PERMITTED
    // at all right now (connected, transmit allowed, a voice mode, not tuning).
    //
    // `permitted` false, or VOX off, never produces Key; if a hold is running
    // it produces Release at once -- the hang is for speech pauses, not for
    // outliving a reason to stop.
    //
    // `antiVoxReferenceLinear` is ANTI-VOX (Hl2AntiVox below): a NEW key needs
    // the block to clear the threshold AND to exceed this reference. It gates
    // the Key edge ONLY. A running hold renews its hang on the threshold alone
    // and releases exactly as before, so anti-VOX can never lengthen an over
    // and never cut one short. 0 (the default) is no anti-VOX; +infinity
    // refuses every new key for as long as the caller passes it.
    [[nodiscard]] Edge feed(double peakLinear, int frames, int sampleRateHz, bool permitted,
                            double antiVoxReferenceLinear = 0.0) noexcept
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
            if (!m_holding && !(peakLinear > antiVoxReferenceLinear)) {
                // Loud enough to key, but not louder than what our own output
                // may be putting into the microphone. No hold starts, so the
                // next block is judged afresh.
                ++m_antiVoxSuppressed;
                return Edge::None;
            }
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
    std::uint64_t m_antiVoxSuppressed = 0;
};

// ANTI-VOX: the reference Hl2VoxDetector::feed compares a new key against --
// what OUR OWN OUTPUT may be putting into the microphone. A Flex has it on the
// radio (anti_vox_level), an Icom as ANTI VOX; the HL2's host VOX had none,
// and d167 D-vox (2026-09-30, hl2-lab d167-vox-chatter.md) is what that cost:
// after the operator stopped talking VOX re-keyed 14 times in 14 at
// 0.246-0.356 s after each of its own releases, at -17.9 to -27.1 dBFS, 2.9-12.1
// dB over the threshold. Speech does not lock to our T/R edge; our output does.
//
// WHICH PROPERTY OF THE OUTPUT IS THE STIMULUS IS NOT ESTABLISHED, so this
// class carries both candidates and says which one is on:
//
//   RESTART (on). For kRestartHoldMs after our output restarts -- the unkey
//   edge, and again when the receive audio resumes after the unkey hold --
//   the reference is +infinity: no NEW key. This is the term the d167 data
//   supports, because it is the time-lock: 70 ms unmute hold + 176-286 ms
//   => re-key at 246-356 ms after the release. And it is the only term that
//   separates d167 at all: the operator's own key asks were at -21.1 and -22.5
//   dBFS, INSIDE the chatter's -17.9..-27.1, so no level floor can tell them
//   apart. Speech that starts inside the hold is delayed, not lost: it keys on
//   the first loud block after the hold if it is still going.
//
//   LEVEL (off until measured). The peak of what we published to the output
//   in the last kLevelWindowMs, times a gain: the Flex-style anti-VOX for a
//   stimulus that scales with our output. OFF by default because its gain is
//   unmeasured, and on d167's figures it would have to bridge a -45 dBFS
//   output to a -18 dBFS microphone, +27 dB -- a gain that would also have
//   refused the operator's own -21 dBFS key asks while receive audio played.
//   setLevelGainDb() turns it on when a receive-only measurement of the
//   output-to-microphone path supplies the number.
//
// kRestartHoldMs = 850 is CHOSEN, bounded below by d167: the latest re-key
// came 286 ms after the receive audio resumed (356 - 70), and the stimulus
// then stayed over the threshold for up to ~0.53 s (the tail each chatter over
// ran beyond its hang). 286 + 530 = 816 ms; 850 covers it with 34 ms. One
// radio, one headset (AirPods Max over HFP, 165 ms reported output latency),
// one evening. A shorter value is choosing to let d167's chatter back in; a
// longer one clips more of an operator who resumes quickly after a pause
// longer than the VOX delay. The cost is real and it is this: no NEW VOX key
// for 850 ms after any unkey (920 ms after MOX-off, counting the unmute hold).
//
// THE CLOCK IS THE CALLER'S steady clock in nanoseconds, passed in, so the
// class is deterministic under test. It keeps no thread of its own.
class Hl2AntiVox {
public:
    static constexpr int kRestartHoldMs = 850;
    static constexpr int kLevelWindowMs = 850;

    // Our output (re)started after a gap or a change of what it plays: the
    // unkey edge, and the receive audio resuming after the unkey hold. A later
    // restart extends the hold; it never shortens one already running.
    void noteOutputRestart(std::int64_t nowNs) noexcept
    {
        const std::int64_t until = nowNs + static_cast<std::int64_t>(kRestartHoldMs) * 1'000'000;
        m_restartHoldUntilNs = std::max(m_restartHoldUntilNs, until);
        ++m_restarts;
    }

    // One block of what we handed the output device, as its linear peak. Only
    // kept while the level term is on.
    void noteOutputPeak(double peakLinear, std::int64_t nowNs)
    {
        if (m_levelGainLinear <= 0.0) {
            return;
        }
        const std::int64_t horizon = nowNs - static_cast<std::int64_t>(kLevelWindowMs) * 1'000'000;
        while (!m_peaks.empty() && m_peaks.front().first < horizon) {
            m_peaks.pop_front();
        }
        // Monotone: anything older AND no louder can never be the maximum again.
        while (!m_peaks.empty() && m_peaks.back().second <= peakLinear) {
            m_peaks.pop_back();
        }
        m_peaks.emplace_back(nowNs, peakLinear);
    }

    // Any finite gain turns the level term on (0 dB: the microphone must beat
    // our output's own digital peak); setLevelOff() turns it off and forgets
    // what it held.
    void setLevelGainDb(double db)
    {
        m_levelGainLinear = std::pow(10.0, db / 20.0);
    }
    void setLevelOff()
    {
        m_levelGainLinear = 0.0;
        m_peaks.clear();
    }
    [[nodiscard]] bool levelOn() const noexcept { return m_levelGainLinear > 0.0; }

    [[nodiscard]] bool restartHoldActive(std::int64_t nowNs) const noexcept
    {
        return nowNs < m_restartHoldUntilNs;
    }
    [[nodiscard]] std::int64_t restartHoldRemainingMs(std::int64_t nowNs) const noexcept
    {
        return restartHoldActive(nowNs) ? (m_restartHoldUntilNs - nowNs) / 1'000'000 : 0;
    }
    [[nodiscard]] std::uint64_t restarts() const noexcept { return m_restarts; }

    // What Hl2VoxDetector::feed's new key must exceed, right now.
    [[nodiscard]] double referenceLinear(std::int64_t nowNs) const noexcept
    {
        if (restartHoldActive(nowNs)) {
            return std::numeric_limits<double>::infinity();
        }
        if (m_levelGainLinear <= 0.0) {
            return 0.0;
        }
        const std::int64_t horizon = nowNs - static_cast<std::int64_t>(kLevelWindowMs) * 1'000'000;
        for (const auto& [at, peak] : m_peaks) {
            if (at >= horizon) {
                return peak * m_levelGainLinear;   // the deque is monotone: first live entry is the max
            }
        }
        return 0.0;
    }

private:
    std::int64_t m_restartHoldUntilNs = std::numeric_limits<std::int64_t>::min();
    std::uint64_t m_restarts = 0;
    double m_levelGainLinear = 0.0;   // 0 = level term off
    std::deque<std::pair<std::int64_t, double>> m_peaks;
};

}  // namespace AetherSDR::hl2
