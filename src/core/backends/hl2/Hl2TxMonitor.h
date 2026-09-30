#pragma once

#include <algorithm>
#include <span>
#include <vector>

namespace AetherSDR::hl2 {

// The operator's MON switch and level on a radio whose modulator runs HERE.
//
// A Flex monitors on the radio: `transmit set mon=1` and `mon_gain_sb=N` feed
// its own post-processing transmit audio into its own speaker path. An HL2 has
// no speaker path and no modulator of its own -- Hl2TxDsp runs on this
// computer -- so the only place a transmit monitor can exist is the same
// place the receive audio already comes out: this computer's output.
//
// WHAT IS MONITORED is the post-ALC, post-clamp level Hl2TxDsp hands the
// modulator (its m_levelled block, the one alcPeak() measures). Not the raw
// mic: the point of MON is to hear what goes on the air, and on this backend
// that includes the HL2's own mic gain (up to +40 dB) and the ALC's reduction,
// neither of which the mic capture has seen yet.
//
// WHEN it is heard is the gate below, and every term in it is a refusal:
//   * MON off                         -> nothing
//   * not keyed                        -> nothing (queued blocks that arrive
//                                         after the unkey are dropped, so the
//                                         monitor cannot outlive the over)
//   * the diagnostic TX-audio monitor  -> nothing: setTxAudioMonitor() exists
//     is on                               so a measurement can demodulate our
//                                         own signal, and mixing the MON copy
//                                         into that capture would corrupt the
//                                         measurement it was turned on for
//   * CW                               -> nothing: CW has its own sidetone, and
//                                         the mic audio Hl2TxDsp may still be
//                                         levelling during a CW over is not
//                                         what is on the air
//
// LEVEL is 0..100 -> 0.0..1.0 LINEAR, the same law Hl2Backend::
// setSliceAudioGain() uses for the receive fader, so MON at 100 sits at the
// same loudness as a full-scale receive slice at 100 and the two faders mean
// the same thing. No dB curve is invented here.
//
// Pure: no Qt, no thread, no clock. Hl2Backend owns one on its own thread.
class Hl2TxMonitor {
public:
    void setMonitor(bool on, int levelPercent) noexcept
    {
        m_on = on;
        m_levelPercent = std::clamp(levelPercent, 0, 100);
    }

    [[nodiscard]] bool enabled() const noexcept { return m_on; }
    [[nodiscard]] int levelPercent() const noexcept { return m_levelPercent; }
    [[nodiscard]] float gain() const noexcept { return gainForLevel(m_levelPercent); }

    [[nodiscard]] static constexpr float gainForLevel(int levelPercent) noexcept
    {
        return static_cast<float>(std::clamp(levelPercent, 0, 100)) / 100.0f;
    }

    // Whether a block delivered NOW may be heard. Evaluated at DELIVERY, not
    // when the block was levelled, so a block in flight across the unkey is
    // judged by the state after it.
    [[nodiscard]] bool audible(bool keyed, bool diagnosticMonitor, bool cwMode) const noexcept
    {
        return m_on && keyed && !diagnosticMonitor && !cwMode && m_levelPercent > 0;
    }

    // Post-ALC mono -> interleaved stereo float at the MON level, the shape
    // IRadioBackend::publishLegacyAudio() carries (24 kHz, L/R). Empty when the
    // gate above says the block must not be heard, so the caller publishes
    // nothing rather than a block of zeros: a gap lets the engine's playout
    // buffer behave as it does for any other absent audio, and zeros would be
    // a claim that the transmitter is silent.
    [[nodiscard]] std::vector<float> render(std::span<const float> postAlcMono,
                                            bool keyed, bool diagnosticMonitor,
                                            bool cwMode) const
    {
        std::vector<float> stereo;
        if (!audible(keyed, diagnosticMonitor, cwMode) || postAlcMono.empty()) {
            return stereo;
        }
        const float g = gain();
        stereo.resize(postAlcMono.size() * 2);
        for (std::size_t i = 0; i < postAlcMono.size(); ++i) {
            // Post-ALC is already inside [-1, 1] (Hl2TxDsp's hard clamp), and
            // g <= 1, so no clamp is needed here and none is added to hide one.
            const float s = postAlcMono[i] * g;
            stereo[2 * i] = s;
            stereo[2 * i + 1] = s;
        }
        return stereo;
    }

private:
    bool m_on = false;          // off until the operator asks, as on a Flex
    int m_levelPercent = 50;    // TransmitModel's m_monGainSb default
};

}  // namespace AetherSDR::hl2
