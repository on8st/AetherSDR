#pragma once

#include <atomic>

// ── The transmit voice chain's latency, published for the key-down log ──────
//
// TxVoiceProcessor::latencyFrames() is the configured delay of the microphone
// chain -- serial SRC group delay, RNNoise's frame, the gate's lookahead -- in
// 48 kHz frames. It had no consumer anywhere in the tree (#6052's triage),
// which left the upstream half of the HL2 key-down budget as arithmetic.
//
// It lives on the audio thread (AudioEngine owns the processor) and the figure
// it belongs next to, MetisClient's txUnderflowSamples, lives on the HL2 I/O
// thread. Nothing else crosses between the two, so this is one relaxed atomic
// rather than a new seam signal: AudioEngine stores after every processed
// capture block, and MetisClient reads it once per over, at key-down, for
// lcHl2Tx. -1 means nothing has been published, or that the transmit audio is
// not on the microphone path (DAX/TCI audio does not pass through the voice
// processor), and the log says so rather than printing a number.
namespace AetherSDR::TxChainLatency {

inline std::atomic<int> g_voiceProcessorFrames{-1};

inline void publishVoiceProcessorFrames(int frames) noexcept
{
    g_voiceProcessorFrames.store(frames, std::memory_order_relaxed);
}

[[nodiscard]] inline int voiceProcessorFrames() noexcept
{
    return g_voiceProcessorFrames.load(std::memory_order_relaxed);
}

}  // namespace AetherSDR::TxChainLatency
