#pragma once

// Per-bin peak hold for the panadapter trace, as pure functions plus the small
// amount of state they need.
//
// HERE RATHER THAN IN SpectrumWidget for the reason SpectrumPreviewLogic.h
// already states about itself: keeping the rule here rather than inline at the
// draw site is what lets a test pin the behaviour the renderer actually runs.
// SpectrumWidget cannot be instantiated by any target in tests/ today — it
// pulls in QRhi, the shaders and aethercore — so a rule written inside it has
// no offline guard at all.
//
// THIS HEADER IS THE HOLD ONLY. It draws nothing and knows nothing about
// either render path. The render half is deliberately not here; see the note
// at the end of this comment.
//
// ── Why a peak hold may live above the family seam, when averaging may not ──
//
// This is the property the whole design rests on, and it is arithmetic rather
// than taste.
//
// Upstream #5782 establishes that spectrum AVERAGING must not be computed on
// the emitted dBFS values: the mean of logarithms is the logarithm of the
// geometric mean, so a control labelled "average the power over N frames"
// would compute something else — 39 dB of a 60 dB event thrown away in that
// RFC's worked example, plus a fixed −2.51 dB bias on a Gaussian noise floor.
// That is why the accumulator for rows 2.1 and 2.2 belongs below the seam,
// inside the spectrum object, where the magnitudes still exist.
//
// A MAXIMUM has no such defect. log is strictly monotonic, so
//
//     max(10*log10(p_1) .. 10*log10(p_n))  ==  10*log10(max(p_1 .. p_n))
//
// exactly, for every input, with no bias and no dependence on how the signal
// varies. A peak hold computed here on dBFS is bit-for-bit the peak hold that
// would be computed on power below the seam. It is therefore the one member of
// this family of controls that is CORRECT above the seam, and #5782 does not
// reach it: that RFC rules on where averaging lives and in which domain, and
// names rows 2.1 and 2.2. It does not name this one, because there is nothing
// to rule on.
//
// ── Why the decay is per SECOND and not per FRAME ───────────────────────────
//
// This is the property that makes the hold behave the same on every backend,
// and getting it wrong is the specific way a shared display feature breaks one
// family while looking right on another.
//
// Frames do not arrive at one rate. A Flex runs its own display engine and
// sends frames at the rate the radio was asked for; the raw-spectrum backends
// (HL2, ANAN, RTL) shape their own cadence at the source via
// IRadioBackend::setPanFrameRate, and the operator moves that with the FFT FPS
// control. A decay expressed per frame therefore falls at a different rate on
// every backend, and changes rate under the operator whenever they touch a
// control that has nothing to do with peak hold.
//
// Expressed per second it is the same hold everywhere, which is the only
// version of "shared" worth having. The caller supplies the elapsed time; this
// header does not read a clock, so a test can drive it deterministically.
//
// ── The default is an INFINITE hold, and that is a refusal to invent a number
//
// kDefaultDecayDbPerSec is 0 — the hold never decays and is cleared only by an
// explicit reset or a geometry change. That is what "peak hold" means on a
// spectrum analyser, and more to the point it is the only setting that carries
// no free parameter.
//
// A leaky hold is supported (any positive dB/s) and may well be what an
// operator wants, but there is no defensible number for it in this repository.
// The one adjacent precedent is AdaptiveFilterEngine's kHoldFrames = 75,
// "~2.5 s edge peak-hold" — a different surface with a different job, offered
// here as the only in-tree prior art rather than as a derivation. Picking the
// default leak is a maintainer's call, not this header's.
//
// ── What is NOT here, and why ───────────────────────────────────────────────
//
// The render half. AETHER_GPU_SPECTRUM defaults ON (CMakeLists.txt), so in a
// normal build the trace is a fragment shader (resources/shaders/panscope.frag)
// fed a width x 1 R32F texture from SpectrumWidget::renderGpuFrame, and
// SpectrumWidget::drawSpectrum — the QPainterPath path — has exactly one
// caller, inside the #else-branch paintEvent. An overlay added only to
// drawSpectrum is invisible in the shipped build. Drawing a held trace
// therefore means a second RHI draw or a per-frame overlay upload, and a
// visible second trace on the panadapter engages GOVERNANCE.md's first RFC
// bullet ("Any change to visual design"). Splitting the part that is invariant
// under that ruling, and landing it tested, is the same move #5794 made out of
// #5782.

#include <cstddef>
#include <vector>

namespace AetherSDR::spectrum {

// No decay: the hold rises and never falls on its own.
inline constexpr float kNoDecay = 0.0f;

// See the header comment: a refusal to invent a leak rate, not a preference
// for infinite holds.
inline constexpr float kDefaultDecayDbPerSec = kNoDecay;

// One bin, one step, in dB.
//
// Instant attack, linear release — the same shape as
// Hl2TxLevelPolicy::fwdPeakHoldStep, which holds the transmit forward power,
// and for the same reason: a smoothed attack cannot hold a peak it did not
// reach in one step, and a peak that arrived is the entire point.
//
// The release is linear in dB rather than exponential because the display axis
// IS dB. A linear-in-dB fall is a straight line on screen at a stated dB per
// second, which is a rate an operator can predict without measuring — the same
// argument WaterfallRate.h makes for its own linear law.
//
// `decayDb` is the fall allowed for THIS step (dB/s multiplied by the elapsed
// seconds), not a rate. Keeping the rate-to-step conversion outside lets this
// stay constexpr and lets the test drive irregular frame timing directly.
//
// The hold never falls below the instantaneous value: a decaying hold that
// undershot would draw a line beneath the live trace, which reads as a second
// signal rather than as a memory of the first.
[[nodiscard]] constexpr float peakHoldStep(float heldDb,
                                           float instantDb,
                                           float decayDb) noexcept
{
    if (instantDb >= heldDb)
        return instantDb;
    if (decayDb <= 0.0f)
        return heldDb;
    const float decayed = heldDb - decayDb;
    return decayed < instantDb ? instantDb : decayed;
}

// A decay RATE and an elapsed interval as the fall allowed for one step.
//
// Negative or non-finite elapsed times yield no decay rather than a rise: a
// clock that went backwards (or a first frame with no predecessor) must not be
// able to push the hold UP, which is the one direction it can never be
// corrected from without a reset.
[[nodiscard]] constexpr float decayForElapsed(float decayDbPerSec,
                                              double elapsedSec) noexcept
{
    if (decayDbPerSec <= 0.0f || !(elapsedSec > 0.0))
        return 0.0f;
    return static_cast<float>(static_cast<double>(decayDbPerSec) * elapsedSec);
}

// The held trace, and the rules for when it must be thrown away.
//
// Ownership note for the caller: the bin grid this holds against is the grid it
// was last fed. SpectrumWidget already tracks exactly this — a geometry change
// either reprojects the live trace (SpectrumWidget::reprojectSpectrum, whose
// reprojectBins lambda is one line away from doing the same for a held vector)
// or gives up and sets m_resetFftSmoothingOnNextFrame. A held trace must follow
// whichever of those the live trace took, or it will draw one span's maxima
// over another span's frequencies.
class PeakHold {
  public:
    [[nodiscard]] float decayDbPerSec() const noexcept { return m_decayDbPerSec; }

    // A negative rate is clamped to kNoDecay. There is no meaning for a hold
    // that rises on its own.
    void setDecayDbPerSec(float dbPerSec) noexcept
    {
        m_decayDbPerSec = dbPerSec > 0.0f ? dbPerSec : kNoDecay;
    }

    [[nodiscard]] bool empty() const noexcept { return m_held.empty(); }
    [[nodiscard]] std::size_t size() const noexcept { return m_held.size(); }
    [[nodiscard]] const std::vector<float>& held() const noexcept { return m_held; }

    // Throw the hold away. The next frame seeds it.
    void reset() noexcept { m_held.clear(); }

    // Fold one frame in. Returns true if `held()` is now a valid trace of the
    // same length as the input.
    //
    // A BIN-COUNT CHANGE RESEEDS RATHER THAN RESIZES, and it is the case most
    // likely to be got wrong by resizing: the bin count changes when the span
    // or the FFT length changes, and after either of those the stored maxima
    // describe different frequencies. Keeping the overlapping prefix would
    // smear the old span's peaks across the new one's low edge, which looks
    // like a real signal and is not. SpectrumWidget::updateSpectrum takes the
    // same branch for m_smoothed for the same reason.
    bool update(const float* bins, std::size_t count, double elapsedSec)
    {
        if (bins == nullptr || count == 0) {
            reset();
            return false;
        }
        if (m_held.size() != count) {
            m_held.assign(bins, bins + count);
            return true;
        }
        const float decayDb = decayForElapsed(m_decayDbPerSec, elapsedSec);
        for (std::size_t i = 0; i < count; ++i)
            m_held[i] = peakHoldStep(m_held[i], bins[i], decayDb);
        return true;
    }

  private:
    std::vector<float> m_held;
    float m_decayDbPerSec{kDefaultDecayDbPerSec};
};

}  // namespace AetherSDR::spectrum
