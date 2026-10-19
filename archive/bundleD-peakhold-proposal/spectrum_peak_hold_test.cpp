// The panadapter peak hold (gui/SpectrumPeakHold.h).
//
// Two of the cases below are the whole reason this file exists, and neither is
// about peak hold being drawn correctly — they are about it being CORRECT TO
// COMPUTE WHERE IT IS COMPUTED:
//
//   - "max in dB equals dB of max" is what separates this control from
//     spectrum AVERAGING. Upstream #5782 establishes that an average taken on
//     emitted dBFS computes a geometric mean and biases a noise floor by a
//     fixed -2.51 dB, which is why the averaging accumulator for rows 2.1/2.2
//     belongs below the family seam. A maximum is exact under a monotonic log,
//     so this one does not. If that ever stopped being true, a peak hold above
//     the seam would be as wrong as the averaging is, and silently so: on a
//     steady carrier the two agree, which is precisely the case anybody would
//     test by hand.
//
//   - "the same elapsed time decays the same amount however many frames carry
//     it" is what makes the hold SHARED. Frame cadence is not a constant here:
//     a Flex sends what its own display engine produced, the raw-spectrum
//     backends shape their rate at the source through setPanFrameRate, and the
//     operator moves that with the FFT FPS control. A hold that leaked per
//     frame would fall at a different rate on every backend and would change
//     rate under the operator whenever they touched an unrelated slider. That
//     is the shape of bug that looks fine on the family it was written against.
//
// SpectrumWidget evaluates these same functions rather than its own copy. It
// has to: no target in tests/ can instantiate SpectrumWidget — it pulls in
// QRhi, the shaders and aethercore — so a rule written inline at the draw site
// would have no offline guard at all.

#include "gui/SpectrumPeakHold.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

using AetherSDR::spectrum::decayForElapsed;
using AetherSDR::spectrum::kNoDecay;
using AetherSDR::spectrum::PeakHold;
using AetherSDR::spectrum::peakHoldStep;

namespace {

int g_failures = 0;

void check(bool ok, const char* what)
{
    std::printf("%s %s\n", ok ? "[ OK ]" : "[FAIL]", what);
    if (!ok)
        ++g_failures;
}

bool near(double a, double b, double tol = 1e-4)
{
    return std::fabs(a - b) <= tol;
}

// ---- 1. The domain property this control rests on --------------------------

void testMaxIsExactUnderTheLog()
{
    // Powers spanning 11 decades, deliberately including a single burst far
    // above a cluster of floor-like values -- #5782's worked example shape,
    // where a dB-domain AVERAGE throws away 39 dB of a 60 dB event.
    const std::vector<double> powers = {
        1e-10, 1e-10, 1e-10, 1e-4, 1e-10, 3.7e-9, 1e-10, 2.2e-7,
    };

    double maxPower = 0.0;
    for (double p : powers)
        maxPower = p > maxPower ? p : maxPower;
    const double dbOfMaxPower = 10.0 * std::log10(maxPower);

    // Now run the hold over the dB values, which is what the display path has.
    float held = -std::numeric_limits<float>::infinity();
    for (double p : powers)
        held = peakHoldStep(held, static_cast<float>(10.0 * std::log10(p)), kNoDecay);

    check(near(held, dbOfMaxPower, 1e-3),
          "max of dB equals dB of max power (the hold is exact above the seam)");
    check(near(held, -40.0, 1e-3),
          "the 60 dB burst is held at its full height, not averaged away");
}

// ---- 2. Attack -------------------------------------------------------------

void testInstantAttack()
{
    check(near(peakHoldStep(-100.0f, -40.0f, kNoDecay), -40.0f),
          "a rise is taken in one step (instant attack)");
    check(near(peakHoldStep(-100.0f, -40.0f, 50.0f), -40.0f),
          "a rise is taken in one step even with a large decay allowance");

    PeakHold hold;
    const std::vector<float> floorFrame(8, -110.0f);
    std::vector<float> burst(8, -110.0f);
    burst[3] = -30.0f;

    hold.update(floorFrame.data(), floorFrame.size(), 0.04);
    hold.update(burst.data(), burst.size(), 0.04);
    hold.update(floorFrame.data(), floorFrame.size(), 0.04);

    check(near(hold.held()[3], -30.0f), "a one-frame burst survives the frames after it");
    check(near(hold.held()[0], -110.0f), "bins that never rose are untouched");
}

// ---- 3. The default is an infinite hold ------------------------------------

void testDefaultNeverDecays()
{
    PeakHold hold;
    check(hold.decayDbPerSec() == kNoDecay, "the default decay rate is zero");

    std::vector<float> burst(4, -120.0f);
    burst[1] = -20.0f;
    hold.update(burst.data(), burst.size(), 0.0);

    const std::vector<float> floorFrame(4, -120.0f);
    for (int i = 0; i < 100; ++i)
        hold.update(floorFrame.data(), floorFrame.size(), 3600.0);

    check(near(hold.held()[1], -20.0f),
          "an infinite hold survives 100 hours of floor frames");
}

// ---- 4. The property that makes the hold SHARED ----------------------------

void testDecayIsPerSecondNotPerFrame()
{
    // 10 dB/s for one second, delivered as one frame, as ten, and as a hundred.
    // A Flex, an HL2 at 25 fps and an HL2 at 60 fps must all land in the same
    // place.
    const std::vector<float> floorFrame(1, -100.0f);
    const std::vector<float> peakFrame(1, 0.0f);

    auto runAtCadence = [&](int frames) {
        PeakHold hold;
        hold.setDecayDbPerSec(10.0f);
        hold.update(peakFrame.data(), peakFrame.size(), 0.0);
        const double step = 1.0 / static_cast<double>(frames);
        for (int i = 0; i < frames; ++i)
            hold.update(floorFrame.data(), floorFrame.size(), step);
        return hold.held()[0];
    };

    const float one = runAtCadence(1);
    const float ten = runAtCadence(10);
    const float hundred = runAtCadence(100);

    check(near(one, -10.0f, 1e-3), "10 dB/s falls 10 dB in one second (1 frame)");
    check(near(ten, -10.0f, 1e-3), "...the same 10 dB across 10 frames");
    check(near(hundred, -10.0f, 1e-3), "...and the same 10 dB across 100 frames");
    check(near(one, ten, 1e-3) && near(ten, hundred, 1e-3),
          "the fall is independent of frame cadence (shared across backends)");
}

// ---- 5. A decaying hold never undershoots the live trace -------------------

void testHoldNeverFallsBelowTheTrace()
{
    check(near(peakHoldStep(-50.0f, -55.0f, 100.0f), -55.0f),
          "a huge decay allowance stops at the instantaneous value");

    PeakHold hold;
    hold.setDecayDbPerSec(1000.0f);
    std::vector<float> frame(4, -70.0f);
    hold.update(frame.data(), frame.size(), 0.0);
    for (int i = 0; i < 20; ++i)
        hold.update(frame.data(), frame.size(), 1.0);

    bool allAtTrace = true;
    for (float v : hold.held())
        allAtTrace = allAtTrace && near(v, -70.0f);
    check(allAtTrace, "the hold settles ON the trace, never beneath it");
}

// ---- 6. A clock that misbehaves must not raise the hold --------------------

void testBadElapsedCannotRaiseTheHold()
{
    check(decayForElapsed(10.0f, -1.0) == 0.0f, "a negative elapsed time yields no decay");
    check(decayForElapsed(10.0f, 0.0) == 0.0f, "a zero elapsed time yields no decay");
    check(decayForElapsed(10.0f, std::numeric_limits<double>::quiet_NaN()) == 0.0f,
          "a NaN elapsed time yields no decay");
    check(decayForElapsed(-5.0f, 1.0) == 0.0f, "a negative rate yields no decay");

    PeakHold hold;
    hold.setDecayDbPerSec(-5.0f);
    check(hold.decayDbPerSec() == kNoDecay, "a negative decay rate clamps to no decay");
}

// ---- 7. Geometry changes reseed rather than smear ---------------------------

void testBinCountChangeReseeds()
{
    PeakHold hold;
    std::vector<float> wide(8, -120.0f);
    wide[0] = -10.0f;  // a strong peak at the low edge of the old span
    hold.update(wide.data(), wide.size(), 0.0);
    check(near(hold.held()[0], -10.0f), "the old span's peak is held");

    const std::vector<float> narrow(4, -100.0f);
    hold.update(narrow.data(), narrow.size(), 0.04);

    check(hold.size() == 4, "the hold follows the new bin count");
    check(near(hold.held()[0], -100.0f),
          "the old span's peak does NOT smear onto the new span's low edge");
}

void testResetAndEmptyInput()
{
    PeakHold hold;
    const std::vector<float> frame(4, -50.0f);
    hold.update(frame.data(), frame.size(), 0.0);
    check(!hold.empty(), "a fed hold is not empty");

    hold.reset();
    check(hold.empty(), "reset clears the hold");

    check(!hold.update(nullptr, 0, 0.04), "a null frame is refused");
    check(hold.empty(), "...and leaves the hold empty");
}

}  // namespace

int main()
{
    testMaxIsExactUnderTheLog();
    testInstantAttack();
    testDefaultNeverDecays();
    testDecayIsPerSecondNotPerFrame();
    testHoldNeverFallsBelowTheTrace();
    testBadElapsedCannotRaiseTheHold();
    testBinCountChangeReseeds();
    testResetAndEmptyInput();

    std::printf("%s: %d failure(s)\n", g_failures == 0 ? "PASS" : "FAIL", g_failures);
    return g_failures == 0 ? 0 : 1;
}
