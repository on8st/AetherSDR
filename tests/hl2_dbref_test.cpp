// Every LNA gain change shifts the absolute signal reference by exactly the same
// amount. If the panadapter displays dBm and the gain moves, the whole trace
// jumps and the waterfall paints a horizontal band that reads as a real on-air
// event. Hl2DbReference exists so the gain value and its compensating offset
// live in one object and cannot drift apart.
//
// The same gain change also moves the AGC ceiling's footing -- the half the
// operator hears rather than sees -- so this pins that too.
//
// This asserts the property that matters: a signal of CONSTANT strength reports
// a CONSTANT dBm across a gain change.

#include "core/backends/hl2/Hl2DbReference.h"

#include <cmath>
#include <cstdio>

using AetherSDR::hl2::Hl2DbReference;

static int g_failures = 0;
static void check(bool ok, const char* what)
{
    if (!ok) { std::fprintf(stderr, "FAIL: %s\n", what); ++g_failures; }
}
static bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) < tol; }

int main()
{
    Hl2DbReference ref;

    // ---------------------------------------------------------------
    // THE ABSOLUTE ANCHOR
    //
    // The four assertions this block replaces were a deliberate tripwire, not
    // stale expectations. They said: this axis is dBFS wearing a dBm label, we
    // know it, and nothing may quietly change that. One of them guarded the
    // exact regression that got the absolute form REVERTED once -- "default
    // gain leaves the displayed floor exactly where it was".
    //
    // The tripwire is crossed ON PURPOSE, and what makes that legitimate is the
    // thing the revert was missing: kFullScaleDbmAtZeroGain is DERIVED from the
    // AD9866 datasheet and the HL2's own input network rather than being a
    // second arbitrary number. The floor still moves. It now moves onto a
    // figure that can be checked.
    //
    // If this block ever fails, the question is not "has the arithmetic
    // drifted" but "has the DERIVATION been falsified" -- and the answer
    // belongs beside kFullScaleDbmAtZeroGain, not here.
    // ---------------------------------------------------------------
    check(near(Hl2DbReference::kFullScaleDbmAtZeroGain, 0.97, 5e-3),
          "full scale at 0 dB LNA gain is the derived +0.97 dBm -- 2.0 Vpp "
          "differential into 400 ohm is 1.25 mW, and the matched input "
          "transformer conserves power");
    check(near(ref.fullScaleDbm(), Hl2DbReference::kFullScaleDbmAtZeroGain),
          "a fresh reference carries the derived figure, not 0.0");

    // DERIVED IS NOT CALIBRATED, and this pair is what keeps the two apart.
    // Moving the default off 0.0 while isCalibrated() was still
    // `m_fullScaleDbm != 0.0` would flip the predicate TRUE for a radio nobody
    // has ever measured -- and Hl2Backend::capabilities() publishes it as
    // PanAmplitudeModel::calibratedDbm, which licenses comparing this radio's
    // levels with another station's. The derivation does not license that; a
    // measurement does. NOTHING WAS MEASURED FOR THIS CHANGE.
    check(!ref.isCalibrated(),
          "a derived default is NOT a calibration -- nothing has measured this "
          "radio");

    // ...and the setter is the only thing that changes the answer, which is
    // what makes the predicate mean PROVENANCE rather than "a number is
    // present". Both directions are pinned, because a value comparison gets
    // each of them wrong in turn.
    {
        Hl2DbReference measured;
        measured.setFullScaleDbm(Hl2DbReference::kFullScaleDbmAtZeroGain);
        check(measured.isCalibrated(),
              "applying a measurement calibrates it -- EVEN WHEN THE MEASURED "
              "FIGURE EQUALS THE DERIVED ONE, which is why this is a flag and "
              "not a comparison against the constant");

        Hl2DbReference elsewhere;
        elsewhere.setFullScaleDbm(0.0);
        check(elsewhere.isCalibrated(),
              "and a measurement of 0.0 dBm calibrates it too -- the old "
              "`!= 0.0` predicate got this one wrong in the other direction");
    }

    // P(dBm) = dBFS + fullScale - Glna, ABSOLUTELY. The relative form -- which
    // is what this replaces -- would give fullScale + (reference - live), i.e.
    // +0.97 at the default gain, so these two assertions fail if offsetDb()
    // goes back to it.
    ref.setLnaGainDb(Hl2DbReference::kDefaultLnaGainDb);
    check(near(ref.offsetDb(),
               Hl2DbReference::kFullScaleDbmAtZeroGain
                   - Hl2DbReference::kDefaultLnaGainDb),
          "offset at the default gain is fullScale - gain, not fullScale plus a "
          "referral that cancels");
    check(near(ref.toDbm(-100.0), -100.0 + 0.97 - 20.0, 5e-3),
          "-100 dBFS at the default 20 dB of gain reads -119.03 dBm");

    // THE STEP, ASSERTED SO IT CANNOT ARRIVE SILENTLY. Before this change
    // toDbm() was the identity at the default gain; it is now 19.03 dB lower,
    // and the shift is the same at EVERY gain because both forms subtract the
    // live gain.
    check(near(Hl2DbReference::kFullScaleDbmAtZeroGain
                   - Hl2DbReference::kDefaultLnaGainDb,
               -19.03, 5e-3),
          "the displayed floor drops 19.03 dB from the old identity mapping");
    for (const double gain : {-12.0, 0.0, 20.0, 48.0}) {
        Hl2DbReference stepped;
        stepped.setLnaGainDb(gain);
        const double oldOffset = Hl2DbReference::kDefaultLnaGainDb - gain;  // the relative form
        check(near(stepped.offsetDb() - oldOffset,
                   Hl2DbReference::kFullScaleDbmAtZeroGain
                       - Hl2DbReference::kDefaultLnaGainDb),
              "the step from the old relative form is the SAME at every gain");
    }

    // AND AT 0 dB GAIN THE CONSTANT IS THE WHOLE OFFSET, which is what makes it
    // checkable against a signal generator without any arithmetic.
    ref.setLnaGainDb(0.0);
    check(near(ref.toDbm(0.0), Hl2DbReference::kFullScaleDbmAtZeroGain),
          "full scale at 0 dB gain reads exactly the derived figure");

    ref.setLnaGainDb(Hl2DbReference::kDefaultLnaGainDb);

    // A fixed antenna signal. Raising the LNA by 20 dB raises the digitised
    // level by 20 dB -- and must NOT change the reported strength.
    ref.setFullScaleDbm(-60.0);
    ref.setLnaGainDb(0.0);
    const double reported = ref.toDbm(-13.0);          // -13 dBFS at 0 dB gain

    ref.setLnaGainDb(20.0);
    check(near(ref.toDbm(-13.0 + 20.0), reported),
          "a 20 dB gain increase does not move the reported dBm");

    ref.setLnaGainDb(-12.0);                            // the AD9866 floor, real
    check(near(ref.toDbm(-13.0 - 12.0), reported),
          "a 12 dB gain cut does not move the reported dBm");

    // The documented native range includes +48. This tests the commanded
    // gain arithmetic; actual board response needs independent measurement.
    ref.setLnaGainDb(48.0);
    check(near(ref.toDbm(-13.0 + 48.0), reported),
          "the reference removes whatever gain it is told about, exactly");

    // The spectrum path applies offsetDb() per frame rather than toDbm() per
    // bin; the two must agree or the trace and the S-meter would disagree.
    ref.setLnaGainDb(20.0);
    check(near(-13.0 + ref.offsetDb(), ref.toDbm(-13.0)),
          "offsetDb() and toDbm() agree (spectrum vs S-meter)");

    // ---------------------------------------------------------------
    // THE AGC CEILING, WHICH IS THE HALF THE OPERATOR HEARS
    //
    // Same invariant, different sense. WDSP's maximum gain is a setpoint about
    // the signal AT THE ANTENNA applied to a signal that has already been
    // through the LNA, so an RF gain change that leaves the ceiling alone
    // changes how far down into the noise the AGC chases. THE EXPECTED DELTA
    // IS ZERO HERE TOO -- not the step size.
    // ---------------------------------------------------------------
    Hl2DbReference agc;
    constexpr int kDefaultThresholdUnits = 65;   // Hl2Backend::Receiver's default

    // No change for an operator who never touches RF gain: at the reference
    // gain this is the plain 0.6-per-unit map, the same number the backend
    // derived before this term existed.
    check(near(agc.agcCeilingDb(kDefaultThresholdUnits), 39.0),
          "at the reference gain the default AGC-T is still 39 dB");
    check(near(agc.agcCeilingDb(0), 0.0), "AGC-T 0 is still 0 dB");
    check(near(agc.agcCeilingDb(100), 60.0), "AGC-T 100 is still 60 dB");

    // A gain change moves the ceiling by exactly minus the gain change, so a
    // constant antenna signal keeps a constant heard level.
    const double ceilingAtReference = agc.agcCeilingDb(kDefaultThresholdUnits);

    agc.setLnaGainDb(Hl2DbReference::kDefaultLnaGainDb + 6.0);
    check(near(agc.agcCeilingDb(kDefaultThresholdUnits), ceilingAtReference - 6.0),
          "a 6 dB LNA rise lowers the AGC ceiling by 6 dB");
    check(near(agc.agcCeilingDb(kDefaultThresholdUnits) + 6.0, ceilingAtReference),
          "the antenna-referred ceiling does not move on a 6 dB rise");

    agc.setLnaGainDb(Hl2DbReference::kDefaultLnaGainDb - 6.0);
    check(near(agc.agcCeilingDb(kDefaultThresholdUnits), ceilingAtReference + 6.0),
          "a 6 dB LNA cut raises the AGC ceiling by 6 dB");

    // Item 14's regulator steps the RxPGA 3-6 dB several times a day. Every
    // step in a run must leave the antenna-referred ceiling exactly where it
    // started, or the regulator walks the operator's AGC over an afternoon.
    for (const double step : {3.0, -6.0, 6.0, -3.0, 4.0}) {
        agc.setLnaGainDb(agc.lnaGainDb() + step);
        check(near(agc.agcCeilingDb(kDefaultThresholdUnits) + agc.lnaGainDb(),
                   ceilingAtReference + Hl2DbReference::kDefaultLnaGainDb),
              "a regulator step leaves the antenna-referred ceiling unmoved");
    }

    // The commanded limits, where referring saturates. A negative ceiling would
    // be the AGC attenuating a signal it was asked to amplify. 48 is
    // the maximum documented commanded gain; this tests the arithmetic clamp,
    // not the board's analog response.
    agc.setLnaGainDb(48.0);
    check(agc.agcCeilingDb(kDefaultThresholdUnits) >= 0.0,
          "the referred ceiling never goes negative at full LNA gain");
    check(near(agc.agcCeilingDb(0), 0.0),
          "AGC-T 0 at full LNA gain clamps at zero rather than going negative");

    // Referring UPWARD past the slider's nominal 60 dB top is correct, not an
    // overrun: an operator 12 dB down on the LNA needs 12 dB more AGC gain to
    // hear the same signal at the same level.
    agc.setLnaGainDb(-12.0);                            // the AD9866 floor, real
    check(near(agc.agcCeilingDb(100), 60.0 + 32.0),
          "a 32 dB LNA cut refers AGC-T 100 above the slider's nominal top");
    check(agc.agcCeilingDb(100) <= Hl2DbReference::kAgcCeilingDbMax,
          "the referred ceiling stays inside WDSP's own maximum");

    // The display term must NOT leak into the audio term. fullScaleDbm
    // calibrates a dBm axis; the AGC lives inside the digital chain and never
    // sees dBm, so calibrating the display must not move the heard level.
    agc.setLnaGainDb(Hl2DbReference::kDefaultLnaGainDb);
    const double beforeCalibration = agc.agcCeilingDb(kDefaultThresholdUnits);
    agc.setFullScaleDbm(-60.0);
    check(near(agc.agcCeilingDb(kDefaultThresholdUnits), beforeCalibration),
          "calibrating the display does not move the AGC ceiling");
    // ABSOLUTE, so at the default 20 dB of gain a -60 dBm full scale is -80,
    // not -60. The assertion above is the one carrying the meaning -- that
    // calibrating the display leaves the AGC ceiling untouched -- and it still
    // passes, which is the point: the two terms stay separate even though one
    // of them changed form.
    check(near(agc.offsetDb(), -60.0 - Hl2DbReference::kDefaultLnaGainDb),
          "...while it does move the display offset");

    // Compatibility, for every documented stored gain including values above
    // +19. THE GAIN RANGE AND THE DEFAULT ARE UNCHANGED BY THIS PR -- this is
    // the absolute anchor only, and kLnaGainMaxDb still publishes +48 exactly
    // as #5752 left it.
    check(Hl2DbReference::kDefaultLnaGainDb == 20.0,
          "fresh profiles retain the existing +20 dB reference");
    check(AetherSDR::hl2::kLnaGainMaxDb == 48,
          "the documented six-bit range is untouched -- the single-unit fold "
          "above code 31 is a SEPARATE question and #5752 deliberately kept "
          "this at 48");
    for (int stored = -12; stored <= 48; ++stored) {
        Hl2DbReference after;
        const auto seed = AetherSDR::hl2::connectLna(
            true, true, stored, false, 0, AetherSDR::hl2::kLnaDefaultGainDb,
            AetherSDR::hl2::kLnaGainMinDb, AetherSDR::hl2::kLnaGainMaxDb);
        after.setLnaGainDb(seed.liveDb);

        // THE DISPLAY MOVES, by the same constant step for every stored value.
        // Nobody is singled out and nobody is left behind.
        const double oldOffset = 20.0 - stored;
        check(near(after.offsetDb() - oldOffset,
                   Hl2DbReference::kFullScaleDbmAtZeroGain
                       - Hl2DbReference::kDefaultLnaGainDb),
              "every stored gain's display reference moves by the same step");
        check(near(after.offsetDb(),
                   Hl2DbReference::kFullScaleDbmAtZeroGain - stored),
              "and lands on fullScale - storedGain, absolutely");

        // THE AGC DOES NOT MOVE AT ALL. lnaOffsetDb() stays relative and
        // fullScaleDbm never enters the ceiling, so what the operator HEARS is
        // bit-for-bit what it was before this PR, for every stored gain.
        check(near(after.agcCeilingDb(65), 39.0 + oldOffset),
              "stored gain preserves the old AGC ceiling at 65");
    }

    if (g_failures == 0)
        std::fprintf(stderr, "hl2_dbref_test: all checks passed\n");
    return g_failures == 0 ? 0 : 1;
}
