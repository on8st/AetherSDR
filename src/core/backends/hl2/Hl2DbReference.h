#pragma once

#include <algorithm>

#include "core/backends/hl2/Hl2BandMemoryPolicy.h"

namespace AetherSDR::hl2 {

// The single owner of everything that relates a raw dBFS number to a dBm one,
// and of the one audio-chain setpoint that has to move with it.
//
// WHY THIS IS ONE OBJECT
//
// Every LNA gain change shifts the absolute signal reference by exactly the
// same amount. If the panadapter displays dBm and the gain moves, the whole
// trace jumps and the waterfall paints a horizontal band that reads as a real
// on-air event. The fix is to apply an equal and opposite offset in the display
// chain -- and the only way to guarantee the two can never drift apart is to
// keep the gain value and its offset in the same object, which is what this is.
//
// This matters before there is any automatic RF AGC, because a manual gain
// change has the identical problem.
//
// WHAT IS AND IS NOT CALIBRATED
//
// The LNA term is exact AS FAR AS THE COMMANDED CODE IS THE APPLIED GAIN: it is
// the gain we ourselves commanded, so removing it is arithmetic, not
// estimation. Within that range a gain change provably cannot move a reported
// dBm value.
//
// A fold above code 31 was reported on one board in upstream issue #177.
// Its scope is unresolved: ad9866.v at 883a338 passes all six gain bits when
// the native-format flag is set, as AetherSDR sets it. Keep the documented
// range and existing reference until the measured behavior is reconciled with
// that command path; do not reinterpret stored gains from this observation.
// https://github.com/softerhardware/Hermes-Lite2/issues/177
//
// This object knows the commanded gain, not the analog response of a board.
// Any future hardware-specific correction needs a qualified mapping shared
// with the reported gain and separate validation of the display and AGC paths.
//
// THE ABSOLUTE TERM IS NOW DERIVED, AND THIS PARAGRAPH USED TO ARGUE THE
// OPPOSITE. It said fullScaleDbm "is NOT calibrated here ... so it defaults to
// 0.0 and the gain term is measured RELATIVE to a reference gain, not
// absolutely", and then explained why the relative form mattered. That was
// true of the tree that carried it and none of it is true now; left standing
// it would be the strongest argument against the change it sits inside.
//
// WHAT THE OLD FORM ACTUALLY DID, stated plainly because it is the defect this
// replaces. offsetDb() was fullScaleDbm + (referenceGain - liveGain) with
// fullScaleDbm 0.0 and both gains seeded from kDefaultLnaGainDb, so AT THE
// DEFAULT GAIN THE OFFSET WAS EXACTLY ZERO and toDbm() was the identity. The
// panadapter's dBm axis was the raw dBFS number with no conversion at all --
// not approximately, exactly -- and only a gain change away from the default
// moved it at all, and then only relative to that default.
//
// WHAT CHANGED IS THE OBJECTION, NOT THE STANDARD. The old reasoning turned on
// "neither number is calibrated, so an absolute form buys nothing and only
// moves the floor", and it was right, because the alternative on offer then
// was a per-unit calibration nobody had. That is not the alternative here.
// fullScaleDbm is DERIVED from the AD9866 datasheet and the HL2's own input
// network (see kFullScaleDbmAtZeroGain below), and a derived figure is not a
// per-unit one. Quisk and SparkSDR build per-unit tables for the analogous TX
// power question; this is a different KIND of number and the comparison does
// not carry.
//
// THE DISPLAYED FLOOR MOVES, by kFullScaleDbmAtZeroGain - kDefaultLnaGainDb at
// every gain, and that is the point rather than a side effect: it moves onto a
// figure that can be checked, from one that could not. What it must never do is
// move without saying so, which is why the step is asserted in hl2_dbref_test
// rather than left to arrive.
//
// DERIVED IS NOT CALIBRATED, and isCalibrated() keeps saying so. See the
// predicate below: it reports whether a MEASUREMENT was applied, not whether a
// number is present, and nothing in src/ applies one.
//
// THE THIRD TERM: THE AGC CEILING, WHICH THE OPERATOR HEARS
//
// The display half of this is the half you can see, and it was built first.
// The AGC-T half is the half you hear, and it has the same cause.
//
// The operator's AGC-T is a 0..100 slider that becomes a WDSP MAXIMUM GAIN in
// dB -- the most gain the AGC is allowed to apply, which is what decides how
// far down into the noise it will chase a weak signal. It is a setpoint about
// the signal AT THE ANTENNA, but WDSP applies it to a signal that has already
// been through the LNA. Raise the LNA 6 dB and the same antenna signal arrives
// at the AGC 6 dB hotter, so the same ceiling now lets the AGC amplify 6 dB
// further into the noise than the operator asked for -- the band floor comes
// up in the headphones and the operator turns the AGC-T down to compensate.
// Lower the LNA and weak signals fall out from under the ceiling instead.
//
// So the ceiling is referred to the reference the same way the display is:
// subtract the LNA term, and a gain change moves no reported number AND no
// heard level. This is the dependency item 14's RF-gain regulator needs --
// that regulator steps the RxPGA 3-6 dB several times a day, and the display
// half was already safe. This is the half that was not.
//
// fullScaleDbm deliberately does NOT enter the ceiling. It is the dBFS->dBm
// calibration of a DISPLAY axis; the AGC lives entirely inside the digital
// chain and never sees dBm. The two consumers share the LNA term and differ in
// the calibration term, which is an argument for keeping the terms in one
// object and deriving each consumer's combination here, rather than handing
// out a single "offset" and hoping both callers apply it correctly.
//
// WHY THIS IS NOT ONE OBJECT PER SLICE
//
// The backlog row that asked for this (docs/HERMES.md 13, item 12) says "one
// dB-reference object per slice". Built literally that would be wrong on this
// radio, and the reason is worth stating because the row will outlive it.
//
// Of the three terms, TWO ARE PROPERTIES OF THE RADIO, not of a slice:
//
//   * The LNA gain is one AD9866 field (0x0a[5:0]) in front of ALL four DDCs.
//     There is no per-receiver RF gain to hold. N copies of one number is
//     precisely the drift this class was created to make impossible.
//   * fullScaleDbm is a property of the board, the ADC reference and the front
//     end -- again one per radio, shared by every DDC.
//
// ONE IS GENUINELY PER SLICE: the AGC-T, an operator judgement about one
// receiver's audio, which two receivers on different bands may legitimately
// disagree about. It lives with the rest of that receiver's state, in
// Hl2Backend::Receiver::agcThresholdDb, and is passed to agcCeilingDb() at the
// point of use.
//
// So the reference is ONE object per radio, and the per-slice quantity is an
// ARGUMENT to it rather than a copy inside it. That gives the row what it was
// actually after -- a slice's AGC ceiling that moves with the reference -- with
// no second copy of a gain that physically cannot differ between slices.
//
// If the hardware ever changes (per-DDC front-end gain, or per-slice
// calibration), split it then, and the split will be forced by a real
// difference rather than by a sentence.
class Hl2DbReference {
public:
    // Matches Hl2Backend/MetisClient's default LNA setting.
    static constexpr double kDefaultLnaGainDb = kLnaDefaultGainDb;

    // Operator AGC-T units (0..100) -> WDSP maximum-gain ceiling in dB. 0.6
    // spans 0..60 dB, which puts the default of 65 at 39 dB -- measured clean
    // on live hardware where the previous 0..100 mapping had the DEFAULT
    // sitting 25 dB past the clipping point. See Hl2Backend::setSliceAgc for
    // that measurement. This is a WDSP-range fact, not an HL2 one, but it
    // belongs here because the ceiling it produces is referred to this object.
    static constexpr double kAgcCeilingDbPerUnit = 0.6;

    // WHAT 0 dBFS IS AT THE ANTENNA, with 0 dB of LNA gain: +0.97 dBm.
    //
    // DERIVED, NOT AVERAGED and NOT MEASURED, and that distinction is the whole
    // reason this is a constant rather than a per-unit calibration. Every step
    // is from the AD9866 datasheet and the HL2's own input network:
    //
    //   full scale at RxPGA = 0 dB, STATED by the datasheet  2.0 Vpp differential
    //   as RMS, 2.0 / (2 * sqrt 2)                           0.7071 Vrms
    //   into the 400 ohm differential input, V^2/R           1.25 mW
    //   in dBm, AT THE CONVERTER                             +0.97 dBm
    //   the 50->400 ohm input transformer, being MATCHED,
    //   conserves power and contributes                       0 dB
    //   -----------------------------------------------------------------
    //   full scale at the antenna, 0 dB LNA gain             +0.97 dBm
    //
    // BOTH INPUTS ARE READ OFF THE DATASHEET DIRECTLY, and neither is inferred.
    // Table 1's Rx path composite AC performance block is indexed by RxPGA
    // setting and names the full scale of each: "RxPGA Gain = 0 dB (Full-Scale
    // = 2.0 V p-p)", alongside 126 mVpp at 24 dB and 8.0 mVpp at 48 dB. The
    // same table gives "Differential Input Impedance ... 400 ohm || 4.0 pF".
    //
    // THE OTHER TWO ROWS ARE A FREE CROSS-CHECK on the 2.0 Vpp figure, because
    // referring them back through their own PGA settings must land on it:
    // 8.0 mVpp x 251.19 = 2.0095 Vpp, and 126 mVpp x 15.85 = 1.9970 Vpp. Three
    // stated rows, one number, 0.5% spread. An earlier draft of this derivation
    // reached 2.01 Vpp by referring the 48 dB row alone; the 0 dB row states it
    // outright and is what this cites.
    //
    // THE TRANSFORMER MOVES THE IMPEDANCE, NOT THE POWER. The Hermes-Lite 2's
    // 1:9 input transformer (BN-43-2402, 5:14) gives 50 x (14/5)^2 = 392 ohm,
    // which is the 400 the converter wants; a matched transformer conserves
    // power by construction, so it contributes 0 dB to this sum.
    //
    // THE INSERTION-LOSS TERM IS DELIBERATELY ABSENT, AND THAT IS THE ONE OPEN
    // QUESTION IN THIS CONSTANT. A ~2 dB lumped "transformer + N2ADR filter
    // board" loss was proposed, which would make this +3.0 dBm -- loss ahead of
    // the converter raises the antenna-referred full-scale point, P_ant =
    // P_adc + L, so the sign of such a term would be POSITIVE. It is left out
    // for two reasons, both of which a reviewer may overturn with evidence:
    //
    //   * IT DOUBLE-COUNTS THE TRANSFORMER. The same derivation states the
    //     matched transformer conserves power (0 dB) and then folds a
    //     "transformer + filter board" loss on top of it. Whatever the
    //     transformer's real dissipative loss is, it is one term, not two.
    //   * THE N2ADR FILTER BOARD IS AN OPTIONAL ACCESSORY, not part of the HL2
    //     signal path. This code does not know whether one is fitted -- the
    //     J16 one-hot writes go out unconditionally (MetisProtocol.h) precisely
    //     because there is no "do you have the filter board" setting, and
    //     hl2_live_band_filter_probe exists to ask the question on hardware.
    //     A class-wide constant cannot carry an accessory's loss; every HL2
    //     without one would then read uniformly wrong in the other direction.
    //
    // So this is the CONVERTER-REFERRED figure with a power-conserving match in
    // front of it. Real dissipative loss between antenna port and converter is
    // a positive addend to it, is station-dependent, and is exactly the kind of
    // thing the measurement below resolves rather than the kind of thing a
    // header should estimate.
    //
    // NO INDEPENDENT CONFIRMATION, and the one previously offered is WITHDRAWN.
    // DL1YCF's "-34 dBm clipping at +33 dB of gain" arithmetically gives
    // -1 dBm, and was quoted as agreeing "to the digit" with a figure since
    // shown to be several dB out -- the agreement was the tell, not the
    // evidence. It also assumes +33 dB was DELIVERED rather than commanded,
    // which the published figure cannot establish. It confirms nothing in
    // either direction and is recorded here only so nobody re-derives it.
    //
    // WHAT WOULD SETTLE IT is a bench measurement on this radio: a known level
    // into the antenna port at a known APPLIED gain, read against the ADC clip
    // counter. That is receive-only and wants a calibrated source. NO SUCH
    // MEASUREMENT HAS BEEN MADE, which is why isCalibrated() stays false.
    //
    // NOT THE openHPSDR FIGURE. piHPSDR and deskHPSDR carry +14 dB and their
    // own notes describe it as "average, varies per unit". That is a different
    // KIND of number, not a competing estimate of this one.
    //
    // WHAT IT DOES NOT COVER. The input transformer runs away above ~20 MHz --
    // IN3OTD measured return loss falling to -12.5 dB at 30 MHz -- so 10 m
    // carries a band-dependent residual on top of this. A per-band table could
    // take that later; it is not a reason to leave the reference at zero, which
    // is what "uncalibrated" actually meant here.
    static constexpr double kFullScaleDbmAtZeroGain = 0.97;

    // WDSP's own default maximum gain, used here only as the bound on what
    // referring the ceiling may produce. Referring can push the ceiling ABOVE
    // the slider's nominal 60 dB top -- an operator who cut the LNA 12 dB is
    // asking for 12 dB more AGC gain to hear the same signal at the same level,
    // and that is the correct answer, not an overrun. What it must never do is
    // run away, and it must never go negative: a ceiling below zero would be
    // the AGC attenuating a signal it was asked to amplify.
    static constexpr double kAgcCeilingDbMax = 120.0;

    // Gain we commanded on the AD9866 LNA, in dB.
    void setLnaGainDb(double db) noexcept { m_lnaGainDb = db; }
    double lnaGainDb() const noexcept { return m_lnaGainDb; }

    // APPLY A MEASURED full-scale figure, replacing the derived default.
    //
    // This setter is the ONLY thing that makes isCalibrated() true, and that is
    // its whole contract rather than a side effect of the value it happens to
    // write. Calling it says "somebody measured THIS radio against a reference
    // source"; it is not the way to nudge the derived figure. Calibrating is a
    // REPLACEMENT, not an addition -- the derived default is what the object
    // starts with and there is no trim term to combine with.
    void setFullScaleDbm(double dbm) noexcept
    {
        m_fullScaleDbm = dbm;
        m_fullScaleMeasured = true;
    }

    // What 0 dBFS means at the antenna with 0 dB of LNA gain, in dBm. Defaults
    // to kFullScaleDbmAtZeroGain; setFullScaleDbm replaces it with a measured
    // figure on the day one exists.
    double fullScaleDbm() const noexcept { return m_fullScaleDbm; }

    // CALIBRATED MEANS A MEASUREMENT WAS APPLIED. It does not mean "a number is
    // present", and that difference is the whole of this predicate.
    //
    // IT USED TO BE `m_fullScaleDbm != 0.0`, which worked only for as long as
    // the field started at zero. The derived default is not zero, so the same
    // test would answer TRUE on a radio nobody has ever measured -- and
    // Hl2Backend::capabilities() publishes this straight into
    // PanAmplitudeModel::calibratedDbm, whose documented meaning
    // (RadioCapabilities.h) is that a level from this radio MAY be compared
    // with another station's, published as a spot, or used as an absolute
    // threshold. A datasheet derivation does not earn that; a measurement
    // against a reference source does. The derived figure is a far better ZERO
    // POINT than 0.0 was, and it is still not a calibration.
    //
    // SNIFFING THE VALUE CANNOT WORK, which is why this holds a flag instead.
    // `m_fullScaleDbm != kFullScaleDbmAtZeroGain` is the smaller edit and
    // repeats the original bug one constant along: a genuine measurement that
    // landed on the derived figure would read UNCALIBRATED, exactly as a
    // genuine measurement of 0.0 dBm did before. Provenance is not recoverable
    // from a double, so it is carried rather than inferred.
    //
    // AND NOT AN isDerived()/isCalibrated() PAIR. The reference is derived
    // whenever it is not measured, so the second accessor would be the negation
    // of the first with nothing to call it. If a UI ever has to say "derived"
    // rather than "uncalibrated", it lands with that UI.
    //
    // NOTHING IN src/ CALLS setFullScaleDbm TODAY, so this is false in
    // production and the HL2 still declares an uncalibrated dBm axis. That is
    // the honest answer until the bench measurement named beside
    // kFullScaleDbmAtZeroGain is made.
    bool isCalibrated() const noexcept { return m_fullScaleMeasured; }

    // The gain the uncalibrated scale is referred to. At this gain the offset
    // is zero, so the reported number is raw dBFS. Defaults to the backend's
    // default LNA setting, which is what keeps the displayed floor where the
    // operator has always seen it.
    void setReferenceLnaGainDb(double db) noexcept { m_referenceLnaGainDb = db; }
    double referenceLnaGainDb() const noexcept { return m_referenceLnaGainDb; }

    // The whole point: subtracting the gain we applied is what keeps a signal
    // of constant strength reading the same dBm across a gain change.
    double toDbm(double dbfs) const noexcept
    {
        return dbfs + offsetDb();
    }

    // Offset form, for applying to a whole spectrum frame without a call per bin.
    // ABSOLUTE, not referred to a nominal gain: P(dBm) = dBFS + fullScale - Glna.
    //
    // The first version of this class subtracted the gain absolutely and was
    // REVERTED for a reason its own comment recorded: it "silently moved the
    // whole displayed noise floor by 20 dB ... Neither number is calibrated, so
    // that shift bought nothing." That objection was correct and it is what
    // kFullScaleDbmAtZeroGain removes -- the floor still moves, but it moves
    // onto a derived figure instead of from one arbitrary number to another.
    // The two halves cannot be separated: this form without the constant fails
    // on exactly the old grounds.
    //
    // NOT CONFIRMED INDEPENDENTLY, and isCalibrated() stays false until a bench
    // measurement says otherwise.
    double offsetDb() const noexcept
    {
        return m_fullScaleDbm - m_lnaGainDb;
    }

    // The LNA term alone, RELATIVE to the reference gain -- what has to be
    // undone, wherever it is undone.
    //
    // STILL RELATIVE, DELIBERATELY, AND ONLY THE AGC USES IT NOW. The display
    // path moved to an absolute form when fullScaleDbm became a real figure
    // (see offsetDb), but the AGC's invariant is a DIFFERENCE: a constant
    // antenna signal must stay at a constant heard level across a gain change,
    // and at the reference gain the operator must see exactly the 0.6-per-unit
    // map they saw before this term existed. An absolute form here would move
    // every operator's AGC-T the moment they connected, for no gain -- the AGC
    // never sees dBm.
    double lnaOffsetDb() const noexcept
    {
        return m_referenceLnaGainDb - m_lnaGainDb;
    }

    // The operator's AGC-T, referred to this reference. Same invariant as the
    // display: a constant antenna signal keeps a constant heard level across a
    // gain change, because the ceiling moves down by exactly what the LNA moved
    // up. At the reference gain this is the plain 0.6-per-unit map, so an
    // operator who never touches RF gain sees no change from before this term
    // existed.
    double agcCeilingDb(int thresholdUnits) const noexcept
    {
        const double base = static_cast<double>(thresholdUnits) * kAgcCeilingDbPerUnit;
        return std::clamp(base + lnaOffsetDb(), 0.0, kAgcCeilingDbMax);
    }

private:
    double m_lnaGainDb = kDefaultLnaGainDb;
    double m_referenceLnaGainDb = kDefaultLnaGainDb;
    double m_fullScaleDbm = kFullScaleDbmAtZeroGain;

    // DERIVED UNTIL SOMEBODY MEASURES IT. Only setFullScaleDbm sets this, and
    // nothing clears it: a radio does not become uncalibrated again.
    bool m_fullScaleMeasured = false;
};

}  // namespace AetherSDR::hl2
