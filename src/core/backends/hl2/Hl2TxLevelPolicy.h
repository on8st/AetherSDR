#pragma once

// The two level calculations on the HL2's transmit path, as pure functions.
//
// Both were live bugs rather than refinements, and both are the kind that a
// running radio reports as "the control does nothing" — which is the hardest
// symptom to act on, because it is indistinguishable from the operator having
// misunderstood the control.
//
// They live in a header, evaluated by Hl2Backend rather than copied into it, so
// the suite exercises the SAME expressions the backend runs. A test against a
// re-typed copy of a mapping proves only that two copies agree; the convention
// error it is meant to catch would sit in both.
//
// See Hl2Backend::setMicGain and Hl2Backend::publishTelemetry for the reasoning
// about WHY each is shaped this way; this header is the arithmetic only.

#include <cmath>

namespace AetherSDR::hl2 {

// ---- Microphone gain -------------------------------------------------------

// The Phone applet's MIC slider (0..100) as dB of gain.
//
// 50 is unity, because TransmitModel constructs m_micLevel at 50 and nothing
// restores it at startup: a session where the operator never touches the slider
// must leave the modulator exactly at its own 1.0 default. +/-20 dB across the
// travel, linear in dB.
//
// Level 0 is NOT -20 dB — see micSliderToLinear, which handles it as a mute.
// This function is the continuous part of the mapping only.
[[nodiscard]] constexpr double micSliderToGainDb(int level) noexcept
{
    const int clamped = level < 0 ? 0 : (level > 100 ? 100 : level);
    return (static_cast<double>(clamped) - 50.0) * 0.4;
}

// The same slider as the linear multiplier the modulator takes.
//
// Level 0 mutes outright rather than resolving to the -20 dB the line above
// would give it. A slider at the bottom of its travel means off — and a mic
// merely 20 dB down would be hauled back up by the ALC's 40 dB of makeup
// anyway, so without the special case "0" would sound barely different from
// "50", which is the sort of control that teaches an operator to distrust every
// other one on the panel.
//
// SCOPE, because "mic" undersells it: this multiplier is applied to everything
// entering Hl2TxDsp::processAudioBlock, and on a host-modulating backend that
// includes digital-mode and WSPR-beacon audio arriving through submitTxAudio,
// not only voice. For MIC-path audio, above the ALC's hold threshold it is
// very nearly a no-op — the ALC normalizes each block's peak to alcTargetPeak
// and hands the gain straight back. For CLIENT-LEVELED audio (TCI/DAX) the ALC
// may only reduce, never lift (#4796), so below its target there is no handing
// back: this slider is a straight proportional attenuator on that path, and
// TX gain 5 (-18 dB) is a real -18 dB on the air. It stops being straight only
// where it has to — drive a full-scale client through the top of this slider's
// +20 dB and the ALC limits, rather than letting the modulator's hard clamp
// flat-top it, so the last stretch of travel buys reduced headroom rather than
// more power. At 0 neither path transmits: the mic path because silence
// sits below the hold threshold so the ALC declines to lift it, the
// client-leveled path as a plain 0.0x multiply. That is the honest reading of
// a slider at the bottom of its travel on a host modulator — there is one
// modulator and it is off — but it is worth knowing before parking the control
// at 0 between voice sessions.
[[nodiscard]] inline double micSliderToLinear(int level) noexcept
{
    if (level <= 0)
        return 0.0;
    return std::pow(10.0, micSliderToGainDb(level) / 20.0);
}

// ---- Transmit drive ---------------------------------------------------------

// The RF POWER slider (0..100) as the drive register byte.
//
// This is the mapping Hl2Backend::applyDrive already ran, lifted here UNCHANGED
// and pinned by the suite. It is deliberately not "improved": see the block
// below for why a different curve would be a regression rather than a fix.
//
// kTxDriveMax is 255, so this is `percent * 255 / 100` in integer arithmetic —
// linear in the byte, truncating.
[[nodiscard]] constexpr int driveByteForPercent(int percent, int driveMax) noexcept
{
    const int clamped = percent < 0 ? 0 : (percent > 100 ? 100 : percent);
    return clamped * driveMax / 100;
}

// The step the RADIO actually holds, which is what the operator hears.
//
// The gateware decodes only the drive field's TOP NIBBLE, so the byte above has
// 256 values and the radio has 16 states. Measured on hardware (docs/HERMES.md
// 17.7, 14.200 MHz USB, 1 kHz tone at -10 dBFS, dummy load, gateware v74):
// slider 44% and 50% both land on nibble 7 and read 1.984 W and 2.001 W — a
// 0.04 dB difference, which is measurement noise — while 50% to 51% crosses to
// nibble 8 and is +1.25 dB.
//
// The consequence worth naming, because it is the one an operator meets: the
// SIX lowest non-zero slider positions, 1% through 6%, all produce byte 2..15
// and therefore all land on step 0. They are one radio state, at the PA's
// minimum output, with the PA enabled — MetisClient::setTxDriveLevel ties the
// PA-enable bit to a non-zero byte, so only 0% is genuinely off.
[[nodiscard]] constexpr int driveStepForPercent(int percent, int driveMax) noexcept
{
    return driveByteForPercent(percent, driveMax) >> 4;
}

// WHY THIS CURVE IS NOT CHANGED, recorded here because the next reader will ask.
//
// Every HPSDR client that drives this radio maps the operator's setting
// LINEARLY onto the drive byte and lets the gateware discard the low nibble.
// None of them rounds to the nearest step:
//
//   * piHPSDR   `buffer[C1] = power & 0xFF;` — the drive level straight into
//               C1 of 0x12, no HL2 special case (src/old_protocol.c).
//   * Quisk     `tx_level = int(tx_level * reduc / 100.0)` then
//               `self.pc2hermes[4 * 9] = tx_level` — linear and TRUNCATING,
//               which is this function's arithmetic exactly
//               (hermes/quisk_hardware.py).
//   * Thetis    the mi0bot HL2 fork computes RadioVolume = slider * pctBand /
//               100 / 93.75, then wire_byte = RadioVolume * 1.02 * 255 —
//               linear, no nibble awareness (Console/clsHardwareSpecific.cs
//               767-795, Console/console.cs 49290-49299).
//
// Rounding to the nearest 16-count step, which is what Zeus's
// HermesLite2DriveProfile does, is the only counter-example found and it moves
// the step edges without removing a single one: the slider still has 101
// positions and the radio still has 16. docs/HERMES.md 17.7 reaches the same
// conclusion from the hardware side — "the quantisation is a hardware fact, not
// a defect ... rounding differently would only move the step edges."
//
// So this stays linear and truncating. What was missing was not a better curve
// but the ability to SAY which step a percentage lands on, which is what
// driveStepForPercent() above provides.

// ---- Forward-power peak hold -----------------------------------------------

// One step of the transmit forward-power peak hold, in watts.
//
// The HL2's forward power is a single 12-bit conversion from an I2C
// instrumentation ADC with no peak detector and no averaging in the gateware
// (rtl/slow_adc.v), reaching us at 10 Hz. Speech peaks last tens of
// milliseconds, so sampling that envelope at 10 Hz lands on a peak essentially
// never: an SSB reading sat 8-12 dB below PEP while constant-envelope FT8 —
// where every instant IS the peak — read full scale. Both were making the same
// power.
//
// Instant attack, exponential release. What this recovers is NOT an
// instantaneous PEP reading; no filter can recover a peak that was never
// sampled. What it does is accumulate the maximum ACROSS a transmission, so the
// displayed value climbs toward PEP as the over goes on and settles within a
// few dB of it.
//
// `keyed` is a real term, not a guard: unkeyed, the reading must follow the
// instantaneous sample straight down, or a hold outliving the transmission
// keeps re-arming MeterModel's filter and the gauge claims power out of a radio
// that has stopped.
[[nodiscard]] constexpr double fwdPeakHoldStep(double previousPeakW,
                                               double instantW,
                                               bool keyed,
                                               double releaseAlpha) noexcept
{
    if (!keyed)
        return instantW;
    if (instantW >= previousPeakW)
        return instantW;
    return previousPeakW + releaseAlpha * (instantW - previousPeakW);
}

}  // namespace AetherSDR::hl2
