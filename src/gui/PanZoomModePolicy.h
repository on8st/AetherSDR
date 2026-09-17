#pragma once

// WHEN BAND ZOOM AND SEGMENT ZOOM ARE REAL CONTROLS, in one place, because
// until now the answer lived only inside the code that DISABLES the buttons and
// the code that SENDS the command never asked it.
//
// `band_zoom=` / `segment_zoom=` engage a mode the RADIO owns: it computes the
// center and span from its own band plan and broadcasts the engaged flag back
// in pan status, which is what PanadapterModel decodes. A radio with no such
// mode never receives the string — RadioModel::sendCmd drops it — so on a
// Hermes-Lite 2, an ANAN, an RTL or an Icom these are dead controls.
//
// SpectrumWidget::setBandSegmentZoomAvailable already knew that and disabled
// the two right-click menu entries, computed at both of MainWindow's connect
// seams. Every OTHER entry point into the same send — the `band_zoom` /
// `segment_zoom` ShortcutManager actions, the MIDI bindings that fire them, the
// automation bridge's `shortcut` verb, the FlexControl and RC28 handlers, and
// the rotary's explicit-state form — reached
// MainWindow::togglePanZoomModeForPan and MainWindow::setPanZoomMode, which
// checked only `isConnected()`. The gate was per-widget, not per-action.
//
// (Lab finding FIND-52, which also measured what happens next: nothing visible.
// These two are not state-corrupting — togglePanZoomModeForPan reads the pan's
// RADIO-authoritative flag rather than a client-side bool, "so a failed send
// can't invert anything because nothing is latched client-side", #4057 — so the
// press is silently swallowed, which is precisely why it never got reported.)
//
// A CAPABILITY, NOT A FAMILY CHECK. The predicate this replaces was
// RadioModel::usesFlexCommandPlane(), a `family() == "flex"` branch above the
// seam — the shape docs/HERMES.md rules out, and the shape the neighbouring
// gate in the very same block already moved away from: hasDdcPanEdgeRolloff
// exists "rather than a family-string check at the one call site ... so a
// future DDC backend gets the same ... automatically instead of needing its own
// family added to a hardcoded list" (RadioCapabilities.h). Same call site, same
// argument. RadioCapabilities::hasRadioBandSegmentZoom is declared explicitly
// by all six backends and defaults false, so a seventh cannot silently claim a
// zoom mode it does not have.
//
// NOT hasCommandPlane(), either, and MainWindow's own comment says why: that is
// `m_wanConn || m_connection`, and SimBackend/demo mode owns a RadioConnection
// while understanding neither keyword. "Some connection object exists" was
// always one indirection looser than the question being asked.

#include "core/backends/RadioCapabilities.h"

namespace AetherSDR {

// `caps` is RadioModel::backendCapabilities(). Disconnected is FALSE rather
// than permissive: unlike the mode and tone controls, which stay live while
// disconnected so an operator can set up before connecting, these two do
// nothing but emit a command, and there is nowhere to emit it to. (It also has
// to be false, because backendCapabilities() on a disconnected model is a
// default-constructed descriptor, and reading a default as permission is the
// bug this field's `= false` default exists to prevent.)
[[nodiscard]] inline bool bandSegmentZoomAvailable(bool connected,
                                                   const RadioCapabilities& caps) noexcept
{
    return connected && caps.hasRadioBandSegmentZoom;
}

}  // namespace AetherSDR
