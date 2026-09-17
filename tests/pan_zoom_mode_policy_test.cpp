// Band/segment zoom availability, as one predicate both paths ask.
//
// `display pan set <pan> band_zoom=1` engages a mode the RADIO owns: it derives
// the center and span from its own band plan and broadcasts the engaged flag
// back in pan status, which is what PanadapterModel decodes. A radio with no
// such mode never receives the string — RadioModel::sendCmd drops it — so the
// two controls are dead there.
//
// The right-click menu entries already knew that: MainWindow disables them
// through SpectrumWidget::setBandSegmentZoomAvailable at both connect seams.
// MainWindow::togglePanZoomModeForPan and MainWindow::setPanZoomMode, which are
// where the command is actually emitted, checked only isConnected(). Every
// other entry point lands there: the `band_zoom`/`segment_zoom` ShortcutManager
// actions, the MIDI bindings that fire them, the automation bridge's `shortcut`
// verb, FlexControl, RC28, and the rotary's explicit-state form. Lab finding
// FIND-52 measured the result — the command dropping three of three presses, on
// two binaries, with nothing shown to the operator.
//
// WHAT THIS TEST PINS. That the predicate refuses on every production backend
// that does not DECLARE a radio-side zoom mode, read from each backend's own
// capabilities() rather than from a re-typed list. Deliberately NOT from
// caps.family: docs/HERMES.md §19.10 says a capability test that asserted the
// family "would pass just as happily against the anti-pattern it exists to
// prevent", and the family branch is precisely the anti-pattern this change
// removes from the gate.
//
// WHAT THIS TEST DOES NOT OBSERVE, and it is the more important half:
//
//   1. THAT MainWindow's FOUR CALL SITES USE IT. Two availability sites and two
//      send sites call bandSegmentZoomAvailable(); that coupling is four lines
//      a reader can see and this file cannot reach, because no test in this
//      tree constructs a MainWindow. There is no socket-free route to the
//      shortcut handler itself — the honest statement is that the SHAPE and the
//      six DECLARATIONS are pinned here, and the WIRING is not.
//   2. THAT A FLEX'S ZOOM ACTUALLY WORKS. FlexBackend declaring true is a claim
//      about the radio, checked against nothing here; what the true case really
//      guards is that the predicate is not vacuously false for everyone, which
//      would pass every other assertion in this file.
//   3. ANY RADIO BEHAVIOUR. Nothing is connected, nothing transmits, and
//      nothing here was measured; FIND-52's three-of-three is a lab
//      observation quoted above, not something this file reproduces.

#include "TestSettingsProfile.h"
#include "gui/PanZoomModePolicy.h"

#include "core/backends/anan/AnanBackend.h"
#include "core/backends/flex/FlexBackend.h"
#include "core/backends/hl2/Hl2Backend.h"
#include "core/backends/icom/IcomCivBackend.h"
#include "core/backends/sim/SimBackend.h"
#ifdef AETHER_BACKEND_RTL
#include "core/backends/rtl/RtlSdrBackend.h"
#endif

#include <QCoreApplication>

#include <cstdio>

using namespace AetherSDR;

namespace {
int failures = 0;
void check(bool condition, const char* label)
{
    std::printf("%s %s\n", condition ? "[ OK ]" : "[FAIL]", label);
    if (!condition) {
        ++failures;
    }
}
}  // namespace

int main(int argc, char** argv)
{
    // Hl2Backend touches AppSettings on construction; redirect before
    // QCoreApplication so no backend constructed here can read or write the
    // operator's live configuration.
    TestSettingsProfile settingsProfile(QStringLiteral("pan-zoom-mode-policy-test"));
    if (!settingsProfile.isValid()) {
        std::fprintf(stderr, "FAIL: could not create an isolated settings profile\n");
        return 1;
    }
    QCoreApplication app(argc, argv);

    // ---- the default, which is what a seventh backend would inherit ----
    //
    // FALSE, so silence declares ABSENCE. Had this defaulted true the way
    // hasFmRepeaterOffset does, adding a backend would have handed it a zoom
    // mode nobody had read it on. Pinned separately so it fails on its own.
    check(!RadioCapabilities{}.hasRadioBandSegmentZoom,
          "a descriptor nobody has written declares NO radio band/segment zoom");
    check(!bandSegmentZoomAvailable(true, RadioCapabilities{}),
          "and a default descriptor is refused even while 'connected' — which is "
          "the state backendCapabilities() reports before a radio is attached");

    // Constructors only — no connect, no discovery, no DSP, no stream.
    FlexBackend flex;
    const RadioCapabilities flexCaps = flex.capabilities();

    check(bandSegmentZoomAvailable(true, flexCaps),
          "a connected Flex is the one radio these two controls work on");
    // Disconnected is refused rather than left permissive. Unlike the mode and
    // tone controls, which stay live before a connect so an operator can set up
    // first, these do nothing but emit a command — there is nowhere to send it.
    check(!bandSegmentZoomAvailable(false, flexCaps),
          "and not while disconnected: the command has nowhere to go");

    // ---- every other production backend, from its own declaration ----
    {
        hl2::Hl2Backend hl2;
        check(!bandSegmentZoomAvailable(true, hl2.capabilities()),
              "Hermes-Lite 2: dead controls, and now refused where they are sent");
    }
    {
        anan::AnanBackend anan;
        check(!bandSegmentZoomAvailable(true, anan.capabilities()),
              "ANAN: same raw-IQ seam, same absent display command plane");
    }
    {
        icom::IcomCivBackend icom;
        check(!bandSegmentZoomAvailable(true, icom.capabilities()),
              "Icom CI-V's scope has no band/segment zoom verb");
    }
    {
        // The case that makes hasCommandPlane() the WRONG predicate, and the
        // reason this is a capability rather than "is a connection object
        // present": demo mode owns a RadioConnection and understands neither
        // keyword. MainWindow's own comment at the button gate says as much.
        SimBackend sim;
        check(!bandSegmentZoomAvailable(true, sim.capabilities()),
              "demo mode owns a connection and still has no radio-side zoom");
    }
#ifdef AETHER_BACKEND_RTL
    {
        rtl::RtlSdrBackend rtl;
        check(!bandSegmentZoomAvailable(true, rtl.capabilities()),
              "RTL-SDR: a dongle with no control protocol of its own");
    }
#else
    // Stated rather than silently skipped: this build has no librtlsdr, so the
    // RTL declaration is compiled out here and is pinned only where that
    // backend builds. See tests.cmake's note on AETHER_BACKEND_RTL.
    std::printf("[SKIP] RTL-SDR backend not built in this configuration\n");
#endif

    std::printf("%s: %d failure(s)\n", argv[0], failures);
    return failures == 0 ? 0 : 1;
}
