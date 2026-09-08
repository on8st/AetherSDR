// Regression cover for #4580 — the tuning-range gate on the utility band
// buttons (WWV / GEN / 2200 / 630) must survive a band-panel rebuild.
//
// The gate lives in SpectrumOverlayMenu::applyTuningRangeToBandButtons(),
// which only ever looks at m_bandBtnFreqs. buildBandPanel() registers the
// utility buttons there; setXvtrBands() clears the vector and rebuilds the
// whole panel, so it has to re-register them. The two build sites are
// near-identical copies of the same ladder, and #4580 was filed because one
// copy had lost its m_bandBtnFreqs.append(). Nothing pinned that, so this
// test does: a rebuild must not hand the operator a button for a frequency
// the radio cannot tune.
//
// The radio modelled here is an HF-only transceiver, 1.8-30 MHz, with no
// "bands=" declaration. That range is deliberately discriminating:
//
//   2200 (0.1375) below range -> gated
//   630  (0.475)  below range -> gated
//   GEN  (0.500)  below range -> gated
//   WWV  (10.000) in range    -> live
//   160  (1.900)  in range    -> live
//   6    (50.150) above range -> gated  (rebuilt via makeBandBtn, the path
//                                        that never lost its registration)
//
// Leaving m_declaredBands empty matters: with a declared band set the utility
// row is presence-gated by declaredBandMenuIncludesUtility() and the buttons
// are omitted outright rather than greyed, which is a different mechanism and
// would not exercise m_bandBtnFreqs at all.

#include "gui/SpectrumOverlayMenu.h"

#include <QApplication>
#include <QPushButton>
#include <QWidget>

#include <cstdio>

using namespace AetherSDR;

namespace {

int g_failed = 0;
int g_total = 0;

void report(const char* name, bool ok)
{
    ++g_total;
    std::printf("%s %s\n", ok ? "[ OK ]" : "[FAIL]", name);
    if (!ok) {
        ++g_failed;
    }
}

// Panel rebuilds deleteLater() the old buttons, so drain the deferred deletes
// before matching on text or a stale twin can answer first.
QPushButton* bandButton(QWidget& parent, const QString& text)
{
    QCoreApplication::sendPostedEvents(nullptr, QEvent::DeferredDelete);
    const auto buttons = parent.findChildren<QPushButton*>();
    for (auto* btn : buttons) {
        if (btn->text() == text) {
            return btn;
        }
    }
    return nullptr;
}

bool gated(QWidget& parent, const QString& text)
{
    auto* btn = bandButton(parent, text);
    return btn && !btn->isEnabled();
}

bool live(QWidget& parent, const QString& text)
{
    auto* btn = bandButton(parent, text);
    return btn && btn->isEnabled();
}

} // namespace

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    QWidget parent;
    SpectrumOverlayMenu menu(&parent);

    report("utility buttons exist on the first build",
           bandButton(parent, "WWV") && bandButton(parent, "GEN")
               && bandButton(parent, "2200") && bandButton(parent, "630"));

    // No range reported yet (0/0 is the disconnected contract) -> nothing gated.
    report("an unreported range leaves every band button live",
           live(parent, "2200") && live(parent, "630") && live(parent, "GEN")
               && live(parent, "WWV") && live(parent, "6"));

    // --- first build: the gate applies -------------------------------------
    menu.setTuningRangeMhz(1.8, 30.0);
    app.processEvents();

    report("2200 m is gated on first build (0.1375 below 1.8 MHz)",
           gated(parent, "2200"));
    report("630 m is gated on first build (0.475 below 1.8 MHz)",
           gated(parent, "630"));
    report("GEN is gated on first build (0.500 below 1.8 MHz)",
           gated(parent, "GEN"));
    report("6 m is gated on first build (50.150 above 30 MHz)",
           gated(parent, "6"));
    report("WWV stays live on first build (10.000 in range)",
           live(parent, "WWV"));
    report("160 m stays live on first build (1.900 in range)",
           live(parent, "160"));

    auto* tooltipped = bandButton(parent, "2200");
    report("a gated button explains itself in a tooltip",
           tooltipped && !tooltipped->toolTip().isEmpty());

    // --- the rebuild -------------------------------------------------------
    // setXvtrBands() destroys and rebuilds the whole band panel and clears
    // m_bandBtnFreqs on the way in. It runs off RadioModel::infoChanged (the
    // refreshXvtr lambda in MainWindow), i.e. routinely, not rarely.
    const QVector<SpectrumOverlayMenu::XvtrBand> xvtrs{
        {"1296", 1296.100, "X1"},
    };
    menu.setXvtrBands(xvtrs);
    app.processEvents();

    report("the rebuild really happened (XVTR button present)",
           bandButton(parent, "1296") != nullptr);
    report("utility buttons survive the rebuild",
           bandButton(parent, "WWV") && bandButton(parent, "GEN")
               && bandButton(parent, "2200") && bandButton(parent, "630"));

    // The #4580 assertions. Each of these flips to [FAIL] if the utility
    // branch of setXvtrBands() stops doing m_bandBtnFreqs.append().
    report("2200 m stays gated after the rebuild",
           gated(parent, "2200"));
    report("630 m stays gated after the rebuild",
           gated(parent, "630"));
    report("GEN stays gated after the rebuild",
           gated(parent, "GEN"));
    report("6 m stays gated after the rebuild",
           gated(parent, "6"));
    report("WWV stays live after the rebuild",
           live(parent, "WWV"));
    report("160 m stays live after the rebuild",
           live(parent, "160"));

    // An XVTR band button is deliberately NOT range-gated: rfFreqMhz is by
    // definition outside the radio's native range, and gating it would kill
    // exactly the band the transverter exists to provide. Guard the carve-out
    // so a future "fix" for #4580 cannot quietly swallow it.
    report("XVTR band buttons are exempt from the range gate",
           live(parent, "1296"));

    // --- a second rebuild --------------------------------------------------
    // infoChanged fires repeatedly; the gate must not decay on the Nth pass.
    menu.setXvtrBands(xvtrs);
    app.processEvents();
    report("the gate survives a second rebuild",
           gated(parent, "2200") && gated(parent, "630") && gated(parent, "GEN")
               && live(parent, "WWV"));

    std::printf("%d/%d checks passed\n", g_total - g_failed, g_total);
    return g_failed == 0 ? 0 : 1;
}
