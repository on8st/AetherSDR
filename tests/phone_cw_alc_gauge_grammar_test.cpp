// The Phone panel's ALC gauge must fill the way an operator reads it: EMPTY
// when the ALC is doing nothing, and a band that grows leftward from the 0 dB
// end as the stage pulls level away, its length equal to the reduction.
//
// WHY THIS TEST IS A PIXEL TEST, AND WHY NOTHING CHEAPER WORKS.
//
// The defect this pins (hl2-lab FINDINGS.md FIND-02) was a gauge that was
// correct about its SCALE and wrong about its GRAMMAR: the face ran -20..0 dB
// with 0 = unity = the resting value, and the widget was constructed without
// setReversed(true), so the bar read FULL when the chain was idle and emptied
// as the ALC worked. Backwards, and opposite to every other gauge on the panel.
//
// Nothing in the value-level API can see that. HGauge::filledFraction() is the
// value-normalised fraction and is IDENTICAL for both grammars — its own header
// says so: on a reversed gauge "the painted width there is 1.0f -
// filledFraction()". Asserting on it requires already knowing the reversal, and
// HGauge::publishAutomationState() does not publish m_reversed, so the
// automation bridge's dumpTree cannot see it either. value(), min(), max(),
// gaugeFraction and every tick label read correct while the bar paints the
// opposite of what it means.
//
// So the instrument has to be the pixels. This test grabs the widget and reads
// which end of the bar is coloured.
//
// TWO CONTROLS, because a pixel scan that quietly detects nothing would pass
// every assertion below by vacuum:
//   - Compression, the reference implementation. Identical one-sided shape
//     (-25..0, 0 = nothing happening) and already reversed. Its correct answer
//     is known in advance and it must scan the same way as ALC.
//   - Level, a NORMAL gauge. It must fill from the LEFT at the top of its
//     range, which proves the scan can see left-half fill at all — without it,
//     "no fill in the left half" is not evidence.

#include "TestSettingsProfile.h"
#include "gui/HGauge.h"
#include "gui/PhoneCwApplet.h"

#include <QApplication>
#include <QImage>
#include <QList>
#include <QPixmap>

#include <cstdio>

using namespace AetherSDR;

namespace {

int failures = 0;

void check(bool condition, const char* description)
{
    std::printf("%s %s\n", condition ? "[ OK ]" : "[FAIL]", description);
    if (!condition) {
        ++failures;
    }
}

// Every colour HGauge::paintEvent uses for a FILL, and no colour it uses for
// text. The reversed bar is 0xff4444; a normal gauge paints 0x1a9030 /
// 0x998800 / 0xcc3333 across its three zones; setFillFromRight() paints
// 0xcc3333. Tick labels also use 0xcc3333 and 0x998800 but are drawn at y=10,
// ABOVE the bar, and the scan below never looks there.
bool isFillPixel(QRgb px)
{
    switch (px & 0x00ffffffu) {
    case 0xff4444u: // reversed fill
    case 0x1a9030u: // normal, below yellow
    case 0x998800u: // normal, yellow zone
    case 0xcc3333u: // normal red zone, and fill-from-right
        return true;
    default:
        return false;
    }
}

struct FillScan {
    int total = 0;
    int left  = 0;
    int right = 0;
};

// Scan the bar INTERIOR only. paintEvent puts the bar at y = 12 with height
// (h - 14) and draws its 1px border at the edges, so rows 13 .. 12+barH-2 are
// interior. The centred white label overwrites a few fill pixels near the
// middle; that reduces both halves and cannot flip a verdict.
FillScan scanFill(const QImage& img)
{
    FillScan scan;
    const int barY = 12;
    const int barH = img.height() - barY - 2;
    const int half = img.width() / 2;
    for (int y = barY + 1; y < barY + barH - 1; ++y) {
        for (int x = 0; x < img.width(); ++x) {
            if (!isFillPixel(img.pixel(x, y)))
                continue;
            ++scan.total;
            if (x < half) ++scan.left; else ++scan.right;
        }
    }
    return scan;
}

FillScan scanAt(HGauge* gauge, float value)
{
    gauge->setValueImmediate(value);
    return scanFill(gauge->grab().toImage());
}

// HGauge has no Q_OBJECT (it is custom-painted and deliberately meta-object
// free — see publishAutomationState), so findChildren<HGauge*> will not
// compile and qobject_cast cannot check the type. Match on the accessible
// name, which on this panel belongs to a gauge and nothing else, then
// static_cast the way phone_cw_level_meter_state_test already does.
HGauge* gaugeByAccessibleName(PhoneCwApplet& applet, const char* name)
{
    const QList<QWidget*> widgets = applet.findChildren<QWidget*>();
    for (QWidget* w : widgets) {
        if (w->accessibleName() == QLatin1String(name))
            return static_cast<HGauge*>(w);
    }
    return nullptr;
}

} // namespace

int main(int argc, char** argv)
{
    qputenv("AETHER_AUTOMATION", "1");
    TestSettingsProfile profile(QStringLiteral("phone-cw-alc-gauge-grammar-test"));
    QApplication app(argc, argv);

    PhoneCwApplet applet;
    applet.resize(360, 640);

    auto* alcWidget = applet.findChild<QWidget*>(QStringLiteral("phoneAlcGainGauge"));
    check(alcWidget != nullptr, "the Phone panel exposes the ALC gauge");
    if (!alcWidget)
        return 1;
    auto* alc = static_cast<HGauge*>(alcWidget);

    // A zero-width grab would make every "no fill" assertion below pass for the
    // wrong reason, so the geometry is checked before anything is read from it.
    std::printf("       ALC gauge grabbed at %dx%d\n", alc->width(), alc->height());
    check(alc->width() >= 80 && alc->height() >= 20,
          "the ALC gauge has a real size to grab (a 0-wide grab would pass everything)");
    if (alc->width() < 80)
        return 1;

    // ── The grammar, stated as three readings ────────────────────────────
    //
    // 0 dB is unity: the ALC ceilings there, so this is where the needle sits
    // whenever nothing is happening. An operator must see NOTHING.
    const FillScan rest = scanAt(alc, 0.0f);
    std::printf("       ALC at 0 dB (rest): total=%d left=%d right=%d\n",
                rest.total, rest.left, rest.right);
    check(rest.total == 0,
          "at 0 dB the ALC gauge paints no fill at all — unity is not an event");

    // Half the face. The coloured band must span from the reading up to the
    // 0 dB end, so its pixels lie wholly in the RIGHT half and its length is
    // the reduction read against the scale.
    const FillScan half = scanAt(alc, -10.0f);
    std::printf("       ALC at -10 dB: total=%d left=%d right=%d\n",
                half.total, half.left, half.right);
    check(half.total > 0,
          "at -10 dB the ALC gauge paints a band");
    check(half.left == 0,
          "the -10 dB band lies wholly in the right half — it grows leftward "
          "FROM the 0 dB end, it does not start at the floor");

    // The presentation floor. Deeper reductions than this cannot be shown, so
    // the bar is full and stays full.
    const FillScan floorScan = scanAt(alc, -20.0f);
    std::printf("       ALC at -20 dB (floor): total=%d left=%d right=%d\n",
                floorScan.total, floorScan.left, floorScan.right);
    check(floorScan.total > half.total,
          "a deeper reduction paints MORE bar, not less");
    check(floorScan.left > 0 && floorScan.right > 0,
          "at the floor the band spans both halves");

    // ── Control 1: Compression, the reference implementation ─────────────
    // Same one-sided shape, same resting value at the right, already reversed.
    // If this scans differently from ALC, the two gauges disagree about a
    // grammar they share and one of them is wrong.
    HGauge* comp = gaugeByAccessibleName(applet, "Compression gauge");
    check(comp != nullptr, "the Phone panel exposes the Compression gauge");
    if (comp) {
        const FillScan compRest = scanAt(comp, 0.0f);
        const FillScan compHalf = scanAt(comp, -12.5f);
        std::printf("       Compression at 0: total=%d | at -12.5: total=%d left=%d\n",
                    compRest.total, compHalf.total, compHalf.left);
        check(compRest.total == 0,
              "CONTROL: Compression is also empty at 0 — the two share a grammar");
        check(compHalf.total > 0 && compHalf.left == 0,
              "CONTROL: Compression's band also lies wholly in the right half");
    }

    // ── Control 2: Level, a NORMAL gauge ─────────────────────────────────
    // Proves the scan can see left-half fill. Without this leg, every
    // "left == 0" above is satisfied just as well by a scan that detects
    // nothing anywhere.
    HGauge* level = gaugeByAccessibleName(applet, "Microphone level gauge");
    check(level != nullptr, "the Phone panel exposes the Level gauge");
    if (level) {
        const FillScan levelTop = scanAt(level, 10.0f);
        std::printf("       Level at +10 dB (top): total=%d left=%d right=%d\n",
                    levelTop.total, levelTop.left, levelTop.right);
        check(levelTop.left > 0,
              "CONTROL: a normal gauge at the top of its range DOES fill the "
              "left half — so the scan can see left-half fill");
    }

    if (failures == 0)
        std::printf("phone_cw_alc_gauge_grammar_test: all checks passed\n");
    else
        std::printf("phone_cw_alc_gauge_grammar_test: %d FAILED\n", failures);
    return failures;
}
