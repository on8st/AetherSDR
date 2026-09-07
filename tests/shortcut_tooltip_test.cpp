// Shortcut tooltip annotation (ShortcutManager::applyShortcutTooltips).
//
// ON8ST asked whether a keyboard shortcut existed for MOX. It did — Space is
// PTT (Hold), T is MOX Toggle — but nothing in the UI said so. The mechanism
// under test annotates an opted-in widget's tooltip with its bound key.
//
// Runs offscreen (QT_QPA_PLATFORM=offscreen, set by tests.cmake); QWidget needs
// a QApplication, so this cannot live in shortcut_manager_test, which is a
// QCoreApplication test.

#include "TestSettingsProfile.h"
#include "core/AppSettings.h"
#include "core/ShortcutManager.h"

#include <QApplication>
#include <QLabel>
#include <QPushButton>
#include <QWidget>

#include <iostream>

using namespace AetherSDR;

namespace {

bool expect(bool condition, const char* label)
{
    std::cout << (condition ? "[ OK ] " : "[FAIL] ") << label << '\n';
    return condition;
}

bool expectEq(const QString& actual, const QString& expected, const char* label)
{
    const bool ok = (actual == expected);
    std::cout << (ok ? "[ OK ] " : "[FAIL] ") << label << '\n';
    if (!ok) {
        std::cout << "        expected: \"" << expected.toStdString() << "\"\n"
                  << "        actual:   \"" << actual.toStdString() << "\"\n";
    }
    return ok;
}

// The action set under test mirrors the real TX category closely enough to be
// honest about it: mox_toggle ships bound to T, atu_start ships unbound.
void registerTxLikeActions(ShortcutManager& m)
{
    m.registerAction(QStringLiteral("mox_toggle"), QStringLiteral("MOX Toggle"),
                     QStringLiteral("TX"), QKeySequence(Qt::Key_T), {});
    m.registerAction(QStringLiteral("atu_start"), QStringLiteral("ATU Start"),
                     QStringLiteral("TX"), QKeySequence(), {});
    m.registerAction(QStringLiteral("tune_toggle"), QStringLiteral("TUNE Toggle"),
                     QStringLiteral("TX"), QKeySequence(), {});
    m.registerAction(QStringLiteral("modifier_action"), QStringLiteral("Modifier Action"),
                     QStringLiteral("TX"), QKeySequence(Qt::CTRL | Qt::Key_M), {});
}

} // namespace

int main(int argc, char** argv)
{
    TestSettingsProfile settingsProfile(QStringLiteral("aether-shortcut-tooltip-test"));
    if (!settingsProfile.isValid()) {
        std::cerr << "[FAIL] create temporary home\n";
        return 1;
    }
    QApplication app(argc, argv);
    AppSettings::instance().load();

    bool ok = true;

    // ── A bound action produces the suffix; an unbound one does not ──────────
    {
        ShortcutManager m;
        registerTxLikeActions(m);

        QWidget root;
        auto* mox = new QPushButton(QStringLiteral("MOX"), &root);
        mox->setToolTip(QStringLiteral("Toggle manual transmit"));
        mox->setProperty(ShortcutManager::kActionProperty, "mox_toggle");

        auto* atu = new QPushButton(QStringLiteral("ATU"), &root);
        atu->setToolTip(QStringLiteral("Start automatic antenna tuner"));
        atu->setProperty(ShortcutManager::kActionProperty, "atu_start");

        auto* untagged = new QPushButton(QStringLiteral("MEM"), &root);
        untagged->setToolTip(QStringLiteral("Toggle ATU memory recall"));

        m.applyShortcutTooltips(&root);

        ok &= expectEq(mox->toolTip(), QStringLiteral("Toggle manual transmit (T)"),
                       "bound action appends the key");
        ok &= expectEq(atu->toolTip(), QStringLiteral("Start automatic antenna tuner"),
                       "unbound action leaves the tooltip alone (no empty bracket)");
        ok &= expectEq(untagged->toolTip(), QStringLiteral("Toggle ATU memory recall"),
                       "widget without the property is untouched");
    }

    // ── Applying twice does not double the suffix ───────────────────────────
    {
        ShortcutManager m;
        registerTxLikeActions(m);

        QWidget root;
        auto* mox = new QPushButton(&root);
        mox->setToolTip(QStringLiteral("Toggle manual transmit"));
        mox->setProperty(ShortcutManager::kActionProperty, "mox_toggle");

        m.applyShortcutTooltips(&root);
        m.applyShortcutTooltips(&root);
        m.applyShortcutTooltips(&root);
        ok &= expectEq(mox->toolTip(), QStringLiteral("Toggle manual transmit (T)"),
                       "three applies produce one suffix");
    }

    // ── A rebind REPLACES the suffix rather than stacking one per rebind ────
    {
        ShortcutManager m;
        registerTxLikeActions(m);

        QWidget root;
        auto* mox = new QPushButton(&root);
        mox->setToolTip(QStringLiteral("Toggle manual transmit"));
        mox->setProperty(ShortcutManager::kActionProperty, "mox_toggle");
        m.applyShortcutTooltips(&root);

        m.setBinding(QStringLiteral("mox_toggle"), QKeySequence(Qt::Key_F5));
        m.applyShortcutTooltips(&root);
        ok &= expectEq(mox->toolTip(), QStringLiteral("Toggle manual transmit (F5)"),
                       "rebind replaces the suffix");

        // ...and clearing the binding must take the suffix back off entirely.
        m.clearBinding(QStringLiteral("mox_toggle"));
        m.applyShortcutTooltips(&root);
        ok &= expectEq(mox->toolTip(), QStringLiteral("Toggle manual transmit"),
                       "cleared binding removes the suffix");
    }

    // ── A widget with no tooltip gets the bare key ──────────────────────────
    {
        ShortcutManager m;
        registerTxLikeActions(m);

        QWidget root;
        auto* mox = new QPushButton(&root);
        mox->setProperty(ShortcutManager::kActionProperty, "mox_toggle");
        auto* atu = new QPushButton(&root);
        atu->setProperty(ShortcutManager::kActionProperty, "atu_start");

        m.applyShortcutTooltips(&root);
        ok &= expectEq(mox->toolTip(), QStringLiteral("T"),
                       "no tooltip + bound action gives the bare key");
        ok &= expect(atu->toolTip().isEmpty(),
                     "no tooltip + unbound action stays empty");
    }

    // ── A property naming no known action is left alone ─────────────────────
    {
        ShortcutManager m;
        registerTxLikeActions(m);

        QWidget root;
        auto* typo = new QPushButton(&root);
        typo->setToolTip(QStringLiteral("Some other control"));
        typo->setProperty(ShortcutManager::kActionProperty, "mox_togle");

        m.applyShortcutTooltips(&root);
        ok &= expectEq(typo->toolTip(), QStringLiteral("Some other control"),
                       "unknown action id leaves the tooltip untouched");
    }

    // ── The root widget itself is walked, not only its children ─────────────
    {
        ShortcutManager m;
        registerTxLikeActions(m);

        QWidget root;
        root.setToolTip(QStringLiteral("Root"));
        root.setProperty(ShortcutManager::kActionProperty, "mox_toggle");
        m.applyShortcutTooltips(&root);
        ok &= expectEq(root.toolTip(), QStringLiteral("Root (T)"),
                       "the root widget is annotated too");
    }

    // ── An out-of-band setToolTip() is re-annotated ─────────────────────────
    // TxApplet rewrites the TUNE and ATU tooltips on every availability change.
    // Without this the annotation would vanish the first time the radio
    // connects, which is exactly when the operator is looking at the button.
    {
        ShortcutManager m;
        registerTxLikeActions(m);
        m.setBinding(QStringLiteral("tune_toggle"), QKeySequence(Qt::Key_F2));

        QWidget root;
        auto* tune = new QPushButton(&root);
        tune->setToolTip(QStringLiteral("Start or stop tune carrier"));
        tune->setProperty(ShortcutManager::kActionProperty, "tune_toggle");
        m.applyShortcutTooltips(&root);
        ok &= expectEq(tune->toolTip(), QStringLiteral("Start or stop tune carrier (F2)"),
                       "tune button annotated");

        // The widget rewrites its own tooltip, knowing nothing about shortcuts.
        tune->setToolTip(QStringLiteral("Transmitter is not available"));
        ok &= expectEq(tune->toolTip(), QStringLiteral("Transmitter is not available (F2)"),
                       "out-of-band setToolTip is re-annotated, not clobbered");

        // ...and the new text became the new base, so it is not stacked either.
        m.applyShortcutTooltips(&root);
        ok &= expectEq(tune->toolTip(), QStringLiteral("Transmitter is not available (F2)"),
                       "re-apply after an out-of-band change does not stack");
    }

    // ── The master toggle: no annotation while shortcuts are switched off ───
    // KeyboardShortcutsEnabled defaults to "False" in AppSettings, so on a stock
    // profile shortcutGuard() refuses every handler. A tooltip promising "(T)"
    // there would be a lie, not a hint.
    {
        ShortcutManager m;
        registerTxLikeActions(m);

        QWidget root;
        auto* mox = new QPushButton(&root);
        mox->setToolTip(QStringLiteral("Toggle manual transmit"));
        mox->setProperty(ShortcutManager::kActionProperty, "mox_toggle");

        m.applyShortcutTooltips(&root, /*shortcutsEnabled=*/false);
        ok &= expectEq(mox->toolTip(), QStringLiteral("Toggle manual transmit"),
                       "master toggle off: no key is advertised");

        // Switching shortcuts on brings the annotation back...
        m.applyShortcutTooltips(&root, /*shortcutsEnabled=*/true);
        ok &= expectEq(mox->toolTip(), QStringLiteral("Toggle manual transmit (T)"),
                       "master toggle on: the key appears");

        // ...and switching them off again takes it away without eating the base.
        m.applyShortcutTooltips(&root, /*shortcutsEnabled=*/false);
        ok &= expectEq(mox->toolTip(), QStringLiteral("Toggle manual transmit"),
                       "master toggle off again: the base survives intact");

        // The live re-annotation path must honour the same latch: an out-of-band
        // tooltip written while shortcuts are off must not acquire a key.
        mox->setToolTip(QStringLiteral("Transmitter is not available"));
        ok &= expectEq(mox->toolTip(), QStringLiteral("Transmitter is not available"),
                       "ToolTipChange while off does not smuggle the key back in");
    }

    // ── The display string is NativeText, not PortableText ──────────────────
    // On macOS Ctrl+M renders as the command glyph; on Linux and Windows the
    // two forms coincide and the second assertion is vacuously true.
    {
        ShortcutManager m;
        registerTxLikeActions(m);

        QWidget root;
        auto* btn = new QPushButton(&root);
        btn->setToolTip(QStringLiteral("Base"));
        btn->setProperty(ShortcutManager::kActionProperty, "modifier_action");
        m.applyShortcutTooltips(&root);

        const QKeySequence seq(Qt::CTRL | Qt::Key_M);
        const QString native = seq.toString(QKeySequence::NativeText);
        const QString portable = seq.toString(QKeySequence::PortableText);
        ok &= expectEq(btn->toolTip(), QStringLiteral("Base (%1)").arg(native),
                       "modifier key rendered as NativeText");
        if (native != portable) {
            ok &= expect(!btn->toolTip().contains(portable),
                         "PortableText form is not what is shown");
        } else {
            std::cout << "[SKIP] NativeText == PortableText on this platform\n";
        }
    }

    return ok ? 0 : 1;
}
