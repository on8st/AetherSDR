#pragma once

#include <QObject>
#include <QEvent>
#include <QByteArray>
#include <QKeySequence>
#include <QShortcut>
#include <QString>
#include <QStringList>
#include <QVector>
#include <functional>

namespace AetherSDR {

struct ShortcutImportResult {
    int importedCount{0};
    QStringList unknownActions;
    // Local (non-imported) actions whose binding this import displaced by a
    // colliding customized incoming key — the caller should surface these so
    // the user notices customizations they didn't intend to overwrite.
    QStringList displacedActions;
    QStringList errors;

    bool ok() const { return errors.isEmpty(); }
};

struct ShortcutExportResult {
    int exportedCount{0};
    QString error;

    bool ok() const { return error.isEmpty(); }
};

class ShortcutManager : public QObject {
    Q_OBJECT
public:
    struct Action {
        QString id;
        QString displayName;
        QString category;
        QKeySequence defaultKey;
        QKeySequence currentKey;
        std::function<void()> handler;
        bool autoRepeat{false};   // allow key-hold repeat (e.g. tuning)
        bool persisted{false};    // explicit user intent (set/cleared) → written to settings
        bool keysTx{false};       // keys the transmitter — declared at the
                                  // registration site (like markTxKeying for
                                  // widgets) so TX gates read one source of
                                  // truth, not a hand-maintained id list that
                                  // drifts (#4057 review: atu_start was missed).
    };

    explicit ShortcutManager(QObject* parent = nullptr);

    // Register an action with its default key binding and handler. Pass
    // keysTx=true for any action that keys the transmitter (MOX, TUNE, ATU,
    // two-tone, PTT, CW keying) — automation gates honor the flag.
    void registerAction(const QString& id, const QString& displayName,
                        const QString& category, const QKeySequence& defaultKey,
                        std::function<void()> handler,
                        bool autoRepeat = false,
                        bool keysTx = false);

    // Binding management
    void setBinding(const QString& actionId, const QKeySequence& key);
    void clearBinding(const QString& actionId);
    void resetToDefaults();

    // Persistence
    void loadBindings();
    void saveBindings();

    // Portable backup format. Rows identify actions by stable id and also carry
    // their human-readable names so imports can fall back across an id rename.
    // QKeySequence::PortableText keeps modifier names cross-platform.
    QByteArray exportBindingsCsv() const;
    ShortcutImportResult importBindingsCsv(const QByteArray& bytes);

    // File I/O for the portable backup — atomic write via QSaveFile, size-gated
    // and error-string-surfacing read. Kept on the manager itself so the gui
    // never learns about QFile/QSaveFile (matches ThemeManager's precedent).
    ShortcutExportResult exportToFile(const QString& path) const;
    ShortcutImportResult importFromFile(const QString& path);

    // Create/destroy QShortcuts on the target widget.
    // guardFn is called before each handler — return false to suppress.
    void rebuildShortcuts(QWidget* parent,
                          std::function<bool()> guardFn = nullptr);

    // Enable or disable all active QShortcut objects. Used to yield key
    // events to focused child widgets (e.g. sliders) that would otherwise
    // have their arrow keys stolen by window-level shortcuts.
    void setShortcutsEnabled(bool enabled);

    // ── Shortcut tooltips ────────────────────────────────────────────────
    // TELL THE OPERATOR WHICH KEY WORKS A BUTTON, ON THE BUTTON.
    //
    // The shortcuts have always been there; nothing pointed at them. A widget
    // opts in by carrying a dynamic property naming its action:
    //
    //     btn->setProperty(ShortcutManager::kActionProperty, "mox_toggle");
    //
    // and applyShortcutTooltips() walks the tree appending " (T)" to its
    // tooltip. The property idiom rather than a list of button pointers here,
    // for the same reason markTxKeying uses one: the declaration lives WITH the
    // widget, and a hand-maintained list in this header would drift the way
    // #4057's TX id list did (atu_start was missed).
    //
    // Re-runnable, and it stays correct in three situations that all occur:
    //   * Re-applied after a rebind — the un-annotated text is stashed in
    //     kBaseTooltipProperty, so the suffix is REPLACED, not accumulated.
    //   * The widget rewrites its own tooltip later (TxApplet rewrites TUNE and
    //     ATU on every availability change). applyShortcutTooltips installs
    //     this manager as an event filter on each opted-in widget and
    //     re-annotates on QEvent::ToolTipChange, so the annotation survives
    //     without every such call site having to know about shortcuts.
    //   * An action bound to no key is left entirely alone. An empty bracket
    //     would advertise a shortcut that does not exist, and most TX actions
    //     ship unbound.
    //
    // shortcutsEnabled is the View-menu master toggle (KeyboardShortcutsEnabled,
    // which defaults to False), NOT setShortcutsEnabled's transient slider
    // lease. Pass it through: with the master toggle off, no key does anything,
    // and a tooltip promising one would be a lie. Passing false strips the
    // annotations back to the widgets' own text; call it again from the toggle
    // so they come back when the operator switches shortcuts on.
    //
    // A widget with no tooltip of its own gets the key alone — still more than
    // nothing. A property naming an unknown action is left untouched, so a typo
    // cannot silently eat a tooltip.
    //
    // Display uses QKeySequence::NativeText: a Mac shows the glyph forms, Linux
    // and Windows show "Ctrl".
    static constexpr const char* kActionProperty = "shortcutAction";
    static constexpr const char* kBaseTooltipProperty = "shortcutBaseToolTip";
    static constexpr const char* kAppliedTooltipProperty = "shortcutAppliedToolTip";
    static constexpr const char* kTooltipFilterProperty = "shortcutToolTipFiltered";
    void applyShortcutTooltips(QWidget* root, bool shortcutsEnabled = true);

    // Query
    const QVector<Action>& actions() const { return m_actions; }
    Action* action(const QString& id);
    const Action* actionForKey(const QKeySequence& key) const;
    QString conflictCheck(const QKeySequence& key,
                          const QString& excludeId = {}) const;

    // Categories (ordered for legend display)
    static QStringList categories();

signals:
    void bindingsChanged();

protected:
    // Watches opted-in widgets for QEvent::ToolTipChange so an out-of-band
    // setToolTip() is re-annotated rather than silently dropping the key.
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    void normalizeDuplicateBindings();
    void annotateShortcutTooltip(QWidget* w) const;

    QVector<Action> m_actions;
    QVector<QShortcut*> m_shortcuts;
    // Latched by applyShortcutTooltips so the ToolTipChange re-annotation,
    // which takes no arguments, honours the same master toggle.
    bool m_shortcutTooltipsEnabled{true};
};

} // namespace AetherSDR
