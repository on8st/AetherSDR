#include "core/BuildIdentity.h"

#include <QStringList>

namespace AetherSDR {

namespace {

constexpr int kShortShaLength = 8;

QString shortSha(const QString& fullSha)
{
    const QString trimmed = fullSha.trimmed();
    if (trimmed.isEmpty()) {
        return QStringLiteral("unknown");
    }
    return trimmed.left(kShortShaLength);
}

QString branchName(const QString& branch)
{
    const QString trimmed = branch.trimmed();
    if (trimmed.isEmpty()) {
        return QStringLiteral("unknown");
    }
    if (trimmed == QLatin1String("HEAD")) {
        return QStringLiteral("detached");
    }
    return trimmed;
}

} // namespace

QString composeBuildLabel(const BuildIdentity& id)
{
    if (id.official) {
        return {};
    }
    const QString dirty = id.dirty ? QStringLiteral("+dirty") : QString();
    const QString label = id.label.trimmed();
    if (label.isEmpty()) {
        return QStringLiteral("LOCAL %1 @ %2%3")
            .arg(branchName(id.branch), shortSha(id.fullSha), dirty);
    }
    return QStringLiteral("%1 @ %2%3").arg(label, shortSha(id.fullSha), dirty);
}

QString composeBuildDetails(const BuildIdentity& id)
{
    if (id.official) {
        return {};
    }
    const QString fullSha = id.fullSha.trimmed();
    QStringList lines;
    lines << QStringLiteral("Unofficial local build — not an AetherSDR release");
    if (!id.label.trimmed().isEmpty()) {
        lines << QStringLiteral("Label: %1").arg(id.label.trimmed());
    }
    lines << QStringLiteral("Branch: %1").arg(branchName(id.branch));
    lines << QStringLiteral("Commit: %1")
                 .arg(fullSha.isEmpty() ? QStringLiteral("unknown") : fullSha);
    lines << QStringLiteral("Working tree: %1")
                 .arg(id.dirty ? QStringLiteral("dirty (uncommitted changes)")
                               : QStringLiteral("clean"));
    if (!id.buildTime.trimmed().isEmpty()) {
        lines << QStringLiteral("Built: %1").arg(id.buildTime.trimmed());
    }
    const QString details = id.details.trimmed();
    if (!details.isEmpty()) {
        lines << QString() << details;
    }
    return lines.join(QLatin1Char('\n'));
}

} // namespace AetherSDR
