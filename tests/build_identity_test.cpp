// Socket-free: composition of the visible local-build label. No settings,
// devices, sockets, radio or TX.
#include "core/BuildIdentity.h"

#include <cstdio>

using namespace AetherSDR;

namespace {
int checks = 0;
int failures = 0;

void checkEq(const QString& actual, const QString& expected, const char* what)
{
    ++checks;
    if (actual != expected) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n  expected: \"%s\"\n  actual:   \"%s\"\n", what,
                     expected.toUtf8().constData(), actual.toUtf8().constData());
    }
}

void check(bool condition, const char* what)
{
    ++checks;
    if (!condition) {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", what);
    }
}

BuildIdentity local()
{
    BuildIdentity id;
    id.official = false;
    id.branch = QStringLiteral("local/build-label");
    id.fullSha = QStringLiteral("0123456789abcdef0123456789abcdef01234567");
    id.buildTime = QStringLiteral("2026-09-29T12:00:00Z");
    return id;
}
} // namespace

int main()
{
    // Official: nothing, whatever else is set — the UI must look as before.
    {
        BuildIdentity id = local();
        id.official = true;
        id.label = QStringLiteral("INTEGRATION 3 PRs: #1…#3");
        id.dirty = true;
        checkEq(composeBuildLabel(id), QString(), "official build has no label");
        checkEq(composeBuildDetails(id), QString(), "official build has no details");
    }
    // Default-constructed identity is official (fail towards "looks as today").
    checkEq(composeBuildLabel(BuildIdentity{}), QString(), "default identity is official");

    // Local, no label.
    checkEq(composeBuildLabel(local()), QStringLiteral("LOCAL local/build-label @ 01234567"),
            "local clean, no label");
    {
        BuildIdentity id = local();
        id.dirty = true;
        checkEq(composeBuildLabel(id), QStringLiteral("LOCAL local/build-label @ 01234567+dirty"),
                "local dirty, no label");
    }
    {
        BuildIdentity id = local();
        id.branch = QStringLiteral("HEAD");
        checkEq(composeBuildLabel(id), QStringLiteral("LOCAL detached @ 01234567"),
                "detached HEAD");
    }
    {
        BuildIdentity id = local();
        id.fullSha.clear();
        id.branch.clear();
        checkEq(composeBuildLabel(id), QStringLiteral("LOCAL unknown @ unknown"),
                "no git available");
    }
    {
        BuildIdentity id = local();
        id.label = QStringLiteral("   ");
        checkEq(composeBuildLabel(id), QStringLiteral("LOCAL local/build-label @ 01234567"),
                "whitespace label counts as no label");
    }

    // Local, labelled: the label replaces "LOCAL <branch>".
    {
        BuildIdentity id = local();
        id.label = QStringLiteral("INTEGRATION 19 PRs: #5894…#6015");
        checkEq(composeBuildLabel(id),
                QStringLiteral("INTEGRATION 19 PRs: #5894…#6015 @ 01234567"),
                "labelled clean");
        id.dirty = true;
        checkEq(composeBuildLabel(id),
                QStringLiteral("INTEGRATION 19 PRs: #5894…#6015 @ 01234567+dirty"),
                "labelled dirty still says dirty");
    }

    // Details carry every field, full SHA included.
    {
        BuildIdentity id = local();
        id.label = QStringLiteral("INTEGRATION 2 PRs: #1…#2");
        id.details = QStringLiteral("#1 aaaa\n#2 bbbb\n");
        const QString d = composeBuildDetails(id);
        check(d.contains(QStringLiteral("not an AetherSDR release")), "details say unofficial");
        check(d.contains(QStringLiteral("Label: INTEGRATION 2 PRs: #1…#2")), "details carry label");
        check(d.contains(QStringLiteral("Branch: local/build-label")), "details carry branch");
        check(d.contains(QStringLiteral("Commit: 0123456789abcdef0123456789abcdef01234567")),
              "details carry full sha");
        check(d.contains(QStringLiteral("Working tree: clean")), "details carry clean state");
        check(d.contains(QStringLiteral("Built: 2026-09-29T12:00:00Z")), "details carry build time");
        check(d.endsWith(QStringLiteral("#1 aaaa\n#2 bbbb")), "details carry trimmed free text");
    }

    std::printf("%d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
