#pragma once

#include <QString>

namespace AetherSDR {

// What a running binary knows about the tree it was built from. Official
// (CI / release) builds carry only `official == true`; the git fields are
// captured for local builds by cmake/BuildIdentity.cmake at BUILD time, so
// they name the tree actually compiled rather than the one configured.
struct BuildIdentity {
    bool official{true};
    QString label;      // -DAETHER_LOCAL_BUILD_LABEL, may be empty
    QString branch;     // `git rev-parse --abbrev-ref HEAD`; "HEAD" when detached
    QString fullSha;    // `git rev-parse HEAD`; empty when git is unavailable
    bool dirty{false};  // tracked files modified at build time
    QString buildTime;  // UTC, ISO 8601, when the identity last changed
    QString details;    // free text (e.g. an integration build's PR list)
};

// The at-a-glance label. Empty for an official build: callers show nothing,
// so an official build looks exactly as it did before this existed.
//   local, no label  -> "LOCAL <branch> @ <sha8>[+dirty]"
//   local, labelled  -> "<label> @ <sha8>[+dirty]"
QString composeBuildLabel(const BuildIdentity& id);

// Multi-line tooltip / accessible description with every field. Empty for
// an official build.
QString composeBuildDetails(const BuildIdentity& id);

// The identity compiled into this binary.
BuildIdentity currentBuildIdentity();

} // namespace AetherSDR
