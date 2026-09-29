// The one TU that reads the generated identity header, so a change of HEAD
// or dirty state recompiles this file alone (plus the relink).
#include "core/BuildIdentity.h"

#include "aether/BuildIdentityGenerated.h"

namespace AetherSDR {

BuildIdentity currentBuildIdentity()
{
    BuildIdentity id;
    id.official = kAetherBuildOfficial;
    id.label = QString::fromUtf8(kAetherBuildLabel);
    id.branch = QString::fromUtf8(kAetherBuildBranch);
    id.fullSha = QString::fromUtf8(kAetherBuildSha);
    id.dirty = kAetherBuildDirty;
    id.buildTime = QString::fromUtf8(kAetherBuildTime);
    id.details = QString::fromUtf8(kAetherBuildDetails);
    return id;
}

} // namespace AetherSDR
