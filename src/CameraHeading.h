#pragma once
#include "YawFrame.h"

namespace fc {
struct CameraHeading {
    float worldYaw{};
    static std::optional<CameraHeading> capture(float actorYaw,float relativeYaw) {
        if(!std::isfinite(actorYaw)||!std::isfinite(relativeYaw))return {};
        return CameraHeading{normalizeYaw(normalizeYaw(actorYaw)+normalizeYaw(relativeYaw))};
    }
    std::optional<float> relativeTo(float actorYaw) const {
        if(!std::isfinite(actorYaw)||!std::isfinite(worldYaw))return {};
        return yawDifference(worldYaw,normalizeYaw(actorYaw));
    }
};
}
