#pragma once
#include "Core.h"
#include "UserSettings.h"

namespace fc {
inline bool deferredRuntimeSettings(const UserSettings& active,const UserSettings& requested) {
    return active.bindings!=requested.bindings||active.gamepad!=requested.gamepad||active.threepeatAnimations!=requested.threepeatAnimations;
}
inline UserSettings liveRuntimeSettings(const UserSettings& active,const UserSettings& requested,bool retainInputs) {
    auto settings=sanitizeUserSettings(requested);
    if(retainInputs) {
        settings.bindings=active.bindings;settings.gamepad=active.gamepad;
        settings.threepeatAnimations=active.threepeatAnimations;
    }
    return settings;
}
inline void applyLiveTraversalSettings(const UserSettings& u,Settings& c) {
    c.contextActions=u.contextActions;c.automaticClimbActions=u.automaticClimbActions;c.legacyAutomaticHops=false;
    c.surfaceActionVariants=u.surfaceActionVariants;c.wallRunEnabled=u.wallRunEnabled;c.wallRunObstacleJumps=u.wallRunObstacleJumps;
    c.contextualMantleEnabled=u.contextualMantleEnabled;c.automaticSideWeights=u.automaticSideWeights;
    c.climbSpeed=u.upSpeed;c.downSpeed=u.downSpeed;c.sideSpeed=u.sideSpeed;
    c.runSpeed=u.wallRunSpeed>0?u.wallRunSpeed:379.5f;c.diagonalRunMultiplier=u.diagonalRunMultiplier;
    c.autoActionMinSeconds=u.autoActionMinSeconds;c.autoActionMaxSeconds=u.autoActionMaxSeconds;
    c.fancyJumps=u.fancyJumps;c.hopOut=u.hopOut;c.kickOut=u.kickOut;
    c.reach=u.reach;c.groundJumpHeight=u.groundJumpHeight;c.maxNormalZ=u.maxNormalZ;
    c.staminaEnabled=u.staminaEnabled;c.drain=u.movingPerSecond;c.hangDrain=u.hangingPerSecond;c.startStamina=u.requiredToGrab;
}
}
