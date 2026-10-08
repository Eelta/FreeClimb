#pragma once
#include "InputBindings.h"
#include "GamepadInput.h"
#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace fc {
struct UserSettings {
    std::string language="english";
    bool enabled=true,notifications=true,lowStaminaNotifications=true,jumpToAttach=true,autoMantle=true;
    bool climbSneakEnabled=false;
    bool contextActions=true,threepeatAnimations=true,automaticClimbActions=true,legacyAutomaticHops=false;
    bool surfaceActionVariants=true,wallRunObstacleJumps=true,contextualMantleEnabled=true,diagnostics=false,fancyJumps=true;
    bool wallRunEnabled=true;
    bool audioEnabled=true,staminaEnabled=true;
    float upSpeed=100,downSpeed=78,sideSpeed=82,wallRunSpeed=379.5f,diagonalRunMultiplier=1.15f;
    float autoActionMinSeconds=.8f,autoActionMaxSeconds=1.25f,audioVolume=.75f;
    float hopOut=32,kickOut=52,reach=110,grabMaxSnap=60,groundJumpHeight=88,maxNormalZ=.7f;
    float movingPerSecond=10,hangingPerSecond=0,requiredToGrab=12;
    std::array<float,2> automaticSideWeights{1,1};
    InputBindings bindings;
    GamepadSettings gamepad;
};
enum class SettingsPage { general, movement, automatic, stamina, audio, keys, diagnostics };
struct SettingsLoadResult {
    UserSettings settings;
    bool found=false;
    std::vector<std::string> warnings;
};
UserSettings sanitizeUserSettings(UserSettings settings);
UserSettings restoreSettingsPage(SettingsPage page,const UserSettings& current);
SettingsLoadResult loadUserSettings(const std::filesystem::path& path);
bool saveUserSettings(const std::filesystem::path& path,const UserSettings& settings,std::string& error);
std::string userSettingsIni(const UserSettings& settings);
}
