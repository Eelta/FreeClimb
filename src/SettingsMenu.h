#pragma once
#include "UserSettings.h"
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace fc {
struct MenuAnimationSlot {
    std::string name,file,reason;
    int status=0;
    std::uint64_t triggers=0;
    std::uint64_t observed=0;
    std::size_t samples=0;
    float seconds=0;
};
struct SettingsMenuSnapshot {
    bool ready=false,traversalActive=false,movementPending=false,reloadPending=false;
    bool audioReady=false,diagnosticsEnabled=false;
    std::string status,error,packName;
    std::uint64_t automaticAttempts=0,automaticActions=0,wallRunObstacleJumps=0;
    std::vector<MenuAnimationSlot> slots;
};
struct SettingsMenuCallbacks {
    std::function<UserSettings()> getSettings;
    std::function<SettingsMenuSnapshot()> snapshot;
    std::function<void(UserSettings)> requestSave;
    std::function<void(bool,float)> requestAudio;
    std::function<void()> requestReloadAnimations;
};
bool registerSettingsMenu(SettingsMenuCallbacks callbacks);
bool settingsMenuBlocking();
}
