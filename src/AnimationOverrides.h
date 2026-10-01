#pragma once
#include "Pose.h"
#include "MotionSlots.h"
#include <filesystem>

namespace fc {
struct AnimationOverrideLimits {
    std::size_t fileBytes=64*1024*1024,totalBytes=256*1024*1024,frames=1201,totalOutputBytes=128*1024*1024;
    unsigned fileMilliseconds=3000,totalMilliseconds=30000;
};
enum class OverrideStatus { missing,loaded,rejected };
struct AnimationOverrideResult {
    Motion motion{};
    std::string file,reason;
    OverrideStatus status=OverrideStatus::missing;
    std::size_t samples{};
};
struct AnimationOverrideReport {
    std::vector<AnimationOverrideResult> slots;
    std::size_t loaded{},rejected{},missing{},inputBytes{},outputBytes{};
};
AnimationOverrideReport loadHkxOverrides(Library& library,const std::filesystem::path& directory,
    AnimationOverrideLimits limits={});
}
