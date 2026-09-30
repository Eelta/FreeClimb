#pragma once
#include "AnimationOverrides.h"

namespace fc {
struct AnimationPackSlotResult {
    Motion motion{};
    std::string file,reason;
    OverrideStatus status=OverrideStatus::missing;
    std::size_t samples{};
    float seconds{};
};
struct AnimationPackReport {
    bool committed{};
    std::string error;
    std::vector<AnimationPackSlotResult> slots;
    std::size_t loaded{},rejected{},missing{},inputBytes{},outputBytes{};
};
AnimationPackReport loadAnimationPack(Library& library,const std::filesystem::path& manifest,
    AnimationOverrideLimits limits={});
}
