#pragma once
#include "Pose.h"
#include <span>

namespace fc {
struct HkxClip {
    float duration{};
    bool identityMapping{};
    std::string skeletonName;
    std::vector<int> boneIndices;
    std::vector<std::string> trackNames;
    std::vector<std::vector<Quat>> rotations;
    std::vector<Pose> frames;
};
bool decodeHkxAnimation(std::span<const std::uint8_t> bytes,HkxClip& clip,std::string& error);
bool decodeHkxAnimation(std::span<const std::uint8_t> bytes,HkxClip& clip,std::string& error,std::string_view member);
bool decodeHkxAnimationMembers(std::span<const std::uint8_t> bytes,std::vector<std::pair<std::string,HkxClip>>& clips,
    std::string& error,std::size_t maximumDecodedBytes=200*1024*1024);
}
