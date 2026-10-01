#pragma once
#include "Pose.h"
#include <string>

namespace fc {
struct HkxSplineData {
    std::uint32_t tracks{},numFrames{},numBlocks{},maxFramesPerBlock{},maskAndQuantizationSize{};
    float blockDuration{},blockInverseDuration{},frameDuration{};
    std::vector<std::uint32_t> blockOffsets,floatBlockOffsets,transformOffsets,floatOffsets;
    std::vector<std::uint8_t> data;
};
bool decodeHkxSplineTransforms(const HkxSplineData& source,std::vector<Pose>& frames,std::string& error);
bool decodeHkxSpline(const HkxSplineData& source,std::vector<std::vector<Quat>>& rotations,std::string& error);
}
