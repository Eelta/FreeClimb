#pragma once
#include "ConverterInput.h"
#include "../../external/nlohmann/json.hpp"

namespace fc {
struct ConverterTimelinePart {
    std::string name;
    ConverterInputClip clip;
};
struct ConverterTimelineRange {
    std::string name;
    std::array<std::size_t,2> frames{};
    Vec rootShift{};
};
struct ConverterTimeline {
    ConverterInputClip clip;
    std::vector<ConverterTimelineRange> ranges;
    std::vector<std::string> warnings;
};
ConverterTimeline composeConverterTimeline(const std::vector<ConverterTimelinePart>&,float transitionSeconds=.08f);
ConverterInputClip sliceConverterTimeline(const ConverterInputClip&,const nlohmann::json&);
}
