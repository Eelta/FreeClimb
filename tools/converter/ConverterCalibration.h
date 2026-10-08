#pragma once
#include "ConverterInput.h"
#include "../../external/nlohmann/json.hpp"
#include <string_view>

namespace fc {
struct ConverterCalibration {
    nlohmann::json config;
    std::vector<std::string> warnings;
    float confidence{};
    std::vector<std::array<float,2>> phaseMap;
};
ConverterCalibration calibrateConverterClip(const Library&,std::string_view,const ConverterInputClip&,const nlohmann::json&);
}
