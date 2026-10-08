#pragma once
#include "ConverterInput.h"
#include "../../external/nlohmann/json.hpp"
#include "../../src/AnimationConfig.h"
#include <filesystem>

namespace fc {
struct ConverterRequest {
    std::filesystem::path input,pack,output,work;
    std::string slot;
    bool overwrite{};
};
struct ConverterFileSnapshot {std::filesystem::path path;std::vector<std::uint8_t> bytes;};
struct ConverterPreparedClip {
    ConverterInputClip clip;
    nlohmann::json config;
    std::vector<std::string> warnings;
    std::vector<ConverterFileSnapshot> snapshots;
    std::string direction;
    nlohmann::json contextGroup;
};
std::vector<std::uint8_t> writeCanonicalConverterHkx(const ConverterInputClip&,const Library&);
std::vector<std::uint8_t> writeConverterHkx(const ConverterInputClip&,const Library&,std::vector<std::string>&);
std::vector<std::uint8_t> bundleConverterHkx(const std::vector<std::pair<std::string,std::vector<std::uint8_t>>>&);
nlohmann::json convertHkxGroup(const ConverterRequest&,const std::vector<ConverterPreparedClip>&);
nlohmann::json convertHkx(const ConverterRequest&);
nlohmann::json convertHkx(const ConverterRequest&,const ConverterPreparedClip&);
struct ConverterWallRunPack {
    std::vector<std::uint8_t> hkx;
    nlohmann::json config;
    std::vector<std::string> warnings;
};
ConverterWallRunPack composeConverterWallRunPack(const Library&,const std::vector<ConverterPreparedClip>&);
ConverterWallRunPack composeConverterWallRunDirection(const Library&,std::string_view,const std::vector<ConverterPreparedClip>&);
ConverterWallRunPack composeConverterContextHopDirection(const Library&,std::string_view,const std::vector<ConverterPreparedClip>&);
}
