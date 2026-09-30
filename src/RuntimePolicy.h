#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace fc::runtime {
constexpr std::uint32_t pack(unsigned major,unsigned minor,unsigned patch,unsigned build=0) {
    return (major<<24)|(minor<<16)|(patch<<4)|build;
}
inline constexpr std::array supportedVersions{
    pack(1,5,97),pack(1,6,317),pack(1,6,318),pack(1,6,323),pack(1,6,342),
    pack(1,6,353),pack(1,6,629),pack(1,6,640),pack(1,6,659),pack(1,6,1130),
    pack(1,6,1170),pack(1,6,1179),pack(1,7,99)};
enum class Family {unsupported,se,ae,ae629,ae17};
constexpr bool supported(std::uint32_t version) {
    for(const auto candidate:supportedVersions)if(version==candidate)return true;
    return false;
}
constexpr Family family(std::uint32_t version) {
    if(!supported(version))return Family::unsupported;
    if(version==pack(1,5,97))return Family::se;
    if(version>=pack(1,7,99))return Family::ae17;
    return version>=pack(1,6,629)?Family::ae629:Family::ae;
}
inline constexpr std::size_t actorUpdateSlot=0xAD;
inline constexpr std::size_t inputFilterSlot=1;
inline constexpr std::size_t keyboardProcessSlot=2;
constexpr unsigned addressFormat(std::uint32_t version) {
    const auto kind=family(version);
    return kind==Family::se?1:kind==Family::ae17?5:kind==Family::unsupported?0:2;
}
}
