#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cmath>

namespace fc {

template<class T> T field(const void* base,std::size_t offset) {
    T value{};std::memcpy(&value,static_cast<const std::byte*>(base)+offset,sizeof(T));return value;
}
struct PoseTrack { std::byte* data{}; int count{}; };
inline PoseTrack densePose(void* tracks) {
    if(!tracks)return {};
    const auto bytes=field<std::int32_t>(tracks,0),count=field<std::int32_t>(tracks,4);
    if(count<3||count>128||bytes<16+count*16||bytes>1048576)return {};
    constexpr int header=16+2*16;
    const int capacity=field<std::int16_t>(tracks,header),size=field<std::int16_t>(tracks,header+2);
    const int offset=field<std::int16_t>(tracks,header+4),stride=field<std::int16_t>(tracks,header+6);
    const float weight=field<float>(tracks,header+8);
    if(capacity<1||capacity>1024||size<1||size>capacity||offset<16+count*16||stride!=48||
        offset+capacity*48>bytes||offset%16||!std::isfinite(weight)||weight<=0||
        field<std::uint8_t>(tracks,header+12)!=0||field<std::uint8_t>(tracks,header+13)!=1)return {};
    return {static_cast<std::byte*>(tracks)+offset,size};
}
}
