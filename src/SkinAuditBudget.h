#pragma once
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

namespace fc {

class SkinAuditBudget {
    std::atomic_flag busy=ATOMIC_FLAG_INIT;
    bool initialized{};
    std::uint32_t currentFrame{},calls{},previousCalls=256,start{},cursor{},samples{};
public:
    static constexpr std::uint32_t samplesPerFrame=4;
    static constexpr std::uint32_t inputsPerSample=64;
    static constexpr std::uint32_t ancestorsPerSample=64;
    bool trySample(std::uint32_t frame) {
        if(busy.test_and_set(std::memory_order_acquire))return false;
        struct Release {std::atomic_flag& flag;~Release(){flag.clear(std::memory_order_release);}} release{busy};
        if(!initialized||frame!=currentFrame) {

            if(initialized&&std::uint32_t(frame-currentFrame)>0x7fffffffu)return false;
            if(initialized)previousCalls=std::max(calls,1u);
            initialized=true;currentFrame=frame;calls=samples=0;
            start=cursor%previousCalls;
            cursor=std::uint32_t((std::uint64_t(start)+samplesPerFrame)%previousCalls);
        }
        const auto ordinal=calls;if(calls<std::numeric_limits<std::uint32_t>::max())++calls;
        if(samples>=samplesPerFrame||ordinal<start||std::uint64_t(ordinal)>=std::uint64_t(start)+samplesPerFrame)return false;
        ++samples;return true;
    }
};
struct SkinAuditWork {
    std::uint32_t inputs{},ancestors{};
    bool exhausted{};
    bool input() {if(inputs>=SkinAuditBudget::inputsPerSample){exhausted=true;return false;}++inputs;return true;}
    bool ancestor() {if(ancestors>=SkinAuditBudget::ancestorsPerSample){exhausted=true;return false;}++ancestors;return true;}
};
enum class SkinInputKind { body,bridge,extra };
inline SkinInputKind skinInputKind(std::optional<std::size_t> slot) {
    return !slot?SkinInputKind::extra:*slot<99?SkinInputKind::body:SkinInputKind::bridge;
}
}
