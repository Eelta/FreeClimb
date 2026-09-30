#pragma once
#include <cstring>
#include <string_view>

namespace fc {

class AnimationCalls {
    using BoolSetter=bool(*)(RE::BShkbAnimationGraph*,const RE::BSFixedString&,bool);
    using FloatSetter=bool(*)(RE::BShkbAnimationGraph*,const RE::BSFixedString&,float);
    static inline BoolSetter boolean{};
    static inline FloatSetter floating{};
    static inline RE::BSFixedString synced,jumping,sprinting,sprintOK,direction,speed,sampled,damped,forceIdle;
    static bool checked(std::uint64_t id,std::uintptr_t rva,std::size_t offset,std::string_view signature) {
        const auto address=REL::ID(id).address();
        return address==REL::Module::get().base()+rva&&
            std::memcmp(reinterpret_cast<const void*>(address+offset),signature.data(),signature.size())==0;
    }
public:
    static bool initialize() {

        if(!checked(62710,0xAF7310,0x17,"\x41\x0f\xb6\xe8\x48\x8b\x89\x08\x02\x00\x00"sv)||
           !checked(62709,0xAF7250,0x17,"\x48\x8b\x89\x08\x02\x00\x00\x0f\x28\xf2"sv)) {
            SKSE::log::error("Animation setter ABI mismatch; FreeClimb hooks not installed");return false;
        }
        boolean=reinterpret_cast<BoolSetter>(REL::ID(62710).address());
        floating=reinterpret_cast<FloatSetter>(REL::ID(62709).address());
        synced="bIsSynced";jumping="bInJumpState";sprinting="IsSprinting";direction="Direction";speed="Speed";
        sampled="SpeedSampled";damped="SpeedDamped";forceIdle="bForceIdleStop";
        sprintOK="bSprintOK";
        SKSE::log::info("SE animation setters verified: bool=62710/AF7310, float=62709/AF7250");
        return true;
    }
    static bool locomotion(RE::BShkbAnimationGraph* graph,float requestedSpeed) {
        if(!boolean||!floating||!graph||!graph->behaviorGraph||!graph->projectDBData)return false;
        const bool syncOK=boolean(graph,synced,false);
        boolean(graph,jumping,false);boolean(graph,sprinting,requestedSpeed>0);
        if(requestedSpeed>0)boolean(graph,sprintOK,true);
        boolean(graph,forceIdle,false);
        floating(graph,direction,0);
        const bool speedOK=floating(graph,speed,requestedSpeed);

        floating(graph,sampled,requestedSpeed);floating(graph,damped,requestedSpeed);
        return syncOK&&speedOK;
    }
};
}
