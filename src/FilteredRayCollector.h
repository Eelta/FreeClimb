#pragma once
#include <RE/Skyrim.h>
#include <SKSE/SKSE.h>
#include "RuntimeSupport.h"
#include "RayCollectorValidationCache.h"
#include <cstddef>
#include <cstring>
#include <string_view>

namespace fc {

class alignas(16) FilteredRayCollector final:public RE::hkpClosestRayHitCollector {
public:
    using NativeAdd=void(*)(void*,const RE::hkpCdBody&,const RE::hkpShapeRayCastCollectorOutput&);
    using Keep=bool(*)(void*,const RE::hkpCollidable&,float);
    void* context{};
    Keep keep{};
    NativeAdd original{};
    bool malformed{};
    FilteredRayCollector(void* c,Keep k,NativeAdd add):context(c),keep(k),original(add) {
        rayHit={};Reset();pad0C=0;
    }
    void AddRayHit(const RE::hkpCdBody& body,const RE::hkpShapeRayCastCollectorOutput& hit) override {
        auto* root=&body;
        unsigned depth=0;
        while(root->parent&&depth++<64)root=root->parent;
        if(root->parent||!original||!keep){malformed=true;return;}

        if(keep(context,*static_cast<const RE::hkpCollidable*>(root),hit.hitFraction))
            original(this,body,hit);
    }
    ~FilteredRayCollector() override=default;
    RE::hkpClosestRayHitCollector* enginePrefix() {
        return this;
    }
};
static_assert(sizeof(RE::hkpRayHitCollector)==0x10);
static_assert(sizeof(RE::hkpClosestRayHitCollector)==0x70);
static_assert(offsetof(FilteredRayCollector,rayHit)==0x10);
static_assert(offsetof(FilteredRayCollector,context)==0x70);
static_assert(alignof(FilteredRayCollector)==16);
static_assert(offsetof(RE::hkpCdBody,parent)==0x18);
static_assert(offsetof(RE::hkpWorldRayCastOutput,rootCollidable)==0x50);
static_assert(offsetof(RE::bhkPickData,closestRayHitCollector)==0xA8);

inline void nativeClosestRayAdd(void* collector,const RE::hkpCdBody& body,
                                const RE::hkpShapeRayCastCollectorOutput& hit) {
    static_cast<RE::hkpClosestRayHitCollector*>(collector)->RE::hkpClosestRayHitCollector::AddRayHit(body,hit);
}

inline RayCollectorValidation<FilteredRayCollector::NativeAdd> verifiedRayCollectorAdd(RE::bhkWorld* world,std::uintptr_t table) {
    using Verified=RayCollectorValidation<FilteredRayCollector::NativeAdd>;
    static const auto verified=[]()->Verified {
        if(!runtime::supported())return {};
        const auto base=REL::Module::get().base();
        const auto add=REL::RelocationID(59653,60338).address();
        const auto table=RE::VTABLE_hkpClosestRayHitCollector[0].address();
        const auto worldTable=RE::VTABLE_bhkWorld[0].address();
        const bool tablesReadable=runtime::readable(table,sizeof(std::uintptr_t))&&
            runtime::readable(worldTable,0x34*sizeof(std::uintptr_t));
        if(!tablesReadable || !runtime::executable(add) ||
           *reinterpret_cast<const std::uintptr_t*>(table)!=add) {
            SKSE::log::warn("Ray collector mapped native Add/vtable validation failed: retaining unfiltered blocking rays");
            return {};
        }
        const auto pick=reinterpret_cast<const std::uintptr_t*>(worldTable)[0x33];
        if(!runtime::executable(pick)) {
            SKSE::log::warn("Ray collector native world Pick slot is not executable: retaining unfiltered blocking rays");
            return {};
        }
        auto exact=[](std::uintptr_t address,std::string_view bytes) {
            return runtime::readable(address,bytes.size())&&
                std::memcmp(reinterpret_cast<const void*>(address),bytes.data(),bytes.size())==0;
        };
        const bool valid=!runtime::isSE() || (add==base+0xA5BE20&&pick==base+0xDA7580&&table==base+0x15A89C8&&
            *reinterpret_cast<const std::uintptr_t*>(table)==add&&
            exact(add,{"\x48\x89\x5c\x24\x08\x48\x89\x74\x24\x10\x57\x48\x83\xec\x20\xf3\x0f\x10\x41\x20\x49\x8b\xf0\x41\x0f\x2f\x40\x10",28})&&
            exact(pick,{"\x48\x8b\xc4\x57\x41\x54\x41\x55\x41\x56\x41\x57\x48\x81\xec\xb0\x00\x00\x00",19})&&
            exact(base+0xDA7716,{"\x48\x8b\xb3\xa8\x00\x00\x00\x48\x85\xf6\x0f\x84\xf0\x00\x00\x00",16})&&
            exact(base+0xDA7799,{"\x4c\x8b\xc6\x48\x8b\xd3\x49\x8b\xce\xe8\x79\x39\xcd\xff",14})&&
            exact(base+0xDA77B7,{"\x0f\x28\x46\x10\x0f\x29\x43\x30",8})&&
            exact(base+0xDA780D,{"\x48\x8b\x46\x60\xe9\x12\x01\x00\x00",9})&&
            exact(base+0xDA7928,{"\x48\x89\x83\x80\x00\x00\x00",7}));
        if(!valid)SKSE::log::warn("Ray collector SE 1.5.97 ABI mismatch: retaining unfiltered blocking rays");
        else SKSE::log::info("Ray collector typed closest collector verified: runtime={} validation={} add={:X} pick={:X}; exact-player and non-solid ActorZone filtering enabled",
            REL::Module::get().version().string(),runtime::isSE()?"SE bytes and mapped vtables":"mapped vtables and executable sections",add-base,pick-base);
        return valid?Verified{nativeClosestRayAdd,pick}:Verified{};
    }();
    if(!verified.add||!world||!runtime::readable(reinterpret_cast<std::uintptr_t>(world),sizeof(std::uintptr_t)))return {};

    if(*reinterpret_cast<const std::uintptr_t*>(world)!=table)return {};
    if(!runtime::readable(table,0x34*sizeof(std::uintptr_t)))return {};
    return reinterpret_cast<const std::uintptr_t*>(table)[0x33]==verified.pick?verified:Verified{};
}
}
