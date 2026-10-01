#pragma once
#include <cstdint>

namespace fc {
template<class NativeAdd> struct RayCollectorValidation {NativeAdd add{};std::uintptr_t pick{};};
template<class WorldHandle,class NativeAdd> class RayCollectorValidationCache {
    WorldHandle world;
    std::uintptr_t table{},pick{};
    NativeAdd add{};
    bool checked{};
    void invalidate(){table=pick=0;add={};checked=false;}
public:
    using World=typename WorldHandle::element_type;
    RayCollectorValidationCache()=default;
    RayCollectorValidationCache(const RayCollectorValidationCache&)=delete;
    RayCollectorValidationCache& operator=(const RayCollectorValidationCache&)=delete;
    void bind(World* value) {
        if(world.get()==value)return;
        invalidate();world=WorldHandle(value);
    }
    void reset(){invalidate();world.reset();}
    template<class ReadPick,class Verify> NativeAdd resolve(std::uintptr_t currentTable,ReadPick readPick,Verify verify) {
        if(!world)return {};
        if(checked&&table==currentTable&&(!add||readPick(table)==pick))return add;
        invalidate();table=currentTable;
        const auto validated=verify(world.get(),table);
        add=validated.add;pick=validated.pick;
        checked=true;
        return add;
    }
};
}
