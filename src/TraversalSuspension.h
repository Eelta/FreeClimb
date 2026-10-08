#pragma once
#include "InputBindings.h"
#include <span>

namespace fc {
inline bool gameTimeSuspended(float dt,bool frozen=false,unsigned pauseCount=0) {
    return frozen||pauseCount!=0||!std::isfinite(dt)||dt<=1e-6f;
}
class TraversalSuspension {
    bool held{};
public:
    bool suspend(){const bool changed=!held;held=true;return changed;}
    bool resume(){const bool changed=held;held=false;return changed;}
    bool active()const{return held;}
    void clear(){held=false;}
};
class KeyboardReleaseGate {
    bool blocked{};
public:
    void suspend(){blocked=true;}
    bool sample(std::span<const std::uint8_t,256> raw,const InputBindings& bindings) {
        if(blocked) {
            for(unsigned scan=0;scan<raw.size();++scan)if((raw[scan]&0x80)&&ownsScan(bindings,scan))return true;
            blocked=false;
        }
        return blocked;
    }
    bool waiting()const{return blocked;}
};
}
