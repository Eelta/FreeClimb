#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace fc {
class PoseFrameClock {
    std::uint32_t previousFrame{};
    bool primed{};
public:
    void clear(){*this={};}
    float sample(std::uint32_t frameMilliseconds,float delta,float realTimeDelta,bool paused){
        const auto interval=frameMilliseconds-previousFrame;const bool ready=primed;
        previousFrame=frameMilliseconds;
        if(paused||!std::isfinite(delta)||!std::isfinite(realTimeDelta)||delta<=0||realTimeDelta<=0){primed=false;return 0;}
        primed=true;
        if(!ready||interval==0||interval>0x7fffffffu)return 0;
        const double elapsed=double(interval)*.001*double(delta)/double(realTimeDelta);
        return float(std::min(elapsed,.05));
    }
};
}
