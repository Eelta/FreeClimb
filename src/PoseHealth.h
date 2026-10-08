#pragma once
#include <algorithm>
#include <cstdint>
#include <cmath>
namespace fc {
inline bool recentPoseCallback(std::uint64_t now,std::uint64_t last) {
    return last!=0&&now>=last&&now-last<=250;
}

class PoseHealth {
    std::uint32_t last{};
    float silence{};
    bool seen{};
public:
    void reset(){last=0;silence=0;seen=false;}
    void resume(std::uint32_t count){last=count;silence=.13f;}
    void sample(std::uint32_t count,float dt) {
        if(count!=last){seen=true;silence=0;last=count;}
        else if(std::isfinite(dt)&&dt>0)silence+=std::clamp(dt,0.f,.05f);
    }
    bool ready()const{return seen&&silence<=.12f;}
    void invalidate(){silence=std::max(silence,.13f);}
    bool failed()const{return silence>=(seen?1.5f:.75f);}
    float stale()const{return silence;}
};
}

