#pragma once
#include "Core.h"

namespace fc {

class GroundMotionProbe {
    Vec previous{},heading{};
    float stalled{},cooldown{};
    bool initialized{};
    unsigned reports{};
public:
    void suspend(){initialized=false;stalled=0;}
    bool sample(Vec position,Vec intent,float dt,bool eligible) {
        if(!std::isfinite(dt)||dt<=0)return false;
        dt=std::min(dt,.05f);cooldown=std::max(0.f,cooldown-dt);
        intent.z=0;
        if(!eligible||!position.finite()||!intent.finite()||intent.length()<.1f) {suspend();return false;}
        intent=intent.unit();
        const Vec delta=position-previous;
        if(!initialized||intent.dot(heading)<.7f||delta.length()>50) {
            initialized=true;previous=position;heading=intent;stalled=0;return false;
        }
        previous=position;heading=intent;

        const float speed=delta.length()/dt;
        if(speed>8)stalled=0;else stalled+=dt;
        if(stalled<.65f||cooldown>0||reports>=64)return false;
        ++reports;cooldown=5;stalled=0;return true;
    }
};
}
