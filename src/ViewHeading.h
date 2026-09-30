#pragma once
#include "YawFrame.h"

namespace fc {

class ViewHeading {
    float heading{},velocity{};
    bool initialized{};
public:
    static constexpr float maxSpeed=6.f;
    static constexpr float response=14.f;
    void reset() {initialized=false;velocity=0;}
    bool ready() const {return initialized;}
    float value() const {return heading;}
    float speed() const {return velocity;}
    void begin(float actual) {
        if(!std::isfinite(actual)){reset();return;}
        heading=normalizeYaw(actual);velocity=0;initialized=true;
    }
    float advance(float target,float dt) {
        if(!initialized||!std::isfinite(target)||!std::isfinite(dt)||dt<=0)return heading;
        dt=std::min(dt,.05f);
        const float error=yawDifference(target,heading);

        const float offset=-error,c=velocity+response*offset;
        const float decay=std::exp(-response*dt);
        const float nextOffset=(offset+c*dt)*decay;
        float step=std::clamp(nextOffset-offset,-maxSpeed*dt,maxSpeed*dt);
        velocity=std::clamp((velocity-response*c*dt)*decay,-maxSpeed,maxSpeed);

        if(step*error>0&&std::abs(step)>std::abs(error)){step=error;velocity=0;}
        heading=normalizeYaw(heading+step);
        return heading;
    }
};
}
