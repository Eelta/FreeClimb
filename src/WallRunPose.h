#pragma once
#include "Pose.h"
#include <mutex>
namespace fc {

class NativeRunDrive {
    std::mutex mutex;
    bool active{};
    std::uint64_t lastKick{},staleSince{},healthySince{};
    unsigned retries{};
public:
    void reset(){std::scoped_lock lock(mutex);active=false;lastKick=staleSince=healthySince=0;retries=0;}
    template<class Dispatch> bool update(bool requested,bool fresh,std::uint64_t now,Dispatch dispatch) {
        int action=0;
        {
            std::scoped_lock lock(mutex);
            if(requested!=active) {
                active=requested;action=requested?1:-1;lastKick=now;staleSince=healthySince=0;retries=0;
            } else if(active) {
                if(fresh){staleSince=0;if(!healthySince)healthySince=now;if(now-healthySince>=750)retries=0;}
                else {
                    healthySince=0;if(!staleSince)staleSince=now;
                    if(now-staleSince>=180&&now-lastKick>=650&&retries<3){action=2;lastKick=now;++retries;}
                }
            }
        }
        if(!action)return false;

        if(action>0){if(action==1)dispatch("IdleForceDefaultState");dispatch("moveStart");dispatch("SprintStart");}
        else {dispatch("SprintStop");dispatch("moveStop");}
        return true;
    }
};
class WallModeTransition {
    bool target{};
    float phase=1,from{};
public:
    void reset(){target=false;phase=1;from=0;}
    float fraction()const{return smooth(phase);}
    float weight()const{return from+((target?1.f:0.f)-from)*fraction();}
    bool select(bool sprint) {
        if(sprint==target)return false;
        from=weight();target=sprint;phase=0;return true;
    }
    void advance(float dt){phase=std::min(1.f,phase+std::clamp(dt,0.f,.05f)/(target?.46f:.40f));}
};
inline bool animatedNativePose(const Pose& native,const Pose& rest) {
    if(native.size()!=99||rest.size()!=99)return false;
    float difference=0;
    for(int i:{6,7,9,10,28,29,31,32}) {
        if(!native[i].t.finite()||!std::isfinite(native[i].q.dot(native[i].q)))return false;
        difference+=angleBetween(native[i].q,rest[i].q);
    }
    return difference>.2f;
}
class NativeRunEvidence {
    std::array<Quat,4> previous{};
    std::uint64_t lastMotion{};
    std::uint64_t motionStart{};
    unsigned changes{};
    bool initialized{};
public:
    void reset(){*this={};}
    bool sample(const Pose& native,const Pose& rest,bool requested,std::uint64_t now) {
        if(!requested||!animatedNativePose(native,rest)){reset();return false;}
        constexpr int legs[]={6,7,9,10};float change=0;
        for(int i=0;i<4;++i) {
            if(initialized)change+=angleBetween(previous[i],native[legs[i]].q);
            previous[i]=native[legs[i]].q;
        }
        if(initialized&&change>.015f) {
            if(!lastMotion||now-lastMotion>100){motionStart=now;changes=0;}
            lastMotion=now;++changes;
        }
        initialized=true;
        return changes>=3&&now>=motionStart&&now-motionStart>=60&&lastMotion!=0&&now>=lastMotion&&now-lastMotion<=100;
    }
};

inline Pose nativeWallRun(const Pose& native,float slope,float gap,float scale,Vec direction) {
    if(native.size()!=99)return {};
    Pose result=native;
    const float heading=std::atan2(-direction.x,direction.y);
    const Quat rotation=Quat::axis({1,0,0},std::acos(std::clamp(slope,-.15f,.75f)))*Quat::axis({0,0,1},heading);
    result[0].q=rotation*native[0].q;
    result[0].t=rotation.rotate(native[0].t)+Vec{0,(gap-5)/std::clamp(scale,.5f,2.f),0};
    return result;
}
}
