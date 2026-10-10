#pragma once

inline bool climbGripMotion(const Library& lib,Motion motion) {
    return (motion==Motion::hang||(motion>=Motion::up&&motion<=Motion::right))&&
        !lib.clip(motion).authoredPlayback&&!lib.hasAnimationOverride(motion);
}

inline void applyClimbGrip(const Library& lib,Pose& pose,int hand,float weight) {
    if(hand<0||hand>1||pose.size()!=99||!std::isfinite(weight)||weight<=0)return;
    const auto& reference=lib.rig.source().empty()?lib.rest:lib.rig.source();
    if(reference.size()!=99)return;
    static constexpr std::array<std::array<float,3>,5> curl{{
        {30,30,35},{40,64,38},{44,70,43},{46,73,45},{48,68,42}
    }};
    constexpr float radians=.01745329252f;
    const float amount=smooth(std::min(weight,1.f));
    const int first=hand?82:67;
    for(int digit=0;digit<5;++digit)for(int joint=0;joint<3;++joint) {
        const auto bone=first+digit*3+joint;
        auto target=(reference[bone].q*Quat::axis({1,0,0},curl[digit][joint]*radians)).unit();
        if(digit==0&&joint==0)target=(Quat::axis({0,0,1},(hand?-8.f:8.f)*radians)*target).unit();
        pose[bone].q=blend(pose[bone].q,lib.rig.rotation(bone,target),amount);
    }
}

struct ClimbGripPose {
    std::array<float,2> weights{};
    void update(const Library& lib,Pose& pose,Motion motion,std::array<float,2> contacts,float dt,bool allow=true) {
        if(!allow||!climbGripMotion(lib,motion)){weights={};return;}
        const float elapsed=std::isfinite(dt)?std::clamp(dt,0.f,.05f):0.f;
        for(int hand=0;hand<2;++hand) {
            const float target=std::isfinite(contacts[hand])?std::clamp(contacts[hand],0.f,1.f):0.f;
            const float seconds=target>weights[hand]?.24f:.22f;
            weights[hand]+=std::clamp(target-weights[hand],-elapsed/seconds,elapsed/seconds);
            applyClimbGrip(lib,pose,hand,weights[hand]);
        }
    }
};
