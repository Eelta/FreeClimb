#pragma once
#include "Pose.h"

namespace fc {
inline float normalizeYaw(float yaw) {
    constexpr float circle=6.28318530718f;
    yaw=std::fmod(yaw,circle);
    if(yaw<0)yaw+=circle;

    return yaw>=circle?0.f:yaw;
}
inline float yawDifference(float target,float actual) {
    return std::remainder(target-actual,6.28318530718f);
}
inline std::optional<float> headingYaw(Vec forward) {
    if(!forward.finite()||forward.x*forward.x+forward.y*forward.y<1e-8f)return {};
    return normalizeYaw(std::atan2(forward.x,forward.y));
}
inline std::optional<float> facingWallYaw(Vec outward) {return headingYaw(outward*-1);}
inline std::optional<float> frameYaw(Quat rotation) {return headingYaw(rotation.rotate({0,1,0}));}

struct WallYawFrame {
    Quat parentFromWall;
    WallYawFrame(Quat parentWorld,float wallYaw):
        parentFromWall((parentWorld.inverse()*Quat::axis({0,0,1},-wallYaw)).unit()) {}
    Transform toParent(Transform value) const {
        value.t=parentFromWall.rotate(value.t);
        value.q=(parentFromWall*value.q).unit();return value;
    }
    Transform toWall(Transform value) const {
        const auto inverse=parentFromWall.inverse();
        value.t=inverse.rotate(value.t);
        value.q=(inverse*value.q).unit();return value;
    }
};
}
