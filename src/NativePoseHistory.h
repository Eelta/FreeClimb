#pragma once
#include "YawFrame.h"
#include <cstdint>
#include <optional>

namespace fc {
class NativePoseHistory {
    Pose current,older;
    std::uint64_t currentTime{},olderTime{},currentFrame{},lastSeen{};
    static bool validRotation(Quat rotation) {
        const float norm=rotation.dot(rotation);
        return std::isfinite(norm)&&std::abs(norm-1.f)<.01f;
    }
    static bool validPose(const Pose& pose) {
        if(pose.size()!=99)return false;
        for(const auto& bone:pose)
            if(!bone.t.finite()||!bone.s.finite()||bone.s.x<=0||bone.s.y<=0||bone.s.z<=0||!validRotation(bone.q))return false;
        return true;
    }
public:
    struct Seed {Pose current,older;float seconds{};};
    void clear(){*this={};}
    bool capture(const Pose& native,Quat parentWorld,std::uint64_t now,std::uint64_t frame) {
        if(!validPose(native)||!validRotation(parentWorld)||(!current.empty()&&(now<lastSeen||frame<currentFrame)))return false;
        auto value=native;parentWorld=parentWorld.unit();
        value[0].t=parentWorld.rotate(value[0].t);
        value[0].q=(parentWorld*value[0].q).unit();
        if(!validPose(value))return false;
        if(current.empty()||frame!=currentFrame) {
            if(!current.empty()&&now-currentTime<=250){older=std::move(current);olderTime=currentTime;}
            else {older.clear();olderTime=0;}
            currentTime=now;currentFrame=frame;
        }
        current=std::move(value);lastSeen=now;return true;
    }
    bool capture(const Pose& native,Quat parentWorld,std::uint64_t now){return capture(native,parentWorld,now,now);}
    std::optional<Seed> seed(float wallYaw,std::uint64_t now) const {
        if(current.empty()||now<lastSeen||now-lastSeen>250||!std::isfinite(wallYaw))return {};
        Seed result{current,older.empty()?current:older,older.empty()?0.f:float(currentTime-olderTime)/1000.f};
        const WallYawFrame frame({},wallYaw);
        result.current[0]=frame.toWall(result.current[0]);result.older[0]=frame.toWall(result.older[0]);
        if(!validPose(result.current)||!validPose(result.older))return {};
        return result;
    }
};
}
