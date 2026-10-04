#pragma once
#include "Pose.h"
#include <array>
#include <span>

namespace fc {
class ExitPoseTrace {
public:
    static constexpr std::array<std::size_t,14> tracks{0,4,5,8,11,24,26,28,29,31,32,36,38,39};
    static constexpr std::size_t capacity=24;
    struct Frame {
        std::uint64_t wallMs{},nativeFrame{};
        std::uint32_t applied{},stamp{};
        float delta{},realDelta{},clockDt{},seconds{},weight{},recovery{};
        unsigned pass{};
        bool timer{},stable{},paused{},terminal{};
    };
    struct Difference {
        float nativeAngle{},nativeDistance{},outputAngle{},outputDistance{};
    };
    struct Record {
        Frame frame;
        std::array<Difference,tracks.size()> changes{};
        std::uint64_t previousWallMs{};
        std::uint32_t overwritten{},worldMismatches{};
        float overwrittenAngle{},overwrittenDistance{},worldAngle{},worldDistance{};
        bool previous{};
    };
    struct Snapshot {
        Frame frame;
        std::array<Transform,tracks.size()> native{},output{};
        std::uint64_t serial{};
        bool valid{};
    };
private:
    std::array<Record,capacity> records{};
    Snapshot previous,older;
    std::uint64_t serial{};
    std::size_t count{},accepted{};
    bool enabled{},finished{};
    static bool sameFrame(const Frame& a,const Frame& b) {
        return a.timer==b.timer&&(a.timer?a.stamp==b.stamp:a.nativeFrame==b.nativeFrame);
    }
public:
    void reset(bool active=false) {
        ++serial;count=accepted=0;enabled=active;finished=false;previous.valid=older.valid=false;
    }
    bool active() const {return enabled&&!finished;}
    std::size_t frames() const {return accepted;}
    std::span<const Record> samples() const {return {records.data(),count};}
    Snapshot capture(Frame frame,const Pose& native,const Pose& output) const {
        Snapshot result;
        if(!active()||native.size()!=99||output.size()!=99)return result;
        result.frame=frame;result.serial=serial;result.valid=true;
        for(std::size_t i=0;i<tracks.size();++i) {
            result.native[i]=native[tracks[i]];result.output[i]=output[tracks[i]];
        }
        return result;
    }
    bool commit(const Snapshot& sample,std::uint32_t overwritten,std::uint32_t worldMismatches,
        float overwrittenAngle=0,float overwrittenDistance=0,float worldAngle=0,float worldDistance=0) {
        if(!active()||!sample.valid||sample.serial!=serial)return false;
        if(previous.valid&&sample.frame.wallMs<previous.frame.wallMs)return false;
        const bool duplicate=previous.valid&&sameFrame(sample.frame,previous.frame);
        if(duplicate&&(!sample.frame.terminal||(previous.frame.terminal&&(!records[count-1].worldMismatches||worldMismatches))))return false;
        const auto& before=duplicate?older:previous;
        Record result;
        result.frame=sample.frame;result.previous=before.valid;
        result.previousWallMs=before.frame.wallMs;
        result.overwritten=overwritten;result.worldMismatches=worldMismatches;
        result.overwrittenAngle=overwrittenAngle;result.overwrittenDistance=overwrittenDistance;
        result.worldAngle=worldAngle;result.worldDistance=worldDistance;
        if(before.valid)for(std::size_t i=0;i<tracks.size();++i) {
            result.changes[i]={angleBetween(sample.native[i].q,before.native[i].q),(sample.native[i].t-before.native[i].t).length(),
                angleBetween(sample.output[i].q,before.output[i].q),(sample.output[i].t-before.output[i].t).length()};
        }
        if(!duplicate) {
            older=previous;++accepted;
            if(count<capacity)++count;
        }
        previous=sample;records[count-1]=result;return true;
    }
    bool finish() {
        if(!active())return false;
        finished=true;return true;
    }
};
}
