#pragma once
#include "Pose.h"

namespace fc {
inline float topRecoveryBegin(float seconds) {
    return std::isfinite(seconds)&&seconds>0?std::max(.72f,1.f-.24f/seconds):1.f;
}
inline float topRecovery(Motion motion,float progress,float seconds) {
    const float begin=topRecoveryBegin(seconds);
    if(motion!=Motion::contextMantle||!std::isfinite(progress)||begin>=1.f||progress<=begin)return 0.f;
    if(progress>=1.f)return 1.f;
    return std::clamp(smooth((progress-begin)/(1.f-begin)),0.f,1.f);
}

class TopRecoveryGate {
    bool attempted{},accepted{};
    float seconds{};
public:
    void clear() {*this={};}
    bool request(State state,float progress,float duration) {
        const float begin=topRecoveryBegin(duration);
        if(attempted||state!=State::mantle||!std::isfinite(progress)||begin>=1.f||progress<begin)return false;
        seconds=duration;attempted=true;return true;
    }
    void resolve(bool success) {if(attempted)accepted=success;}
    bool ready() const {return accepted;}
    float weight(Motion motion,float progress) const {return accepted?topRecovery(motion,progress,seconds):0.f;}
};

class PoseHandoff {
    Pose entry,displayed,olderDisplayed,displayedSource,olderSource,exitSource,nativeExitSource;
    PoseContinuation exitContinuation,nativeExitContinuation;
    float displayedTime{},olderTime{};
    float displayedRecovery{},exitWeight=1;
    std::uint64_t revision{};
    bool exiting{},movingExit{},nativeExit{};
    float nativeExitAge{};
public:
    static constexpr float nativeExitSeconds=.12f;
    struct Output {
        Pose pose,source;
        float recovery{};
        std::uint64_t revision{};
        float sampleTime{};
    };
    void clear() {const auto next=revision+1;*this={};revision=next;}
    bool hasOutput() const {return !displayed.empty();}
    float exitContribution() const {return exitWeight;}
    bool nativeExitActive() const {return nativeExit&&nativeExitAge<nativeExitSeconds;}
    bool beginExit(bool continueMotion=false,bool smoothNativeTakeover=false) {
        if(!hasOutput())return false;

        exitSource=displayedSource;exitWeight=1-displayedRecovery;
        movingExit=continueMotion;
        if(movingExit)exitContinuation.begin(displayedSource,olderSource,displayedTime-olderTime);

        nativeExit=smoothNativeTakeover&&!continueMotion;nativeExitAge=0;
        if(nativeExit) {
            nativeExitSource=displayed;
            nativeExitContinuation.begin(displayed,olderDisplayed,displayedTime-olderTime);
        }

        ++revision;exiting=true;return true;
    }
    void advanceExitSource(float elapsed,const Library& library,float acknowledgedNativeElapsed=-1) {
        if(!exiting)return;
        if(movingExit) {
            exitSource=exitContinuation.sample(elapsed);
            library.guardArmBends(exitSource);
        }
        const float nativeElapsed=acknowledgedNativeElapsed>=0?acknowledgedNativeElapsed:elapsed;
        if(nativeExit&&std::isfinite(nativeElapsed)) {
            nativeExitAge=std::max(nativeExitAge,std::max(0.f,nativeElapsed));
            if(nativeExitAge<nativeExitSeconds) {
                nativeExitSource=nativeExitContinuation.sample(nativeExitAge);
                if(nativeExitAge>0)library.guardArmBends(nativeExitSource);
            }
        }
    }
    Output evaluate(const Pose& native,const Pose& authored,float weight,float recovery=0,float sampleTime=0) {
        if(native.size()!=authored.size())return {};
        weight=std::clamp(weight,0.f,1.f);recovery=std::clamp(recovery,0.f,1.f);
        if(entry.empty())entry=native;
        Output result;result.source=native;result.pose=native;
        result.revision=revision;
        result.sampleTime=sampleTime;
        result.recovery=exiting?1-weight*exitWeight:recovery;
        for(std::size_t i=0;i<native.size();++i) {
            result.source[i]=exiting?exitSource[i]:blend(entry[i],authored[i],weight);
            result.pose[i]=blend(result.source[i],native[i],result.recovery);
            if(nativeExitActive())result.pose[i]=blend(nativeExitSource[i],result.pose[i],smooth(nativeExitAge/nativeExitSeconds));
        }
        return result;
    }

    bool consumed(const Output& result) {
        if(result.revision!=revision||result.pose.empty()||result.pose.size()!=result.source.size())return false;
        if(!exiting) {

            if(result.sampleTime<displayedTime)return false;
            if(result.sampleTime>displayedTime) {
                olderSource=displayedSource;olderTime=displayedTime;
                olderDisplayed=displayed;
                displayedTime=result.sampleTime;
            }
        }
        displayed=result.pose;displayedSource=result.source;displayedRecovery=result.recovery;
        return true;
    }

    Pose compose(const Pose& native,const Pose& authored,float weight,float recovery=0) {
        auto result=evaluate(native,authored,weight,recovery);
        consumed(result);return result.pose.empty()?native:result.pose;
    }
};
}
