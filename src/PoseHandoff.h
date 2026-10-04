#pragma once
#include "Pose.h"
#include "PoseBlendEnvelope.h"

namespace fc {
class PoseHandoff {
    Pose entry,displayed,olderDisplayed,displayedSource,olderSource,exitSource,nativeExitSource;
    PoseContinuation entryContinuation,exitContinuation,nativeExitContinuation;
    float displayedTime{},olderTime{};
    float displayedRecovery{},exitWeight=1;
    std::uint64_t revision{};
    bool exiting{},movingExit{},nativeExit{},seededEntry{},entryClockStarted{},nativeExitGuardReady{};
    float nativeExitAge{},entryOrigin{},entryElapsed{};
    std::array<bool,2> entryArmGuard{},nativeExitArmGuard{};
    static bool finitePose(const Pose& pose) {
        if(pose.size()!=99)return false;
        for(const auto& bone:pose) {
            const float norm=bone.q.dot(bone.q);
            if(!bone.t.finite()||!bone.s.finite()||!std::isfinite(norm)||std::abs(norm-1.f)>.01f)return false;
        }
        return true;
    }
public:
    static constexpr float nativeExitSeconds=.24f;
    struct Output {
        Pose pose,source;
        float recovery{};
        std::uint64_t revision{};
        float sampleTime{};
    };
    void clear() {const auto next=revision+1;*this={};revision=next;}
    void beginEntry(const Pose& current,const Pose& older,float interval) {
        clear();
        if(!finitePose(current))return;
        entry=current;seededEntry=true;
        if(older.size()==current.size()&&finitePose(older)&&std::isfinite(interval)&&interval>0)
            entryContinuation.begin(current,older,interval);
        else entryContinuation.begin(current,{},0);
    }
    void advanceEntrySource(float elapsed,const Library& library) {
        if(!seededEntry||exiting||!std::isfinite(elapsed)||elapsed<0)return;
        if(!entryClockStarted) {
            entryOrigin=elapsed;entryClockStarted=true;
            for(int hand=0;hand<2;++hand)entryArmGuard[hand]=library.armBendValid(entry,hand);
            return;
        }
        const float age=elapsed-entryOrigin;
        if(age<=entryElapsed)return;
        auto candidate=entryContinuation.sample(age);
        if(!finitePose(candidate))return;
        for(int hand=0;hand<2;++hand)if(entryArmGuard[hand])library.guardArmBend(candidate,hand);
        if(!finitePose(candidate))return;
        entry=std::move(candidate);entryElapsed=age;
    }
    bool hasOutput() const {return !displayed.empty();}
    float exitContribution() const {return exitWeight;}
    bool nativeExitActive() const {return nativeExit&&nativeExitAge<nativeExitSeconds;}
    bool beginExit(bool continueMotion=false,bool smoothNativeTakeover=false) {
        if(!hasOutput())return false;

        exitSource=displayedSource;exitWeight=1-displayedRecovery;
        movingExit=continueMotion;
        if(movingExit)exitContinuation.begin(displayedSource,olderSource,displayedTime-olderTime);

        nativeExit=smoothNativeTakeover&&!continueMotion;nativeExitAge=0;nativeExitGuardReady=false;nativeExitArmGuard={};
        if(nativeExit) {
            nativeExitSource=displayed;
            nativeExitContinuation.begin(displayed,olderDisplayed,displayedTime-olderTime);
        }

        ++revision;exiting=true;seededEntry=false;return true;
    }
    void advanceExitSource(float elapsed,const Library& library,float acknowledgedElapsed=-1) {
        if(!exiting||!std::isfinite(acknowledgedElapsed))return;
        const float outputElapsed=acknowledgedElapsed>=0?acknowledgedElapsed:elapsed;
        if(!std::isfinite(outputElapsed)||outputElapsed<0)return;
        if(movingExit) {
            exitSource=exitContinuation.sample(outputElapsed);
            library.guardArmBends(exitSource);
        }
        if(nativeExit) {
            if(!nativeExitGuardReady) {
                for(int hand=0;hand<2;++hand)nativeExitArmGuard[hand]=library.armBendValid(nativeExitSource,hand);
                nativeExitGuardReady=true;
            }
            nativeExitAge=std::max(nativeExitAge,std::max(0.f,outputElapsed));
            if(nativeExitAge<nativeExitSeconds) {
                nativeExitSource=nativeExitContinuation.sample(nativeExitAge);
                if(nativeExitAge>0)for(int hand=0;hand<2;++hand)if(nativeExitArmGuard[hand])library.guardArmBend(nativeExitSource,hand);
            }
        }
    }
    Output evaluate(const Pose& native,const Pose& authored,float weight,float recovery=0,float sampleTime=0) {
        if(native.size()!=authored.size())return {};
        weight=std::clamp(weight,0.f,1.f);recovery=std::clamp(recovery,0.f,1.f);
        if(entry.empty())entry=native;
        if(weight>=1.f)seededEntry=false;
        Output result;result.source=native;result.pose=native;
        result.revision=revision;
        result.sampleTime=sampleTime;
        result.recovery=nativeExit?PoseBlendEnvelope::exitRecovery(nativeExitAge/nativeExitSeconds):exiting?1-weight*exitWeight:recovery;
        for(std::size_t i=0;i<native.size();++i) {
            result.source[i]=nativeExit?nativeExitSource[i]:exiting?exitSource[i]:blend(entry[i],authored[i],weight);
            result.pose[i]=blend(result.source[i],native[i],result.recovery);
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
