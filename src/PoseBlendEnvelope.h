#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace fc {

class PoseBlendEnvelope {
    enum class Mode { idle, entering, exiting };
    Mode mode=Mode::idle;
    float phase{},duration=.28f,pendingExit{};
    std::uint32_t observedOutputs{};
    bool terminalOutput{},exitStarted{},continuousExit{};
    static float ease(float x) {x=std::clamp(x,0.f,1.f);return x*x*x*(10+x*(-15+6*x));}
    static float step(float dt) {return std::isfinite(dt)&&dt>0?std::min(dt,.05f):0.f;}
public:
    static constexpr float entrySeconds=.18f,exitSeconds=.28f,fallExitSeconds=.16f;
    static float exitRecovery(float x) {
        x=std::clamp(x,0.f,1.f);constexpr float shoulder=.125f;
        if(x>1-shoulder)return 1-exitRecovery(1-x);
        if(x>=shoulder)return (x-shoulder*.5f)/(1-shoulder);
        const float u=x/shoulder;
        return shoulder*u*u*u*u*(2.5f+u*(-3+u))/(1-shoulder);
    }
    void clear() {*this={};}
    void beginEntry() {mode=Mode::entering;phase=0;observedOutputs=0;}
    float weight() const {
        if(mode==Mode::idle)return 0;

        return mode==Mode::entering?ease(phase):1-(continuousExit?exitRecovery(phase):ease(phase));
    }
    float elapsedExitSeconds() const {return mode==Mode::exiting?phase*duration:0.f;}
    void advanceEntry(std::uint32_t applied,float dt) {
        const float elapsed=step(dt);
        if(mode!=Mode::entering||applied==observedOutputs||elapsed<=0)return;
        observedOutputs=applied;phase=std::min(1.f,phase+elapsed/entrySeconds);
    }

    void beginExit(std::uint32_t applied=0,float seconds=exitSeconds,bool continuous=false) {
        mode=Mode::exiting;phase=pendingExit=0;observedOutputs=applied;terminalOutput=exitStarted=false;
        continuousExit=continuous;
        duration=std::isfinite(seconds)&&seconds>0?std::clamp(seconds,.05f,1.f):exitSeconds;
    }
    void advanceExit(std::uint32_t applied,float dt) {
        const float elapsed=step(dt);
        if(mode!=Mode::exiting||phase>=1.f||elapsed<=0)return;
        if(!exitStarted) {
            if(applied==observedOutputs)return;
            exitStarted=true;
        }
        pendingExit=std::min(.05f,pendingExit+elapsed);
        if(applied==observedOutputs)return;
        observedOutputs=applied;phase=std::min(1.f,phase+pendingExit/duration);pendingExit=0;
    }
    bool exitComplete(std::uint32_t applied) const {
        return mode==Mode::exiting&&phase>=1.f&&terminalOutput&&applied!=observedOutputs;
    }
    void acknowledgeExit(bool native) {if(mode==Mode::exiting&&phase>=1.f)terminalOutput=native;}
};
}
