#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace fc {

class PoseBlendEnvelope {
    enum class Mode { idle, entering, exiting };
    Mode mode=Mode::idle;
    float phase{},duration=.28f;
    std::uint32_t observedOutputs{};
    static float ease(float x) {x=std::clamp(x,0.f,1.f);return x*x*x*(10+x*(-15+6*x));}
    static float step(float dt) {return std::isfinite(dt)&&dt>0?std::min(dt,.05f):0.f;}
public:
    static constexpr float entrySeconds=.18f,exitSeconds=.28f,fallExitSeconds=.16f;
    void clear() {*this={};}
    void beginEntry() {mode=Mode::entering;phase=0;observedOutputs=0;}
    float weight() const {
        if(mode==Mode::idle)return 0;

        return mode==Mode::entering?.01f+.99f*ease(phase):1-ease(phase);
    }
    float elapsedExitSeconds() const {return mode==Mode::exiting?phase*duration:0.f;}
    void advanceEntry(std::uint32_t applied,float dt) {
        const float elapsed=step(dt);
        if(mode!=Mode::entering||applied==observedOutputs||elapsed<=0)return;
        observedOutputs=applied;phase=std::min(1.f,phase+elapsed/entrySeconds);
    }

    void beginExit(std::uint32_t applied=0,float seconds=exitSeconds) {
        mode=Mode::exiting;phase=0;observedOutputs=applied;
        duration=std::isfinite(seconds)&&seconds>0?std::clamp(seconds,.05f,1.f):exitSeconds;
    }
    void advanceExit(std::uint32_t applied,float dt) {
        const float elapsed=step(dt);
        if(mode!=Mode::exiting||applied==observedOutputs||elapsed<=0)return;
        observedOutputs=applied;phase=std::min(1.f,phase+elapsed/duration);
    }
};
}
