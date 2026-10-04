#include "PoseHealth.h"
#include "PoseBlendEnvelope.h"
#include "PoseHandoff.h"
#include <stdexcept>
#include <iostream>
using namespace fc;
void check(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
static void samePose(const Pose& a,const Pose& b,const char* reason) {
    check(a.size()==99&&b.size()==99,reason);
    for(std::size_t i=0;i<a.size();++i)
        check((a[i].t-b[i].t).length()<.0001f&&angleBetween(a[i].q,b[i].q)<.002f&&(a[i].s-b[i].s).length()<.0001f,reason);
}
static Pose mixPoses(const Pose& a,const Pose& b,float weight) {
    Pose out=a;for(std::size_t i=0;i<out.size();++i)out[i]=blend(a[i],b[i],weight);return out;
}
static void exitClockBoundaries(){
    PoseBlendEnvelope e;e.beginExit(0);
    for(int tick=0;tick<120;++tick)e.advanceExit(0,.01f);
    check(e.elapsedExitSeconds()==0&&e.weight()==1,"waiting for the first callback accumulates no hidden exit time");
    for(float dt:{0.f,-1.f,NAN,INFINITY})e.advanceExit(1,dt);
    e.advanceExit(1,.01f);
    check(std::abs(e.elapsedExitSeconds()-.01f)<.000001f,"first valid callback begins with only its current update after an arbitrarily long initial wait");
    e.advanceExit(1,.01f);e.advanceExit(1,.01f);
    check(std::abs(e.elapsedExitSeconds()-.01f)<.000001f,"updates without a new displayed frame cannot change the visible exit phase");
    for(float dt:{0.f,-1.f,NAN,INFINITY})e.advanceExit(2,dt);
    check(std::abs(e.elapsedExitSeconds()-.01f)<.000001f,"invalid updates cannot consume an acknowledgement or previously accumulated valid time");
    e.advanceExit(2,.01f);
    check(std::abs(e.elapsedExitSeconds()-.04f)<.000001f,"a resumed callback consumes the valid updates between ordinary rendered frames");
    e.advanceExit(9,.01f);
    check(std::abs(e.elapsedExitSeconds()-.05f)<.000001f,"seven scene callbacks in one update contribute one update of elapsed time");
    const float beforeStall=e.elapsedExitSeconds();
    for(int tick=0;tick<120;++tick)e.advanceExit(9,.02f);
    check(e.elapsedExitSeconds()==beforeStall,"an extended render stall cannot advance an unseen fade");
    for(float dt:{0.f,-1.f,NAN,INFINITY})e.advanceExit(10,dt);
    check(e.elapsedExitSeconds()==beforeStall,"invalid resumption does not consume a full pending recovery interval");
    e.advanceExit(10,.01f);
    check(std::abs(e.elapsedExitSeconds()-beforeStall-.05f)<.000001f,"a long render stall recovers by at most fifty milliseconds on its next confirmed frame");
    e.advanceExit(11,.01f);
    check(std::abs(e.elapsedExitSeconds()-beforeStall-.06f)<.000001f,"time beyond the recovery cap is discarded instead of leaking into later frames");
    for(int tick=0;tick<120;++tick)e.advanceExit(11,.02f);
    e.beginExit(11);
    for(int tick=0;tick<120;++tick)e.advanceExit(11,.01f);
    e.advanceExit(12,.01f);
    check(std::abs(e.elapsedExitSeconds()-.01f)<.000001f,"a fresh exit discards both previous pending time and the previous callback-start state");
    e.clear();e.beginExit(0xffffffffu);
    for(int tick=0;tick<120;++tick)e.advanceExit(0xffffffffu,.01f);
    e.advanceExit(0,.01f);
    check(std::abs(e.elapsedExitSeconds()-.01f)<.000001f,"counter rollover to zero is a valid first callback without inheriting initial wait time");
    e.advanceExit(0,.01f);e.advanceExit(1,.5f);
    check(std::abs(e.elapsedExitSeconds()-.06f)<.000001f,"a long individual update and pending time share the same fifty-millisecond output cap");
}
static unsigned exitClockCadence(int fps,unsigned interval,unsigned passes,float seconds){
    const float dt=1.f/fps;PoseBlendEnvelope e;e.beginExit(0,seconds);std::uint32_t applied=0;
    for(unsigned frame=1;frame<=unsigned(fps);++frame){
        const bool rendered=(frame-1)%interval==0;const float before=e.elapsedExitSeconds();
        if(rendered)applied+=passes;
        e.advanceExit(applied,dt);
        if(rendered)check(std::abs(e.elapsedExitSeconds()-std::min(frame*dt,seconds))<.000001f,"ordinary sparse callbacks preserve elapsed simulation time rather than counting render frames");
        else check(e.elapsedExitSeconds()==before,"player updates alone do not publish a later fade phase");
        if(e.weight()==0){
            check(frame*dt>=seconds&&frame*dt<=seconds+interval*dt+.000001f,"normal callback spacing adds at most one render interval to the configured fade");
            check(!e.exitComplete(applied),"a sparse final weighted acknowledgement cannot prove a pure native terminal frame");return frame;
        }
    }
    check(false,"ordinary sparse rendering must finish the configured exit within one second");return 0;
}
static void renderedPoseHandoffs() {
    Pose nativeA(99),nativeB(99),custom(99);
    for(int i=0;i<99;++i) {
        nativeA[i]={{float(i)*.2f,float(i%7)-3,float(i%11)},Quat::axis({.2f,1,.3f},i*.005f),{1,1,1}};
        nativeB[i]={nativeA[i].t+Vec{12,-8,5},Quat::axis({1,.4f,.2f},.65f)*nativeA[i].q,{1.1f,1.1f,1.1f}};
        custom[i]={nativeA[i].t+Vec{-4,9,21},Quat::axis({.3f,.1f,1},1.1f)*nativeA[i].q,{1,1,1}};
    }
    PoseHandoff handoff;check(!handoff.hasOutput()&&!handoff.beginExit(),"unrendered output cannot be an exit source");
    const auto first=handoff.compose(nativeA,custom,.15f);
    samePose(first,mixPoses(nativeA,custom,.15f),"first custom contribution starts from the displayed native snapshot");
    samePose(handoff.compose(nativeB,custom,.15f),first,"a native graph refresh cannot move the entry source halfway through fade-in");
    samePose(handoff.compose(nativeB,custom,1),custom,"completed entry reaches every authored bone");
    samePose(handoff.compose(nativeB,custom,1,.55f),mixPoses(custom,nativeB,.55f),"top recovery blends the whole skeleton, including COM and legs");
    samePose(handoff.compose(nativeB,custom,1,1),nativeB,"top recovery reaches the exact live native endpoint for all 99 bones");
    samePose(handoff.compose(nativeA,custom,1,1),nativeA,"full recovery continues the native pose without an old COM offset");
    check(handoff.beginExit()&&handoff.exitContribution()==0,"a successfully rendered fully native top-out has no custom contribution left to fade");
    samePose(handoff.compose(nativeB,custom,1),nativeB,"top completion cannot replay a frozen native snapshot at exit weight one");
    samePose(handoff.compose(nativeA,custom,.8f),nativeA,"native movement continues immediately after a fully recovered top-out");
    handoff.clear();
    const auto terminalPublication=handoff.evaluate(nativeA,custom,1,1);
    check(!handoff.hasOutput()&&!handoff.beginExit(),"evaluating a complete recovery without successful propagation cannot claim displayed output");
    handoff.consumed(terminalPublication);
    check(handoff.hasOutput()&&handoff.beginExit()&&handoff.exitContribution()==0,"only confirmed propagation can complete native recovery");
    handoff.clear();
    const auto residual=handoff.compose(nativeA,custom,1,.92f);
    samePose(residual,mixPoses(custom,nativeA,.92f),"late top-out fixture has only eight percent custom pose remaining");

    const auto unconsumedTerminal=handoff.evaluate(nativeB,custom,1,1);
    check(!unconsumedTerminal.pose.empty(),"terminal publication was actually evaluated for the fixture");
    check(handoff.beginExit()&&std::abs(handoff.exitContribution()-.08f)<.00001f,
        "unconsumed recovery one cannot erase the last displayed custom residual");
    samePose(handoff.compose(nativeA,custom,1),residual,"late top-out release preserves the displayed pose when native has not moved yet");
    samePose(handoff.compose(nativeB,custom,1),mixPoses(custom,nativeB,.92f),
        "late top-out release retains ninety-two percent live native motion rather than freezing the displayed pose");
    samePose(handoff.compose(nativeA,custom,.5f),mixPoses(custom,nativeA,.96f),
        "exit fades only the remaining custom contribution without a second native takeover");
    samePose(handoff.compose(nativeB,custom,0),nativeB,"residual exit reaches the moving native endpoint exactly");
    handoff.clear();check(!handoff.hasOutput(),"cleanup discards stale displayed and entry poses");
    const auto displayed=handoff.compose(nativeA,custom,.7f);

    Pose unpublished=custom;for(auto& tr:unpublished){tr.t=tr.t+Vec{90,-70,40};tr.q=Quat::axis({1,0,0},2.4f)*tr.q;}
    check(handoff.beginExit(),"an observed partial entry is a valid release source");
    check(handoff.exitContribution()==1,"ordinary early release retains its baked entry source without multiplying its entry weight again");
    samePose(handoff.compose(nativeB,unpublished,1),displayed,"release starts at the observed pose and ignores an unconsumed terminal publication");
    samePose(handoff.compose(nativeB,unpublished,.5f),mixPoses(nativeB,displayed,.5f),"exit uses its captured source rather than recursively blending the last frame");
    samePose(handoff.compose(nativeA,unpublished,.25f),mixPoses(nativeA,displayed,.25f),"exit follows the current native target while retaining its fixed displayed source");
    samePose(handoff.compose(nativeB,unpublished,0),nativeB,"release finishes with no residual hand, leg or COM contribution");
    handoff.clear();samePose(handoff.compose(nativeB,custom,.15f),mixPoses(nativeB,custom,.15f),"reattachment captures the new native pose rather than reusing an old entry snapshot");
    {
        PoseHandoff interrupted;
        const auto observed=interrupted.evaluate(nativeA,custom,1,.60f);
        check(interrupted.consumed(observed),"fixture has a successfully displayed top recovery");
        const auto late=interrupted.evaluate(nativeB,custom,1,1);
        check(interrupted.beginExit(),"exit captures only the previously confirmed recovery");
        check(!interrupted.consumed(late),"a callback evaluated before exit cannot acknowledge the new exit phase after propagation returns");
        check(std::abs(interrupted.exitContribution()-.40f)<.00001f,"late native publication cannot erase the confirmed residual contribution");
        const auto exitFrame=interrupted.evaluate(nativeB,custom,.9f);
        check(interrupted.consumed(exitFrame),"a newly evaluated exit frame can acknowledge the new composition phase");
        interrupted.clear();
        check(!interrupted.consumed(exitFrame)&&!interrupted.hasOutput(),"cleanup rejects a retired callback without recreating a displayed source");
        const auto fresh=interrupted.evaluate(nativeA,custom,.1f);
        check(interrupted.consumed(fresh),"fresh attachment output remains acceptable after cleanup");
    }
    PoseBlendEnvelope envelope;envelope.beginExit(40);
    for(int i=0;i<120;++i)envelope.advanceExit(40,1.f/60);
    check(envelope.weight()==1,"a queued exit cannot finish while the renderer has not consumed any output");
    envelope.advanceExit(41,0);envelope.advanceExit(41,NAN);
    check(envelope.weight()==1,"invalid time cannot consume the pending exit acknowledgement");
    envelope.advanceExit(41,1.f/60);const float observed=envelope.weight();
    check(observed<1&&observed>.99f,"the first displayed exit frame begins gradually");
    for(int i=0;i<120;++i)envelope.advanceExit(41,1.f/60);
    check(envelope.weight()==observed,"a render stall preserves the last visible exit weight");
    for(std::uint32_t frame=42;frame<80;++frame)envelope.advanceExit(frame,1.f/60);
    check(envelope.weight()==0,"newly consumed exit frames eventually release the layer completely");
}
int main(){try {
    check(!recentPoseCallback(1000,0),"binding is not an observed callback");
    check(recentPoseCallback(1000,950),"recent render observation allows preflight");
    check(!recentPoseCallback(1000,700),"stale previous binding cannot authorize control");
    check(!recentPoseCallback(500,600),"invalid timestamp cannot authorize control");
    PoseHealth missing;
    for(int i=0;i<90;++i){missing.sample(0,1.f/60);check(!missing.ready(),"zero callbacks cannot move up the wall");}
    check(missing.failed(),"no output must terminate ownership, not freeze forever");
    missing.reset();check(!missing.ready()&&!missing.failed(),"new attempt resets watchdog");
    PoseHealth interrupted;
    for(unsigned i=1;i<120;++i){interrupted.sample(i,1.f/60);check(interrupted.ready()&&!interrupted.failed(),"healthy continuous output");}
    for(int i=0;i<20;++i)interrupted.sample(119,1.f/60);
    check(!interrupted.ready()&&!interrupted.failed(),"brief interruption holds geometry without dropping");
    interrupted.sample(120,1.f/60);check(interrupted.ready(),"output resumption recovers without reattach");
    interrupted.invalidate();check(!interrupted.ready(),"replacement must produce its own first pose");
    for(int i=0;i<100;++i)interrupted.sample(120,1.f/60);
    check(interrupted.failed(),"lost output must eventually restore control");
    {
        PoseBlendEnvelope blend;
        check(blend.weight()==0,"unused pose layer has no contribution");
        blend.beginEntry();const float first=blend.weight();
        check(first==0,"entry starts at the exact observed native pose before its first acknowledged callback");
        for(int i=0;i<60;++i)blend.advanceEntry(0,1.f/60);
        check(blend.weight()==first,"a delayed first callback cannot complete the entry blend invisibly");
        blend.advanceEntry(1,1.f/60);const float observed=blend.weight();
        check(observed>first&&observed<.04f,"first observed pose starts a gradual entry");
        for(int i=0;i<60;++i)blend.advanceEntry(1,1.f/60);
        check(blend.weight()==observed,"missing callbacks freeze fade-in progress at its displayed weight");
        blend.advanceEntry(2,NAN);blend.advanceEntry(2,0);
        check(blend.weight()==observed,"invalid or zero time cannot consume an output acknowledgement");
        blend.advanceEntry(2,1.f/60);
        check(blend.weight()>observed,"a valid frame can still consume the pending acknowledgement");
        for(unsigned i=3;i<30;++i)blend.advanceEntry(i,1.f/60);
        check(blend.weight()>.9999f,"observed entry reaches full custom weight");
        blend.clear();check(blend.weight()==0,"load or invalid-model cleanup immediately removes ownership");
        blend.beginEntry();for(unsigned i=1;i<=4;++i)blend.advanceEntry(i,1.f/60);
        const float entryWeight=blend.weight();
        check(entryWeight>.05f&&entryWeight<.9f,"release fixture is partway through entry");

        const float native=2,custom=18,displayed=native+(custom-native)*entryWeight;
        blend.beginExit();
        check(blend.weight()==1&&std::abs(native+(displayed-native)*blend.weight()-displayed)<.00001f,
            "release during entry preserves its exact displayed starting pose without a second entry multiplier");
        float previous=blend.weight(),firstDelta=0,lastDelta=0;
        for(int frame=0;frame<31;++frame) {
            blend.advanceExit(std::uint32_t(frame+1),.01f);const float current=blend.weight();
            check(current>=0&&current<=previous,"exit weight decreases monotonically");
            if(frame==0)firstDelta=previous-current;
            if(frame==27)lastDelta=previous-current;
            previous=current;
        }
        check(blend.weight()==0&&firstDelta<.003f&&lastDelta<.003f,"smooth exit eases both ends and restores native output within 0.28 seconds");
        blend.beginEntry();check(blend.weight()==first,"reattachment starts a fresh acknowledged entry");
    }
    exitClockBoundaries();
    for(int fps:{40,60,120})for(unsigned interval:{2u,3u,4u})if(float(interval)/fps<=.050001f)
        for(float seconds:{PoseHandoff::nativeExitSeconds,PoseBlendEnvelope::fallExitSeconds,PoseBlendEnvelope::exitSeconds})
            check(exitClockCadence(fps,interval,1,seconds)==exitClockCadence(fps,interval,7,seconds),"duplicate scene callbacks do not change any configured exit duration");
    renderedPoseHandoffs();
    std::cout<<"PASS: observed preflight, missing output timeout, brief gap recovery, replacement invalidation, consumed pose handoffs\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
