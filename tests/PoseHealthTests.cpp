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
    {
        TopRecoveryGate gate;
        gate.resolve(true);
        check(!gate.ready()&&gate.weight(Motion::contextMantle,1)==0,"an unsolicited endpoint result cannot enable native recovery");
        check(!gate.request(State::wall,1,.8f)&&!gate.request(State::mantle,.71f,.8f),"ordinary climbing and the grip phase cannot request a native standing endpoint");
        check(gate.request(State::mantle,.73f,.8f),"late top-out requests the endpoint once");
        gate.resolve(false);
        check(!gate.ready()&&gate.weight(Motion::contextMantle,1)==0,"a rejected standing event retains the authored top-out instead of blending to an unprepared native pose");
        check(!gate.request(State::mantle,.8f,.8f)&&!gate.request(State::mantle,1,.8f),"an unrecognized graph event is not retried every frame");
        gate.clear();
        check(gate.request(State::mantle,.73f,.8f),"a later climb gets its own endpoint attempt");
        gate.resolve(true);
        check(gate.ready()&&gate.weight(Motion::contextMantle,.85f)>0&&gate.weight(Motion::contextMantle,.85f)<1,
            "an accepted standing endpoint enables gradual native recovery");
        check(gate.weight(Motion::contextMantle,1)==1&&gate.weight(Motion::up,1)==0&&gate.weight(Motion(6),1)==0&&gate.weight(Motion(7),1)==0,
            "accepted recovery reaches native only for top-out motions, never for ordinary climbing");
        check(gate.weight(Motion::contextMantle,.72f)==0&&gate.weight(Motion::contextMantle,.85f)>0&&
            gate.weight(Motion::contextMantle,1)==1&&gate.weight(Motion::contextHang,1)==0&&
            gate.weight(Motion::contextHopLeft,1)==0&&gate.weight(Motion::contextHopRight,1)==0,
            "Threepeat mantle recovers the live native endpoint; hanging and leaps never recover to standing");
        check(gate.weight(Motion::contextMantle,1.01f)==1&&gate.weight(Motion::contextMantle,.72f)==0,
            "native recovery clamps both endpoints exactly even when progress slightly exceeds completion");
        for(int i=0;i<=1000;++i) {
            const float weight=gate.weight(Motion::contextMantle,.72f+.28f*float(i)/1000.f);
            check(weight>=0&&weight<=1,"quintic rounding cannot extrapolate beyond the native endpoint");
        }
        check(!gate.request(State::mantle,1,.8f),"accepted top-out does not issue a second standing reset");
    }
    for(float duration:{.5f,.817f,1.867f,3.f,8.f}) {
        TopRecoveryGate gate;
        const float start=topRecoveryBegin(duration);
        check(start>=.72f&&(1-start)*duration<=.24001f,"native standing cannot replace the authored finish more than .24 seconds before release");
        check(!gate.request(State::mantle,start-.001f,duration),"no native standing request before the timed transition");
        check(gate.request(State::mantle,start,duration),"native request begins at the same boundary as recovery");
        gate.resolve(true);
        check(gate.weight(Motion::contextMantle,start)==0,"the native endpoint starts with no pose jump");
        float previous=0;
        for(int i=0;i<=1000;++i) {
            const float phase=start+(1-start)*float(i)/1000.f;
            const float weight=gate.weight(Motion::contextMantle,phase);
            check(weight>=previous-1e-6f&&weight<=1,"timed native transition remains monotonic and bounded");previous=weight;
        }
        check(gate.weight(Motion::contextMantle,1)==1,"timed native transition completes without holding the authored last pose");
        check(!gate.request(State::mantle,1,duration),"timed native transition never resets the graph twice");
    }
    for(float duration:{0.f,-1.f,NAN,INFINITY}) {
        TopRecoveryGate gate;
        check(!gate.request(State::mantle,1,duration)&&topRecovery(Motion::contextMantle,1,duration)==0,"invalid duration cannot enable a native endpoint");
    }
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
        check(first>0&&first<=.02f,"first pose has a small nonzero weight so its callback cannot deadlock");
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
    renderedPoseHandoffs();
    std::cout<<"PASS: observed preflight, missing output timeout, brief gap recovery, replacement invalidation, consumed pose handoffs\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
