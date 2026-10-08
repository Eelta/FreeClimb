#include "TraversalSuspension.h"
#include "TraversalStealth.h"
#include "GamepadInput.h"
#include "MenuInterruption.h"
#include "PoseFrameClock.h"
#include "PoseHealth.h"
#include "PoseBlendEnvelope.h"
#include <iostream>
#include <stdexcept>

using namespace fc;
static unsigned checks{};
static void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
struct Wall final:World {
    std::optional<Hit> ray(Vec from,Vec to)override {
        if(from.y>=0||to.y<0)return {};
        const auto delta=to-from;return Hit{from+delta*(-from.y/delta.y),{0,-1,0},true};
    }
};
struct Actor {
    struct State {struct {bool sneaking{};} actorState1;} state;
    bool graphSneaking{},dead{};
    State* AsActorState(){return &state;}
    bool IsDead()const{return dead;}
    bool GetGraphVariableBool(const char*,bool& value){value=graphSneaking;return true;}
    bool SetGraphVariableBool(const char*,bool value){check(!value,"traversal never requests native crouch animation");graphSneaking=value;return true;}
};
struct Session {
    Wall world;
    Traversal traversal;
    TraversalSuspension suspension;
    Actor actor;TraversalStealth stealth;
    KeyboardReleaseGate keyboardGate;
    InputState keyboard;
    InputBindings bindings;
    GamepadState gamepad;
    GamepadSettings gamepadSettings;
    WallRunEntryGate runGate;
    PoseFrameClock clock;
    PoseHealth health;
    float stamina=150,poseSeconds{};
    std::uint32_t milliseconds{},callbacks{1};
    std::array<std::uint8_t,256> physical{};
    bool controller{};
    explicit Session(bool pad=false):controller(pad) {
        traversal.cfg.contextActions=false;traversal.cfg.automaticClimbActions=false;traversal.cfg.contextualMantleEnabled=false;
        traversal.cfg.approachSeconds=.3f;traversal.cfg.hangDrain=7.f;
        check(traversal.attach(world,{0,-40,100},{0,1,0},stamina),"fixture attaches to actual plane");
        check(stealth.acquire(&actor,false),"actual attached session acquires stealth state");
        health.sample(callbacks,.02f);clock.sample(milliseconds,.02f,.02f,false);
    }
    void suspend() {
        if(!suspension.suspend())return;
        keyboard.reset();keyboardGate.suspend();gamepad.blockUntilButtonsReleased();runGate.reset();clock.clear();
    }
    Result frame(float dt,bool menu=false,bool frozen=false,unsigned timerPause=0,bool freshCallback=true) {
        milliseconds+=20;
        if(traversal.active())check(stealth.update(&actor,traversal.wallRunning()),"owned stealth remains valid across update and suspension");
        const bool blocked=menu||gameTimeSuspended(dt,frozen,timerPause);
        if(blocked)suspend();
        if(blocked) {
            check(clock.sample(milliseconds,.02f,.02f,true)==0,"held scene output consumes no animation time");
            return {};
        }
        if(suspension.resume()) {
            keyboard.reset();keyboardGate.suspend();gamepad.blockUntilButtonsReleased();runGate.reset();clock.clear();
            health.resume(callbacks);return {};
        }
        if(keyboardGate.sample(physical,bindings))keyboard.reset();
        else for(unsigned scan=0;scan<physical.size();++scan)keyboard.set(scan,(physical[scan]&0x80)!=0);
        gamepad.resumeIfButtonsReleased();
        if(freshCallback)++callbacks;
        health.sample(callbacks,dt);
        poseSeconds+=clock.sample(milliseconds,dt,.02f,false);
        const auto k=controller?gamepad.keys(gamepadSettings.bindings):mapKeys(keyboard,bindings);
        const auto result=traversal.update(world,wallInput(runGate.filter(k),k.space,true,false,traversal.wallRunning()),health.ready()?dt:0.f,stamina);
        stamina-=result.staminaCost;
        if(result.released)stealth.release(&actor);else stealth.update(&actor,traversal.wallRunning());
        return result;
    }
    void settle() {
        for(unsigned frameIndex=0;frameIndex<80&&traversal.state==State::approach;++frameIndex)frame(.02f);
        check(traversal.state==State::wall,"entry completes normally before pause test");
    }
    void press(std::initializer_list<unsigned> scans) {physical.fill(0);for(auto scan:scans)physical[scan]=0x80;}
    void hardReset() {stealth.release(&actor);traversal.reset();suspension.clear();clock.clear();health.reset();keyboard.reset();gamepad.reset();}
};
static void holdActualTraversal() {
    for(bool pad:{false,true})for(unsigned cause=0;cause<4;++cause)for(unsigned mode=0;mode<4;++mode) {
        Session session(pad);
        session.frame(.02f);
        if(mode>0)session.settle();
        if(mode==2) {
            session.press({0x11,0x2a});session.gamepad.sampleXInput(0x0100,0,0,0,32767,session.gamepadSettings);
            session.frame(.02f);check(session.traversal.wallRunning(),"fixture begins actual wall running");
        }
        if(mode==3) {
            session.press({0x1e,0x1f,0x20,0x39});session.gamepad.sampleXInput(0x2000,0,0,0,0,session.gamepadSettings);
            check(session.frame(.02f).motion==Motion::drop&&session.traversal.state==State::action,"fixture begins a real timed action");
        }
        const auto position=session.traversal.position;const float phase=session.traversal.progress();
        const float stamina=session.stamina,poseSeconds=session.poseSeconds;
        const auto state=session.traversal.state;const bool sneaking=session.actor.state.actorState1.sneaking;
        session.press({0x1e,0x1f,0x20,0x39});
        session.gamepad.sampleXInput(0x2000,0,0,0,-32768,session.gamepadSettings);
        for(unsigned frame=0;frame<400;++frame)session.frame(cause==3?0.f:.02f,cause==0,cause==1,cause==2?3:0);
        check(session.traversal.active()&&session.traversal.state==state,"pause retains an in-progress entry");
        check((session.traversal.position-position).length()==0&&session.traversal.progress()==phase,"pause preserves exact position and action phase for eight seconds");
        check(session.stamina==stamina&&session.poseSeconds==poseSeconds&&!session.health.failed(),"pause preserves stamina and pose watchdog");
        check(session.actor.state.actorState1.sneaking==sneaking&&sneaking!=session.traversal.wallRunning(),"pause keeps climbing stealth or wall-run non-stealth without native crouch events");
        session.frame(.5f,false,false,0,false);
        check(session.traversal.progress()==phase&&(session.traversal.position-position).length()==0,"resume frame discards a long update instead of catching up");
        session.frame(.02f,false,false,0,false);
        check(session.traversal.progress()==phase&&!session.health.ready(),"an old callback cannot authorize movement after pause");
        session.press({});session.gamepad.sampleXInput(0,0,0,0,0,session.gamepadSettings);
        session.frame(.02f);
        check(session.health.ready()&&session.traversal.active(),"fresh pose callback resumes the same retained traversal");
        if(mode==0||mode==3)check(session.traversal.progress()>phase,"fresh callbacks advance the original timed action");
    }
}
static void menuOverlap() {
    Session session;session.settle();const auto position=session.traversal.position;
    bool console=true,inventory=true;
    const auto blocked=[&]{return (console&&menuBlocksTraversal("Console"))||(inventory&&menuBlocksTraversal("InventoryMenu",{false,true,false}));};
    session.frame(.02f,blocked());console=false;
    for(unsigned frame=0;frame<100;++frame)session.frame(.02f,blocked());
    check(session.suspension.active()&&(session.traversal.position-position).length()==0,"closing one overlapping menu does not resume traversal");
    inventory=false;session.frame(.02f,blocked());check(!session.suspension.active()&&session.traversal.active(),"closing the last menu resumes retained ownership without reacquiring");
}
static void staleDepartures() {
    for(bool pad:{false,true}) {
        Session session(pad);session.settle();session.frame(.02f,true);
        session.press({0x1e,0x1f,0x20,0x39});
        session.gamepad.sampleXInput(0x2000,0,0,0,-32768,session.gamepadSettings);
        session.frame(.02f);
        for(unsigned frame=0;frame<25;++frame)session.frame(.02f);
        check(session.traversal.state==State::wall&&session.traversal.active(),"held in-place exit chord or controller drop cannot detach after menu");
        session.press({});session.gamepad.sampleXInput(0,0,0,0,0,session.gamepadSettings);session.frame(.02f);
        check(!session.keyboardGate.waiting()&&!session.gamepad.waitingForButtonsRelease(),"real releases rearm both input devices");
        session.press({0x1e,0x1f,0x20,0x39});session.gamepad.sampleXInput(0x2000,0,0,0,0,session.gamepadSettings);
        const auto exit=session.frame(.02f);
        check(exit.motion==Motion::drop&&session.traversal.state==State::action,"a new deliberate drop still starts normally");
        session.press({});session.gamepad.sampleXInput(0,0,0,0,0,session.gamepadSettings);
        for(unsigned frame=0;frame<20&&session.traversal.active();++frame)session.frame(.02f);
        check(!session.traversal.active(),"deliberate exit completes after rearming");
        check(!session.actor.state.actorState1.sneaking&&!session.stealth.active(),"actual completed drop cancels stealth before native control resumes");
    }
    KeyboardReleaseGate gate;InputBindings bindings;bindings.hop=*parseKeyChord("RShift+F");
    std::array<std::uint8_t,256> raw{};gate.suspend();raw[0x36]=raw[0x21]=0x80;
    check(gate.sample(raw,bindings),"custom right modifier chord remains blocked");raw[0x21]=0;
    check(gate.sample(raw,bindings),"partial custom chord release cannot rearm");raw[0x36]=0;raw[0x29]=0x80;
    check(!gate.sample(raw,bindings),"unrelated console toggle key does not prevent rearming");
}
static void saveAndInvalidation() {
    Session session;session.settle();const auto position=session.traversal.position;
    session.suspend();session.suspend();
    check(session.traversal.active()&&(session.traversal.position-position).length()==0,"repeated save notifications hold rather than release the wall");
    session.frame(.02f);check(session.traversal.active(),"save completion resumes the existing traversal");
    session.hardReset();check(!session.traversal.active()&&!session.suspension.active()&&!session.health.ready()&&!session.actor.state.actorState1.sneaking,"actual load/revert clears traversal and transient ownership state");
    for(const char* cause:{"death","missing 3D","replaced rig","teleport"}) {
        Session invalid;invalid.settle();invalid.frame(.02f,true);invalid.hardReset();
        check(!invalid.traversal.active()&&!invalid.suspension.active(),cause);
    }
}
static void frozenExitClock() {
    PoseFrameClock clock;PoseBlendEnvelope envelope;envelope.beginExit(0);std::uint32_t callbacks=0;
    clock.sample(0,.02f,.02f,false);++callbacks;envelope.advanceExit(callbacks,clock.sample(20,.02f,.02f,false));
    const float before=envelope.elapsedExitSeconds();
    for(std::uint32_t frame=40;frame<5000;frame+=20) {
        const bool frozen=gameTimeSuspended(.02f,true,0);
        envelope.advanceExit(++callbacks,clock.sample(frame,.02f,.02f,frozen));
        check(envelope.elapsedExitSeconds()==before,"exit handoff clock cannot advance during non-menu freezing");
    }
    clock.clear();envelope.advanceExit(++callbacks,clock.sample(9000,.02f,.02f,false));
    check(envelope.elapsedExitSeconds()==before,"resume clock origin discards hidden frozen time");
    envelope.advanceExit(++callbacks,clock.sample(9020,.02f,.02f,false));
    check(envelope.elapsedExitSeconds()>before,"exit handoff continues from the retained phase");
    const bool enabled=false;
    for(std::uint32_t frame=9040;frame<10000;frame+=20) {
        const auto dt=clock.sample(frame,.02f,.02f,false);
        envelope.advanceExit(++callbacks,dt);envelope.acknowledgeExit(envelope.weight()<=0);
        if(enabled)throw std::runtime_error("disabled fixture unexpectedly enabled");
    }
    check(envelope.exitComplete(callbacks),"disabling traversal still permits resumed exit fade to finish");
}
int main()try {
    holdActualTraversal();menuOverlap();staleDepartures();saveAndInvalidation();frozenExitClock();
    std::cout<<"Traversal suspension checks passed: "<<checks<<'\n';return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}

