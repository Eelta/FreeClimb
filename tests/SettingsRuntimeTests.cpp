#include "SettingsRuntime.h"
#include "CornerTestWorld.h"
#include "TraversalSuspension.h"
#include "TraversalStealth.h"
#include <iostream>
#include <stdexcept>
#include <string_view>

using namespace fc;
static unsigned checks{};
static void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
static bool same(Vec a,Vec b){return (a-b).length()<.00001f;}
static fc_test::CornerWorld wall(){fc_test::CornerWorld w;w.boxes={{{-20000,0,-10000},{20000,1000,20000}}};return w;}
static Traversal attached(fc_test::CornerWorld& w) {
    Traversal t;t.cfg=fc_test::settings();
    check(t.attach(w,{0,-37,100},{0,1,0},1000000,60),"supported wall attaches before live edits");
    for(unsigned i=0;i<30&&t.state==State::approach;++i)t.update(w,{},1.f/60,1000000);
    check(t.state==State::wall,"entry settles before live edits");return t;
}
struct Actor {
    struct State {struct Bits {bool sneaking{};} actorState1;} state;
    unsigned graphWrites{};
    auto* AsActorState(){return &state;}
    bool IsDead()const{return false;}
    bool GetGraphVariableBool(std::string_view,bool& value){value=false;return true;}
    bool SetGraphVariableBool(std::string_view,bool){++graphWrites;return true;}
};
static void inputBoundary() {
    UserSettings active;active.climbSneakEnabled=true;auto requested=active;
    requested.bindings.entry=parseKeyChord("LShift+W").value();
    requested.gamepad.enabled=false;requested.gamepad.deadzone=.4f;requested.gamepad.triggerThreshold=.7f;
    requested.threepeatAnimations=false;requested.upSpeed=137;requested.diagnostics=true;
    requested.staminaEnabled=false;requested.automaticClimbActions=false;requested.wallRunEnabled=false;
    requested.climbSneakEnabled=false;
    check(deferredRuntimeSettings(active,requested),"input and animation edits need safe detach");
    const auto live=liveRuntimeSettings(active,requested,true);
    check(live.bindings==active.bindings&&live.gamepad==active.gamepad,"active input ownership is preserved");
    check(live.threepeatAnimations==active.threepeatAnimations,"active animation calibration is preserved");
    check(live.upSpeed==137&&live.diagnostics&&!live.staminaEnabled&&!live.automaticClimbActions&&!live.wallRunEnabled,"ordinary edits apply during traversal");
    check(!live.climbSneakEnabled,"Climbing sneak turns off immediately while input changes wait for safe detach");
    check(requested.bindings!=active.bindings&&!requested.gamepad.enabled,"pending requested inputs remain intact");
    const auto complete=liveRuntimeSettings(live,requested,false);
    check(complete.bindings==requested.bindings&&complete.gamepad==requested.gamepad&&!complete.threepeatAnimations,"safe detach activates deferred inputs and animation option");
    check(!deferredRuntimeSettings(complete,requested),"fully applied request no longer stays pending");
    auto ordinary=active;ordinary.upSpeed=124;ordinary.audioVolume=.3f;ordinary.diagnostics=true;ordinary.wallRunEnabled=false;
    check(!deferredRuntimeSettings(active,ordinary),"ordinary changes do not wait for safe detach");
    auto sneakOnly=active;sneakOnly.climbSneakEnabled=false;
    check(!deferredRuntimeSettings(active,sneakOnly)&&!liveRuntimeSettings(active,sneakOnly,true).climbSneakEnabled,
        "Changing only climbing sneak applies on the wall without a deferred request");
    check(!deferredRuntimeSettings(sneakOnly,active)&&liveRuntimeSettings(sneakOnly,active,true).climbSneakEnabled,
        "Reenabling climbing sneak also applies immediately while input ownership is retained");
}
static void preservedCalibration() {
    auto cfg=fc_test::settings();
    cfg.chest=75;cfg.grip=121;cfg.contextScale=1.11f;cfg.approachSeconds=.36f;cfg.mantleSeconds=1.33f;
    cfg.threepeatAnimations=false;cfg.threepeatHangHeight=141;cfg.threepeatHandHalfWidth=24;
    cfg.threepeatHangForward=30;cfg.threepeatHopDistance={165,171};cfg.threepeatHopSeconds={1.3f,1.4f};
    cfg.threepeatMantleSeconds=1.77f;cfg.threepeatMantlePalmHeight=132;
    cfg.threepeatMantleReplant={Vec{1,2,3},Vec{4,5,6}};
    const auto before=cfg;UserSettings edit;edit.upSpeed=153;edit.sideSpeed=126;edit.wallRunSpeed=480;
    edit.staminaEnabled=false;edit.automaticClimbActions=false;edit.diagnostics=true;edit.wallRunEnabled=false;
    applyLiveTraversalSettings(edit,cfg);
    check(cfg.climbSpeed==153&&cfg.sideSpeed==126&&cfg.runSpeed==480,"speeds apply without reacquiring controller");
    check(!cfg.staminaEnabled&&!cfg.automaticClimbActions,"stamina and automatic selection apply immediately");
    check(!cfg.wallRunEnabled&&cfg.wallRunObstacleJumps==edit.wallRunObstacleJumps,"master wall-run option applies independently of automatic obstacle jumps");
    check(cfg.gap==before.gap&&cfg.radius==before.radius&&cfg.height==before.height&&cfg.chest==before.chest&&cfg.grip==before.grip&&cfg.contextScale==before.contextScale,"actor hand and body geometry remain calibrated");
    check(!cfg.threepeatAnimations&&cfg.threepeatHangHeight==before.threepeatHangHeight&&cfg.threepeatHandHalfWidth==before.threepeatHandHalfWidth&&cfg.threepeatHangForward==before.threepeatHangForward,"unavailable animation profile cannot be reenabled by ordinary edits");
    check(cfg.threepeatHopDistance==before.threepeatHopDistance&&cfg.threepeatHopSeconds==before.threepeatHopSeconds&&cfg.threepeatMantleSeconds==before.threepeatMantleSeconds&&cfg.threepeatMantlePalmHeight==before.threepeatMantlePalmHeight,"animation-dependent durations and contacts remain unchanged");
    check(same(cfg.threepeatMantleReplant[0],before.threepeatMantleReplant[0])&&same(cfg.threepeatMantleReplant[1],before.threepeatMantleReplant[1])&&cfg.approachSeconds==before.approachSeconds&&cfg.mantleSeconds==before.mantleSeconds,"route replant and fixed ongoing timing stay unchanged");
}
static void movementAndStamina() {
    auto geometry=wall();auto slow=attached(geometry),fast=slow;
    UserSettings settings;settings.automaticClimbActions=false;settings.surfaceActionVariants=false;
    settings.staminaEnabled=false;settings.upSpeed=30;applyLiveTraversalSettings(settings,slow.cfg);
    settings.upSpeed=150;applyLiveTraversalSettings(settings,fast.cfg);
    const auto start=slow.position;
    for(unsigned frame=0;frame<30;++frame) {
        const auto a=slow.update(geometry,{0,1},1.f/60,0),b=fast.update(geometry,{0,1},1.f/60,0);
        check(!a.released&&!b.released&&slow.active()&&fast.active(),"live speed and no-stamina edits preserve wall attachment");
        check(a.staminaCost==0&&b.staminaCost==0,"live disabled stamina immediately produces zero cost");
    }
    check(fast.position.z-start.z>3*(slow.position.z-start.z),"actual wall movement responds to new speed without reset");
    settings.staminaEnabled=true;settings.movingPerSecond=17;applyLiveTraversalSettings(settings,fast.cfg);
    const auto charged=fast.update(geometry,{0,1},1.f/60,1000000);
    check(!charged.released&&charged.staminaCost>0,"stamina reenable applies on next actual traversal tick");
}
static void pausedAction() {
    auto geometry=wall();auto t=attached(geometry);
    UserSettings active;active.automaticClimbActions=false;active.surfaceActionVariants=false;active.climbSneakEnabled=true;
    applyLiveTraversalSettings(active,t.cfg);
    const auto hop=t.update(geometry,{1,0,false,false,true},1.f/60,1000000);
    check(hopMotion(hop.motion)&&t.state==State::action,"real checked manual hop begins before paused edit");
    const auto position=t.position,normal=t.normal,surface=t.surfaceNormal;
    const auto phase=t.progress(),duration=t.actionDuration();const auto state=t.state;
    TraversalSuspension suspended;check(suspended.suspend(),"menu pause enters hold");
    Actor actor;TraversalStealth stealth;check(stealth.acquire(&actor,false,active.climbSneakEnabled),"climb stealth ownership begins");
    auto requested=active;requested.upSpeed=165;requested.sideSpeed=141;
    requested.staminaEnabled=false;requested.automaticClimbActions=true;requested.diagnostics=true;
    requested.fancyJumps=false;requested.hopOut=65;
    requested.bindings.entry=parseKeyChord("LShift+W").value();requested.gamepad.deadzone=.38f;
    const auto live=liveRuntimeSettings(active,requested,true);applyLiveTraversalSettings(live,t.cfg);
    check(suspended.active()&&t.active()&&t.state==state&&same(t.position,position)&&same(t.normal,normal)&&same(t.surfaceNormal,surface),"ordinary edits during pause retain exact attached action and geometry");
    check(t.progress()==phase&&t.actionDuration()==duration,"paused live settings do not restart or retime ongoing hop");
    check(stealth.active()&&stealth.sneaking()&&actor.state.actorState1.sneaking&&!actor.graphWrites,"settings do not cancel held stealth or send animation transitions");
    check(live.bindings==active.bindings&&live.gamepad==active.gamepad,"paused edits keep current keyboard and controller ownership");
    auto applied=live;
    for(const bool enabled:{false,true}) {
        requested.climbSneakEnabled=enabled;
        applied=liveRuntimeSettings(applied,requested,true);applyLiveTraversalSettings(applied,t.cfg);
        check(stealth.update(&actor,t.wallRunning(),applied.climbSneakEnabled)&&stealth.active()&&
            stealth.sneaking()==enabled&&actor.state.actorState1.sneaking==enabled&&!actor.graphWrites,
            "Disabling and reenabling climbing sneak applies to the owned actor while suspended");
        check(suspended.active()&&t.active()&&t.state==state&&same(t.position,position)&&same(t.normal,normal)&&same(t.surfaceNormal,surface)&&
            t.progress()==phase&&t.actionDuration()==duration,
            "Paused sneak changes preserve attachment, position and the exact ongoing action phase");
        check(applied.bindings==active.bindings&&applied.gamepad==active.gamepad,
            "Paused sneak changes keep input ownership while pending keyboard and controller edits remain deferred");
    }
    check(suspended.resume(),"resume ends menu hold");
    bool landed=false;
    for(unsigned frame=0;frame<120&&t.state==State::action;++frame) {
        const auto out=t.update(geometry,{},1.f/60,0);
        check(!out.released&&out.staminaCost==0,"existing checked hop resumes with live stamina option");
        landed|=t.state!=State::action;
    }
    check(landed&&t.active(),"ongoing action reaches wall after hot edit without forced release");
}
static UserSettings wallRunSettings() {
    UserSettings settings;settings.automaticClimbActions=false;settings.surfaceActionVariants=false;
    return settings;
}
static void disabledRunModifier() {
    check(Settings{}.wallRunEnabled&&UserSettings{}.wallRunEnabled,"wall running remains enabled by default");
    for(int fps:{30,60,120})for(Input input:{Input{0,1},Input{1,0},Input{-1,1},Input{0,-1}}) {
        auto geometry=wall();auto held=attached(geometry),plain=held;
        auto settings=wallRunSettings();settings.wallRunEnabled=false;
        applyLiveTraversalSettings(settings,held.cfg);applyLiveTraversalSettings(settings,plain.cfg);
        for(int frame=0;frame<fps;++frame) {
            auto modified=input;modified.run=true;
            const auto a=held.update(geometry,modified,1.f/fps,1000000),b=plain.update(geometry,input,1.f/fps,1000000);
            check(!a.released&&!b.released&&held.active()&&!held.wallRunning(),"disabled wall running keeps all movement directions attached in climbing mode");
            check(a.motion==b.motion&&a.staminaCost==b.staminaCost&&held.state==plain.state&&same(held.position,plain.position),"disabled held modifier exactly matches ordinary climb movement and stamina");
        }
        if(input.y>=0) {
            auto modified=input;modified.run=true;modified.hop=true;input.hop=true;
            const auto a=held.update(geometry,modified,1.f/fps,1000000),b=plain.update(geometry,input,1.f/fps,1000000);
            check(hopMotion(a.motion)&&a.motion==b.motion&&held.state==State::action&&same(held.edgeTarget(),plain.edgeTarget()),"disabled modifier preserves checked ordinary climbing hops");
        }
    }
}
static void liveWallRunToggle() {
    for(int fps:{30,60,120})for(Input input:{Input{0,1,false,false,false,false,true},Input{1,0,false,false,false,false,true},Input{-1,1,false,false,false,false,true}}) {
        auto geometry=wall();auto t=attached(geometry);auto settings=wallRunSettings();applyLiveTraversalSettings(settings,t.cfg);
        for(int frame=0;frame<fps;++frame) {
            auto pressed=input;pressed.hop=true;
            const auto out=t.update(geometry,pressed,1.f/fps,1000000);
            check(!out.released&&!hopMotion(out.motion)&&t.wallRunning(),"enabled wall running continues to suppress manual hop input");
        }
        auto released=t;Actor actor;TraversalStealth stealth;
        check(stealth.acquire(&actor,t.wallRunning())&&stealth.active()&&!stealth.sneaking(),"running mode acquires and retains typed stealth ownership");
        const auto position=t.position;const auto state=t.state;const auto normal=t.normal;
        settings.wallRunEnabled=false;applyLiveTraversalSettings(settings,t.cfg);
        check(t.active()&&t.state==state&&same(t.position,position)&&same(t.normal,normal)&&stealth.active(),"live master switch preserves exact attachment and ownership before the next update");
        float firstDistance=0,lastDistance=0;
        for(int frame=0;frame<fps;++frame) {
            auto climbing=input;climbing.run=false;
            auto pressed=input;pressed.hop=frame==0;
            const auto before=t.position;
            const auto a=t.update(geometry,pressed,1.f/fps,1000000),b=released.update(geometry,climbing,1.f/fps,1000000);
            if(!frame)firstDistance=(t.position-before).length();lastDistance=(t.position-before).length();
            check(!a.released&&!hopMotion(a.motion)&&t.active()&&!t.wallRunning(),"disabling a stable run returns to climb without a manual jump or detachment");
            check(a.motion==b.motion&&a.staminaCost==b.staminaCost&&same(t.position,released.position),"master switch reuses the checked smooth modifier-release transition");
            check(stealth.update(&actor,t.wallRunning())&&stealth.active()&&stealth.sneaking()&&actor.state.actorState1.sneaking&&!actor.graphWrites,"wall-run disable changes only owned sneak state without surrendering ownership or graph events");
        }
        check(firstDistance>lastDistance*1.5f&&lastDistance>0,"residual run speed decays to climbing speed instead of stopping or snapping");
        settings.wallRunEnabled=true;applyLiveTraversalSettings(settings,t.cfg);
        for(int frame=0;frame<fps/2;++frame) {
            const auto out=t.update(geometry,input,1.f/fps,1000000);
            check(!out.released&&t.active()&&t.wallRunning()&&runMotion(out.motion),"reenabled wall running resumes on the same held modifier and wall attachment");
            check(stealth.update(&actor,t.wallRunning())&&stealth.active()&&!stealth.sneaking()&&!actor.state.actorState1.sneaking&&!actor.graphWrites,"reenabled run keeps ownership and clears only the owned sneak state");
        }
    }
}
static void liveObstacleToggle() {
    for(int fps:{30,60,120}) {
        auto geometry=wall();geometry.boxes.push_back({{-20000,-30,600},{20000,20,612}});
        auto t=attached(geometry);auto settings=wallRunSettings();applyLiveTraversalSettings(settings,t.cfg);
        Input input{0,1,false,true,false,false,true};const float dt=1.f/fps;
        for(int frame=0;frame<fps*4&&!t.obstacleJumpCount();++frame)t.update(geometry,input,dt,1000000);
        check(t.state==State::action&&t.obstacleJumpActive()&&t.obstacleJumpCount()==1,"real obstacle starts one completely checked automatic wall-run action");
        auto released=t;const auto source=t.edgeStart(),target=t.edgeTarget();const auto duration=t.actionDuration(),phase=t.actionProgress();
        settings.wallRunEnabled=false;applyLiveTraversalSettings(settings,t.cfg);
        check(t.state==State::action&&t.actionDuration()==duration&&t.actionProgress()==phase&&same(t.edgeStart(),source)&&same(t.edgeTarget(),target),"master switch never resets an ongoing checked obstacle action or its source route");
        {
            auto blocked=t;auto removed=geometry;removed.boxes[0].high.z=target.z-4;
            const auto before=blocked.position;const auto out=blocked.update(removed,input,dt,1000000);
            check(!out.released&&blocked.active()&&blocked.state==State::action&&same(before,blocked.position)&&blocked.actionProgress()==phase&&
                std::string_view(out.reason)=="wall-run jump support changed","disabled running retains a stopped action when its landing support changes");
            for(int frame=0;frame<fps;++frame) {
                const auto held=blocked.update(removed,input,dt,1000000);
                check(!held.released&&held.staminaCost==0&&same(before,blocked.position)&&blocked.actionProgress()==phase,
                    "live wall-run settings cannot bypass a blocked landing or force a drop");
            }
            for(int frame=0;frame<fps*3&&blocked.state==State::action;++frame) {
                const auto resumed=blocked.update(geometry,input,dt,1000000);
                check(!resumed.released&&geometry.clearance(blocked.position,blocked.cfg)+.03f>=blocked.cfg.radius,
                    "restoring landing geometry retains safety after the running option changes");
            }
            check(blocked.active()&&blocked.state==State::wall&&!blocked.wallRunning(),
                "restored ongoing obstacle action lands into the currently selected climb mode");
        }
        for(int frame=0;frame<fps*3&&t.state==State::action;++frame) {
            auto climbing=input;climbing.run=false;
            const auto a=t.update(geometry,input,dt,1000000),b=released.update(geometry,climbing,dt,1000000);
            check(!a.released&&t.runningAction()&&t.obstacleJumpCount()==1,"already checked automatic action completes once with its incoming run identity");
            check(a.motion==b.motion&&a.staminaCost==b.staminaCost&&t.state==released.state&&t.actionProgress()==released.actionProgress()&&same(t.position,released.position),"disabled automatic action follows the identical prechecked position and source clock as released modifier");
            check(t.actionDuration()==duration&&same(t.edgeStart(),source)&&same(t.edgeTarget(),target)&&geometry.clearance(t.position,t.cfg)+.03f>=t.cfg.radius,"ongoing automatic route and collision clearance remain unchanged after disable");
        }
        check(t.state==State::wall&&t.active(),"automatic action lands safely after live master switch");
        const auto landed=t.update(geometry,input,dt,1000000);
        check(!landed.released&&!runMotion(landed.motion)&&!t.wallRunning()&&t.obstacleJumpCount()==1,"completed obstacle action resumes normal climbing while the modifier stays held");
    }
}
int main() {
    try {inputBoundary();preservedCalibration();movementAndStamina();pausedAction();disabledRunModifier();liveWallRunToggle();liveObstacleToggle();
        std::cout<<checks<<" live settings checks passed\n";return 0;}
    catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}

