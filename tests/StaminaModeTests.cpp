#include "CornerTestWorld.h"
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
using namespace fc;
using fc_test::CornerWorld;
static unsigned checks{},chains{};
static void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
static CornerWorld wall(float top=20000) {
    CornerWorld w;w.boxes={{{-20000,0,-10000},{20000,1000,top}}};return w;
}
static Settings settings(bool enabled=true) {
    auto cfg=fc_test::settings();cfg.staminaEnabled=enabled;cfg.approachSeconds=.08f;return cfg;
}
static Settings capturedSettings(bool enabled=true) {
    auto cfg=settings(enabled);
    cfg.threepeatAnimations=cfg.surfaceActionVariants=cfg.automaticClimbActions=true;
    cfg.autoActionMinSeconds=.8f;cfg.autoActionMaxSeconds=1.25f;
    cfg.threepeatHangHeight=138.12f;cfg.threepeatHandHalfWidth=25.14f;cfg.threepeatHangForward=30.f;
    cfg.threepeatHopDistance={165.f,170.f};cfg.threepeatHopSeconds={1.3f,1.3f};
    return cfg;
}
static Traversal attach(CornerWorld& w,Settings cfg,Vec point={0,-37,100}) {
    Traversal t;t.cfg=cfg;
    check(t.attach(w,w.global(point),w.direction({0,1,0}),cfg.staminaEnabled?1000000.f:0.f,60),"checked real-wall attachment");
    return t;
}
static void equivalent(const Traversal& free,const Traversal& charged,const Result& a,const Result& b,unsigned raysA,unsigned raysB) {
    check(a.staminaCost==0,"disabled stamina always reports exactly zero cost");
    check(std::isfinite(b.staminaCost)&&b.staminaCost>=0,"enabled stamina reports a valid cost");
    check(a.motion==b.motion&&a.released==b.released&&a.completed==b.completed&&std::string(a.reason)==b.reason,"stamina toggle retains output motion and outcome");
    check(free.state==charged.state&&free.active()==charged.active(),"stamina toggle retains state");
    check((free.position-charged.position).length()<.00001f&&(free.normal-charged.normal).length()<.00001f&&
        (free.surfaceNormal-charged.surfaceNormal).length()<.00001f,"stamina toggle retains actual body path and facing");
    check((a.releaseVelocity-b.releaseVelocity).length()<.00001f&&std::abs(free.actionProgress()-charged.actionProgress())<.00001f,
        "stamina toggle retains action phase and physical exit velocity");
    check(free.automaticActionCount()==charged.automaticActionCount()&&free.obstacleJumpCount()==charged.obstacleJumpCount(),"stamina toggle retains checked action counts");
    check(raysA==raysB,"disabled stamina retains the entire same geometry query path");
}
static float solidDistance(const CornerWorld& w,const Traversal& t) {
    const Vec point=w.local(t.position);float distance=100000;
    for(const auto& box:w.boxes) {
        const float dx=std::max({box.low.x-point.x,0.f,point.x-box.high.x});
        const float dy=std::max({box.low.y-point.y,0.f,point.y-box.high.y});
        const float dz=std::max({box.low.z-(point.z+t.cfg.height-t.cfg.radius),0.f,point.z+t.cfg.radius-box.high.z});
        distance=std::min(distance,std::sqrt(dx*dx+dy*dy+dz*dz));
    }
    return distance;
}
struct Paired {
    CornerWorld a,b;
    Traversal free,charged;
    float spent{};
    explicit Paired(CornerWorld geometry,Settings cfg=settings(),Vec point={0,-37,100}):a(geometry),b(geometry) {
        cfg.staminaEnabled=false;free=attach(a,cfg,point);
        cfg.staminaEnabled=true;charged=attach(b,cfg,point);
    }
    Result tick(Input input,float dt) {
        a.rays=b.rays=0;
        auto result=free.update(a,input,dt,0);const auto reference=charged.update(b,input,dt,1000000.f);
        equivalent(free,charged,result,reference,a.rays,b.rays);spent+=reference.staminaCost;
        check(solidDistance(a,free)>=free.cfg.radius-.06f,"disabled stamina keeps the full collision radius");
        return result;
    }
    void settled(float dt) {
        for(unsigned frame=0;frame<60&&free.state==State::approach;++frame)tick({},dt);
        check(free.state==State::wall,"entry completes before fixture action");
    }
};
static void attachmentAndExhaustion(int fps) {
    auto geometry=wall();Traversal denied;denied.cfg=settings();
    check(!denied.attach(geometry,{0,-37,100},{0,1,0},0)&&denied.lastFailure==AttachFailure::stamina,"enabled zero stamina refuses attachment");
    auto free=attach(geometry,settings(false));
    check(free.active(),"disabled zero stamina acquires supported wall");
    auto charged=attach(geometry,settings());const float dt=1.f/fps;
    while(charged.state==State::approach)charged.update(geometry,{},dt,1000);
    const auto restAt=charged.position;const auto restNormal=charged.normal;const auto restPhase=charged.progress();
    for(int frame=0;frame<fps*2;++frame) {
        const auto exhausted=charged.update(geometry,{0,1,false,false,true,false,true},dt,0);
        check(!exhausted.released&&charged.active()&&exhausted.staminaCost==0&&
            (charged.position-restAt).length()==0&&(charged.normal-restNormal).length()==0&&charged.progress()==restPhase,
            "enabled exhaustion holds the checked wall pose without movement, running or new hop cost");
    }
    const float resumeAt=std::max(1.f,charged.cfg.startStamina);
    for(int frame=0;frame<fps;++frame) {
        const auto recovering=charged.update(geometry,{0,1},dt,resumeAt-.01f);
        check(!recovering.released&&recovering.staminaCost==0&&(charged.position-restAt).length()==0,
            "partial stamina recovery cannot oscillate between rest and movement below the resume threshold");
    }
    for(int frame=0;frame<fps&&charged.position.z<=restAt.z;++frame)charged.update(geometry,{0,1},dt,resumeAt);
    check(charged.active()&&charged.position.z>restAt.z,"rested climber resumes checked movement at the recovery threshold");
    for(bool back:{false,true}) {
        auto manual=attach(geometry,settings());while(manual.state==State::approach)manual.update(geometry,{},dt,1000);
        manual.update(geometry,{0,1},dt,0);
        Input release;release.release=true;release.backDrop=back;
        auto result=manual.update(geometry,release,dt,0);
        for(int frame=0;frame<fps*3&&manual.active();++frame)result=manual.update(geometry,{},dt,0);
        check(result.released&&!manual.active()&&result.staminaCost==0,
            "manual drop and back-push remain available throughout zero-stamina rest");
    }
    auto toggled=attach(geometry,settings());while(toggled.state==State::approach)toggled.update(geometry,{},dt,1000);
    toggled.update(geometry,{0,1},dt,0);const auto toggleAt=toggled.position;toggled.cfg.staminaEnabled=false;
    const auto unchecked=toggled.update(geometry,{0,1},dt,0);
    check(!unchecked.released&&unchecked.staminaCost==0&&toggled.position.z>toggleAt.z,
        "disabling stamina while resting restores ordinary checked movement without requiring regeneration");
    for(bool enabled:{false,true}) {
        Traversal absent;absent.cfg=settings(enabled);CornerWorld empty;
        check(!absent.attach(empty,{0,-37,100},{0,1,0},enabled?1000000.f:0.f)&&absent.lastFailure!=AttachFailure::stamina,"no-stamina mode cannot create a missing wall");
        auto blocked=wall();blocked.boxes.push_back({{-500,-8,115},{500,-2,145},false});
        Traversal t;t.cfg=settings(enabled);
        check(!t.attach(blocked,{0,-37,100},{0,1,0},enabled?1000000.f:0.f),"no-stamina mode cannot acquire through solid body obstruction");
    }
    ++chains;
}
static void movementAndManual(int fps) {
    const float dt=1.f/fps;
    for(const Input input:{Input{0,1},Input{1,0},Input{-1,0},Input{0,-1},Input{1,1},Input{0,1,false,false,false,false,true}}) {
        Paired pair(wall());pair.settled(dt);const Vec start=pair.free.position;
        for(int frame=0;frame<fps;++frame) {
            const auto out=pair.tick(input,dt);check(!out.released,"zero stamina ordinary movement stays supported");
        }
        check((pair.free.position-start).length()>40&&pair.spent>0,"enabled movement costs stamina while disabled still moves");
        const auto expected=pair.free.cfg.drain*(input.run?2.f:1.f);
        check(std::abs(pair.spent-expected)<.002f,"ordinary climb versus wall run retain one/two-times drain");
        ++chains;
    }
    for(const Input input:{Input{0,1,false,false,true},Input{-1,0,false,false,true},Input{1,0,false,false,true}}) {
        Paired pair(wall());pair.settled(dt);auto out=pair.tick(input,dt);
        check(hopMotion(out.motion)&&pair.free.state==State::action&&pair.spent==15,"zero stamina begins the same checked manual hop");
        for(int frame=0;frame<fps*2&&pair.free.state==State::action;++frame)out=pair.tick({},dt);
        check(pair.free.active()&&pair.free.state!=State::action,"zero stamina hop lands without mandatory drop");
        ++chains;
    }
}
static void obstacleAndTop(int fps) {
    const float dt=1.f/fps;
    for(const Input input:{Input{0,1,false,false,false,false,true},Input{1,0,false,false,false,false,true},Input{-1,0,false,false,false,false,true}}) {
        auto geometry=wall();
        if(input.x==0)geometry.boxes.push_back({{-20000,-30,600},{20000,20,612}});
        else if(input.x>0)geometry.boxes.push_back({{500,-30,-10000},{512,20,20000}});
        else geometry.boxes.push_back({{-512,-30,-10000},{-500,20,20000}});
        auto cfg=settings();cfg.wallRunObstacleJumps=true;cfg.runSpeed=379.5f;
        Paired pair(geometry,cfg);pair.settled(dt);bool committed=false,landed=false;
        for(int frame=0;frame<fps*4;++frame) {
            const auto out=pair.tick(input,dt);check(!out.released,"zero stamina automatic obstacle route remains supported");
            committed|=pair.free.obstacleJumpCount()>0;
            if(committed&&pair.free.state!=State::action){landed=true;break;}
        }
        check(committed&&landed&&pair.spent>30,"zero stamina crosses a real projecting obstacle with same automatic jump");
        ++chains;
    }
    Paired top(wall(330));top.settled(dt);bool mantle=false,complete=false;
    for(int frame=0;frame<fps*8&&!complete;++frame) {
        const auto result=top.tick({0,1,false,true},dt);mantle|=top.free.state==State::mantle;complete=result.completed;
    }
    check(mantle&&complete&&!top.free.active()&&top.free.position.z>=330&&top.spent>=top.free.cfg.mantleCost,
        "disabled zero stamina finishes a genuine checked top-out");
    ++chains;
}
static void blockedAndLostSupport(int fps) {
    const float dt=1.f/fps;
    auto geometry=wall();geometry.boxes.push_back({{-20000,-1000,500},{20000,1000,510},false});
    Paired blocked(geometry);blocked.settled(dt);const Vec start=blocked.free.position;
    for(int frame=0;frame<fps*4;++frame) {
        const auto out=blocked.tick({0,1,false,true,false},dt);
        check(!out.completed&&!out.released&&blocked.free.position.z+blocked.free.cfg.height<=500.06f,
            "disabled stamina cannot bypass a real overhead solid");
    }
    check(blocked.free.position.z>start.z&&blocked.free.position.z<400,"movement stops below the overhang without a phantom success");
    Paired lost(wall());lost.settled(dt);const auto heldAt=lost.free.position;
    const auto original=lost.a.boxes;lost.a.boxes.clear();lost.b.boxes.clear();
    for(int frame=0;frame<fps*2;++frame) {
        const auto held=lost.tick({0,1},dt);
        check(!held.released&&lost.free.active()&&(lost.free.position-heldAt).length()==0&&held.staminaCost==0,
            "both stamina modes hold missing geometry without inventing a movable support");
    }
    lost.a.boxes=original;lost.b.boxes=original;
    for(int frame=0;frame<fps&&lost.free.position.z<=heldAt.z;++frame)lost.tick({0,1},dt);
    check(lost.free.active()&&lost.free.position.z>heldAt.z,"both stamina modes resume when actual support returns");
    chains+=2;
}
static void actionRest(int fps) {
    const float dt=1.f/fps;
    for(bool mantle:{false,true}) {
        auto geometry=wall(mantle?330.f:20000.f);auto t=attach(geometry,settings());
        while(t.state==State::approach)t.update(geometry,{},dt,1000);
        if(mantle) {
            for(int frame=0;frame<fps*6&&t.state!=State::mantle;++frame)t.update(geometry,{0,1,false,true},dt,1000);
            check(t.state==State::mantle,"rest fixture reaches a checked mantle");
        } else {
            const auto started=t.update(geometry,{0,1,false,false,true},dt,1000);
            check(t.state==State::action&&hopMotion(started.motion),"rest fixture reaches a checked hop");
            for(int frame=0;frame<3;++frame)t.update(geometry,{},dt,1000);
        }
        const auto state=t.state;const auto position=t.position;const float phase=t.progress();
        for(int frame=0;frame<fps;++frame) {
            const auto held=t.update(geometry,{0,1,false,true},dt,0);
            check(!held.released&&!held.completed&&held.staminaCost==0&&t.state==state&&
                (t.position-position).length()==0&&t.progress()==phase,
                "exhaustion suspends an ongoing hop or mantle without restarting its source phase");
        }
        bool completed=false;
        for(int frame=0;frame<fps*4&&t.state==state;++frame) {
            const auto resumed=t.update(geometry,{},dt,std::max(1.f,t.cfg.startStamina));completed|=resumed.completed;
            check(!resumed.released||resumed.completed,"regeneration resumes the existing action without an unexpected drop");
        }
        check(mantle?completed&&!t.active():t.active()&&t.state==State::wall,
            "a suspended hop lands and a suspended mantle completes after regeneration");
        ++chains;
    }
}
static float randomValue(std::uint32_t& state) {
    state^=state<<13;state^=state>>17;state^=state<<5;return float(state>>8)*(1.f/16777216.f);
}
static void automaticSides(int fps,int side) {
    const float dt=1.f/fps;const Input input{float(side),0};
    auto cfg=capturedSettings();cfg.automaticSideWeights[side<0?0:1]=0;
    Paired disabled(wall(),cfg);disabled.settled(dt);
    for(int frame=0;frame<fps*5;++frame) {
        const auto out=disabled.tick(input,dt);
        check(!threepeatHop(out.motion)&&!disabled.free.preparingEdge()&&disabled.free.automaticActionCount()==0,
            "zero side weight suppresses new automatic side preparation and commits");
    }
    check(disabled.free.automaticOpportunityCount()>0&&disabled.free.automaticAttemptCount()>0,"zero weights are exercised at actual scheduling opportunities");
    auto manualGeometry=wall(100+cfg.threepeatHangHeight);Paired manual(manualGeometry,cfg);manual.settled(dt);
    Input press=input;press.hop=true;auto out=manual.tick(press,dt);bool manualCaptured=threepeatHop(out.motion);
    for(int frame=0;frame<fps*2&&!manualCaptured;++frame)manualCaptured|=threepeatHop(manual.tick(input,dt).motion);
    check(manualCaptured&&manual.free.automaticActionCount()==0,"zero automatic weight retains deliberate Space captured side hop");
    cfg.automaticSideWeights={1,1};Paired normal(wall(),cfg);normal.settled(dt);
    std::uint32_t rng=0x6D2B79F5u;unsigned intervals=0;bool awaiting=true;unsigned commits=0;
    for(int frame=0;frame<fps*9;++frame) {
        const auto result=normal.tick(input,dt);
        if(awaiting&&!normal.free.preparingEdge()&&normal.free.state!=State::action&&normal.free.automaticActionPendingSeconds()<cfg.autoActionMaxSeconds&&
            std::abs(normal.free.automaticActionPendingSeconds()-cfg.autoActionMinSeconds)>.00001f) {
            const float expected=cfg.autoActionMinSeconds+(cfg.autoActionMaxSeconds-cfg.autoActionMinSeconds)*randomValue(rng)-dt;
            check(std::abs(normal.free.automaticActionPendingSeconds()-expected)<.00001f,"default one side weight does not consume extra random draws");
            ++intervals;awaiting=false;
        }
        if(normal.free.automaticActionCount()!=commits){commits=normal.free.automaticActionCount();awaiting=true;}
        if(threepeatHop(result.motion))check(result.motion==(side<0?Motion::contextHopLeft:Motion::contextHopRight),"weighted action retains requested side");
    }
    check(commits>=2&&intervals>=2,"default side weights repeatedly produce new checked actions");
    cfg.automaticSideWeights[side<0?0:1]=0;cfg.automaticSideWeights[side<0?1:0]=1;
    Paired opposite(wall(),cfg);opposite.settled(dt);const Input other{float(-side),0};
    for(int frame=0;frame<fps*4;++frame)opposite.tick(other,dt);
    check(opposite.free.automaticActionCount()>0,"zeroing one side does not suppress the opposite side");
    chains+=4;
}
static void legacyCannotBypassWeight(int fps,int side) {
    const float dt=1.f/fps;auto cfg=capturedSettings();cfg.automaticSideWeights={0,0};cfg.legacyAutomaticHops=true;
    Paired pair(wall(100+cfg.threepeatHangHeight),cfg);pair.settled(dt);bool ordinary=false;
    for(int frame=0;frame<fps*5;++frame) {
        const auto result=pair.tick({float(side),0},dt);
        check(!threepeatHop(result.motion)&&result.motion!=Motion::contextHang,"legacy opt-in cannot bypass disabled new automatic side weight");
        ordinary|=result.motion==(side<0?Motion::hopLeft:Motion::hopRight);
    }
    check(ordinary&&pair.free.automaticActionCount()>0,"explicit legacy opt-in still permits its ordinary checked hop");
    ++chains;
}
int main(){try {
    for(int fps:{30,60,120}) {
        attachmentAndExhaustion(fps);movementAndManual(fps);obstacleAndTop(fps);blockedAndLostSupport(fps);actionRest(fps);
        automaticSides(fps,-1);automaticSides(fps,1);
        legacyCannotBypassWeight(fps,-1);legacyCannotBypassWeight(fps,1);
    }
    std::cout<<"PASS: "<<checks<<" stamina and side-weight checks across "<<chains<<" real geometry chains\n";
}catch(const std::exception& e){std::cerr<<"FAIL stamina mode: "<<e.what()<<'\n';return 1;}}
