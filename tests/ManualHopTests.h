#pragma once
#include "CornerTestWorld.h"

static fc_test::CornerWorld manualHopRoom(float yaw=0) {
    fc_test::CornerWorld world;world.yaw=yaw;
    world.boxes.push_back({{-10000,0,-10000},{10000,1000,10000}});
    return world;
}
static Traversal manualHopAttached(fc_test::CornerWorld& world,float height=300) {
    Traversal t;t.cfg=fc_test::settings();t.cfg.contextActions=false;t.cfg.contextualMantleEnabled=false;
    check(t.attach(world,world.global({0,-37,height}),world.direction({0,1,0}),1000),"manual hop fixture has a real supported wall");
    return t;
}
static Motion manualHopFor(Input input) {return input.x<0?Motion::hopLeft:input.x>0?Motion::hopRight:Motion::hopUp;}
static bool retainedHop(Motion motion) {return motion==Motion::hopUp||motion==Motion::hopLeft||motion==Motion::hopRight||threepeatHop(motion);}
static void finishManualHop(fc_test::CornerWorld& world,Traversal& t,float dt=1.f/60) {
    for(unsigned frame=0;frame<300&&t.state==State::action;++frame) {
        const auto result=t.update(world,{},dt,1000);
        check(!result.released&&t.active()&&retainedHop(result.motion),"manual action retains its checked ordinary or contextual source through landing");
    }
    check(t.state==State::wall&&t.actionProgress()==1,"manual action reaches the complete supported landing");
    for(unsigned frame=0;frame<30;++frame)t.update(world,{},dt,1000);
}
static void repeatedManualHops(const Library& lib) {
    unsigned cases=0,peakQueries=0;float worstEndpoint=0;
    for(int fps:{10,20,60,120})for(float yaw:{0.f,.78539816f})for(bool fancy:{false,true})
    for(Input intent:{Input{0,1},Input{-1,0},Input{1,0}}) {
        auto world=manualHopRoom(yaw);auto t=manualHopAttached(world);SurfacePose surface;
        check(lib.configureThreepeat(t.cfg),"real default pack supplies production context calibration");
        t.cfg.fancyJumps=fancy;t.cfg.threepeatAnimations=true;t.cfg.contextActions=true;
        t.cfg.surfaceActionVariants=true;t.cfg.automaticClimbActions=true;
        const float dt=1.f/fps,effective=std::min(dt,.05f);Pose previous,previousWorld;Vec previousPosition;
        auto tick=[&](Input input) {
            const auto result=t.update(world,input,dt,1000);
            const auto pose=surface.update(lib,world,t,result.motion,dt,1),bones=lib.world(pose);
            check(int(result.motion)!=17&&int(result.motion)!=30&&int(result.motion)!=31&&int(result.motion)!=32,"live output never selects a removed catch or flip ID");
            for(const auto& transform:pose)check(transform.t.finite()&&std::isfinite(transform.q.dot(transform.q))&&std::abs(transform.q.dot(transform.q)-1)<.002f,
                "real default repeated hop output stays finite and normalized");
            check(lib.armBendValid(pose,0)&&lib.armBendValid(pose,1),"repeated hop retains actual elbow guards");
            if(!previous.empty()) {
                for(unsigned bone=0;bone<pose.size();++bone)
                    check(angleBetween(previous[bone].q,pose[bone].q)<=18.849556f*effective+.007f,"repeated hop preserves bounded bone rotations");
                const auto point=[&](Vec local,Vec root){return root+Vec{-t.normal.y,t.normal.x,0}*local.x-t.normal*local.y+Vec{0,0,local.z};};
                for(int bone:{8,11,38,39}) {
                    const float distance=(point(bones[bone].t,t.position)-point(previousWorld[bone].t,previousPosition)).length();
                    worstEndpoint=std::max(worstEndpoint,distance/effective);
                    check(distance<=600.f*effective+.6f,"repeated hop keeps the established world endpoint speed bound");
                }
            }
            previous=pose;previousWorld=bones;previousPosition=t.position;return result;
        };
        for(unsigned frame=0;frame<30;++frame)tick({});
        for(unsigned attempt=0;attempt<3;++attempt) {
            auto input=intent;input.hop=true;const auto origin=t.position;world.rays=0;
            auto result=tick(input);peakQueries=std::max(peakQueries,world.rays);
            const Motion expected=manualHopFor(intent);
            check(result.motion==expected&&t.state==State::action,"first and repeated same-direction inputs all retain the ordinary hop");
            const float seconds=t.actionDuration();float elapsed=0,cost=result.staminaCost;
            check(seconds==jumpActionSeconds(false),"the backflip departure option cannot change ordinary hop duration");
            while(t.state==State::action&&elapsed<3) {
                const auto before=t.position;const auto phase=t.actionProgress();const unsigned rays=world.rays;
                const auto paused=t.update(world,{},0,1000);
                check(paused.motion==expected&&paused.staminaCost==0&&t.actionProgress()==phase&&(t.position-before).length()==0&&world.rays==rays,
                    "paused manual hop neither advances nor repeats a checked preflight");
                result=tick({});elapsed+=effective;cost+=result.staminaCost;
                check(t.active()&&!result.released&&result.motion==expected,"repeated manual hop plays through its complete landing");
                check(world.clearance(t.position,t.cfg)>=t.cfg.radius-.01f,"actual hop root path never enters solid geometry");
                check((t.position-t.actionPathPoint(t.actionProgress())).length()<.01f,"manual hop follows exactly its preflighted root route");
            }
            check(t.state==State::wall&&t.actionProgress()==1&&elapsed>=seconds-.001f&&elapsed-seconds<effective+.001f,
                "low frame rates retain the complete repeated action clock and wall landing");
            check(std::abs(cost-15)<.001f,"each repeated manual hop charges fifteen stamina exactly once");
            check(std::abs((t.position-origin).dot(t.normal))<.01f,"hop finishes on the verified wall rather than leaving it");
            for(unsigned frame=0;frame<30;++frame)tick({});
        }
        ++cases;
    }
    check(peakQueries<5000,"manual hop request keeps its bounded geometry workload including pose queries");
    std::cout<<"repeated manual hop cases="<<cases<<" peakQueries="<<peakQueries<<" endpointSpeed="<<worstEndpoint<<'\n';
}
static void repeatedHopSafety() {
    for(Input intent:{Input{0,1},Input{-1,0},Input{1,0}})for(unsigned kind=0;kind<5;++kind) {
        auto world=manualHopRoom();auto t=manualHopAttached(world);auto input=intent;input.hop=true;
        check(t.update(world,input,1.f/60,1000).motion==manualHopFor(intent),"safety fixture first completes the ordinary input");
        finishManualHop(world,t);
        if(kind==0)t.cfg.fancyJumps=false;
        if(kind==1)world.boxes.push_back({{-10000,-160,-10000},{10000,-150,10000},false});
        if(kind==2)world.boxes.push_back({{-10000,-10000,-10000},{10000,10000,t.position.z-8},false});
        if(kind==3)world.boxes[0].low.z=t.position.z+t.cfg.height+200;
        const auto position=t.position;const auto result=t.update(world,input,1.f/60,kind==4?14.f:1000.f);
        if(kind<3) {
            check(result.motion==manualHopFor(intent)&&result.staminaCost==15.f&&t.state==State::action,
                "valid short arcs remain available without a flip or duplicate stamina charge");
            check((t.position-position).length()<.001f&&std::abs(t.actionDuration()-.52f)<.001f,"repeated hop retains its short arc and launch position");
            finishManualHop(world,t);
        } else check(t.state!=State::action&&result.staminaCost<1.f,"missing landing or stamina still prevents a repeated hop");
    }
    for(Input intent:{Input{0,1},Input{-1,0},Input{1,0}})for(bool run:{false,true}) {
        auto world=manualHopRoom();auto t=manualHopAttached(world);auto input=intent;input.hop=true;
        t.update(world,input,1.f/60,1000);finishManualHop(world,t);input.hop=run;input.run=run;
        for(unsigned frame=0;frame<180;++frame) {
            const auto result=t.update(world,input,1.f/60,1000);
            check(!hopMotion(result.motion),"ordinary movement and wall-running Space cannot create another manual hop");
        }
    }
}
static void manualHopAfterContext(const Library& lib) {
    for(Input intent:{Input{-1,0},Input{1,0}}) {
        auto world=manualHopRoom();auto t=manualHopAttached(world);
        check(lib.configureThreepeat(t.cfg),"context-to-hop fixture uses the actual default pack");
        t.cfg.threepeatAnimations=true;t.cfg.contextActions=true;t.cfg.surfaceActionVariants=true;t.cfg.automaticClimbActions=true;
        const auto context=intent.x<0?Motion::contextHopLeft:Motion::contextHopRight;bool captured=false;
        for(unsigned frame=0;frame<600;++frame) {
            const auto result=t.update(world,intent,1.f/60,1000);
            check(int(result.motion)<30||int(result.motion)>32,"automatic side captures never select retired flip IDs");
            if(result.motion==context&&t.state==State::action){captured=true;break;}
        }
        check(captured,"the original automatic contextual side action remains reachable with the real default configuration");
        finishManualHop(world,t);auto input=intent;input.hop=true;auto result=t.update(world,input,1.f/60,1000);
        for(unsigned frame=0;frame<180&&t.state!=State::action;++frame)result=t.update(world,intent,1.f/60,1000);
        check(t.state==State::action&&(result.motion==manualHopFor(intent)||result.motion==context)&&result.staminaCost==15,
            "a completed contextual hop permits the next ordinary or contextual manual catch without a flip variant");
        finishManualHop(world,t);
    }
}
