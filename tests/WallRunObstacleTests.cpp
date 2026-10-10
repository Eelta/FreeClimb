#include "CornerTestWorld.h"
#include "TraversalCapture.h"
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
using namespace fc;
using fc_test::CornerWorld;
static void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
static CornerWorld fixture(Input input,float offset=0) {
    CornerWorld w;w.boxes.push_back({{-10000,0,-10000},{10000,1000,10000}});
    if(std::abs(input.x)<.1f||input.y>.1f)w.boxes.push_back({{-10000,-30,600+offset},{10000,20,612+offset}});
    else if(input.x>0)w.boxes.push_back({{500+offset,-30,-10000},{512+offset,20,10000}});
    else w.boxes.push_back({{-512-offset,-30,-10000},{-500-offset,20,10000}});
    return w;
}
static Traversal attached(CornerWorld& world) {
    Traversal t;t.cfg=fc_test::settings();t.cfg.runSpeed=379.5f;t.cfg.wallRunObstacleJumps=true;
    check(t.attach(world,{0,-37,100},{0,1,0},1000),"wall-run test attaches to real wall");return t;
}
static Traversal startJump(CornerWorld& world,Input input,int fps) {
    auto t=attached(world);
    for(int frame=0;frame<fps*4&&!t.obstacleJumpCount();++frame)t.update(world,input,1.f/fps,1000);
    check(t.state==State::action&&t.obstacleJumpActive(),"dynamic test starts a fully preflighted real obstacle kick");return t;
}
static void positiveCases() {
    unsigned cases=0,globalPeak=0;float longest=0;
    for(int fps:{30,60,120})for(Input input:{Input{0,1,false,true,false,false,true},Input{1,0,false,false,false,false,true},Input{-1,0,false,false,false,false,true},Input{1,1,false,true,false,false,true},Input{-1,1,false,true,false,false,true}})
    for(float offset:{0.f,5.f,10.f,15.f,20.f,25.f,30.f,35.f,40.f}) {
        auto world=fixture(input,offset);auto t=attached(world);bool started=false,landed=false;unsigned peakQueries=0;
        auto tape=std::make_unique<TraversalCapture>();TraversalCapture::RecordingWorld recorded(world,*tape);
        for(int frame=0;frame<fps*4;++frame) {
            const auto before=t.position;const auto count=t.obstacleJumpCount();world.rays=0;
            tape->begin(t,input,1.f/fps,1000);const auto result=t.update(recorded,input,1.f/fps,1000);tape->finish(t,result);
            peakQueries=std::max(peakQueries,world.rays);
            if(t.obstacleJumpCount()!=count){started=true;
                check(result.staminaCost==30,"running jump pays thirty stamina once");
                check(tape->complete()&&tape->replay().matched,"entire accepted planner including new state replays exactly");
                auto decoded=std::make_unique<TraversalCapture>();std::string error;
                check(decoded->deserialize(tape->serialize(),error)&&decoded->replay().matched,"serialized full planner replay keeps duration, budget, speed and marker");
                longest=std::max(longest,t.actionDuration());
            }
            check(!result.released,"automatic running kick retains checked wall route");
            check(world.clearance(t.position,t.cfg)+.03f>=t.cfg.radius,"jump never shrinks actor through solid obstacle");
            if(started&&t.state!=State::action){landed=true;check(t.obstacleJumpActive(),"last action tick still identifies running obstacle motion");
                check(tape->replay().matched,"landing and subframe supported continuation replay exactly");
                check((t.position-before).length()>t.cfg.runSpeed/fps*.55f,"landing tick does not discard action-end subframe travel");
                const auto landedAt=t.position;const auto resumed=t.update(world,input,1.f/fps,1000);
                check(runMotion(resumed.motion)&&(t.position-landedAt).length()>t.cfg.runSpeed/fps*.9f,"first attached frame immediately resumes the same held run");break;}
        }
        if(!started||!landed)std::cerr<<"failed fixture fps="<<fps<<" direction="<<input.x<<','<<input.y<<" offset="<<offset<<" root="<<t.position.x<<','<<t.position.z<<" reason="<<t.blockedReason<<'\n';
        check(started&&landed,"actual projecting obstacle starts and completes a running kick");
        check(peakQueries<4600,"planner query cap remains bounded including ordinary traversal probes");
        globalPeak=std::max(globalPeak,peakQueries);++cases;
    }
    std::cout<<"checked obstacle cases="<<cases<<" peakFrameQueries="<<globalPeak<<" longestAction="<<longest<<'\n';
}
static void negativeCases() {
    for(int fps:{30,60,120})for(int kind=0;kind<7;++kind) {
        Input input{0,1,false,true,false,false,true};auto world=fixture(input);auto t=attached(world);float stamina=1000;
        if(kind==0)t.cfg.wallRunObstacleJumps=false;
        if(kind==1)world.boxes.resize(1);
        if(kind==2)stamina=37.9f;
        if(kind==3){world.boxes.resize(1);world.boxes[0].high.z=520;input.mantle=false;}
        if(kind==4)world.boxes[1].low.y=-1000;
        if(kind==5){input.run=false;input.mantle=false;}
        if(kind==6){world.boxes.resize(1);input.hop=true;}
        for(int frame=0;frame<fps*3&&t.active();++frame) {
            const auto out=t.update(world,input,1.f/fps,stamina);
            check(t.obstacleJumpCount()==0,"disabled, no blocker, low budget, missing landing, ceiling, climb and manual Space cannot auto-run-hop");
            if(kind==6)check(!hopMotion(out.motion),"running manual Space stays suppressed on an open wall");
        }
    }
    for(int fps:{30,60,120}) {
        Input input{0,1,false,true,false,false,true};auto world=fixture(input);world.boxes.resize(1);world.boxes[0].high.z=450;
        auto t=attached(world);bool top=false;
        for(int frame=0;frame<fps*4&&t.active();++frame){auto out=t.update(world,input,1.f/fps,1000);top|=t.state==State::mantle||out.completed;}
        check(top&&t.obstacleJumpCount()==0,"real reachable summit uses existing top-out before obstacle planning");
    }
}
static void dynamicCases() {
    for(int fps:{30,60,120}) {
        const float dt=1.f/fps;Input input{0,1,false,true,false,false,true};auto world=fixture(input);auto started=startJump(world,input,fps);
        {
            auto t=started;const auto p=t.position;const auto phase=t.actionProgress();const auto paused=t.update(world,input,0,1000);
            check(!paused.released&&paused.staminaCost==0&&(t.position-p).length()==0&&t.actionProgress()==phase,"paused output cannot advance or repeat a jump charge");
        }
        {
            auto t=started;auto removed=world;removed.boxes[0].high.z=t.edgeTarget().z-4;
            const auto p=t.position;const auto out=t.update(removed,input,dt,1000);
            check(!out.released&&t.active()&&t.state==State::action&&std::string(out.reason)=="wall-run jump support changed"&&
                (t.position-p).length()==0&&t.actionProgress()==started.actionProgress(),"removed target holds safely before stale movement is committed");
            for(int frame=0;frame<fps;++frame) {
                const auto held=t.update(removed,input,dt,1000);
                check(!held.released&&held.staminaCost==0&&(t.position-p).length()==0&&t.actionProgress()==started.actionProgress(),
                    "missing running-jump destination cannot advance or charge a frozen action");
            }
            for(int frame=0;frame<fps*3&&t.state==State::action;++frame) {
                const auto resumed=t.update(world,input,dt,1000);
                check(!resumed.released&&world.clearance(t.position,t.cfg)+.03f>=t.cfg.radius,
                    "restored running-jump destination resumes through actual collision checks");
            }
            check(t.active()&&t.state==State::wall,"running jump finishes after its real destination returns");
        }
        {
            auto t=started;Input climb=input;climb.run=false;
            while(t.state==State::action){const auto out=t.update(world,climb,dt,1000);check(!out.released&&t.runningAction(),"Shift release keeps the already checked incoming jump source");}
            const auto out=t.update(world,climb,dt,1000);check(!runMotion(out.motion)&&!t.wallRunning(),"released Shift lands into ordinary climbing");
        }
        {
            auto t=started;t.update(world,input,dt,1000);const auto p=t.position;
            auto out=t.update(world,{0,-1,true,false,false,true},dt,1000);
            check(!out.released&&out.motion==Motion::dropBack&&(t.position-p).length()<2,"S+Space changes an airborne kick to outward departure from its current position");
            for(int frame=0;frame<fps&&t.active();++frame)out=t.update(world,{},dt,1000);
            check(out.released&&out.releaseVelocity.dot(t.normal)>0,"S+Space retains checked physical outward velocity");
        }
        {
            auto t=started;const float duration=t.actionDuration();
            while(t.actionProgress()+dt/duration<.5f)t.update(world,input,dt,1000);
            const auto p=t.position;const auto phase=t.actionProgress();auto peak=t,end=t;
            peak.update(world,input,(.5f-phase)*duration,1000);end.update(world,input,dt,1000);
            const float endBack=std::min(p.y,end.position.y)-t.cfg.radius,peakBack=peak.position.y-t.cfg.radius;
            check(peakBack<endBack-.0001f,"dynamic fixture places a thin body blocker strictly between sampled endpoints");
            auto changed=world;const float rear=(endBack+peakBack)*.5f;
            changed.boxes.push_back({{-10000,rear-1,-10000},{10000,rear,10000},false});
            const auto out=t.update(changed,input,dt,1000);
            check(!out.released&&t.active()&&t.state==State::action&&(t.position-p).length()<.0001f&&t.actionProgress()==phase,
                "crossed intermediate action knots hold before a new obstacle without phase or position commit");
            for(int frame=0;frame<fps;++frame) {
                const auto held=t.update(changed,input,dt,1000);
                check(!held.released&&(t.position-p).length()<.0001f&&t.actionProgress()==phase,
                    "a thin intermediate blocker cannot be bypassed by waiting or repeated input");
            }
            for(int frame=0;frame<fps*3&&t.state==State::action;++frame) {
                const auto resumed=t.update(world,input,dt,1000);
                check(!resumed.released&&world.clearance(t.position,t.cfg)+.03f>=t.cfg.radius,
                    "removing an intermediate blocker resumes the checked running-jump curve");
            }
            check(t.active()&&t.state==State::wall,"interrupted running-jump curve still completes after collision recovery");
        }
    }
}
static void routingAndBudgets() {
    const Input up{0,1,false,true,false,false,true};
    for(int fps:{30,60,120}) {
        {
            auto world=fixture(up);world.boxes.resize(1);world.boxes.push_back({{-10000,-200,300},{10000,-75,600}});
            auto t=attached(world);bool ran=false,climbed=false,resumed=false;
            for(int frame=0;frame<fps*8;++frame) {
                const auto out=t.update(world,up,1.f/fps,1000);
                check(t.active()&&!hopMotion(out.motion)&&t.obstacleJumpCount()==0,"enabled automatic jumps never mistake narrow animation clearance for a capsule blocker");
                ran|=t.wallRunning()&&t.position.z<250;
                climbed|=!t.wallRunning()&&t.position.z>300&&t.position.z<600;
                resumed|=t.wallRunning()&&t.position.z>650;
            }
            check(ran&&climbed&&resumed,"narrow-runway climb fallback and resumed running survive enabled obstacle jumps");
        }
        for(bool convex:{false,true}) {
            CornerWorld world;world.corner(convex);Traversal t;t.cfg=fc_test::settings();t.cfg.wallRunObstacleJumps=true;t.cfg.runSpeed=379.5f;
            check(t.attach(world,{convex?-180.f:-200.f,-37,0},{0,1,0},1000),"enabled automatic-jump corner test attaches");
            bool turn=false,finished=false;
            for(int frame=0;frame<fps*4;++frame) {
                const auto out=t.update(world,{1,0,false,false,false,false,true},1.f/fps,1000);
                check(t.active()&&!hopMotion(out.motion)&&t.obstacleJumpCount()==0,"joined wall route keeps priority over automatic obstacle jumps");
                turn|=t.turningCorner();if(turn&&!t.turningCorner()){finished=true;break;}
            }
            check(turn&&finished,"real inside and outside turns finish with automatic jump option enabled");
        }
        {
            auto world=fixture(up);world.boxes.push_back({{-10000,-240,-10000},{10000,-230,10000},false});
            auto t=attached(world);unsigned peak=0;bool compact=false,landed=false,exhausted=false;
            for(int frame=0;frame<fps*4;++frame) {
                const auto count=t.obstacleJumpCount();
                world.rays=0;const auto out=t.update(world,up,1.f/fps,1000);peak=std::max(peak,world.rays);
                if(world.rays>=4096) {
                    exhausted=true;
                    check(t.obstacleJumpCount()==count,"exhausted extended-body planner cannot commit a partial action that frame");
                }
                if(t.obstacleJumpCount()!=count) {
                    check(exhausted&&world.rays<4096&&out.motion==Motion::hopUp&&
                        std::string(t.blockedReason)=="checked wall-run obstacle bypass",
                        "a later independently checked compact catch may replace the rejected extended kick");
                    compact=true;
                }
                check(!out.released&&world.clearance(t.position,t.cfg)+.03f>=t.cfg.radius,
                    "compact fallback retains the complete capsule outside both obstacle and rear wall");
                check(!hopMotion(out.motion)||compact,"no unchecked jump can precede the accepted compact route");
                landed|=compact&&t.state==State::wall;
            }
            check(compact&&landed,"fully checked compact recovery catches the real wall after rejected extended-body plans");
            check(peak>=4096&&peak<4600,"late extended-body failures exercise exact planner budget and remain bounded");
        }
        {
            auto world=fixture(up);world.boxes.push_back({{-10000,-30,1050},{10000,20,1062}});
            auto t=attached(world);unsigned previous=0;float previousStart=-10;bool landed=false;
            for(int frame=0;frame<fps*4;++frame) {
                const auto out=t.update(world,up,1.f/fps,1000);check(!out.released,"two independent projecting obstacles retain a continuous checked run");
                if(t.obstacleJumpCount()!=previous) {
                    check(t.obstacleJumpCount()==previous+1&&float(frame)/fps-previousStart>.6f,"each real obstacle schedules exactly one action after landing cooldown");
                    previous=t.obstacleJumpCount();previousStart=float(frame)/fps;
                }
                if(previous>=2&&t.state!=State::action)landed=true;
            }
            check(previous==2&&landed,"automatic run can cross a second real obstruction without repeating on open wall");
        }
    }
}
static void staminaBoundary() {
    for(int fps:{30,60,120})for(Input input:{Input{0,1,false,true,false,false,true},Input{1,0,false,false,false,false,true},Input{1,1,false,true,false,false,true}}) {
        auto world=fixture(input);auto t=attached(world);Traversal before;
        for(int frame=0;frame<fps*4&&!t.obstacleJumpCount();++frame){before=t;t.update(world,input,1.f/fps,1000);}
        check(t.obstacleJumpCount()==1,"stamina boundary fixture reaches a planned automatic kick");
        const float threshold=30.f+2*t.cfg.drain*(t.actionDuration()+.05f);
        auto low=before;low.update(world,input,1.f/fps,threshold-.01f);
        check(low.obstacleJumpCount()==0,"automatic action refuses a balance below its rounded completion reserve");
        t=before;float remaining=threshold+.01f;
        auto out=t.update(world,input,1.f/fps,remaining);remaining-=out.staminaCost;
        check(t.obstacleJumpCount()==1,"balance just above full action reserve permits the checked action");
        while(t.state==State::action) {
            out=t.update(world,input,1.f/fps,remaining);remaining-=out.staminaCost;
            check(!out.released,"automatic stamina reservation actually survives per-frame engine-style deductions");
        }
        check(remaining>0,"rounded final flight update lands with positive stamina instead of compulsory falling");
    }
}
int main(){try{
    positiveCases();negativeCases();dynamicCases();routingAndBudgets();staminaBoundary();
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}return 0;}
