#include "CornerTestWorld.h"
#include "TraversalCapture.h"
#include <iostream>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
using namespace fc;
using fc_test::CornerWorld;
namespace {
const char* failedCapturePath=nullptr;
void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
CornerWorld fixture(float depth,float thickness,bool lateral=false,int side=1) {
    CornerWorld w;
    w.boxes.push_back({{-3000,0,-3000},{3000,500,3000}});
    if(lateral) {
        w.boxes.push_back({{side>0?80.f:-80.f-thickness,-depth,-3000},{side>0?80.f+thickness:-80.f,20,3000}});
    } else w.boxes.push_back({{-3000,-depth,200},{3000,20,200+thickness}});
    return w;
}
Traversal attached(CornerWorld& w) {
    Traversal t;t.cfg=fc_test::settings();t.cfg.runSpeed=379.5f;t.cfg.wallRunObstacleJumps=true;
    check(t.attach(w,w.global({0,-37,-100}),w.direction({0,1,0}),1000),"adaptive test attaches real source");return t;
}
Input movement(bool run,bool lateral=false,int side=1) {
    return {lateral?float(side):0.f,lateral?0.f:1.f,false,true,false,false,run};
}
bool adaptive(const Traversal& t) {
    const std::string reason=t.blockedReason;
    return reason.find("eave")!=std::string::npos||reason.find("checked wall-run obstacle bypass")!=std::string::npos;
}
struct Started {Traversal traversal;Traversal before;Input input;bool found{};unsigned peak{};};
Started start(CornerWorld& w,int fps,bool run=false,bool lateral=false,int side=1,float stamina=1000) {
    auto t=attached(w);Started result;result.input=movement(run,lateral,side);
    auto failureTape=std::make_unique<TraversalCapture>();TraversalCapture::RecordingWorld observed(w,*failureTape);
    for(int frame=0;frame<fps*5&&t.active();++frame) {
        result.before=t;w.rays=0;failureTape->begin(t,result.input,1.f/fps,stamina);
        const auto out=t.update(observed,result.input,1.f/fps,stamina);failureTape->finish(t,out);
        result.peak=std::max(result.peak,w.rays);
        if(w.clearance(t.position,t.cfg)+.04f<t.cfg.radius&&failedCapturePath)std::ofstream(failedCapturePath)<<failureTape->serialize();
        if(w.clearance(t.position,t.cfg)+.04f<t.cfg.radius)std::cerr<<"unsafe frame="<<frame<<" run="<<run<<" side="<<side<<" lateral="<<lateral<<" pos="<<w.local(t.position).x<<","<<w.local(t.position).y<<","<<w.local(t.position).z<<" motion="<<int(out.motion)<<" reason="<<t.blockedReason<<'\n';
        check(w.clearance(t.position,t.cfg)+.04f>=t.cfg.radius,"planner keeps independent capsule outside obstacle");
        check(!out.released,"blocked source retains actual support");
        if(t.state==State::action&&adaptive(t)){result.traversal=t;result.found=true;return result;}
    }
    result.traversal=t;return result;
}
void positive() {
    unsigned count=0,peak=0;float minimumSeconds=10,maximumSeconds=0,maximumSettle=0;
    for(int fps:{30,60,120})for(float depth:{68.f,96.f})for(float thickness:{8.f,19.f})for(bool run:{false,true}) {
        auto w=fixture(depth,thickness);
        if(fps==120){w.origin={134559.219f,36994.7031f,-11691.1309f};w.yaw=.57f;}
        auto planned=start(w,fps,run);peak=std::max(peak,planned.peak);
        if(!planned.found)std::cerr<<"missing fps="<<fps<<" depth="<<depth<<" thick="<<thickness<<" run="<<run<<" pos="<<w.local(planned.traversal.position).z<<" reason="<<planned.traversal.blockedReason<<'\n';
        check(planned.found,"measured overhead route reaches real vertical wall above the beam");
        auto t=planned.traversal;const auto initial=t.position;bool caught=false;
        minimumSeconds=std::min(minimumSeconds,t.actionDuration());maximumSeconds=std::max(maximumSeconds,t.actionDuration());
        auto tape=std::make_unique<TraversalCapture>();TraversalCapture::RecordingWorld recorded(w,*tape);
        tape->begin(planned.before,planned.input,1.f/fps,1000);
        auto replayActor=planned.before;const auto launched=replayActor.update(recorded,planned.input,1.f/fps,1000);tape->finish(replayActor,launched);
        check(tape->complete()&&tape->replay().matched,"accepted geometry planner is exactly replayable");
        check(launched.staminaCost==(run?30.f:15.f),"recovery charges only the chosen mode action cost");
        for(int frame=0;frame<fps*2&&t.state==State::action;++frame) {
            const float remaining=(1-t.actionProgress())*t.actionDuration();
            if(remaining<=1.f/fps) {
                auto endpoint=t;const auto exact=endpoint.update(w,planned.input,remaining,1000);
                check(!exact.released,"exact action endpoint retains measured catch");
                const float correction=(endpoint.position-t.edgeTarget()).length();maximumSettle=std::max(maximumSettle,correction);
                check(correction<.04f,"landing does not snap root away from planned endpoint");
            }
            const Vec previous=t.position;
            const auto out=t.update(w,planned.input,1.f/fps,1000);
            check((t.position-previous).length()<=420.f/fps+.04f,"actual adaptive root motion leaves the original hand-foot velocity budget intact");
            check(!out.released,"new vertical target stays attached through playback");
            check(w.clearance(t.position,t.cfg)+.04f>=t.cfg.radius,"entire actual path clears independent capsule");
            caught=t.state==State::wall;
        }
        check(caught&&w.local(t.position).z+6>w.boxes[1].high.z,"body clears actual obstruction before catching vertical wall");
        check((t.position-initial).length()<=t.cfg.reach+80.1f,"measured path retains bounded regrab reach");
        const auto next=t.update(w,planned.input,1.f/fps,1000);
        check(!next.released&&(run?runMotion(next.motion):next.motion==Motion::up),"landing resumes currently held movement without extra key press");
        ++count;
    }
    std::cout<<"adaptive overhead="<<count<<" peak="<<peak<<" seconds="<<minimumSeconds<<".."<<maximumSeconds<<" maxSettle="<<maximumSettle<<'\n';
}
void lateral() {
    unsigned count=0;
    for(int fps:{30,60,120})for(int side:{-1,1})for(bool run:{false,true}) {
        auto world=fixture(96,19,true,side);world.boxes[1].climbable=false;
        if(fps==120){world.origin={132559.219f,32994.7031f,-12691.1309f};world.yaw=.83f;}
        auto planned=start(world,fps,run,true,side);
        if(!planned.found)std::cerr<<"missing lateral fps="<<fps<<" side="<<side<<" run="<<run<<" pos="<<world.local(planned.traversal.position).x<<" reason="<<planned.traversal.blockedReason<<'\n';
        check(planned.found,"side blocker obtains same-direction actual wall beyond obstruction");
        auto t=planned.traversal;const auto initial=world.local(t.position);
        auto tape=std::make_unique<TraversalCapture>();TraversalCapture::RecordingWorld recorded(world,*tape);
        for(int i=0;i<fps*2&&t.state==State::action;++i) {
            const Vec previous=t.position;
            tape->begin(t,planned.input,1.f/fps,1000);const auto out=t.update(recorded,planned.input,1.f/fps,1000);tape->finish(t,out);
            check((t.position-previous).length()<=420.f/fps+.04f,"lateral adaptive root motion retains hand-foot velocity headroom");
            if(out.released&&failedCapturePath)std::ofstream(failedCapturePath)<<tape->serialize();
            if(out.released)std::cerr<<"lateral release fps="<<fps<<" side="<<side<<" run="<<run<<" phase="<<t.actionProgress()<<" pos="<<world.local(t.position).x<<","<<world.local(t.position).y<<","<<world.local(t.position).z<<" reason="<<out.reason<<" rayFrom="<<world.local(t.blockedFrom).x<<","<<world.local(t.blockedFrom).y<<","<<world.local(t.blockedFrom).z<<" rayTo="<<world.local(t.blockedTo).x<<","<<world.local(t.blockedTo).y<<","<<world.local(t.blockedTo).z<<'\n';
            check(!out.released,"lateral checked route keeps support");
            check(world.clearance(t.position,t.cfg)+.04f>=t.cfg.radius,"lateral route clears independent body");
            check((world.local(t.position).x-initial.x)*side>=-.03f,"lateral recovery never travels against held direction");
        }
        check(t.state==State::wall&&(world.local(t.position).x*side)>130,"lateral recovery lands beyond actual column width");
        ++count;
    }
    std::cout<<"adaptive lateral="<<count<<'\n';
}
void negative() {
    for(int mode=0;mode<6;++mode) {
        auto w=fixture(mode==0?300.f:96.f,mode==1?200.f:19.f);auto t=attached(w);
        auto input=movement(mode==2||mode==3||mode==5);float stamina=mode==3?30.f:1000;
        if(mode==1)w.boxes[1].climbable=false;
        if(mode==2)t.cfg.wallRunObstacleJumps=false;
        if(mode==4){w.boxes[0].high.z=200;w.boxes[1].climbable=false;}
        if(mode==5)w.boxes.resize(1);
        for(int frame=0;frame<300&&t.active();++frame) {
            const auto out=t.update(w,input,1.f/60,stamina);
            if(adaptive(t))std::cerr<<"negative mode="<<mode<<" frame="<<frame<<" reason="<<t.blockedReason<<" pos="<<t.position.z<<'\n';
            check(!adaptive(t),"deep thick missing disabled depleted or absent obstruction cannot launch adaptive route");
            check(w.clearance(t.position,t.cfg)+.04f>=t.cfg.radius,"rejected candidate never penetrates geometry");
            if(mode==5)check(!hopMotion(out.motion),"unobstructed running never creates an unsolicited jump");
        }
    }
}
void dynamic() {
    for(int fps:{30,60,120})for(bool run:{false,true}) {
        auto world=fixture(96,19);auto planned=start(world,fps,run);
        check(planned.found,"dynamic case obtains measured route");
        {
            auto t=planned.traversal,witness=t;auto changed=world;changed.boxes[0].high.z=210;
            const auto out=t.update(changed,planned.input,1.f/fps,1000);
            check(!out.released&&t.active()&&t.geometryHolding()&&(t.position-witness.position).length()<.001f&&
                t.progress()==witness.progress()&&out.staminaCost==0,"removed upper grip holds before committing stale travel");
            for(int i=0;i<fps;++i) {
                const auto held=t.update(changed,planned.input,1.f/fps,1000);
                check(!held.released&&t.geometryHolding()&&(t.position-witness.position).length()<.001f&&
                    t.progress()==witness.progress(),"missing upper grip cannot advance during retries");
            }
            for(int i=0;i<fps*3&&t.state==State::action;++i)check(!t.update(world,planned.input,1.f/fps,1000).released,"restored adaptive target resumes without dropping");
            check(t.active()&&!t.geometryHolding()&&t.state==State::wall,"restored adaptive target completes the checked catch");
        }
        {
            auto t=planned.traversal;
            for(int i=0;i<fps&&t.state==State::action&&t.actionProgress()<.4f;++i)t.update(world,planned.input,1.f/fps,1000);
            const auto previous=t.position;auto changed=world;
            const auto p=world.local(previous);changed.boxes.push_back({{p.x-200,p.y-180,p.z+140},{p.x+200,p.y+180,p.z+145},false});
            bool released=false;
            for(int i=0;i<fps&&t.state==State::action;++i){auto out=t.update(changed,planned.input,1.f/fps,1000);released|=out.released;check(changed.clearance(t.position,t.cfg)+.04f>=t.cfg.radius,"late blocker stops before occupied body volume");}
            check(!released&&t.active()&&t.geometryHolding(),"new overhead solid pauses the actual path");
            const auto heldPosition=t.position;const float heldPhase=t.progress();
            for(int i=0;i<fps;++i) {
                const auto held=t.update(changed,planned.input,1.f/fps,1000);
                check(!held.released&&(t.position-heldPosition).length()==0&&t.progress()==heldPhase,
                    "blocked adaptive route retains its exact root and phase");
            }
            Input release;release.release=true;auto out=t.update(changed,release,1.f/fps,1000);
            for(int i=0;i<fps&&!out.released;++i)out=t.update(changed,{},1.f/fps,1000);
            check(out.released&&!t.active(),"blocked adaptive route permits deliberate release");
        }
        if(run) {
            auto t=planned.traversal;auto climb=planned.input;climb.run=false;
            for(int i=0;i<fps*2&&t.state==State::action;++i)check(!t.update(world,climb,1.f/fps,1000).released,"release run during checked recovery stays continuous");
            const auto next=t.update(world,climb,1.f/fps,1000);
            check(!runMotion(next.motion)&&!t.wallRunning(),"released modifier lands into climbing");
        }
    }
}
}
int main(int argc,char** argv){if(argc>1)failedCapturePath=argv[1];try{positive();negative();dynamic();lateral();return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
