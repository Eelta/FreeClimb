#include "PoseHandoff.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
static void check(bool ok,const char* message) {if(!ok)throw std::runtime_error(message);}
static float distance(const Pose& a,const Pose& b) {
    float value=0;
    for(std::size_t i=0;i<a.size();++i)value=std::max(value,(a[i].t-b[i].t).length()+angleBetween(a[i].q,b[i].q));
    return value;
}
struct InterruptedExitWall:World {
    Vec outward{0,-1,0};std::optional<float> rearDistance;
    std::optional<Hit> ray(Vec from,Vec to) override {
        const float a=from.dot(outward),b=to.dot(outward);
        if(a>0&&b<=0){const float t=a/(a-b);return Hit{from+(to-from)*t,outward,true};}
        if(rearDistance&&a<*rearDistance&&b>=*rearDistance){const float t=(*rearDistance-a)/(b-a);return Hit{from+(to-from)*t,outward*-1,false};}
        return {};
    }
    bool actionBodyClear(Motion motion,Vec,Vec,float begin,float end,Vec) override {
        return motion==Motion::backFlipOut&&begin>=0&&end>=begin&&end<=1;
    }
};
static void interruptedDepartureMotion(int fps,bool flip,float yaw){
    InterruptedExitWall wall;wall.outward={std::sin(yaw),-std::cos(yaw),0};Traversal traversal;
    traversal.cfg.approachSeconds=0;traversal.cfg.fancyJumps=flip;
    check(traversal.attach(wall,wall.outward*30+Vec{0,0,1000},wall.outward*-1,100),"interrupt fixture attaches to a real wall plane");
    const float dt=1.f/fps;const Motion expected=flip?Motion::backFlipOut:Motion::dropBack;
    auto result=traversal.update(wall,{0,-1,true,false,false,true},dt,100);
    while(traversal.active()&&traversal.actionProgress()<.35f)result=traversal.update(wall,{},dt,100);
    check(traversal.active()&&result.motion==expected,"obstacle appears only after the checked departure is underway");
    const auto before=traversal.position;const float phase=traversal.actionProgress();
    wall.rearDistance=before.dot(wall.outward)+traversal.cfg.radius+.05f;
    result=traversal.update(wall,{},dt,100);
    std::cout<<"INTERRUPTED_DEPARTURE fps="<<fps<<" flip="<<flip<<" yaw="<<yaw<<" beforeMotion="<<int(expected)<<" releasedMotion="<<int(result.motion)<<'\n';
    check(result.released&&!result.completed&&!traversal.active()&&std::string(result.reason)=="jump path changed","new obstacle aborts the departure through the existing collision failure");
    check((traversal.position-before).length()<.00001f&&traversal.actionProgress()==phase,"interruption commits no unchecked root movement or animation phase");
    check(result.releaseVelocity.length()==0,"blocking departure cannot add an unchecked outward impulse");
    check(result.motion==expected&&isActiveMotion(result.motion),"interrupted departure retains its actual action instead of publishing none and falling back to hang");
}
int main(int argc,char** argv) {try {
    Library lib;check(argc==2&&lib.load(argv[1]),"load actual motion resource");
    for(int fps:{20,30,48,60,120})for(bool flip:{false,true})for(float yaw:{0.f,.8f,2.2f})interruptedDepartureMotion(fps,flip,yaw);
    for(float fps:{20.f,30.f,48.f,60.f,120.f}) {
        const float dt=1/fps;
        PoseHandoff handoff;
        Pose old(1),shown(1),unseen(1),native(1);
        old[0].t={1-40*dt,0,0};shown[0].t={1,0,0};unseen[0].t={900,0,0};
        old[0].q=Quat::axis({0,0,1},.5f-2*dt);shown[0].q=Quat::axis({0,0,1},.5f);
        const auto first=handoff.evaluate(native,old,1,0,1-dt);
        check(handoff.consumed(first),"older displayed sample accepted");
        const auto current=handoff.evaluate(native,shown,1,0,1);
        for(int pass=0;pass<6;++pass)check(handoff.consumed(current),"repeated scene passes accepted without inventing samples");
        check(!handoff.consumed(first),"late output cannot rewind displayed history");
        const auto unconsumed=handoff.evaluate(native,unseen,1,0,1+dt);
        check(handoff.beginExit(true),"physical fall begins from consumed source");
        check(!handoff.consumed(unconsumed),"pending terminal output cannot reset new fade");
        handoff.advanceExitSource(0,lib);
        check(distance(handoff.evaluate(native,unseen,1).pose,shown)<1e-6f,"release starts at exactly the displayed pose");
        handoff.advanceExitSource(.0001f,lib);
        const auto advanced=handoff.evaluate(native,unseen,1).pose;
        check((advanced[0].t.x-shown[0].t.x)/.0001f>39.f,"outgoing translation is continuous through release");
        check(angleBetween(advanced[0].q,shown[0].q)/.0001f>1.95f,"outgoing rotation continues instead of freezing");
        handoff.advanceExitSource(2,lib);
        const auto bounded=handoff.evaluate(native,unseen,1).pose;
        check(distance(bounded,shown)<2.74f,"prediction decays rather than extrapolating through the full fall");
        check(distance(handoff.evaluate(native,unseen,0).pose,native)<1e-6f,"exit reaches live native exactly");

        PoseHandoff real;
        const auto previous=lib.sample(Motion::dropBack,std::max(0.f,.92f-dt/.32f));
        const auto displayed=lib.sample(Motion::dropBack,.92f);
        check(real.consumed(real.evaluate(lib.rest,previous,1,0,1-dt)),"real kick previous sample");
        check(real.consumed(real.evaluate(lib.rest,displayed,1,0,1)),"real kick displayed sample");
        check(real.beginExit(true),"real kick moving exit");
        for(int frame=0;frame<=32;++frame) {
            const float elapsed=frame*.005f;
            real.advanceExitSource(elapsed,lib);
            const auto result=real.evaluate(lib.rest,displayed,1-smooth(elapsed/.16f));
            check(lib.armBendValid(result.source,0)&&lib.armBendValid(result.source,1),"kick-off extrapolation cannot reverse the elbows");
            for(const auto& bone:result.pose)check(bone.t.finite()&&std::isfinite(bone.q.dot(bone.q)),"finite real kick exit");
        }
    }
    std::cout<<"PASS: consumed kick-off velocity, repeated callbacks, unseen terminal rejection, bounded anatomical continuation, live native endpoint\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
