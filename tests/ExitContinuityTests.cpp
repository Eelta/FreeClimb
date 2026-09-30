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
int main(int argc,char** argv) {try {
    Library lib;check(argc==2&&lib.load(argv[1]),"load actual motion resource");
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
