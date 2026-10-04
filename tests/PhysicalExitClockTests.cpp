#include "PoseHandoff.h"
#include "PoseBlendEnvelope.h"
#include "PoseFrameClock.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
static void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
static bool same(const Pose& a,const Pose& b){
    if(a.size()!=99||b.size()!=99)return false;
    for(std::size_t i=0;i<a.size();++i)if((a[i].t-b[i].t).length()>.0001f||angleBetween(a[i].q,b[i].q)>.0001f||(a[i].s-b[i].s).length()>.0001f)return false;
    return true;
}
struct DepartureWall:World {
    const Library& library;
    explicit DepartureWall(const Library& value):library(value){}
    std::optional<Hit> ray(Vec a,Vec b)override{
        if(a.y>=0||b.y<0)return {};return Hit{a+(b-a)*(-a.y/(b.y-a.y)),{0,-1,0},true};
    }
    bool actionBodyClear(Motion motion,Vec from,Vec to,float begin,float end,Vec normal)override{
        return motion==Motion::backFlipOut&&backFlipBodyClear(*this,library,from,to,begin,end,normal,0,37,1);
    }
};
struct Departure {Pose older,displayed,unconsumed;float interval{},time{};Motion motion{};};
static Departure departure(const Library& library,bool flip,int fps){
    DepartureWall wall(library);Traversal traversal;SurfacePose surface;Departure captured;
    traversal.cfg.approachSeconds=0;traversal.cfg.fancyJumps=flip;traversal.cfg.contextActions=false;
    traversal.cfg.radius=31;traversal.cfg.gap=37;traversal.cfg.height=138;
    require(traversal.attach(wall,{0,-37,1000},{0,1,0},1000),"physical exit starts from a real supported wall attachment");
    captured.interval=1.f/fps;float time=0;
    for(int frame=0;frame<fps;++frame){
        const auto result=traversal.update(wall,{},captured.interval,1000);time+=captured.interval;
        surface.update(library,wall,traversal,result.motion,captured.interval,1);
    }
    for(int frame=0;frame<fps*2;++frame){
        Input input;if(frame==0){input.y=-1;input.release=input.backDrop=true;}
        const auto result=traversal.update(wall,input,captured.interval,1000);time+=captured.interval;
        require(result.motion==(flip?Motion::backFlipOut:Motion::dropBack),"actual checked departure retains the requested push or flip");
        const auto pose=surface.update(library,wall,traversal,result.motion,captured.interval,1);
        if(result.released){
            require(traversal.actionProgress()==1&&!result.completed&&result.releaseVelocity.length()>100,"the physical handoff follows the complete actual action and its checked release impulse");
            captured.unconsumed=pose;captured.motion=result.motion;break;
        }
        captured.older=captured.displayed;captured.displayed=pose;captured.time=time;
    }
    require(captured.older.size()==99&&captured.displayed.size()==99&&captured.unconsumed.size()==99,"retain two actually displayed SurfacePose frames and the final unpublished action frame");
    require(library.armBendValid(captured.displayed,0)&&library.armBendValid(captured.displayed,1),"the actual outgoing physical pose satisfies the existing elbow guards");
    return captured;
}
static Pose nativeFall(const Library& library,float time){
    auto pose=library.sample(Motion::drop,1);pose[0]=library.rest[0];pose[4]=library.rest[4];
    pose[4].t=pose[4].t+Vec{3*time,-2*time,2*std::sin(5*time)};
    pose[6].q=(Quat::axis({1,0,0},.18f*std::sin(7*time))*pose[6].q).unit();
    pose[9].q=(Quat::axis({1,0,0},-.18f*std::sin(7*time))*pose[9].q).unit();
    pose[36].q=(Quat::axis({0,0,1},.7f*time)*pose[36].q).unit();return pose;
}
static void schedule(const Library& library,const Departure& captured,int playerFps,int renderFps,unsigned passes,bool pause,float firstDelay){
    const auto base=nativeFall(library,0);PoseHandoff handoff;
    require(handoff.consumed(handoff.evaluate(base,captured.older,1,0,captured.time-captured.interval)),"older SurfacePose output completes propagation");
    require(handoff.consumed(handoff.evaluate(base,captured.displayed,1,0,captured.time)),"last displayed SurfacePose output completes propagation");
    const auto pending=handoff.evaluate(base,captured.unconsumed,1,0,captured.time+captured.interval);
    require(handoff.beginExit(true)&&!handoff.consumed(pending),"physical release rejects an unconsumed final action publication");
    PoseContinuation reference;reference.begin(captured.displayed,captured.older,captured.interval);
    PoseBlendEnvelope envelope;PoseFrameClock clock;std::uint32_t applied=0;envelope.beginExit(0,PoseBlendEnvelope::fallExitSeconds);
    bool terminal=false;float previousPhase=0;
    for(int frame=0;frame<renderFps&&!terminal;++frame){
        const float time=firstDelay+float(frame)/renderFps+(pause&&frame>=3?.25f:0.f);
        const float playerAge=std::floor(time*playerFps+.0001f)*std::min(1.f/playerFps,.05f);
        const auto stamp=std::uint32_t(std::lround(time*1000));
        const float elapsed=clock.sample(stamp,1.f/renderFps,1.f/renderFps,false);envelope.advanceExit(applied,elapsed);
        const float phase=envelope.elapsedExitSeconds();
        require(phase>=previousPhase&&phase-previousPhase<=.050001f,"missing rendered outputs preserve the bounded acknowledged exit interval");previousPhase=phase;
        handoff.advanceExitSource(playerAge,library,phase);
        const auto native=nativeFall(library,time);const auto output=handoff.evaluate(native,captured.unconsumed,envelope.weight());
        auto expectedSource=reference.sample(phase);library.guardArmBends(expectedSource);
        require(same(output.source,expectedSource),"physical continuation follows acknowledged scene time instead of lower-frequency player ticks");
        if(frame==0)require(phase==0&&same(output.pose,captured.displayed),"a delayed first scene callback retains the exact displayed departure pose");
        Pose expected=native;for(std::size_t bone=0;bone<expected.size();++bone)expected[bone]=blend(expectedSource[bone],native[bone],1-envelope.weight());
        require(same(output.pose,expected),"the physical source and live moving fall target use the same single exit phase");
        require(library.armBendValid(output.source,0)&&library.armBendValid(output.source,1),"clock synchronization retains both existing physical elbow protections");
        for(unsigned pass=0;pass<passes;++pass){
            envelope.advanceExit(applied,clock.sample(stamp,1.f/renderFps,1.f/renderFps,false));
            handoff.advanceExitSource(playerAge+.01f*pass,library,envelope.elapsedExitSeconds());
            const auto repeated=handoff.evaluate(native,captured.unconsumed,envelope.weight());
            require(same(repeated.pose,output.pose)&&same(repeated.source,output.source),"duplicate scene passes cannot change source motion even when the player clock advances between them");
            require(handoff.consumed(repeated),"each repeated scene pass confirms the same valid output");
            if(envelope.weight()==0){require(same(repeated.pose,native),"the final physical exit output is exactly the current native fall pose");terminal=true;}
            envelope.acknowledgeExit(terminal);++applied;
        }
        if(!terminal){
            handoff.advanceExitSource(playerAge+1,library,NAN);
            require(same(handoff.evaluate(native,captured.unconsumed,envelope.weight()).pose,output.pose),"invalid explicit scene time cannot fall back to a later player continuation time");
        }
    }
    require(terminal&&envelope.exitComplete(applied),"physical exit finishes only after a confirmed fully native scene output");
    std::cout<<"PHYSICAL_EXIT_CLOCK motion="<<int(captured.motion)<<" sourceFps="<<int(std::round(1/captured.interval))<<" playerFps="<<playerFps<<" renderFps="<<renderFps<<" passes="<<passes<<" pause="<<pause<<" firstDelay="<<firstDelay<<'\n';
}
int main(int argc,char** argv){try{
    Library library;require(argc==2&&library.load(argv[1]),"load the actual current full animation pack");
    for(bool flip:{false,true})for(int sourceFps:{20,40,60}){
        const auto captured=departure(library,flip,sourceFps);
        for(int playerFps:{18,60})for(int renderFps:{40,60})for(unsigned passes:{1u,7u})for(bool pause:{false,true})for(float delay:{0.f,.1f})
            schedule(library,captured,playerFps,renderFps,passes,pause,delay);
    }
    std::cout<<"PASS physical exit clock: complete checked push and flip routes, actual SurfacePose history, moving native fall, delayed and repeated callbacks\n";return 0;
}catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}}
