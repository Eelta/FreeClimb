#include "Pose.h"
#include "TraversalCapture.h"
#include "TraversalAudio.h"
#include "CornerTestWorld.h"
#include <iostream>
#include <memory>
#include <stdexcept>
using namespace fc;
namespace {
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
struct Plane:World {
    Vec normal{0,-1,0};
    std::optional<Hit> ray(Vec from,Vec to)override {
        const float a=from.dot(normal),b=to.dot(normal);
        if(a<=0||b>=0)return {};
        return Hit{from+(to-from)*(a/(a-b)),normal,true};
    }
};
struct Rig {
    Plane world;Traversal traversal;SurfacePose surface;TraversalAudio audio;Pose previous;Motion last=Motion::none;
    float dt;unsigned frames{},stepSounds{};float largestBoneStep{};
    Rig(float slope,int fps,float multiplier,float runSpeed=330):dt(1.f/fps) {
        world.normal={0,-std::sqrt(1-slope*slope),slope};
        traversal.cfg=fc_test::settings();traversal.cfg.diagonalRunMultiplier=multiplier;traversal.cfg.runSpeed=runSpeed;
        require(traversal.attach(world,{0,-37,0},{0,1,0},1000),"speed fixture attaches to a real wall plane");
    }
    Result tick(const Library& library,Input input) {
        const float oldPhase=surface.sampledPhase();
        const auto result=traversal.update(world,input,dt,1000);
        require(traversal.active()&&!result.released&&traversal.state==State::wall,"clear wall movement retains physical support");
        const auto pose=surface.update(library,world,traversal,result.motion,dt,1);
        require(pose.size()==99,"speed change retains complete actual SurfacePose output");
        for(unsigned bone=0;bone<pose.size();++bone) {
            require(pose[bone].t.finite()&&std::isfinite(pose[bone].q.dot(pose[bone].q)),"faster gait stays finite");
            if(!previous.empty()) {
                const float angle=angleBetween(previous[bone].q,pose[bone].q);
                largestBoneStep=std::max(largestBoneStep,angle/dt);
                const float limit=runMotion(result.motion)||runMotion(last)?18.849556f:12.566371f;
                require(angle<=limit*dt+.016f,"faster gait and its transitions retain the existing full-pose angular budget");
            }
        }
        require(library.armBendValid(pose,0)&&library.armBendValid(pose,1),"actual faster output retains legal elbow directions");
        const float phase=surface.sampledPhase();
        const auto sounds=audio.update(result.motion,phase,traversal.position,library.contactWeights(result.motion,phase),dt,true);
        if(runMotion(result.motion)&&result.motion!=last)
            require(sounds.count==0,"real cycle matching cannot emit a burst merely because the mode or selected gait phase changed");
        for(unsigned i=0;i<sounds.count;++i)if(sounds.items[i].cue==SoundCue::step&&runMotion(result.motion)) {
            const auto crossed=[&](float mark){return phase<oldPhase?(mark>oldPhase||mark<=phase):(oldPhase<mark&&phase>=mark);};
            require(crossed(.3421053f)||crossed(.8421053f),"actual wall-run audio occurs only when the rendered source gait crosses a foot support onset");
            ++stepSounds;
        }
        previous=pose;last=result.motion;++frames;return result;
    }
};
struct Measurement {float speed{},phaseRate{},staminaRate{};unsigned stepSounds{};};
Measurement steady(const Library& library,float slope,int fps,float multiplier,Input input,float runSpeed=330) {
    Rig rig(slope,fps,multiplier,runSpeed);
    for(int frame=0;frame<fps;++frame)rig.tick(library,input);
    float distance=0,phase=0,stamina=0;const unsigned initialSounds=rig.stepSounds;
    for(int frame=0;frame<fps*3;++frame) {
        const Vec start=rig.traversal.position;const float before=rig.surface.gaitPhase();
        stamina+=rig.tick(library,input).staminaCost;
        distance+=(rig.traversal.position-start).length();
        const float advance=rig.surface.gaitPhase()-before;phase+=advance<0?advance+1:advance;
        require(std::abs(rig.surface.sampledPhase()-rig.surface.gaitPhase())<.00001f,
            "loop audio sample phase is exactly the gait phase used by the real renderer");
    }
    return {distance/3,phase/3,stamina/3,rig.stepSounds-initialSounds};
}
void unchangedDirections(const Library& library,int fps) {
    for(Input input:{Input{0,1,false,false,false,false,true},Input{1,0,false,false,false,false,true},
        Input{-1,0,false,false,false,false,true},Input{1,1},Input{0,-1,false,false,false,false,true},
        Input{1,-1,false,false,false,false,true}}) {
        const auto old=steady(library,0,fps,1,input),now=steady(library,0,fps,1.15f,input);
        require(std::abs(old.speed-now.speed)<.002f&&std::abs(old.phaseRate-now.phaseRate)<.0001f,
            "stable straight/side runs, ordinary climbing and Shift descent retain their original travel and gait");
    }
}
void easedSwitches(const Library& library,int fps) {
    Rig rig(0,fps,1.15f);Input run{1,0,false,false,false,false,true};
    for(int frame=0;frame<fps;++frame)rig.tick(library,run);
    float previousSpeed=330;
    for(float vertical:{1.f,0.f}) {
        run.y=vertical;
        for(int frame=0;frame<fps/2;++frame) {
            const Vec from=rig.traversal.position;const auto result=rig.tick(library,run);
            const float speed=(rig.traversal.position-from).length()/rig.dt;
            require(std::abs(speed-previousSpeed)<=330*.15f*1.5f/.24f*rig.dt+.15f,
                "adding/removing upward direction eases the diagonal speed change rather than jumping fifteen percent");
            require(result.staminaCost<=20*rig.dt+.00001f,"diagonal acceleration does not increase the established double stamina rate");
            previousSpeed=speed;
        }
        require(std::abs(previousSpeed-(vertical>0?379.5f:330.f))<.05f,"direction blend converges to the intended speed");
    }
    run.y=1;run.hop=true;
    for(int frame=0;frame<fps/2;++frame) {
        const auto result=rig.tick(library,run);
        require(!hopMotion(result.motion)&&rig.traversal.state==State::wall,"accelerated wall-run Space remains consumed");
    }
    run.hop=false;run.y=-1;
    const Vec before=rig.traversal.position;const auto down=rig.tick(library,run);
    require(!runMotion(down.motion)&&!rig.traversal.wallRunning()&&
        (rig.traversal.position-before).length()/rig.dt<=std::hypot(82.f,64.f)*.707107f+.1f,
        "S immediately selects ordinary downward climbing without a residual diagonal run boost");
    const Vec stationary=rig.traversal.position;
    for(int frame=0;frame<fps/2;++frame)rig.tick(library,{});
    require((rig.traversal.position-stationary).length()<.0001f,"stopping the faster run never drifts or restarts locomotion");
    const float phase=rig.surface.gaitPhase(),sample=rig.surface.sampledPhase();
    const auto held=rig.surface.update(library,rig.world,rig.traversal,rig.last,0,1);
    require(rig.surface.gaitPhase()==phase&&rig.surface.sampledPhase()==sample&&held.size()==rig.previous.size(),
        "zero-delta render callbacks cannot advance animation or audio phase");
    auto tape=std::make_unique<TraversalCapture>(),loaded=std::make_unique<TraversalCapture>();
    TraversalCapture::RecordingWorld recording(rig.world,*tape);
    run.y=1;for(int frame=0;frame<3;++frame)rig.tick(library,run);
    tape->begin(rig.traversal,run,rig.dt,1000);
    const auto result=rig.traversal.update(recording,run,rig.dt,1000);tape->finish(rig.traversal,result);
    std::string error;
    require(tape->complete()&&loaded->deserialize(tape->serialize(),error)&&loaded->replay().matched,
        "capture preserves diagonal multiplier and in-flight acceleration for exact same-version replay");
}
void cornerLimit(const Library& library,int fps) {
    fc_test::CornerWorld world;world.corner(true);world.boxes[0].high.z=10000;
    Traversal traversal;traversal.cfg=fc_test::settings();traversal.cfg.runSpeed=379.5f;
    require(traversal.attach(world,{-155,-37,0},{0,1,0},1000),"accelerated corner test attaches");
    SurfacePose surface;Pose previous;const float dt=1.f/fps;bool entered=false,finished=false;
    for(int frame=0;frame<fps*3&&!finished;++frame) {
        const Vec normal=traversal.normal;
        const auto result=traversal.update(world,{1,1,false,false,true,false,true},dt,1000);
        require(traversal.active()&&!result.released&&traversal.state==State::wall&&!hopMotion(result.motion),
            "diagonal corner running stays supported and cannot jump");
        if(traversal.turningCorner())entered=true;else if(entered)finished=true;
        require(world.clearance(traversal.position,traversal.cfg)>=30.95f,
            "independent box distance certifies full capsule clearance at the faster requested speed");
        require(std::acos(std::clamp(normal.dot(traversal.normal),-1.f,1.f))<=8.2f*dt+.02f,
            "diagonal speed increase preserves the corner curvature speed limit");
        const auto pose=surface.update(library,world,traversal,result.motion,dt,1);
        if(!previous.empty())for(unsigned bone=0;bone<pose.size();++bone)
            require(angleBetween(previous[bone].q,pose[bone].q)<=18.849556f*dt+.016f,
                "accelerated actual corner pose preserves all joint continuity limits");
        previous=pose;
    }
    require(entered&&finished,"accelerated diagonal traversal completes the physical corner");
}
void allRunDirections(const Library& library,int fps) {
    for(float slope:{0.f,.35f})for(Input input:{Input{0,1,false,false,false,false,true},
        Input{-1,0,false,false,false,false,true},Input{1,0,false,false,false,false,true},
        Input{-1,1,false,false,false,false,true},Input{1,1,false,false,false,false,true}}) {
        const auto old=steady(library,slope,fps,1.15f,input,330),now=steady(library,slope,fps,1.15f,input,379.5f);
        const bool diagonal=std::abs(input.x)>.1f&&input.y>.1f;
        require(std::abs(now.speed-(diagonal?436.425f:379.5f))<.025f,"all requested wall-run directions reach new fifteen-percent-higher production speed");
        require(std::abs(now.speed/old.speed-1.15f)<.0005f&&std::abs(now.phaseRate/old.phaseRate-1.15f)<.0005f,
            "forward, side and diagonal rendered gait tracks the extra accepted travel once");
        require(std::abs(now.staminaRate-20)<.001f&&now.stepSounds>=old.stepSounds,
            "accelerated wall running preserves double stamina and source-timed audio contacts");
        std::cout<<"all-run fps="<<fps<<" slope="<<slope<<" direction="<<input.x<<','<<input.y<<" speed="<<old.speed<<" -> "<<now.speed
            <<" phase="<<old.phaseRate<<" -> "<<now.phaseRate<<" steps="<<old.stepSounds<<" -> "<<now.stepSounds<<'\n';
    }
    for(Input input:{Input{0,1},Input{1,0},Input{-1,0},Input{1,1},Input{0,-1,false,false,false,false,true}}) {
        const auto old=steady(library,0,fps,1.15f,input,330),now=steady(library,0,fps,1.15f,input,379.5f);
        require(std::abs(old.speed-now.speed)<.001f&&std::abs(old.phaseRate-now.phaseRate)<.0001f,
            "new run base speed never accelerates ordinary climbing or Shift descent");
    }
    for(Input run:{Input{0,1,false,false,false,false,true},Input{1,0,false,false,false,false,true},Input{1,1,false,false,false,false,true}}) {
        Rig rig(0,fps,1.15f,379.5f);Input climb=run;climb.run=false;
        for(int frame=0;frame<fps;++frame)rig.tick(library,climb);
        for(int mode:{1,0}) {
            Input input=run;input.run=bool(mode);float previousSpeed=0;
            for(int frame=0;frame<fps;++frame) {
                const Vec before=rig.traversal.position;const auto out=rig.tick(library,input);
                const float speed=(rig.traversal.position-before).length()/rig.dt;
                if(frame)require(std::abs(speed-previousSpeed)<1900*rig.dt+.2f,
                    "entering/leaving new run speed retains bounded continuous acceleration");
                require(!hopMotion(out.motion),"faster movement never enables wall-run jumping");
                previousSpeed=speed;
            }
            const float climbSpeed=std::abs(input.x)>.1f&&input.y>.1f?std::hypot(82.f,100.f)*.70710678f:input.y>0?100.f:82.f;
            const float expected=mode?(std::abs(input.x)>.1f&&input.y>.1f?436.425f:379.5f):climbSpeed;
            require(std::abs(previousSpeed-expected)<.05f,"release Shift settles completely to original climb speed");
        }
    }
}

}
int main(int argc,char** argv) {
    try {
        require(argc==2,"motion path required");Library library;require(library.load(argv[1]),"motion library loads");
        for(int fps:{30,60,120}) {
            for(float slope:{0.f,.35f})for(float side:{-1.f,1.f}) {
                Input input{side,1,false,false,false,false,true};
                const auto before=steady(library,slope,fps,1,input),after=steady(library,slope,fps,1.15f,input);
                require(std::abs(after.speed/before.speed-1.15f)<.0005f,"diagonal travel increases by fifteen percent");
                require(std::abs(after.phaseRate/before.phaseRate-1.15f)<.0005f,
                    "actual gait speeds up once with accepted distance, without a second playback multiplier or phase cap");
                require(std::abs(after.staminaRate-20)<.001f,"run stamina remains two times climbing per second");
                require(after.stepSounds>=before.stepSounds&&after.stepSounds>=8,
                    "actual faster diagonal gait produces correspondingly frequent source-timed steps over equal three-second intervals");
                std::cout<<"diagonal fps="<<fps<<" slope="<<slope<<" side="<<side<<" speed="<<before.speed<<" -> "<<after.speed
                    <<" cycles/s="<<before.phaseRate<<" -> "<<after.phaseRate<<" steps/3s="<<before.stepSounds<<" -> "<<after.stepSounds<<'\n';
            }
            unchangedDirections(library,fps);easedSwitches(library,fps);cornerLimit(library,fps);allRunDirections(library,fps);
        }
        std::cout<<"DiagonalSpeedTests passed: real Core, SurfacePose, capture, and corner clearance at 30/60/120 FPS\n";return 0;
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
