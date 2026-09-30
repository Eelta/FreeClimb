#include "Pose.h"
#include "CornerTestWorld.h"
#include <fstream>
#include <iostream>
#include <stdexcept>
using namespace fc;
namespace {
std::ofstream dump;bool firstSample=true;
struct NoPoseContacts:World {std::optional<Hit> ray(Vec,Vec)override{return {};}};
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
bool kick(Motion motion){return motion>=Motion::kickUp&&motion<=Motion::kickRight;}
Vec point(Vec value,Vec root,Vec normal){return root+Vec{-normal.y,normal.x,0}*value.x-normal*value.y+Vec{0,0,value.z};}
void sample(const char* name,const char* stage,float phase,Motion motion,Vec position,Vec normal,const Pose& body) {
    if(!dump)return;if(!firstSample)dump<<',';firstSample=false;
    dump<<"{\"case\":\""<<name<<"\",\"stage\":\""<<stage<<"\",\"phase\":"<<phase<<",\"motion\":"<<int(motion)
        <<",\"position\":["<<position.x<<','<<position.y<<','<<position.z<<"],\"normal\":["<<normal.x<<','<<normal.y<<','<<normal.z<<"],\"joints\":[";
    for(unsigned bone=0;bone<body.size();++bone) {if(bone)dump<<',';const auto p=point(body[bone].t,position,normal);dump<<'['<<p.x<<','<<p.y<<','<<p.z<<']';}
    dump<<"]}";
}
Pose authoredCycle(const Library& lib,const Traversal& t,Motion motion,float phase) {
    auto pose=lib.sample(motion,phase);const auto direction=t.direction();
    if(runMotion(motion)) {
        if(std::abs(direction.x)>.1f) {
            const auto frame=sideRunFrame(t.surfaceNormal.z,direction);
            placeSideRun(lib,pose,frame,-t.cfg.gap*std::sqrt(1-t.surfaceNormal.z*t.surfaceNormal.z));
            applySideRunBrace(lib,pose,frame,phase);limitSideRunUpperRoll(lib,pose,direction.x>0?0:1);
        } else {
            const auto rotation=Quat::axis({1,0,0},std::acos(t.surfaceNormal.z));
            pose[0].q=rotation*pose[0].q;pose[0].t=rotation.rotate(pose[0].t)+Vec{0,t.cfg.gap-3,0};
        }
    } else pose[4].t.y+=t.cfg.gap-38;
    lib.guardArmBends(pose);lib.forearmTwist(pose);return pose;
}
float steadyError(const Library& lib,int fps,Input input) {
    fc_test::CornerWorld world;world.boxes.push_back({{-10000,0,-10000},{10000,1000,10000}});
    Traversal t;t.cfg=fc_test::settings();t.cfg.runSpeed=379.5f;
    require(t.attach(world,{0,-37,100},{0,1,0},1000),"baseline output attaches");
    SurfacePose surface;float error=0;
    for(int frame=0;frame<fps*3;++frame) {
        const auto result=t.update(world,input,1.f/fps,1000);
        const auto pose=surface.update(lib,world,t,result.motion,1.f/fps,1);
        if(frame<fps)continue;
        const auto authored=authoredCycle(lib,t,result.motion,surface.gaitPhase());
        for(int bone:{6,7,9,10,24,28,29,31,32})error=std::max(error,angleBetween(pose[bone].q,authored[bone].q));
    }
    return error;
}
struct SlopedWorld:World {
    fc_test::CornerWorld base;float slope{},flat{};
    explicit SlopedWorld(float value):slope(value),flat(std::sqrt(1-value*value)) {
        base.boxes.push_back({{-10000,0,-10000},{10000,1000,10000}});
    }
    Vec global(Vec p)const{return {p.x,flat*p.y+slope*p.z,-slope*p.y+flat*p.z};}
    Vec local(Vec p)const{return {p.x,flat*p.y-slope*p.z,slope*p.y+flat*p.z};}
    std::optional<Hit> ray(Vec a,Vec b)override {
        if(auto hit=base.ray(local(a),local(b)))return Hit{global(hit->point),global(hit->normal),hit->climbable};return {};
    }
};
void sloped(const Library& lib,int fps,float slope,Input input) {
    SlopedWorld world(slope);Traversal t;t.cfg=fc_test::settings();t.cfg.runSpeed=379.5f;t.cfg.wallRunObstacleJumps=true;
    t.cfg.contextActions=t.cfg.automaticClimbActions=false;
    require(t.attach(world,world.global({0,-37,100}),{0,1,0},1000),"sloped obstacle fixture really attaches to transformed collision");
    SurfacePose surface;Pose previous,previousWorld;Vec oldRoot=t.position;Motion last=Motion::none;const float dt=1.f/fps;
    float previousPhase=0,previousAlong=0,previousProgress=0;bool started=false,landed=false;unsigned tails=0,returns=0;
    auto tick=[&] {
        const auto result=t.update(world,input,dt,1000);
        require(t.active()&&!result.released,"sloped obstacle action retains its actual destination surface");
        const auto pose=surface.update(lib,world,t,result.motion,dt,1),body=lib.world(pose);
        require(lib.armBendValid(pose,0)&&lib.armBendValid(pose,1),"sloped returning kick keeps anatomical elbows");
        if(!previous.empty()) {
            for(unsigned bone=0;bone<pose.size();++bone)
                require(angleBetween(previous[bone].q,pose[bone].q)<=(kick(result.motion)||runMotion(result.motion)?18.849556f:12.566371f)*dt+.006f,
                    "actual sloped jump output keeps every bone in the established angular budget");
            for(int bone:{8,11,38,39}) {
                const float distance=(point(body[bone].t,t.position,t.normal)-point(previousWorld[bone].t,oldRoot,t.normal)).length();
                const float limit=(result.motion!=last&&!runMotion(last)?600.f:1100.f)*dt;
                require(distance<=limit+.03f,"actual sloped hand/foot travel stays inside existing output budget");
            }
        }
        if(kick(result.motion)) {
            started=true;
            if(kick(last)&&previousProgress>.48f) {
                const Motion target=std::abs(input.x)<.1f?Motion::runUp:input.y>.1f?Motion::runDiagonalRight:Motion::runRight;
                const auto& clip=lib.clip(target);
                const float advance=std::min((t.actionRouteDistance()-previousAlong)/clip.stride,dt*2.8f/std::max(.1f,clip.seconds));
                const float expected=std::fmod(previousPhase+advance,1.f);
                require(std::abs(expected-surface.gaitPhase())<.0002f,"sloped return phase uses full true route distance while excluding both jump arc offsets");
                ++tails;
            }
        } else if(started) {
            if(!landed) {
                const auto& clip=lib.clip(result.motion);
                const float expected=std::fmod(previousPhase+std::min((t.position-oldRoot).length()/clip.stride,dt*2.8f/std::max(.1f,clip.seconds)),1.f);
                require(std::abs(surface.gaitPhase()-expected)<.0002f,"slope landing preserves its progressing tail phase exactly once");landed=true;
            }
            ++returns;
        }
        previous=pose;previousWorld=body;oldRoot=t.position;last=result.motion;
        previousPhase=surface.gaitPhase();previousProgress=t.actionProgress();previousAlong=t.actionRouteDistance();
    };
    for(int frame=0;frame<fps;++frame)tick();
    const auto feet=world.local(t.position);
    const float protrusion=slope>.5f?18.f:30.f;
    if(input.y>.1f)world.base.boxes.push_back({{-10000,-protrusion,feet.z+500},{10000,20,feet.z+512}});
    else world.base.boxes.push_back({{feet.x+500,-protrusion,-10000},{feet.x+512,20,10000}});
    for(int frame=0;frame<fps*6&&returns<unsigned(fps/2);++frame)tick();
    if(!started||!landed)std::cerr<<"slope fixture fps="<<fps<<" z="<<slope<<" dir="<<input.x<<','<<input.y<<" root="<<t.position.x<<','<<t.position.y<<','<<t.position.z<<" reason="<<t.blockedReason<<'\n';
    require(started&&landed&&tails>2,"sloped wall really exercises the new automatic kick and its advancing recovery");
    std::cout<<"slope fps="<<fps<<" normalZ="<<slope<<" direction="<<input.x<<','<<input.y<<" tailFrames="<<tails<<'\n';
}
void exercise(const Library& lib,int fps,Input input,bool releaseShift) {
    fc_test::CornerWorld world;world.boxes.push_back({{-10000,0,-10000},{10000,1000,10000}});
    Traversal t;t.cfg=fc_test::settings();t.cfg.runSpeed=379.5f;t.cfg.wallRunObstacleJumps=true;
    t.cfg.contextActions=t.cfg.automaticClimbActions=false;
    require(t.attach(world,{0,-37,100},{0,1,0},1000),"obstacle output test attaches to actual wall");
    SurfacePose surface,isolated;NoPoseContacts noContacts;Pose previous,previousWorld,lastPlain;Vec oldPosition=t.position,oldNormal=t.normal;
    Motion last=Motion::none;float lastPhase=0,dt=1.f/fps,maxAngleRate=0,maxEndpointSpeed=0,sourceLegMotion=0;
    auto recoveredInput=input;recoveredInput.run=!releaseShift;
    float baselineError=steadyError(lib,fps,recoveredInput),recoveryError=0,isolatedError=0,lateLegMotion=0,actionElapsed=0;
    unsigned actionFrames=0,returnFrames=0;bool started=false,landed=false,shiftReleased=false,phaseInherited=false,settling=false;
    const bool writeSamples=bool(dump)&&fps==60&&!releaseShift&&input.x>=0;
    const char* caseName=std::abs(input.x)<.1f?"up":input.y>.1f?"diagonal":"side";
    constexpr float sampleAt[]={0,.1f,.3f,.5f,.7f,.9f,1.f};unsigned sampleIndex=0;
    auto tick=[&](Input controls) {
        const auto result=t.update(world,controls,dt,1000);
        require(t.active()&&!result.released,"supported automatic kick stays attached through live path and landing");
        const auto pose=surface.update(lib,world,t,result.motion,dt,1),body=lib.world(pose);
        const auto plain=isolated.update(lib,noContacts,t,result.motion,dt,1);
        require(pose.size()==99&&lib.armBendValid(pose,0)&&lib.armBendValid(pose,1),"complete output retains legal elbows through rapid kick and recovery");
        for(unsigned bone=0;bone<pose.size();++bone) {
            require(pose[bone].t.finite()&&std::abs(pose[bone].q.dot(pose[bone].q)-1)<.003f,"all kick output transforms stay finite normalized");
            if(!previous.empty()) {
                const float angle=angleBetween(previous[bone].q,pose[bone].q);
                maxAngleRate=std::max(maxAngleRate,angle/dt);
                const float limit=(kick(result.motion)||runMotion(result.motion)?18.849556f:12.566371f)*dt;
                if(angle>limit+.006f)std::cerr<<"angle fps="<<fps<<" direction="<<input.x<<','<<input.y<<" from="<<int(last)<<" to="<<int(result.motion)<<" bone="<<bone<<" angle="<<angle<<" limit="<<limit<<'\n';
                require(angle<=limit+.006f,"all 99 actual output bones retain the existing angular speed budget");
            }
        }
        if(!previous.empty()) {
            float distance=0;
            for(int bone:{8,11,38,39})distance=std::max(distance,(point(body[bone].t,t.position,t.normal)-point(previousWorld[bone].t,oldPosition,oldNormal)).length());
            maxEndpointSpeed=std::max(maxEndpointSpeed,distance/dt);
            const float speed=runMotion(result.motion)||kick(result.motion)||runMotion(last)?1100.f:600.f;
            const float limit=(result.motion!=last?std::min(speed,runMotion(last)?1100.f:600.f):speed)*dt;
            if(distance>limit+.03f)std::cerr<<"endpoint fps="<<fps<<" direction="<<input.x<<','<<input.y<<" phase="<<t.actionProgress()<<" from="<<int(last)<<" to="<<int(result.motion)<<" distance="<<distance<<" limit="<<limit<<'\n';
            require(distance<=limit+.03f,"short jump cannot evade the unchanged measured hand/foot world travel budget");
            float legs=0;for(int bone:{6,7,9,10})legs+=angleBetween(previous[bone].q,pose[bone].q);
            if(kick(result.motion))sourceLegMotion+=legs;
            if(landed&&returnFrames>unsigned(fps/3))lateLegMotion+=legs;
        }
        if(world.clearance(t.position,t.cfg)<t.cfg.radius-.08f)std::cerr<<"clearance fps="<<fps<<" direction="<<input.x<<','<<input.y<<" phase="<<t.actionProgress()<<" motion="<<int(result.motion)<<" root="<<t.position.x<<','<<t.position.y<<','<<t.position.z<<" clearance="<<world.clearance(t.position,t.cfg)<<'\n';
        require(world.clearance(t.position,t.cfg)>=t.cfg.radius-.08f,"independent collider distance remains valid while animating the kick");
        if(kick(result.motion)) {
            if(writeSamples&&!started)sample(caseName,"before",-1,last,oldPosition,oldNormal,previousWorld);
            started=true;++actionFrames;actionElapsed+=dt;
            if(writeSamples&&sampleIndex<std::size(sampleAt)&&t.actionProgress()+.00001f>=sampleAt[sampleIndex]) {
                sample(caseName,"jump",t.actionProgress(),result.motion,t.position,t.normal,body);++sampleIndex;
            }
            require(surface.runFootContacts==0,"airborne kick never pins either foot to a stale running contact");
            require(result.motion==(std::abs(input.x)<.1f?Motion::kickUp:input.x<0?Motion::kickLeft:Motion::kickRight),"automatic jump uses the actual directional captured kick");
        } else if(started) {
            if(!landed) {
                landed=true;
                {
                    const auto& clip=lib.clip(result.motion);const float expected=std::fmod(lastPhase+std::min((t.position-oldPosition).length()/clip.stride,dt*2.8f/std::max(.1f,clip.seconds)),1.f);
                    require(std::abs(surface.gaitPhase()-expected)<.0002f,"landing inherits the advancing recovery phase without rematching/resetting its cycle");
                    phaseInherited=true;
                }
            }
            ++returnFrames;
            if(writeSamples&&(returnFrames==unsigned(fps/10)||returnFrames==unsigned(fps/3)))
                sample(caseName,"return",float(returnFrames)/fps,result.motion,t.position,t.normal,body);
            if(!settling)require(releaseShift?!runMotion(result.motion):runMotion(result.motion),"held Shift lands running; released Shift lands into ordinary climbing");
        }
        if(!kick(result.motion)) {
            const auto target=authoredCycle(lib,t,result.motion,surface.gaitPhase());float error=0;
            for(int bone:{6,7,9,10,24,28,29,31,32})error=std::max(error,angleBetween(pose[bone].q,target[bone].q));
            if(!settling&&landed&&returnFrames>unsigned(fps/3)) {
                recoveryError=std::max(recoveryError,error);
                require(angleBetween(pose[0].q,target[0].q)<.025f,"after recovery the body frame has fully returned from the airborne orientation");
                const auto unpinnedTarget=authoredCycle(lib,t,result.motion,isolated.gaitPhase());
                for(unsigned bone=0;bone<plain.size();++bone)isolatedError=std::max(isolatedError,angleBetween(plain[bone].q,unpinnedTarget[bone].q));
            }
        }
        const auto held=surface.update(lib,world,t,result.motion,0,1);
        for(unsigned bone=0;bone<pose.size();++bone)
            require(angleBetween(held[bone].q,pose[bone].q)<.00001f&&(held[bone].t-pose[bone].t).length()<.00001f,"duplicate output callback cannot restart a jump or landing blend");
        lastPlain=plain;previous=pose;previousWorld=body;oldPosition=t.position;oldNormal=t.normal;last=result.motion;lastPhase=surface.gaitPhase();
    };
    for(int frame=0;frame<fps;++frame)tick(input);
    const auto start=t.position;
    if(input.y>.1f)world.boxes.push_back({{-10000,-30,start.z+500},{10000,20,start.z+512}});
    else if(input.x>0)world.boxes.push_back({{start.x+500,-30,-10000},{start.x+512,20,10000}});
    else world.boxes.push_back({{start.x-512,-30,-10000},{start.x-500,20,10000}});
    for(int frame=0;frame<fps*5&&returnFrames<unsigned(fps);++frame) {
        if(started&&!shiftReleased&&releaseShift&&t.actionProgress()>.38f){input.run=false;shiftReleased=true;}
        tick(input);
    }
    require(started&&landed&&t.obstacleJumpCount()==1,"real obstacle produces exactly one complete automatic running jump");
    require(actionElapsed<=t.actionDuration()+2*dt+.0001f&&actionElapsed>=t.actionDuration(),"faster action completes on schedule without a held recovery pose");
    require(sourceLegMotion>.5f&&lateLegMotion>.5f,"captured kick and recovered locomotion both keep advancing actual legs");
    require(phaseInherited,"completed action returns with its already advancing phase");
    settling=true;tick({});
    SurfacePose reference;const auto resting=reference.update(lib,noContacts,t,Motion::hang,dt,1);
    const Vec fixed=t.position;
    for(int frame=0;frame<fps;++frame)tick({});
    require((t.position-fixed).length()<.001f&&last==Motion::hang,"stopping after the landing holds real supported rest without actor drift");
    float restAngle=0,restTranslation=0;
    for(unsigned bone=0;bone<lastPlain.size();++bone) {
        restAngle=std::max(restAngle,angleBetween(lastPlain[bone].q,resting[bone].q));
        restTranslation=std::max(restTranslation,(lastPlain[bone].t-resting[bone].t).length());
    }
    require(restAngle<.035f&&restTranslation<.05f,"all 99 bones finish returning after the landing instead of freezing a partially blended kick");
    std::cout<<"kick pose fps="<<fps<<" direction="<<input.x<<','<<input.y<<" releaseShift="<<releaseShift<<" seconds="<<actionElapsed
        <<" angleRate="<<maxAngleRate<<" endpointSpeed="<<maxEndpointSpeed<<" baseline="<<baselineError<<" recovery="<<recoveryError<<'\n';
    std::cout<<"unPinnedCycleError="<<isolatedError<<" restAngle="<<restAngle<<" restTranslation="<<restTranslation<<'\n';
}
}
int main(int argc,char** argv){try {
    Library lib;require((argc==2||argc==3)&&lib.load(argv[1]),"load actual complete motion library");
    if(argc==3) {dump.open(argv[2]);require(bool(dump),"open requested diagnostic pose sample file");dump<<"{\"parents\":[";for(unsigned bone=0;bone<lib.parents.size();++bone){if(bone)dump<<',';dump<<lib.parents[bone];}dump<<"],\"samples\":[";}
    int failures=0;
    for(int fps:{30,60,120})for(Input input:{Input{0,1,false,false,false,false,true},Input{1,0,false,false,false,false,true},
        Input{-1,0,false,false,false,false,true},Input{1,1,false,false,false,false,true},Input{-1,1,false,false,false,false,true}})
        for(bool releaseShift:{false,true})try {exercise(lib,fps,input,releaseShift);}catch(const std::exception& error) {
            ++failures;std::cerr<<"FAIL fps="<<fps<<" direction="<<input.x<<','<<input.y<<" releaseShift="<<releaseShift<<": "<<error.what()<<'\n';
        }
    for(int fps:{30,60,120})for(float slope:{.3f,.6f})for(Input input:{Input{0,1,false,false,false,false,true},Input{1,0,false,false,false,false,true},Input{1,1,false,false,false,false,true}})
        try {sloped(lib,fps,slope,input);}catch(const std::exception& error) {
            ++failures;std::cerr<<"FAIL sloped fps="<<fps<<" normalZ="<<slope<<" direction="<<input.x<<','<<input.y<<": "<<error.what()<<'\n';
        }
    if(dump)dump<<"]}";
    if(failures)return 1;
    std::cout<<"PASS: real Core/SurfacePose obstacle run-kick-run/climb, directional phase carry, no foot pin, complete bone/endpoint budgets at 30/60/120 FPS\n";
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
