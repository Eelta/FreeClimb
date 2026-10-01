#ifdef _WIN32
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#undef far
#undef near
#endif
#include "Pose.h"
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace fc;
static void require(bool b,const char* message){if(!b)throw std::runtime_error(message);}
struct Summary {
    float palm{},finger{},foot{},earlyExtra{},angle{},speed{};
    unsigned coreRayPeak{},poseRayPeak{},ordinaryRayPeak{};
    double coreSeconds{},poseSeconds{};
} summary;
struct CrestWorld:World {
    struct Plane{Vec n;float d;};using Solid=std::vector<Plane>;
    std::vector<Solid> solids;float nz,nx,farZ,farX,skew{},yaw{};Vec origin{};unsigned calls{};
    CrestWorld(float z,float other=-1,float angle=0):nz(z),nx(std::sqrt(1-z*z)),farZ(other<0?z:other),farX(std::sqrt(1-farZ*farZ)),skew(angle) {
        solids={{{{1,0,0},300},{{-1,0,0},300},{{0,1,0},300},{{0,-1,0},300},{{0,0,-1},1000},{{nx,0,nz},0},{{-farX*std::cos(skew),farX*std::sin(skew),farZ},0}}};
    }
    Vec rotate(Vec p,float angle)const{return {p.x*std::cos(angle)-p.y*std::sin(angle),p.x*std::sin(angle)+p.y*std::cos(angle),p.z};}
    Vec global(Vec p)const{return origin+rotate(p,yaw);}
    Vec local(Vec p)const{return rotate(p-origin,-yaw);}
    std::optional<Hit> ray(Vec from,Vec to)override {
        ++calls;const Vec a=local(from),d=local(to)-a;double best=2;std::optional<Hit> result;
        for(const auto& solid:solids){double enter=0,leave=1;Vec normal{};bool invalid=false;
            for(auto p:solid){const double dist=a.dot(p.n)-p.d,rate=d.dot(p.n);
                if(std::abs(rate)<1e-9){if(dist>0){invalid=true;break;}continue;}
                const double t=-dist/rate;if(rate<0){if(t>enter){enter=t;normal=p.n;}}else leave=std::min(leave,t);
                if(leave<enter){invalid=true;break;}}
            if(invalid||enter<=1e-6||enter>1||enter>=best||normal.length()<.5f)continue;
            const auto p=a+d*float(enter);
            best=enter;result=Hit{global(p),rotate(normal,yaw),true};
        }return result;
    }
};
static Traversal start(CrestWorld& w) {
    Traversal t;t.cfg.gap=37;t.cfg.radius=31;t.cfg.height=138;t.cfg.approachSeconds=0;
    const Vec feet{37-w.nz*(-200+6)/w.nx,0,-200};
    require(t.attach(w,w.global(feet),w.rotate({-1,0,0},w.yaw),1000),"approach starts on an actual supported sloping roof");return t;
}
static float depthAt(const CrestWorld& w,Vec point) {
    point=w.local(point);float result=0;
    for(const auto& solid:w.solids){float d=10000;for(const auto& plane:solid)d=std::min(d,plane.d-point.dot(plane.n));result=std::max(result,d);}
    return result;
}
static bool runPose(const Library& lib,float z,int fps,bool distant,float scale) {
    CrestWorld w(z,z>.68f?.626594f:-1);w.yaw=.633f;if(distant)w.origin={131316.86f,38643.36f,-11331.41f};
    auto t=start(w);require(lib.configureThreepeat(t.cfg),"runtime calibration");t.cfg.threepeatAnimations=true;t.cfg.contextScale=scale;
    SurfacePose surface;Pose previous;std::array<Vec,4> lastEnds{};Motion prior=Motion::none;float dt=1.f/fps;
    float finger=0,foot=0,palm=0,angular=0,speed=0,wrist=0,onset=0,fingerPhase=0,footPhase=0;
    float allFinger=0,allFingerPhase=0,allFingerPrep=0,earlyExtra=0,earlyPhase=0,earlyPrep=0,entryFinger=0;
    int allFingerBone=-1,allFingerJoint=-1,earlyBone=-1,earlyJoint=-1;Vec allFingerPoint{},earlyPoint{};
    std::array<float,40> priorFingers{},entryFingers{};unsigned corePeak=0,surfacePeak=0,ordinaryPeak=0;
    double coreSeconds=0,surfaceSeconds=0;
    float preFoot=0;std::array<float,4> stageFeet{},stageFingers{},stagePalms{};int fingerBone=-1,footBone=-1;bool selected=false,done=false,release=false;unsigned bends=0;
    auto point=[&](Vec p){return t.position+(Vec{-t.normal.y,t.normal.x,0}*p.x-t.normal*p.y+Vec{0,0,p.z})*scale;};
    for(int frame=-10;frame<fps*8&&t.active();++frame) {
        const auto coreStart=std::chrono::steady_clock::now();const auto beforeCore=w.calls;
        Result r;if(frame<0)r.motion=Motion::hang;else r=t.update(w,{0,1,false,true},dt,1000);
        coreSeconds+=std::chrono::duration<double>(std::chrono::steady_clock::now()-coreStart).count();corePeak=std::max(corePeak,w.calls-beforeCore);
        const auto surfaceStart=std::chrono::steady_clock::now();const auto beforeSurface=w.calls;
        const auto root=t.position;const auto pose=surface.update(lib,w,t,r.motion,dt,scale);require((t.position-root).length()==0,"pose cannot alter physical root");
        surfaceSeconds+=std::chrono::duration<double>(std::chrono::steady_clock::now()-surfaceStart).count();
        auto& rayPeak=r.motion==Motion::contextMantle?surfacePeak:ordinaryPeak;rayPeak=std::max(rayPeak,w.calls-beforeSurface);
        if(r.motion==Motion::contextMantle&&!selected){entryFingers=priorFingers;for(float d:entryFingers)entryFinger=std::max(entryFinger,d);}
        require(pose.size()==99,"full pose required");const auto body=lib.world(pose);std::array<Vec,4> ends{};
        for(unsigned i=0;i<4;++i)ends[i]=point(body[std::array{8,11,38,39}[i]].t);
        const int stage=r.motion!=Motion::contextMantle?0:t.topPreparation()<1?1:
            std::max(t.topHandWeight(0,surface.sampledPhase()),t.topHandWeight(1,surface.sampledPhase()))>.05f?2:3;
        for(int bone:{8,11,50,51})stageFeet[stage]=std::max(stageFeet[stage],depthAt(w,point(body[bone].t)));
        for(int hand=0;hand<2;++hand) {
            if(stage!=0&&t.topHandWeight(hand,surface.sampledPhase())>.95f)
                stagePalms[stage]=std::max(stagePalms[stage],(point(lib.palm(body,hand))-(t.topHand(hand)+t.topHandNormal(hand)*(.8f*scale))).length());
            for(int digit=0;digit<5;++digit)for(int joint=0;joint<4;++joint) {
                const int bone=(hand?82:67)+digit*3+std::min(joint,2);Vec value=body[bone].t;
                if(joint==3)value=value+body[bone].q.rotate({0,0,lib.rest[bone].t.length()*.75f});
                const Vec actual=point(value);const float depth=depthAt(w,actual);const int index=hand*20+digit*4+joint;
                stageFingers[stage]=std::max(stageFingers[stage],depth);
                if(r.motion==Motion::contextMantle) {
                    if(t.topPreparation()>=.75f&&depth>allFinger){allFinger=depth;allFingerBone=bone;allFingerJoint=joint;allFingerPhase=surface.sampledPhase();allFingerPrep=t.topPreparation();allFingerPoint=w.local(actual);}
                    if(t.topPreparation()<.75f&&depth-entryFingers[index]>earlyExtra){earlyExtra=depth-entryFingers[index];earlyBone=bone;earlyJoint=joint;earlyPhase=surface.sampledPhase();earlyPrep=t.topPreparation();earlyPoint=w.local(actual);}
                }
                priorFingers[index]=depth;
            }
        }
        if(r.motion!=Motion::contextMantle&&!selected)for(int bone:{8,11,50,51})preFoot=std::max(preFoot,depthAt(w,point(body[bone].t)));
        if(r.motion==Motion::contextMantle) {
            if(!selected){selected=true;onset=t.topLip().z-t.topStart().z;}
            for(std::size_t bone=0;bone<pose.size();++bone) {
                require(pose[bone].t.finite()&&std::abs(pose[bone].q.dot(pose[bone].q)-1)<.002f,"finite unit pose");
                if(bone!=0&&bone!=4)require(std::abs(pose[bone].t.length()-lib.rest[bone].t.length())<.002f,"fixed bone lengths");
                if(!previous.empty())angular=std::max(angular,angleBetween(previous[bone].q,pose[bone].q)-12.566371f*dt);
            }
            if(!previous.empty())for(unsigned i=0;i<4;++i)speed=std::max(speed,(ends[i]-lastEnds[i]).length()-(r.motion!=prior?600.f:750.f)*dt*scale);
            for(int hand=0;hand<2;++hand) {
                if(!lib.armBendValid(pose,hand))++bends;
                const int elbow=hand?32:29,wr=hand?39:38,middle=hand?88:73;
                wrist=std::max(wrist,std::acos(std::clamp((body[wr].t-body[elbow].t).unit().dot((body[middle].t-body[wr].t).unit()),-1.f,1.f)));
                if(t.topPreparation()<.75f||t.topHandWeight(hand,surface.sampledPhase())<=.95f)continue;
                if(t.topPreparation()>=.99f)palm=std::max(palm,(point(lib.palm(body,hand))-(t.topHand(hand)+t.topHandNormal(hand)*(.8f*scale))).length());
                for(int digit=0;digit<5;++digit)for(int joint=0;joint<4;++joint) {
                    const int bone=(hand?82:67)+digit*3+std::min(joint,2);Vec value=body[bone].t;
                    if(joint==3)value=value+body[bone].q.rotate({0,0,lib.rest[bone].t.length()*.75f});
                    const auto depth=depthAt(w,point(value));if(depth>finger){finger=depth;fingerBone=bone;fingerPhase=surface.sampledPhase();}
                }
            }
            for(int bone:{8,11,50,51}){const auto depth=depthAt(w,point(body[bone].t));if(depth>foot){foot=depth;footBone=bone;footPhase=surface.sampledPhase();}}
        }
        previous=pose;lastEnds=ends;prior=r.motion;done|=r.completed;release|=r.released&&!r.completed;
    }
    const bool pass=selected&&done&&!release&&onset>=70&&palm<5*scale&&finger<=.15001f*scale&&allFinger<=.15001f*scale&&earlyExtra<=.15001f*scale&&foot<=.15001f*scale&&angular<=.006f&&speed<=.03f&&bends==0&&wrist<=1.659063f;
    summary.palm=std::max(summary.palm,palm/scale);summary.finger=std::max(summary.finger,allFinger/scale);
    summary.foot=std::max(summary.foot,foot/scale);summary.earlyExtra=std::max(summary.earlyExtra,earlyExtra/scale);
    summary.angle=std::max(summary.angle,angular);summary.speed=std::max(summary.speed,speed);
    summary.coreRayPeak=std::max(summary.coreRayPeak,corePeak);summary.poseRayPeak=std::max(summary.poseRayPeak,surfacePeak);
    summary.ordinaryRayPeak=std::max(summary.ordinaryRayPeak,ordinaryPeak);summary.coreSeconds+=coreSeconds;summary.poseSeconds+=surfaceSeconds;
    if(!pass)for(int stage=0;stage<4;++stage)
        std::cerr<<"segment="<<stage<<" feet="<<stageFeet[stage]<<" allFingers="<<stageFingers[stage]<<" loadedPalm="<<stagePalms[stage]<<'\n';
    if(!pass)std::cerr<<"crest-pose z="<<z<<" fps="<<fps<<" distant="<<distant<<" scale="<<scale<<" preFoot="<<preFoot<<" onset="<<onset<<" done="<<done<<" release="<<release<<" palm="<<palm<<" finger="<<finger<<" fingerBone="<<fingerBone<<" fingerPhase="<<fingerPhase<<" foot="<<foot<<" footBone="<<footBone<<" footPhase="<<footPhase<<" angleExcess="<<angular<<" endpointExcess="<<speed<<" bends="<<bends<<" wrist="<<wrist<<" allFinger="<<allFinger<<" allBone="<<allFingerBone<<" allJoint="<<allFingerJoint<<" allPhase="<<allFingerPhase<<" allPrep="<<allFingerPrep<<" allPoint="<<allFingerPoint.x<<","<<allFingerPoint.y<<","<<allFingerPoint.z<<" entryFinger="<<entryFinger<<" earlyExtra="<<earlyExtra<<" earlyBone="<<earlyBone<<" earlyJoint="<<earlyJoint<<" earlyPhase="<<earlyPhase<<" earlyPrep="<<earlyPrep<<" earlyPoint="<<earlyPoint.x<<","<<earlyPoint.y<<","<<earlyPoint.z<<" coreRayPeak="<<corePeak<<" crestPoseRayPeak="<<surfacePeak<<" ordinaryPoseRayPeak="<<ordinaryPeak<<" coreOfflineSeconds="<<coreSeconds<<" poseOfflineSeconds="<<surfaceSeconds<<" pass="<<pass<<'\n';return pass;
}
int main(int argc,char** argv)try {
#ifdef _WIN32
    SetErrorMode(SEM_FAILCRITICALERRORS|SEM_NOGPFAULTERRORBOX);
#endif
    require(argc==2,"runtime manifest argument");Library library;require(library.load(argv[1]),"runtime real HKX load");unsigned failed=0,total=0;
    for(float z:{.35f,.491f,.502f,.626594f,.69f})for(int fps:{30,60,120})for(bool distant:{false,true})for(float scale:{1.f,1.03f}){++total;if(!runPose(library,z,fps,distant,scale))++failed;}
    std::cout<<"roof-crest-pose total="<<total<<" failures="<<failed<<" maxPalm="<<summary.palm<<" allFinger="<<summary.finger
        <<" foot="<<summary.foot<<" earlyExtra="<<summary.earlyExtra<<" angleExcess="<<summary.angle<<" endpointExcess="<<summary.speed
        <<" coreRayPeak="<<summary.coreRayPeak<<" poseRayPeak="<<summary.poseRayPeak<<" ordinaryRayPeak="<<summary.ordinaryRayPeak
        <<" coreOfflineSeconds="<<summary.coreSeconds<<" poseOfflineSeconds="<<summary.poseSeconds<<'\n';return failed?1:0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
