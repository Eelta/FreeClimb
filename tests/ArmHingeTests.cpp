#include "Pose.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
static unsigned checks{};
static void require(bool value,const char* text){++checks;if(!value)throw std::runtime_error(text);}
static Vec physicalPlane(const Library& lib,const Pose& pose,int hand){
    const auto body=lib.world(pose);const int upper=hand?31:28,elbow=hand?32:29,wrist=hand?39:38;
    return body[upper].q.inverse().rotate((body[elbow].t-body[upper].t).cross(body[wrist].t-body[elbow].t)).unit();
}
static float sourcePlaneDeparture(const Library& lib,const Pose& pose,int hand){
    const int elbow=hand?32:29,wrist=hand?39:38;
    return std::asin(std::clamp(std::abs(pose[elbow].q.rotate(pose[wrist].t).unit().dot(lib.armBends[hand].normal)),0.f,1.f));
}
static float formerIK(const Library& lib,Pose& p,int a,int b,int c,Vec target,Vec pole,bool guard=true){
    const int hand=a==28?0:1;if(guard)lib.guardArmBend(p,hand);
    auto body=lib.world(p);Vec start=body[a].t,mid=body[b].t,end=body[c].t;
    const float upper=(mid-start).length(),lower=(end-mid).length();const Vec axis=(target-start).unit();
    const float distance=std::clamp((target-start).length(),std::abs(upper-lower)+.02f,upper+lower-.04f);
    Vec plane=pole-start;plane=plane-axis*plane.dot(axis);
    if(plane.length()<.01f){plane=axis.cross({0,1,0});if(plane.length()<.01f)plane=axis.cross({1,0,0});}
    const float along=(upper*upper-lower*lower+distance*distance)/(2*distance);
    const float radial=std::sqrt(std::max(0.f,upper*upper-along*along));
    const auto originalA=p[a].q,originalB=p[b].q,upperWorld=body[a].q;
    auto solve=[&](float side){p[a].q=originalA;p[b].q=originalB;const Vec joint=start+axis*along+plane.unit()*(radial*side);
        lib.rotateWorld(p,a,Quat::between(mid-start,joint-start)*upperWorld);body=lib.world(p);
        lib.rotateWorld(p,b,Quat::between(body[c].t-body[b].t,start+axis*distance-body[b].t)*body[b].q);};
    solve(1);if(guard&&!lib.armBendValid(p,hand)){solve(-1);if(!lib.armBendValid(p,hand))lib.guardArmBend(p,hand);}
    return (target-lib.world(p)[c].t).length();
}
static Library customized(const Library& base,int variant){
    auto library=base;auto native=base.rest;std::array<Transform,99> basis{};std::array<bool,99> mapped{};mapped.fill(true);
    for(int bone:{1,2,3,42,43,46,47,48,49,60,61,62,63,64,65,66,97,98})mapped[bone]=false;
    if(variant==1){for(int bone:{29,32})native[bone].t=native[bone].t*.817401f;for(int bone:{38,39})native[bone].t=native[bone].t*.934444f;
        for(int bone:{28,31})native[bone].s={1.1f,.9f,1.03f};}
    if(variant==2){for(int bone:{29,32}){native[bone].t=native[bone].t*.997739f;basis[bone].q=Quat::axis({1,2,3},-.333003f);}
        for(int bone:{38,39})native[bone].t=native[bone].t*.993491f;}
    require(library.configureRig(native,basis,mapped),"supported native proportions and adjustment nodes configure");return library;
}
static void directIK(const Library& base){
    unsigned cases=0,oldFailures=0;float maximumError=0,maximumPlaneChange=0,maximumOldPlaneChange=0;
    for(int variant=0;variant<3;++variant){const auto library=customized(base,variant);
        for(Motion motion:{Motion::hang,Motion::up,Motion::down,Motion::left,Motion::right})for(float phase:{0.f,.17f,.37f,.65f,.91f})for(int hand:{0,1})
        for(Vec offset:{Vec{},Vec{4,0,0},Vec{-4,0,0},Vec{0,-4,0},Vec{0,-9,3},Vec{0,3,4}}){
            const int upper=hand?31:28,elbow=hand?32:29,wrist=hand?39:38;const auto source=library.sample(motion,phase),body=library.world(source);
            const Vec target=body[wrist].t+body[upper].q.rotate(offset),pole=body[elbow].t;
            const float a=(body[elbow].t-body[upper].t).length(),b=(body[wrist].t-body[elbow].t).length(),distance=(target-body[upper].t).length();
            if(distance<std::abs(a-b)+.03f||distance>a+b-.05f||physicalPlane(library,source,hand).length()<.9f)continue;
            auto result=source,former=source,external=source,oldExternal=source;
            const float error=library.ik(result,upper,elbow,wrist,target,pole);formerIK(library,former,upper,elbow,wrist,target,pole);
            library.ik(external,upper,elbow,wrist,target,pole,false);formerIK(library,oldExternal,upper,elbow,wrist,target,pole,false);
            const float changed=std::acos(std::clamp(physicalPlane(library,source,hand).dot(physicalPlane(library,result,hand)),-1.f,1.f));
            const float oldChanged=std::acos(std::clamp(physicalPlane(library,source,hand).dot(physicalPlane(library,former,hand)),-1.f,1.f));
            if(oldChanged>.1f)++oldFailures;
            if(changed>=.003f||error>=.02f)std::cerr<<"IK variant="<<variant<<" motion="<<int(motion)<<" phase="<<phase<<" hand="<<hand<<" plane="<<changed<<" error="<<error<<'\n';
            require(changed<.003f,"the actual scaled elbow plane remains aligned with its source");
            require(error<.02f,"hinge preservation keeps the reachable contact rather than rejecting it");
            require(library.armBendValid(result,hand),"existing reverse-bend guard remains satisfied");
            const auto solved=library.world(result);
            require(std::abs((solved[elbow].t-solved[upper].t).length()-a)<.002f&&std::abs((solved[wrist].t-solved[elbow].t).length()-b)<.002f,"both physical arm lengths remain unchanged");
            for(std::size_t bone=0;bone<result.size();++bone){
                if(int(bone)!=upper&&int(bone)!=elbow)require(angleBetween(source[bone].q,result[bone].q)<.000001f&&(source[bone].t-result[bone].t).length()==0,"IK preserves wrist, fingers, helper tracks and the other arm");
                require(angleBetween(external[bone].q,oldExternal[bone].q)<.00001f&&(external[bone].t-oldExternal[bone].t).length()==0,"external authored IK retains the previous calculation");
            }
            if(offset.length()==0)for(int bone:{upper,elbow})require(angleBetween(source[bone].q,result[bone].q)<.00003f,"already reached targets do not add shoulder roll or erase forearm twist");
            maximumError=std::max(maximumError,error);maximumPlaneChange=std::max(maximumPlaneChange,changed);maximumOldPlaneChange=std::max(maximumOldPlaneChange,oldChanged);++cases;
        }
    }
    require(cases>500&&oldFailures>100,"negative controls expose repeated sideways elbow deviation in the former solver");
    std::cout<<"direct IK cases="<<cases<<" formerFailures="<<oldFailures<<" maxPlane="<<maximumPlaneChange<<" formerPlane="<<maximumOldPlaneChange<<" maxReachError="<<maximumError<<'\n';
}
static void singularArms(const Library& base){
    unsigned cases=0;
    for(int variant=0;variant<3;++variant){const auto library=customized(base,variant);
        for(int hand:{0,1})for(float folding:{0.f,.00001f,.002f,3.13959f,3.14159265f}){
            const int upper=hand?31:28,elbow=hand?32:29,wrist=hand?39:38;
            auto source=library.sample(Motion::hang,0);auto body=library.world(source);
            const auto humerus=(body[elbow].t-body[upper].t).unit();
            const auto normal=body[upper].q.rotate(physicalPlane(library,source,hand));
            const auto direction=Quat::axis(normal,folding).rotate(humerus);
            library.rotateWorld(source,elbow,Quat::between(body[wrist].t-body[elbow].t,direction)*body[elbow].q);
            body=library.world(source);const float a=(body[elbow].t-body[upper].t).length(),b=(body[wrist].t-body[elbow].t).length();
            for(float side:{-1.f,1.f}){
                auto result=source;const auto target=body[upper].t+(humerus+normal.cross(humerus)*(.02f*side)).unit()*(a+b-1);
                const float error=library.ik(result,upper,elbow,wrist,target,body[elbow].t);
                require(std::isfinite(error)&&error<.001f,"near-straight and folded arms retain finite reachable output");
                require(library.armBendValid(result,hand),"singular source poses retain the reverse-bend protection");
                for(const auto& bone:result)require(bone.t.finite()&&bone.s.finite()&&std::isfinite(bone.q.dot(bone.q))&&std::abs(bone.q.dot(bone.q)-1)<.001f,"no singular plane produces invalid transforms");
                const auto solved=library.world(result);
                require(std::abs((solved[elbow].t-solved[upper].t).length()-a)<.002f&&std::abs((solved[wrist].t-solved[elbow].t).length()-b)<.002f,"singular source handling does not resize either arm segment");++cases;
            }
        }
    }
    std::cout<<"singular arm cases="<<cases<<'\n';
}
struct ReliefWall:World{
    Vec normal{0,-1,0},point{0,0,106};float relief{};
    std::optional<Hit> ray(Vec a,Vec b)override{
        std::optional<Hit> best;float closest=2;
        for(int side=0;side<3;++side){const Vec plane=point+normal*(side==0?0:relief*(side==1?1:-1));const float from=(a-plane).dot(normal),to=(b-plane).dot(normal);
            if(from<=0||to>=0)continue;const float at=from/(from-to);const Vec hit=a+(b-a)*at;const int region=hit.x<-12?1:hit.x>12?2:0;
            if(region!=side||at>=closest)continue;closest=at;best=Hit{hit,normal,true};}return best;
    }
};
static void stoppedArm(const Library& base,int variant,int fps){
    const auto library=customized(base,variant);ReliefWall world;const float slope=variant==1?.0000860477f:.0200129f;
    world.normal={0,-std::sqrt(1-slope*slope),slope};world.relief=variant==1?-2.01319f:-6.88021f;
    Traversal traversal;traversal.cfg.gap=37;traversal.cfg.radius=31;traversal.cfg.height=138;traversal.cfg.automaticClimbActions=false;
    traversal.state=State::wall;traversal.position={0,-37,100};traversal.normal={0,-1,0};traversal.surfaceNormal=world.normal;
    SurfacePose surface;const float dt=1.f/fps;const int moving=std::max(1,int(std::round((variant==1?17.f/120:13.f/30)*fps)));
    Pose previous;float worst=0,step=0,settled=0;int contacts=0;
    for(int frame=0;frame<moving+fps*3;++frame){const Motion motion=frame<moving?(variant==1?Motion::left:Motion::right):Motion::hang;
        if(frame<moving){const Vec delta{(variant==1?-1.f:1.f)*dt*80,0,0};traversal.position=traversal.position+delta;world.point=world.point+delta;}
        const auto pose=surface.update(library,world,traversal,motion,dt,1);
        require(traversal.active(),"visual arm constraints never release physical wall support");contacts+=surface.contactCount;
        for(int hand:{0,1}){require(library.armBendValid(pose,hand),"moving and stopped output keeps legal elbow branches");
            const float departure=sourcePlaneDeparture(library,pose,hand);if(frame>=moving)worst=std::max(worst,departure);if(frame>=moving+fps)settled=std::max(settled,departure);}
        if(!previous.empty())for(int bone:{28,29,31,32}){const float change=angleBetween(previous[bone].q,pose[bone].q);step=std::max(step,change);
            require(change<=12.566371f*dt+.0001f,"hinge-preserving contact does not introduce an unbounded shoulder or elbow step");}
        previous=pose;
    }
    require(worst<.70f&&settled<.55f,"ordinary stopped arms do not inherit the former sideways contact bend");
    require(contacts>fps,"corrected arm output retains real wall contacts");
    std::cout<<"stop variant="<<variant<<" fps="<<fps<<" maxDeparture="<<worst<<" settled="<<settled<<" maxStep="<<step<<" contacts="<<contacts<<'\n';
}
int main(int argc,char**argv)try{require(argc==2,"runtime animation pack required");Library base;require(base.load(argv[1]),"load current HKX animation pack");directIK(base);singularArms(base);for(int variant:{1,2})for(int fps:{30,60,120})stoppedArm(base,variant,fps);std::cout<<"PASS arm hinge tests checks="<<checks<<'\n';return 0;}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
