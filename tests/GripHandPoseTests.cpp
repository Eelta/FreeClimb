#define main gripHandThreepeatFixtureMain
#define check gripHandFixtureCheck
#include "ThreepeatMotionTests.cpp"
#undef check
#undef main
namespace fc {
#include "GripHandPose.h"
}
#include "CanonicalSkeleton.h"
#include <bit>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace fc;
static unsigned checks{};
static void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
static bool exact(float a,float b){return std::bit_cast<std::uint32_t>(a)==std::bit_cast<std::uint32_t>(b);}
static bool exact(Vec a,Vec b){return exact(a.x,b.x)&&exact(a.y,b.y)&&exact(a.z,b.z);}
static bool exact(Quat a,Quat b){return exact(a.x,b.x)&&exact(a.y,b.y)&&exact(a.z,b.z)&&exact(a.w,b.w);}
static Library canonical(){
    Library lib;lib.parents.assign(canonicalBoneParents.begin(),canonicalBoneParents.end());
    for(auto name:canonicalBoneNames)lib.names.emplace_back(name);
    for(const auto& value:canonicalBoneRest)lib.rest.push_back({{value[0],value[1],value[2]},
        {value[3],value[4],value[5],value[6]},{value[7],value[8],value[9]}});
    return lib;
}
static Vec localPoint(const Pose& world,int hand,Vec point){
    const auto& wrist=world[hand?39:38];return wrist.q.inverse().rotate(point-wrist.t);
}
static Vec tip(const Library& lib,const Pose& world,int bone){
    return world[bone].t+world[bone].q.rotate({0,0,lib.rest[bone].t.length()*.75f});
}
static void shapeAndIsolation(const Library& lib){
    for(int hand=0;hand<2;++hand){
        auto pose=lib.rest;const auto before=pose;applyClimbGrip(lib,pose,hand,1);
        const auto body=lib.world(pose),original=lib.world(before);const int first=hand?82:67;
        for(int bone=0;bone<99;++bone){
            check(exact(pose[bone].t,before[bone].t)&&exact(pose[bone].s,before[bone].s),"grip preserves every translation and scale");
            if(bone<first||bone>=first+15)check(exact(pose[bone].q,before[bone].q),"grip cannot rotate wrist, arm or opposite hand");
            check(PoseRig<Pose>::validLocal(pose[bone]),"finger target remains a finite normalized transform");
        }
        for(int digit=1;digit<5;++digit){
            const int base=first+digit*3,end=base+2;
            const Vec knuckle=localPoint(body,hand,body[base].t),point=localPoint(body,hand,tip(lib,body,end));
            const Vec oldTip=localPoint(original,hand,tip(lib,original,end));
            check(point.z<knuckle.z-.3f,"four fingertips turn back toward the palm instead of a common straight bend");
            check(point.y<knuckle.y-.7f&&point.y>knuckle.y-5,"fingers fold toward the same palm side within a bounded arc");
            check(point.length()<oldTip.length()-1,"closed fingers are closer to the wrist than the original relaxed hand");
            const auto a=angleBetween(pose[base].q,before[base].q),b=angleBetween(pose[base+1].q,before[base+1].q);
            check(std::abs(a-b)>.15f,"proximal and middle joints use distinct curls");
        }
        const Vec thumb=localPoint(body,hand,tip(lib,body,first+2));
        check(std::abs(thumb.x)<1&&thumb.y<-2&&thumb.z>4&&thumb.z<7,"thumb opposes inward across the curled index side");
    }
    for(float weight:{0.f,-1.f,std::numeric_limits<float>::quiet_NaN()}){
        auto pose=lib.rest;applyClimbGrip(lib,pose,0,weight);
        for(int bone=0;bone<99;++bone)check(exact(pose[bone].q,lib.rest[bone].q),"invalid or unloaded grip preserves authored rotations exactly");
    }
}
static void timeAndEligibility(const Library& source){
    for(int fps:{30,60,120}){
        ClimbGripPose state;Pose previous=source.rest;float peak=0;
        for(int frame=0;frame<fps;++frame){
            auto pose=source.rest;state.update(source,pose,Motion::hang,{frame<fps/2?1.f:0.f,0},1.f/fps);
            for(int bone=0;bone<99;++bone)peak=std::max(peak,angleBetween(pose[bone].q,previous[bone].q)*fps);
            if(frame==fps/3)check(state.weights[0]>.999f&&state.weights[1]==0,"loaded hand closes independently in bounded time");
            const auto weight=state.weights;auto paused=source.rest;state.update(source,paused,Motion::hang,{0,1},0);
            check(state.weights==weight,"pause cannot advance grip closure");previous=pose;
        }
        check(state.weights[0]==0&&state.weights[1]==0,"released hand returns completely to source fingers");
        std::cout<<"grip fps="<<fps<<" rawPeakRadPerSecond="<<peak<<'\n';
        check(peak<11,"contact-driven grip changes stay below the ordinary whole-pose angular rate budget");
        for(auto motion:{Motion::runUp,Motion::runLeft,Motion::runDiagonalRight,Motion::contextMantle,Motion::reach,Motion::hopUp,Motion::contextHopLeft,Motion::backFlipOut}){
            auto pose=source.rest;state.update(source,pose,motion,{1,1},.05f);
            for(int bone=0;bone<99;++bone)check(exact(pose[bone].q,source.rest[bone].q),"palm supports and reaching actions retain source fingers");
        }
    }
    for(auto motion:{Motion::hang,Motion::up,Motion::down,Motion::left,Motion::right}){
        auto lib=source;lib.clips[int(motion)-1].authoredPlayback=true;auto pose=lib.rest;ClimbGripPose state;
        for(int frame=0;frame<30;++frame)state.update(lib,pose,motion,{1,1},.05f);
        for(int bone=0;bone<99;++bone)check(exact(pose[bone].q,lib.rest[bone].q),"authored replacements bypass the bundled grip pose");
        lib.clips[int(motion)-1].authoredPlayback=false;
        lib.rotationOverrides[int(motion)-1].frames.resize(2);state.update(lib,pose,motion,{1,1},.05f);
        for(int bone=0;bone<99;++bone)check(exact(pose[bone].q,lib.rest[bone].q),"legacy rotation replacements retain their authored fingers");
    }
}
static void capturedRig(const Library& source){
    auto lib=source;Pose actual=source.rest;std::array<Transform,99> basis{};std::array<bool,99> mapped{};mapped.fill(true);
    for(int bone=67;bone<97;++bone){
        actual[bone].t=actual[bone].t*(.85f+float(bone%5)*.07f);actual[bone].s={1.13f,.91f,1.07f};
        basis[bone]={{.07f,-.03f,.05f},Quat::axis({1,2,-1},float(bone%7)*.04f),{.97f,1.02f,1.04f}};
    }
    check(lib.configureRig(actual,basis,mapped)&&lib.rig.active(),"fixture captures noncanonical hand lengths, scales and intermediate bases");
    for(float amount:{.1f,.35f,.7f,1.f}){
        auto expected=source.rest;for(int hand=0;hand<2;++hand)applyClimbGrip(source,expected,hand,amount);lib.rig.adapt(expected);
        auto pose=source.rest;lib.rig.adapt(pose);const auto before=pose;
        for(int hand=0;hand<2;++hand)applyClimbGrip(lib,pose,hand,amount);
        for(int bone=0;bone<99;++bone){
            check(angleBetween(pose[bone].q,expected[bone].q)<.0005f,"grip target is transformed to captured basis exactly once");
            check(exact(pose[bone].t,before[bone].t)&&exact(pose[bone].s,before[bone].s),"captured hand proportions and scales remain exact");
        }
    }
}
static void actualPack(const char* path,const char* output){
    Library lib;check(lib.load(path),"actual bundled pack loads for finger-shape evidence");
    auto before=lib.sample(Motion::hang,.5f),after=before;
    for(int hand=0;hand<2;++hand)applyClimbGrip(lib,after,hand,1);
    if(!output)return;
    std::ofstream file(output);file<<"{\"description\":\"Actual bundled hang, canonical skeleton proxy only\",\"hands\":[";
    const auto original=lib.world(before),body=lib.world(after);
    for(int hand=0;hand<2;++hand){
        if(hand)file<<',';file<<"{\"before\":[";
        for(int mode=0;mode<2;++mode){
            if(mode)file<<"],\"after\":[";const auto& pose=mode?body:original;const int first=hand?82:67;
            for(int digit=0;digit<5;++digit){
                if(digit)file<<',';file<<'[';
                for(int joint=0;joint<4;++joint){
                    if(joint)file<<',';const int bone=first+digit*3+std::min(joint,2);
                    const auto p=localPoint(pose,hand,joint==3?tip(lib,pose,bone):pose[bone].t);
                    file<<'['<<p.x<<','<<p.y<<','<<p.z<<']';
                }
                file<<']';
            }
        }
        file<<"]}";
    }
    file<<"]}";
}
static void surfaceContacts(const Library& lib,int fps,float scale){
    ThreepeatWorld world;world.boxes={{{-10000,0,-5000},{10000,1000,20000}}};
    world.origin={2170,-1200,30};world.yaw=.63f;
    Traversal traversal;traversal.cfg.gap=37*scale;traversal.cfg.radius=31*scale;traversal.cfg.height=138*scale;
    traversal.cfg.contextScale=scale;traversal.cfg.approachSeconds=.01f;traversal.cfg.automaticClimbActions=false;
    check(lib.configureThreepeat(traversal.cfg),"grip fixture uses bundled calibration");
    check(traversal.attach(world,world.point({0,-45*scale,0}),world.vector({0,1,0}),1000,60*scale),"grip fixture attaches to a real wall");
    traversal.update(world,{},.05f,1000);SurfacePose surface,unmodified;Pose previous,settled;
    auto control=lib;for(auto motion:{Motion::hang,Motion::up,Motion::down,Motion::left,Motion::right})
        control.rotationOverrides[int(motion)-1].frames.resize(2);
    const float dt=1.f/fps;float minClearance=100,oldClearance=100,addedPenetration=0,peak=0,closure=0;unsigned closedSamples=0;int worstBone=-1;
    auto point=[&](Vec local){return traversal.position+(Vec{-traversal.normal.y,traversal.normal.x,0}*local.x-
        traversal.normal*local.y+Vec{0,0,local.z})*scale;};
    for(int frame=0;frame<fps*5;++frame){
        Input input;if(frame<fps)input.y=1;else if(frame>=fps*3&&frame<fps*4)input.x=-1;
        const auto result=traversal.update(world,input,dt,1000);
        check(traversal.active()&&!result.released,"grip style cannot change traversal ownership or movement");
        const auto pose=surface.update(lib,world,traversal,result.motion,dt,scale),body=lib.world(pose);
        const auto controlPose=unmodified.update(control,world,traversal,result.motion,dt,scale),sourceBody=lib.world(controlPose);
        if(!previous.empty())for(int bone=0;bone<99;++bone){
            const float rate=angleBetween(pose[bone].q,previous[bone].q)/dt;peak=std::max(peak,rate);
            check(rate<=12.566371f+.04f,"contact closure retains the complete SurfacePose bone-rate budget");
        }
        const auto sampled=lib.sample(result.motion,surface.sampledPhase());
        for(int bone=0;bone<99;++bone)check(exact(pose[bone].s,sampled[bone].s),"grip integration does not rescale any bone");
        if((frame>fps*2&&frame<fps*3)||frame>fps*4+fps/2){
            check(result.motion==Motion::hang,"neutral movement reaches ordinary hanging");
            for(int hand=0;hand<2;++hand){
                const int first=hand?82:67;float changed=0;
                check(lib.armBendValid(pose,hand),"fist transition retains both elbow branch guards");
                for(int bone=first;bone<first+15;++bone){
                    check(exact(pose[bone].t,sampled[bone].t),"grip keeps original phalange lengths");
                    changed=std::max(changed,angleBetween(pose[bone].q,sampled[bone].q));
                    const float clearance=-world.rotate(point(body[bone].t)-world.origin,-world.yaw).y/scale;
                    if(clearance<minClearance){minClearance=clearance;worstBone=bone;}
                    const float baseline=-world.rotate(point(sourceBody[bone].t)-world.origin,-world.yaw).y/scale;
                    oldClearance=std::min(oldClearance,baseline);
                    addedPenetration=std::max(addedPenetration,std::max(0.f,-clearance)-std::max(0.f,-baseline));
                }
                for(int digit=0;digit<5;++digit){const int end=first+digit*3+2;
                    const float clearance=-world.rotate(point(tip(lib,body,end))-world.origin,-world.yaw).y/scale;
                    if(clearance<minClearance){minClearance=clearance;worstBone=end;}
                    const float baseline=-world.rotate(point(tip(lib,sourceBody,end))-world.origin,-world.yaw).y/scale;
                    oldClearance=std::min(oldClearance,baseline);
                    addedPenetration=std::max(addedPenetration,std::max(0.f,-clearance)-std::max(0.f,-baseline));
                }
                closure=std::max(closure,changed);if(changed>.2f)++closedSamples;
            }
            if(!settled.empty()&&frame<fps*3)for(int bone=67;bone<97;++bone)
                check(angleBetween(settled[bone].q,pose[bone].q)<.002f,"settled closed fingers remain still on an unchanged wall");
            settled=pose;
        }
        previous=pose;
    }
    std::cout<<"surface grip fps="<<fps<<" scale="<<scale<<" peakRate="<<peak<<" clearance="<<minClearance<<" before="<<oldClearance<<" added="<<addedPenetration<<" closure="<<closure<<" samples="<<closedSamples<<" worstBone="<<worstBone<<'\n';
    check(closedSamples>unsigned(fps),"real supported hanging visibly uses the closed grip");
    check(addedPenetration<=.15001f,"closed fingers and terminal tips do not add penetration to the unmodified supported pose");
}
int main(int argc,char** argv){
    try{const auto lib=canonical();shapeAndIsolation(lib);timeAndEligibility(lib);capturedRig(lib);
        if(argc>1){actualPack(argv[1],argc>2?argv[2]:nullptr);Library actual;check(actual.load(argv[1]),"actual pack for surface contacts");
            for(int fps:{30,60,120})for(float scale:{.75f,1.f,1.4f})surfaceContacts(actual,fps,scale);}
        std::cout<<"PASS grip hand checks="<<checks<<'\n';return 0;
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
