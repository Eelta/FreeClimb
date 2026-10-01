#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#undef near
#undef far
#endif
#include "Pose.h"
#include "CornerTestWorld.h"
#include "TraversalCapture.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
static void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
struct ObstacleMantleWorld:World {
    fc_test::CornerWorld geometry;
    std::optional<Vec> missingTop;
    std::optional<Hit> ray(Vec from,Vec to)override {
        const auto hit=geometry.ray(from,to);
        if(hit&&missingTop&&hit->normal.z>.95f&&(geometry.local(hit->point)-*missingTop).length()<.35f)return {};
        return hit;
    }
};
static ObstacleMantleWorld fixture(float top,bool far,bool obstacle=true) {
    ObstacleMantleWorld world;
    world.geometry.boxes={{{-10000,0,-10000},{10000,1000,top}}};
    if(obstacle)world.geometry.boxes.push_back({{-10000,-30,600},{10000,20,612}});
    if(far){world.geometry.origin={134559.219f,36994.7031f,-11691.1309f};world.geometry.yaw=.633f;}
    return world;
}
static Traversal attached(ObstacleMantleWorld& world,const Library& library) {
    Traversal traversal;traversal.cfg=fc_test::settings();traversal.cfg.runSpeed=379.5f;
    traversal.cfg.wallRunObstacleJumps=true;
    check(library.configureThreepeat(traversal.cfg),"real HKX pack must configure the authored climb-up contacts");
    traversal.cfg.threepeatAnimations=true;
    check(traversal.attach(world,world.geometry.global({0,-37,100}),world.geometry.direction({0,1,0}),1000),"automatic obstacle-mantle chain starts on actual wall support");
    return traversal;
}
static constexpr Input heldRun{0,1,false,true,false,false,true};
static void positive(const Library& library,int fps,bool far,float top) {
    auto world=fixture(top,far);auto traversal=attached(world,library);SurfacePose animation;Pose previous;
    bool jumped=false,landed=false,newTop=false,completed=false,replayed=false;unsigned topFrames=0,peak=0;
    float worstAngle=0,worstPalm=0,worstPhase=0,worstExcess=0,worstPalmPhase=0;int worstBone=-1,worstMotion=-1;const float dt=1.f/fps;
    std::array<Vec,4> oldEndpoints{};Motion oldMotion=Motion::none;
    float worstEndpointExcess=0,worstEndpointSpeed=0,worstFingerDepth=0,worstFingerPhase=0;
    unsigned fingerSamples=0;int worstFingerBone=-1;bool worstFingerTip=false;
    float endpointPhase=0,endpointPreparation=0,endpointDistance=0,endpointLimit=0;int endpointBone=-1,endpointFrom=-1,endpointTo=-1;
    auto capture=std::make_unique<TraversalCapture>();TraversalCapture::RecordingWorld recorder(world,*capture);
    for(int frame=0;frame<fps*8&&traversal.active();++frame) {
        world.geometry.rays=0;const auto before=traversal.position;
        capture->begin(traversal,heldRun,dt,1000);
        const auto result=traversal.update(recorder,heldRun,dt,1000);capture->finish(traversal,result);
        peak=std::max(peak,world.geometry.rays);jumped|=traversal.obstacleJumpCount()>0;
        if(jumped&&traversal.state!=State::action)landed=true;
        check(!result.released||result.completed,"supported obstacle kick to mantle must not detach early");
        const auto pose=animation.update(library,world,traversal,result.motion,dt,1);
        check(pose.size()==99,"actual HKX pose chain retains all canonical bones");
        for(const auto& bone:pose)check(bone.t.finite()&&std::isfinite(bone.q.dot(bone.q)),"all output transforms remain finite through kick-to-climb-up");
        for(int hand=0;hand<2;++hand)check(library.armBendValid(pose,hand),"run kick to climb-up cannot reverse the elbow bend branch");
        if(!previous.empty())for(std::size_t bone=0;bone<pose.size();++bone) {
            const float angle=angleBetween(previous[bone].q,pose[bone].q);
            const bool quick=runMotion(result.motion)||(result.motion>=Motion::kickUp&&result.motion<=Motion::kickRight);
            worstExcess=std::max(worstExcess,angle-(quick?18.849556f:12.566371f)*dt);
            if(angle>worstAngle){worstAngle=angle;worstBone=int(bone);worstMotion=int(result.motion);worstPhase=traversal.progress();}
        }
        const auto body=library.world(pose);
        const auto point=[&](Vec local){return traversal.position+Vec{-traversal.normal.y,traversal.normal.x,0}*local.x-traversal.normal*local.y+Vec{0,0,local.z};};
        std::array<Vec,4> endpoints{};
        for(unsigned index=0;index<endpoints.size();++index)endpoints[index]=point(body[std::array{8,11,38,39}[index]].t);
        if(!previous.empty()) {
            const bool quick=runMotion(result.motion)||(result.motion>=Motion::kickUp&&result.motion<=Motion::kickRight)||runMotion(oldMotion);
            const float speed=quick?1100.f:result.motion==Motion::contextMantle?750.f:600.f;
            const float limit=(result.motion!=oldMotion?std::min(speed,runMotion(oldMotion)?1100.f:600.f):speed)*dt;
            for(unsigned index=0;index<endpoints.size();++index) {
                const float step=(endpoints[index]-oldEndpoints[index]).length();
                worstEndpointSpeed=std::max(worstEndpointSpeed,step/dt);
                if(step-limit>worstEndpointExcess){worstEndpointExcess=step-limit;endpointPhase=traversal.progress();endpointPreparation=traversal.topPreparation();endpointDistance=step;endpointLimit=limit;endpointBone=std::array{8,11,38,39}[index];endpointFrom=int(oldMotion);endpointTo=int(result.motion);}
            }
        }
        previous=pose;oldEndpoints=endpoints;oldMotion=result.motion;
        if(result.motion==Motion::contextMantle) {
            check(jumped&&landed,"new climb-up is selected only after actual automatic obstacle flight and landing");
            newTop=true;++topFrames;if(animation.topPalmError>worstPalm){worstPalm=animation.topPalmError;worstPalmPhase=traversal.progress();}
            if(!replayed){check(capture->complete()&&capture->replay().matched,"new mantle selection replays with existing obstacle and cooldown fields");replayed=true;}
            for(int hand=0;hand<2;++hand)check(std::abs(world.geometry.local(traversal.topHand(hand)).z-top)<.05f,"climb-up hand targets lie on the real platform");
            if(traversal.topPreparation()>=1.f)for(int hand=0;hand<2;++hand)
                if(threepeatMantleWeight(hand,traversal.progress(),traversal.cfg.threepeatProfile)>.95f) {
                    const auto measure=[&](Vec local,int bone,bool tip) {
                        const Vec shown=world.geometry.local(point(local));++fingerSamples;
                        const auto& box=world.geometry.boxes.front();
                        const float depth=std::max(0.f,std::min({shown.x-box.low.x,box.high.x-shown.x,
                            shown.y-box.low.y,box.high.y-shown.y,shown.z-box.low.z,box.high.z-shown.z}));
                        if(depth>worstFingerDepth){worstFingerDepth=depth;worstFingerPhase=traversal.progress();worstFingerBone=bone;worstFingerTip=tip;}
                    };
                    for(int digit=0;digit<5;++digit) {
                        const int start=(hand?82:67)+digit*3;
                        for(int joint=0;joint<3;++joint)measure(body[start+joint].t,start+joint,false);
                        const int end=start+2;
                        measure(body[end].t+body[end].q.rotate({0,0,library.rest[end].t.length()*.75f}),end,true);
                    }
                }
        }
        completed|=result.completed;
        check((traversal.position-before).finite(),"root movement stays finite");
    }
    std::cout<<"obstacle mantle top="<<top<<" fps="<<fps<<" far="<<far<<" new="<<newTop<<" completed="<<completed<<" frames="<<topFrames<<" angle="<<worstAngle<<" palm="<<worstPalm<<" peak="<<peak<<" worstBone="<<worstBone<<" worstMotion="<<worstMotion<<" worstPhase="<<worstPhase<<" budgetExcess="<<worstExcess<<" worstPalmPhase="<<worstPalmPhase<<" endpointSpeed="<<worstEndpointSpeed<<" endpointExcess="<<worstEndpointExcess<<" fingerDepth="<<worstFingerDepth<<" fingerPhase="<<worstFingerPhase<<" fingerBone="<<worstFingerBone<<" fingerTip="<<worstFingerTip<<" fingerSamples="<<fingerSamples<<" endpointMotion="<<endpointFrom<<'/'<<endpointTo<<" endpointBone="<<endpointBone<<" endpointPhase="<<endpointPhase<<" endpointPreparation="<<endpointPreparation<<" endpointDistance/limit="<<endpointDistance<<'/'<<endpointLimit<<'\n';
    check(jumped&&landed&&newTop&&completed&&!traversal.active()&&topFrames>unsigned(fps/2),"automatic obstacle jump proceeds into full authored climb-up and exits to native standing");
    check(worstExcess<=.006f,"actual kick-to-mantle output keeps the existing per-motion angular rate limit");
    check(worstPalm<5,"actual loaded climb-up palms keep the existing five-unit contact bound");
    check(worstEndpointExcess<=.03f,"pregrip and climb-up preserve actual hand-foot world endpoint speed including the incoming transition budget");
    check(fingerSamples>unsigned(fps*10)&&worstFingerDepth<=.15001f,"loaded finger joints and physical tips stay outside the real finite platform after pregrip");
    check(peak<5000,"obstacle and mantle candidate work stays bounded");
}
static void routing(const Library& library,int kind,int fps) {
    auto world=fixture(kind==7?10000.f:kind==6?920.f:kind==5?780.f:840.f,false,kind!=4);
    auto traversal=attached(world,library);
    if(kind==0)traversal.cfg.contextualMantleEnabled=false;
    if(kind==1)traversal.cfg.contextActions=false;
    if(kind==2)traversal.cfg.threepeatAnimations=false;
    if(kind==8)traversal.cfg.threepeatMantlePalmHeight+=60;
    if(kind==3)world.missingTop=Vec{-traversal.cfg.threepeatMantleHalfWidth+traversal.cfg.threepeatMantleReplant[0].x,
        3+traversal.cfg.threepeatMantleReplant[0].y,840};
    bool expected=kind==4||kind==6;bool top=false,completed=false,selected=false;
    for(int frame=0;frame<fps*8&&traversal.active();++frame) {
        const auto result=traversal.update(world,heldRun,1.f/fps,1000);

        if(kind==5&&!top&&traversal.state==State::mantle) {
            const auto travel=traversal.topTarget()-traversal.topStart();
            const Vec side{-traversal.normal.y,traversal.normal.x,0};
            std::cout<<"kind5 selection fps="<<fps<<" precise="<<traversal.preciseTopContacts()<<" phase="<<traversal.progress()<<" height="<<(traversal.topLip().z-traversal.topStart().z)<<" authoredHeight="<<traversal.cfg.threepeatMantlePalmHeight<<" forward="<<travel.dot(traversal.normal*-1)<<" authoredForward="<<traversal.cfg.threepeatMantleForward<<" side="<<travel.dot(side)<<" from="<<traversal.topStart().z<<" position="<<traversal.position.z<<'\n';
            expected=std::abs(traversal.topLip().z-traversal.topStart().z-traversal.cfg.threepeatMantlePalmHeight)<=24.f&&
                std::abs(travel.dot(traversal.normal*-1)-traversal.cfg.threepeatMantleForward)<=24.f&&std::abs(travel.dot(side))<=8.f;
            check(traversal.preciseTopContacts()==expected,"a real matching landing uses precise contacts and a lower landing retains the general climb-up route");
        }
        if(!expected&&traversal.state==State::mantle)check(!traversal.preciseTopContacts(),"disabled family invalid replant or incompatible edge must not choose the precise contact route");
        if(!top&&traversal.state==State::mantle&&expected) {
            check(result.motion==Motion::contextMantle&&traversal.preciseTopContacts(),"ordinary running and expired obstacle cooldown allow geometrically valid precise climb-up");
            check(traversal.topPreparation()<.001f,"newly allowed running route retains the full pregrip transition");
        }
        selected|=traversal.state==State::mantle&&traversal.preciseTopContacts();
        if(traversal.state==State::mantle)check(result.motion==Motion::contextMantle,"every valid mantle uses the retained climb-up clip");
        top|=traversal.state==State::mantle;completed|=result.completed;
        if(kind==7)check(!top&&!completed,"continued vertical wall cannot become a fictitious summit");
    }
    if(kind!=4)check(traversal.obstacleJumpCount()==1,"selection routing case still exercises a real automatic obstacle jump");
    if(kind==4)check(traversal.obstacleJumpCount()==0,"ordinary running top-out requires no invented automatic obstacle");
    check(selected==expected,"precise contact availability changes only the geometry and family gates");
    if(kind!=7)check(top&&completed,"valid platform still completes through authored or fallback mantle");
    std::cout<<"routing kind="<<kind<<" fps="<<fps<<" new="<<selected<<" complete="<<completed<<" reason="<<traversal.topSelectionReason()<<'\n';
}
static void lostSupport(const Library& library,int fps,bool feetOnly=false) {
    auto world=fixture(840,false);auto traversal=attached(world,library);bool newTop=false;
    for(int frame=0;frame<fps*6&&traversal.active();++frame) {
        const auto result=traversal.update(world,heldRun,1.f/fps,1000);
        if(result.motion==Motion::contextMantle){newTop=true;break;}
    }
    check(newTop,"dynamic support test starts real new mantle after obstacle");
    if(feetOnly) {
        check(traversal.topPreparation()<1.f,"foot-only mutation happens while the real mantle still prepares its grasp");
        world.geometry.boxes.front().low.z=world.geometry.local(traversal.position).z+30.f;
        const Vec chest=traversal.position+Vec{0,0,70};
        check(world.ray(chest,chest-traversal.normal*60).has_value(),"foot-only mutation retains the chest wall");
        for(int hand=0;hand<2;++hand)check(world.ray(traversal.topHand(hand)+Vec{0,0,3},traversal.topHand(hand)-Vec{0,0,3}).has_value(),"foot-only mutation retains both real top grips");
    } else {
        while(traversal.progress()<.4f)check(!traversal.update(world,{},1.f/fps,1000).released,"real top stays supported before mutation");
        world.geometry.boxes.clear();
    }
    const auto position=traversal.position;
    const auto result=traversal.update(world,{},1.f/fps,1000);
    check(result.released&&!result.completed&&!traversal.active()&&(traversal.position-position).length()==0,"removed top aborts new obstacle-mantle before committing stale motion");
}
int main(int argc,char** argv){
#ifdef _WIN32
    SetErrorMode(SEM_FAILCRITICALERRORS|SEM_NOGPFAULTERRORBOX|SEM_NOOPENFILEERRORBOX);
#endif
try{
    check(argc==2,"provide shipped HKX pack manifest");Library library;check(library.load(argv[1])&&library.hasThreepeat(),"load actual shipped Threepeat HKX family");
    check(library.clip(Motion::contextMantle).frames.size()>=40,"climb-up retains full authored animation");
    for(int fps:{30,60,120})for(bool far:{false,true})for(float top:{740.f,840.f})positive(library,fps,far,top);
    for(int fps:{30,60,120}){for(int kind=0;kind<9;++kind)routing(library,kind,fps);lostSupport(library,fps);lostSupport(library,fps,true);}
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}return 0;}




