#include "Pose.h"
#include "CornerTestWorld.h"
#include <iostream>
#include <stdexcept>
#include <string>
using namespace fc;
static unsigned checks{},cases{};
static void check(bool good,const char* message) {++checks;if(!good)throw std::runtime_error(message);}
struct Plane final:World {
    Vec normal{0,-1,0},point{},right{1,0,0};
    bool present=true,narrow=false,incoherent=false;
    std::optional<Hit> ray(Vec from,Vec to) override {
        if(!present)return {};
        const float den=(to-from).dot(normal);
        if(den>=-.000001f)return {};
        const float offset=incoherent?(from-point).dot(right)<-4?8.f:(from-point).dot(right)>4?-8.f:0.f:0.f;
        const float distance=(from-point-normal*offset).dot(normal);
        const float t=-distance/den;
        if(distance<=.000001f||t<0||t>1)return {};
        const Vec hit=from+(to-from)*t;
        if(narrow&&std::abs((hit-point).dot(right))>.2f)return {};
        return Hit{hit,normal,true};
    }
};
static Vec worldPoint(Vec value,const Traversal& t,float scale) {
    return t.position+(Vec{-t.normal.y,t.normal.x,0}*value.x-t.normal*value.y+Vec{0,0,value.z})*scale;
}
static void legalPose(const Library& lib,const Pose& pose,const Pose* before,float dt,Motion motion) {
    check(pose.size()==99,"full99 bone output");
    for(unsigned bone=0;bone<pose.size();++bone) {
        check(pose[bone].t.finite()&&std::abs(pose[bone].q.dot(pose[bone].q)-1)<.002f,"finite normalized output");
        if(bone!=0&&bone!=4)check(std::abs(pose[bone].t.length()-lib.rest[bone].t.length())<.002f,"unchanged bone lengths");
        if(before) {
            const float angle=angleBetween((*before)[bone].q,pose[bone].q),limit=(runMotion(motion)?18.849556f:12.566371f)*dt+.015f;
            if(angle>=limit)std::cerr<<"orientation motion="<<int(motion)<<" bone="<<bone<<" angle="<<angle<<" limit="<<limit<<'\n';
            check(angle<limit,"bounded orientation change");
        }
    }
    for(int hand=0;hand<2;++hand)check(lib.armBendValid(pose,hand),"valid elbow branch");
}
static Traversal fixture(Plane& world,float slope,float margin,float yaw,bool far) {
    const Vec origin=far?Vec{134559.219f,36994.7031f,-11691.1309f}:Vec{};
    const Vec outward{std::sin(yaw),-std::cos(yaw),0};
    world.normal=outward*std::sqrt(1-slope*slope)+Vec{0,0,slope};world.point=origin+Vec{0,0,106};
    world.right={-outward.y,outward.x,0};
    Traversal t;t.cfg.gap=37;t.cfg.radius=31;t.cfg.height=138;t.state=State::wall;
    t.position=origin+outward*(37+margin)+Vec{0,0,100};t.normal=outward;t.surfaceNormal=world.normal;
    return t;
}
static void stationary(const Library& lib,float slope,float margin,float scale,float yaw,int fps) {
    Plane world;auto t=fixture(world,slope,margin,yaw,yaw!=0);
    SurfacePose surface;Pose previous,pose;
    const auto initial=t.position;const float dt=1.f/fps;
    for(int frame=0;frame<fps*2;++frame) {
        pose=surface.update(lib,world,t,Motion::hang,dt,scale);
        legalPose(lib,pose,previous.empty()?nullptr:&previous,dt,Motion::hang);previous=pose;
    }
    check((t.position-initial).length()==0&&t.cfg.gap==37,"pose adaptation cannot move the physics root or alter clearance");
    check(surface.surfaceGapValid&&surface.surfaceSamples>=2&&surface.surfaceSamples<=6,"real coherent support measured");
    const auto body=lib.world(pose);float gap=0;
    for(int hand=0;hand<2;++hand) {
        const float distance=(worldPoint(lib.palm(body,hand),t,scale)-world.point).dot(world.normal);
        gap=std::max(gap,distance);
        check(distance>2.5f*scale&&distance<5.2f*scale,"displayed palms remain close to the actual collision plane");
        check(std::abs(distance-surface.surfacePalmGaps[hand])<.035f,"diagnostic gap describes final displayed palms");
    }
    check(surface.contactCount>=2&&surface.maxReachError<.1f,"coherent surface restores actual contacts within reach");
    check(std::abs(surface.measuredSurfaceGap-(37+margin))<.05f,"measured support matches known physical plane");
    std::cout<<"steady slope="<<slope<<" margin="<<margin<<" scale="<<scale<<" yaw="<<yaw<<" fps="<<fps<<" palm="<<gap<<" contacts="<<surface.contactCount<<'\n';++cases;
}
static void unsupported(const Library& lib,int kind) {
    Plane world;auto t=fixture(world,0,14,0,false);
    if(kind==0)world.present=false;
    if(kind==1)world.narrow=true;
    if(kind==2)world.incoherent=true;
    if(kind==3)world.normal={std::sin(.45f),-std::cos(.45f),0};
    SurfacePose surface;Pose pose;
    for(int frame=0;frame<120;++frame)pose=surface.update(lib,world,t,Motion::hang,1.f/60,1);
    check(!surface.surfaceGapValid&&surface.appliedSurfaceGap==37,"unsupported or inconsistent surfaces cannot shift the skeleton toward guessed contacts");
    if(kind==0)check(surface.contactCount==0,"absent geometry does not claim contacts");
    legalPose(lib,pose,nullptr,1.f/60,Motion::hang);++cases;
}
static void transitions(const Library& lib,float slope,int fps) {
    Plane world;auto t=fixture(world,slope,14,0,false);SurfacePose surface;Pose previous;
    const float dt=1.f/fps;float peakStep=0,peakGapStep=0,previousGap=37;
    const Motion sequence[]={Motion::jumpCatch,Motion::hang,Motion::up,Motion::hang,Motion::runUp,Motion::hopUp,Motion::runUp,Motion::hang,Motion::hopRight,Motion::hang};
    for(const auto motion:sequence)for(int frame=0;frame<fps;++frame) {
        const auto position=t.position;const auto pose=surface.update(lib,world,t,motion,dt,1);
        legalPose(lib,pose,previous.empty()?nullptr:&previous,dt,motion);
        check((t.position-position).length()==0,"transition adaptation keeps collision root unchanged");
        if(!previous.empty()) {
            peakGapStep=std::max(peakGapStep,std::abs(surface.appliedSurfaceGap-previousGap));
            const auto before=lib.world(previous),after=lib.world(pose);
            for(int bone:{8,11,38,39}) {
                const float step=(before[bone].t-after[bone].t).length();peakStep=std::max(peakStep,step);
                check(step<=1100*dt+.8f,"action and loop changes cannot snap visible limb endpoints");
            }
        }
        previous=pose;previousGap=surface.appliedSurfaceGap;
    }
    world.present=false;
    for(int frame=0;frame<fps;++frame) {
        const auto pose=surface.update(lib,world,t,Motion::hang,dt,1);legalPose(lib,pose,&previous,dt,Motion::hang);
        peakGapStep=std::max(peakGapStep,std::abs(surface.appliedSurfaceGap-previousGap));
        previous=pose;previousGap=surface.appliedSurfaceGap;
    }
    check(!surface.surfaceGapValid&&surface.contactCount==0&&surface.appliedSurfaceGap==37,"lost support retires actual contacts and settles the placement offset");
    check(peakGapStep<=100*dt,"surface placement changes are gradual through action and support boundaries");
    std::cout<<"transitions slope="<<slope<<" fps="<<fps<<" endpointStep="<<peakStep<<" gapStep="<<peakGapStep<<'\n';++cases;
}
static void movingRun(const Library& lib,float slope,float side) {
    Plane world;auto initial=fixture(world,slope,0,0,false);Traversal t;t.cfg=initial.cfg;
    check(t.attach(world,initial.position+initial.normal*5,initial.normal*-1,1000),"real run scenario attaches to observed support");
    SurfacePose surface;Pose previous;unsigned contacts=0,runningFrames=0;float worstToeGap=0;
    for(int frame=0;frame<240;++frame) {
        Input input{};input.x=side;input.y=side==1?0.f:1.f;input.run=true;
        const auto result=t.update(world,input,1.f/60,1000);
        check(!result.released,"run retains checked physical support");
        auto displayed=t;displayed.position=displayed.position+displayed.normal*14;
        const auto pose=surface.update(lib,world,displayed,result.motion,1.f/60,1);
        legalPose(lib,pose,previous.empty()?nullptr:&previous,1.f/60,result.motion);previous=pose;
        if(runMotion(result.motion)&&frame>100) {
            ++runningFrames;
            if(surface.runFootContacts) {
                ++contacts;const auto body=lib.world(pose);
                float closest=10000;
                for(int toe:{50,51})closest=std::min(closest,(worldPoint(body[toe].t,displayed,1)-world.point).dot(world.normal));
                worstToeGap=std::max(worstToeGap,closest);
                check(closest<7.f,"loaded wall-running sole stays near actual support despite capsule margin");
            }
        }
    }
    check(runningFrames>60&&contacts>10,"real straight/side/diagonal wall run retains repeated planted feet");
    std::cout<<"running slope="<<slope<<" side="<<side<<" contacts="<<contacts<<" nearestToe="<<worstToeGap<<'\n';++cases;
}
static void adaptiveRunHop(const Library& lib,int fps) {
    fc_test::CornerWorld world;
    world.boxes.push_back({{-3000,0,-3000},{3000,500,3000}});
    world.boxes.push_back({{-3000,-96,200},{3000,20,219}});
    Traversal t;t.cfg.radius=31;t.cfg.gap=37;t.cfg.height=138;t.cfg.approachSeconds=0;
    t.cfg.runSpeed=379.5f;t.cfg.wallRunObstacleJumps=true;
    check(t.attach(world,{0,-37,-100},{0,1,0},1000),"actual beam fallback attaches");
    SurfacePose surface;Pose previous;Vec oldRoot{},oldNormal{};float oldGap=37;
    bool jumped=false,resumed=false;unsigned recovery=0;float gapStep=0,endpointStep=0;
    const float dt=1.f/fps;
    for(int frame=0;frame<fps*6&&recovery<unsigned(fps);++frame) {
        const auto result=t.update(world,{0,1,false,false,false,false,true},dt,1000);
        check(!result.released,"actual beam fallback remains controlled");
        const auto pose=surface.update(lib,world,t,result.motion,dt,1);
        legalPose(lib,pose,previous.empty()?nullptr:&previous,dt,result.motion);
        if(!previous.empty()) {
            gapStep=std::max(gapStep,std::abs(surface.appliedSurfaceGap-oldGap));
            const auto a=lib.world(previous),b=lib.world(pose);Traversal prior=t;prior.position=oldRoot;prior.normal=oldNormal;
            for(int bone:{8,11,38,39}) {
                const float step=(worldPoint(a[bone].t,prior,1)-worldPoint(b[bone].t,t,1)).length();
                endpointStep=std::max(endpointStep,step);
                check(step<=1100*dt+.8f,"real run-hop-run fallback has no displayed endpoint snap");
            }
        }
        jumped|=result.motion==Motion::hopUp&&t.obstacleJumpActive();
        resumed|=jumped&&runMotion(result.motion)&&t.position.z>220;
        if(resumed)++recovery;
        oldGap=surface.appliedSurfaceGap;previous=pose;oldRoot=t.position;oldNormal=t.normal;
    }
    check(jumped&&resumed&&recovery>=unsigned(fps),"deep beam triggers the actual upward-hop fallback and resumes running");
    check(gapStep<=100*dt,"actual fallback surface correction changes continuously");
    std::cout<<"adaptive run-hop-run fps="<<fps<<" endpointStep="<<endpointStep<<" gapStep="<<gapStep<<'\n';++cases;
}
int main(int argc,char** argv) {try {
    check(argc==2,"supply runtime animation pack");Library lib;check(lib.load(argv[1])&&lib.animationPack,"load real complete HKX pack");
    for(float slope:{0.f,.601809323f})for(float margin:{0.f,4.f,14.f})for(float scale:{1.f,1.03f})for(float yaw:{0.f,.7f})for(int fps:{30,60})
        stationary(lib,slope,margin,scale,yaw,fps);
    for(int kind=0;kind<4;++kind)unsupported(lib,kind);
    for(float slope:{0.f,.601809323f})for(int fps:{30,60,144})transitions(lib,slope,fps);
    for(float slope:{0.f,.601809323f})for(float side:{0.f,1.f,.7f})movingRun(lib,slope,side);
    for(int fps:{30,60,120})adaptiveRunHop(lib,fps);
    std::cout<<"PASS pose surface distance cases="<<cases<<" checks="<<checks<<'\n';return 0;
} catch(const std::exception& error){std::cerr<<"FAIL after "<<cases<<" cases: "<<error.what()<<'\n';return 1;}}
