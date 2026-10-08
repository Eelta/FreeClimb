#include "Pose.h"
#include "CornerTestWorld.h"
#include "TraversalCapture.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
static unsigned checks=0;
static void require(bool value,const char* reason){++checks;if(!value)throw std::runtime_error(reason);}
static Library fixture(const Library& base,int kind,float seconds) {
    auto result=base;auto& clip=result.clips[41];clip.authoredPlayback=true;clip.seconds=seconds;clip.height=150;clip.travel={0,66,150};
    clip.trajectory={};clip.trajectory.count=5;
    clip.trajectory.knots[0]={0,{}};clip.trajectory.knots[4]={1,{0,66,150}};
    if(kind==0) {
        clip.trajectory.knots[1]={.20f,{0,30,5}};clip.trajectory.knots[2]={.55f,{0,45,100}};clip.trajectory.knots[3]={.85f,{0,66,150}};
    } else if(kind==1) {
        clip.trajectory.knots[1]={.25f,{0,0,60}};clip.trajectory.knots[2]={.60f,{0,0,165}};clip.trajectory.knots[3]={.85f,{0,66,165}};
    } else if(kind==2) {
        clip.trajectory.knots[1]={.20f,{6,28,-20}};clip.trajectory.knots[2]={.50f,{-8,10,90}};clip.trajectory.knots[3]={.80f,{4,66,156}};
    } else if(kind==3) {
        clip.trajectory.knots[1]={.20f,{0,30,5}};clip.trajectory.knots[2]={.50f,{0,30,5}};clip.trajectory.knots[3]={.80f,{0,66,150}};
    } else if(kind==4) {
        clip.trajectory.knots[1]={.25f,{0,45,0}};clip.trajectory.knots[2]={.50f,{0,66,160}};clip.trajectory.knots[3]={.80f,{0,8,10}};clip.trajectory.knots[4].displacement={};
    } else clip.trajectory={};
    clip.frames.assign(121,base.clip(Motion::hang).frames.front());clip.contacts.assign(121,{});
    for(std::size_t i=0;i<clip.frames.size();++i)clip.frames[i][0].t=clip.trajectory.sample(float(i)/120);
    return result;
}
static Traversal start(World& world,const Library& library,Vec feet={0,-42,0},Vec facing={0,1,0}) {
    Traversal t;require(library.configureThreepeat(t.cfg),"authored test config is valid");
    t.cfg.gap=37;t.cfg.radius=31;t.cfg.height=138;t.cfg.approachSeconds=0;t.cfg.staminaEnabled=false;
    t.cfg.contextActions=t.cfg.automaticClimbActions=t.cfg.wallRunObstacleJumps=false;
    require(t.attach(world,feet,facing,1000),"fixture attaches to a real supporting surface");return t;
}
static void flat(const Library& library,int kind,int fps,float height,float startHeight) {
    fc_test::CornerWorld world;world.boxes={{{-10000,0,-10000},{10000,10000,height}}};
    auto t=start(world,library,{0,-42,startHeight});bool completed=false,mantle=false,corridor=false;unsigned mantleFrames=0;
    SurfacePose surface;const float dt=1.f/fps;float stalledPhase=-1;Vec pausePosition{};
    for(unsigned i=0;i<unsigned(fps)*12&&t.active();++i) {
        const bool wasMantle=t.state==State::mantle;const float prior=t.progress();const auto result=t.update(world,{0,1,false,true},dt,1000);
        require(!result.released||result.completed,"safe fitted source route cannot unexpectedly release");
        if(result.motion==Motion::contextMantle) {
            mantle=true;corridor|=std::string_view(t.topRouteKind())=="corridor";
            if(wasMantle){++mantleFrames;require(std::abs(t.progress()-std::min(1.f,prior+dt/t.topSeconds()))<.00002f,"fitted route keeps the complete unaccelerated source clock");}
            require((t.position-t.topPathPoint(t.progress())).length()<.001f,"live path and published pose position match the accepted curve");
            const auto pose=surface.update(library,world,t,result.motion,dt,1),source=library.sample(result.motion,t.progress());
            for(int bone:{4,24,28,38})require(angleBetween(pose[bone].q,source[bone].q)<.001f,"corridor fitting does not rewrite imported source joint poses");
            require(world.clearance(t.position,t.cfg)>t.cfg.radius-.1f||t.position.z+6>height,"complete fitted body remains outside the solid platform");
            if(kind==3&&t.progress()>.25f&&t.progress()<.45f) {
                if(stalledPhase<0){stalledPhase=t.progress();pausePosition=t.position;}
                else require((t.position-pausePosition).length()<.001f,"authored source pauses remain pauses on the fitted corridor");
            }
        }
        completed|=result.completed;
    }
    require(mantle&&completed,"valid platform completes using every tested source trajectory");
    require(std::abs(float(mantleFrames)/fps-library.clip(Motion::contextMantle).seconds)<=1.1f/fps,"source duration is preserved at every frame rate");
    require(std::abs(t.position.z-height-7)<.05f,"fitted route reaches verified ground support");
    if(kind==0||kind==3)require(corridor,"early-forward sources select checked clearance fitting");
    if(kind==1)require(!corridor,"already clear source routes retain direct authored displacement");
}
static void blocked(const Library& library,int kind) {
    fc_test::CornerWorld world;world.boxes={{{-10000,0,-10000},{10000,10000,120}}};
    auto t=start(world,library);
    if(kind==0)world.boxes.push_back({{-10000,-10000,170},{10000,10000,176}});
    if(kind==1)world.boxes[0].high.y=8;
    bool completed=false,changed=false,aborted=false;
    for(int i=0;i<600&&t.active();++i) {
        if(kind==2&&t.state==State::mantle&&t.progress()>.35f) {
            const Vec next=t.topPathPoint(std::min(1.f,t.progress()+.04f));
            world.boxes.push_back({next+Vec{-200,-200,45},next+Vec{200,200,55}});changed=true;
        }
        const auto before=t.position;const auto r=t.update(world,{0,1,false,true},1.f/60,1000);completed|=r.completed;
        if(changed&&r.released){aborted=true;require((t.position-before).length()<.001f,"new obstruction blocks movement before committing its segment");}
    }
    require(!completed,"clearance fitting never bypasses real ceilings, insufficient platforms or new live obstacles");
    if(kind==2)require(changed&&aborted,"active fitted mantle still performs live collision checks");
}
static void capture(const Library& library) {
    fc_test::CornerWorld world;world.boxes={{{-10000,0,-10000},{10000,10000,120}}};
    world.origin={131316.86f,38643.36f,-11331.41f};world.yaw=1.13f;world.mirror=-1;
    auto t=start(world,library,world.global({0,-42,0}),world.direction({0,1,0}));
    for(unsigned i=0;i<600&&t.active()&&t.state!=State::mantle;++i)t.update(world,{0,1,false,true},1.f/60,1000);
    require(t.state==State::mantle&&std::string_view(t.topRouteKind())=="corridor","capture starts on the selected fallback route");
    auto tape=std::make_unique<TraversalCapture>(),loaded=std::make_unique<TraversalCapture>();TraversalCapture::RecordingWorld recorder(world,*tape);
    for(unsigned i=0;i<180&&t.active();++i) {
        tape->begin(t,{0,1,false,true},1.f/60,1000);const auto r=t.update(recorder,{0,1,false,true},1.f/60,1000);tape->finish(t,r);
        std::string error;require(tape->complete()&&loaded->deserialize(tape->serialize(),error),"fitted route state serializes with valid normalized knots");
        require(loaded->replay().matched,"fitted route captures reproduce every collision query and final state");
    }
    require(!t.active(),"capture includes terminal fitted mantle frame");
}
struct RoofWorld:World {
    struct Plane{Vec normal;float distance;};
    std::vector<Plane> planes;
    float z=.5f,x=std::sqrt(.75f),capWidth{},capHeight{};
    bool missing{};
    RoofWorld(float cap):capWidth(cap),capHeight(-cap*x/z) {
        planes={{{1,0,0},300},{{-1,0,0},300},{{0,1,0},300},{{0,-1,0},300},{{0,0,-1},1000},{{x,0,z},0},{{-x,0,z},0}};
        if(capWidth>0)planes.push_back({{0,0,1},capHeight});
    }
    std::optional<Hit> ray(Vec a,Vec b)override {
        const Vec delta=b-a;float enter=0,leave=1;Vec normal{};std::optional<Hit> hit;
        for(const auto& plane:planes) {
            const float dist=a.dot(plane.normal)-plane.distance,rate=delta.dot(plane.normal);
            if(std::abs(rate)<.0000001f){if(dist>0)return {};continue;}
            const float phase=-dist/rate;
            if(rate<0){if(phase>enter){enter=phase;normal=plane.normal;}}else leave=std::min(leave,phase);
            if(leave<enter)return {};
        }
        if(enter>.000001f&&enter<=1&&normal.length()>.9f) {
            const Vec point=a+delta*enter;
            if(!missing||std::abs(point.y)<8||point.z<capHeight-30)hit=Hit{point,normal,true};
        }
        return hit;
    }
};
static void roof(const Library& library,float cap,bool missing) {
    RoofWorld world(cap);const Vec feet{37-world.z*(-200+6)/world.x,0,-200};auto t=start(world,library,feet,{-1,0,0});
    world.missing=missing;bool completed=false,mantle=false;
    for(int frame=0;frame<720&&t.active();++frame) {
        const auto r=t.update(world,{0,1,false,true},1.f/60,1000);completed|=r.completed;mantle|=r.motion==Motion::contextMantle;
        if(completed&&missing)std::cerr<<"missing crest cap="<<cap<<" position="<<t.position.x<<','<<t.position.y<<','<<t.position.z<<" reason="<<r.reason<<" route="<<t.topRouteKind()<<'\n';
        require(!completed||!missing,"missing crest support cannot complete a fitted route");
    }
    if(!missing)require(completed&&mantle,"authored routes preserve sloping roof and capped crest traversal");
}
int main(int argc,char** argv)try {
    require(argc==2,"animation pack required");Library base;require(base.load(argv[1]),"base library loads");
    for(int kind=0;kind<6;++kind)for(float seconds:{.4f,2.2f,5.f}) {
        const auto library=fixture(base,kind,seconds);
        for(int fps:{30,60,120})for(float height:{60.f,90.f,140.f,190.f})flat(library,kind,fps,height,0);
    }
    const auto library=fixture(base,0,2.2f);for(int kind=0;kind<3;++kind)blocked(library,kind);capture(library);
    for(int kind:{0,1,2,3,5})for(float cap:{0.f,6.f,20.f}){roof(fixture(base,kind,2.2f),cap,false);roof(fixture(base,kind,2.2f),cap,true);}
    std::cout<<"Authored mantle geometry passed "<<checks<<" checks\n";return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
