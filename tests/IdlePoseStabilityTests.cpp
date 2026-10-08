#include "Pose.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
static unsigned checks{},cases{};
static void check(bool value,const char* text) {++checks;if(!value)throw std::runtime_error(text);}
struct ReliefWorld final:World {
    Vec normal{0,-1,0},point{0,0,106};
    float relief{};
    bool absent{},missMeasurement{};
    std::optional<Hit> ray(Vec a,Vec b) override {
        if(absent||(missMeasurement&&std::abs((b-a).length()-73.f)<.05f))return {};
        std::optional<Hit> result;float earliest=2;
        for(bool center:{false,true}) {
            const Vec plane=point+normal*(center?0.f:relief);
            const float from=(a-plane).dot(normal),to=(b-plane).dot(normal);
            if(from<=0||to>=0)continue;
            const float progress=from/(from-to);const Vec hit=a+(b-a)*progress;
            if((std::abs(hit.x-point.x)<=12)!=center||progress>=earliest)continue;
            earliest=progress;result=Hit{hit,normal,true};
        }
        return result;
    }
};
static Traversal fixture(ReliefWorld& world,float slope,float margin=0) {
    world.normal={0,-std::sqrt(1-slope*slope),slope};
    Traversal t;t.cfg.gap=37;t.cfg.radius=31;t.cfg.height=138;t.cfg.automaticClimbActions=false;
    t.state=State::wall;t.position={0,-37-margin,100};t.normal={0,-1,0};t.surfaceNormal=world.normal;return t;
}
static float stepTime(int fps,unsigned frame) {return fps?1.f/fps:frame%2?1.f/144:1.f/30;}
struct MotionRange {
    std::array<Vec,6> before{},first{};
    float step{},drift{};
    bool started{};
    void add(const Library& library,const Pose& pose) {
        const auto body=library.world(pose);const int bones[]={0,4,38,39,8,11};
        for(unsigned i=0;i<6;++i) {
            const Vec point=body[bones[i]].t;
            if(started){step=std::max(step,(point-before[i]).length());drift=std::max(drift,(point-first[i]).length());}
            else first[i]=point;
            before[i]=point;
        }
        started=true;
    }
};
static float reliefCase(const Library& library,float slope,float relief,Motion source,float movingSeconds,int fps) {
    ReliefWorld world;world.relief=relief;auto t=fixture(world,slope);SurfacePose surface;MotionRange range;
    float elapsed=0;int retireBefore=-1;Vec stopped{};
    for(unsigned frame=0;elapsed<12;++frame) {
        const float dt=stepTime(fps,frame);const Motion motion=elapsed<movingSeconds?source:Motion::hang;
        if(motion!=Motion::hang) {
            const Vec delta=source==Motion::up?Vec{0,0,dt*80}:Vec{source==Motion::left?-dt*80:dt*80,0,0};
            t.position=t.position+delta;world.point=world.point+delta;
        }
        const auto pose=surface.update(library,world,t,motion,dt,1);
        if(elapsed>=7) {
            if(retireBefore<0){retireBefore=surface.retiredContacts;stopped=t.position;}
            range.add(library,pose);
            check((t.position-stopped).length()==0,"idle adaptation never changes the physical root");
            check(surface.surfaceGapValid&&std::abs(surface.appliedSurfaceGap-37)<.001f,"stable relief keeps real body clearance");
            check(library.armBendValid(pose,0)&&library.armBendValid(pose,1),"stable idle keeps valid elbow branches");
        }
        elapsed+=dt;
    }
    if(range.step>=.001f||range.drift>=.001f||surface.retiredContacts!=retireBefore)
        std::cerr<<"relief slope="<<slope<<" relief="<<relief<<" source="<<int(source)<<" moving="<<movingSeconds<<" fps="<<fps<<" step="<<range.step<<" drift="<<range.drift<<" retires="<<surface.retiredContacts-retireBefore<<'\n';
    check(range.step<.001f&&range.drift<.001f,"unchanged wall, source pose and target cannot repeatedly pull or release limbs");
    check(surface.retiredContacts==retireBefore,"an unchanged rejected contact is not repeatedly retired and reacquired");
    if(relief==0&&source==Motion::hang)check(surface.contactCount>=2,"stability retains at least two legitimate support contacts");
    ++cases;return range.step;
}
static void changedSupport(const Library& library) {
    ReliefWorld world;world.relief=-4;auto t=fixture(world,.491f);SurfacePose surface;Pose previous;
    for(unsigned frame=0;frame<180;++frame)previous=surface.update(library,world,t,Motion::hang,1.f/60,1);
    check(surface.rejectedIdleContacts()>0,"uneven fixture has a rejected static candidate");
    world.relief=0;
    for(unsigned frame=0;frame<180;++frame)previous=surface.update(library,world,t,Motion::hang,1.f/60,1);
    check(surface.rejectedIdleContacts()==0&&surface.contactCount>=2,"a changed real target is immediately eligible for valid contact");
    world.relief=-4;surface.reset();
    for(unsigned frame=0;frame<180;++frame)previous=surface.update(library,world,t,Motion::hang,1.f/60,1);
    check(surface.rejectedIdleContacts()>0,"a new acquisition reevaluates uneven support after reset");
    t.position.x+=1;world.point.x+=1;
    surface.update(library,world,t,Motion::hang,1.f/60,1);
    check(surface.rejectedIdleContacts()==0,"real actor movement invalidates stationary contact rejection");
    for(unsigned frame=0;frame<180;++frame)previous=surface.update(library,world,t,Motion::hang,1.f/60,1);
    world.normal={0,-1,0};world.relief=0;t.surfaceNormal=world.normal;
    for(unsigned frame=0;frame<180;++frame)previous=surface.update(library,world,t,Motion::hang,1.f/60,1);
    check(surface.contactCount>=2,"slope changes retain normal contact adaptation");
    surface.update(library,world,t,Motion::left,1.f/60,1);
    check(surface.rejectedIdleContacts()==0,"changing the action clears idle candidate rejection");
    ++cases;
}
static void missedMeasurement(const Library& library,int fps) {
    ReliefWorld world;auto t=fixture(world,.65f,14);SurfacePose surface;MotionRange range;float elapsed=0;unsigned held=0;
    for(unsigned frame=0;elapsed<12;++frame) {
        const float dt=stepTime(fps,frame);world.missMeasurement=elapsed>=2&&frame%9==0;
        const auto pose=surface.update(library,world,t,Motion::hang,dt,1);
        if(elapsed>=7) {
            range.add(library,pose);
            check(std::abs(surface.appliedSurfaceGap-51)<.001f,"an isolated distance miss cannot move the settled body");
            if(world.missMeasurement) {
                ++held;check(!surface.surfaceGapValid&&surface.surfaceHoldSeconds()>0,"held placement is not reported as a fresh collision hit");
            } else check(surface.surfaceGapValid&&surface.surfaceHoldSeconds()==0,"fresh support clears the missing-measurement hold");
        }
        elapsed+=dt;
    }
    check(held>10&&range.step<.001f&&range.drift<.001f,"single-frame measurement misses cannot produce repeated vertical idle motion");
    world.missMeasurement=false;world.absent=true;elapsed=0;
    for(unsigned frame=0;elapsed<1;++frame) {
        const float dt=stepTime(fps,frame);surface.update(library,world,t,Motion::hang,dt,1);elapsed+=dt;
        check(!surface.surfaceGapValid,"missing geometry never becomes observed support");
        if(elapsed>.11f)check(surface.surfaceHoldSeconds()==0,"placement hold expires after one tenth of a second");
    }
    check(surface.appliedSurfaceGap==37&&surface.contactCount==0,"sustained geometry loss clears placement correction and contacts");
    ReliefWorld absent;absent.absent=true;auto fresh=fixture(absent,.65f,14);SurfacePose initial;
    initial.update(library,absent,fresh,Motion::hang,1.f/60,1);
    initial.update(library,absent,fresh,Motion::hang,1.f/60,1);
    check(initial.surfaceHoldSeconds()==0&&!initial.surfaceGapValid,"no hold is reported before a real correction has been measured");
    world.absent=false;
    for(unsigned frame=0;frame<180;++frame)surface.update(library,world,t,Motion::hang,1.f/60,1);
    world.missMeasurement=true;t.position.z+=1;world.point.z+=1;
    surface.update(library,world,t,Motion::up,1.f/60,1);
    check(surface.surfaceHoldSeconds()==0&&surface.appliedSurfaceGap<51,"moving actions use live placement rather than static idle retention");
    ++cases;
}
static void authoredIdle(const Library& base) {
    auto library=base;library.clearAnimationOverrides();auto& clip=library.clips[int(Motion::hang)-1];
    clip.authoredPlayback=true;clip.seconds=1;clip.frames.assign(121,base.clip(Motion::hang).frames.front());clip.contacts.assign(121,{});
    for(unsigned frame=0;frame<clip.frames.size();++frame)clip.frames[frame][4].t.z+=2*std::sin(float(frame)*6.2831853f/120);
    ReliefWorld world;world.relief=-4;auto t=fixture(world,.491f);SurfacePose surface;
    float lo=1e9f,hi=-1e9f;unsigned loops=0;float before=0;
    for(unsigned frame=0;frame<720;++frame) {
        const auto pose=surface.update(library,world,t,Motion::hang,1.f/120,1);const float phase=surface.sampledPhase();
        if(phase<before)++loops;before=phase;
        if(frame>120){lo=std::min(lo,pose[4].t.z);hi=std::max(hi,pose[4].t.z);}
        check(surface.rejectedIdleContacts()==0,"authored idle is excluded from frozen default contact rejection");
    }
    check(loops>=5&&hi-lo>3.9f,"authored idle continues its full source clock and COM motion");++cases;
}
int main(int argc,char** argv) {try {
    check(argc==2,"supply runtime animation pack");Library library;check(library.load(argv[1]),"load actual animation library");
    float peak=0;
    for(float slope:{0.f,.491f,.65f})for(float relief:{-4.f,0.f,8.f})for(Motion source:{Motion::hang,Motion::up,Motion::left,Motion::right})
        for(float seconds:{.25f,.7f})for(int fps:{30,60,144,0})peak=std::max(peak,reliefCase(library,slope,relief,source,seconds,fps));
    changedSupport(library);for(int fps:{30,60,144,0})missedMeasurement(library,fps);authoredIdle(library);
    std::cout<<"PASS idle pose stability cases="<<cases<<" checks="<<checks<<" maximum settled step="<<peak<<'\n';return 0;
} catch(const std::exception& error) {std::cerr<<"FAIL after "<<cases<<" cases: "<<error.what()<<'\n';return 1;}}
