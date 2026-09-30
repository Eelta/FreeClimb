#include "Pose.h"
#include "WallRunPose.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
void check(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}

struct Rock:World {
    Vec n{0,-.75f,.6614378f};
    Vec origin{112118.23f,79550.38f,2177.79f};
    bool seams{},jitter{};int frame{};
    std::optional<Hit> ray(Vec a,Vec b)override {
        float x=(a-origin).dot(n),y=(b-origin).dot(n);
        if(x<=0||y>=0)return {};
        Vec p=a+(b-a)*(x/(x-y)),out=n;
        if(seams&&p.z-origin.z>60) {

            const float angle=std::sin((p.x-origin.x)*.16f)*.6f;
            out=Vec{std::sin(angle)*.9f,n.y*std::cos(angle),n.z}.unit();
        }
        if(jitter)out=Vec{(frame%2?1.f:-1.f)*.02f,n.y,n.z}.unit();
        return Hit{p,out,true};
    }
};
struct RoundedSummit:World {
    Vec origin{106300.43f,68076.12f,8000};

    std::optional<Hit> ray(Vec a,Vec b)override {
        a=a-origin;b=b-origin;std::optional<Hit> result;float nearest=2;
        struct Face{float low,high,k,intercept;};
        for(auto f:{Face{-1000,120,0,0},Face{120,220,.6f,-72},Face{220,300,1.8f,-336},Face{300,2000,4,-996}}) {
            const Vec n=Vec{0,-1,f.k}.unit();
            const float from=a.dot(n)+f.intercept/std::sqrt(1+f.k*f.k);
            const float to=b.dot(n)+f.intercept/std::sqrt(1+f.k*f.k);
            if(from<=0||to>0)continue;
            const float t=from/(from-to);const Vec p=a+(b-a)*t;
            if(t<nearest&&p.z>=f.low-.001f&&p.z<=f.high+.001f) {nearest=t;result=Hit{p+origin,n,true};}
        }
        return result;
    }
};
int main(int argc,char**argv){try{
    check(argc==2,"motion file required");Library lib;check(lib.load(argv[1]),"motion file");
    for(bool seams:{false,true}) {
        Rock rock;rock.seams=seams;Traversal t;t.cfg.approachSeconds=0;t.cfg.radius=31;t.cfg.gap=37;t.cfg.height=138;
        check(t.attach(rock,rock.origin+Vec{0,-42,0},{0,1,0},100),"rock attach");
        float longest=0;
        for(int i=0;i<360;++i){t.update(rock,{0,1},1.f/60,100);longest=std::max(longest,t.stalledSeconds());}
        check(t.active()&&t.position.z-rock.origin.z>160&&longest<.6f,"upward movement crosses fine rock seams");
        float startX=t.position.x;
        for(int i=0;i<180;++i)t.update(rock,{1,0},1.f/60,100);
        check(t.active()&&t.position.x-startX>60,"sideways movement crosses fine rock seams");
    }
    {
        RoundedSummit summit;Traversal climb;climb.cfg.approachSeconds=0;
        climb.cfg.radius=31;climb.cfg.gap=37;climb.cfg.height=138;
        check(climb.attach(summit,summit.origin+Vec{0,-37,0},{0,1,0},100),"rounded mountain attach");
        bool completed=false;
        for(int i=0;i<900&&climb.active();++i)completed|=climb.update(summit,{0,1,false,true},1.f/60,100).completed;
        if(!completed)std::cerr<<"summit z="<<climb.position.z-summit.origin.z<<" reason="<<climb.blockedReason<<" top="<<climb.ledgeReason<<'\n';
        check(completed&&!climb.active(),"continuous cliff-to-walkable summit must finish without hanging in air");
        check(climb.position.z-summit.origin.z>220,"summit exit must be onto the upper walkable facet");
    }
    Rock rock;Traversal t;t.cfg.approachSeconds=0;
    check(t.attach(rock,rock.origin+Vec{0,-40,0},{0,1,0},100),"hold attach");
    SurfacePose pose;
    for(int i=0;i<120;++i){auto result=t.update(rock,{0,1},1.f/60,100);pose.update(lib,rock,t,result.motion,1.f/60,1);}
    rock.jitter=true;auto fixed=t.position;auto normal=t.normal;Pose stable;
    for(int i=0;i<300;++i){
        rock.frame=i;auto result=t.update(rock,{},1.f/60,100);auto p=pose.update(lib,rock,t,result.motion,1.f/60,1);
        check((t.position-fixed).length()==0&&(t.normal-normal).length()==0,"hanging must not correct the capsule or rotate the actor");
        check(result.staminaCost==0,"resting hang must not consume stamina");
        if(i==30)stable=p;
        if(i>30)for(int j=0;j<99;++j)check((p[j].t-stable[j].t).length()<.0001f&&std::abs(p[j].q.dot(stable[j].q))>.999999f,"settled grip must not jitter when collision normals alternate");
    }
    check(!animatedNativePose(lib.rest,lib.rest),"never rotate the reference T-pose as a runner");
    Pose animated=lib.rest;animated[6].q=Quat::axis({1,0,0},.4f)*animated[6].q;
    check(animatedNativePose(animated,lib.rest),"a non-reference live leg pose is allowed");
    NativeRunEvidence evidence;
    check(!evidence.sample(animated,lib.rest,true,1000),"one non-reference pose is insufficient proof of running");
    check(!evidence.sample(animated,lib.rest,true,1020),"a static A-pose is not a run cycle");
    animated[7].q=Quat::axis({1,0,0},.15f)*animated[7].q;
    check(!evidence.sample(animated,lib.rest,true,1040),"one changed pose is insufficient proof of sustained locomotion");
    for(int i=1;i<=3;++i) {
        animated[7].q=Quat::axis({1,0,0},.15f)*animated[7].q;
        const bool ready=evidence.sample(animated,lib.rest,true,1040+i*20);
        check(ready==(i==3),"require several changing samples across at least 60 ms");
    }
    check(!evidence.sample(animated,lib.rest,true,1201),"frozen sprint loses movement permission after 100 ms, not 300 ms");
    check(!evidence.sample(animated,lib.rest,true,1400),"stale/frozen run output loses permission");
    check(!evidence.sample(animated,lib.rest,false,1420),"releasing run invalidates motion evidence");
    NativeRunDrive drive;bool moving=false,sprinting=false;int starts=0,stops=0,resets=0,sprints=0;
    auto dispatch=[&](const char* name){
        if(std::string_view(name)=="IdleForceDefaultState"){moving=false;++resets;}
        if(std::string_view(name)=="moveStart"){moving=true;++starts;}
        if(std::string_view(name)=="moveStop"){moving=false;++stops;}
        if(std::string_view(name)=="SprintStart"){sprinting=true;++sprints;}
        if(std::string_view(name)=="SprintStop")sprinting=false;
    };

    check(drive.update(true,false,1000,dispatch)&&moving&&sprinting,"request starts SprintStart before pose readiness exists");
    for(int i=1;i<=90;++i){drive.update(true,true,1000+i*20,dispatch);check(moving&&sprinting,"healthy sprint must not restart");}
    check(starts==1&&resets==1,"one held run request sends one start, never an idle/reset loop");
    moving=sprinting=false;drive.update(true,false,3000,dispatch);
    check(drive.update(true,false,3200,dispatch)&&moving&&sprinting,"externally stopped sprint recovers without a new key press");
    for(int i=0;i<80;++i)drive.update(true,false,3300+i*100,dispatch);
    check(starts==4&&sprints==4&&resets==1,"at most three retries, with no Idle reset on recovery");
    drive.update(false,false,12000,dispatch);check(!moving&&!sprinting&&stops==1,"release stops sprint and locomotion");
    WallModeTransition transition;check(transition.weight()==0,"mode starts in climbing");
    transition.select(true);float last=0;

    for(int i=0;i<28;++i){transition.advance(1.f/60);float value=transition.weight();check(value>=last&&value-last<.07f,"sprint weight ramps smoothly over 0.46 s");last=value;}
    check(std::abs(last-1)<.001f,"sprint transition completes");
    transition.select(false);check(transition.weight()==last,"switch back preserves the current pose weight");
    for(int i=0;i<12;++i)transition.advance(1.f/60);
    const float middle=transition.weight();transition.select(true);
    check(std::abs(transition.weight()-middle)<.00001f,"rapid Shift reversal cannot jump to an endpoint");
    transition.advance(1.f/60);check(transition.weight()>middle&&transition.weight()-middle<.02f,"reversed transition remains bounded");
    std::cout<<"PASS: receding slope, fine normal seams, stable hanging capsule/pose, zero rest drain, reference-pose rejection\n";
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
