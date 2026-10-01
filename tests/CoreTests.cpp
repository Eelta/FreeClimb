#include "Core.h"
#include "Controls.h"
#include <iostream>
#include <stdexcept>
#include <limits>
#include <string>
#include <vector>
using namespace fc;
void check(bool b,const char* m) { if(!b) throw std::runtime_error(m); }
struct Box { Vec lo,hi; bool climbable=true; };
struct Scene: World {
    std::vector<Box> boxes;
    std::optional<Hit> ray(Vec a,Vec b) override {
        std::optional<Hit> result; float nearest=2;
        const auto d=b-a;
        for(auto box:boxes) {
            float enter=0,leave=1; Vec normal{}; bool misses=false;
            const float aa[]={a.x,a.y,a.z},dd[]={d.x,d.y,d.z};
            const float lo[]={box.lo.x,box.lo.y,box.lo.z},hi[]={box.hi.x,box.hi.y,box.hi.z};
            for(int i=0;i<3;i++) {
                if(std::abs(dd[i])<1e-6f) { if(aa[i]<lo[i]||aa[i]>hi[i]) misses=true; continue; }
                float l=(lo[i]-aa[i])/dd[i],h=(hi[i]-aa[i])/dd[i];
                float sign=-1;
                if(l>h) { std::swap(l,h); sign=1; }
                if(l>enter) { enter=l; normal={}; if(i==0)normal.x=sign;if(i==1)normal.y=sign;if(i==2)normal.z=sign; }
                leave=std::min(leave,h);
            }
            if(!misses&&enter<=leave&&enter>0.00001f&&enter<nearest&&enter<=1) {
                result=Hit{a+d*enter,normal,box.climbable}; nearest=enter;
            }
        }
        return result;
    }
};
struct Slope: World {
    Vec normal{0,-0.8f,0.6f};
    std::optional<Hit> ray(Vec a,Vec b) override {
        const float start=a.dot(normal),end=b.dot(normal);
        if(start<=0||end>=0) return {};
        return Hit{a+(b-a)*(start/(start-end)),normal,true};
    }
};
struct RoughWall: Scene {
    std::optional<Hit> ray(Vec a,Vec b) override {
        auto hit=Scene::ray(a,b);
        if(hit&&hit->normal.y<-.9f&&std::abs(a.x)>5)
            hit->normal=Vec{a.x>0 ? .4f : -.4f,-.916515f,0};
        return hit;
    }
};
struct Summit:World {
    std::optional<Hit> ray(Vec a,Vec b) override {
        Vec delta=b-a;std::optional<Hit> hit;float nearest=2;
        if(a.y<0&&b.y>=0) {
            float t=-a.y/delta.y;Vec p=a+delta*t;
            if(p.z<=95) {hit=Hit{p,{0,-1,0},true};nearest=t;}
        }
        Vec n=Vec{0,-.35f,1}.unit();float from=(a-Vec{0,0,95}).dot(n),to=(b-Vec{0,0,95}).dot(n);
        if(from>0&&to<=0) {
            float t=from/(from-to);Vec p=a+delta*t;
            if(p.y>=0&&t<nearest)hit=Hit{p,n,true};
        }
        return hit;
    }
};
struct Cylinder:World {
    Vec origin{};float radius=90;
    std::optional<Hit> ray(Vec a,Vec b) override {
        a=a-origin;b=b-origin;const auto d=b-a;
        const double aa=double(d.x)*d.x+double(d.y)*d.y;
        const double bb=2*(double(a.x)*d.x+double(a.y)*d.y);
        const double cc=double(a.x)*a.x+double(a.y)*a.y-double(radius)*radius;
        const double discriminant=bb*bb-4*aa*cc;
        if(aa<1e-12||discriminant<0)return {};
        const double root=std::sqrt(discriminant);
        for(double t:{(-bb-root)/(2*aa),(-bb+root)/(2*aa)}) {
            if(t<=.000001||t>1)continue;
            const auto point=a+d*float(t);const Vec normal=Vec{point.x,point.y,0}.unit();
            if(d.dot(normal)<0)return Hit{point+origin,normal,true};
        }
        return {};
    }
};
struct RoundedCap:World {
    float radius=100,base=160,halfWidth=17.5f;
    std::optional<Hit> ray(Vec a,Vec b) override {
        const auto d=b-a;std::optional<Hit> best;float nearest=2;
        auto take=[&](float t,Vec n) {
            if(t<=.000001f||t>1||t>=nearest||d.dot(n)>=0)return;
            const auto p=a+d*t;if(std::abs(p.x)>halfWidth)return;
            best=Hit{p,n,true};nearest=t;
        };
        if(d.y!=0) {const float t=-a.y/d.y;if((a+d*t).z<=base)take(t,{0,-1,0});}
        if(d.z!=0) {const float t=(base+radius-a.z)/d.z;if((a+d*t).y>=radius)take(t,{0,0,1});}
        const double y=a.y-radius,z=a.z-base,aa=double(d.y)*d.y+double(d.z)*d.z;
        const double bb=2*(y*d.y+z*d.z),cc=y*y+z*z-double(radius)*radius,disc=bb*bb-4*aa*cc;
        if(aa>1e-12&&disc>=0)for(double t:{(-bb-std::sqrt(disc))/(2*aa),(-bb+std::sqrt(disc))/(2*aa)}) {
            const auto p=a+d*float(t);
            if(p.y>=0&&p.y<=radius&&p.z>=base)take(float(t),Vec{0,p.y-radius,p.z-base}.unit());
        }
        return best;
    }
};
static void continuousCylinderTraversal() {
    for(int fps:{30,60,120})for(float direction:{-1.f,1.f})for(float radius:{45.f,150.f})
    for(Vec origin:{Vec{},Vec{110190,77709,2509}}) {
        Cylinder wall;wall.origin=origin;wall.radius=radius;
        Traversal t;t.cfg.approachSeconds=0;t.cfg.radius=31;t.cfg.gap=37;t.cfg.height=138;
        check(t.attach(wall,origin+Vec{0,-radius-37,200},{0,1,0},100),"convex cylinder attach");
        const float dt=1.f/fps;float angle=0,maximumGapError=0,minimumNormalDot=1,longestStall=0;
        int badSamples=0;
        const int limit=int(6.6f*(radius+t.cfg.gap)/t.cfg.sideSpeed*fps*1.5f);
        for(int frame=0;frame<limit&&angle<6.45f;++frame) {
            const Vec before=t.position-origin;const auto r=t.update(wall,{direction,0},dt,100);
            const Vec after=t.position-origin;const Vec radial=Vec{after.x,after.y,0}.unit();
            angle+=direction*std::atan2(before.x*after.y-before.y*after.x,before.x*after.x+before.y*after.y);
            maximumGapError=std::max(maximumGapError,std::abs(std::sqrt(after.x*after.x+after.y*after.y)-radius-t.cfg.gap));
            minimumNormalDot=std::min(minimumNormalDot,t.normal.dot(radial));
            longestStall=std::max(longestStall,t.stalledSeconds());
            if((std::abs(std::sqrt(after.x*after.x+after.y*after.y)-radius-t.cfg.gap)>3||t.normal.dot(radial)<.97f)&&badSamples++<12)
                std::cerr<<"cylinder detail frame="<<frame<<" position="<<after.x<<','<<after.y<<" normal="<<t.normal.x<<','<<t.normal.y
                    <<" radial="<<radial.x<<','<<radial.y<<" step="<<(after-before).length()<<" motion="<<int(r.motion)
                    <<" angle="<<angle<<" reason="<<t.blockedReason<<'\n';
            if(!t.active()||r.released)std::cerr<<"cylinder release="<<r.reason<<" angle="<<angle<<" radius="<<radius<<" fps="<<fps<<'\n';
            check(t.active()&&!r.released&&!r.completed&&!hopMotion(r.motion),
                "a continuously supported cylinder cannot trigger a detour, top-out or false drop");
            check(r.motion==(direction<0?Motion::left:Motion::right),"held lateral input follows the changing cylinder tangent");
            check(std::abs(after.z-200)<.01f,"circling a vertical cylinder cannot acquire vertical drift");
        }
        std::cout<<"cylinder radius="<<radius<<" fps="<<fps<<" direction="<<direction<<" origin="<<origin.x
            <<" radians="<<angle<<" gapError="<<maximumGapError<<" normalDot="<<minimumNormalDot<<" stall="<<longestStall<<'\n';
        check(angle>=6.2831853f&&maximumGapError<3&&minimumNormalDot>.97f&&longestStall<.10f,
            "both directions complete a full real convex orbit while facing and retaining the current surface");
        const auto fixed=t.position;for(int frame=0;frame<fps;++frame)check(t.update(wall,{},dt,100).motion==Motion::hang,"a completed orbit settles into hanging");
        check((fixed-t.position).length()<.001f,"stopping around a cylinder does not chase its changing normal");
    }
    for(float direction:{-1.f,1.f}) {
        Cylinder wall;wall.radius=150;Traversal t;t.cfg.approachSeconds=0;t.cfg.gap=37;t.cfg.radius=31;
        check(t.attach(wall,{0,-187,200},{0,1,0},100),"curved landing attach");
        auto r=t.update(wall,{direction,0,false,false,true},1.f/60,100);
        check(r.motion==(direction<0?Motion::hopLeft:Motion::hopRight),"curved wall has a checked side landing");
        for(int frame=0;frame<90&&t.state==State::action;++frame)r=t.update(wall,{},1.f/60,100);
        if(r.released)std::cerr<<"curved landing release="<<r.reason<<'\n';
        check(t.active()&&!r.released&&t.state!=State::action,"landing uses a fresh check of its destination face instead of the takeoff face");
        const Vec radial=Vec{t.position.x,t.position.y,0}.unit();
        check(t.normal.dot(radial)>.995f,"the landing hands off the verified destination normal");
        for(int frame=0;frame<180;++frame) {
            r=t.update(wall,{direction,0},1.f/60,100);
            check(t.active()&&!r.released,"lateral movement continues after a curved landing without losing support");
        }
    }
}
static void supportedTopRecovery() {
    for(int fixture=0;fixture<4;++fixture) {
        Scene world;world.boxes={{{-500,0,-100},{500,300,1000},true}};
        Traversal t;t.cfg.approachSeconds=0;
        check(t.attach(world,{0,-30,200},{0,1,0},100),"supported summit recovery attach");

        if(fixture!=1)world.boxes.clear();
        if(fixture<2)world.boxes.push_back({{-500,-500,-100},{500,500,193},true});
        if(fixture==2)world.boxes.push_back({{-4,-34,-100},{4,-26,193},true});
        if(fixture==3)world.boxes.push_back({{-500,-500,360},{500,500,380},false});
        const auto r=t.update(world,{0,1,false,true},1.f/60,100);
        if(fixture==0)check(r.completed&&r.released&&!t.active()&&std::abs(t.position.z-200)<.01f,
            "a real footprint below the actor completes standing despite its old vertical wall normal");
        else check(!r.completed&&t.active(),
            "a continuing tall wall, isolated point or empty air cannot masquerade as a supported summit");
    }
    for(int fps:{30,60,120}) {
        RoundedCap cap;Traversal t;t.cfg.approachSeconds=0;t.cfg.radius=31;t.cfg.gap=37;t.cfg.height=138;
        check(t.attach(cap,{0,-37,0},{0,1,0},100),"rounded narrow cap attach");
        Result r;float highestSlope=0;
        for(int frame=0;frame<fps*8&&t.active();++frame) {
            r=t.update(cap,{0,1,false,true},1.f/fps,100);highestSlope=std::max(highestSlope,t.surfaceNormal.z);
        }
        std::cout<<"rounded narrow cap fps="<<fps<<" completed="<<r.completed<<" active="<<t.active()
            <<" position="<<t.position.y<<','<<t.position.z<<" slope="<<highestSlope<<" reason="<<r.reason<<'\n';
        check(r.completed&&r.released&&!t.active()&&(highestSlope>.7f||std::string(r.reason)=="top-out complete"),
            "a rounded summit with real foot support can finish even when both broad palm targets do not fit");
        for(Vec offset:{Vec{},Vec{17,0,0},Vec{-17,0,0},Vec{0,17,0},Vec{0,-17,0}}) {
            const auto hit=cap.ray(t.position+offset+Vec{0,0,12},t.position+offset-Vec{0,0,38});
            check(hit&&hit->normal.z>=.70f,"completed rounded top has actual walkable support beneath its footprint");
        }
    }
    for(bool neighbour:{true,false}) {
        Scene world;world.boxes={{{-500,0,-100},{500,300,1000},true}};
        Traversal t;t.cfg.approachSeconds=0;check(t.attach(world,{0,-30,0},{0,1,0},100),"vertical edge support attach");
        world.boxes={{{-4,0,neighbour?106.f:112.f},{4,100,112},true}};
        Result r;for(int frame=0;frame<120&&t.active();++frame)r=t.update(world,{},1.f/60,100);
        if(neighbour)check(t.active()&&!r.released,"an existing grip retains two real close contacts at a vertical triangle edge");
        else check(!t.active()&&r.released&&!r.completed,"one isolated collision point cannot indefinitely support a hanging actor");
    }
    {
        Scene world;world.boxes={{{-500,0,-100},{500,300,1000},true}};Traversal t;
        check(t.attach(world,{0,-55,200},{0,1,0},100),"entry obstruction reason attach");
        world.boxes.push_back({{-500,-45,0},{500,-43,1000},false});Result r;
        for(int frame=0;frame<30&&t.active();++frame)r=t.update(world,{},1.f/60,100);
        check(r.released&&std::string(r.reason)=="entry path blocked","entry collision releases with a specific actionable reason");
    }
}
static void roundedTopCorners() {
    for(float height:{45.f,80.f,120.f}) {
        Scene world;world.boxes={{{-500,0,-100},{500,300,height},true}};
        Traversal t;t.cfg.approachSeconds=0;
        check(t.attach(world,{0,-30,0},{0,1,0},100),"rounded top path attach");
        auto r=t.update(world,{0,1,false,true},1.f/60,100);
        check(t.state==State::mantle&&t.roundedTopPath(),"clear top preflight selects the small continuous corner path");
        const Vec from=t.topStart(),to=t.topTarget(),apex{from.x,from.y,std::max(from.z,to.z+4)};
        auto blendPoint=[](Vec a,Vec b,float p){p=std::clamp(p,0.f,1.f);return a+(b-a)*(p*p*(3-2*p));};
        auto oldPoint=[&](float phase) {
            if(phase<.60f)return blendPoint(from,apex,(phase-.22f)/.38f);
            if(phase<.88f)return blendPoint(apex,{to.x,to.y,apex.z},(phase-.60f)/.28f);
            return blendPoint({to.x,to.y,apex.z},to,(phase-.88f)/.12f);
        };
        const bool shortened=t.lowTopStep();
        auto sourcePhase=[&](float phase){return shortened?.22f+.78f*phase:phase;};
        float maximumDeviation=0;
        for(int sample=0;sample<=200;++sample)maximumDeviation=std::max(maximumDeviation,
            (t.topPathPoint(sample/200.f)-oldPoint(sourcePhase(sample/200.f))).length());
        check((t.topPathPoint(0)-from).length()==0&&(t.topPathPoint(1)-to).length()==0&&maximumDeviation<4,
            "corner smoothing preserves the real start and destination within the existing four-unit apex margin");
        const float seconds=t.topSeconds();
        for(float originalCorner:{.60f,.88f}) {
            const float corner=shortened?(originalCorner-.22f)/.78f:originalCorner;
            constexpr float epsilon=.0002f;
            const auto before=(t.topPathPoint(corner)-t.topPathPoint(corner-epsilon))/(epsilon*seconds);
            const auto after=(t.topPathPoint(corner+epsilon)-t.topPathPoint(corner))/(epsilon*seconds);
            const float oldSpeed=(oldPoint(sourcePhase(corner+epsilon))-oldPoint(sourcePhase(corner-epsilon))).length()/(2*epsilon*seconds);
            std::cout<<"top corner height="<<height<<" phase="<<corner<<" priorSpeed="<<oldSpeed
                <<" before="<<before.length()<<" after="<<after.length()<<" deviation="<<maximumDeviation<<'\n';
            check(before.length()>5&&after.length()>5&&before.unit().dot(after.unit())>.98f,
                "top path passes both old stop points with nonzero continuous velocity");
        }
        for(int frame=0;frame<150&&t.active();++frame)r=t.update(world,{0,1,false,true},1.f/60,100);
        check(r.completed&&t.position.z>=height,"the continuously rounded path finishes on the actual solid platform");
    }
    {
        Scene clear;clear.boxes={{{-500,0,-100},{500,300,80},true}};
        Traversal reference;reference.cfg.approachSeconds=0;
        check(reference.attach(clear,{0,-30,0},{0,1,0},100),"corner clearance reference attach");
        reference.update(clear,{0,1,false,true},1.f/60,100);
        const auto corner=reference.topPathPoint(19.f/32);
        const float rawFront=reference.topStart().y+reference.cfg.radius;
        const float curvedFront=corner.y+reference.cfg.radius;
        const float halfWidth=(curvedFront-rawFront)*.20f;
        check(halfWidth>.0001f,"rounded preflight fixture isolates a real cut corner");
        Scene blocked=clear;

        const float obstacleZ=corner.z+reference.cfg.radius;
        blocked.boxes.push_back({{-.5f,curvedFront-halfWidth,obstacleZ-.02f},
            {.5f,curvedFront+halfWidth,obstacleZ+.02f},false});
        check(blocked.ray({0,curvedFront,obstacleZ-1},{0,curvedFront,obstacleZ+1}).has_value(),
            "rounded equator trajectory meets the physical corner obstruction");
        check(!blocked.ray({0,rawFront,obstacleZ-1},{0,rawFront,obstacleZ+1}),
            "straight ascent remains outside the corner obstruction");
        Traversal fallback;fallback.cfg.approachSeconds=0;
        check(fallback.attach(blocked,{0,-30,0},{0,1,0},100),"corner-only obstruction attach");
        auto r=fallback.update(blocked,{0,1,false,true},1.f/60,100);
        check(fallback.state==State::mantle&&!fallback.roundedTopPath(),
            "an obstacle inside only the rounded corner retains the separately checked original path");
        for(int frame=0;frame<150&&fallback.active();++frame)r=fallback.update(blocked,{0,1,false,true},1.f/60,100);
        check(r.completed,"the original safe corner remains traversable when rounded preflight is blocked");

        while(reference.progress()<.40f&&reference.active())reference.update(clear,{0,1,false,true},1.f/60,100);
        const float ceiling=reference.position.z+reference.cfg.height+.2f;
        clear.boxes.push_back({{-500,-500,ceiling},{500,500,ceiling+2},false});
        r=reference.update(clear,{0,1,false,true},1.f/60,100);
        check(r.released&&!r.completed&&(std::string(r.reason)=="top-out path changed"||std::string(r.reason)=="mantle support changed"),
            "runtime clearance still prevents a prechecked rounded top from crossing a new solid obstruction");
    }
}
static void summitExitAndProbeBudget() {

    for(int fps:{30,60,120})for(float height:{28.f,48.f,65.f}) {
        Scene world;world.boxes={{{-500,-500,-100},{500,500,0},true},
            {{-500,0,0},{500,300,height},true}};
        Traversal t;
        check(t.attach(world,{0,-30,7},{0,1,0},100),"ground beside low rock attaches to its actual low wall");
        Result r;
        for(int frame=0;frame<fps&&t.active()&&t.state!=State::mantle;++frame)
            r=t.update(world,{0,1,false,true},1.f/fps,100);
        std::cout<<"low rock summit fps="<<fps<<" height="<<height<<" state="<<int(t.state)
            <<" completed="<<r.completed<<" reason="<<r.reason<<'\n';
        check(!r.completed&&t.state==State::mantle&&r.motion==Motion::contextMantle,
            "floor at a low wall base must play a checked top-out instead of immediately restoring standing");
        for(int frame=0;frame<fps*2&&t.active();++frame)r=t.update(world,{0,1,false,true},1.f/fps,100);
        check(r.completed&&t.position.y>12&&t.position.z>height,
            "the low rock finishes only after reaching its actual upper footprint");
    }
    struct CountedScene:Scene {
        int casts{};
        std::optional<Hit> ray(Vec a,Vec b) override {++casts;return Scene::ray(a,b);}
    };
    for(float height:{45.f,80.f,120.f}) {
        CountedScene world;world.boxes={{{-500,0,-100},{500,300,height},true}};
        Traversal t;t.cfg.approachSeconds=0;
        check(t.attach(world,{0,-30,0},{0,1,0},100),"top probe budget fixture attach");
        int peakHang=0;
        for(int frame=0;frame<60;++frame) {
            world.casts=0;const auto r=t.update(world,{},1.f/60,100);
            peakHang=std::max(peakHang,world.casts);
            check(t.active()&&!r.completed&&t.state==State::ledge,"waiting at a ledge never commits an unrequested top-out");
        }
        std::cout<<"top probe height="<<height<<" peak stationary casts="<<peakHang<<'\n';

        check(peakHang<=150,"stationary ledge queries must not rerun the full rounded-path preflight");
        world.casts=0;t.update(world,{0,1,false,true},1.f/60,100);
        const int committed=world.casts;
        check(t.state==State::mantle&&t.roundedTopPath()&&committed>peakHang+300,
            "committing a mantle still performs the complete rounded collision preflight exactly when needed");
        int peakPlayback=0;Result r;
        for(int frame=0;frame<150&&t.active();++frame) {
            world.casts=0;r=t.update(world,{0,1,false,true},1.f/60,100);
            peakPlayback=std::max(peakPlayback,world.casts);
        }
        std::cout<<"top probe commit="<<committed<<" peak playback="<<peakPlayback<<'\n';
        check(r.completed&&peakPlayback<=40,"mantle playback performs current-step clearance without repeating preflight");
    }
}
static void changedFutureTopClearance() {
    for(int fps:{30,60,120}) {
        Scene world;world.boxes={{{-500,0,-100},{500,300,120},true}};
        Traversal t;t.cfg.approachSeconds=0;
        check(t.attach(world,{0,-30,0},{0,1,0},100),"future top obstruction attaches to the actual source");
        t.update(world,{0,1,false,true},1.f/fps,100);
        check(t.state==State::mantle,"future obstruction starts from a fully checked mantle");
        const Vec end=t.topPathPoint(1);
        const Box obstacle{{-5,end.y-3,end.z+t.cfg.height-2},{5,end.y+3,end.z+t.cfg.height+2},false};
        world.boxes.push_back(obstacle);
        auto r=t.update(world,{0,1,false,true},1.f/fps,100);
        check(t.active()&&!r.released&&!r.completed,"a changed future head space does not invalidate the current clear segment");
        for(int frame=0;frame<fps*4&&t.active();++frame) {
            r=t.update(world,{0,1,false,true},1.f/fps,100);
            const float x=std::max({obstacle.lo.x-t.position.x,0.f,t.position.x-obstacle.hi.x});
            const float y=std::max({obstacle.lo.y-t.position.y,0.f,t.position.y-obstacle.hi.y});
            const float z=std::max({obstacle.lo.z-(t.position.z+t.cfg.height-t.cfg.radius),0.f,t.position.z+t.cfg.radius-obstacle.hi.z});
            check(x*x+y*y+z*z>=t.cfg.radius*t.cfg.radius-.02f,"live mantle segments stop before the physical capsule reaches a new future obstruction");
        }
        check(r.released&&!r.completed&&std::string(r.reason)=="top-out path changed","future obstruction is rejected before completing the mantle");
    }
}
static void flowingParkourActions() {
    for(int fps:{30,48,120})for(bool fancy:{false,true})
    for(Vec direction:{Vec{0,1,0},Vec{-1,0,0},Vec{1,0,0},Vec{-1,1,0},Vec{1,1,0},Vec{}}) {
        Scene wall;wall.boxes={{{-5000,0,-1000},{5000,300,5000},true}};
        Traversal t;t.cfg.approachSeconds=0;t.cfg.fancyJumps=fancy;
        check(t.attach(wall,{0,-30,200},{0,1,0},100),"wall-run no-jump fixture attach");
        const float dt=1.f/fps;Input moving{direction.x,direction.y,false,false,false,false,true};
        for(int frame=0;frame<fps;++frame)t.update(wall,moving,dt,100);
        auto space=moving;space.hop=true;
        for(int frame=0;frame<fps;++frame) {
            const Vec before=t.position;const auto r=t.update(wall,space,dt,100);
            check(t.active()&&t.state!=State::action&&!hopMotion(r.motion)&&!r.released,
                "Space cannot trigger an upward/lateral/diagonal wall-running jump or flip, including a resting Shift chord");
            check(r.staminaCost<1&&std::abs((t.position-before).dot(t.normal))<.01f,
                "disabled wall-running jumps do not charge an action or depart from the wall");
        }
        const auto release=t.update(wall,{0,-1,true,false,false,true,true},dt,100);
        check(release.motion==Motion::dropBack&&t.state==State::action,
            "S+Space still starts the checked physical departure when wall-running jumps are disabled");
    }
    Scene lip;lip.boxes={{{-5000,0,-1000},{5000,300,5000},true},{{-5000,-25,600},{5000,0,680},true}};
    Traversal t;t.cfg.approachSeconds=0;t.cfg.radius=31;t.cfg.gap=37;t.cfg.height=138;
    check(t.attach(lip,{0,-37,0},{0,1,0},100),"wall-run obstruction fixture attach");
    float maximumStall=0;
    for(int frame=0;frame<480;++frame) {
        const auto r=t.update(lip,{0,1,false,false,false,false,true},1.f/60,100);
        check(t.active()&&!hopMotion(r.motion)&&t.state!=State::action,
            "held Shift cannot turn a blocking rock into an automatic kick or climbing detour");
        maximumStall=std::max(maximumStall,t.stalledSeconds());
    }
    check(maximumStall>.12f&&t.position.z<600,"the automatic-jump exclusion actually encounters its solid obstruction");
    for(int fps:{30,48,120}) {
        Scene summit;summit.boxes={{{-500,0,-100},{500,300,600},true}};
        Traversal runner;runner.cfg.approachSeconds=0;runner.cfg.radius=31;runner.cfg.gap=37;runner.cfg.height=138;
        check(runner.attach(summit,{0,-37,0},{0,1,0},100),"running automatic top-out attach");
        Result r;bool sawRun=false,sawTop=false;
        for(int frame=0;frame<fps*6&&runner.active();++frame) {
            r=runner.update(summit,{0,1,false,true,true,false,true},1.f/fps,100);
            sawRun|=runMotion(r.motion);sawTop|=runner.state==State::mantle;
            check(!hopMotion(r.motion),"running Space cannot insert a jump before or during an automatic measured summit");
        }
        check(sawRun&&sawTop&&r.completed&&!runner.active()&&runner.position.z>=600,
            "removing running jumps preserves the actual automatic top-out from a sustained wall run");
        for(Vec offset:{Vec{},Vec{17,0,0},Vec{-17,0,0},Vec{0,17,0},Vec{0,-17,0}}) {
            const auto floor=summit.ray(runner.position+offset+Vec{0,0,12},runner.position+offset-Vec{0,0,38});
            check(floor&&floor->normal.z>=.70f,"running top-out ends over a complete real standing footprint");
        }
    }
}
static void ordinaryIdleWithoutFootSupport() {
    for(int fps:{30,60,120})for(bool restoreSupport:{false,true})for(bool hop:{false,true}) {
        Scene world;world.boxes={{{-500,0,70},{500,300,120},true}};
        Traversal t;t.cfg.approachSeconds=0;
        check(t.attach(world,{0,-30,0},{0,1,0},100),"ledge-only idle fixture attaches to a real wall");
        const auto idlePosition=t.position;
        auto r=t.update(world,{},1.f/fps,100);
        check(t.state!=State::action&&r.motion==Motion::hang,"missing foot support keeps ordinary stationary motion");
        for(int frame=0;frame<fps;++frame) {
            r=t.update(world,{},1.f/fps,100);
            check(t.state!=State::action&&r.motion==Motion::hang&&(t.position-idlePosition).length()<.00001f,
                "ledge-only idle neither selects retired transfers nor moves the body");
        }
        if(restoreSupport) {
            world.boxes[0].lo.z=-100;
            r=t.update(world,{},1.f/fps,100);
            check(t.state!=State::action&&r.motion==Motion::hang,"returning real foot support keeps ordinary stationary motion");
        }
        const auto before=t.position;const auto state=t.state;
        Input intent{1,0,false,false,hop};
        t.update(world,intent,0,100);
        check(t.state==state&&(t.position-before).length()==0,
            "zero time cannot change stationary support state or move geometry");
        r=t.update(world,intent,1.f/fps,100);
        if(hop)check(t.state==State::action&&r.motion==Motion::hopRight&&r.staminaCost==15,
            "Space from ordinary ledge idle starts a checked ordinary hop");
        else check(t.state!=State::action&&r.motion==Motion::right&&t.position.x>before.x,
            "new lateral movement leaves ordinary ledge idle on its first actual frame");
        check(t.active()&&!r.released,"resuming movement preserves wall ownership");
    }
}
static void releasedShiftTakeoffs() {
    int cases=0;
    for(int fps:{30,60,120})for(int releasedFrames:{0,1})
    for(Vec direction:{Vec{0,1,0},Vec{-1,0,0},Vec{1,0,0},Vec{-1,1,0},Vec{1,1,0}}) {
        Scene wall;wall.boxes={{{-5000,0,-1000},{5000,300,5000},true}};
        Traversal t;t.cfg.approachSeconds=0;
        check(t.attach(wall,{0,-30,200},{0,1,0},100),"Shift-release no-jump fixture attach");
        const float dt=1.f/fps;Input moving{direction.x,direction.y,false,false,false,false,true};
        for(int frame=0;frame<fps;++frame)t.update(wall,moving,dt,100);
        moving.run=false;
        for(int frame=0;frame<releasedFrames;++frame)t.update(wall,moving,dt,100);
        auto jump=moving;jump.hop=true;const auto ignored=t.update(wall,jump,dt,100);
        check(!hopMotion(ignored.motion)&&t.state!=State::action&&ignored.staminaCost<1,
            "Space coincident with Shift release cannot reintroduce a removed running jump through momentum inheritance");
        for(int frame=0;frame<fps;++frame)t.update(wall,moving,dt,100);
        const auto r=t.update(wall,jump,dt,100);
        const Motion ordinary=direction.x<0?Motion::hopLeft:direction.x>0?Motion::hopRight:Motion::hopUp;
        check(r.motion==ordinary&&!t.runningAction()&&r.staminaCost==15,
            "a later deliberate ordinary climbing Space remains available after running fully decelerates");
        ++cases;
    }
    std::cout<<"released Shift jump exclusions="<<cases<<" with ordinary climbing jump recovery\n";
}
static void explicitAirCatchSafety() {
    Scene wall;wall.boxes={{{-5000,0,-1000},{5000,300,5000},true}};
    Traversal t;t.cfg.approachSeconds=0;
    check(t.attach(wall,{0,-37,1000},{0,1,0},100),"air regrab setup attaches");
    t.stop();check(!t.attach(wall,{0,-37,1000},{0,1,0},100),"ordinary attachment still respects post-release cooldown");
    check(t.attach(wall,{0,-37,1000},{0,1,0},100,150,true),
        "a separately authorized fresh airborne chord can regrab a reachable wall during cooldown");
    t.stop();Scene absent;
    check(!t.attach(absent,{0,-37,1000},{0,1,0},100,150,true),"airborne intent never substitutes for actual wall support");
    Scene blocked=wall;blocked.boxes.push_back({{-500,-45,900},{500,-43,1300},false});
    check(!t.attach(blocked,{0,-55,1000},{0,1,0},100,150,true),"airborne cooldown exception cannot cross a solid obstruction");
    check(!t.attach(wall,{0,-500,1000},{0,1,0},100,150,true),"airborne catch keeps the original bounded discovery reach");
}
static void reachableSupportContracts() {
    int failures=0;
    auto verify=[&](bool success,const char* message) {
        if(!success) {++failures;std::cout<<"support regression: "<<message<<'\n';}
    };
    for(int fps:{30,60,120}) {
        const float dt=1.f/fps;
        for(bool moving:{false,true}) {
            Scene wall;wall.boxes={{{-500,0,-100},{500,300,1000},true}};
            Traversal t;t.cfg.approachSeconds=0;
            check(t.attach(wall,{0,-30,100},{0,1,0},100),"support range fixture attach");
            const auto start=t.position;
            wall.boxes[0].lo.y=70;
            Result r;float farthest=0;
            for(int frame=0;frame<fps*2&&t.active();++frame) {
                r=t.update(wall,moving?Input{0,1}:Input{},dt,100);
                farthest=std::max(farthest,(t.position-start).length());
            }
            verify(r.released&&!t.active()&&farthest<.001f,
                "a distant wall cannot renew grip or pull an unsupported body through air");
        }
        {
            Scene wall;wall.boxes={{{-500,0,-100},{500,300,1000},true}};
            Traversal t;t.cfg.approachSeconds=0;
            check(t.attach(wall,{0,-30,0},{0,1,0},100),"foot-only support fixture attach");

            wall.boxes[0].hi={500,10,32};
            const auto start=t.position;Result r;
            for(int frame=0;frame<fps*2&&t.active();++frame)r=t.update(wall,{},dt,100);
            verify(r.released&&!t.active()&&(t.position-start).length()<.001f,
                "a foot-height fragment alone cannot indefinitely renew a hanging hand grip");
        }
        {
            Scene wall;wall.boxes={{{-500,0,-100},{500,300,1000},true}};
            Traversal t;t.cfg.approachSeconds=0;
            check(t.attach(wall,{0,-30,0},{0,1,0},100),"remote top fixture attach");
            const auto start=t.position;
            wall.boxes[0].lo.y=70;wall.boxes[0].hi.z=45;
            const auto r=t.update(wall,{0,1,false,true},dt,100);
            verify(t.state!=State::mantle&&!r.completed&&(t.position-start).length()<.001f,
                "solid palms on a remote platform do not make that platform reachable");
        }
        for(bool lowContactRemains:{false,true}) {
            Scene wall;wall.boxes={{{-500,0,-100},{500,300,1000},true}};
            Traversal t;t.cfg.approachSeconds=0;
            check(t.attach(wall,{0,-30,100},{0,1,0},100),"temporary seam fixture attach");
            t.update(wall,{0,1},dt,100);const auto start=t.position;
            const auto original=wall.boxes;
            if(lowContactRemains)wall.boxes[0].hi={500,10,start.z+40};
            else wall.boxes.clear();
            for(int frame=0;frame<3;++frame) {
                const auto r=t.update(wall,{1,1},dt,100);
                verify(t.active()&&!r.released&&(t.position-start).length()<.001f,
                    "a short missing seam holds the last supported anchor without moving into air");
            }
            wall.boxes=original;const auto resumed=t.update(wall,{0,1},dt,100);
            verify(t.active()&&!resumed.released&&t.position.z>start.z,
                "nearby support returning after a short seam resumes movement without detachment");
        }
    }
    Scene toe;toe.boxes={{{-500,0,-100},{500,10,32},true}};
    Traversal entry;entry.cfg.approachSeconds=0;
    verify(!entry.attach(toe,{0,-30,0},{0,1,0},100)&&entry.lastFailure==AttachFailure::support,
        "new attachment also rejects a low fragment with no reachable grip or feasible top-out");
    std::cout<<"reachable support contract failures="<<failures<<'\n';
    check(failures==0,"reachable support contracts must all hold");
}
int main() {
 try {
    reachableSupportContracts();
    flowingParkourActions();
    ordinaryIdleWithoutFootSupport();
    releasedShiftTakeoffs();explicitAirCatchSafety();
    continuousCylinderTraversal();
    supportedTopRecovery();
    roundedTopCorners();
    summitExitAndProbeBudget();changedFutureTopClearance();
    Scene s; s.boxes={{{-500,0,-100},{500,300,1000},true}};
    Traversal t; t.cfg.approachSeconds=0;
    check(!t.attach(s,{0,-40,0},{0,1,0},0),"no stamina must reject");
    check(!t.attach(s,{0,-200,0},{0,1,0},100),"distant wall must reject");
    check(t.attach(s,{0,-40,0},{0,1,0},100),"near static wall must attach");
    check(std::abs(t.position.y+30)<.001f,"body offset");
    float drain=0;
    for(int i=0;i<60;i++) drain+=t.update(s,{0,1},1.f/60,100).staminaCost;
    check(std::abs(t.position.z-t.cfg.climbSpeed)<.1f,"one second ascent");
    check(std::abs(drain-10)<.01f,"frame independent stamina");
    const auto held=t.position;
    auto still=t.update(s,{},.02f,100);
    check((held-t.position).length()<.001f&&still.motion==Motion::hang,"stationary hang");
    t.update(s,{1,0},.05f,100); check(t.position.x>held.x,"right must move right");
    {
        Scene blocked=s;
        blocked.boxes.push_back({{t.position.x+t.cfg.radius+.1f,-200,-100},{500,100,1000},true});
        check(t.update(blocked,{1,0},.02f,100).motion==Motion::right,"one blocked frame must not restart hang");
        for(int i=0;i<20;++i)t.update(blocked,{1,0},.02f,100);
        check(t.stalledSeconds()>.18f&&t.update(blocked,{1,0},.02f,100).motion==Motion::hang,"sustained obstruction must eventually settle into hang");
        check(t.update(s,{1,0},.02f,100).motion==Motion::right,"movement resumes when the obstruction clears");
    }
    check(t.update(s,{0,0,true},.02f,100).motion==Motion::drop,"release starts captured let-go");
    Result drop;
    for(int i=0;i<20&&t.active();++i)drop=t.update(s,{},.02f,100);
    check(drop.released&&!t.active(),"drop returns gravity after bounded let-go time");
    check(!t.attach(s,{0,-30,0},{0,1,0},100),"regrab cooldown");
    t.reset(); s.boxes[0].climbable=false;
    check(!t.attach(s,{0,-30,0},{0,1,0},100),"movable object rejected");
    s.boxes[0].climbable=true; s.boxes[0].hi.z=120;
    check(t.attach(s,{0,-30,0},{0,1,0},100),"ledge attach");
    t.update(s,{},.02f,100); check(t.state==State::ledge,"ledge detected");
    auto mantle=t.update(s,{0,0,false,true},.02f,100);
    check(t.state==State::mantle&&mantle.staminaCost==12,"mantle start");
    for(int i=0;i<160&&t.active();i++) t.update(s,{},.02f,100);
    check(!t.active()&&t.position.z>120&&t.position.y>12,"mantle reaches supported top with both feet over the ledge");
    t.reset(); s.boxes.push_back({{-100,-100,140},{100,100,170},true});
    check(t.attach(s,{0,-30,0},{0,1,0},100),"low ceiling initial attach");
    t.update(s,{0,0,false,true},.02f,100);
    check(t.state!=State::mantle,"low ceiling prevents mantle");
    check(t.update(s,{},.02f,0).released,"exhaustion drops");
    t.reset(); check(!t.attach(s,{NAN,0,0},{0,1,0},100),"NaN rejects");
    s.boxes={{{-500,0,-100},{500,300,1000},true}};
    t.reset(); check(t.attach(s,{0,-30,0},{0,1,0},100),"reattach for collision checks");
    auto initial=t.position;
    t.update(s,{1,1},10,100);
    check((t.position-initial).length()<5,"frame hitch cannot teleport through geometry");
    s.boxes.clear();
    check(!t.update(s,{},.02f,100).released,"one missed surface frame retains the checked anchor");
    for(int i=0;i<70&&t.active();++i)t.update(s,{},.02f,100);
    check(!t.active(),"persistently missing support eventually releases");
    s.boxes={{{-500,0,-100},{500,25,120},true}};
    t.reset(); check(t.attach(s,{0,-30,0},{0,1,0},100),"narrow wall attach");
    t.update(s,{0,0,false,true},.02f,100);
    check(t.state!=State::mantle,"insufficient platform depth prevents mantle");
    s.boxes={{{-500,0,-100},{500,300,1000},true},{{25,-100,-100},{40,100,1000},true}};
    t.reset(); check(t.attach(s,{0,-30,0},{0,1,0},100),"side blocker attach");
    for(int i=0;i<20;i++) t.update(s,{1,0},.05f,100);
    check(t.position.x<4,"body radius prevents lateral clipping into blocker");
    s.boxes={{{-500,0,-100},{500,300,1000},true}};
    t.reset(); check(!t.attach(s,{0,-100,0},{0,1,0},100,30)&&t.lastFailure==AttachFailure::tooFar,
        "automatic attach must not teleport to a distant wall");
    check(t.attach(s,{0,-55,0},{0,1,0},100,30),"automatic attach within approach distance");

    Slope slope;
    t.reset();check(t.attach(slope,{0,-30,0},{0,1,0},100,30),"steep mountain slope attaches");
    for(int i=0;i<60;++i) t.update(slope,{0,1},1.f/60,100);
    check(t.active()&&t.position.z>37&&t.position.y>0,"ascent follows receding mountain surface");
    check((t.position+Vec{0,22,6}).dot(slope.normal)>0,"feet radius remains outside slope");
    slope.normal={0,-.6f,.8f};
    t.reset();check(!t.attach(slope,{0,-30,0},{0,1,0},100),"walkable gentle slope is not a climbing wall");

    RoughWall rough;rough.boxes={{{-500,0,-100},{500,300,1000},true}};
    t.reset();check(t.attach(rough,{0,-40,0},{0,1,0},100),"uneven rock normals accepted");

    s.boxes={{{-500,0,-100},{500,300,45},true}};
    t.reset();check(t.attach(s,{0,-40,0},{0,1,0},100),"low rock detected below chest");
    t.update(s,{0,1,false,true},.02f,100);
    check(t.state==State::mantle,"climb key can request auto mantle on low rock");
    for(int i=0;i<180&&t.active();i++)t.update(s,{0,1,false,true},.02f,100);
    check(!t.active()&&t.position.z>45&&t.position.y>12,"automatic mantle reaches supported rock top");
    s.boxes={{{-500,0,-100},{500,300,55},true},{{-500,65,55},{500,300,1000},true}};
    t.reset();check(t.attach(s,{0,-35,0},{0,1,0},100,30),"near rock base wins over a distant chest-height face");
    check(std::abs(t.position.y+30)<.01f,"stepped rock attaches at the near lower face");
    Summit summit;t.reset();
    check(t.attach(summit,{0,-40,0},{0,1,0},100),"sloped summit attaches");
    bool completed=false;
    for(int i=0;i<150&&t.active();++i)completed=t.update(summit,{0,1,false,true},.02f,100).completed;
    check(completed&&t.position.y>12&&t.position.z>95,"uneven walkable summit supports automatic top-out");

    s.boxes={{{-500,0,-100},{500,300,1000},true}};
    t.reset();t.cfg.approachSeconds=.24f;
    check(t.attach(s,{0,-55,0},{0,1,0},100),"approach catch");
    check(t.state==State::approach&&std::abs(t.position.y+55)<.01f,"catch starts at actual position");
    float lastY=t.position.y;
    for(int i=0;i<16;++i) {
        t.update(s,{},1.f/60,100);
        check(t.position.y>=lastY&&t.position.y-lastY<3,"catch path has no teleport");lastY=t.position.y;
    }
    check(t.state==State::wall&&t.reachProgress()==1,"approach progress remains available for final pose");
    {
        Keys k;k.w=true;check(!approachIntent(k),"W alone must never attach");
        auto i=wallInput(k,false);check(i.y==1&&!i.run,"W climbs");
        k.shift=true;i=wallInput(k,false);check(i.y==1&&i.run,"Shift switches to running while attached");
        check(!approachIntent(k),"ordinary Shift plus W cannot approach a wall");
        k.space=true;i=wallInput(k,true);check(!approachIntent(k)&&!i.hop&&!i.release,"sprint jumping remains native outside traversal and cannot trigger a wall-run leap");
        k.a=k.d=true;check(approachIntent(k),"the full Space+A+W+D chord deliberately requests climbing");
        i=wallInput(k,true,true,true);check(!i.hop&&!i.release,"the catch's held Space cannot immediately start another hop");
        k.s=true;i=wallInput(k,true);check(i.release&&!i.hop&&!i.backDrop&&!i.run&&i.x==0&&i.y==0,"A+S+D+Space has priority and lets go without outward intent");
        k.a=k.d=false;i=wallInput(k,true);check(i.release&&!i.hop&&i.backDrop&&!i.run,"S+Space back jump has priority and cannot start a downward run");
        k={};check(wallInput(k,false).y==0&&!approachIntent(k),"releasing WASD rests without automatic ascent");
        k.shift=true;check(!approachIntent(k)&&wallInput(k,false).y==0,"Shift alone cannot attach");
        k.space=true;check(!approachIntent(k),"Space without the full chord remains a normal jump");
        k.w=true;check(!approachIntent(k),"W+Shift+Space remains native until the entry chord is complete");
        k.a=k.d=true;check(approachIntent(k),"both A and D complete the default entry chord");
        k.s=true;check(!approachIntent(k),"S blocks entry");
        for(unsigned mask=0;mask<32;++mask) {
            Keys chord;chord.w=(mask&1)!=0;chord.a=(mask&2)!=0;chord.d=(mask&4)!=0;chord.space=(mask&8)!=0;chord.shift=(mask&16)!=0;
            check(approachIntent(chord)==(chord.w&&chord.a&&chord.d&&chord.space),"entry requires every chord member independently of Shift");
            chord.s=true;check(!approachIntent(chord),"S vetoes every entry combination");
        }
        ClimbEntryIntent edge;Keys staged;
        for(unsigned scan:{0x39u,0x11u,0x1eu}) {
            keyboardKey(staged,scan,true);check(!edge.sample(staged).requested,"incomplete default chord cannot grab");
        }
        keyboardKey(staged,0x20,true);const auto firstEntry=edge.sample(staged);
        check(firstEntry.requested&&firstEntry.fresh&&firstEntry.began,"the final chord key immediately requests climbing");
        const auto held=edge.sample(staged);check(held.requested&&!held.fresh,"holding the full chord retries without fresh air intent");
        edge.blockUntilRelease();check(!edge.sample(staged).requested,"departure requires chord release before reacquisition");
        staged.a=false;edge.sample(staged);staged.a=true;check(edge.sample(staged).began,"repressing a missing chord key starts a new entry");
        JumpGrabGate jump;jump.request({1,0,0},true);jump.tick(.05f);
        check(jump.pending()&&jump.facing().x==1&&jump.startedNativeJump(),"preflight preserves facing and an already native-jumped request after key-up");
        for(int f=0;f<20;++f)jump.tick(.05f);check(!jump.pending(),"expired jump request must not auto-grab a later wall");
        jump.request();check(!jump.startedNativeJump(),"fresh entry does not inherit an earlier native jump");
        jump.cancel();check(!jump.pending()&&!jump.startedNativeJump(),"release/load/cancel removes jump intent");
        struct Event {int id;Event* next;};
        Event end{3,nullptr},spaceTail{32,&end},middle{2,&spaceTail},spaceHead{32,&middle};
        Event* head=&spaceHead;Event* tail=&end;
        removeInputEvents(head,tail,[](auto* e){return e->id==32;});
        check(head==&middle&&middle.next==&end&&tail==&end,"attached Space removal preserves unrelated events and order");
        removeInputEvents(head,tail,[](auto* e){return e->id==3;});
        check(head==&middle&&!middle.next&&tail==&middle,"tail is repaired after consuming last queued key");
        removeInputEvents(head,tail,[](auto*){return true;});
        check(!head&&!tail,"fully consumed queue remains valid");
    }
    {
        Traversal descending;descending.cfg.approachSeconds=0;
        check(descending.attach(s,{0,-40,200},{0,1,0},100),"downward run exclusion attach");
        for(int frame=0;frame<30;++frame)descending.update(s,{0,1,false,false,false,false,true},1.f/60,100);
        check(descending.wallRunning(),"downward exclusion begins from an actual upward run");
        Input downInput{0,-1,false,false,false,false,true};downInput.modeBlend=1;
        const auto before=descending.position;const auto r=descending.update(s,downInput,.02f,100);
        check(!descending.wallRunning()&&r.motion==Motion::down,"Shift+S switches to downward climbing");
        check(std::abs(before.z-descending.position.z-descending.cfg.downSpeed*.02f)<.01f,
            "downward climbing never inherits run speed or external run blend");
        check(std::abs(r.staminaCost-descending.cfg.drain*.02f)<.0001f,"downward climbing uses normal stamina even when Shift remains held");
        downInput.x=1;const auto diagonal=descending.update(s,downInput,.02f,100);
        check(!descending.wallRunning()&&!runMotion(diagonal.motion),"Shift+S+D cannot select a diagonal downward run");
    }
    {
        for(int fixture=0;fixture<4;++fixture) {
            Scene falling;falling.boxes={{{-500,0,-100},{500,300,1000},true}};
            Traversal back;back.cfg.approachSeconds=0;
            check(back.attach(falling,{0,-30,200},{0,1,0},100),"physical back-jump fixture attach");
            const Vec start=back.position;
            if(fixture==1)falling.boxes.push_back({{-500,-500,0},{500,-55,1000},false});
            if(fixture==2)falling.boxes.push_back({{-500,-500,326},{500,0,400},false});
            Keys releaseKeys;releaseKeys.s=releaseKeys.space=true;
            auto r=back.update(falling,wallInput(releaseKeys,true),.02f,100);
            check(r.motion==Motion::dropBack&&back.state==State::action,"S+Space starts a visible checked foot push");
            float elapsed=.02f;bool changed=false;
            for(int frame=0;frame<30&&back.active();++frame) {

                if(fixture==3&&!changed&&back.actionProgress()>.55f) {
                    falling.boxes.push_back({{-500,-500,0},{500,-72,1000},false});changed=true;
                }
                check(r.releaseVelocity.length()==0,"push preparation cannot emit the physical impulse early");
                r=back.update(falling,{},.02f,100);
                elapsed+=.02f;const Vec travel=back.position-start;
                check(travel.dot(back.normal)>=-.001f&&travel.dot(back.normal)<=220*(.32f/3+.02f)+.001f&&
                    travel.z>=-.001f&&travel.z<=90*(.32f/3+.02f)+.001f,
                    "only the short prechecked outward foot push may precede physical falling");
            }
            check(r.released&&!back.active(),"back jump always returns ownership to normal gravity");
            check(elapsed<=.36f&&(fixture==3||elapsed>=.30f),"a complete push precedes physical release unless a new obstacle invalidates it");
            if(fixture==0) {
                const float travelTime=.32f/3+std::max(0.f,elapsed-.32f);
                check(std::abs((back.position-start).dot(back.normal)-220*travelTime)<.002f&&
                    std::abs(back.position.z-start.z-90*travelTime)<.002f,
                    "unobstructed captured push reaches a velocity-matched departure including the last update's remaining time");
                check(r.releaseVelocity.dot(back.normal)>219&&std::abs(r.releaseVelocity.z-90)<.001f,
                    "unobstructed release gives one real outward and upward physical impulse");
            }
            if(fixture==1)check(r.releaseVelocity.length()<.001f,"blocked departure safely releases without pushing through the rear obstacle");
            if(fixture==2)check(r.releaseVelocity.dot(back.normal)>219&&r.releaseVelocity.z==0,
                "low headroom permits only the checked horizontal departure");
            if(fixture==3)check(changed&&r.releaseVelocity.dot(back.normal)>=0&&r.releaseVelocity.dot(back.normal)<220&&back.position.y>-50.001f,
                "a new rear obstruction safely stops or reduces departure without moving the capsule through it");
            check(back.update(falling,{},.02f,100).releaseVelocity.length()==0,"physical impulse is emitted once, never on subsequent free-fall frames");
        }
    }
    {
        Traversal climb,run;climb.cfg.approachSeconds=run.cfg.approachSeconds=0;run.cfg.runSpeed=370;
        check(climb.attach(s,{0,-40,0},{0,1,0},100)&&run.attach(s,{0,-40,0},{0,1,0},100),"mode comparison attach");
        float climbDrain=0,runDrain=0;
        for(int frame=0;frame<60;++frame) {
            climbDrain+=climb.update(s,{0,1},1.f/60,100).staminaCost;
            runDrain+=run.update(s,{0,1,false,false,false,false,true},1.f/60,100).staminaCost;
        }
        check(std::abs(runDrain-2*climbDrain)<.001f,"wall run stamina is exactly twice climb over equal time");
        check(run.position.z>climb.position.z*2&&run.wallRunning(),"run travels faster without a release/reattach");
        const auto before=run.position;
        run.update(s,{0,1},1.f/60,100);
        check(!run.wallRunning()&&run.active()&&(run.position-before).length()<=run.cfg.runSpeed/60+.01f,"release Shift decelerates without a position jump");
        for(int i=0;i<24;++i)run.update(s,{0,1},1.f/60,100);
        const auto settled=run.position;run.update(s,{0,1},1.f/60,100);
        check(std::abs((run.position-settled).length()-run.cfg.climbSpeed/60)<.01f,"run-to-climb deceleration finishes at climb speed");
        run.update(s,{},1.f/60,100);check(run.active(),"release movement holds wall");
    }
    {
        for(float blend:{0.f,.25f,.5f,.75f,1.f}) {
            Traversal mixed;mixed.cfg.approachSeconds=0;mixed.cfg.runSpeed=350;
            check(mixed.attach(s,{0,-40,0},{0,1,0},100),"mode blend movement attach");
            Input input{0,1,false,false,false,false,true};input.modeBlend=blend;
            const float before=mixed.position.z;const auto result=mixed.update(s,input,.02f,100);
            check(std::abs(mixed.position.z-before-.02f*(mixed.cfg.climbSpeed+(350-mixed.cfg.climbSpeed)*blend))<.01f,
                "movement speed uses the displayed climb-to-sprint weight");
            check(std::abs(result.staminaCost-.4f)<.0001f,"native mode retains two-times moving stamina through blending");
        }
    }
    {

        Scene narrowing;narrowing.boxes={{{-500,0,-100},{500,300,5000},true},
            {{-500,-200,300},{500,-75,600},true}};
        Traversal runner;runner.cfg.approachSeconds=0;
        check(runner.attach(narrowing,{0,-40,0},{0,1,0},100),"narrowing runway attach");
        bool ranBefore=false,climbedInside=false,ranAfter=false;float longest=0;
        for(int frame=0;frame<600;++frame) {
            auto r=runner.update(narrowing,{0,1,false,false,false,false,true},1.f/60,100);
            check(runner.active()&&runner.state!=State::action,"narrow runway fallback retains grip without an automatic hop");
            ranBefore|=runner.wallRunning()&&runner.position.z<250;
            climbedInside|=!runner.wallRunning()&&runner.position.z>300&&runner.position.z<600&&r.motion==Motion::up;
            ranAfter|=runner.wallRunning()&&runner.position.z>640;
            longest=std::max(longest,runner.stalledSeconds());
        }
        check(ranBefore&&climbedInside&&ranAfter&&runner.position.z>700&&longest<.12f,
            "destination-only runway rejection must switch to climb and resume running after the passage");

        narrowing.boxes.push_back({{-500,-500,500},{500,0,600},true});
        Traversal blocked;blocked.cfg.approachSeconds=0;
        check(blocked.attach(narrowing,{0,-40,0},{0,1,0},100),"narrow runway with solid ceiling attach");
        bool usedFallback=false;
        for(int frame=0;frame<600;++frame) {
            auto r=blocked.update(narrowing,{0,1,false,false,false,false,true},1.f/60,100);
            usedFallback|=!blocked.wallRunning()&&blocked.position.z>290;
            check(blocked.active()&&blocked.position.z<375.01f&&!hopMotion(r.motion),
                "runway fallback cannot bypass the body's solid ceiling collision");
        }
        check(usedFallback,"ceiling regression reaches the narrowed climbing fallback");
    }
    {
        Traversal jumping;jumping.cfg.approachSeconds=.32f;
        check(jumping.attach(s,{0,-55,0},{0,1,0},100),"jump catch attach");jumping.entry(Motion::jumpCatch,true);
        float lift=0;for(int f=0;f<21;++f){jumping.update(s,{},1.f/60,100);lift=std::max(lift,jumping.position.z);}
        check(lift>15&&jumping.active(),"Shift catch has a checked physical jump arc");
        Traversal alreadyJumped;alreadyJumped.cfg.approachSeconds=.32f;
        check(alreadyJumped.attach(s,{0,-55,200},{0,1,0},100),"midair catch attach");alreadyJumped.entry(Motion::jumpCatch,false);
        for(int frame=0;frame<20;++frame) {
            alreadyJumped.update(s,{},1.f/60,100);
            check(std::abs(alreadyJumped.position.z-200)<.001f,"an existing native jump does not receive a second plugin takeoff arc");
        }
        Traversal immediate;immediate.cfg.approachSeconds=0;
        check(immediate.attach(s,{0,-30,200},{0,1,0},100),"zero-duration catch attach");
        Keys chord;chord.w=chord.a=chord.d=chord.space=true;
        const auto catchFrame=immediate.update(s,wallInput(chord,true,true,true),.02f,100);
        check(immediate.state!=State::action&&!hopMotion(catchFrame.motion),"entry Space cannot be reused as a hop even with zero approach duration");
    }
    {
        Scene obstacle;obstacle.boxes={{{-500,0,-100},{500,300,1500},true},{{-500,-25,155},{500,0,170},true}};
        Traversal climb;climb.cfg.approachSeconds=0;
        check(climb.attach(obstacle,{0,-40,0},{0,1,0},100),"projecting seam attach");
        int hops=0;float longest=0;
        for(int frame=0;frame<360;++frame) {
            const auto before=climb.state;auto r=climb.update(obstacle,{0,1},1.f/60,100);
            if(before!=State::action&&r.motion==Motion::hopUp)++hops;
            longest=std::max(longest,climb.stalledSeconds());
            check(climb.active(),"small projecting lip must not cause an automatic fall");
        }
        std::cout<<"obstacle hops="<<hops<<" rise="<<climb.position.z<<" longestStall="<<longest<<'\n';
        check(hops>0&&climb.position.z>200,"checked short leap must get above a projecting seam");
        Scene ceiling;ceiling.boxes={{{-500,0,-100},{500,300,1500},true},{{-500,-500,155},{500,0,170},true}};
        Traversal stopped;stopped.cfg.approachSeconds=0;
        check(stopped.attach(ceiling,{0,-40,0},{0,1,0},100),"solid ceiling attach");
        for(int frame=0;frame<240;++frame) {
            auto r=stopped.update(ceiling,{0,1},1.f/60,100);
            check(r.motion!=Motion::hopUp&&stopped.position.z<31&&stopped.active(),"no motion may jump through a closed ceiling");
        }
    }
    {
        Scene overhang;overhang.boxes={{{-500,0,70},{500,300,120},true}};
        Traversal hanging;hanging.cfg.approachSeconds=0;
        check(hanging.attach(overhang,{0,-30,0},{0,1,0},100),"ledge-only idle fixture attach");
        for(int i=0;i<40;++i) {
            const auto r=hanging.update(overhang,{},.02f,100);
            check(r.motion==Motion::hang&&hanging.state!=State::action,"feet absent at an actual ledge retain ordinary idle");
        }
        overhang.boxes[0].hi.z=800;
        for(int i=0;i<100;++i)hanging.update(overhang,{0,1},.02f,100);
        check(hanging.active()&&hanging.position.z>60,"ordinary ledge idle can resume upward climbing when the ledge probe disappears");
    }
    {
        Scene ridge;ridge.boxes={{{-500,0,-100},{500,50,120},true}};
        Traversal summit;summit.cfg.approachSeconds=0;summit.cfg.radius=31;summit.cfg.gap=37;summit.cfg.height=138;
        check(summit.attach(ridge,{0,-37,0},{0,1,0},100),"ridge attach");
        bool completed=false;
        for(int i=0;i<220&&summit.active();++i)completed|=summit.update(ridge,{0,1,false,true},.02f,100).completed;
        check(completed&&summit.position.y>17&&summit.position.y<33,"usable 50-unit ridge uses foot support, with full body clearance");
    }
    {
        Scene ended;ended.boxes={{{-500,0,-100},{500,25,1000},true}};
        Traversal floating;floating.cfg.approachSeconds=0;
        check(floating.attach(ended,{0,-30,0},{0,1,0},100),"ending wall fixture attach");
        ended.boxes[0].hi.z=-8;
        for(int i=0;i<100&&floating.active();++i)floating.update(ended,{},.02f,100);
        check(!floating.active(),"lower retry contacts cannot renew an unsupported air hang forever");
    }
    std::cout<<"Traversal tests passed, including ledge-idle upward escape, unsupported-air recovery and narrow supported ridge top-out.\n";
 } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
