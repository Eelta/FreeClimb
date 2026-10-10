#include "Core.h"
#include <iostream>
#include <vector>
using namespace fc;

namespace fc {
class TraversalCapture {
public:
    template<class T> static bool clear(const T& t,World& w,Vec a,Vec b,Vec destination={}) {
        if constexpr(requires{t.clearPath(w,a,b,false,destination);})return t.clearPath(w,a,b,false,destination);
        else return t.clearPath(w,a,b);
    }
    static bool heading(const Traversal& t,World& w,Vec p,Vec destination,Vec source,bool targetChecked=false,bool supportedTop=false) {
        return t.clearPath(w,p,p,supportedTop,destination,source,true,targetChecked);
    }
    static bool supported(const Traversal& t,World& w,Vec p) {return t.support(w,p,t.normal*-1).has_value();}
    static void landingFrame(Traversal& t,Vec from,Vec to,Vec landing) {
        t.beginAction(Motion::hopUp,from,to,.5f);t.actionLandingNormal=landing;
        t.actionTime=.97f;t.position=t.actionPoint(t.actionTime);
    }
};
}

struct ClearanceWorld final:World {
    struct Plane {Vec normal;float limit;};
    struct Solid {std::vector<Plane> planes;bool climbable=true;};
    std::vector<Solid> solids;unsigned calls{};
    static Vec rotate(Vec p,float angle) {return {p.x*std::cos(angle)-p.y*std::sin(angle),p.x*std::sin(angle)+p.y*std::cos(angle),p.z};}
    void box(Vec lo,Vec hi,bool climbable=true,float yaw=0) {
        Solid solid{{{{-1,0,0},-lo.x},{{1,0,0},hi.x},{{0,-1,0},-lo.y},{{0,1,0},hi.y},{{0,0,-1},-lo.z},{{0,0,1},hi.z}},climbable};
        for(auto& p:solid.planes)p.normal=rotate(p.normal,yaw);
        solids.push_back(solid);
    }
    std::optional<Hit> ray(Vec a,Vec b) override {
        ++calls;const Vec delta=b-a;float nearest=2;std::optional<Hit> result;
        for(const auto& solid:solids) {
            float enter=-1e20f,exit=1e20f;Vec entryNormal{},exitNormal{};bool valid=true;
            for(const auto& plane:solid.planes) {
                const float distance=a.dot(plane.normal)-plane.limit,denominator=delta.dot(plane.normal);
                if(std::abs(denominator)<1e-7f){if(distance>0)valid=false;continue;}
                const float phase=-distance/denominator;
                if(denominator<0&&phase>enter){enter=phase;entryNormal=plane.normal;}
                if(denominator>0&&phase<exit){exit=phase;exitNormal=plane.normal;}
            }
            if(!valid||enter>exit)continue;
            const bool inside=enter<0;const float phase=inside?exit:enter;
            if(phase<0||phase>1||phase>=nearest)continue;
            Vec normal=inside?exitNormal:entryNormal;if(normal.dot(delta)>0)normal=normal*-1;
            nearest=phase;result=Hit{a+delta*phase,normal,solid.climbable};
        }
        return result;
    }
};

static unsigned failures{},checks{};
static void check(bool value,const char* message) {++checks;if(!value){++failures;std::cerr<<"FAIL "<<message<<'\n';}}
static Traversal actor(Vec position={0,-56,0}) {
    Traversal t;t.cfg.radius=31;t.cfg.height=138;t.cfg.gap=37;t.cfg.approachSeconds=0;
    t.state=State::wall;t.position=position;t.normal=t.surfaceNormal={0,-1,0};return t;
}
static ClearanceWorld correctionScene() {
    ClearanceWorld world;world.box({-1000,0,-1000},{1000,100,1000});
    world.box({-2,-24.5f,136},{2,-10,145},false);return world;
}
static bool outsideBox(Vec feet,Vec lo,Vec hi,const Settings& cfg) {
    if(feet.z+cfg.height<=lo.z||feet.z+6>=hi.z)return true;
    const float x=std::clamp(feet.x,lo.x,hi.x),y=std::clamp(feet.y,lo.y,hi.y);
    return std::hypot(feet.x-x,feet.y-y)+.0001f>=cfg.radius;
}
static float capsuleBoxDistance(Vec feet,Vec lo,Vec hi,const Settings& cfg) {
    const float x=std::clamp(feet.x,lo.x,hi.x),y=std::clamp(feet.y,lo.y,hi.y);
    const float z=std::max({0.f,lo.z-(feet.z+cfg.height-cfg.radius),(feet.z+cfg.radius)-hi.z});
    return std::sqrt((feet.x-x)*(feet.x-x)+(feet.y-y)*(feet.y-y)+z*z);
}

static void changedHeading() {
    ClearanceWorld world;world.box({21.5f,21.5f,138.5f},{22.4f,22.4f,145},false);
    auto t=actor({});const Vec start{},next{0,0,1},heading=Vec{-1,-1,0}.unit();
    check(TraversalCapture::clear(t,world,start,next),"the original heading reproduces the sparse old-route negative control");
    check(outsideBox(start,{21.5f,21.5f,138.5f},{22.4f,22.4f,145},t.cfg),"the turn fixture begins outside the finite solid");
    check(!TraversalCapture::clear(t,world,start,next,heading),"the destination heading blocks its newly sampled head obstruction before commitment");
    check(t.position.length()==0&&t.normal.x==0&&t.normal.y==-1,"a rejected heading check cannot change root or facing");
    check(TraversalCapture::clear(t,world,start,start,heading),"the destination heading is safe at the original lower position");
    t.normal=heading;
    check(!TraversalCapture::clear(t,world,next,next),"the old acceptance would leave the next frame already obstructed");
}

static void headingTargetReuse() {
    for(bool supportedTop:{false,true}) {
        ClearanceWorld world;world.box({19.9f,-.1f,69.9f},{20.1f,.1f,70.1f},false);
        auto t=actor({});
        check(!TraversalCapture::heading(t,world,{},t.normal,t.normal,false,supportedTop),
            "heading-only checks without a verified endpoint still reject a finite side obstruction");
    }
    for(bool obstacle:{false,true}) {
        ClearanceWorld world;
        if(obstacle)world.box({21.5f,21.5f,138.5f},{22.4f,22.4f,145},false);
        auto t=actor({});const Vec target{0,0,1},heading=Vec{-1,-1,0}.unit();
        check(TraversalCapture::clear(t,world,{},target,t.normal),"same-target heading reuse follows a successful complete clearance check");
        world.calls=0;const bool original=TraversalCapture::heading(t,world,target,heading,t.normal);const auto originalCalls=world.calls;
        world.calls=0;const bool reused=TraversalCapture::heading(t,world,target,heading,t.normal,true);const auto reusedCalls=world.calls;
        check(original==reused&&reused!=obstacle,"verified-endpoint reuse preserves changed-heading collision results");
        check(originalCalls-reusedCalls==(obstacle?0u:8u),"successful same-endpoint heading reuse removes only the eight duplicate radial probes");
    }
}

static void nearlyUnchangedHeading() {
    ClearanceWorld world;world.box({-22.2f,21.6f,138.5f},{-21.6f,22.2f,145},false);
    auto t=actor({});t.normal=Vec{-1,-1,0}.unit();
    const Vec start{},next{0,0,1},heading=ClearanceWorld::rotate(t.normal,.0001f);
    check(!TraversalCapture::clear(t,world,start,next),"pure vertical stable-heading movement retains the diagonal side sample");
    check(!TraversalCapture::clear(t,world,start,next,heading),"a tiny changed heading must not deduplicate its required side sample against an unexecuted old side sample");
}

static void embeddedEndpoint() {
    ClearanceWorld world;
    constexpr float yaw=-.260155f;
    world.box({-100,30.2f,-1000},{100,40,1000},true,yaw);
    auto t=actor({});const Vec start{},lower{0,0,-1},facing=ClearanceWorld::rotate({0,-1,0},yaw);
    check(!TraversalCapture::clear(t,world,start,start),"radial confirmation detects a nearby skewed wall missed by the old fixed columns");
    check(!TraversalCapture::clear(t,world,start,lower,facing),"a new sample column wholly inside a finite wall cannot pass a vertical movement check");
    t.normal=facing;
    check(!TraversalCapture::clear(t,world,lower,lower),"a stationary endpoint radial detects a parallel embedded column");
    check(!TraversalCapture::clear(t,world,lower,lower+facing*3),"an existing embedded sample never bypasses the solid exit crossing");
    const Vec safe=start+facing*2;
    check(TraversalCapture::clear(t,world,safe,safe),"real empty body clearance beside the same wall remains valid");
    check(TraversalCapture::clear(t,world,safe,safe+Vec{0,0,-2}),"a safe descent alongside the finite skewed wall remains available");
    check(TraversalCapture::clear(t,world,safe,safe+facing*2),"an outward retreat from a genuinely empty body volume remains available");
}

static void finiteRadialConfirmation() {
    auto t=actor({});ClearanceWorld distant;
    const float yaw=-1.0471975512f;
    distant.box({16.5f,30.3109f,-1000},{18.5f,32.3109f,1000},false,yaw);
    check(TraversalCapture::clear(t,distant,{},{}),"a long probe finding a finite patch outside the body must confirm real contact before rejecting");
    const auto probe=distant.ray({0,0,70},{43.8406f,0,70});
    check(probe.has_value()&&(probe->point-Vec{0,0,70}).length()>31.f,"the finite patch is actually detected beyond the physical radius");
    ClearanceWorld wall;wall.box({-100,30.2f,-1000},{100,40,1000},true,-.260155f);
    const Vec normal=ClearanceWorld::rotate({0,-1,0},-.260155f),safe=normal*2;
    check(TraversalCapture::clear(t,wall,safe,safe),"a real safe root beside the finite wall passes with the original heading");
    check(!TraversalCapture::clear(t,wall,safe,{}),"a main-heading movement cannot enter the skewed body face before support selection turns");
    check(TraversalCapture::clear(t,wall,safe,safe+Vec{1,0,0}),"a supported-size lateral step that remains outside the body radius is preserved");
    ClearanceWorld cornerPatch;cornerPatch.box({-100,30.2f,-1000},{100,40,1000},true,.260155f);
    cornerPatch.solids.back().planes.push_back({{1,0,0},-2.f});
    for(const Vec axis:{Vec{1,0,0},Vec{-1,0,0},Vec{0,1,0},Vec{0,-1,0}})
        check(!cornerPatch.ray({0,0,70},axis*(31.f*1.41421356237f)+Vec{0,0,70}),"the finite quadrant patch is outside every cardinal broad ray");
    check(cornerPatch.ray({0,0,70},{-31,31,70}).has_value(),"a diagonal broad ray finds the finite quadrant patch");
    check(!TraversalCapture::clear(t,cornerPatch,{},{}),"diagonal discovery confirms the actual close quadrant patch with a radius-bounded ray");
    wall.box({-2,-7,68},{2,-5,72},false);
    check(!TraversalCapture::clear(t,wall,safe,safe+normal*7),"radial endpoint checks do not bypass another finite obstacle on an escape path");
}

static void checkedDescent(int fps) {
    auto world=correctionScene();auto t=actor();const Vec start=t.position;
    const Vec raw=start+Vec{0,0,-t.cfg.downSpeed/fps};
    check(TraversalCapture::supported(t,world,start)&&TraversalCapture::supported(t,world,raw),"the existing outer hang and raw descent both have actual hand support");
    check(TraversalCapture::clear(t,world,start,raw),"the requested descent has an independently checked complete clear route");
    Input input;input.y=-1;const auto result=t.update(world,input,1.f/fps,1000);
    check(!result.released&&t.active()&&t.position.z<start.z-.1f,"down input escapes correction-only blocking through a supported clear descent");
    check(std::abs(t.position.x-start.x)<.001f&&std::abs(t.position.y-start.y)<.001f,"fallback descent preserves the existing outward gap without increasing it");
    for(unsigned index=0;index<=32;++index) {
        const Vec p=start+(t.position-start)*(index/32.f);
        check(outsideBox(p,{-2,-24.5f,136},{2,-10,145},t.cfg)&&p.y+t.cfg.radius<=.001f,"the actual movement chord keeps the full conservative cylinder outside both closed solids");
    }
    std::cout<<"descent fps="<<fps<<" delta="<<(t.position-start).length()<<'\n';
}

static void committedHeading() {
    ClearanceWorld world;world.box({-1000,37,-1000},{1000,1000,1000},true,-.7853981634f);
    world.box({4,29,139},{8,31.2f,145},false);auto t=actor({});const Vec start=t.position;
    check(TraversalCapture::supported(t,world,start),"the changed-heading update starts with real diagonal wall grips");
    Input input;input.y=1;const auto result=t.update(world,input,1.f/60,1000);
    check(!result.released&&t.active()&&t.position.z<=1.0001f,"ordinary movement cannot commit a root step that obstructs its simultaneously committed heading");
    check(TraversalCapture::clear(t,world,t.position,t.position),"the committed movement remains clear using its actual new body heading");
    for(unsigned index=0;index<=32;++index)
        check(outsideBox(start+(t.position-start)*(index/32.f),{4,29,139},{8,31.2f,145},t.cfg),"the turning update keeps the complete conservative cylinder outside the new head blocker");
}

static void stableHeadingDescent(int fps) {
    ClearanceWorld world;world.box({-1000,37,-1000},{1000,1000,1000},true,-.7853981634f);
    const Vec lo{-32,10,138.5f},hi{-10,32,145};world.box(lo,hi,false);
    auto t=actor({});t.normal=t.surfaceNormal=Vec{-1,-1,0}.unit();const Vec start=t.position;
    check(TraversalCapture::supported(t,world,start)&&TraversalCapture::clear(t,world,start,start),"the stable diagonal heading begins on supported clear geometry");
    Input up;up.y=1;const auto upward=t.update(world,up,1.f/fps,1000);const Vec reached=t.position;
    std::cout<<"stable ascent fps="<<fps<<" pos="<<reached.x<<','<<reached.y<<','<<reached.z<<" reason="<<t.blockedReason<<'\n';
    check(!upward.released&&t.active()&&reached.z<=.5001f,"stable-heading ascent stops before its unchanging lateral body envelope reaches the head obstruction");
    for(unsigned index=0;index<=32;++index)
        check(outsideBox(start+(reached-start)*(index/32.f),lo,hi,t.cfg),"the accepted stable-heading ascent never enters the finite closed head blocker");
    const Vec tangent{-t.normal.y,t.normal.x,0};
    check(TraversalCapture::clear(t,world,reached,reached),"the stopped root remains clear with no input");
    check(TraversalCapture::clear(t,world,reached,reached-tangent*.1f),"switching to a lateral direction does not newly classify the stopped root as embedded");
    Input down;down.y=-1;const auto downward=t.update(world,down,1.f/fps,1000);
    check(!downward.released&&t.active()&&t.position.z<reached.z-.1f,"down input retains the supported retreat after upward clearance stops ascent");
    for(unsigned index=0;index<=32;++index)
        check(outsideBox(reached+(t.position-reached)*(index/32.f),lo,hi,t.cfg),"the entire retreat chord stays outside the same head blocker");
}

static void blockedDescent(int fps) {
    auto world=correctionScene();world.box({30.5f,-58,3},{32,-54,5.98f},false);auto t=actor();const Vec start=t.position;
    check(TraversalCapture::clear(t,world,start,start),"the lower blocker starts below the existing checked body envelope");
    check(!TraversalCapture::clear(t,world,start,start+Vec{0,0,-t.cfg.downSpeed/(4*fps)}),"even the shortest raw descent intersects a separate real lower blocker");
    Input input;input.y=-1;const auto result=t.update(world,input,1.f/fps,1000);
    std::cout<<"blocked descent fps="<<fps<<" delta="<<(t.position-start).length()<<" pos="<<t.position.x<<','<<t.position.y<<','<<t.position.z<<" active="<<t.active()<<" released="<<result.released<<" result="<<result.reason<<" reason="<<t.blockedReason<<'\n';
    check(!result.released&&t.active()&&(t.position-start).length()<.001f,"raw fallback cannot bypass a genuine movement obstruction");
}

static void retainedOverlapBlock() {
    ClearanceWorld world;world.box({-2,-7,137.71f},{2,-5,142},false);auto t=actor({0,-37,0});const Vec start=t.position,down=start+Vec{0,0,-1.3f},outward=start+Vec{0,-3,0};
    check(!TraversalCapture::clear(t,world,start,start),"the overlap fixture explicitly begins with a shallow existing obstruction");
    check(TraversalCapture::clear(t,world,down,down)&&TraversalCapture::clear(t,world,outward,outward),"both proposed escape endpoints are clear");
    check(!TraversalCapture::clear(t,world,start,down)&&!TraversalCapture::clear(t,world,start,outward),"clear endpoints never excuse the existing two-sided exit obstruction");
}

static void landingHeading() {
    for(bool obstacle:{false,true}) {
        ClearanceWorld world;world.box({-1000,37,-1000},{1000,1000,1000},true,-.7853981634f);
        const Vec lo{21.9f,21.9f,80},hi{21.94f,21.94f,90};if(obstacle)world.box(lo,hi,false);
        auto t=actor({});const Vec landing=Vec{-1,-1,0}.unit();
        TraversalCapture::landingFrame(t,{0,-20,-20},{},landing);const Vec start=t.position;
        check(TraversalCapture::supported(t,world,{}),"the hop landing has real diagonal-wall grip support");
        check(capsuleBoxDistance(start,lo,hi,t.cfg)>=t.cfg.radius,"the last hop frame starts with the real capsule outside the finite blocker");
        const auto result=t.update(world,{},1.f/60,1000);
        if(obstacle) {
            check(!result.released&&t.active()&&t.geometryHolding(),"the landing heading holds before the newly measured finite body blocker");
            check((t.position-start).length()<.0001f,"a rejected landing preserves its last verified root");
            check(capsuleBoxDistance(t.position,lo,hi,t.cfg)>=t.cfg.radius,"a rejected landing cannot first commit its root inside the real capsule blocker");
            const float phase=t.actionProgress();
            for(unsigned frame=0;frame<60;++frame) {
                const auto held=t.update(world,{},1.f/60,1000);
                check(!held.released&&t.geometryHolding()&&t.actionProgress()==phase&&(t.position-start).length()<.0001f,
                    "persistent landing obstruction cannot advance the held jump");
            }
            world.solids.pop_back();
            for(unsigned frame=0;frame<60&&t.state==State::action;++frame)t.update(world,{},1.f/60,1000);
            check(t.active()&&t.state==State::wall&&!t.geometryHolding(),"removing the finite blocker lets the validated landing finish");
        } else check(!result.released&&t.state==State::wall&&t.normal.dot(landing)>.999f,"clear hop landing commits the real destination heading normally");
        std::cout<<"landing obstacle="<<obstacle<<" root="<<t.position.x<<','<<t.position.y<<','<<t.position.z<<" clearance="<<capsuleBoxDistance(t.position,lo,hi,t.cfg)<<" reason="<<result.reason<<'\n';
    }
}

static void controlsAndSupport() {
    auto world=correctionScene();auto idle=actor();const Vec start=idle.position;
    for(unsigned frame=0;frame<15;++frame)idle.update(world,{},1.f/60,1000);
    check((idle.position-start).length()<.001f&&idle.active(),"zero input does not invoke outward-gap recovery movement");
    auto lost=actor();world.solids.erase(world.solids.begin());Input down;down.y=-1;
    for(unsigned frame=0;frame<12;++frame)lost.update(world,down,1.f/60,1000);
    check((lost.position-start).length()<.001f,"removed hand support cannot authorize the otherwise clear raw descent");
    world=correctionScene();auto released=actor();Input release;release.release=true;bool finished=false;
    for(unsigned frame=0;frame<60&&released.active();++frame) {
        const auto result=released.update(world,frame==0?release:Input{},1.f/60,1000);finished|=result.released;
    }
    check(finished&&!released.active(),"manual release remains available at the obstructed correction location");
}

static void wideningGap() {
    ClearanceWorld world;world.box({-1000,0,-1000},{0,1000,1000});
    world.box({0,0,-1000},{1000,1000,1000},true,.1745329252f);
    world.box({-80,-24.5f,136},{80,-10,145},false);
    auto t=actor({-.01f,-56,0});const Vec start=t.position;Input input;input.x=1;
    const Vec target=start+Vec{t.cfg.sideSpeed/60,0,0};
    check(TraversalCapture::supported(t,world,start)&&TraversalCapture::supported(t,world,target),"the diverging finite-wall seam retains real source and destination grips");
    check(TraversalCapture::clear(t,world,start,target),"the diverging raw lateral path itself remains clear before the gap restriction");
    const auto result=t.update(world,input,1.f/60,1000);
    check(!result.released&&(t.position-start).length()<.001f,"fallback cannot increase distance from the destination support plane");
}

int main() {
    changedHeading();headingTargetReuse();nearlyUnchangedHeading();embeddedEndpoint();finiteRadialConfirmation();committedHeading();retainedOverlapBlock();landingHeading();controlsAndSupport();wideningGap();
    for(int fps:{30,60,120}){checkedDescent(fps);blockedDescent(fps);stableHeadingDescent(fps);}
    std::cout<<"climb clearance checks="<<checks<<" failures="<<failures<<'\n';return failures?1:0;
}
