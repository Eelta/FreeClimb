#define main irregularFixtureMain
#include "IrregularCornerTests.cpp"
#undef main
#include "TraversalCapture.h"
namespace {
unsigned continuedCases{},safetyCases{},integratedCases{},peakPlan{},peakLive{},peakCore{};
void continued(float slope,float gap,float lateral,int fps,float side,bool distant,float degrees=90) {
    auto world=fixture(true,degrees,{degrees>90?-.25f:0.f,slope,0},side,distant);auto cfg=config();
    const Vec start=world.point({lateral,-gap+6*slope,0}),normal=world.normal({0,-1,0});
    auto route=findCornerRoute(world,cfg,start,normal,side,[](Vec,Vec){return true;});
    peakPlan=std::max(peakPlan,world.casts);
    if(!route)std::cerr<<"missing continued slope="<<slope<<" gap="<<gap<<" lateral="<<lateral<<" fps="<<fps<<" side="<<side<<" far="<<distant<<" degrees="<<degrees<<'\n';
    require(route.has_value(),"steep joined source and destination surfaces produce a fully supported route");
    if(!route)return;
    const float speed=cornerSpeedLimit(*route,cfg.sideSpeed);Vec previous=start,oldNormal=normal;
    float clearance=1.e9f,turn=0;bool complete=false;
    for(int frame=0;frame<fps*6;++frame) {
        world.casts=0;const auto step=advanceCornerRoute(world,cfg,*route,speed/fps,[](Vec,Vec){return true;});
        peakLive=std::max(peakLive,world.casts);
        require(step.has_value(),"every live steep turn retains measured paired support and complete body checks");
        if(!step)break;
        const unsigned samples=std::max(1u,unsigned(std::ceil((step->position-previous).length()/.5f)));
        for(unsigned sample=0;sample<=samples;++sample)
            clearance=std::min(clearance,world.solid.capsuleDistance(previous+(step->position-previous)*(float(sample)/samples)));
        turn=std::max(turn,std::acos(std::clamp(oldNormal.dot(step->normal),-1.f,1.f)));
        previous=step->position;oldNormal=step->normal;
        if(step->complete){complete=true;break;}
    }
    if(!complete||clearance<30.96f)std::cerr<<"continued failure slope="<<slope<<" gap="<<gap<<" lateral="<<lateral<<" complete="<<complete<<" clearance="<<clearance<<'\n';
    require(complete,"held side intent reaches the actual steep destination hand face");
    require(clearance>=30.96f,"independent closed triangle distance keeps the entire radius31 height138 capsule clear");
    require(turn<=8.3f/fps+.015f,"steep destination advance preserves the existing continuous surface turning limit");
    require(cornerLandingGrip(world,cfg,*route,previous),"completion includes centered actual destination contact pairs");
    require(peakPlan<4096&&peakLive<512,"steep route planning and playback fit bounded World ray budgets");
    ++continuedCases;
}
void integrated(float slope,int fps,float side,bool run) {
    auto world=fixture(true,90,{0,slope,0},side,true);Traversal t;t.cfg=config();
    const Vec start=world.point({-155,-37+6*slope,0});
    require(t.attach(world,start,world.normal({0,-1,0})*-1,1000),
        "actual Core attaches to the coherent steep source before its joined side face");
    if(!t.active())return;
    Vec previous=t.position;float clearance=1.e9f;bool entered=false,completed=false,paused=false,released=false,sawRun=false;
    unsigned pauseFrames=0;
    for(int frame=0;frame<fps*9&&t.active();++frame) {
        Input input;input.x=side;input.run=run;
        if(entered&&!paused){pauseFrames=unsigned(fps/2);paused=true;}
        const bool pause=pauseFrames>0;
        if(pause){input.x=0;--pauseFrames;}
        world.casts=0;const auto result=t.update(world,input,1.f/fps,1000);peakCore=std::max(peakCore,world.casts);
        entered|=t.turningCorner();released|=result.released;sawRun|=t.wallRunning();
        if(pause)require((t.position-previous).length()<.02f,"released lateral intent holds a stable measured steep corner without drifting");
        const unsigned samples=std::max(1u,unsigned(std::ceil((t.position-previous).length()/.5f)));
        for(unsigned sample=0;sample<=samples;++sample)
            clearance=std::min(clearance,world.solid.capsuleDistance(previous+(t.position-previous)*(float(sample)/samples)));
        previous=t.position;
        if(entered&&!t.turningCorner()&&t.surfaceNormal.dot(world.normal({1,0,0}))>.995f){completed=true;break;}
    }
    if(!completed)std::cerr<<"Core steep slope="<<slope<<" fps="<<fps<<" side="<<side<<" run="<<run<<" entered="<<entered<<" released="<<released<<" reason="<<t.blockedReason<<'\n';
    require(entered&&completed&&paused&&!released,"actual Core completes a resumed steep turn in climbing and wall-running modes");
    require(!run||sawRun,"wall-run input actually enters the existing running mode on the checked steep route");
    require(clearance>=30.96f,"actual Core movement chords retain the independently checked entire capsule clearance");
    require(peakCore<TraversalCapture::capacity,"complete Core including clearPath stays below the bounded capture query capacity");
    ++integratedCases;
}
void safety(float side,bool distant) {
    auto world=fixture(true,90,{0,.684f,0},side,distant);const auto cfg=config();
    const Vec start=world.point({-45,-37+6*.684f,0}),normal=world.normal({0,-1,0});
    auto route=findCornerRoute(world,cfg,start,normal,side,[](Vec,Vec){return true;});
    require(route.has_value(),"steep negative fixtures initially have an actual legal route");if(!route)return;
    require(!findCornerRoute(world,cfg,start,normal,side,[](Vec,Vec){return false;}),"new steep continuation never bypasses the caller's full body clearance rejection");
    struct Limited:World {
        TriangleWorld& source;unsigned limit{},calls{};bool exhausted{};
        Limited(TriangleWorld& w,unsigned n):source(w),limit(n){}
        std::optional<Hit> ray(Vec a,Vec b)override {
            if(calls++>=limit){exhausted=true;return Hit{a,(a-b).unit(),false};}
            return source.ray(a,b);
        }
    };
    for(unsigned budget:{0u,8u,100u,600u,1200u}) {
        Limited limited(world,budget);
        require(!findCornerRoute(limited,cfg,start,normal,side,[](Vec,Vec){return true;})&&limited.exhausted,
            "exhausted ray budgets remain blocking for steep destination support and full body routes");
    }
    const float progress=route->distance;world.solid.triangles.clear();
    require(!advanceCornerRoute(world,cfg,*route,2,[](Vec,Vec){return true;})&&route->distance==progress,
        "a removed real steep support face cannot advance a cached corner path");
    world=fixture(true,90,{0,.684f,0},side,distant);
    struct NoSide:World {
        TriangleWorld& source;Vec denied;
        NoSide(TriangleWorld& w):source(w),denied(w.normal({1,0,0})){}
        std::optional<Hit> ray(Vec a,Vec b)override {auto hit=source.ray(a,b);if(hit&&hit->normal.dot(denied)>.99f)hit->climbable=false;return hit;}
    } denied(world);
    require(!findCornerRoute(denied,cfg,start,normal,side,[](Vec,Vec){return true;}),
        "an unclimbable destination remains rejected even beside a valid steep source");
    world=fixture(true,105,{0,.684f,0},side,distant);
    require(!findCornerRoute(world,cfg,start,normal,side,[](Vec,Vec){return true;}),
        "a target overhang requiring an unsupported body approach cannot be forced into a steep turn");
    world=fixture(true,105,{-.25f,.684f,0},side,distant);
    require(!findCornerRoute(world,cfg,start,normal,side,[](Vec,Vec){return true;}),
        "a visible target face cannot certify a turn with missing intermediate support");
    ++safetyCases;
}
}
int main() {
    for(float slope:{.4f,.684f,.92f})for(float gap:{37.f,51.f})for(float lateral:{-45.f,8.f})
      for(int fps:{30,60,120})for(float side:{-1.f,1.f})for(bool distant:{false,true})
        continued(slope,gap,lateral,fps,side,distant);
    for(float degrees:{75.f})for(float side:{-1.f,1.f})
        continued(.684f,37,-45,60,side,true,degrees);
    for(float slope:{.4f,.684f,.92f})for(int fps:{30,60,120})for(float side:{-1.f,1.f})for(bool run:{false,true})
        integrated(slope,fps,side,run);
    for(float side:{-1.f,1.f})for(bool distant:{false,true})safety(side,distant);
    std::cout<<"SlopeCornerContinuationTests positives="<<continuedCases<<" safety="<<safetyCases
        <<" integrated="<<integratedCases<<" coreRays="<<peakCore<<" rays="<<peakPlan<<'/'<<peakLive<<" failures="<<failures<<'\n';return failures?1:0;
}
