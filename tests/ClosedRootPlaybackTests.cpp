#include "Core.h"
#include "CornerTestWorld.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
namespace fc {
class TraversalCapture {
public:
    static void action(Traversal& t,Motion m,Vec to){t.state=State::wall;t.beginAction(m,t.position,to,1);}
    static bool clear(Traversal& t,World& w){return t.authoredActionClear(w);}
    static void detour(Traversal& t){t.detour=true;t.detourOut=t.actionFrom+t.normal*50;t.detourOver=t.detourOut+Vec{0,0,80};}
};
}
static unsigned checks{};
static void require(bool value,const char* text){++checks;if(!value)throw std::runtime_error(text);}
struct SourceWorld:World {
    fc_test::CornerWorld solids;
    unsigned bodyQueries{};
    Traversal* current{};
    float midpointError{};
    SourceWorld(){solids.boxes.push_back({{-1000,0,-1000},{1000,1000,3000}});}
    std::optional<Hit> ray(Vec a,Vec b)override{return solids.ray(a,b);}
    bool actionBodyClear(Motion,Vec,Vec,float,float,Vec)override{return true;}
    bool actionBodyPathClear(Motion,Vec,Vec,float a,float b,Vec,Vec midpoint)override {
        ++bodyQueries;if(current)midpointError=std::max(midpointError,(midpoint-current->actionPathPoint((a+b)*.5f)).length());return true;
    }
};
static Settings settings(Motion m,Vec excursion,Vec end={}) {
    auto cfg=fc_test::settings();cfg.automaticClimbActions=false;
    auto motions=std::make_shared<std::array<AuthoredMotion,42>>();auto& motion=(*motions)[int(m)-1];
    motion.enabled=true;motion.seconds=1;motion.trajectory.count=3;
    motion.trajectory.knots[0]={0,{}};motion.trajectory.knots[1]={.5f,excursion};motion.trajectory.knots[2]={1,end};
    cfg.authoredMotions=motions;return cfg;
}
static Traversal attached(SourceWorld& world,Motion m,Vec excursion,Vec end={},float gap=37) {
    Traversal t;t.cfg=settings(m,excursion,end);const auto sources=t.cfg.authoredMotions;t.cfg.authoredMotions.reset();
    require(t.attach(world,{0,-gap,200},{0,1,0},1000),"fixture starts at a real solid wall");t.cfg.authoredMotions=sources;return t;
}
static void obstructionTests() {
    for(Motion motion:{Motion::hopUp,Motion::hopLeft,Motion::hopRight,Motion::contextHopLeft,Motion::contextHopRight,
        Motion::runLaunch,Motion::runLaunchLeft,Motion::runLaunchRight,Motion::runCatch,Motion::sideBrace,Motion::drop,Motion::dropBack,Motion::backFlipOut}) {
        SourceWorld world;auto t=attached(world,motion,{0,120,0});const auto start=t.position;
        TraversalCapture::action(t,motion,start+Vec{0,0,70});
        require(t.authoredPath(motion),"closed source excursion is a checked one-shot path");
        require((t.authoredRoot(motion,.5f)-Vec{0,120,0}).length()<.001f,"source Root is consumed once when the controller executes it");
        require(!TraversalCapture::clear(t,world),"a closed source curve toward a solid wall fails preflight");
        require((t.position-start).length()<.001f,"failed preflight has no displacement side effect");
    }
    for(int axis:{0,1,2,3}) {
        SourceWorld world;const Vec excursion=axis==0?Vec{120,0,0}:axis==1?Vec{-120,0,0}:axis==2?Vec{0,0,120}:Vec{0,0,-120};
        auto t=attached(world,Motion::hopUp,excursion);const auto start=t.position;
        if(axis==0)world.solids.boxes.push_back({{85,-200,-1000},{130,0,3000},false});
        if(axis==1)world.solids.boxes.push_back({{-130,-200,-1000},{-85,0,3000},false});
        if(axis==2)world.solids.boxes.push_back({{-500,-300,450},{500,0,480},false});
        if(axis==3)world.solids.boxes.push_back({{-500,-300,0},{500,0,150},false});
        TraversalCapture::action(t,Motion::hopUp,start+Vec{0,0,70});
        require(!TraversalCapture::clear(t,world),"lateral returns and up/down excursions are included in solid-body clearance");
    }
}
static void continuousPaths() {
    for(Motion motion:{Motion::hopUp,Motion::hopLeft,Motion::runLaunch,Motion::runCatch,Motion::drop,Motion::dropBack,Motion::backFlipOut})
        for(float net:{0.f,1.5f})for(int fps:{30,60,120}) {
            SourceWorld world;auto t=attached(world,motion,{0,-100,40},{net,0,0});
            const auto start=t.position,target=start+Vec{60,0,20};TraversalCapture::action(t,motion,target);world.current=&t;
            require(TraversalCapture::clear(t,world),"closed outward source curve has an unobstructed checked route");
            require((t.actionPathPoint(0)-start).length()<.001f&&(t.actionPathPoint(1)-target).length()<.001f,"closed and near-closed source curves retain exact checked endpoints");
            const auto mid=t.actionPathPoint(.5f);
            require(std::abs(mid.y-(start.y-100))<.001f,"source excursion is applied once without a second base hop arc");
            float elapsed=0;Result result;
            while(t.state==State::action&&elapsed<2) {
                result=t.update(world,{},1.f/fps,1000);elapsed+=1.f/fps;
                require((t.position-t.actionPathPoint(t.actionProgress())).length()<.002f,"live movement executes the identical closed-root preflight route");
                require(t.position.finite(),"closed-root live path remains finite");
            }
            require(std::abs(elapsed-1.f)<=1.01f/fps&&t.actionProgress()==1,"closed source actions retain their full one-second clock");
            require(world.midpointError<.002f,"backflip body queries use the same closed-root midpoint");
            require(result.releaseVelocity.finite()&&result.releaseVelocity.length()<=600.01f,"closed departure terminal velocity remains finite and bounded");
        }
}
static void entriesAndDetours() {
    for(Motion motion:{Motion::reach,Motion::jumpCatch,Motion::ledgeCatch}) {
        SourceWorld blocked;auto rejected=attached(blocked,motion,{0,120,0},{},80);
        require(!rejected.entry(blocked,motion),"closed source entry through the wall is rejected before ownership");
        SourceWorld world;auto t=attached(world,motion,{0,-45,25},{},80);
        require(t.entry(world,motion),"closed source entry away from the wall is accepted");
        float elapsed=0;
        while(t.state==State::approach&&elapsed<2) {
            const auto result=t.update(world,{},1.f/60,1000);elapsed+=1.f/60;
            require(!result.released&&(t.position-t.entryPathPoint(t.reachProgress())).length()<.002f,"live entry retains its checked closed-root route");
        }
        require(t.state==State::wall&&std::abs(elapsed-1)<.018f,"closed entry retains its complete source clock");
    }
    SourceWorld world;auto t=attached(world,Motion::hopUp,{0,-60,20});
    const auto start=t.position,target=start+Vec{0,0,80};TraversalCapture::action(t,Motion::hopUp,target);TraversalCapture::detour(t);
    for(int sample=0;sample<=100;++sample)require(t.actionPathPoint(float(sample)/100).finite(),"closed detour progress never divides by zero");
    require((t.actionPathPoint(0)-start).length()<.001f&&(t.actionPathPoint(1)-target).length()<.001f,"closed detours keep their checked endpoints");
}
static void loopAndThresholdContracts() {
    for(Motion motion:{Motion::hang,Motion::contextHang,Motion::up,Motion::down,Motion::left,Motion::right,
        Motion::runUp,Motion::runLeft,Motion::runRight,Motion::runDiagonalLeft,Motion::runDiagonalRight})
        for(float net:{0.f,1.5f,48.f}) {
            SourceWorld world;auto t=attached(world,motion,{0,-8,3},{0,net,0});
            require(!t.authoredPath(motion),"idle and moving loops never become one-shot paths");
            for(int cycle=0;cycle<50;++cycle)for(float phase:{0.f,.2f,.5f,.9f,1.f}) {
                const Vec expected=authoredMovingLoop(motion)&&net>=2?Vec{0,net*phase,0}:Vec{};
                require((t.authoredRoot(motion,phase)-expected).length()<.0001f,"loop Root consumption remains linear without accumulating local residuals");
            }
        }
    for(float excursion:{0.f,1.99f,2.f}) {
        SourceWorld world;auto t=attached(world,Motion::hopUp,{excursion,0,0});
        require(t.authoredPath(Motion::hopUp)==(excursion>=2),"two Skyrim units distinguish meaningful one-shot travel from local pose noise");
    }
    Traversal legacy;require(!legacy.authoredPath(Motion::hopUp)&&legacy.authoredRoot(Motion::hopUp,.5f).length()==0,"default version-one actions retain their existing source-consumption behavior");
}
int main()try{
    obstructionTests();continuousPaths();entriesAndDetours();loopAndThresholdContracts();
    std::cout<<"Closed Root one-shot paths, real collisions, entries, bridges, departures and loop invariants passed: "<<checks<<'\n';
    return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
