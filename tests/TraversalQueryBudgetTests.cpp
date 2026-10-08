#include "Core.h"
#include "CornerTestWorld.h"
#include <iostream>
#include <stdexcept>
#include <string>
using namespace fc;
static unsigned checks{};
static void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
namespace fc {
class TraversalCapture {
public:
    static bool preflight(Traversal& t,World& world,Motion motion,bool roof=false) {
        t.beginAction(motion,t.position,t.position+Vec{0,40,80},.5f);
        t.roofTransfer=roof;
        return t.authoredActionClear(world);
    }
    static std::optional<Result> obstacle(Traversal& t,World& world) {
        return t.tryWallRunObstacle(world,{0,0,1},379.5f,{0,1,false,true,false,false,true},1000);
    }
};
}
struct CountingWorld:World {
    unsigned rays{},bodies{},paths{};
    bool block{},bodyAllowed=true;
    Vec lastMiddle{};
    std::optional<Hit> ray(Vec a,Vec b) override {
        ++rays;return block?std::optional<Hit>{Hit{(a+b)*.5f,(a-b).unit(),false}}:std::nullopt;
    }
    bool actionBodyClear(Motion,Vec,Vec,float,float,Vec) override {++bodies;return bodyAllowed;}
    bool actionBodyPathClear(Motion,Vec,Vec,float,float,Vec,Vec middle) override {++paths;lastMiddle=middle;return bodyAllowed;}
};
static std::shared_ptr<std::array<AuthoredMotion,42>> motions(bool dense) {
    auto result=std::make_shared<std::array<AuthoredMotion,42>>();
    for(int id=1;id<=motionCount;++id)if(isActiveMotion(Motion(id))) {
        auto& m=(*result)[id-1];m.enabled=true;m.seconds=.8f;m.trajectory.count=dense?65:2;
        for(unsigned i=0;i<m.trajectory.count;++i) {
            const float phase=std::pow(float(i)/float(m.trajectory.count-1),1.37f);
            m.trajectory.knots[i]={phase,dense?Vec{3*std::sin(phase*20),40*phase+3*std::sin(phase*80),80*phase+4*std::sin(phase*24)}:Vec{}};
        }
        check(m.trajectory.valid(),"dense nonmonotonic source satisfies the author contract");
    }
    return result;
}
static void forwarding() {
    CountingWorld world;AuthoredQueryWorld limited(world,3);
    check(!limited.ray({0,0,0},{0,0,1}),"budget forwards ray results");
    check(limited.actionBodyClear(Motion::backFlipOut,{},{},0,.1f,{0,-1,0}),"budget forwards body checks");
    check(limited.actionBodyPathClear(Motion::backFlipOut,{},{},0,.1f,{0,-1,0},{1,2,3}),"budget forwards authored midpoint body checks");
    check(world.rays==1&&world.bodies==1&&world.paths==1&&world.lastMiddle.z==3,"every forwarded callback retains its actual arguments");
    check(limited.ray({0,0,0},{0,0,1}).has_value()&&limited.exhausted&&world.rays==1,"budget exhaustion is blocking without another engine query");
    check(!limited.actionBodyClear(Motion::backFlipOut,{},{},0,.1f,{0,-1,0})&&world.bodies==1,"exhausted body checks do not escape the budget");
}
static void completeRoutes() {
    unsigned worst{};
    for(bool dense:{false,true}) {
        const auto authored=motions(dense);
        for(Motion motion:{Motion::reach,Motion::jumpCatch,Motion::ledgeCatch,Motion::hopUp,Motion::hopLeft,Motion::hopRight,
            Motion::drop,Motion::dropBack,Motion::backFlipOut,Motion::runLaunch,Motion::runLaunchLeft,Motion::runLaunchRight,
            Motion::runCatch,Motion::sideBrace,Motion::kickUp,Motion::kickLeft,Motion::kickRight,Motion::contextHopLeft,Motion::contextHopRight}) {
            Traversal t;t.cfg.authoredMotions=authored;t.position={0,-37,0};t.normal=t.surfaceNormal={0,-1,0};
            CountingWorld world;
            check(TraversalCapture::preflight(t,world,motion,motion==Motion::kickUp),"valid complete source route remains within the shared bound");
            worst=std::max(worst,world.rays+world.paths);
            check(world.rays+world.paths<=AuthoredQueryWorld::authoredLimit&&!t.authoredQueryBudgetExhausted,"all finite full-route checks remain bounded without rejecting valid dense source knots");
            if(motion==Motion::backFlipOut)check(world.paths>0,"backflip keeps the actual pose-aware body checks");
            world.block=true;
            check(!TraversalCapture::preflight(t,world,motion),"a later obstruction is not hidden by a successful previous preflight");
            world.block=false;world.bodyAllowed=false;
            if(motion==Motion::backFlipOut)check(!TraversalCapture::preflight(t,world,motion),"body rejection remains blocking through the budget wrapper");
        }
    }
    std::cout<<"dense source peak operations="<<worst<<'\n';
}
static void boundedFailure() {
    Traversal t;t.cfg.authoredMotions=motions(true);t.position={0,-37,0};t.normal=t.surfaceNormal={0,-1,0};
    CountingWorld world;AuthoredQueryWorld limited(world,70);
    const Vec start=t.position;
    check(!TraversalCapture::preflight(t,limited,Motion::kickUp,true)&&limited.exhausted&&world.rays==70,"complex full preflight stops exactly at the enclosing query budget");
    check((t.position-start).length()==0,"an exhausted preflight never moves the actor");
    check(TraversalCapture::preflight(t,world,Motion::kickUp,true),"fresh sufficient budget does not inherit a stale negative collision result");
}
static void sourceObstacleDeduplication() {
    fc_test::CornerWorld world;
    world.boxes={{{-10000,0,-10000},{10000,1000,10000}}};
    Traversal t;t.cfg=fc_test::settings();t.cfg.approachSeconds=0;t.cfg.wallRunObstacleJumps=true;
    check(t.attach(world,{0,-37,500},{0,1,0},1000),"obstacle negative-path fixture attaches");
    world.boxes.push_back({{-10000,-30,600},{10000,20,612}});
    auto data=std::make_shared<std::array<AuthoredMotion,42>>();auto& clip=(*data)[int(Motion::kickUp)-1];
    clip.enabled=true;clip.seconds=.8f;clip.trajectory.count=2;clip.trajectory.knots[1]={1,{0,0,224}};t.cfg.authoredMotions=data;
    world.rays=0;const Vec before=t.position;
    const auto result=TraversalCapture::obstacle(t,world);
    std::cout<<"rejected identical source obstacle candidates rays="<<world.rays<<'\n';
    check(!result&&(t.position-before).length()==0,"a rooted kick through a solid beam is still rejected");
    check(world.rays<=400,"identical rooted obstacle paths are checked once per target instead of once per unused outward setting");
}
static void rejectedMantleRetry() {
    fc_test::CornerWorld world;world.boxes={{{-500,0,-100},{500,300,80},true}};
    Traversal reference;reference.cfg.approachSeconds=0;reference.cfg.gap=37;reference.cfg.radius=31;
    check(reference.attach(world,{0,-37,0},{0,1,0},100),"negative mantle fixture attaches");
    reference.update(world,{0,0,false,true},1.f/60,100);
    const auto corner=reference.topPathPoint(19.f/32);
    const float rawFront=reference.topStart().y+reference.cfg.radius,curvedFront=corner.y+reference.cfg.radius;
    const float half=(curvedFront-rawFront)*.20f,z=corner.z+reference.cfg.radius;
    world.boxes.push_back({{-.5f,curvedFront-half,z-.02f},{.5f,curvedFront+half,z+.02f},false});
    Traversal t;t.cfg.approachSeconds=0;t.cfg.gap=37;t.cfg.radius=31;t.cfg.authoredMantle=true;t.cfg.threepeatMantleSeconds=1;
    t.cfg.authoredMantleTrajectory.count=2;t.cfg.authoredMantleTrajectory.knots[1]={1,{0,40,120}};
    auto data=std::make_shared<std::array<AuthoredMotion,42>>();auto& clip=(*data)[int(Motion::contextMantle)-1];
    clip.enabled=true;clip.seconds=1;clip.trajectory=t.cfg.authoredMantleTrajectory;t.cfg.authoredMotions=data;
    check(t.attach(world,{0,-37,0},{0,1,0},100),"synthetic source mantle attaches");
    const Vec start=t.position;State last=t.state;unsigned rays{},peak{},retries{},switches{};bool completed{};
    for(int i=0;i<600&&t.active();++i) {
        world.rays=0;completed|=t.update(world,{0,0,false,true},1.f/60,100).completed;
        rays+=world.rays;peak=std::max(peak,world.rays);
        retries+=std::string(t.ledgeReason)=="authored mantle path blocked";switches+=last!=t.state;last=t.state;
    }
    std::cout<<"blocked mantle retries="<<retries<<" rays="<<rays<<" peak="<<peak<<'\n';
    check(retries>0&&retries<=80,"unchanged blocked mantle does not repeat its full preflight every frame");
    check(rays<90000&&peak<1200,"rejected direct and corridor paths keep a bounded retry workload");
    check(t.active()&&!completed&&switches==0&&(t.position-start).length()<.001f,"rejected mantle stays attached without repeatedly entering and leaving ledge state");
    world.boxes.pop_back();
    for(int i=0;i<300&&t.active();++i)completed|=t.update(world,{0,0,false,true},1.f/60,100).completed;
    check(completed&&!t.active(),"removing the obstruction allows the same attachment to mantle without a stale failure cache");
}
int main()try {
    forwarding();completeRoutes();boundedFailure();sourceObstacleDeduplication();rejectedMantleRetry();
    std::cout<<"Traversal query budget checks="<<checks<<'\n';return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
