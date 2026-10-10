#include "CornerTestWorld.h"
#include <iostream>
#include <memory>
#include <stdexcept>

using namespace fc;
namespace fc {
class TraversalCapture {
public:
    static std::optional<Result> recover(Traversal& t,World& world,Input input) {return t.trySupportRecovery(world,input,1000);}
    static std::optional<Hit> genericLanding(Traversal& t,World& world) {return t.support(world,t.actionTo,t.actionLandingNormal*-1);}
    static Vec targetNormal(const Traversal& t) {return t.actionTargetSurface;}
};
}
static unsigned checks{};
static void require(bool value,const char* text){++checks;if(!value)throw std::runtime_error(text);}

struct RecoveryWorld final:World {
    fc_test::CornerWorld source,targets,obstacles;
    bool sourceVisible=true;
    unsigned calls{};
    RecoveryWorld(){source.boxes={{{-10000,0,-10000},{10000,10000,10000}}};}
    std::optional<Hit> ray(Vec from,Vec to)override {
        ++calls;
        auto hit=sourceVisible?source.ray(from,to):std::optional<Hit>{};
        for(auto* world:{&targets,&obstacles})if(auto next=world->ray(from,to))
            if(!hit||(next->point-from).length()<(hit->point-from).length())hit=next;
        return hit;
    }
};

static Traversal start(RecoveryWorld& world) {
    Traversal t;t.cfg=fc_test::settings();t.cfg.contextActions=false;t.cfg.automaticClimbActions=false;
    t.cfg.wallRunObstacleJumps=false;t.cfg.fancyJumps=false;
    require(t.attach(world,{0,-42,200},{0,1,0},1000),"source fixture has a real initial wall");
    world.sourceVisible=false;
    return t;
}

static void destination(RecoveryWorld& world,int direction) {
    if(direction==0)world.targets.boxes={{{-10000,0,350},{10000,10000,10000}}};
    else if(direction==1)world.targets.boxes={{{100,0,-10000},{10000,10000,10000}}};
    else if(direction==2)world.targets.boxes={{{-10000,0,-10000},{-100,10000,10000}}};
    else world.targets.boxes={{{-10000,0,-10000},{10000,10000,224}}};
}

static Input intent(int direction) {
    Input input;
    if(direction==0)input.y=1;
    else if(direction==1)input.x=1;
    else if(direction==2)input.x=-1;
    else input.y=-1;
    return input;
}

static bool launch(RecoveryWorld& world,Traversal& t,Input input,unsigned frames=180) {
    for(unsigned frame=0;frame<frames;++frame) {
        const auto result=t.update(world,input,.025f,1000);
        require(t.active()&&!result.released,"searching nearby support does not release the player");
        if(t.recoveringSupport())return true;
    }
    return false;
}

static void recoveryDirection(int direction) {
    RecoveryWorld world;auto t=start(world);destination(world,direction);const Vec startPosition=t.position;
    const auto input=intent(direction);
    require(launch(world,t,input),"an actually reachable nearby wall starts a checked recovery action");
    require((t.position-startPosition).length()<.001f,"planning a recovery does not teleport to its destination");
    const auto target=t.actionPathPoint(1);const auto motion=t.update(world,{},0,1000).motion;
    if(direction==3)require(motion==Motion::down&&target.z<startPosition.z,"downward recovery uses a descending clip instead of an upward jump");
    else require(hopMotion(motion),"upward and sideways recovery reuse a leap clip");
    std::array<Vec,33> path{};
    for(unsigned sample=0;sample<path.size();++sample)path[sample]=t.actionPathPoint(float(sample)/32);
    for(unsigned frame=0;frame<200&&t.state==State::action;++frame) {
        const auto result=t.update(world,{},.025f,1000);
        require(!result.released&&!t.geometryHolding(),"a stable checked recovery route completes without dropping or freezing");
        require(world.targets.clearance(t.position,t.cfg)>=t.cfg.radius-.01f,"recovery never enters the actual destination solid");
        if(t.state==State::action)for(unsigned sample=0;sample<path.size();++sample)
            require((t.actionPathPoint(float(sample)/32)-path[sample]).length()<.001f,"live orientation changes cannot alter a prechecked recovery trajectory");
    }
    require(t.state==State::wall&&!t.recoveringSupport(),"successful recovery returns to ordinary wall movement");
    require((t.position-target).length()<.01f,"recovery lands at its prechecked endpoint");
    const Vec landed=t.position;
    for(unsigned frame=0;frame<8;++frame)t.update(world,input,.025f,1000);
    require((t.position-landed).length()>.1f,"normal movement continues after recovery");
}

static void noIntentAndWrongDirection() {
    RecoveryWorld world;auto t=start(world);destination(world,1);const Vec point=t.position;
    for(int frame=0;frame<160;++frame) {
        Input idle{};if(frame%2)idle.x=.01f;
        t.update(world,idle,.025f,1000);
        require(!t.recoveringSupport()&&(t.position-point).length()<.001f,"an idle held player does not jump to a nearby wall");
    }
    for(int frame=0;frame<120;++frame) {
        t.update(world,intent(2),.025f,1000);
        require(!t.recoveringSupport()&&(t.position-point).length()<.001f,"recovery does not jump opposite the player's direction");
    }
    require(launch(world,t,intent(1)),"changing direction permits a newly relevant valid destination");
}

static void blockedRoute(int type) {
    RecoveryWorld world;auto t=start(world);destination(world,0);const Vec point=t.position;
    if(type==0)world.obstacles.boxes={{{-10000,-10000,339},{10000,10000,343},false}};
    else if(type==1)world.obstacles.boxes={{{-4,-10000,339},{4,10000,400},false}};
    else world.obstacles.boxes={{{-10000,-10000,339},{10000,10000,10000},false}};
    for(int frame=0;frame<160;++frame) {
        const unsigned before=world.calls;
        const auto result=t.update(world,intent(0),.025f,1000);
        require(!result.released&&!t.recoveringSupport(),"a real ceiling, beam or solid wall blocks recovery");
        require((t.position-point).length()<.001f,"failed route prechecks never commit part of a jump");
        require(world.calls-before<6000,"failed recovery search retains a bounded per-update collision cost");
    }
}

static void liveRouteChange() {
    RecoveryWorld world;auto t=start(world);destination(world,0);
    require(launch(world,t,intent(0)),"live route test starts a checked recovery");
    t.update(world,{},.025f,1000);
    const Vec point=t.position;const float phase=t.actionProgress();
    world.targets.boxes.clear();
    for(int frame=0;frame<120;++frame) {
        t.update(world,{},.025f,1000);
        require(t.geometryHolding()&&t.recoveringSupport(),"a vanished destination freezes the ongoing recovery");
        require((t.position-point).length()<.001f&&t.actionProgress()==phase,"lost recovery support cannot consume trajectory progress");
    }
    destination(world,0);
    for(int frame=0;frame<200&&t.state==State::action;++frame)t.update(world,{},.025f,1000);
    require(t.state==State::wall&&!t.geometryHolding(),"the same destination can safely resume a held recovery");
}

static void pauseRestAndManualExit() {
    RecoveryWorld world;auto t=start(world);destination(world,0);const Vec point=t.position;
    for(int frame=0;frame<40;++frame)t.update(world,intent(0),.025f,0);
    require(t.resting()&&!t.recoveringSupport()&&(t.position-point).length()<.001f,"empty stamina never launches a recovery");
    const auto calls=world.calls;
    for(int frame=0;frame<100;++frame)t.update(world,intent(0),0,1000);
    require(world.calls==calls&&!t.recoveringSupport(),"paused game time performs no recovery search");
    require(launch(world,t,intent(0)),"recovered stamina allows a real route search");
    Input release{};release.release=true;
    auto result=t.update(world,release,.025f,0);
    for(int frame=0;frame<40&&!result.released;++frame)result=t.update(world,{},.025f,0);
    require(result.released&&!t.active(),"manual release overrides an active recovery even with no stamina");
}

static void liveBlockingGeometry() {
    RecoveryWorld world;auto t=start(world);destination(world,0);
    require(launch(world,t,intent(0)),"dynamic collision test starts from a clear prechecked path");
    world.obstacles.boxes={{{-10000,-10000,342},{10000,10000,346},false}};
    for(int frame=0;frame<160;++frame) {
        const auto result=t.update(world,{},.025f,1000);
        require(!result.released&&t.position.z+t.cfg.height<=342.001f,"a newly inserted ceiling stops an ongoing recovery before penetration");
    }
    require(t.geometryHolding()&&t.recoveringSupport(),"live route collision retains an interrupted recovery");
    Input release{};release.release=true;release.backDrop=true;release.y=-1;
    auto result=t.update(world,release,.025f,1000);
    for(int frame=0;frame<80&&!result.released;++frame)result=t.update(world,{},.025f,1000);
    require(result.released&&!t.active(),"backward manual departure can leave a blocked recovery");
}

static void wallRunSwitch() {
    RecoveryWorld world;auto t=start(world);world.sourceVisible=true;
    t.cfg.wallRunObstacleJumps=false;
    world.obstacles.boxes={{{-10000,-10000,339},{10000,10000,343},false}};
    auto input=intent(0);input.run=true;
    for(int frame=0;frame<120;++frame) {
        t.update(world,input,.025f,1000);
        require(!t.recoveringSupport(),"an obstructed supported wall run respects the disabled obstacle-jump option");
    }
    world.sourceVisible=false;world.obstacles.boxes.clear();destination(world,0);
    require(launch(world,t,input),"lost physical support can recover after dropping the wall-run mode");
    require(!t.wallRunning(),"support-loss recovery does not re-enable disabled automatic wall-run jumps");
}

static void authoredRoute() {
    RecoveryWorld world;auto t=start(world);destination(world,0);const Vec point=t.position;
    auto motions=std::make_shared<std::array<AuthoredMotion,42>>();
    auto& hop=(*motions)[std::size_t(int(Motion::hopUp)-1)];
    hop.enabled=true;hop.seconds=.5f;hop.trajectory.count=3;
    hop.trajectory.knots[0]={0,{}};hop.trajectory.knots[1]={.5f,{0,100,24}};hop.trajectory.knots[2]={1,{0,0,48}};
    require(hop.trajectory.valid(),"authored recovery fixture contains a valid curved root trajectory");
    t.cfg.authoredMotions=motions;
    for(int frame=0;frame<100;++frame) {
        t.update(world,intent(0),.025f,1000);
        require(!t.recoveringSupport()&&(t.position-point).length()<.001f,"an imported root curve that enters the actual target wall fails complete preflight");
    }
    hop.trajectory.knots[1].displacement={0,0,24};
    require(launch(world,t,intent(0)),"a collision-free imported root trajectory can recover onto real support");
}

static Traversal plannedSideRecovery(RecoveryWorld& world) {
    auto t=start(world);
    world.targets.boxes={{{38,0,-1000},{500,100,1500}}};
    require(bool(TraversalCapture::recover(t,world,{1,0})),"a finite side wall provides a checked initial recovery");
    return t;
}

static void changedDestinationReroute(bool barrier) {
    RecoveryWorld world;auto t=plannedSideRecovery(world);
    t.update(world,{},.025f,1000);
    world.targets.boxes.clear();t.update(world,{},.025f,1000);
    require(t.geometryHolding()&&t.recoveringSupport(),"vanished destination holds an already moving action");
    const Vec held=t.position;const float phase=t.actionProgress();
    world.targets.boxes={{{-500,0,-1000},{-38,100,1500}}};
    if(barrier)world.obstacles.boxes={{{-34,-500,-1000},{-32,500,1500},false}};
    const unsigned pausedCalls=world.calls;
    for(unsigned frame=0;frame<30;++frame) {
        t.update(world,{-1,0},0,1000);
        require(world.calls==pausedCalls&&t.actionProgress()==phase&&(t.position-held).length()<.001f,"paused reroute consumes neither collision queries nor action phase");
    }
    for(unsigned frame=0;frame<30;++frame)t.update(world,{-1,0},.025f,0);
    require(world.calls==pausedCalls&&t.actionProgress()==phase&&(t.position-held).length()<.001f,"exhaustion cannot launch or advance a held reroute");
    float paid{};bool replanned{};
    for(unsigned frame=0;frame<160;++frame) {
        const unsigned before=world.calls;
        const auto result=t.update(world,{-1,0},.025f,1000);paid+=result.staminaCost;
        require(t.active()&&!result.released,"invalidated recovery permits rerouting without dropping the player");
        require(world.calls-before<5000,"one transactional reroute retains its bounded collision budget");
        if(t.recoveringSupport()&&t.actionPathPoint(1).x<held.x)replanned=true;
        if(barrier) {
            require(!replanned&&paid==0,"a real barrier rejects replanning without charging stamina");
            require((t.position-held).length()<.001f&&t.actionProgress()==phase,"a failed reroute preserves the original root and phase");
        } else {
            require(world.targets.clearance(t.position,t.cfg)>=t.cfg.radius-.001f,"a replacement route remains outside the destination solid");
            if(t.state==State::wall)break;
        }
    }
    if(barrier) {
        Input release{};release.release=true;
        auto result=t.update(world,release,.025f,1000);
        for(unsigned frame=0;frame<80&&!result.released;++frame)result=t.update(world,{},.025f,1000);
        require(result.released&&!t.active(),"manual release still overrides an unplannable held route");
    } else {
        require(replanned&&t.state==State::wall&&t.position.x<-38,"new directional intent reaches an alternative real wall");
        require(paid>=15&&paid<16,"a successful replacement route charges the recovery cost only once");
    }
}

static void finalCorrectionRecovery() {
    RecoveryWorld world;auto t=plannedSideRecovery(world);
    for(unsigned frame=0;frame<100&&t.actionProgress()<.8f;++frame)t.update(world,{},.025f,1000);
    const Vec endpoint=t.actionPathPoint(1);
    world.targets.boxes.front().low.y=-.8f;
    const Vec obstacle=endpoint+Vec{0,-t.cfg.radius-.4f,t.cfg.chest};
    world.obstacles.boxes={{obstacle-Vec{.15f,.1f,.15f},obstacle+Vec{.15f,.1f,.15f},false}};
    for(unsigned frame=0;frame<20&&!t.geometryHolding();++frame)t.update(world,{},.025f,1000);
    require(t.geometryHolding()&&t.actionProgress()<1&&t.blockedReason==std::string_view("jump landing clearance blocked"),"a tiny real obstruction prevents the final endpoint correction");
    const Vec held=t.position;
    for(unsigned frame=0;frame<30&&t.recoveringSupport();++frame)t.update(world,{-1,0},.025f,1000);
    require(t.state==State::wall&&!t.recoveringSupport(),"verified current support can cancel a blocked final correction");
    require((t.position-held).length()<.001f,"returning to real current support does not teleport through the obstruction");
}

static void stableMultiFaceLanding() {
    struct MultiWorld final:World {
        std::array<fc_test::CornerWorld,2> solids;
        std::optional<Hit> ray(Vec from,Vec to)override {
            std::optional<Hit> hit;
            for(auto& world:solids)if(auto next=world.ray(from,to))
                if(!hit||(next->point-from).length()<(hit->point-from).length())hit=next;
            return hit;
        }
    } world;
    constexpr float values[]={.200848699f,.868164241f,.408150315f,.562136471f,.654349148f,.699662387f,.713826895f,.915579081f,.575786114f};
    world.solids[0].boxes={{{values[0]*80,0,-1000},{500,100,1500}}};
    world.solids[0].yaw=(values[1]-.5f)*1.4f;
    world.solids[1].boxes={{{values[2]*150,values[3]*30-20,100+values[4]*160},{160+values[5]*300,40+values[6]*60,300+values[7]*300}}};
    world.solids[1].yaw=(values[8]-.5f)*2;
    Traversal t;t.cfg=fc_test::settings();t.cfg.contextActions=false;t.cfg.automaticClimbActions=false;
    t.state=State::wall;t.position={0,-37,200};t.normal=t.surfaceNormal={0,-1,0};
    require(bool(TraversalCapture::recover(t,world,{1,0})),"a finite two-face seam retains a reachable recovery route");
    const auto generic=TraversalCapture::genericLanding(t,world);
    require(generic&&generic->normal.dot(TraversalCapture::targetNormal(t))<.95f,"the seam fixture contains a distinct competing valid support plane");
    const Vec endpoint=t.actionPathPoint(1);
    for(unsigned frame=0;frame<100&&t.state==State::action;++frame) {
        const auto result=t.update(world,{},.025f,1000);
        require(!result.released&&!t.geometryHolding(),"an unchanged intended face is not rejected because another support plane is preferred");
        for(const auto& solid:world.solids)require(solid.clearance(t.position,t.cfg)>=t.cfg.radius-.001f,"the matched-face recovery never enters either finite solid");
    }
    require(t.state==State::wall&&(t.position-endpoint).length()<.01f,"planning and live validation complete the same verified seam landing");
}

int main() {
    try {
        for(int direction=0;direction<4;++direction)recoveryDirection(direction);
        noIntentAndWrongDirection();for(int type=0;type<3;++type)blockedRoute(type);
        liveRouteChange();liveBlockingGeometry();pauseRestAndManualExit();wallRunSwitch();authoredRoute();
        changedDestinationReroute(false);changedDestinationReroute(true);finalCorrectionRecovery();stableMultiFaceLanding();
        std::cout<<"support recovery checks="<<checks<<" failures=0\n";return 0;
    }catch(const std::exception& error) {
        std::cerr<<"support recovery checks="<<checks<<" failure="<<error.what()<<'\n';return 1;
    }
}
