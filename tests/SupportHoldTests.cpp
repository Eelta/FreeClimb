#include "CornerTestWorld.h"
#include "TraversalCapture.h"
#include <iostream>
#include <memory>
#include <stdexcept>

using namespace fc;
static unsigned checks{};
static void require(bool value,const char* text){++checks;if(!value)throw std::runtime_error(text);}

struct HoldWorld final:World {
    fc_test::CornerWorld wall,obstacles;
    bool supportVisible=true;
    unsigned calls{};
    HoldWorld(){wall.boxes={{{-10000,0,-10000},{10000,10000,10000}}};}
    std::optional<Hit> ray(Vec from,Vec to)override {
        ++calls;
        auto hit=supportVisible?wall.ray(from,to):std::optional<Hit>{};
        const auto obstacle=obstacles.ray(from,to);
        if(obstacle&&(!hit||(obstacle->point-from).length()<(hit->point-from).length()))hit=obstacle;
        return hit;
    }
};

static Traversal start(HoldWorld& world) {
    Traversal t;t.cfg=fc_test::settings();t.cfg.contextActions=false;t.cfg.automaticClimbActions=false;
    t.cfg.wallRunObstacleJumps=false;t.cfg.fancyJumps=false;t.cfg.hangDrain=7;
    require(t.attach(world,{0,-42,200},{0,1,0},1000),"finite wall permits initial attachment");
    require(t.state==State::wall&&!t.geometryHolding()&&!t.resting(),"new attachment has no retained hold state");
    return t;
}

static void assertPosition(const Traversal& t,Vec expected,const char* text) {
    require((t.position-expected).length()<.0001f,text);
}

static void loseSupport(HoldWorld& world,Traversal& t) {
    world.supportVisible=false;
    const auto position=t.position;
    const auto result=t.update(world,{0,1},.025f,1000);
    require(t.active()&&!result.released&&t.geometryHolding(),"missing real wall enters a geometry hold");
    assertPosition(t,position,"first missing-support frame does not commit movement");
    require(result.motion==Motion::hang&&result.staminaCost==0,"ordinary unsupported hold uses idle without charging stamina");
}

static void sustainedLoss(int fps) {
    HoldWorld world;auto t=start(world);loseSupport(world,t);
    const auto held=t.position;const auto state=t.state;const float phase=t.progress();
    for(int frame=0;frame<fps*8;++frame) {
        const auto result=t.update(world,{0,1},1.f/fps,1000);
        require(t.active()&&!result.released&&t.geometryHolding(),"support loss lasting eight seconds never forces release");
        assertPosition(t,held,"holding without support never invents movement");
        require(t.state==state&&t.progress()==phase,"hold preserves the active state and clock");
        require(result.motion==Motion::hang&&result.staminaCost==0,"held wall state consumes no hanging or moving stamina");
    }
    world.supportVisible=true;
    for(int frame=0;frame<fps&&t.position.z<=held.z+.1f;++frame) {
        const auto result=t.update(world,{0,1},1.f/fps,1000);
        require(!result.released,"restored real wall does not cause release");
    }
    require(t.active()&&!t.geometryHolding()&&t.position.z>held.z+.1f,"verified restored support allows the original movement to continue");
}

static void cooldownAndPausedTime() {
    HoldWorld world;auto t=start(world);loseSupport(world,t);
    const unsigned calls=world.calls;const auto position=t.position;const float phase=t.progress();
    for(int i=0;i<4;++i) {
        const auto result=t.update(world,{0,1},.02f,1000);
        require(!result.released&&world.calls==calls,"geometry retry cooldown performs no additional World query");
    }
    for(int i=0;i<200;++i) {
        const auto result=t.update(world,{1,1},0,1000);
        require(!result.released&&world.calls==calls&&result.staminaCost==0,"zero game time neither queries geometry nor consumes stamina");
        assertPosition(t,position,"zero game time preserves held position");
        require(t.progress()==phase&&t.geometryHolding(),"zero game time preserves held phase and ownership");
    }
    const auto early=t.update(world,{0,1},.02f,1000);
    require(!early.released&&world.calls==calls,"paused updates do not secretly exhaust the retry cooldown");
    for(int i=0;i<5&&world.calls==calls;++i)t.update(world,{0,1},.025f,1000);
    require(world.calls>calls&&t.active()&&t.geometryHolding(),"elapsed live time eventually retries the missing support");
}

static void restoredButBlocked() {
    HoldWorld world;auto t=start(world);loseSupport(world,t);const Vec position=t.position;
    world.supportVisible=true;
    const float ceiling=position.z+t.cfg.height+.1f;
    world.obstacles.boxes.push_back({{-10000,-10000,ceiling},{10000,10000,ceiling+12},false});
    for(int frame=0;frame<150;++frame) {
        const auto result=t.update(world,{0,1},.025f,1000);
        require(t.active()&&!result.released,"a restored wall behind a real ceiling does not force a drop");
        require(t.position.finite()&&t.position.z+t.cfg.height<=ceiling+.001f,"hold recovery cannot pass the capsule through an actual blocking ceiling");
        require(t.position.y+t.cfg.radius<=.001f&&-t.position.y<=t.cfg.reach,"ceiling recovery retains collision clearance and actual wall reach");
    }
    world.obstacles.boxes.clear();
    for(int frame=0;frame<40&&t.position.z<=position.z+.1f;++frame)t.update(world,{0,1},.025f,1000);
    require(t.active()&&t.position.z>position.z+.1f,"removing the physical obstacle allows movement to resume");
}

static void manualExit(bool back,bool emptyStamina) {
    HoldWorld world;auto t=start(world);loseSupport(world,t);const Vec position=t.position;
    Input release{};release.release=true;release.backDrop=back;release.y=back?-1.f:0;
    const float stamina=emptyStamina?0.f:1000.f;
    auto result=t.update(world,release,.025f,stamina);
    require(result.released||t.state==State::action,"manual release is processed before geometry or stamina hold");
    for(int frame=0;frame<80&&!result.released;++frame)result=t.update(world,{},.025f,stamina);
    require(result.released&&!t.active(),"manual release completes while the original wall is absent");
    require(!t.geometryHolding()&&!t.resting(),"manual exit clears both hold flags");
    if(back)require((t.position-position).dot(Vec{0,-1,0})>1,"backward release retains a collision-checked outward displacement");
    else assertPosition(t,position,"in-place release retains the last safe root position");
}

static void blockedAuthoredManualExit() {
    HoldWorld world;auto t=start(world);loseSupport(world,t);const Vec position=t.position;
    world.supportVisible=true;
    auto motions=std::make_shared<std::array<AuthoredMotion,42>>();
    auto& drop=(*motions)[std::size_t(int(Motion::drop)-1)];
    drop.enabled=true;drop.seconds=.4f;drop.trajectory.count=3;
    drop.trajectory.knots[0]={0,{}};
    drop.trajectory.knots[1]={.5f,{0,70,0}};
    drop.trajectory.knots[2]={1,{}};
    require(drop.trajectory.valid(),"the authored manual-exit fixture is a valid source trajectory");
    t.cfg.authoredMotions=motions;
    Input release{};release.release=true;
    const auto result=t.update(world,release,.025f,0);
    require(result.released&&!t.active()&&result.motion==Motion::drop,"an authored exit crossing a real wall releases safely instead of rolling back into climbing");
    require(std::string_view(result.reason)=="manual release: authored exit path blocked","the authored manual-exit fixture exercises collision preflight rejection");
    assertPosition(t,position,"a rejected authored manual exit retains the last collision-safe position");
    require(!t.geometryHolding()&&!t.resting(),"a rejected authored exit clears geometry and stamina ownership");
}
static void staminaRest(float threshold) {
    HoldWorld world;auto t=start(world);t.cfg.startStamina=threshold;
    const Vec position=t.position;const float required=std::max(1.f,threshold);
    for(int frame=0;frame<320;++frame) {
        const auto result=t.update(world,{0,1},.025f,0);
        require(t.active()&&!result.released&&t.resting(),"zero stamina rests indefinitely on the wall");
        assertPosition(t,position,"zero stamina never advances the root");
        require(result.motion==Motion::hang&&result.staminaCost==0,"resting overrides configured hanging drain");
    }
    for(int frame=0;frame<20;++frame) {
        const auto result=t.update(world,{0,1},.025f,required-.01f);
        require(t.resting()&&!result.released&&result.staminaCost==0,"positive stamina below the recovery threshold remains resting");
        assertPosition(t,position,"below-threshold stamina does not inch the character forward");
    }
    for(int frame=0;frame<40&&t.position.z<=position.z+.1f;++frame)t.update(world,{0,1},.025f,required);
    require(t.active()&&!t.resting()&&t.position.z>position.z+.1f,"the exact configured recovery threshold permits climbing");
}

static void disabledStamina() {
    HoldWorld world;auto t=start(world);t.cfg.staminaEnabled=false;const Vec position=t.position;
    for(int frame=0;frame<12;++frame) {
        const auto result=t.update(world,{0,1},.025f,0);
        require(t.active()&&!t.resting()&&!result.released&&result.staminaCost==0,"disabled stamina never enters a stamina hold");
    }
    require(t.position.z>position.z+1,"no-consumption mode retains normal movement");
}

static void actionInterruption() {
    HoldWorld world;auto t=start(world);Input hop{};hop.y=1;hop.hop=true;
    const auto started=t.update(world,hop,.025f,1000);
    require(t.state==State::action&&hopMotion(started.motion),"real wall geometry permits an upward leap");
    t.update(world,{},.025f,1000);
    const Vec position=t.position;const float phase=t.actionProgress();
    const float next=std::min(1.f,phase+.025f/t.actionDuration());
    const Vec target=t.actionPathPoint(next)+Vec{0,0,t.cfg.chest};
    world.obstacles.boxes.push_back({target-Vec{2,2,2},target+Vec{2,2,2},false});
    for(int frame=0;frame<240;++frame) {
        const auto result=t.update(world,{},.025f,1000);
        require(t.active()&&!result.released&&t.geometryHolding(),"a changed live leap route holds instead of dropping the actor");
        require(t.state==State::action&&t.actionProgress()==phase,"blocked live action retains its state and exact phase");
        assertPosition(t,position,"blocked live action cannot cross newly inserted collision");
        require(result.staminaCost==0,"blocked live action incurs no repeated cost");
    }
    world.obstacles.boxes.clear();
    for(int frame=0;frame<200&&t.state==State::action;++frame) {
        const auto result=t.update(world,{},.025f,1000);
        require(!result.released,"a recovered leap path resumes without losing control ownership");
    }
    require(t.active()&&t.state!=State::action&&!t.geometryHolding(),"cleared leap route completes and returns to wall locomotion");
    require(t.position.z>position.z+1,"recovered leap retains its intended upward travel");
}

static void capturedHold() {
    HoldWorld world;auto t=start(world);loseSupport(world,t);
    auto capture=std::make_unique<TraversalCapture>(),loaded=std::make_unique<TraversalCapture>();
    TraversalCapture::RecordingWorld recorder(world,*capture);
    for(int frame=0;frame<24;++frame) {
        capture->begin(t,{0,1},.025f,1000);
        const auto result=t.update(recorder,{0,1},.025f,1000);capture->finish(t,result);
        std::string error;
        require(capture->complete()&&loaded->deserialize(capture->serialize(),error),"a held snapshot serializes within the geometry capture contract");
        require(loaded->before().geometryHolding()&&loaded->after().geometryHolding(),"capture preserves the geometry-holding state");
        require(loaded->replay().matched,"held capture replays cooldown, retries and final state exactly");
    }
}

int main() {
    try {
        for(int fps:{30,60,120})sustainedLoss(fps);
        cooldownAndPausedTime();restoredButBlocked();
        for(bool back:{false,true})for(bool empty:{false,true})manualExit(back,empty);
        for(float threshold:{0.f,12.f,30.f})staminaRest(threshold);
        blockedAuthoredManualExit();disabledStamina();actionInterruption();capturedHold();
        std::cout<<"support hold checks="<<checks<<" failures=0\n";
        return 0;
    }catch(const std::exception& error) {
        std::cerr<<"support hold checks="<<checks<<" failure="<<error.what()<<'\n';
        return 1;
    }
}
