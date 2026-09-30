#include "Controls.h"
#include <algorithm>
#include <array>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace fc;
static void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
struct CatchWall:World {
    bool present=true;
    std::optional<Hit> ray(Vec a,Vec b) override {
        if(present&&a.y<0&&b.y>=0)return Hit{a+(b-a)*(-a.y/(b.y-a.y)),{0,-1,0},true};
        return {};
    }
};
static Keys grabKeys(){Keys k;k.w=k.a=k.d=k.space=true;return k;}
static void chordTruthTableAndNativeSpace() {
    for(unsigned mask=0;mask<64;++mask) {
        const Keys keys{bool(mask&1),bool(mask&2),bool(mask&4),bool(mask&8),bool(mask&16),bool(mask&32)};
        check(approachIntent(keys)==(keys.w&&keys.a&&keys.d&&keys.space&&!keys.s),
            "entry requires every default chord key, independent of Shift, and S vetoes attachment");
    }
    Keys sprint;sprint.w=sprint.shift=true;
    check(!approachIntent(sprint),"ordinary Shift+W never requests entry");
    Keys mapped;mapped.bindingsMapped=true;mapped.entry=true;
    check(approachIntent(mapped),"a custom complete entry chord needs no mandatory movement binding");
    mapped.entry=false;mapped.w=mapped.a=mapped.d=mapped.space=true;
    check(!approachIntent(mapped),"mapped entry never falls back to the physical default chord");
    for(bool attachedAtRelease:{false,true}) {
        SpacePressOwnership ownership;
        check(!ownership.filter(true,false,false),"unattached Space down remains native during an entry request");
        check(ownership.startedNativeJump(),"native Space history remains available to physical flight classification");
        check(!ownership.filter(false,true,attachedAtRelease),"native Space up reaches the engine even after catch");
        check(!ownership.startedNativeJump(),"Space up clears native jump history");
    }
    SpacePressOwnership active;
    check(active.filter(true,false,true)&&active.filter(false,true,true),"already attached Space stays owned by traversal");
    std::array<unsigned,4> scans{0x11,0x1e,0x20,0x39};unsigned orders=0;
    do {
        ClimbEntryIntent intent;Keys keys;
        for(unsigned i=0;i<scans.size();++i) {
            keyboardKey(keys,scans[i],true);const auto result=intent.sample(keys);
            check(result.requested==(i==3),"all 24 key orders wait for the final chord key");
            check(result.fresh==(i==3)&&result.began==(i==3),"only full chord completion creates fresh intent");
        }
        check(!wallInput(keys,true,true,true).hop,"entry Space cannot become a hop on the attachment frame");
        ++orders;
    }while(std::next_permutation(scans.begin(),scans.end()));
    check(orders==24,"all key-order permutations exercised");
}
static void heldApproachAndReleaseRearming() {
    for(int fps:{30,60,120}) {
        Keys k=grabKeys();ClimbEntryIntent intent;JumpGrabGate gate;int fresh=0;
        for(int frame=0;frame<fps*5;++frame) {
            gate.tick(1.f/fps);const auto request=intent.sample(k,false,false,1.f/fps);fresh+=request.fresh;
            check(request.requested,"complete chord immediately requests climbing and retries failed approach while held");
            check(request.began==(frame==0),"held entry begins once without classification delay");
            gate.hold({frame<fps?0.f:1.f,frame<fps?1.f:0.f,0},false,request.airborneAtBegin,request.fresh);
            check(gate.pending(),"held request renews bounded physical preflight");
        }
        check(fresh==1&&gate.facing().x==1,"retry refreshes facing without manufacturing fresh intent");
        check(!intent.sample(k,true).requested&&intent.waitingForRelease(),"successful attachment disarms reacquisition");
        intent.blockUntilRelease();gate.cancel();
        for(int frame=0;frame<fps*5;++frame)check(!intent.sample(k).requested,"holding chord after exit never automatically reacquires on landing");
        k.s=true;check(!intent.sample(k).requested,"backward exit cannot rearm held entry");
        k.s=false;check(!intent.sample(k).requested,"releasing only S cannot rearm entry");
        k.a=false;check(!intent.sample(k).requested&&!intent.waitingForRelease(),"breaking any required chord key rearms future entry");
        k.a=true;auto retry=intent.sample(k);
        check(retry.requested&&retry.fresh&&retry.began,"new chord completion immediately requests a new climb");
        check(!intent.sample(k,false,true).requested,"menu or focus suspension cancels entry");
        for(int frame=0;frame<fps;++frame)check(!intent.sample(k).requested,"resuming while chord is held does not catch unexpectedly");
        k.space=false;intent.sample(k);k.space=true;retry=intent.sample(k);
        check(retry.requested&&retry.fresh,"Space release also rearms after a menu");
        gate.hold({0,1,0},false,false,retry.fresh);gate.tick(0,true);
        check(!gate.pending(),"suspension cancels preflight even without simulation time");
        k.d=false;check(!intent.sample(k).requested,"incomplete chord cancels immediately instead of leaving a tap buffer");
        for(int frame=0;frame<fps;++frame)check(!intent.sample(k).requested,"released chord cannot catch a later wall");
    }
    for(float dt:{0.f,-1.f,std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity()}) {
        ClimbEntryIntent intent;const auto result=intent.sample(grabKeys(),false,false,dt);
        check(result.requested&&result.fresh,"chord recognition does not depend on elapsed-time classification");
    }
}
static void flightClassificationAndFreshAirBypass() {
    const auto ground=grabFlight(false,false,false,false,0);
    check(!ground.airborne&&!ground.confirmedAirborne&&!ground.descending,"ground intent is not flight");
    check(grabEntryMotion(ground)==Motion::jumpCatch,"ground entry always uses a climbing catch");
    check(!grabFlight(false,false,false,false,-150).airborne,"down slope velocity alone is not flight");
    const auto fall=grabFlight(true,false,false,false,-160);
    for(float velocity:{-2.f,-40.f,-800.f}) {
        const auto flight=grabFlight(true,false,false,false,velocity);
        check(flight.confirmedAirborne&&flight.descending,"walking off ledge needs no native jump flag");
        check(grabEntryMotion(flight)==Motion::ledgeCatch,"descending entry uses ledge catch without a ground lift");
    }
    const auto rise=grabFlight(false,true,true,true,120);
    check(rise.confirmedAirborne&&!rise.descending&&grabEntryMotion(rise)==Motion::jumpCatch,"rising native jump uses climbing catch");
    const auto intentOnly=grabFlight(false,false,false,true,0);
    check(intentOnly.airborne&&!intentOnly.confirmedAirborne,"native jump avoids duplicate lift but cannot certify cooldown bypass");
    const auto invalid=grabFlight(true,false,false,false,std::numeric_limits<float>::quiet_NaN());
    check(!invalid.confirmedAirborne&&std::isfinite(invalid.verticalSpeed),"invalid physics cannot certify fresh air catch");
    JumpGrabGate gate;gate.hold({0,1,0},false,false,true);
    for(int frame=0;frame<120;++frame) {
        gate.tick(1.f/60);gate.hold({0,1,0},false,fall.confirmedAirborne,false);
        check(!gate.explicitAirCatch(fall)&&!gate.permitted(.6f,fall),"ground-started hold later falling cannot gain fresh-air cooldown bypass");
    }
    gate.hold({0,1,0},true,true,true);
    check(gate.startedNativeJump()&&gate.permitted(.6f,fall)&&gate.explicitAirCatch(fall),"fresh complete chord in confirmed flight may retry cooldown");
    check(!gate.permitted(.6f,ground)&&!gate.explicitAirCatch(ground),"landing ends airborne cooldown exception");
    gate.hold({1,0,0},false,true,false);
    check(!gate.startedNativeJump()&&gate.explicitAirCatch(fall),"native Space release clears history while retaining flight origin");
    gate.cancel();gate.hold({0,1,0},false,true,false);
    check(!gate.explicitAirCatch(fall),"restoring canceled held request cannot create air bypass");
    gate.hold({std::numeric_limits<float>::quiet_NaN(),1,0},false,true,true);
    check(!gate.pending(),"invalid facing cancels entry");
    ClimbEntryIntent intent;auto request=intent.sample(grabKeys(),false,false,1.f/60,false);
    check(request.fresh&&!request.airborneAtBegin,"gesture records actual initial flight state");
    request=intent.sample(grabKeys(),false,false,1.f/60,true);
    check(request.requested&&!request.fresh&&!request.airborneAtBegin,"later flight cannot alter a held gesture's origin");
}
static void currentPositionCatchAndDistantHold() {
    const auto fall=grabFlight(true,false,false,false,-300);
    for(int fps:{30,60,120}) {
        CatchWall world;Traversal traversal;traversal.stop();Keys k=grabKeys();ClimbEntryIntent intent;JumpGrabGate gate;
        Vec actual{0,-30,300};const float dt=1.f/fps;
        for(int waiting=0;waiting<fps/5+3;++waiting) {
            actual.z+=fall.verticalSpeed*dt;gate.tick(dt);traversal.tickCooldown(dt);
            const auto request=intent.sample(k,false,false,dt,true);
            gate.hold({0,1,0},false,request.airborneAtBegin,request.fresh);
            check(!traversal.active(),"render preflight leaves falling untouched");
        }
        check(gate.permitted(traversal.cooldown,fall),"fresh air request survives callback preparation");
        check(traversal.attach(world,actual,gate.facing(),100,35,gate.explicitAirCatch(fall)),"fresh falling catch retains collision and support checks");
        traversal.entry(grabEntryMotion(fall),!fall.airborne);
        check((traversal.position-actual).length()<.001f,"catch starts at current feet rather than request-time snapshot");
        const auto first=traversal.update(world,wallInput(k,false,true,true),0,100);
        check(first.motion==Motion::ledgeCatch&&traversal.state==State::approach,"falling entry publishes a climbing catch before locomotion");
        check((traversal.position-actual).length()<.001f,"zero-output frame cannot advance entry geometry");
        intent.blockUntilRelease();gate.cancel();
        check(!intent.sample(k).requested&&!gate.pending(),"success consumes entry and requires release");
        Traversal distant;distant.stop();
        check(!distant.attach(world,{0,-100,300},{0,1,0},100,35,true),"held approach cannot extend correction across empty space");
        check(distant.lastFailure==AttachFailure::tooFar&&!distant.active(),"distant failure retains native controller");
        world.present=false;Traversal absent;
        check(!absent.attach(world,{0,-30,300},{0,1,0},100,60,true)&&!absent.active(),"input does not manufacture absent walls");
    }
}
static void wallRunSpaceRoutingAndEntryGate() {
    for(bool shift:{false,true})for(bool wasRunning:{false,true}) {
        Keys k;k.w=true;k.shift=shift;k.space=true;const auto input=wallInput(k,true,true,false,wasRunning);
        check(input.hop==(!shift&&!wasRunning),"wall running suppresses Space including same-frame Shift release");
        k.s=true;const auto exit=wallInput(k,true,true,false,wasRunning);
        check(exit.release&&exit.backDrop&&!exit.hop&&!exit.run,"S+Space retains priority in every running mode");
    }
    Keys keys=grabKeys();keys.shift=true;WallRunEntryGate run;run.begin(keys);
    for(int frame=0;frame<300;++frame) {
        const auto filtered=run.filter(keys);
        check(!filtered.shift&&filtered.w&&filtered.a&&filtered.d&&filtered.space,"held entry run modifier cannot start running or disturb the entry chord");
    }
    keys.shift=false;check(!run.filter(keys).shift,"actual release clears the run block");
    keys.shift=true;check(run.filter(keys).shift,"a subsequent run modifier press may start wall running");
    run.reset();keys.shift=false;run.begin(keys);keys.shift=true;
    check(run.filter(keys).shift,"an entry without the run modifier does not block a new attached run command");
    run.reset();keys.shift=true;check(run.filter(keys).shift,"reset cannot leave a stale run suppression");
    run.begin(keys);keys.s=true;
    const auto released=wallInput(run.filter(keys),true);
    check(released.release&&!released.backDrop,"entry run gate does not swallow the in-place let-go chord");
    Keys native;native.w=native.space=true;check(!approachIntent(native),"W+Space alone remains native jumping");
    native.shift=true;check(!approachIntent(native),"ordinary sprint jumping cannot attach without the complete configured chord");
}
int main(){try {
    chordTruthTableAndNativeSpace();heldApproachAndReleaseRearming();flightClassificationAndFreshAirBypass();
    currentPositionCatchAndDistantHold();wallRunSpaceRoutingAndEntryGate();
    std::cout<<"PASS: full configurable climb chord, all 24 key orders, held retry and release rearm, physical-air provenance, native Space and attached run gate\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
