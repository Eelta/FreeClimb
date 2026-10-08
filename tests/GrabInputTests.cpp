#include "Controls.h"
#include "EntryInputDiagnostics.h"
#include "NativeWalkableApproach.h"
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
static void contextualEntrySelection() {
    const auto ground=grabFlight(false,false,false,false,0);
    check(grabEntryMotion(ground,0,12)==Motion::reach,"stationary near-wall entry uses the supported standing reach");
    check(grabEntryMotion(ground,90,40)==Motion::reach,"slow grounded approach reaches without a sprint or native jump");
    check(grabEntryMotion(ground,240,60)==Motion::jumpCatch,"fast ground entry keeps the climbing catch rather than adding a sprint pose");
    check(grabEntryMotion(ground,240)==Motion::jumpCatch,"speed alone cannot select a different entry pose");
    check(grabEntryMotion(ground,240,12)==Motion::reach,"a fast but physically near entry uses the same supported reach");
    check(grabEntryMotion(ground,90,80)==Motion::jumpCatch,"a distant slow entry retains the jumping catch");
    check(grabEntryMotion(ground)==Motion::jumpCatch&&grabEntryMotion(ground,90)==Motion::jumpCatch,
        "callers without measured close-wall geometry retain the existing entry fallback");
    const auto predicted=grabFlight(false,false,false,true,0);
    check(grabEntryMotion(predicted,0,12,true)==Motion::reach,
        "the unchanged default Space chord can reach while independent physics still verifies ground support");
    check(grabEntryMotion(predicted,240,60,true)==Motion::jumpCatch,
        "native jump prediction and speed cannot restore the removed sprint catch");
    check(grabEntryMotion(grabFlight(false,false,false,true,-20),0,12,true)==Motion::reach,
        "downhill grounded motion cannot turn native jump prediction into a physical falling catch");
    for(float speed:{0.f,90.f,240.f})for(float distance:{0.f,40.f,80.f}) {
        check(grabEntryMotion(grabFlight(false,true,true,true,120),speed,distance)==Motion::jumpCatch,
            "actual rising flight takes precedence over grounded reach styling");
        check(grabEntryMotion(grabFlight(true,false,false,false,-160),speed,distance)==Motion::ledgeCatch,
            "actual descending flight retains its airborne catch at any approach speed");
        check(grabEntryMotion(grabFlight(false,false,false,true,0),speed,distance)==Motion::jumpCatch,
            "pending native jump remains a jump catch and cannot be restyled into a grounded reach");
        check(grabEntryMotion(grabFlight(false,true,true,true,120),speed,distance,true)==Motion::jumpCatch,
            "confirmed rising physics cannot be overridden by contradictory grounded styling context");
        check(grabEntryMotion(grabFlight(true,false,false,false,-160),speed,distance,true)==Motion::ledgeCatch,
            "confirmed descending physics cannot be overridden by contradictory grounded styling context");
    }
    for(float speed:{-240.f,std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()})
        check(grabEntryMotion(ground,speed,12)==Motion::jumpCatch,"retreating or invalid approach speed cannot request an approach-specific clip");
    for(float distance:{-2.f,-.5f,std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()})
        for(float speed:{0.f,240.f})check(grabEntryMotion(ground,speed,distance)==Motion::jumpCatch,
            "invalid distance retains the conservative catch instead of choosing a grounded variant");
    check(grabEntryMotion(grabFlight(true,false,false,false,-160),std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::infinity())==Motion::ledgeCatch,"invalid horizontal context cannot override physical descent");
}
static void wallRunSpaceRoutingAndEntryGate() {
    for(bool enabled:{false,true})for(bool shift:{false,true})for(bool wasRunning:{false,true})for(bool attached:{false,true}) {
        Keys k;k.w=true;k.space=true;k.shift=shift;
        const auto input=wallInput(k,true,true,attached,wasRunning,enabled);
        check(input.run==(enabled&&shift),"master switch changes attached run input without changing movement keys");
        check(input.hop==(!attached&&!wasRunning&&!(enabled&&shift)),"disabled wall running permits stable climbing hops but preserves entry and exit guards");
        k.s=true;const auto exit=wallInput(k,true,true,false,wasRunning,enabled);
        check(exit.release&&exit.backDrop&&!exit.run&&!exit.hop,"master switch cannot interfere with backward departure");
        k.letGo=true;const auto drop=wallInput(k,false,true,false,wasRunning,enabled);
        check(drop.release&&!drop.backDrop&&!drop.run&&!drop.hop,"master switch preserves independent let-go priority");
    }
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
    run.begin(keys);
    check(!wallInput(run.filter(keys),false,true,false,false,false).run,"disabled mode retains the physical held-at-entry gate");
    check(!wallInput(run.filter(keys),false,true,false,false,true).run,"enabling wall running cannot turn an unreleased entry modifier into a run");
    keys.shift=false;run.filter(keys);keys.shift=true;
    check(wallInput(run.filter(keys),false,true,false,false,true).run,"physical release and repress still rearms after a disabled entry");
    run.begin(keys);keys.s=true;
    const auto released=wallInput(run.filter(keys),true);
    check(released.release&&!released.backDrop,"entry run gate does not swallow the in-place let-go chord");
    Keys native;native.w=native.space=true;check(!approachIntent(native),"W+Space alone remains native jumping");
    native.shift=true;check(!approachIntent(native),"ordinary sprint jumping cannot attach without the complete configured chord");
}
static void nativeJumpWindow() {
    for(int fps:{30,60,120}) {
        const float dt=1.f/fps;NativeJumpIntent jump;ClimbEntryIntent climb;const auto keys=grabKeys();
        check(jump.sample(true,0),"initial physical native jump press opens a short intent window");
        const auto first=climb.sample(keys,false,false,0,false);
        check(first.requested&&first.fresh&&first.began,"complete entry chord still starts immediately");
        bool expired=false;int fresh=1;
        for(int frame=1;frame<=fps*4;++frame) {
            const float elapsed=frame*dt;const bool native=jump.sample(true,dt);
            if(elapsed>.15f+dt)check(!native,"held jump does not remain a native jump request after its short window");
            if(expired)check(!native,"landing while Space is still held cannot manufacture another native press");
            expired|=!native;
            const auto request=climb.sample(keys,false,false,dt,false);fresh+=request.fresh;
            check(request.requested&&!request.fresh&&!request.began,"full held chord retries immediately without another physical press");
            if(expired) {
                check(groundEntryGeometryAllowed(true,false,native,false,false),"expired native Space intent retains staircase and low-obstacle exclusion");
                const auto onGround=grabFlight(false,false,false,native,0);
                check(!onGround.airborne&&!onGround.confirmedAirborne,"expired native intent cannot mark a landed character airborne");
                const auto falling=grabFlight(true,false,false,native,-100);
                check(falling.airborne&&falling.confirmedAirborne&&falling.descending,"real physical flight remains airborne after native intent expires");
                check(!groundEntryGeometryAllowed(false,false,native,falling.airborne,falling.confirmedAirborne),"actual flight does not use grounded entry exclusions");
            }
        }
        check(expired&&fresh==1,"native press expires independently of immediate held-chord retries");
        check(!jump.sample(false,dt)&&jump.sample(true,0),"releasing native jump permits a genuinely new short press window");
        for(int frame=0;frame<fps;++frame)jump.sample(true,dt);
        for(int repeat=0;repeat<50;++repeat)for(float invalid:{0.f,-1.f,std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity()})
            check(!jump.sample(true,invalid),"invalid timing cannot revive an expired still-held jump press");
    }
}
static void preparationGrace() {
    for(int fps:{10,12,15,30,60,120})for(bool gamepad:{false,true}) {
        const float dt=1.f/fps;EntryPreparationGrace grace;const Keys released;
        for(int frame=0;frame<fps;++frame) {
            check(!grace.sample(dt,gamepad,released),"unarmed background frames cannot create a deferred catch");
            check(!grace.sample(dt,gamepad,grabKeys()),"a chord alone does not authorize preparation grace without validated geometry");
        }
        grace.arm(gamepad);check(grace.sample(.000001f,gamepad,released),"a newly armed verified preparation survives an immediate release on the next valid update");
        bool expired=false;float expiry=0;
        for(int frame=1;frame<=fps;++frame) {
            const float elapsed=frame*dt;const bool permitted=grace.sample(dt,gamepad,released);
            if(elapsed<.15f-.0001f)check(permitted,"verified render preparation may finish within the short grace window");
            if(elapsed>.15f+dt+.0001f)check(!permitted,"preparation grace cannot persist past the150ms deadline");
            if(expired)check(!permitted,"expired grace cannot reactivate while released");
            if(!permitted&&!expired){expired=true;expiry=elapsed;}
            const auto movement=wallInput(released,false);
            check(!movement.hop&&!movement.run&&!movement.release&&movement.x==0&&movement.y==0,"retaining verified preparation does not fabricate direction jump or run input");
        }
        check(expired&&expiry>=.15f-.0001f&&expiry<=.15f+dt+.0001f,"grace deadlines remain bounded across low and high frame rates");
        for(int frame=0;frame<fps;++frame)check(!grace.sample(dt,gamepad,grabKeys()),"pressing keys cannot revive expired grace without new verified arming");
        for(int cancelKind=0;cancelKind<4;++cancelKind) {
            grace.arm(gamepad);check(grace.sample(dt,gamepad,released),"cancellation fixture starts with live preparation grace");
            auto keys=released;
            if(cancelKind==0)grace.cancel();
            if(cancelKind==1)keys.s=true;
            if(cancelKind==2)keys.letGo=true;
            check(!grace.sample(dt,cancelKind==3?!gamepad:gamepad,keys),"menu or explicit cancel backward let-go and device switch discard preparation immediately");
            for(int frame=0;frame<fps;++frame)check(!grace.sample(dt,gamepad,released),"clearing cancellation input cannot silently restore a prepared catch");
        }
    }
    for(float dt:{0.f,-1.f,std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity(),.2f,1000.f}) {
        EntryPreparationGrace grace;grace.arm(false);
        check(!grace.sample(dt,false,{}),"invalid timing or a frame beyond the full deadline cancels grace without a time clamp");
        check(!grace.sample(0,false,{})&&!grace.sample(.001f,false,grabKeys()),"hitches and invalid samples cannot leave a revivable authorization");
    }
    for(bool airborneOrigin:{false,true}) {
        ClimbEntryIntent intent;JumpGrabGate gate;EntryPreparationGrace grace;
        const auto request=intent.sample(grabKeys(),false,false,0,airborneOrigin);
        gate.hold({0,1,0},false,request.airborneAtBegin,request.fresh);grace.arm(false);
        const auto released=intent.sample({},false,false,.01f,true);
        check(!released.requested&&!released.fresh&&!released.began,"released chord cannot produce new airborne input intent");
        check(grace.sample(.01f,false,{}),"confirmed preparation may finish briefly after the original physical chord ends");
        gate.hold({1,0,0},false,true,released.fresh);
        const auto falling=grabFlight(true,false,false,false,-100);
        check(gate.explicitAirCatch(falling)==airborneOrigin,"preparation grace cannot upgrade ground-origin intent into a fresh-air cooldown bypass");
        check(!grace.sample(.2f,false,{}),"a pending preparation expires through a real frame hitch");
        gate.cancel();check(!gate.pending()&&!gate.explicitAirCatch(falling),"expiration can fully retire the pending physical preflight");
    }
}
struct PreparedProbeResult {bool armed{},attached{};unsigned probes{};float completedAt{};};
static PreparedProbeResult preparedProbeRetry(int fps,bool oldCooldown,bool initialWall,bool currentWall,unsigned cancellation=0) {
    const float dt=1.f/fps;
    ClimbEntryIntent intent;EntryPreparationGrace grace;float cooldown{};PreparedProbeResult result;
    const auto request=intent.sample(grabKeys(),false,false,dt,true);
    check(request.requested&&entryProbeReady(cooldown,dt,request.fresh),"complete entry gets its immediate initial probe");
    CatchWall world;world.present=initialWall;
    Traversal initial;
    if(initial.attach(world,{0,-30,300},{0,1,0},100,35,true)) {
        grace.arm(true);result.armed=true;
    }
    world.present=currentWall;
    for(int frame=1;frame<=fps/3+2;++frame) {
        Keys released;
        if(cancellation==1)grace.cancel();
        if(cancellation==3)released.letGo=true;
        if(cancellation==5)released.s=true;
        const float elapsed=cancellation==4?.2f:dt;
        const bool prepared=grace.sample(elapsed,cancellation!=2,released);
        const auto held=intent.sample(released,false,cancellation==1,elapsed,true);
        if(!held.requested&&!prepared)break;
        bool probe{};
        if(oldCooldown) {
            cooldown=std::max(0.f,cooldown-std::min(elapsed,.05f));
            probe=cooldown<=0;if(probe)cooldown=.08f;
        } else probe=entryProbeReady(cooldown,elapsed,held.fresh);
        if(!probe)continue;
        ++result.probes;
        Traversal current;
        if(current.attach(world,{0,-30,300},{0,1,0},100,35,true)) {
            result.attached=true;result.completedAt=frame*dt;break;
        }
        grace.cancel();
    }
    return result;
}
static void preparationProbeCadence() {
    for(int fps:{10,12,15,30,60,120}) {
        const auto old=preparedProbeRetry(fps,true,true,true);
        const auto current=preparedProbeRetry(fps,false,true,true);
        check(old.armed&&current.armed,"both cadence fixtures begin with actual validated wall geometry");
        check(old.attached==(fps>=15),"old capped cooldown loses the released prepared entry at10and12fps");
        check(current.attached&&current.probes==1&&current.completedAt>=.08f-.00001f&&current.completedAt<.15f,
            "real elapsed cooldown permits one current-geometry retry within the unchanged150ms grace");
        const auto absent=preparedProbeRetry(fps,false,false,true);
        check(!absent.armed&&!absent.attached&&absent.probes==0,"released input cannot arm grace from a later wall without validated initial geometry");
        const auto disappeared=preparedProbeRetry(fps,false,true,false);
        check(disappeared.armed&&!disappeared.attached&&disappeared.probes==1,"prepared retry rechecks the live wall and rejects support that disappeared");
        for(unsigned cancellation=1;cancellation<=5;++cancellation) {
            const auto canceled=preparedProbeRetry(fps,false,true,true,cancellation);
            check(canceled.armed&&!canceled.attached&&canceled.probes==0,"menu device switch drop expiration and backward cancel before retry");
        }
    }
    for(bool fresh:{false,true})for(float dt:{0.f,-1.f,std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity()}) {
        float remaining=.063f;
        check(!entryProbeReady(remaining,dt,fresh)&&remaining==.063f,"invalid elapsed time cannot advance or reset a probe cooldown even for fresh input");
    }
    float remaining=.063f;
    check(entryProbeReady(remaining,.001f,true)&&remaining==.08f,"genuinely fresh complete chord gets one immediate probe");
    check(!entryProbeReady(remaining,.025f,false)&&std::abs(remaining-.055f)<.000001f,"ordinary held requests retain their real-time throttle");
    check(entryProbeReady(remaining,.1f,false)&&remaining==.08f,"elapsed frame time is not capped before consuming the probe cooldown");
    Keys drop=grabKeys();drop.letGo=true;ClimbEntryIntent intent;
    check(!approachIntent(drop)&&!intent.sample(drop).requested&&intent.waitingForRelease(),"explicit let-go vetoes both approach and entry recognition");
    drop.letGo=false;
    check(!intent.sample(drop).requested,"removing only the drop request does not rearm a still-held entry chord");
    drop.space=false;intent.sample(drop);drop.space=true;
    check(intent.sample(drop).fresh,"release and repress of an entry member rearms after explicit drop");
}
static void entryDiagnosticSnapshotsAndCadence() {
    InputBindings bindings;
    std::array<std::uint8_t,256> raw{};
    raw[0x11]=0x80;
    auto snapshot=diagnosticKeyboardSnapshot(raw);
    check(entryHeldMask(snapshot,bindings.entry)==1&&!chordHeld(snapshot,bindings.entry),"raw W alone stays a partial entry witness");
    EntryInputDiagnostics diagnostic;
    for(std::uint64_t now=0;now<10000;++now)
        check(diagnostic.sample(chordHeld(snapshot,bindings.entry),now)==EntryDiagnosticEvent::none,"ordinary movement never emits entry diagnostic events");
    raw[0x1e]=raw[0x20]=0x80;raw[0x39]=1;
    snapshot=diagnosticKeyboardSnapshot(raw);
    check(entryHeldMask(snapshot,bindings.entry)==7&&!chordHeld(snapshot,bindings.entry),"snapshot requires the actual DirectInput pressed bit for Space");
    raw[0x39]=0x80;snapshot=diagnosticKeyboardSnapshot(raw);
    check(entryHeldMask(snapshot,bindings.entry)==15&&chordHeld(snapshot,bindings.entry),"all four raw chord members are recorded independently of mapped input");
    InputState mapped;
    check(!mapKeys(mapped,bindings).entry&&diagnostic.sample(true,10000)==EntryDiagnosticEvent::began,"raw complete chord remains observable when mapped input is suppressed");
    for(std::uint64_t now=10001;now<12000;++now)
        check(diagnostic.sample(true,now)==EntryDiagnosticEvent::none,"held diagnostics cannot flood within two seconds");
    check(diagnostic.sample(true,12000)==EntryDiagnosticEvent::held,"held chord reports at the bounded interval");
    check(diagnostic.sample(true,12000)==EntryDiagnosticEvent::none,"duplicate timestamps cannot duplicate a held report");
    check(diagnostic.sample(false,12001)==EntryDiagnosticEvent::released,"release is reported immediately for rearming diagnosis");
    check(diagnostic.sample(false,12002)==EntryDiagnosticEvent::none,"released chords do not keep reporting");
    check(diagnostic.sample(true,12003)==EntryDiagnosticEvent::began,"a released and repressed chord starts a new observation immediately");
    check(diagnostic.sample(true,14003,false)==EntryDiagnosticEvent::none,"disabled diagnostics suppress held reports and clear prior state");
    check(diagnostic.sample(false,14004)==EntryDiagnosticEvent::none,"re-enabling diagnostics does not invent a release");
    check(diagnostic.sample(true,14005)==EntryDiagnosticEvent::began,"diagnostics can resume on a currently complete chord");
    bindings.entry=parseKeyChord("Shift+W").value();raw.fill(0);raw[0x36]=raw[0x11]=0x80;
    snapshot=diagnosticKeyboardSnapshot(raw);
    check(chordHeld(snapshot,bindings.entry)&&entryHeldMask(snapshot,bindings.entry)==3,"generic Shift witness accepts the actual right modifier");
    bindings.entry=parseKeyChord("LShift+W").value();
    check(!chordHeld(snapshot,bindings.entry),"exact left Shift witness does not substitute right Shift");
    raw[0x2a]=0x80;
    check(chordHeld(diagnosticKeyboardSnapshot(raw),bindings.entry),"exact modifier witness accepts the configured physical key");
    EntryInputDiagnostics keyboard,gamepad;
    check(keyboard.sample(true,0)==EntryDiagnosticEvent::began&&gamepad.sample(true,0)==EntryDiagnosticEvent::began,"device diagnostic histories remain independent");
    check(keyboard.sample(false,1)==EntryDiagnosticEvent::released&&gamepad.sample(true,1)==EntryDiagnosticEvent::none,"one device release cannot reset the other device throttle");
}
int main(){try {
    chordTruthTableAndNativeSpace();heldApproachAndReleaseRearming();flightClassificationAndFreshAirBypass();
    currentPositionCatchAndDistantHold();contextualEntrySelection();wallRunSpaceRoutingAndEntryGate();nativeJumpWindow();preparationGrace();preparationProbeCadence();entryDiagnosticSnapshotsAndCadence();
    std::cout<<"PASS: full configurable climb chord, all 24 key orders, held retry and release rearm, physical-air provenance, native jump expiry, bounded verified preparation grace and attached run gate\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
