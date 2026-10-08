#include "TraversalStealth.h"
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string_view>

static unsigned checks{};
static void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
struct ActorState {
    struct Bits1 {
        std::uint32_t moving:1{};
        std::uint32_t running:1{};
        std::uint32_t sneaking:1{};
        std::uint32_t swimming:1{};
        std::uint32_t reserved:28{};
    } actorState1;
    struct Bits2 {
        std::uint32_t forceSneak:1{};
        std::uint32_t reserved:31{};
    } actorState2;
};
struct Actor {
    ActorState state;
    bool valid{true},dead{},graphPresent{true},graphSneaking{};
    unsigned stateReads{},graphReads{},graphWrites{};
    ActorState* AsActorState(){++stateReads;return valid?&state:nullptr;}
    bool IsDead()const{return dead;}
    bool GetGraphVariableBool(std::string_view name,bool& value) {
        ++graphReads;
        check(name=="IsSneaking","only the verified native sneak bool is queried");
        check(!state.actorState1.sneaking,"game state clears before graph handoff");
        if(!graphPresent)return false;
        value=graphSneaking;return true;
    }
    bool SetGraphVariableBool(std::string_view name,bool value) {
        ++graphWrites;
        check(name=="IsSneaking"&&!value,"no graph sneak-on or animation event is sent");
        check(graphPresent&&graphSneaking,"only an existing true bool is cleared");
        check(!state.actorState1.sneaking,"game sneak state ends before graph output");
        graphSneaking=value;return true;
    }
};
static void climbRunClimb() {
    for(bool original:{false,true}) {
        fc::TraversalStealth stealth;Actor actor;
        actor.state.actorState1.sneaking=original;
        actor.state.actorState1.moving=true;actor.state.actorState1.running=true;
        actor.state.actorState1.swimming=true;actor.state.actorState1.reserved=12345;
        actor.state.actorState2.forceSneak=true;actor.state.actorState2.reserved=54321;
        check(stealth.acquire(&actor,false),"acquisition owns a live character");
        check(stealth.active()&&stealth.sneaking()&&actor.state.actorState1.sneaking,"climbing has the game sneak state");
        check(!actor.graphReads&&!actor.graphWrites,"climbing does not start a crouch animation");
        for(unsigned frame=0;frame<240;++frame) {
            const bool wallRunning=frame>=80&&frame<160;
            check(stealth.update(&actor,wallRunning),"logical mode remains owned");
            check(stealth.sneaking()==!wallRunning&&static_cast<bool>(actor.state.actorState1.sneaking)==!wallRunning,"run transitions alter only game stealth state");
        }
        check(!actor.graphReads&&!actor.graphWrites,"mode transitions leave animation graph unchanged");
        check(actor.state.actorState1.moving&&actor.state.actorState1.running&&actor.state.actorState1.swimming&&actor.state.actorState1.reserved==12345,"other live ActorState1 bits are preserved");
        check(actor.state.actorState2.forceSneak&&actor.state.actorState2.reserved==54321,"scripted forceSneak and unrelated flags remain untouched");
        check(stealth.release(&actor),"owned release succeeds");
        check(!actor.state.actorState1.sneaking&&!stealth.active(),"exit never restores pre-climb sneaking");
    }
}
static void allExits() {
    for(unsigned exit=0;exit<3;++exit)for(unsigned graph=0;graph<3;++graph)for(bool wallRunning:{false,true}) {
        fc::TraversalStealth stealth;Actor actor;
        actor.graphPresent=graph!=0;actor.graphSneaking=graph==2;
        check(stealth.acquire(&actor,wallRunning),"all three exit scenarios acquire traversal first");
        check(stealth.release(&actor),"mantle kick and drop share safe cleanup");
        check(!stealth.active()&&!stealth.sneaking()&&!actor.state.actorState1.sneaking,"each exit ends game stealth state");
        check(actor.graphReads==1&&actor.graphWrites==(graph==2?1u:0u),"only a verified true graph bool needs reset");
        const auto reads=actor.stateReads;
        check(!stealth.release(&actor)&&actor.stateReads==reads,"repeated release does not touch a released player");
    }
}
static void menuHoldAndIdle() {
    fc::TraversalStealth stealth;Actor actor;
    check(stealth.acquire(&actor,false),"climbing begins before menu hold");
    for(unsigned frame=0;frame<500;++frame) {
        check(stealth.active()&&stealth.sneaking()&&actor.state.actorState1.sneaking,"menu suspension retains sneak ownership");
    }
    check(stealth.update(&actor,false)&&actor.state.actorState1.sneaking,"resuming does not introduce a stand transition");
    check(stealth.release(&actor),"normal exit follows menu resume");
    actor.state.actorState1.sneaking=true;actor.graphSneaking=true;
    const auto reads=actor.stateReads,graphReads=actor.graphReads,graphWrites=actor.graphWrites;
    for(unsigned frame=0;frame<300;++frame) {
        check(!stealth.update(&actor,false),"idle cannot acquire implicit ownership");
        check(!stealth.release(&actor),"idle release cannot cancel manual crouching");
        check(actor.state.actorState1.sneaking&&actor.graphSneaking,"post-exit manual crouching remains available");
    }
    check(actor.stateReads==reads&&actor.graphReads==graphReads&&actor.graphWrites==graphWrites,"idle has no game state reads or writes");
}
static void invalidAndLoaded() {
    fc::TraversalStealth stealth;Actor actor,other;
    actor.dead=true;actor.state.actorState1.sneaking=true;
    check(!stealth.acquire(&actor,false)&&actor.state.actorState1.sneaking,"dead rejected entry leaves unowned state alone");
    actor.dead=false;check(stealth.acquire(&actor,false),"valid actor later acquires");
    actor.dead=true;
    check(!stealth.update(&actor,false)&&!stealth.active()&&!actor.state.actorState1.sneaking,"death clears an owned state safely");
    actor.dead=false;check(stealth.acquire(&actor,false),"alive reacquires");
    actor.valid=false;
    check(!stealth.update(&actor,false)&&!stealth.active()&&!actor.graphWrites,"invalid current state never touches graph or old storage");
    actor.valid=true;check(stealth.acquire(&actor,false),"fresh typed state reacquires");
    other.state.actorState1.sneaking=true;
    check(!stealth.acquire(&other,false)&&stealth.active(),"different character cannot steal current ownership");
    const auto otherReads=other.stateReads;
    check(!stealth.release(&other)&&!stealth.active()&&other.stateReads==otherReads&&other.state.actorState1.sneaking,"mismatched release never alters an unrelated character");
    check(stealth.acquire(&actor,false),"player ownership recovers explicitly");
    check(!stealth.release(static_cast<Actor*>(nullptr))&&!stealth.active(),"missing player only forgets stored identity");
    other.graphSneaking=true;
    check(stealth.cleanupLoaded(&other)&&!other.state.actorState1.sneaking&&!other.graphSneaking,"identified FreeClimb saved state is cleaned using the current live actor");
    check(!stealth.cleanupLoaded(static_cast<Actor*>(nullptr))&&!stealth.active(),"missing saved actor has no stale dereference");
    check(stealth.acquire(&actor,false),"fresh session acquires before revert");
    stealth.clear();const auto stateReads=actor.stateReads;
    check(!stealth.active()&&!stealth.release(&actor)&&actor.stateReads==stateReads,"revert discards identity without touching unloaded memory");
}
static void liveSetting() {
    for(bool wallRunning:{false,true}) {
        fc::TraversalStealth stealth;Actor actor;
        check(stealth.acquire(&actor,wallRunning,true),"enabled acquisition retains traversal ownership");
        actor.graphSneaking=true;
        check(stealth.update(&actor,wallRunning,false),"disabling sneak retains wall attachment");
        check(stealth.active()&&!stealth.sneaking()&&!actor.state.actorState1.sneaking&&!actor.graphSneaking,"disabling clears owned game and graph states immediately");
        check(actor.graphReads==1&&actor.graphWrites==1,"owned state clears once");
        for(unsigned frame=0;frame<120;++frame)check(stealth.update(&actor,frame%2,false),"disabled mode still validates current actor");
        check(actor.graphReads==1&&actor.graphWrites==1,"disabled mode does not repeatedly clear sneak graph");
        check(stealth.update(&actor,wallRunning,true),"re-enabling applies without reattachment");
        check(stealth.sneaking()==!wallRunning&&static_cast<bool>(actor.state.actorState1.sneaking)==!wallRunning,"re-enabled state matches climbing or wall running");
        check(actor.graphReads==1&&actor.graphWrites==1,"re-enabling sends no crouch animation");
        check(stealth.release(&actor)&&!actor.state.actorState1.sneaking,"all enabled exits clear state");
    }
    for(bool original:{false,true}) {
        fc::TraversalStealth stealth;Actor actor;
        actor.state.actorState1.sneaking=actor.graphSneaking=original;
        check(stealth.acquire(&actor,false,false),"disabled setting allows valid traversal ownership");
        for(unsigned frame=0;frame<120;++frame)check(stealth.update(&actor,frame%2,false),"disabled traversal updates normally");
        check(stealth.release(&actor),"disabled exit releases traversal ownership");
        check(static_cast<bool>(actor.state.actorState1.sneaking)==original&&actor.graphSneaking==original&&!actor.graphReads&&!actor.graphWrites,"disabled feature never owns or clears unrelated sneak state");
        check(!stealth.update(&actor,false,true)&&static_cast<bool>(actor.state.actorState1.sneaking)==original,"enabling on ground cannot force sneak");
    }
    fc::TraversalStealth stealth;Actor saved;
    check(stealth.acquire(&saved,false,false)&&stealth.release(&saved),"current disabled setting does not own sneak state");
    saved.state.actorState1.sneaking=true;saved.graphSneaking=true;
    check(stealth.cleanupLoaded(&saved)&&!saved.state.actorState1.sneaking&&!saved.graphSneaking,"identified saved traversal still clears a previous enabled build's sneak state");
}
int main() {
    try {
        climbRunClimb();allExits();menuHoldAndIdle();invalidAndLoaded();liveSetting();
        std::cout<<checks<<" traversal stealth checks passed\n";return 0;
    } catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
