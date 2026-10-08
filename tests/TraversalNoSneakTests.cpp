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
    bool valid{true},dead{},graphSneaking{};
    unsigned stateReads{},graphReads{},graphWrites{};
    ActorState* AsActorState(){++stateReads;return valid?&state:nullptr;}
    bool IsDead()const{return dead;}
    bool GetGraphVariableBool(std::string_view,bool& value){++graphReads;value=graphSneaking;return true;}
    bool SetGraphVariableBool(std::string_view,bool value){++graphWrites;graphSneaking=value;return true;}
};
static void preserved(const Actor& actor,bool sneaking,bool graphSneaking) {
    check(static_cast<bool>(actor.state.actorState1.sneaking)==sneaking&&actor.graphSneaking==graphSneaking,"manual sneak state and graph state remain unchanged");
    check(!actor.graphReads&&!actor.graphWrites,"optional traversal never reads or writes sneak graph state");
}
static void transitionsAndExits() {
    for(bool sneaking:{false,true})for(bool graphSneaking:{false,true})for(unsigned exit=0;exit<3;++exit) {
        fc::TraversalStealth stealth;Actor actor;
        actor.state.actorState1.sneaking=sneaking;actor.graphSneaking=graphSneaking;
        actor.state.actorState1.moving=true;actor.state.actorState1.running=true;
        actor.state.actorState1.swimming=true;actor.state.actorState1.reserved=12345;
        actor.state.actorState2.forceSneak=true;actor.state.actorState2.reserved=54321;
        check(stealth.acquire(&actor,false)&&stealth.active()&&!stealth.sneaking(),"optional traversal acquires valid ownership without automatic sneaking");
        for(unsigned frame=0;frame<240;++frame) {
            check(stealth.update(&actor,frame>=80&&frame<160,frame%2==0)&&!stealth.sneaking(),"runtime setting cannot enable sneak in the optional build");
            preserved(actor,sneaking,graphSneaking);
        }
        check(stealth.release(&actor)&&!stealth.active(),"mantle kick and drop release optional ownership");
        preserved(actor,sneaking,graphSneaking);
        check(actor.state.actorState1.moving&&actor.state.actorState1.running&&actor.state.actorState1.swimming&&actor.state.actorState1.reserved==12345,"other ActorState1 flags remain unchanged");
        check(actor.state.actorState2.forceSneak&&actor.state.actorState2.reserved==54321,"scripted and unrelated ActorState2 flags remain unchanged");
        const auto reads=actor.stateReads;
        check(!stealth.release(&actor)&&!stealth.update(&actor,false)&&actor.stateReads==reads,"idle cannot acquire ownership or clear manual crouching");
    }
}
static void menuLoadAndRevert() {
    fc::TraversalStealth stealth;Actor actor;
    check(stealth.acquire(&actor,false),"menu hold begins attached");
    actor.state.actorState1.sneaking=true;actor.graphSneaking=true;
    for(unsigned frame=0;frame<500;++frame) {
        check(stealth.active()&&!stealth.sneaking(),"menu hold preserves attachment without owning stealth");
        preserved(actor,true,true);
    }
    check(stealth.update(&actor,false)&&stealth.release(&actor),"menu resume and pre-load release remain successful");
    preserved(actor,true,true);
    check(stealth.cleanupLoaded(&actor)&&!stealth.active(),"loaded marker cleanup discards ownership");
    preserved(actor,true,true);
    check(stealth.acquire(&actor,true)&&stealth.release(&actor),"revert release validates its current owned actor");
    preserved(actor,true,true);
    check(stealth.acquire(&actor,false),"session reacquires before clear");
    stealth.clear();const auto reads=actor.stateReads;
    check(!stealth.release(&actor)&&actor.stateReads==reads,"cleared identity never dereferences an old actor");
    preserved(actor,true,true);
}
static void invalidOwnership() {
    fc::TraversalStealth stealth;Actor actor,other;
    actor.state.actorState1.sneaking=true;actor.graphSneaking=true;actor.dead=true;
    check(!stealth.acquire(&actor,false)&&!stealth.active(),"dead entry retains rejection");
    actor.dead=false;actor.valid=false;
    check(!stealth.acquire(&actor,false)&&!stealth.cleanupLoaded(&actor),"invalid ActorState is never accepted or migrated");
    actor.valid=true;check(stealth.acquire(&actor,false),"valid state reacquires");
    actor.dead=true;
    check(!stealth.update(&actor,false)&&!stealth.active(),"death releases ownership without changing stealth");
    preserved(actor,true,true);
    actor.dead=false;check(stealth.acquire(&actor,false),"alive actor reacquires");
    check(!stealth.acquire(&other,false)&&stealth.active(),"foreign actor cannot steal ownership");
    const auto reads=other.stateReads;
    check(!stealth.release(&other)&&!stealth.active()&&other.stateReads==reads,"foreign release only discards identity");
    check(stealth.acquire(&actor,false),"actor reacquires before null cleanup");
    check(!stealth.release(static_cast<Actor*>(nullptr))&&!stealth.active(),"null release forgets ownership safely");
    check(!stealth.cleanupLoaded(static_cast<Actor*>(nullptr)),"null loaded actor is rejected");
    preserved(actor,true,true);
}
int main() {
    try {
        transitionsAndExits();menuLoadAndRevert();invalidOwnership();
        std::cout<<checks<<" traversal no-sneak checks passed\n";return 0;
    } catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
