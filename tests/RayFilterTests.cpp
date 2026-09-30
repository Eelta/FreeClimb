#include "RayFilterPolicy.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace fc;
static void check(bool ok,const char* reason){if(!ok)throw std::runtime_error(reason);}
static RayHitFacts trigger() {
    RayHitFacts h;h.actorZone=true;h.phantom=true;h.primitiveActivatorWithoutModel=true;return h;
}
static void policy() {
    check(rayHitDecision(trigger())==RayDecision::ignoreTrigger,"primitive-only ActorZone phantom is excluded");
    auto h=trigger();h.primitiveActivatorWithoutModel=false;
    check(rayHitDecision(h)==RayDecision::keep,"unknown/custom phantom is not a verified trigger");
    h=trigger();h.actor=true;
    check(rayHitDecision(h)==RayDecision::keep,"other actors stay blocking even with trigger-like fields");
    h=trigger();h.actorZone=false;
    check(rayHitDecision(h)==RayDecision::keep,"no layer-wide phantom filtering outside ActorZone");
    for(auto response:{RayResponse::unknown,RayResponse::simpleContact,RayResponse::reporting,RayResponse::none}) {
        h={};h.actorZone=true;h.entity=true;h.response=response;
        const auto expected=response==RayResponse::reporting||response==RayResponse::none?
            RayDecision::ignoreTrigger:RayDecision::keep;
        check(rayHitDecision(h)==expected,"only explicitly nonresponsive ActorZone entities may be ignored");
        h.actor=true;check(rayHitDecision(h)==RayDecision::keep,"entity response never hides another actor");
        h.actor=false;h.actorZone=false;check(rayHitDecision(h)==RayDecision::keep,"other collision layers retain original behavior");
    }
    h=trigger();h.phantom=false;
    check(rayHitDecision(h)==RayDecision::keep,"unknown broadphase type fails closed");
    h.actor=true;h.exactPlayer=true;
    check(rayHitDecision(h)==RayDecision::ignorePlayer,"exact owned controller/ref is the sole actor exception");
}

struct Plane {float x;RayHitFacts facts;int id;};
struct Closest {
    float early=1;int id=-1,calls=0;
    void nativeAdd(float fraction,int value) {
        ++calls;if(fraction<early){early=fraction;id=value;}
    }
};

static Closest cast(float from,float to,const std::vector<Plane>& geometry,const std::vector<int>& order) {
    Closest result;
    for(int index:order) {
        const auto& p=geometry[index];
        const float fraction=(p.x-from)/(to-from);
        if(fraction<0||fraction>=result.early)continue;
        dispatchRayHit(p.facts,[&]{result.nativeAdd(fraction,p.id);});
    }
    return result;
}
static void geometricCollection() {
    RayHitFacts self;self.exactPlayer=true;

    const std::vector<Plane> surfaces{{5,self,1},{25,trigger(),2},{25.001f,{},3},{80,{},4}};
    std::vector<int> order{0,1,2,3};int permutations=0;
    do {
        const auto hit=cast(0,100,surfaces,order);
        check(hit.id==3&&std::abs(hit.early-.25001f)<1e-6f,"thin solid immediately behind trigger remains nearest, in every callback order");
        ++permutations;
    } while(std::next_permutation(order.begin(),order.end()));
    check(permutations==24,"all broadphase callback orders exercised");
    auto physical=trigger();physical.phantom=false;physical.entity=true;physical.response=RayResponse::simpleContact;
    auto blocked=surfaces;blocked[1].facts=physical;
    check(cast(0,100,blocked,{0,1,2,3}).id==2,"solid ActorZone hides the wall and blocks traversal");
    auto otherActor=trigger();otherActor.actor=true;blocked[1].facts=otherActor;
    check(cast(0,100,blocked,{0,1,2,3}).id==2,"foreign actor cannot be traversed");
    auto unknown=trigger();unknown.primitiveActivatorWithoutModel=false;blocked[1].facts=unknown;
    check(cast(0,100,blocked,{0,1,2,3}).id==2,"unidentified phantom remains nearest blocker");
    const auto onlyTriggers=cast(0,100,{{5,self,1},{25,trigger(),2}},{0,1});
    check(onlyTriggers.id==-1&&onlyTriggers.early==1&&onlyTriggers.calls==0,"ignored callbacks never mutate native output or search fraction");
    const auto before=cast(0,100,{{24,{},1},{25,trigger(),2},{25.001f,{},3}},{1,2,0});
    check(before.id==1,"solid in front of ignored trigger remains the actual nearest hit");
    check(cast(100,0,surfaces,{1,0,2,3}).id==4,"reverse rays preserve nearest geometry rather than filtering by travel direction");
}
int main() {
    try {policy();geometricCollection();std::cout<<"PASS: narrow ray policy and thin-wall nearest-hit collection in all callback orders\n";}
    catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
