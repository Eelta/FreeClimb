#include "RayCandidateCache.h"
#include "RayFilterPolicy.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <iostream>
#include <random>
#include <stdexcept>
#include <vector>

namespace {
unsigned checks{};
void require(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
struct alignas(16) Body {fc::RayHitFacts facts;unsigned reference{};};
struct Classification {fc::RayDecision decision{};unsigned reference{};};
struct Callback {Body* body;float fraction;};
struct Result {
    unsigned classifications{},candidates{},accepted{},self{},triggers{},reference{};
    float fraction=1;
};
Result trace(const std::vector<Callback>& callbacks,bool reuse) {
    fc::RayCandidateCache<Classification> cache;Result result;
    for(const auto& callback:callbacks) {
        if(callback.fraction>=result.fraction)continue;
        ++result.candidates;
        const auto classify=[&] {
            ++result.classifications;
            return Classification{fc::rayHitDecision(callback.body->facts),callback.body->reference};
        };
        const auto value=reuse?cache.resolve(callback.body,classify):classify();
        if(value.decision==fc::RayDecision::ignorePlayer)++result.self;
        else if(value.decision==fc::RayDecision::ignoreTrigger)++result.triggers;
        else {++result.accepted;result.fraction=callback.fraction;result.reference=value.reference;}
    }
    return result;
}
void equal(const Result& a,const Result& b) {
    require(a.candidates==b.candidates&&a.accepted==b.accepted&&a.self==b.self&&a.triggers==b.triggers&&
        a.reference==b.reference&&a.fraction==b.fraction,"reuse preserves nearest hit, filtering, callback counters and native early-out");
}
fc::RayHitFacts trigger() {
    fc::RayHitFacts facts;facts.actorZone=true;facts.phantom=true;facts.primitiveActivatorWithoutModel=true;return facts;
}
void denseMesh() {
    Body wall{{},17};std::vector<Callback> callbacks;
    for(unsigned i=0;i<8192;++i)callbacks.push_back({&wall,.9f-float(i)/16384});
    const auto before=trace(callbacks,false),after=trace(callbacks,true);equal(before,after);
    require(before.classifications==8192&&after.classifications==1,"one ray classifies a repeated mesh root once");
    std::cout<<"Dense mesh callbacks="<<after.candidates<<" classifications="<<before.classifications<<" -> "<<after.classifications<<'\n';
}
void mixedGeometry() {
    std::array<Body,64> bodies{};
    for(unsigned i=0;i<bodies.size();++i) {
        auto& body=bodies[i];body.reference=i+1;
        switch(i%7) {
        case 0:body.facts.exactPlayer=true;break;
        case 1:body.facts=trigger();break;
        case 2:body.facts=trigger();body.facts.actor=true;break;
        case 3:body.facts.actorZone=true;body.facts.entity=true;body.facts.response=fc::RayResponse::simpleContact;break;
        case 4:body.facts=trigger();body.facts.primitiveActivatorWithoutModel=false;break;
        case 5:body.facts.actorZone=true;body.facts.entity=true;body.facts.response=fc::RayResponse::reporting;break;
        default:break;
        }
    }
    std::mt19937 random(51);
    for(unsigned trial=0;trial<200;++trial) {
        std::vector<Callback> callbacks;
        for(unsigned i=0;i<2048;++i)callbacks.push_back({&bodies[random()%bodies.size()],.99f-float(i)/4096});
        const auto before=trace(callbacks,false),after=trace(callbacks,true);equal(before,after);
        require(after.classifications<=after.candidates,"cache saturation still evaluates every unremembered body");
    }
    std::array<Body,4> thin{{{trigger(),1},{{},2},{{},3},{{},4}}};thin[3].facts.exactPlayer=true;
    std::array<unsigned,4> order{0,1,2,3};
    do {
        const std::array<float,4> fractions{.25f,.250001f,.8f,.05f};std::vector<Callback> callbacks;
        for(auto i:order)for(unsigned n=0;n<4;++n)callbacks.push_back({&thin[i],fractions[i]});
        const auto after=trace(callbacks,true);equal(trace(callbacks,false),after);
        require(after.reference==2&&after.fraction==fractions[1],"thin wall immediately behind a trigger remains the exact nearest solid");
    } while(std::next_permutation(order.begin(),order.end()));
}
void nearestReferenceAfterEviction() {
    std::array<Body,64> bodies{};
    for(unsigned i=0;i<bodies.size();++i)bodies[i].reference=i+1;
    for(unsigned i=1;i<bodies.size();++i) {
        bodies[i].facts=trigger();fc::RayCandidateCache<Classification> cache;unsigned calls{};
        const auto read=[&](unsigned index){return cache.resolve(&bodies[index],[&]{
            ++calls;return Classification{fc::rayHitDecision(bodies[index].facts),bodies[index].reference};});};
        const auto solid=read(0),ignored=read(i),final=read(0);
        require(solid.decision==fc::RayDecision::keep&&ignored.decision==fc::RayDecision::ignoreTrigger&&
            final.reference==solid.reference&&final.decision==solid.decision,
            "reading the nearest solid after an ignored trigger never reuses the trigger reference");
        if(calls==3)return;
    }
    require(false,"fixture must exercise an actual fixed-capacity cache eviction");
}
void nextRayReclassifies() {
    Body body{trigger(),23};std::vector<Callback> callbacks{{&body,.3f},{&body,.2f}};
    require(trace(callbacks,true).reference==0,"verified trigger is ignored");
    body.facts={};body.reference=47;
    const auto solid=trace(callbacks,true);equal(trace(callbacks,false),solid);
    require(solid.reference==47&&solid.classifications==1,"same address and changed facts are freshly read on the next ray");
    body.facts.actor=true;body.facts.actorZone=true;body.facts.phantom=true;body.facts.primitiveActivatorWithoutModel=true;
    require(trace(callbacks,true).reference==47,"foreign actor remains blocking on subsequent ray");
    body.facts.exactPlayer=true;
    require(trace(callbacks,true).self==2,"next ray independently recognizes exact player ownership");
}
}
int main()try{denseMesh();mixedGeometry();nearestReferenceAfterEviction();nextRayReclassifies();std::cout<<checks<<" checks passed\n";return 0;}
catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
