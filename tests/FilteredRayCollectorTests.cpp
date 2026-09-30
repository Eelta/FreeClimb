#include "PCH.h"
#include "FilteredRayCollector.h"
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <type_traits>

namespace {
unsigned checks{};
void require(bool condition,const char* message) {
    ++checks;
    if(!condition){std::cerr<<"FAIL "<<message<<'\n';std::exit(1);}
}
struct Context {
    const RE::hkpCollidable* ignored{};
    const RE::hkpCollidable* observed{};
    const RE::hkpCdBody* acceptedBody{};
    unsigned keepCalls{};
    unsigned addCalls{};
};
bool keep(void* pointer,const RE::hkpCollidable& root,float fraction) {
    auto& context=*static_cast<Context*>(pointer);
    context.observed=&root;
    ++context.keepCalls;
    return &root!=context.ignored&&std::isfinite(fraction)&&fraction>=0&&fraction<=1;
}
void add(void* pointer,const RE::hkpCdBody& body,const RE::hkpShapeRayCastCollectorOutput& hit) {
    auto& collector=*static_cast<fc::FilteredRayCollector*>(pointer);
    auto& context=*static_cast<Context*>(collector.context);
    ++context.addCalls;
    context.acceptedBody=&body;
    if(hit.hitFraction>=collector.rayHit.hitFraction)return;
    auto* root=&body;
    while(root->parent)root=root->parent;
    collector.rayHit.rootCollidable=static_cast<const RE::hkpCollidable*>(root);
    collector.rayHit.hitFraction=hit.hitFraction;
    collector.earlyOutHitFraction=hit.hitFraction;
}
RE::hkpShapeRayCastCollectorOutput hit(float fraction) {
    RE::hkpShapeRayCastCollectorOutput value{};
    value.hitFraction=fraction;
    return value;
}
void dispatch(RE::hkpRayHitCollector& collector,const RE::hkpCdBody& body,float fraction) {
    collector.AddRayHit(body,hit(fraction));
}
}

int main() {
    static_assert(std::is_base_of_v<RE::hkpClosestRayHitCollector,fc::FilteredRayCollector>);
    Context context{};
    RE::hkpCollidable player{},wall{},farWall{};
    context.ignored=&player;
    fc::FilteredRayCollector collector(&context,keep,add);
    require(collector.enginePrefix()==static_cast<RE::hkpClosestRayHitCollector*>(&collector),"typed closest upcast");
    require(reinterpret_cast<std::uintptr_t>(collector.enginePrefix())==reinterpret_cast<std::uintptr_t>(&collector),"native prefix origin");
    require(reinterpret_cast<std::uintptr_t>(&collector.rayHit)-reinterpret_cast<std::uintptr_t>(&collector)==0x10,"native output offset");
    require(reinterpret_cast<std::uintptr_t>(&collector.context)-reinterpret_cast<std::uintptr_t>(&collector)==0x70,"context after native prefix");
    require(!collector.HasHit()&&collector.earlyOutHitFraction==1.f,"reset closest state");
    dispatch(collector,player,.1f);
    require(context.keepCalls==1&&context.addCalls==0&&!collector.HasHit()&&collector.earlyOutHitFraction==1.f,"rejected nearest leaves initial range unchanged");
    RE::hkpCdBody wallChild{};
    wallChild.parent=&wall;
    dispatch(collector,wallChild,.7f);
    require(context.observed==&wall&&context.acceptedBody==&wallChild,"keep receives root and native receives original child");
    require(context.addCalls==1&&collector.rayHit.rootCollidable==&wall&&collector.earlyOutHitFraction==.7f,"accepted result writes real typed output");
    dispatch(collector,player,.05f);
    require(context.addCalls==1&&collector.rayHit.hitFraction==.7f&&collector.earlyOutHitFraction==.7f,"rejected hit preserves existing closest output");
    dispatch(collector,farWall,.8f);
    require(context.addCalls==2&&collector.rayHit.rootCollidable==&wall&&collector.rayHit.hitFraction==.7f,"accepted farther hit delegates closest policy");
    dispatch(collector,farWall,.5f);
    require(context.addCalls==3&&collector.rayHit.rootCollidable==&farWall&&collector.rayHit.hitFraction==.5f,"accepted nearer hit updates closest policy");
    RE::hkpCdBody cycle{};
    cycle.parent=&cycle;
    const auto previousKeep=context.keepCalls,previousAdd=context.addCalls;
    dispatch(collector,cycle,.01f);
    require(collector.malformed&&context.keepCalls==previousKeep&&context.addCalls==previousAdd&&collector.rayHit.hitFraction==.5f,"cyclic parent chain fails closed without writing hit");
    std::array<RE::hkpCdBody,65> chain{};
    for(std::size_t i=0;i+1<chain.size();++i)chain[i].parent=&chain[i+1];
    chain.back().parent=&wall;
    Context bounded{};
    fc::FilteredRayCollector limited(&bounded,keep,add);
    dispatch(limited,chain[0],.2f);
    require(limited.malformed&&bounded.keepCalls==0&&bounded.addCalls==0&&!limited.HasHit(),"65 parent hops exceed bounded walk");
    fc::FilteredRayCollector maximum(&bounded,keep,add);
    dispatch(maximum,chain[1],.2f);
    require(!maximum.malformed&&bounded.observed==&wall&&maximum.rayHit.rootCollidable==&wall,"64 parent hops retain original valid boundary");
    fc::FilteredRayCollector noKeep(&context,nullptr,add);
    dispatch(noKeep,wall,.1f);
    require(noKeep.malformed&&!noKeep.HasHit(),"absent predicate fails closed");
    fc::FilteredRayCollector noNative(&context,keep,nullptr);
    dispatch(noNative,wall,.1f);
    require(noNative.malformed&&!noNative.HasHit(),"absent native callback fails closed");
    collector.Reset();
    require(!collector.HasHit()&&collector.earlyOutHitFraction==1.f&&collector.rayHit.hitFraction==1.f,"native closest Reset remains coherent");
    std::cout<<"PASS typed collector checks="<<checks<<"; native game dispatch is validated separately\n";
}
