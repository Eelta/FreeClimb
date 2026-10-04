#include "HookPublication.h"
#include <array>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
void require(bool value,const char* message) {if(!value)throw std::runtime_error(message);}
struct Device {unsigned calls{};float elapsed{};std::vector<unsigned> order;};
using Poll=void(*)(Device*,float);
fc::HookPublication<Poll>* inner{};
fc::HookPublication<Poll>* outer{};
void nativePoll(Device* self,float elapsed) {++self->calls;self->elapsed=elapsed;self->order.push_back(0);}
void otherPoll(Device* self,float elapsed) {++self->calls;self->elapsed=elapsed;self->order.push_back(3);}
void innerPoll(Device* self,float elapsed) {
    auto prior=inner->get();require(prior!=nullptr,"inner predecessor must exist before publication");
    self->order.push_back(1);prior(self,elapsed);
}
void outerPoll(Device* self,float elapsed) {
    auto prior=outer->get();require(prior!=nullptr,"outer predecessor must exist before publication");
    self->order.push_back(2);prior(self,elapsed);
}
using Filter=bool(*)(Device*,int);
fc::HookPublication<Filter>* filter{};
bool nativeFilter(Device* self,int key) {++self->calls;return key==7;}
bool filterHook(Device* self,int key) {return filter->get()(self,key);}

void delayedOriginalControl() {
    std::atomic<Poll> original{},slot{nativePoll};
    bool missing{};
    const auto prior=slot.exchange(innerPoll);
    std::thread callback([&]{missing=slot.load()==innerPoll&&original.load()==nullptr;});callback.join();
    original.store(prior);
    require(missing&&original.load()==nativePoll,
        "publishing the hook before assigning its original exposes a null target to the immediate callback");
}

void publicationBoundary() {
    fc::HookPublication<Poll> first,second;
    inner=&first;outer=&second;
    std::atomic<Poll> slot{nativePoll};
    Device device;
    unsigned publishes{};
    auto publish=[&](Poll expected,Poll replacement) {
        ++publishes;
        const bool changed=slot.compare_exchange_strong(expected,replacement);
        if(changed) {
            std::thread callback([&]{slot.load()(&device,.125f);});callback.join();
        }
        return changed;
    };
    require(first.get()==nullptr,"uninstalled hook starts empty");
    require(first.install(nativePoll,innerPoll,publish),"first hook installs");
    require(device.calls==1&&device.elapsed==.125f&&device.order==std::vector<unsigned>{1,0},
        "callback during publication preserves this, float and exactly one original call");
    device.order.clear();
    require(second.install(innerPoll,outerPoll,publish),"outer hook preserves the existing chain");
    require(device.calls==2&&device.order==std::vector<unsigned>{2,1,0},"nested hook chain calls original once");
    const auto count=publishes;
    require(first.install(slot.load(),innerPoll,publish)&&second.install(slot.load(),outerPoll,publish),
        "repeated installation succeeds without reinserting inner or outer hooks");
    require(publishes==count&&slot.load()==outerPoll&&first.get()==nativePoll&&second.get()==innerPoll,
        "repeated installation preserves predecessor pointers and outer ownership");
    device.order.clear();slot.load()(&device,.25f);
    require(device.calls==3&&device.elapsed==.25f&&device.order==std::vector<unsigned>{2,1,0},
        "duplicate install introduces no recursion or lost chain");
    require(!first.install(nativePoll,otherPoll,publish),"holder rejects a different replacement after success");
}

void failedPublication() {
    fc::HookPublication<Poll> holder;
    inner=&holder;
    std::atomic<Poll> slot{nativePoll};
    unsigned publishes{};
    auto replace=[&](Poll expected,Poll replacement) {
        ++publishes;return slot.compare_exchange_strong(expected,replacement);
    };
    require(!holder.install(nullptr,innerPoll,replace)&&!holder.install(nativePoll,nullptr,replace)&&
        !holder.install(innerPoll,innerPoll,replace),"null and self-recursive installations are rejected");
    require(publishes==0&&holder.get()==nullptr&&slot.load()==nativePoll,"invalid targets leave the slot and holder untouched");
    require(!holder.install(nativePoll,innerPoll,[&](Poll expected,Poll replacement) {
        require(holder.get()==nativePoll,"predecessor is prepared before publisher runs");
        slot.store(otherPoll);return replace(expected,replacement);
    }),"competing hook causes publication to fail");
    require(slot.load()==otherPoll&&holder.get()==nullptr,"failed CAS preserves competing hook and clears unpublished predecessor");
    require(holder.install(otherPoll,innerPoll,replace),"explicit retry can chain the newly observed predecessor");
    Device device;slot.load()(&device,.375f);
    require(device.calls==1&&device.elapsed==.375f&&device.order==std::vector<unsigned>{1,3},"retry preserves the competing hook");
}

void booleanReturn() {
    fc::HookPublication<Filter> holder;filter=&holder;
    Filter slot=nativeFilter;Device device;
    require(holder.install(nativeFilter,filterHook,[&](Filter expected,Filter replacement) {
        require(slot==expected,"filter original target");slot=replacement;
        require(slot(&device,7)&&!slot(&device,8),"bool result and arguments survive publication-time callbacks");return true;
    }),"boolean hook installs");
    require(device.calls==2,"each boolean callback calls its predecessor exactly once");
}

void simultaneousInstall() {
    fc::HookPublication<Poll> holder;
    std::atomic<Poll> slot{nativePoll};
    std::atomic<unsigned> publishes{},successes{};
    std::array<std::thread,8> installers;
    for(auto& installer:installers)installer=std::thread([&] {
        if(holder.install(nativePoll,innerPoll,[&](Poll expected,Poll replacement) {
            ++publishes;std::this_thread::yield();return slot.compare_exchange_strong(expected,replacement);
        }))++successes;
    });
    for(auto& installer:installers)installer.join();
    require(publishes==1&&successes==installers.size()&&holder.get()==nativePoll&&slot.load()==innerPoll,
        "simultaneous installers publish once and preserve the original");
}
}

int main() {
    try {
        delayedOriginalControl();publicationBoundary();failedPublication();booleanReturn();simultaneousInstall();
        std::cout<<"PASS hook publication: immediate callbacks, competing hooks, idempotent chains, signatures and concurrent installers\n";
        return 0;
    } catch(const std::exception& error) {std::cerr<<"FAIL hook publication: "<<error.what()<<'\n';return 1;}
}
