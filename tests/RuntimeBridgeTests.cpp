#include "PCH.h"
#include "RuntimeSupport.h"
#include <iostream>
#include <stdexcept>

namespace {
std::size_t checks{};
void require(bool value,const char* message) {
    ++checks;
    if(!value)throw std::runtime_error(message);
}
template<class T,std::size_t Size> struct Storage {
    alignas(T) std::array<std::byte,Size> bytes{};
    T* object() {return reinterpret_cast<T*>(bytes.data());}
    std::uintptr_t address() const {return reinterpret_cast<std::uintptr_t>(bytes.data());}
    template<class V> void put(std::size_t offset,V value) {
        require(offset+sizeof(V)<=Size,"fixture write bounds");
        std::memcpy(bytes.data()+offset,&value,sizeof(V));
    }
    template<class V> V get(std::size_t offset) const {
        require(offset+sizeof(V)<=Size,"fixture read bounds");
        V value{};
        std::memcpy(&value,bytes.data()+offset,sizeof(V));
        return value;
    }
};
template<class T> std::uintptr_t address(T* pointer) {return reinterpret_cast<std::uintptr_t>(pointer);}

void runtimeCase(REL::Version version) {
    require(REL::Module::mock(version),"CommonLib mock initialization");
    require(REL::Module::get().version()==version,"real Module version selection");
    require(fc::runtime::supported(),"bridge runtime must be supported");
    const bool modern=version>=REL::Version(1,7,99,0);
    const bool shifted=version>=REL::Version(1,6,629,0);
    require(REL::Module::IsAE()==(version[1]>=6),"real SE/AE runtime family");
    require(fc::runtime::isSE()==(version==REL::Version(1,5,97,0)),"SE-only native helper gate");

    Storage<RE::BSInputEventQueue,0x600> queue;
    alignas(void*) std::array<std::byte,32> events{};
    auto* oldHead=reinterpret_cast<RE::InputEvent*>(events.data());
    auto* oldTail=reinterpret_cast<RE::InputEvent*>(events.data()+8);
    auto* newHead=reinterpret_cast<RE::InputEvent*>(events.data()+16);
    auto* newTail=reinterpret_cast<RE::InputEvent*>(events.data()+24);
    queue.put(0x380,oldHead);
    queue.put(0x388,oldTail);
    queue.put(0x558,newHead);
    queue.put(0x560,newTail);
    auto* q=queue.object();
    const auto headOffset=modern?0x558u:0x380u;
    const auto tailOffset=modern?0x560u:0x388u;
    require(address(&q->GetQueueHead())==queue.address()+headOffset,"actual queue head reference offset");
    require(address(&q->GetQueueTail())==queue.address()+tailOffset,"actual queue tail reference offset");
    require(q->GetQueueHead()==(modern?newHead:oldHead),"queue reads selected head sentinel");
    require(q->GetQueueTail()==(modern?newTail:oldTail),"queue reads selected tail sentinel");
    require(address(&q->GetRuntimeData())==queue.address()+(modern?0x28:0x20),"actual input event data offset");
    const auto* constQueue=q;
    require(address(&constQueue->GetRuntimeData())==address(&q->GetRuntimeData()),"const input data accessor");
    require(address(q->GetAe1799EventData())==(modern?queue.address()+0x388:0),"optional 1.7.99 event data");
    q->GetQueueHead()=nullptr;
    q->GetQueueTail()=nullptr;
    require(queue.get<RE::InputEvent*>(headOffset)==nullptr&&queue.get<RE::InputEvent*>(tailOffset)==nullptr,
        "queue writes selected head and tail");
    require(queue.get<RE::InputEvent*>(modern?0x380:0x558)==(modern?oldHead:newHead)&&
        queue.get<RE::InputEvent*>(modern?0x388:0x560)==(modern?oldTail:newTail),"queue preserves opposite ABI sentinels");

    Storage<RE::Actor,0x500> actor;
    auto* a=actor.object();
    const auto stateOffset=shifted?0xC0u:0xB8u;
    const auto ownerOffset=shifted?0xB8u:0xB0u;
    require(address(a->AsActorState())==actor.address()+stateOffset,"actual ActorState offset");
    require(address(a->AsActorValueOwner())==actor.address()+ownerOffset,"actual ActorValueOwner offset");
    require(address(&a->GetActorRuntimeData())==actor.address()+(shifted?0xE8:0xE0),"actual Actor runtime data offset");
    const auto* constActor=a;
    require(address(constActor->AsActorState())==actor.address()+stateOffset&&
        address(constActor->AsActorValueOwner())==actor.address()+ownerOffset,"const Actor subobject accessors");
    actor.put(stateOffset+8,std::uint32_t{1u<<10});
    require(a->AsActorState()->IsSwimming()&&!a->AsActorState()->IsSprinting(),"typed ActorState reads selected flags");
    a->AsActorState()->actorState1.swimming=0;
    a->AsActorState()->actorState1.sprinting=1;
    require(actor.get<std::uint32_t>(stateOffset+8)==(1u<<8),"typed ActorState writes selected flags");

    using Entry=RE::BSFlattenedBoneTree::BoneEntry;
    Storage<RE::BSFlattenedBoneTree,0x200> tree;
    Storage<Entry,0x100> entries;
    auto* entry=entries.object();
    tree.put(0x128,std::uint32_t{2});
    tree.put(0x12C,std::uint32_t{1});
    tree.put(0x130,entry);
    entries.put(0x68,std::int16_t{-1});
    entries.put(0x80+0x68,std::int16_t{0});
    entries.put(0x24,3.5f);
    entries.put(0x34+0x24,17.25f);
    auto& cache=tree.object()->GetRuntimeData();
    require(address(&cache)==tree.address()+0x128,"actual flattened tree cache accessor");
    require(cache.numBones==2&&cache.numPopulatedBones==1&&cache.boneEntries==entry,"typed flattened cache fields");
    require(cache.boneEntries[0].parentIndex==-1&&cache.boneEntries[1].parentIndex==0,"typed flattened parent links");
    require(cache.boneEntries[0].local.translate.x==3.5f&&cache.boneEntries[0].world.translate.x==17.25f,
        "typed flattened local and world transforms");
    cache.boneEntries[1].world.translate.z=-8.25f;
    cache.boneEntries[1].world.scale=1.25f;
    require(entries.get<float>(0x80+0x34+0x2C)==-8.25f&&entries.get<float>(0x80+0x34+0x30)==1.25f,
        "typed flattened world writes use actual 0x80 stride");
    const auto* constTree=tree.object();
    require(address(&constTree->GetRuntimeData())==tree.address()+0x128,"const flattened tree accessor");
    std::cout<<version.string()<<" queue=0x"<<std::hex<<headOffset<<"/0x"<<tailOffset
        <<" actor=0x"<<ownerOffset<<"/0x"<<stateOffset<<" flat=0x128"<<std::dec<<" PASS\n";
}

void hookTarget() {}

void memoryGuards() {
    SYSTEM_INFO system{};
    GetSystemInfo(&system);
    const auto page=std::size_t(system.dwPageSize);
    auto* block=static_cast<std::byte*>(VirtualAlloc(nullptr,page*3,MEM_RESERVE|MEM_COMMIT,PAGE_READWRITE));
    require(block!=nullptr,"allocate independent guard fixture");
    struct Release {void* p;~Release(){VirtualFree(p,0,MEM_RELEASE);}} release{block};
    const auto base=address(block);
    DWORD previous{};
    require(fc::runtime::readable(base,page*3),"readable committed pages");
    require(!fc::runtime::readable(0,1)&&!fc::runtime::readable(base,0),"null and zero byte ranges rejected");
    require(!fc::runtime::readable(std::numeric_limits<std::uintptr_t>::max()-1,4),"overflow range rejected");
    require(VirtualProtect(block+page,page,PAGE_READONLY,&previous)!=0,"read-only page setup");
    require(fc::runtime::readable(base+page-8,16),"readable mixed adjacent regions");
    require(VirtualProtect(block+page,page,PAGE_NOACCESS,&previous)!=0,"no-access page setup");
    require(!fc::runtime::readable(base+page-8,16)&&!fc::runtime::readable(base+page,1),"cross-page inaccessible region rejected");
    require(VirtualProtect(block+page,page,PAGE_READWRITE|PAGE_GUARD,&previous)!=0,"guard page setup");
    require(!fc::runtime::readable(base+page,1),"guard page rejected without access");
    require(VirtualProtect(block+page,page,PAGE_READWRITE,&previous)!=0,"restore fixture protection");
    require(fc::runtime::readable(base,page*3),"restored page accepted");
    const auto target=reinterpret_cast<std::uintptr_t>(&hookTarget);
    std::array<std::uintptr_t,2> table{target,base};
    require(fc::runtime::callable(target)&&!fc::runtime::callable(base),"compiled test function versus non-executable data");
    require(fc::runtime::hookSite(address(table.data()),0),"hook preflight accepts executable test function");
    require(!fc::runtime::hookSite(address(table.data()),1)&&!fc::runtime::hookSite(0,0)&&
        !fc::runtime::hookSite(address(table.data()),0x1001),"hook preflight rejects data, null and unbounded slot");
    require(VirtualProtect(block+page,page,PAGE_NOACCESS,&previous)!=0,"hook table boundary setup");
    const auto boundary=base+page-sizeof(std::uintptr_t);
    std::memcpy(reinterpret_cast<void*>(boundary),&target,sizeof(target));
    require(fc::runtime::hookSite(boundary,0)&&!fc::runtime::hookSite(boundary,1),"hook preflight validates complete pointer range");
    require(!fc::runtime::executable(target),"external hook target is not implicitly executable game text");
    std::cout<<"self-owned readable / callable / hookSite guard cases PASS\n";
}
}

int main() {
    try {
        constexpr std::array versions{
            REL::Version(1,5,97,0),REL::Version(1,6,317,0),REL::Version(1,6,318,0),REL::Version(1,6,323,0),
            REL::Version(1,6,342,0),REL::Version(1,6,353,0),REL::Version(1,6,629,0),REL::Version(1,6,640,0),
            REL::Version(1,6,659,0),REL::Version(1,6,1130,0),REL::Version(1,6,1170,0),REL::Version(1,6,1179,0),
            REL::Version(1,7,99,0)};
        for(const auto version:versions)runtimeCase(version);
        memoryGuards();
        REL::Module::reset();
        std::cout<<"PASS runtime bridge: "<<versions.size()<<" real CommonLib runtime branches, "<<checks<<" checks\n";
        return 0;
    } catch(const std::exception& error) {
        std::cerr<<"FAIL runtime bridge after "<<checks<<" checks: "<<error.what()<<'\n';
        return 1;
    }
}
