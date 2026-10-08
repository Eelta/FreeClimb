#include "PCH.h"
#include "RuntimeSupport.h"
#include "AnimationSkeletonLayout.h"
#include "FlatSkeleton.h"
#include "CanonicalSkeleton.h"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

#ifndef ENABLE_COMMONLIBSSE_TESTING
#error Runtime bridge tests require CommonLib testing error handling
#endif

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

RE::BSPCGamepadDeviceHandler* polledGamepadHandler{};
float polledGamepadDelta{};
unsigned polledGamepadCalls{};
void gamepadPollTarget(RE::BSPCGamepadDeviceHandler* handler,float elapsed) {
    polledGamepadHandler=handler;polledGamepadDelta=elapsed;++polledGamepadCalls;
}
__declspec(noinline) void dispatchGamepadPoll(RE::BSPCGamepadDeviceHandler* handler,float elapsed) {
    handler->Poll(elapsed);
}
void gamepadBridgeCase() {
    Storage<RE::BSPCGamepadDeviceHandler,0x20> handlerStorage;
    Storage<RE::BSWin32GamepadDevice,0x140> deviceStorage;
    auto* handler=handlerStorage.object();
    auto* device=deviceStorage.object();
    auto* delegate=static_cast<RE::BSPCGamepadDeviceDelegate*>(device);
    std::array<std::uintptr_t,9> table{};
    table[2]=reinterpret_cast<std::uintptr_t>(&gamepadPollTarget);
    handlerStorage.put(0,table.data());
    handlerStorage.put(8,delegate);
    require(address(&handler->GetRuntimeData())==handlerStorage.address()+8,"typed PC gamepad handler delegate accessor");
    require(handler->GetRuntimeData().currentPCGamePadDelegate==delegate,"handler exposes its current delegate");
    const auto* constHandler=handler;
    require(address(&constHandler->GetRuntimeData())==handlerStorage.address()+8,"const handler delegate accessor");
    require(fc::runtime::hookSite(address(table.data()),2),"gamepad Poll slot two is a callable hook target");
    polledGamepadHandler=nullptr;polledGamepadDelta=0;polledGamepadCalls=0;
    dispatchGamepadPoll(handler,.125f);
    require(polledGamepadHandler==handler&&polledGamepadDelta==.125f&&polledGamepadCalls==1,
        "typed handler Poll dispatches slot two with the actual this pointer and float delta");
    handler->GetRuntimeData().currentPCGamePadDelegate=nullptr;
    require(handlerStorage.get<RE::BSPCGamepadDeviceDelegate*>(8)==nullptr,
        "handler accessor observes delegate removal after a disconnected poll");

    auto* baseDevice=static_cast<RE::BSGamepadDevice*>(device);
    require(address(&baseDevice->GetRuntimeData())==deviceStorage.address()+0xC8,"typed gamepad connection data accessor");
    deviceStorage.put(0xC8,std::int32_t{2});deviceStorage.put(0xCC,true);deviceStorage.put(0xCD,true);
    require(baseDevice->GetRuntimeData().userIndex==2&&baseDevice->GetRuntimeData().connected&&
        baseDevice->GetRuntimeData().listeningForInput,"typed connection and user index sentinels");
    deviceStorage.put(0xD0,handler);
    require(address(&delegate->GetRuntimeData())==deviceStorage.address()+0xD0&&
        delegate->GetRuntimeData().gamepadDeviceHandler==handler,"typed delegate handler backlink");
    const auto* constDelegate=delegate;
    require(address(&constDelegate->GetRuntimeData())==deviceStorage.address()+0xD0,"const delegate backlink accessor");
    auto& runtime=device->GetRuntimeData();
    const auto* constDevice=device;
    require(address(&runtime)==deviceStorage.address()+0xD8&&
        address(&constDevice->GetRuntimeData())==address(&runtime),"actual Win32 gamepad runtime data accessor");
    require(address(&runtime.previousState)==deviceStorage.address()+0xD8&&
        address(&runtime.currentState)==deviceStorage.address()+0x100,"current and previous XInput snapshots use separate offsets");
    deviceStorage.put(0xD8,std::uint32_t{41});deviceStorage.put(0xDC,std::uint16_t{0x1000});
    deviceStorage.put(0x100,std::uint32_t{42});deviceStorage.put(0x104,std::uint16_t{0x8301});
    deviceStorage.put(0x106,std::uint8_t{85});deviceStorage.put(0x107,std::uint8_t{170});
    deviceStorage.put(0x108,std::int16_t{-12345});deviceStorage.put(0x10A,std::int16_t{23456});
    deviceStorage.put(0x10C,std::int16_t{1234});deviceStorage.put(0x10E,std::int16_t{-4321});
    require(runtime.currentState.packetNumber==42&&runtime.previousState.packetNumber==41,
        "typed XInput packet numbers distinguish fresh and prior samples");
    const auto& physical=runtime.currentState.gamepad;
    require(physical.buttons==0x8301&&physical.leftTrigger==85&&physical.rightTrigger==170,
        "typed complete snapshot retains button and trigger state without requiring queued events");
    require(physical.thumbLX==-12345&&physical.thumbLY==23456&&physical.thumbRX==1234&&physical.thumbRY==-4321,
        "typed complete snapshot retains both signed thumbstick axes");
    const auto currentButtons=device->GetCurrentButtonState(),previousButtons=device->GetPreviousButtonState();
    require(currentButtons.up&&currentButtons.leftShoulder&&currentButtons.rightShoulder&&currentButtons.y&&!currentButtons.a&&
        previousButtons.a&&!previousButtons.y,"typed button helpers preserve XInput mask layout and snapshot choice");
    struct FloatField {float* field;std::size_t offset;float sentinel;};
    const std::array<FloatField,12> fields{{
        {&runtime.previousLT,0xE8,.125f},{&runtime.previousRT,0xEC,.25f},
        {&runtime.previousLX,0xF0,-.375f},{&runtime.previousLY,0xF4,.5f},
        {&runtime.previousRX,0xF8,.625f},{&runtime.previousRY,0xFC,-.75f},
        {&runtime.currentLT,0x110,.875f},{&runtime.currentRT,0x114,.75f},
        {&runtime.currentLX,0x118,-.625f},{&runtime.currentLY,0x11C,.375f},
        {&runtime.currentRX,0x120,.25f},{&runtime.currentRY,0x124,-.125f}
    }};
    for(const auto& field:fields) {
        require(address(field.field)==deviceStorage.address()+field.offset,"typed normalized gamepad field offset");
        deviceStorage.put(field.offset,field.sentinel);
        require(*field.field==field.sentinel,"typed normalized gamepad field reads its own sentinel");
    }
    runtime.currentState.gamepad.leftTrigger=0;runtime.currentLT=0;runtime.currentLX=0;runtime.currentLY=0;
    require(deviceStorage.get<std::uint8_t>(0x106)==0&&deviceStorage.get<float>(0x110)==0&&
        deviceStorage.get<float>(0x118)==0&&deviceStorage.get<float>(0x11C)==0,"typed left input writes use current-state storage");
    require(runtime.currentRT==.75f&&runtime.currentRX==.25f&&runtime.currentRY==-.125f&&
        runtime.previousLT==.125f&&runtime.previousLX==-.375f,"left input access leaves right camera and prior sample intact");
    baseDevice->GetRuntimeData().connected=false;baseDevice->GetRuntimeData().userIndex=-1;
    require(!deviceStorage.get<bool>(0xCC)&&deviceStorage.get<std::int32_t>(0xC8)==-1,
        "typed disconnect state can be distinguished before using a stale snapshot");

    Storage<RE::ButtonEvent,0x40> buttonStorage;
    auto* button=buttonStorage.object();
    buttonStorage.put(8,RE::INPUT_DEVICE::kGamepad);buttonStorage.put(0xC,RE::INPUT_EVENT_TYPE::kButton);
    buttonStorage.put(0x20,std::uint32_t{RE::BSWin32GamepadDevice::Key::kY});
    buttonStorage.put(0x28,1.f);buttonStorage.put(0x2C,0.f);
    require(button->GetDevice()==RE::INPUT_DEVICE::kGamepad&&button->GetEventType()==RE::INPUT_EVENT_TYPE::kButton,
        "gamepad button event device and type retain native offsets");
    require(address(&button->GetRuntimeData())==buttonStorage.address()+0x28&&
        button->GetIDCode()==RE::BSWin32GamepadDevice::Key::kY,"typed button payload and XInput button ID");
    const auto* constButton=button;
    require(address(&constButton->GetRuntimeData())==buttonStorage.address()+0x28&&
        constButton->Value()==1&&constButton->HeldDuration()==0&&constButton->IsDown()&&!constButton->IsUp(),
        "const button access identifies an actual press edge");
    button->GetRuntimeData().heldDownSecs=.25f;
    require(button->IsHeld()&&!button->IsDown()&&!button->IsUp(),"held native gamepad events do not invent a new down edge");
    button->GetRuntimeData().value=0;
    require(button->IsUp()&&!button->IsHeld()&&!button->IsDown(),"native release retains its matched held-duration semantics");
    button->SetIDCode(RE::BSWin32GamepadDevice::Key::kLeftTrigger);button->GetRuntimeData().value=.625f;
    require(buttonStorage.get<std::uint32_t>(0x20)==9&&button->Value()==.625f&&button->IsHeld(),
        "analog trigger button IDs and float values use typed event fields");

    Storage<RE::ThumbstickEvent,0x40> stickStorage;
    auto* stick=stickStorage.object();
    stickStorage.put(8,RE::INPUT_DEVICE::kGamepad);stickStorage.put(0xC,RE::INPUT_EVENT_TYPE::kThumbstick);
    stickStorage.put(0x20,std::uint32_t{RE::ThumbstickEvent::InputType::kLeftThumbstick});
    stickStorage.put(0x28,-.75f);stickStorage.put(0x2C,.5f);
    require(stick->GetDevice()==RE::INPUT_DEVICE::kGamepad&&stick->GetEventType()==RE::INPUT_EVENT_TYPE::kThumbstick&&
        stick->GetIDCode()==11,"left thumbstick native event has its own device type and ID");
    require(stick->xValue==-.75f&&stick->yValue==.5f&&address(&stick->xValue)==stickStorage.address()+0x28&&
        address(&stick->yValue)==stickStorage.address()+0x2C,"native thumbstick coordinates use the verified event layout");
    stick->idCode=RE::ThumbstickEvent::InputType::kRightThumbstick;
    require(stickStorage.get<std::uint32_t>(0x20)==12&&stick->GetIDCode()!=RE::ThumbstickEvent::InputType::kLeftThumbstick,
        "right camera thumbstick is distinguishable from traversal movement");
}

void skseRuntimeEncodingCases() {
    struct Case {std::uint32_t encoded;REL::Version reported,game;std::string_view database;};
    const std::array cases{
        Case{0x01062931u,REL::Version(1,6,659,1),REL::Version(1,6,659,0),"versionlib-1-6-659-0.bin"},
        Case{0x010649B1u,REL::Version(1,6,1179,1),REL::Version(1,6,1179,0),"versionlib-1-6-1179-0.bin"},
        Case{0x01064920u,REL::Version(1,6,1170,0),REL::Version(1,6,1170,0),"versionlib-1-6-1170-0.bin"}};
    for(const auto& test:cases){
        SKSE::Impl::SKSEInterface native{};native.runtimeVersion=test.encoded;native.skseVersion=fc::runtime::pack(2,2,6);
        const auto* loader=reinterpret_cast<const SKSE::LoadInterface*>(&native);
        const auto reported=loader->RuntimeVersion();
        require(reported==test.reported&&reported.pack()==test.encoded,"real LoadInterface preserves the official packed platform nibble");
        require(fc::runtime::supportedSKSE(reported.pack()),"real loader interface value passes exact SKSE runtime policy");
        const auto game=REL::Version::unpack(fc::runtime::gameVersionFromSKSE(reported.pack()));
        require(game==test.game&&game[3]==0,"loader platform encoding resolves to executable file version");
        require(fc::runtime::skseVersion(game.pack())==native.runtimeVersion,"loader and executable representations round-trip exactly");
        require(REL::Module::mock(test.game)&&REL::Module::get().version()==game,"CommonLib executable version agrees with normalized loader version");
        require(REL::Module::IsAE()&&fc::runtime::addressFormat(game.pack())==2,"GOG and Steam 1.6 select the AE database format");
        require("versionlib-"+game.string("-")+".bin"==test.database,"address-library filename uses executable suffix zero instead of GOG platform one");
        require(loader->RuntimeVersion().pack()==test.encoded,"normalization does not mutate the native SKSE interface");
    }
    for(const auto raw:{0x01062930u,0x010649B0u,0x010649B2u,0x010649BFu,0x01064921u,
        fc::runtime::pack(1,4,15),fc::runtime::pack(1,7,105,1),fc::runtime::pack(1,6,1180,1)}){
        SKSE::Impl::SKSEInterface native{};native.runtimeVersion=raw;
        const auto* loader=reinterpret_cast<const SKSE::LoadInterface*>(&native);
        require(loader->RuntimeVersion().pack()==raw,"unsupported platform encodings are preserved by the real loader accessor");
        require(!fc::runtime::supportedSKSE(loader->RuntimeVersion().pack()),"GOG file spelling, unknown platform, VR and future loader identifiers are rejected");
    }
    std::cout<<"SKSE LoadInterface GOG platform decoding and executable Address Library filenames PASS\n";
}

void animationSkeletonBridgeCase() {
    static_assert(sizeof(RE::hkaSkeleton)==0x78&&sizeof(RE::hkaBone)==0x10&&sizeof(RE::hkQsTransform)==0x30);
    Storage<RE::hkaSkeleton,0x78> storage;
    Storage<RE::hkaBone,116*sizeof(RE::hkaBone)> bones;
    std::array<std::int16_t,126> parents{};
    Storage<RE::hkQsTransform,126*sizeof(RE::hkQsTransform)> references;
    storage.put(0x18,parents.data());storage.put(0x20,std::int32_t{126});storage.put(0x24,std::uint32_t{0x8000007e});
    storage.put(0x28,bones.object());storage.put(0x30,std::int32_t{116});storage.put(0x34,std::uint32_t{0x80000074});
    storage.put(0x38,references.object());storage.put(0x40,std::int32_t{126});storage.put(0x44,std::uint32_t{0x8000007e});
    const auto* skeleton=storage.object();const auto captured=fc::animationSkeletonLayout(*skeleton);
    require(address(&skeleton->parentIndices)==storage.address()+0x18&&address(&skeleton->bones)==storage.address()+0x28&&
        address(&skeleton->referencePose)==storage.address()+0x38,"typed hkaSkeleton arrays retain their verified Havok ABI offsets");
    require(captured.bones==bones.object()&&captured.parents==parents.data()&&captured.references==references.object(),
        "the production layout adapter reads all three actual typed array pointers");
    require(captured.boneCount==116&&captured.parentCount==126&&captured.referenceCount==126,
        "the production layout adapter preserves independent array lengths instead of capacity flags");
    require(captured.valid(fc::runtime::readable),"real typed 116/126/126 storage passes native readability validation");
    for(std::size_t sizeOffset:{0x20u,0x30u,0x40u}) {
        const auto original=storage.get<std::int32_t>(sizeOffset);storage.put(sizeOffset,std::int32_t{-1});
        const auto invalid=fc::animationSkeletonLayout(*skeleton);unsigned probes=0;
        require(!invalid.valid([&](std::uintptr_t,std::size_t){++probes;return true;})&&probes==0,
            "negative native hkArray lengths are rejected before unsigned byte counts or memory probes");
        require(invalid!=captured,"a changed native array length invalidates the captured storage identity");
        storage.put(sizeOffset,original);
    }
    for(std::size_t pointerOffset:{0x18u,0x28u,0x38u}) {
        const auto original=storage.get<std::uintptr_t>(pointerOffset);storage.put(pointerOffset,std::uintptr_t{});
        const auto invalid=fc::animationSkeletonLayout(*skeleton);
        require(invalid!=captured&&!invalid.valid(fc::runtime::readable),"typed null array replacement invalidates the captured layout");
        storage.put(pointerOffset,original);
    }
    require(fc::animationSkeletonLayout(*skeleton)==captured,"restoring native array headers restores the exact captured identity");
}

void animationSkeletonMemoryGuards() {
    SYSTEM_INFO system{};GetSystemInfo(&system);const auto page=std::size_t(system.dwPageSize);
    require(page*2>=116*sizeof(RE::hkQsTransform),"guard fixture has enough readable prefix pages for actual Havok poses");
    auto* block=static_cast<std::byte*>(VirtualAlloc(nullptr,page*9,MEM_RESERVE|MEM_COMMIT,PAGE_READWRITE));
    require(block!=nullptr,"allocate isolated native skeleton guard pages");
    struct Release {void* p;~Release(){VirtualFree(p,0,MEM_RELEASE);}} release{block};
    auto* bones=reinterpret_cast<RE::hkaBone*>(block+2*page-116*sizeof(RE::hkaBone));
    auto* parents=reinterpret_cast<std::int16_t*>(block+5*page-116*sizeof(std::int16_t));
    auto* references=reinterpret_cast<RE::hkQsTransform*>(block+8*page-116*sizeof(RE::hkQsTransform));
    DWORD previous{};
    for(std::size_t guard:{2u,5u,8u})require(VirtualProtect(block+guard*page,page,PAGE_NOACCESS,&previous)!=0,"protect each skeleton array boundary independently");
    fc::AnimationSkeletonLayout<RE::hkaBone,RE::hkQsTransform> layout{bones,parents,references,116,116,116};
    require(layout.valid(fc::runtime::readable),"exact declared ranges ending at inaccessible pages remain readable");
    auto changed=layout;changed.parentCount=126;
    require(!changed.valid(fc::runtime::readable),"declared trailing parent elements in an inaccessible page are rejected");
    changed=layout;changed.referenceCount=126;
    require(!changed.valid(fc::runtime::readable),"declared trailing reference poses in an inaccessible page are rejected");
    changed=layout;changed.bones=bones+1;require(!changed.valid(fc::runtime::readable),"an inaccessible active bone suffix is rejected");
    changed=layout;changed.parents=parents+1;require(!changed.valid(fc::runtime::readable),"an inaccessible active parent suffix is rejected");
    changed=layout;changed.references=references+1;require(!changed.valid(fc::runtime::readable),"an inaccessible active reference suffix is rejected");
    for(std::size_t guard:{5u,8u})require(VirtualProtect(block+guard*page,page,PAGE_READWRITE,&previous)!=0,"restore readable companion array tails");
    for(int i=116;i<126;++i)parents[i]=std::numeric_limits<std::int16_t>::max();
    std::memset(references+116,0xff,10*sizeof(RE::hkQsTransform));
    layout.parentCount=126;layout.referenceCount=126;
    require(layout.valid(fc::runtime::readable),"readable trailing storage contents do not become additional named bones or pose validation inputs");
    require(VirtualProtect(block+8*page,page,PAGE_READWRITE|PAGE_GUARD,&previous)!=0,"guard a declared reference tail without reading it");
    require(!layout.valid(fc::runtime::readable),"PAGE_GUARD in a declared tail is rejected without causing a guarded memory access");
    MEMORY_BASIC_INFORMATION info{};
    require(VirtualQuery(block+8*page,&info,sizeof(info))&&(info.Protect&PAGE_GUARD),"readability validation does not consume a trailing page guard by dereferencing it");
    std::cout<<"animation skeleton declared ranges, asymmetric lengths and native guard pages PASS\n";
}

void runtimeCase(REL::Version version) {
    require(REL::Module::mock(version),"CommonLib mock initialization");
    require(REL::Module::get().version()==version,"real Module version selection");
    require(fc::runtime::supported(),"bridge runtime must be supported");
    gamepadBridgeCase();
    animationSkeletonBridgeCase();
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
        <<" actor=0x"<<ownerOffset<<"/0x"<<stateOffset<<" flat=0x128 gamepad=0xD8 poll=2"<<std::dec<<" PASS\n";
}

struct DatabaseFixtures {
    std::filesystem::path directory;
    std::vector<std::filesystem::path> files;
    DatabaseFixtures() {
        const auto project=std::filesystem::path(__FILE__).parent_path().parent_path();
        require(project.is_absolute(),"fixture source path must identify the project");
        const auto parent=project/"diagnostics"/"runtime-17104";
        std::filesystem::create_directories(parent);
        directory=parent/("format5-"+std::to_string(GetCurrentProcessId()));
        require(std::filesystem::create_directory(directory),"fixture directory must be newly owned");
    }
    ~DatabaseFixtures() {
        REL::IDDB::reset();
        std::error_code error;
        for(const auto& path:files)std::filesystem::remove(path,error);
        std::filesystem::remove(directory,error);
    }
    std::filesystem::path write(const std::string& name,std::span<const std::uint32_t> words) {
        const auto path=directory/name;
        files.push_back(path);
        std::ofstream stream(path,std::ios::binary|std::ios::trunc);
        stream.write(reinterpret_cast<const char*>(words.data()),std::streamsize(words.size_bytes()));
        stream.close();
        require(bool(stream),"write complete address library fixture");
        return path;
    }
};

void addressLibraryCases() {
    DatabaseFixtures fixtures;
    for(const auto version:{REL::Version(1,7,99,0),REL::Version(1,7,104,0)}) {
        constexpr std::uintptr_t base=0x140000000;
        require(REL::Module::mock(version,REL::Module::Runtime::AE,L"SkyrimSE.exe",base),
            "format-5 runtime mock initialization");
        std::array<std::uint32_t,32> words{};
        words[0]=5;
        for(std::size_t i=0;i<4;++i)words[1+i]=version[i];
        std::memcpy(words.data()+5,"SkyrimSE.exe",sizeof("SkyrimSE.exe"));
        words[21]=8;
        words[23]=8;
        words[24]=0x1000+version[2];
        words[25]=0x2000+version[2];
        words[27]=0x4000+version[2];
        words[31]=0xF0000000+version[2];
        const auto suffix=version.string()+".bin";
        const auto complete=fixtures.write("versionlib-"+suffix,words).wstring();
        require(REL::IDDB::inject(complete,REL::IDDB::Format::SSEv5,version),
            "explicit format-5 database injection");
        require(REL::IDDB::get().id2offset(1)==words[25],"explicit format-5 dense lookup");
        require(REL::IDDB::inject(complete,version),"production format autodetection accepts format 5");
        for(const auto id:{0u,1u,3u,7u})
            require(REL::IDDB::get().id2offset(id)==words[24+id],"dense lookup preserves first, middle and final offsets");
        const REL::ID last(7);
        require(last.offset()==words[31]&&last.address()==base+words[31],
            "REL ID preserves unsigned offsets and adds the selected module base");
        const auto rejectedLookup=[](std::uint64_t id) {
            bool rejected=false;
            try {(void)REL::IDDB::get().id2offset(id);}
            catch(const std::runtime_error&) {rejected=true;}
            require(rejected,"missing or out-of-bounds ID must throw through the test interface");
        };
        for(const auto id:{std::uint64_t{2},std::uint64_t{8},std::numeric_limits<std::uint64_t>::max()})
            rejectedLookup(id);
        const auto other=version==REL::Version(1,7,99,0)?REL::Version(1,7,104,0):REL::Version(1,7,99,0);
        require(!REL::IDDB::inject(complete,other),"format-5 runtime version mismatch is rejected");
        rejectedLookup(1);
        const auto truncated=fixtures.write("truncated-"+suffix,std::span(words).first(words.size()-1)).wstring();
        require(!REL::IDDB::inject(truncated,version),"truncated dense offset array is rejected");
        rejectedLookup(1);
        words[0]=4;
        const auto invalid=fixtures.write("invalid-format-"+suffix,words).wstring();
        require(!REL::IDDB::inject(invalid,version),"unknown address database format is rejected");
        rejectedLookup(1);
        require(REL::IDDB::inject(complete,version)&&REL::IDDB::get().id2offset(7)==words[31],
            "valid format-5 database can load after rejected injections");
        REL::IDDB::reset();
        std::cout<<version.string()<<" format-5 file mapping, autodetection, dense IDs and rejection cases PASS\n";
    }
}

void hookTarget() {}

using GamepadPoll=void(*)(RE::BSPCGamepadDeviceHandler*,float);
fc::HookPublication<GamepadPoll> publishedGamepadPoll,publishedOuterGamepadPoll;
unsigned publishedGamepadCalls{},publishedOuterGamepadCalls{};
void gamepadPublishedTarget(RE::BSPCGamepadDeviceHandler* handler,float elapsed) {
    ++publishedGamepadCalls;
    const auto original=publishedGamepadPoll.get();
    require(original!=nullptr,"published gamepad original exists");original(handler,elapsed);
}
void gamepadPublishedOuter(RE::BSPCGamepadDeviceHandler* handler,float elapsed) {
    ++publishedOuterGamepadCalls;
    const auto original=publishedOuterGamepadPoll.get();
    require(original!=nullptr,"published outer gamepad original exists");original(handler,elapsed);
}

void virtualHookPublication() {
    SYSTEM_INFO system{};GetSystemInfo(&system);
    auto* block=static_cast<std::uintptr_t*>(VirtualAlloc(nullptr,system.dwPageSize,MEM_RESERVE|MEM_COMMIT,PAGE_READWRITE));
    require(block!=nullptr,"allocate independent virtual table fixture");
    struct Release {void* p;~Release(){VirtualFree(p,0,MEM_RELEASE);}} release{block};
    block[2]=address(&gamepadPollTarget);block[4]=address(&gamepadPublishedTarget);block[5]=address(block);
    DWORD previous{};
    require(VirtualProtect(block,system.dwPageSize,PAGE_READONLY,&previous)!=0,"protect fixture virtual table as read-only");
    fc::HookPublication<GamepadPoll> rejected;
    require(!fc::runtime::installVfunc(0,2,&gamepadPublishedTarget,rejected)&&
        !fc::runtime::installVfunc(address(block)+1,2,&gamepadPublishedTarget,rejected)&&
        !fc::runtime::installVfunc(address(block),0x1001,&gamepadPublishedTarget,rejected),
        "virtual installer rejects null, misaligned and unbounded slots");
    require(!fc::runtime::installVfunc(address(block),0,&gamepadPublishedTarget,rejected)&&
        !fc::runtime::installVfunc(address(block),5,&gamepadPublishedTarget,rejected)&&
        !fc::runtime::installVfunc(address(block),4,&gamepadPublishedTarget,rejected)&&
        !fc::runtime::installVfunc(address(block),2,GamepadPoll{},rejected),
        "virtual installer rejects null, non-executable, self and empty replacement targets");
    require(rejected.get()==nullptr&&block[2]==address(&gamepadPollTarget),"rejected installation leaves original target intact");
    require(fc::runtime::installVfunc(address(block),2,&gamepadPublishedTarget,publishedGamepadPoll),
        "virtual installer publishes into a real read-only pointer slot");
    MEMORY_BASIC_INFORMATION info{};
    require(VirtualQuery(block,&info,sizeof(info))&&info.Protect==PAGE_READONLY,
        "virtual installer restores the original page protection");
    Storage<RE::BSPCGamepadDeviceHandler,0x20> handlerStorage;handlerStorage.put(0,block);
    auto* handler=handlerStorage.object();
    polledGamepadCalls=publishedGamepadCalls=publishedOuterGamepadCalls=0;
    dispatchGamepadPoll(handler,.375f);
    require(polledGamepadCalls==1&&publishedGamepadCalls==1&&polledGamepadHandler==handler&&polledGamepadDelta==.375f,
        "real virtual dispatch preserves handler pointer, float delta and one original call");
    require(fc::runtime::installVfunc(address(block),2,&gamepadPublishedOuter,publishedOuterGamepadPoll),
        "second virtual hook chains the first");
    require(fc::runtime::installVfunc(address(block),2,&gamepadPublishedTarget,publishedGamepadPoll)&&
        block[2]==address(&gamepadPublishedOuter)&&publishedGamepadPoll.get()==&gamepadPollTarget&&
        publishedOuterGamepadPoll.get()==&gamepadPublishedTarget,"repeated inner install preserves the external outer chain");
    dispatchGamepadPoll(handler,.625f);
    require(polledGamepadCalls==2&&publishedGamepadCalls==2&&publishedOuterGamepadCalls==1&&polledGamepadDelta==.625f,
        "nested real virtual hooks avoid recursion and duplicate native polling");
    require(VirtualQuery(block,&info,sizeof(info))&&info.Protect==PAGE_READONLY,"nested installs retain read-only page protection");
    std::cout<<"real Windows virtual hook publication, restored protections and nested native dispatch PASS\n";
}

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

RE::NiRTTI diagnosticNodeType{"NiNode",nullptr};
RE::NiRTTI diagnosticFlatType{"BSFlattenedBoneTree",&diagnosticNodeType};
const RE::NiRTTI* diagnosticNodeRTTI(const RE::NiObject*) {return &diagnosticNodeType;}
const RE::NiRTTI* diagnosticFlatRTTI(const RE::NiObject*) {return &diagnosticFlatType;}
RE::NiNode* diagnosticAsNode(RE::NiObject* object) {return reinterpret_cast<RE::NiNode*>(object);}
using SceneStorage=Storage<RE::NiNode,0x200>;
void diagnosticChildren(SceneStorage& storage,std::span<RE::NiAVObject*> children) {
    const auto offset=address(&storage.object()->GetChildren())-storage.address();
    storage.put(offset+8,children.data());storage.put(offset+0x10,std::uint16_t(children.size()));
}
struct BinderName {
    RE::BSStringPool::Entry entry{};
    std::array<char,96> text{};
    void assign(std::string_view name) {
        require(name.size()<text.size(),"binder string fixture length");entry._flags=1;entry._length=std::uint32_t(name.size());
        std::copy(name.begin(),name.end(),text.begin());text[name.size()]='\0';
    }
};
static_assert(offsetof(BinderName,text)==sizeof(RE::BSStringPool::Entry));
void flattenedBindingStructures(REL::Version version) {
    const auto runtime=version.minor()==5?REL::Module::Runtime::SE:REL::Module::Runtime::AE;
    require(REL::Module::mock(version,runtime,L"SkyrimSE.exe",0x140000000),"binding fixture mock runtime");
    std::array<std::uintptr_t,4> nodeTable{},flatTable{};nodeTable[2]=address(&diagnosticNodeRTTI);nodeTable[3]=address(&diagnosticAsNode);
    flatTable[2]=address(&diagnosticFlatRTTI);flatTable[3]=address(&diagnosticAsNode);
    std::array<std::string,99> names;std::array<BinderName,99> pooled;
    for(std::size_t i=0;i<names.size();++i){names[i]=fc::canonicalBoneNames[i];pooled[i].assign(names[i]);}
    BinderName actorName,cacheName;actorName.assign("skeleton.nif");cacheName.assign("body-cache");
    SceneStorage actor,root,com,pelvis,nested,outside;
    const auto initialize=[&](SceneStorage& storage,BinderName& name,bool flat=false) {
        storage.put(0,flat?flatTable.data():nodeTable.data());storage.object()->_refCount=1;
        const char* text=name.text.data();std::memcpy(&storage.object()->name,&text,sizeof(text));
        for(int axis=0;axis<3;++axis)storage.object()->local.rotate.entry[axis][axis]=storage.object()->world.rotate.entry[axis][axis]=1;
        storage.object()->local.scale=storage.object()->world.scale=1;
        require(std::string_view(storage.object()->name.c_str())==name.text.data(),"real BSFixedString resolves its string-pool Entry");
    };
    initialize(actor,actorName);initialize(root,pooled[0],true);initialize(com,pooled[4]);initialize(pelvis,pooled[5]);initialize(nested,cacheName,true);initialize(outside,pooled[6]);
    root.object()->parent=actor.object();com.object()->parent=root.object();pelvis.object()->parent=com.object();outside.object()->parent=actor.object();nested.object()->parent=pelvis.object();
    std::array<RE::NiAVObject*,2> actorChildren{root.object(),outside.object()};std::array<RE::NiAVObject*,1> rootChildren{com.object()},comChildren{pelvis.object()},pelvisChildren{nested.object()};
    diagnosticChildren(actor,actorChildren);diagnosticChildren(root,rootChildren);diagnosticChildren(com,comChildren);
    using Entry=RE::BSFlattenedBoneTree::BoneEntry;Storage<Entry,sizeof(Entry)*99> entries;
    for(std::size_t i=0;i<99;++i){auto& entry=entries.object()[i];entry.parentIndex=std::int16_t(fc::canonicalBoneParents[i]);entry.unk6A=-1;entry.nextSiblingIndex=-1;
        for(int axis=0;axis<3;++axis)entry.local.rotate.entry[axis][axis]=entry.world.rotate.entry[axis][axis]=1;entry.local.scale=entry.world.scale=1;
        if(i<1||i>3){const char* text=pooled[i].text.data();std::memcpy(&entry.nodeName,&text,sizeof(text));}
    }
    entries.object()[0].node=root.object();entries.object()[4].node=com.object();entries.object()[5].node=pelvis.object();
    const auto storage=[&](SceneStorage& owner,Entry* bones,std::uint32_t count,std::uint32_t populated) {
        auto& data=reinterpret_cast<RE::BSFlattenedBoneTree*>(owner.object())->GetRuntimeData();data.numBones=count;data.numPopulatedBones=populated;data.boneEntries=bones;
        require(fc::flatEntries(owner.object()).has_value(),"fixture has bounded valid typed flat entries");
    };
    storage(root,entries.object(),99,3);const auto originalEntries=entries.bytes;
    const auto baselineNodes=std::array{actor.bytes,root.bytes,com.bytes,pelvis.bytes,nested.bytes,outside.bytes};
    {
        const auto bound=fc::bindRuntimeScene(root.object(),names,fc::canonicalBoneParents);
        require(bool(bound)&&bound.count==81&&bound.virtualLeaves==3&&bound.unownedTracks==15,"production binder accepts the ordinary selected-root flat skeleton");
        require(bound.nodes[6].flat==&entries.object()[6].local&&!bound.nodes[6].node,"ordinary flat binding selects actual left-thigh storage without creating a node");
        for(std::size_t i=0;i<99;++i)if(fc::engineOwnedTrack(i,names,fc::canonicalBoneParents))require(!bound.nodes[i],"production binding excludes engine-owned camera and attachment tracks");
        std::cout<<"Production binder, mocked runtime "<<version.string()<<", selected-root flat: missing="<<bound.missing<<", mapped="<<bound.count<<", virtual="<<bound.virtualLeaves<<", unowned="<<bound.unownedTracks<<'\n';
    }
    const auto refs=[&]{for(auto* node:{actor.object(),root.object(),com.object(),pelvis.object(),nested.object(),outside.object()})require(node->GetRefCount()==1,"binder releases only its temporary node references");};refs();
    require(std::array{actor.bytes,root.bytes,com.bytes,pelvis.bytes,nested.bytes,outside.bytes}==baselineNodes&&entries.bytes==originalEntries,"successful production lookup does not mutate nodes or flat storage");
    BinderName lowerThigh,lowerRoot;lowerThigh.assign("npc l thigh [lthg]");lowerRoot.assign("npc root [root]");
    const auto assignName=[](RE::BSFixedString& name,const char* text){std::memcpy(&name,&text,sizeof(text));};
    {
        Storage<RE::BSFixedString,sizeof(RE::BSFixedString)> nativeName;nativeName.put(0,lowerThigh.text.data());
        require(*nativeName.object()==std::string_view(names[6])&&std::string_view(nativeName.object()->c_str())!=names[6],"real BSFixedString comparison matches case-only spelling that legacy exact comparison rejects");
        require(fc::sameSceneBoneName(names[6],lowerThigh.text.data())&&!fc::sameSceneBoneName(names[6],"NPC L Thigh [LThg] ")&&
            !fc::sameSceneBoneName(names[6],"NPC R Thigh [LThg]")&&!fc::sameSceneBoneName(names[6],"NPC L Thigh [LThg]x"),"scene comparison folds ASCII case only and retains exact length and spelling");
        require(!fc::sameSceneBoneName(std::string_view("\xC0",1),std::string_view("\xE0",1)),"scene comparison never folds non-ASCII bytes");
        assignName(entries.object()[6].nodeName,lowerThigh.text.data());fc::FlatNameSnapshot mixedNames;
        {
            const auto bound=fc::bindRuntimeScene(root.object(),names,fc::canonicalBoneParents,&mixedNames);
            require(bool(bound)&&bound.nodes[6].flat==&entries.object()[6].local&&mixedNames.caseAliases==1,"case-only flat entry resolves its exact existing storage");
        }
        const auto detail=fc::describeSceneLookup(actor.object(),root.object(),names[6],&mixedNames);
        require(detail.flat.caseOnlyCount==1&&detail.flat.missingIndex==6&&detail.flat.caseOnlyCandidates.find("'npc l thigh [lthg]' len=18 hex=6E7063")!=std::string::npos,"diagnostic retains raw case-only spelling, length and bytes from the existing snapshot");
        assignName(entries.object()[7].nodeName,pooled[6].text.data());
        require(!fc::bindRuntimeScene(root.object(),names,fc::canonicalBoneParents),"flat entries whose names differ only in case but identify different bones are ambiguous");
        assignName(entries.object()[7].nodeName,pooled[7].text.data());
        assignName(entries.object()[6].nodeName,nullptr);
        SceneStorage actualCase,actualDuplicate;initialize(actualCase,lowerThigh);initialize(actualDuplicate,pooled[6]);
        actualCase.object()->parent=pelvis.object();actualDuplicate.object()->parent=pelvis.object();
        std::array<RE::NiAVObject*,1> one{actualCase.object()};diagnosticChildren(pelvis,one);
        {
            fc::FlatNameSnapshot actualNames;const auto bound=fc::bindRuntimeScene(root.object(),names,fc::canonicalBoneParents,&actualNames);
            require(bool(bound)&&bound.nodes[6].node.get()==actualCase.object()&&actualNames.caseAliases==1,"case-only existing physical bone fallback retains that node without materialization");
        }
        const auto actualDetail=fc::describeSceneLookup(actor.object(),root.object(),names[6]);
        require(actualDetail.inside==0&&actualDetail.caseOnlyInside==1&&actualDetail.actualCaseCandidates.find("npc l thigh [lthg]")!=std::string::npos,"physical diagnostic distinguishes exact and case-only matches");
        require(actualDetail.flat.nearbyEntries.find("row=6 ")!=std::string::npos&&actualDetail.flat.nearbyEntries.find("row=13 ")!=std::string::npos&&
            actualDetail.flat.nearbyEntries.find("row=14 ")==std::string::npos&&!actualDetail.flat.relatedNames.empty(),"missing exact storage diagnostics include only bounded neighboring rows and related names");
        require(fc::sceneDiagnosticRawName(std::string(256,'x')).size()<300,"raw diagnostic names and hexadecimal bytes stay bounded");
        std::array<RE::NiAVObject*,2> two{actualCase.object(),actualDuplicate.object()};diagnosticChildren(pelvis,two);
        require(!fc::existingNode(root.object(),names[6])&&!fc::bindRuntimeScene(root.object(),names,fc::canonicalBoneParents),"different physical nodes with native-equivalent names never choose the first match");
        diagnosticChildren(pelvis,std::span<RE::NiAVObject*>{});assignName(entries.object()[6].nodeName,pooled[6].text.data());
        assignName(root.object()->name,lowerRoot.text.data());
        require(fc::existingNode(actor.object(),names[0])==root.object(),"case-only selected Root is found within the actor subtree");
        assignName(outside.object()->name,pooled[0].text.data());
        require(!fc::existingNode(actor.object(),names[0]),"two distinct case-equivalent roots are rejected as ambiguous");
        assignName(outside.object()->name,pooled[6].text.data());assignName(root.object()->name,pooled[0].text.data());
    }
    refs();
    require(std::array{actor.bytes,root.bytes,com.bytes,pelvis.bytes,nested.bytes,outside.bytes}==baselineNodes&&entries.bytes==originalEntries,"case-only fixture restores every native node and flat entry");
    const auto missing=[&](const char* label) {
        const auto actorBefore=actor.bytes,rootBefore=root.bytes,comBefore=com.bytes,pelvisBefore=pelvis.bytes,nestedBefore=nested.bytes,outsideBefore=outside.bytes;
        const auto bonesBefore=entries.bytes;
        {const auto bound=fc::bindRuntimeScene(root.object(),names,fc::canonicalBoneParents);require(!bound&&bound.missing==6&&bound.count==3&&bound.virtualLeaves==3,label);require(!bound.nodes[6],"failure cannot substitute the same-named thigh outside the owned NPC Root");
            std::cout<<"Production binder, mocked runtime "<<version.string()<<", "<<label<<": missing="<<bound.missing<<", mapped="<<bound.count<<", virtual="<<bound.virtualLeaves<<", unowned="<<bound.unownedTracks<<'\n';}
        refs();require(actor.bytes==actorBefore&&root.bytes==rootBefore&&com.bytes==comBefore&&pelvis.bytes==pelvisBefore&&nested.bytes==nestedBefore&&outside.bytes==outsideBefore&&entries.bytes==bonesBefore,"failed production lookup does not mutate nodes or flat storage");
    };
    root.put(0,nodeTable.data());actor.put(0,flatTable.data());storage(actor,entries.object(),99,3);
    missing("production binder reproduces index6 mapped3 when the actual flat storage belongs to an ancestor");
    actor.put(0,nodeTable.data());Storage<Entry,sizeof(Entry)> nestedEntries;auto& thigh=*nestedEntries.object();thigh.parentIndex=-1;thigh.unk6A=-1;thigh.nextSiblingIndex=-1;
    const char* thighName=pooled[6].text.data();std::memcpy(&thigh.nodeName,&thighName,sizeof(thighName));storage(nested,nestedEntries.object(),1,0);diagnosticChildren(pelvis,pelvisChildren);
    const auto nestedBefore=nestedEntries.bytes;
    missing("production binder reproduces index6 mapped3 for a named unmaterialized thigh in a nested flat tree");
    require(nestedEntries.bytes==nestedBefore,"failed lookup leaves nested flat entry unchanged");diagnosticChildren(pelvis,std::span<RE::NiAVObject*>{});
    root.put(0,flatTable.data());const char* empty=nullptr;std::memcpy(&entries.object()[6].nodeName,&empty,sizeof(empty));
    using Map=RE::BSTHashMap<RE::BSFixedString,std::int32_t>;struct MapSlot {const char* name{};std::int32_t index{};std::uint32_t padding{};const void* next{};};
    static_assert(sizeof(Map)==0x30&&sizeof(Map::value_type)==0x10&&sizeof(MapSlot)==0x18);std::array<MapSlot,8> mapSlots{};Storage<Map,sizeof(Map)> map;
    const auto sentinel=RE::detail::BSTScatterTableSentinel;const auto bucket=RE::BSCRC32<const void*>()(thighName)&7;mapSlots[bucket]={thighName,6,0,sentinel};
    map.put(0x0C,std::uint32_t{8});map.put(0x10,std::uint32_t{7});map.put(0x14,std::uint32_t{7});map.put(0x18,sentinel);map.put(0x28,mapSlots.data());
    auto& boneMap=reinterpret_cast<RE::BSFlattenedBoneTree*>(root.object())->GetRuntimeData().boneMap;std::memcpy(&boneMap,map.bytes.data(),map.bytes.size());Storage<RE::BSFixedString,sizeof(RE::BSFixedString)> key;key.put(0,thighName);
    const auto found=boneMap.find(*key.object());require(boneMap.size()==1&&found!=boneMap.end()&&found->second==6&&std::string_view(found->first.c_str())==names[6],"real typed boneMap resolves the bounded thigh index when entry.nodeName is empty");
    fc::FlatNameSnapshot snapshot;
    {
        const auto bound=fc::bindRuntimeScene(root.object(),names,fc::canonicalBoneParents,&snapshot);
        require(bool(bound)&&bound.count==81&&bound.nodes[6].flat==&entries.object()[6].local,"map-only thigh resolves existing selected-root flat storage");
        require(snapshot.valid&&snapshot.mapUsed&&snapshot.find(names[6])==6&&snapshot.current(root.object()),"map lookup captures a current bounded identity snapshot");
    }
    refs();
    const auto acceptedMap=mapSlots;
    require(std::memcmp(mapSlots.data(),acceptedMap.data(),sizeof(mapSlots))==0,"map lookup does not mutate native map entries");
    mapSlots[bucket].name=lowerThigh.text.data();
    {
        fc::FlatNameSnapshot mixedMap;const auto bound=fc::bindRuntimeScene(root.object(),names,fc::canonicalBoneParents,&mixedMap);
        require(bool(bound)&&bound.nodes[6].flat==&entries.object()[6].local&&mixedMap.caseAliases==1,"case-only native map key resolves the unnamed required flat bone");
        require(!snapshot.current(root.object()),"case-equivalent map key replacement still invalidates exact raw identity");
    }
    mapSlots=acceptedMap;
    mapSlots[bucket].index=-1;missing("negative required map index is rejected");require(!snapshot.current(root.object()),"changed map index invalidates snapshot");
    mapSlots[bucket].index=99;missing("out-of-range required map index is rejected");
    mapSlots[bucket].index=7;missing("map name conflicting with the selected entry name is rejected");
    mapSlots=acceptedMap;
    const auto extra=(bucket+1)&7;
    const auto mapFree=[&](std::uint32_t free){std::memcpy(reinterpret_cast<std::byte*>(&boneMap)+0x10,&free,sizeof(free));};
    mapSlots[extra]={thighName,6,0,sentinel};mapFree(6);
    {
        const auto bound=fc::bindRuntimeScene(root.object(),names,fc::canonicalBoneParents);
        require(bool(bound),"duplicate aliases identifying the same storage remain unambiguous");
    }
    mapSlots[extra].index=7;mapSlots[extra].name=lowerThigh.text.data();missing("case-equivalent canonical map names identifying two storages are rejected");
    BinderName unrelated;unrelated.assign("optional-alias");
    mapSlots[extra]={unrelated.text.data(),6,0,sentinel};
    require(bool(fc::bindRuntimeScene(root.object(),names,fc::canonicalBoneParents)),"unrelated alias to a required bone does not reject it");
    mapSlots[extra].index=-1;
    require(bool(fc::bindRuntimeScene(root.object(),names,fc::canonicalBoneParents)),"unrelated sentinel map index is never dereferenced");
    const auto calfName=fc::flatRawName(entries.object()[7].nodeName);
    std::memcpy(&entries.object()[7].nodeName,&empty,sizeof(empty));mapSlots[extra]={pooled[7].text.data(),6,0,sentinel};
    {
        const auto duplicate=fc::bindRuntimeScene(root.object(),names,fc::canonicalBoneParents);
        require(!duplicate&&duplicate.missing==7,"different required canonical names cannot claim one unnamed flat transform");
    }
    std::memcpy(&entries.object()[7].nodeName,&calfName,sizeof(calfName));
    mapSlots[extra]={pooled[43].text.data(),6,0,sentinel};
    missing("engine-owned attachment alias cannot become a body output transform");
    mapSlots[extra]={pooled[0].text.data(),-1,0,sentinel};
    require(bool(fc::bindRuntimeScene(root.object(),names,fc::canonicalBoneParents)),"selected Root owns its actual node despite a sentinel root map alias");
    mapSlots[extra]={pooled[46].text.data(),1,0,sentinel};
    const auto attachmentNames=fc::captureFlatNames(root.object());
    require(attachmentNames.valid&&attachmentNames.attachment(1,names,fc::canonicalBoneParents),"map-only magic ancestor is recognized as engine-owned");
    mapSlots[extra]={pooled[43].text.data(),2,0,sentinel};
    require(fc::captureFlatNames(root.object()).attachment(2,names,fc::canonicalBoneParents),"map-only weapon ancestor is recognized as engine-owned");
    mapSlots=acceptedMap;mapFree(7);
    require(snapshot.current(root.object()),"restored map identity matches the captured snapshot");
    entries.object()[6].node=outside.object();
    require(!snapshot.currentEntry(root.object(),6)&&!snapshot.current(root.object()),"newly populated flat bone invalidates old raw storage ownership");
    entries.object()[6].node=nullptr;
    std::memcpy(&entries.object()[6].nodeName,&thighName,sizeof(thighName));
    require(!snapshot.current(root.object()),"entry-name identity change invalidates the snapshot");
    std::memcpy(&entries.object()[6].nodeName,&empty,sizeof(empty));
    entries.object()[6].local.translate.x=123;
    require(snapshot.current(root.object()),"ordinary animated transform changes retain map identity");
    entries.object()[6].local.translate.x=0;
    const auto invalidMap=[&](const char* reason) {
        const auto invalid=fc::captureFlatNames(root.object());require(!invalid.valid,reason);
        require(!fc::bindRuntimeScene(root.object(),names,fc::canonicalBoneParents),"malformed map cannot produce a binding");
    };
    mapSlots[bucket].next=mapSlots.data()+bucket;
    invalidMap("cyclic map chain is bounded and rejected");mapSlots=acceptedMap;
    mapSlots[bucket].next=reinterpret_cast<const std::byte*>(mapSlots.data())+1;
    invalidMap("unaligned map chain link is rejected");mapSlots=acceptedMap;
    mapSlots[bucket].name=reinterpret_cast<const char*>(1);
    invalidMap("unreadable map key is rejected without dereference");mapSlots=acceptedMap;
    mapFree(8);invalidMap("occupied map count must match header");mapFree(7);
    const auto mapPut=[&](std::size_t offset,auto value){std::memcpy(reinterpret_cast<std::byte*>(&boneMap)+offset,&value,sizeof(value));};
    mapPut(0x0C,std::uint32_t{8193});invalidMap("oversized map capacity is bounded");
    mapPut(0x0C,std::uint32_t{8});mapFree(9);invalidMap("map free count cannot exceed capacity");mapFree(7);
    mapPut(0x28,reinterpret_cast<void*>(1));invalidMap("unreadable map storage is rejected");
    mapPut(0x28,mapSlots.data());
    require(snapshot.current(root.object()),"malformed map fixtures restore original identity");
    {
        using Clock=std::chrono::steady_clock;
        Storage<Entry,sizeof(Entry)*536> sparse;
        std::vector<BinderName> sparseNames(536);std::vector<SceneStorage> extraNodes(260);
        std::vector<MapSlot> sparseMap(1024);
        for(std::size_t i=0;i<536;++i) {
            sparseNames[i].assign("Additional Bone "+std::to_string(i));const char* name=sparseNames[i].text.data();
            auto& entry=sparse.object()[i];entry.parentIndex=-1;entry.local.scale=entry.world.scale=1;
            std::memcpy(&entry.nodeName,&name,sizeof(name));
        }
        std::memcpy(sparse.object(),&entries.object()[4],sizeof(Entry));sparse.object()[0].parentIndex=-1;
        std::memcpy(sparse.object()+1,&entries.object()[5],sizeof(Entry));sparse.object()[1].parentIndex=0;
        for(std::size_t i=0;i<extraNodes.size();++i) {
            initialize(extraNodes[i],sparseNames[i+2]);extraNodes[i].object()->parent=root.object();
            sparse.object()[i+2].node=extraNodes[i].object();
        }
        for(std::size_t i=6;i<99;++i) {
            auto& entry=sparse.object()[294+i];std::memcpy(&entry,&entries.object()[i],sizeof(Entry));
            const auto parent=entry.parentIndex;entry.parentIndex=std::int16_t(parent>=6?parent+294:parent>=4?parent-4:-1);
        }
        for(std::size_t i=0;i<536;++i)sparseMap[i]={i==300?thighName:fc::flatRawName(sparse.object()[i].nodeName),std::int32_t(i),0,sentinel};
        mapPut(0x0C,std::uint32_t{1024});mapFree(488);mapPut(0x14,std::uint32_t{1023});mapPut(0x28,sparseMap.data());
        storage(root,sparse.object(),536,262);
        fc::FlatNameSnapshot sparseSnapshot;
        const auto captureBegin=Clock::now();
        {
            const auto bound=fc::bindRuntimeScene(root.object(),names,fc::canonicalBoneParents,&sparseSnapshot);
            require(bool(bound)&&bound.nodes[6].flat==&sparse.object()[300].local,"536-entry 262-populated sparse layout resolves a map bone beyond the populated prefix");
            require(bound.nodes[0].node.get()==root.object()&&sparseSnapshot.find(names[4])==0,"selected Root stays outside the COM-first entry table");
        }
        const auto captureMs=std::chrono::duration<double,std::milli>(Clock::now()-captureBegin).count();
        const auto begin=Clock::now();std::size_t current=0;
        for(int i=0;i<1000;++i)current+=sparseSnapshot.current(root.object());
        require(current==1000,"sparse snapshot repeated live guards remain valid");
        std::cout<<"Sparse 536-entry 262-node 1024-bucket snapshot, mocked "<<version.string()<<", bind ms="<<captureMs<<", 1000 guard passes ms="
            <<std::chrono::duration<double,std::milli>(Clock::now()-begin).count()<<'\n';
        std::vector<SceneStorage> lookupNodes(535);std::vector<RE::NiAVObject*> lookupChildren;
        for(std::size_t i=0;i<lookupNodes.size();++i) {
            initialize(lookupNodes[i],sparseNames[i]);lookupNodes[i].object()->parent=root.object();lookupChildren.push_back(lookupNodes[i].object());
        }
        diagnosticChildren(root,lookupChildren);const auto lookupBegin=Clock::now();std::size_t found=0;
        for(int i=0;i<1000;++i)found+=fc::existingNode(root.object(),names[0])==root.object();
        require(found==1000,"bounded unique Root search succeeds across 536 actual nodes");
        std::cout<<"Unique root lookup, 536 actual nodes, mocked "<<version.string()<<", 1000 passes ms="
            <<std::chrono::duration<double,std::milli>(Clock::now()-lookupBegin).count()<<'\n';
        diagnosticChildren(root,rootChildren);
        storage(root,entries.object(),99,3);mapSlots=acceptedMap;
        mapPut(0x0C,std::uint32_t{8});mapFree(7);mapPut(0x14,std::uint32_t{7});mapPut(0x28,mapSlots.data());
    }
    require(snapshot.current(root.object()),"sparse fixture leaves prior map identity restorable");
    require(entries.bytes!=originalEntries,"map-only case actually removes the entry name rather than changing expected counts");
    std::cout<<"PASS production scene binder synthetic fixtures for mocked runtime "<<version.string()<<"; these structures do not establish the player's root cause\n";
}
void sceneLookupDiagnostics() {
    require(REL::Module::mock(REL::Version(1,6,640,0),REL::Module::Runtime::AE,L"SkyrimSE.exe",0x140000000),"diagnostic AE fixture runtime");
    std::array<std::uintptr_t,4> nodeTable{},flatTable{};
    nodeTable[2]=address(&diagnosticNodeRTTI);nodeTable[3]=address(&diagnosticAsNode);
    flatTable[2]=address(&diagnosticFlatRTTI);flatTable[3]=address(&diagnosticAsNode);
    SceneStorage actor,root,inside,outside;
    auto initialize=[&](SceneStorage& storage,const char* name,bool flat=false) {
        storage.put(0,flat?flatTable.data():nodeTable.data());storage.put(0x10,name);
    };
    initialize(actor,"skeleton.nif");initialize(root,"NPC Root [Root]",true);
    initialize(inside,"NPC L Thigh [LThg]");initialize(outside,"NPC L Thigh [LThg]");
    root.put(0x30,actor.object());inside.put(0x30,root.object());outside.put(0x30,actor.object());
    std::array<RE::NiAVObject*,2> actorChildren{root.object(),outside.object()};
    std::array<RE::NiAVObject*,1> rootChildren{inside.object()};
    diagnosticChildren(actor,actorChildren);diagnosticChildren(root,rootChildren);
    using Entry=RE::BSFlattenedBoneTree::BoneEntry;
    Storage<Entry,0x100> entries;
    entries.put(0x68,std::int16_t{-1});entries.put(0x80+0x68,std::int16_t{0});
    entries.put(0x78,"NPC Root [Root]");entries.put(0x80+0x78,"NPC L Thigh [LThg]");
    root.put(0x128,std::uint32_t{2});root.put(0x12C,std::uint32_t{0});root.put(0x130,entries.object());
    const auto actorBefore=actor.bytes,rootBefore=root.bytes,insideBefore=inside.bytes,outsideBefore=outside.bytes;
    const auto entriesBefore=entries.bytes;
    auto result=fc::describeSceneLookup(actor.object(),root.object(),"NPC L Thigh [LThg]");
    require(result.actorClass=="NiNode"&&result.rootClass=="BSFlattenedBoneTree"&&result.rootName=="NPC Root [Root]","diagnostic reports actual actor and selected root classes");
    require(result.flat.state=="valid"&&result.flat.count==2&&result.flat.populated==0&&result.flat.samples=="0:NPC Root [Root]; 1:NPC L Thigh [LThg]","diagnostic samples checked flat storage");
    require(result.inside==1&&result.outside==1&&result.scanned==4&&!result.incomplete&&!result.parentChainIncomplete,"diagnostic distinguishes exact bone matches inside and outside the selected root");
    require(actor.bytes==actorBefore&&root.bytes==rootBefore&&inside.bytes==insideBefore&&outside.bytes==outsideBefore&&entries.bytes==entriesBefore,"diagnostics never mutate actor, skeleton, children or flat storage");
    Storage<Entry,0x400> namedEntries;
    for(std::size_t i=0;i<8;++i){namedEntries.put(i*0x80+0x68,std::int16_t{-1});namedEntries.put(i*0x80+0x78,"named");}
    root.put(0x128,std::uint32_t{8});root.put(0x130,namedEntries.object());
    result=fc::describeSceneLookup(actor.object(),root.object(),"Missing Thigh");
    require(result.flat.samples=="0:named; 1:named; 2:named; 3:named; 4:named; 5:named","flat diagnostic output is capped at six named samples");
    root.put(0x128,std::uint32_t{2});root.put(0x130,entries.object());
    result=fc::describeSceneLookup(actor.object(),root.object(),"Missing Thigh");
    require(!result.inside&&!result.outside&&!result.incomplete,"complete absent lookup remains absent without manufacturing a node");
    root.put(0x130,reinterpret_cast<Entry*>(1));
    result=fc::describeSceneLookup(actor.object(),root.object(),"Missing Thigh");
    require(result.flat.state=="invalid"&&result.flat.samples.empty(),"unreadable flat data cannot be sampled");
    root.put(0x130,entries.object());root.put(0x128,std::uint32_t{4097});
    require(fc::describeSceneLookup(actor.object(),root.object(),"Missing Thigh").flat.state=="invalid","oversized flat storage rejected before traversal");
    root.put(0x128,std::uint32_t{2});entries.put(0x80+0x68,std::int16_t{1});
    require(fc::describeSceneLookup(actor.object(),root.object(),"Missing Thigh").flat.state=="invalid","self-parented flat storage marked invalid");
    entries.put(0x80+0x68,std::int16_t{0});
    root.put(0,nodeTable.data());actor.put(0,flatTable.data());
    actor.put(0x128,std::uint32_t{2});actor.put(0x130,entries.object());
    result=fc::describeSceneLookup(actor.object(),root.object(),"NPC L Thigh [LThg]");
    require(result.flat.state=="not-flat"&&result.ancestorFlatTrees=="1:skeleton.nif(valid,2)","diagnostic records a flattened parent without expanding ownership");
    actor.put(0x30,root.object());
    require(fc::describeSceneLookup(actor.object(),root.object(),"Missing Thigh").parentChainIncomplete,"parent cycles terminate with explicit incomplete flag");
    actor.put(0x30,static_cast<RE::NiNode*>(nullptr));actor.put(0,nodeTable.data());
    rootChildren[0]=actor.object();
    require(fc::describeSceneLookup(actor.object(),root.object(),"Missing Thigh").incomplete,"child cycles terminate with explicit incomplete flag");
    rootChildren[0]=reinterpret_cast<RE::NiAVObject*>(1);
    require(fc::describeSceneLookup(actor.object(),root.object(),"Missing Thigh").incomplete,"unreadable actual child is not dereferenced");
    rootChildren[0]=inside.object();
    const auto childOffset=address(&root.object()->GetChildren())-root.address();root.put(childOffset+8,reinterpret_cast<void*>(1));
    require(fc::describeSceneLookup(actor.object(),root.object(),"Missing Thigh").incomplete,"unreadable children array is not traversed");
    diagnosticChildren(root,rootChildren);
    root.put(0,reinterpret_cast<void*>(1));
    result=fc::describeSceneLookup(actor.object(),root.object(),"Missing Thigh");
    require(result.rootClass=="<invalid-vtable>"&&result.incomplete,"unreadable vtable cannot be invoked");root.put(0,nodeTable.data());
    require(fc::sceneDiagnosticText(reinterpret_cast<const char*>(1))=="<unreadable>","unreadable diagnostic name is safe");
    const std::string longName(200,'a');
    require(fc::sceneDiagnosticText(longName.c_str()).size()==83&&fc::sceneDiagnosticText("a\nb")=="a?b","diagnostic names are bounded and cannot inject log lines");
    auto absent=fc::describeSceneLookup(nullptr,nullptr,"Missing Thigh");
    require(absent.actorClass=="<unreadable>"&&absent.parentChainIncomplete,"null roots are reported without dereference");
    std::vector<SceneStorage> chain(130);
    std::vector<std::array<RE::NiAVObject*,1>> links(130);
    for(std::size_t i=0;i<chain.size();++i) {
        initialize(chain[i],"branch");
        if(i)chain[i].put(0x30,chain[i-1].object());
        if(i+1<chain.size()){links[i][0]=chain[i+1].object();diagnosticChildren(chain[i],links[i]);}
    }
    result=fc::describeSceneLookup(chain.front().object(),chain.front().object(),"Missing Thigh");
    require(result.incomplete&&result.scanned==129,"actual tree depth is capped at 128");
    result=fc::describeSceneLookup(chain.back().object(),chain.back().object(),"Missing Thigh");
    require(result.parentChainIncomplete,"ancestor traversal depth is capped at 128");
    std::vector<SceneStorage> wide(4096);std::vector<RE::NiAVObject*> leaves;leaves.reserve(wide.size());
    for(auto& leaf:wide){initialize(leaf,"leaf");leaves.push_back(leaf.object());}
    diagnosticChildren(actor,leaves);
    result=fc::describeSceneLookup(actor.object(),actor.object(),"Missing Thigh");
    require(result.incomplete&&result.scanned==4096,"actual tree node budget is capped at 4096");
    std::cout<<"PASS bounded scene diagnostics: exact names, typed flat fields, owned-domain distinction, read-only guards and traversal limits\n";
}
int main() {
    try {
        constexpr std::array versions{
            REL::Version(1,5,97,0),REL::Version(1,6,317,0),REL::Version(1,6,318,0),REL::Version(1,6,323,0),
            REL::Version(1,6,342,0),REL::Version(1,6,353,0),REL::Version(1,6,629,0),REL::Version(1,6,640,0),
            REL::Version(1,6,659,0),REL::Version(1,6,1130,0),REL::Version(1,6,1170,0),REL::Version(1,6,1179,0),
            REL::Version(1,7,99,0),REL::Version(1,7,104,0)};
        skseRuntimeEncodingCases();
        for(const auto version:versions)runtimeCase(version);
        memoryGuards();
        animationSkeletonMemoryGuards();
        sceneLookupDiagnostics();
        for(const auto version:{REL::Version(1,5,97,0),REL::Version(1,6,640,0),REL::Version(1,6,1170,0)})flattenedBindingStructures(version);
        virtualHookPublication();
        addressLibraryCases();
        REL::Module::reset();
        std::cout<<"PASS runtime bridge: "<<versions.size()<<" real CommonLib runtime branches, "<<checks<<" checks\n";
        return 0;
    } catch(const std::exception& error) {
        std::cerr<<"FAIL runtime bridge after "<<checks<<" checks: "<<error.what()<<'\n';
        return 1;
    }
}
