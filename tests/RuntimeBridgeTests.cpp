#include "PCH.h"
#include "RuntimeSupport.h"
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

void runtimeCase(REL::Version version) {
    require(REL::Module::mock(version),"CommonLib mock initialization");
    require(REL::Module::get().version()==version,"real Module version selection");
    require(fc::runtime::supported(),"bridge runtime must be supported");
    gamepadBridgeCase();
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
            REL::Version(1,7,99,0),REL::Version(1,7,104,0)};
        skseRuntimeEncodingCases();
        for(const auto version:versions)runtimeCase(version);
        memoryGuards();
        addressLibraryCases();
        REL::Module::reset();
        std::cout<<"PASS runtime bridge: "<<versions.size()<<" real CommonLib runtime branches, "<<checks<<" checks\n";
        return 0;
    } catch(const std::exception& error) {
        std::cerr<<"FAIL runtime bridge after "<<checks<<" checks: "<<error.what()<<'\n';
        return 1;
    }
}
