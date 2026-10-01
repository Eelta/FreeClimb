#include "InputBindings.h"
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace fc;
static unsigned checks{};
static void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
static KeyChord chord(std::string_view text){auto result=parseKeyChord(text);check(result.has_value(),"valid fixture chord");return *result;}
static void parsingAndSafety() {
    for(const auto& named:keyNames) {
        const auto parsed=parseKeyChord(named.name);
        check(parsed&&parsed->count==1&&parsed->keys[0]==named.code,"known names round trip");
        check(parseKeyChord(serializeKeyChord(*parsed))==parsed,"serialized key round trip");
    }
    check(serializeKeyChord(chord(" spacebar + control + lalt "))=="LAlt+Ctrl+Space","whitespace alias case and stable sorting");
    check(chord("Ctrl+Alt")==chord("alt+Control"),"combination order canonical");
    for(const auto text:{""," ","+","Shift+","+W","Ctrl++W","W+W","Shift+LShift","Alt+RAlt",
        "Mouse1","Escape","Win","Grave","Scan256","Alt+F4","LAlt+Tab","Ctrl+RAlt+Delete","A+B+C+D+E"," Ctrl + Nope "})
        check(!parseKeyChord(text),"invalid reserved or unsafe binding rejected");
    check(parseKeyChord("Ctrl+Alt+Shift+G").has_value(),"four-key combination accepted");
    check(!parseKeyChord(std::string(129,'A')),"bounded parser input");
    KeyChord invalid{{0x11},5};
    check(!validKeyChord(invalid)&&serializeKeyChord(invalid).empty(),"oversized in-memory combination rejected");
    InputBindings defaults;
    check(validateBindings(defaults).valid,"defaults valid");
    for(const auto& field:bindingFields) {
        auto invalidBindings=defaults;invalidBindings.*field.member={};
        check(!validateBindings(invalidBindings).valid&&normalizeBindings(invalidBindings)==defaults,"invalid field safely defaults");
    }
    InputBindings b; b.forward=chord("Up");b.backward=chord("Down");b.left=chord("Left");b.right=chord("Right");
    b.entry=chord("Ctrl+Alt");b.runModifier=chord("RShift");b.hop=chord("E");
    check(validateBindings(b).valid&&normalizeBindings(b)==b,"complete independent remapping retained");
    b.right=b.left;check(!validateBindings(b).valid,"identical direction conflict");
    b=defaults;b.hop=chord("Ctrl+W");check(!validateBindings(b).valid,"subset conflict with direction");
    b=defaults;b.forward=chord("Ctrl+W");b.entry=chord("Ctrl+Alt");
    check(validateBindings(b).valid,"entry may share components with movement bindings");
    b=defaults;b.hop=chord("RShift");check(!validateBindings(b).valid,"generic and sided modifier conflict");
    b=defaults;b.entry=chord("Ctrl+Alt");b.runModifier=chord("Ctrl");
    check(validateBindings(b).valid,"entry and run may deliberately share modifier");
    b=defaults;b.entry=chord("Ctrl+Alt+Shift+G");
    check(validateBindings(b).valid,"four key entry requires no additional forward key");
    b=defaults;b.entry={{anyAlt,0x3e},2};
    check(!validateBindings(b).valid,"unsafe full entry rejected");
    check(combineKeyChords(chord("Ctrl+W"),chord("RCtrl+Alt"))==parseKeyChord("RCtrl+Alt+W"),"legacy combo union preserves narrower modifier and canonical ordering");
    check(!combineKeyChords(chord("Ctrl+Alt+Shift+G"),chord("W")),"oversized legacy union rejected");
    check(!combineKeyChords(chord("Alt"),chord("F4")),"unsafe legacy union rejected");
    b=defaults;b.entry=chord("Ctrl+S");
    check(!validateBindings(b).valid,"entry cannot require backward cancellation at the same time");
    b=defaults;b.left=chord("Tab");b.runModifier=chord("Alt");
    check(!validateBindings(b).valid,"derived Alt Tab running rejected");
    b=defaults;b.backward=chord("Tab");b.hop=chord("Alt");
    check(!validateBindings(b).valid,"derived Alt Tab departure rejected");
}
static void eventMapping() {
    const InputBindings defaults;
    for(unsigned mask=0;mask<64;++mask)for(unsigned shift:{0x2au,0x36u}) {
        InputState state;
        for(auto [scan,bit]:std::array<std::pair<unsigned,unsigned>,6>{{{0x11,1},{0x1e,2},{0x1f,4},{0x20,8},{shift,16},{0x39,32}}})state.set(scan,mask&bit);
        const auto k=mapKeys(state,defaults);
        check(k.w==bool(mask&1)&&k.a==bool(mask&2)&&k.s==bool(mask&4)&&k.d==bool(mask&8)&&k.shift==bool(mask&16)&&k.space==bool(mask&32),"default event state matches legacy keys");
        check(k.entry==(k.w&&k.a&&k.d&&k.space),"mapped entry requires the entire four-key chord");
        check(approachIntent(k)==(k.w&&k.a&&k.d&&k.space&&!k.s),"Shift cannot substitute for the deliberate entry chord");
        const Keys legacy{k.w,k.a,k.s,k.d,k.shift,k.space};
        const auto a=wallInput(k,k.space),b=wallInput(legacy,legacy.space);
        check(a.x==b.x&&a.y==b.y&&a.release==b.release&&a.hop==b.hop&&a.run==b.run,"default wall routing preserved");
    }
    InputState state;state.set(0x2a,true);state.set(0x36,true);state.set(0x2a,false);
    check(mapKeys(state,defaults).shift,"releasing one Shift preserves other");
    state.set(0x36,false);check(!mapKeys(state,defaults).shift,"both Shift released");
    state.set(0xffff,true);check(!state.held(0xffff),"out of range events ignored");
    InputBindings b;b.forward=chord("Up");b.backward=chord("Down");b.left=chord("Left");b.right=chord("Right");
    b.entry=chord("Ctrl+Alt");b.runModifier=chord("G");b.hop=chord("E");
    state.reset();state.set(0x1d,true);
    check(!approachIntent(mapKeys(state,b)),"partial modifier cannot enter");
    state.set(0xb8,true);auto k=mapKeys(state,b);
    check(approachIntent(k)&&!k.w&&!k.shift,"custom entry needs neither Forward nor the run modifier");
    state.set(0xc8,true);
    check(!ownsScan(state,b,0x1d,false)&&ownsScan(state,b,0x1d,true),"ownership requires attachment");
    state.set(0xb8,false);check(!ownsScan(state,b,0x1d,true),"partial combo preserves native key");
    state.set(0x22,true);state.set(0x12,true);k=mapKeys(state,b);
    check(wallInput(k,true).run&&!wallInput(k,true).hop,"rebound run suppresses manual hop");
    state.set(0xd0,true);k=mapKeys(state,b);
    check(wallInput(k,true).release&&!wallInput(k,true).run,"rebound backward and hop leave wall");
    state.reset();check(!approachIntent(mapKeys(state,b))&&!mapKeys(state,b).space,"focus reset clears all state");
}
static void ownershipLifetimes() {
    InputBindings b;b.entry=chord("Ctrl+Alt");b.runModifier=chord("Ctrl+Alt");b.hop=chord("E");
    InputState state;InputOwnership ownership;
    const auto event=[&](unsigned scan,bool down,bool attached) {
        const auto before=state;state.set(scan,down);
        return ownership.filter(scan,down,!down,attached,before,state,b);
    };
    check(!event(0x1d,true,false)&&ownership.nativeDown(0x1d),"ground Ctrl native");
    check(!event(0x38,true,false),"ground full modifier remains native");
    check(!event(0x11,true,false),"ground approach direction remains native");
    check(!event(0x1d,false,true),"native down receives up after attachment");
    check(!event(0x38,false,true)&&!event(0x11,false,true),"all original native releases delivered");
    check(event(0x12,true,true),"attached hop key consumed");
    check(event(0x12,false,false),"owned release consumed after detachment");
    check(!event(0x1d,true,true),"partial attached modifier remains native");
    check(event(0x38,true,true),"completing attached modifier consumes completing key");
    check(!event(0x1d,false,true),"partial native modifier up preserved");
    check(event(0x38,false,false),"owned completion up consumed after split and detach");
    check(!event(0x10,true,true)&&!event(0x10,false,true),"unrelated key untouched");
    check(!ownership.filter(256,true,false,true,state,state,b),"non keyboard code untouched");
    ownership.reset();state.reset();check(!ownership.nativeDown(0x1d),"reset ownership history");
}
static void movementHandoff() {
    for(unsigned scan:{0x11u,0x1eu,0x1fu,0x20u}) {
        InputBindings bindings;InputState state;InputOwnership ownership;
        const auto event=[&](bool down,bool up,bool attached,bool resume) {
            const auto before=state;state.set(scan,!up);
            return ownership.filter(scan,down,up,attached,before,state,bindings,resume);
        };
        check(event(true,false,true,true),"wall movement down remains owned");
        check(event(false,false,true,true)&&!ownership.nativeDown(scan),"held movement stays blocked while attached");
        check(event(false,false,false,false)&&!ownership.nativeDown(scan),"non-movement or suspended event cannot transfer ownership");
        check(!event(false,false,false,true)&&ownership.nativeDown(scan),"actual native movement held resumes after release without a fabricated keydown");
        for(int frame=0;frame<60;++frame)check(!event(false,false,false,true),"resumed movement keeps reaching native controls");
        check(!event(false,true,false,true)&&!ownership.nativeDown(scan),"resumed movement receives its matching native keyup");
        check(!event(true,false,false,true)&&!event(false,true,false,true),"fresh ground press retains ordinary native lifetime");
    }
    for(unsigned scan:{0xc8u,0xcbu,0xd0u,0xcdu}) {
        InputBindings bindings;bindings.forward=chord("Up");bindings.left=chord("Left");bindings.backward=chord("Down");bindings.right=chord("Right");
        InputOwnership ownership;InputState before,after;after.set(scan,true);
        check(ownership.filter(scan,true,false,true,before,after,bindings),"custom direction is owned on the wall");
        check(ownership.filter(scan,false,false,false,after,after,bindings,false),"custom FreeClimb direction alone does not authorize native movement");
        check(!ownership.filter(scan,false,false,false,after,after,bindings,true)&&ownership.nativeDown(scan),"matching native rebound movement resumes on the real held event");
        check(!ownership.filter(scan,false,true,false,after,before,bindings,true),"rebound movement release reaches native controls");
    }
    {
        InputBindings bindings;bindings.forward=chord("E");
        InputOwnership ownership;InputState before,after;after.set(0x12,true);
        check(ownership.filter(0x12,true,false,true,before,after,bindings),"FreeClimb E direction was consumed");
        check(ownership.filter(0x12,false,false,false,after,after,bindings,false)&&!ownership.nativeDown(0x12),"non-native movement E cannot resume the activation key");
        check(ownership.filter(0x12,false,true,false,after,before,bindings,false),"untransferred activation-key release stays consumed");
    }
    for(unsigned scan:{0x39u,0x2au}) {
        InputBindings bindings;InputOwnership ownership;InputState before,after;after.set(scan,true);
        check(ownership.filter(scan,true,false,true,before,after,bindings),"hop and modifier fixture is owned on the wall");
        check(ownership.filter(scan,false,false,false,after,after,bindings,true)&&!ownership.nativeDown(scan),"native movement mapping cannot replay owned hop or modifier");
        check(ownership.filter(scan,false,true,false,after,before,bindings,true),"untransferred hop and modifier release stays consumed");
    }
    {
        InputBindings bindings;bindings.forward=chord("E");bindings.hop=chord("W");
        InputOwnership ownership;InputState before,after;after.set(0x11,true);
        check(ownership.filter(0x11,true,false,true,before,after,bindings),"native Forward used as a custom hop is owned");
        bindings=InputBindings{};
        check(ownership.filter(0x11,false,false,false,after,after,bindings,true),"rebind cannot turn a held old hop into a direction handoff");
    }
    {
        InputBindings bindings;bindings.forward=chord("Ctrl+W");
        InputOwnership ownership;InputState before,after;after.set(0x1d,true);after.set(0x11,true);
        check(ownership.filter(0x11,true,false,true,before,after,bindings),"custom directional chord owns its movement key");
        check(!ownership.filter(0x11,false,false,false,after,after,bindings,true),"movement component of a custom chord can resume native movement");
        check(ownership.filter(0x1d,true,false,true,before,after,bindings),"direction chord modifier fixture is owned");
        check(ownership.filter(0x1d,false,false,false,after,after,bindings,true),"direction chord modifier never resumes through movement handoff");
    }
    {
        InputBindings bindings;InputOwnership ownership;InputState before,after;after.set(0x11,true);
        check(ownership.filter(0x11,true,false,true,before,after,bindings),"suspension fixture starts with an owned movement press");
        check(ownership.filter(0x11,false,false,false,after,after,bindings,false),"menu and focus suspension do not deliver held movement");
        check(ownership.filter(0x11,false,false,false,before,before,bindings,true),"reset input state cannot fabricate a physically held movement key");
        check(ownership.filter(0x11,false,true,false,after,before,bindings,false),"suspended untransferred movement release stays paired");
        ownership.reset();
        check(!ownership.filter(0x11,true,false,false,before,after,bindings,true),"fresh native press after suspension remains native");
        check(!ownership.filter(0x11,false,true,true,after,before,bindings,true),"native keyup is preserved even if another wall entry started");
    }
}
static void customEntryRequests() {
    for(int fps:{30,60,120})for(const char* value:{"G","Ctrl+E","Ctrl+Alt+E","W+A+D+Space"}) {
        const float dt=1.f/fps;InputBindings b;b.entry=chord(value);
        InputState state;ClimbEntryIntent intent;
        const auto scan=[](KeyCode code){return code==anyControl?KeyCode{0x1d}:code==anyAlt?KeyCode{0x38}:code;};
        for(unsigned i=0;i<b.entry.count;++i) {
            state.set(scan(b.entry.keys[i]),true);
            const auto request=intent.sample(mapKeys(state,b),false,false,dt,true);
            check(request.requested==(i+1==b.entry.count),"only the complete custom entry chord requests attachment");
            if(request.requested)check(request.fresh&&request.began&&request.airborneAtBegin,"complete chord immediately captures one fresh airborne intent");
        }
        for(int frame=0;frame<fps;++frame) {
            const auto request=intent.sample(mapKeys(state,b),false,false,dt,false);
            check(request.requested&&!request.fresh&&!request.began&&request.airborneAtBegin,"held entry keeps probing without replaying a fresh gesture or losing its real origin");
        }
        intent.blockUntilRelease();
        for(int frame=0;frame<fps;++frame)
            check(!intent.sample(mapKeys(state,b),false,false,dt).requested,"held entry cannot rearm itself after detachment");
        state.set(scan(b.entry.keys[0]),false);
        check(!intent.sample(mapKeys(state,b),false,false,dt).requested&&!intent.waitingForRelease(),"release of one chord member rearms without a delayed tap request");
        state.set(scan(b.entry.keys[0]),true);
        auto request=intent.sample(mapKeys(state,b),false,false,dt);
        check(request.requested&&request.fresh&&!request.airborneAtBegin,"new complete chord captures the current grounded origin");
        check(!intent.sample(mapKeys(state,b),true,false,dt).requested,"attachment blocks further entry requests");
        check(!intent.sample(mapKeys(state,b),false,true,dt).requested,"menu suspension cannot start entry");
        state.reset();intent.sample(mapKeys(state,b),false,false,dt);
        check(!intent.waitingForRelease(),"focus reset leaves no stuck chord ownership");
    }
    for(int fps:{30,60,120}) {
        ClimbEntryIntent intent;InputBindings b;InputState state;
        state.set(0x11,true);state.set(0x2a,true);
        for(int frame=0;frame<fps*3;++frame)
            check(!intent.sample(mapKeys(state,b),false,false,1.f/fps).requested,"Shift plus Forward never triggers ground entry regardless of duration");
    }
    std::array<unsigned,4> order{0x11,0x1e,0x20,0x39};
    do {
        InputBindings b;InputState state;InputOwnership ownership;ClimbEntryIntent intent;
        for(unsigned i=0;i<order.size();++i) {
            const auto before=state;state.set(order[i],true);
            check(!ownership.filter(order[i],true,false,false,before,state,b),"ground entry chord events remain native before attachment");
            check(intent.sample(mapKeys(state,b)).requested==(i+1==order.size()),"all 24 chord orders enter only after their final member");
        }
        for(auto scan:order) {
            const auto before=state;state.set(scan,false);
            check(!ownership.filter(scan,false,true,true,before,state,b),"entry keys receive all matching native keyups after attachment");
        }
    } while(std::next_permutation(order.begin(),order.end()));
}
static void entryAndSuspension() {
    InputBindings b;b.entry=chord("Ctrl+Alt");b.runModifier=chord("Shift");
    InputState state;state.set(0x11,true);state.set(0x1d,true);state.set(0x38,true);
    auto k=mapKeys(state,b);
    check(k.entry&&!k.shift&&!wallInput(k,false).run,"custom entry cannot implicitly latch wall run");
    state.set(0x11,false);state.set(0x1e,true);k=mapKeys(state,b);
    check(k.entry&&!k.shift&&!wallInput(k,false).run&&wallInput(k,false).x==-1,"held entry permits ordinary sideways climbing without turning into wall run");
    state.set(0x36,true);k=mapKeys(state,b);
    check(k.shift&&wallInput(k,false).run,"independent run modifier still enables wall run while attached");
    state.set(0x1f,true);state.set(0x39,true);k=mapKeys(state,b);
    check(wallInput(k,true).release&&!wallInput(k,true).run,"backward departure retains priority over the run modifier");
    state.reset();k=mapKeys(state,b);
    check(!k.entry&&!k.shift,"menu focus reset clears entry and wall run keys");
    InputOwnership ownership;
    auto before=state;state.set(0x39,true);
    check(!ownership.filter(0x39,true,false,false,before,state,b),"native Space down is delivered before suspend");
    state.reset();
    check(ownership.nativeDown(0x39)&&!(state.held(0x39)&&ownership.nativeDown(0x39)),"missing focus-loss keyup cannot masquerade as a held native jump");
    check(!ownership.filter(0x39,false,true,false,state,state,b)&&!ownership.nativeDown(0x39),"preserved native ownership still forwards release during menu");
    before=state;state.set(0x39,true);
    check(ownership.filter(0x39,true,false,true,before,state,b),"old hop owned before rebind");
    b.hop=chord("E");state.reset();
    check(ownership.filter(0x39,false,true,false,state,state,b),"old binding release stays owned after menu and rebind");
    before=state;state.set(0x12,true);
    check(!ownership.filter(0x12,true,false,false,before,state,b),"new ordinary ground key remains native after rebind");
    before=state;state.set(0x12,false);
    check(!ownership.filter(0x12,false,true,true,before,state,b),"rebound native key receives up even after later attachment");
    before=state;state.set(0x12,true);
    check(ownership.filter(0x12,true,false,true,before,state,b),"new attached press is owned before a lost focus release");
    state.reset();before=state;
    check(!ownership.filter(0x12,true,false,false,before,state,b),"a fresh menu keydown supersedes stale owned history when focus lost its keyup");
    check(!ownership.filter(0x12,false,true,false,state,state,b),"new menu press still receives its matching native keyup");
}
int main(){try {
    parsingAndSafety();eventMapping();ownershipLifetimes();movementHandoff();customEntryRequests();entryAndSuspension();
    std::cout<<"PASS: "<<checks<<" keyboard binding and deliberate entry checks\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
