#pragma once
#include "Controls.h"
#include <array>
#include <cctype>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace fc {
using KeyCode=std::uint16_t;
inline constexpr KeyCode anyShift=0x100,anyControl=0x101,anyAlt=0x102;
struct KeyChord {
    std::array<KeyCode,4> keys{};
    std::uint8_t count{};
    bool operator==(const KeyChord&)const=default;
};
struct KeyName {std::string_view name;KeyCode code;};
inline constexpr KeyName keyNames[]={
    {"Shift",anyShift},{"Ctrl",anyControl},{"Alt",anyAlt},
    {"LShift",0x2a},{"RShift",0x36},{"LCtrl",0x1d},{"RCtrl",0x9d},{"LAlt",0x38},{"RAlt",0xb8},
    {"A",0x1e},{"B",0x30},{"C",0x2e},{"D",0x20},{"E",0x12},{"F",0x21},{"G",0x22},
    {"H",0x23},{"I",0x17},{"J",0x24},{"K",0x25},{"L",0x26},{"M",0x32},{"N",0x31},
    {"O",0x18},{"P",0x19},{"Q",0x10},{"R",0x13},{"S",0x1f},{"T",0x14},{"U",0x16},
    {"V",0x2f},{"W",0x11},{"X",0x2d},{"Y",0x15},{"Z",0x2c},
    {"0",0x0b},{"1",0x02},{"2",0x03},{"3",0x04},{"4",0x05},{"5",0x06},{"6",0x07},{"7",0x08},{"8",0x09},{"9",0x0a},
    {"Space",0x39},{"Tab",0x0f},{"Enter",0x1c},{"Backspace",0x0e},{"CapsLock",0x3a},
    {"Minus",0x0c},{"Equals",0x0d},{"LeftBracket",0x1a},{"RightBracket",0x1b},
    {"Semicolon",0x27},{"Apostrophe",0x28},{"Backslash",0x2b},{"Comma",0x33},{"Period",0x34},{"Slash",0x35},
    {"Up",0xc8},{"Down",0xd0},{"Left",0xcb},{"Right",0xcd},{"Home",0xc7},{"End",0xcf},
    {"PageUp",0xc9},{"PageDown",0xd1},{"Insert",0xd2},{"Delete",0xd3},
    {"F1",0x3b},{"F2",0x3c},{"F3",0x3d},{"F4",0x3e},{"F5",0x3f},{"F6",0x40},
    {"F7",0x41},{"F8",0x42},{"F9",0x43},{"F10",0x44},{"F11",0x57},{"F12",0x58},
    {"Num0",0x52},{"Num1",0x4f},{"Num2",0x50},{"Num3",0x51},{"Num4",0x4b},
    {"Num5",0x4c},{"Num6",0x4d},{"Num7",0x47},{"Num8",0x48},{"Num9",0x49},
    {"NumEnter",0x9c},{"NumPlus",0x4e},{"NumMinus",0x4a},{"NumMultiply",0x37},{"NumDivide",0xb5},{"NumDecimal",0x53}
};
inline bool keyNameEqual(std::string_view a,std::string_view b) {
    if(a.size()!=b.size())return false;
    for(std::size_t i=0;i<a.size();++i)
        if(std::tolower(static_cast<unsigned char>(a[i]))!=std::tolower(static_cast<unsigned char>(b[i])))return false;
    return true;
}
inline std::string_view trimKeyName(std::string_view text) {
    while(!text.empty()&&std::isspace(static_cast<unsigned char>(text.front())))text.remove_prefix(1);
    while(!text.empty()&&std::isspace(static_cast<unsigned char>(text.back())))text.remove_suffix(1);
    return text;
}
inline KeyCode keyGroup(KeyCode key) {
    if(key==0x2a||key==0x36||key==anyShift)return anyShift;
    if(key==0x1d||key==0x9d||key==anyControl)return anyControl;
    if(key==0x38||key==0xb8||key==anyAlt)return anyAlt;
    return key;
}
inline bool keyImplies(KeyCode a,KeyCode b) {return a==b||(b>=anyShift&&keyGroup(a)==b);}
inline bool keyOverlaps(KeyCode a,KeyCode b) {return keyImplies(a,b)||keyImplies(b,a);}
inline bool validKeyCode(KeyCode key) {
    for(const auto& named:keyNames)if(named.code==key)return true;
    return false;
}
inline bool chordHasGroup(const KeyChord& chord,KeyCode key) {
    for(unsigned i=0;i<chord.count&&i<chord.keys.size();++i)if(keyGroup(chord.keys[i])==key)return true;
    return false;
}
inline bool validKeyChord(const KeyChord& chord) {
    if(!chord.count||chord.count>chord.keys.size())return false;
    for(unsigned i=0;i<chord.count;++i) {
        if(!validKeyCode(chord.keys[i]))return false;
        for(unsigned j=0;j<i;++j)if(keyOverlaps(chord.keys[i],chord.keys[j]))return false;
    }
    if(chordHasGroup(chord,anyAlt)&&(chordHasGroup(chord,0x0f)||chordHasGroup(chord,0x3e)))return false;
    if(chordHasGroup(chord,anyAlt)&&chordHasGroup(chord,anyControl)&&chordHasGroup(chord,0xd3))return false;
    return true;
}
inline std::optional<KeyChord> parseKeyChord(std::string_view text) {
    KeyChord result;
    if(text.size()>128)return {};
    while(!text.empty()) {
        if(result.count>=result.keys.size())return {};
        const auto plus=text.find('+');
        auto name=trimKeyName(text.substr(0,plus));
        if(name.empty())return {};
        if(keyNameEqual(name,"Control"))name="Ctrl";
        if(keyNameEqual(name,"Spacebar"))name="Space";
        if(keyNameEqual(name,"Return"))name="Enter";
        bool found=false;
        for(const auto& named:keyNames)if(keyNameEqual(name,named.name)) {
            result.keys[result.count++]=named.code;found=true;break;
        }
        if(!found)return {};
        if(plus==std::string_view::npos)break;
        text.remove_prefix(plus+1);
        if(text.empty())return {};
    }
    if(!validKeyChord(result))return {};
    std::sort(result.keys.begin(),result.keys.begin()+result.count,[](KeyCode a,KeyCode b) {
        const bool am=keyGroup(a)>=anyShift,bm=keyGroup(b)>=anyShift;
        if(am!=bm)return am;
        return a<b;
    });
    return result;
}
inline std::string serializeKeyChord(const KeyChord& chord) {
    if(!validKeyChord(chord))return {};
    std::string result;
    for(unsigned i=0;i<chord.count;++i)for(const auto& named:keyNames)if(named.code==chord.keys[i]) {
        if(!result.empty())result+='+';
        result+=named.name;break;
    }
    return result;
}
struct InputBindings {
    KeyChord forward{{0x11},1},backward{{0x1f},1},left{{0x1e},1},right{{0x20},1};
    KeyChord entry{{0x11,0x1e,0x20,0x39},4},runModifier{{anyShift},1},hop{{0x39},1};
    static InputBindings defaults(){return {};}
    bool operator==(const InputBindings&)const=default;
};
struct BindingValidation {bool valid{};std::string message,first,second;};
struct BindingField {std::string_view name;KeyChord InputBindings::* member;};
inline constexpr std::array<BindingField,7> bindingFields{{
    {"forward",&InputBindings::forward},{"backward",&InputBindings::backward},
    {"left",&InputBindings::left},{"right",&InputBindings::right},
    {"entry",&InputBindings::entry},{"runModifier",&InputBindings::runModifier},{"hop",&InputBindings::hop}
}};
inline bool chordImplies(const KeyChord& a,const KeyChord& b) {
    for(unsigned i=0;i<b.count;++i) {
        bool found=false;
        for(unsigned j=0;j<a.count;++j)found|=keyImplies(a.keys[j],b.keys[i]);
        if(!found)return false;
    }
    return true;
}
inline bool chordSharesKey(const KeyChord& a,const KeyChord& b) {
    for(unsigned i=0;i<a.count;++i)for(unsigned j=0;j<b.count;++j)if(keyOverlaps(a.keys[i],b.keys[j]))return true;
    return false;
}
inline unsigned combinedKeyCount(const KeyChord& a,const KeyChord& b) {
    unsigned count=a.count;
    for(unsigned i=0;i<b.count;++i) {
        bool found=false;
        for(unsigned j=0;j<a.count;++j)found|=keyOverlaps(b.keys[i],a.keys[j]);
        count+=!found;
    }
    return count;
}
inline std::optional<KeyChord> combineKeyChords(const KeyChord& a,const KeyChord& b) {
    if(!validKeyChord(a)||!validKeyChord(b))return {};
    KeyChord combined=a;
    for(unsigned i=0;i<b.count;++i) {
        bool found=false;
        for(unsigned j=0;j<combined.count;++j)if(keyOverlaps(b.keys[i],combined.keys[j])) {
            if(keyImplies(b.keys[i],combined.keys[j]))combined.keys[j]=b.keys[i];
            found=true;break;
        }
        if(!found) {
            if(combined.count>=combined.keys.size())return {};
            combined.keys[combined.count++]=b.keys[i];
        }
    }
    return parseKeyChord(serializeKeyChord(combined));
}
inline bool validCombinedChord(const KeyChord& a,const KeyChord& b) {return combineKeyChords(a,b).has_value();}
inline BindingValidation validateBindings(const InputBindings& bindings) {
    for(const auto& field:bindingFields)if(!validKeyChord(bindings.*field.member))
        return {false,"Invalid keyboard combination",std::string(field.name),{}};
    for(unsigned i=0;i<bindingFields.size();++i)for(unsigned j=i+1;j<bindingFields.size();++j) {
        if(i==4||j==4)continue;
        const auto& a=bindings.*bindingFields[i].member;
        const auto& b=bindings.*bindingFields[j].member;
        const bool modifierDirection=i<4&&j==5;
        if(chordImplies(a,b)||chordImplies(b,a)||(modifierDirection&&chordSharesKey(a,b)))
            return {false,"Conflicting keyboard combinations",std::string(bindingFields[i].name),std::string(bindingFields[j].name)};
    }
    if(chordImplies(bindings.entry,bindings.backward))
        return {false,"Entry cannot include the complete backward binding","entry","backward"};
    if(combinedKeyCount(bindings.backward,bindings.hop)>4)
        return {false,"Wall departure requires more than four keys","backward","hop"};
    if(!validCombinedChord(bindings.backward,bindings.hop))
        return {false,"Unsafe wall departure combination","backward","hop"};
    for(const auto& field:bindingFields) {
        if(field.member!=&InputBindings::forward&&field.member!=&InputBindings::left&&field.member!=&InputBindings::right)continue;
        if(!validCombinedChord(bindings.*field.member,bindings.runModifier))
            return {false,"Unsafe or oversized wall running combination",std::string(field.name),"runModifier"};
    }
    return {true,{},{},{}};
}
inline InputBindings normalizeBindings(const InputBindings& bindings) {return validateBindings(bindings).valid?bindings:InputBindings{};}
struct InputState {
    std::array<bool,256> down{};
    void set(unsigned scan,bool pressed){if(scan<down.size())down[scan]=pressed;}
    void reset(){down.fill(false);}
    bool held(KeyCode key)const {
        if(key==anyShift)return down[0x2a]||down[0x36];
        if(key==anyControl)return down[0x1d]||down[0x9d];
        if(key==anyAlt)return down[0x38]||down[0xb8];
        return key<down.size()&&down[key];
    }
};
inline bool chordHeld(const InputState& state,const KeyChord& chord) {
    if(!validKeyChord(chord))return false;
    for(unsigned i=0;i<chord.count;++i)if(!state.held(chord.keys[i]))return false;
    return true;
}
inline Keys mapKeys(const InputState& state,const InputBindings& bindings) {
    return {chordHeld(state,bindings.forward),chordHeld(state,bindings.left),chordHeld(state,bindings.backward),
        chordHeld(state,bindings.right),chordHeld(state,bindings.runModifier),chordHeld(state,bindings.hop),
        chordHeld(state,bindings.entry),true};
}
inline bool chordOwnsScan(const KeyChord& chord,unsigned scan) {
    if(scan>=256||!validKeyChord(chord))return false;
    for(unsigned i=0;i<chord.count;++i)if(keyImplies(static_cast<KeyCode>(scan),chord.keys[i]))return true;
    return false;
}
inline bool ownsScan(const InputBindings& bindings,unsigned scan) {
    for(const auto& field:bindingFields)if(chordOwnsScan(bindings.*field.member,scan))return true;
    return false;
}
inline bool ownsScan(const InputState& state,const InputBindings& bindings,unsigned scan,bool attached) {
    if(!attached)return false;
    for(const auto& field:bindingFields) {
        const auto& chord=bindings.*field.member;
        if(chordHeld(state,chord)&&chordOwnsScan(chord,scan))return true;
    }
    return false;
}
class InputOwnership {
    std::array<bool,256> owned{},native{},movementOwned{};
    static bool movementPress(unsigned scan,const InputState& state,const InputBindings& bindings) {
        if(keyGroup(static_cast<KeyCode>(scan))>=anyShift||chordOwnsScan(bindings.hop,scan)||
            chordOwnsScan(bindings.runModifier,scan))return false;
        for(unsigned i=0;i<4;++i) {
            const auto& chord=bindings.*bindingFields[i].member;
            if(chordHeld(state,chord)&&chordOwnsScan(chord,scan))return true;
        }
        return false;
    }
public:
    bool filter(unsigned scan,bool isDown,bool isUp,bool attached,const InputState& before,const InputState& after,const InputBindings& bindings,bool resumeMovement=false) {
        if(scan>=owned.size())return false;
        if(isUp) {
            const bool consume=owned[scan]&&!native[scan];
            owned[scan]=native[scan]=movementOwned[scan]=false;
            return consume;
        }
        if(isDown)owned[scan]=native[scan]=movementOwned[scan]=false;
        if(!isDown&&!attached&&resumeMovement&&owned[scan]&&movementOwned[scan]&&after.held(static_cast<KeyCode>(scan))) {
            owned[scan]=movementOwned[scan]=false;native[scan]=true;
        }
        const bool active=ownsScan(before,bindings,scan,attached)||ownsScan(after,bindings,scan,attached);
        const bool consume=owned[scan]||active;
        if(isDown) {
            if(consume) {owned[scan]=true;movementOwned[scan]=movementPress(scan,after,bindings);}
            else native[scan]=true;
        }
        return consume;
    }
    bool nativeDown(unsigned scan)const{return scan<native.size()&&native[scan];}
    void reset(){owned.fill(false);native.fill(false);movementOwned.fill(false);}
};
}
