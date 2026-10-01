#pragma once
#include "InputBindings.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace fc {
using GamepadChord=std::uint16_t;
struct GamepadButtonName {std::string_view name;unsigned index;};
inline constexpr std::array<GamepadButtonName,16> gamepadButtonNames{{
    {"DPadUp",0},{"DPadDown",1},{"DPadLeft",2},{"DPadRight",3},
    {"Start",4},{"Back",5},{"LS",6},{"RS",7},{"LB",8},{"RB",9},
    {"A",10},{"B",11},{"X",12},{"Y",13},{"LT",14},{"RT",15}
}};
inline constexpr GamepadChord gamepadButtonMask(unsigned index) {
    return index<16?static_cast<GamepadChord>(1u<<index):GamepadChord{};
}
inline bool validGamepadChord(GamepadChord chord) {
    return chord&&!(chord&(gamepadButtonMask(4)|gamepadButtonMask(5)))&&std::popcount(chord)<=4;
}
inline std::optional<GamepadChord> parseGamepadChord(std::string_view text) {
    GamepadChord result{};
    if(text.size()>128)return {};
    while(!text.empty()) {
        const auto plus=text.find('+');
        const auto name=trimKeyName(text.substr(0,plus));
        if(name.empty())return {};
        GamepadChord button{};
        for(const auto& named:gamepadButtonNames)if(keyNameEqual(name,named.name)) {
            button=gamepadButtonMask(named.index);break;
        }
        if(!button||(result&button))return {};
        result=static_cast<GamepadChord>(result|button);
        if(plus==std::string_view::npos)break;
        text.remove_prefix(plus+1);
        if(text.empty())return {};
    }
    if(!validGamepadChord(result))return {};
    return result;
}
inline std::string serializeGamepadChord(GamepadChord chord) {
    if(!validGamepadChord(chord))return {};
    std::string result;
    for(const auto& named:gamepadButtonNames)if(chord&gamepadButtonMask(named.index)) {
        if(!result.empty())result+='+';
        result+=named.name;
    }
    return result;
}
struct GamepadBindings {
    GamepadChord entry=gamepadButtonMask(8)|gamepadButtonMask(13);
    GamepadChord runModifier=gamepadButtonMask(8),hop=gamepadButtonMask(13),drop=gamepadButtonMask(11);
    static GamepadBindings defaults(){return {};}
    bool operator==(const GamepadBindings&)const=default;
};
struct GamepadBindingField {std::string_view name;GamepadChord GamepadBindings::* member;};
inline constexpr std::array<GamepadBindingField,4> gamepadBindingFields{{
    {"entry",&GamepadBindings::entry},{"runModifier",&GamepadBindings::runModifier},
    {"hop",&GamepadBindings::hop},{"drop",&GamepadBindings::drop}
}};
inline bool gamepadChordImplies(GamepadChord a,GamepadChord b) {return b&&(a&b)==b;}
inline BindingValidation validateGamepadBindings(const GamepadBindings& bindings) {
    for(const auto& field:gamepadBindingFields)if(!validGamepadChord(bindings.*field.member))
        return {false,"Invalid gamepad combination",std::string(field.name),{}};
    for(unsigned i=1;i<gamepadBindingFields.size();++i)for(unsigned j=i+1;j<gamepadBindingFields.size();++j) {
        const auto a=bindings.*gamepadBindingFields[i].member,b=bindings.*gamepadBindingFields[j].member;
        if(gamepadChordImplies(a,b)||gamepadChordImplies(b,a))
            return {false,"Conflicting gamepad combinations",std::string(gamepadBindingFields[i].name),std::string(gamepadBindingFields[j].name)};
    }
    if(gamepadChordImplies(bindings.entry,bindings.drop))
        return {false,"Entry cannot include the complete drop binding","entry","drop"};
    return {true,{},{},{}};
}
struct GamepadSettings {
    bool enabled=true;
    float deadzone=.25f,triggerThreshold=.5f;
    GamepadBindings bindings;
    bool operator==(const GamepadSettings&)const=default;
};
inline GamepadSettings sanitizeGamepadSettings(GamepadSettings settings) {
    settings.deadzone=std::isfinite(settings.deadzone)?std::clamp(settings.deadzone,.1f,.8f):.25f;
    settings.triggerThreshold=std::isfinite(settings.triggerThreshold)?std::clamp(settings.triggerThreshold,.1f,.95f):.5f;
    if(!validateGamepadBindings(settings.bindings).valid)settings.bindings=GamepadBindings{};
    return settings;
}
class GamepadState {
    GamepadChord buttons{};
    int horizontal{},vertical{};
    bool stickActive{},blocked{};
    static int axis(float value,int previous) {
        const float magnitude=std::abs(value);
        const int direction=value>0?1:value<0?-1:0;
        if(direction==previous&&magnitude>=.35f)return previous;
        return magnitude>=.45f?direction:0;
    }
public:
    void setButton(unsigned index,float value,float triggerThreshold) {
        if(index>=gamepadButtonNames.size())return;
        const float threshold=std::isfinite(triggerThreshold)?std::clamp(triggerThreshold,.1f,.95f):.5f;
        const bool down=std::isfinite(value)&&value>0&&(index<14||value>=threshold);
        const auto button=gamepadButtonMask(index);
        buttons=static_cast<GamepadChord>(down?(buttons|button):(buttons&~button));
    }
    void setStick(float x,float y,float deadzone) {
        if(!std::isfinite(x)||!std::isfinite(y)) {horizontal=vertical=0;stickActive=false;return;}
        x=std::clamp(x,-1.f,1.f);y=std::clamp(y,-1.f,1.f);
        const float threshold=std::isfinite(deadzone)?std::clamp(deadzone,.1f,.8f):.25f;
        const float magnitude=std::hypot(x,y);
        stickActive=magnitude>threshold;
        if(!stickActive) {horizontal=vertical=0;return;}
        horizontal=axis(x/magnitude,horizontal);vertical=axis(y/magnitude,vertical);
    }
    void sampleXInput(std::uint16_t buttonBits,std::uint8_t leftTrigger,std::uint8_t rightTrigger,
        std::int16_t lx,std::int16_t ly,const GamepadSettings& settings) {
        for(unsigned index=0;index<14;++index) {
            const unsigned bit=index<10?index:index+2;
            setButton(index,(buttonBits&(1u<<bit))?1.f:0.f,settings.triggerThreshold);
        }
        setButton(14,static_cast<float>(leftTrigger)/255.f,settings.triggerThreshold);
        setButton(15,static_cast<float>(rightTrigger)/255.f,settings.triggerThreshold);
        const auto normalized=[](std::int16_t value){return static_cast<float>(value)/(value<0?32768.f:32767.f);};
        setStick(normalized(lx),normalized(ly),settings.deadzone);
    }
    void reset(){buttons=0;horizontal=vertical=0;stickActive=false;blocked=true;}
    void blockUntilButtonsReleased(){blocked=true;}
    bool resumeIfButtonsReleased(){if(!buttons)blocked=false;return !blocked;}
    bool waitingForButtonsRelease()const{return blocked;}
    bool held(unsigned index)const{return (buttons&gamepadButtonMask(index))!=0;}
    bool neutral()const{return !buttons&&!stickActive;}
    bool heldChord(GamepadChord chord)const{return validGamepadChord(chord)&&gamepadChordImplies(buttons,chord);}
    Keys keys(const GamepadBindings& bindings)const {
        Keys result;result.bindingsMapped=true;
        if(blocked)return result;
        result.w=vertical>0;result.a=horizontal<0;result.s=vertical<0;result.d=horizontal>0;
        result.shift=heldChord(bindings.runModifier);result.space=heldChord(bindings.hop);
        result.entry=heldChord(bindings.entry);result.letGo=heldChord(bindings.drop);
        return result;
    }
};
inline bool ownsGamepadButton(const GamepadState& state,const GamepadBindings& bindings,unsigned index,bool attached) {
    if(!attached||index>=gamepadButtonNames.size()||state.waitingForButtonsRelease())return false;
    for(const auto& field:gamepadBindingFields) {
        const auto chord=bindings.*field.member;
        if((chord&gamepadButtonMask(index))&&state.heldChord(chord))return true;
    }
    return false;
}
class GamepadOwnership {
    std::array<bool,16> owned{},native{};
public:
    bool filter(unsigned index,bool isDown,bool isUp,bool attached,const GamepadState& before,const GamepadState& after,const GamepadBindings& bindings) {
        if(index>=owned.size())return false;
        if(isUp) {
            const bool consume=owned[index]&&!native[index];
            owned[index]=native[index]=false;
            return consume;
        }
        if(isDown)owned[index]=native[index]=false;
        const bool active=ownsGamepadButton(before,bindings,index,attached)||ownsGamepadButton(after,bindings,index,attached);
        if(active)owned[index]=true;
        const bool consume=owned[index];
        if(isDown&&!consume)native[index]=true;
        return consume;
    }
    bool nativeDown(unsigned index)const{return index<native.size()&&native[index];}
    bool startedNativeJump(unsigned index)const{return nativeDown(index);}
    void reset(){owned.fill(false);native.fill(false);}
};
}
