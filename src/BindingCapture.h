#pragma once
#include "GamepadInput.h"
#include <bitset>

namespace fc {
enum class BindingCaptureDevice {keyboard,gamepad};
enum class BindingCaptureStatus {idle,waiting,armed,captured,error,cancelled};
enum class BindingCaptureError {none,invalid,tooMany,notSimultaneous};
class BindingCaptureOwnership {
    std::bitset<272> owned;
    static unsigned slot(BindingCaptureDevice device,unsigned code) {
        const bool keyboard=device==BindingCaptureDevice::keyboard;
        return code<(keyboard?256u:16u)?code+(keyboard?0u:256u):272u;
    }
public:
    bool filter(BindingCaptureDevice device,unsigned code,bool isDown,bool isUp,bool capturing) {
        const auto index=slot(device,code);
        if(index>=owned.size())return capturing;
        const bool previous=owned[index];
        if(isUp){owned[index]=false;return previous;}
        if(isDown){owned[index]=capturing;return capturing;}
        return capturing||previous;
    }
    bool pending()const{return owned.any();}
    bool owns(BindingCaptureDevice device,unsigned code)const {
        const auto index=slot(device,code);return index<owned.size()&&owned[index];
    }
    void reset(){owned.reset();}
};
struct BindingCaptureSnapshot {
    InputState keyboard{};
    GamepadChord gamepad{};
    bool keyboardValid{},gamepadValid{},activationHeld{};
};
struct BindingCaptureResult {
    BindingCaptureStatus status=BindingCaptureStatus::idle;
    BindingCaptureDevice device=BindingCaptureDevice::keyboard;
    BindingCaptureError error=BindingCaptureError::none;
    KeyChord keyboard{};
    GamepadChord gamepad{};
    std::string preview;
};
class BindingCapture {
    BindingCaptureResult value;
    std::bitset<256> candidate;
    const BindingCaptureResult& finish(BindingCaptureStatus status,BindingCaptureError error=BindingCaptureError::none) {
        const auto device=value.device;
        value={};value.device=device;value.status=status;value.error=error;candidate.reset();
        return value;
    }
public:
    const BindingCaptureResult& begin(BindingCaptureDevice device) {
        value={};value.device=device;value.status=BindingCaptureStatus::waiting;candidate.reset();
        return value;
    }
    const BindingCaptureResult& sample(const BindingCaptureSnapshot& snapshot) {
        if(!active())return value;
        if(snapshot.keyboardValid&&snapshot.keyboard.held(0x01))return cancel();
        const bool keyboard=value.device==BindingCaptureDevice::keyboard;
        if(snapshot.gamepadValid&&(snapshot.gamepad&gamepadButtonMask(5)))return cancel();
        if(keyboard?!snapshot.keyboardValid:!snapshot.gamepadValid)return value;
        std::bitset<256> pressed;
        if(keyboard) {
            for(std::size_t i=0;i<snapshot.keyboard.down.size();++i)pressed[i]=snapshot.keyboard.down[i];
        } else {
            for(unsigned i=0;i<16;++i)pressed[i]=(snapshot.gamepad&gamepadButtonMask(i))!=0;
        }
        if(value.status==BindingCaptureStatus::waiting) {
            if(pressed.none()&&!snapshot.activationHeld)value.status=BindingCaptureStatus::armed;
            return value;
        }
        if(pressed.none()) {
            if(candidate.any())value.status=BindingCaptureStatus::captured;
            return value;
        }
        if(pressed.count()>4)return finish(BindingCaptureStatus::error,BindingCaptureError::tooMany);
        KeyChord chord;
        if(keyboard) {
            for(unsigned i=0;i<256;++i)if(pressed[i])chord.keys[chord.count++]=static_cast<KeyCode>(i);
            if(!validKeyChord(chord))return finish(BindingCaptureStatus::error,BindingCaptureError::invalid);
        } else if(!validGamepadChord(snapshot.gamepad))return finish(BindingCaptureStatus::error,BindingCaptureError::invalid);
        if((pressed&~candidate).none())return value;
        if((pressed&candidate)!=candidate)return finish(BindingCaptureStatus::error,BindingCaptureError::notSimultaneous);
        candidate=pressed;
        if(keyboard) {
            std::sort(chord.keys.begin(),chord.keys.begin()+chord.count,[](KeyCode a,KeyCode b) {
                const bool am=keyGroup(a)>=anyShift,bm=keyGroup(b)>=anyShift;
                return am!=bm?am:a<b;
            });
            value.keyboard=chord;value.preview=serializeKeyChord(chord);
        } else {
            value.gamepad=snapshot.gamepad;value.preview=serializeGamepadChord(snapshot.gamepad);
        }
        return value;
    }
    const BindingCaptureResult& cancel(){return finish(BindingCaptureStatus::cancelled);}
    const BindingCaptureResult& result()const{return value;}
    bool active()const{return value.status==BindingCaptureStatus::waiting||value.status==BindingCaptureStatus::armed;}
    void reset(){value={};candidate.reset();}
};
}
