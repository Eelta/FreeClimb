#pragma once
#include "InputBindings.h"
#include <span>

namespace fc {
inline InputState diagnosticKeyboardSnapshot(std::span<const std::uint8_t,256> raw) {
    InputState result;
    for(unsigned scan=0;scan<raw.size();++scan)result.set(scan,(raw[scan]&0x80)!=0);
    return result;
}
inline unsigned entryHeldMask(const InputState& state,const KeyChord& chord) {
    if(!validKeyChord(chord))return 0;
    unsigned result{};
    for(unsigned i=0;i<chord.count;++i)if(state.held(chord.keys[i]))result|=1u<<i;
    return result;
}
enum class EntryDiagnosticEvent {none,began,held,released};
inline const char* name(EntryDiagnosticEvent event) {
    switch(event) {
    case EntryDiagnosticEvent::began:return "begin";
    case EntryDiagnosticEvent::held:return "held";
    case EntryDiagnosticEvent::released:return "released";
    default:return "none";
    }
}
class EntryInputDiagnostics {
    bool down{};
    std::uint64_t nextReport{};
public:
    void reset(){down=false;nextReport=0;}
    EntryDiagnosticEvent sample(bool complete,std::uint64_t now,bool enabled=true) {
        if(!enabled){reset();return EntryDiagnosticEvent::none;}
        if(!complete) {
            const bool released=down;reset();
            return released?EntryDiagnosticEvent::released:EntryDiagnosticEvent::none;
        }
        if(!down){down=true;nextReport=now+2000;return EntryDiagnosticEvent::began;}
        if(now<nextReport)return EntryDiagnosticEvent::none;
        nextReport=now+2000;return EntryDiagnosticEvent::held;
    }
};
}
