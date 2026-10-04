#pragma once
#include "Core.h"
namespace fc {
struct Keys { bool w{},a{},s{},d{},shift{},space{},entry{},bindingsMapped{},letGo{}; };

inline Input wallInput(Keys k,bool spacePressed,bool autoMantle=true,bool justAttached=false,bool wasWallRunning=false) {
    if(k.letGo||(k.a&&k.s&&k.d&&(spacePressed||k.space)))return {0,0,true,false,false,false,false};
    spacePressed=spacePressed&&!justAttached;
    return {float(k.d)-float(k.a),float(k.w)-float(k.s),spacePressed&&k.s,
        autoMantle&&k.w,spacePressed&&!k.s&&!k.shift&&!wasWallRunning,k.s,k.shift&&!k.s};
}
inline bool entryChord(Keys k) {return k.bindingsMapped?k.entry:k.w&&k.a&&k.d&&k.space;}
inline bool approachIntent(Keys k) {return entryChord(k)&&!k.s&&!k.letGo;}
class WallRunEntryGate {
    bool heldAtEntry{};
public:
    void begin(Keys keys){heldAtEntry=keys.shift;}
    void reset(){heldAtEntry=false;}
    Keys filter(Keys keys){heldAtEntry=heldAtEntry&&keys.shift;keys.shift=keys.shift&&!heldAtEntry;return keys;}
};
inline void keyboardKey(Keys& k,unsigned scan,bool pressed) {
    switch(scan) {
    case 0x11:k.w=pressed;break; case 0x1e:k.a=pressed;break;
    case 0x1f:k.s=pressed;break; case 0x20:k.d=pressed;break;
    case 0x39:k.space=pressed;break;
    }
}
struct ClimbEntryRequest {
    bool requested{},fresh{},began{},airborneAtBegin{};
};
class ClimbEntryIntent {
    bool blocked{},gesture{},originAirborne{};
public:
    void blockUntilRelease(){blocked=true;gesture=originAirborne=false;}
    ClimbEntryRequest sample(Keys k,bool attached=false,bool suspended=false,float=1.f/60,bool confirmedAirborne=false) {
        if(attached||suspended){blockUntilRelease();return {};}
        const bool chord=entryChord(k);
        if(blocked){if(!chord)blocked=false;return {};}
        if(k.s||k.letGo){blockUntilRelease();return {};}
        if(!chord){gesture=originAirborne=false;return {};}
        const bool began=!gesture;
        if(began){gesture=true;originAirborne=confirmedAirborne;}
        return {true,began,began,originAirborne};
    }
    bool waitingForRelease()const{return blocked;}
};
inline bool entryProbeReady(float& remaining,float dt,bool fresh) {
    if(!std::isfinite(dt)||dt<=0)return false;
    remaining=std::max(0.f,remaining-dt);
    if(remaining>0&&!fresh)return false;
    remaining=.08f;return true;
}
class EntryPreparationGrace {
    float remaining{};
    bool gamepad{};
public:
    void arm(bool fromGamepad){remaining=.15f;gamepad=fromGamepad;}
    void cancel(){remaining=0;}
    bool sample(float dt,bool fromGamepad,Keys keys) {
        if(!std::isfinite(dt)||dt<=0||fromGamepad!=gamepad||keys.s||keys.letGo){cancel();return false;}
        remaining=std::max(0.f,remaining-dt);
        return remaining>0;
    }
};
class NativeJumpIntent {
    bool down{};
    float remaining{};
public:
    bool sample(bool held,float dt) {
        if(!held){down=false;remaining=0;return false;}
        if(!down){down=true;remaining=.15f;}
        else if(std::isfinite(dt)&&dt>0)remaining=std::max(0.f,remaining-std::min(dt,.1f));
        return remaining>0;
    }
};

struct GrabFlight {bool airborne{},descending{};float verticalSpeed{};bool confirmedAirborne{};};
inline GrabFlight grabFlight(bool inAir,bool jumping,bool jumpGraph,bool nativeJump,float verticalSpeed) {
    const float speed=std::isfinite(verticalSpeed)?verticalSpeed:0.f;
    const bool air=inAir||jumping||jumpGraph||nativeJump;
    return {air,air&&(speed<-.5f||(inAir&&!jumping&&speed<=.5f)),speed,(inAir||jumping)&&std::isfinite(verticalSpeed)};
}
inline Motion grabEntryMotion(GrabFlight flight) {
    return flight.descending?Motion::ledgeCatch:Motion::jumpCatch;
}
class JumpGrabGate {
    float remaining{};
    Vec direction{0,1,0};
    bool nativeJump{},airRequest{};
public:
    void request(Vec facing={0,1,0},bool alreadyJumped=false,bool airborneRequest=false) {
        facing.z=0;
        if(!facing.finite()||facing.length()<.9f){cancel();return;}
        remaining=.8f;direction=facing.unit();nativeJump=alreadyJumped;airRequest=airborneRequest;
    }
    void hold(Vec facing,bool alreadyJumped,bool airborneRequest,bool fresh) {

        facing.z=0;
        if(!facing.finite()||facing.length()<.9f){cancel();return;}
        nativeJump=alreadyJumped;
        if(fresh)airRequest=airborneRequest;
        else if(!pending())airRequest=false;
        remaining=.8f;direction=facing.unit();
    }
    void cancel(){remaining=0;nativeJump=airRequest=false;}
    void tick(float dt,bool suspended=false){
        if(suspended){cancel();return;}
        if(std::isfinite(dt)&&dt>0){remaining=std::max(0.f,remaining-std::clamp(dt,0.f,.05f));if(remaining<=0)cancel();}
    }
    bool pending()const{return remaining>0;}
    Vec facing()const{return direction;}
    bool startedNativeJump()const{return nativeJump;}
    bool explicitAirCatch(GrabFlight current)const{return pending()&&airRequest&&current.confirmedAirborne;}
    bool permitted(float cooldown,GrabFlight current)const{return pending()&&(cooldown<=0||explicitAirCatch(current));}
};

class SpacePressOwnership {
    bool owned{},native{};
public:
    void reserve(){owned=true;}
    bool startedNativeJump()const{return native;}
    bool filter(bool isDown,bool isUp,bool attached) {
        const bool consume=(owned||attached)&&!(isUp&&native);
        if(isDown&&!consume)native=true;
        if(isUp){owned=native=false;}
        return consume;
    }
    void synchronize(bool held){if(!held)owned=native=false;}
};

template<class Event,class Predicate> void removeInputEvents(Event*& head,Event*& tail,Predicate remove) {
    Event* previous=nullptr;
    for(Event* event=head;event;) {
        Event* next=event->next;
        if(remove(event)) {
            if(previous)previous->next=next;else head=next;
            if(tail==event)tail=previous;
        } else previous=event;
        event=next;
    }
}
}
