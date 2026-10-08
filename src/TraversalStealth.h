#pragma once

namespace fc {
class TraversalStealth {
    const void* owner{};
    bool desired{};
    template<class Actor> static bool finish(Actor* actor) {
        if(!actor)return false;
        auto* state=actor->AsActorState();
        if(!state)return false;
        if(state->actorState1.sneaking)state->actorState1.sneaking=false;
        bool graphSneaking{};
        if(actor->GetGraphVariableBool("IsSneaking",graphSneaking)&&graphSneaking)
            actor->SetGraphVariableBool("IsSneaking",false);
        return true;
    }
public:
    bool active()const{return owner!=nullptr;}
    bool sneaking()const{return active()&&desired;}
    void clear(){owner=nullptr;desired=false;}
    template<class Actor> bool acquire(Actor* actor,bool wallRunning) {
        if(!actor||actor->IsDead()||(owner&&owner!=actor)||!actor->AsActorState())return false;
        owner=actor;
        return update(actor,wallRunning);
    }
    template<class Actor> bool update(Actor* actor,bool wallRunning) {
        if(!owner)return false;
        if(!actor||owner!=actor){clear();return false;}
        if(actor->IsDead()){release(actor);return false;}
        auto* state=actor->AsActorState();
        if(!state){clear();return false;}
        desired=!wallRunning;
        if(static_cast<bool>(state->actorState1.sneaking)!=desired)state->actorState1.sneaking=desired;
        return true;
    }
    template<class Actor> bool release(Actor* actor) {
        const bool owned=actor&&owner==actor;
        clear();
        return owned&&finish(actor);
    }
    template<class Actor> bool cleanupLoaded(Actor* actor) {
        clear();
        return finish(actor);
    }
};
}
