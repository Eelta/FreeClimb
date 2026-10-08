#pragma once

namespace fc {
class TraversalStealth {
#if defined(FREECLIMB_NO_TRAVERSAL_SNEAK) && FREECLIMB_NO_TRAVERSAL_SNEAK
    static constexpr bool enabled=false;
#else
    static constexpr bool enabled=true;
#endif
    const void* owner{};
    bool desired{},managed{};
    template<class Actor> static bool finish(Actor* actor,bool clearSneak=true) {
        if(!actor)return false;
        auto* state=actor->AsActorState();
        if(!state)return false;
        if constexpr(enabled)if(clearSneak) {
            if(state->actorState1.sneaking)state->actorState1.sneaking=false;
            bool graphSneaking{};
            if(actor->GetGraphVariableBool("IsSneaking",graphSneaking)&&graphSneaking)
                actor->SetGraphVariableBool("IsSneaking",false);
        }
        return true;
    }
public:
    static constexpr bool supported(){return enabled;}
    bool active()const{return owner!=nullptr;}
    bool sneaking()const{return active()&&desired;}
    void clear(){owner=nullptr;desired=managed=false;}
    template<class Actor> bool acquire(Actor* actor,bool wallRunning,bool autoSneak=true) {
        if(!actor||actor->IsDead()||(owner&&owner!=actor)||!actor->AsActorState())return false;
        owner=actor;
        return update(actor,wallRunning,autoSneak);
    }
    template<class Actor> bool update(Actor* actor,bool wallRunning,bool autoSneak=true) {
        if(!owner)return false;
        if(!actor||owner!=actor){clear();return false;}
        if(actor->IsDead()){release(actor);return false;}
        auto* state=actor->AsActorState();
        if(!state){clear();return false;}
        desired=enabled&&autoSneak&&!wallRunning;
        if constexpr(enabled) {
            if(autoSneak) {
                managed=true;
                if(static_cast<bool>(state->actorState1.sneaking)!=desired)state->actorState1.sneaking=desired;
            } else if(managed) {finish(actor);managed=false;}
        }
        return true;
    }
    template<class Actor> bool release(Actor* actor) {
        const bool owned=actor&&owner==actor,clearSneak=managed;
        clear();
        return owned&&finish(actor,clearSneak);
    }
    template<class Actor> bool cleanupLoaded(Actor* actor) {
        clear();
        return finish(actor);
    }
};
}
