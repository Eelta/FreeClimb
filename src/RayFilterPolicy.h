#pragma once
#include <cstdint>
#include <utility>

namespace fc {
enum class RayResponse : std::uint8_t { unknown, simpleContact, reporting, none };
enum class RayDecision : std::uint8_t { keep, ignorePlayer, ignoreTrigger };
struct RayHitFacts {
    bool exactPlayer{},actorZone{},entity{},phantom{},actor{};
    bool primitiveActivatorWithoutModel{};
    RayResponse response=RayResponse::unknown;
};
constexpr RayDecision rayHitDecision(const RayHitFacts& hit) {
    if(hit.exactPlayer)return RayDecision::ignorePlayer;

    if(!hit.actorZone||hit.actor)return RayDecision::keep;
    if(hit.entity&&(hit.response==RayResponse::reporting||hit.response==RayResponse::none))
        return RayDecision::ignoreTrigger;
    if(hit.phantom&&hit.primitiveActivatorWithoutModel)return RayDecision::ignoreTrigger;
    return RayDecision::keep;
}

template<class Accept> RayDecision dispatchRayHit(const RayHitFacts& hit,Accept&& accept) {
    const auto decision=rayHitDecision(hit);
    if(decision==RayDecision::keep)std::forward<Accept>(accept)();
    return decision;
}
}
