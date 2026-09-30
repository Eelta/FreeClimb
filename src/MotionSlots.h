#pragma once
#include "Core.h"
#include <string_view>

namespace fc {
inline constexpr std::array<std::string_view,motionCount> motionSlotNames{{
    "hang","up","down","left","right","","","reach",
    "hopLeft","hopRight","hopUp","","","","drop",
    "jumpCatch","sprintCatch","dropBack","ledgeCatch","runUp","runLeft",
    "runRight","runDiagonalLeft","runDiagonalRight","runLaunch","runCatch",
    "kickUp","kickLeft","kickRight","flipUp","flipLeft","flipRight",
    "","runLaunchLeft","runLaunchRight","sideBrace","backFlipOut",
    "","contextHang","contextHopLeft","contextHopRight","contextMantle"
}};
inline constexpr auto activeMotions=[] {
    std::array<Motion,activeMotionCount> motions{};
    std::size_t count=0;
    for(int id=1;id<=motionCount;++id)if(isActiveMotion(Motion(id)))motions[count++]=Motion(id);
    return motions;
}();
static_assert([] {
    std::size_t count=0;
    for(int id=1;id<=motionCount;++id) {
        const bool active=isActiveMotion(Motion(id));
        if(active!=!motionSlotNames[id-1].empty())return false;
        if(active)++count;
    }
    return count==activeMotionCount&&int(activeMotions.front())==1&&int(activeMotions.back())==motionCount;
}());
}
