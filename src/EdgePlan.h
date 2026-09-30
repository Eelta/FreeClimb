#pragma once

struct EdgePreparation {
    bool active{};
    Motion motion=Motion::none;
    Vec from{},to{},destination{},direction{};
    GripEdge source{},target{};
    float settle{};
    unsigned status{};
};

inline bool capturedSideIntent(Vec direction) {
    return direction.finite()&&std::abs(direction.x)>.1f&&direction.y>=0&&
        direction.y<=std::abs(direction.x)*1.15f;
}
inline float capturedSideRise(Vec direction,float sideDistance,float scale) {
    return direction.y>0?std::min(sideDistance*direction.y/std::abs(direction.x),32.f*scale):0.f;
}
inline bool capturedSideRouteMatches(Vec direction,Vec displacement,Vec outward,float scale) {
    const Vec side{-outward.y,outward.x,0};
    const Vec route{displacement.dot(side),displacement.z,0};

    return capturedSideIntent(direction)&&route.x*direction.x>0&&
        (direction.y<=0||(route.y>=0&&route.y<=32.f*scale+.05f&&
            route.unit().dot(direction.unit())>=.85f));
}
inline const char* edgePlanReason(unsigned status) {
    switch(status) {
    case 0:return "idle";
    case 1:return "no source edge in acquisition range";
    case 2:return "no compatible destination edge";
    case 3:return "source alignment blocked";
    case 4:return "destination support rejected";
    case 5:return "braced foot support missing";
    case 6:return "edge action body route blocked";
    case 7:return "aligning body to source edge";
    case 8:return "settling source grip";
    case 9:return "measured edge action committed";
    case 10:return "edge preparation cancelled by input";
    case 11:return "prepared edge geometry changed";
    }
    return "unknown edge preparation status";
}
