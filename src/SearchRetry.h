#pragma once
#include <algorithm>
#include <cmath>

namespace fc {
template<class Vector> struct SearchRetry {
    float remaining{};
    Vector position{},normal{},input{};
    unsigned mode{};
    bool valid{},standingPath{};
    void tick(float dt) {
        if(std::isfinite(dt)&&dt>0)remaining=std::max(0.f,remaining-std::min(dt,.05f));
    }
    bool ready(Vector p,Vector n,Vector intent,unsigned nextMode)const {
        return !valid||!std::isfinite(remaining)||remaining<=0||
            !p.finite()||!n.finite()||!intent.finite()||!position.finite()||!normal.finite()||!input.finite()||
            (p-position).length()>.25f||(n-normal).length()>.01f||(intent-input).length()>.01f||mode!=nextMode;
    }
    void defer(Vector p,Vector n,Vector intent,unsigned nextMode,float seconds,bool standing=false) {
        position=p;normal=n;input=intent;mode=nextMode;standingPath=standing;
        valid=p.finite()&&n.finite()&&intent.finite()&&std::isfinite(seconds);
        remaining=valid?std::clamp(seconds,0.f,.25f):0.f;
    }
    void reset(){*this={};}
};
}
