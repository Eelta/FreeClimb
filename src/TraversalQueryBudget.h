#pragma once
#include <optional>

namespace fc {
template<class BaseWorld,class Vector,class CollisionHit,class Motion>
class TraversalQueryBudget final:public BaseWorld {
public:
    static constexpr unsigned authoredLimit=16384;
    BaseWorld& source;
    unsigned calls{},bodyCalls{},limit{};
    bool exhausted{};
    explicit TraversalQueryBudget(BaseWorld& world,unsigned maximum=authoredLimit):source(world),limit(maximum){}
    bool reserve() {
        if(calls+bodyCalls>=limit){exhausted=true;return false;}
        return true;
    }
    std::optional<CollisionHit> ray(Vector from,Vector to) override {
        if(!reserve())return CollisionHit{from,(from-to).unit(),false};
        ++calls;return source.ray(from,to);
    }
    bool actionBodyClear(Motion motion,Vector from,Vector to,float begin,float end,Vector normal) override {
        if(!reserve())return false;
        ++bodyCalls;return source.actionBodyClear(motion,from,to,begin,end,normal);
    }
    bool actionBodyPathClear(Motion motion,Vector from,Vector to,float begin,float end,Vector normal,Vector middle) override {
        if(!reserve())return false;
        ++bodyCalls;return source.actionBodyPathClear(motion,from,to,begin,end,normal,middle);
    }
};
}
