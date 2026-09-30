#pragma once

namespace fc {

template<class Controller> class ControllerGravityLease {
    Controller* owner{};
    float baseline{};
    bool suppressed{};
    bool ownsZero() const {return owner&&suppressed&&owner->gravity==0.f;}
public:
    struct Release {bool owned{},replacement{};};
    float savedGravity() const {return baseline;}
    bool active() const {return owner!=nullptr;}
    void take(Controller* next) {
        if(next==owner)return;
        const bool inherited=ownsZero()&&next&&next->gravity==0.f;
        const float nextBaseline=inherited?baseline:(next?next->gravity:0.f);
        if(ownsZero())owner->gravity=baseline;
        owner=next;baseline=nextBaseline;suppressed=false;
    }
    void suppress() {
        if(owner){owner->gravity=0.f;suppressed=true;}
    }
    Release release(Controller* current) {
        Release result;

        if(ownsZero()) {
            owner->gravity=baseline;result.owned=true;
            if(current&&current!=owner&&current->gravity==0.f) {
                current->gravity=baseline;result.replacement=true;
            }
        }
        owner=nullptr;baseline=0;suppressed=false;
        return result;
    }
};
}
