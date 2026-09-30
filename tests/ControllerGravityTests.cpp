#include "ControllerGravityLease.h"
#include <cstdlib>
#include <iostream>

namespace {
struct Controller {float gravity=1.f;};
void require(bool ok,const char* message) {
    if(!ok){std::cerr<<message<<'\n';std::exit(1);}
}
using Lease=fc::ControllerGravityLease<Controller>;
}

int main() {

    for(float gravity:{.25f,1.f,1.75f,0.f}) {
        Controller current{gravity};Lease lease;
        lease.take(&current);lease.suppress();
        require(current.gravity==0,"active climbing did not suppress gravity");
        lease.release(&current);
        require(current.gravity==gravity&&!lease.active(),"release lost original gravity");
        current.gravity=.65f;
        for(int i=0;i<120;++i){lease.suppress();lease.release(&current);}
        require(current.gravity==.65f,"idle callbacks modified native gravity");
    }

    {
        Controller old{1.4f};Lease lease;lease.take(&old);lease.suppress();
        Controller replacement{old.gravity};
        const auto result=lease.release(&replacement);
        require(result.owned&&result.replacement,"early replacement was not recognized");
        require(old.gravity==1.4f&&replacement.gravity==1.4f,"early interrupt left zero gravity");
    }

    {
        Controller first{.8f},second{0},third{0};Lease lease;
        lease.take(&first);lease.suppress();lease.take(&second);
        require(first.gravity==.8f&&lease.savedGravity()==.8f,"replacement lost baseline");
        lease.suppress();lease.release(&third);
        require(second.gravity==.8f&&third.gravity==.8f,"replacement chain retained zero");
    }

    {
        Controller old{1},replacement{.4f};Lease lease;
        lease.take(&old);lease.suppress();
        auto result=lease.release(&replacement);
        require(old.gravity==1&&!result.replacement&&replacement.gravity==.4f,"release overwrote external gravity");
        lease.take(&old);lease.suppress();lease.take(&replacement);lease.suppress();lease.release(&replacement);
        require(replacement.gravity==.4f,"observed replacement did not preserve its native value");
    }

    {
        Controller old{1},replacement{0};Lease lease;
        lease.take(&old);lease.release(&replacement);
        require(replacement.gravity==0,"failed acquisition repaired unowned zero");
        lease.take(&old);lease.suppress();old.gravity=2;
        lease.release(&replacement);
        require(old.gravity==2&&replacement.gravity==0,"foreign update was treated as our zero");
        lease.take(&old);lease.suppress();lease.release(nullptr);
        require(old.gravity==2,"null live controller prevented owned restoration");
        lease.release(&replacement);
        require(replacement.gravity==0,"duplicate cleanup changed an unrelated controller");
    }
    std::cout<<"Controller gravity restoration: interrupt/replacement/idle/native ownership passed\n";
}
