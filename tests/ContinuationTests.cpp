#include "Pose.h"
#include <stdexcept>
#include <iostream>
using namespace fc;
static void check(bool value,const char* message) {if(!value)throw std::runtime_error(message);}
int main() {

    for(float fps:{30.f,60.f,120.f}) {
        const float dt=1/fps;
        Pose old(1),current(1),target(1);
        old[0].q=Quat::axis({0,0,1},.5f-dt*2);current[0].q=Quat::axis({0,0,1},.5f);
        old[0].t={10-dt*50,0,0};current[0].t={10,0,0};target[0].q=Quat::axis({0,0,1},1.0f);
        PoseContinuation transfer;transfer.begin(current,old,dt);
        const auto first=transfer.sample(.0001f);
        check((first[0].t-current[0].t).x/.0001f>49,"outgoing linear motion retains its initial derivative");
        check(angleBetween(current[0].q,first[0].q)/.0001f>1.95f,"outgoing angular motion retains its initial derivative");
        const auto start=transfer.sample(0),settled=transfer.sample(2);
        check((start[0].t-current[0].t).length()<1e-6f&&angleBetween(start[0].q,current[0].q)<1e-6f,"a new transition starts at the displayed pose");
        check((settled[0].t-current[0].t).length()<3.26f&&angleBetween(current[0].q,settled[0].q)<.131f,"short anticipation cannot extrapolate the limb indefinitely");
        const auto middle=transfer.sample(.07f);
        transfer.begin(middle,current,.07f);
        check(angleBetween(transfer.sample(0)[0].q,middle[0].q)<1e-6f,"an interrupted crossfade starts at its visible pose");
    }
    Pose a(1),b(1);a[0].q=Quat::axis({0,0,1},3.13f);b[0].q=Quat::axis({0,0,1},-3.13f);
    PoseContinuation crossing;crossing.begin(b,a,1.f/60);
    check(angleBetween(crossing.sample(.05f)[0].q,b[0].q)<.1f,"quaternion sign/wrap does not cause a full rotation");
    crossing.begin(b,{},0);
    check(angleBetween(crossing.sample(.1f)[0].q,b[0].q)<1e-6f,"missing history has no invented velocity");
    std::cout<<"Pose continuation preserves outgoing velocity, bounded prediction and interruptions\n";
}
