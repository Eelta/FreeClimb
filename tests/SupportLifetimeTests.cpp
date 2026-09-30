#include "Core.h"
#include "CornerTestWorld.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
using namespace fc_test;
static void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
int main(){try{
    unsigned cases=0;
    for(int fps:{30,60,120})for(bool upward:{false,true}) {
        CornerWorld world;world.boxes={{{-400,0,-400},{400,400,400}}};
        Traversal traversal;traversal.cfg=settings();
        check(traversal.attach(world,{0,-37,0},{0,1,0},1000),"real wall fixture attaches");

        world.boxes={{{-400,0,122},{400,400,180}}};
        Input input;input.y=upward?1.f:0.f;const Vec start=traversal.position;
        bool released=false;float elapsed=0;
        for(int frame=0;frame<fps*2&&traversal.active();++frame) {
            const auto result=traversal.update(world,input,1.f/fps,10);
            elapsed+=1.f/fps;released|=result.released;
            if(result.motion!=Motion::hang&&!result.released)std::cerr<<"fps="<<fps<<" up="<<upward<<" frame="<<frame<<" motion="<<int(result.motion)<<" reason="<<traversal.blockedReason<<" z="<<traversal.position.z<<'\n';
            check(result.motion==Motion::hang||result.released,"displaced support never produces climbing in empty space");
            check(std::abs(traversal.position.z-start.z)<.001f,"nearby patch does not manufacture upward travel");
        }
        check(released&&!traversal.active()&&elapsed>=1.19f&&elapsed<1.26f,
            "upper retry expires after the same finite grace as lower retries");
        ++cases;
        world.boxes={{{-400,0,-400},{400,400,400}}};
        traversal.reset();check(traversal.attach(world,{0,-37,0},{0,1,0},1000),"recovery fixture attaches");
        world.boxes={{{-400,0,122},{400,400,180}}};
        for(int frame=0;frame<fps/2;++frame)check(!traversal.update(world,{},1.f/fps,1000).released,"brief seam loss retains hang grace");
        world.boxes={{{-400,0,-400},{400,400,400}}};
        for(int frame=0;frame<fps*2;++frame)check(!traversal.update(world,{},1.f/fps,1000).released,"actual recovered grips renew support");
        check(traversal.active(),"live restored support remains attached");++cases;
    }
    std::cout<<"PASS SupportLifetimeTests cases="<<cases<<'\n';return 0;
}catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}}
