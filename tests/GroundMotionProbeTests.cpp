#include "GroundMotionProbe.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
void check(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main(){try {
    for(int fps:{30,60,120}) {
        GroundMotionProbe gate;Vec p{};int reports=0;
        for(int i=0;i<fps*3;++i){p.x+=30.f/fps;reports+=gate.sample(p,{0,1,0},1.f/fps,true);}
        check(!reports,"ordinary sideways sliding must not be diagnosed as a stop");
        for(int i=0;i<fps;++i)reports+=gate.sample(p,{0,1,0},1.f/fps,true);
        check(reports==1,"held movement with native position stopped is observed");
        for(int i=0;i<fps*4;++i)reports+=gate.sample(p,{0,1,0},1.f/fps,true);
        check(reports==1,"repeated blocked samples must obey cooldown");
        for(int i=0;i<fps;++i)reports+=gate.sample(p,{0,1,0},1.f/fps,true);
        check(reports==2,"persistent native stall receives bounded follow-up");
        gate.suspend();
        for(int i=0;i<fps*10;++i)check(!gate.sample(p,{0,1,0},1.f/fps,false),"attached/menu output excluded");
        for(int i=0;i<fps*10;++i)check(!gate.sample(p,{},1.f/fps,true),"no movement request excluded");
        for(int i=0;i<fps*10;++i)check(!gate.sample(p,{0,1,0},0,true),"zero-time callbacks cannot trigger diagnostics");
        for(int i=0;i<fps*2;++i){p.z-=40.f/fps;check(!gate.sample(p,{0,1,0},1.f/fps,true),"falling is native motion");}
        for(int i=0;i<fps*2;++i){p.x+=100;check(!gate.sample(p,{0,1,0},1.f/fps,true),"teleports reset evidence");}
        gate.suspend();
        for(int i=0;i<fps*2;++i)check(!gate.sample(p,i%2?Vec{1,0,0}:Vec{0,1,0},1.f/fps,true),"rapid direction changes reset evidence");
    }
    GroundMotionProbe bounded;int reports=0;
    for(int i=0;i<60*600;++i)reports+=bounded.sample({}, {0,1,0},1.f/60,true);
    check(reports==64,"total diagnostic count bounded per game session");
    std::cout<<"Native-ground diagnostic gate: 30/60/120 FPS, movement, pause and rate limits passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
