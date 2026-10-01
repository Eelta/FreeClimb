#include "Pose.h"
#include "CornerTestWorld.h"
#include <iostream>
#include <stdexcept>
#include <string_view>
using namespace fc;
static void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
static void selection(const Library& library,int fps,bool far,bool running,float scale,float top,bool expected) {
    fc_test::CornerWorld world;world.boxes={{{-10000,0,-10000},{10000,1000,top}}};
    if(far){world.origin={134559.219f,36994.7031f,-11691.1309f};world.yaw=.633f;}
    Traversal traversal;traversal.cfg=fc_test::settings();traversal.cfg.runSpeed=379.5f;traversal.cfg.contextScale=scale;
    traversal.cfg.wallRunObstacleJumps=true;
    check(library.configureThreepeat(traversal.cfg),"real pack configures authored mantle reach and duration");
    traversal.cfg.threepeatAnimations=true;
    check(traversal.attach(world,world.global({0,-37,100}),world.direction({0,1,0}),1000),"direct mantle starts on actual wall geometry");
    const Input input{0,1,false,true,false,false,running};const float dt=1.f/fps;
    bool selected=false,completed=false,sawMantle=false;float firstHeight=0,firstPreparation=1;Motion first=Motion::none;unsigned peak=0;
    for(int frame=0;frame<fps*12&&traversal.active();++frame) {
        world.rays=0;const auto result=traversal.update(world,input,dt,1000);peak=std::max(peak,world.rays);
        check(!result.released||result.completed,"matching direct platform route must not detach before completion");
        check(traversal.obstacleJumpCount()==0,"ordinary climbing and running need no obstacle jump to select an authored climb-up");
        if(traversal.state==State::mantle&&!sawMantle) {
            sawMantle=true;first=result.motion;firstHeight=traversal.topHand(0).z-traversal.position.z;firstPreparation=traversal.topPreparation();
            check(first==Motion::contextMantle&&traversal.preciseTopContacts()==expected,"every real platform uses42 and matching geometry selects precise hand contacts");
            check(std::string_view(traversal.topSelectionReason())==(expected?"selected":"height mismatch"),"selection diagnosis distinguishes valid full climb-up from a low incompatible edge");
            if(expected) {
                check(std::abs(firstHeight-traversal.cfg.threepeatMantlePalmHeight*scale)<=24*scale,"first-top discovery preserves authored palm height tolerance");
                check(firstHeight<=traversal.cfg.grip+21.02f,"standing seven-unit offset remains inside the first-discovery upper probe limit");
                if(running)check(firstPreparation<.001f,"ordinary wall-run uses the complete pregrip transition before the authored climb-up");
                for(int hand=0;hand<2;++hand)check(std::abs(world.local(traversal.topHand(hand)).z-top)<.05f,"both chosen palms target the actual platform");
            }
        }
        selected|=result.motion==Motion::contextMantle&&traversal.preciseTopContacts();completed|=result.completed;
    }
    std::cout<<"selection fps="<<fps<<" far="<<far<<" run="<<running<<" scale="<<scale<<" top="<<top<<" expected="<<expected<<" motion="<<int(first)<<" height="<<firstHeight<<" prepare="<<firstPreparation<<" reason="<<traversal.topSelectionReason()<<" complete="<<completed<<" peak="<<peak<<'\n';
    check(sawMantle&&selected==expected&&completed&&!traversal.active(),"direct valid platform completes and restores native movement");
    check(peak<5000,"direct authored mantle discovery remains bounded");
}
int main(int argc,char** argv){try{
    check(argc==2,"provide shipped HKX pack manifest");Library library;check(library.load(argv[1])&&library.hasThreepeat(),"load actual shipped Threepeat HKX family");
    for(int fps:{30,60,120})for(bool far:{false,true})for(bool running:{false,true})for(float scale:{1.f,1.03f})
        for(float top:{300.f,840.f})selection(library,fps,far,running,scale,top,true);
    for(int fps:{30,60,120})for(bool running:{false,true})for(float scale:{1.f,1.03f})selection(library,fps,false,running,scale,200,false);
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}return 0;}
