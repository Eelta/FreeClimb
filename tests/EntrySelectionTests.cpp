#include "Controls.h"
#include "CornerTestWorld.h"
#include <iostream>
#include <stdexcept>

using namespace fc;
static void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
static fc_test::CornerWorld wall() {
    fc_test::CornerWorld value;value.boxes.push_back({{-5000,0,-2000},{5000,400,8000}});return value;
}
static Traversal traversal() {
    Traversal value;value.cfg=fc_test::settings();value.cfg.approachSeconds=.36f;value.cfg.contextActions=false;
    value.cfg.automaticClimbActions=false;return value;
}
static void source(Traversal& value,Motion motion,bool blocked,float seconds=.8f) {
    auto data=value.cfg.authoredMotions?std::make_shared<std::array<AuthoredMotion,42>>(*value.cfg.authoredMotions):std::make_shared<std::array<AuthoredMotion,42>>();
    auto& clip=(*data)[int(motion)-1];clip.enabled=true;clip.seconds=seconds;clip.trajectory.count=3;
    clip.trajectory.knots[0]={0,{}};clip.trajectory.knots[1]={.5f,{0,blocked?160.f:-60.f,50}};clip.trajectory.knots[2]={1,{0,0,100}};
    value.cfg.authoredMotions=std::move(data);
}
static void selection() {
    for(int nearAuthored=0;nearAuthored<2;++nearAuthored) {
        auto world=wall();auto value=traversal();source(value,Motion::jumpCatch,true);if(nearAuthored)source(value,Motion::reach,false);
        require(value.attach(world,{0,-55,200},{0,1,0},1000,40,false,true,Motion::jumpCatch,Motion::reach),"a blocked unrelated jump source cannot veto the selected near reach");
        require(value.entrySelection()==Motion::reach,"the near candidate selects its final actual entry before preflight");
        const auto checked=value;const auto count=world.rays;
        require(value.entry(world,value.entrySelection(),true,&checked),"selected near entry remains valid after pose preparation");
        require(world.rays==count,"unchanged selected source path is not fully probed twice");
        while(value.state==State::approach) {
            const auto result=value.update(world,{},1.f/60,1000);
            require(!result.released&&result.motion==Motion::reach,"selected near reach follows its validated complete clock");
            require((value.position-value.entryPathPoint(value.reachProgress())).length()<.002f,"selected near reach uses the exact preflighted route");
        }
        require(value.reachProgress()==1,"near source reaches its last frame");
    }
    {
        auto world=wall();auto value=traversal();source(value,Motion::reach,true);source(value,Motion::jumpCatch,false);
        const auto selected=grabEntryMotion(grabFlight(false,false,false,false,0),280);
        require(selected==Motion::jumpCatch&&value.attach(world,{0,-80,200},{0,1,0},1000,60,false,true,selected),"fast entry uses the checked climbing catch without sprint styling");
        require(value.entrySelection()==Motion::jumpCatch,"fast entry preserves the standard catch route");
    }
    {
        auto world=wall();auto value=traversal();source(value,Motion::reach,true);source(value,Motion::jumpCatch,false);
        require(!value.attach(world,{0,-55,200},{0,1,0},1000,40,false,true,Motion::jumpCatch,Motion::reach),"a blocked actually selected near source is rejected before control ownership");
        require(!value.active(),"failed selected entry cannot acquire the wall");
    }
    {
        auto world=wall();auto value=traversal();source(value,Motion::reach,false);source(value,Motion::jumpCatch,true);
        require(!value.attach(world,{0,-105,200},{0,1,0},1000,90,false,true,Motion::jumpCatch,Motion::reach),"far entry cannot silently substitute a near reach to evade its blocked source path");
    }
    for(auto selected:{Motion::jumpCatch,Motion::ledgeCatch}) {
        auto world=wall();auto value=traversal();source(value,Motion::reach,true);source(value,selected,false);
        require(value.attach(world,{0,-55,200},{0,1,0},1000,40,true,false,selected),"airborne entry retains its own catch despite a blocked unrelated reach");
        require(value.entrySelection()==selected,"airborne catch never uses the grounded near override");
    }
}
static void proof() {
    for(int changed=0;changed<10;++changed) {
        auto world=wall();auto value=traversal();source(value,Motion::reach,false);
        require(value.attach(world,{0,-60,200},{0,1,0},1000,60,false,true,Motion::reach),"proof fixture has a fully checked selected source");
        const auto checked=value;
        if(changed==1)value.cfg.radius+=1;
        if(changed==2)value.cfg.height+=1;
        if(changed==3)value.cfg.chest+=1;
        if(changed==4)value.cfg.contextScale+=.01f;
        if(changed==5)source(value,Motion::reach,false,.81f);
        if(changed==6)value.normal=Vec{.01f,-1,0}.unit();
        if(changed==7)value.position.x+=.01f;
        if(changed==8)value.cfg.gap+=1;
        if(changed==9)source(value,Motion::reach,true);
        const auto before=world.rays;const bool accepted=value.entry(world,Motion::reach,true,&checked);
        require(changed==9?!accepted:accepted,"actual source obstruction remains rejected after preparation changes");
        require(changed?world.rays>before:world.rays==before,"same-update proof is reused only with exactly matching path and collision dimensions");
    }
    {
        auto world=wall();auto value=traversal();value.cfg.approachSeconds=0;source(value,Motion::reach,false);
        require(value.attach(world,{0,-60,200},{0,1,0},1000,60,false,false,Motion::reach),"instant base approach still fully checks the authored path");
        const auto checked=value;const auto before=world.rays;
        require(value.entry(world,Motion::reach,false,&checked)&&value.state==State::approach&&value.reachProgress()==0&&world.rays==before,"proof reuse retains full authored entry initialization when base approach duration is zero");
    }
}
int main()try {selection();proof();std::cout<<"Final-slot entry selection, mixed sources, same-update path proof and live routes passed\n";return 0;}
catch(const std::exception& failure){std::cerr<<failure.what()<<'\n';return 1;}
