#include "Core.h"
namespace fc {
#include "CornerTraversal.h"
}
#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include "CornerTestWorld.h"
using namespace fc;
namespace {
using namespace fc_test;
bool noExtraClear(Vec,Vec){return true;}
unsigned failures{},positives{},negativeGroups{};
void require(bool condition,const char* message){if(!condition){++failures;std::cerr<<"FAIL "<<message<<'\n';}}
void positive(bool convex,int fps,float sign,float yaw,bool far,bool running) {
    CornerWorld world;world.corner(convex);world.mirror=sign;world.yaw=yaw;
    if(far)world.origin={131065.836f,41791.117f,-11204.176f};
    const auto cfg=settings();const Vec start=world.global({convex?-45.f:-85.f,-37,0});
    auto route=findCornerRoute(world,cfg,start,world.direction({0,-1,0}),sign,noExtraClear);
    require(route.has_value(),"real right-angle walls produce an attached lateral route");if(!route)return;
    require(route->convex==convex&&route->count<=CornerRoute::capacity,"planned route preserves actual corner type and bounded knots");
    const unsigned planning=world.rays;float minClear=10000,maxYaw=0,maxStep=0;Vec previous=start,previousNormal=world.direction({0,-1,0});
    bool completed=false,failed=false;unsigned peak=0;
    const float speed=cornerSpeedLimit(*route,running?cfg.runSpeed:cfg.sideSpeed);
    for(int frame=0;frame<fps*5;++frame) {
        world.rays=0;
        auto step=advanceCornerRoute(world,cfg,*route,speed/fps,noExtraClear);
        peak=std::max(peak,world.rays);
        if(!step){failed=true;break;}

        for(unsigned sample=0;sample<=32;++sample)minClear=std::min(minClear,world.clearance(previous+(step->position-previous)*(float(sample)/32),cfg));
        maxYaw=std::max(maxYaw,std::acos(std::clamp(previousNormal.dot(step->normal),-1.f,1.f)));
        maxStep=std::max(maxStep,(step->position-previous).length());previous=step->position;previousNormal=step->normal;
        if(step->complete){completed=true;break;}
    }
    ++positives;
    const Vec finish=world.local(previous);
    std::cout<<"corner "<<(convex?"outer":"inner")<<" fps="<<fps<<" side="<<sign<<" yaw="<<yaw<<" far="<<far<<" run="<<running
        <<" p="<<finish.x<<','<<finish.y<<" clear="<<minClear<<" yawStep="<<maxYaw<<" moveStep="<<maxStep<<" rays="<<planning<<'/'<<peak<<'\n';
    require(!failed&&completed,"corner can finish without a support loss or airborne action");
    require(minClear>=cfg.radius-.03f,"independent full cylinder distance remains outside solid corner walls");
    require(maxStep<=speed/fps+.04f,"continuous corner movement is bounded by actual requested speed");
    require(maxYaw<8.1f/fps+.002f,"turn is bounded at eight radians per second instead of a ninety-degree snap");
    require(previousNormal.dot(world.direction(convex?Vec{1,0,0}:Vec{-1,0,0}))>.999f,"end heading follows the actual adjacent wall");
    require(convex?finish.x>36&&finish.y>17:finish.x<-36&&finish.y<-55,"route reaches the correct destination side");
    require(planning<2048&&peak<512,"corner planning/playback keep bounded ray budgets");
}
}
int main() {
    for(bool convex:{false,true})for(int fps:{30,60,120})for(float sign:{-1.f,1.f})for(float yaw:{0.f,.73f})for(bool far:{false,true})for(bool running:{false,true})
        positive(convex,fps,sign,yaw,far,running);
    const auto cfg=settings();
    for(bool convex:{false,true})for(float residual:{-.50f,-.35f,-.08f,-.03f,-.01f,-.005f,.005f,.01f,.03f,.08f,.35f,.50f})
      for(bool slopeResidual:{false,true})for(float side:{-1.f,1.f})for(bool far:{false,true}) {
        CornerWorld world;world.corner(convex);world.mirror=side;world.yaw=.73f;
        if(far)world.origin={131065.836f,41791.117f,-11204.176f};
        const Vec real=world.direction({0,-1,0});
        const Vec stale=world.direction(slopeResidual?Vec{0,-1,residual}.unit():cornerRotate({0,-1,0},residual));
        const Vec start=world.global({convex?-45.f:-85.f,-37,0});
        auto route=findCornerRoute(world,cfg,start,stale,side,noExtraClear);
        if(!route)std::cerr<<"residual="<<residual<<" slope="<<slopeResidual<<" convex="<<convex<<" side="<<side<<" far="<<far<<'\n';
        require(route.has_value(),"smoothed-facing residual cannot replace the real source plane or prevent an otherwise valid corner");
        if(!route)continue;
        require(route->sourceNormal.dot(real)>.999999f,"stored source plane is the measured wall beside the seam");
        float minimum=10000;bool completed=false;Vec previous=start;
        for(unsigned frame=0;frame<240;++frame) {
            const auto step=advanceCornerRoute(world,cfg,*route,cornerSpeedLimit(*route,330)/60,noExtraClear);
            require(step.has_value(),"recalibrated turn retains original live join/grip checks");if(!step)break;
            for(unsigned sample=0;sample<=8;++sample)minimum=std::min(minimum,world.clearance(previous+(step->position-previous)*(float(sample)/8),cfg));
            previous=step->position;if(step->complete){completed=true;break;}
        }
        require(completed&&minimum>=cfg.radius-.03f,"recalibrated inner/outer turn completes without shrinking full-body clearance");++positives;
      }
    for(bool convex:{false,true})for(float degrees:{75.f,85.f,95.f,105.f})for(float side:{-1.f,1.f}) {
        CornerWorld world;world.yaw=.73f;world.mirror=side;world.origin={131065.836f,41791.117f,-11204.176f};
        const float angle=degrees*.01745329252f*(convex?1.f:-1.f);
        const Vec targetNormal=cornerRotate({0,-1,0},angle),targetTravel{-targetNormal.y,targetNormal.x,0};
        if(convex)world.footprints.push_back({{-400,0,0},{0,0,0},targetTravel*400,{-400,400,0}});
        else {world.corner(false);world.boxes.resize(1);world.footprints.push_back({{0,0,0},targetTravel*400,targetTravel*400-targetNormal*400,targetNormal*-400});}
        auto route=findCornerRoute(world,cfg,world.global({convex?-45.f:-85.f,-37,0}),world.direction({0,-1,0}),side,noExtraClear);
        require(route.has_value(),"measured75..105degree corners do not rely on exact world-axis normals");if(!route)continue;
        float minimum=10000;bool complete=false,valid=true;
        for(unsigned frame=0;frame<180;++frame) {
            const auto step=advanceCornerRoute(world,cfg,*route,cornerSpeedLimit(*route,330)/60,noExtraClear);
            if(!step){valid=false;break;}
            minimum=std::min(minimum,world.clearance(step->position,cfg));
            if(step->complete){complete=true;break;}
        }
        require(valid&&complete,"nonorthogonal joined planes retain real hand support through the whole turn");
        require(minimum>=cfg.radius-.03f,"independent polygon perimeter distance proves body clearance at nonorthogonal corners");++positives;
    }
    for(bool convex:{false,true}) {
        CornerWorld world;world.corner(convex);auto route=findCornerRoute(world,cfg,{convex?-45.f:-85.f,-37,0},{0,-1,0},1,noExtraClear);
        require(route.has_value(),"pause and reverse fixture plans real corner");if(!route)continue;
        for(unsigned frame=0;frame<30;++frame)require(advanceCornerRoute(world,cfg,*route,1.f,noExtraClear).has_value(),"partial turn remains supported");
        const auto paused=cornerSample(*route,route->distance);const float before=route->distance;
        for(unsigned frame=0;frame<20;++frame) {
            const auto still=advanceCornerRoute(world,cfg,*route,0,noExtraClear);
            require(still&&(still->position-paused.position).length()<.001f&&route->distance==before,"released movement keys retain one stable physical hang");
        }
        bool returned=false;
        for(unsigned frame=0;frame<90;++frame) {const auto back=advanceCornerRoute(world,cfg,*route,-1,noExtraClear);require(back.has_value(),"opposite lateral input reverses checked route");if(!back)break;if(back->reversed){returned=true;break;}}
        require(returned&&(cornerSample(*route,route->distance).position-route->points[0]).length()<.001f,"reversal reaches the original supported side");
        ++positives;
        route->distance=route->length*.55f;const auto old=cornerSample(*route,route->distance);
        for(unsigned frame=0;frame<40;++frame)require(shiftCornerRoute(world,cfg,*route,1,noExtraClear).has_value(),"W can climb upward while partway around a genuine corner");
        const auto upper=cornerSample(*route,route->distance);
        require(std::abs(upper.position.z-old.position.z-40)<.001f&&std::abs(upper.position.x-old.position.x)<.001f&&std::abs(upper.position.y-old.position.y)<.001f,
            "vertical climbing preserves lateral corner progress instead of projecting across walls");
        for(unsigned frame=0;frame<40;++frame)require(shiftCornerRoute(world,cfg,*route,-1,noExtraClear).has_value(),"S can descend while partway around a genuine corner");
        require((cornerSample(*route,route->distance).position-old.position).length()<.001f,"up/down corner climbing returns to the same supported route");++positives;

        route->distance=route->length*.4f;const auto point=world.local(cornerSample(*route,route->distance).position);
        world.boxes.push_back({point+Vec{-3,-80,45},point+Vec{3,80,55},false});
        const float saved=route->distance;require(!advanceCornerRoute(world,cfg,*route,4,noExtraClear)&&route->distance==saved,"new obstruction cannot be crossed or advance route state");++negativeGroups;
    }
    {
        CornerWorld world;world.corner(true);world.boxes[0].high.y=.35f;
        require(!findCornerRoute(world,cfg,{-45,-37,0},{0,-1,0},1,noExtraClear),"a thin plate lacking a destination support face cannot turn");++negativeGroups;
    }
    {
        CornerWorld world;world.boxes={{{-400,0,-400},{400,400,400}}};
        require(!findCornerRoute(world,cfg,{-85,-37,0},{0,-1,0},1,noExtraClear)&&world.rays<=2,"flat walls avoid contextual work");++negativeGroups;
    }
    {
        CornerWorld world;world.corner(true);world.boxes.push_back({{4,3,-400},{14,70,400},false});
        require(!findCornerRoute(world,cfg,{-45,-37,0},{0,-1,0},1,noExtraClear),"obstruction near the outside destination rejects the whole planned route");++negativeGroups;
    }
    {
        CornerWorld world;world.boxes={{{-400,0,-400},{400,400,400}},{{0,-200,-400},{80,-12,400}}};
        require(!findCornerRoute(world,cfg,{-85,-37,0},{0,-1,0},1,noExtraClear),"a separate nearby face with a physical gap is not a joined inside corner");++negativeGroups;
    }
    {
        CornerWorld world;world.corner(true);auto route=findCornerRoute(world,cfg,{-45,-37,0},{0,-1,0},1,noExtraClear);
        require(route.has_value(),"removed support fixture initially valid");if(route){world.boxes[0].high.y=.1f;require(!advanceCornerRoute(world,cfg,*route,1,noExtraClear),"a disappearing destination face invalidates playback immediately");}++negativeGroups;
    }
    std::cout<<"CornerTraversalTests positives="<<positives<<" negativeGroups="<<negativeGroups<<" failures="<<failures<<'\n';
    return failures?1:0;
}
