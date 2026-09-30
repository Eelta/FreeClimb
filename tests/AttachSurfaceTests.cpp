#include "Core.h"
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
using namespace fc;
static void check(bool value,const std::string& message){if(!value)throw std::runtime_error(message);}
struct Box {Vec low,high;bool climbable=true;};

struct AttachWorld:World {
    std::vector<Box> boxes;Vec origin{};float rotation{};unsigned casts{};
    Vec rotate(Vec p,float a)const{return {p.x*std::cos(a)-p.y*std::sin(a),p.x*std::sin(a)+p.y*std::cos(a),p.z};}
    Vec global(Vec p)const{return origin+rotate(p,rotation);}
    Vec local(Vec p)const{return rotate(p-origin,-rotation);}
    Vec direction(Vec p)const{return rotate(p,rotation);}
    std::optional<Hit> ray(Vec from,Vec to)override {
        ++casts;const auto a=local(from),d=local(to)-a;double nearest=2;std::optional<Hit> result;
        for(const auto& box:boxes) {
            double entry=0,exit=1;Vec normal{};bool valid=true;
            const double p[]{a.x,a.y,a.z},v[]{d.x,d.y,d.z},low[]{box.low.x,box.low.y,box.low.z},high[]{box.high.x,box.high.y,box.high.z};
            for(int axis=0;axis<3;++axis) {
                if(std::abs(v[axis])<1e-10){if(p[axis]<low[axis]||p[axis]>high[axis])valid=false;continue;}
                double begin=(low[axis]-p[axis])/v[axis],end=(high[axis]-p[axis])/v[axis];float sign=-1;
                if(begin>end){std::swap(begin,end);sign=1;}
                if(begin>entry){entry=begin;normal={};if(axis==0)normal.x=sign;else if(axis==1)normal.y=sign;else normal.z=sign;}
                exit=std::min(exit,end);
            }
            if(valid&&entry<=exit&&entry>1e-6&&entry<=1&&entry<nearest) {
                nearest=entry;result=Hit{global(a+d*float(entry)),direction(normal),box.climbable};
            }
        }
        return result;
    }
    bool capsuleClear(Vec point,float radius,float height)const {
        const auto p=local(point);
        for(const auto& box:boxes) {
            if(p.z+height<=box.low.z+.02f||p.z+6>=box.high.z-.02f)continue;
            const float x=std::clamp(p.x,box.low.x,box.high.x),y=std::clamp(p.y,box.low.y,box.high.y);
            if(std::hypot(p.x-x,p.y-y)<radius-.05f)return false;
        }
        return true;
    }
    void validateFront(float centerX=0) {
        const auto hit=ray(global({centerX,-100,70}),global({centerX,100,70}));
        check(hit&&std::abs(local(hit->point).y)<.02f&&direction({0,-1,0}).dot(hit->normal)>.999f,"analytic front point/normal correct");
        check(!ray(global({centerX,20,70}),global({centerX,-100,70})),"inside-to-outside ray not treated as front entry");
    }
};
static const Box wall{{-1000,0,-1000},{1000,100,1000}};
struct Attempt {bool accepted{};float snap{};Vec end{};AttachFailure reason{};unsigned casts{};};
static Attempt attempt(AttachWorld world,Vec feet,float yaw,int fps,bool expected,std::string label) {
    Traversal t;t.cfg.radius=31;t.cfg.gap=37;t.cfg.height=138;
    const auto start=world.global(feet),facing=world.direction({std::sin(yaw),std::cos(yaw),0});
    const bool accepted=t.attach(world,start,facing,1000,35);
    if(accepted!=expected) {
        std::cerr<<label<<" accepted="<<accepted<<" expected="<<expected<<" reason="<<name(t.lastFailure)<<" snap="<<t.lastAttachDistance<<" casts="<<world.casts<<'\n';
    }
    check(accepted==expected,label+" acceptance");
    if(!accepted){check(!t.active(),label+" rejection must retain native movement");return {false,t.lastAttachDistance,t.position,t.lastFailure,world.casts};}
    check(t.lastAttachDistance<=35.0001f,label+" never broadens snap limit");
    check((t.position-start).length()<.001f,label+" does not teleport at attachment");
    check(t.normal.dot(facing)<-.35f,label+" original forward cone remains enforced");
    t.entry(Motion::ledgeCatch,false);
    Vec previous=t.position;
    for(int frame=0;frame<fps&&t.state==State::approach;++frame) {
        const auto result=t.update(world,{},1.f/fps,1000);
        check(!result.released,label+" checked catch remains uninterrupted");
        check(world.capsuleClear(t.position,31,138),label+" catch capsule cannot enter a collider");
        check((t.position-previous).length()<1.6f*35/(fps*t.cfg.approachSeconds)+.05f,
            label+" smooth catch respects the bounded 35-unit approach speed");previous=t.position;
    }
    check(t.state==State::wall,label+" catch reaches stable wall state");
    const Vec endpoint=t.position;
    for(int frame=0;frame<fps/4;++frame) {
        const auto result=t.update(world,{},1.f/fps,1000);
        check(!result.released&&(t.position-endpoint).length()<.02f,label+" actual hand support keeps new catch stable");
    }
    return {true,t.lastAttachDistance,world.local(t.position),t.lastFailure,world.casts};
}
int main(){try {
    constexpr bool newSuccess=true;
    std::cout<<"current bounded forward fan\n";
    int newlyReachable=0,ordinary=0,negative=0;
    constexpr float degree=3.14159265f/180.f;
    for(Vec origin:{Vec{},Vec{109000.125f,72000.0625f,1800.375f}})for(float rotation:{0.f,1.13f})for(int fps:{30,120}) {
        const std::string context=" fps="+std::to_string(fps)+" rotated="+std::to_string(rotation)+" far="+std::to_string(origin.x!=0);
        AttachWorld world;world.origin=origin;world.rotation=rotation;world.boxes={wall};world.validateFront();
        attempt(world,{0,-37,20},0,fps,true,"ordinary facing wall"+context);++ordinary;
        attempt(world,{0,-71.5f,20},0,fps,true,"34.5-unit approach"+context);++ordinary;
        for(float side:{-1.f,1.f}) {
            auto angled=attempt(world,{0,-37,20},side*55*degree,fps,newSuccess,"55-degree oblique wall"+context);newlyReachable+=angled.accepted;
            if(angled.accepted)check(std::abs(angled.end.y+37)<.03f,"oblique wall endpoint stays at true gap");
            AttachWorld pillar=world;pillar.boxes={side>0?Box{{14,0,-1000},{34,100,1000}}:Box{{-34,0,-1000},{-14,100,1000}}};pillar.validateFront(side*24);
            check(!pillar.ray(pillar.global({0,-37,90}),pillar.global({0,73,90})),"central discovery ray truly misses off-axis pillar");
            auto caught=attempt(pillar,{0,-37,20},0,fps,newSuccess,"off-axis x14..34 pillar"+context);newlyReachable+=caught.accepted;
            if(caught.accepted)check(caught.end.x*side>=14&&caught.end.x*side<=34&&std::abs(caught.end.y+37)<.03f,"pillar catch uses actual supporting front face");

            AttachWorld sideBlocked=pillar;
            sideBlocked.boxes.push_back(side>0?Box{{35,-42,85},{40,-32,95},false}:Box{{-40,-42,85},{-35,-32,95},false});
            check(sideBlocked.capsuleClear(sideBlocked.global({0,-37,20}),31,138),"tangent obstacle starts beyond actual capsule");
            attempt(sideBlocked,{0,-37,20},0,fps,false,"solid obstacle beside tangent-aligned destination"+context);++negative;

            attempt(world,{0,-37,20},side*90*degree,fps,false,"exactly sideways wall"+context);++negative;
            attempt(world,{0,-37,20},side*72*degree,fps,false,"outside original facing cone"+context);++negative;
            AttachWorld narrow=world;narrow.boxes={side>0?Box{{14,0,-1000},{22,100,1000}}:Box{{-22,0,-1000},{-14,100,1000}}};
            attempt(narrow,{0,-37,20},0,fps,false,"8-unit wide pillar lacks neighbour grip"+context);++negative;
        }
        attempt(world,{0,-72.25f,20},0,fps,false,"35.25-unit approach remains too far"+context);++negative;
        attempt(world,{0,-200,20},0,fps,false,"wall beyond discovery reach"+context);++negative;
        AttachWorld back=world;back.boxes={{{-1000,-100,-1000},{1000,0,1000}}};
        attempt(back,{0,37,20},0,fps,false,"wall behind actor"+context);++negative;
        AttachWorld blocked=world;blocked.boxes.push_back({{-100,-30,85},{100,-25,95},false});
        check(blocked.capsuleClear(blocked.global({0,-72,20}),31,138),"clearance fixture starts outside the obstruction");
        attempt(blocked,{0,-72,20},0,fps,false,"intervening nonclimbable obstruction"+context);++negative;
        AttachWorld tooNarrow=world;tooNarrow.boxes={{{-4,0,-1000},{4,100,1000}}};
        attempt(tooNarrow,{0,-37,20},0,fps,false,"centered 8-unit ribbon has no grip neighbour"+context);++negative;
    }
    check(newlyReachable==(newSuccess?32:0),"exact new-success count across orientations, frames and far origin");
    std::cout<<"PASS newlyReachable="<<newlyReachable<<" ordinary="<<ordinary<<" negative="<<negative<<" maxSnap=35\n";
    return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
