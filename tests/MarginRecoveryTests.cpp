#include "Core.h"
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
using namespace fc;

namespace fc {class TraversalCapture {public:static void seedMargin(Traversal& t){t.clearanceMargin=14;}};}

struct MarginWorld:World {
    struct Box{Vec lo,hi;};std::vector<Box> boxes;
    Vec origin{};float yaw{};unsigned calls{};
    Vec rotate(Vec p,float a)const{return {p.x*std::cos(a)-p.y*std::sin(a),p.x*std::sin(a)+p.y*std::cos(a),p.z};}
    Vec global(Vec p)const{return origin+rotate(p,yaw);}
    Vec local(Vec p)const{return rotate(p-origin,-yaw);}
    void scene(float setback=16.34f) {
        boxes={{{-500,0,-500},{500,1.5f,70.5f}},{{-500,setback,70.5f},{500,setback+110,84.103f}}};
    }
    std::optional<Hit> ray(Vec from,Vec to)override {
        ++calls;const auto a=local(from),d=local(to)-a;float nearest=2;std::optional<Hit> result;
        for(auto box:boxes) {
            const float aa[]{a.x,a.y,a.z},dd[]{d.x,d.y,d.z},lo[]{box.lo.x,box.lo.y,box.lo.z},hi[]{box.hi.x,box.hi.y,box.hi.z};
            float enter=0,leave=1;Vec n{};bool miss=false;
            for(int axis=0;axis<3;++axis) {
                if(std::abs(dd[axis])<1e-6f){if(aa[axis]<lo[axis]||aa[axis]>hi[axis])miss=true;continue;}
                float first=(lo[axis]-aa[axis])/dd[axis],last=(hi[axis]-aa[axis])/dd[axis],sign=-1;
                if(first>last){std::swap(first,last);sign=1;}
                if(first>enter){enter=first;n={};if(axis==0)n.x=sign;else if(axis==1)n.y=sign;else n.z=sign;}
                leave=std::min(leave,last);
            }
            if(!miss&&enter<=leave&&enter>.000001f&&enter<=1&&enter<nearest){nearest=enter;result=Hit{global(a+d*enter),rotate(n,yaw),true};}
        }
        return result;
    }
    float capsuleDistance(Vec feet,float radius,float height)const {
        const auto p=local(feet);float distance=1e9f;
        for(auto box:boxes) {
            const float x=std::max({box.lo.x-p.x,0.f,p.x-box.hi.x});
            const float y=std::max({box.lo.y-p.y,0.f,p.y-box.hi.y});
            const float z=std::max({box.lo.z-(p.z+height-radius),0.f,p.z+radius-box.hi.z});
            distance=std::min(distance,std::sqrt(x*x+y*y+z*z));
        }
        return distance;
    }
};
static int failures=0;
static void check(bool v,const char* why){if(!v){++failures;std::cerr<<"FAIL "<<why<<'\n';}}
static Traversal attach(MarginWorld& w) {
    Traversal t;t.cfg.approachSeconds=0;t.cfg.gap=37;t.cfg.radius=31;t.cfg.height=138;
    check(t.attach(w,w.global({0,-37,0}),w.rotate({0,1,0},w.yaw),1000,35),"the actual lower wall supports an ordinary initial grab");return t;
}
static Traversal recordedMargin(MarginWorld& w) {
    Traversal t;t.cfg.approachSeconds=0;t.cfg.gap=37;t.cfg.radius=31;t.cfg.height=138;
    t.state=State::wall;t.position=w.global({0,-51,0});t.normal=w.rotate({0,-1,0},w.yaw);t.surfaceNormal=t.normal;
    TraversalCapture::seedMargin(t);return t;
}
static Input up(){Input in;in.y=1;in.mantle=true;return in;}

static void recoverAndTop(bool far,int fps,float yaw) {
    MarginWorld w;w.yaw=yaw;if(far)w.origin={131439.671875f,37265.72265625f,-11866.0439453125f};w.scene();
    auto t=recordedMargin(w);bool completed=false,released=false;float maximumMargin=14,minDistance=1e9f,peakRecovery=0;int recoveries=0;unsigned peakCalls=0;
    for(int frame=0;frame<fps*7;++frame) {
        const auto before=w.local(t.position);const auto beforeState=t.state;w.calls=0;
        const auto r=t.update(w,up(),1.f/fps,1000);const auto p=w.local(t.position);
        peakCalls=std::max(peakCalls,w.calls);minDistance=std::min(minDistance,w.capsuleDistance(t.position,t.cfg.radius,t.cfg.height));
        if(beforeState==State::wall&&t.state==State::wall) {
            maximumMargin=std::max(maximumMargin,-37-p.y);
            if(std::abs(p.z-before.z)<.04f&&p.y-before.y>.06f){++recoveries;peakRecovery=std::max(peakRecovery,p.y-before.y);}
            check(p.y<=-36.94f,"ordinary recovery never consumes nominal gap or invents extra inward reach");
        }
        if(r.completed){completed=true;break;}
        if(r.released){released=true;break;}
    }
    std::cout<<"margin far="<<far<<" fps="<<fps<<" yaw="<<yaw<<" maximum="<<maximumMargin<<" recoveries="<<recoveries<<" maxRecovery="<<peakRecovery<<" capsule="<<minDistance<<" completed="<<completed<<" calls="<<peakCalls<<" blocked="<<t.blockedReason<<" top="<<t.ledgeReason<<'\n';
    check(maximumMargin>10&&maximumMargin<14.1f,"recovery consumes the recorded fourteen-unit margin without inventing additional room");
    check(completed&&!released,"a reachable recessed top finishes after checked recovery of the old obstacle margin");
    check(recoveries>0,"the summit regression actually exercises bounded inward recovery");
    check(peakRecovery<=100.f/fps+.06f,"recovery obeys climb speed at every tested frame rate");
    check(minDistance>=30.90f,"independent capsule-to-solid distances stay outside every real obstruction");
}

static Traversal nearTop(MarginWorld& w) {
    w.scene();return recordedMargin(w);
}
static void blockedRecovery() {
    MarginWorld w;auto t=nearTop(w);const auto p=w.local(t.position);

    w.boxes.push_back({{-80,p.y+32,p.z+100},{80,20,p.z+200}});
    float minDistance=1e9f;const auto initialY=p.y;bool completed=false;
    for(int frame=0;frame<50;++frame){const auto r=t.update(w,up(),1.f/60,1000);completed|=r.completed;minDistance=std::min(minDistance,w.capsuleDistance(t.position,31,138));}
    std::cout<<"blocked recovery y="<<w.local(t.position).y<<" initial="<<initialY<<" capsule="<<minDistance<<" reason="<<t.blockedReason<<'\n';
    check(!completed&&w.local(t.position).y<=initialY+1.06f,"recovery cannot cross the one-unit physical clearance before the new obstruction");
    check(minDistance>=30.9f,"blocked recovery leaves the complete capsule outside the new solid");
}
static void absentSupport() {
    MarginWorld w;auto t=nearTop(w);const auto before=t.position;w.boxes.clear();
    const auto r=t.update(w,up(),1.f/60,1000);
    check(!r.completed&&(t.position-before).length()<.001f,"absent geometry never authorizes unsupported inward movement");
}
static void unreachablePlatform(bool withOldMargin) {
    MarginWorld w;w.scene(30);auto t=withOldMargin?recordedMargin(w):attach(w);bool completed=false;float closest=-37,maximumMargin=withOldMargin?14.f:0.f;int reversals=0,previousDirection=0;
    for(int frame=0;frame<240;++frame){
        const auto before=w.local(t.position);const auto r=t.update(w,up(),1.f/60,1000);const auto p=w.local(t.position);completed|=r.completed;
        if(t.state==State::wall){closest=std::max(closest,p.y);maximumMargin=std::max(maximumMargin,-37-p.y);}
        if(frame>=180&&std::abs(p.z-before.z)<.02f&&std::abs(p.y-before.y)>.1f) {
            const int direction=p.y>before.y?1:-1;
            if(previousDirection&&direction!=previousDirection)++reversals;
            previousDirection=direction;
        }
    }
    std::cout<<"unreachable oldMargin="<<withOldMargin<<" maximumMargin="<<maximumMargin<<" closest="<<closest<<" completed="<<completed<<" stationary reversals="<<reversals<<'\n';
    if(withOldMargin)check(maximumMargin>10,"the unreachable-platform negative also starts with the actual recorded clearance margin");
    check(!completed&&closest<=-36.94f,"a platform beyond hand reach cannot steal nominal gap after any old margin is exhausted");
    check(reversals<=1,"unsupported movement must not alternate inward recovery with re-added escape margin indefinitely");
}
int main(){
    for(bool far:{false,true})for(int fps:{30,60,120})for(float yaw:{0.f,.633f})recoverAndTop(far,fps,yaw);
    blockedRecovery();absentSupport();unreachablePlatform(false);unreachablePlatform(true);std::cout<<"margin recovery failures="<<failures<<'\n';return failures?1:0;
}
