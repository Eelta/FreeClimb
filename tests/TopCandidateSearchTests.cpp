#include "Core.h"
#include "TopCandidateSearch.h"
#include <limits>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace fc;
struct Plane {Vec n;float d;};
using Solid=std::vector<Plane>;
static Solid box(Vec lo,Vec hi){return {{{1,0,0},hi.x},{{-1,0,0},-lo.x},{{0,1,0},hi.y},{{0,-1,0},-lo.y},{{0,0,1},hi.z},{{0,0,-1},-lo.z}};}
static void check(bool b,const char* s){if(!b)throw std::runtime_error(s);}
struct TopWorld final:World {
    std::vector<Solid> solids;unsigned calls=0;Vec origin{};float yaw=0;
    TopWorld(bool mirror=false){solids.push_back(box({-120,0,-500},{120,10,130}));auto triangle=box({-120,10,-500},{120,300,130});triangle.push_back({{mirror?1.f:-1.f,.9f,0},19});solids.push_back(triangle);}
    Vec turn(Vec p,float a)const{return {p.x*std::cos(a)-p.y*std::sin(a),p.x*std::sin(a)+p.y*std::cos(a),p.z};}
    Vec global(Vec p)const{return origin+turn(p,yaw);}
    Vec local(Vec p)const{return turn(p-origin,-yaw);}
    std::optional<Hit> ray(Vec from,Vec to)override{
        ++calls;auto a=local(from),d=local(to)-a;std::optional<Hit> out;double nearest=2;
        for(const auto& shape:solids){double enter=0,leave=1;Vec en{},ln{};bool reject=false,inside=true;
            for(const auto& p:shape){double gap=a.dot(p.n)-p.d,rate=d.dot(p.n);inside&=gap<-.0001;if(std::abs(rate)<1e-9){if(gap>0){reject=true;break;}continue;}double t=-gap/rate;if(rate<0){if(t>enter){enter=t;en=p.n;}}else if(t<leave){leave=t;ln=p.n;}if(enter>leave){reject=true;break;}}
            double t=inside?leave:enter;Vec n=inside?ln:en;if(!reject&&t>1e-6&&t<=1&&t<nearest&&n.length()>.9){nearest=t;out=Hit{global(a+d*float(t)),turn(n.unit(),yaw),true};}}
        return out;
    }
    bool capsuleClear(Vec feet)const {
        const Vec root=local(feet);
        for(int z=0;z<=138;++z){const float c=z<31?31.f-z:z>107?z-107.f:0,r=std::sqrt(std::max(0.f,31.f*31-c*c));
            for(int ring=0;ring<=32;++ring){float a=ring*6.28318530718f/32;Vec p=root+Vec{ring==32?0:r*std::cos(a),ring==32?0:r*std::sin(a),float(z)};
                for(const auto& solid:solids){bool inside=true;for(const auto& plane:solid)inside&=p.dot(plane.n)<plane.d-.06f;if(inside)return false;}}}
        return true;
    }
};
static Traversal actor(TopWorld& w){Traversal t;t.cfg.radius=31;t.cfg.gap=37;t.cfg.height=138;t.cfg.approachSeconds=0;t.cfg.automaticClimbActions=false;check(t.attach(w,w.global({0,-37,0}),w.turn({0,1,0},w.yaw),1000),"solid front wall must attach");return t;}
static bool run(bool mirror,float yaw,bool distant,int fps,int negative=0){TopWorld w(mirror);w.yaw=yaw;if(distant)w.origin={134999.047f,41413.3164f,-11817.2373f};auto t=actor(w);
    if(negative==1)w.solids.push_back(box({-150,-80,140},{150,350,150}));
    if(negative==2)w.solids.erase(w.solids.begin()+1);
    if(negative==3){w.solids[1].push_back({{0,1,0},26});}
    bool completed=false,mantle=false,aborted=false;unsigned peak=0;Vec previous=t.position;
    for(int f=0;f<fps*3&&t.active();++f){w.calls=0;auto result=t.update(w,{0,1,false,true},1.f/fps,1000);peak=std::max(peak,w.calls);completed|=result.completed;mantle|=t.state==State::mantle;aborted|=result.released&&!result.completed;
        for(int k=0;k<=8;++k)check(w.capsuleClear(previous+(t.position-previous)*(float(k)/8)),"complete continuous body route must avoid the triangular top");previous=t.position;
        if(mantle&&negative==0){for(int hand=0;hand<2;++hand){auto p=w.local(t.topHand(hand));check(std::abs(p.z-130)<.08f&&p.y>=0&&p.y<=10.1f,"both palms must remain on the real front lip");}}
    }
    auto p=w.local(t.position);std::cout<<"top mirror="<<mirror<<" yaw="<<yaw<<" distant="<<distant<<" fps="<<fps<<" negative="<<negative<<" complete="<<completed<<" mantle="<<mantle<<" pos="<<p.x<<","<<p.y<<","<<p.z<<" peak="<<peak<<" reason="<<t.ledgeReason<<"\n";
    if(negative)check(!completed,"missing footprint or full body blocker must reject completion");
    else{
        check(completed&&mantle&&!aborted,"bounded side search must find the physically valid off-axis top");

    }
    check(peak<4096,"query workload must remain bounded");return completed;
}

static void laneContract(){
    unsigned calls=0;auto found=searchTopCandidateLanes(31,[&](float side)->std::optional<float>{++calls;return side==0?std::optional<float>{7.f}:std::nullopt;});
    check(found&&*found==7&&calls==1,"existing safe centre target keeps priority and adds no side queries");
    calls=0;std::array<float,3> offsets{};found=searchTopCandidateLanes(31,[&](float side)->std::optional<float>{offsets[calls++]=side;return {};});
    check(!found&&calls==3&&offsets[0]==0&&offsets[1]==23.25f&&offsets[2]==-23.25f,"failure searches exactly two bounded side lanes");
    for(float radius:{0.f,-1.f,std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity()}){
        calls=0;found=searchTopCandidateLanes(radius,[&](float)->std::optional<float>{++calls;return {};});
        check(!found&&calls==1,"invalid radius cannot create off-axis search coordinates");
    }
}
static void liveObstacle(int kind){TopWorld w;auto t=actor(w);bool changed=false,aborted=false,completed=false;Vec previous=t.position;
    for(int f=0;f<240&&t.active();++f){
        if(t.state==State::mantle&&t.progress()>.55f&&!changed){changed=true;
            if(kind==0)w.solids.push_back(box({15,0,170},{60,350,175}));
            else w.solids.erase(w.solids.begin()+1);
        }
        auto result=t.update(w,{0,1,false,true},1.f/60,1000);aborted|=result.released&&!result.completed;completed|=result.completed;
        for(int k=0;k<=8;++k)check(w.capsuleClear(previous+(t.position-previous)*(float(k)/8)),"live route revalidation stops before a new collision");previous=t.position;
    }
    check(changed&&aborted&&!completed,"lost support or a new body blocker aborts the active side top-out");
}

static void queryBudget(){TopWorld source;TopCandidateBudgetWorld<World,Vec,Hit> world(source);
    const Vec from=source.global({0,5,140}),to=source.global({0,5,100});
    for(unsigned ray=0;ray<TopCandidateBudgetWorld<World,Vec,Hit>::limit;++ray){const auto hit=world.ray(from,to);check(hit&&hit->climbable&&std::abs(hit->point.z-130.f)<.01f,"every query below the budget returns the real floor");}
    check(source.calls==4096&&world.count==4096&&!world.exhausted,"the 4096th ray remains an ordinary verified query");
    for(unsigned ray=0;ray<16;++ray){const auto hit=world.ray(from,to);check(hit&&!hit->climbable&&world.exhausted,"the 4097th and later requests become blockers");}
    check(source.calls==4096&&world.count==4096,"no later query reaches the real geometry after budget exhaustion");
}
int main(){try{queryBudget();laneContract();liveObstacle(0);liveObstacle(1);for(bool mirror:{false,true})for(float yaw:{0.f,.73f})for(bool distant:{false,true})for(int fps:{30,60,120})run(mirror,yaw,distant,fps);for(int negative:{1,2,3})for(bool mirror:{false,true})run(mirror,.73f,true,60,negative);std::cout<<"PASS off-axis triangular top and negative geometry\n";return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
