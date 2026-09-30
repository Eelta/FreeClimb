#include "Core.h"
#include <iostream>
#include <vector>
#include <string>
using namespace fc;
struct Solid {Vec low,high;};
struct Geometry:World {
    std::vector<Solid> solids;Vec origin{};float yaw{};unsigned casts{};
    Vec rotate(Vec p,float a)const{return {p.x*std::cos(a)-p.y*std::sin(a),p.x*std::sin(a)+p.y*std::cos(a),p.z};}
    Vec global(Vec p)const{return origin+rotate(p,yaw);}
    Vec local(Vec p)const{return rotate(p-origin,-yaw);}
    std::optional<Hit> ray(Vec from,Vec to)override {
        ++casts;const auto a=local(from),d=local(to)-a;std::optional<Hit> result;double nearest=2;
        for(const auto& solid:solids) {
            double enter=0,leave=1;Vec normal{};bool valid=true;
            const double p[]={a.x,a.y,a.z},v[]={d.x,d.y,d.z},lo[]={solid.low.x,solid.low.y,solid.low.z},hi[]={solid.high.x,solid.high.y,solid.high.z};
            for(int axis=0;axis<3;++axis) {
                if(std::abs(v[axis])<1e-10){if(p[axis]<lo[axis]||p[axis]>hi[axis])valid=false;continue;}
                double l=(lo[axis]-p[axis])/v[axis],h=(hi[axis]-p[axis])/v[axis];float sign=-1;
                if(l>h){std::swap(l,h);sign=1;}
                if(l>enter){enter=l;normal={};if(axis==0)normal.x=sign;else if(axis==1)normal.y=sign;else normal.z=sign;}
                leave=std::min(leave,h);
            }
            if(valid&&enter<=leave&&enter>.000001&&enter<=1&&enter<nearest) {
                nearest=enter;result=Hit{global(a+d*float(enter)),rotate(normal,yaw),true};
            }
        }
        return result;
    }
    bool capsuleClear(Vec globalFeet,float radius,float height)const {
        const auto p=local(globalFeet);
        for(const auto& s:solids) {
            if(p.z+height<=s.low.z+.15f||p.z+6>=s.high.z-.15f)continue;
            const float x=std::clamp(p.x,s.low.x,s.high.x),y=std::clamp(p.y,s.low.y,s.high.y);
            if(std::hypot(p.x-x,p.y-y)<radius-.15f)return false;
        }
        return true;
    }
};
struct Crest:World {
    struct Segment{float low,high,y,slope;};std::vector<Segment> segments;
    Vec origin{};float yaw{},top=240,topY{},capNormalZ=1,capHalfWidth=10000;bool lowCeiling=false;unsigned casts{};
    Crest() {
        float y=0,z=0;
        for(float normalZ:{.528f,.665f,.605f,.584f}) {
            const float slope=normalZ/std::sqrt(1-normalZ*normalZ);
            segments.push_back({z,z+60,y,slope});y+=60*slope;z+=60;
        }
        topY=y;segments.front().low=-1000;
    }
    Vec rotate(Vec p,float a)const{return {p.x*std::cos(a)-p.y*std::sin(a),p.x*std::sin(a)+p.y*std::cos(a),p.z};}
    Vec global(Vec p)const{return origin+rotate(p,yaw);}
    Vec local(Vec p)const{return rotate(p-origin,-yaw);}
    std::optional<Hit> ray(Vec from,Vec to)override {
        ++casts;const Vec a=local(from),b=local(to),d=b-a;std::optional<Hit> result;float nearest=2;
        for(const auto& s:segments) {
            const Vec n=Vec{0,-1,s.slope}.unit();const Vec base{0,s.y,s.low<0?0:s.low};
            const float f=(a-base).dot(n),t=(b-base).dot(n);
            if(f<=0||t>=0)continue;
            const float k=f/(f-t);const Vec p=a+d*k;
            if(p.z>=s.low&&p.z<=s.high&&k<nearest){nearest=k;result=Hit{global(p),rotate(n,yaw),true};}
        }
        const Vec capNormal{0,-std::sqrt(1-capNormalZ*capNormalZ),capNormalZ},capBase{0,topY,top};
        const float capFrom=(a-capBase).dot(capNormal),capTo=(b-capBase).dot(capNormal);
        if(capFrom>0&&capTo<=0) {
            const float k=capFrom/(capFrom-capTo);const Vec p=a+d*k;
            if(p.y>=topY&&std::abs(p.x)<=capHalfWidth&&k<nearest){nearest=k;result=Hit{global(p),rotate(capNormal,yaw),true};}
        }
        if(lowCeiling&&a.z<top+80&&b.z>=top+80) {
            const float k=(top+80-a.z)/d.z;const Vec p=a+d*k;
            if(p.y>=topY-40&&k<nearest){nearest=k;result=Hit{global(p),{0,0,-1},true};}
        }
        if(lowCeiling&&a.y<topY-40&&b.y>=topY-40) {
            const float k=(topY-40-a.y)/d.y;const Vec p=a+d*k;
            if(p.z>=top+80&&k<nearest)result=Hit{global(p),rotate({0,-1,0},yaw),true};
        }
        return result;
    }
};
static void realDimensions(Traversal& t){t.cfg.approachSeconds=0;t.cfg.radius=31;t.cfg.gap=37;t.cfg.height=138;}
int main() {
    int failures=0;
    auto require=[&](bool value,const char* message){if(!value){std::cerr<<"FAIL: "<<message<<'\n';++failures;}};
    for(int fps:{30,48,120})for(bool exterior:{false,true}) {
        Geometry world;world.solids={{{-17.5f,0,-100},{17.5f,200,240}}};
        if(exterior){world.origin={105758,69620,6287};world.yaw=.7f;}
        Traversal t;realDimensions(t);
        if(!t.attach(world,world.global({0,-37,0}),world.rotate({0,1,0},world.yaw),100)) {
            require(false,"35-unit ridge starts with two real grips");continue;
        }
        bool penetrated=false;Result result;unsigned peak=0;
        for(int frame=0;frame<fps*8&&t.active();++frame) {
            world.casts=0;result=t.update(world,{0,1,false,true},1.f/fps,100);peak=std::max(peak,world.casts);
            penetrated|=!world.capsuleClear(t.position,t.cfg.radius,t.cfg.height);
        }
        const auto p=world.local(t.position);
        std::cout<<"narrow vertical summit fps="<<fps<<" exterior="<<exterior<<" completed="<<result.completed
            <<" pos="<<p.x<<','<<p.y<<','<<p.z<<" top="<<t.ledgeReason<<" blocked="<<t.blockedReason<<" peak="<<peak<<'\n';
        require(result.completed&&!t.active()&&p.z>=240&&p.y>=17,
            "a narrow real standing footprint can complete its top-out with reachable adjusted palms");
        require(!penetrated,"narrow summit route preserves the full standing capsule rather than cutting through its lip");
    }
    for(int fps:{30,48,120})for(bool closed:{false,true})for(bool waitUntilBlocked:{false,true}) {
        Geometry world;world.solids={{{-500,0,-100},{500,300,2000}},{{-500,closed?-500.f:-25.f,155},{500,0,235}}};
        world.origin={105758,69620,6287};world.yaw=.4f;
        Traversal t;realDimensions(t);
        if(!t.attach(world,world.global({0,-37,0}),world.rotate({0,1,0},world.yaw),100)) {
            require(false,"thick projecting rock band attaches below the obstruction");continue;
        }
        int hops=0;bool penetrated=false,observedStall=false;Result result;float maxStall=0;int nextPress=0;
        for(int frame=0;frame<fps*7&&t.active();++frame) {
            maxStall=std::max(maxStall,t.stalledSeconds());
            observedStall|=t.stalledSeconds()>.4f;
            const auto before=t.state;const bool press=(!waitUntilBlocked||observedStall)&&frame>=nextPress;
            if(press)nextPress=frame+std::max(1,fps/2);
            result=t.update(world,{0,1,false,false,press},1.f/fps,100);
            if(before!=State::action&&hopMotion(result.motion))++hops;
            penetrated|=!world.capsuleClear(t.position,t.cfg.radius,t.cfg.height);
        }
        const auto p=world.local(t.position);
        std::cout<<"thick rock band fps="<<fps<<" closed="<<closed<<" waitUntilBlocked="<<waitUntilBlocked
            <<" observedStall="<<observedStall<<" maxStall="<<maxStall<<" rise="<<p.z<<" hops="<<hops<<" blocked="<<t.blockedReason<<'\n';
        require(!penetrated,"a longer climbing bypass must still check all parts of the capsule against the entire band");
        if(closed)require(t.active()&&p.z<18&&hops==0,"a continuous ceiling remains physically impassable even with repeated Space");
        else require(t.active()&&p.z>260&&hops>0,"ordinary climbing can use an available outward-and-up route beyond a thick projecting band");
    }
    for(int fps:{30,48,120})for(bool exterior:{false,true})for(float capNormalZ:{1.f,.7001f,.705f,.71f}) {
        Crest world;world.capNormalZ=capNormalZ;if(exterior){world.origin={105758,69620,6287};world.yaw=1.724f;}
        Traversal t;realDimensions(t);
        if(!t.attach(world,world.global({0,-37,0}),world.rotate({0,1,0},world.yaw),100)){
            require(false,"real-normal continuous mountain slope attach");continue;
        }
        Result result;
        for(int frame=0;frame<fps*9&&t.active();++frame)result=t.update(world,{0,1,false,true},1.f/fps,100);
        const auto p=world.local(t.position);
        std::cout<<"faceted mountain cap fps="<<fps<<" exterior="<<exterior<<" capNormalZ="<<capNormalZ<<" completed="<<result.completed
            <<" pos="<<p.y<<','<<p.z<<" capY="<<world.topY<<" top="<<t.ledgeReason<<" blocked="<<t.blockedReason<<'\n';
        require(result.completed&&!t.active()&&p.z>=world.top&&p.y>world.topY,
            "continuous real mountain facets reach their genuinely walkable flat or .7001/.705/.71 cap at near and exterior coordinates");
        if(result.completed) {
            const Vec capNormal{0,-std::sqrt(1-capNormalZ*capNormalZ),capNormalZ};

            const float sphereCenterDistance=(p+Vec{0,0,t.cfg.radius}-Vec{0,world.topY,world.top}).dot(capNormal);
            require(sphereCenterDistance+.15f>=t.cfg.radius,
                "walkable sloped cap completion clears the actual lower capsule sphere against the cap plane");
        }
    }
    for(int fps:{30,48,120})for(bool exterior:{false,true})for(bool ceiling:{false,true}) {
        Crest world;world.capNormalZ=.705f;world.lowCeiling=ceiling;
        if(!ceiling)world.capHalfWidth=10;
        if(exterior){world.origin={105758,69620,6287};world.yaw=.7f;}
        Traversal t;realDimensions(t);
        if(!t.attach(world,world.global({0,-37,0}),world.rotate({0,1,0},world.yaw),100)) {
            require(false,"restricted sloped cap attaches to its legal lower wall");continue;
        }
        bool completed=false,penetrated=false;Result result;
        for(int frame=0;frame<fps*9&&t.active();++frame) {
            result=t.update(world,{0,1,false,true},1.f/fps,100);completed|=result.completed;
            if(ceiling) {
                const auto p=world.local(t.position);
                const float dy=std::max(0.f,world.topY-40-p.y);
                const float dz=std::max(0.f,world.top+80-(p.z+t.cfg.height-t.cfg.radius));
                penetrated|=std::hypot(dy,dz)<t.cfg.radius-.15f;
            }
        }
        std::cout<<"unsafe sloped cap fps="<<fps<<" exterior="<<exterior<<" ceiling="<<ceiling
            <<" completed="<<completed<<" active="<<t.active()<<" top="<<t.ledgeReason<<" blocked="<<t.blockedReason<<'\n';
        require(!completed,"rounded capsule clearance cannot authorize a summit with a low ceiling or missing side foot supports");
        require(!penetrated,"a sloped summit under a low ceiling never penetrates the true upper capsule hemisphere");
    }
    std::cout<<"geometry failures="<<failures<<'\n';return failures?1:0;
}
