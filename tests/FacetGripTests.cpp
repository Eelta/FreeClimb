#include "Core.h"
#include <bit>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>
using namespace fc;

struct FacetWorld final:World {
    struct Triangle {Vec a,b,c;};
    std::vector<Triangle> triangles;
    Vec origin{};float yaw{};unsigned calls{},peakCalls{};
    std::uint64_t digest=1469598103934665603ULL;
    Vec rotate(Vec p,float angle)const{return {p.x*std::cos(angle)-p.y*std::sin(angle),p.x*std::sin(angle)+p.y*std::cos(angle),p.z};}
    Vec global(Vec p)const{return origin+rotate(p,yaw);}
    Vec local(Vec p)const{return rotate(p-origin,-yaw);}
    void quad(Vec a,Vec b,Vec c,Vec d){triangles.push_back({a,b,c});triangles.push_back({a,c,d});}
    void box(Vec lo,Vec hi) {
        const float x=lo.x,X=hi.x,y=lo.y,Y=hi.y,z=lo.z,Z=hi.z;
        quad({x,y,z},{X,y,z},{X,y,Z},{x,y,Z});
        quad({X,Y,z},{x,Y,z},{x,Y,Z},{X,Y,Z});
        quad({x,Y,z},{x,y,z},{x,y,Z},{x,Y,Z});
        quad({X,y,z},{X,Y,z},{X,Y,Z},{X,y,Z});
        quad({x,Y,z},{X,Y,z},{X,y,z},{x,y,z});
        quad({x,y,Z},{X,y,Z},{X,Y,Z},{x,Y,Z});
    }
    void mix(Vec v){for(float x:{v.x,v.y,v.z}){digest^=std::bit_cast<std::uint32_t>(x);digest*=1099511628211ULL;}}
    std::optional<Hit> ray(Vec from,Vec to)override {
        ++calls;mix(from);mix(to);const auto a=local(from),d=local(to)-a;
        double nearest=2;std::optional<Hit> result;
        for(const auto& tri:triangles) {
            const auto e1=tri.b-tri.a,e2=tri.c-tri.a,n=e1.cross(e2).unit();
            const double denominator=d.dot(n),distance=(a-tri.a).dot(n);
            if(denominator>=-.000001||distance<=.000001)continue;
            const double time=-distance/denominator;if(time<0||time>1||time>=nearest)continue;
            const auto p=a+d*float(time),q=p-tri.a;
            const double uu=e1.dot(e1),uv=e1.dot(e2),vv=e2.dot(e2),qu=q.dot(e1),qv=q.dot(e2),det=uu*vv-uv*uv;
            if(det<=0)continue;
            const double u=(qu*vv-qv*uv)/det,v=(qv*uu-qu*uv)/det;
            if(u<-.000002||v<-.000002||u+v>1.000002)continue;
            nearest=time;result=Hit{global(p),rotate(n,yaw),true};
        }
        if(result){mix(result->point);mix(result->normal);}else mix({});return result;
    }
    void resetCalls(){calls=0;digest=1469598103934665603ULL;}
};

static int failures=0;
static void check(bool value,const char* message) {if(!value){++failures;std::cerr<<"FAIL "<<message<<'\n';}}
static void configure(Traversal& t){t.cfg.approachSeconds=0;t.cfg.radius=31;t.cfg.gap=37;t.cfg.height=138;}
static void slot(FacetWorld& w,float halfWidth=3,bool left=true,bool right=true) {
    w.box({-500,0,-500},{500,80,71});
    if(left)w.box({-500,0,71},{-halfWidth,80,600});
    if(right)w.box({halfWidth,0,71},{500,80,600});
}
static bool attach(Traversal& t,FacetWorld& w,Vec feet={0,-37,0}) {
    configure(t);return t.attach(w,w.global(feet),w.rotate({0,1,0},w.yaw),1000,35);
}

static void supportedSlot(bool far,int fps,float yaw,bool left=true,bool right=true) {
    FacetWorld w;w.yaw=yaw;if(far)w.origin={111505.8125f,79429.109375f,7544.5f};slot(w,3,left,right);
    Traversal t;const float start=1.f-12.5f/fps;
    check(attach(t,w,{0,-37,start}),"a wide actual lower face allows the initial grab");
    unsigned maxCalls=0;bool released=false,action=false;Input input;input.y=1;input.mantle=true;
    for(int frame=0;frame<fps/2;++frame){w.resetCalls();auto result=t.update(w,input,1.f/fps,1000);maxCalls=std::max(maxCalls,w.calls);released|=result.released;action|=t.state==State::action;}
    const auto p=w.local(t.position);
    std::cout<<"slot far="<<far<<" fps="<<fps<<" yaw="<<yaw<<" sides="<<left<<right<<" rise="<<p.z-start<<" peakCalls="<<maxCalls<<" reason="<<t.blockedReason<<'\n';
    check(!released&&t.active(),"two real side grips retain an active climb across a missing midline");
    check(p.z-start>45,"a six-unit vertical slot does not stop climbing when both hands have real support");
    check(!action,"grip continuity uses ordinary checked movement rather than a decorative obstacle hop");
    check(std::abs(p.x)<.1f&&p.y<=-30.99f,"the supported slot traversal retains physical body clearance");
}

static void unsupportedSlot(const char* label,float width,bool isolatedPatch=false,float patchSide=-1) {
    FacetWorld w;slot(w,width,!isolatedPatch,!isolatedPatch);
    if(isolatedPatch) {
        w.box({patchSide*10-1,0,113},{patchSide*10+1,80,115});

        w.box({-500,0,130},{500,80,600});
    }
    Traversal t;
    check(attach(t,w,{0,-37,.8f}),"the negative fixture starts on the same valid lower face");
    Input input;input.y=1;input.mantle=true;w.resetCalls();const auto before=t.position;
    const auto result=t.update(w,input,1.f/60,1000);
    std::cout<<label<<" rise="<<t.position.z-before.z<<" calls="<<w.calls<<" state="<<int(t.state)<<" reason="<<t.blockedReason<<" top="<<t.ledgeReason<<'\n';
    check(!result.completed&&t.position.z-before.z<.01f,"one isolated two-unit patch or a gap wider than the existing hand window cannot authorize ascent");
    check(std::string(t.blockedReason)=="destination surface missing","isolated or absent contacts fail the actual destination-support branch");
}

static void incompatibleFaces() {
    FacetWorld w;w.box({-500,0,-500},{500,80,71});

    w.box({-500,0,130},{500,80,600});

    const float slope=std::sqrt(3.f);
    for(float sign:{-1.f,1.f}){
        Vec a{sign*6.f,0,71.1f},b{sign*14.f,slope*8.f,71.1f};
        if(sign<0)std::swap(a,b);
        w.quad(a,b,b+Vec{0,0,2.9f},a+Vec{0,0,2.9f});
    }
    Traversal t;check(attach(t,w,{0,-37,.8f}),"incompatible facets preserve a valid starting grip below them");
    Input input;input.y=1;input.mantle=true;const auto before=t.position;
    const auto result=t.update(w,input,1.f/60,1000);
    std::cout<<"incompatible rise="<<t.position.z-before.z<<" reason="<<t.blockedReason<<'\n';
    check(!result.completed&&t.position.z-before.z<.01f,"two contradictory facet normals are not a valid substitute grip pair");
    check(std::string(t.blockedReason)=="destination surface missing","incompatible facets fail at real grip validation rather than an unrelated ledge transition");
}

static void blockedBody() {
    FacetWorld w;slot(w);w.box({-80,-80,140},{80,-1,160});Traversal t;
    check(attach(t,w,{0,-37,.8f}),"body obstruction starts above the currently valid capsule");
    Input input;input.y=1;input.mantle=true;const auto start=t.position;bool touched=false;
    for(int frame=0;frame<18;++frame){const auto r=t.update(w,input,1.f/60,1000);touched|=std::string(t.blockedReason)=="body clearance";check(!r.completed,"a low ceiling never becomes a completed top-out");}
    std::cout<<"ceiling rise="<<t.position.z-start.z<<" reason="<<t.blockedReason<<'\n';
    check(t.position.z+t.cfg.height<=140.01f,"real hand contacts never permit the head through a ceiling");
    check(touched,"the positive hand fallback reaches and respects the independent body-clearance stage");
}

static void horizontalSeams(bool far) {
    FacetWorld w;if(far){w.origin={111505.8125f,79429.109375f,7544.5f};w.yaw=.633f;}
    w.box({-500,0,-500},{500,80,67});
    w.box({-500,0,73},{500,80,109});
    w.box({-500,0,115},{500,80,600});
    Traversal fresh;
    check(!attach(fresh,w),"vertical pairs alone do not relax the width requirement for an initial grab");
    Traversal climbing;check(attach(climbing,w,{0,-37,-4}),"seam traversal starts with an ordinary horizontal grip pair");
    Input input;input.y=1;input.mantle=true;unsigned peak=0;
    for(int frame=0;frame<7;++frame){w.resetCalls();const auto r=climbing.update(w,input,1.f/60,1000);peak=std::max(peak,w.calls);check(!r.released&&!r.completed,"nearby vertical support remains an attached traversal");}
    const auto p=w.local(climbing.position);
    std::cout<<"horizontal seams far="<<far<<" z="<<p.z<<" peakCalls="<<peak<<" reason="<<climbing.blockedReason<<'\n';
    check(p.z>7,"tracking can bridge a six-unit horizontal seam using two actual vertical contacts");
    check(p.y<=-30.99f,"vertical contact fallback retains full body clearance");
}

static void existingFastPath() {
    FacetWorld w;w.box({-500,0,-500},{500,80,600});Traversal t;
    check(attach(t,w),"ordinary flat wall still attaches");Input input;input.y=1;input.mantle=true;
    w.resetCalls();const auto result=t.update(w,input,1.f/60,1000);
    std::cout<<"flat calls="<<w.calls<<" digest="<<w.digest<<" z="<<t.position.z<<'\n';
    check(!result.released&&t.position.z>1.6f,"ordinary supported movement remains intact");
    FacetWorld ribbon;ribbon.box({-4,0,-500},{4,80,600});Traversal thin;
    check(!attach(thin,ribbon),"an eight-unit initial ribbon is still too narrow for a new attachment");
    FacetWorld empty;Traversal none;check(!attach(none,empty),"empty space cannot become supporting geometry");
}

int main() {
    existingFastPath();
    for(bool far:{false,true})for(int fps:{30,60,120})for(float yaw:{0.f,.633f})supportedSlot(far,fps,yaw);
    for(bool far:{false,true}){supportedSlot(far,60,.633f,true,false);supportedSlot(far,60,.633f,false,true);}
    horizontalSeams(false);horizontalSeams(true);
    unsupportedSlot("isolated left patch",3,true,-1);unsupportedSlot("isolated right patch",3,true,1);
    unsupportedSlot("wide slot",12);incompatibleFaces();blockedBody();
    std::cout<<"facet grip failures="<<failures<<'\n';return failures?1:0;
}
