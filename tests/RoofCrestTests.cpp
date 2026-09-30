#include "Core.h"
#include "TraversalCapture.h"
#include <iostream>
#include <vector>
using namespace fc;
struct CrestWorld:World {
    struct Plane{Vec n;float d;};using Solid=std::vector<Plane>;
    std::vector<Solid> solids;float nz,nx,farZ,farX,skew{},yaw{},gap{},capHalf{},capHeight{};Vec origin{};unsigned calls{};bool hole{},thin{};
    CrestWorld(float z,float other=-1,float angle=0):nz(z),nx(std::sqrt(1-z*z)),farZ(other<0?z:other),farX(std::sqrt(1-farZ*farZ)),skew(angle) {
        solids={{{{1,0,0},300},{{-1,0,0},300},{{0,1,0},300},{{0,-1,0},300},{{0,0,-1},1000},{{nx,0,nz},0},{{-farX*std::cos(skew),farX*std::sin(skew),farZ},0}}};
    }
    Vec rotate(Vec p,float angle)const{return {p.x*std::cos(angle)-p.y*std::sin(angle),p.x*std::sin(angle)+p.y*std::cos(angle),p.z};}
    Vec global(Vec p)const{return origin+rotate(p,yaw);}
    Vec local(Vec p)const{return rotate(p-origin,-yaw);}
    void cap(float width){capHalf=width*.5f;capHeight=-capHalf*nx/nz;solids[0].push_back({{0,0,1},capHeight});}
    void ceiling(float z){solids.push_back({{{1,0,0},300},{{-1,0,0},300},{{0,1,0},300},{{0,-1,0},300},{{0,0,1},z+4},{{0,0,-1},-z}});}
    std::optional<Hit> ray(Vec from,Vec to)override {
        ++calls;const Vec a=local(from),d=local(to)-a;double best=2;std::optional<Hit> result;
        for(const auto& solid:solids){double enter=0,leave=1;Vec normal{};bool invalid=false;
            for(auto p:solid){const double dist=a.dot(p.n)-p.d,rate=d.dot(p.n);
                if(std::abs(rate)<1e-9){if(dist>0){invalid=true;break;}continue;}
                const double t=-dist/rate;if(rate<0){if(t>enter){enter=t;normal=p.n;}}else leave=std::min(leave,t);
                if(leave<enter){invalid=true;break;}}
            if(invalid||enter<=1e-6||enter>1||enter>=best||normal.length()<.5f)continue;
            const auto p=a+d*float(enter);
            if((hole&&std::abs(p.y)>8&&p.z>-30)||(gap>0&&std::abs(p.x)<gap))continue;
            best=enter;result=Hit{global(p),rotate(normal,yaw),true};
        }return result;
    }
    float capsuleDistance(Vec globalFeet)const {
        const auto feet=local(globalFeet);const Vec centre=feet+Vec{0,0,31};
        const Vec near{nx,0,nz},far{-farX*std::cos(skew),farX*std::sin(skew),farZ};
        const Vec axis=near.cross(far).unit();
        float nearest=(centre-axis*centre.dot(axis)).length();
        if(capHalf<=0) {
            for(auto n:{near,far}) {
                const Vec point=centre-n*centre.dot(n);
                if(point.dot(near)<=.0001f&&point.dot(far)<=.0001f)nearest=std::min(nearest,(centre-point).length());
            }
            return nearest;
        }
        nearest=10000;
        auto segment=[&](Vec start,Vec end) {
            start.y=end.y=centre.y;
            const auto delta=end-start;
            const auto point=start+delta*std::clamp((centre-start).dot(delta)/delta.dot(delta),0.f,1.f);
            nearest=std::min(nearest,(centre-point).length());
        };
        const float farHalf=-capHeight*farZ/farX;
        segment({capHalf,0,capHeight},{300,0,-300*nx/nz});
        segment({-farHalf,0,capHeight},{-300,0,-300*farX/farZ});
        segment({-farHalf,0,capHeight},{capHalf,0,capHeight});
        return nearest;
    }
};
static unsigned failures=0;
static void check(bool b,const char* text){if(!b){++failures;std::cerr<<"FAIL "<<text<<'\n';}}
static Traversal start(CrestWorld& w) {
    Traversal t;t.cfg.gap=37;t.cfg.radius=31;t.cfg.height=138;t.cfg.approachSeconds=0;
    const Vec feet{37-w.nz*(-200+6)/w.nx,0,-200};
    check(t.attach(w,w.global(feet),w.rotate({-1,0,0},w.yaw),1000),"approach starts on an actual supported sloping roof");return t;
}
static void positive(float nz,bool far,float yaw,int fps) {
    CrestWorld w(nz,nz>.68f?.626594f:-1);w.yaw=yaw;if(far)w.origin={131316.86f,38643.36f,-11331.41f};auto t=start(w);
    bool completed=false,aborted=false,crest=false;float minimum=10000,firstHeight=-1;unsigned peak=0;
    for(int frame=0;frame<fps*8&&t.active();++frame){w.calls=0;const auto r=t.update(w,{0,1,false,true},1.f/fps,1000);
        peak=std::max(peak,w.calls);minimum=std::min(minimum,w.capsuleDistance(t.position));
        if(t.state==State::mantle){if(!crest)firstHeight=t.topLip().z-t.topStart().z;crest=true;
            for(int h=0;h<2;++h){const auto hand=w.local(t.topHand(h));check(std::abs(hand.x*w.nx+hand.z*w.nz)<.035f,"palms lie on the actual near roof, not in air");}}
        completed|=r.completed;aborted|=r.released&&!r.completed;
    }
    std::cout<<"crest nz="<<nz<<" far="<<far<<" yaw="<<yaw<<" fps="<<fps<<" done="<<completed<<" onset="<<firstHeight<<" minCapsule="<<minimum<<" peak="<<peak<<'\n';
    check(completed&&crest&&!aborted&&!t.active(),"ridge pull-up exits traversal instead of retaining the wall state");
    const auto p=w.local(t.position);check(std::abs(p.x)<.07f&&p.z>6.9f&&p.z<7.2f,"endpoint is centred above the actual ridge at the normal seven-unit ground offset");
    check(minimum>=30.95f,"independent capsule distance clears both roof slopes through every phase");
    check(firstHeight>=70&&firstHeight<=t.cfg.grip+28,"verified crest traversal begins above waist height without extending real reach");
    check(peak<1400,"early crest opportunity adds only bounded seeds and retains limited full search cost");
}
static void capped(float width,bool far,float yaw,int fps) {
    CrestWorld world(.50f);world.cap(width);world.yaw=yaw;
    if(far)world.origin={21927.37f,-8411.72f,-2980.40f};
    auto traversal=start(world);Vec previous=traversal.position;
    bool completed=false,released=false,mantle=false;float minimum=10000;
    for(int frame=0;frame<fps*8&&traversal.active();++frame) {
        const auto result=traversal.update(world,{0,1,false,true},1.f/fps,1000);
        for(unsigned sample=0;sample<=8;++sample)minimum=std::min(minimum,world.capsuleDistance(previous+(traversal.position-previous)*(float(sample)/8)));
        previous=traversal.position;completed|=result.completed;released|=result.released&&!result.completed;
        if(traversal.state==State::mantle) {
            mantle=true;
            for(int hand=0;hand<2;++hand) {
                const Vec p=world.local(traversal.topHand(hand));
                const float surface=std::min(world.capHeight,-std::abs(p.x)*world.nx/world.nz);
                check(std::abs(p.z-surface)<.04f,"capped-crest palm lies on its real slope or top");
            }
        }
    }
    const Vec endpoint=world.local(traversal.position);
    std::cout<<"capped width="<<width<<" far="<<far<<" yaw="<<yaw<<" fps="<<fps<<" done="<<completed<<" minCapsule="<<minimum<<'\n';
    check(completed&&mantle&&!released&&!traversal.active(),"narrow flat-capped ridge completes a verified pull-up instead of hanging forever");
    check(std::abs(endpoint.x)<=world.capHalf+.1f&&std::abs(endpoint.z-world.capHeight-7)<.1f,"capped ridge ends above the real cap at normal standing offset");
    check(minimum>=30.95f,"complete capsule remains outside both roof faces and cap through the actual path");
}
static void cappedNegative(int kind) {
    CrestWorld world(.50f);world.cap(20);auto traversal=start(world);
    if(kind==0)world.gap=1;
    if(kind==1)world.hole=true;
    if(kind==2)world.ceiling(world.capHeight+135);
    if(kind==3)world.solids[0][world.solids[0].size()-2]={{-1,0,0},world.capHalf};
    bool complete=false;
    for(int frame=0;frame<600&&traversal.active();++frame)complete|=traversal.update(world,{0,1,false,true},1.f/60,1000).completed;
    check(!complete,"capped ridge with missing centre/foot/other slope or blocked standing capsule cannot complete");
}
static void cappedBlendedNormal(int fps) {
    CrestWorld world(.50f);world.cap(12);auto traversal=start(world);
    for(int frame=0;frame<fps*5&&traversal.active()&&traversal.stalledSeconds()<.4f;++frame)
        traversal.update(world,{0,1},1.f/fps,1000);
    check(traversal.active()&&traversal.stalledSeconds()>=.4f,"capped crest reproduces attached blocked pre-top state");

    traversal.surfaceNormal=traversal.normal*std::sqrt(1-.851f*.851f)+Vec{0,0,.851f};
    bool completed=false;
    for(int frame=0;frame<fps*4&&traversal.active();++frame)
        completed|=traversal.update(world,{0,1,false,true},1.f/fps,1000).completed;
    check(completed,"cached normal above .70 cannot suppress verified opposing-face crest detection");
}
static void negative(int kind) {
    CrestWorld w(.502f);auto t=start(w);
    if(kind==0)w.solids[0].pop_back();
    if(kind==1)w.hole=true;
    if(kind==2)w.gap=1;
    if(kind==3)w.ceiling(145);
    bool complete=false,mantle=false;
    for(int f=0;f<600&&t.active();++f){auto r=t.update(w,{0,1,false,true},1.f/60,1000);complete|=r.completed;mantle|=t.state==State::mantle;}
    check(!complete&&!mantle,"one-sided, missing-foot, separated or obstructed crest is rejected");
}
static void dynamicAndCapture(bool remove,float capWidth=0) {
    CrestWorld w(.491f);if(capWidth>0)w.cap(capWidth);auto t=start(w);bool changed=false,complete=false,aborted=false;unsigned snapshots=0;
    auto tape=std::make_unique<TraversalCapture>();auto loaded=std::make_unique<TraversalCapture>();TraversalCapture::RecordingWorld recorder(w,*tape);
    for(int f=0;f<600&&t.active();++f) {
        if(t.state!=State::mantle){t.update(w,{0,1,false,true},1.f/60,1000);continue;}
        if(remove&&!changed&&t.progress()>.55f){w.hole=true;changed=true;}
        tape->begin(t,{0,1,false,true},1.f/60,1000);auto r=t.update(recorder,{0,1,false,true},1.f/60,1000);tape->finish(t,r);
        std::string error;check(tape->complete()&&loaded->deserialize(tape->serialize(),error)&&loaded->replay().matched,"crest flag and support checks round-trip in same-version captures");++snapshots;
        complete|=r.completed;aborted|=r.released&&!r.completed;
    }
    check(snapshots>20,"capture test covers real crest-mantle frames");
    check(remove?(changed&&aborted&&!complete):(complete&&!aborted),"changing ridge support aborts; intact ridge completes");
}
static void skewed(bool exterior,float angle,int fps,int failure=0) {
    CrestWorld w(.601809323f,.379001409f,angle);
    w.yaw=.633f;if(exterior)w.origin={134559.219f,36994.7031f,-11691.1309f};
    auto t=start(w);
    if(failure==1)w.hole=true;
    if(failure==2)w.gap=1;
    if(failure==3)w.ceiling(125);
    if(failure==4)w.solids[0].pop_back();
    bool complete=false,mantle=false,released=false;float minimum=10000;unsigned peak=0;
    Vec previous=t.position;
    for(int frame=0;frame<fps*8&&t.active();++frame) {
        w.calls=0;const auto r=t.update(w,{0,1,false,true},1.f/fps,1000);
        peak=std::max(peak,w.calls);complete|=r.completed;released|=r.released&&!r.completed;
        for(unsigned step=0;step<=8;++step)minimum=std::min(minimum,w.capsuleDistance(previous+(t.position-previous)*(float(step)/8)));
        previous=t.position;
        if(t.state==State::mantle){mantle=true;
            for(int hand=0;hand<2;++hand){const Vec p=w.local(t.topHand(hand));check(std::abs(p.x*w.nx+p.z*w.nz)<.045f,"oblique crest palms contact the measured near roof");}}
    }
    std::cout<<"skew="<<angle<<" exterior="<<exterior<<" fps="<<fps<<" negative="<<failure<<" done="<<complete<<" minimum="<<minimum<<" peak="<<peak<<'\n';
    if(failure){check(!complete,"unsupported split narrow obstructed or one-sided oblique ridge cannot complete");return;}
    check(complete&&mantle&&!released&&!t.active(),"asymmetric oblique roof ridge completes supported mantle");
    check(minimum>=30.94f,"independent sphere-to-solid-roof clearance holds for actual oblique mantle path");
    const Vec p=w.local(t.position)-Vec{0,0,7};
    const Vec far{-w.farX*std::cos(w.skew),w.farX*std::sin(w.skew),w.farZ};
    check(std::abs(p.dot({w.nx,0,w.nz}))<.05f&&std::abs(p.dot(far))<.05f,"oblique mantle ends over actual intersection of both roof planes");
}
static void longitudinalClearance(bool exterior,float farZ,float skew,int fps) {
    CrestWorld world(.40f,farZ,skew);world.yaw=.633f;
    if(exterior)world.origin={134559.219f,36994.7031f,-11691.1309f};
    auto traversal=start(world);Vec previous=traversal.position;
    const Vec near{world.nx,0,world.nz};
    const Vec far{-world.farX*std::cos(skew),world.farX*std::sin(skew),world.farZ};
    const Vec axis=near.cross(far).unit();
    const float terminalDistance=38.f*std::sqrt(std::max(0.f,1-axis.z*axis.z));
    bool completed=false;float minimum=10000;
    for(int frame=0;frame<fps*8&&traversal.active();++frame) {
        const auto result=traversal.update(world,{0,1,false,true},1.f/fps,1000);
        completed|=result.completed;
        for(unsigned step=0;step<=8;++step)
            minimum=std::min(minimum,world.capsuleDistance(previous+(traversal.position-previous)*(float(step)/8)));
        previous=traversal.position;
    }
    std::cout<<"longitudinal farZ="<<farZ<<" skew="<<skew<<" exterior="<<exterior<<" fps="<<fps<<" done="<<completed<<" minimum="<<minimum<<" terminal="<<terminalDistance<<'\n';
    check(minimum>=30.94f,"oblique longitudinal ridge cannot intersect the full lower capsule between frames");
    if(terminalDistance<31.f)check(!completed,"longitudinal crest with intersecting terminal capsule cannot complete");
    else check(completed,"longitudinal crest with valid full-capsule support remains reachable");
}
static void dynamicSkewed(int change) {
    CrestWorld world(.601809323f,.379001409f,.402767f);
    world.yaw=.633f;world.origin={134559.219f,36994.7031f,-11691.1309f};
    auto traversal=start(world);bool changed=false,completed=false,released=false;
    for(int frame=0;frame<480&&traversal.active();++frame) {
        if(traversal.state==State::mantle&&!changed&&traversal.progress()>.55f) {
            changed=true;
            if(change==1)world.hole=true;
            if(change==2)world.solids[0].pop_back();
            if(change==3)world.ceiling(125);
        }
        const auto result=traversal.update(world,{0,1,false,true},1.f/60,1000);
        completed|=result.completed;released|=result.released&&!result.completed;
    }
    check(changed,"oblique dynamic-support test reaches a real in-progress crest mantle");
    check(!completed&&released&&!traversal.active(),"missing foot support opposing slope or blocked capsule aborts an in-progress oblique mantle");
}
int main(){
    for(float nz:{.35f,.491f,.502f,.626594f,.69f})for(bool far:{false,true})for(float yaw:{0.f,.633f})for(int fps:{30,60,120})positive(nz,far,yaw,fps);
    for(int kind=0;kind<4;++kind)negative(kind);dynamicAndCapture(false);dynamicAndCapture(true);
    for(float width:{4.f,12.f,20.f,28.f,32.f})for(bool far:{false,true})for(float yaw:{0.f,.633f})for(int fps:{30,60,120})capped(width,far,yaw,fps);
    for(int kind=0;kind<4;++kind)cappedNegative(kind);
    for(int fps:{30,60,120})cappedBlendedNormal(fps);
    dynamicAndCapture(false,20);dynamicAndCapture(true,20);
    for(bool far:{false,true})for(float skew:{-.402767f,0.f,.402767f})for(int fps:{30,60,120})skewed(far,skew,fps);
    for(int kind=1;kind<=4;++kind)skewed(true,.402767f,60,kind);
    for(bool far:{false,true})for(float farZ:{.301f,.379001409f})for(float skew:{-.64f,-.50f,.50f,.64f})for(int fps:{30,60,120})longitudinalClearance(far,farZ,skew,fps);
    for(int change=1;change<=3;++change)dynamicSkewed(change);
    std::cout<<"roof crest failures="<<failures<<'\n';return failures?1:0;
}
