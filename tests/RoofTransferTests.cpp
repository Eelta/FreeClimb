#include "Core.h"
#include "TraversalCapture.h"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <limits>
using namespace fc;
namespace {
struct Plane{Vec normal;float d;};
using Solid=std::vector<Plane>;
Solid box(Vec low,Vec high){return {{{1,0,0},high.x},{{-1,0,0},-low.x},{{0,1,0},high.y},{{0,-1,0},-low.y},{{0,0,1},high.z},{{0,0,-1},-low.z}};}
struct RoofWorld:World {
    std::vector<Solid> solids;Vec origin{};float yaw{},roofSign=1,depth=300;unsigned casts{};
    RoofWorld(float sign=1,float roofDepth=300):roofSign(sign),depth(roofDepth) {
        const float nz=.512723f,nx=std::sqrt(1-nz*nz)*sign;
        solids.push_back({{{1,0,0},200},{{-1,0,0},200},{{0,1,0},depth},{{0,-1,0},0},{{0,0,-1},300},{{nx,0,nz},nz*70}});
    }
    Vec rotate(Vec p,float angle)const{return {p.x*std::cos(angle)-p.y*std::sin(angle),p.x*std::sin(angle)+p.y*std::cos(angle),p.z};}
    Vec global(Vec p)const{return origin+rotate(p,yaw);}
    Vec local(Vec p)const{return rotate(p-origin,-yaw);}
    std::optional<Hit> ray(Vec from,Vec to)override {
        ++casts;const Vec a=local(from),delta=local(to)-a;std::optional<Hit> hit;double nearest=2;
        for(const auto& solid:solids) {
            double enter=0,leave=1;Vec normal{};bool rejected=false;
            for(const auto& plane:solid) {
                const double distance=a.dot(plane.normal)-plane.d,rate=delta.dot(plane.normal);
                if(std::abs(rate)<1.e-9){if(distance>0){rejected=true;break;}continue;}
                const double phase=-distance/rate;
                if(rate<0){if(phase>enter){enter=phase;normal=plane.normal;}}else leave=std::min(leave,phase);
                if(leave<enter){rejected=true;break;}
            }
            if(!rejected&&enter>1.e-6&&enter<=1&&enter<nearest&&normal.length()>.9f){nearest=enter;hit=Hit{global(a+delta*float(enter)),rotate(normal,yaw),true};}
        }
        return hit;
    }
    float cylinderDistance(Vec globalFeet)const {
        const Vec p=local(globalFeet);float low=-200,high=200;
        for(const auto& plane:solids[0])if(plane.normal.z>0&&std::abs(plane.normal.x)>.1f) {
            const float nx=plane.normal.x*roofSign;
            const float boundary=(plane.d-plane.normal.z*(p.z+6))/nx;
            if(nx>0)high=std::min(high,boundary);else low=std::max(low,boundary);
        }
        if(low>high)return 10000;
        const float x=std::clamp(p.x*roofSign,low,high);
        return std::hypot(p.x*roofSign-x,p.y-std::clamp(p.y,0.f,depth));
    }
};
static void dimensions(Traversal& t){t.cfg.radius=31;t.cfg.gap=37;t.cfg.height=138;t.cfg.approachSeconds=0;}
struct Run{bool attached{},transferred{},complete{},released{},safe=true;unsigned actions{};float maxYawStep{},minimumClearance=10000;Vec final{};float finalNormalZ{};State state{};};
Run run(RoofWorld& w,int fps,bool sprint=false,bool changePath=false,bool finalSideways=false) {
    Traversal t;dimensions(t);Run r;r.attached=t.attach(w,w.global({0,-37,-120}),w.rotate({0,1,0},w.yaw),100);
    Vec priorNormal=t.normal;bool inserted=false;Result result;
    for(int frame=0;frame<fps*4&&t.active();++frame) {
        if(changePath&&r.transferred&&!inserted){w.solids.push_back(box({-150,-100,180},{150,100,184}));inserted=true;}
        const bool sideways=finalSideways&&frame>=fps*3;

        result=t.update(w,{sideways?1.f:0.f,sideways?0.f:1.f,false,!finalSideways,false,false,sprint},1.f/fps,100);
        if(std::string(t.blockedReason)=="checked steep roof transfer")r.transferred=true;
        if(t.state==State::action)++r.actions;
        r.minimumClearance=std::min(r.minimumClearance,w.cylinderDistance(t.position));
        r.safe&=r.minimumClearance>=30.98f;
        r.maxYawStep=std::max(r.maxYawStep,std::acos(std::clamp(priorNormal.dot(t.normal),-1.f,1.f)));
        priorNormal=t.normal;r.complete|=result.completed;r.released|=result.released;
    }
    r.final=w.local(t.position);r.finalNormalZ=t.surfaceNormal.z;r.state=t.state;return r;
}
}
int main(int argc,char**argv){
    int failures=0,positives=0,negatives=0;
    auto require=[&](bool value,const char* message){if(!value){++failures;std::cerr<<"FAIL "<<message<<'\n';}};
    {
        RoofWorld world;world.yaw=.73f;Traversal traversal;dimensions(traversal);
        require(traversal.attach(world,world.global({0,-37,-120}),world.rotate({0,1,0},world.yaw),100),"capture fixture attaches to the real wall");
        auto tape=std::make_unique<TraversalCapture>();auto loaded=std::make_unique<TraversalCapture>();
        TraversalCapture::RecordingWorld recorder(world,*tape);unsigned frames=0;
        for(int frame=0;frame<240&&traversal.active();++frame) {
            Input input{0,1,false,true};
            if(traversal.state!=State::action){traversal.update(world,input,1.f/60,100);continue;}
            tape->begin(traversal,input,1.f/60,100);auto result=traversal.update(recorder,input,1.f/60,100);tape->finish(traversal,result);
            std::string error;require(tape->complete()&&loaded->deserialize(tape->serialize(),error),"roof action snapshot remains complete and readable");
            require(loaded->replay().matched,"roof-specific source/target normals, flag and cooldown round-trip exactly during playback");++frames;
        }
        require(frames>20,"same-version captures actually exercise the rotating roof-transfer action");
        std::cout<<"roof action snapshots="<<frames<<'\n';
    }
    for(int fps:{30,48,120})for(float sign:{-1.f,1.f})for(float yaw:{0.f,1.57079632679f,.73f})for(bool far:{false,true}) {
        RoofWorld world(sign);world.yaw=yaw;if(far)world.origin={131065.836f,41791.117f,-11204.176f};
        const auto r=run(world,fps);++positives;
        std::cout<<"roof fps="<<fps<<" sign="<<sign<<" yaw="<<yaw<<" far="<<far<<" transfer="<<r.transferred<<" state="<<int(r.state)
            <<" complete="<<r.complete<<" p="<<r.final.x<<','<<r.final.y<<','<<r.final.z<<" normalz="<<r.finalNormalZ<<" clear="<<r.minimumClearance<<" yawStep="<<r.maxYawStep<<'\n';
        require(r.attached,"closed roof starts on a genuinely supported front wall");
        require(r.transferred&&r.actions>0&&r.final.z>130&&r.state==State::wall,"ordinary climb transfers to the true steep roof and continues upward");
        require(!r.complete&&!r.released&&std::abs(r.finalNormalZ-.512723f)<.001f,"steep roof remains climbing rather than fake ground completion");
        require(r.maxYawStep<.27f,"ninety degree facing change occurs continuously before landing");
        require(r.safe,"actual closed wedge cross section stays outside unchanged radius31");
    }
    for(int fps:{30,120})for(bool far:{false,true}) {
        RoofWorld small(1,8);small.yaw=.73f;if(far)small.origin={131065.836f,41791.117f,-11204.176f};
        const auto narrow=run(small,fps);require(!narrow.transferred,"a roof without a genuine upper grip patch cannot authorize transfer");++negatives;
        RoofWorld blocked;blocked.solids.push_back(box({-150,-100,180},{150,100,184}));blocked.yaw=.73f;if(far)blocked.origin=small.origin;
        const auto ceiling=run(blocked,fps);require(!ceiling.transferred,"the transfer must not lift through an overhead obstruction");++negatives;
        RoofWorld running;running.yaw=.73f;if(far)running.origin=small.origin;
        const auto sprint=run(running,fps,true);require(!sprint.transferred&&sprint.actions==0,"holding Shift must never start a roof-transfer jump");++negatives;
        RoofWorld dynamic;dynamic.yaw=.73f;if(far)dynamic.origin=small.origin;
        const auto changed=run(dynamic,fps,false,true);require(changed.transferred&&changed.released&&!changed.complete,"an obstruction entering the accepted arc aborts on runtime revalidation");++negatives;
        RoofWorld gable;const Vec otherNormal{-std::sqrt(1-.512723f*.512723f),0,.512723f};
        const Vec ridge{-2.17f,0,73.6332f};gable.solids[0].push_back({otherNormal,otherNormal.dot(ridge)});
        gable.yaw=.73f;if(far)gable.origin=small.origin;
        const auto ridgeCatch=run(gable,fps,false,false,true);
        require(ridgeCatch.transferred&&ridgeCatch.state==State::wall&&!ridgeCatch.complete&&!ridgeCatch.released,
            "a lower genuine gable catch preserves climbing rather than standing on the sharp ridge");
        require(ridgeCatch.final.y>70&&ridgeCatch.safe,"the gable catch restores real sideways movement without crossing the radius31 body into the roof");
        ++positives;
    }
    std::cout<<"RoofTransferTests positives="<<positives<<" negativeGroups="<<negatives<<" failures="<<failures<<'\n';
    return failures?1:0;
}
