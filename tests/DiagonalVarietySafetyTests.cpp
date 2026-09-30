#include "Pose.h"
#include "CornerTestWorld.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
static void require(bool value,const char* reason){if(!value)throw std::runtime_error(reason);}

struct DiagonalWorld final:World {
    fc_test::CornerWorld geometry;
    float pitch{};
    Vec origin{};
    bool queryUnavailable{};
    unsigned calls{};
    Vec rotate(Vec p,float angle)const{return {p.x,p.y*std::cos(angle)-p.z*std::sin(angle),p.y*std::sin(angle)+p.z*std::cos(angle)};}
    Vec point(Vec p)const{return origin+rotate(p,pitch);}
    Vec local(Vec p)const{return rotate(p-origin,-pitch);}
    Vec direction(Vec p)const{return rotate(p,pitch);}
    std::optional<Hit> ray(Vec a,Vec b)override {
        ++calls;

        if(queryUnavailable)return Hit{a,(a-b).unit(),false};
        const auto h=geometry.ray(local(a),local(b));
        if(!h)return {};
        return Hit{point(h->point),direction(h->normal),h->climbable};
    }
};
static DiagonalWorld wall(float pitch=0) {
    DiagonalWorld w;w.pitch=pitch;w.geometry.boxes={{{-20000,0,-10000},{20000,1000,20000}}};return w;
}
static Traversal attached(DiagonalWorld& w,const Library& lib) {
    Traversal t;t.cfg=fc_test::settings();t.cfg.approachSeconds=.01f;
    require(lib.configureThreepeat(t.cfg),"actual production library calibration");
    t.cfg.threepeatAnimations=t.cfg.surfaceActionVariants=t.cfg.automaticClimbActions=true;
    t.cfg.autoActionMinSeconds=1;t.cfg.autoActionMaxSeconds=1.6f;
    require(t.attach(w,w.point({0,-45,300}),{0,1,0},1000,60),"real box face attaches");
    t.update(w,{},.05f,1000);require(t.state==State::wall,"entry completes before test");return t;
}
static Traversal pending(DiagonalWorld& w,const Library& lib,int fps,int side) {
    auto t=attached(w,lib);
    for(int frame=0;frame<fps*4&&!t.preparingEdge();++frame) {
        const auto r=t.update(w,{float(side),1},1.f/fps,1000);
        require(!r.released,"diagonal setup preserves support");
    }
    require(t.preparingEdge()&&t.automaticActionCount()==0,"actual diagonal wall route prepares before unique commit");
    return t;
}
static void cancelledIntent(const Library& lib,int fps,int side) {
    auto w=wall();const auto prepared=pending(w,lib,fps,side);const float dt=1.f/fps;
    Input run{float(side),1};run.run=true;
    const Input inputs[]={Input{},run,Input{float(-side),1},Input{float(side),-1},Input{0,1}};
    for(const auto input:inputs) {
        auto t=prepared;const auto r=t.update(w,input,dt,1000);
        require(!t.preparingEdge()&&t.automaticActionCount()==0&&t.surfaceActionCount()==0&&!threepeatHop(r.motion),
            "stop, Shift, reverse side, descending and pure-up cancel stale diagonal preparation");
    }
    for(float stamina:{14.f,0.f}) {
        auto t=prepared;const auto r=t.update(w,{float(side),1},dt,stamina);
        require(!t.preparingEdge()&&t.automaticActionCount()==0&&!threepeatHop(r.motion),"insufficient stamina cannot commit prepared leap");
        if(stamina==0)require(r.released&&!t.active(),"exhaustion physically releases");
    }
    auto t=prepared;w.queryUnavailable=true;
    const auto r=t.update(w,{float(side),1},dt,1000);
    require(!t.preparingEdge()&&t.automaticActionCount()==0&&!threepeatHop(r.motion),"unavailable blocking queries never authorize stale captured travel");
}
static void changeEndpoint(DiagonalWorld& w,Vec source,float span,int side,bool changeSource) {
    const float seam=source.x+side*span*.5f;
    const bool leftIsSource=side>0;
    const float leftY=(leftIsSource==changeSource)?5.f:0.f;
    const float rightY=(leftIsSource!=changeSource)?5.f:0.f;
    w.geometry.boxes={{{-20000,leftY,-10000},{seam,1000,20000}},{{seam,rightY,-10000},{20000,1000,20000}}};
}
static void changedGeometry(const Library& lib,int fps,int side) {
    const float dt=1.f/fps;
    for(bool source:{false,true}) {
        auto w=wall();auto t=pending(w,lib,fps,side);
        changeEndpoint(w,w.local(t.position),t.cfg.threepeatHopDistance[side<0?0:1],side,source);
        const auto r=t.update(w,{float(side),1},dt,1000);
        require(!t.preparingEdge()&&t.automaticActionCount()==0&&!threepeatHop(r.motion),
            "moving only one prepared physical endpoint invalidates its stored contact");
    }
    auto w=wall();auto t=pending(w,lib,fps,side);
    for(int frame=0;frame<fps&&t.automaticActionCount()==0;++frame) {
        const auto r=t.update(w,{float(side),1},dt,1000);
        if(t.automaticActionCount())require(r.staminaCost>=15&&r.staminaCost<16,
            "the unique diagonal commit charges exactly the existing fifteen-point action cost");
    }
    require(t.automaticActionCount()==1&&t.surfaceActionCount()==1&&t.state==State::action,"diagonal action commits once");
    const Vec source=w.local(t.edgeStart());
    const Vec displacement=w.local(t.edgeTarget())-source;
    require(displacement.x*side>50&&displacement.z>20&&displacement.z<=32.05f,
        "committed diagonal body displacement gains bounded real height in requested side");
    while(t.active()&&t.actionProgress()<.30f)t.update(w,{float(side),1},dt,1000);
    require(t.active()&&t.state==State::action,"test reaches checked flight before target mutation");
    changeEndpoint(w,source,std::abs(displacement.x),side,false);
    const auto lost=t.update(w,{float(side),1},dt,1000);
    require(lost.released&&!lost.completed&&!t.active()&&t.automaticActionCount()==1,
        "changed target in flight releases rather than catching old air or recounting action");
}
static void middleObstacle(const Library& lib,int fps,int side) {
    auto w=wall();auto t=pending(w,lib,fps,side);const float dt=1.f/fps;
    const Vec start=w.local(t.position);const float span=t.cfg.threepeatHopDistance[side<0?0:1];

    const float middle=start.x+side*span*.5f;
    w.geometry.boxes.push_back({{middle-8,-100,start.z+t.cfg.height+32},
        {middle+8,-5,start.z+t.cfg.height+45}});
    bool cancelled=false;
    for(int frame=0;frame<fps/2;++frame) {
        const auto r=t.update(w,{float(side),1},dt,1000);
        require(!r.released,"middle obstacle retains source climbing support");
        require(t.automaticActionCount()==0&&!threepeatHop(r.motion),"full precommit curved route rejects raised middle obstacle");
        require(w.geometry.clearance(w.local(t.position),t.cfg)>=t.cfg.radius-.06f,"blocked variation never reduces actual capsule clearance");
        if(!t.preparingEdge()){cancelled=true;break;}
    }
    require(cancelled,"full route obstruction cancels pending captured action");
}
static void ineligibleInputsAndSlopes(const Library& lib,int fps) {
    for(Input input:{Input{0,1},Input{.25f,1},Input{1,-1}}) {
        auto w=wall();auto t=attached(w,lib);
        for(int frame=0;frame<fps*4;++frame) {
            const auto r=t.update(w,input,1.f/fps,1000);
            require(!r.released,"ordinary incompatible direction remains supported");
            require(!threepeatHop(r.motion)&&(r.motion==Motion::none||isActiveMotion(r.motion)),
                "pure upward, strongly upward and descending diagonal input never choose new side or pure-down captures");
        }
    }
    for(float slope:{-.13f,.13f}) {
        auto w=wall(std::asin(slope));auto t=attached(w,lib);
        require(std::abs(t.surfaceNormal.z)>.12f,"independent pitched face is beyond calibrated near-vertical band");
        for(int frame=0;frame<fps*3;++frame) {
            const auto r=t.update(w,{1,0},1.f/fps,1000);
            require(!r.released,"unsupported capture slope retains ordinary climbing");
            require(!threepeatHop(r.motion)&&t.surfaceActionCount()==0,"inclined geometry never bypasses calibrated capture slope gate");
        }
    }
}
int main(int argc,char**argv){try{
    require(argc==2,"provide actual licensed 42-motion library");Library lib;
    require(lib.load(argv[1])&&lib.hasThreepeat(),"production captured library loads");
    for(int fps:{30,60,120}) {
        for(int side:{-1,1}){cancelledIntent(lib,fps,side);changedGeometry(lib,fps,side);middleObstacle(lib,fps,side);}
        ineligibleInputsAndSlopes(lib,fps);
        std::cout<<"PASS diagonal safety fps="<<fps<<" intent/stamina/endpoints/middle-obstacle/slopes\n";
    }
    return 0;
}catch(const std::exception&e){std::cerr<<"FAIL diagonal safety: "<<e.what()<<'\n';return 1;}}
