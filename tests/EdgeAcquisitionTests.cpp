#include "Pose.h"
#include "TraversalCapture.h"
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace fc;
static void require(bool condition,const char* reason) {if(!condition)throw std::runtime_error(reason);}
static constexpr float sourceLip=138.12f,lowerLip=76.12f;

struct ContextWorld:World {
    struct Box {Vec low,high;bool climbable=true;};
    std::vector<Box> boxes;
    Vec origin{};
    float yaw{};
    unsigned rays{};
    Vec rotate(Vec p,float angle) const {
        return {p.x*std::cos(angle)-p.y*std::sin(angle),p.x*std::sin(angle)+p.y*std::cos(angle),p.z};
    }
    Vec point(Vec p) const {return origin+rotate(p,yaw);}
    Vec vector(Vec p) const {return rotate(p,yaw);}
    Vec local(Vec p) const {return rotate(p-origin,-yaw);}
    std::optional<Hit> ray(Vec a,Vec b) override {
        ++rays;a=local(a);b=local(b);const Vec d=b-a;
        std::optional<Hit> result;float nearest=2;
        const float start[]{a.x,a.y,a.z},direction[]{d.x,d.y,d.z};
        for(const auto& box:boxes) {
            const float low[]{box.low.x,box.low.y,box.low.z},high[]{box.high.x,box.high.y,box.high.z};
            for(int axis=0;axis<3;++axis)for(int side=0;side<2;++side) {
                if(std::abs(direction[axis])<1e-7f)continue;
                const float t=((side?high[axis]:low[axis])-start[axis])/direction[axis];
                if(t<0||t>1||t>=nearest)continue;
                const Vec hit=a+d*t;const float coordinate[]{hit.x,hit.y,hit.z};bool within=true;
                for(int other=0;other<3;++other)if(other!=axis)
                    within&=coordinate[other]>=low[other]-.0001f&&coordinate[other]<=high[other]+.0001f;
                if(!within)continue;
                Vec normal{};const float sign=side?1.f:-1.f;
                if(axis==0)normal.x=sign;else if(axis==1)normal.y=sign;else normal.z=sign;
                nearest=t;result=Hit{point(hit),vector(normal),box.climbable};
            }
        }
        return result;
    }

    float capsuleDistance(Vec feet,float radius=31,float height=138) const {
        feet=local(feet);const float bottom=feet.z+radius,top=feet.z+height-radius;
        float nearest=100000;
        for(const auto& box:boxes) {
            const float dx=std::max({box.low.x-feet.x,0.f,feet.x-box.high.x});
            const float dy=std::max({box.low.y-feet.y,0.f,feet.y-box.high.y});
            const float dz=std::max({box.low.z-top,0.f,bottom-box.high.z});
            nearest=std::min(nearest,std::sqrt(dx*dx+dy*dy+dz*dz));
        }
        return nearest;
    }
};
static ContextWorld shelves() {
    ContextWorld w;
    w.boxes={{{-500,8,-500},{500,100,500}},
        {{-500,0,-500},{500,8,lowerLip}},
        {{-500,0,sourceLip-8},{500,8,sourceLip}}};
    return w;
}
static Traversal attach(ContextWorld& w) {
    Traversal t;t.cfg.gap=37;t.cfg.radius=31;t.cfg.height=138;t.cfg.approachSeconds=0;
    require(t.attach(w,w.point({0,-37,0}),w.vector({0,1,0}),1000),"fixture attaches through actual collision geometry");
    require((w.local(t.position)-Vec{0,-37,0}).length()<.05f,"attachment preserves the expected clear body location");
    return t;
}
static void clearCapsule(const ContextWorld& w,const Traversal& t) {
    require(w.capsuleDistance(t.position,t.cfg.radius,t.cfg.height)>=t.cfg.radius-.05f,
        "independent continuous solid distance finds capsule penetration");
}
static Result tick(ContextWorld& w,Traversal& t,Input input,float dt) {
    const auto r=t.update(w,input,dt,1000);clearCapsule(w,t);return r;
}
static void recordedTick(ContextWorld& w,Traversal& t,Input input,float dt) {
    auto tape=std::make_unique<TraversalCapture>();auto parsed=std::make_unique<TraversalCapture>();
    TraversalCapture::RecordingWorld recording(w,*tape);tape->begin(t,input,dt,1000);
    const auto r=t.update(recording,input,dt,1000);tape->finish(t,r);clearCapsule(w,t);
    std::string error;require(tape->complete(),"context action capture remains within the bounded ray budget");
    require(parsed->deserialize(tape->serialize(),error),"context action capture serializes all fields");
    const auto replay=parsed->replay();
    if(!replay.matched)std::cerr<<"context capture: "<<replay.error<<'\n';
    require(replay.matched,"serialized contact targets reproduce the same ordered queries and action state");
}

static void enteringSourceWindow() {
    unsigned scenarios=0;
    for(int fps:{30,60,120})for(bool distant:{false,true})for(float offset=7;offset<=45;offset+=1) {
        auto w=shelves();w.yaw=distant?.633f:0;if(distant)w.origin={131316.86f,38643.36f,-11331.41f};
        auto t=attach(w);t.position=w.point({0,-37,offset});t.cfg.downSpeed=78;
        t.cfg.automaticClimbActions=t.cfg.legacyAutomaticHops=t.cfg.surfaceActionVariants=t.cfg.threepeatAnimations=true;
        unsigned maximumRays=0;const Vec start=t.position;
        for(int frame=0;frame<fps*2;++frame) {
            const auto before=w.rays;const Vec old=t.position;const auto r=tick(w,t,{0,-1},1.f/fps);
            maximumRays=std::max(maximumRays,w.rays-before);
            require(r.motion==Motion::down&&!t.preparingEdge()&&!t.usesEdgeTargets(r.motion),
                "crossing every former drop source-height window cannot restore38 or pause ordinary S movement");
            require(!r.released&&t.position.z<old.z,"the actual continuous backing wall supports normal descent");
        }
        require((t.position-start).z<-100&&t.automaticActionCount()==0,"S passes the former capture window with no automatic descending commit");
        require(maximumRays<=TraversalCapture::capacity,"continuous source-window movement retains the query budget");
        ++scenarios;
    }
    std::cout<<"continuous source-window entries="<<scenarios<<'\n';
}
static Vec displayedPoint(Vec point,const Traversal& t) {
    return t.position+Vec{-t.normal.y,t.normal.x,0}*point.x-t.normal*point.y+Vec{0,0,point.z};
}
static void movingPoseEntry(const Library& lib,int fps,float offset) {
    auto w=shelves();auto t=attach(w);t.position=w.point({0,-37,offset});t.cfg.downSpeed=78;
    t.cfg.threepeatAnimations=true;lib.configureThreepeat(t.cfg);
    SurfacePose animator;const float dt=1.f/fps;bool requested=false,started=false,finished=false;float maxPalmError=0,maxStep=0;
    Pose lastPose;std::array<Vec,2> lastPalms{};unsigned planted=0;
    for(int frame=0;frame<fps*4;++frame) {
        Input input=requested?Input{}:Input{0,-1};
        if(!requested&&w.local(t.position).z<=5){input={1,0};input.hop=true;requested=true;}
        const auto r=tick(w,t,input,dt);const auto pose=animator.update(lib,w,t,r.motion,dt,1);
        require((r.motion==Motion::none||isActiveMotion(r.motion)),"moving source acquisition cannot emit removed descending38");
        require(pose.size()==99,"retained lateral acquisition produces the complete live skeleton");
        const auto body=lib.world(pose);std::array<Vec,2> palms{};
        for(int hand=0;hand<2;++hand) {
            require(lib.armBendValid(pose,hand),"moving edge entry preserves elbow anatomy");
            palms[hand]=displayedPoint(lib.palm(body,hand),t);
            if(!lastPose.empty())maxStep=std::max(maxStep,(palms[hand]-lastPalms[hand]).length());
            if(t.usesEdgeTargets(r.motion)) {
                const float phase=t.actionProgress();
                const float source=t.holdsPreparedEdge(r.motion)?t.preparedEdgeWeight(r.motion):
                    threepeatHop(r.motion)?threepeatSourceWeight(r.motion==Motion::contextHopLeft,hand,phase):0.f;
                const float destination=t.holdsDestinationEdge(r.motion)?1.f:
                    threepeatHop(r.motion)?threepeatTargetWeight(r.motion==Motion::contextHopLeft,hand,phase):0.f;
                if(source>.95f){maxPalmError=std::max(maxPalmError,(palms[hand]-(t.edgeHand(hand,false)+Vec{0,0,.8f})).length());++planted;}
                if(destination>.95f){maxPalmError=std::max(maxPalmError,(palms[hand]-(t.edgeHand(hand,true)+Vec{0,0,.8f})).length());++planted;}
            }
        }
        if(!lastPose.empty())for(std::size_t index=0;index<pose.size();++index)
            require(angleBetween(pose[index].q,lastPose[index].q)<=12.566371f*dt+.015f,
                "moving acquisition obeys the existing full-body transition speed bound");
        lastPose=pose;lastPalms=palms;
        if(threepeatHop(r.motion))started=true;
        if(started&&t.state==State::wall){finished=true;break;}
    }
    std::cout<<"moving retained lateral pose fps="<<fps<<" offset="<<offset<<" palm="<<maxPalmError<<" step="<<maxStep<<'\n';
    require(requested&&started&&finished&&planted>fps/8,"actual runtime still reaches lateral source preparation, flight and loaded catch after descent");
    require(maxPalmError<5,"retained lateral source acquisition keeps the original five-unit loaded-palm bound");
    require(maxStep<=600*dt+1,"acquiring a moving source edge cannot teleport displayed hands");
}
static void noEdgeAndInputs() {
    auto geometry=shelves();auto enabled=attach(geometry),disabled=attach(geometry);
    enabled.position=disabled.position=geometry.point({0,-37,30});disabled.cfg.contextActions=false;
    enabled.cfg.downSpeed=disabled.cfg.downSpeed=78;
    enabled.cfg.automaticClimbActions=enabled.cfg.legacyAutomaticHops=enabled.cfg.surfaceActionVariants=enabled.cfg.threepeatAnimations=true;
    for(int frame=0;frame<150;++frame) {
        const auto a=tick(geometry,enabled,{0,-1},1.f/60),b=tick(geometry,disabled,{0,-1},1.f/60);
        require((a.motion==Motion::none||isActiveMotion(a.motion))&&(b.motion==Motion::none||isActiveMotion(b.motion))&&!enabled.preparingEdge()&&!disabled.preparingEdge(),
            "neither enabled nor disabled legacy options can restore the removed descending action");
        require(a.motion==b.motion&&(enabled.position-disabled.position).length()<.0001f,
            "old context settings cannot alter ordinary S across formerly eligible lips");
    }
    ContextWorld smooth;smooth.boxes={{{-500,0,-500},{500,100,500}}};
    enabled=attach(smooth);disabled=attach(smooth);disabled.cfg.contextActions=false;
    for(int frame=0;frame<80;++frame) {
        const auto a=tick(smooth,enabled,{0,-1},1.f/60),b=tick(smooth,disabled,{0,-1},1.f/60);
        require(a.motion==b.motion&&(enabled.position-disabled.position).length()<.0001f,
            "responsive probes cannot invent a hold or alter a continuous wall's motion");
    }
    for(bool release:{false,true}) {
        auto w=shelves();auto t=attach(w);t.position=w.point({0,-37,30});Input input=release?Input{0,-1}:Input{1,0};
        input.hop=true;input.run=!release;input.release=input.backDrop=release;
        const auto r=tick(w,t,input,1.f/60);
        require(!t.usesEdgeTargets(r.motion)&&(r.motion==Motion::none||isActiveMotion(r.motion)),
            "responsive source acquisition preserves wall-run Space prohibition and S+Space outward release");
    }
    auto shiftedWorld=shelves();auto shifted=attach(shiftedWorld),plain=attach(shiftedWorld);
    shifted.position=plain.position=shiftedWorld.point({0,-37,30});plain.cfg.contextActions=false;
    Input shiftedDown{0,-1};shiftedDown.run=true;
    for(int frame=0;frame<100;++frame) {
        const auto a=tick(shiftedWorld,shifted,shiftedDown,1.f/60),b=tick(shiftedWorld,plain,{0,-1},1.f/60);
        require(!shifted.preparingEdge()&&!shifted.wallRunning()&&(a.motion==Motion::none||isActiveMotion(a.motion))&&
            a.motion==b.motion&&(shifted.position-plain.position).length()<.0001f,
            "held Shift+S cannot repeatedly start/cancel preparation or interrupt ordinary descending movement");
    }
}
static void preparationSafety() {

    for(bool ceiling:{false,true}) {
        auto w=shelves();auto t=attach(w);t.position=w.point({0,-37,ceiling?-6.f:13.f});
        if(ceiling)w.boxes.push_back({{-80,-90,136},{80,-10,146}});
        else w.boxes[1].high.z=120;
        const auto before=t.position;const auto r=tick(w,t,{0,-1},1.f/60);
        require(!t.preparingEdge()&&(r.motion==Motion::none||isActiveMotion(r.motion))&&t.position.z<before.z,
            "missing destination or blocked source alignment cannot pause ordinary descent");
    }
    for(int variant=0;variant<4;++variant) {
        auto w=shelves();auto t=attach(w);t.position=w.point({0,-37,13});
        auto first=tick(w,t,{0,-1},1.f/60);
        require(!t.preparingEdge()&&first.motion==Motion::down&&first.staminaCost<1,
            "former downward preparation now remains ordinary movement with no unstarted action charge");
        Input input{0,-1};
        if(variant==0)input={};
        if(variant==1)input={0,1};
        if(variant==2)w.boxes.erase(w.boxes.begin()+2);
        if(variant==3)w.boxes.erase(w.boxes.begin()+1);
        const auto r=tick(w,t,input,1.f/60);
        require(!t.preparingEdge()&&!t.usesEdgeTargets(r.motion)&&(r.motion==Motion::none||isActiveMotion(r.motion)),
            "input changes and changed real geometry never resurrect removed preparation");
        require(t.active()&&!r.released,"preparation cancellation preserves an otherwise supported continuous wall state");
    }

    auto w=shelves();auto t=attach(w);t.position=w.point({0,-37,13});Input hop{1,0};hop.hop=true;
    auto r=tick(w,t,hop,1.f/60);require(t.preparingEdge(),"off-height one-frame manual hop creates preparation");
    unsigned commits=0;float cost=r.staminaCost;
    for(int frame=0;frame<180;++frame) {
        const bool wasPreparing=t.preparingEdge();r=tick(w,t,{},1.f/60);cost+=r.staminaCost;
        if(wasPreparing&&t.state==State::action){++commits;require(r.staminaCost==15,"latched hop charges its one action cost at commit");}
        if(commits&&t.state==State::wall)break;
    }
    require(commits==1&&cost>=15&&cost<20&&t.state==State::wall,
        "one Space request commits exactly once and returns to live hanging after catch");

    w=shelves();t=attach(w);t.position=w.point({0,-37,13});
    recordedTick(w,t,hop,1.f/60);unsigned preparedFrames=0,actionFrames=0;
    for(int frame=0;frame<180;++frame) {
        preparedFrames+=t.preparingEdge();actionFrames+=t.state==State::action;
        recordedTick(w,t,{},1.f/60);
        if(actionFrames&&t.state==State::wall)break;
    }
    require(preparedFrames>10&&actionFrames>20,"exact captures cover alignment, settle, commit, flight and target catch");

    w=shelves();w.boxes.push_back({{-500,0,sourceLip+62-8},{500,8,sourceLip+62}});
    t=attach(w);t.position=w.point({0,-37,13});Input spaceOnly;spaceOnly.hop=true;
    r=tick(w,t,spaceOnly,1.f/60);
    require(t.preparingEdge()&&t.state==State::wall,"off-height Space alone queues a checked upward edge hop");
    const auto beforeCancel=t.position;r=tick(w,t,{0,-1},1.f/60);
    require(!t.preparingEdge()&&!t.usesEdgeTargets(r.motion)&&r.motion==Motion::down&&t.position.z<beforeCancel.z,
        "S contradicts a Space-only upward plan and immediately restores ordinary downward climbing");
}
static void boundedLateralCandidates() {
    for(int side:{-1,1})for(bool distant:{false,true}) {
        auto w=shelves();w.boxes[2].low.x=-26;w.boxes[2].high.x=26;
        w.boxes.push_back({{side>0?40.f:-92.f,0,sourceLip-8},{side>0?92.f:-40.f,8,sourceLip}});
        w.yaw=distant?.633f:0;if(distant)w.origin={131316.86f,38643.36f,-11331.41f};
        auto t=attach(w);Input hop{float(side),0};hop.hop=true;
        const auto r=tick(w,t,hop,1.f/60);
        require(t.state==State::action&&t.usesEdgeTargets(r.motion),
            "narrow adjacent edge can be acquired at a checked source-compatible distance");
        require(std::abs(w.local(t.edgeTarget()).x-side*66)<.1f,
            "the actual nearby shelf determines travel instead of an unconditional ninety-unit destination");
        for(int frame=0;t.state==State::action&&frame<100;++frame) {
            const auto next=tick(w,t,{},1.f/60);require(!next.released,"shorter checked edge hop retains real support");
        }
        require(t.state==State::wall,"bounded adjacent-edge hop finishes with normal wall controls");
    }
}
int main(int argc,char** argv) {
    try {
        enteringSourceWindow();noEdgeAndInputs();preparationSafety();boundedLateralCandidates();
        if(argc>1) {
            Library lib;require(lib.load(argv[1]),"moving acquisition requires the actual runtime motion library");
            unsigned failures=0;for(int fps:{30,60,120})for(float offset:{13.f,30.f,44.f}) { try {movingPoseEntry(lib,fps,offset);} catch(const std::exception& error){++failures;std::cerr<<"moving acquisition: "<<error.what()<<'\n';} }
            require(!failures,"all moving context pose entries must satisfy the existing contact and transition limits");
        }
        std::cout<<"edge acquisition timing, geometry, controls and moving-pose checks passed\n";return 0;
    }catch(const std::exception& error){std::cerr<<"edge acquisition failure: "<<error.what()<<'\n';return 1;}
}
