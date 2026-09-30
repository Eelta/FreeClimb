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
static void descentWithoutRegrab(int fps,bool distant,float angle) {
    auto w=shelves();w.yaw=angle;if(distant)w.origin={131316.86f,38643.36f,-11331.41f};
    auto t=attach(w);const float dt=1.f/fps;const Vec before=t.position;
    t.cfg.automaticClimbActions=t.cfg.legacyAutomaticHops=t.cfg.surfaceActionVariants=t.cfg.threepeatAnimations=true;
    for(int frame=0;frame<fps*2;++frame) {
        const Vec old=t.position;const auto result=tick(w,t,{0,-1},dt);
        require(result.motion==Motion::down&&t.state==State::wall&&!t.preparingEdge()&&!t.usesEdgeTargets(result.motion),
            "S stays continuous descent across formerly valid paired lips even with every old action option enabled");
        require(!result.released&&!result.completed&&t.position.z<old.z,"supported ordinary descent neither locks nor releases");
        require(result.staminaCost<1,"continuous descent never charges a contextual jump cost");
        const Vec position=t.position;const auto duplicate=t.update(w,{0,-1},0,1000);
        require((position-t.position).length()==0&&(duplicate.motion==Motion::none||isActiveMotion(duplicate.motion)),
            "duplicate callbacks neither advance nor resurrect the removed drop");
    }
    require((t.position-before).z<-62,"ordinary descent passes the former sixty-two-unit lower-catch destination");
    require(t.automaticActionCount()==0,"removed descending action cannot inflate automatic commit counts");
    const Vec stopped=t.position;tick(w,t,{},dt);
    require((t.position-stopped).length()<.01f,"releasing S immediately leaves a normal supported stop");
    for(int i=0;i<fps/3;++i)tick(w,t,{1,0},dt);
    require(t.active()&&(t.position-stopped).dot(w.vector({1,0,0}))>15,"side movement remains available after ordinary descent");
    std::cout<<"no descending regrab fps="<<fps<<" far="<<distant<<" yaw="<<angle<<'\n';
}
static void contextualHops() {
    for(int direction:{-1,0,1})for(int fps:{30,60,120}) {
        auto w=shelves();
        if(direction==0)w.boxes.push_back({{-500,0,sourceLip+62-8},{500,8,sourceLip+62}});
        auto t=attach(w);Input input{float(direction),direction==0?1.f:0.f};input.hop=true;
        const auto r=tick(w,t,input,1.f/fps);
        require(t.state==State::action&&hopMotion(r.motion)&&t.usesEdgeTargets(r.motion),
            "ordinary Space prefers real measured lateral or upper-edge contacts");
        const auto destination=t.edgeTarget();
        for(int frame=0;t.state==State::action&&frame<fps*2;++frame) {
            const auto next=tick(w,t,{},1.f/fps);require(!next.released,"measured edge hop preserves support through catch");
        }
        require(t.state==State::wall&&(t.position-destination).length()<.05f,"edge hop lands on its checked target");
    }
}
static void negativeAndLegacy() {
    for(int variant=0;variant<6;++variant) {
        auto w=shelves();
        if(variant==0)w.boxes={{{-500,0,-500},{500,100,500}}};
        if(variant==1){w.boxes[0].low.z=0;w.boxes[1].low.z=0;}
        if(variant==2)w.boxes.push_back({{-40,-80,146},{40,-10,152}});
        if(variant==3)w.boxes[2].high.x=12;
        if(variant==4)w.boxes[1].high.z=120;
        auto t=attach(w);if(variant==5)t.cfg.contextActions=false;
        const auto before=t.position;const auto r=tick(w,t,{0,-1},1.f/60);
        require((r.motion==Motion::none||isActiveMotion(r.motion))&&!t.usesEdgeTargets(r.motion),
            "the removed drop stays disabled on missing geometry, foot support, body route or disabled settings");
        require(t.active()&&!r.released,"rejected contextual action keeps existing climb behavior");
        require(t.position.z<before.z,"failed contextual planning falls back to ordinary downward climbing");
    }

    for(int fps:{30,60,120}) {
        auto missing=shelves();auto t=attach(missing);tick(missing,t,{0,-1},1.f/fps);
        const Vec supported=t.position;missing.boxes.clear();bool released=false;
        for(int frame=0;frame<fps*2&&t.active();++frame) {
            const auto r=tick(missing,t,{0,-1},1.f/fps);
            require((r.motion==Motion::none||isActiveMotion(r.motion))&&!t.preparingEdge()&&!t.usesEdgeTargets(r.motion),
                "missing backing geometry cannot reactivate a descending transfer");
            require((t.position-supported).length()<.01f,"ordinary S never advances without measured support");
            released|=r.released;
        }
        require(released&&!t.active(),"lost real support keeps the original bounded retry and physical release policy");
    }

    ContextWorld w;w.boxes={{{-500,0,-500},{500,100,500}}};
    auto enabled=attach(w),disabled=attach(w);disabled.cfg.contextActions=false;
    for(int frame=0;frame<80;++frame) {
        const Input input{frame<30?1.f:0.f,frame<30?0.f:-1.f};
        const auto a=tick(w,enabled,input,1.f/60),b=tick(w,disabled,input,1.f/60);
        require(a.motion==b.motion&&enabled.state==disabled.state&&(enabled.position-disabled.position).length()<.0001f,
            "smooth-wall movement is unchanged when contextual planning finds no real edges");
    }
}
static void inputPreservation() {
    for(int side:{-1,0,1}) {
        auto w=shelves();auto t=attach(w);Input run{float(side),side==0?1.f:0.f};run.run=run.hop=true;
        const auto r=tick(w,t,run,1.f/60);
        require(!t.usesEdgeTargets(r.motion)&&!hopMotion(r.motion)&&(r.motion==Motion::none||isActiveMotion(r.motion)),
            "Shift wall-run plus Space never starts an edge jump or lower catch");
        run.run=false;
        const auto releasedShift=tick(w,t,run,1.f/60);
        require(!hopMotion(releasedShift.motion)&&!t.usesEdgeTargets(releasedShift.motion),
            "simultaneous Shift release and Space cannot leak into an ordinary context hop");
    }
    auto w=shelves();auto t=attach(w);Input release{0,-1};release.release=release.backDrop=true;
    const auto r=tick(w,t,release,1.f/60);
    require((r.motion==Motion::dropBack||r.motion==Motion::backFlipOut)&&!t.usesEdgeTargets(r.motion),
        "S plus Space retains the independent outward release rather than a lower-edge catch");

    t=attach(w);Input down{0,-1};down.run=true;const auto descending=tick(w,t,down,1.f/60);
    require(!t.wallRunning()&&!runMotion(descending.motion),"Shift+S never restores downward wall running");
}
static ContextWorld separatedHopShelves() {
    auto w=shelves();w.boxes[2].low.x=-26;w.boxes[2].high.x=26;
    w.boxes.push_back({{65,0,sourceLip-8},{122,8,sourceLip}});return w;
}
static void changedGeometryAndCaptures() {
    Input hop{1,0};hop.hop=true;
    for(bool afterRelease:{false,true}) {
        auto w=separatedHopShelves();auto t=attach(w);recordedTick(w,t,hop,1.f/60);
        require(t.state==State::action&&t.usesEdgeTargets(Motion::hopRight),"dynamic target test retains a real planned lateral hop");
        if(afterRelease)while(t.actionProgress()<.4f)recordedTick(w,t,{},1.f/60);
        w.boxes.pop_back();const auto r=tick(w,t,{},1.f/60);
        if(afterRelease)require(r.released&&!r.completed&&!t.active(),"lost target after release produces physical fall");
        else require(!r.released&&t.state==State::wall,"lost target before release preserves the existing source hang");
    }
    auto w=separatedHopShelves();auto t=attach(w);recordedTick(w,t,hop,1.f/60);unsigned frames=0;
    while(t.state==State::action&&frames<120){recordedTick(w,t,{},1.f/60);++frames;}
    require(frames>10&&t.state==State::wall,"same-version captures retain complete supported lateral flight and catch");
    w=separatedHopShelves();t=attach(w);recordedTick(w,t,hop,1.f/60);
    w.boxes.erase(w.boxes.begin()+2);const auto lostSource=tick(w,t,{},1.f/60);
    require(lostSource.released&&!lostSource.completed&&!t.active(),"vanished loaded source releases instead of grasping air");
    w=separatedHopShelves();t=attach(w);recordedTick(w,t,hop,1.f/60);
    while(t.actionProgress()<.5f)recordedTick(w,t,{},1.f/60);
    w.boxes.erase(w.boxes.begin()+2);
    while(t.state==State::action)require(!tick(w,t,{},1.f/60).released,"unloaded former source does not cancel a valid lateral target");
    require(t.state==State::wall,"lateral catch completes after the irrelevant source disappears");
    w=shelves();t=attach(w);t.cfg.automaticClimbActions=t.cfg.legacyAutomaticHops=t.cfg.surfaceActionVariants=true;
    for(int frame=0;frame<90;++frame)recordedTick(w,t,{0,-1},1.f/60);
    require(t.state==State::wall&&!t.preparingEdge()&&w.local(t.position).z<-62,
        "same-version replay records ordinary descent with no deleted lower-catch state");
}

static Vec posePoint(Vec point,const Traversal& t) {
    return t.position+Vec{-t.normal.y,t.normal.x,0}*point.x-t.normal*point.y+Vec{0,0,point.z};
}
static void actualPoseTransitions(const Library& lib,int fps,bool distant,float angle) {
    auto w=shelves();w.yaw=angle;if(distant)w.origin={131316.86f,38643.36f,-11331.41f};
    auto t=attach(w);t.cfg.threepeatAnimations=true;lib.configureThreepeat(t.cfg);SurfacePose animator;Pose previous;std::array<Vec,4> oldEndpoints{};
    const float dt=1.f/fps;float worstAngle=0,worstStep=0,sourceContact=0,targetContact=0,heldContact=0,sourcePhase=0;
    int sourceHand=0;
    Motion last=Motion::none;bool sawEnter=false,sawExit=false;int plantedSamples=0;
    auto frame=[&](Input input) {
        const auto r=tick(w,t,input,dt);const auto pose=animator.update(lib,w,t,r.motion,dt,1);
        require(pose.size()==99,"real runtime animator outputs the complete Skyrim skeleton");
        for(const auto& bone:pose)
            require(bone.t.finite()&&std::isfinite(bone.q.dot(bone.q))&&std::abs(bone.q.dot(bone.q)-1)<.002f,
                "context playback transforms remain finite and normalized");
        for(int hand=0;hand<2;++hand)require(lib.armBendValid(pose,hand),
            "real context playback and return transitions never reverse the calibrated elbow bend");
        const auto body=lib.world(pose);
        const std::array<Vec,4> endpoints{posePoint(lib.palm(body,0),t),posePoint(lib.palm(body,1),t),
            posePoint(body[8].t,t),posePoint(body[11].t,t)};
        if(!previous.empty()) {
            for(std::size_t bone=0;bone<pose.size();++bone)
                worstAngle=std::max(worstAngle,angleBetween(previous[bone].q,pose[bone].q));
            for(int index=0;index<4;++index)worstStep=std::max(worstStep,(endpoints[index]-oldEndpoints[index]).length());
            sawEnter|=!threepeatHop(last)&&threepeatHop(r.motion);
            sawExit|=threepeatHop(last)&&(r.motion==Motion::contextHang||r.motion==Motion::hang);
        }
        if(t.usesEdgeTargets(r.motion))for(int hand=0;hand<2;++hand) {
            const float phase=t.actionProgress();
            const float source=t.holdsPreparedEdge(r.motion)?t.preparedEdgeWeight(r.motion):
                threepeatHop(r.motion)?threepeatSourceWeight(r.motion==Motion::contextHopLeft,hand,phase):0.f;
            const float destination=t.holdsDestinationEdge(r.motion)?1.f:
                threepeatHop(r.motion)?threepeatTargetWeight(r.motion==Motion::contextHopLeft,hand,phase):0.f;
            if(source>.95f) {
                const float error=(endpoints[hand]-(t.edgeHand(hand,false)+Vec{0,0,.8f})).length();
                if(error>sourceContact){sourceContact=error;sourcePhase=phase;sourceHand=hand;}++plantedSamples;
            }
            if(destination>.95f) {
                targetContact=std::max(targetContact,(endpoints[hand]-(t.edgeHand(hand,true)+Vec{0,0,.8f})).length());++plantedSamples;
                if(t.holdsDestinationEdge(r.motion))heldContact=std::max(heldContact,(endpoints[hand]-(t.edgeHand(hand,true)+Vec{0,0,.8f})).length());
            }
        }
        const auto duplicate=animator.update(lib,w,t,r.motion,0,1);
        for(std::size_t bone=0;bone<pose.size();++bone)
            require(angleBetween(duplicate[bone].q,pose[bone].q)<.00001f&&(duplicate[bone].t-pose[bone].t).length()==0,
                "zero-time output callbacks preserve the exact displayed context pose");
        previous=pose;oldEndpoints=endpoints;last=r.motion;return r;
    };
    for(int i=0;i<fps;++i)frame({});
    Input hop{1,0};hop.hop=true;frame(hop);
    require(t.preparingEdge()||t.state==State::action,"runtime pose integration prepares a measured retained lateral hop");
    for(int i=0;(t.preparingEdge()||t.state==State::action)&&i<fps*3;++i)frame({});
    require(t.state==State::wall&&!t.preparingEdge(),"runtime lateral catch returns to movable wall control");
    for(int i=0;i<fps/2;++i)frame({});
    for(int i=0;i<fps/2;++i)frame({1,0});
    std::cout<<"context pose fps="<<fps<<" far="<<distant<<" yaw="<<angle<<" angle="<<worstAngle
        <<" step="<<worstStep<<" sourcePalm="<<sourceContact<<" sourcePhase="<<sourcePhase<<" sourceHand="<<sourceHand
        <<" targetPalm="<<targetContact<<" heldPalm="<<heldContact<<'\n';
    require(sawEnter&&sawExit&&plantedSamples>fps/8,"runtime trace covers retained lateral entry, contact windows, catch and hang return");
    require(worstAngle<=12.566371f*dt+.015f,"all 99 bones share a continuous angular budget during the new action and recovery");
    require(worstStep<=600*dt+1.f,"hands and feet do not teleport at context action boundaries");
    require(sourceContact<5.f&&targetContact<5.f&&heldContact<5.f,
        "fully weighted hands and stopped hang remain within five units of actual measured ledges");
}

int main(int argc,char** argv) {
    try {
        for(int fps:{30,60,120})for(bool distant:{false,true})for(float angle:{0.f,.633f,1.5707963f})
            descentWithoutRegrab(fps,distant,angle);
        contextualHops();negativeAndLegacy();inputPreservation();changedGeometryAndCaptures();
        if(argc>1) {
            Library lib;require(lib.load(argv[1]),"context tests require the actual runtime motion library");
            unsigned failed=0;
            for(int fps:{30,60,120})for(bool distant:{false,true})for(float angle:{0.f,.633f}) {
                try {actualPoseTransitions(lib,fps,distant,angle);}
                catch(const std::exception& error) {++failed;std::cerr<<"context pose failure: "<<error.what()<<'\n';}
            }
            require(!failed,"all runtime context pose frame-rate and world-transform cases must pass");
        }
        std::cout<<"context action geometry, controls, runtime poses and captures passed\n";return 0;
    } catch(const std::exception& error) {std::cerr<<"context action failure: "<<error.what()<<'\n';return 1;}
}
