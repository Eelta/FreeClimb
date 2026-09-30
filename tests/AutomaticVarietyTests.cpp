#include "Pose.h"
#include "Controls.h"
#include "TraversalCapture.h"
#include "CornerTestWorld.h"
#include <array>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>
using namespace fc;
static void require(bool value,const char* reason){if(!value)throw std::runtime_error(reason);}

struct VarietyWorld final:World {
    fc_test::CornerWorld geometry;
    std::optional<Hit> ray(Vec from,Vec to)override{return geometry.ray(from,to);}
    Vec point(Vec p)const{return geometry.global(p);}
    Vec direction(Vec v)const{return geometry.direction(v);}
    Vec local(Vec p)const{return geometry.local(p);}
    void transform(bool far){if(far){geometry.origin={131146.67f,38997.66f,-11793.61f};geometry.yaw=.633f;}}
};
static VarietyWorld flat(bool far=false) {
    VarietyWorld w;w.geometry.boxes={{{-20000,0,-10000},{20000,1000,20000}}};w.transform(far);return w;
}
static Traversal attached(VarietyWorld& w,const Library& lib,bool variants=true,Vec feet={0,-45,300}) {
    Traversal t;t.cfg=fc_test::settings();t.cfg.approachSeconds=.01f;
    require(lib.configureThreepeat(t.cfg),"production 42-clip calibration is required");
    t.cfg.threepeatAnimations=true;t.cfg.surfaceActionVariants=variants;
    t.cfg.automaticClimbActions=true;t.cfg.autoActionMinSeconds=1;t.cfg.autoActionMaxSeconds=1.6f;
    require(t.attach(w,w.point(feet),w.direction({0,1,0}),1000,60),"fixture acquires an actual checked wall");
    t.update(w,{},.05f,1000);require(t.state==State::wall,"checked entry completes before scheduling tests");return t;
}
static void actualWallAnchors(const VarietyWorld& w,const Traversal& t,Motion m) {
    require(t.usesWallTargets(m),"wall variant explicitly identifies measured wall contacts");
    for(int hand=0;hand<2;++hand)for(bool destination:{false,true}) {
        const Vec p=w.local(t.edgeHand(hand,destination));
        const auto n=t.edgeContactNormal(hand,destination);
        require(std::abs(p.y)<.06f,"wall contact must lie on the actual solid front, not an invented top inset");
        require(n.dot(w.direction({0,-1,0}))>.99f&&std::abs(n.z)<.01f,
            "front-face contact stores its real horizontal normal rather than a ledge normal");
    }
}
struct RunStats {unsigned commits{},newFrames{},legacyFrames{},peakRays{};float first=-1,lastStart=-1;std::vector<float> starts;};
static RunStats moving(const Library& lib,int fps,int direction,bool far,bool variants=true) {
    auto w=flat(far);auto t=attached(w,lib,variants);const float dt=1.f/fps;

    t.cfg.legacyAutomaticHops=direction==0;
    const Input input=direction<0?Input{-1,0}:direction==1?Input{1,0}:Input{0,1};
    const Motion expected=direction<0?Motion::contextHopLeft:direction==1?Motion::contextHopRight:
        Motion::hopUp;
    RunStats stats;unsigned counted=0;
    for(int frame=0;frame<fps*9;++frame) {
        const Vec before=t.position;const auto casts=w.geometry.rays;
        const auto result=t.update(w,input,dt,1000);const float now=float(frame+1)/fps;
        stats.peakRays=std::max(stats.peakRays,w.geometry.rays-casts);
        require(!result.released&&t.active(),"valid continuous wall retains ordinary traversal and checked variants");
        require(w.geometry.clearance(t.position,t.cfg)>=t.cfg.radius-.06f,"all variant motion retains full body clearance");
        if(direction==0)require(!threepeatHop(result.motion)&&(result.motion==Motion::none||isActiveMotion(result.motion)),
            "forward input never chooses an unsolicited lateral leap or downward drop");
        require((result.motion==Motion::none||isActiveMotion(result.motion)),"retired descending regrab never appears in runtime traversal");
        if(variants&&direction!=0&&threepeatHop(result.motion)) {
            require(result.motion==expected,"variation preserves the user's exact requested travel direction");
            actualWallAnchors(w,t,result.motion);++stats.newFrames;
        }
        if(result.motion==Motion::hopLeft||result.motion==Motion::hopRight)++stats.legacyFrames;
        if(t.automaticActionCount()!=counted) {
            require(t.automaticActionCount()==counted+1,"one committed automatic action counts exactly once");
            counted=t.automaticActionCount();++stats.commits;
            require(t.surfaceActionCount()==(direction==0?0u:counted),
                "measured wall-action counter increments once per wall variant and excludes ordinary upward hops");
            require(t.state==State::action&&result.motion==expected,
                "successful flat-wall variants win over old hop fallbacks at the same deadline");
            require(result.staminaCost>=15&&result.staminaCost<16,"automatic commit charges one existing action cost");
            if(stats.first<0)stats.first=now;
            if(stats.lastStart>=0)require(now-stats.lastStart>1.f,"automatic jumps cannot restart every frame or chain without a travel interval");
            stats.lastStart=now;stats.starts.push_back(now);
        }
        if(result.motion==Motion::contextHang)require(t.automaticActionCount()==counted,"idle output is not a new automatic jump");
        require((t.position-before).length()<900*dt+1.f,"action travel remains continuous without a positional snap");
    }
    require(stats.commits>=2,"continuous compatible input must visibly commit multiple complete variations");
    require(stats.first>=.99f&&stats.first<2.5f,"plain-wall fallback observes its randomized delay and bounded preparation");
    if(direction!=0)require(stats.newFrames>unsigned(fps)&&stats.legacyFrames==0,
        "new compatible clips receive substantial playback instead of one-frame labels or old-hop starvation");
    if(direction==-1||direction==1)require(t.automaticOpportunityCount()>0,
        "new-edge opportunities are probed separately from random fallback attempts on a long wall");
    std::cout<<"variety fps="<<fps<<" direction="<<direction<<" far="<<far<<" commits="<<stats.commits
        <<" newFrames="<<stats.newFrames<<" first="<<stats.first<<" peakRays="<<stats.peakRays<<" starts=";
    for(const float at:stats.starts)std::cout<<at<<',';std::cout<<'\n';
    return stats;
}
static void idleAndExclusions(const Library& lib,int fps) {
    const float dt=1.f/fps;auto w=flat();auto t=attached(w,lib);
    for(int frame=0;frame<fps;++frame) {
        const auto result=t.update(w,{},dt,1000);
        require(!result.released&&t.automaticActionCount()==0,"standing hang never counts or pays for a random jump");
        require(result.motion==Motion::hang&&!t.preparingEdge()&&!t.usesEdgeTargets(result.motion),
            "ordinary neutral input stays on hang1 without independently acquiring captured39");
    }
    require(t.contextIdleCount()==0&&t.surfaceActionCount()==0,
        "ordinary hanging cannot increment a removed independent39 acquisition counter");
    for(int kind=0;kind<5;++kind) {
        t=attached(w,lib);float stamina=1000;Input input{1,0};
        if(kind==0)t.cfg.surfaceActionVariants=false;
        if(kind==1)t.cfg.threepeatAnimations=false;
        if(kind==2)t.cfg.automaticClimbActions=false;
        if(kind==3)stamina=14;
        if(kind==4)input.run=true;
        for(int frame=0;frame<fps*5;++frame) {
            const auto result=t.update(w,input,dt,stamina);
            require(!result.released,"disabled/ineligible variation retains ordinary supported motion");
            require(!threepeatHop(result.motion)&&(result.motion==Motion::none||isActiveMotion(result.motion)),"disabled, uncalibrated, low-stamina and running states cannot play surface leap variants");
            if(kind>=2)require(t.automaticActionCount()==0,"disabled scheduling/low stamina/wall running cannot commit ordinary automatic jumps");
        }
    }
    t=attached(w,lib);
    for(int frame=0;frame<fps*2;++frame) {
        Keys keys;keys.w=keys.shift=true;keys.space=frame==fps;
        const auto result=t.update(w,wallInput(keys,keys.space,false,false,t.wallRunning()),dt,1000);
        require(!hopMotion(result.motion)&&(result.motion==Motion::none||isActiveMotion(result.motion))&&t.automaticActionCount()==0,
            "manual wall-run Space never becomes a surface leap or random climbing action");
    }
    t=attached(w,lib);const auto dead=t.update(w,{1,0},dt,0);
    require(dead.released&&!t.active()&&t.automaticActionCount()==0,"zero stamina cannot start or retain an automatic action");
}
static void finiteEdgeOpportunity(const Library& lib,int fps,int side,bool far) {
    Settings calibration;require(lib.configureThreepeat(calibration),"opportunity geometry uses captured hand dimensions");
    const float h=300+calibration.threepeatHangHeight;
    const float span=calibration.threepeatHopDistance[side<0?0:1];
    VarietyWorld w;w.transform(far);

    auto append=[&](float a,float b){if(side<0){const float tmp=a;a=-b;b=-tmp;}w.geometry.boxes.push_back({{a,0,-10000},{b,8,h}});};
    w.geometry.boxes.push_back({{-20000,8,-10000},{20000,1000,20000}});
    append(20,100);append(20+span,100+span);
    auto t=attached(w,lib);bool prepared=false,newHop=false;float prepareTime=-1;unsigned count=0;
    for(int frame=0;frame<fps*4;++frame) {
        const auto result=t.update(w,{float(side),0},1.f/fps,1000);const float now=float(frame+1)/fps;
        require(!result.released,"finite real edges preserve supported traversal");
        if(t.preparingEdge()&&!prepared){prepared=true;prepareTime=now;}
        require(result.motion!=Motion::hopLeft&&result.motion!=Motion::hopRight,"old hop must not consume a valid new-edge opportunity");
        if(threepeatHop(result.motion)) {
            require(result.motion==(side<0?Motion::contextHopLeft:Motion::contextHopRight),"real-edge opportunity keeps side direction");
            require(!t.usesWallTargets(result.motion),"real ledges take priority over the generic measured wall fallback");
            require(t.surfaceActionCount()==0,"real ledge captures are not mislabeled as measured wall-patch actions");
            require(t.edgeContactNormal(0,false).z>.95f&&t.edgeContactNormal(0,true).z>.95f,
                "real edge mode retains actual top contact normals");
            newHop=true;
        }
        if(t.automaticActionCount()) {require(t.automaticActionCount()==1,"one source opportunity commits once");count=1;}
        if(newHop&&t.state==State::wall)break;
    }
    require(prepared&&prepareTime>=.35f-.04f&&prepareTime<1.f&&newHop&&count==1,
        "bounded opportunity scan catches a finite real ledge before the legacy random deadline");
}
static void obstructedVariantRoute(const Library& lib,int fps) {
    for(int side:{-1,1}) {
        auto w=flat();auto t=attached(w,lib);const float dt=1.f/fps;

        w.geometry.boxes.push_back({{-20000,-200,t.position.z+t.cfg.height+12},
            {20000,30,t.position.z+t.cfg.height+22}});
        for(int frame=0;frame<fps*5;++frame) {
            const auto result=t.update(w,{float(side),0},dt,1000);
            require(!result.released&&w.geometry.clearance(t.position,t.cfg)>=t.cfg.radius-.06f,
                "blocked leap leaves ordinary same-direction wall movement available");
            require(!threepeatHop(result.motion)&&t.surfaceActionCount()==0,
                "actual intermediate-body obstruction prevents a captured leap despite valid contact endpoints");
        }
    }
}
static void pendingCancellationAndReplay(const Library& lib,int fps) {
    auto w=flat();auto t=attached(w,lib);const float dt=1.f/fps;
    for(int frame=0;frame<fps*4&&!t.preparingEdge();++frame)t.update(w,{1,0},dt,1000);
    require(t.preparingEdge()&&t.automaticActionCount()==0,"automatic surface leap exposes cancellable source preparation");
    const auto prepared=t;
    for(Input cancel:std::array<Input,5>{Input{},Input{-1,0},Input{0,-1},Input{1,0,false,false,false,false,true},Input{0,-1,true,false,false,true}}) {
        t=prepared;const auto result=t.update(w,cancel,dt,1000);
        require(!t.preparingEdge()&&t.automaticActionCount()==0&&!threepeatHop(result.motion),
            "stop/reverse/down/Shift/departure cancels an uncommitted automatic side leap");
    }
    t=prepared;w.geometry.boxes.clear();const auto removed=t.update(w,{1,0},dt,1000);
    require(!t.preparingEdge()&&t.automaticActionCount()==0&&!threepeatHop(removed.motion),
        "removed source wall cannot commit a prepared action into air");
    w=flat();t=attached(w,lib);bool capturedPending=false,capturedCommit=false,capturedFlight=false;
    for(int frame=0;frame<fps*5&&!capturedFlight;++frame) {
        auto tape=std::make_unique<TraversalCapture>();tape->begin(t,{1,0},dt,1000);
        TraversalCapture::RecordingWorld recording(w,*tape);
        const auto before=t.automaticActionCount();const auto result=t.update(recording,{1,0},dt,1000);tape->finish(t,result);
        const bool pending=t.preparingEdge(),commit=t.automaticActionCount()!=before;
        const bool flight=threepeatHop(result.motion)&&t.actionProgress()>.4f;
        if((pending&&!capturedPending)||commit||flight) {
            require(tape->complete(),"opportunity/complete route and playback fit the existing capture ray budget");
            auto decoded=std::make_unique<TraversalCapture>();std::string error;
            require(decoded->deserialize(tape->serialize(),error)&&decoded->replay().matched,
                "automatic deadlines, opportunity state, wall descriptors and commits replay exactly");
            capturedPending|=pending;capturedCommit|=commit;capturedFlight|=flight;
        }
    }
    require(capturedPending&&capturedCommit&&capturedFlight,"replay covers planning, the unique commit, and live wall contact playback");
    w.geometry.boxes.clear();const auto lost=t.update(w,{1,0},dt,1000);
    require(!lost.completed&&!t.active(),"losing a committed target releases instead of declaring an airborne catch");
}
int main(int argc,char** argv){try {
    require(argc==2,"supply production 42-clip motion library");Library lib;require(lib.load(argv[1])&&lib.hasThreepeat(),"actual appended clips load");
    std::array<std::vector<float>,3> starts;
    for(int fps:{30,60,120}) {
        for(int direction:{-1,1,0}) {
            const auto result=moving(lib,fps,direction,false);const unsigned index=direction<0?0:direction==1?1:2;
            if(fps==30)starts[index]=result.starts;
            else {
                if(direction!=0)require(result.starts.size()==starts[index].size(),"new-family committed distribution is stable across frame rates");
                else if(result.starts.size()!=starts[index].size()) {
                    const auto& longer=result.starts.size()>starts[index].size()?result.starts:starts[index];
                    const auto shorter=std::min(result.starts.size(),starts[index].size());
                    require(longer.size()==shorter+1&&longer.back()>8.7f,
                        "legacy upward cadence may differ only by one action at the fixed observation window boundary");
                }
                for(unsigned n=0;n<std::min(result.starts.size(),starts[index].size());++n)
                    require(std::abs(result.starts[n]-starts[index][n])<.3f,"scheduler phase and action preparation stay frame-rate stable");
            }
        }
        idleAndExclusions(lib,fps);pendingCancellationAndReplay(lib,fps);obstructedVariantRoute(lib,fps);
        for(int side:{-1,1})for(bool far:{false,true})finiteEdgeOpportunity(lib,fps,side,far);
    }
    for(int direction:{-1,1,0})moving(lib,60,direction,true);
    std::cout<<"PASS: actual automatic new-motion selection, finite edge opportunities, intent, physics, counters and replay\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL automatic variety: "<<e.what()<<'\n';return 1;}}
