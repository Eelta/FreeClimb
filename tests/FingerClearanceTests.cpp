#define main fingerClearanceExistingThreepeatMain
#include "ThreepeatMotionTests.cpp"
#undef main

struct FingerClearanceMetrics {
    struct Witness {float depth{},phase{},load{};Vec point{},palm{},anchor{};int bone=-1,samples{};bool tip{};};
    std::array<Witness,43> byMotion{};
    Pose previous;
    float depth{},maxPalm{},maxAngle{},phase{};int bone=-1,motion{},samples{};
    float maxPalmPhase{};int maxPalmMotion{},maxPalmHand=-1;bool maxPalmDestination{};
    Vec point{};bool tip{},hopSeen{},topSeen{},unloadedHop{},unloadedTop{};
    float minimumHop=1,maximumHop{},minimumTop=1,maximumTop{};
};
static float fingerFiniteDepth(const ThreepeatWorld&world,Vec p){
    p=world.rotate(p-world.origin,-world.yaw);float deepest=0;
    for(const auto&box:world.boxes){

        const float inside=std::min({p.x-box.low.x,box.high.x-p.x,p.y-box.low.y,
            box.high.y-p.y,p.z-box.low.z,box.high.z-p.z});
        deepest=std::max(deepest,inside);
    }
    return deepest;
}
static bool fingerClearanceCase(const Library&lib,int fps,int side,float scale,bool far,bool lip){
    Settings calibration;check(lib.configureThreepeat(calibration),"finger clearance uses actual licensed calibration");
    ThreepeatWorld world;world.boxes={{{-10000,0,-5000},{10000,1000,
        lip?calibration.threepeatHangHeight*scale:20000.f}}};
    if(far){world.origin={131146.67f,38997.66f,-11793.61f};world.yaw=.633f;}
    Traversal t;t.cfg.gap=37;t.cfg.radius=31;t.cfg.height=138;t.cfg.approachSeconds=.01f;
    check(lib.configureThreepeat(t.cfg),"finger case library calibration");
    t.cfg.threepeatAnimations=true;t.cfg.contextScale=scale;
    t.cfg.surfaceActionVariants=!lip;t.cfg.automaticClimbActions=!lip;
    t.cfg.autoActionMinSeconds=.8f;t.cfg.autoActionMaxSeconds=1.25f;
    check(t.attach(world,world.point({0,-45,0}),world.vector({0,1,0}),1000,60),"finite fixture attaches through unchanged physical checks");
    t.update(world,{},.05f,1000);check(t.state==State::wall,"actual entry completes before finger test");
    SurfacePose poses;FingerClearanceMetrics metric;const float dt=1.f/fps;
    auto frame=[&](Input input){
        const auto result=t.update(world,input,dt,1000);
        const auto p=poses.update(lib,world,t,result.motion,dt,scale),body=lib.world(p);
        auto point=[&](Vec v){return t.position+(Vec{-t.normal.y,t.normal.x,0}*v.x-t.normal*v.y+Vec{0,0,v.z})*scale;};
        for(int hand=0;hand<2;++hand)check(lib.armBendValid(p,hand),"finger clearance never reverses an elbow");
        if(!metric.previous.empty())for(std::size_t b=0;b<p.size();++b){
            const float angle=angleBetween(metric.previous[b].q,p[b].q);metric.maxAngle=std::max(metric.maxAngle,angle);
            check(angle<=12.566371f*dt+.015f,"finger clearance retains original complete-skeleton angular budget");
        }
        if(threepeatMotion(result.motion))for(int hand=0;hand<2;++hand){
            const int elbow=hand?32:29,wrist=hand?39:38,middle=hand?88:73;
            check(std::acos(std::clamp((body[wrist].t-body[elbow].t).unit().dot((body[middle].t-body[wrist].t).unit()),-1.f,1.f))<=1.658064f,
                "finger clearance retains95-degree wrist limit");
        }
        const bool hopping=threepeatHop(result.motion),top=result.motion==Motion::contextMantle;
        if(hopping||top){
            const float phase=hopping?t.actionProgress():t.progress();
            if(hopping){
                check(result.motion==(side<0?Motion::contextHopLeft:Motion::contextHopRight),"actual directional input selects its captured side");
                metric.hopSeen=true;metric.minimumHop=std::min(metric.minimumHop,phase);metric.maximumHop=std::max(metric.maximumHop,phase);
            }else{metric.topSeen=true;metric.minimumTop=std::min(metric.minimumTop,phase);metric.maximumTop=std::max(metric.maximumTop,phase);}
            for(int hand=0;hand<2;++hand){
                const float a=hopping?threepeatSourceWeight(side<0,hand,phase):threepeatMantleWeight(hand,phase);
                const float b=hopping?threepeatTargetWeight(side<0,hand,phase):0.f;
                if(hopping&&std::max(a,b)<.1f&&phase>.3f&&phase<.65f)metric.unloadedHop=true;
                if(top&&a<.1f&&phase>.45f&&phase<.7f)metric.unloadedTop=true;
                if(hopping&&std::max(a,b)>.95f){
                    const float offset=t.usesWallTargets(result.motion)?capturedWallPalmOffset:.8f;
                    const Vec target=t.edgeHand(hand,b>a)+t.edgeContactNormal(hand,b>a)*(offset*scale);
                    const float error=(point(lib.palm(body,hand))-target).length();
                    if(error>metric.maxPalm){metric.maxPalm=error;metric.maxPalmMotion=int(result.motion);metric.maxPalmPhase=phase;metric.maxPalmHand=hand;metric.maxPalmDestination=b>a;}
                }
            }
            if(top&&poses.topPalmError>metric.maxPalm){metric.maxPalm=poses.topPalmError;metric.maxPalmMotion=int(result.motion);metric.maxPalmPhase=phase;metric.maxPalmHand=-1;metric.maxPalmDestination=false;}
            auto measure=[&](Vec local,int bone,bool tip){
                ++metric.samples;const Vec shown=point(local);const float depth=fingerFiniteDepth(world,shown);
                auto& witness=metric.byMotion[int(result.motion)];++witness.samples;
                if(depth>witness.depth){
                    const int hand=bone>=82?1:0;
                    witness.depth=depth;witness.phase=phase;witness.bone=bone;witness.tip=tip;witness.point=shown;
                    witness.load=hopping?std::max(threepeatSourceWeight(side<0,hand,phase),threepeatTargetWeight(side<0,hand,phase)):threepeatMantleWeight(hand,phase);
                    witness.palm=point(lib.palm(body,hand));
                    if(hopping){
                        const bool destination=threepeatTargetWeight(side<0,hand,phase)>threepeatSourceWeight(side<0,hand,phase);
                        const float offset=t.usesWallTargets(result.motion)?capturedWallPalmOffset:.8f;
                        witness.anchor=t.edgeHand(hand,destination)+t.edgeContactNormal(hand,destination)*(offset*scale);
                    }
                }
                if(depth>metric.depth){metric.depth=depth;metric.bone=bone;metric.motion=int(result.motion);metric.phase=phase;metric.tip=tip;metric.point=shown;}
            };
            for(int bone=67;bone<97;++bone)measure(body[bone].t,bone,false);
            for(int base:{67,82})for(int digit=0;digit<5;++digit){
                const int end=base+digit*3+2;

                measure(body[end].t+body[end].q.rotate({0,0,lib.rest[end].t.length()*.75f}),end,true);
            }
        }
        metric.previous=p;return result;
    };
    for(int i=0;i<fps;++i)check(frame({}).motion==Motion::hang,"ordinary neutral never discovers an independent39 idle");
    if(lip){Input hop{float(side),0};hop.hop=true;frame(hop);}
    else for(int i=0;i<fps*6&&!metric.hopSeen;++i)frame({float(side),0});
    for(int i=0;i<fps*4&&(t.preparingEdge()||t.state==State::action);++i)frame({});
    check(metric.hopSeen&&metric.unloadedHop&&metric.minimumHop<.06f&&metric.maximumHop>.94f,
        "real captured hop including its free-hand middle and late catch must run, not select a fallback");
    for(int i=0;i<fps;++i)frame({});
    if(lip){
        Input mantle{0,1};mantle.mantle=true;frame(mantle);
        for(int i=0;i<fps*4&&t.active();++i)frame({});
        check(metric.topSeen&&metric.unloadedTop&&metric.minimumTop<.05f&&metric.maximumTop>.97f&&!t.active(),
            "real42 top-out must traverse its unloading tail and complete, not bypass the diagnosed phase");
    }
    std::cout<<"finger finite fps="<<fps<<" side="<<side<<" scale="<<scale<<" far="<<far<<" lip="<<lip
        <<" samples="<<metric.samples<<" depth="<<metric.depth<<" bone="<<metric.bone<<" tip="<<metric.tip
        <<" motion="<<metric.motion<<" phase="<<metric.phase<<" point="<<metric.point.x<<','<<metric.point.y<<','<<metric.point.z
        <<" palm="<<metric.maxPalm<<" palmMotion="<<metric.maxPalmMotion<<" palmPhase="<<metric.maxPalmPhase
        <<" palmHand="<<metric.maxPalmHand<<" palmDestination="<<metric.maxPalmDestination<<" angle="<<metric.maxAngle<<'\n';
    for(int motion:{40,41,42})if(const auto&w=metric.byMotion[motion];w.samples){
        const Vec local=world.rotate(w.point-world.origin,-world.yaw),palm=world.rotate(w.palm-world.origin,-world.yaw),anchor=world.rotate(w.anchor-world.origin,-world.yaw);
        std::cout<<"  motion="<<motion<<" samples="<<w.samples<<" depth="<<w.depth<<" bone="<<w.bone
            <<" tip="<<w.tip<<" phase="<<w.phase<<" load="<<w.load<<" point="<<w.point.x<<','<<w.point.y<<','<<w.point.z
            <<" local="<<local.x<<','<<local.y<<','<<local.z<<" localPalm="<<palm.x<<','<<palm.y<<','<<palm.z
            <<" boxTop="<<world.boxes.front().high.z;
        if(motion!=42)std::cout<<" localAnchor="<<anchor.x<<','<<anchor.y<<','<<anchor.z;
        std::cout<<'\n';
    }
    check(metric.samples>fps*40&&metric.maxPalm<5.001f,"finite clearance preserves genuine loaded contacts inside original5-unit envelope");

    return metric.depth<=.15001f;
}
int main(int argc,char**argv){try{
    check(argc==2,"supply runtime motion library");Library lib;check(lib.load(argv[1]),"load complete authorized library");
    unsigned failed=0,total=0;
    for(int fps:{30,60,120})for(int side:{-1,1})for(float scale:{1.f,1.03f})for(bool far:{false,true})for(bool lip:{false,true}){
        failed+=!fingerClearanceCase(lib,fps,side,scale,far,lip);++total;
    }
    std::cout<<"finite finger cases="<<total<<" failed="<<failed<<'\n';
    check(failed==0,"new captured hands cannot enter finite solids during loaded OR released motion phases");
    std::cout<<"PASS complete40/41/42 finite-body finger clearance with existing support/anatomy/rate constraints\n";return 0;
}catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
