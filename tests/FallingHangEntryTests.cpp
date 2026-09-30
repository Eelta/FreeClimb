#define main fallingThreepeatFixtureMain
#include "ThreepeatMotionTests.cpp"
#undef main
#include "Controls.h"

static Traversal fallingEntry(ThreepeatWorld& world,const Library& lib,float scale,bool lip,Motion motion=Motion::ledgeCatch) {
    world.boxes={{{-10000,0,-5000},{10000,1000,20000}}};
    Traversal t;t.cfg.gap=37*scale;t.cfg.radius=31*scale;t.cfg.height=138*scale;
    t.cfg.chest=70*scale;t.cfg.grip=112*scale;t.cfg.contextScale=scale;
    t.cfg.approachSeconds=.32f;t.cfg.surfaceActionVariants=true;t.cfg.automaticClimbActions=false;
    check(lib.configureThreepeat(t.cfg),"retained captured side/top library calibration");t.cfg.threepeatAnimations=true;
    if(lip)world.boxes[0].high.z=200*scale+t.cfg.threepeatHangHeight*scale;
    check(t.attach(world,world.point({0,-80*scale,200*scale}),world.vector({0,1,0}),1000,60*scale,true,false),
        "airborne catch still requires the original real wall, reach and complete body path");
    t.entry(motion,motion==Motion::jumpCatch);return t;
}
struct CatchMetrics {float comSpeed{},angularRate{},wrist{};};
static void checkCatchPose(const Library& lib,const Traversal& t,const Pose& pose,const Pose& previous,
    Vec previousCOM,float dt,float scale,bool run,CatchMetrics& metrics) {
    check(pose.size()==99,"retained entry returns every target bone");
    const auto body=lib.world(pose);
    const Vec com=t.position+(Vec{-t.normal.y,t.normal.x,0}*body[4].t.x-t.normal*body[4].t.y+Vec{0,0,body[4].t.z})*scale;
    for(const auto& bone:pose)check(bone.t.finite()&&std::abs(bone.q.dot(bone.q)-1)<.002f,"retained entry output remains finite and normalized");
    for(int hand=0;hand<2;++hand) {
        check(lib.armBendValid(pose,hand),"retained entry and continuation never reverse an elbow");
        const int wrist=hand?39:38,elbow=hand?32:29,middle=hand?88:73;
        const float angle=std::acos(std::clamp((body[wrist].t-body[elbow].t).unit().dot((body[middle].t-body[wrist].t).unit()),-1.f,1.f));
        metrics.wrist=std::max(metrics.wrist,angle);
        check(angle<=1.658064f,"retained entry preserves the original95degree wrist limit");
    }
    if(previous.empty())return;
    const float speed=(com-previousCOM).length()/(scale*dt);metrics.comSpeed=std::max(metrics.comSpeed,speed);
    check(speed<600.f,"retained entry has no COM jump beyond the existing endpoint budget");
    for(std::size_t b=0;b<pose.size();++b) {
        const float rate=angleBetween(previous[b].q,pose[b].q)/dt;metrics.angularRate=std::max(metrics.angularRate,rate);
        check(rate<=(run?18.849556f:12.566371f)+.002f,"retained entry respects the original all-bone angular-rate limit");
    }
}
static void fallingHang(const Library& lib,int fps,float scale,bool lip) {
    ThreepeatWorld world;auto t=fallingEntry(world,lib,scale,lip);SurfacePose animator;Pose previous;
    Vec previousCOM{};CatchMetrics metrics;const float dt=1.f/fps;int lastEntry=-1,firstMove=-1;unsigned entryFrames=0,movedFrames=0;
    for(int frame=0;frame<fps;++frame) {
        const auto result=t.update(world,{lip?1.f:0.f,lip?0.f:1.f},frame?dt:0.f,1000);
        check(!result.released&&t.active(),"supported original airborne catch remains attached");
        check(result.motion!=Motion::contextHang&&(result.motion==Motion::none||isActiveMotion(result.motion))&&!t.preparingEdge(),
            "falling19 cannot start either retired descending adapter or a special39 settle");
        check(!t.usesEdgeTargets(result.motion),"original airborne19 never acquires the removed captured-hand descriptor");
        check(std::abs(t.entryDuration()-.32f)<.00001f,"removed catch adapter cannot extend the original entry duration");
        const auto pose=animator.update(lib,world,t,result.motion,frame?dt:0.f,scale);
        checkCatchPose(lib,t,pose,previous,previousCOM,dt,scale,false,metrics);
        const auto body=lib.world(pose);previousCOM=t.position+(Vec{-t.normal.y,t.normal.x,0}*body[4].t.x-t.normal*body[4].t.y+Vec{0,0,body[4].t.z})*scale;
        if(result.motion==Motion::ledgeCatch){lastEntry=frame;++entryFrames;}
        else if(result.motion==(lip?Motion::right:Motion::up)){if(firstMove<0)firstMove=frame;++movedFrames;}
        previous=pose;
    }
    check(entryFrames>unsigned(fps/4)&&movedFrames>unsigned(fps/2),"actual19 output and ordinary continuation both receive substantial playback");
    check(firstMove==lastEntry+1,"held input resumes on the first update after the original catch without an idle lock");
    std::cout<<"FALL19 fps="<<fps<<" scale="<<scale<<" lip="<<lip<<" entryFrames="<<entryFrames<<" moveFrames="<<movedFrames
        <<" COMspeed="<<metrics.comSpeed<<" rate="<<metrics.angularRate<<" wrist="<<metrics.wrist<<'\n';
}
static void fallingBranches(const Library& lib,int fps,bool lip,int mode) {
    ThreepeatWorld world;
    const Motion entry=mode==1?Motion::runLaunch:mode==2?Motion::jumpCatch:Motion::ledgeCatch;
    auto t=fallingEntry(world,lib,1,lip,entry);
    if(mode==3)t.cfg.contextActions=false;
    if(mode==4)t.cfg.threepeatAnimations=t.cfg.surfaceActionVariants=false;
    SurfacePose animator;Pose previous;Vec previousCOM{};CatchMetrics metrics;
    const float dt=1.f/fps;unsigned entryFrames=0,neutralFrames=0,captures=0;bool cancelled=false,sawRunning=false;
    auto capture=std::make_unique<TraversalCapture>(),decoded=std::make_unique<TraversalCapture>();
    for(int frame=0;frame<fps*2;++frame) {
        Input input;
        if(mode==1){input.run=true;if(lip)input.x=1;else input.y=1;}
        if(mode==5&&t.state==State::approach&&t.reachProgress()>.6f){input.release=input.backDrop=true;cancelled=true;}
        if(mode==6&&t.reachProgress()>.6f){input.run=true;input.x=1;}
        const bool wasApproaching=t.state==State::approach;const bool record=frame==1||frame==fps/4||frame==fps/2||input.release;
        Result result;
        if(record) {
            capture->begin(t,input,dt,1000);TraversalCapture::RecordingWorld recording(world,*capture);
            result=t.update(recording,input,dt,1000);capture->finish(t,result);std::string error;
            check(capture->complete()&&decoded->deserialize(capture->serialize(),error),"original catch snapshots serialize without removed adapter fields");
            const auto replay=decoded->replay();if(!replay.matched)std::cerr<<replay.error<<'\n';
            check(replay.matched,"original catch, input continuation and collision queries replay exactly");++captures;
        }else result=t.update(world,input,dt,1000);
        check(result.motion!=Motion::contextHang&&(result.motion==Motion::none||isActiveMotion(result.motion))&&!t.preparingEdge(),
            "no control or option branch restores19-to39 or38");
        if(input.release) {
            check(t.state==State::action||result.released,"S+Space still immediately interrupts an incomplete catch with the existing departure");
            check(result.motion==Motion::dropBack||result.motion==Motion::backFlipOut||result.released,
                "mid-entry departure uses its checked original fallback rather than a captured regrab");break;
        }
        check(!result.released,"supported entry and next input remain attached");
        if(wasApproaching){check(result.motion==entry,"airborne, grounded and run entries retain their chosen original clip");++entryFrames;}
        if(!wasApproaching&&(mode==0||mode==3||mode==4)) {
            ++neutralFrames;check(result.motion==Motion::hang&&!t.usesEdgeTargets(result.motion),
                "neutral after original19 immediately uses ordinaryhang1 without39 short settle");
        }
        sawRunning|=runMotion(result.motion);
        const auto pose=animator.update(lib,world,t,result.motion,dt,1);
        checkCatchPose(lib,t,pose,previous,previousCOM,dt,1,runMotion(result.motion)||mode==1,metrics);
        const auto body=lib.world(pose);previousCOM=posePoint(body[4].t,t);previous=pose;
    }
    check(entryFrames>0&&captures>0,"branch exercises actual entry and serialized update");
    if(mode==0||mode==3||mode==4)check(neutralFrames>unsigned(fps),"ordinary neutral hanging remains sustained without special capture state");
    if(mode==1||mode==6)check(sawRunning,"held or newly pressed Shift retains wall-running continuation");
    if(mode==5)check(cancelled,"departure negative reaches the interrupted catch phase");
    std::cout<<"BRANCH fps="<<fps<<" lip="<<lip<<" mode="<<mode<<" entry="<<int(entry)<<" neutral="<<neutralFrames
        <<" captures="<<captures<<" COMspeed="<<metrics.comSpeed<<" rate="<<metrics.angularRate<<'\n';
}
static void geometrySafety(const Library& lib) {
    struct Slope final:World {
        std::optional<Hit> ray(Vec a,Vec b)override {
            const Vec n{0,-.8f,.6f};const float da=a.dot(n),db=b.dot(n);
            if(da<=0||db>=0)return {};return Hit{a+(b-a)*(da/(da-db)),n,true};
        }
    } slope;
    Traversal t;t.cfg.gap=37;t.cfg.radius=31;t.cfg.height=138;t.cfg.contextScale=1;t.cfg.surfaceActionVariants=true;
    check(lib.configureThreepeat(t.cfg),"slope calibration");t.cfg.threepeatAnimations=true;
    check(t.attach(slope,{0,-30,0},{0,1,0},1000,1000,true,false),"real sloping plane remains eligible for original catch");t.entry(Motion::ledgeCatch,false);
    unsigned slopeFrames=0;
    for(int frame=0;frame<20&&t.state==State::approach;++frame) {
        const auto result=t.update(slope,{},1.f/60,1000);
        check(!result.released&&result.motion==Motion::ledgeCatch&&!t.usesEdgeTargets(result.motion),"sloped falling catch remains original19");++slopeFrames;
    }
    check(slopeFrames>0,"slope negative actually samples retained catch");
    ThreepeatWorld empty;Traversal absent;
    check(!absent.attach(empty,{0,-65,200},{0,1,0},1000,60,true,false),"removing special catch does not allow attaching to empty space");
    ThreepeatWorld far;auto unused=fallingEntry(far,lib,1,false);Traversal tooFar;tooFar.cfg=unused.cfg;
    check(!tooFar.attach(far,{0,-300,200},{0,1,0},1000,60,true,false),"air catch retains its bounded geometric reach");
    ThreepeatWorld world;t=fallingEntry(world,lib,1,false);
    for(int frame=0;t.reachProgress()<.5f&&frame<60;++frame)t.update(world,{},1.f/60,1000);
    check(t.state==State::approach,"dynamic body negative starts during the original entry");
    const Vec p=t.position;world.boxes.push_back({p+Vec{-35,-12,20},p+Vec{35,12,60}});
    const auto blocked=t.update(world,{},1.f/60,1000);
    check(blocked.released&&!t.active(),"a new solid on the original entry path still cancels before crossing it");
    std::cout<<"GEOMETRY real slope, empty world, bounded reach and dynamic body blocker PASS\n";
}
int main(int argc,char** argv) {
    try{check(argc==2,"motion path required");Library lib;check(lib.load(argv[1]),"unchanged42motion library");
        for(int fps:{30,60,120})for(float scale:{.75f,1.f,1.3f})for(bool lip:{false,true})fallingHang(lib,fps,scale,lip);
        for(int fps:{30,60,120})for(bool lip:{false,true})for(int mode=0;mode<7;++mode)fallingBranches(lib,fps,lip,mode);
        geometrySafety(lib);
        std::cout<<"PASS original airborne19, immediate normal hang/movement, no descending39/38 adapter, geometry and replay\n";return 0;
    }catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
