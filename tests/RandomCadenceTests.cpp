#define main threepeatCadenceFixtureMain
#include "ThreepeatMotionTests.cpp"
#undef main

static ThreepeatWorld longWall(const Library& lib,bool lip=false) {
    Settings c;check(lib.configureThreepeat(c),"calibrated family");
    ThreepeatWorld w;w.boxes={{{-20000,0,-5000},{20000,1000,lip?c.threepeatHangHeight:20000}}};return w;
}
static void interruptedTravel(const Library& lib,int fps,bool diagonal) {
    auto w=longWall(lib);auto t=attached(w,lib);t.cfg.automaticClimbActions=true;t.cfg.legacyAutomaticHops=true;
    const auto wall=w.boxes;const float dt=1.f/fps;
    unsigned blocked=0;SurfacePose pose;Pose previous;
    for(int i=0;i<fps*10;++i) {
        w.boxes=wall;
        const bool obstacle=!diagonal&&t.state==State::wall&&(i%(fps/2))<std::max(1,fps/15);
        if(obstacle)w.boxes.push_back({{-20000,-20000,t.position.z+t.cfg.height+.02f},{20000,-10,t.position.z+t.cfg.height+12}});
        Input input{diagonal&&(i/(fps/2))%2?1.f:0.f,1};
        const auto before=t.position;const auto out=t.update(w,input,dt,1000);
        check(!out.released,"brief blockers or gradual steering retain the real wall");
        if(obstacle&&(t.position-before).length()<.01f)++blocked;
        if(obstacle)check(!hopMotion(out.motion),"a blocked frame cannot itself commit random travel");
        const auto current=pose.update(lib,w,t,out.motion,dt,1);
        for(int hand=0;hand<2;++hand)check(lib.armBendValid(current,hand),"cadence changes do not reverse elbows");
        if(!previous.empty())for(std::size_t bone=0;bone<current.size();++bone)
            check(angleBetween(previous[bone].q,current[bone].q)<=12.566371f*dt+.015f,"random transitions retain angular limits");
        previous=current;
    }
    if(!diagonal)check(blocked>10,"fixture really interrupts movement repeatedly");
    check(t.automaticActionCount()>=2,diagonal?"45-degree steering must not continually redraw the wait":"single-frame body pauses must not starve random actions");
    std::cout<<"cadence fps="<<fps<<" diagonal="<<diagonal<<" pauses="<<blocked<<" actions="<<t.automaticActionCount()<<'\n';
}
static void rejectedRouteRetry(const Library& lib,int fps) {
    auto w=longWall(lib);auto t=attached(w,lib);t.cfg.automaticClimbActions=true;t.cfg.legacyAutomaticHops=true;
    w.boxes.push_back({{-20000,-98,-5000},{20000,-88,20000}});
    const float dt=1.f/fps;int frames=0;
    while(t.automaticAttemptCount()==0&&frames++<fps*5) {
        const auto result=t.update(w,{0,1},dt,1000);
        check(!result.released&&t.automaticActionCount()==0,"outward blocker rejects optional action but preserves normal travel");
    }
    check(t.automaticAttemptCount()==1,"a due blocked route is actually attempted");
    w.boxes.resize(1);float elapsed=0;
    while(t.automaticActionCount()==0&&elapsed<.5f) {t.update(w,{0,1},dt,1000);elapsed+=dt;}
    std::cout<<"retry fps="<<fps<<" elapsed="<<elapsed<<" attempts="<<t.automaticAttemptCount()<<" actions="<<t.automaticActionCount()<<" pending="<<t.automaticActionPendingSeconds()<<'\n';
    check(t.automaticActionCount()==1&&elapsed>=.25f&&elapsed<.4f,"short real-movement retry uses a newly clear route without a full new wait");
    auto fresh=attached(w,lib);fresh.cfg.automaticClimbActions=true;fresh.cfg.legacyAutomaticHops=true;
    w.boxes.push_back({{-20000,-98,-5000},{20000,-88,20000}});
    for(int i=0;i<fps*10;++i)fresh.update(w,{0,1},dt,1000);
    check(fresh.automaticActionCount()==0&&fresh.automaticAttemptCount()<=28,"permanently unsafe route never plays and retry work is bounded");
}
static void productionProfile(const Library& lib,int fps,int side) {
    auto w=longWall(lib,true);auto t=attached(w,lib);t.cfg.automaticClimbActions=true;t.cfg.legacyAutomaticHops=false;
    t.cfg.surfaceActionVariants=true;
    t.cfg.autoActionMinSeconds=1;t.cfg.autoActionMaxSeconds=1.6f;
    const float dt=1.f/fps;bool newHop=false;float elapsed=0;SurfacePose pose;
    for(int i=0;i<fps*5;++i) {
        const auto result=t.update(w,{float(side),0},dt,1000);elapsed+=dt;
        check(!result.released,"production-profile new leap keeps support");
        const auto body=pose.update(lib,w,t,result.motion,dt,1);
        for(int hand=0;hand<2;++hand)check(lib.armBendValid(body,hand),"new leap still has legal elbows");
        if(t.automaticActionCount()==1&&!newHop) {
            check(result.motion==(side<0?Motion::contextHopLeft:Motion::contextHopRight),"actual paired edges choose new captured side leap");
            check(elapsed>=.60f&&elapsed<1.f,"actual edge opportunity requires .35 seconds of movement and the bounded .26-second preparation");newHop=true;
        }
    }
    check(newHop&&t.automaticActionCount()>=2,"production profile repeatedly uses complete new side actions");
}
static void capturedPause(const Library& lib) {
    auto w=longWall(lib);auto t=attached(w,lib);t.cfg.automaticClimbActions=true;t.cfg.legacyAutomaticHops=true;
    for(int i=0;i<75;++i)t.update(w,{0,1},1.f/60,1000);
    w.boxes.push_back({{-20000,-20000,t.position.z+t.cfg.height+.02f},{20000,-10,t.position.z+t.cfg.height+12}});
    auto capture=std::make_unique<TraversalCapture>();capture->begin(t,{0,1},1.f/60,1000);
    TraversalCapture::RecordingWorld recorded(w,*capture);
    const auto result=t.update(recorded,{0,1},1.f/60,1000);capture->finish(t,result);
    auto decoded=std::make_unique<TraversalCapture>();std::string error;
    check(decoded->deserialize(capture->serialize(),error)&&decoded->replay().matched,"held progress and retry state replay exactly");
}
int main(int argc,char** argv){try {
    check(argc==2,"supply motion library");Library lib;check(lib.load(argv[1]),"production motion library loads");
    for(int fps:{30,60,120}) {
        interruptedTravel(lib,fps,false);interruptedTravel(lib,fps,true);
        rejectedRouteRetry(lib,fps);for(int side:{-1,1})productionProfile(lib,fps,side);
    }
    capturedPause(lib);
    std::cout<<"PASS random cadence under brief blocks, steering, rejected routes and new-source selection\n";
    return 0;
}catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}}
