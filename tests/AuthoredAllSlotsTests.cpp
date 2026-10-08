#define main AuthoredBaseTestMain
#include "AuthoredPlaybackTests.cpp"
#undef main
#include <functional>
#include "CornerTestWorld.h"
namespace fc {
class TraversalCapture {
public:
    static void action(Traversal& t,Motion m,Vec to){t.beginAction(m,t.position,to,.41f);}
    static bool check(Traversal& t,World& w){return t.authoredActionClear(w);}
    static Motion selected(const Traversal& t){return t.actionMotion;}
    static std::optional<Result> manual(Traversal& t,World& w){return t.tryManualHop(w,{1,0,false,false,true},false);}
    static bool eave(Traversal& t,World& w){return t.tryEaveTransfer(w,{0,1},false,0,1000);}
    static bool route(Traversal& t,World& w,Motion m,Vec target,Vec outside,Vec over){return t.commitAuthoredRoute(w,m,target,.8f,t.normal,true,outside,over,true,t.surfaceNormal);}
    static bool direct(Traversal& t,World& w,Vec a,Vec b){return t.clearPath(w,a,b);}
    static void settled(Traversal& t,Motion m){t.stableMotion=m;t.actionCooldown=0;t.state=State::wall;}
};
}
struct SourceWall:AuthoredLedge {
    unsigned bodies{};float bodyError{};Traversal* current{};
    SourceWall(){height=10000;floor=false;}
    bool actionBodyClear(Motion,Vec,Vec,float,float,Vec)override{return true;}
    bool actionBodyPathClear(Motion,Vec,Vec,float a,float b,Vec,Vec midpoint)override {
        ++bodies;
        if(current)bodyError=std::max(bodyError,(current->actionPathPoint((a+b)*.5f)-midpoint).length());
        return true;
    }
};
static Library fullLibrary(const Library& base) {
    auto library=base;library.wallRunSequenceValid={};
    for(int id=1;id<=motionCount;++id)if(isActiveMotion(Motion(id))) {
        auto& clip=library.clips[id-1];clip.authoredPlayback=true;clip.seconds=.65f+id*.011f;
        clip.frames.assign(121,base.clip(Motion::hang).frames.front());clip.contacts.assign(121,{});
        if(authoredIdleLoop(Motion(id)))continue;
        if(authoredMovingLoop(Motion(id))) {
            clip.stride=48;clip.trajectory.count=5;
            clip.trajectory.knots[0]={0,{}};clip.trajectory.knots[1]={.2f,{2,4,3}};
            clip.trajectory.knots[2]={.5f,{-3,28,-2}};clip.trajectory.knots[3]={.8f,{1,42,4}};clip.trajectory.knots[4]={1,{0,48,0}};
        } else {
            clip.trajectory.count=6;
            clip.trajectory.knots[0]={0,{}};clip.trajectory.knots[1]={.2f,{-5,-180,18}};
            clip.trajectory.knots[2]={.4f,{15,-180,35}};clip.trajectory.knots[3]={.6f,{40,-180,20}};
            clip.trajectory.knots[4]={.8f,{42,-180,10}};clip.trajectory.knots[5]={1,{48,0,0}};
        }
        for(std::size_t frame=0;frame<clip.frames.size();++frame)clip.frames[frame][0].t=clip.frames[frame][0].t+clip.trajectory.sample(float(frame)/120);
    }
    return library;
}
static Traversal wall(SourceWall& world,const Library& library) {
    auto t=attach(world,library);t.position.z=200;return t;
}
static void clocksAndRoutes(const Library& library) {
    for(int id=1;id<=motionCount;++id) {
        const auto m=Motion(id);if(!isActiveMotion(m)||authoredMovingLoop(m)||authoredIdleLoop(m)||m==Motion::contextMantle)continue;
        for(int fps:{30,60,120}) {
            SourceWall world;auto t=wall(world,library);const auto start=t.position;
            const bool departure=m==Motion::drop||m==Motion::dropBack||m==Motion::backFlipOut;
            const auto destination=departure?(m==Motion::drop?start:start+t.normal*100.f):start+Vec{60,0,0};
            TraversalCapture::action(t,m,destination);world.current=&t;
            require(TraversalCapture::check(t,world),"source path passes actual solid-wall clearance before playback");
            const auto middle=t.actionPathPoint(.4f);
            require((middle-start).y<-100,"authored out-from-wall trajectory is executed instead of the old short hop arc");
            const auto reverse=t.actionPathPoint(.2f);
            require(departure||(reverse-start).x<0,"source path can retain a checked initial reverse movement");
            float elapsed=0;Result result;
            while(t.state==State::action&&elapsed<3) {
                result=t.update(world,{},1.f/fps,1000);elapsed+=1.f/fps;
                require(result.motion==m,"one-shot source animation remains selected through its complete clock");
                require((t.position-t.actionPathPoint(t.actionProgress())).length()<.002f,"actual body movement follows the prechecked source route");
            }
            require(std::abs(elapsed-library.clip(m).seconds)<=1.01f/fps,"one-shot duration equals source duration rather than default action length");
            require(t.actionProgress()==1,"source action reaches its final frame");
            require(result.released==departure,"source action preserves the selected exit mechanism");
            require(world.bodyError<.002f,"backflip body queries use the actual source route midpoint");
            if(departure)require(result.releaseVelocity.length()<=600.01f,"authored terminal release speed remains bounded");
        }
    }
}
static void entryClocks(const Library& library) {
    for(const auto m:{Motion::reach,Motion::jumpCatch,Motion::ledgeCatch,Motion::runLaunch,Motion::runLaunchLeft,Motion::runLaunchRight}) {
        SourceWall world;Traversal t;library.configureThreepeat(t.cfg);
        require(t.attach(world,{0,-60,200},{0,1,0},1000),"entry fixture finds a real supported wall");
        require(t.entry(world,m,false),"source entry route receives a full preflight before taking control");
        float elapsed=0;
        while(t.state==State::approach&&elapsed<3) {
            const auto result=t.update(world,{},1.f/60,1000);elapsed+=1.f/60;
            require(result.motion==m,"selected catch uses its own complete source animation");
            require((t.position-t.entryPathPoint(t.reachProgress())).length()<.002f,"entry live route matches its preflight path");
        }
        require(t.state==State::wall&&std::abs(elapsed-library.clip(m).seconds)<.017f,"entry source clock completes without the old fixed preparation duration");
    }
}
static void originalInputAndBlocks(const Library& base,const Library& library) {
    for(bool rooted:{false,true}) {
        auto source=library;
        if(!rooted)for(auto& clip:source.clips)if(clip.authoredPlayback){clip.trajectory={};for(auto& pose:clip.frames)pose[0].t=base.clip(Motion::hang).frames.front()[0].t;}
        for(bool outward:{false,true}) {
            SourceWall world;auto t=wall(world,source);t.cfg.fancyJumps=false;
            const auto expected=outward?Motion::dropBack:Motion::drop;const auto start=t.position;
            float elapsed=0;Result result;
            do {result=t.update(world,{0,0,true,false,false,outward},1.f/60,1000);elapsed+=1.f/60;}while(t.active()&&elapsed<3);
            require(result.released&&std::abs(elapsed-source.clip(expected).seconds)<.018f,"actual release input starts the full source exit clock");
            require(rooted||t.authoredRoot(expected,.5f).length()==0,"in-place fallback does not claim to consume absent source travel");
            require(!outward?(t.position-start).length()<.001f:true,"in-place let-go ends at its verified original position");
        }
    }
    SourceWall world;auto t=wall(world,library);t.cfg.fancyJumps=false;
    const auto before=t.position;world.barrier=before.y-60;
    const auto failed=t.update(world,{0,0,true,false,false,true},1.f/60,1000);
    require(t.active()&&t.state==State::wall&&(t.position-before).length()<.001f&&!failed.released&&failed.staminaCost==0,
        "blocked authored release is rejected before any movement or stamina charge");
    world.barrier=1e9f;t.cfg.automaticClimbActions=false;
    const auto hop=t.update(world,{1,0,false,false,true},1.f/60,1000);
    require(!hop.released,"failed custom exit retains normal attached control");
}
static void bridgeClocks(const Library& library) {
    for(const auto row:{std::pair{Motion::runLaunch,Input{0,1,false,false,false,false,true}},
        std::pair{Motion::runLaunchLeft,Input{-1,0,false,false,false,false,true}},
        std::pair{Motion::runLaunchRight,Input{1,0,false,false,false,false,true}},
        std::pair{Motion::runCatch,Input{}},std::pair{Motion::sideBrace,Input{1,0,false,false,false,false,true}}}) {
        SourceWall world;auto t=wall(world,library);
        if(row.first==Motion::runCatch||row.first==Motion::sideBrace)TraversalCapture::settled(t,Motion::runLeft);
        const auto started=t.update(world,row.second,1.f/60,1000);
        require(started.motion==row.first&&t.state==State::action,"mode and direction changes select the authored bridge as a real source-timed action");
        float elapsed=0;
        while(t.state==State::action&&elapsed<3){const auto result=t.update(world,row.second,1.f/60,1000);elapsed+=1.f/60;require(result.motion==row.first,"bridge is not replaced by a partial fixed-duration blend");}
        require(std::abs(elapsed-library.clip(row.first).seconds)<.018f,"bridge reaches its complete source duration");
    }
}
static void actualRouteGeometry(const Library& library) {
    using fc_test::CornerWorld;
    auto make=[] {CornerWorld w;w.boxes.push_back({{-10000,0,-10000},{10000,1000,10000}});return w;};
    for(bool blocking:{false,true}) {
        auto world=make();Traversal t;t.cfg=fc_test::settings();library.configureThreepeat(t.cfg);
        require(t.attach(world,{0,-37,200},{0,1,0},1000),"route test attaches before adding nearby obstacle");
        t.state=State::wall;
        world.boxes.push_back({{35,blocking?-400.f:-110.f,-1000},{45,0,2000},false});
        auto legacy=t;legacy.cfg.authoredMotions.reset();
        require(!TraversalCapture::manual(legacy,world),"old short side-hop arc is blocked by the real projecting obstacle");
        const auto start=t.position;const auto action=TraversalCapture::manual(t,world);
        require(bool(action)==!blocking,"source side-hop route replaces obsolete arc prefilter but still rejects a blocked source curve");
        if(!action){require((t.position-start).length()<.001f,"rejected source route cannot move the actor");continue;}
        require(TraversalCapture::check(t,world),"selected side hop preflight and live route agree");
        float elapsed=0;
        while(t.state==State::action&&elapsed<2) {
            const auto out=t.update(world,{},1.f/120,1000);elapsed+=1.f/120;
            require(!out.released,"clear source curve completes despite the blocked default arc");
            require(world.clearance(t.position,t.cfg)+.03f>=t.cfg.radius,"actual source body route preserves the full controller radius");
        }
        require(t.state==State::wall&&t.actionProgress()==1,"source side hop reaches real target support");
    }
    for(bool blocking:{false,true}) {
        auto world=make();Traversal t;t.cfg=fc_test::settings();library.configureThreepeat(t.cfg);
        require(t.attach(world,{0,-37,200},{0,1,0},1000),"detour fixture has real supported source");t.state=State::wall;
        const auto start=t.position,outside=start+t.normal*40,over=outside+Vec{0,0,180},target=start+Vec{0,0,180};
        world.boxes.push_back({{-1000,blocking?-400.f:-100.f,350},{1000,0,362},false});
        require(!TraversalCapture::direct(t,world,outside,over),"old middle detour segment intersects the overhang");
        const bool accepted=TraversalCapture::route(t,world,Motion::hopUp,target,outside,over);
        require(accepted==!blocking,"detour preflight executes the real source residual over the planned bypass");
        if(!accepted){require((t.position-start).length()<.001f,"failed detour leaves original traversal unchanged");continue;}
        float elapsed=0;
        while(t.state==State::action&&elapsed<2) {
            const auto out=t.update(world,{},1.f/120,1000);elapsed+=1.f/120;
            require(!out.released,"source detour uses the same checked curve during playback");
            require(world.clearance(t.position,t.cfg)+.03f>=t.cfg.radius,"source detour never reduces the real capsule radius");
        }
        require(t.state==State::wall,"source detour reaches its supported upper wall");
    }
}
static void entryBudget(const Library& library) {
    struct CountedWall:SourceWall {
        unsigned casts{};
        std::optional<Hit> ray(Vec a,Vec b) override {++casts;return SourceWall::ray(a,b);}
    };
    unsigned peak=0,attempts=0;
    for(float distance:{30.f,60.f,105.f})for(float obstruction:{-15.f,-40.f,-100.f,1e9f})
        for(auto motion:{Motion::reach,Motion::jumpCatch,Motion::ledgeCatch}) {
            CountedWall world;world.barrier=obstruction;Traversal t;library.configureThreepeat(t.cfg);
            const bool attached=t.attach(world,{0,-distance,200},{0,1,0},1000,1000,false,true,motion);
            peak=std::max(peak,world.casts);++attempts;
            require(world.casts<=12288,"authored entry shares a bounded query budget across all candidate routes");
            if(!attached)require(!t.active(),"blocked or exhausted entry does not acquire traversal");
        }
    std::cout<<"authored entry attempts="<<attempts<<" peak queries="<<peak<<'\n';
}
static void authoredManualVariants(const Library& library) {
    struct CountedWall:SourceWall {
        unsigned casts{};
        std::optional<Hit> ray(Vec a,Vec b)override {++casts;return SourceWall::ray(a,b);}
    };
    unsigned peak=0;
    for(bool rooted:{false,true})for(Input input:{Input{0,1},Input{-1,0},Input{1,0}}) {
        auto source=library;
        if(!rooted)for(auto& clip:source.clips)clip.trajectory={};
        CountedWall world;auto t=wall(world,source);t.cfg.contextActions=false;input.hop=true;
        for(unsigned attempt=0;attempt<3;++attempt) {
            const auto hop=input.x<0?Motion::hopLeft:input.x>0?Motion::hopRight:Motion::hopUp;
            const auto expected=hop;
            world.casts=0;auto result=t.update(world,input,1.f/60,1000);peak=std::max(peak,world.casts);
            require(t.state==State::action&&result.motion==expected,"repeated manual input selects the complete authored directional hop without a flip variant");
            require(world.casts<=12500,"one authored manual request keeps its complete preflight within the shared budget");
            const unsigned preflight=world.casts;float elapsed=0,cost=result.staminaCost;
            result=t.update(world,{},1.f/60,1000);elapsed+=1.f/60;cost+=result.staminaCost;
            require(world.casts-preflight<preflight/2,"first live authored action never repeats the complete successful preflight");
            while(t.state==State::action&&elapsed<3) {
                result=t.update(world,{},1.f/60,1000);elapsed+=1.f/60;cost+=result.staminaCost;
                require(!result.released&&result.motion==expected,"authored manual variant retains its exact selected source through landing");
            }
            require(t.state==State::wall&&t.actionProgress()==1&&std::abs(elapsed-source.clip(expected).seconds)<.018f,
                "each repeated authored hop retains its complete source clock");
            require(std::abs(cost-15)<.001f,"authored preflight and live playback charge the action only once");
            for(unsigned frame=0;frame<30;++frame)t.update(world,{},1.f/60,1000);
        }
    }
    std::cout<<"authored manual variant peak queries="<<peak<<'\n';
}
int main(int argc,char** argv)try {
    require(argc==2,"pack argument required");Library base;require(base.load(argv[1]),"load default pack");
    Settings defaults;base.configureThreepeat(defaults);require(!defaults.authoredMotions,"default v1 library allocates no authored playback registry");
    const auto library=fullLibrary(base);Settings settings;library.configureThreepeat(settings);const auto copy=settings;
    require(settings.authoredMotions&&copy.authoredMotions==settings.authoredMotions,"Traversal copies retain immutable shared author data without copying every curve");
    require(sizeof(Settings)<8192,"default Settings does not grow by all 42 source paths");
    for(int id=1;id<=motionCount;++id)if(isActiveMotion(Motion(id)))require((*settings.authoredMotions)[id-1].enabled,"all active slots reach the controller metadata");
    clocksAndRoutes(library);entryClocks(library);originalInputAndBlocks(base,library);bridgeClocks(library);actualRouteGeometry(library);entryBudget(library);authoredManualVariants(library);
    std::cout<<"Authored all-slot clocks, source paths, entry, exits, bridges and collision checks passed; Settings="<<sizeof(Settings)<<'\n';
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
