#include "Pose.h"
#include "PoseHandoff.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
static void require(bool value,const char* text){if(!value)throw std::runtime_error(text);}
namespace fc {
class TraversalCapture {
public:
    static Traversal create(const Library& library,Motion motion) {
        Traversal t;library.configureThreepeat(t.cfg);t.cfg.gap=38;
        t.normal=t.surfaceNormal={0,-1,0};t.position={0,-38,0};
        t.state=motion==Motion::contextMantle?State::mantle:State::wall;
        t.actionMotion=t.entryMotion=motion;t.actionSeconds=t.entrySeconds=library.clip(motion).seconds;
        t.actionFrom=t.approachFrom=t.mantleFrom=t.position;
        t.actionTo=t.approachTo=t.mantleTo=t.position+Vec{0,65,140};
        t.mantleLip={0,0,140};t.moveDirection={0,1,0};t.actionDirection={0,1,0};
        return t;
    }
    static void phase(Traversal& t,float value) {t.approachTime=t.actionTime=t.mantleTime=value;}
    static void clocks(Traversal& t,State state,float entry,float action) {t.state=state;t.approachTime=entry;t.actionTime=action;}
    static void direction(Traversal& t,Vec value) {t.moveDirection=t.actionDirection=value;}
    static void mode(Traversal& t,bool running,bool obstacle) {t.running=running;t.obstacleJump=obstacle;t.actionBeganRunning=running;}
};
}
struct EmptyWorld:World {std::optional<Hit> ray(Vec,Vec) override{return {};}};
struct ContactWorld:World {
    unsigned hits{};
    std::optional<Hit> ray(Vec a,Vec b) override {
        if(a.y<0&&b.y>=0) {const float phase=-a.y/(b.y-a.y);++hits;return Hit{a+(b-a)*phase,{0,-1,0},true};}
        return {};
    }
};
static bool movingLoop(Motion m){return (m>=Motion::up&&m<=Motion::right)||runMotion(m);}
static bool idleLoop(Motion m){return m==Motion::hang||m==Motion::contextHang;}
static Library sourceLibrary(const Library& base,int rootMode) {
    auto library=base;library.clearAnimationOverrides();
    for(int id=1;id<=motionCount;++id)if(isActiveMotion(Motion(id))) {
        auto& clip=library.clips[id-1];clip.authoredPlayback=true;clip.seconds=1.6f;clip.stride=80;
        clip.frames.assign(193,base.clip(Motion::hang).frames.front());clip.contacts.assign(193,{});clip.trajectory={};
        if(rootMode) {
            clip.trajectory.count=5;
            clip.trajectory.knots[0]={0,{}};clip.trajectory.knots[1]={.25f,{4,3,20}};
            clip.trajectory.knots[2]={.5f,{-3,0,40}};clip.trajectory.knots[3]={.75f,{2,-3,60}};
            clip.trajectory.knots[4]={1,{0,0,80}};
            if(rootMode==2)clip.trajectory.knots[4].displacement={};
        }
        for(std::size_t index=0;index<clip.frames.size();++index) {
            const float phase=float(index)/float(clip.frames.size()-1);auto& pose=clip.frames[index];
            pose[0].t=clip.trajectory.sample(phase);pose[4].t.z+=3*std::sin(phase*6.2831853f);
            pose[24].q=(Quat::axis({0,0,1},.1f*std::sin(phase*6.2831853f))*pose[24].q).unit();
            pose[39].q=(Quat::axis({0,0,1},.70f*std::sin(phase*75.398224f))*pose[39].q).unit();
            pose[56].q=(Quat::axis({0,0,1},.25f*std::sin(phase*6.2831853f))*pose[56].q).unit();
        }
    }
    return library;
}
static void sourceCurves(const Library& library,int fps) {
    EmptyWorld world;float worstRotation=0,worstRoot=0,worstCOM=0;unsigned checked=0;
    for(int id=1;id<=motionCount;++id)if(isActiveMotion(Motion(id))) {
        const auto motion=Motion(id);const auto& clip=library.clip(motion);auto t=TraversalCapture::create(library,motion);
        SurfacePose surface;const float dt=1.f/fps;const int intervals=int(std::round(clip.seconds*fps));
        for(int frame=0;frame<=intervals;++frame) {
            const float phase=float(frame)/intervals;TraversalCapture::phase(t,phase);
            if(movingLoop(motion)&&frame)t.position.z+=clip.stride/intervals;
            const auto actual=surface.update(library,world,t,motion,dt,1);const auto sampled=surface.sampledPhase();
            const auto source=library.sample(motion,sampled);
            if(!movingLoop(motion)&&!idleLoop(motion))require(std::abs(sampled-phase)<.00001f,"one-shot keeps the complete source phase, without kick or reach remapping");
            if(movingLoop(motion))require(std::abs(sampled-std::fmod(phase,1.f))<.0001f||std::abs(sampled-1.f)<.0001f,"movement loops follow actual wall displacement");
            for(std::size_t bone=0;bone<actual.size();++bone) {
                worstRotation=std::max(worstRotation,angleBetween(actual[bone].q,source[bone].q));
                require(actual[bone].t.finite()&&std::abs(actual[bone].q.dot(actual[bone].q)-1)<.002f,"every active authored slot produces finite normalized transforms");
                if(bone!=0&&bone!=4)require(std::abs(actual[bone].t.length()-source[bone].t.length())<.0001f,"authored playback preserves every bone length");
            }
            Vec consumedRoot{};
            if(movingLoop(motion)) {
                const auto net=clip.trajectory.sample(1);if(net.length()>=2)consumedRoot=net*sampled;
            } else if(!idleLoop(motion)) {
                bool travel=false;for(std::uint32_t i=0;i<clip.trajectory.count;++i)travel|=clip.trajectory.knots[i].displacement.length()>=2;
                if(travel)consumedRoot=clip.trajectory.sample(sampled);
            }
            worstRoot=std::max(worstRoot,(actual[0].t-(source[0].t-consumedRoot)).length());
            worstCOM=std::max(worstCOM,(actual[4].t-source[4].t).length());
        }
        if(movingLoop(motion)) {
            const float before=surface.sampledPhase();
            for(int frame=0;frame<30;++frame)surface.update(library,world,t,motion,dt,1);
            require(std::abs(surface.sampledPhase()-before)<.00001f,"blocked movement does not advance the gait");
        }
        ++checked;
    }
    std::cout<<"authored source slots="<<checked<<" fps="<<fps<<" rotation="<<worstRotation<<" root="<<worstRoot<<" COM="<<worstCOM<<'\n';
    require(checked==activeMotionCount&&worstRotation<.0001f,"all active slots retain the source rotations without legacy run, brace or catch pose injection");
    require(worstRoot<.001f&&worstCOM<.001f,"only the controller-consumed Root portion is removed, with source COM preserved");
}
static void transitions(const Library& library) {
    EmptyWorld world;auto t=TraversalCapture::create(library,Motion::up);SurfacePose surface;Pose previous;
    for(Motion motion:{Motion::up,Motion::left,Motion::runLeft,Motion::runUp,Motion::kickUp,Motion::runCatch,Motion::hang,Motion::contextHopLeft,Motion::dropBack,Motion::backFlipOut}) {
        for(int frame=0;frame<60;++frame) {
            TraversalCapture::phase(t,float(frame)/60);t.position.z+=1.f;
            const auto pose=surface.update(library,world,t,motion,1.f/120,1);
            if(frame==0&&!previous.empty()) {
                float maximum=0;
                for(std::size_t i=0;i<pose.size();++i)maximum=std::max(maximum,angleBetween(previous[i].q,pose[i].q));
                require(maximum<.35f,"authored direction and action changes blend from the actual displayed pose");
                require(surface.bridgeMotion()==Motion::none,"authored transitions do not insert the unrelated legacy run bridge");
            }
            previous=pose;
        }
    }
    PoseHandoff handoff;const auto source=library.sample(Motion::backFlipOut,.99f),older=library.sample(Motion::backFlipOut,.98f);
    handoff.compose(library.rest,older,1,0);handoff.compose(library.rest,source,1,0);
    require(handoff.beginExit(true,false,true),"authored wall exit starts from the displayed source pose");
    handoff.advanceExitSource(0,library,0);
    const auto first=handoff.compose(library.rest,source,1,0);
    for(std::size_t i=0;i<first.size();++i)require(angleBetween(first[i].q,source[i].q)<.0001f,"exit handoff begins without an unrelated template pose");
    handoff.advanceExitSource(.16f,library,.16f);
    const auto last=handoff.compose(library.rest,source,0,0);
    for(std::size_t i=0;i<last.size();++i)require(angleBetween(last[i].q,library.rest[i].q)<.0001f,"completed exit rejoins the live native pose");
    auto uncalibrated=source;
    const auto from=source[29].q.rotate(source[38].t).unit();
    uncalibrated[29].q=(Quat::between(from,library.armBends[0].direction*-1)*source[29].q).unit();
    require(!library.armBendValid(uncalibrated,0),"exit fixture differs from the canonical hanging elbow calibration");
    handoff.clear();handoff.compose(library.rest,uncalibrated,1,0);
    require(handoff.beginExit(true,false,true),"authored exit accepts a finite source outside hanging-pose calibration");
    handoff.advanceExitSource(0,library,0);
    const auto preserved=handoff.compose(library.rest,uncalibrated,1,0);
    require(angleBetween(preserved[29].q,uncalibrated[29].q)<.0001f,"wall exit does not rewrite legitimate source elbow orientations using the default hanging pose");
    for(Motion motion:{Motion::reach,Motion::runLaunch,Motion::runLaunchLeft,Motion::runLaunchRight,Motion::runCatch}) {
        auto clock=TraversalCapture::create(library,motion);SurfacePose action;
        TraversalCapture::clocks(clock,State::action,.15f,.75f);action.update(library,world,clock,motion,1.f/120,1);
        require(std::abs(action.sampledPhase()-.75f)<.00001f,"Core transition actions use the action clock rather than a stale entry clock");
    }
    auto shortLibrary=library;shortLibrary.clips[int(Motion::sideBrace)-1].seconds=.05f;
    auto shortState=TraversalCapture::create(shortLibrary,Motion::sideBrace);SurfacePose shortSurface;
    shortSurface.update(shortLibrary,world,shortState,Motion::runLeft,1.f/120,1);
    for(int frame=0;frame<3;++frame) {
        TraversalCapture::phase(shortState,float(frame)/6);
        shortSurface.update(shortLibrary,world,shortState,Motion::sideBrace,1.f/120,1);
    }
    require(shortSurface.blendProgress()==1,"a short source transition is not hidden beneath a longer hardcoded blend");
    shortLibrary.clips[int(Motion::hang)-1].seconds=.025f;
    auto idleState=TraversalCapture::create(shortLibrary,Motion::hang);SurfacePose idleSurface;
    idleSurface.update(shortLibrary,world,idleState,Motion::hang,.01f,1);
    require(std::abs(idleSurface.sampledPhase()-.4f)<.00001f,"short authored idle loops do not inherit the legacy minimum clock duration");
}
static void entryHandoff(const Library& library,int fps) {
    for(Motion motion:{Motion::reach,Motion::jumpCatch,Motion::ledgeCatch,Motion::runLaunch,Motion::runLaunchLeft,Motion::runLaunchRight}) {
        ContactWorld wall;EmptyWorld preview;Traversal t;SurfacePose surface;
        require(library.configureThreepeat(t.cfg),"entry fixture configures actual source metadata");
        t.cfg.gap=38;t.cfg.approachSeconds=.2f;
        require(t.attach(wall,{0,-55,0},{0,1,0},100,1000,false,false,motion),"entry fixture attaches through actual wall geometry");
        require(t.entry(wall,motion),"source entry uses its actual checked path");
        const float dt=1.f/fps;float time=0,last=0;bool completed=false;
        while(t.state==State::approach&&time<3) {
            const auto result=t.update(wall,{},dt,100);time+=dt;
            require(result.motion==motion&&!result.released,"actual source entry stays selected throughout approach");
            surface.update(library,preview,t,result.motion,dt,1);
            require(surface.sampledPhase()+.00001f>=last,"source entry clock never resets on the wall-state handoff frame");
            require(std::abs(surface.sampledPhase()-t.reachProgress())<.00001f,"actual approach output uses its own source clock including the terminal wall-state frame");
            last=surface.sampledPhase();completed=t.state==State::wall;
        }
        require(completed&&last==1,"every entry slot displays its complete terminal source phase before ordinary wall movement");
        if(motion==Motion::runLaunch||motion==Motion::runLaunchLeft||motion==Motion::runLaunchRight) {
            Input input;input.x=motion==Motion::runLaunchLeft?-1.f:motion==Motion::runLaunchRight?1.f:0.f;
            input.y=motion==Motion::runLaunch?1.f:0.f;input.run=true;
            auto result=t.update(wall,input,dt,100);
            require(t.state==State::action&&result.motion==motion,"actual wall-run bridge reuses the entry-named slot as an independent source action");
            surface.update(library,preview,t,result.motion,dt,1);
            require(surface.sampledPhase()==0,"wall-run bridge starts from zero instead of stale completed entry time");
            time=0;last=0;
            while(t.state==State::action&&time<3) {
                result=t.update(wall,input,dt,100);time+=dt;
                require(result.motion==motion&&!result.released,"actual authored wall-run bridge remains selected until completion");
                surface.update(library,preview,t,result.motion,dt,1);
                require(std::abs(surface.sampledPhase()-t.actionProgress())<.00001f,"wall-run bridge includes its terminal action source phase");
                last=surface.sampledPhase();
            }
            require(t.state==State::wall&&last==1,"wall-run bridge completes source phase one before the running loop");
        }
    }
}
static void contacts(const Library& source) {
    auto library=source;unsigned totalHits=0;float largest=0;
    for(auto& clip:library.clips)if(clip.authoredPlayback)for(auto& weights:clip.contacts)weights={1,1,1,1};
    for(int id=1;id<=motionCount;++id)if(isActiveMotion(Motion(id))) {
        const auto motion=Motion(id);auto t=TraversalCapture::create(library,motion);SurfacePose surface,reference;ContactWorld world;EmptyWorld empty;
        for(int frame=0;frame<120;++frame) {
            TraversalCapture::phase(t,float(frame)/120);t.position.z+=.25f;
            const auto baseline=reference.update(library,empty,t,motion,1.f/120,1),pose=surface.update(library,world,t,motion,1.f/120,1);
            for(std::size_t bone=0;bone<pose.size();++bone) {
                const float angle=angleBetween(baseline[bone].q,pose[bone].q);largest=std::max(largest,angle);
                const bool extremity=bone==8||bone==11||bone==38||bone==39;
                require(angle<(extremity?.2619f:.7855f),"real contact corrections remain bounded relative to authored source pose");
                require(pose[bone].t.finite()&&std::abs(pose[bone].q.dot(pose[bone].q)-1)<.002f,"contact correction remains finite for every authored slot");
                if(bone!=0&&bone!=4)require(std::abs(pose[bone].t.length()-baseline[bone].t.length())<.0001f,"contact IK cannot lengthen a bone");
            }
            for(int hand=0;hand<2;++hand)if(library.armBendValid(baseline,hand))require(library.armBendValid(pose,hand),"real contact IK does not introduce a reverse elbow bend");
        }
        totalHits+=world.hits;
    }
    require(totalHits>0,"contact test exercises real surface intersections");
    std::cout<<"authored contact intersections="<<totalHits<<" max correction="<<largest<<'\n';
}
static void mixedPackSourceCurves(const Library& base,const Library& source) {
    EmptyWorld world;float worst=0;unsigned checked=0;
    for(int id=1;id<=motionCount;++id)if(isActiveMotion(Motion(id)))for(Vec direction:{Vec{-1,0,0},Vec{1,1,0}}) {
        const auto motion=Motion(id);auto library=base;library.clips[id-1]=source.clips[id-1];
        auto t=TraversalCapture::create(library,motion);TraversalCapture::direction(t,direction);SurfacePose surface;
        for(int frame=0;frame<=120;++frame) {
            const float phase=frame/120.f;TraversalCapture::phase(t,phase);
            if(movingLoop(motion)&&frame)t.position.z+=library.clip(motion).stride/120;
            const auto actual=surface.update(library,world,t,motion,1.f/120,1);
            const auto expected=library.sample(motion,surface.sampledPhase());
            for(std::size_t bone=0;bone<actual.size();++bone)worst=std::max(worst,angleBetween(actual[bone].q,expected[bone].q));
            require(surface.bridgeMotion()==Motion::none,"one external slot cannot select default-pack pose bridges or direction templates");
        }
        ++checked;
    }
    require(checked==activeMotionCount*2&&worst<.0001f,
        "each individually replaced slot preserves its source rotations with every other slot still using the default pack");
    std::cout<<"mixed-pack authored cases="<<checked<<" worst source rotation="<<worst<<'\n';
}
static void interruptedLegacyBridges(const Library& base,const Library& source) {
    for(bool returning:{false,true})for(int fps:{30,60,120}) {
        const Motion imported=returning?Motion::left:Motion::runRight;
        auto library=base;library.clips[int(imported)-1]=source.clip(imported);
        EmptyWorld world;auto t=TraversalCapture::create(library,Motion::hang);SurfacePose surface;
        const float dt=1.f/fps;TraversalCapture::direction(t,{-1,0,0});
        surface.update(library,world,t,Motion::hang,dt,1);
        Pose previous=surface.update(library,world,t,Motion::runLeft,dt,1);
        require(surface.bridgeMotion()==Motion::runLaunchLeft&&surface.blendProgress()<1,
            "mixed-pack fixture first starts an actual default wall-run launch bridge");
        if(returning) {
            for(int frame=0;frame<fps;++frame)surface.update(library,world,t,Motion::runLeft,dt,1);
            previous=surface.update(library,world,t,Motion::up,dt,1);
            require(surface.bridgeMotion()==Motion::runCatch&&surface.blendProgress()<1,
                "mixed-pack fixture starts an actual default return-to-climb bridge");
        }
        TraversalCapture::direction(t,returning?Vec{-1,0,0}:Vec{1,0,0});
        const auto pose=surface.update(library,world,t,imported,dt,1);
        require(surface.bridgeMotion()==Motion::none,
            "switching to an external slot discards an unfinished default bridge before source output");
        float firstStep=0;
        for(std::size_t bone=0;bone<pose.size();++bone)firstStep=std::max(firstStep,angleBetween(previous[bone].q,pose[bone].q));
        require(firstStep<.35f,"discarding a default bridge preserves continuity from the actually displayed outgoing pose");
        for(int frame=0;frame<fps;++frame) {
            t.position.z+=.25f;
            const auto actual=surface.update(library,world,t,imported,dt,1);
            require(surface.bridgeMotion()==Motion::none,"a discarded default bridge never resumes during external playback");
            if(surface.blendProgress()==1) {
                const auto expected=library.sample(imported,surface.sampledPhase());
                for(std::size_t bone=0;bone<actual.size();++bone)
                    require(angleBetween(actual[bone].q,expected[bone].q)<.0001f,"finished mixed-pack transition returns to the complete external source pose");
            }
        }
    }
}
static void externalHelpersAreNotTemplates(const Library& base,const Library& source) {
    struct Route {Motion active,helper;Vec direction;bool running{},obstacle{};};
    const std::array routes={
        Route{Motion::runLeft,Motion::sideBrace,{-1,0,0},true},
        Route{Motion::runRight,Motion::sideBrace,{1,0,0},true},
        Route{Motion::runDiagonalLeft,Motion::sideBrace,{-1,1,0},true},
        Route{Motion::runDiagonalRight,Motion::sideBrace,{1,1,0},true},
        Route{Motion::kickLeft,Motion::kickUp,{-1,0,0}},
        Route{Motion::kickRight,Motion::kickUp,{1,0,0}},
        Route{Motion::kickUp,Motion::hopUp,{0,1,0}},
        Route{Motion::kickLeft,Motion::hopLeft,{-1,0,0}},
        Route{Motion::kickRight,Motion::hopRight,{1,0,0}},
        Route{Motion::kickUp,Motion::runUp,{0,1,0},true,true},
        Route{Motion::kickLeft,Motion::runLeft,{-1,0,0},true,true},
        Route{Motion::kickRight,Motion::runRight,{1,0,0},true,true},
        Route{Motion::kickLeft,Motion::runDiagonalLeft,{-1,1,0},true,true},
        Route{Motion::kickRight,Motion::runDiagonalRight,{1,1,0},true,true},
        Route{Motion::kickUp,Motion::up,{0,1,0},false,true},
        Route{Motion::kickUp,Motion::down,{0,-1,0},false,true},
        Route{Motion::kickLeft,Motion::left,{-1,0,0},false,true},
        Route{Motion::kickRight,Motion::right,{1,0,0},false,true},
        Route{Motion::kickUp,Motion::hang,{},false,true},
        Route{Motion::runLaunch,Motion::runUp,{0,1,0},true},
        Route{Motion::runLaunch,Motion::up,{0,1,0}},
        Route{Motion::runLaunch,Motion::hang,{}}};
    float worst=0,brace=0;unsigned checked=0;
    for(const auto& route:routes) {
        auto a=base;a.clips[int(route.helper)-1]=source.clip(route.helper);auto b=a;
        for(auto& frame:b.clips[int(route.helper)-1].frames)for(std::size_t bone=5;bone<frame.size();++bone)
            frame[bone].q=(Quat::axis({0,0,1},.9f)*frame[bone].q).unit();
        auto ta=TraversalCapture::create(a,route.active),tb=TraversalCapture::create(b,route.active);
        TraversalCapture::direction(ta,route.direction);TraversalCapture::direction(tb,route.direction);
        TraversalCapture::mode(ta,route.running,route.obstacle);TraversalCapture::mode(tb,route.running,route.obstacle);
        EmptyWorld world;SurfacePose sa,sb;
        for(int frame=0;frame<=120;++frame) {
            const float phase=frame/120.f;TraversalCapture::phase(ta,phase);TraversalCapture::phase(tb,phase);
            if(movingLoop(route.active)&&frame){ta.position.z+=1;tb.position.z+=1;}
            const auto pa=sa.update(a,world,ta,route.active,1.f/120,1),pb=sb.update(b,world,tb,route.active,1.f/120,1);
            require(pa.size()==99&&pb.size()==99,"mixed default routes retain complete poses when imported helper templates are skipped");
            for(std::size_t bone=0;bone<pa.size();++bone) {
                worst=std::max(worst,angleBetween(pa[bone].q,pb[bone].q));
                require((pa[bone].t-pb[bone].t).length()<.0001f&&pa[bone].t.finite(),
                    "changing an unused external helper cannot alter another default clip's translations");
            }
            if(route.helper==Motion::sideBrace) {
                require(a.armBendValid(pa,0)&&a.armBendValid(pa,1),"default bracing still enforces its anatomical elbow guards without importing a transition pose");
                const auto plain=a.sample(route.active,sa.sampledPhase());
                for(int bone:{29,32})brace=std::max(brace,angleBetween(pa[bone].q,plain[bone].q));
            }
        }
        ++checked;
    }
    require(worst<.0001f&&brace>.01f,"external helpers are not partial-pose templates, while default run bracing remains active");
    std::cout<<"mixed external helper routes="<<checked<<" worst leaked rotation="<<worst<<" retained brace="<<brace<<'\n';
    for(Motion helper:{Motion::runLaunch,Motion::runLaunchLeft,Motion::runLaunchRight,Motion::runCatch}) {
        auto library=base;library.clips[int(helper)-1]=source.clip(helper);EmptyWorld world;SurfacePose surface;
        auto t=TraversalCapture::create(library,Motion::hang);
        const Motion from=helper==Motion::runCatch?Motion::runUp:Motion::hang;
        const Motion to=helper==Motion::runCatch?Motion::up:helper==Motion::runLaunchLeft?Motion::runLeft:
            helper==Motion::runLaunchRight?Motion::runRight:Motion::runUp;
        TraversalCapture::direction(t,{helper==Motion::runLaunchLeft?-1.f:helper==Motion::runLaunchRight?1.f:0.f,1,0});
        for(int frame=0;frame<60;++frame)surface.update(library,world,t,from,1.f/60,1);
        for(int frame=0;frame<30;++frame) {
            const auto pose=surface.update(library,world,t,to,1.f/60,1);
            require(pose.size()==99&&surface.bridgeMotion()==Motion::none,
                "an external launch or catch is reserved for its full Core action and never sampled as a short default bridge");
        }
    }
}
static void mixedReturnToDefault(const Library& base,const Library& source) {
    EmptyWorld world;
    for(int id=1;id<=motionCount;++id)if(isActiveMotion(Motion(id))) {
        const Motion imported=Motion(id),target=imported==Motion::hang?Motion::up:Motion::hang;
        auto library=base;library.clips[id-1]=source.clips[id-1];
        auto t=TraversalCapture::create(library,imported);SurfacePose surface;
        Pose previous;
        for(int frame=0;frame<60;++frame) {
            TraversalCapture::phase(t,float(frame)/60);t.position.z+=.25f;
            previous=surface.update(library,world,t,imported,1.f/120,1);
        }
        float firstStep=0,initialPhase=0,phaseAdvance=0;
        for(int frame=0;frame<120;++frame) {
            const auto actual=surface.update(library,world,t,target,1.f/120,1);
            require(actual.size()==99&&surface.bridgeMotion()==Motion::none,
                "external-to-default returns retain a complete pose without an unrelated bridge");
            if(frame==0) {
                initialPhase=surface.sampledPhase();
                for(std::size_t bone=0;bone<actual.size();++bone)
                    firstStep=std::max(firstStep,angleBetween(previous[bone].q,actual[bone].q));
            }
            for(const auto& bone:actual)require(bone.t.finite()&&std::abs(bone.q.dot(bone.q)-1)<.002f,
                "external-to-default returns remain finite and normalized");
            for(int hand=0;hand<2;++hand)if(library.armBendValid(previous,hand))
                require(library.armBendValid(actual,hand),"a continuous mixed return cannot cross from a valid elbow into the forbidden branch");
            previous=actual;
            phaseAdvance=std::max(phaseAdvance,std::abs(initialPhase-surface.sampledPhase()));
        }
        if(firstStep>=.35f)std::cerr<<"external return slot="<<id<<" first step="<<firstStep<<'\n';
        require(firstStep<.35f,"external-to-default return blends from the actually displayed external pose");
        if(imported>=Motion::up&&imported<=Motion::right)require(phaseAdvance>.01f,
            "default hang advances its own clock instead of freezing a sampled fragment from an external climb slot");
        auto guarded=previous;library.guardArmBends(guarded);library.forearmTwist(guarded);
        for(std::size_t bone=0;bone<previous.size();++bone)require(angleBetween(previous[bone].q,guarded[bone].q)<.0001f,
            "completed external return rejoins the existing default arm and forearm limits");
    }
}
int main(int argc,char** argv) try {
    require(argc==2,"animation pack argument required");Library base;require(base.load(argv[1]),"load canonical animation pack");
    base.wallRunSequenceValid={};
    const auto mixedSource=sourceLibrary(base,1);mixedPackSourceCurves(base,mixedSource);interruptedLegacyBridges(base,mixedSource);
    externalHelpersAreNotTemplates(base,mixedSource);
    mixedReturnToDefault(base,mixedSource);
    for(int rootMode:{0,1,2}) {const auto library=sourceLibrary(base,rootMode);for(int fps:{30,60,120}){sourceCurves(library,fps);if(rootMode==0)entryHandoff(library,fps);}transitions(library);contacts(library);}
    auto real=base;
    for(auto& clip:real.clips)if(clip.frames.size()>=2) {clip.authoredPlayback=true;clip.contacts.assign(clip.frames.size(),{});clip.trajectory={};}
    sourceCurves(real,60);contacts(real);
    mixedReturnToDefault(base,real);
    std::cout<<"All authored slot source, displacement, transition and contact checks passed\n";return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
