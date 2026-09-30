#include "Pose.h"
#include "Controls.h"
#include <utility>
#include "PoseOutput.h"
#include "WallRunPose.h"
#include "PoseHandoff.h"
#include <iostream>
#include <filesystem>
#include <stdexcept>
#include <sstream>
using namespace fc;
void require(bool v,const char* message) { if(!v)throw std::runtime_error(message); }
struct Plane:World {
    Vec normal{0,-1,0};
    std::optional<Hit> ray(Vec a,Vec b) override {
        float x=a.dot(normal),y=b.dot(normal);
        if(x<=0||y>=0)return {};
        return Hit{a+(b-a)*(x/(x-y)),normal,true};
    }
};
struct Ledge:World {
    float height=120;
    std::optional<Hit> ray(Vec a,Vec b) override {
        std::optional<Hit> out;float nearest=2;
        if(a.y<0&&b.y>=0) {
            float t=-a.y/(b.y-a.y);Vec point=a+(b-a)*t;
            if(point.z<=height&&t<nearest) {nearest=t;out=Hit{point,{0,-1,0},true};}
        }
        if(a.z>height&&b.z<=height) {
            float t=(height-a.z)/(b.z-a.z);Vec point=a+(b-a)*t;
            if(point.y>=0&&t<nearest)out=Hit{point,{0,0,1},true};
        }
        return out;
    }
};

struct TransitionTrace {
    const Library& lib;SurfacePose animator;Pose previous,previousWorld;
    Vec previousPosition{},previousNormal{};Motion previousMotion=Motion::none;
    std::vector<std::pair<Motion,Motion>> edges;
    float worstStartAngle{},worstStartDistance{},worstEndDistance{};
    explicit TransitionTrace(const Library& value):lib(value){}
    static Vec point(Vec v,Vec position,Vec normal) {
        return position+Vec{-normal.y,normal.x,0}*v.x-normal*v.y+Vec{0,0,v.z};
    }
    bool saw(Motion from,Motion to) const {
        return std::find(edges.begin(),edges.end(),std::pair{from,to})!=edges.end();
    }
    Result tick(World& world,Traversal& t,Input input={}) {
        constexpr float dt=1.f/60;
        const auto r=t.update(world,input,dt,100);
        const auto p=animator.update(lib,world,t,r.motion,dt,1),w=lib.world(p);
        for(const auto& tr:p)require(tr.t.finite()&&std::isfinite(tr.q.dot(tr.q))&&std::abs(tr.q.dot(tr.q)-1)<.002f,
            "entry/action/exit transition poses remain finite and normalized");
        if(!previous.empty()) {
            float step=0,angle=0;
            for(int bone:{38,39,8,11})step=std::max(step,(point(w[bone].t,t.position,t.normal)-
                point(previousWorld[bone].t,previousPosition,previousNormal)).length());
            for(int bone:{0,4,6,7,9,10,24,28,29,31,32,38,39})
                angle=std::max(angle,angleBetween(previous[bone].q,p[bone].q));
            if(r.motion!=previousMotion) {
                edges.emplace_back(previousMotion,r.motion);
                worstStartAngle=std::max(worstStartAngle,angle);worstStartDistance=std::max(worstStartDistance,step);
                const float limit=(flipMotion(r.motion)?1500.f:runMotion(r.motion)||runMotion(previousMotion)?1100.f:
                    r.motion==Motion::contextMantle?750.f:600.f)*dt+.6f;
                if(angle>=.28f||step>limit)std::cerr<<"action boundary from="<<int(previousMotion)<<" to="<<int(r.motion)
                    <<" firstAngle="<<angle<<" firstEndpoint="<<step<<" limit="<<limit<<'\n';
                require(angle<.28f&&step<=limit,"a reachable action switch begins at the outgoing pose, not a middle frame");
            }
            if(r.completed||r.released) {
                worstEndDistance=std::max(worstEndDistance,step);
                require(step<13.1f,"last controlled frame remains continuous when top-out or drop finishes");
            }
        }
        const auto position=t.position;const auto state=t.state;const auto progress=t.progress();
        const auto duplicate=t.update(world,{},0,100);
        require((t.position-position).length()==0&&t.state==state&&t.progress()==progress,
            "zero delta preserves action position, state and progress");
        const auto held=animator.update(lib,world,t,duplicate.motion,0,1);
        for(std::size_t bone=0;bone<p.size();++bone)
            require((held[bone].t-p[bone].t).length()<.000001f&&std::abs(held[bone].q.x-p[bone].q.x)+std::abs(held[bone].q.y-p[bone].q.y)+std::abs(held[bone].q.z-p[bone].q.z)+std::abs(held[bone].q.w-p[bone].q.w)<.000001f,
                "zero delta preserves every bone through entry, action, catch and release");
        previous=p;previousWorld=w;previousPosition=t.position;previousNormal=t.normal;previousMotion=r.motion;
        return r;
    }
    void report(const char* label) const {
        std::cout<<"reachable transitions "<<label<<" edges="<<edges.size()<<" firstAngle="<<worstStartAngle
            <<" firstEndpoint="<<worstStartDistance<<" finalEndpoint="<<worstEndDistance<<'\n';
    }
};
struct ChangingFoothold:Ledge {
    bool feet=true;
    std::optional<Hit> ray(Vec a,Vec b) override {
        auto hit=Ledge::ray(a,b);
        if(hit&&!feet&&hit->normal.z<.5f&&hit->point.z<80)return {};
        return hit;
    }
};
static void reachableActionTransitions(const Library& lib) {
    Plane wall;
    for(Motion entry:{Motion::reach,Motion::jumpCatch,Motion::sprintCatch,Motion::ledgeCatch}) {
        Traversal t;t.cfg.gap=37;t.cfg.radius=31;TransitionTrace trace(lib);
        require(t.attach(wall,{0,-42,200},{0,1,0},100),"entry transition attach");t.entry(entry,entry==Motion::sprintCatch);
        for(int frame=0;frame<70;++frame)trace.tick(wall,t,{0,1});
        require(t.active()&&trace.saw(entry,Motion::up),"each real entry reaches continuous climbing");
        trace.report("entry to climb");
    }
    struct ActionCase {Motion motion;float x,y;bool run,fancy,resumeRun;};
    for(const auto c:{ActionCase{Motion::hopUp,0,1,false,true,false},
            ActionCase{Motion::hopRight,1,1,false,true,true},ActionCase{Motion::hopLeft,-1,1,false,false,false},
            ActionCase{Motion::hopUp,0,1,false,true,true}}) {
        Traversal t;t.cfg.approachSeconds=0;t.cfg.fancyJumps=c.fancy;TransitionTrace trace(lib);
        require(t.attach(wall,{0,-37,200},{0,1,0},100),"action catch transition attach");
        for(int frame=0;frame<30;++frame)trace.tick(wall,t,{});
        auto r=trace.tick(wall,t,{c.x,c.y,false,false,true,false,c.run});
        require(r.motion==c.motion&&t.state==State::action,"requested reachable ordinary hop starts");
        for(int frame=0;frame<130&&t.state==State::action;++frame)trace.tick(wall,t,{});
        require(t.active()&&t.state!=State::action,"action finishes at a supported catch");
        const Input next=c.resumeRun?Input{c.x,c.y,false,false,false,false,true}:Input{};
        r=trace.tick(wall,t,next);const auto caught=r.motion;
        require(c.resumeRun?runMotion(caught):caught==Motion::hang,"catch continues into hang or wall run");
        require(trace.saw(c.motion,caught),"completed action uses a continuous outgoing-to-catch edge");
        for(int frame=0;frame<40;++frame)trace.tick(wall,t,next);
        trace.report("ordinary hop to catch");
    }
    for(bool holdShiftOnRelease:{false,true}) {
        Traversal t;t.cfg.approachSeconds=0;TransitionTrace trace(lib);
        require(t.attach(wall,{0,-37,200},{0,1,0},100),"action interruption attach");
        for(int frame=0;frame<30;++frame)trace.tick(wall,t,{0,1});
        auto r=trace.tick(wall,t,{1,1,false,false,true});const auto action=r.motion;
        require(action==Motion::hopRight,"interruption begins from a real ordinary jump");
        for(int frame=0;frame<80&&t.state==State::action&&t.actionProgress()<.43f;++frame)trace.tick(wall,t,{});
        require(t.state==State::action,"S+Space occurs during the ordinary hop");
        r=trace.tick(wall,t,wallInput(Keys{false,false,true,false,holdShiftOnRelease,true},true));
        require(r.motion==Motion::dropBack&&trace.saw(action,Motion::dropBack),"S+Space blends current jump into release");
        for(int frame=0;frame<30&&t.active();++frame)r=trace.tick(wall,t,{});
        require(!t.active()&&r.released,"interrupted action releases control after its visible push");
        trace.report("midair S+Space release");
    }
    {
        ChangingFoothold ledge;Traversal t;require(lib.configureThreepeat(t.cfg),"runtime top calibration");t.cfg.approachSeconds=0;TransitionTrace trace(lib);
        require(t.attach(ledge,{0,-37,0},{0,1,0},100),"foothold transition attach");
        for(int frame=0;frame<30;++frame)trace.tick(ledge,t,{});
        const auto idle=t.position;
        ledge.feet=false;
        for(int frame=0;frame<60;++frame) {
            const auto r=trace.tick(ledge,t,{});
            require(r.motion==Motion::hang&&t.state!=State::action&&(t.position-idle).length()<.00001f,
                "lost foothold preserves ordinary stationary pose and position");
        }
        ledge.feet=true;
        for(int frame=0;frame<60;++frame) {
            const auto r=trace.tick(ledge,t,{});
            require(r.motion==Motion::hang&&t.state!=State::action&&(t.position-idle).length()<.00001f,
                "regained foothold does not insert a retired bracing transfer");
        }
        auto r=trace.tick(ledge,t,{0,1,false,true});
        require(t.state==State::mantle,"restored support permits measured top-out from ordinary idle");
        for(int frame=0;frame<180&&t.active();++frame)r=trace.tick(ledge,t,{0,1,false,true});
        require(r.completed&&t.position.z>=ledge.height,"restored support top-out completes on the real platform");
        trace.report("ordinary idle across support changes");
    }
    for(float height:{80.f,120.f}) {
        Ledge ledge;ledge.height=height;Traversal t;require(lib.configureThreepeat(t.cfg),"runtime top calibration");t.cfg.approachSeconds=0;t.cfg.gap=37;t.cfg.radius=31;TransitionTrace trace(lib);
        require(t.attach(ledge,{0,-42,0},{0,1,0},100),"climbing top transfer attach");
        for(int frame=0;frame<30;++frame)trace.tick(ledge,t,{1,0});
        auto r=trace.tick(ledge,t,{0,1,false,true});const auto top=r.motion;
        require(top==Motion::contextMantle&&trace.saw(Motion::right,top),
            "continuous climbing enters the measured top without a hard cut");
        for(int frame=0;frame<180&&t.active();++frame)r=trace.tick(ledge,t,{0,1,false,true});
        require(r.completed&&t.position.z>=height,"continuous top-out reaches the real platform");
        trace.report("climb to measured top");
    }
    {
        Ledge ledge;ledge.height=300;Traversal t;require(lib.configureThreepeat(t.cfg),"runtime top calibration");t.cfg.approachSeconds=0;TransitionTrace trace(lib);
        require(t.attach(ledge,{0,-37,0},{0,1,0},100),"continuous rising top attach");
        Result r;
        for(int frame=0;frame<400&&t.active();++frame)r=trace.tick(ledge,t,{0,1,false,true});
        require(r.completed&&trace.saw(Motion::up,Motion::contextMantle),
            "held W reaches the lip from below and blends into the measured top-out");
        trace.report("upward climb through lip");
    }
}
static void progressingActionSources(const Library& lib) {

    const auto pushStart=lib.sample(Motion::kickUp,actionPushPhase(.02f));
    const auto pushEnd=lib.sample(Motion::kickUp,actionPushPhase(.12f));
    float pushLegChange=0;
    for(int bone:{6,7,9,10})pushLegChange=std::max(pushLegChange,angleBetween(pushStart[bone].q,pushEnd[bone].q));
    require(pushLegChange>.06f,"the takeoff interval contains changing captured leg poses, not one held anticipation key");
    float lastFlight=0,lastPush=0,lastReturn=0;
    for(int frame=0;frame<=120;++frame) {
        const float phase=frame/120.f,flight=actionFlightPhase(phase),push=actionPushPhase(phase);
        const float returning=actionReturnPhase(Motion::flipUp,phase);
        require(flight>=lastFlight&&flight<=1&&push>=lastPush&&push<=1&&returning>=lastReturn&&returning<=1,
            "takeoff flight and return sample phases advance monotonically");
        if(phase<=.70f)require(returning==0,"the return cannot replace the visible flight halfway through");
        lastFlight=flight;lastPush=push;lastReturn=returning;
    }
    require(actionFlightPhase(.30f)>.10f&&actionFlightPhase(.80f)>.999f&&lastReturn==1,
        "the whole flight capture advances before the short landing continuation");
    for(Motion motion:{Motion::kickUp,Motion::kickLeft,Motion::kickRight})
        require(actionReturnPhase(motion,.60f)==0&&actionReturnPhase(motion,1)==1,
            "each kick reserves an advancing push and ends in its selected catch");
    std::cout<<"takeoff captured leg change="<<pushLegChange<<'\n';
}
static void visibleBackPush(const Library& lib,const std::filesystem::path& directory) {
    Plane wall;
    for(int fps:{30,60})for(int startMode=0;startMode<4;++startMode) {
        const float dt=1.f/fps;Traversal t;t.cfg.approachSeconds=0;SurfacePose animator;
        require(t.attach(wall,{0,-37,1000},{0,1,0},100),"visible back-push attach");
        Pose pose;Result r;std::ofstream dump;int frameNumber=0;
        if(fps==60&&startMode<3) {
            dump.open(directory/("runtime-drop-"+std::to_string(startMode)+".json"));dump<<'[';
        }
        const Input moving=startMode==0?Input{}:startMode==3?Input{0,1}:startMode==2?Input{1,1,false,false,false,false,true}:
            Input{0,1,false,false,false,false,true};
        auto tick=[&](Input input) {
            r=t.update(wall,input,dt,100);pose=animator.update(lib,wall,t,r.motion,dt,1);
            for(const auto& tr:pose)require(tr.t.finite()&&std::isfinite(tr.q.dot(tr.q)),"a release pose remains finite through a running or inverted interruption");
            if(dump) {
                if(frameNumber)dump<<',';
                dump<<"{\"position\":["<<t.position.x<<','<<t.position.y<<','<<t.position.z<<"],\"phase\":"<<frameNumber*dt
                    <<",\"motion\":"<<int(r.motion)<<",\"actionProgress\":"<<t.actionProgress()<<",\"transforms\":[";
                for(std::size_t bone=0;bone<pose.size();++bone) {
                    const auto& tr=pose[bone];if(bone)dump<<',';
                    dump<<"{\"t\":["<<tr.t.x<<','<<tr.t.y<<','<<tr.t.z<<"],\"q\":["<<tr.q.x<<','<<tr.q.y<<','<<tr.q.z<<','<<tr.q.w<<"],\"s\":[1,1,1]}";
                }
                dump<<"]}";
            }
            ++frameNumber;
        };
        for(int frame=0;frame<fps;++frame)tick(moving);
        if(startMode==3) {
            auto jump=moving;jump.hop=true;tick(jump);
            require(r.motion==Motion::hopUp,"back-push interruption enters a real ordinary hop");
            while(t.actionProgress()<.43f&&t.active())tick(moving);
            require(t.state==State::action,"back push interrupts the active flight, not an already caught pose");
        }
        const Vec departure=t.position;const Pose outgoing=pose;
        Keys keys;keys.s=keys.space=true;tick(wallInput(keys,true));
        require(r.motion==Motion::dropBack&&t.active(),"S plus Space visibly begins the separate push capture");
        float visibleSeconds=dt,bestLateLegError=1000,terminalLegError=1000,visibleLegChange=0;int lateSamples=0;
        for(int frame=0;frame<fps&&r.motion==Motion::dropBack;++frame) {
            const auto source=lib.sample(Motion::dropBack,t.actionProgress());float legError=0;
            for(int bone:{6,7,9,10}) {
                legError=std::max(legError,angleBetween(source[bone].q,pose[bone].q));
                visibleLegChange=std::max(visibleLegChange,angleBetween(outgoing[bone].q,pose[bone].q));
            }
            require(animator.contactCount==0,"the departing foot push must not pin limbs to stale wall contacts");
            if(t.actionProgress()>=.75f) {bestLateLegError=std::min(bestLateLegError,legError);++lateSamples;}
            terminalLegError=legError;
            const auto held=animator.update(lib,wall,t,r.motion,0,1);
            for(std::size_t bone=0;bone<pose.size();++bone)
                require((held[bone].t-pose[bone].t).length()<.000001f&&angleBetween(held[bone].q,pose[bone].q)<.0001f,
                    "zero delta cannot skip or replay a release key");
            if(r.released)break;
            require(r.releaseVelocity.length()==0,"the visible push comes before the physical release impulse");
            tick({});visibleSeconds+=dt;
        }
        std::cout<<"visible back push fps="<<fps<<" from="<<startMode<<" seconds="<<visibleSeconds
            <<" lateLegError="<<bestLateLegError<<" terminalLegError="<<terminalLegError<<" legChange="<<visibleLegChange<<'\n';
        require(r.released&&!t.active()&&visibleSeconds>=.30f&&visibleSeconds<=.36f,
            "the full visible push completes before control returns to physical falling");
        require(lateSamples>=2&&bestLateLegError<.35f&&terminalLegError<.35f&&visibleLegChange>.20f,
            "late source hip and knee keys actually reach the displayed pose before the exit fade");
        const float travelTime=.32f/3.f+std::max(0.f,visibleSeconds-.32f);
        require(std::abs((t.position-departure).dot(t.normal)-220*travelTime)<.02f&&std::abs(t.position.z-departure.z-90*travelTime)<.02f&&r.releaseVelocity.dot(t.normal)>219,
            "the displayed push finishes at its checked departure before the one physical impulse");
        if(dump)dump<<']';
    }
}
int main(int argc,char** argv) {
 try {
    require(argc==2,"motion path argument");Library lib;require(lib.load(argv[1]),"motion load");
    auto same=blend(Quat{0,0,0,1},Quat{0,0,0,-1},.5f);require(std::abs(same.w)> .9999f,"quaternion antipodes must not spin");
    for(float slope:{-.1f,0.f,.6f})for(Vec direction:{Vec{0,1,0},Vec{-1,0,0},Vec{1,0,0},Vec{0,-1,0}}) {
        auto ground=lib.sample(Motion::up,.37f);
        auto run=nativeWallRun(ground,slope,37,1,direction);
        for(int i=1;i<99;++i)require(angleBetween(run[i].q,ground[i].q)<.002f&&(run[i].t-ground[i].t).length()<.001f,
            "wall run must preserve every native limb and finger local, including replacements");
        const Vec up=run[0].q.rotate({0,0,1});
        require((up-Vec{0,-std::sqrt(1-slope*slope),slope}).length()<.002f,"runner up aligns to actual surface normal");
        const Vec forward=run[0].q.rotate({0,1,0});
        require(std::abs(forward.dot(up))<.001f,"native run forward stays tangent to wall");
    }
    for(Motion entry:{Motion::reach,Motion::sprintCatch,Motion::jumpCatch,Motion::ledgeCatch}) {
        for(int frame=0;frame<=60;++frame) {
            auto entryWorld=lib.world(lib.sample(entry,frame/60.f));
            require((entryWorld[36].t-entryWorld[4].t).unit().z>.8f,"catch clip must not contain the prone crest of a top-out");
        }
    }
    for(int bone:{38,39,8,11}) {
        auto p=lib.sample(Motion::hang,.5f);const Quat local=p[bone].q;
        auto w=lib.world(p);
        lib.contactOrientation(p,bone,local,Quat::axis({1,0,0},2.6f)*w[bone].q);
        require(angleBetween(local,p[bone].q)<=.262f,"contact cannot twist a wrist or ankle past the 15-degree correction budget");
    }
    for(Motion top:{Motion::contextMantle}) {
        const auto w=lib.world(lib.sample(top,0));
        require((w[36].t-w[4].t).unit().z>.8f,"top-out begins at the grip, not at a crouched middle frame");
    }
    for(int i=1;i<=5;++i) {
        auto a=lib.world(lib.sample(Motion(i),0)),b=lib.world(lib.sample(Motion(i),1));
        for(std::size_t n=0;n<a.size();++n) require((a[n].t-b[n].t).length()<.001f,"seamless loop");
    }
    auto arm=lib.sample(Motion::hang,0);auto w=lib.world(arm);
    Vec target=w[28].t+Vec{-8,20,22};float error=lib.ik(arm,28,29,38,target,{-50,-10,90});
    require(error<.02f,"reachable hand solves without stretching");
    arm=lib.sample(Motion::hang,0);error=lib.ik(arm,28,29,38,{0,1000,1000},{-50,0,90});
    require(error>100,"unreachable target is bounded");
    w=lib.world(arm);require((w[38].t-w[28].t).length()<39,"arm cannot stretch");
    require(lib.ik(arm,28,29,38,w[28].t+Vec{0,20,0},w[28].t+Vec{0,40,0})<.02f,"degenerate pole remains finite");
    std::vector<std::byte> bytes(16+16*4+128*48);
    auto put=[&](std::size_t off,auto value){std::memcpy(bytes.data()+off,&value,sizeof(value));};
    put(0,std::int32_t(bytes.size()));put(4,std::int32_t(4));put(48,std::int16_t(128));put(50,std::int16_t(99));put(52,std::int16_t(80));put(54,std::int16_t(48));put(56,1.f);put(61,std::uint8_t(1));
    require(densePose(bytes.data()).count==99,"dense Havok pose view");
    put(60,std::uint8_t(2));require(!densePose(bytes.data()).data,"palette output rejected");put(60,std::uint8_t(0));
    put(50,std::int16_t(129));require(!densePose(bytes.data()).data,"oversized output rejected");put(50,std::int16_t(99));
    put(52,std::int16_t(32760));require(!densePose(bytes.data()).data,"out of buffer offset rejected");

    put(50,std::int16_t(126));put(52,std::int16_t(80));
    for(std::size_t i=0;i<128;++i) {
        const auto& rest=lib.rest[i%99];const auto offset=80+i*48;
        put(offset,rest.t);put(offset+12,1234.f);put(offset+16,rest.q);put(offset+32,rest.s);put(offset+44,4321.f);
    }
    const auto originalBytes=bytes;
    int playerCharacter{},npcCharacter{},firstPersonCharacter{};
    const std::array<const void*,1> owners{&playerCharacter};
    const auto authored=lib.sample(Motion::up,.3f);
    require(applyCharacterPose(&npcCharacter,owners,bytes.data(),authored,1)==PoseWrite::ignored&&bytes==originalBytes,"NPC output must remain unchanged");
    require(applyCharacterPose(&firstPersonCharacter,owners,bytes.data(),authored,1)==PoseWrite::ignored&&bytes==originalBytes,"first-person output must remain unchanged");
    require(applyCharacterPose(&playerCharacter,owners,bytes.data(),authored,0)==PoseWrite::ignored&&bytes==originalBytes,"inactive layer must not write");
    require(applyCharacterPose(&playerCharacter,owners,bytes.data(),authored,.5f)==PoseWrite::applied,"bound character child output must receive authored pose");
    for(std::size_t i=0;i<99;++i) {
        auto* transform=bytes.data()+80+i*48;
        require((field<Vec>(transform,0)-(lib.rest[i].t+authored[i].t)*.5f).length()<.001f,"native output must interpolate translation");
        require(std::abs(field<Quat>(transform,16).dot(field<Quat>(transform,16))-1)<.001f,"native output rotations remain normalized");
        require(field<float>(transform,12)==1234.f&&field<float>(transform,44)==4321.f,"Havok spare lanes are preserved");
    }
    require(std::equal(bytes.begin()+80+99*48,bytes.end(),originalBytes.begin()+80+99*48),"extra XPMSE bones stay byte identical");
    bytes=originalBytes;put(60,std::uint8_t(1));auto invalidBytes=bytes;
    require(applyCharacterPose(&playerCharacter,owners,bytes.data(),authored,1)==PoseWrite::invalid&&bytes==invalidBytes,"additive output must not be overwritten");
    bytes=originalBytes;put(80+98*48+16,Quat{0,0,0,0});invalidBytes=bytes;
    require(applyCharacterPose(&playerCharacter,owners,bytes.data(),authored,1)==PoseWrite::invalid&&bytes==invalidBytes,"invalid late bone must not leave a partial pose write");
    bytes=originalBytes;
    require(applyCharacterPose(&playerCharacter,owners,bytes.data(),authored,1)==PoseWrite::applied,"full-weight pose applies");
    for(std::size_t i=0;i<99;++i)require(std::abs(field<Quat>(bytes.data(),80+i*48+16).dot(authored[i].q))>.9999f,"full-weight output matches authored orientation");
    Plane wall;
    for(float slope:{0.f,.6f}) {
        wall.normal={0,-std::sqrt(1-slope*slope),slope};
        Traversal t;t.cfg.gap=37;t.cfg.radius=31;
        require(t.attach(wall,{0,-37,0},{0,1,0},100),"pose wall attach");
        SurfacePose animator;float maxError=0,steadyError=0,maxEndpointStep=0;Pose last,lastLocal;Vec lastPos;float plantedSlip=0;int locked=0,worstFrame=0,stepFrame=0,stepBone=0;
        std::ofstream dump(std::filesystem::current_path()/(slope==0?"runtime-wall.json":"runtime-slope.json"));dump<<'[';
        for(int frame=0;frame<240;++frame) {
            auto result=t.update(wall,{0,1,false,false},1.f/60,100);
            require(t.active(),"runtime wall remains attached");
            auto pose=animator.update(lib,wall,t,result.motion,1.f/60,1);
            if(!lastLocal.empty()) {
                for(int bone:{28,29,31,32,38,39})require(angleBetween(lastLocal[bone].q,pose[bone].q)<.21f,
                    "the complete pose angular budget applies equally to both arms and wrists");
                for(int bone=67;bone<=96;++bone)require(angleBetween(lastLocal[bone].q,pose[bone].q)<.21f,
                    "fingers share the whole-pose angular budget instead of accumulating a separate phase lag");
            }
            lastLocal=pose;
            w=lib.world(pose);if(animator.maxReachError>maxError){maxError=animator.maxReachError;worstFrame=frame;}if(frame>90)steadyError=std::max(steadyError,animator.maxReachError);
            if(!last.empty()) for(int bone:{38,39,8,11}) {
                float movement=(w[bone].t+t.position-last[bone].t-lastPos).length();
                if(movement>maxEndpointStep){maxEndpointStep=movement;stepFrame=frame;stepBone=bone;}
                if(movement<.02f)++locked;
            }
            last=w;lastPos=t.position;
            if(frame%2==0) {
                if(frame)dump<<',';
                dump<<"{\"position\":["<<t.position.x<<','<<t.position.y<<','<<t.position.z<<"],\"phase\":"<<frame/60.f<<",\"transforms\":[";
                for(std::size_t i=0;i<pose.size();++i) {
                    auto& tr=pose[i];if(i)dump<<',';
                    dump<<"{\"t\":["<<tr.t.x<<','<<tr.t.y<<','<<tr.t.z<<"],\"q\":["<<tr.q.x<<','<<tr.q.y<<','<<tr.q.z<<','<<tr.q.w<<"],\"s\":[1,1,1]}";
                }
                dump<<"]}";
            }
        }
        dump<<']';
        std::cout<<"slope="<<slope<<" maxReachError="<<maxError<<" maxEndpointStep="<<maxEndpointStep<<" stepFrame="<<stepFrame<<" stepBone="<<stepBone<<" lockedSamples="<<locked<<" worstFrame="<<worstFrame<<" steadyError="<<steadyError<<'\n';

        Pose held,heldLocal;Vec heldPosition;
        for(int frame=0;frame<90;++frame) {
            auto result=t.update(wall,{},1.f/60,100);
            auto p=animator.update(lib,wall,t,result.motion,1.f/60,1);auto current=lib.world(p);
            if(frame>60) {
                if(frame==61)std::cout<<"held slope="<<slope<<" contacts="<<animator.contactCount<<" palmDistances="<<(current[38].t+t.position).dot(wall.normal)<<','<<(current[39].t+t.position).dot(wall.normal)<<" armReach="<<(current[38].t-current[28].t).length()<<','<<(current[39].t-current[31].t).length()<<'\n';
                require(animator.contactCount>=2,"stationary hang must retain real surface contacts");
                for(int finger=67;finger<=96;++finger)
                    require(angleBetween(p[finger].q,heldLocal[finger].q)<.002f,"holding a captured gait frame preserves every finger joint");
                for(int bone:{38,39})require((current[bone].t+t.position-held[bone].t-heldPosition).length()<.1f,"hanging palms must stay planted");
            }
            held=current;heldLocal=p;heldPosition=t.position;
        }
        require(maxError<2.f&&steadyError<.1f,"entry contact tolerance is 2 units; steady contacts must remain locked");
        require(maxEndpointStep<10.1f,"contact transfers stay within the ordinary pose endpoint budget");
    }
    {
        Plane wall;Traversal t;t.cfg.gap=37;t.cfg.radius=31;
        require(t.attach(wall,{0,-37,200},{0,1,0},100),"direction transition attach");
        SurfacePose animator;float worst=0;Pose previous;Vec previousPosition;float step=0;
        for(int frame=0;frame<660;++frame) {
            Input input=frame<180?Input{0,1}:frame<210?Input{}:frame<360?Input{1,0}:frame<510?Input{0,-1}:Input{-1,0};
            input.x*=animator.movementScale();input.y*=animator.movementScale();
            auto result=t.update(wall,input,1.f/60,100);auto pose=animator.update(lib,wall,t,result.motion,1.f/60,1);
            auto w=lib.world(pose);worst=std::max(worst,animator.maxReachError);
            if(!previous.empty())for(int bone:{38,39,8,11})step=std::max(step,(w[bone].t+t.position-previous[bone].t-previousPosition).length());
            previous=w;previousPosition=t.position;
            for(auto& tr:pose)require(tr.t.finite()&&std::isfinite(tr.q.dot(tr.q)),"direction changes stay finite");
            if(frame==359)require(t.position.x>40,"contact correction must still allow side travel");
        }
        std::cout<<"direction transitions reachError="<<worst<<" maxEndpointStep="<<step<<'\n';
        require(worst<3&&step<12,"direction changes regrip before overstretching");
    }
    for(float height:{45.f,80.f,120.f}) {
        Ledge ledge;ledge.height=height;Traversal t;require(lib.configureThreepeat(t.cfg),"runtime top calibration");t.cfg.gap=37;t.cfg.radius=31;
        require(t.attach(ledge,{0,-42,0},{0,1,0},100),"mantle pose attach");
        std::ofstream dump(std::filesystem::current_path()/("runtime-top-"+std::to_string(int(height))+".json"));dump<<'[';
        SurfacePose animator;PoseHandoff renderer;const Pose native=lib.rest;Pose visible;
        float maxError=0,errorPhase=0,lipError=0,footPenetration=0,previousRecovery=0;int lipSamples=0;bool completed=false;float largestStep=0;Pose previous;Vec previousPosition;
        for(int frame=0;frame<240&&t.active();++frame) {
            auto result=t.update(ledge,{0,1,false,true},1.f/60,100);
            auto pose=animator.update(lib,ledge,t,result.motion,1.f/60,1);auto w=lib.world(pose);
            const float recovery=topRecovery(result.motion,t.progress(),t.topSeconds());
            if(result.motion==Motion::contextMantle) {
                require(recovery>=previousRecovery,"top-out recovery cannot reverse toward the captured animation");
                if(t.progress()<.72f)require(recovery==0,"loaded-hand phase must retain the complete authored pose");
                previousRecovery=recovery;
            }
            visible=renderer.compose(native,pose,1,recovery);
            for(int foot:{8,11}) {
                const auto point=w[foot].t+t.position;
                if(point.y>0)footPenetration=std::max(footPenetration,height-point.z);
            }
            if(animator.maxReachError>maxError){maxError=animator.maxReachError;errorPhase=t.progress();}
            if(!previous.empty())for(int bone:{38,39,8,11}) {
                const float step=(w[bone].t+t.position-previous[bone].t-previousPosition).length();
                if(step>12&&step>largestStep)std::cout<<"top step height="<<height<<" phase="<<t.progress()<<" bone="<<bone<<" distance="<<step<<'\n';
                largestStep=std::max(largestStep,step);
            }
            previous=w;previousPosition=t.position;
            completed=result.completed;
            if(t.state==State::mantle&&t.progress()>.23f&&t.progress()<.55f) {
                const float sampled=t.topSamplePhase(t.progress());
                for(int hand=0;hand<2;++hand)if(t.topHandWeight(hand,sampled)>.95f) {
                    Vec anchor=t.topHand(hand)+t.topHandNormal(hand)*.8f;
                    if(t.preciseTopContacts()&&t.topReplanted(sampled)) {
                        const auto actual=threepeatReplantContact(ledge,t.topHand(hand),t.normal,t.topReplantOffset(hand),1);
                        require(actual.has_value(),"loaded captured replant retains its actual collision contact");anchor=*actual;
                    }
                    const Vec difference=lib.palm(lib.world(visible),hand)+t.position-anchor;
                    lipError=std::max(lipError,difference.length());
                    ++lipSamples;
                }
            }
            if(frame%2==0){
                if(frame)dump<<',';
                dump<<"{\"position\":["<<t.position.x<<','<<t.position.y<<','<<t.position.z<<"],\"phase\":"<<frame/60.f<<",\"transforms\":[";
                for(std::size_t i=0;i<pose.size();++i) {
                    auto& tr=visible[i];if(i)dump<<',';
                    dump<<"{\"t\":["<<tr.t.x<<','<<tr.t.y<<','<<tr.t.z<<"],\"q\":["<<tr.q.x<<','<<tr.q.y<<','<<tr.q.z<<','<<tr.q.w<<"],\"s\":[1,1,1]}";
                }
                dump<<"]}";
            }
        }
        dump<<']';
        std::cout<<"ledge="<<height<<" reachError="<<maxError<<" atPhase="<<errorPhase<<" maxEndpointStep="<<largestStep<<" lipError="<<lipError<<" lipSamples="<<lipSamples<<'\n';
        require(completed&&t.position.z>=height,"all authored mantle heights reach the platform");
        require(maxError<5,"mantle targets should remain reachable during blended release");
        require(largestStep<12.6f,"mantle transition endpoints remain within the complete-pose velocity budget");
        require(footPenetration<3,"feet must clear the platform before crossing its edge");
        const bool loadedTop=t.topHandWeight(0,t.topSampleBegin())>.05f||t.topHandWeight(1,t.topSampleBegin())>.05f;
        require((loadedTop?lipSamples>0:lipSamples==0)&&lipError<3,"only genuinely loaded palms contact actual collision tops; low tops do not invent hand support");
        require(previousRecovery>.99999f,"a completed top-out must fully hand over the pose rather than merely stop its timeline");
        for(std::size_t bone=0;bone<native.size();++bone) {

            const float translation=(visible[bone].t-native[bone].t).length(),angle=angleBetween(visible[bone].q.unit(),native[bone].q.unit());
            if(translation>=.0001f||angle>=.002f) {
                const auto actual=visible[bone].q,target=native[bone].q;
                std::cerr<<"terminal native mismatch height="<<height<<" bone="<<bone<<" name="<<lib.names[bone]
                    <<" recovery="<<previousRecovery<<" translation="<<translation<<" angle="<<angle
                    <<" rawAngle="<<angleBetween(actual,target)
                    <<" actualNorm2="<<actual.dot(actual)<<" targetNorm2="<<target.dot(target)
                    <<" actualQ="<<actual.x<<','<<actual.y<<','<<actual.z<<','<<actual.w
                    <<" targetQ="<<target.x<<','<<target.y<<','<<target.z<<','<<target.w<<'\n';
            }
            require(translation<.0001f&&angle<.002f,"all terminal bones, including COM and legs, reach the native output without residual local offsets");
        }
    }
    progressingActionSources(lib);
    visibleBackPush(lib,std::filesystem::current_path());
    reachableActionTransitions(lib);

    auto temp=std::filesystem::current_path()/"FreeClimb-test-invalid.motion";
    {std::ofstream f(temp,std::ios::binary);f<<"FCM3";}
    Library bad;require(!bad.load(temp.string()),"truncated library rejected");std::filesystem::remove(temp);
    std::cout<<"PASS: pose sampling, quaternion seams, bounded IK, child-character output routing, native pose writes and extra-bone preservation, wall/slope contacts, corrupt input\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
