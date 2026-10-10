#include "TraversalCapture.h"
#include "CornerTestWorld.h"
#include <iostream>
#include <fstream>
#include <memory>
#include <stdexcept>
using namespace fc;
static void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
static std::string withoutRecovery(std::string text,bool omit=false) {
    for(const auto label:{"BEFORE ","AFTER "}) {
        const auto start=text.find(label),reasons=text.find(" \"",start);auto recovery=reasons;
        for(unsigned i=0;i<9;++i)recovery=text.rfind(' ',recovery-1);
        text.replace(recovery,reasons-recovery,omit?"":" 0 0 0 0 0 0 0 0 0");
    }
    return text;
}
static std::string priorCapture(std::string text,std::string_view version,bool retiredArc=false) {
    text=withoutRecovery(std::move(text),true);
    text.replace(text.find(TraversalCapture::coreVersion),TraversalCapture::coreVersion.size(),version);
    if(version=="active31-5")return text;
    for(const auto label:{"BEFORE ","AFTER "}) {
        const auto start=text.find(label);auto reasons=text.find(" \"",start);
        auto holds=reasons;
        for(unsigned i=0;i<3;++i)holds=text.rfind(' ',holds-1);
        check(text.substr(holds,reasons-holds)==" 0 0 0","prior snapshot fixture has no traversal hold");
        text.erase(holds,reasons-holds);reasons=holds;
        if(version=="active31-4")continue;
        const auto context=text.rfind(' ',reasons-1);
        check(text.substr(context,reasons-context)==" 0","prior snapshot fixture has no directional side-hop references");
        text.erase(context,reasons-context);reasons=context;
        if(version=="active31-3")continue;
        auto route=reasons;
        for(unsigned i=0;i<261;++i)route=text.rfind(' ',route-1);
        text.erase(route,reasons-route);reasons=route;
        if(version=="active31-2")continue;
        const auto present=text.rfind(' ',reasons-1),direction=text.rfind(' ',present-1);
        check(text.substr(direction,reasons-direction)==" 0 0","prior snapshot fixture has no direction variant metadata");
        text.erase(direction,reasons-direction);
        if(version!="active31-1") {
            std::istringstream row(text.substr(start));std::string token;row>>token;unsigned fields=118;
            for(unsigned index=0;index<fields;++index) {
                check(bool(row>>token),"prior snapshot fixture contains every field preceding its removed arc");
                if(index==34&&token=="1")fields+=7;
            }
            text.insert(start+std::size_t(row.tellg()),retiredArc?" 1":" 0");
        }
        if(version=="active35-9") {
            const auto reasons=text.find(" \"",start),setting=text.rfind(' ',reasons-1);
            check(text.substr(setting,reasons-setting)==" 1","oldest supported snapshot fixture uses enabled wall running");
            text.erase(setting,reasons-setting);
        }
    }
    return text;
}

struct EdgeWall:World {
    Vec origin{109000.125f,72000.0625f,1800.375f};
    unsigned calls{},bodyCalls{};
    std::optional<Hit> ray(Vec a,Vec b)override {
        ++calls;const auto p=a-origin,d=b-a;
        if(p.y>=0||d.y<=0||p.y+d.y<0)return {};
        const float fraction=-p.y/d.y;const auto hit=a+d*fraction;
        if(hit.x-origin.x>4.f)return {};
        return Hit{hit,{0,-1,0},true};
    }
    bool actionBodyClear(Motion motion,Vec,Vec,float a,float b,Vec)override {
        ++bodyCalls;return motion==Motion::backFlipOut&&a<b;
    }
};
static Traversal stalledTraversal(EdgeWall& world,float dt=1.f/48) {
    Traversal t;t.cfg.radius=31;t.cfg.gap=37;t.cfg.height=138;t.cfg.approachSeconds=0;
    check(t.attach(world,world.origin+Vec{0,-37,200},{0,1,0},1000,35),"real wall attachment");
    for(int frame=0;frame<100&&t.stalledSeconds()<=.4f;++frame)t.update(world,{1,0},dt,1000);
    check(t.active()&&t.stalledSeconds()>.35f,"finite wall edge creates a real supported movement stall");
    return t;
}
static void snapshotRoundTrip() {
    EdgeWall world;auto t=stalledTraversal(world);
    auto capture=std::make_unique<TraversalCapture>();auto decoded=std::make_unique<TraversalCapture>();
    const Vec stopped=t.position;const auto sourceCalls=world.calls;
    capture->begin(t,{1,0},1.f/48,997.125f);TraversalCapture::RecordingWorld recorder(world,*capture);
    const auto result=t.update(recorder,{1,0},1.f/48,997.125f);capture->finish(t,result);
    check(capture->complete()&&capture->count()>50,"blocked update has a bounded complete World tape");
    check(capture->count()==world.calls-sourceCalls,"recording adds no World queries");
    check((t.position-stopped).length()<.01f,"fixture remains blocked");
    auto direct=capture->replay();if(!direct.matched)std::cerr<<direct.error<<'\n';
    check(direct.matched&&direct.callsConsumed==capture->count(),"private state supports exact in-memory blocked replay");
    const auto encoded=capture->serialize();std::string error;
    check(encoded.size()<TraversalCapture::maxTextBytes,"serialized frame remains bounded");
    check(decoded->deserialize("[2026-09-28] [FreeClimb] [info] "+encoded,error),"ordinary logger first-line prefix accepted");
    check(decoded->serialize()==encoded,"max_digits10 round trip retains all far-origin floats and private fields");
    const auto replay=decoded->replay();if(!replay.matched)std::cerr<<replay.error<<'\n';
    check(replay.matched&&replay.callsConsumed==capture->count(),"serialized failure reproduces every query and final private state");
    std::cout<<"blocked tape: rays="<<capture->count()<<" bytes="<<encoded.size()<<" reason="<<t.blockedReason<<'\n';
    EdgeWall priorWorld;Traversal priorTraversal;priorTraversal.cfg.approachSeconds=0;
    check(priorTraversal.attach(priorWorld,priorWorld.origin+Vec{-1000,-37,200},{0,1,0},1000),"prior-format fixture attaches to a supported finite wall");
    auto priorFrame=std::make_unique<TraversalCapture>();TraversalCapture::RecordingWorld priorRecorder(priorWorld,*priorFrame);
    priorFrame->begin(priorTraversal,{-1,0},1.f/48,1000);
    const auto priorResult=priorTraversal.update(priorRecorder,{-1,0},1.f/48,1000);priorFrame->finish(priorTraversal,priorResult);
    const auto priorEncoded=priorFrame->serialize();
    for(const auto version:{"active35-9","active35-10","active31-1","active31-2","active31-3","active31-4","active31-5"}) {
        const auto prior=priorCapture(priorEncoded,version);
        check(decoded->deserialize(prior,error)&&decoded->before().cfg.wallRunEnabled&&decoded->after().cfg.wallRunEnabled,"prior version ordinary snapshots retain enabled wall running");
        const auto report=decoded->replay();
        if(!report.matched)std::cerr<<version<<": "<<report.error<<'\n';
        check(report.matched&&decoded->serialize()==withoutRecovery(priorEncoded),"prior supported movement retains queries and state while defaulting absent recovery fields");
        if(std::string_view(version).starts_with("active35"))check(!decoded->deserialize(priorCapture(priorEncoded,version,true),error)&&error.find("retired")!=std::string::npos,"prior snapshot with an active removed arc is rejected without changing its meaning");
        for(int id:{17,30,31,32}) {
            auto removed=prior;const auto resultStart=removed.find("RESULT ")+7;
            const auto motion=removed.find(' ',resultStart)+1,end=removed.find(' ',motion);
            removed.replace(motion,end-motion,std::to_string(id));
            check(!decoded->deserialize(removed,error)&&error.find("retired")!=std::string::npos,"prior sprint and flip outputs cannot be reinterpreted as retained actions");
        }
    }

    capture->begin(t,{-1,0},1.f/48,996);const auto moved=t.update(recorder,{-1,0},1.f/48,996);capture->finish(t,moved);
    check((t.position-stopped).length()>1&&t.stalledSeconds()==0,"opposite direction restores real movement");
    check(decoded->deserialize(capture->serialize(),error)&&decoded->replay().matched,"successful frame replays exactly");

    TraversalCapture::ReplayWorld wrong(*decoded);bool rejected=false;
    try{wrong.ray({1,2,3},{4,5,6});}catch(const std::runtime_error&){rejected=true;}
    check(rejected&&wrong.consumed()==0,"changed query is rejected at the first divergence");

    auto changed=encoded;const auto version=changed.find(TraversalCapture::coreVersion);
    changed.replace(version,TraversalCapture::coreVersion.size(),"0.2.0");
    check(!decoded->deserialize(changed,error)&&error.find("version")!=std::string::npos,
        "old capture layout with removed support state is explicitly refused");
    for(int id:{6,7,12,13,14,17,30,31,32,33,38}) {
        changed=encoded;const auto resultStart=changed.find("RESULT ")+7;
        const auto motion=changed.find(' ',resultStart)+1,end=changed.find(' ',motion);
        changed.replace(motion,end-motion,std::to_string(id));
        check(!decoded->deserialize(changed,error)&&error.find("retired")!=std::string::npos,
            "new captures cannot deserialize any retired motion output");
    }
    check(!decoded->deserialize(encoded.substr(0,encoded.find("FCGEO_END")),error),"truncated log block refused");
    changed=encoded;changed.insert(changed.find("FCGEO_END")+9,"garbage");
    check(!decoded->deserialize(changed,error),"partial end marker refused");
    changed=encoded;const auto meta=changed.find("META "),line=changed.find('\n',meta);
    const auto oversized=std::to_string(TraversalCapture::capacity+1);
    changed.replace(meta,line-meta,"META "+oversized+" "+oversized+" 1");
    check(!decoded->deserialize(changed,error)&&error.find("call count")!=std::string::npos,
        "oversized declared capacity refused before writing storage");
    changed=encoded;const auto input=changed.find("INPUT ");changed.replace(input,changed.find('\n',input)-input,"INPUT nan 0 0 0 0 0 0 -1 .02 1000");
    check(!decoded->deserialize(changed,error),"nonfinite input refused");
}
static void limitsAndWorldCompleteness() {
    EdgeWall world;auto t=stalledTraversal(world);
    auto capture=std::make_unique<TraversalCapture>();auto decoded=std::make_unique<TraversalCapture>();
    capture->begin(t,{},0,1000);TraversalCapture::RecordingWorld recorder(world,*capture);
    const Vec a=world.origin+Vec{0,-20,10},b=world.origin+Vec{0,10,10};
    check(recorder.ray(a,b).has_value(),"hit stored including climbable property");
    check(!recorder.ray(a,a+Vec{0,-10,0}),"miss stored explicitly");
    check(recorder.actionBodyClear(Motion::backFlipOut,a,b,.25f,.5f,{0,-1,0}),"World body test forwards result");
    check(!recorder.actionBodyClear(Motion::up,a,b,.25f,.5f,{0,-1,0}),"false body result stored");
    capture->finish(t,{});std::string error;check(decoded->deserialize(capture->serialize(),error),"mixed World methods deserialize");
    TraversalCapture::ReplayWorld replay(*decoded);
    check(replay.ray(a,b).has_value()&&!replay.ray(a,a+Vec{0,-10,0}),"hit and miss preserve query order");
    check(replay.actionBodyClear(Motion::backFlipOut,a,b,.25f,.5f,{0,-1,0})&&
        !replay.actionBodyClear(Motion::up,a,b,.25f,.5f,{0,-1,0})&&replay.consumed()==4,"body calls retain phases, outward direction, motion and result");
    TraversalCapture::ReplayWorld changed(*decoded);changed.ray(a,b);changed.ray(a,a+Vec{0,-10,0});bool rejected=false;
    try{changed.actionBodyClear(Motion::backFlipOut,a,b,.24f,.5f,{0,-1,0});}catch(const std::runtime_error&){rejected=true;}
    check(rejected,"body phase mismatch cannot masquerade as valid collision evidence");
    check(!decoded->replay().matched,"extra unused calls invalidate a frame replay");

    capture->begin(t,{},0,1000);
    for(std::size_t i=0;i<TraversalCapture::capacity+17;++i)recorder.ray(a,b);
    capture->finish(t,{});
    check(capture->count()==TraversalCapture::capacity&&capture->observed()==TraversalCapture::capacity+17&&!capture->complete(),"overflow preserves bounds and counts truncation");
    check(decoded->deserialize(capture->serialize(),error)&&!decoded->complete()&&!decoded->replay().matched,"incomplete tape stays diagnostic-only after reload");
    std::cout<<"bounded storage="<<sizeof(TraversalCapture)<<" bytes, full-hit log="<<capture->serialize().size()<<" bytes\n";
}
static void sessionBudget() {
    EdgeWall world;auto t=stalledTraversal(world);TraversalCapture::SessionGate gate;
    auto arm=[&](const Traversal& state,Input input,float elapsed) {
        return gate.arm(state,input,elapsed,.025f)&&gate.commit(true);
    };
    const Vec upwardStop{-692.019043f,191.0042267f,799.2728271f},laterStop{-713.58f,162.73f,799.27f};
    t.position=upwardStop;
    check(!arm(t,{},1)&&!arm(t,{.05f,.05f},1),"idle hanging and deadzone input never arm capture");
    check(!arm(t,{0,1,true},1)&&!arm(t,{0,-1,false,false,false,true},1),"release and backward drop do not spend capture budget");
    Traversal unstalled;unstalled.state=State::wall;unstalled.position=upwardStop;
    check(!arm(unstalled,{0,1},1),"active traversal without a sustained stall does not arm capture");
    const float nan=std::numeric_limits<float>::quiet_NaN(),infinity=std::numeric_limits<float>::infinity();
    for(const float elapsed:{nan,infinity,-infinity,-1.f})check(!arm(t,{0,1},elapsed),"nonfinite or negative attachment time does not arm capture");
    for(const Input input:{Input{nan,1},Input{1,nan},Input{infinity,1},Input{1,-infinity}})
        check(!arm(t,input,1),"nonfinite input does not arm capture");
    for(const Vec position:{Vec{nan,0,0},Vec{0,infinity,0},Vec{0,0,-infinity}}) {
        t.position=position;check(!arm(t,{0,1},1),"nonfinite player position does not arm capture");
    }
    check(gate.used()==0,"invalid and idle requests preserve the full budget");
    t.position=upwardStop;
    check(arm(t,{0,1},4.106f)&&gate.used()==1,"15:44:04.868 upward stall arms after the 15:44:00.762 attachment");
    t.position=laterStop;
    check((laterStop-upwardStop).length()>35&&(laterStop-upwardStop).length()<36,"logged second stop moved only about 36 units");
    check(!arm(t,{-1,0},5.100f),"15:44:05.862 later left stall remains below the one-second interval");
    check(arm(t,{-1,0},5.125f)&&gate.used()==2,"next 25ms frame can capture the nearby second stop");
    check(!arm(t,{-1,-1},6.123f),"15:44:06.885 down-left change also respects the interval");
    check(arm(t,{-1,-1},6.148f)&&gate.used()==3,"next frame captures down-left at the same stop");
    t.position.x+=120;
    check(!arm(t,{1,0},20)&&gate.used()==3,"three short stalls preserve the final diagnostic slot for a sustained obstruction");
    auto persistent=stalledTraversal(world);
    for(unsigned frame=0;frame<100&&persistent.stalledSeconds()<1.5f;++frame)persistent.update(world,{1,0},1.f/48,1000);
    check(persistent.active()&&persistent.stalledSeconds()>=1.5f,"the final capture uses a real sustained movement obstruction");
    persistent.position=t.position;
    check(arm(persistent,{1,0},21)&&gate.used()==4,"a later persistent stop is captured after the three earlier brief obstructions");
    persistent.position.x+=120;
    check(!arm(persistent,{-1,0},25)&&gate.used()==4,"the reserved capture keeps a strict four-block per attachment limit");

    gate.reset();t.position=upwardStop;
    check(arm(t,{0,1},4.106f),"reset starts an independent attachment budget");
    t.position=laterStop;
    check(arm(t,{0,1},5.106f),"36-unit displacement alone can capture after exactly one second");
    check(!arm(t,{0,.3f},8),"same direction at a different analog magnitude cannot repeat a capture");
    check(!arm(t,{.05f,1},9),"small analog direction jitter cannot repeat a capture");
    t.position=upwardStop;
    check(!arm(t,{0,1},10),"return to an earlier captured position and direction cannot repeat it");
    check(arm(t,{-1,0},11)&&gate.used()==3,"a significant direction change at an earlier position remains eligible");

    gate.reset();t.position=laterStop;
    check(arm(t,{-1,0},1)&&arm(t,{-1,-1},2),"one position permits a distinct diagonal query after exactly one second");
    check(!arm(t,{-1,0},3)&&!arm(t,{-1,-1},4)&&gate.used()==2,"alternating previously captured directions cannot spam the same position");
    check(!arm(t,{1,0},1.5f),"a rewound attachment clock cannot bypass the interval");
    check(arm(t,{std::numeric_limits<float>::max(),0},5)&&gate.used()==3,"large finite input uses the runtime-clamped direction without overflow");
    gate.reset();check(arm(t,{1,0},.5f)&&gate.used()==1,"new attachment explicitly resets capture budget and time");
    gate.reset();t.stop();check(!arm(t,{1,0},10)&&gate.used()==0,"released traversal cannot arm");
}
static void captureCommitBudget() {
    EdgeWall world;auto t=stalledTraversal(world);TraversalCapture::SessionGate gate;
    const float nan=std::numeric_limits<float>::quiet_NaN(),infinity=std::numeric_limits<float>::infinity();
    for(const float dt:{nan,infinity,-infinity,-1.f,0.f,1e-7f})
        check(!gate.arm(t,{0,1},2,dt)&&gate.used()==0,"invalid or paused frame time cannot reserve a capture");
    check(!gate.commit(true),"commit without an armed frame cannot spend a slot");
    check(gate.arm(t,{0,1},2,.025f)&&gate.used()==0,"arming does not consume or deduplicate a provisional capture");
    check(!gate.commit(false)&&gate.used()==0,"a frame without actual World queries does not consume the budget");
    check(gate.arm(t,{0,1},2,.025f)&&gate.commit(true)&&gate.used()==1,"the same immediate candidate remains available after an empty frame");
    check(!gate.commit(true)&&gate.used()==1,"a committed capture cannot be committed twice");
    check(!gate.arm(t,{1,0},2.9f,.025f),"only an observed frame advances the one-second interval");
    check(gate.arm(t,{1,0},3,.025f),"a changed direction can arm after the observed frame interval");
    check(!gate.arm(t,{1,0},3,0)&&!gate.commit(true)&&gate.used()==1,"a paused frame invalidates a pending unrecorded candidate");
    gate.reset();check(!gate.commit(true)&&gate.used()==0,"reset clears pending state and all committed quotas");
}
struct CaptureHoldWall:EdgeWall {
    bool visible=true;
    std::optional<Hit> ray(Vec a,Vec b)override {
        if(visible)return EdgeWall::ray(a,b);
        ++calls;return {};
    }
};
static Traversal capturedHold(CaptureHoldWall& world) {
    Traversal t;t.cfg.approachSeconds=0;
    check(t.attach(world,world.origin+Vec{-1000,-37,200},{0,1,0},1000),"geometry capture fixture starts on a valid wall");
    world.visible=false;t.update(world,{0,1},.025f,1000);
    check(t.geometryHolding()&&t.stalledSeconds()<.35f,"lost source support holds before the ordinary stall counter reaches its gate");
    return t;
}
static void heldCaptureBudget() {
    CaptureHoldWall world;auto t=capturedHold(world);TraversalCapture::SessionGate gate;
    auto capture=std::make_unique<TraversalCapture>();auto decoded=std::make_unique<TraversalCapture>();
    TraversalCapture::RecordingWorld recorded(world,*capture);
    float elapsed=10;
    for(const float dt:{0.f,-1.f,std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity(),1e-7f})
        check(!gate.arm(t,{0,1},elapsed,dt)&&gate.used()==0,"paused and invalid hold updates do not consume capture quota");
    check(!gate.arm(t,{0,1},elapsed,1.f),"long frame time obeys Core's 50ms clamp instead of prematurely draining a 150ms retry");
    unsigned captured=0,cooldown=0;
    for(unsigned frame=0;frame<80;++frame) {
        elapsed+=.025f;const bool armed=gate.arm(t,{0,1},elapsed,.025f);const auto calls=world.calls;
        if(armed)capture->begin(t,{0,1},.025f,1000);
        const auto result=t.update(armed?static_cast<World&>(recorded):static_cast<World&>(world),{0,1},.025f,1000);
        if(armed) {
            capture->finish(t,result);
            check(world.calls>calls&&capture->observed()>0,"held frame is captured only on a real geometry retry");
            check(gate.commit(capture->observed()>0),"queried held frame commits its reserved slot");
            const auto replay=capture->replay();check(replay.matched,"held retry records an exact Core replay without additional queries");
            const auto encoded=capture->serialize();std::string error;
            check(decoded->deserialize(encoded,error)&&decoded->serialize()==encoded&&decoded->replay().matched,
                "held retry preserves recovery cooldown, origin, direction and search cursor through disk serialization");
            check(decoded->deserialize(priorCapture(encoded,"active31-5"),error)&&decoded->before().geometryHolding()&&
                decoded->serialize()==withoutRecovery(encoded),"active31-5 preserves hold state while safely defaulting absent recovery state");
            ++captured;
        } else if(world.calls==calls) {
            ++cooldown;check(!gate.commit(false),"zero-query cooldown cannot commit a new capture");
        }
    }
    check(captured==1&&gate.used()==1&&cooldown>20,"same held position and direction is captured once while cooldown frames preserve the budget");
    t.update(world,{0,1},.025f,0);
    check(t.resting(),"geometry hold can coexist with exhausted stamina");
    for(unsigned frame=0;frame<100;++frame)
        check(!gate.arm(t,{1,0},elapsed+2+frame*.025f,.025f)&&gate.used()==1,"resting does not capture repeated zero-query frames");
    CaptureHoldWall reserveWorld;auto reserved=capturedHold(reserveWorld);gate.reset();
    EdgeWall ordinaryWorld;auto ordinary=stalledTraversal(ordinaryWorld);
    for(unsigned i=0;i<3;++i) {
        ordinary.position.x+=40;
        check(gate.arm(ordinary,{1,0},float(i+1),.025f)&&gate.commit(true),"three ordinary stalls leave one diagnostic slot reserved");
    }
    elapsed=10;float committedAt=0;
    for(unsigned frame=0;frame<90;++frame) {
        elapsed+=.025f;const bool armed=gate.arm(reserved,{0,1},elapsed,.025f);const auto calls=reserveWorld.calls;
        reserved.update(reserveWorld,{0,1},.025f,1000);
        if(armed) {
            check(reserveWorld.calls>calls&&gate.commit(true),"reserved slot captures a real persistent hold retry");
            committedAt=elapsed-10;break;
        }
    }
    check(committedAt>=1.49f&&committedAt<2&&gate.used()==4,"persistent source loss reaches the fourth slot even when ordinary stalled time stays zero");
    check(!gate.arm(reserved,{-1,0},20,.025f),"held retries retain the strict four-capture session limit");
    CaptureHoldWall idleWorld;auto idle=capturedHold(idleWorld);gate.reset();unsigned idleCaptured=0;
    elapsed=0;
    for(unsigned frame=0;frame<120;++frame) {
        elapsed+=.025f;const Input input=frame%2?Input{}:Input{.03f,-.03f};
        const bool armed=gate.arm(idle,input,elapsed,.025f);const auto calls=idleWorld.calls;
        idle.update(idleWorld,input,.025f,1000);
        if(armed) {
            check(idleWorld.calls>calls&&gate.commit(true),"neutral held retry captures actual geometry queries");
            ++idleCaptured;
        }
    }
    check(idleCaptured==1&&gate.used()==1,"neutral and deadzone holding share one independently deduplicated direction");
}
struct TopWall:World {
    std::optional<Hit> ray(Vec a,Vec b)override {
        const auto d=b-a;float nearest=2;std::optional<Hit> result;
        if(a.y<0&&b.y>=0){const float f=-a.y/d.y;const auto p=a+d*f;if(p.z<=240){nearest=f;result=Hit{p,{0,-1,0},true};}}
        if(a.z>240&&b.z<=240){const float f=(240-a.z)/d.z;const auto p=a+d*f;if(p.y>=0&&f<nearest)result=Hit{p,{0,0,1},true};}
        return result;
    }
};
struct CaptureRecoveryWorld:World {
    fc_test::CornerWorld source,target;
    bool sourceVisible=true;
    unsigned calls{};
    CaptureRecoveryWorld(){source.boxes={{{-10000,0,-10000},{10000,10000,10000}}};}
    std::optional<Hit> ray(Vec a,Vec b)override {
        ++calls;auto hit=sourceVisible?source.ray(a,b):std::optional<Hit>{};
        if(auto next=target.ray(a,b))if(!hit||(next->point-a).length()<(hit->point-a).length())hit=next;
        return hit;
    }
};
static void supportRecoveryReplay() {
    unsigned frames=0;
    for(int direction=0;direction<4;++direction) {
        CaptureRecoveryWorld world;Traversal t;t.cfg=fc_test::settings();t.cfg.contextActions=false;
        t.cfg.automaticClimbActions=false;t.cfg.wallRunObstacleJumps=false;t.cfg.fancyJumps=false;
        check(t.attach(world,{0,-42,200},{0,1,0},1000),"recovery capture starts from a real supported attachment");
        world.sourceVisible=false;Input input{};
        if(direction==0){input.y=1;world.target.boxes={{{-10000,0,350},{10000,10000,10000}}};}
        else if(direction==1){input.x=1;world.target.boxes={{{100,0,-10000},{10000,10000,10000}}};}
        else if(direction==2){input.x=-1;world.target.boxes={{{-10000,0,-10000},{-100,10000,10000}}};}
        else {input.y=-1;world.target.boxes={{{-10000,0,-10000},{10000,10000,224}}};}
        auto capture=std::make_unique<TraversalCapture>(),decoded=std::make_unique<TraversalCapture>();
        TraversalCapture::RecordingWorld recorder(world,*capture);bool began=false,ended=false;
        for(unsigned frame=0;frame<360&&!ended;++frame) {
            const Input current=began?Input{}:input;const auto calls=world.calls;
            capture->begin(t,current,.025f,1000);const auto result=t.update(recorder,current,.025f,1000);capture->finish(t,result);
            check(capture->count()==world.calls-calls&&capture->complete(),"recovery capture preserves the exact bounded source query count");
            std::string error;const auto encoded=capture->serialize();
            check(decoded->deserialize(encoded,error)&&decoded->serialize()==encoded,"recovery planning and movement snapshots retain every recovery field");
            const auto replay=decoded->replay();if(!replay.matched)std::cerr<<replay.error<<'\n';
            check(replay.matched,"recovery search, orientation, live checks and landing replay exactly");
            ++frames;began|=t.recoveringSupport();ended=began&&!t.recoveringSupport()&&t.state==State::wall;
        }
        check(began&&ended,"each direction captures a complete collision-checked support recovery");
    }
    std::cout<<"support recovery snapshots replayed="<<frames<<'\n';
}
static void privateTransitionState() {
    auto capture=std::make_unique<TraversalCapture>();auto decoded=std::make_unique<TraversalCapture>();
    TopWall world;Traversal t;t.cfg.radius=31;t.cfg.gap=37;t.cfg.height=138;
    check(t.attach(world,{0,-37,20},{0,1,0},1000,35),"transition fixture attached");
    t.entry(Motion::ledgeCatch,false);TraversalCapture::RecordingWorld recorder(world,*capture);
    std::array<bool,6> states{};bool sawRun=false,sawCompleted=false;std::size_t replayed=0;
    for(int frame=0;frame<500&&t.active();++frame) {
        Input input{0,1,false,true};

        if(frame==22)input.hop=true;
        if(frame>55&&frame<65)input.run=true;
        states[int(t.state)]=true;
        capture->begin(t,input,1.f/60,1000);const auto result=t.update(recorder,input,1.f/60,1000);capture->finish(t,result);
        check(capture->complete(),"transition frame fits bounded storage");std::string error;
        check(decoded->deserialize(capture->serialize(),error),"transition snapshot loads");
        const auto report=decoded->replay();if(!report.matched)std::cerr<<"frame="<<frame<<" "<<report.error<<'\n';
        check(report.matched,"approach/action/run/top-out private state and complete query sequence replay");
        ++replayed;sawRun|=t.wallRunning();sawCompleted|=result.completed;
    }
    check(states[int(State::approach)]&&states[int(State::wall)]&&states[int(State::action)]&&states[int(State::mantle)]&&sawRun&&sawCompleted,
        "actual transition fixtures cover entry, ordinary wall, hop, run blending and top-out completion");
    std::cout<<"transition snapshots replayed="<<replayed<<'\n';
}

static void failedSearchCooldownState() {
    for(const int fps:{30,60,120}) {
        EdgeWall world;auto t=stalledTraversal(world,1.f/fps);const Vec stopped=t.position;
        auto capture=std::make_unique<TraversalCapture>();auto decoded=std::make_unique<TraversalCapture>();
        TraversalCapture::RecordingWorld recorder(world,*capture);
        unsigned topSearches=0,hopSearches=0;bool topGap=false,hopGap=false,topRetried=false,hopRetried=false;
        for(int frame=0;frame<fps;++frame) {
            const Vec before=t.position,normal=t.normal,right{-normal.y,normal.x,0};
            const Vec topFrom=before+Vec{0,0,t.cfg.grip+28};
            const Vec topTo=before-normal*(t.cfg.gap+std::max(12.f,t.cfg.radius*.55f)+4)+Vec{0,0,t.cfg.grip+28};
            const Vec hopBase=before+right*48+Vec{0,0,6};
            const Vec hopFrom=hopBase+normal*16,hopTo=hopBase-normal*t.cfg.reach;
            const auto sourceCalls=world.calls;
            capture->begin(t,{1,0},1.f/fps,1000);
            const auto result=t.update(recorder,{1,0},1.f/fps,1000);capture->finish(t,result);
            check(capture->complete()&&capture->count()==world.calls-sourceCalls,"cooldown frames retain complete World tape without extra queries");
            if(t.state!=State::wall||result.released||(t.position-stopped).length()>=.01f)
                std::cerr<<"cooldown fixture fps="<<fps<<" frame="<<frame<<" state="<<int(t.state)<<" delta="<<(t.position-stopped).length()<<" step="<<(t.position-before).length()<<" reason="<<result.reason<<" blocked="<<t.blockedReason<<'\n';
            check(t.state==State::wall&&!result.released&&(t.position-stopped).length()<.01f,"failed searches preserve supported blocked position throughout cooldown");
            bool topSearched=false,hopSearched=false;
            for(std::size_t index=0;index<capture->count();++index) {
                const auto& call=capture->call(index);if(call.kind!=TraversalCapture::Kind::ray)continue;
                topSearched|=(call.from-topFrom).length()<.001f&&(call.to-topTo).length()<.001f;
                hopSearched|=(call.from-hopFrom).length()<.001f&&(call.to-hopTo).length()<.001f;
            }
            if(topSearched){topRetried|=topSearches>0&&topGap;++topSearches;}else if(topSearches)topGap=true;
            if(hopSearched){hopRetried|=hopSearches>0&&hopGap;++hopSearches;}else if(hopSearches)hopGap=true;
            const auto encoded=capture->serialize();std::string error;
            check(decoded->deserialize(encoded,error)&&decoded->serialize()==encoded,"active failure retry fields round trip exactly");
            const auto report=decoded->replay();
            if(!report.matched)std::cerr<<"cooldown fps="<<fps<<" frame="<<frame<<" "<<report.error<<'\n';
            check(report.matched&&report.callsConsumed==capture->count(),"failure retry countdown, skipped searches and expiry replay every query and final state");
        }
        check(topRetried&&hopRetried,"top and ordinary hop searches both defer intervening frames and retry after cooldown");
        std::cout<<"cooldown snapshots fps="<<fps<<" replayed="<<fps<<" top searches="<<topSearches<<" hop searches="<<hopSearches<<'\n';
    }
}

static void animationProfileValidation() {
    auto capture=std::make_unique<TraversalCapture>();auto decoded=std::make_unique<TraversalCapture>();
    Traversal t;t.cfg.staminaEnabled=false;t.cfg.automaticSideWeights={.25f,.75f};
    t.cfg.contextualMantleEnabled=false;
    auto roundTrip=[&] {
        capture->begin(t,{},1.f/60,0);capture->finish(t,{});std::string error;
        return decoded->deserialize(capture->serialize(),error);
    };
    check(roundTrip()&&decoded->serialize()==capture->serialize(),"DIY profile, stamina and probability settings round trip exactly");
    for(int id:{17,30,31,32}) {
        auto motions=std::make_shared<std::array<AuthoredMotion,42>>();(*motions)[id-1].enabled=true;
        t.cfg.authoredMotions=std::move(motions);
        check(!roundTrip(),"retired source-motion metadata is rejected before playback");
    }
    t.cfg.authoredMotions.reset();
    t.cfg.threepeatProfile.pathCounts[0]=0;
    check(!roundTrip(),"zero-length profile path rejected before replay");
    t.cfg.threepeatProfile=defaultThreepeatProfile();
    t.cfg.threepeatProfile.paths[0][1].phase=t.cfg.threepeatProfile.paths[0][0].phase;
    check(!roundTrip(),"duplicate path phases rejected before interpolation");
    t.cfg.threepeatProfile=defaultThreepeatProfile();
    t.cfg.threepeatProfile.mantleUnplant[1]=2;
    check(!roundTrip(),"out-of-range contact window rejected before replay");
}
static void wallRunSettingReplay() {
    for(int fps:{30,60,120}) {
        EdgeWall world;Traversal t;t.cfg.radius=31;t.cfg.gap=37;t.cfg.height=138;t.cfg.approachSeconds=0;
        check(t.attach(world,world.origin+Vec{-1000,-37,200},{0,1,0},1000,35),"master-setting replay begins on a broad real wall");
        auto capture=std::make_unique<TraversalCapture>();auto decoded=std::make_unique<TraversalCapture>();
        TraversalCapture::RecordingWorld recorder(world,*capture);Input input{0,1,false,false,false,false,true};
        for(int frame=0;frame<fps;++frame) {
            t.cfg.wallRunEnabled=frame<fps/3||frame>=fps*2/3;
            capture->begin(t,input,1.f/fps,1000);const auto out=t.update(recorder,input,1.f/fps,1000);capture->finish(t,out);
            check(capture->complete()&&!out.released&&t.wallRunning()==t.cfg.wallRunEnabled,"live master setting takes effect while preserving a complete attached capture");
            std::string error;const auto text=capture->serialize();
            check(decoded->deserialize(text,error)&&decoded->before().cfg.wallRunEnabled==t.cfg.wallRunEnabled&&decoded->after().cfg.wallRunEnabled==t.cfg.wallRunEnabled,"serialized capture retains both disabled and reenabled master settings");
            check(decoded->serialize()==text&&decoded->replay().matched,"mode changes and residual speed replay exact queries, positions and private state");
            if(frame==fps/3)check(decoded->deserialize(priorCapture(text,"active35-10"),error)&&!decoded->before().cfg.wallRunEnabled&&
                !decoded->after().cfg.wallRunEnabled&&decoded->replay().matched,"prior master-aware snapshots preserve disabled wall running while dropping only their unused arc field");
        }
    }
}
static void wallRunDirectionReplay() {
    auto sequences=std::make_shared<AuthoredWallRunSequences>();
    for(std::size_t index=0;index<5;++index) {
        sequences->valid[index]=true;
        for(auto* motion:{&sequences->launches[index],&sequences->catches[index]}) {
            motion->enabled=true;motion->seconds=.3f+float(index)*.07f;motion->stride=12;
            motion->trajectory.count=2;motion->trajectory.knots[0]={0,{}};motion->trajectory.knots[1]={1,{0,0,12}};
            for(std::size_t sample=0;sample<65;++sample)motion->contacts[sample]={float(index+1)*.1f,float(sample)/64.f};
        }
    }
    auto capture=std::make_unique<TraversalCapture>();auto decoded=std::make_unique<TraversalCapture>();
    const std::array<Vec,5> headings{Vec{0,1,0},Vec{-1,0,0},Vec{1,0,0},Vec{-1,1,0},Vec{1,1,0}};
    for(std::size_t index=0;index<5;++index) {
        EdgeWall world;Traversal t;t.cfg.authoredWallRunSequences=sequences;t.cfg.approachSeconds=0;
        check(t.attach(world,world.origin+Vec{-1000,-30,200},{0,1,0},1000),"direction capture attaches with independent authored metadata");
        TraversalCapture::RecordingWorld recorder(world,*capture);
        const auto direction=Motion(int(Motion::runUp)+int(index));const auto heading=headings[index];
        const Input running{heading.x,heading.y,false,false,false,false,true};
        auto record=[&](Input input) {
            capture->begin(t,input,1.f/60,1000);const auto result=t.update(recorder,input,1.f/60,1000);capture->finish(t,result);
            check(capture->complete(),"direction source preflight fits the bounded capture");
            const auto encoded=capture->serialize();std::string error;
            if(!decoded->deserialize(encoded,error))throw std::runtime_error(error);
            check(decoded->serialize()==encoded&&decoded->before().cfg.authoredWallRunSequences&&decoded->after().cfg.authoredWallRunSequences,
                "all five source trajectories, contacts and clocks round trip exactly");
            const auto report=decoded->replay();if(!report.matched)throw std::runtime_error(report.error);
            check(report.callsConsumed==capture->count(),"direction source replay retains every collision query and private field");
            return result;
        };
        const auto launched=record(running);
        check(t.state==State::action&&t.wallRunDirection(launched.motion)==direction,"captured launch holds the entered direction");
        unsigned frames=0;
        while(t.state==State::action&&frames++<120)record(running);
        check(t.state==State::wall&&decoded->after().wallRunDirection(launched.motion)==direction,"serialized final launch frame keeps its direction after leaving the action state");
        for(int frame=0;frame<30;++frame)t.update(world,running,1.f/60,1000);
        const auto caught=record({});
        check(caught.motion==Motion::runCatch&&t.state==State::action&&t.wallRunDirection(caught.motion)==direction,"captured catch retains the departed loop direction without movement input");
        frames=0;while(t.state==State::action&&frames++<120)record({});
        check(t.state==State::wall&&decoded->after().wallRunDirection(caught.motion)==direction,"serialized final catch frame keeps the correct independent source");
    }
    Traversal empty;capture->begin(empty,{},0,1000);capture->finish(empty,{});std::string error;
    check(decoded->deserialize(priorCapture(capture->serialize(),"active31-1"),error)&&!decoded->before().cfg.authoredWallRunSequences&&
        !decoded->after().cfg.authoredWallRunSequences&&decoded->replay().matched,"loading an older capture clears previously decoded directional source metadata");
    auto invalid=std::make_shared<AuthoredWallRunSequences>(*sequences);invalid->launches[0].seconds=0;empty.cfg.authoredWallRunSequences=invalid;
    capture->begin(empty,{},0,1000);capture->finish(empty,{});
    check(!decoded->deserialize(capture->serialize(),error),"invalid directional source duration is rejected before replay");
    invalid->launches[0].seconds=.3f;invalid->catches[4].contacts[64][1]=1.1f;
    capture->begin(empty,{},0,1000);capture->finish(empty,{});
    check(!decoded->deserialize(capture->serialize(),error),"invalid directional contact metadata is rejected before replay");
}
static void sharedWallRunSourceReplay() {
    auto sources=std::make_shared<std::array<AuthoredMotion,42>>();
    for(const auto motion:{Motion::runLaunchLeft,Motion::runCatch}) {
        auto& source=(*sources)[int(motion)-1];source.enabled=true;source.seconds=.3f;
        source.trajectory.count=2;source.trajectory.knots[0]={0,{}};source.trajectory.knots[1]={1,{0,0,12}};
    }
    EdgeWall world;Traversal t;t.cfg.authoredMotions=sources;t.cfg.approachSeconds=0;
    check(t.attach(world,world.origin+Vec{-1000,-30,200},{0,1,0},1000),"shared authored capture fixture attaches");
    auto capture=std::make_unique<TraversalCapture>();auto decoded=std::make_unique<TraversalCapture>();
    TraversalCapture::RecordingWorld recorder(world,*capture);const Input running{-1,0,false,false,false,false,true};
    auto record=[&](Input input) {
        capture->begin(t,input,1.f/60,1000);const auto result=t.update(recorder,input,1.f/60,1000);capture->finish(t,result);std::string error;
        check(decoded->deserialize(priorCapture(capture->serialize(),"active31-1"),error),"prior shared authored transitions deserialize without direction variants");
        const auto report=decoded->replay();if(!report.matched)throw std::runtime_error(report.error);
        check(decoded->serialize()==withoutRecovery(capture->serialize()),"old shared launch and catch snapshots replay while defaulting absent recovery fields");
        return result;
    };
    check(record(running).motion==Motion::runLaunchLeft&&t.state==State::action,"shared source launch starts through Core");
    unsigned frames=0;while(t.state==State::action&&frames++<60)record(running);
    check(t.state==State::wall,"shared source launch completes");
    for(int frame=0;frame<30;++frame)t.update(world,running,1.f/60,1000);
    check(record({}).motion==Motion::runCatch&&t.state==State::action,"shared source catch starts through Core");
    frames=0;while(t.state==State::action&&frames++<60)record({});
    check(t.state==State::wall,"shared source catch completes with exact prior-format playback");
}
static int replayLog(const char* path) {
    std::ifstream file(path);if(!file)throw std::runtime_error("cannot open captured log");
    auto capture=std::make_unique<TraversalCapture>();std::string line,block;unsigned blocks=0,failures=0;
    bool collecting=false;
    while(std::getline(file,line)) {
        if(const auto start=line.find("FCGEO_BEGIN ");start!=std::string::npos){block=line.substr(start)+'\n';collecting=true;continue;}
        if(!collecting)continue;
        if(block.size()+line.size()+1>TraversalCapture::maxTextBytes){++failures;collecting=false;std::cerr<<"capture block exceeds bound\n";continue;}
        block+=line+'\n';if(!line.starts_with("FCGEO_END"))continue;
        collecting=false;++blocks;std::string error;
        if(!capture->deserialize(block,error)){++failures;std::cerr<<"block "<<blocks<<": "<<error<<'\n';continue;}
        const auto report=capture->replay();failures+=!report.matched;
        std::cout<<"block "<<blocks<<" calls="<<capture->count()<<" observed="<<capture->observed()<<" matched="<<report.matched<<" consumed="<<report.callsConsumed<<" error="<<report.error<<'\n';
    }
    if(collecting){++failures;std::cerr<<"unterminated capture block\n";}
    if(!blocks){++failures;std::cerr<<"no capture blocks found\n";}
    return failures?1:0;
}
int main(int argc,char** argv){try {
    if(argc==2)return replayLog(argv[1]);
    snapshotRoundTrip();limitsAndWorldCompleteness();sessionBudget();captureCommitBudget();heldCaptureBudget();supportRecoveryReplay();privateTransitionState();failedSearchCooldownState();animationProfileValidation();wallRunSettingReplay();wallRunDirectionReplay();sharedWallRunSourceReplay();
    std::cout<<"PASS TraversalCapture exact versioned replay, no extra queries, truncation and session budgets\n";
    return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
