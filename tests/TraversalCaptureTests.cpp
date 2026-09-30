#include "TraversalCapture.h"
#include <iostream>
#include <fstream>
#include <memory>
#include <stdexcept>
using namespace fc;
static void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}

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
static Traversal stalledTraversal(EdgeWall& world) {
    Traversal t;t.cfg.radius=31;t.cfg.gap=37;t.cfg.height=138;t.cfg.approachSeconds=0;
    check(t.attach(world,world.origin+Vec{0,-37,200},{0,1,0},1000,35),"real wall attachment");
    for(int frame=0;frame<100&&t.stalledSeconds()<=.4f;++frame)t.update(world,{1,0},1.f/48,1000);
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
    for(int id:{6,7,12,13,14,33,38}) {
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
    check(!gate.arm(t,{},1),"idle hanging never arms capture");
    check(gate.arm(t,{1,0},1)&&gate.used()==1,"existing deliberate .35s stall arms first frame");
    check(!gate.arm(t,{1,0},7),"same stuck position cannot spam a second capture");
    t.position.x+=120;
    check(!gate.arm(t,{1,0},5.9f),"different position still respects five-second interval");
    check(gate.arm(t,{0,1},6)&&gate.used()==2,"different later stall arms second frame");
    t.position.x+=120;
    check(!gate.arm(t,{1,0},20),"strict two-block per attachment budget");
    gate.reset();check(gate.arm(t,{1,0},.5f),"new attachment explicitly resets capture budget");
    gate.reset();t.stop();check(!gate.arm(t,{1,0},10),"released traversal cannot arm");
}
struct TopWall:World {
    std::optional<Hit> ray(Vec a,Vec b)override {
        const auto d=b-a;float nearest=2;std::optional<Hit> result;
        if(a.y<0&&b.y>=0){const float f=-a.y/d.y;const auto p=a+d*f;if(p.z<=240){nearest=f;result=Hit{p,{0,-1,0},true};}}
        if(a.z>240&&b.z<=240){const float f=(240-a.z)/d.z;const auto p=a+d*f;if(p.y>=0&&f<nearest)result=Hit{p,{0,0,1},true};}
        return result;
    }
};
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

static void animationProfileValidation() {
    auto capture=std::make_unique<TraversalCapture>();auto decoded=std::make_unique<TraversalCapture>();
    Traversal t;t.cfg.staminaEnabled=false;t.cfg.automaticSideWeights={.25f,.75f};
    t.cfg.contextualMantleEnabled=false;
    auto roundTrip=[&] {
        capture->begin(t,{},1.f/60,0);capture->finish(t,{});std::string error;
        return decoded->deserialize(capture->serialize(),error);
    };
    check(roundTrip()&&decoded->serialize()==capture->serialize(),"DIY profile, stamina and probability settings round trip exactly");
    t.cfg.threepeatProfile.pathCounts[0]=0;
    check(!roundTrip(),"zero-length profile path rejected before replay");
    t.cfg.threepeatProfile=defaultThreepeatProfile();
    t.cfg.threepeatProfile.paths[0][1].phase=t.cfg.threepeatProfile.paths[0][0].phase;
    check(!roundTrip(),"duplicate path phases rejected before interpolation");
    t.cfg.threepeatProfile=defaultThreepeatProfile();
    t.cfg.threepeatProfile.mantleUnplant[1]=2;
    check(!roundTrip(),"out-of-range contact window rejected before replay");
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
    snapshotRoundTrip();limitsAndWorldCompleteness();sessionBudget();privateTransitionState();animationProfileValidation();
    std::cout<<"PASS TraversalCapture exact versioned replay, no extra queries, truncation and session budgets\n";
    return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
