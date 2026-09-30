#define main attachedSurfaceFixtureMain
#include "AttachSurfaceTests.cpp"
#undef main

static Traversal fresh(){Traversal t;t.cfg.radius=31;t.cfg.gap=37;t.cfg.height=138;return t;}
static void checkedApproach(AttachWorld world,float distance,int fps,bool airborne) {
    const float feetZ=airborne?220.f:0.f;const Vec start=world.global({0,-distance,feetZ});
    auto old=fresh();check(!old.attach(world,start,world.direction({0,1,0}),1000,35),"original 35-unit limit rejects measured 90..96 separation");
    check(old.lastFailure==AttachFailure::tooFar&&std::abs(old.lastAttachDistance-(distance-37))<.05f,
        "rejection reports nearest true projection, not farther last fan");
    auto t=fresh();check(t.attach(world,start,world.direction({0,1,0}),1000,60,false,!airborne),"explicit 60-unit approach reaches measured front wall");
    check(t.lastAttachDistance<=60&&(t.position-start).length()<.001f,"attachment starts at actual feet with no teleport");
    check(std::abs(t.lastAttachDistance-(distance-37))<.05f,"successful distance is the actual chosen candidate");
    t.entry(airborne?Motion::ledgeCatch:Motion::jumpCatch,!airborne);
    Vec previous=t.position;float peakLift=0;int frames=0;
    while(t.state==State::approach&&frames<fps) {
        const auto result=t.update(world,{},1.f/fps,1000);++frames;
        check(!result.released,"verified catch path remains available");
        const auto local=world.local(t.position);peakLift=std::max(peakLift,local.z-feetZ);
        check(world.capsuleClear(t.position,31,138),"full analytic body remains outside solids throughout entry");
        const float maxDelta=(1.6f*60+3.14159265f*18)/(fps*t.cfg.approachSeconds)+.06f;
        check((t.position-previous).length()<maxDelta,"animated entry displacement is frame-rate bounded");
        previous=t.position;
    }
    check(t.state==State::wall&&frames>=int(std::floor(fps*t.cfg.approachSeconds)),"entry spans its real duration at each FPS");
    const auto endpoint=world.local(t.position);
    check(std::abs(endpoint.y+37)<.04f&&std::abs(endpoint.z-feetZ)<.02f,"entry finishes at same supported wall gap and catch height");
    check(airborne?peakLift<.02f:peakLift>17.f,"air catch has no new lift; grounded catch keeps its actual 18-unit jump arc");
    for(int frame=0;frame<fps/4;++frame) {
        const auto result=t.update(world,{},1.f/fps,1000);
        check(!result.released&&t.active(),"hand support remains real after the longer approach");
    }
}
static void rejects(AttachWorld world,Vec feet,Vec facing,std::string label) {
    auto t=fresh();check(!t.attach(world,world.global(feet),world.direction(facing),1000,60),label);
    check(!t.active(),label+" leaves traversal inactive");
}
static void negativePaths(AttachWorld world,int fps) {
    rejects(world,{0,-98,0},{0,1,0},"61-unit required approach exceeds 60 and remains rejected");
    AttachWorld behind=world;behind.boxes={{{-1000,-100,-1000},{1000,0,1000}}};
    rejects(behind,{0,93,0},{0,1,0},"wall behind actor remains unavailable");
    AttachWorld narrow=world;narrow.boxes={{{-4,0,-1000},{4,100,1000}}};
    rejects(narrow,{0,-93,0},{0,1,0},"closer narrow ribbon has no real hand neighbours");
    AttachWorld low=world;low.boxes={{{-1000,0,-1000},{1000,100,4}}};
    rejects(low,{0,-93,0},{0,1,0},"toe-height fragment cannot replace a grip or climbable top");
    AttachWorld obstruction=world;obstruction.boxes.push_back({{-100,-55,65},{100,-48,75},false});
    check(obstruction.capsuleClear(obstruction.global({0,-93,0}),31,138),"intervening obstacle starts beyond actual body");
    auto blocked=fresh();check(!blocked.attach(obstruction,obstruction.global({0,-93,0}),obstruction.direction({0,1,0}),1000,60),"intervening chest-level obstacle blocks the longer approach");
    check(blocked.lastFailure==AttachFailure::clearance&&std::abs(blocked.lastAttachDistance-56)<.05f,
        "clearance failure reports its own closest candidate, never a later farther fan or zero distance");

    auto t=fresh();const auto start=world.global({0,-93,0});check(t.attach(world,start,world.direction({0,1,0}),1000,60,false,true),"initial clear path attaches");
    t.entry(Motion::jumpCatch,true);AttachWorld live=world;live.boxes.push_back({{-100,-55,0},{100,-48,300},false});
    bool interrupted=false;
    for(int frame=0;frame<fps&&t.active();++frame) {
        const auto result=t.update(live,{},1.f/fps,1000);
        check(live.capsuleClear(t.position,31,138),"changed geometry halts approach before collision");
        if(result.released){interrupted=true;check(std::string(result.reason)=="entry path blocked","changed entry path reports blockage");}
    }
    check(interrupted&&!t.active(),"appearing obstruction cancels animated approach safely");

    AttachWorld beam=world;beam.boxes.push_back({{-1000,-200,149},{1000,100,153},false});
    check(beam.capsuleClear(beam.global({0,-93,0}),31,138)&&beam.capsuleClear(beam.global({0,-37,0}),31,138),
        "known beam leaves both catch endpoints genuinely clear");
    auto ground=fresh();check(!ground.attach(beam,start,beam.direction({0,1,0}),1000,60,false,true),"known apex-only beam rejected before ground entry acquires control");
    check(ground.lastFailure==AttachFailure::clearance&&!ground.active(),"ground arc preflight failure is a clearance rejection");
    auto air=fresh();check(air.attach(beam,start,beam.direction({0,1,0}),1000,60,false,false),"same beam allows an unlifted airborne catch");
    air.entry(Motion::ledgeCatch,false);
    while(air.state==State::approach){const auto result=air.update(beam,{},1.f/fps,1000);check(!result.released,"air entry stays below the same beam");}

    AttachWorld changing=world;auto crossing=fresh();
    check(crossing.attach(changing,start,changing.direction({0,1,0}),1000,60,false,true),"dynamic-apex fixture preflights in empty space");
    crossing.entry(Motion::jumpCatch,true);
    crossing.update(changing,{},.05f,1000);crossing.update(changing,{},.05f,1000);crossing.update(changing,{},.05f,1000);
    check(std::abs(crossing.reachProgress()-.46875f)<.0001f,"slow frame begins just before apex");
    changing.boxes.push_back({{-1000,-200,155.97f},{1000,100,156.1f},false});
    check(changing.capsuleClear(crossing.position,31,138),"new thin ceiling is clear at previous displayed frame");
    const auto before=crossing.position;const auto stopped=crossing.update(changing,{},.05f,1000);
    check(stopped.released&&std::string(stopped.reason)=="entry path blocked"&&(crossing.position-before).length()<.001f,
        "crossed apex knot rejects dynamic ceiling even when the frame chord would miss it");
}
int main(){try {
    int catches=0,groups=0;
    for(Vec origin:{Vec{},Vec{133820.3f,38370.9f,-12265.4f}})for(float rotation:{0.f,1.13f})for(int fps:{30,60,120}) {
        AttachWorld world;world.origin=origin;world.rotation=rotation;world.boxes={wall};world.validateFront();
        for(float distance:{90.f,93.f,96.f})for(bool airborne:{false,true}){checkedApproach(world,distance,fps,airborne);++catches;}
        negativePaths(world,fps);++groups;
    }
    std::cout<<"PASS distance catches="<<catches<<" old35Rejected="<<catches<<" negativeGroups="<<groups<<" cfg=31/37/138; grounded lift18 / air lift0\n";
    return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
