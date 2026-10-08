#include "Controls.h"
#include "PoseHandoff.h"
#include "CornerTestWorld.h"
#include <iostream>
#include <fstream>
#include <stdexcept>
using namespace fc;
static void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
static std::ofstream samples;static unsigned sampleCount{};
static Keys entryKeys(bool shift=false){Keys keys;keys.w=keys.a=keys.d=keys.space=true;keys.shift=shift;return keys;}

static void climbEntry(const Library& library,int fps,bool falling,bool heldRunModifier,bool far,bool stopDuringEntry=false,int side=0,float entrySpeed=-1,bool nativePrediction=false) {
    fc_test::CornerWorld world;world.boxes.push_back({{-4000,0,-2000},{4000,400,8000}});
    if(far){world.origin={132858.95f,38433.08f,-11268.09f};world.yaw=.63f;}
    Traversal traversal;traversal.cfg=fc_test::settings();traversal.cfg.approachSeconds=.36f;traversal.cfg.runSpeed=379.5f;
    traversal.cfg.automaticClimbActions=false;traversal.cfg.contextActions=false;
    const Vec start=world.global({0,falling?-60.f:entrySpeed>=0&&entrySpeed<180?-55.f:-90.f,300});const float dt=1.f/fps;
    const auto flight=grabFlight(falling,false,false,nativePrediction,falling?-300.f:0.f);
    Keys keys=entryKeys(heldRunModifier);ClimbEntryIntent entry;JumpGrabGate gate;WallRunEntryGate runGate;
    const auto request=entry.sample(keys,false,false,dt,flight.confirmedAirborne);
    require(request.requested&&request.fresh,"full chord immediately requests climbing before pose preparation");
    gate.hold(world.direction({0,1,0}),false,request.airborneAtBegin,request.fresh);
    require(gate.pending()&&traversal.attach(world,start,gate.facing(),1000,60,gate.explicitAirCatch(flight),!flight.airborne),"entry uses an actual checked ground or air approach");
    require((traversal.position-start).length()<.001f,"acquisition keeps the current native root before entry output");
    const auto catchMotion=entrySpeed<0?grabEntryMotion(flight):grabEntryMotion(flight,entrySpeed,(traversal.entryTarget()-start).length(),!falling);
    require(catchMotion==(falling?Motion::ledgeCatch:entrySpeed<0||entrySpeed>=180?Motion::jumpCatch:Motion::reach),
        "physical flight and grounded approach choose the matching climbing entry without starting a wall run");
    require(traversal.entry(world,catchMotion,!flight.airborne),"selected entry retains the final source-path preflight");
    entry.blockUntilRelease();gate.cancel();runGate.begin(keys);
    SurfacePose surface;Pose previous;Motion last=Motion::none;
    Vec oldPosition=traversal.position;float maxAngleRate=0,maxLocalTranslationStep=0,maxRootSpeed=0;
    bool catchSeen=false,runSeen=false,climbSeen=false,firstLoop=false,previousSpace=false;unsigned fullRunFrames=0,frameIndex=0;
    float distanceRun=0,distanceClimb=0;
    auto tick=[&](bool first=false,bool runAllowed=false) {
        const bool spacePressed=keys.space&&!previousSpace;previousSpace=keys.space;
        const auto input=wallInput(runGate.filter(keys),spacePressed,false,first,traversal.wallRunning());
        const auto result=traversal.update(world,input,first?0.f:dt,1000);
        require(!result.released&&traversal.active(),"input transitions cannot release a supported clear wall");
        require(!hopMotion(result.motion)&&result.motion!=Motion::backFlipOut,"entry Space and wall-run Space never leak into leap or departure actions");
        require(result.motion!=Motion::runLaunch,"direct run entry remains absent");
        if(!runAllowed)require(!runMotion(result.motion),"held entry modifier cannot bypass climbing acquisition");
        const auto pose=surface.update(library,world,traversal,result.motion,first?0.f:dt,1);
        require(pose.size()==library.rest.size()&&pose.size()==99,"entry and locomotion retain the complete runtime skeleton");
        require(library.armBendValid(pose,0)&&library.armBendValid(pose,1),"entry, climbing and running retain the anatomical elbow branch");
        for(unsigned bone=0;bone<pose.size();++bone) {
            require(pose[bone].t.finite()&&std::abs(pose[bone].q.dot(pose[bone].q)-1)<.004f,"all transition transforms remain finite and normalized");
            if(!previous.empty()&&!first) {
                const float angle=angleBetween(previous[bone].q,pose[bone].q);
                maxAngleRate=std::max(maxAngleRate,angle/dt);
                maxLocalTranslationStep=std::max(maxLocalTranslationStep,(pose[bone].t-previous[bone].t).length());
                require(angle<=(runMotion(result.motion)||runMotion(last)?18.849556f:12.566371f)*dt+.016f,
                    "real entry, climb and run output retains the complete-pose angular continuity budget");
            }
        }
        if(last==catchMotion&&result.motion!=catchMotion) {
            firstLoop=true;
            require(result.motion==Motion::up||result.motion==Motion::hang,"climbing catch completes into climbing or ordinary wall rest before running");
        }
        if(samples.is_open()&&fps==60&&!falling&&!heldRunModifier&&!far&&!stopDuringEntry&&!side&&
            (frameIndex<=24?frameIndex%3==0:frameIndex<=160&&frameIndex%6==0)) {
            if(sampleCount++)samples<<',';
            samples<<"{\"case\":\"Chord climb entry and attached wall run\",\"stage\":\"frame "<<frameIndex<<"\",\"motion\":"<<int(result.motion)<<",\"phase\":"<<traversal.reachProgress()<<",\"position\":["<<traversal.position.x<<','<<traversal.position.y<<','<<traversal.position.z<<"],\"normal\":["<<traversal.normal.x<<','<<traversal.normal.y<<','<<traversal.normal.z<<"],\"joints\":[";
            const auto body=library.world(pose);const Vec normal=traversal.normal,right{-normal.y,normal.x,0};
            for(unsigned bone=0;bone<body.size();++bone){if(bone)samples<<',';const Vec point=traversal.position+right*body[bone].t.x-normal*body[bone].t.y+Vec{0,0,body[bone].t.z};samples<<'['<<point.x<<','<<point.y<<','<<point.z<<']';}
            samples<<"]}";
        }
        ++frameIndex;
        const float distance=(traversal.position-oldPosition).length();
        if(!first)maxRootSpeed=std::max(maxRootSpeed,distance/dt);
        require(world.clearance(traversal.position,traversal.cfg)>=traversal.cfg.radius-.06f,"independent box distance preserves full capsule clearance through entry and movement");
        require(distance<=std::max(420.f,traversal.cfg.runSpeed*1.15f)*dt+.05f,"checked approach and configured diagonal run cannot teleport between frames");
        const auto paused=surface.update(library,world,traversal,result.motion,0,1);
        for(unsigned bone=0;bone<pose.size();++bone)
            require((pose[bone].t-paused[bone].t).length()<.00001f&&angleBetween(pose[bone].q,paused[bone].q)<.00001f,
                "zero-time callback cannot restart or refresh the displayed pose");
        catchSeen|=result.motion==catchMotion;runSeen|=runMotion(result.motion);climbSeen|=result.motion==Motion::up;
        if(runMotion(result.motion)){++fullRunFrames;distanceRun+=distance;}
        if(result.motion==Motion::up)distanceClimb+=distance;
        previous=pose;last=result.motion;oldPosition=traversal.position;
        return result;
    };
    const auto first=tick(true);
    require(first.motion==catchMotion&&traversal.state==State::approach,"first published frame is a checked climbing catch");
    for(int frame=0;frame<fps;++frame) {
        if(frame==fps/2)keys.a=keys.d=keys.space=false;
        if(stopDuringEntry&&frame==fps/6)keys.w=false;
        if(stopDuringEntry&&frame==fps/2)keys.w=true;
        tick();
    }
    require(catchSeen&&firstLoop&&climbSeen&&!runSeen,"entry and its initial held modifier actually complete into climbing");
    keys.shift=false;tick();
    keys.shift=true;keys.a=side<0;keys.d=side>0;
    for(int frame=0;frame<fps*2;++frame) {
        keys.space=frame>fps/2&&frame<fps;
        tick(false,true);
    }
    require(runSeen&&fullRunFrames>fps&&distanceRun>100,"a fresh attached run command retains sustained faster wall movement");
    keys.shift=keys.space=keys.a=keys.d=false;
    for(int frame=0;frame<fps;++frame)tick(false,true);
    require(last==Motion::up&&!traversal.wallRunning()&&distanceClimb>10,"run modifier release returns to climbing without stale running pose ownership");
    std::cout<<"entry fps="<<fps<<" falling="<<falling<<" heldRunModifier="<<heldRunModifier<<" far="<<far
        <<" stopped="<<stopDuringEntry<<" side="<<side<<" entry="<<int(catchMotion)<<" angleRate="<<maxAngleRate<<" localTranslationStep="<<maxLocalTranslationStep<<" rootSpeed="<<maxRootSpeed<<'\n';
}

static void changedEntryCollision(int fps) {
    fc_test::CornerWorld world;world.boxes.push_back({{-4000,0,-2000},{4000,400,8000}});
    Traversal traversal;traversal.cfg=fc_test::settings();traversal.cfg.approachSeconds=.36f;
    require(traversal.attach(world,{0,-90,300},{0,1,0},1000,60,false,true),"dynamic obstacle fixture obtains a checked climbing entry");
    traversal.entry(Motion::jumpCatch,true);
    Keys keys;keys.w=true;
    traversal.update(world,wallInput(keys,false),1.f/fps,1000);
    world.boxes.push_back({{-50,-75,305},{50,-65,460},false});
    bool released=false;
    for(int frame=0;frame<fps&&!released;++frame) {
        const auto result=traversal.update(world,wallInput(keys,false),1.f/fps,1000);released=result.released;
    }
    require(released&&!traversal.active(),"live entry obstruction cancels approach instead of tunnelling");
}
static void nativeJumpCatch(const Library& library,int fps) {
    fc_test::CornerWorld world;world.boxes.push_back({{-4000,0,-2000},{4000,400,8000}});
    Traversal traversal;traversal.cfg=fc_test::settings();traversal.cfg.approachSeconds=.36f;
    traversal.cfg.contextActions=traversal.cfg.automaticClimbActions=false;
    const float dt=1.f/fps;const auto flight=grabFlight(false,false,false,true,0);
    require(flight.airborne&&!flight.confirmedAirborne,"native jump press avoids duplicate ground launch without fabricating confirmed flight");
    Keys keys=entryKeys();ClimbEntryIntent intent;JumpGrabGate gate;
    const auto request=intent.sample(keys,false,false,dt,flight.confirmedAirborne);
    gate.hold({0,1,0},true,request.airborneAtBegin,request.fresh);
    const Vec native{0,-50,300};
    require(traversal.attach(world,native,gate.facing(),1000,60,gate.explicitAirCatch(flight),!flight.airborne),"native jump entry still requires a checked current wall");
    traversal.entry(grabEntryMotion(flight),!flight.airborne);intent.blockUntilRelease();
    SurfacePose surface;Pose previous;bool climbed=false;keys.a=keys.d=keys.space=false;
    for(int frame=0;frame<fps;++frame) {
        const auto result=traversal.update(world,wallInput(keys,false,false,frame==0),frame?dt:0.f,1000);
        require(!result.released&&result.motion!=Motion::runLaunch&&!runMotion(result.motion),"native jump catch enters climbing without running frames");
        if(traversal.state==State::approach)require(std::abs(traversal.position.z-native.z)<.001f,"already native-jumped entry cannot add a second ground kick");
        const auto pose=surface.update(library,world,traversal,result.motion,frame?dt:0.f,1);
        require(pose.size()==99&&library.armBendValid(pose,0)&&library.armBendValid(pose,1),"native jump catch has complete bones and legal elbows");
        if(!previous.empty())for(unsigned bone=0;bone<99;++bone)
            require(angleBetween(previous[bone].q,pose[bone].q)<=12.566371f*dt+.016f,"native jump catch-to-climb preserves complete-pose angular budget");
        require(world.clearance(traversal.position,traversal.cfg)>=traversal.cfg.radius-.06f,"native jump route retains full body radius");
        previous=pose;climbed|=result.motion==Motion::up;
    }
    require(climbed&&traversal.state==State::wall,"native jump catch completes into ordinary upward climbing");
}
static void changedEntryMotionPreflight(Motion initial) {
    auto motions=std::make_shared<std::array<AuthoredMotion,42>>();
    auto& source=(*motions)[int(initial)-1];source.enabled=true;source.seconds=2.4f;
    source.trajectory.count=5;
    source.trajectory.knots[0]={0,{}};
    source.trajectory.knots[1]={.2f,{90,0,0}};
    source.trajectory.knots[2]={.5f,{90,35,0}};
    source.trajectory.knots[3]={.8f,{90,70,0}};
    source.trajectory.knots[4]={1,{0,70,0}};
    require(source.trajectory.valid(),"changed-entry fixture has a checked source trajectory");
    for(Motion selected:{Motion::reach,Motion::jumpCatch,Motion::ledgeCatch})for(bool jump:{false,true}) {
        if(selected==initial)continue;
        fc_test::CornerWorld world;world.boxes.push_back({{-4000,0,-2000},{4000,400,8000}});
        world.boxes.push_back({{-5,-67,305},{5,-63,350},false});
        Traversal t;t.cfg.authoredMotions=motions;t.cfg.approachSeconds=.36f;
        const Vec from{0,-100,300};
        require(t.attach(world,from,{0,1,0},1000,80,false,jump,initial),
            "the initial v2 entry safely passes around an obstruction in the final v1 route");
        require(std::abs(t.entryDuration()-source.seconds)<.00001f,"initial v2 entry retains its source duration");
        Traversal unchanged=t;
        require(unchanged.entry(world,initial,jump)&&unchanged.entrySelection()==initial&&
            std::abs(unchanged.entryDuration()-source.seconds)<.00001f&&(unchanged.position-from).length()<.00001f,
            "retaining the actual v2 source rechecks its clear route without replacing its duration or moving the player");
        require(!t.entry(world,selected,jump),"switching to a different default entry checks its actual route before ownership");
        require((t.position-from).length()<.00001f&&t.state==State::approach,
            "rejected final entry cannot move the player or leave the prepared entry state");
        require(std::abs(t.entryDuration()-t.cfg.approachSeconds)<.00001f,
            "a final v1 entry restores the checked geometry duration rather than inheriting another slot's source clock");
        world.boxes.pop_back();
        Traversal clear;clear.cfg=t.cfg;
        require(clear.attach(world,from,{0,1,0},1000,80,false,jump,initial),"unobstructed source entry is available");
        require(clear.entry(world,selected,jump),"unobstructed final v1 entry passes its own complete route preflight");
        float elapsed=0;bool seen=false;
        while(clear.state==State::approach&&elapsed<1) {
            const auto result=clear.update(world,{},1.f/120,1000);elapsed+=1.f/120;
            require(!result.released,"final v1 route preflight agrees with live checked movement");
            seen|=result.motion==selected;
        }
        require(seen&&clear.state==State::wall&&std::abs(elapsed-.36f)<.009f,
            "changed final v1 entry plays its selected clip and finishes at the geometry clock");
        Traversal same;
        require(same.attach(world,from,{0,1,0},1000,80,false,jump,selected),"same-slot legacy entry obtains its existing route validation");
        const auto before=world.rays;
        require(same.entry(world,selected,jump)&&world.rays==before,"unchanged legacy entry does not repeat its complete route preflight");
    }
}
int main(int argc,char** argv){try {
    Library library;require((argc==2||argc==3)&&library.load(argv[1]),"load actual bundled motion library");
    for(Motion initial:{Motion::jumpCatch,Motion::ledgeCatch})changedEntryMotionPreflight(initial);
    if(argc==3){samples.open(argv[2]);require(samples.good(),"open diagnostic sample output");samples<<"{\"samples\":[";}
    for(int fps:{30,60,120}) {
        for(bool falling:{false,true})for(bool heldRunModifier:{false,true})for(bool far:{false,true})
            climbEntry(library,fps,falling,heldRunModifier,far);
        changedEntryCollision(fps);nativeJumpCatch(library,fps);
        climbEntry(library,fps,false,false,false,true);
        for(int side:{-1,1})climbEntry(library,fps,false,false,false,false,side);
        for(float speed:{0.f,240.f})for(bool heldRunModifier:{false,true})for(bool nativePrediction:{false,true})
            climbEntry(library,fps,false,heldRunModifier,false,false,0,speed,nativePrediction);
    }
    if(samples.is_open())samples<<"]}";
    std::cout<<"PASS: real Core/SurfacePose ground and airborne chord catch, climb-first transition and attached wall run, complete-pose budgets, native jump and live collision\n";
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
