#define main attachFixtureMain
#include "AttachSurfaceTests.cpp"
#undef main
#include "InputBindings.h"
#include "TraversalCapture.h"
#include "PoseHandoff.h"
struct DetachWorld:AttachWorld {
    bool actionBodyClear(Motion motion,Vec,Vec,float begin,float end,Vec)override {
        return motion==Motion::backFlipOut&&begin<=end;
    }
};
static Traversal detachTraversal(DetachWorld& world,float approach=0,float height=4000) {
    world.boxes={{{-1000,0,-1000},{1000,100,height}}};
    Traversal traversal;traversal.cfg.radius=31;traversal.cfg.gap=37;traversal.cfg.height=138;
    traversal.cfg.approachSeconds=approach;traversal.cfg.contextActions=false;traversal.cfg.fancyJumps=false;
    check(traversal.attach(world,world.global({0,approach>0?-55.f:-37.f,300}),world.direction({0,1,0}),1000,60),
        "detach fixture uses a real supported wall attachment");return traversal;
}
static Keys detachKeys(bool shift=false) {Keys keys;keys.a=keys.s=keys.d=keys.space=true;keys.shift=shift;return keys;}
static void routing() {
    for(unsigned mask=0;mask<32;++mask) {
        auto keys=detachKeys(mask&1);keys.w=mask&2;
        const auto input=wallInput(keys,true,mask&4,mask&8,mask&16);
        check(input.release&&!input.backDrop&&!input.run&&!input.hop&&!input.mantle&&input.x==0&&input.y==0,
            "semantic left backward right hop edge has highest priority over running, mantle and new attachment");
        check(wallInput(keys,false,mask&4,mask&8,mask&16).release,"held hop recognizes a combination completed by any direction key");
        keys.space=false;check(!wallInput(keys,false,mask&4,mask&8,mask&16).release,"incomplete held combination cannot request in-place detachment");
    }
    Keys back;back.s=back.space=true;
    for(bool running:{false,true})for(bool shift:{false,true}) {
        back.shift=shift;const auto input=wallInput(back,true,true,false,running);
        check(input.release&&input.backDrop&&!input.run,"backward hop keeps its existing outward departure");
        check(!wallInput(back,false,true,false,running).release,"ordinary backward-hop departure still requires a fresh hop edge");
    }
    Keys opposing;opposing.a=opposing.d=opposing.space=true;
    check(!wallInput(opposing,true).release,"left and right without backward cannot request detach");
    InputBindings bindings;bindings.forward=*parseKeyChord("Up");bindings.backward=*parseKeyChord("Down");
    bindings.left=*parseKeyChord("Left");bindings.right=*parseKeyChord("Right");bindings.hop=*parseKeyChord("E");
    check(validateBindings(bindings).valid,"independent replacement keys retain valid binding policy");
    InputState state;for(unsigned scan:{0xcb,0xd0,0xcd,0x12,0x2a})state.set(scan,true);
    std::array<unsigned,4> order{0xcb,0xcd,0xd0,0x12};std::sort(order.begin(),order.end());unsigned permutations=0;
    do {
        InputState sequential;
        for(unsigned i=0;i<order.size();++i) {
            sequential.set(order[i],true);
            const auto next=wallInput(mapKeys(sequential,bindings),order[i]==0x12,true,true,true);
            check(next.release==(i==3),"only the complete mapped four-key chord releases regardless of the last pressed key");
            if(next.release)check(!next.backDrop&&!next.hop&&!next.run&&next.x==0&&next.y==0,"every completed order selects the same in-place intent");
        }
        ++permutations;
    }while(std::next_permutation(order.begin(),order.end()));
    check(permutations==24,"all four-key input orders are exercised");
    const auto input=wallInput(mapKeys(state,bindings),true,true,true,true);
    check(input.release&&!input.backDrop&&!input.run&&input.x==0&&input.y==0,
        "detachment follows mapped actions rather than hardcoded keyboard scans");
}
static void scenario(int kind,int fps,bool shifted,float yaw,Vec origin,unsigned& checks) {
    DetachWorld world;world.rotation=yaw;world.origin=origin;
    auto traversal=detachTraversal(world,kind==2?.32f:0,kind==7||kind==8?420.f:4000.f);
    if(kind==1) {
        for(unsigned frame=0;frame<45;++frame)check(!traversal.update(world,{0,1,false,false,false,false,true},1.f/60,1000).released,"fixture establishes actual wall running");
        check(traversal.wallRunning(),"wall running exists before the detach request");
    } else if(kind==3) {
        const auto hop=traversal.update(world,{0,1,false,false,true},1.f/60,1000);
        check(!hop.released&&traversal.state==State::action&&hopMotion(hop.motion),"fixture starts an actual supported leap action");
    } else if(kind==9||kind==10) {
        traversal.cfg.fancyJumps=kind==10;
        Keys back;back.s=back.space=true;
        const auto departure=traversal.update(world,wallInput(back,true),1.f/60,1000);
        check(!departure.released&&traversal.state==State::action&&departure.motion==(kind==10?Motion::backFlipOut:Motion::dropBack),
            "fixture starts the actual existing outward push or checked backflip before cancelling it");
    } else if(kind==11) {
        check(!traversal.update(world,wallInput(detachKeys(),true),1.f/120,1000).released,
            "fixture starts the existing short drop before repeated release requests");
    } else if(kind==4)world.boxes.clear();
    else if(kind==7||kind==8) {
        const auto reached=traversal.update(world,{0,0,false,kind==7},1.f/60,1000);
        check(!reached.released&&traversal.state==(kind==7?State::mantle:State::ledge),"fixture reaches an actual mantle or ledge state");
    }
    const auto start=traversal.position;const float dt=kind==6?0.f:1.f/fps;
    const auto input=wallInput(detachKeys(shifted),true,true,kind==2,traversal.wallRunning());
    TraversalCapture capture,decoded;capture.begin(traversal,input,dt,kind==5?0.f:1000.f);
    TraversalCapture::RecordingWorld recorder(world,capture);
    auto result=traversal.update(recorder,input,dt,kind==5?0.f:1000.f);capture.finish(traversal,result);
    const auto encoded=capture.serialize();std::string error;
    check(decoded.deserialize(encoded,error)&&decoded.serialize()==encoded&&decoded.replay().matched,
        "in-place release captures its real state and unchanged World route exactly");
    check((traversal.position-start).length()<.003f,"first detach frame cannot translate the player root");
    check(result.releaseVelocity.length()==0,"in-place release supplies no outward or upward impulse");
    if(kind!=5)check(result.motion==Motion::drop,"in-place release selects the existing short let-go action");
    unsigned frames=0;
    while(traversal.active()&&frames++<fps) {
        const float previousProgress=traversal.actionProgress();
        result=traversal.update(world,wallInput(detachKeys(shifted),kind==11),1.f/fps,1000);
        check(traversal.actionProgress()>previousProgress,"held or repeated release cannot restart the existing let-go action");
        check((traversal.position-start).length()<.003f,"complete let-go keeps the root at the release point");
        check(result.releaseVelocity.length()==0,"let-go completion never borrows backward jump velocity");
    }
    check(!traversal.active()&&result.released&&!result.completed,"every supported state leaves traversal into ordinary falling");
    check(frames<=unsigned(std::ceil(.16f*fps))+1,"let-go completes within its existing short action duration");
    const auto held=traversal.update(world,wallInput(detachKeys(shifted),false),1.f/fps,1000);
    check(!held.released&&!traversal.active()&&(traversal.position-start).length()<.003f,"held combination cannot restart or repeat detached traversal");
    ++checks;
}
static void backwardDeparture() {
    DetachWorld world;auto traversal=detachTraversal(world);const auto start=traversal.position;
    Keys back;back.s=back.space=true;const auto input=wallInput(back,true);
    check(input.backDrop,"ordinary backward hop still selects backward departure intent");
    const auto result=traversal.update(world,input,1.f/60,1000);
    check(result.motion==Motion::dropBack&&!result.released&&(traversal.position-start).dot(traversal.normal)>0,
        "ordinary backward hop still plays the actual existing outward push route");
}
static void poseTransitions(const Library& library,int fps,int kind,bool far,unsigned& cases,unsigned& frames) {
    DetachWorld world;if(far){world.origin={134767.25f,39721.29f,-11971.05f};world.rotation=.73f;}
    auto traversal=detachTraversal(world,0,kind==4?420.f:4000.f);
    check(library.configureThreepeat(traversal.cfg),"in-place exit uses the actual installed HKX metadata");
    const float dt=1.f/fps;SurfacePose surface;Pose previous;PoseHandoff handoff;float time=0;Motion prior=Motion::none;
    auto tick=[&](Input input) {
        const auto result=traversal.update(world,input,dt,1000);time+=dt;
        const auto root=traversal.position;const auto pose=surface.update(library,world,traversal,result.motion,dt,1);
        check((traversal.position-root).length()==0,"exit pose adaptation never modifies the actual player root");
        check(pose.size()==99,"in-place exit retains the complete actual HKX skeleton");
        const float rate=runMotion(result.motion)||flipMotion(result.motion)||result.motion==Motion::backFlipOut?18.849556f:12.566371f;
        for(unsigned bone=0;bone<pose.size();++bone) {
            check(pose[bone].t.finite()&&std::abs(pose[bone].q.dot(pose[bone].q)-1)<.002f,"complete exit output remains finite and normalized");
            if(bone!=0&&bone!=4)check(std::abs(pose[bone].t.length()-library.rest[bone].t.length())<.002f,"every exit transition preserves fixed bone lengths");
            if(!previous.empty()) {
                const float allowed=rate*dt+.006f;
                if(angleBetween(previous[bone].q,pose[bone].q)>allowed)std::cerr<<"exit pose rate kind="<<kind<<" fps="<<fps<<" far="<<far<<" bone="<<bone<<" motion="<<int(result.motion)<<" prior="<<int(prior)<<" angle="<<angleBetween(previous[bone].q,pose[bone].q)<<" allowed="<<allowed<<'\n';
                check(angleBetween(previous[bone].q,pose[bone].q)<=allowed,"in-place let-go retains the existing full-pose angular rate limit");
                if(result.motion==Motion::drop)check((pose[bone].t-previous[bone].t).length()<=600.f*dt+.03f,
                    "ordinary let-go preserves the existing complete-pose translation speed limit");
            }
        }
        const auto body=library.world(pose);
        for(int hand=0;hand<2;++hand) {
            check(library.armBendValid(pose,hand),"in-place let-go cannot reverse the elbow branch");
            if(result.motion==Motion::drop) {
                const int elbow=hand?32:29,wrist=hand?39:38,middle=hand?88:73;
                const float bend=std::acos(std::clamp((body[wrist].t-body[elbow].t).unit().dot((body[middle].t-body[wrist].t).unit()),-1.f,1.f));
                check(bend<=1.658063f+.001f,"let-go wrists retain the existing 95-degree bend limit");
            }
        }
        const auto displayed=handoff.evaluate(library.rest,pose,1,0,time);
        check(handoff.consumed(displayed),"only actually displayed full poses become exit continuation sources");
        previous=pose;prior=result.motion;++frames;return result;
    };
    for(unsigned frame=0;frame<unsigned(fps/2);++frame)
        check(!tick(kind==1?Input{0,1,false,false,false,false,true}:Input{}).released,"actual supported source action remains attached before request");
    if(kind==2) {
        check(!tick({0,1,false,false,true}).released&&traversal.state==State::action,"actual leap source precedes in-place exit");
        for(unsigned frame=0;frame<3;++frame)check(!tick({}).released,"leap source displays its real intermediate poses");
    } else if(kind==3) {
        traversal.cfg.fancyJumps=true;Keys back;back.s=back.space=true;
        check(!tick(wallInput(back,true)).released&&traversal.state==State::action,"actual checked outward flip precedes cancellation");
        for(unsigned frame=0;frame<3;++frame)check(!tick({}).released,"outward flip source displays its real intermediate poses");
    } else if(kind==4) {
        check(!tick({0,0,false,true}).released&&traversal.state==State::mantle,"actual top-out preparation precedes cancellation");
        for(unsigned frame=0;frame<3;++frame)check(!tick({}).released,"mantle preparation displays its real intermediate poses");
    }
    const auto start=traversal.position;Result result;
    for(unsigned frame=0;traversal.active()&&frame<unsigned(fps);++frame) {
        result=tick(wallInput(detachKeys(),frame==0,true,false,traversal.wallRunning()));
        check(result.motion==Motion::drop&&(traversal.position-start).length()<.003f&&result.releaseVelocity.length()==0,
            "the real HKX let-go action remains physically in place through every output frame");
    }
    check(result.released&&!traversal.active()&&handoff.beginExit(true),"short release starts physical fall continuation from its last displayed output");
    handoff.advanceExitSource(0,library);
    auto shown=handoff.evaluate(library.rest,previous,1).pose;
    for(unsigned bone=0;bone<shown.size();++bone)check(angleBetween(shown[bone].q,previous[bone].q)<.002f&&(shown[bone].t-previous[bone].t).length()<.0001f,
        "first physical fall callback keeps exactly the consumed let-go pose without a stand reset");
    for(unsigned frame=1;frame<=unsigned(std::ceil(.16f*fps))+1;++frame) {
        const float elapsed=frame*dt;handoff.advanceExitSource(elapsed,library);
        const auto output=handoff.evaluate(library.rest,previous,1-smooth(elapsed/.16f));
        check(library.armBendValid(output.source,0)&&library.armBendValid(output.source,1),"continued physical fall source retains both anatomical elbows");
        for(const auto& bone:output.pose)check(bone.t.finite()&&std::abs(bone.q.dot(bone.q)-1)<.002f,"physical fall continuation remains finite normalized");
        if(elapsed>=.16f)for(unsigned bone=0;bone<output.pose.size();++bone)
            check(angleBetween(output.pose[bone].q,library.rest[bone].q)<.002f&&(output.pose[bone].t-library.rest[bone].t).length()<.0001f,
                "physical fall continuation reaches the continuously supplied native target exactly");
    }
    ++cases;
}
int main(int argc,char** argv){try {
    Library library;check(argc==2&&library.load(argv[1]),"load actual editable HKX animation pack");
    routing();unsigned checks=0;
    for(int fps:{30,60,120})for(bool shifted:{false,true})for(float yaw:{0.f,.73f})
        for(Vec origin:{Vec{},Vec{134767.25f,39721.29f,-11971.05f}})for(int kind=0;kind<12;++kind)
            scenario(kind,fps,shifted,yaw,origin,checks);
    backwardDeparture();unsigned poseCases=0,poseFrames=0;
    for(int fps:{30,60,120})for(int kind=0;kind<5;++kind)for(bool far:{false,true})poseTransitions(library,fps,kind,far,poseCases,poseFrames);
    std::cout<<"Real HKX in-place exit transitions="<<poseCases<<" poseFrames="<<poseFrames<<" rate/bone/elbow/display/fall checks PASS\n";
    std::cout<<"In-place detach scenarios="<<checks<<" wall/run/entry/action/missing/zeroStamina/zeroDt/mantle/ledge/outwardPush/backflip/repeatedDrop and all input orders/mapping/capture PASS\n";return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
