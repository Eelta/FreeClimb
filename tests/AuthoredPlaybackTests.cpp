#include "Pose.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
static void require(bool condition,const char* text){if(!condition)throw std::runtime_error(text);}
struct AuthoredLedge:World {
    float height=120,barrier=1e9f;
    bool floor=true;
    std::optional<Hit> ray(Vec a,Vec b) override {
        std::optional<Hit> hit;float nearest=2;
        const auto offer=[&](float t,Vec normal,bool solid){if(t>=0&&t<=1&&t<nearest){nearest=t;hit=Hit{a+(b-a)*t,normal,solid};}};
        if(a.y<0&&b.y>=0){const float t=-a.y/(b.y-a.y);if((a+(b-a)*t).z<=height)offer(t,{0,-1,0},true);}
        if(floor&&a.z>height&&b.z<=height){const float t=(height-a.z)/(b.z-a.z);if((a+(b-a)*t).y>=0)offer(t,{0,0,1},true);}
        if(a.y<barrier&&b.y>=barrier)offer((barrier-a.y)/(b.y-a.y),{0,-1,0},false);
        return hit;
    }
};
struct EmptyWorld:World {std::optional<Hit> ray(Vec,Vec) override{return {};}};
static Library authoredLibrary(const Library& base) {
    auto library=base;auto& clip=library.clips[int(Motion::contextMantle)-1];
    clip.authoredPlayback=true;clip.seconds=2;clip.height=127;clip.travel={0,66,127};
    clip.trajectory.count=5;
    clip.trajectory.knots[0]={0,{}};clip.trajectory.knots[1]={.25f,{0,0,40}};
    clip.trajectory.knots[2]={.55f,{0,0,132}};clip.trajectory.knots[3]={.8f,{0,66,132}};
    clip.trajectory.knots[4]={1,{0,66,127}};
    clip.frames.assign(121,base.clip(Motion::hang).frames.front());clip.contacts.assign(121,{});
    for(std::size_t i=0;i<clip.frames.size();++i) {
        const float phase=float(i)/float(clip.frames.size()-1);auto& pose=clip.frames[i];
        pose[0].t=pose[0].t+clip.trajectory.sample(phase);
        pose[4].t.z+=12.f*phase;
        pose[24].q=(Quat::axis({0,0,1},.10f*std::sin(phase*6.2831853f))*pose[24].q).unit();
        pose[39].q=(Quat::axis({0,0,1},.70f*std::sin(phase*75.398224f))*pose[39].q).unit();
        pose[56].q=(Quat::axis({0,0,1},.30f*std::sin(phase*6.2831853f))*pose[56].q).unit();
    }
    return library;
}
static Traversal attach(AuthoredLedge& world,const Library& library) {
    Traversal t;require(library.configureThreepeat(t.cfg),"authored fixture retains valid rig geometry");
    t.cfg.approachSeconds=0;
    require(t.attach(world,{0,-30,0},{0,1,0},100),"authored fixture attaches through actual collision geometry");
    return t;
}
static void mantle(const Library& library,int fps) {
    AuthoredLedge world;auto t=attach(world,library);const auto initial=t.position;
    auto result=t.update(world,{0,1,false,true},1.f/fps,100);
    require(t.state==State::mantle&&t.progress()==0,"authored mantle begins at frame zero without a preparation hold");
    require(t.topSampleBegin()==0&&t.topSeconds()==2,"authored timing does not inherit shortened or accelerated template phases");
    require((t.topPathPoint(0)-initial).length()<.001f&&(t.topPathPoint(1)-t.topTarget()).length()<.001f,"authored path keeps verified endpoints");
    EmptyWorld preview;SurfacePose surface;float time=0,worstCOM=0,worstSpine=0,worstRotation=0;
    surface.update(library,preview,t,Motion::contextMantle,1.f/fps,1);
    while(t.active()&&time<3) {
        const float previous=t.progress();result=t.update(world,{0,1,false,true},1.f/fps,100);time+=1.f/fps;
        require(result.motion==Motion::contextMantle,"clear authored mantle retains the selected animation");
        require(std::abs(t.progress()-std::min(1.f,previous+1.f/(2*fps)))<.00001f,"every authored mantle sample uses its original one-times clock");
        const auto pose=surface.update(library,preview,t,result.motion,1.f/fps,1),source=library.sample(result.motion,t.progress());
        const auto body=library.world(pose),original=library.world(source);
        const auto expected=original[4].t-library.clip(result.motion).trajectory.sample(t.progress());
        worstCOM=std::max(worstCOM,(body[4].t-expected).length());
        worstSpine=std::max(worstSpine,angleBetween(pose[24].q,source[24].q));
        for(std::size_t bone=0;bone<pose.size();++bone)worstRotation=std::max(worstRotation,angleBetween(pose[bone].q,source[bone].q));
        require((t.position-t.topPathPoint(t.progress())).length()<.001f,"live movement follows the prechecked authored path");
        for(const auto& transform:pose)require(transform.t.finite()&&std::abs(transform.q.dot(transform.q)-1)<.002f,"author playback preserves finite normalized pose output");
    }
    std::cout<<"authored mantle fps="<<fps<<" seconds="<<time<<" COM="<<worstCOM<<" spine="<<worstSpine<<" rotation="<<worstRotation<<'\n';
    require(result.completed&&std::abs(time-2)<=1.01f/fps,"authored mantle completes at the complete source duration");
    require(worstCOM<.001f&&worstSpine<.001f,"authored Root movement is subtracted exactly once while local COM and torso curves retain their source pose");
    require(worstRotation<.001f,"authored twist and fast source hand curves do not overwrite or stall unrelated source joints");
}
static void blocked(const Library& library) {
    AuthoredLedge world;auto t=attach(world,library);t.update(world,{0,1,false,true},1.f/60,100);
    require(t.state==State::mantle,"live obstruction fixture starts on a checked authored path");
    while(t.progress()<.65f&&t.active())t.update(world,{0,1,false,true},1.f/60,100);
    world.barrier=t.position.y+t.cfg.radius+.1f;const auto before=t.position;const auto phase=t.progress();
    for(int frame=0;frame<60;++frame) {
        const auto result=t.update(world,{0,1,false,true},1.f/60,100);
        require(t.active()&&t.state==State::mantle&&t.geometryHolding()&&!result.released&&!result.completed&&
            (t.position-before).length()<.001f&&t.progress()==phase,
            "a new live solid obstruction holds the authored movement segment without committing position or phase");
    }
    world.barrier=1e9f;bool completed=false;
    for(int frame=0;frame<180&&t.active();++frame) {
        const auto result=t.update(world,{0,1,false,true},1.f/60,100);completed|=result.completed;
        require(!result.released||result.completed,"restored authored mantle never forces an incomplete release");
    }
    require(completed&&!t.active(),"removing the live barrier resumes and completes the original authored mantle");
    auto impossible=library;auto& trajectory=impossible.clips[int(Motion::contextMantle)-1].trajectory;
    trajectory.knots[1].displacement={0,40,0};trajectory.knots[2].displacement={0,66,20};
    AuthoredLedge blockedWorld;auto candidate=attach(blockedWorld,impossible);
    const auto rejected=candidate.update(blockedWorld,{0,1,false,true},1.f/60,100);
    require(candidate.state==State::mantle&&!rejected.completed&&std::string_view(candidate.topRouteKind())=="corridor","early forward source movement fits a separately checked clearance route");
    AuthoredLedge sealedWorld;auto sealed=attach(sealedWorld,impossible);sealedWorld.barrier=-2;
    const auto refused=sealed.update(sealedWorld,{0,1,false,true},1.f/60,100);
    require(sealed.state!=State::mantle&&!refused.completed&&sealed.active(),"real obstruction rejects both source and clearance routes without disabling collision");
}
static void idle(const Library& base) {
    auto library=base;auto& clip=library.clips[int(Motion::hang)-1];clip.authoredPlayback=true;
    clip.seconds=2;clip.stride=100;clip.frames.assign(121,base.clip(Motion::hang).frames.front());clip.contacts.assign(121,{});
    for(std::size_t i=0;i<clip.frames.size();++i)clip.frames[i][35].q=(Quat::axis({0,0,1},.15f*std::sin(float(i)/120*6.2831853f))*clip.frames[i][35].q).unit();
    AuthoredLedge world;world.height=1000;auto t=attach(world,library);SurfacePose surface;EmptyWorld preview;
    for(int i=0;i<30;++i){const auto result=t.update(world,{0,1},1.f/60,100);surface.update(library,preview,t,result.motion,1.f/60,1);}
    float minimum=1,maximum=0,worst=0;
    for(int i=0;i<150;++i) {
        const auto result=t.update(world,{},1.f/60,100);const auto pose=surface.update(library,preview,t,result.motion,1.f/60,1);
        require(result.motion==Motion::hang,"authored idle stays in the actual hang slot");
        if(i<30)continue;
        minimum=std::min(minimum,surface.sampledPhase());maximum=std::max(maximum,surface.sampledPhase());
        worst=std::max(worst,angleBetween(pose[35].q,library.sample(Motion::hang,surface.sampledPhase())[35].q));
    }
    require(maximum-minimum>.9f&&worst<.001f,"stopping movement plays the authored idle through its whole loop instead of freezing the prior climb pose");
}
int main(int argc,char** argv) try {
    require(argc==2,"pack argument required");Library library;require(library.load(argv[1]),"load canonical animation library");
    for(const auto& clip:library.clips)require(!clip.authoredPlayback&&clip.trajectory.count==0,"all default clips retain their existing playback contract");
    const auto replacement=authoredLibrary(library);
    for(int fps:{30,60,120})mantle(replacement,fps);
    blocked(replacement);idle(library);
    std::cout<<"Authored playback timing, pose, idle and collision tests passed\n";
    return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
