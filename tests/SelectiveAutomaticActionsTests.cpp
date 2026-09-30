#define main olderVarietyFixtureMain
#include "AutomaticVarietyTests.cpp"
#undef main

static bool legacyJump(Motion m){return m==Motion::hopUp||m==Motion::hopLeft||m==Motion::hopRight;}
static void ordinaryDownRemains(const Library& lib,int fps,bool far,bool legacy) {
    auto w=flat(far);auto t=attached(w,lib);const float dt=1.f/fps;
    t.cfg.legacyAutomaticHops=legacy;t.cfg.autoActionMinSeconds=.8f;t.cfg.autoActionMaxSeconds=1.25f;

    const Vec initial=w.local(t.position);SurfacePose animator;Pose previous;unsigned downFrames=0;
    float maxRate=0,minStep=100,maxStep=0;
    for(int frame=0;frame<fps*6;++frame) {
        const Vec before=w.local(t.position);const auto result=t.update(w,{0,-1},dt,1000);
        require(!result.released&&result.motion==Motion::down&&!t.preparingEdge()&&!t.usesEdgeTargets(result.motion),
            "S remains ordinary downward movement on formerly eligible captured wall patches");
        require(t.automaticActionCount()==0&&t.automaticAttemptCount()==0&&t.automaticOpportunityCount()==0&&t.surfaceActionCount()==0,
            "S neither starts a descending action nor accumulates automatic attempts/opportunities");
        require(std::abs(t.automaticActionPendingSeconds()-t.cfg.autoActionMinSeconds)<.00001f,
            "ordinary descent cannot create a randomized pending deadline");
        require(w.geometry.clearance(t.position,t.cfg)>=t.cfg.radius-.06f,"ordinary descent retains the full body radius");
        const Vec delta=w.local(t.position)-before;minStep=std::min(minStep,-delta.z);maxStep=std::max(maxStep,-delta.z);
        require(delta.z<0&&std::abs(delta.x)<.05f&&std::abs(delta.y)<.05f,
            "every S frame descends with no unload rise, lateral leap or one-frame freeze");
        const auto pose=animator.update(lib,w,t,result.motion,dt,1);
        require(pose.size()==99,"ordinary down has complete pose output");
        for(int hand=0;hand<2;++hand)require(lib.armBendValid(pose,hand),"ordinary down retains the original elbow branch");
        if(!previous.empty())for(std::size_t b=0;b<pose.size();++b) {
            const float rate=angleBetween(previous[b].q,pose[b].q)/dt;maxRate=std::max(maxRate,rate);
            require(rate<=12.566371f+.002f,"ordinary down retains every bone's original angular-rate limit");
        }
        previous=pose;++downFrames;
    }
    require(w.local(t.position).z<initial.z-200&&downFrames==unsigned(fps*6),"held S completes sustained uninterrupted descending travel");

    t=attached(w,lib);t.cfg.autoActionMinSeconds=t.cfg.autoActionMaxSeconds=1.25f;
    for(int i=0;i<fps/4;++i)t.update(w,{1,0},dt,1000);
    for(int i=0;i<fps*2;++i) {
        const auto r=t.update(w,{0,-1},dt,1000);
        require(r.motion==Motion::down&&t.automaticActionCount()==0&&t.automaticAttemptCount()==0,
            "S clears an earlier lateral timer and never inherits a delayed regrab");
    }

    Keys keys;keys.s=keys.space=true;
    auto leave=t.update(w,wallInput(keys,true,false,false,t.wallRunning()),dt,1000);
    require(t.state==State::action&&(leave.motion==Motion::backFlipOut||leave.motion==Motion::dropBack)&&t.automaticActionCount()==0,
        "S+Space still requests the checked outward departure or its original constrained-space fallback, not38");
    const Vec departureStart=t.position,outward=t.normal;
    for(int i=0;i<fps*2&&!leave.released;++i)leave=t.update(w,{},dt,1000);
    require(leave.released&&(t.position-departureStart).dot(outward)>1&&leave.releaseVelocity.dot(outward)>0,
        "S+Space actually leaves the wall with outward travel and release velocity");
    std::cout<<"ordinary down fps="<<fps<<" far="<<far<<" legacy="<<legacy<<" frames="<<downFrames
        <<" step="<<minStep<<'/'<<maxStep<<" angularRate="<<maxRate<<'\n';
}
static void newFamilyRemains(const Library& lib,int fps,Input input,bool far) {
    auto w=flat(far);auto t=attached(w,lib);const float dt=1.f/fps;
    require(!t.cfg.legacyAutomaticHops,"old random hops are off by default even when new automatic variety is enabled");
    t.cfg.autoActionMinSeconds=.8f;t.cfg.autoActionMaxSeconds=1.25f;
    const Motion expected=input.x<0?Motion::contextHopLeft:Motion::contextHopRight;
    const Vec initial=w.local(t.position);unsigned newFrames=0;SurfacePose animator;
    for(int frame=0;frame<fps*6;++frame) {
        const auto result=t.update(w,input,dt,1000);
        require(!result.released&&!legacyJump(result.motion),"enabled new family never falls back to random old W/A/D leaps");
        const auto pose=animator.update(lib,w,t,result.motion,dt,1);
        require(pose.size()==99,"selected actions have real complete motion output");
        for(int hand=0;hand<2;++hand)require(lib.armBendValid(pose,hand),"selective policy preserves existing elbow constraints");
        require((result.motion==Motion::none||isActiveMotion(result.motion)),"retired descending catch cannot be selected by an enabled new-family scheduler");
        if(threepeatHop(result.motion)) {
            require(result.motion==expected,"new automatic capture preserves requested side intent");
            actualWallAnchors(w,t,result.motion);++newFrames;
        }
    }
    require(t.automaticActionCount()>=1&&newFrames>unsigned(fps/3),"new40/41 still commit and visibly play with legacy random disabled");
    const Vec delta=w.local(t.position)-initial;
    require(delta.x*input.x>40,"new side capture continues the requested direction");
    std::cout<<"selective new fps="<<fps<<" input="<<input.x<<','<<input.y<<" far="<<far<<" commits="<<t.automaticActionCount()<<" frames="<<newFrames<<'\n';
}
static void ordinaryFallbackAndManual(const Library& lib,int fps) {
    for(Input input:{Input{0,1},Input{-1,0},Input{1,0}}) {
        auto w=flat();auto t=attached(w,lib);t.cfg.autoActionMinSeconds=t.cfg.autoActionMaxSeconds=.8f;

        t.cfg.surfaceActionVariants=false;
        const Vec initial=w.local(t.position);
        for(int i=0;i<fps*6;++i) {
            const auto result=t.update(w,input,1.f/fps,1000);
            require(!result.released&&!hopMotion(result.motion)&&(result.motion==Motion::none||isActiveMotion(result.motion)),
                "no compatible new capture means ordinary climb, never an old automatic replacement");
        }
        require(t.automaticActionCount()==0&&t.automaticAttemptCount()>0,"only legacy selection is disabled, not movement or the scheduler");
        const Vec delta=w.local(t.position)-initial;
        require(input.y>0?delta.z>400&&std::abs(delta.x)<.1f:delta.x*input.x>300,
            "same-input ordinary wall travel remains responsive");
        auto manual=attached(w,lib);manual.cfg.contextActions=false;manual.cfg.surfaceActionVariants=false;
        input.hop=true;const auto result=manual.update(w,input,1.f/fps,1000);
        const Motion expected=input.x<0?Motion::hopLeft:input.x>0?Motion::hopRight:Motion::hopUp;
        require(result.motion==expected&&manual.state==State::action&&manual.automaticActionCount()==0,
            "deliberate Space still starts the original checked up/left/right action with legacy random disabled");
    }

    auto w=flat();auto up=attached(w,lib);const Vec start=w.local(up.position);
    for(int i=0;i<fps*6;++i) {
        const auto result=up.update(w,{0,1},1.f/fps,1000);
        require(!hopMotion(result.motion)&&(result.motion==Motion::none||isActiveMotion(result.motion))&&up.automaticActionCount()==0,
            "pure W with new family enabled does not request an unsolicited sideways/old upward leap");
    }
    require(w.local(up.position).z>start.z+400&&std::abs(w.local(up.position).x-start.x)<.1f,"pure W keeps smooth upward movement");
    auto opted=attached(w,lib);opted.cfg.legacyAutomaticHops=true;bool old=false;
    for(int i=0;i<fps*6;++i)old|=opted.update(w,{0,1},1.f/fps,1000).motion==Motion::hopUp;
    require(old&&opted.automaticActionCount()>0,"legacy behavior remains explicit opt-in for compatibility only");
}
int main(int argc,char** argv){try {
    require(argc==2,"supply existing42-motion library");Library lib;require(lib.load(argv[1])&&lib.hasThreepeat(),"load calibrated licensed captured actions");
    for(int fps:{30,60,120}) {
        for(Input input:{Input{-1,0},Input{1,0},Input{-1,1},Input{1,1}})
            newFamilyRemains(lib,fps,input,false);
        ordinaryFallbackAndManual(lib,fps);
        for(bool legacy:{false,true})ordinaryDownRemains(lib,fps,false,legacy);
    }
    newFamilyRemains(lib,60,{-1,1},true);newFamilyRemains(lib,60,{1,1},true);
    ordinaryDownRemains(lib,60,true,false);
    std::cout<<"PASS selective automatic captures, default legacy exclusion, same-intent fallback and manual Space\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL selective automatic policy: "<<e.what()<<'\n';return 1;}}
