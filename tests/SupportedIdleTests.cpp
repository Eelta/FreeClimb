#define main supportedIdleThreepeatFixtureMain
#include "ThreepeatMotionTests.cpp"
#undef main

template<class T> concept HasCameraIntent=requires(T& value){value.setLookIntent(1.f,0.f,true);};
template<class T> concept HasOldLookOptions=requires(T& value){value.setIdleOptions(true,true);};
template<class T> concept HasReleasedHand=requires(T& value){value.releasedIdleHand();};
static_assert(!HasCameraIntent<SurfacePose> && !HasOldLookOptions<SurfacePose> && !HasReleasedHand<SurfacePose>);

static void supportedRest(const Library& lib,int fps,float scale,int kind) {
    ThreepeatWorld world;world.boxes={{{-10000,0,-5000},{10000,1000,20000}}};
    world.origin={2160,-1070,50};world.yaw=.633f;
    Traversal t;t.cfg.gap=37*scale;t.cfg.radius=31*scale;t.cfg.height=138*scale;
    t.cfg.contextScale=scale;t.cfg.approachSeconds=.01f;t.cfg.automaticClimbActions=false;
    check(lib.configureThreepeat(t.cfg),"existing captured hang calibration");
    t.cfg.threepeatAnimations=true;t.cfg.surfaceActionVariants=true;
    if(kind==2)world.boxes[0].high.z=t.cfg.threepeatHangHeight*scale;
    check(t.attach(world,world.point({0,-45*scale,0}),world.vector({0,1,0}),1000,60*scale),"rest attaches through real geometry");
    t.update(world,{},.05f,1000);
    if(kind!=0) {
        t.cfg.automaticClimbActions=kind==1;t.cfg.autoActionMinSeconds=.1f;t.cfg.autoActionMaxSeconds=.1f;
        bool prepared=false;
        for(int frame=0;frame<240&&!prepared;++frame) {
            const auto result=t.update(world,{1,0,false,false,kind==2&&frame==0},1.f/120,1000);
            prepared=result.motion==Motion::contextHang&&t.preparingEdge();
        }
        check(prepared&&t.usesEdgeTargets(Motion::contextHang),"captured preparation starts from a real side-hop request");
    }
    SurfacePose surface;
    const float dt=1.f/fps;Pose previous,initial;Vec restPosition=t.position;
    float minCOM=1e9f,maxCOM=-1e9f,chestTravel=0,palmError=0,wristAngle=0;
    bool captured=false,realLip=false;
    auto toWorld=[&](Vec p){return t.position+(Vec{-t.normal.y,t.normal.x,0}*p.x-t.normal*p.y+Vec{0,0,p.z})*scale;};
    for(int frame=0;frame<fps*10;++frame) {
        Result result;
        if(kind==0)result=t.update(world,{},dt,1000);
        else result.motion=Motion::contextHang;
        const auto pose=surface.update(lib,world,t,result.motion,dt,scale),body=lib.world(pose);
        captured|=result.motion==Motion::contextHang;
        realLip|=t.usesEdgeTargets(result.motion)&&!t.usesWallTargets(result.motion);
        check(t.active()&&!result.released,"resting output never changes supported traversal ownership");
        if(kind==0)check(result.motion==Motion::hang&&!t.preparingEdge(),"neutral input cannot independently start a contextual preparation");
        check((t.position-restPosition).length()<.001f,"resting output never moves the actor or collider");
        if(!previous.empty())for(std::size_t bone=0;bone<pose.size();++bone)
            check(angleBetween(previous[bone].q,pose[bone].q)<=12.566371f*dt+.001f,"resting output keeps the original bone-rate budget");
        if(frame==fps*2)initial=pose;
        if(!initial.empty()) {
            minCOM=std::min(minCOM,pose[4].t.z);maxCOM=std::max(maxCOM,pose[4].t.z);
            chestTravel=std::max(chestTravel,angleBetween(initial[26].q,pose[26].q));
            const auto authored=lib.sample(result.motion,surface.sampledPhase());
            for(int bone:{35,36})check(angleBetween(pose[bone].q,authored[bone].q)<.00001f,"idle adds no independent head or neck turn");
            for(int hand=0;hand<2;++hand) {
                check(lib.armBendValid(pose,hand),"resting hands preserve both original elbow branches");
                const Vec palm=toWorld(lib.palm(body,hand));Vec anchor;
                if(t.usesEdgeTargets(result.motion))anchor=t.edgeHand(hand,true)+
                    t.edgeContactNormal(hand,true)*((t.usesWallTargets(result.motion)?capturedWallPalmOffset:.8f)*scale);
                else {
                    const auto actual=world.ray(palm+t.normal*(14*scale),palm-t.normal*(20*scale));
                    check(actual&&actual->climbable,"both retained palms must have real support");
                    anchor=actual->point+actual->normal.unit()*(.8f*scale);
                }
                const float error=(palm-anchor).length()/scale;palmError=std::max(palmError,error);
                check(error<5.001f,"both hands stay inside the original five-unit contact envelope");
                const int elbow=hand?32:29,wrist=hand?39:38,middle=hand?88:73;
                const float wristBend=std::acos(std::clamp((body[wrist].t-body[elbow].t).unit().dot((body[middle].t-body[wrist].t).unit()),-1.f,1.f));
                wristAngle=std::max(wristAngle,wristBend);
                check(wristBend<1.658064f+.001f,"resting output preserves the original wrist-flexion bound");
                for(int bone=(hand?82:67);bone<(hand?97:82);++bone)
                    check(angleBetween(pose[bone].q,authored[bone].q)<.000001f,"supported idle retains authored finger articulation");
            }
        }
        previous=pose;
    }
    if(kind==0)check(chestTravel<.0001f&&maxCOM-minCOM<.0001f,"ordinary supported idle has no added chest or pelvis cycle");
    else check(captured&&chestTravel>.001f,"necessary preparation retains its authored source movement");
    if(kind==2)check(realLip,"the ledge case exercises a real top/front descriptor");
    const Vec before=t.position;
    for(int frame=0;frame<fps/2;++frame) {
        const auto result=t.update(world,{-1,0},dt,1000);
        const auto pose=surface.update(lib,world,t,result.motion,dt,scale);
        for(std::size_t bone=0;bone<pose.size();++bone)check(angleBetween(previous[bone].q,pose[bone].q)<=12.566371f*dt+.001f,
            "ordinary movement resumes with the unchanged output angular budget");
        previous=pose;
    }
    check((t.position-before).length()>scale,"resuming movement requires no idle release or regrip wait");
    std::cout<<"SUPPORTED_IDLE fps="<<fps<<" scale="<<scale<<" kind="<<kind<<" chest="<<chestTravel<<" comRange="<<maxCOM-minCOM<<
        " palms="<<palmError<<" wrist="<<wristAngle<<'\n';
}

static void supportChanges(const Library& lib,int fps,float scale,int missing) {
    ThreepeatWorld world;world.origin={2160,-1070,50};world.yaw=.633f;
    const std::vector<ThreepeatWorld::Box> complete{{{-10000,0,-5000},{0,1000,20000}},{{0,0,-5000},{10000,1000,20000}}};
    world.boxes=complete;
    Traversal traversal;traversal.cfg.gap=37*scale;traversal.cfg.radius=31*scale;traversal.cfg.height=138*scale;
    traversal.cfg.contextScale=scale;traversal.cfg.approachSeconds=.01f;traversal.cfg.automaticClimbActions=false;
    check(traversal.attach(world,world.point({0,-45*scale,0}),world.vector({0,1,0}),1000,60*scale),"changing-support fixture really attaches");
    traversal.update(world,{},.05f,1000);
    SurfacePose surface;const float dt=1.f/fps;Pose previous;
    const Vec restPosition=traversal.position;int removedSupports=-1,restoredSupports=-1;
    float maxPalm=0,maxStep=0;
    auto point=[&](Vec p){return traversal.position+(Vec{-traversal.normal.y,traversal.normal.x,0}*p.x-
        traversal.normal*p.y+Vec{0,0,p.z})*scale;};
    for(int frame=0;frame<fps*5;++frame) {
        if(frame==fps*2) {
            if(missing==0)world.boxes={complete[1]};
            else if(missing==1)world.boxes={complete[0]};
            else world.boxes.clear();
        }
        if(frame==fps*3)world.boxes=complete;
        if(frame<fps*2||frame>=fps*3) {
            const auto result=traversal.update(world,{},dt,1000);
            check(traversal.active()&&!result.released&&result.motion==Motion::hang,"real supported idle retains traversal ownership");
        }
        const auto pose=surface.update(lib,world,traversal,Motion::hang,dt,scale),body=lib.world(pose);
        check((traversal.position-restPosition).length()<.001f,"contact changes cannot move the actor from the pose layer");
        int supports=0;
        for(int hand=0;hand<2;++hand) {
            check(lib.armBendValid(pose,hand),"changing support retains both calibrated elbow branches");
            const Vec palm=point(lib.palm(body,hand));const auto hit=world.ray(palm+traversal.normal*(14*scale),palm-traversal.normal*(20*scale));
            if(hit&&hit->climbable&&hit->normal.dot(traversal.normal)>.95f) {
                ++supports;const float error=(palm-hit->point-hit->normal*(.8f*scale)).length()/scale;maxPalm=std::max(maxPalm,error);
                if(frame>fps)check(error<5.001f,"a surviving real palm keeps the original five-unit support envelope");
            }
            const int elbow=hand?32:29,wrist=hand?39:38,middle=hand?88:73;
            const float angle=std::acos(std::clamp((body[wrist].t-body[elbow].t).unit().dot((body[middle].t-body[wrist].t).unit()),-1.f,1.f));
            check(angle<1.658064f+.001f,"changing support never requires larger wrist-flexion allowance");
        }
        if(frame==fps*2)removedSupports=supports;
        if(frame==fps*4)restoredSupports=supports;
        if(!previous.empty())for(unsigned bone=0;bone<pose.size();++bone) {
            const float step=angleBetween(previous[bone].q,pose[bone].q);maxStep=std::max(maxStep,step);
            check(step<=12.566371f*dt+.001f,"contact removal and recovery retain the whole-pose angular budget");
        }
        previous=pose;
    }
    check(removedSupports==(missing==2?0:1),"closed geometry removes exactly the intended real supports");
    check(restoredSupports==2,"restoring wall geometry recovers both real supports");
    std::cout<<"IDLE_SUPPORT_CHANGE fps="<<fps<<" scale="<<scale<<" missing="<<missing<<" palm="<<maxPalm<<" boneStep="<<maxStep<<'\n';
}

int main(int argc,char** argv) {
    try {
        check(argc==2,"animation pack path required");Library lib;check(lib.load(argv[1]),"load unchanged runtime animation pack");
        for(int fps:{30,60,120})for(float scale:{.75f,1.f,1.3f})for(int kind:{0,1,2})supportedRest(lib,fps,scale,kind);
        for(int fps:{30,60,120})for(float scale:{.75f,1.f,1.3f})for(int missing:{0,1,2})supportChanges(lib,fps,scale,missing);
        std::cout<<"PASS stable supported idle, authored preparation, real contact recovery and original anatomical/output budgets\n";return 0;
    } catch(const std::exception& error) {std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}