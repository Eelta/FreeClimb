#define main fluidityThreepeatFixturesMain
#include "ThreepeatMotionTests.cpp"
#undef main

struct FluidMetrics {
    float armTravel{},maxRate{},maxAcceleration{},maxJerk{},maxReach{};
    int contactFrames{},frames{};
    Pose previous;std::array<Vec,4> velocity{},acceleration{};
    void add(const Library& lib,const Pose& p,const SurfacePose& surface,float dt,bool measured) {
        constexpr int arms[]{28,29,31,32};
        for(int h=0;h<2;++h)check(lib.armBendValid(p,h),"fluidity never reverses anatomical elbows");
        for(const auto& bone:p)check(bone.t.finite()&&std::isfinite(bone.q.dot(bone.q)),"finite complete pose");
        if(!previous.empty()) {
            for(std::size_t b=0;b<p.size();++b)check(angleBetween(previous[b].q,p[b].q)<=18.849556f*dt+.016f,"unchanged complete-pose maximum angular budget");
            for(int i=0;i<4;++i) {
                auto delta=(p[arms[i]].q*previous[arms[i]].q.inverse()).unit();if(delta.w<0)delta={-delta.x,-delta.y,-delta.z,-delta.w};
                const Vec axis{delta.x,delta.y,delta.z};const float angle=2*std::atan2(axis.length(),std::max(0.f,delta.w));
                const Vec v=axis.unit()*(angle/dt),a=(v-velocity[i])/dt,j=(a-acceleration[i])/dt;
                if(measured){armTravel+=angle;maxRate=std::max(maxRate,v.length());maxAcceleration=std::max(maxAcceleration,a.length());maxJerk=std::max(maxJerk,j.length());}
                velocity[i]=v;acceleration[i]=a;
            }
        }
        if(measured){++frames;if(surface.contactCount>=2)++contactFrames;maxReach=std::max(maxReach,surface.maxReachError);}
        previous=p;
    }
};
static void stopAndResume(const Library& lib,int fps,Input moving,float seconds) {
    ThreepeatWorld world;world.boxes={{{-10000,0,-5000},{10000,1000,20000}}};
    auto t=attached(world,lib,false);t.cfg.contextActions=t.cfg.automaticClimbActions=false;
    SurfacePose surface;FluidMetrics metrics;const float dt=1.f/fps;
    for(int frame=0;frame<int(seconds*fps);++frame){const auto r=t.update(world,moving,dt,1000);metrics.add(lib,surface.update(lib,world,t,r.motion,dt,1),surface,dt,false);}
    Pose before=metrics.previous;
    for(int frame=0;frame<fps;++frame){const auto r=t.update(world,{},dt,1000);check(r.motion==Motion::hang,"ordinary fixture reaches actual ordinary hang");metrics.add(lib,surface.update(lib,world,t,r.motion,dt,1),surface,dt,frame<fps/2);}
    float returnDistance=0;for(int bone:{28,29,31,32})returnDistance+=angleBetween(before[bone].q,metrics.previous[bone].q);
    const auto stopped=t.position;
    const auto resumed=t.update(world,moving,dt,1000);const auto p=surface.update(lib,world,t,resumed.motion,dt,1);
    check(!resumed.released&&(t.position-stopped).length()>.1f,"stop remains immediately interruptible by movement");metrics.add(lib,p,surface,dt,false);
    std::cout<<"STOP fps="<<fps<<" dir="<<moving.x<<','<<moving.y<<" at="<<seconds<<" travel="<<metrics.armTravel<<" restDelta="<<returnDistance<<" maxRate="<<metrics.maxRate<<" accel="<<metrics.maxAcceleration<<" jerk="<<metrics.maxJerk<<" contacts="<<metrics.contactFrames<<'/'<<metrics.frames<<" reach="<<metrics.maxReach<<'\n';
}
static void steeredBridge(const Library& lib,int fps) {
    ThreepeatWorld world;world.boxes={{{-10000,0,-5000},{10000,1000,20000}}};
    auto t=attached(world,lib,false);t.cfg.contextActions=t.cfg.automaticClimbActions=false;
    SurfacePose surface;FluidMetrics metrics;const float dt=1.f/fps;
    for(int frame=0;frame<fps;++frame){const auto r=t.update(world,{0,1},dt,1000);metrics.add(lib,surface.update(lib,world,t,r.motion,dt,1),surface,dt,false);}
    float oldBlend=0;bool began=false,steered=false;
    for(int frame=0;frame<fps;++frame){Input input{frame>fps/20?1.f:0.f,1};input.run=true;const auto r=t.update(world,input,dt,1000);metrics.add(lib,surface.update(lib,world,t,r.motion,dt,1),surface,dt,frame<fps/2);
        if(frame==0){check(surface.bridgeMotion()!=Motion::none,"climb to run really starts a captured bridge");began=true;}
        if(frame==fps/20+1){check(surface.blendProgress()>oldBlend,"steering cannot restart the ongoing bridge clock");check(surface.bridgeMotion()!=Motion::none,"steering retains authored bridge");steered=true;}
        oldBlend=surface.blendProgress();
    }
    check(began&&steered&&surface.blendProgress()==1,"bridge finishes despite early steering");
    std::cout<<"BRIDGE fps="<<fps<<" travel="<<metrics.armTravel<<" maxRate="<<metrics.maxRate<<" accel="<<metrics.maxAcceleration<<" jerk="<<metrics.maxJerk<<'\n';
}
static void diagonalCapturedWall(const Library& lib,int fps,int side,float scale) {
    ThreepeatWorld world;world.boxes={{{-10000,0,-5000},{10000,1000,20000}}};
    auto t=attached(world,lib);t.cfg.surfaceActionVariants=t.cfg.automaticClimbActions=true;
    t.cfg.contextScale=scale;
    t.cfg.autoActionMinSeconds=1;t.cfg.autoActionMaxSeconds=1.6f;
    SurfacePose surface;Pose previous;const float dt=1.f/fps;bool hopped=false,returned=false,landing=false;unsigned ordinaryFrames=0;float maximumPalm=0,minimumFinger=100,rise=0;
    auto tick=[&](Input input){
        const auto r=t.update(world,input,dt,1000);check(t.active()&&!r.released,"diagonal supported route stays attached");
        const auto p=surface.update(lib,world,t,r.motion,dt,scale),body=lib.world(p);
        auto shownPoint=[&](Vec v){return t.position+Vec{-t.normal.y,t.normal.x,0}*(v.x*scale)-t.normal*(v.y*scale)+Vec{0,0,v.z*scale};};
        for(int hand=0;hand<2;++hand)check(lib.armBendValid(p,hand),"diagonal captured elbow stays anatomical");
        if(!previous.empty())for(std::size_t b=0;b<p.size();++b)check(angleBetween(previous[b].q,p[b].q)<=12.566371f*dt+.015f,"all diagonal captured bones retain original rate bound");
        if(threepeatHop(r.motion)) {
            hopped=true;rise=std::max(rise,t.edgeHand(0,true).z-t.edgeHand(0,false).z);
            for(int hand=0;hand<2;++hand) {
                const int wrist=hand?39:38,elbow=hand?32:29,middle=hand?88:73;
                check(std::acos(std::clamp((body[wrist].t-body[elbow].t).unit().dot((body[middle].t-body[wrist].t).unit()),-1.f,1.f))<=1.658064f,"diagonal capture keeps95degree wrist bound");
                const bool left=r.motion==Motion::contextHopLeft;
                const float a=threepeatSourceWeight(left,hand,t.actionProgress()),b=threepeatTargetWeight(left,hand,t.actionProgress());
                if(std::max(a,b)>.95f) {
                    const Vec hit=t.edgeHand(hand,b>a),normal=t.edgeContactNormal(hand,b>a);
                    const float error=(shownPoint(lib.palm(body,hand))-hit-normal*(capturedWallPalmOffset*scale)).length();
                    if(error>maximumPalm){maximumPalm=error;if(error>4.9f)std::cout<<"DIAG_WORST fps="<<fps<<" side="<<side<<" phase="<<t.actionProgress()<<" hand="<<hand<<" load="<<a<<','<<b<<" error="<<error<<'\n';}
                    for(int bone=hand?82:67;bone<(hand?97:82);++bone)minimumFinger=std::min(minimumFinger,(shownPoint(body[bone].t)-hit).dot(normal));
                    for(int digit=0;digit<5;++digit){const int end=(hand?82:67)+digit*3+2;const Vec tip=body[end].t+body[end].q.rotate({0,0,lib.rest[end].t.length()*.75f});minimumFinger=std::min(minimumFinger,(shownPoint(tip)-hit).dot(normal));}
                }
            }
        }else if(hopped&&r.motion==Motion::contextHang)landing=true;
        if(hopped&&r.motion==Motion::hang&&!t.preparingEdge()&&t.state==State::wall) {
            ++ordinaryFrames;returned=ordinaryFrames>=unsigned(fps/2);
            if(ordinaryFrames>=unsigned(fps/3))for(int hand=0;hand<2;++hand) {
                const auto palm=shownPoint(lib.palm(body,hand));
                const auto hit=world.ray(palm+t.normal*(14*scale),palm-t.normal*(20*scale));
                check(hit&&hit->climbable,"settled ordinary return retains both real palm supports");
                check((palm-hit->point-hit->normal.unit()*(.8f*scale)).length()/scale<5.001f,
                    "ordinary return keeps the original five-unit palm envelope after the short captured landing");
            }
        }else ordinaryFrames=0;
        previous=p;
    };
    for(int i=0;i<fps;++i)tick({});
    for(int i=0;i<fps*6&&!hopped;++i)tick({float(side),1});
    for(int i=0;i<fps*3&&!returned;++i)tick({});
    std::cout<<"DIAGONAL fps="<<fps<<" side="<<side<<" scale="<<scale<<" rise="<<rise<<" palm="<<maximumPalm<<" finger="<<minimumFinger<<'\n';
    check(hopped&&landing&&returned&&rise>31.9f*scale&&rise<=32.01f*scale,"W plus side exercises full32 rise, brief39 landing and complete ordinary-hang recovery");
    check(maximumPalm<=5.001f,"diagonal captured palms retain original five-unit loaded envelope");
    check(minimumFinger>=-.15f,"loaded diagonal fingers do not enter the wall");
}
int main(int argc,char**argv){try{
    check(argc==2,"supply actual licensed motion file");Library lib;check(lib.load(argv[1]),"load unchanged 42 motions");
    for(int fps:{30,60,120}) {
        for(Input input:{Input{0,1},Input{0,-1},Input{1,0},Input{-1,0}})for(float duration:{.35f,.7f,1.1f})stopAndResume(lib,fps,input,duration);
        steeredBridge(lib,fps);
        for(int side:{-1,1})for(float scale:{1.f,1.03f})diagonalCapturedWall(lib,fps,side,scale);
    }
    std::cout<<"PASS fluidity complete-pose anatomy and immediately interruptible ordinary stops\n";return 0;
}catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
