#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#undef near
#undef far
#endif
#include "Pose.h"
#include "Controls.h"
#include <iostream>
#include <stdexcept>
#include <filesystem>
#include <fstream>
using namespace fc;
void check(bool value,const char* why){if(!value)throw std::runtime_error(why);}
struct Wall:World {
    bool rearObstacle{},farRear{},ceiling{},sideObstacle{},missingWall{};
    float rearY=-175;
    std::optional<Hit> ray(Vec a,Vec b) override {
        if(farRear&&a.y>rearY&&b.y<=rearY)return Hit{a+(b-a)*((rearY-a.y)/(b.y-a.y)),{0,1,0},true};
        if(ceiling&&a.z<430&&b.z>=430)return Hit{a+(b-a)*((430-a.z)/(b.z-a.z)),{0,0,-1},true};
        if(sideObstacle&&a.x<65&&b.x>=65)return Hit{a+(b-a)*((65-a.x)/(b.x-a.x)),{-1,0,0},true};
        if(rearObstacle&&a.y> -74&&b.y<=-74) {
            const auto p=a+(b-a)*((-74-a.y)/(b.y-a.y));
            if(p.z>210&&p.z<390)return Hit{p,{0,1,0},true};
        }
        if(!missingWall&&a.y<0&&b.y>=0)return Hit{a+(b-a)*(-a.y/(b.y-a.y)),{0,-1,0},true};
        return {};
    }
};
struct UnevenTop:World {
    bool missingLeft{};float missingLeftX=28,missingLeftY=25;
    std::optional<Hit> ray(Vec a,Vec b) override {
        std::optional<Hit> best;float nearest=2;
        const auto height=[](float x){return 120+.18f*x;};
        if(a.y<0&&b.y>=0) {
            const float t=-a.y/(b.y-a.y);const auto p=a+(b-a)*t;
            if(p.z<=height(p.x)){nearest=t;best=Hit{p,{0,-1,0},true};}
        }
        const float from=a.z-height(a.x),to=b.z-height(b.x);
        if(from>0&&to<=0) {
            const float t=from/(from-to);const auto p=a+(b-a)*t;
            if(t<nearest&&p.y>=0&&!(missingLeft&&p.x<missingLeftX&&p.y<missingLeftY))best=Hit{p,Vec{-.18f,0,1}.unit(),true};
        }
        return best;
    }
};

static void authoredRunLoops(const Library& lib) {
    for(Motion motion:{Motion::runUp,Motion::runLeft,Motion::runRight,
            Motion::runDiagonalLeft,Motion::runDiagonalRight}) {
        const auto& clip=lib.clip(motion);
        check(clip.frames.size()>12&&clip.stride>10,"a wall-run loop needs a real advancing gait");
        const auto& first=clip.frames.front();const auto& last=clip.frames.back();
        for(std::size_t bone=0;bone<first.size();++bone) {
            check((first[bone].t-last[bone].t).length()<.002f,"all exported run-loop translations close exactly");
            check(angleBetween(first[bone].q,last[bone].q)<.003f,"all exported run-loop rotations close exactly");
        }
        float worstClosure=0,worstVelocityKink=0,largestAuthoredVelocityChange=0;
        const float frameSeconds=clip.seconds/float(clip.frames.size()-1);
        for(int bone:{38,39,8,11}) {
            std::vector<Vec> points;points.reserve(clip.frames.size());
            for(const auto& p:clip.frames)points.push_back(lib.world(p)[bone].t);
            worstClosure=std::max(worstClosure,(points.front()-points.back()).length());
            const Vec outgoing=(points[1]-points[0])/frameSeconds;
            const Vec incoming=(points.back()-points[points.size()-2])/frameSeconds;
            worstVelocityKink=std::max(worstVelocityKink,(outgoing-incoming).length());
            for(std::size_t frame=2;frame<points.size();++frame) {
                const Vec before=(points[frame-1]-points[frame-2])/frameSeconds;
                const Vec after=(points[frame]-points[frame-1])/frameSeconds;
                largestAuthoredVelocityChange=std::max(largestAuthoredVelocityChange,(after-before).length());
            }
        }
        std::cout<<"authored loop "<<int(motion)<<" frames="<<clip.frames.size()<<" closure="<<worstClosure
            <<" seamVelocityChange="<<worstVelocityKink<<" internalVelocityChange="<<largestAuthoredVelocityChange<<'\n';
        check(worstClosure<.01f,"exported wrist and ankle positions close before runtime correction");
        check(worstVelocityKink<largestAuthoredVelocityChange*1.5f+2.f,
            "a loop boundary cannot introduce an acceleration spike absent from the recorded gait");
    }
}
static Motion expectedRun(Vec direction) {
    if(direction.y<-.1f)return std::abs(direction.y)>std::abs(direction.x)?Motion::down:direction.x<0?Motion::left:Motion::right;
    if(std::abs(direction.x)<.1f)return Motion::runUp;
    if(std::abs(direction.y)<.1f)return direction.x<0?Motion::runLeft:Motion::runRight;
    return direction.x<0?Motion::runDiagonalLeft:Motion::runDiagonalRight;
}
static Vec actorPoint(Vec point,Vec position,Vec normal,float scale) {
    const Vec right{-normal.y,normal.x,0};
    return position+(right*point.x-normal*point.y+Vec{0,0,point.z})*scale;
}
static void longRunCycles(const Library& lib,Wall& wall) {
    auto palmPlane=[](const Pose& world,int hand) {
        const int finger=hand==0?67:82,wrist=hand==0?38:39;
        return (world[finger+3].t-world[finger+12].t).cross(world[finger+6].t-world[wrist].t).unit();
    };
    const auto reference=lib.world(lib.sample(Motion::ledgeCatch,.25f));
    for(Vec direction:{Vec{0,1,0},Vec{-1,0,0},Vec{1,0,0},Vec{-1,1,0},Vec{1,1,0},
            Vec{0,-1,0},Vec{-1,-1,0},Vec{1,-1,0}}) {
        Traversal t;t.cfg.approachSeconds=0;SurfacePose animator;
        check(t.attach(wall,{0,-37,2000},{0,1,0},100),"long cycle attach");
        Pose previous;int animated=0,longestFreeze=0,freeze=0;float travelled=0,maxStep=0,torsoUp=0,supportError=0,palmError=0;int torsoSamples=0,supportSamples=0,palmSamples=0;
        const auto& clip=lib.clip(expectedRun(direction));
        const bool requestedRun=direction.y>=-.1f;
        const float speed=requestedRun?t.cfg.runSpeed:std::min(t.cfg.downSpeed,t.cfg.sideSpeed);
        const float seconds=std::max(6.f,5.f*clip.stride/speed+1.f),dt=1.f/60;
        const int frames=int(std::ceil(seconds/dt));
        for(int frame=0;frame<frames;++frame) {
            const auto before=t.position;
            const auto r=t.update(wall,{direction.x,direction.y,false,false,false,false,true},dt,100);
            const auto p=animator.update(lib,wall,t,r.motion,dt,1);
            check(r.motion==expectedRun(direction)&&t.active(),"all eight directions stay attached, with downward input selecting ordinary climbing");
            if(!requestedRun)check(!t.wallRunning()&&r.staminaCost<=t.cfg.drain*dt+.0001f&&
                (t.position-before).length()<=std::max(t.cfg.downSpeed,t.cfg.sideSpeed)*dt+.01f,"Shift plus down cannot turn into a backwards sprint or charge running stamina");
            travelled+=(t.position-before).length();
            if(frame>60&&!previous.empty()) {
                float movement=0;
                for(int bone:{6,7,9,10}) {
                    const float angle=angleBetween(previous[bone].q,p[bone].q);
                    movement+=angle;maxStep=std::max(maxStep,angle);
                }
                if(movement>.006f){++animated;freeze=0;}else longestFreeze=std::max(longestFreeze,++freeze);
                const auto w=lib.world(p);torsoUp+=(w[36].t-w[4].t).unit().z;++torsoSamples;
                if(requestedRun&&animator.runFootContacts>0) {
                    float closest=1e9f;
                    for(int toe:{50,51}) {
                        const auto point=actorPoint(w[toe].t,t.position,t.normal,1);
                        closest=std::min(closest,std::abs(point.dot(t.normal)-3.f));
                    }
                    supportError=std::max(supportError,closest);++supportSamples;
                }
                if(requestedRun&&std::abs(direction.x)>.1f&&animator.sidePalmContact) {
                    const auto frame=sideRunFrame(t.surfaceNormal.z,direction);
                    const auto palm=actorPoint(lib.palm(w,frame.innerHand),t.position,t.normal,1);
                    palmError=std::max(palmError,std::abs(palm.dot(t.normal)-1.2f));++palmSamples;
                    const float sign=palmPlane(reference,frame.innerHand).dot({0,1,0})>0?1.f:-1.f;
                    check((palmPlane(w,frame.innerHand)*sign).dot(frame.normal*-1)>.75f,
                        "the final displayed hand presents the anatomical palm, not its back, after all pose blending");
                    const int elbow=frame.innerHand==0?29:32;
                    check(actorPoint(w[elbow].t,t.position,t.normal,1).dot(t.normal)>0,
                        "the final supporting elbow remains outside the wall");
                }
            }
            previous=p;
        }
        std::cout<<"long run direction="<<direction.x<<','<<direction.y<<" cycles="<<travelled/clip.stride
            <<" animated="<<animated<<" frozenFrames="<<longestFreeze<<" jointStep="<<maxStep<<" torsoUp="<<torsoUp/torsoSamples
            <<" plantedToeError="<<supportError<<" plantedSamples="<<supportSamples<<" palmError="<<palmError<<" palmSamples="<<palmSamples<<'\n';
        check(travelled/clip.stride>4.5f,"exercise repeated wraps rather than just the first launch");
        check(animated>(frames-61)*.65f&&longestFreeze<20,"wall feet cannot freeze or remain in a reference pose during sustained movement");
        check(maxStep<.70f,"repeated run-loop wraps must not snap a leg");
        check(requestedRun&&std::abs(direction.x)<.1f?std::abs(torsoUp/torsoSamples)<.6f:torsoUp/torsoSamples>.55f,
            "side and diagonal runs keep their head above the pelvis while banking; only upward running treats the wall as ground");
        if(requestedRun&&std::abs(direction.x)>.1f)check(palmError<5.f,
            "any accepted real palm contact remains close; a back-facing source hand must be allowed to keep its motion");
        if(requestedRun)check(supportSamples>20&&supportError<3.f,
            "actual running toe support stays near the three-unit collision offset throughout repeated cycles");
    }
}
static void wallRunTransfers(const Library& lib,Wall& wall) {
    struct Segment {float seconds,x,y;bool run;};

    constexpr Segment segments[]={
        {.8f,0,0,false},{.8f,0,1,false},{1.2f,0,1,true},
        {1.0f,-1,1,true},{1.0f,-1,0,true},{1.0f,1,0,true},{1.0f,1,1,true},
        {.8f,1,1,false},{.8f,-1,0,false},{1.0f,-1,0,true},
        {1.0f,0,-1,true},{1.0f,-1,-1,true},{1.0f,1,-1,true},
        {.8f,0,-1,false},{.20f,-1,1,true},{.20f,1,1,true},{.20f,0,1,true},
        {.8f,1,0,false},{1.5f,0,0,true},{.8f,0,0,false}};
    for(float dt:{1.f/60,1.f/30})for(float scale:{.85f,1.15f}) {
        Traversal t;t.cfg.approachSeconds=0;SurfacePose animator;
        check(t.attach(wall,{0,-37,2000},{0,1,0},100),"direction transfer attach");
        Pose previous,previousWorld;Vec previousPosition{},previousNormal{};Motion previousMotion=Motion::none;
        float maxEndpointSpeed=0,maxArmSpeed=0,worstRatio=0,restDrift=0;int checkedFrames=0,worstBone=0,worstFrom=0,worstTo=0;
        for(const auto& segment:segments) {
            const int frames=int(std::ceil(segment.seconds/dt));
            for(int frame=0;frame<frames;++frame) {
                const auto r=t.update(wall,{segment.x,segment.y,false,false,false,false,segment.run},dt,100);
                const auto p=animator.update(lib,wall,t,r.motion,dt,scale),w=lib.world(p);
                const bool moving=std::abs(segment.x)+std::abs(segment.y)>.1f;
                check(t.active()&&!r.released,"changing run direction or gait must not drop a supported actor");
                if(segment.run&&moving)check(r.motion==expectedRun({segment.x,segment.y,0}),"direction changes select the specialized family");
                if(!moving)check(!runMotion(r.motion)&&r.staminaCost==0,"holding Shift at rest retains a free stable hang");
                for(const auto& transform:p)check(transform.t.finite()&&std::abs(transform.q.dot(transform.q)-1)<.002f,
                    "rapid mode/direction changes keep finite normalized poses");
                if(!previous.empty()) {
                    const float limit=(runMotion(r.motion)||runMotion(previousMotion)?1100.f:600.f)*dt*scale;
                    for(int bone:{38,39,8,11}) {
                        const auto point=actorPoint(w[bone].t,t.position,t.normal,scale);
                        const auto old=actorPoint(previousWorld[bone].t,previousPosition,previousNormal,scale);
                        const float distance=(point-old).length();
                        maxEndpointSpeed=std::max(maxEndpointSpeed,distance/(dt*scale));
                        const float ratio=distance/(limit+.6f*scale);
                        if(ratio>worstRatio){worstRatio=ratio;worstBone=bone;worstFrom=int(previousMotion);worstTo=int(r.motion);}
                        if(!moving&&frame*dt>1.f)restDrift=std::max(restDrift,distance);
                    }
                    for(int bone:{28,29,31,32,38,39}) {
                        const float angle=angleBetween(previous[bone].q,p[bone].q);
                        maxArmSpeed=std::max(maxArmSpeed,angle/dt);
                        const float angularLimit=(runMotion(r.motion)?18.849556f:12.566371f)*dt+.004f;
                        if(angle>=angularLimit)std::cerr<<"arm snap fps="<<1/dt<<" scale="<<scale<<" bone="<<bone<<" from="<<int(previousMotion)<<" to="<<int(r.motion)<<" angle="<<angle<<'\n';
                        check(angle<angularLimit,"the complete pose angular budget applies consistently to arms and wrists");
                    }
                    ++checkedFrames;
                }
                previous=p;previousWorld=w;previousPosition=t.position;previousNormal=t.normal;previousMotion=r.motion;
            }
        }
        std::cout<<"transfers fps="<<1/dt<<" scale="<<scale<<" endpointSpeed="<<maxEndpointSpeed
            <<" armSpeed="<<maxArmSpeed<<" endpointLimitRatio="<<worstRatio<<" bone="<<worstBone<<" from="<<worstFrom<<" to="<<worstTo<<" restDrift="<<restDrift<<'\n';
        check(checkedFrames>400&&worstRatio<=1,"world wrists/ankles obey movement limits through every transition");
        check(restDrift<.02f,"a settled hang after wall running must not buzz or drift");
    }
}
struct NoPoseContacts:World {
    std::optional<Hit> ray(Vec,Vec) override {return {};}
};

static void settledWallRunStops(const Library& lib,Wall& wall) {
    NoPoseContacts noContacts;float worstOneAngle=0,worstTwoAngle=0,worstTranslation=0,worstDrift=0,worstPalm=0;
    const Vec directions[]={{0,1,0},{1,0,0},{-1,1,0},{1,1,0}};
    auto exercise=[&](Vec direction,int stopPhase,bool rapid) {
        Traversal t;t.cfg.approachSeconds=0;t.cfg.gap=37;t.cfg.radius=31;
        SurfacePose isolated,contact;Pose actual,plain;
        check(t.attach(wall,{0,-37,2000},{0,1,0},100),"settling stop attach");
        auto tick=[&](Input input) {
            const auto r=t.update(wall,input,1.f/60,100);
            check(t.active(),"settling fixture keeps a real supporting wall");
            plain=isolated.update(lib,noContacts,t,r.motion,1.f/60,1);
            actual=contact.update(lib,wall,t,r.motion,1.f/60,1);
            for(const auto& tr:actual)check(tr.t.finite()&&std::abs(tr.q.dot(tr.q)-1)<.002f,"rapid control changes keep finite anatomical transforms");
            return r;
        };
        for(int frame=0;frame<30;++frame)tick({0,1});
        if(rapid) {
            std::uint32_t seed=0x63a12u;Input input{};
            for(int frame=0;frame<180;++frame) {
                if(frame%5==0) {
                    seed=1664525u*seed+1013904223u;
                    const auto d=directions[(seed>>16)%4];input={d.x,d.y,false,false,false,false,(seed&1)!=0};
                }
                tick(input);
            }
        }
        for(int frame=0;frame<45+stopPhase*5;++frame)tick({direction.x,direction.y,false,false,false,false,true});

        SurfacePose reference;const auto target=reference.update(lib,noContacts,t,Motion::hang,1.f/60,1);
        const auto fixedPosition=t.position;Pose previousWorld;float caseOne=0,caseTwo=0,caseTranslation=0,caseDrift=0,casePalm=0;
        for(int frame=0;frame<120;++frame) {
            const auto r=tick({});check(r.motion==Motion::hang&&(t.position-fixedPosition).length()==0,"rest has no controller drift");
            if(frame==59||frame==119)for(std::size_t bone=0;bone<plain.size();++bone) {
                const float error=angleBetween(plain[bone].q,target[bone].q);
                if(frame==59)caseOne=std::max(caseOne,error);else caseTwo=std::max(caseTwo,error);
                caseTranslation=std::max(caseTranslation,(plain[bone].t-target[bone].t).length());
            }
            const auto worldPose=lib.world(actual);
            if(frame>=60&&!previousWorld.empty()) {
                for(int bone:{38,39,8,11})caseDrift=std::max(caseDrift,(worldPose[bone].t-previousWorld[bone].t).length());
                for(int hand=0;hand<2;++hand) {
                    const auto palm=actorPoint(lib.palm(worldPose,hand),t.position,t.normal,1);
                    casePalm=std::max(casePalm,std::abs(palm.y));
                }
            }
            previousWorld=worldPose;
        }
        worstOneAngle=std::max(worstOneAngle,caseOne);worstTwoAngle=std::max(worstTwoAngle,caseTwo);
        worstTranslation=std::max(worstTranslation,caseTranslation);worstDrift=std::max(worstDrift,caseDrift);worstPalm=std::max(worstPalm,casePalm);
        if(caseOne>=.035f||caseTwo>=.035f||caseTranslation>=.05f||caseDrift>=.1f||casePalm>=12)
            std::cerr<<"stop convergence direction="<<direction.x<<','<<direction.y<<" phase="<<stopPhase<<" rapid="<<rapid
                <<" oneSecond="<<caseOne<<" twoSeconds="<<caseTwo<<" translation="<<caseTranslation<<" drift="<<caseDrift<<" palm="<<casePalm<<'\n';
        check(caseOne<.035f&&caseTwo<.035f&&caseTranslation<.05f,"all 99 bones must reach the resting target instead of locking midway through a transition");
        check(caseDrift<.1f&&casePalm<12,"a converged real-wall hang has stable visible palms close to the wall");
    };
    for(Vec direction:directions)for(int stopPhase=0;stopPhase<12;++stopPhase)exercise(direction,stopPhase,false);
    exercise({0,1,0},3,true);

    for(Vec direction:{Vec{0,1,0},Vec{0,-1,0},Vec{-1,0,0},Vec{1,0,0}}) {
        Traversal t;t.cfg.approachSeconds=0;SurfacePose animator,isolated;
        check(t.attach(wall,{0,-37,2000},{0,1,0},100),"climbing stop attach");
        const auto expected=direction.y>0?Motion::up:direction.y<0?Motion::down:direction.x<0?Motion::left:Motion::right;
        for(int frame=0;frame<60;++frame) {
            const auto r=t.update(wall,{direction.x,direction.y},1.f/60,100);
            check(r.motion==expected,"climbing stop fixture exercises each ordinary movement capture");
            animator.update(lib,wall,t,r.motion,1.f/60,1);
            isolated.update(lib,noContacts,t,r.motion,1.f/60,1);
        }
        SurfacePose reference;const auto target=reference.update(lib,noContacts,t,Motion::hang,1.f/60,1);
        Pose heldWorld;float drift=0,palmDistance=0,targetAngle=0,targetTranslation=0;const auto fixed=t.position;
        for(int frame=0;frame<120;++frame) {
            const auto r=t.update(wall,{},1.f/60,100);
            const auto p=animator.update(lib,wall,t,r.motion,1.f/60,1),w=lib.world(p);
            const auto plain=isolated.update(lib,noContacts,t,r.motion,1.f/60,1);
            check(r.motion==Motion::hang&&r.staminaCost==0&&(t.position-fixed).length()==0,"a stopped climbing stroke remains stationary and free of stamina drain");
            if(frame==59)heldWorld=w;
            if(frame>=60) {
                for(int bone:{38,39,8,11})drift=std::max(drift,(w[bone].t-heldWorld[bone].t).length());
                for(int hand=0;hand<2;++hand) {
                    const auto palm=actorPoint(lib.palm(w,hand),t.position,t.normal,1);
                    palmDistance=std::max(palmDistance,std::abs(palm.dot(t.normal)));
                }
                for(std::size_t bone=0;bone<plain.size();++bone) {
                    targetAngle=std::max(targetAngle,angleBetween(plain[bone].q,target[bone].q));
                    targetTranslation=std::max(targetTranslation,(plain[bone].t-target[bone].t).length());
                }
            }
        }
        std::cout<<"climbing stop direction="<<direction.x<<','<<direction.y<<" totalDrift="<<drift<<" bothPalmDistance="<<palmDistance
            <<" targetAngle="<<targetAngle<<" targetTranslation="<<targetTranslation<<'\n';
        check(drift<.25f&&palmDistance<12,"both palms must return near the actual wall and remain stable over the whole second settling interval");
        check(targetAngle<.035f&&targetTranslation<.05f,"ordinary climbing stops must finish their transition instead of preserving a half-finished stroke");
    }
    std::cout<<"settled stops oneSecondAngle="<<worstOneAngle<<" twoSecondsAngle="<<worstTwoAngle
        <<" translation="<<worstTranslation<<" finalDrift="<<worstDrift<<" palmDistance="<<worstPalm<<'\n';
}
static void dumpWallTransfers(const Library& lib,Wall& wall,const std::filesystem::path& path) {
    struct Segment {int frames;float x,y;bool run;};
    constexpr Segment segments[]={{48,0,1,false},{60,1,1,true},{48,1,0,true},
        {60,-1,1,true},{48,-1,0,false},{60,0,0,false}};
    Traversal t;t.cfg.approachSeconds=0;SurfacePose animator;
    check(t.attach(wall,{0,-37,200},{0,1,0},100),"transition preview attach");
    std::ofstream dump(path);dump<<'[';int frame=0;
    for(const auto& segment:segments)for(int sample=0;sample<segment.frames;++sample,++frame) {
        const auto r=t.update(wall,{segment.x,segment.y,false,false,false,false,segment.run},1.f/60,100);
        const auto p=animator.update(lib,wall,t,r.motion,1.f/60,1);
        check(t.active(),"transition preview stays attached");
        if(frame)dump<<',';
        dump<<"{\"position\":["<<t.position.x<<','<<t.position.y<<','<<t.position.z<<"],\"phase\":"<<frame/60.f
            <<",\"motion\":"<<int(r.motion)<<",\"transforms\":[";
        for(std::size_t bone=0;bone<p.size();++bone) {
            const auto& tr=p[bone];if(bone)dump<<',';
            dump<<"{\"t\":["<<tr.t.x<<','<<tr.t.y<<','<<tr.t.z<<"],\"q\":["<<tr.q.x<<','<<tr.q.y<<','<<tr.q.z<<','<<tr.q.w<<"],\"s\":[1,1,1]}";
        }
        dump<<"]}";
    }
    dump<<']';
}

static void runningParkourTransitions(const Library& lib,Wall& wall,const std::filesystem::path&) {
    int cases=0;
    for(int fps:{30,48,60,120})for(bool fancy:{false,true})
    for(Vec direction:{Vec{0,1,0},Vec{-1,0,0},Vec{1,0,0},Vec{-1,1,0},Vec{1,1,0}}) {
        const float dt=1.f/fps;Traversal pressed,control;
        pressed.cfg.approachSeconds=control.cfg.approachSeconds=0;
        pressed.cfg.fancyJumps=control.cfg.fancyJumps=fancy;SurfacePose withSpace,withoutSpace;
        check(pressed.attach(wall,{0,-37,1000},{0,1,0},100)&&control.attach(wall,{0,-37,1000},{0,1,0},100),
            "running Space paired-output attach");
        const Input moving{direction.x,direction.y,false,false,false,false,true};
        Pose previous;int changedGaitFrames=0;
        for(int frame=0;frame<fps*3;++frame) {
            auto input=moving;input.hop=frame>=fps;
            const auto a=pressed.update(wall,input,dt,100),b=control.update(wall,moving,dt,100);
            const auto pa=withSpace.update(lib,wall,pressed,a.motion,dt,1);
            const auto pb=withoutSpace.update(lib,wall,control,b.motion,dt,1);
            check(pressed.active()&&pressed.state!=State::action&&!hopMotion(a.motion)&&runMotion(a.motion),
                "running Space never starts a hop, flight or replacement hang");
            check(a.motion==b.motion&&std::abs(a.staminaCost-b.staminaCost)<.000001f&&
                (pressed.position-control.position).length()<.000001f,
                "suppressed Space has no movement, stamina or mode side effects");
            float gaitChange=0;
            for(std::size_t bone=0;bone<pa.size();++bone) {
                check(pa[bone].t.finite()&&std::isfinite(pa[bone].q.dot(pa[bone].q))&&std::abs(pa[bone].q.dot(pa[bone].q)-1)<.002f,
                    "uninterrupted running remains finite and normalized");
                check((pa[bone].t-pb[bone].t).length()<.000001f&&angleBetween(pa[bone].q,pb[bone].q)<.0001f,
                    "every displayed bone matches the no-Space control throughout repeated presses");
            }
            if(!previous.empty())for(int bone:{6,7,9,10})
                gaitChange=std::max(gaitChange,angleBetween(previous[bone].q,pa[bone].q));
            if(frame>=fps&&gaitChange>.001f)++changedGaitFrames;
            const auto held=withSpace.update(lib,wall,pressed,a.motion,0,1);
            for(std::size_t bone=0;bone<pa.size();++bone)
                check((held[bone].t-pa[bone].t).length()<.000001f&&angleBetween(held[bone].q,pa[bone].q)<.0001f,
                    "duplicate callbacks cannot reset the uninterrupted running clock");
            previous=pa;
        }
        check(changedGaitFrames>fps*1.8f,"pressing Space cannot freeze the captured running cycle");
        ++cases;
    }
    std::cout<<"running Space paired pose sequences="<<cases<<'\n';
}
int main(int argc,char**argv){
#ifdef _WIN32
    SetErrorMode(SEM_FAILCRITICALERRORS|SEM_NOGPFAULTERRORBOX|SEM_NOOPENFILEERRORBOX);
#endif
try{
    check(argc==2,"motion path");Library lib;check(lib.load(argv[1]),"bundled library");Wall wall;
    for(auto direction:{Vec{0,1,0},Vec{-1,0,0},Vec{1,0,0},Vec{-1,1,0},Vec{1,1,0}}) {
        Traversal t;t.cfg.approachSeconds=0;SurfacePose animator;
        check(t.attach(wall,{0,-37,200},{0,1,0},100),"parkour attach");
        std::ofstream runDump(std::filesystem::current_path()/("runtime-run-"+std::to_string(int(direction.x))+"-"+std::to_string(int(direction.y))+".json"));runDump<<'[';bool runComma=false;
        Pose last;float maxStep=0;int animated=0;Motion selected=Motion::none;
        for(int frame=0;frame<210;++frame) {
            const bool run=frame>=45&&frame<150;
            const auto r=t.update(wall,{direction.x,direction.y,false,false,false,false,run},1.f/60,100);
            auto p=animator.update(lib,wall,t,r.motion,1.f/60,1);
            if(run){check(runMotion(r.motion),"Shift selects an included run clip without native pose input");selected=r.motion;}
            if(!last.empty()) {
                float legs=0;
                for(int bone:{6,7,9,10})legs+=angleBetween(last[bone].q,p[bone].q);
                if(run&&legs>.01f)++animated;
                for(int bone:{6,7,9,10,28,29,31,32,38,39})maxStep=std::max(maxStep,angleBetween(last[bone].q,p[bone].q));
            }
            for(auto tr:p)check(tr.t.finite()&&std::abs(tr.q.dot(tr.q)-1)<.002f,"finite normalized parkour pose");
            if(frame>=30&&frame<=205) {
                if(runComma)runDump<<',';runComma=true;
                runDump<<"{\"position\":["<<t.position.x<<','<<t.position.y<<','<<t.position.z<<"],\"phase\":"<<(frame-45)/60.f<<",\"transforms\":[";
                for(int i=0;i<99;++i){const auto& tr=p[i];if(i)runDump<<',';runDump<<"{\"t\":["<<tr.t.x<<','<<tr.t.y<<','<<tr.t.z<<"],\"q\":["<<tr.q.x<<','<<tr.q.y<<','<<tr.q.z<<','<<tr.q.w<<"],\"s\":[1,1,1]}";}
                runDump<<"]}";
            }
            last=p;
        }
        runDump<<']';
        std::cout<<"run "<<int(selected)<<" animated="<<animated<<" maxJointStep="<<maxStep<<'\n';
        check(animated>90,"bundled feet keep cycling throughout the wall run");
        check(maxStep<.70f,"mode switches and gait seams cannot flip a limb in one frame");
        check(t.active(),"run-climb switch retains the wall");
    }
    authoredRunLoops(lib);
    longRunCycles(lib,wall);
    wallRunTransfers(lib,wall);
    settledWallRunStops(lib,wall);
    runningParkourTransitions(lib,wall,std::filesystem::current_path());
    dumpWallTransfers(lib,wall,std::filesystem::current_path()/"runtime-wall-transitions.json");
    {
        Traversal t;t.cfg.approachSeconds=0;check(t.attach(wall,{0,-37,200},{0,1,0},100),"ordinary diagonal hop attach");
        const auto start=t.position;
        auto r=t.update(wall,{1,1,false,false,true},1.f/60,100);
        check(r.motion==Motion::hopRight&&r.staminaCost==15,"ordinary diagonal Space retains its hop and one stamina charge");
        float excursion=0;
        for(int i=0;i<100&&t.state==State::action;++i) {
            r=t.update(wall,{},1.f/60,100);excursion=std::max(excursion,start.y-t.position.y);
        }
        check(excursion>=30,"ordinary hop retains an obvious checked outward arc");
        check(t.active()&&std::abs(t.position.y-start.y)<.01f&&t.position.x>start.x+85&&t.position.z>start.z+50,
            "ordinary diagonal hop returns to a real higher side grab");
    }
    {
        Traversal t;t.cfg.approachSeconds=0;check(t.attach(wall,{0,-37,200},{0,1,0},100),"stationary Shift Space attach");
        const auto r=t.update(wall,wallInput(Keys{false,false,false,false,true,true},true),1.f/60,100);
        check(!hopMotion(r.motion)&&t.state!=State::action&&r.staminaCost==0,
            "Shift plus Space from a resting hang cannot start a removed running jump");
        const auto drop=t.update(wall,wallInput(Keys{false,false,true,false,true,true},true),1.f/60,100);
        check(drop.motion==Motion::dropBack,"S plus Space release keeps priority with Shift held");
    }
    {
        Wall blocked;blocked.rearObstacle=true;Traversal t;t.cfg.approachSeconds=0;
        check(t.attach(blocked,{0,-37,200},{0,1,0},100),"obstructed ordinary takeoff attach");
        auto r=t.update(blocked,{0,0,false,false,true},1.f/60,100);
        check(t.state!=State::action&&r.staminaCost==0,"a blocked ordinary outward arc is rejected before takeoff or stamina cost");
    }

    for(Motion motion:{Motion::kickUp,Motion::kickLeft,Motion::kickRight,Motion::flipUp,Motion::flipLeft,Motion::flipRight}) {
        float inverted=1,legChange=0;Pose previous;
        for(int frame=0;frame<=120;++frame) {
            const auto pose=lib.sample(motion,frame/120.f),worldPose=lib.world(pose);
            for(const auto& tr:pose)check(tr.t.finite()&&std::isfinite(tr.q.dot(tr.q))&&std::abs(tr.q.dot(tr.q)-1)<.002f,
                "retained jump assets remain finite and normalized even though wall-run input does not select them");
            inverted=std::min(inverted,(worldPose[36].t-worldPose[4].t).unit().z);
            if(!previous.empty())for(int bone:{6,7,9,10})legChange+=angleBetween(previous[bone].q,pose[bone].q);
            previous=pose;
        }
        check(legChange>.1f,"retained jump asset has actual captured leg motion");
        if(flipMotion(motion))check(inverted<-.55f,"retained flip source preserves its captured torso inversion");
    }
    {
        Wall corridor;corridor.rearObstacle=true;Traversal t;t.cfg.approachSeconds=0;
        check(t.attach(corridor,{0,-37,200},{0,1,0},100),"narrow runway attach");
        auto r=t.update(corridor,{0,1,false,false,false,false,true},1.f/60,100);
        check(!runMotion(r.motion)&&t.active(),"a runner's outward torso must fit, otherwise retain climbing");
    }
    for(int obstruction=0;obstruction<3;++obstruction) {
        Wall narrow;narrow.farRear=obstruction==0;narrow.ceiling=obstruction==1;narrow.sideObstacle=obstruction==2;narrow.rearY=-235;
        Traversal t;t.cfg.approachSeconds=0;check(t.attach(narrow,{0,-37,200},{0,1,0},100),"restricted running Space attach");
        const auto r=t.update(narrow,{0,1,false,false,true,false,true},1.f/60,100);
        check(!hopMotion(r.motion)&&t.state!=State::action&&t.active()&&r.staminaCost<1,
            "restricted running Space cannot become a kick, flip or ordinary climbing fallback hop");
    }
    {
        Wall rear;rear.farRear=true;rear.rearY=-130;Traversal t;t.cfg.approachSeconds=0;
        check(t.attach(rear,{0,-37,200},{0,1,0},100),"close rear wall permits upright climbing");
        const auto start=t.position;
        const auto ignored=t.update(rear,{0,0,false,false,true,false,true},1.f/60,100);
        check(t.active()&&t.state!=State::action&&!hopMotion(ignored.motion)&&!ignored.released&&
            ignored.staminaCost==0&&(t.position-start).length()<.001f,
            "running Space leaves a stationary hang intact, without action cost");
        const auto resumed=t.update(rear,{0,1},1.f/60,100);
        check(t.active()&&resumed.motion==Motion::up&&t.position.z>start.z&&!t.wallRunning(),
            "ignoring running Space does not freeze ordinary climbing in a narrow passage");
    }
    {
        Wall disappearing;Traversal t;t.cfg.approachSeconds=0;
        check(t.attach(disappearing,{0,-37,200},{0,1,0},100),"lost ordinary-hop landing attach");
        const auto launch=t.update(disappearing,{0,1,false,false,true},1.f/60,100);
        check(launch.motion==Motion::hopUp,"missing destination check starts from an actual ordinary hop");
        disappearing.missingWall=true;
        for(int i=0;i<130&&t.active();++i)t.update(disappearing,{},1.f/60,100);
        check(!t.active(),"a missing destination cannot become an invisible grip at an ordinary-hop landing");
    }
    for(float scale:{.85f,1.f,1.15f}) {
        UnevenTop top;Traversal t;t.cfg.approachSeconds=0;t.cfg.contextScale=scale;SurfacePose animator;
        check(lib.configureThreepeat(t.cfg),"uneven top uses the actual authored contact calibration");
        check(t.attach(top,{0,-37,0},{0,1,0},100),"uneven top attach");
        float worst=0,worstPhase=0,worstDistance=0,worstReach=0;int contacts=0,worstHand=-1;bool done=false;
        Vec worstShoulder{},worstGoal{},worstPalm{},worstCOM{},worstSourceCOM{},worstActor{};
        for(int i=0;i<180&&t.active();++i) {
            auto r=t.update(top,{0,1,false,true},1.f/60,100);
            auto p=animator.update(lib,top,t,r.motion,1.f/60,scale);auto w=lib.world(p);
            if(t.state==State::mantle&&t.progress()>.24f&&t.progress()<.55f) {
                const float sourcePhase=t.topSamplePhase(t.progress());
                for(int hand=0;hand<2;++hand) {
                    if(t.topHandWeight(hand,sourcePhase)<.95f)continue;
                    const auto actual=t.position+lib.palm(w,hand)*scale;
                    auto desired=t.topHand(hand)+t.topHandNormal(hand)*(.8f*scale);
                    if(t.topReplanted(sourcePhase)) {
                        const auto contact=threepeatReplantContact(top,t.topHand(hand),t.normal,t.topReplantOffset(hand),scale);
                        check(contact.has_value(),"every loaded replanted palm requires its measured top support");desired=*contact;
                    }
                    const float error=(actual-desired).length();
                    if(error>worst) {
                        worst=error;worstPhase=t.progress();worstHand=hand;
                        const int shoulder=hand==0?28:31,elbow=hand==0?29:32,wrist=hand==0?38:39;
                        worstShoulder=t.position+w[shoulder].t*scale;
                        worstGoal=desired-(actual-(t.position+w[wrist].t*scale));
                        worstPalm=actual;worstActor=t.position;worstCOM=p[4].t;
                        worstSourceCOM=lib.sample(r.motion,t.progress())[4].t;
                        worstDistance=(worstGoal-worstShoulder).length();
                        worstReach=((w[elbow].t-w[shoulder].t).length()+(w[wrist].t-w[elbow].t).length())*scale;
                    }
                    ++contacts;
                }
            }
            done|=r.completed;
        }
        std::cout<<"uneven top scale="<<scale<<" palmError="<<worst<<" samples="<<contacts<<'\n';
        if(worst>=3) {
            auto point=[](const char* label,Vec v){std::cout<<' '<<label<<"=("<<v.x<<','<<v.y<<','<<v.z<<')';};
            std::cout<<"worst top contact phase="<<worstPhase<<" hand="<<worstHand<<" targetDistance="<<worstDistance<<" armTotal="<<worstReach;
            point("shoulderWorld",worstShoulder);point("wristGoalWorld",worstGoal);point("palmWorld",worstPalm);
            point("actor",worstActor);point("COMLocal",worstCOM);point("sourceCOMLocal",worstSourceCOM);std::cout<<'\n';
        }
        check(done&&contacts>20&&worst<3,"both loaded palms follow separately measured sloping tops at different actor scales");
    }
    {
        UnevenTop top;top.missingLeft=true;top.missingLeftX=-8;top.missingLeftY=35;
        Traversal t;t.cfg.approachSeconds=0;
        check(t.attach(top,{0,-37,0},{0,1,0},100),"partial front hole still permits a real wall attachment");
        const Vec initial=t.position;t.update(top,{0,1,false,true},1.f/60,100);
        check(t.state==State::mantle,"a supported side top remains available beside a partial front hole");
        for(int hand=0;hand<2;++hand){const Vec palm=t.topHand(hand);const auto actual=top.ray(palm+Vec{0,0,4},palm-Vec{0,0,4});
            check(actual&&actual->climbable&&(actual->point-palm).length()<.01f,"both selected side palms remain on separately verified real surfaces");}
        float minimum=10000;bool done=false;Vec previous=initial;
        auto capsuleDistance=[&](Vec feet){const Vec roof=Vec{-.18f,0,1}.unit();const float front=std::max(0.f,-feet.y),above=std::max(0.f,(feet+Vec{0,0,t.cfg.radius}).dot(roof)-120.f*roof.z);return std::sqrt(front*front+above*above);};
        for(int frame=0;frame<180&&t.active();++frame){const auto r=t.update(top,{0,1,false,true},1.f/60,100);check(!r.released||r.completed,"verified partial-hole top cannot drop during its planned route");
            for(int step=0;step<=8;++step)minimum=std::min(minimum,capsuleDistance(previous+(t.position-previous)*(step/8.f)));previous=t.position;done|=r.completed;}
        check(done&&minimum>=t.cfg.radius-.04f,"the complete side top-out capsule stays outside the full sloping solid including between frames");
        std::cout<<"partial-front-hole side top completed="<<done<<" minimumCapsule="<<minimum<<'\n';
    }
    {
        UnevenTop top;top.missingLeft=true;Traversal t;t.cfg.approachSeconds=0;

        for(float x:{-18.f,-14.f,-10.f})for(float y:{7.f,12.f,19.f,24.f})
            check(!top.ray({x,y,145},{x,y,90}),"front lip has no support for any allowed left-palm spacing");
        const float footprint=std::max(12.f,t.cfg.radius*.55f);
        const Vec floor{0,44.1f,120};
        for(Vec offset:{Vec{},Vec{footprint,0,0},Vec{-footprint,0,0},Vec{0,footprint,0},Vec{0,-footprint,0}})
            check(top.ray(floor+offset+Vec{0,0,30},floor+offset-Vec{0,0,30}).has_value(),"missing-hand fixture still has a complete standing footprint");
        check(t.attach(top,{0,-37,0},{0,1,0},100),"incomplete top attach");
        unsigned missingContacts=0;
        for(float x=-72;x<=28;x+=1)for(float y=0;y<=24;y+=1)for(float height:{32.f,t.cfg.chest,t.cfg.grip}){
            const Vec palm{x,y,120+.18f*x},anchor=t.position+Vec{-18,0,height};
            if((palm-anchor).length()>t.cfg.gap+24)continue;
            check(!top.ray(palm+Vec{0,0,4},palm-Vec{0,0,4}),"the complete reachable left-hand envelope lacks actual top support");++missingContacts;}
        check(missingContacts>20,"the unsupported hand fixture covers a substantial reachable surface envelope");
        check(top.ray({34.5f,19.1f,135},{34.5f,19.1f,115}).has_value(),"the physically reachable right-hand top support remains available");
        t.update(top,{0,1,false,true},1.f/60,100);
        check(t.state!=State::mantle&&std::string(t.ledgeReason)=="no solid top under both palms","a central floor alone cannot authorize pulling up on an unsupported hand");
    }
    std::cout<<"PASS: independent parkour, mode changes, outward arcs, preflight obstructions and measured palm contacts\n";
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
