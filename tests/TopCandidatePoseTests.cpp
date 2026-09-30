#ifdef _WIN32
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#undef far
#undef near
#endif
#include "Pose.h"
#define main topCandidateEntityMain
#include "TopCandidateSearchTests.cpp"
#undef main

static float poseDepth(const TopWorld& world,Vec point){point=world.local(point);float depth=0;
    for(const auto& solid:world.solids){float minimum=10000;for(const auto& plane:solid)minimum=std::min(minimum,(plane.d-point.dot(plane.n))/plane.n.length());depth=std::max(depth,minimum);}return depth;
}
static bool poseCase(const Library& lib,bool mirror,float yaw,bool distant,int fps,float scale){
    TopWorld world(mirror);world.yaw=yaw;if(distant)world.origin={134999.047f,41413.3164f,-11817.2373f};auto t=actor(world);check(lib.configureThreepeat(t.cfg),"actual runtime HKX mantle configuration");t.cfg.threepeatAnimations=true;t.cfg.contextScale=scale;
    SurfacePose surface;Pose previous;std::array<Vec,4> previousEnds{};Motion prior=Motion::none;const float dt=1.f/fps;
    float angle=0,speed=0,wrist=0,finger=0,foot=0,palm=0,axis=0;unsigned bends=0,corePeak=0,posePeak=0;bool selected=false,completed=false,aborted=false;int worstAxis=-1;
    auto point=[&](Vec p){return t.position+(Vec{-t.normal.y,t.normal.x,0}*p.x-t.normal*p.y+Vec{0,0,p.z})*scale;};
    for(int frame=-10;frame<fps*5&&t.active();++frame){world.calls=0;Result result;if(frame<0)result.motion=Motion::hang;else result=t.update(world,{0,1,false,true},dt,1000);corePeak=std::max(corePeak,world.calls);
        const Vec root=t.position;world.calls=0;auto pose=surface.update(lib,world,t,result.motion,dt,scale);posePeak=std::max(posePeak,world.calls);check((t.position-root).length()==0,"surface output cannot move the collision-controlled root");check(pose.size()==99,"all 99 canonical bones are retained");
        auto body=lib.world(pose);std::array<Vec,4> ends{};
        for(std::size_t bone=0;bone<pose.size();++bone){check(pose[bone].t.finite()&&pose[bone].s.finite()&&std::abs(pose[bone].q.dot(pose[bone].q)-1)<.002f,"full finite normalized pose");if(bone!=0&&bone!=4)check(std::abs(pose[bone].t.length()-lib.rest[bone].t.length())<.002f,"every skeletal bone retains its fixed length");if(!previous.empty())angle=std::max(angle,angleBetween(previous[bone].q,pose[bone].q)-12.566371f*dt);}
        for(unsigned e=0;e<4;++e){ends[e]=point(body[std::array{8,11,38,39}[e]].t);if(!previous.empty())speed=std::max(speed,(ends[e]-previousEnds[e]).length()-(result.motion!=prior?600.f:750.f)*dt*scale);}
        for(int hand=0;hand<2;++hand)if(!lib.armBendValid(pose,hand))++bends;
        if(result.motion==Motion::contextMantle){selected=true;
            for(int hand=0;hand<2;++hand){int elbow=hand?32:29,wr=hand?39:38,middle=hand?88:73;wrist=std::max(wrist,std::acos(std::clamp((body[wr].t-body[elbow].t).unit().dot((body[middle].t-body[wr].t).unit()),-1.f,1.f)));
                if(t.topPreparation()>=.99f&&t.topHandWeight(hand,surface.sampledPhase())>.95f)palm=std::max(palm,(point(lib.palm(body,hand))-(t.topHand(hand)+t.topHandNormal(hand)*(.8f*scale))).length());
                if(t.topPreparation()<.75f||t.topHandWeight(hand,surface.sampledPhase())<=.95f)continue;
                for(int digit=0;digit<5;++digit)for(int joint=0;joint<4;++joint){int bone=(hand?82:67)+digit*3+std::min(joint,2);Vec q=body[bone].t;if(joint==3)q=q+body[bone].q.rotate({0,0,lib.rest[bone].t.length()*.75f});finger=std::max(finger,poseDepth(world,point(q)));}
            }
            for(int bone:{8,11,50,51})foot=std::max(foot,poseDepth(world,point(body[bone].t)));
            if(t.topPreparation()>=.99f){auto source=lib.sample(Motion::contextMantle,surface.sampledPhase());for(int bone:{69,72,75,78,81,84,87,90,93,96}){const float delta=angleBetween(pose[bone].q,source[bone].q);if(delta>axis){axis=delta;worstAxis=bone;}const float extension=lib.rest[bone].t.length()*.75f;const Vec tip=body[bone].q.rotate({0,0,extension});check(tip.finite()&&std::abs(tip.length()-extension)<.002f,"canonical local Z terminal axis keeps its real extension length");}}
        }
        completed|=result.completed;aborted|=result.released&&!result.completed;previous=pose;previousEnds=ends;prior=result.motion;
    }
    bool pass=selected&&completed&&!aborted&&bends==0&&angle<=.006f&&speed<=.03f&&wrist<=1.659063f&&palm<5.f*scale&&finger<=.15001f*scale&&foot<=.15001f*scale&&axis<=.7853982f;
    std::cout<<"top-candidate-pose mirror="<<mirror<<" yaw="<<yaw<<" distant="<<distant<<" fps="<<fps<<" scale="<<scale<<" completed="<<completed<<" angleExcess="<<angle<<" endpointExcess="<<speed<<" wrist="<<wrist<<" illegalElbows="<<bends<<" palm="<<palm<<" finger="<<finger<<" foot="<<foot<<" terminalSwing="<<axis<<" axisBone="<<worstAxis<<" coreRayPeak="<<corePeak<<" poseRayPeak="<<posePeak<<" pass="<<pass<<'\n';return pass;
}
int main(int argc,char** argv)try{
#ifdef _WIN32
    SetErrorMode(SEM_FAILCRITICALERRORS|SEM_NOGPFAULTERRORBOX);
#endif
    check(argc==2,"runtime HKX manifest argument");Library lib;check(lib.load(argv[1]),"complete licensed real HKX pack loads");unsigned failures=0,total=0;
    for(bool mirror:{false,true})for(float yaw:{0.f,.73f})for(bool distant:{false,true})for(int fps:{30,60,120})for(float scale:{1.f,1.03f}){++total;failures+=!poseCase(lib,mirror,yaw,distant,fps,scale);}
    std::cout<<"top-candidate-pose total="<<total<<" failures="<<failures<<'\n';return failures?1:0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
