#include "Pose.h"
#include "CornerTestWorld.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
static void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
static void direct(const Library& library,int mode,int fps,bool far,float scale) {
    const float top=mode==2?230.f:300.f,dt=1.f/fps;
    fc_test::CornerWorld world;world.boxes={{{-10000,0,-10000},{10000,1000,top}}};
    if(far){world.origin={134559.219f,36994.7031f,-11691.1309f};world.yaw=.633f;}
    Traversal traversal;traversal.cfg=fc_test::settings();traversal.cfg.runSpeed=379.5f;traversal.cfg.contextScale=scale;
    check(library.configureThreepeat(traversal.cfg),"real HKX metadata configures the climb-up");
    traversal.cfg.threepeatAnimations=true;traversal.cfg.automaticClimbActions=false;
    if(mode==2)traversal.cfg.approachSeconds=.36f;
    check(traversal.attach(world,world.global({0,mode==2?-60.f:-37.f,100}),world.direction({0,1,0}),1000,60,mode==2),"direct mantle begins on an independently intersected solid wall");
    if(mode==2)traversal.entry(Motion::runLaunch,false);
    Input input{0,1};input.run=mode!=0;input.mantle=true;
    SurfacePose surface;Pose previous;Motion prior=Motion::none;
    std::array<Vec,4> oldEndpoints{};
    float palm=0,palmPhase=0,finger=0,fingerPhase=0,angleExcess=0,endpointExcess=0,endpointSpeed=0;
    int endpointFrom=0,endpointTo=0,endpointBone=0;float endpointPhase=0,endpointPreparation=0;
    unsigned mantleFrames=0,preparationFrames=0,fingerSamples=0,runFrames=0,entryFrames=0;
    bool completed=false;Motion source=Motion::none;
    for(int frame=0;frame<fps*6&&traversal.active();++frame) {
        const auto result=traversal.update(world,input,dt,1000);
        check(!result.released||result.completed,"verified direct mantle must finish rather than drop");
        const auto root=traversal.position;
        const auto pose=surface.update(library,world,traversal,result.motion,dt,scale);
        check((traversal.position-root).length()==0,"pose adaptation cannot move the physical root");
        check(pose.size()==99,"mantle transition retains complete skeleton");
        const auto body=library.world(pose);
        const auto point=[&](Vec value){return traversal.position+(Vec{-traversal.normal.y,traversal.normal.x,0}*value.x-traversal.normal*value.y+Vec{0,0,value.z})*scale;};
        for(unsigned bone=0;bone<pose.size();++bone) {
            check(pose[bone].t.finite()&&std::abs(pose[bone].q.dot(pose[bone].q)-1)<.002f,"finite normalized full pose");
            if(bone!=0&&bone!=4)check(std::abs(pose[bone].t.length()-library.rest[bone].t.length())<.002f,"fixed bone lengths throughout the transition");
            if(!previous.empty())angleExcess=std::max(angleExcess,angleBetween(previous[bone].q,pose[bone].q)-(runMotion(result.motion)?18.849556f:12.566371f)*dt);
        }
        for(int hand=0;hand<2;++hand) {
            check(library.armBendValid(pose,hand),"transition keeps the legal elbow bend branch");
            if(result.motion==Motion::contextMantle) {
                const int elbow=hand?32:29,wrist=hand?39:38,middle=hand?88:73;
                const float bend=std::acos(std::clamp((body[wrist].t-body[elbow].t).unit().dot((body[middle].t-body[wrist].t).unit()),-1.f,1.f));
                check(bend<=1.658063f+.001f,"loaded new mantle wrists stay within 95 degrees");
            }
        }
        std::array<Vec,4> endpoints{};
        for(unsigned index=0;index<endpoints.size();++index)endpoints[index]=point(body[std::array{8,11,38,39}[index]].t);
        if(!previous.empty()) {
            const bool quick=runMotion(result.motion)||(result.motion>=Motion::kickUp&&result.motion<=Motion::kickRight)||runMotion(prior);
            const float speed=quick?1100.f:result.motion==Motion::contextMantle?750.f:600.f;
            const float limit=(result.motion!=prior?std::min(speed,runMotion(prior)?1100.f:600.f):speed)*dt*scale;
            for(unsigned index=0;index<endpoints.size();++index) {
                const float step=(endpoints[index]-oldEndpoints[index]).length();endpointSpeed=std::max(endpointSpeed,step/dt);
                if(step-limit>endpointExcess){endpointExcess=step-limit;endpointFrom=int(prior);endpointTo=int(result.motion);endpointBone=std::array{8,11,38,39}[index];endpointPhase=traversal.progress();endpointPreparation=traversal.topPreparation();}
            }
        }
        if(result.motion==Motion::contextMantle) {
            if(!mantleFrames){source=prior;if(mode==0)check(surface.bridgeMotion()!=Motion::runCatch,"ordinary climbing cannot borrow a running catch bridge");}
            ++mantleFrames;
            if(traversal.topPreparation()<1.f)++preparationFrames;
            if(surface.topPalmError>palm){palm=surface.topPalmError;palmPhase=traversal.progress();}
            for(int hand=0;hand<2;++hand) {
                check(std::abs(world.local(traversal.topHand(hand)).z-top)<.05f,"selected hands match the independent solid top");
                if(traversal.topPreparation()>=1.f&&threepeatMantleWeight(hand,traversal.progress(),traversal.cfg.threepeatProfile)>.95f) {
                    const auto measure=[&](Vec value) {
                        const Vec shown=world.local(point(value));++fingerSamples;const auto& box=world.boxes.front();
                        const float depth=std::max(0.f,std::min({shown.x-box.low.x,box.high.x-shown.x,shown.y-box.low.y,box.high.y-shown.y,shown.z-box.low.z,box.high.z-shown.z}));if(depth>finger){finger=depth;fingerPhase=traversal.progress();}
                    };
                    for(int digit=0;digit<5;++digit) {
                        const int start=(hand?82:67)+digit*3;
                        for(int joint=0;joint<3;++joint)measure(body[start+joint].t);
                        const int last=start+2;measure(body[last].t+body[last].q.rotate({0,0,library.rest[last].t.length()*.75f}));
                    }
                }
            }
        }
        runFrames+=runMotion(result.motion);entryFrames+=result.motion==Motion::runLaunch;completed|=result.completed;
        previous=pose;oldEndpoints=endpoints;prior=result.motion;
    }
    std::cout<<"direct mode="<<mode<<" fps="<<fps<<" far="<<far<<" scale="<<scale<<" source="<<int(source)<<" mantle="<<mantleFrames<<" preparation="<<preparationFrames<<" run="<<runFrames<<" entry="<<entryFrames<<" completed="<<completed<<" palm="<<palm<<" palmPhase="<<palmPhase<<" finger="<<finger<<" fingerPhase="<<fingerPhase<<" angleExcess="<<angleExcess<<" endpointExcess="<<endpointExcess<<" endpointSpeed="<<endpointSpeed<<" endpointMotion="<<endpointFrom<<'/'<<endpointTo<<" endpointBone="<<endpointBone<<" endpointPhase="<<endpointPhase<<" endpointPreparation="<<endpointPreparation<<'\n';
    check(mantleFrames>unsigned(fps/2)&&completed&&!traversal.active(),"compatible direct climb or running entry uses the complete new mantle and returns to native control");
    check(traversal.obstacleJumpCount()==0,"direct activation does not require an obstacle jump");
    check(preparationFrames>unsigned(fps/3),"all sources prepare real top contact before physical pull-up");
    if(mode==1)check(runFrames>0&&runMotion(source),"direct running fixture includes an actual incoming wall-run pose");
    if(mode==2)check(entryFrames>0&&source==Motion::runLaunch,"entry fixture hands directly from run-launch into the mantle");
    check(angleExcess<=.006f,"all output bones obey existing per-motion angular speed");
    check(endpointExcess<=.03f,"actual wrist and ankle world speed respects the transition and action budgets");
    check(palm<5*scale,"actual loaded palms remain within five scaled units of their real contact targets");
    check(fingerSamples>unsigned(fps*10)&&finger<=.15001f*scale,"loaded finger joints and terminal tips remain outside the real solid");
}
int main(int argc,char** argv){try{
    check(argc==2,"provide shipped HKX manifest");Library library;check(library.load(argv[1])&&library.hasThreepeat(),"load actual shipped animations");
    unsigned failed=0;
    for(int mode:{0,1,2})for(int fps:{30,60,120})for(bool far:{false,true})for(float scale:{1.f,1.03f})
        try{direct(library,mode,fps,far,scale);}catch(const std::exception& error){++failed;std::cerr<<"FAILED mode="<<mode<<" fps="<<fps<<" far="<<far<<" scale="<<scale<<" "<<error.what()<<'\n';}
    check(failed==0,"all direct mantle pose scenarios must pass");
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}return 0;}
