#include "CornerTestWorld.h"
#include <iostream>
#include <stdexcept>
#include <string>
using namespace fc;
using fc_test::CornerWorld;
static unsigned checks{};
static void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
static CornerWorld world(float recess,bool distant) {
    CornerWorld w;w.boxes={{{-220,0,-600},{220,350,380}},{{-220,recess,380},{220,350,1000}}};
    if(distant){w.origin={132559.219f,32994.7031f,-12691.1309f};w.yaw=.73f;}
    return w;
}
struct Measurements {bool planned{},landed{},resumed{},released{},changed{};unsigned peak{};float rise{};};
static Measurements run(int fps,bool sprint,float recess,bool distant,int mode=0) {
    auto w=world(recess,distant);Traversal t;t.cfg=fc_test::settings();t.cfg.wallRunObstacleJumps=mode!=1;
    check(t.attach(w,w.global({0,-37,250}),w.direction({0,1,0}),1000),"ordinary wall entry attaches to actual lower wall");
    check(t.state==State::wall,"transfer fixture starts directly in ordinary wall state");
    if(mode==3)w.boxes.push_back({{-220,-180,450},{220,350,458}});
    if(mode==6)w.boxes[1].climbable=false;
    const Input input{0,1,false,true,false,false,sprint};
    Measurements found;float landedZ=0;
    for(int frame=0;frame<fps*6&&t.active();++frame) {
        if(found.planned&&!found.changed&&t.actionProgress()>.25f&&(mode==4||mode==5)) {
            if(mode==4)w.boxes.erase(w.boxes.begin()+1);
            else w.boxes.push_back({{-220,-180,450},{220,350,458}});
            found.changed=true;
        }
        w.rays=0;const auto before=t.position;const auto output=t.update(w,input,1.f/fps,mode==2?20.f:1000.f);
        found.peak=std::max(found.peak,w.rays);
        const bool planned=t.state==State::action&&std::string(t.blockedReason).find("checked recessed")!=std::string::npos;
        if(planned&&!found.planned) {
            found.planned=true;
            check(output.motion==Motion::hopUp||output.motion==Motion::kickUp,"recess transfer uses an existing upward jump");
            const auto target=w.local(t.edgeTarget());
            check(std::abs(target.y-(recess-t.cfg.gap))<.05f&&target.z>380,"target root attaches to the actual recessed upper wall");
            for(float x:{-16.f,0.f,16.f}) {
                const auto from=t.edgeTarget()+w.direction({x,0,t.cfg.grip});
                const auto hit=w.ray(from,from+w.direction({0,t.cfg.gap+4,0}));
                check(hit&&hit->climbable&&std::abs(w.local(hit->point).y-recess)<.05f,"planned landing has three real upper-wall hand-support rays");
            }
        }
        check(int(output.motion)>=0&&int(output.motion)<=42,"removed action never appears in traversal output");
        check(!output.completed,"a narrow recessed wall strip is not a standing summit");
        check(w.clearance(t.position,t.cfg)+.05f>=t.cfg.radius,"independent body clearance holds along the full root route");
        found.released|=output.released;
        if(found.changed&&output.released)check((t.position-before).length()<.001f,"changed support or route aborts before applying an unsafe root position");
        if(found.planned&&!output.released&&t.state==State::wall&&!found.landed) {
            found.landed=true;landedZ=w.local(t.position).z;
        }
        if(found.landed){found.rise=w.local(t.position).z-landedZ;if(found.rise>90){found.resumed=true;break;}}
        if(!sprint&&mode==0&&w.local(t.position).z>470) {
            for(float x:{-16.f,0.f,16.f}) {
                const auto from=t.position+w.direction({x,0,t.cfg.grip});
                const auto hit=w.ray(from,from+w.direction({0,t.cfg.gap+20,0}));
                check(hit&&hit->climbable&&std::abs(w.local(hit->point).y-recess)<.05f,
                    "ordinary climb reconnects to the real upper-wall hand supports within the existing clearance margin");
            }
            found.resumed=true;break;
        }
    }
    if(mode==0) {
        if((sprint&&(!found.planned||!found.landed))||!found.resumed||found.released)std::cerr<<"case fps="<<fps<<" run="<<sprint<<" recess="<<recess<<" planned="<<found.planned<<" landed="<<found.landed<<" resumed="<<found.resumed<<" released="<<found.released<<" root="<<w.local(t.position).y<<","<<w.local(t.position).z<<" reason="<<t.blockedReason<<'\n';
        check((!sprint||(found.planned&&found.landed))&&found.resumed&&!found.released,
            "ordinary wall entry reconnects through a checked jump or native climb and continues upward");
    } else if(mode==4||mode==5)check(found.planned&&found.changed&&found.released&&!found.landed,"dynamic upper support or route loss invalidates transfer");
    else check(!found.planned&&!found.landed,"disabled, unaffordable, blocked or unsupported route cannot commit a recessed transfer");
    return found;
}
int main()try {
    unsigned positives=0,negatives=0,peak=0;
    for(int fps:{30,60,120})for(bool sprint:{false,true})for(bool distant:{false,true}) {
        const auto result=run(fps,sprint,40,distant);peak=std::max(peak,result.peak);++positives;
    }
    for(int mode=1;mode<=6;++mode){const auto result=run(60,true,40,false,mode);peak=std::max(peak,result.peak);++negatives;}
    std::cout<<"Recessed wall transfer positives="<<positives<<" negatives="<<negatives<<" checks="<<checks<<" peakQueries="<<peak<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
