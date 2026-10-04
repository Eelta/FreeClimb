#include "Pose.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
static void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
struct TailLedge:World {
    float height=132,barrier=1e9f;
    bool floor=true;
    std::optional<Hit> ray(Vec a,Vec b) override {
        std::optional<Hit> hit;float nearest=2;
        auto offer=[&](float fraction,Vec normal,bool solid){if(fraction>=0&&fraction<=1&&fraction<nearest){nearest=fraction;hit=Hit{a+(b-a)*fraction,normal,solid};}};
        if(a.y<0&&b.y>=0){const float t=-a.y/(b.y-a.y);if((a+(b-a)*t).z<=height)offer(t,{0,-1,0},true);}
        if(floor&&a.z>height&&b.z<=height){const float t=(height-a.z)/(b.z-a.z);if((a+(b-a)*t).y>=0)offer(t,{0,0,1},true);}
        if(a.y<barrier&&b.y>=barrier)offer((barrier-a.y)/(b.y-a.y),{0,-1,0},false);
        return hit;
    }
};
static Traversal fixture(TailLedge& world,const Library& library){
    Traversal t;require(library.configureThreepeat(t.cfg),"load actual contact timing");t.cfg.threepeatAnimations=true;t.cfg.gap=37;t.cfg.radius=31;
    require(t.attach(world,{0,-42,0},{0,1,0},100),"attach actual checked ledge");return t;
}
static float endpoints(const Library& library,const Pose& a,const Pose& b){
    const auto x=library.world(a),y=library.world(b);float value=0;
    for(int bone:{4,8,11,26,36,38,39})value=std::max(value,(x[bone].t-y[bone].t).length());return value;
}
static float complete(const Library& library,int fps,float height){
    TailLedge w;w.height=height;auto t=fixture(w,library);SurfacePose surface;Pose previous;Vec oldPosition=t.position;
    float begin=-1,previousPhase=0,previousPreparation=0,seconds=0,sourceBegin=0,release=0,tailStart=-1;
    float maxAngle=0,maxSpeed=0;bool mantling=false,low=false;unsigned unchanged=0;Pose finish;
    for(int frame=0;frame<fps*8&&t.active();++frame){
        const float age=float(frame)/fps;const auto result=t.update(w,{0,1,false,true},1.f/fps,100);
        const auto pose=surface.update(library,w,t,result.motion,1.f/fps,1);
        if(result.motion==Motion::contextMantle){
            if(begin<0){begin=age;low=t.lowTopStep();seconds=t.topSeconds();sourceBegin=t.topSampleBegin();release=std::max(t.cfg.threepeatProfile.mantleRelease[0][1],t.cfg.threepeatProfile.mantleRelease[1][1]);}
            if(mantling&&previousPreparation>=1){
                const float original=std::min(1.f,previousPhase+1.f/(fps*seconds));
                if(low||sourceBegin+(1-sourceBegin)*original<=release){
                    require(std::abs(t.progress()-original)<.00001f,"low steps and loaded-hand phases preserve original clock");++unchanged;
                }
            }
            if(t.topSamplePhase(t.progress())>=release&&tailStart<0)tailStart=age;
            require((t.position-t.topPathPoint(t.progress())).length()<.001f,"accelerated movement stays on the original complete root path");
            if(mantling&&previous.size()==99&&t.topPreparation()>=1){
                const auto a=library.world(previous),b=library.world(pose);
                for(std::size_t bone=0;bone<pose.size();++bone){const float angle=angleBetween(previous[bone].q,pose[bone].q);maxAngle=std::max(maxAngle,angle*fps);require(angle<=12.566371f/fps+.005f,"tail retains original angular speed budget");}
                for(int bone:{8,11,38,39}){const float step=(b[bone].t+t.position-a[bone].t-oldPosition).length();maxSpeed=std::max(maxSpeed,step*fps);require(step<=750.f/fps+.6f,"tail retains original endpoint speed budget");}
            }
            mantling=true;previousPhase=t.progress();previousPreparation=t.topPreparation();
            if(result.completed){
                require(result.released&&!t.active()&&t.progress()==1.f,"completion reaches phase one before native handoff");
                require((t.position-t.topTarget()).length()<.001f,"all accelerated routes reach the actual standing target");
                const float duration=age-begin,oldDuration=seconds+(low?0.f:.60f);
                require(unchanged>0||sourceBegin>=release,"fixtures retain loaded-hand samples or begin after both hands release");
                if(low)require(std::abs(duration-oldDuration)<=1.01f/fps,"low step total duration stays unchanged");
                else require(duration<=oldDuration-.23f&&duration>=oldDuration-.60f,"released-hand tail removes waiting without skipping most of the action");
                finish=pose;Pose settled=pose;
                for(int i=0;i<30;++i)settled=surface.update(library,w,t,result.motion,1.f/fps,1);
                const float pending=endpoints(library,finish,settled);
                std::cout<<"MANTLE_TAIL fps="<<fps<<" height="<<height<<" low="<<low<<" sourceBegin="<<sourceBegin<<" old="<<oldDuration<<" current="<<duration<<" tail="<<age-tailStart<<" endpointPending="<<pending<<" speed="<<maxSpeed<<" angular="<<maxAngle<<'\n';
                require(pending<.15f,"phase one does not leave a visible animation catch-up behind the root path");return duration;
            }
        }
        previous=pose;oldPosition=t.position;
    }
    throw std::runtime_error("checked mantle must complete");
}
static void changedGeometry(const Library& library,int fps,bool removeFloor){
    TailLedge w;auto t=fixture(w,library);Result result;
    for(int frame=0;frame<fps*8&&t.active();++frame){result=t.update(w,{0,1,false,true},1.f/fps,100);if(t.state==State::mantle&&t.progress()>=.68f)break;}
    require(t.active()&&t.state==State::mantle&&t.progress()<.82f,"obstruction arrives during accelerated horizontal tail");
    if(removeFloor)w.floor=false;else w.barrier=t.position.y+t.cfg.radius+.2f;
    const Vec before=t.position;result=t.update(w,{0,1,false,true},1.f/fps,100);
    require(result.released&&!result.completed&&!t.active(),"live standing loss and newly obstructed tail reject completion");
    require((t.position-before).length()<.001f,"blocked subdivided motion cannot partially commit unchecked movement");
}
int main(int argc,char** argv){try{
    Library library;require(argc==2&&library.load(argv[1]),"load current HKX animation pack");
    for(float height:{45.f,80.f,120.f,131.71f,132.53f,145.f}){
        float fastest=100,slowest=0;
        for(int fps:{30,40,60,120}){const float duration=complete(library,fps,height);fastest=std::min(fastest,duration);slowest=std::max(slowest,duration);}
        require(slowest-fastest<.045f,"tail timing stays consistent across frame rates");
    }
    for(int fps:{30,40,60,120})for(bool floor:{false,true})changedGeometry(library,fps,floor);
    std::cout<<"PASS mantle tail timing, current geometry, complete paths and actual pose constraints\n";return 0;
}catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}}

