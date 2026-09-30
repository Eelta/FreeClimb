#include "Pose.h"
#include "MotionSlots.h"
#include "TraversalAudio.h"
#include <iostream>
#include <stdexcept>
using namespace fc;

static unsigned checks{};
static void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
struct Wall:World {
    std::optional<Hit> ray(Vec a,Vec b) override {
        if(a.y>=0||b.y<0)return {};
        return Hit{a+(b-a)*(-a.y/(b.y-a.y)),{0,-1,0},true};
    }
};
static void slots(const Library& lib) {
    check(motionCount==42&&activeMotionCount==35&&activeMotions.size()==35,"stable indexed capacity exposes exactly 35 available motions");
    int previous=0;
    for(Motion motion:activeMotions) {
        const int id=int(motion);
        check(id>previous&&isActiveMotion(motion)&&!motionSlotNames[id-1].empty(),"available slot names retain their distinct original IDs");
        check(lib.clip(motion).frames.size()>=2,"every available motion has playable runtime frames");
        previous=id;
    }
    check(isActiveMotion(Motion::drop)&&int(Motion::drop)==15&&isActiveMotion(Motion::contextHang)&&int(Motion::contextHang)==39,
        "ordinary release and contextual edge idle remain available with unchanged IDs");
    for(int id:{-1,0,6,7,12,13,14,33,38,43,44,1000000}) {
        const auto motion=Motion(id);
        check(!isActiveMotion(motion)&&!runMotion(motion),"retired and invalid IDs cannot be available or downward wall running");
        check(lib.clip(motion).frames.empty()&&!lib.hasAnimationOverride(motion),"retired and invalid IDs have no playable clip or override");
        if(id>=1&&id<=motionCount)check(motionSlotNames[id-1].empty()&&lib.clips[id-1].frames.empty(),"retired indexed slots hold neither names nor motion data");
        for(float phase:{0.f,.25f,.5f,1.f}) {
            const auto sampled=lib.sample(motion,phase);
            check(sampled.size()==lib.rest.size(),"unavailable sampling returns the rig rest pose");
            for(std::size_t bone=0;bone<sampled.size();++bone)
                check((sampled[bone].t-lib.rest[bone].t).length()==0&&angleBetween(sampled[bone].q,lib.rest[bone].q)<.000001f&&
                    (sampled[bone].s-lib.rest[bone].s).length()==0,"unavailable IDs never alias an active animation");
            check(lib.contactWeights(motion,phase)==std::array<float,4>{},"unavailable motion contacts are all released");
        }
    }
}
static void controller() {
    Wall wall;
    for(int fps:{30,60,120})for(int id:{6,7,12,13,14,33,38}) {
        Traversal t;
        check(t.attach(wall,{0,-42,200},{0,1,0},100),"retired entry fixture attaches to a real wall");
        t.entry(Motion(id),true);
        const auto entry=t.update(wall,{},1.f/fps,100);
        check(entry.motion==Motion::reach,"attempting a retired entry keeps the ordinary checked entry");
        for(int frame=0;frame<fps*2;++frame) {
            const auto r=t.update(wall,{},1.f/fps,100);
            check(isActiveMotion(r.motion)&&!r.released,"entry and stationary update never select retired IDs");
        }
        const auto before=t.position;
        const auto down=t.update(wall,{0,-1,false,false,false,false,true},1.f/fps,100);
        check(down.motion==Motion::down&&!t.wallRunning()&&t.position.z<before.z,
            "holding run while moving down retains ordinary descending climb");
        const auto drop=t.update(wall,{0,0,true},1.f/fps,100);
        check(drop.motion==Motion::drop,"retiring old slots keeps the ordinary release action");
    }
}
int main(int argc,char** argv) {
    try {
        check(argc==2,"runtime pack path required");Library lib;check(lib.load(argv[1]),"runtime pack loads");
        slots(lib);controller();std::cout<<"PASS retired slot removal checks="<<checks<<'\n';return 0;
    }catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}
