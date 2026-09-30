#include "Pose.h"
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace fc;
static void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}

struct FlipGeometry:World {
    struct Plane {Vec normal;float distance;};
    std::vector<Plane> planes;
    unsigned casts{};
    bool oneSided{};
    std::optional<Vec> ball;
    float ballRadius=1;
    std::optional<Hit> ray(Vec a,Vec b) override {
        ++casts;
        for(const auto& p:planes) {
            const float x=a.dot(p.normal)-p.distance,y=b.dot(p.normal)-p.distance;
            if((oneSided&&!(x>0&&y<0))||x*y>=0)continue;
            return Hit{a+(b-a)*(x/(x-y)),p.normal,true};
        }
        if(ball) {
            const auto d=b-a,o=a-*ball;const float dd=d.dot(d);
            if(dd>.000001f) {
                const float t=std::clamp(-o.dot(d)/dd,0.f,1.f);
                if((o+d*t).length()<ballRadius)return Hit{a+d*t,(a+d*t-*ball).unit(),false};
            }
        }
        return {};
    }
};

static bool preflight(FlipGeometry& world,const Library& lib,float slope=0,float scale=1,float gap=37) {
    const Vec start{0,-gap,300},outward{0,-1,0};
    for(int i=0;i<backFlipExitSegments;++i) {
        const float a=float(i)/backFlipExitSegments,b=float(i+1)/backFlipExitSegments;
        if(!backFlipBodyClear(world,lib,backFlipExitPoint(start,outward,a),backFlipExitPoint(start,outward,b),
            a,b,outward,slope,gap,scale))return false;
    }
    return true;
}

static void captureAndPlacement(const Library& lib) {
    const auto& clip=lib.clip(Motion::backFlipOut);
    check(clip.frames.size()>=30&&clip.seconds>.5f&&clip.seconds<.9f,"load a real finite backflip flight capture");
    for(const auto& weights:clip.contacts)for(float weight:weights)check(weight==0,"airborne capture cannot acquire source wall contacts");
    bool inverted=false;
    for(float slope:{0.f,.35f,.68f})for(float scale:{.7f,1.f,1.35f}) {
        const float gap=37*scale,h=std::sqrt(1-slope*slope);const Vec n{0,-h,slope};
        const auto first=sampleBackFlipOut(lib,0,slope,gap,scale);
        for(const auto& marker:backFlipBody(lib,first))
            check(marker.point.dot(n)-marker.radius>=-gap*h/scale+.9f/scale,
                "initial fixed calibration keeps padded head/limbs outside the actual sloped wall plane");
        const Vec firstCOM=lib.world(first)[4].t;
        for(int i=0;i<=48;++i) {
            const float phase=i/48.f;const auto source=lib.sample(Motion::backFlipOut,phase);
            const auto pose=sampleBackFlipOut(lib,phase,slope,gap,scale),world=lib.world(pose);
            check((world[4].t-firstCOM).length()<.01f,"capture COM does not add hidden displacement to the checked actor arc");
            for(std::size_t bone=1;bone<pose.size();++bone)
                check(angleBetween(pose[bone].q,source[bone].q)<.00001f&&
                    (pose[bone].t-source[bone].t).length()<.00001f,
                    "placement preserves captured articulation including wrist/finger locals");
            if(slope==0&&scale==1&&world[36].t.z<world[4].t.z-20)inverted=true;
        }
    }
    check(inverted,"actual flight capture passes through an inverted head-below-pelvis pose");
}

static void realGeometry(const Library& lib) {
    unsigned largest=0;
    for(float slope:{0.f,.35f,.68f})for(float scale:{.7f,1.f,1.35f})for(bool oneSided:{false,true}) {
        FlipGeometry world;world.oneSided=oneSided;
        world.planes.push_back({{0,-std::sqrt(1-slope*slope),slope},300*slope});
        world.planes.push_back({{0,0,1},0});
        check(preflight(world,lib,slope,scale,37*scale),"unobstructed hanging wall permits actual backflip source at supported actor sizes/slopes");
        largest=std::max(largest,world.casts);
    }
    check(largest<=1440,"source preflight is bounded to 120 rays for each of twelve intervals");
    FlipGeometry rear;rear.planes.push_back({{0,1,0},-180});
    check(!preflight(rear,lib),"rear obstacle rejects outward flight");
    FlipGeometry distantRear;distantRear.planes.push_back({{0,1,0},-300});
    check(preflight(distantRear,lib),"remote rear surface does not reject an open full-body exit");
    FlipGeometry ceiling;ceiling.planes.push_back({{0,0,-1},-455});
    check(!preflight(ceiling,lib),"extended limbs reaching above the upright controller reject a low ceiling");
    std::cout<<"maximum source preflight casts="<<largest<<"; controller casts are additional\n";
}

static void midpointAndLiveSweep(const Library& lib) {
    const Vec start{0,-37,300},outward{0,-1,0};
    const float a=5.f/12,b=6.f/12,mid=(a+b)*.5f;
    const Vec origin=backFlipExitPoint(start,outward,mid);
    const auto body=backFlipBody(lib,sampleBackFlipOut(lib,mid,0,37,1));

    FlipGeometry midObstacle;midObstacle.ball=origin+body[4].point;
    check(!backFlipBodyClear(midObstacle,lib,backFlipExitPoint(start,outward,a),backFlipExitPoint(start,outward,b),
        a,b,outward,0,37,1),"true intermediate source joint participates in the sweep");
    const float phase=.55f;const Vec feet=backFlipExitPoint(start,outward,phase);
    const auto local=backFlipBody(lib,sampleBackFlipOut(lib,phase,0,37,1));
    FlipGeometry moved;const Vec destination=feet+Vec{0,-20,0};
    moved.ball=feet+local[2].point+Vec{0,-10,0};
    check(!backFlipBodyClear(moved,lib,feet,destination,phase,phase,outward,0,37,1),
        "same-phase emergency release still sweeps translation of the held pose");
    FlipGeometry open;
    check(backFlipBodyClear(open,lib,feet,destination,phase,phase,outward,0,37,1),
        "same-phase clear impulse remains available");
    check(open.casts<=120,"one live body sweep has a fixed cast budget");
    Library missing=lib;missing.clips[static_cast<int>(Motion::backFlipOut)-1].frames.clear();
    check(!backFlipBodyClear(open,missing,feet,destination,phase,phase,outward,0,37,1),"missing actual capture fails closed");
    check(!backFlipBodyClear(open,lib,start,start,0,1,outward,0,37,1),"oversized unsegmented source intervals fail closed");
    check(!backFlipBodyClear(open,lib,start,start,0,0,{},0,37,1),"invalid orientation fails closed");
    check(!backFlipBodyClear(open,lib,start,start,0,0,outward,0,37,std::numeric_limits<float>::quiet_NaN()),"nonfinite body scale fails closed");
}

int main(int argc,char** argv) {
    try {
        check(argc==2,"supply FreeClimb.motion path");Library lib;check(lib.load(argv[1]),"load runtime capture library");
        captureAndPlacement(lib);realGeometry(lib);midpointAndLiveSweep(lib);
        std::cout<<"BackFlipPose tests passed\n";return 0;
    } catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
