#define main attachFixtureMain
#include "AttachSurfaceTests.cpp"
#undef main
#include "NativeWalkableApproach.h"

struct WalkWorld:AttachWorld {
    bool ramp{};float slope{};
    std::optional<Hit> ray(Vec from,Vec to) override {
        auto hit=AttachWorld::ray(from,to);
        if(!ramp)return hit;
        const auto a=local(from),d=local(to)-a;
        const float start=a.z-slope*a.y,delta=d.z-slope*d.y;
        if(start>0&&delta<0) {
            const float t=-start/delta;
            if(t>0&&t<=1) {
                const auto point=from+(to-from)*t;
                if(!hit||(point-from).length()<(hit->point-from).length())
                    hit=Hit{point,direction(Vec{0,-slope,1}.unit()),true};
            }
        }
        return hit;
    }
};
static Settings body(){Settings cfg;cfg.radius=31;cfg.height=138;cfg.gap=37;return cfg;}
static WalkWorld staircase(float depth=28,float rise=12) {
    WalkWorld w;w.boxes.push_back({{-1000,-1000,-1000},{1000,1000,0}});
    for(int i=0;i<12;++i)w.boxes.push_back({{-1000,float(i)*depth,-1000},{1000,1000,float(i+1)*rise}});
    return w;
}
int main(){try {
    unsigned groups=0,peak=0,prevented=0;
    for(Vec origin:{Vec{},Vec{24852.2637f,-6398.9f,-3270.01782f}})for(float rotation:{0.f,.73f,1.57f}) {
        auto run=[&](WalkWorld w,Vec feet,bool grounded,bool expected,const std::string& label) {
            w.origin=origin;w.rotation=rotation;
            const auto result=nativeWalkableApproach(w,w.global(feet),w.direction({0,1,0}),body(),grounded);
            if(result.walkable!=expected)std::cerr<<label<<" origin="<<origin.x<<" rotation="<<rotation<<" casts="<<result.casts<<'\n';
            check(result.walkable==expected,label);check(result.casts<=3072,"query budget");peak=std::max(peak,result.casts);
            if(!grounded)check(result.casts==0,"air catches never probe or suppress native entry");
        };
        auto steps=staircase();
        run(steps,{0,-50,0},true,true,"normal stairs retain native walking");
        run(staircase(19.35f,12.55f),{0,-50,0},true,true,"closer steep stairs retain native walking");
        run(steps,{0,-50,0},false,false,"airborne stair-side regrab remains available");
        for(float separation:{37.f,47.f,57.f,67.f,77.f}) {
            auto fixture=steps;fixture.origin=origin;fixture.rotation=rotation;
            Traversal oldEntry;oldEntry.cfg=body();
            const auto feet=fixture.global({0,-separation,0}),facing=fixture.direction({0,1,0});
            if(oldEntry.attach(fixture,feet,facing,1000,60,false,true)) {
                const auto guard=nativeWalkableApproach(fixture,feet,facing,body(),true);
                check(guard.walkable,"existing grounded stair catch is intercepted before acquiring control");++prevented;
            }
        }
        WalkWorld flat;flat.boxes={{{-1000,-1000,-1000},{1000,1000,0}}};
        run(flat,{0,0,0},true,true,"flat floor stays native");
        WalkWorld ramp;ramp.ramp=true;ramp.slope=.6f;
        run(ramp,{0,0,0},true,true,"walkable ramp stays native");
        ramp.slope=1.1f;run(ramp,{0,0,0},true,false,"steep climbable slope is not walking");
        auto high=flat;high.boxes.push_back({{-1000,0,-1000},{1000,1000,38}});
        run(high,{0,-50,0},true,false,"single high ledge still permits climbing");
        auto wallWorld=flat;wallWorld.boxes.push_back(wall);
        run(wallWorld,{0,-50,0},true,false,"real wall still permits climbing");
        auto gap=flat;gap.boxes={{{-1000,-1000,-1000},{1000,10,0}},{{-1000,50,-1000},{1000,1000,0}}};
        run(gap,{0,-50,0},true,false,"gap cannot be called native walking");
        auto narrow=steps;for(auto& box:narrow.boxes){box.low.x=-10;box.high.x=10;}
        run(narrow,{0,-50,0},true,false,"thin staircase has no full foot corridor");
        auto beam=steps;beam.boxes.push_back({{-1000,5,125},{1000,15,132},false});
        run(beam,{0,-50,0},true,false,"overhead beam prevents native-route exemption");
        auto post=flat;post.boxes.push_back({{20,20,30},{25,28,80},false});
        run(post,{0,-50,0},true,false,"off-centre shoulder obstacle blocks full body");
        auto underside=flat;underside.boxes.push_back({{-1000,20,20},{1000,30,90},false});
        run(underside,{0,-50,0},true,false,"suspended obstacle is not a stair");
        ++groups;
    }
    check(prevented>0,"fixture reproduces old stair attachment");
    std::cout<<"Native walkable approach groups="<<groups<<" preventedStairCatches="<<prevented<<" maxQueries="<<peak<<" stairs/ramp/air/wall/gap/body checks passed\n";
    return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
