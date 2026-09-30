#define main attachFixtureMain
#include "AttachSurfaceTests.cpp"
#undef main
#include "NativeWalkableApproach.h"

static Settings guardBody(){Settings c;c.radius=31;c.gap=37;c.height=138;return c;}
static AttachWorld groundWorld(){AttachWorld w;w.boxes={{{-1000,-1000,-1000},{1000,1000,0}}};return w;}
int main(){try {
    unsigned cases=0,missed=0;
    for(float yaw:{0.f,.73f,1.57f})for(Vec origin:{Vec{},Vec{134034.2f,36638.8f,-12262.5f}}) {
        auto inspect=[&](AttachWorld w,Vec start,Vec selected,bool grounded,bool expected,const char* label) {
            w.rotation=yaw;w.origin=origin;
            const auto cfg=guardBody();const auto feet=w.global(start),normal=w.direction({0,-1,0});
            const auto old=groundedEntryExclusion(w,feet,w.direction({0,1,0}),cfg,grounded,w.global(selected),normal);
            const auto result=groundedLowFace(w,feet,cfg,grounded,w.global(selected),normal);
            if(expected&&!old.excludes())++missed;
            if(result.excludes()!=expected)std::cerr<<label<<" yaw="<<yaw<<" origin="<<origin.x<<" casts="<<result.casts<<'\n';
            check(result.excludes()==expected,label);check(result.casts<=31,"selected low-face checks stay within seven face rays plus bounded inner and outer footprint evidence");
            if(!grounded)check(result.casts==0,"airborne catch skips the grounded low-face filter");
            ++cases;
        };
        auto low=groundWorld();low.boxes.push_back({{-8,0,-1000},{8,160,38}});
        inspect(low,{0,-50,0},{0,-37,0},true,true,"narrow solid low obstacle cannot borrow an unwalkable corridor to trigger climbing");
        inspect(low,{0,-50,12},{0,-37,12},true,true,"supported root above actual ground still recognizes the same low obstacle");
        inspect(low,{0,-50,18},{0,-37,18},true,false,"floor outside bounded ground tolerance does not classify an entry as grounded");
        inspect(low,{0,-50,0},{0,-37,0},false,false,"falling catch is not suppressed by the grounded low-object rule");
        auto high=low;high.boxes[1].high.z=70;
        inspect(high,{0,-50,0},{0,-37,0},true,false,"a higher ledge remains climbable");
        high.boxes[1].high.z=1000;
        inspect(high,{0,-50,0},{0,-37,0},true,false,"a real wall remains climbable");
        auto floating=low;floating.boxes[1].low.z=35;
        inspect(floating,{0,-50,0},{0,-37,0},true,false,"floating slab is not a verified grounded solid riser");
        auto neighbor=low;neighbor.boxes.push_back({{20,0,-1000},{200,1000,1000}});
        inspect(neighbor,{0,-50,0},{50,-37,0},true,false,"selected high neighboring wall never borrows a low object at the original facing");
        auto wide=low;wide.boxes[1].low.x=-200;wide.boxes[1].high.x=200;
        wide.boxes[1].high.z=48;
        inspect(wide,{0,-50,0},{0,-37,0},true,true,"48-unit low obstacle limit is inclusive");
        wide.boxes[1].high.z=48.1f;
        inspect(wide,{0,-50,0},{0,-37,0},true,false,"above the low obstacle limit retains climb eligibility");
        auto stairs=groundWorld();
        for(int i=0;i<8;++i)stairs.boxes.push_back({{-1000,float(i)*32,-1000},{1000,1000,float(i+1)*20}});
        stairs.rotation=yaw;stairs.origin=origin;
        const auto g=groundedEntryExclusion(stairs,stairs.global({0,-50,12}),stairs.direction({0,1,0}),guardBody(),true,
            stairs.global({0,-5,12}),stairs.direction({0,-1,0}));
        check(g.excludes(),"continuous steps are excluded even when supported actor root is twelve units above the ground");++cases;
        auto walk=groundWorld();
        for(int i=0;i<12;++i)walk.boxes.push_back({{-1000,float(i)*28,-1000},{1000,1000,float(i+1)*12}});
        walk.rotation=yaw;walk.origin=origin;
        check(nativeWalkableApproach(walk,walk.global({0,-50,12}),walk.direction({0,1,0}),guardBody(),true).walkable,
            "native step corridor is preserved for supported root offset");++cases;
    }
    check(missed>=6,"narrow object fixture reproduces the previous full-corridor exclusion blind spot");
    std::cout<<"Low-face entry cases="<<cases<<" priorCorridorMisses="<<missed<<" PASS\n";return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
