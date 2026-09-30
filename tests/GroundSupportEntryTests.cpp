#define main fixtureMain
#include "AttachSurfaceTests.cpp"
#undef main
#ifdef FC_ENTRY_BASELINE
#include "NativeWalkableApproach.baseline.h"
#else
#include "NativeWalkableApproach.h"
#endif
static Settings entryBody(){Settings c;c.radius=31;c.gap=37;c.height=138;return c;}
static AttachWorld entryGround(int shape) {
    AttachWorld w;
    if(shape==0)w.boxes={{{-1000,-1000,-1000},{1000,1000,0}}};
    if(shape==1)w.boxes={{{-1000,-1000,-1000},{-4,1000,0}},{{4,-1000,-1000},{1000,1000,0}},
        {{-4,-1000,-1000},{4,-54,0}},{{-4,-46,-1000},{4,1000,0}}};
    if(shape==2)w.boxes={{{-1000,-1000,-1000},{1000,-58,0}}};
    if(shape==3)w.boxes={{{17,-51,-1000},{20,-49,0}},{{-20,-51,-1000},{-17,-49,0}}};
    return w;
}
static GroundEntryExclusionResult decision(AttachWorld& w,Vec feet,Vec selected,Vec normal,bool grounded=true) {
    const auto cfg=entryBody();
    const auto native=nativeWalkableApproach(w,feet,w.direction({0,1,0}),cfg,grounded);
    if(native.walkable) {
        GroundEntryExclusionResult result;result.reason=GroundEntryExclusion::staircase;result.casts=native.casts;return result;
    }
    auto result=groundedLowFace(w,feet,cfg,grounded,selected,normal);
#ifndef FC_ENTRY_BASELINE
    if(!result.excludes()){
        auto step=groundedStepFace(w,feet,w.direction({0,1,0}),cfg,grounded,selected,normal);
        if(step.excludes())return step;
    }
#endif
    if(!result.excludes())return groundedEntryExclusion(w,feet,w.direction({0,1,0}),cfg,grounded,selected,normal);
    return result;
}
int main(){try {
    unsigned checks=0,failures=0,actual=0,peak=0;
    for(float yaw:{0.f,.73f,1.57f})for(Vec origin:{Vec{},Vec{136542.8f,37636.9f,-12071.8f}}) {
        auto test=[&](AttachWorld w,Vec start,Vec selected,bool grounded,bool expected,const char* label,bool requireAttach=false) {
            w.rotation=yaw;w.origin=origin;const Vec feet=w.global(start);Vec normal=w.direction({0,-1,0});
            Traversal candidate;candidate.cfg=entryBody();
            if(requireAttach&&candidate.attach(w,feet,w.direction({0,1,0}),1000,60,false,true)) {
                selected=w.local(candidate.entryTarget());normal=candidate.surfaceNormal;++actual;
            }
            const auto r=decision(w,feet,w.global(selected),normal,grounded);peak=std::max(peak,r.casts);
            if(r.excludes()!=expected){++failures;std::cout<<"FAIL "<<label<<" yaw="<<yaw<<" far="<<(origin.x!=0)<<" expected="<<expected<<" observed="<<r.excludes()<<" casts="<<r.casts<<" selected="<<selected.y<<","<<selected.z<<"\n";}
            if(!grounded)check(r.casts==0,"airborne regrab has no ground classification rays");
            ++checks;
        };
        for(int shape:{0,1,2}) {
            auto low=entryGround(shape);low.boxes.push_back({{-40,0,-1000},{40,1.25f,38}});
            test(low,{0,-50,0},{0,-37,0},true,true,"short low object with measured foot support",shape<2);
            auto tall=low;tall.boxes.back().high.z=1000;
            test(tall,{0,-50,0},{0,-37,0},true,false,"real tall wall beside the same foot support",shape<2);
            auto beyond=low;beyond.boxes.back().high.z=48.1f;
            test(beyond,{0,-50,0},{0,-37,0},true,false,"low height boundary remains strict");
            test(low,{0,-50,0},{0,-37,0},false,false,"airborne explicit low ledge regrab preserved");
            auto floating=low;floating.boxes.back().low.z=35;floating.boxes.back().high.z=48;
            test(floating,{0,-50,0},{0,-37,0},true,false,"floating thin slab remains unclassified");
        }
        auto unsupported=entryGround(3);unsupported.boxes.push_back({{-40,0,-1000},{40,1.25f,38}});
        test(unsupported,{0,-50,0},{0,-37,0},true,false,"two isolated support points cannot invent a ground plane");
        auto empty=entryGround(4);empty.boxes.push_back({{-40,0,-1000},{40,1.25f,38}});
        test(empty,{0,-50,0},{0,-37,0},true,false,"grounded engine flag alone is insufficient");
        auto offset=entryGround(0);offset.boxes.push_back({{-40,0,-1000},{40,1.25f,38}});
        test(offset,{0,-50,18},{0,-37,18},true,false,"ground beyond original support tolerance is not accepted");
        auto steps=entryGround(0);for(int i=0;i<5;++i)steps.boxes.push_back({{-12,float(i)*4,-1000},{12,20,float(i+1)*20}});
        test(steps,{0,-50,0},{0,-33,0},true,true,"short narrow stairs actually accepted by Core",true);
        test(steps,{0,-50,0},{0,-29,0},true,true,"selected third short stair above low-object total height");
        auto wall=steps;wall.boxes.push_back({{-12,8,-1000},{12,20,1000}});
        test(wall,{0,-50,0},{0,-29,0},true,false,"selected high wall cannot borrow adjacent steps");
        auto upperWall=entryGround(0);
        upperWall.boxes.push_back({{-80,0,-1000},{80,100,20}});
        upperWall.boxes.push_back({{-80,4,-1000},{80,100,40}});
        upperWall.boxes.push_back({{-80,4,60},{80,100,1000}});
        test(upperWall,{0,-30,0},{0,-33,0},true,false,"actual selected upper wall cannot borrow low steps below",true);
        auto gap=steps;gap.boxes.front().high.y=-58;
        test(gap,{0,-50,0},{0,-29,0},true,false,"disconnected elevated stairs do not become continuous steps");
        auto wide=entryGround(0);for(int i=0;i<4;++i)wide.boxes.push_back({{-80,float(i)*12,-1000},{80,48,float(i+1)*20}});
        test(wide,{0,-50,0},{0,-1,0},true,true,"finite staircase ending immediately after selected top step");
    }
    std::cout<<"Ground-support/short-step checks="<<checks<<" actualCoreCandidates="<<actual<<" failures="<<failures<<" peakDecisionQueries="<<peak<<"\n";
    check(actual>=24,"real successful Core candidates must be exercised across rotated and distant worlds");
    return failures?1:0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
