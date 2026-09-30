#define main oldAttachFixtureMain
#include "AttachSurfaceTests.cpp"
#undef main
#include "NativeWalkableApproach.h"

static Settings exclusionBody(){Settings c;c.radius=31;c.height=138;c.gap=37;return c;}
static AttachWorld exclusionGround(float yaw,Vec origin) {
    AttachWorld w;w.rotation=yaw;w.origin=origin;w.boxes={{{-1000,-1000,-1000},{1000,1000,0}}};return w;
}
static AttachWorld exclusionSteps(float yaw,Vec origin,float rise=24,float depth=32) {
    auto w=exclusionGround(yaw,origin);for(int i=0;i<8;++i)w.boxes.push_back({{-1000,i*depth,-1000},{1000,1000,(i+1)*rise}});return w;
}
static GroundEntryExclusionResult inspect(AttachWorld& w,Vec start,Vec selected,bool grounded=true) {
    return groundedEntryExclusion(w,w.global(start),w.direction({0,1,0}),exclusionBody(),grounded,w.global(selected),w.direction({0,-1,0}));
}
static void exclusions(float yaw,Vec origin) {
    auto stairs=exclusionSteps(yaw,origin);auto r=inspect(stairs,{0,-50,0},{0,-5,0});
    if(!r.excludes())std::cerr<<"stairs yaw="<<yaw<<" origin="<<origin.x<<" casts="<<r.casts<<"\n";
    check(r.reason==GroundEntryExclusion::staircase,"two real24-unit risers exclude automatic grab of the selected second riser");
    check(r.casts<=3072,"stair exclusion stays within its independent finite ray budget");
    auto air=inspect(stairs,{0,-50,0},{0,-5,0},false);check(!air.excludes()&&air.casts==0,"airborne regrab gets no grounded exclusion");
    auto low=exclusionGround(yaw,origin);low.boxes.push_back({{-1000,0,-1000},{1000,200,48}});
    auto curb=inspect(low,{0,-50,0},{0,-37,0});check(curb.reason==GroundEntryExclusion::lowObstacle,"real48-unit low object suppresses automatic entry without declaring it native-walkable");
    check(!nativeWalkableApproach(low,low.global({0,-50,0}),low.direction({0,1,0}),exclusionBody(),true).walkable,"legacy native16-unit contract remains unchanged");
    auto beyond=low;beyond.boxes[1].high.z=48.1f;check(!inspect(beyond,{0,-50,0},{0,-37,0}).excludes(),"objects over explicit48 limit remain available to climb");
    auto floating=exclusionGround(yaw,origin);floating.boxes.push_back({{-1000,0,36},{1000,200,40}});
    check(!inspect(floating,{0,-50,0},{0,-37,0}).excludes(),"a thin floating top does not substitute for a solid low obstacle riser");
    auto thin=low;thin.boxes[1].low.x=-10;thin.boxes[1].high.x=10;
    check(!inspect(thin,{0,-50,0},{0,-37,0}).excludes(),"a narrow top with ground below its edges is not a real foot corridor");
    auto roof=stairs;roof.boxes.push_back({{-1000,0,155},{1000,100,157},false});
    check(!inspect(roof,{0,-50,0},{0,-5,0}).excludes(),"thin ceiling blocks the complete checked body corridor");
    auto distantWall=stairs;distantWall.boxes.push_back({{-1000,140,-1000},{1000,150,1000}});
    check(inspect(distantWall,{0,-50,0},{0,-5,0}).excludes(),"unrelated far wall cannot veto a verified local selected stair");
    auto nearWall=stairs;nearWall.boxes.push_back({{-1000,70,-1000},{1000,80,1000}});
    check(!inspect(nearWall,{0,-50,0},{0,33,0}).excludes(),"a selected high wall at the stair top remains a real climb target");
    auto noFloor=stairs;noFloor.boxes.erase(noFloor.boxes.begin());
    check(!inspect(noFloor,{0,-50,0},{0,-5,0}).excludes(),"grounded flag alone cannot supply missing real starting floor");
    auto invalid=stairs;for(auto& b:invalid.boxes)b.climbable=false;
    check(!inspect(invalid,{0,-50,0},{0,-5,0}).excludes(),"unknown or invalid geometry remains unclassified");
}
static void actualAttachment(float yaw,Vec origin) {
    auto w=exclusionSteps(yaw,origin,20,32);unsigned intercepted=0;
    for(float distance:{38.f,47.f,57.f,67.f,77.f}) {
        Traversal candidate;candidate.cfg=exclusionBody();const Vec feet=w.global({0,-distance,0}),facing=w.direction({0,1,0});
        if(!candidate.attach(w,feet,facing,1000,60,false,true))continue;
        check(!nativeWalkableApproach(w,feet,facing,candidate.cfg,true).walkable,"fixture reproduces the previous16-unit stair exemption rejection");
        const auto decision=groundedEntryExclusion(w,feet,facing,candidate.cfg,true,candidate.entryTarget(),candidate.surfaceNormal);
        if(!decision.excludes())std::cerr<<"unfiltered existing stair entry distance="<<distance<<" target="<<w.local(candidate.entryTarget()).y<<" casts="<<decision.casts<<"\n";
        check(decision.excludes(),"actual successful stair candidate is excluded before native ownership is acquired");++intercepted;
    }
    check(intercepted>0,"fixture must reproduce at least one current automatic stair attachment");
}
int main(){try {unsigned groups=0;
    for(float yaw:{0.f,.73f,1.57f})for(Vec origin:{Vec{},Vec{131146.67f,38939.656f,-12205.819f}}) {
        exclusions(yaw,origin);actualAttachment(yaw,origin);++groups;
    }
    std::cout<<"PASS grounded selected-face entry exclusions: "<<groups<<" rotated/far groups, low/stair/air/wall/unknown/body and actual attach cases\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
