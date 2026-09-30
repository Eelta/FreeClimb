#define main attachedSurfaceFixtureMain
#include "AttachSurfaceTests.cpp"
#undef main

static Traversal coherentTraversal() {
    Traversal traversal;traversal.cfg.radius=31;traversal.cfg.gap=37;traversal.cfg.height=138;
    return traversal;
}
static AttachWorld recessedWall(float recess,Vec origin,float yaw) {
    AttachWorld world;world.origin=origin;world.rotation=yaw;
    world.boxes={{{-500,-500,-500},{500,500,0}},
        {{-500,0,-500},{500,100,40}},{{-500,recess,40},{500,100,1000}}};
    return world;
}
static float capsuleDistance(const AttachWorld& world,Vec feet,float radius,float height) {
    const Vec point=world.local(feet);const float bottom=point.z+radius,top=point.z+height-radius;
    float distance=1e9f;
    for(const auto& box:world.boxes) {
        const float dx=point.x-std::clamp(point.x,box.low.x,box.high.x);
        const float dy=point.y-std::clamp(point.y,box.low.y,box.high.y);
        const float dz=top<box.low.z?box.low.z-top:bottom>box.high.z?bottom-box.high.z:0;
        distance=std::min(distance,std::sqrt(dx*dx+dy*dy+dz*dz));
    }
    return distance;
}
int main(){try {
    unsigned cases=0,peakCasts=0;
    for(Vec origin:{Vec{},Vec{133820.3f,38370.9f,-12265.4f}})for(float yaw:{0.f,.73f,1.57f})
    for(int fps:{30,60,120})for(float recess:{12.f,16.f,19.f,32.f})for(float distance:{37.f,55.f}) {
        auto world=recessedWall(recess,origin,yaw);const auto start=world.global({0,-distance,0});
        auto unsupported=coherentTraversal();
        check(!unsupported.attach(world,start,world.direction({0,1,0}),1000,60,false,false),
            "a low close plinth cannot lend its plane to a recessed upper wall during an unlifted catch");
        check(!unsupported.active(),"rejected incoherent plane leaves native movement ownership available");
        world.casts=0;auto grounded=coherentTraversal();
        check(grounded.attach(world,start,world.direction({0,1,0}),1000,60,false,true),
            "real recessed upper wall remains reachable through a checked raised entry");
        peakCasts=std::max(peakCasts,world.casts);
        const auto target=world.local(grounded.entryTarget());
        check(std::abs(target.y-(recess-grounded.cfg.gap))<.04f&&target.z>=39.98f,
            "entry targets the upper support plane and clears the projecting plinth");
        check((grounded.position-start).length()<.02f&&grounded.lastAttachDistance<=60.01f,
            "coherent entry preserves approach distance and never teleports");
        grounded.entry(Motion::sprintCatch,true);
        while(grounded.state==State::approach) {
            const auto result=grounded.update(world,{},1.f/fps,1000);
            check(!result.released,"verified raised route retains its real support during playback");
            check(capsuleDistance(world,grounded.position,31,138)>=30.92f,
                "independent capsule-to-box distances stay clear for every entry frame");
        }
        const auto endpoint=grounded.position;
        for(int frame=0;frame<fps/2;++frame) {
            check(!grounded.update(world,{},1.f/fps,1000).released,"coherent upper grip stays attached when idle");
            check((grounded.position-endpoint).length()<.02f,"settled catch does not seek a second support plane");
        }
        check(std::abs(world.local(grounded.position).y-(recess-37))<.04f,
            "idle root remains at the actual upper wall gap");
        auto overhead=world;overhead.boxes.push_back({{-500,-500,150},{500,500,154},false});
        auto blocked=coherentTraversal();
        check(!blocked.attach(overhead,start,world.direction({0,1,0}),1000,60,false,true),
            "blocked raised entry never falls back to a floating lower-plane catch");
        auto higher=coherentTraversal();
        check(higher.attach(world,world.global({0,recess-55,80}),world.direction({0,1,0}),1000,60,true,false),
            "airborne catch already above the plinth retains its actual upper face");
        ++cases;
    }
    for(float unevenness:{0.f,4.f,7.5f}) {
        auto world=recessedWall(unevenness,{},.73f);auto traversal=coherentTraversal();
        check(traversal.attach(world,world.global({0,-37,0}),world.direction({0,1,0}),1000,60,false,false),
            "small local surface variation retains ordinary attachment tolerance");
        ++cases;
    }
    for(int fps:{30,60,120}) {
        auto world=recessedWall(12,{},0);auto traversal=coherentTraversal();
        check(traversal.attach(world,{0,-55,0},{0,1,0},1000,60,false,true),
            "dynamic recessed-wall route starts with a clear real endpoint");
        traversal.entry(Motion::sprintCatch,true);
        while(traversal.reachProgress()<.65f)check(!traversal.update(world,{},1.f/fps,1000).released,
            "live entry advances through its original checked route");
        world.boxes[1].high.z=62;
        const auto previous=traversal.position;const auto result=traversal.update(world,{},1.f/fps,1000);
        check(result.released&&!traversal.active()&&(traversal.position-previous).length()<.001f,
            "changed low side obstacle invalidates the endpoint before the entry can advance");
        ++cases;
    }
    std::cout<<"PASS initial grip-plane coherence cases="<<cases<<" peakEntryCasts="<<peakCasts<<'\n';return 0;
}catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}}
