#include "CornerTestWorld.h"
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

using namespace fc;
namespace fc {
class TraversalCapture {
public:
    static CornerRoute placeCorner(Traversal& actor,float phase) {
        auto& route=actor.cornerRoute;route.distance=route.length*phase;
        const auto at=cornerSample(route,route.distance);
        actor.position=at.position;actor.surfaceNormal=at.normal;actor.normal=cornerHorizontal(at.normal);
        return route;
    }
};
}
namespace {
struct Ray { Vec from,to; };
bool same(Vec a,Vec b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
bool same(const Ray& a,const Ray& b){return same(a.from,b.from)&&same(a.to,b.to);}
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
struct EmptyWorld final:World {
    std::vector<Ray> calls;
    unsigned stop=std::numeric_limits<unsigned>::max();
    std::optional<Hit> ray(Vec from,Vec to)override {
        calls.push_back({from,to});
        if(calls.size()==stop)return Hit{(from+to)*.5f,{0,0,1},false};
        return {};
    }
};
struct RecordedCorner final:World {
    fc_test::CornerWorld geometry;
    std::vector<Ray> calls;
    std::optional<Hit> ray(Vec from,Vec to)override {calls.push_back({from,to});return geometry.ray(from,to);}
    unsigned count(Ray target)const {
        unsigned result=0;for(const auto& value:calls)result+=same(value,target);return result;
    }
};
bool clear(Vec,Vec){return true;}
void exactDirections() {
    const auto cfg=fc_test::settings();CornerRoute route;
    route.sourceNormal={0,-1,0};route.targetNormal={1,0,0};
    for(Vec facing:{route.sourceNormal,route.targetNormal}) {
        EmptyWorld world;
        require(!cornerGrip(world,cfg,route,{},facing)&&world.calls.size()==10,
            "identical raw normal is not retried within a failed corner grip");
    }
    for(Vec facing:{Vec{.000001f,-1,0},Vec{0,-1,.000001f}}) {
        EmptyWorld world;
        require(!cornerGrip(world,cfg,route,{},facing)&&world.calls.size()==15,
            "nearby directions remain distinct contact probes");
    }
    route.sourceNormal={0,-.8f,.6f};route.targetNormal={1,0,0};
    EmptyWorld inclined;
    require(!cornerGrip(inclined,cfg,route,{},route.sourceNormal)&&inclined.calls.size()==20,
        "inclined grip keeps both complete height searches for each distinct normal");
    RecordedCorner actual;actual.geometry.corner(true);
    const Vec start{-45,-37,0};const auto planned=findCornerRoute(actual,cfg,start,{0,-1,0},1,clear);
    require(planned.has_value(),"contact freshness fixture plans a physical corner");
    require(cornerGrip(actual,cfg,*planned,start,{0,-1,0}),"initial actual grip exists");
    actual.geometry.boxes.clear();actual.calls.clear();
    require(!cornerGrip(actual,cfg,*planned,start,{0,-1,0})&&!actual.calls.empty(),
        "a later grip call observes removed support instead of retaining a positive result");
}
void overlappingSamples(float startX,bool overlap) {
    RecordedCorner world;world.geometry.corner(true);const auto cfg=fc_test::settings();
    const auto route=findCornerRoute(world,cfg,{startX,-37,0},{0,-1,0},1,clear);
    require(route&&route->count>2,"long-approach corner produces a fully checked route");
    const Vec a=route->points[0],b=route->points[1];
    require((b-a).length()>80&&same(route->normals[0],route->normals[1]),
        "sampling fixture has a long straight first segment");
    auto rayAt=[&](float phase) {
        const Vec feet=a+(b-a)*phase;
        const Vec normal=(route->normals[0]*(1-phase)+route->normals[1]*phase).unit();
        const Vec anchor=cornerAnchor(feet,normal,cfg.grip-5.f);
        return Ray{anchor+normal*12.f,anchor-normal*(cfg.gap+20.f)};
    };
    require(world.count(rayAt(.5f))==1,"overlapping or exclusive midpoint is checked exactly once");
    if(!overlap) {
        require(world.count(rayAt(15.f/31.f))==1&&world.count(rayAt(16.f/31.f))==1,
            "nearby dense samples are retained alongside the distinct coarse midpoint");
    }
    const auto before=world.calls.size();world.geometry.boxes.clear();
    require(!findCornerRoute(world,cfg,{startX,-37,0},{0,-1,0},1,clear)&&world.calls.size()>before,
        "planning never reuses contact samples from an earlier call");
}
void bodyQueries() {
    const auto cfg=fc_test::settings();
    for(bool inclined:{false,true})for(bool moving:{false,true})for(bool distant:{false,true}) {
        const Vec from=distant?Vec{131065.836f,41791.117f,-11204.176f}:Vec{3,7,11};
        const Vec to=moving?from+Vec{2,-3,4}:from;EmptyWorld world;
        require(cornerBodyClear(world,cfg,from,to,inclined),"empty body path remains clear");
        const unsigned expected=inclined?(moving?104:48):(moving?32:8);
        require(world.calls.size()==expected,"all body rays are retained after arithmetic hoisting");
        const float tolerance=distant?.025f:.00005f;
        const float radius=cfg.radius+std::max(4.f,cfg.radius*.085f);
        const float lowWidth=std::sqrt(radius*radius-(cfg.radius-6.f)*(cfg.radius-6.f));
        const auto& first=world.calls.front();
        if(moving) {
            const Vec offset{inclined?lowWidth:radius,0,6};
            require((first.from-(from+offset)).length()<tolerance&&(first.to-(to+offset)).length()<tolerance,
                "first swept capsule endpoint retains its physical offset");
        } else if(inclined) {
            const float upper=(6.f+cfg.radius)*.5f;
            const float width=std::sqrt(radius*radius-(cfg.radius-upper)*(cfg.radius-upper));
            require((first.from-(to+Vec{lowWidth,0,6})).length()<tolerance&&
                (first.to-(to+Vec{width,0,upper})).length()<tolerance,
                "rounded-cap chord endpoints remain unchanged when stationary");
        } else require((first.from-(to+Vec{radius,0,6})).length()<tolerance&&
            (first.to-(to+Vec{radius,0,cfg.height})).length()<tolerance,
            "stationary vertical envelope retains its complete height");
        for(unsigned index=0;index<world.calls.size();++index) {
            EmptyWorld blocked;blocked.stop=index+1;
            require(!cornerBodyClear(blocked,cfg,from,to,inclined)&&blocked.calls.size()==index+1,
                "every original body ray can independently reject a newly occupied path");
            require(same(blocked.calls.back(),world.calls[index]),
                "collision short-circuit retains the same ordered ray endpoints");
        }
        EmptyWorld checked;
        require(cornerBodyClear(checked,cfg,from,to,inclined,true),
            "previously checked destination retains the complete remaining sweep");
        require(checked.calls.size()==(moving?(inclined?56:24):0),
            "destination reuse removes only stationary cross-section rays");
        std::size_t index=0;
        for(const auto& ray:checked.calls) {
            while(index<world.calls.size()&&!same(ray,world.calls[index]))++index;
            require(index<world.calls.size(),"every retained sweep keeps its exact original endpoints and order");
            ++index;
        }
        for(unsigned blockedAt=1;blockedAt<=checked.calls.size();++blockedAt) {
            EmptyWorld blocked;blocked.stop=blockedAt;
            require(!cornerBodyClear(blocked,cfg,from,to,inclined,true)&&blocked.calls.size()==blockedAt&&
                same(blocked.calls.back(),checked.calls[blockedAt-1]),
                "each combined-path sweep still independently rejects new collisions");
        }
    }
}
void verticalCore(bool convex,int fps,float direction) {
    RecordedCorner world;world.geometry.corner(convex);Traversal actor;actor.cfg=fc_test::settings();
    require(actor.attach(world,{-155,-37,0},{0,1,0},1000),"vertical performance fixture attaches");
    for(unsigned frame=0;frame<unsigned(fps*3)&&(!actor.turningCorner()||actor.normal.dot({0,-1,0})>.8f);++frame)
        actor.update(world,{1,0},1.f/fps,1000);
    require(actor.turningCorner()&&actor.normal.dot({0,-1,0})<.8f,
        "vertical reuse starts inside a naturally acquired physical corner");
    const Vec before=actor.position;world.calls.clear();
    const auto result=actor.update(world,{0,direction},1.f/fps,1000);
    require(actor.turningCorner()&&!result.released&&(actor.position.z-before.z)*direction>0,
        "pure vertical input continues the same checked corner");
    require(std::hypot(actor.position.x-before.x,actor.position.y-before.y)<.001f,
        "vertical reuse never changes lateral progress on an upright corner");
    unsigned joins=0;
    for(const auto& ray:world.calls)if(std::abs((ray.to-ray.from).length()-15.f)<.00001f) {
        ++joins;require(world.count(ray)==1,"vertical interior reuses its already checked seam within the same update");
    }
    require(joins>=8,"vertical step retains complete current seam validation");
    require(world.geometry.clearance(actor.position,actor.cfg)>=actor.cfg.radius-.03f,
        "vertical reuse retains independent cylinder clearance");
    auto unsupported=actor;RecordedCorner absent=world;absent.geometry.boxes.clear();absent.calls.clear();
    const auto lost=unsupported.update(absent,{0,direction},1.f/fps,1000);
    require(!unsupported.turningCorner()&&unsupported.active()&&!lost.released&&unsupported.geometryHolding()&&
        (unsupported.position-actor.position).length()==0&&!absent.calls.empty(),
        "lost support cancels the stale corner route and holds the last checked root");
    const float heldPhase=unsupported.progress();
    for(int frame=0;frame<fps;++frame) {
        const auto held=unsupported.update(absent,{0,direction},1.f/fps,1000);
        require(!held.released&&held.staminaCost==0&&(unsupported.position-actor.position).length()==0&&unsupported.progress()==heldPhase,
            "unsupported corner retries never move or advance the held root");
    }
    Input release;release.release=true;auto exit=unsupported.update(absent,release,1.f/fps,1000);
    for(int frame=0;frame<fps&&!exit.released;++frame)exit=unsupported.update(absent,{},1.f/fps,1000);
    require(exit.released&&!unsupported.active(),"unsupported corner remains manually releasable");
    const auto risen=actor.position;
    if(direction>0) {
        world.geometry.boxes.push_back({{-400,-400,risen.z+actor.cfg.height+.5f},
            {400,400,risen.z+actor.cfg.height+15},false});
        actor.update(world,{0,direction},1.f/fps,1000);
        require(actor.position.z<risen.z+.51f,"new overhead collision is checked before the next vertical move");
        world.geometry.boxes.pop_back();
    }
}
void landingGuard() {
    const auto cfg=fc_test::settings();RecordedCorner source;source.geometry.corner(true);
    auto route=findCornerRoute(source,cfg,{-45,-37,0},{0,-1,0},1,clear);
    require(route.has_value(),"landing guard fixture plans a real corner");
    route->distance=route->length;const auto endpoint=cornerSample(*route,route->distance);
    const Vec anchor=cornerAnchor(endpoint.position,route->targetNormal,cfg.grip-5.f);
    const Ray landing{anchor+route->targetNormal*12.f,anchor-route->targetNormal*(cfg.gap+20.f)};
    struct RejectLanding final:World {
        RecordedCorner& world;Ray target;unsigned hits{};
        RejectLanding(RecordedCorner& value,Ray query):world(value),target(query){}
        std::optional<Hit> ray(Vec from,Vec to)override {
            if(same(Ray{from,to},target)){++hits;return {};}
            return world.ray(from,to);
        }
    } blocked(source,landing);
    require(cornerGrip(blocked,cfg,*route,endpoint.position,endpoint.normal),
        "offset contacts still support the body when a centered landing contact is absent");
    blocked.hits=0;const auto distance=route->distance;
    require(!advanceCornerRoute(blocked,cfg,*route,0,clear)&&blocked.hits>=2&&route->distance==distance,
        "zero-travel endpoint still validates centered landing in addition to general support");
    require(advanceCornerRoute(source,cfg,*route,0,clear).has_value(),
        "restored centered landing is queried again and accepted");
}
void coreEndpoint(float phase) {
    RecordedCorner world;world.geometry.corner(true);Traversal actor;actor.cfg=fc_test::settings();
    require(actor.attach(world,{-85,-37,0},{0,1,0},1000),"endpoint Core fixture attaches");
    actor.update(world,{1,0},1.f/60,1000);
    require(actor.turningCorner(),"endpoint fixture owns an actually planned route");
    const auto route=TraversalCapture::placeCorner(actor,phase);world.calls.clear();
    const auto result=actor.update(world,{0,1},1.f/60,1000);
    require(!result.released&&actor.turningCorner(),"vertical endpoint remains supported");
    unsigned repeated=0;
    for(const auto& ray:world.calls)if(std::abs((ray.to-ray.from).length()-15.f)<.00001f&&world.count(ray)>=2)++repeated;
    require(repeated>=16,"endpoint keeps advance validation rather than the interior-only reuse");
    if(phase==1.f) {
        const Vec anchor=cornerAnchor(actor.position,route.targetNormal,actor.cfg.grip-5.f);
        const Ray landing{anchor+route.targetNormal*12.f,anchor-route.targetNormal*(actor.cfg.gap+20.f)};
        require(world.count(landing)>=3,"Core endpoint retains the final centered landing check");
    }
}
void diagonalCore(bool convex,int fps) {
    RecordedCorner world;world.geometry.corner(convex);Traversal actor;actor.cfg=fc_test::settings();
    require(actor.attach(world,{-155,-37,0},{0,1,0},1000),"diagonal fixture attaches to actual geometry");
    for(unsigned frame=0;frame<unsigned(fps*3)&&(!actor.turningCorner()||actor.normal.dot({0,-1,0})>.8f);++frame)
        actor.update(world,{1,0},1.f/fps,1000);
    require(actor.turningCorner(),"diagonal fixture acquires a checked turn");
    const Vec start=actor.position;world.calls.clear();
    const auto result=actor.update(world,{1,1},1.f/fps,1000);
    const Vec movement=actor.position-start;
    require(!result.released&&movement.z>0&&std::hypot(movement.x,movement.y)>0,
        "destination reuse preserves simultaneous vertical and lateral movement");
    const Ray combined{start+Vec{actor.cfg.radius+4.f,0,actor.cfg.chest},
        actor.position+Vec{actor.cfg.radius+4.f,0,actor.cfg.chest}};
    require(world.count(combined)>=1,"Core retains the direct combined body sweep after axis-path checks");
    const Vec before=actor.position;
    world.geometry.boxes.push_back({{-400,-400,before.z+actor.cfg.height+.1f},
        {400,400,before.z+actor.cfg.height+15},false});
    actor.update(world,{1,1},1.f/fps,1000);
    require(actor.position.z<=before.z+.11f,"a new obstruction prevents unchecked diagonal ascent on the next frame");
    require(world.geometry.clearance(actor.position,actor.cfg)>=actor.cfg.radius-.03f,
        "diagonal fallback keeps independent complete body clearance");
}
}
int main()try {
    exactDirections();overlappingSamples(-90.f,true);overlappingSamples(-90.25f,false);bodyQueries();landingGuard();
    coreEndpoint(0);coreEndpoint(1);
    for(bool convex:{false,true})for(int fps:{30,60,120})for(float direction:{-1.f,1.f})verticalCore(convex,fps,direction);
    for(bool convex:{false,true})for(int fps:{30,60,120})diagonalCore(convex,fps);
    std::cout<<"CornerPerformanceTests passed: exact contacts, shared samples, complete body paths, live vertical and landing guards\n";
    return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
