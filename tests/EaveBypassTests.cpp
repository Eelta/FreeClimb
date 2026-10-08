#include "Pose.h"
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
using namespace fc;
namespace {
const Library* runtimeLibrary=nullptr;
struct Plane {Vec n;float d;};
using Solid=std::vector<Plane>;
Solid box(Vec lo,Vec hi) {return {{{1,0,0},hi.x},{{-1,0,0},-lo.x},{{0,1,0},hi.y},{{0,-1,0},-lo.y},{{0,0,1},hi.z},{{0,0,-1},-lo.z}};}
void check(bool x,const char* why){if(!x)throw std::runtime_error(why);}
struct EaveWorld final:World {
    struct Shape {Solid planes;bool climbable=true;};
    std::vector<Shape> shapes;Vec origin{};float yaw{};unsigned casts{};
    bool roofMissing{},invalidRoof{};
    explicit EaveWorld(float depth=106,bool crossing=false) {
        shapes.push_back({box({-220,0,-600},{220,350,84.137f})});
        const Vec n{0,-.7540443f,.6568235f};
        shapes.push_back({{{{1,0,0},220},{{-1,0,0},220},{{0,1,0},350},{{0,-1,0},depth},
            {{0,0,-1},-138.137f},{n,n.dot({0,-depth,138.137f})}}});
        if(crossing)shapes.push_back({box({12,-350,138.137f},{220,350,150.137f})});
    }
    Vec rotate(Vec p,float a)const{return {p.x*std::cos(a)-p.y*std::sin(a),p.x*std::sin(a)+p.y*std::cos(a),p.z};}
    Vec global(Vec p)const{return origin+rotate(p,yaw);}
    Vec local(Vec p)const{return rotate(p-origin,-yaw);}
    std::optional<Hit> ray(Vec from,Vec to)override {
        ++casts;const Vec a=local(from),d=local(to)-a;std::optional<Hit> answer;double nearest=2;
        for(std::size_t k=0;k<shapes.size();++k) {
            if(k==1&&roofMissing)continue;const auto& shape=shapes[k];
            double enter=0,leave=1;Vec entering{},leaving{};bool rejected=false,inside=true;
            for(const auto& p:shape.planes) {
                const double dist=a.dot(p.n)-p.d,rate=d.dot(p.n);inside&=dist<-.0001;
                if(std::abs(rate)<1e-9){if(dist>0){rejected=true;break;}continue;}
                const double f=-dist/rate;
                if(rate<0){if(f>enter){enter=f;entering=p.n;}}else if(f<leave){leave=f;leaving=p.n;}
                if(enter>leave){rejected=true;break;}
            }
            const double f=inside?leave:enter;const Vec n=inside?leaving:entering;
            if(!rejected&&f>1e-6&&f<=1&&f<nearest&&n.length()>.9) {
                nearest=f;answer=Hit{global(a+d*float(f)),rotate(n,yaw),shape.climbable&&!(invalidRoof&&k==1)};
            }
        }
        return answer;
    }

    bool bodyInside(Vec feet)const {
        const auto p=local(feet);
        for(float height:{6.f,31.f,70.f,107.f,138.f})for(int ring=0;ring<=32;++ring) {
            const float a=ring*(6.28318530718f/32),radius=ring==32?0.f:31.f;
            const Vec sample=p+Vec{radius*std::cos(a),radius*std::sin(a),height};
            for(std::size_t k=0;k<shapes.size();++k) {
                if(k==1&&roofMissing)continue;bool inside=true;
                for(const auto& plane:shapes[k].planes)inside&=sample.dot(plane.n)<plane.d-.04f;
                if(inside)return true;
            }
        }
        return false;
    }
};
struct Outcome {bool planned{},caught{},released{},safe=true,complete{};float gain{},side{};unsigned peak{};float maxAngle{};};
Outcome run(EaveWorld& w,int fps,bool sprint=false,bool obstacle=false,bool removeTarget=false,bool recoveryEnabled=false) {
    Traversal t;t.cfg.radius=31;t.cfg.gap=37;t.cfg.height=138;t.cfg.approachSeconds=0;t.cfg.wallRunObstacleJumps=recoveryEnabled;
    check(t.attach(w,w.global({0,-37,-130}),w.rotate({0,1,0},w.yaw),1000),"attach real source wall");
    Outcome out;const auto start=t.position;bool changed=false;SurfacePose animator;Pose previous;
    for(int frame=0;frame<fps*6&&t.active();++frame) {
        if(out.planned&&!changed&&(obstacle||removeTarget)) {
            if(obstacle)w.shapes.push_back({box({-220,-180,142},{220,350,147})});
            if(removeTarget)w.roofMissing=true;changed=true;
        }
        w.casts=0;const auto result=t.update(w,{0,1,false,false,false,false,sprint},1.f/fps,1000);
        out.peak=std::max(out.peak,w.casts);
        if(runtimeLibrary&&t.active()) {
            const auto pose=animator.update(*runtimeLibrary,w,t,result.motion,1.f/fps,1);
            check(pose.size()==99,"actual pose supplies the full Skyrim skeleton");
            for(const auto& bone:pose)check(bone.t.finite()&&std::isfinite(bone.q.dot(bone.q))&&std::abs(bone.q.dot(bone.q)-1)<.002f,"finite normalized eave poses");
            for(int hand=0;hand<2;++hand)check(runtimeLibrary->armBendValid(pose,hand),"eave transfer does not reverse elbow bends");
            if(!previous.empty())for(std::size_t i=0;i<pose.size();++i) {
                const float angle=angleBetween(previous[i].q,pose[i].q);
                out.maxAngle=std::max(out.maxAngle,angle);
                if(recoveryEnabled) {
                    const bool parkour=runMotion(result.motion)||
                        (result.motion>=Motion::kickUp&&result.motion<=Motion::kickRight);
                    check(angle<=(parkour?18.849556f:12.566371f)/fps+.015f,
                        "scheduled eave fallback retains the existing per-motion angular bound on every frame");
                }
            }
            previous=pose;
        }
        out.planned|=std::string(t.blockedReason)=="checked eave bypass"||
            (recoveryEnabled&&std::string(t.blockedReason)=="checked wall-run obstacle bypass");
        out.caught|=out.planned&&t.state==State::wall&&t.surfaceNormal.z>.2f;
        out.complete|=result.completed;out.released|=result.released;
        if(result.released)std::cout<<"release frame="<<frame<<" reason="<<result.reason<<" pos="<<w.local(t.position).x<<","<<w.local(t.position).y<<","<<w.local(t.position).z<<"\n";

        out.safe&=!w.bodyInside(t.position);
    }
    const auto delta=w.local(t.position)-w.local(start);out.gain=delta.z;out.side=delta.x;return out;
}
void recoveredOuterCorner(int fps,bool distant) {
    EaveWorld world;world.shapes.clear();world.shapes.push_back({box({-300,0,-300},{0,300,500})});
    if(distant){world.origin={131132.67f,39082.54f,-11827.61f};world.yaw=.61f;}
    Settings cfg;cfg.radius=31;cfg.gap=37;cfg.height=138;
    const auto initial=world.global({10,-51,0}),normal=world.rotate({0,-1,0},world.yaw);

    auto distance=[&](Vec feet) {
        const Vec p=world.local(feet);
        const float x=std::max({-300-p.x,0.f,p.x}),y=std::max({-p.y,0.f,p.y-300});
        const float z=std::max({-300-(p.z+107),0.f,p.z+31-500});
        return std::sqrt(x*x+y*y+z*z);
    };
    auto clear=[&](Vec a,Vec b) {
        for(int i=0;i<=32;++i)if(distance(a+(b-a)*(i/32.f))<30.98f)return false;
        return true;
    };
    auto route=findCornerRoute(world,cfg,initial,normal,1,clear);
    check(route.has_value(),"previously stopped just beyond an outer edge retains a reachable upper hand window");
    for(int i=0;i<=600;++i)check(distance(cornerSample(*route,route->length*i/600.f).position)>=30.98f,
        "recovered corner path clears the independent rounded body");
    bool completed=false;
    for(int frame=0;frame<fps*3&&!completed;++frame) {
        auto next=advanceCornerRoute(world,cfg,*route,82.f/fps,clear);
        check(next.has_value(),"recovered contact persists through the measured ninety-degree turn");
        completed=next->complete;
    }
    check(completed,"recovered outer turn reaches the next real face");
    world.shapes[0].climbable=false;
    check(!findCornerRoute(world,cfg,initial,normal,1,clear),"larger hand window never approves an invalid wall");
    world.shapes.clear();world.shapes.push_back({box({-300,0,-300},{-12,300,500})});
    check(!advanceCornerRoute(world,cfg,*route,1,clear),"displaced join is rejected during playback");
}
}
int main(int argc,char** argv){try {
    Library lib;check(argc>1&&lib.load(argv[1]),"runtime motion library required");runtimeLibrary=&lib;
    int positive=0;
    for(float depth:{66.f,106.f})for(bool crossing:{false,true})for(int fps:{30,60,120}) {
        EaveWorld w(depth,crossing);if(fps==120){w.origin={131675.4f,41941.6f,-11425.7f};w.yaw=.83f;}
        const auto r=run(w,fps);
        std::cout<<"depth="<<depth<<" cross="<<crossing<<" fps="<<fps<<" planned="<<r.planned<<" caught="<<r.caught<<" released="<<r.released<<" safe="<<r.safe<<" gain="<<r.gain<<" side="<<r.side<<" peak="<<r.peak<<" maxAngle="<<r.maxAngle<<'\n';
        check(r.planned&&r.caught&&!r.released&&!r.complete&&r.safe&&r.gain>260,"checked eave route keeps climbing a steep roof");
        check(r.maxAngle<=12.566371f/fps+.015f,"all eave pose transitions retain the bounded angular budget");
        ++positive;
    }
    for(float sourceTop:{64.f,70.f})for(int fps:{30,60,120}) {
        EaveWorld w(66);w.shapes[0].planes=box({-220,0,-600},{220,350,sourceTop});
        const auto result=run(w,fps,true,false,false,true);
        std::cout<<"fallback sourceTop="<<sourceTop<<" fps="<<fps<<" planned="<<result.planned
            <<" caught="<<result.caught<<" gain="<<result.gain<<" safe="<<result.safe<<" maxAngle="<<result.maxAngle<<'\n';
        check(result.planned&&result.caught&&!result.released&&!result.complete&&result.safe&&result.gain>260,
            "a rejected recessed-wall candidate must leave the checked eave fallback reachable through the real update scheduler");
    }
    for(bool invalid:{false,true}) {
        EaveWorld w(invalid?106.f:360.f);w.invalidRoof=invalid;const auto r=run(w,60);
        std::cout<<"negative invalid="<<invalid<<" peak="<<r.peak<<"\n";
        check(!r.planned&&!r.released,"excess depth and invalid surfaces are not bypassed");
    }
    {EaveWorld w;const auto r=run(w,60,true);check(!r.planned,"wall-run controls never launch eave hops");}
    {EaveWorld w;const auto r=run(w,60,false,true);check(r.planned&&r.released&&r.safe,"new path blocker aborts before crossing");}
    {EaveWorld w;const auto r=run(w,60,false,false,true);check(r.planned&&r.released&&!r.caught,"removed target does not create an air catch");}
    for(int fps:{30,60,120})for(bool distant:{false,true})recoveredOuterCorner(fps,distant);
    std::cout<<positive<<" eave scenarios plus six scheduled fallbacks, negative geometry and live changes passed\n";
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
