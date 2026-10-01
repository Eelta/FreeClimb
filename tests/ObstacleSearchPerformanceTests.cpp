#include "CornerTestWorld.h"
#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>
using namespace fc;
using fc_test::CornerWorld;
namespace {
void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
struct Plane {Vec n;float d;};
using Solid=std::vector<Plane>;
Solid box(Vec lo,Vec hi) {return {{{1,0,0},hi.x},{{-1,0,0},-lo.x},{{0,1,0},hi.y},{{0,-1,0},-lo.y},{{0,0,1},hi.z},{{0,0,-1},-lo.z}};}
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

struct Measurement {
    std::vector<unsigned> queries;
    std::uint64_t total{},tail{};
    unsigned peak{},heavy{},tailHeavy{},stationary{};
    int planned=-1,landed=-1;
    bool released{},completed{};
};
unsigned& queries(CornerWorld& world){return world.rays;}
unsigned& queries(EaveWorld& world){return world.casts;}
bool safe(const CornerWorld& world,const Traversal& actor){return world.clearance(actor.position,actor.cfg)+.05f>=actor.cfg.radius;}
bool safe(const EaveWorld& world,const Traversal& actor){return !world.bodyInside(actor.position);}
bool recovery(const Traversal& actor) {
    const std::string_view reason=actor.blockedReason;
    return actor.state==State::action&&(actor.obstacleJumpActive()||reason.find("checked eave")!=reason.npos||reason.find("checked recessed")!=reason.npos);
}
template<class Scene> void run(const char* name,Scene& world,int fps,Vec start,bool sprint,bool expectRoute,bool rejectRoute,bool allowRelease=false) {
    Traversal actor;actor.cfg=fc_test::settings();actor.cfg.runSpeed=379.5f;actor.cfg.wallRunObstacleJumps=true;
    check(actor.attach(world,world.global(start),world.global({0,1,0})-world.global({0,0,0}),1000),"performance fixture attaches to real support");
    Input input{0,1,false,true,false,false,sprint};
    Measurement measured;
    for(int frame=0;frame<fps*6&&actor.active();++frame) {
        const auto before=actor.position;
        queries(world)=0;
        const auto result=actor.update(world,input,1.f/fps,1000);
        const auto count=queries(world);
        measured.queries.push_back(count);measured.total+=count;measured.peak=std::max(measured.peak,count);
        measured.heavy+=count>=1000;
        if(frame>=fps*4){measured.tail+=count;measured.tailHeavy+=count>=1000;}
        if((actor.position-before).length()<.001f)++measured.stationary;
        if(measured.planned<0&&recovery(actor))measured.planned=frame;
        if(measured.planned>=0&&measured.landed<0&&actor.state==State::wall)measured.landed=frame;
        measured.released|=result.released;measured.completed|=result.completed;
        if(!safe(world,actor))std::cerr<<name<<" fps="<<fps<<" frame="<<frame<<" unsafe at "<<actor.position.x<<','<<actor.position.y<<','<<actor.position.z<<'\n';
        check(safe(world,actor),"every actual frame retains independent body clearance");
        check(allowRelease||!result.released,"supported performance fixtures do not release while searching");
        if(rejectRoute)check(measured.planned<0,"blocked geometry never starts an unchecked recovery");
    }
    auto ordered=measured.queries;std::sort(ordered.begin(),ordered.end());
    const auto p95=ordered.empty()?0:ordered[std::min(ordered.size()-1,ordered.size()*95/100)];
    const auto position=world.local(actor.position);
    std::cout<<name<<','<<fps<<','<<measured.queries.size()<<','<<measured.total<<','<<measured.peak<<','<<p95<<','
        <<std::fixed<<std::setprecision(2)<<(measured.queries.empty()?0.:double(measured.total)/measured.queries.size())<<','
        <<measured.tail<<','<<measured.heavy<<','<<measured.tailHeavy<<','<<measured.stationary<<','
        <<measured.planned<<','<<measured.landed<<','<<position.x<<','<<position.y<<','<<position.z<<','
        <<measured.released<<','<<measured.completed<<'\n';
    std::cout.flush();
    if(expectRoute)check(measured.planned>=0&&measured.landed>measured.planned,"feasible recovery still starts and returns to actual wall support");
}
CornerWorld beam(float depth,float thickness,bool upper=true) {
    CornerWorld world;
    world.boxes.push_back({{-3000,0,-3000},{3000,500,upper?3000.f:200.f}});
    world.boxes.push_back({{-3000,-depth,200},{3000,20,200+thickness},upper});
    return world;
}
CornerWorld runBeam(bool blocked) {
    CornerWorld world;world.boxes.push_back({{-10000,0,-10000},{10000,1000,10000}});
    world.boxes.push_back({{-10000,blocked?-1000.f:-30.f,600},{10000,20,612}});
    return world;
}
CornerWorld recessed(bool blocked,bool invalid=false) {
    CornerWorld world;world.boxes={{{-220,0,-600},{220,350,380}},{{-220,40,380},{220,350,4000},!invalid}};
    if(blocked)world.boxes.push_back({{-220,-180,450},{220,350,458}});
    return world;
}
template<class Scene,class Change> void changedObstacle(const char* name,Scene& world,int fps,Vec start,bool sprint,Change change) {
    Traversal actor;actor.cfg=fc_test::settings();actor.cfg.runSpeed=379.5f;actor.cfg.wallRunObstacleJumps=true;
    check(actor.attach(world,world.global(start),world.global({0,1,0})-world.global({0,0,0}),1000),"changing fixture attaches to real support");
    const Input input{0,1,false,true,false,false,sprint};
    int planned=-1,landed=-1;unsigned peak=0;std::uint64_t total=0;
    for(int frame=0;frame<fps*6&&actor.active();++frame) {
        if(frame==fps*3)change(world);
        queries(world)=0;
        const auto result=actor.update(world,input,1.f/fps,1000);
        peak=std::max(peak,queries(world));total+=queries(world);
        check(safe(world,actor),"geometry changes retain independent body clearance");
        check(!result.released,"newly feasible route retains its existing support");
        if(frame<fps*3)check(!recovery(actor),"blocked geometry stays rejected before the obstacle changes");
        if(planned<0&&recovery(actor))planned=frame;
        if(planned>=0&&actor.state==State::wall){landed=frame;break;}
    }
    std::cerr<<name<<" fps="<<fps<<" peak="<<peak<<" total="<<total<<" retry_frames="
        <<(planned<0?-1:planned-fps*3)<<" landed="<<(landed>=0)<<'\n';
    check(planned>=fps*3&&planned<=fps*3+fps,"newly clear or supported obstacles retry within one second");
    check(landed>planned,"retry after a geometry change completes a checked supported route");
}
void matrix(int fps) {
    for(bool sprint:{false,true}) {
        const auto suffix=sprint?"run":"climb";
        auto open=beam(96,19);open.boxes.resize(1);
        run((std::string("open_")+suffix).c_str(),open,fps,{0,-37,-100},sprint,false,true);
        auto valid=beam(96,19);
        run((std::string("beam_valid_")+suffix).c_str(),valid,fps,{0,-37,-100},sprint,true,false);
        auto deep=beam(300,19);
        run((std::string("beam_deep_")+suffix).c_str(),deep,fps,{0,-37,-100},sprint,false,true);
        auto thick=beam(96,200);thick.boxes[1].climbable=false;
        run((std::string("beam_thick_")+suffix).c_str(),thick,fps,{0,-37,-100},sprint,false,true);
    }
    for(bool crossing:{false,true}) {
        EaveWorld world(106,crossing);
        run(crossing?"eave_crossing":"eave_valid",world,fps,{0,-37,-130},false,true,false);
    }
    EaveWorld deep(360);
    run("eave_deep",deep,fps,{0,-37,-130},false,false,true);
    EaveWorld invalid(106);invalid.invalidRoof=true;
    run("eave_invalid",invalid,fps,{0,-37,-130},false,false,true);
    auto projecting=runBeam(false);
    run("run_beam_valid",projecting,fps,{0,-37,100},true,true,false);
    auto ceiling=runBeam(true);
    run("run_beam_ceiling",ceiling,fps,{0,-37,100},true,false,true);
    auto compact=runBeam(false);compact.boxes.push_back({{-10000,-240,-10000},{10000,-230,10000},false});
    run("run_beam_compact",compact,fps,{0,-37,100},true,true,false);
    auto recess=recessed(false);
    run("recess_valid",recess,fps,{0,-37,250},true,true,false);
    auto recessBlocked=recessed(true);
    run("recess_ceiling",recessBlocked,fps,{0,-37,250},true,false,true,true);
    auto recessInvalid=recessed(false,true);
    run("recess_invalid",recessInvalid,fps,{0,-37,250},true,false,true,true);
    auto changedBeam=beam(300,19);
    changedObstacle("changed_beam",changedBeam,fps,{0,-37,-100},false,[](CornerWorld& world){world.boxes[1].low.y=-96;});
    EaveWorld changedEave(106);changedEave.invalidRoof=true;
    changedObstacle("changed_eave",changedEave,fps,{0,-37,-130},false,[](EaveWorld& world){world.invalidRoof=false;});
    auto changedRecess=recessed(true);
    changedObstacle("changed_recess",changedRecess,fps,{0,-37,250},true,[](CornerWorld& world){world.boxes.pop_back();});
}
}
int main(int argc,char** argv)try {
    std::cout<<"scenario,fps,frames,total_rays,peak_rays,p95_rays,mean_rays,tail_2s_rays,heavy_frames,tail_heavy_frames,stationary_frames,plan_frame,land_frame,final_x,final_y,final_z,released,completed\n";
    if(argc>1)matrix(std::stoi(argv[1]));else for(int fps:{30,60,120})matrix(fps);
    return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
