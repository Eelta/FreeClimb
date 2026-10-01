#include "Core.h"
#include <bit>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
using namespace fc;
namespace {
void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
struct Plane {Vec n;float d;};
using Solid=std::vector<Plane>;
Solid box(Vec lo,Vec hi){return {{{1,0,0},hi.x},{{-1,0,0},-lo.x},{{0,1,0},hi.y},{{0,-1,0},-lo.y},{{0,0,1},hi.z},{{0,0,-1},-lo.z}};}
struct PlaneWorld final:World {
    struct Shape {Solid planes;bool climbable=true;};
    std::vector<Shape> shapes;
    Vec origin{};float yaw{};unsigned calls{};
    std::set<std::array<std::uint32_t,6>> endpoints;
    Vec rotate(Vec p,float angle)const{return {p.x*std::cos(angle)-p.y*std::sin(angle),p.x*std::sin(angle)+p.y*std::cos(angle),p.z};}
    Vec global(Vec p)const{return origin+rotate(p,yaw);}
    Vec local(Vec p)const{return rotate(p-origin,-yaw);}
    void reset(){calls=0;endpoints.clear();}
    std::optional<Hit> intersect(Vec from,Vec to)const {
        const Vec a=local(from),delta=local(to)-a;std::optional<Hit> answer;double nearest=2;
        for(const auto& shape:shapes) {
            double enter=0,leave=1;Vec entering{},leaving{};bool rejected=false,inside=true;
            for(const auto& plane:shape.planes) {
                const double distance=a.dot(plane.n)-plane.d,rate=delta.dot(plane.n);inside&=distance<-.0001;
                if(std::abs(rate)<1.e-9){if(distance>0){rejected=true;break;}continue;}
                const double fraction=-distance/rate;
                if(rate<0){if(fraction>enter){enter=fraction;entering=plane.n;}}
                else if(fraction<leave){leave=fraction;leaving=plane.n;}
                if(enter>leave){rejected=true;break;}
            }
            const double fraction=inside?leave:enter;const Vec normal=inside?leaving:entering;
            if(!rejected&&fraction>1.e-6&&fraction<=1&&fraction<nearest&&normal.length()>.9f) {
                nearest=fraction;answer=Hit{global(a+delta*float(fraction)),rotate(normal,yaw),shape.climbable};
            }
        }
        return answer;
    }
    std::optional<Hit> ray(Vec from,Vec to)override {
        ++calls;std::array<std::uint32_t,6> key;unsigned i=0;
        for(float v:{from.x,from.y,from.z,to.x,to.y,to.z})key[i++]=std::bit_cast<std::uint32_t>(v==0?0.f:v);
        endpoints.insert(key);return intersect(from,to);
    }
    float cylinderClearance(Vec globalFeet,const Settings& cfg)const {
        const Vec feet=local(globalFeet);float minimum=10000;
        for(const auto& shape:shapes) {
            auto planes=shape.planes;planes.push_back({{0,0,1},feet.z+cfg.height});planes.push_back({{0,0,-1},-feet.z-6});
            std::vector<Vec> points;
            for(std::size_t a=0;a<planes.size();++a)for(std::size_t b=a+1;b<planes.size();++b)for(std::size_t c=b+1;c<planes.size();++c) {
                const auto& p=planes[a];const auto& q=planes[b];const auto& r=planes[c];
                const float determinant=p.n.dot(q.n.cross(r.n));if(std::abs(determinant)<1.e-6f)continue;
                const Vec vertex=(q.n.cross(r.n)*p.d+r.n.cross(p.n)*q.d+p.n.cross(q.n)*r.d)/determinant;
                bool inside=true;for(const auto& plane:planes)if(vertex.dot(plane.n)>plane.d+.002f){inside=false;break;}
                if(inside)points.push_back({vertex.x,vertex.y,0});
            }
            if(points.size()<3)continue;
            std::sort(points.begin(),points.end(),[](Vec a,Vec b){return a.x<b.x||(a.x==b.x&&a.y<b.y);});
            std::vector<Vec> hull;
            for(Vec p:points){while(hull.size()>=2&&(hull.back()-hull[hull.size()-2]).cross(p-hull.back()).z<=0)hull.pop_back();hull.push_back(p);}
            const auto lower=hull.size();
            for(auto i=points.rbegin()+1;i!=points.rend();++i){while(hull.size()>lower&&(hull.back()-hull[hull.size()-2]).cross(*i-hull.back()).z<=0)hull.pop_back();hull.push_back(*i);}
            if(hull.size()<4)continue;hull.pop_back();bool inside=true;float distance=10000;
            for(std::size_t i=0;i<hull.size();++i) {
                const Vec a=hull[i],b=hull[(i+1)%hull.size()],edge=b-a,offset=Vec{feet.x,feet.y,0}-a;
                if(edge.cross(offset).z<-.001f)inside=false;
                const float fraction=std::clamp(offset.dot(edge)/std::max(.0001f,edge.dot(edge)),0.f,1.f);
                distance=std::min(distance,(offset-edge*fraction).length());
            }
            minimum=std::min(minimum,inside?0.f:distance);
        }
        return minimum;
    }
    bool upperSupport(const Traversal& actor)const {
        const Vec plane=actor.surfaceNormal.unit(),facing=Vec{plane.x,plane.y,0}.unit(),right{-facing.y,facing.x,0};
        const Vec tangent=(Vec{0,0,1}-plane*plane.z).unit();
        for(float height:{actor.cfg.chest,actor.cfg.grip})for(bool inclined:{false,true}) {
            const Vec center=actor.position+(inclined?Vec{0,0,6}+tangent*(height-6):Vec{0,0,height});
            for(float side:{-10.f,0.f,10.f}) {
                const Vec a=center+right*side;
                const auto first=intersect(a+facing*16,a-facing*(actor.cfg.reach+20));
                const auto second=intersect(a+right*10+facing*16,a+right*10-facing*(actor.cfg.reach+20));
                if(first&&second&&first->climbable&&second->climbable&&first->normal.z>=-.45f&&
                    first->normal.dot(second->normal)>.7f&&(first->point-second->point).length()<45)return true;
            }
        }
        return false;
    }
};
struct Scenario {
    std::string name;PlaneWorld world;Vec start{0,-37,-100};
    bool blocked{},route{},sprint{},change{};int removeShape=-1;
};
PlaneWorld flat(){PlaneWorld world;world.shapes.push_back({box({-1000,0,-3000},{1000,1200,3000})});return world;}
PlaneWorld fold(float nz,float turn=200,float nx=0) {
    PlaneWorld world;world.shapes.push_back({box({-1000,0,-3000},{1000,1200,turn})});
    auto upper=box({-1000,-3000,turn},{1000,1200,3000});const Vec n{nx,-std::sqrt(1-nz*nz-nx*nx),nz};
    upper.push_back({n,nz*turn});if(nx!=0)upper.push_back({{-nx,n.y,n.z},nz*turn});
    world.shapes.push_back({upper});return world;
}
void ceiling(PlaneWorld& world,float z=360,float depth=500,float xmin=-1000){world.shapes.push_back({box({xmin,-depth,z},{1000,1200,z+200}),false});}
PlaneWorld eave(float depth=106,bool crossing=false) {
    PlaneWorld world;world.shapes.push_back({box({-220,0,-600},{220,350,84.137f})});const Vec n{0,-.7540443f,.6568235f};
    world.shapes.push_back({{{{1,0,0},220},{{-1,0,0},220},{{0,1,0},350},{{0,-1,0},depth},{{0,0,-1},-138.137f},{n,n.dot({0,-depth,138.137f})}}});
    if(crossing)world.shapes.push_back({box({12,-350,138.137f},{220,350,150.137f})});return world;
}
std::vector<Scenario> scenarios() {
    std::vector<Scenario> out;
    out.push_back({"flat_open",flat()});
    auto flatCeiling=flat();ceiling(flatCeiling,200);out.push_back({"flat_ceiling",flatCeiling,{0,-37,-100},true});
    auto gentle=fold(-.1f,-1000);out.push_back({"overhang_open_10",gentle,{0,-150,-100}});
    ceiling(gentle,360);out.push_back({"overhang_ceiling_10",gentle,{0,-150,-100},true});
    for(float nz:{-.15f,-.30f,-.44f,-.50f}) {
        const auto suffix=std::to_string(int(std::round(-nz*100)));
        out.push_back({"fold_open_"+suffix,fold(nz),{0,-37,-100},nz<-.45f,nz==-.15f});
        auto blocked=fold(nz);ceiling(blocked);out.push_back({"fold_ceiling_"+suffix,blocked,{0,-37,-100},true});
    }
    for(float nx:{.2f,.4f}) {
        auto world=fold(-.3f,200,nx);const auto suffix=std::to_string(int(nx*100));
        out.push_back({"convex_fold_"+suffix,world});ceiling(world);
        out.push_back({"convex_ceiling_"+suffix,world,{0,-37,-100},true});
    }
    auto shoulder=fold(-.30f);ceiling(shoulder,360,500,20);out.push_back({"overhang_head_corner",shoulder,{0,-37,-100},true});
    auto beam=flat();beam.shapes.push_back({box({-1000,-96,200},{1000,20,219})});
    out.push_back({"beam_valid",beam,{0,-37,-100},false,true});
    out.push_back({"eave_crossing_valid",eave(106,true),{0,-37,-130},false,true});
    out.push_back({"eave_deep",eave(360),{0,-37,-130},true});
    auto changed=fold(-.10f,-1000);ceiling(changed);out.push_back({"changed_overhang_ceiling",changed,{0,-150,-100},false,false,false,true,2});
    auto changedBeam=flat();changedBeam.shapes.push_back({box({-1000,-300,200},{1000,20,219})});
    out.push_back({"changed_deep_beam",changedBeam,{0,-37,-100},false,true,false,true,1});
    return out;
}
struct Measurement {
    std::vector<unsigned> counts;std::uint64_t total{},unique{},tail{},tailUnique{};unsigned peak{},stationary{},over1000{};
    int planned=-1,landed=-1,resumed=-1;bool released{},completed{};float minimumClearance=10000,changeHeight{};
};
bool recovery(const Traversal& actor){return actor.state==State::action;}
void run(Scenario scenario,int fps,std::ostream* frames,bool enforcePerformance) {
    auto& world=scenario.world;Traversal actor;actor.cfg.gap=37;actor.cfg.radius=31;actor.cfg.height=138;actor.cfg.approachSeconds=0;
    actor.cfg.runSpeed=379.5f;actor.cfg.wallRunObstacleJumps=true;
    if(!actor.attach(world,world.global(scenario.start),world.rotate({0,1,0},world.yaw),1000)) {
        std::cerr<<scenario.name<<" attach="<<name(actor.lastFailure)<<'\n';check(false,"performance fixture attaches to real supporting geometry");
    }
    Input input{0,1,false,true,false,false,scenario.sprint};Measurement m;
    for(int frame=0;frame<fps*8&&actor.active();++frame) {
        if(scenario.change&&frame==fps*4) {
            m.changeHeight=world.local(actor.position).z;
            if(scenario.name=="changed_deep_beam")world.shapes[scenario.removeShape].planes=box({-1000,-96,200},{1000,20,219});
            else world.shapes.erase(world.shapes.begin()+scenario.removeShape);
        }
        const Vec before=actor.position,beforeNormal=actor.surfaceNormal;world.reset();const auto result=actor.update(world,input,1.f/fps,1000);
        const auto count=world.calls,unique=unsigned(world.endpoints.size());m.counts.push_back(count);m.total+=count;m.unique+=unique;
        m.peak=std::max(m.peak,count);m.over1000+=count>=1000;if(frame>=fps*6){m.tail+=count;m.tailUnique+=unique;}
        m.stationary+=(actor.position-before).length()<.001f;m.released|=result.released;m.completed|=result.completed;
        if(scenario.change&&frame>=fps*4&&m.resumed<0&&world.local(actor.position).z>m.changeHeight+.5f)m.resumed=frame;
        if(m.planned<0&&recovery(actor))m.planned=frame;
        if(m.planned>=0&&m.landed<0&&actor.state==State::wall)m.landed=frame;
        const float clearance=world.cylinderClearance(actor.position,actor.cfg);m.minimumClearance=std::min(m.minimumClearance,clearance);
        if(frames)*frames<<std::setprecision(9)<<scenario.name<<','<<fps<<','<<frame<<','<<count<<','<<unique<<','<<(count-unique)<<','<<int(actor.state)<<','
            <<actor.position.x<<','<<actor.position.y<<','<<actor.position.z<<','<<actor.surfaceNormal.z<<','<<(actor.position-before).length()<<','
            <<(actor.surfaceNormal-beforeNormal).length()<<','<<actor.stalledSeconds()<<','<<actor.blockedReason<<'\n';
        if(clearance+.06f<actor.cfg.radius||result.released||((actor.state==State::wall)&&!world.upperSupport(actor)))
            std::cerr<<scenario.name<<" fps="<<fps<<" frame="<<frame<<" pos="<<actor.position.x<<','<<actor.position.y<<','<<actor.position.z
                <<" clearance="<<clearance<<" support="<<world.upperSupport(actor)<<" state="<<int(actor.state)<<" reason="<<actor.blockedReason<<'\n';
        check(clearance+.06f>=actor.cfg.radius,"actual frame has independent full cylinder clearance");
        check(!result.released,"supported search never loses real source support");
        check(actor.state!=State::wall||world.upperSupport(actor),"every attached wall frame retains real neighbouring upper contacts");
        if(scenario.blocked)check(!result.completed&&!recovery(actor),"infeasible geometry never starts an unchecked route");
        if(scenario.change&&frame<fps*4)check(!recovery(actor),"blocked fixture has no recovery before geometry changes");
    }
    auto ordered=m.counts;std::sort(ordered.begin(),ordered.end());const auto p95=ordered.empty()?0:ordered[std::min(ordered.size()-1,ordered.size()*95/100)];
    const Vec final=world.local(actor.position);
    std::cout<<scenario.name<<','<<fps<<','<<m.counts.size()<<','<<m.total<<','<<m.peak<<','<<p95<<','<<m.tail<<','
        <<m.unique<<','<<(m.total-m.unique)<<','<<(m.tail-m.tailUnique)<<','<<m.stationary<<','<<m.over1000<<','<<m.planned<<','<<m.landed<<','
        <<std::fixed<<std::setprecision(3)<<final.x<<','<<final.y<<','<<final.z<<','<<m.minimumClearance<<','<<m.released<<','<<m.completed<<'\n';
    std::cout.flush();
    if(enforcePerformance) {
        if(scenario.name=="overhang_ceiling_10") {
            check(m.peak<=3000,"blocked overhang planning stays below three thousand rays per frame");
            check(m.tail<=std::uint64_t(fps)*2000,"sustained blocked overhang averages at most one thousand rays per frame");
        }
        if(scenario.name=="fold_open_15")check(m.peak<=900,"feasible outward fold recovery avoids repeated thousand-ray search bursts");
        if(scenario.name=="flat_open")check(m.peak<=80,"ordinary flat-wall traversal preserves its low-query fast path");
    }
    if(scenario.route)check(m.planned>=0&&m.landed>m.planned,"feasible recovery completes onto real wall support");
    if(scenario.change){check(final.z>m.changeHeight+100,"newly feasible geometry resumes upward progress");check(m.resumed>=fps*4&&m.resumed<=fps*5,"newly clear geometry restores upward progress within one second");}
    if(scenario.blocked)check(m.stationary>=unsigned(fps),"blocked fixtures exercise sustained stationary searching");
    if(scenario.name=="fold_open_15")check(final.z>450,"feasible outward fold recovers and continues above the joint");
    if(scenario.name=="overhang_open_10")check(final.z>scenario.start.z+700,"ordinary outward sloping wall retains upward movement");
    if(scenario.name=="flat_open")check(final.z>scenario.start.z+700,"ordinary flat wall retains upward movement");
}
}
int main(int argc,char** argv)try {
    std::ofstream frames;if(argc>2){frames.open(argv[2]);check(bool(frames),"per-frame report opens");frames<<"scenario,fps,frame,rays,unique_rays,duplicate_rays,state,x,y,z,normal_z,position_delta,normal_delta,stalled,reason\n";}
    std::cout<<"scenario,fps,frames,total_rays,peak_rays,p95_rays,tail_2s_rays,unique_rays,duplicate_rays,tail_duplicate_rays,stationary_frames,frames_over_1000,plan_frame,land_frame,final_x,final_y,final_z,minimum_clearance,released,completed\n";
    const int selected=argc>1?std::stoi(argv[1]):0;
    const bool enforcePerformance=argc<=3||std::string_view(argv[3])!="--measure-only";
    for(int fps:{30,40,60,120})if(!selected||selected==fps)for(auto scenario:scenarios())run(std::move(scenario),fps,frames?&frames:nullptr,enforcePerformance);
    return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
