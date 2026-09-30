#include "Core.h"
#include <iostream>
#include <vector>
using namespace fc;
struct ThinWorld:World {
    struct Tri {Vec a,b,c;};std::vector<Tri> triangles;Vec origin{};float yaw{};unsigned calls{};
    Vec rotate(Vec p,float a)const{return {p.x*std::cos(a)-p.y*std::sin(a),p.x*std::sin(a)+p.y*std::cos(a),p.z};}
    Vec global(Vec p)const{return origin+rotate(p,yaw);}
    Vec local(Vec p)const{return rotate(p-origin,-yaw);}
    void quad(Vec a,Vec b,Vec c,Vec d){triangles.push_back({a,b,c});triangles.push_back({a,c,d});}
    void scene(float width=43.7f,float top=70.17f,float handWidth=160,bool ceiling=false) {
        triangles.clear();
        quad({-handWidth/2,0,-400},{handWidth/2,0,-400},{handWidth/2,0,top},{-handWidth/2,0,top});
        quad({-300,52.9f,19.27f},{300,52.9f,19.27f},{300,52.9f+width,19.27f},{-300,52.9f+width,19.27f});
        if(ceiling)quad({-300,-100,205},{-300,150,205},{300,150,205},{300,-100,205});
    }
    std::optional<Hit> ray(Vec a,Vec b)override {
        ++calls;const auto from=local(a),delta=local(b)-from;float best=2;std::optional<Hit> result;
        for(auto t:triangles) {
            const auto e=t.b-t.a,f=t.c-t.a,h=delta.cross(f);const double determinant=e.dot(h);
            if(std::abs(determinant)<1e-8)continue;
            const double inv=1/determinant;const auto s=from-t.a;const double u=s.dot(h)*inv;
            if(u<0||u>1)continue;const auto q=s.cross(e);const double v=delta.dot(q)*inv;
            if(v<0||u+v>1)continue;const double time=f.dot(q)*inv;
            if(time<=.000001||time>1||time>=best)continue;
            best=float(time);result=Hit{global(from+delta*best),rotate(e.cross(f).unit(),yaw),true};
        }
        return result;
    }
};
static int failures=0;
static void check(bool condition,const char* message){if(!condition){++failures;std::cerr<<"FAIL "<<message<<'\n';}}
static Traversal initial(ThinWorld& world) {
    Traversal t;t.cfg.radius=31;t.cfg.gap=37;t.cfg.height=138;t.cfg.approachSeconds=0;

    t.position=world.global({0,-51,0});t.normal=world.rotate({0,-1,0},world.yaw);t.surfaceNormal=t.normal;t.state=State::wall;return t;
}
static void positive(bool far,float yaw,int fps) {
    ThinWorld w;w.yaw=yaw;if(far)w.origin={131360.953f,37429.633f,-11801.213f};w.scene();
    auto t=initial(w);bool complete=false,unexpectedRelease=false,sawMantle=false;float wallDistance=1e9f;unsigned peak=0;
    for(int frame=0;frame<fps*4;++frame) {
        w.calls=0;const auto r=t.update(w,{0,1,false,true},1.f/fps,1000);peak=std::max(peak,w.calls);
        if(t.state==State::mantle) {
            sawMantle=true;
            for(int hand=0;hand<2;++hand){const auto h=w.local(t.topHand(hand));check(std::abs(h.y)<.04f&&h.z<=70.2f&&h.z>=67.9f,"both palms lie on the real front edge, not the recessed air gap");}
        }
        const auto p=w.local(t.position);

        const float dz=std::max(0.f,p.z+31-70.17f);
        wallDistance=std::min(wallDistance,std::sqrt(p.y*p.y+dz*dz));
        if(r.completed){complete=true;break;}if(r.released){unexpectedRelease=true;break;}
    }
    std::cout<<"thin wall far="<<far<<" yaw="<<yaw<<" fps="<<fps<<" completed="<<complete<<" distance="<<wallDistance<<" rays="<<peak<<'\n';
    check(complete&&sawMantle&&!unexpectedRelease,"thin wall and bounded intermediate landing reach a supported summit");
    check(wallDistance>=30.95f,"independent full capsule clears the higher wall throughout the top-out");
}
static void negative(float width,float top,float hands,bool ceiling,const char* reason) {
    ThinWorld w;w.scene(width,top,hands,ceiling);auto t=initial(w);bool mantle=false,completed=false;
    for(int frame=0;frame<90&&t.active();++frame){const auto r=t.update(w,{0,1,false,true},1.f/60,1000);mantle|=t.state==State::mantle;completed|=r.completed;}
    check(!mantle&&!completed,reason);
}
static void changingPath() {
    ThinWorld w;w.scene();auto t=initial(w);auto r=t.update(w,{0,1,false,true},1.f/60,1000);
    check(t.state==State::mantle,"dynamic obstacle fixture starts a valid top-out");
    w.quad({-300,-100,150},{-300,150,150},{300,150,150},{300,-100,150});
    bool released=false,completed=false;
    for(int frame=0;frame<90&&t.active();++frame){r=t.update(w,{0,1,false,true},1.f/60,1000);released|=r.released;completed|=r.completed;}
    check(released&&!completed,"new ceiling aborts the top-out instead of crossing collision");
}
int main() {
    for(bool far:{false,true})for(float yaw:{0.f,.633f})for(int fps:{30,60,120})positive(far,yaw,fps);
    negative(22,70.17f,160,false,"a platform narrower than the footprint is not a summit");
    negative(43.7f,70.17f,12,false,"a wall without two real reachable hand contacts cannot top out");
    negative(43.7f,70.17f,160,true,"a low ceiling remains blocking");
    negative(43.7f,180,160,false,"a taller wall is not an edge inside reach");
    negative(43.7f,110,160,false,"a recessed standing point over 64 units below the real edge cannot authorize this top-out");
    changingPath();std::cout<<"thin wall failures="<<failures<<'\n';return failures?1:0;
}
