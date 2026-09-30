#include "Core.h"
#include <iostream>
#include <vector>
#include <stdexcept>
using namespace fc;
static void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
struct SteppedTop:World {
    struct Edge{Vec a,b;};std::vector<Edge> edges;Vec origin{};float yaw{};Vec top{};
    bool missingLeft{},missingRight{};unsigned rays{};
    SteppedTop(float step=32,bool far=false) {
        if(far){origin={106816.53f,70554.54f,5204.43f};yaw=.633f;}
        const float low=.197f,cap=.94f,slope=low/std::sqrt(1-low*low),upperSlope=cap/std::sqrt(1-cap*cap);
        const Vec lip{0,240*slope,240},end=lip+Vec{0,30,30/upperSlope};top=end+Vec{0,0,step};
        edges={{{0,-1000*slope,-1000},lip},{lip,end},{end,top},{top,top+Vec{0,2000,0}}};
    }
    Vec rotate(Vec p,float a)const{return {p.x*std::cos(a)-p.y*std::sin(a),p.x*std::sin(a)+p.y*std::cos(a),p.z};}
    Vec global(Vec p)const{return origin+rotate(p,yaw);}
    Vec local(Vec p)const{return rotate(p-origin,-yaw);}
    std::optional<Hit> ray(Vec from,Vec to)override {
        ++rays;const auto a=local(from),b=local(to),d=b-a;std::optional<Hit> result;double nearest=2;
        for(const auto& e:edges){const auto tangent=e.b-e.a;const auto n=Vec{0,-tangent.z,tangent.y}.unit();
            const double f=(a-e.a).dot(n),g=(b-e.a).dot(n);if(f<=.000001||g>0)continue;
            const double t=f/(f-g);const auto p=a+d*float(t);const float segment=(p-e.a).dot(tangent)/tangent.dot(tangent);
            if(t>nearest||segment<-.000001f||segment>1.000001f)continue;
            if(n.z>.99f&&p.y<top.y+30&&((missingLeft&&p.x<-8)||(missingRight&&p.x>8)))continue;
            nearest=t;result=Hit{global(p),rotate(n,yaw),true};
        }
        return result;
    }
    static float pointSegment(Vec p,Vec a,Vec b){const auto d=b-a;return (p-(a+d*std::clamp((p-a).dot(d)/d.dot(d),0.f,1.f))).length();}
    float capsuleBoundaryDistance(Vec feet,float radius,float height)const {
        const auto p=local(feet);const Vec a{0,p.y,p.z+radius},b{0,p.y,p.z+height-radius};float nearest=1e9f;
        auto cross=[](Vec u,Vec v){return u.y*v.z-u.z*v.y;};
        for(auto e:edges){
            const auto u=b-a,v=e.b-e.a;const float divisor=cross(u,v);
            if(std::abs(divisor)>.00001f){const float s=cross(e.a-a,v)/divisor,t=cross(e.a-a,u)/divisor;if(s>=0&&s<=1&&t>=0&&t<=1)return 0;}
            nearest=std::min({nearest,pointSegment(a,e.a,e.b),pointSegment(b,e.a,e.b),pointSegment(e.a,a,b),pointSegment(e.b,a,b)});
        }
        return nearest;
    }
};
static Traversal attached(SteppedTop& world){
    Traversal t;t.cfg.approachSeconds=0;t.cfg.radius=31;t.cfg.gap=37;t.cfg.height=138;
    require(t.attach(world,world.global({0,-37,0}),world.rotate({0,1,0},world.yaw),1000,35),"stepped summit starts at a real supported lower face");return t;
}
int main(){try {
    for(int fps:{30,48,120})for(bool far:{false,true}) {
        SteppedTop world(32,far);auto t=attached(world);Result r;bool mantle=false,nearEdge=false;unsigned peak=0;
        float minimumClearance=1e9f;Traversal beforeTop;bool saved=false;
        for(int frame=0;frame<fps*10&&t.active();++frame){
            const auto before=t;world.rays=0;r=t.update(world,{0,1,false,true},1.f/fps,1000);peak=std::max(peak,world.rays);
            minimumClearance=std::min(minimumClearance,world.capsuleBoundaryDistance(t.position,t.cfg.radius,t.cfg.height));
            if(t.state==State::mantle&&!mantle){
                beforeTop=before;saved=true;mantle=true;
                const auto start=world.local(before.position),stand=world.local(t.topTarget());
                std::cout<<"selected top fps="<<fps<<" far="<<far<<" before="<<start.y<<','<<start.z
                    <<" stand="<<stand.y<<','<<stand.z<<" actualUpper="<<world.top.y<<','<<world.top.z<<'\n';
                for(int hand=0;hand<2;++hand){
                    const auto contact=world.local(t.topHand(hand));nearEdge|=contact.y<world.top.y+10;
                    std::cout<<" hand="<<hand<<" contact="<<contact.x<<','<<contact.y<<','<<contact.z<<'\n';
                    require(std::abs(contact.z-world.top.z)<.03f&&contact.y>=world.top.y,
                        "both authored top contacts lie on the actual upper platform rather than its lower shelf");
                    const auto hit=world.ray(t.topHand(hand)+Vec{0,0,2},t.topHand(hand)-Vec{0,0,2});
                    require(hit&&hit->normal.z>.99f,"each selected edge palm retains a real top ray hit");
                }
            }
        }
        std::cout<<"stepped summit fps="<<fps<<" far="<<far<<" completed="<<r.completed<<" minCapsuleDistance="<<minimumClearance<<" radius="<<t.cfg.radius<<" peakCasts="<<peak<<'\n';
        require(saved&&nearEdge&&r.completed&&!t.active(),"a reachable real second edge must finish the old stalled stepped-summit route");
        require(minimumClearance>=t.cfg.radius-.15f,"the whole capsule stays outside every actual profile segment through top-out");

        for(int variant=0;variant<5;++variant){
            SteppedTop unsafe=world;
            if(variant==0)unsafe.missingLeft=true;
            if(variant==1)unsafe.missingRight=true;
            if(variant==2){unsafe.edges[2].b.z+=200;unsafe.edges[3].a.z+=200;unsafe.edges[3].b.z+=200;}
            if(variant==3){const float z=world.top.z+90,y=world.top.y-20;
                unsafe.edges.push_back({{0,y,z},{0,y,z+2000}});unsafe.edges.push_back({{0,y+2000,z},{0,y,z}});}
            if(variant==4){unsafe.edges.erase(unsafe.edges.begin()+1,unsafe.edges.begin()+3);unsafe.edges.back().a.y+=200;unsafe.edges.back().b.y+=200;}
            auto rejected=beforeTop;const auto result=rejected.update(unsafe,{0,1,false,true},1.f/fps,1000);
            std::cout<<" unsafe variant="<<variant<<" mantle="<<(rejected.state==State::mantle)<<" completed="<<result.completed<<" top="<<rejected.ledgeReason<<'\n';
            require(rejected.state!=State::mantle&&!result.completed,
                "edge search cannot bypass absent left/right palms, excessive height, overhead collision or an unreachable separated platform");
            require(result.staminaCost<1,"a rejected unsafe top does not consume a mantle charge");
        }
    }
    std::cout<<"PASS: measured second platform edges and five rejected unsafe variants\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
