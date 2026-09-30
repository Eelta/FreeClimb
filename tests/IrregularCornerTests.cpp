#include "Pose.h"
#include <iostream>
#include <vector>
#include <limits>
#include <cmath>
using namespace fc;
using Vec=fc::Vec;
struct GeoTri{Vec a,b,c;};
float pointGeoTriDistance(Vec p,const GeoTri& t){
    const Vec ab=t.b-t.a,ac=t.c-t.a,ap=p-t.a;const float d1=ab.dot(ap),d2=ac.dot(ap);
    if(d1<=0&&d2<=0)return ap.length();const Vec bp=p-t.b;const float d3=ab.dot(bp),d4=ac.dot(bp);
    if(d3>=0&&d4<=d3)return bp.length();const float vc=d1*d4-d3*d2;
    if(vc<=0&&d1>=0&&d3<=0)return (p-(t.a+ab*(d1/(d1-d3)))).length();
    const Vec cp=p-t.c;const float d5=ab.dot(cp),d6=ac.dot(cp);if(d6>=0&&d5<=d6)return cp.length();const float vb=d5*d2-d1*d6;
    if(vb<=0&&d2>=0&&d6<=0)return (p-(t.a+ac*(d2/(d2-d6)))).length();const float va=d3*d6-d5*d4;
    if(va<=0&&(d4-d3)>=0&&(d5-d6)>=0)return (p-(t.b+(t.c-t.b)*((d4-d3)/((d4-d3)+(d5-d6))))).length();
    const float den=1/(va+vb+vc);return (p-(t.a+ab*(vb*den)+ac*(vc*den))).length();
}
float segmentDistance(Vec p,Vec q,Vec a,Vec b){
    Vec d1=q-p,d2=b-a,r=p-a;float aa=d1.dot(d1),ee=d2.dot(d2),f=d2.dot(r),s=0,t=0;
    if(aa<=1.e-8&&ee<=1.e-8)return r.length();
    if(aa<=1.e-8)t=std::clamp(f/ee,0.f,1.f);
    else {float c=d1.dot(r);if(ee<=1.e-8)s=std::clamp(-c/aa,0.f,1.f);else{float bb=d1.dot(d2),den=aa*ee-bb*bb;if(std::abs(den)>1.e-8)s=std::clamp((bb*f-c*ee)/den,0.f,1.f);t=(bb*s+f)/ee;if(t<0){t=0;s=std::clamp(-c/aa,0.f,1.f);}else if(t>1){t=1;s=std::clamp((bb-c)/aa,0.f,1.f);}}}
    return (p+d1*s-a-d2*t).length();
}

struct DistanceWorld {std::vector<GeoTri> triangles;
    float capsuleDistance(Vec feet,float radius=31,float height=138)const {

        const Vec p=feet+Vec{0,0,radius},q=feet+Vec{0,0,height-radius},delta=q-p;float nearest=1.e9f;
        for(const auto& t:triangles){const Vec e=t.b-t.a,f=t.c-t.a,h=delta.cross(f);const double det=e.dot(h);
            if(std::abs(det)>1.e-7){const Vec a=p-t.a,k=a.cross(e);const double u=a.dot(h)/det,v=delta.dot(k)/det,s=f.dot(k)/det;if(u>=0&&v>=0&&u+v<=1&&s>=0&&s<=1)return 0;}
            nearest=std::min({nearest,pointGeoTriDistance(p,t),pointGeoTriDistance(q,t),segmentDistance(p,q,t.a,t.b),segmentDistance(p,q,t.b,t.c),segmentDistance(p,q,t.c,t.a)});
        }return nearest;
    }
};
namespace {
unsigned failures=0,cases=0;
void require(bool value,const char* text) {if(!value){++failures;std::cerr<<"FAIL "<<text<<'\n';}}
struct TriangleWorld:World {
    DistanceWorld solid;
    Vec shear{},origin{};float yaw=0,mirror=1;unsigned casts{};
    Vec point(Vec p)const {p.x+=shear.x*p.z;p.y+=shear.y*p.z;p.x*=mirror;return origin+cornerRotate(p,yaw);}
    Vec normal(Vec n)const {n.z-=n.x*shear.x+n.y*shear.y;n.x*=mirror;return cornerRotate(n,yaw).unit();}
    void prism(std::vector<Vec> polygon,float low=-400,float high=400) {
        for(unsigned i=0;i<polygon.size();++i) {
            Vec a=polygon[i],b=polygon[(i+1)%polygon.size()];a.z=b.z=low;
            Vec c=b,d=a;c.z=d.z=high;
            auto triangle=[&](Vec x,Vec y,Vec z){if(mirror<0)std::swap(y,z);solid.triangles.push_back({point(x),point(y),point(z)});};
            triangle(a,b,c);triangle(a,c,d);
        }
        for(unsigned i=1;i+1<polygon.size();++i) {
            Vec a=polygon[0],b=polygon[i],c=polygon[i+1];a.z=b.z=c.z=low;
            if(mirror<0)std::swap(b,c);solid.triangles.push_back({point(a),point(c),point(b)});
            a.z=b.z=c.z=high;solid.triangles.push_back({point(a),point(b),point(c)});
        }
    }
    std::optional<Hit> ray(Vec a,Vec b)override {
        ++casts;const Vec d=b-a;double nearest=2;std::optional<Hit> hit;
        for(const auto& t:solid.triangles) {
            const Vec e=t.b-t.a,f=t.c-t.a,p=d.cross(f);const double det=e.dot(p);
            if(std::abs(det)<1.e-7)continue;
            const Vec v=a-t.a,q=v.cross(e);const double u=v.dot(p)/det,w=d.dot(q)/det,s=f.dot(q)/det;
            if(u<-.00001||w<-.00001||u+w>1.00001||s<.000001||s>1||s>=nearest)continue;
            nearest=s;hit=Hit{a+d*float(s),e.cross(f).unit(),true};
        }
        return hit;
    }
};
Settings config(){Settings c;c.gap=37;c.radius=31;c.height=138;c.approachSeconds=0;return c;}
TriangleWorld fixture(bool convex,float degrees,Vec shear,float sign,bool distant,bool pyramid=false) {
    TriangleWorld w;w.shear=shear;w.mirror=sign;w.yaw=.47f;if(distant)w.origin={131065.84f,41791.12f,-11204.17f};
    if(pyramid) {
        w.shear={};
        const Vec top{-150,110,500};
        const std::array<Vec,4> base{Vec{-420,-88,-400},Vec{120,-88,-400},Vec{120,308,-400},Vec{-420,308,-400}};
        for(unsigned i=0;i<base.size();++i) {
            Vec a=w.point(base[i]),b=w.point(base[(i+1)%base.size()]),c=w.point(top);
            if(sign<0)std::swap(b,c);w.solid.triangles.push_back({a,b,c});
        }
        w.shear=shear;return w;
    }
    const float angle=degrees*.01745329252f*(convex?1.f:-1.f);
    const Vec target=cornerRotate({0,-1,0},angle),travel{-target.y,target.x,0};
    if(convex)w.prism({{-400,0,0},{0,0,0},travel*400,{-400,400,0}});
    else {w.prism({{-400,0,0},{400,0,0},{400,400,0},{-400,400,0}});w.prism({{0,0,0},travel*400,travel*400-target*400,target*-400});}
    return w;
}
void supported(bool convex,float degrees,Vec shear,int fps,float sign,bool distant,bool pyramid=false) {
    auto w=fixture(convex,degrees,shear,sign,distant,pyramid);const auto cfg=config();
    const Vec outward=w.normal({0,-1,0});

    const float exposed=outward.z>=0?6.f:cfg.height;
    Vec start=w.point({convex?-45.f:-65.f,-37,0});
    start=start+cornerRotate(Vec{0,shear.y*exposed,0},w.yaw);
    auto route=findCornerRoute(w,cfg,start,outward,sign,[](Vec,Vec){return true;});
    if(!route){std::cerr<<"no route convex="<<convex<<" degrees="<<degrees<<" shear="<<shear.x<<','<<shear.y<<" sign="<<sign<<" far="<<distant<<'\n';require(false,"joined tilted/acute triangle solid must produce checked turn");return;}
    float clearance=1.e9f,maxTurn=0;Vec old=start,oldNormal=outward;bool completed=false;
    const float speed=cornerSpeedLimit(*route,cfg.sideSpeed);
    for(int i=0;i<fps*5;++i) {
        auto step=advanceCornerRoute(w,cfg,*route,speed/fps,[](Vec,Vec){return true;});
        require(step.has_value(),"irregular corner retains live measured support throughout playback");if(!step)break;
        for(int sample=0;sample<=4;++sample)clearance=std::min(clearance,w.solid.capsuleDistance(old+(step->position-old)*(sample/4.f)));
        maxTurn=std::max(maxTurn,std::acos(std::clamp(step->normal.dot(oldNormal),-1.f,1.f)));
        old=step->position;oldNormal=step->normal;
        if(step->complete){completed=true;break;}
    }
    require(completed,"held side intent finishes triangle turn");require(clearance>=30.96f,"independent full capsule-to-triangle distance remains outside solid");
    require(maxTurn<=8.3f/fps+.015f,"3D surface direction changes continuously");
    if(completed) {
        route->distance=route->length*.55f;const Vec before=cornerSample(*route,route->distance).position;
        const auto raised=shiftCornerRoute(w,cfg,*route,8,[](Vec,Vec){return true;});
        require(raised.has_value(),"height input follows slanted physical seam");
        if(raised) {
            const Vec expected=cornerRotate({shear.x*sign*8,shear.y*8,8},w.yaw);
            require((raised->position-before-expected).length()<.025f,"height shift includes actual horizontal seam displacement");
        }
        const auto beforeRemoval=route->distance;w.solid.triangles.clear();
        require(!advanceCornerRoute(w,cfg,*route,2,[](Vec,Vec){return true;})&&route->distance==beforeRemoval,"removed support never advances a stored irregular path");
    }
    ++cases;std::cout<<"triangle corner convex="<<convex<<" pyramid="<<pyramid<<" deg="<<degrees<<" slope="<<shear.x<<','<<shear.y<<" fps="<<fps<<" side="<<sign<<" far="<<distant<<" minCapsule="<<clearance<<" maxTurn="<<maxTurn<<'\n';
}
void integration(const Library& library,bool pyramid,int fps,float sign,bool run) {
    auto w=fixture(true,90,{- .30f,.22f,0},sign,true,pyramid);auto cfg=config();
    Traversal t;t.cfg=cfg;const Vec from=w.point({-155,-37+6*w.shear.y,0});
    require(t.attach(w,from,w.normal({0,-1,0})*-1,1000),"Core attaches to sloping real mesh before its seam");
    SurfacePose surface;Pose previous;Vec last=t.position,lastNormal=t.normal;
    bool entered=false,completed=false,released=false,poseValid=true;float minimum=1e9f,maxYaw=0;
    for(int frame=0;frame<fps*5&&t.active();++frame) {
        Input input;input.x=sign;input.run=run;const auto result=t.update(w,input,1.f/fps,1000);
        entered|=t.turningCorner();completed|=entered&&!t.turningCorner();released|=result.released;
        for(int sample=0;sample<=4;++sample)minimum=std::min(minimum,w.solid.capsuleDistance(last+(t.position-last)*(sample/4.f)));
        maxYaw=std::max(maxYaw,std::acos(std::clamp(lastNormal.dot(t.normal),-1.f,1.f)));
        const auto pose=surface.update(library,w,t,result.motion,1.f/fps,1);
        poseValid&=pose.size()==99;
        for(unsigned bone=0;bone<pose.size();++bone) {
            poseValid&=pose[bone].t.finite()&&std::isfinite(pose[bone].q.dot(pose[bone].q));
            if(previous.size()==pose.size())poseValid&=angleBetween(previous[bone].q,pose[bone].q)<=(runMotion(result.motion)?18.849556f:12.566371f)/fps+.016f;
        }
        previous=pose;last=t.position;lastNormal=t.normal;if(completed)break;
    }
    require(entered&&completed&&!released,"Core finishes sloped seam without suspension or automatic release");
    require(poseValid,"actual SurfacePose retains all 99 finite bones within established angular output budgets");
    require(minimum>=30.96f,"Core sloped path retains independent complete capsule clearance");
    require(t.surfaceNormal.dot(w.normal({1,0,0}))>.995f&&std::abs(t.normal.z)<.00001f,"physical 3D destination normal and horizontal scene heading stay distinct");
    require(maxYaw<=8.3f/fps+.025f,"Core yaw remains continuous through inclined corner");
    ++cases;std::cout<<"integrated irregular pyramid="<<pyramid<<" fps="<<fps<<" side="<<sign<<" run="<<run<<" entered="<<entered<<" completed="<<completed<<" minCapsule="<<minimum<<" maxYaw="<<maxYaw<<'\n';
}

}
int main(int argc,char** argv) {
    Library library;if(argc!=2||!library.load(argv[1])){std::cerr<<"motion library required\n";return 1;}
    {
        const auto cfg=config();const float radius=cfg.radius+std::max(4.f,cfg.radius*.085f);
        const std::array<float,7> heights{6.f,(6.f+cfg.radius)*.5f,cfg.radius,cfg.chest,cfg.height-cfg.radius,cfg.height-(6.f+cfg.radius)*.5f,cfg.height-6.f};
        for(unsigned segment=1;segment<heights.size();++segment)for(unsigned sample=0;sample<=100;++sample) {
            const float u=sample/100.f,z=heights[segment-1]*(1-u)+heights[segment]*u;
            auto width=[&](float h){const float axial=std::max({cfg.radius-h,0.f,h-(cfg.height-cfg.radius)});return std::sqrt(std::max(0.f,radius*radius-axial*axial));};
            const float sampled=(width(heights[segment-1])*(1-u)+width(heights[segment])*u)*std::cos(.3926990817f);
            const float axial=std::max({cfg.radius-z,0.f,z-(cfg.height-cfg.radius)}),actual=std::sqrt(std::max(0.f,cfg.radius*cfg.radius-axial*axial));
            require(sampled>=actual,"inclined ring's polygon/chord envelope encloses actual radius-31 rounded capsule between all sampled rings");
        }
    }
    for(int fps:{30,60,120})for(float sign:{-1.f,1.f})for(bool far:{false,true}) {
        for(float degrees:{45.f,135.f})supported(true,degrees,{},fps,sign,far);
        for(float degrees:{45.f,90.f,105.f})supported(true,degrees,{-.30f,.22f,0},fps,sign,far);
        supported(false,60.f,{},fps,sign,far);
        supported(true,90.f,{-.30f,.22f,0},fps,sign,far,true);
    }
    for(int fps:{30,60,120})for(float sign:{-1.f,1.f})for(bool pyramid:{false,true})for(bool run:{false,true})integration(library,pyramid,fps,sign,run);
    std::cout<<"IrregularCornerTests cases="<<cases<<" failures="<<failures<<'\n';return failures?1:0;
}
