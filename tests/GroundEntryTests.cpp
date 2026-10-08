#define main attachedSurfaceFixtureMain
#include "AttachSurfaceTests.cpp"
#undef main
#include <fstream>
#include "Pose.h"
static unsigned peakEntryRays=0;

struct EntryWorld:AttachWorld {
    float crossSlope{},floorSlope{},riseSlope{},riseStart=40;
    unsigned rays{};
    std::optional<Hit> ray(Vec from,Vec to)override {
        ++rays;
        auto result=AttachWorld::ray(from,to);
        const auto a=local(from),d=local(to)-a;
        const std::array<Vec,2> planes{Vec{-crossSlope,-floorSlope,1},Vec{0,-riseSlope,1}};
        for(int index=0;index<2;++index) {
            const auto plane=planes[index];const float intercept=index==0?0:riseSlope*riseStart;
            const float start=plane.dot(a)+intercept,delta=plane.dot(d);
            if(start<=1e-5f||delta>=-1e-6f)continue;
            const float t=-start/delta;if(t<=0||t>1)continue;
            const auto point=from+(to-from)*t;
            if(!result||(point-from).length()<(result->point-from).length())result=Hit{point,direction(plane.unit()),true};
        }
        return result;
    }
    bool actualCapsule(Vec globalFeet,float radius,float height) const {
        const auto p=local(globalFeet);const float low=p.z+radius,high=p.z+height-radius;
        const std::array<Vec,2> planes{Vec{-crossSlope,-floorSlope,1},Vec{0,-riseSlope,1}};
        for(int index=0;index<2;++index) {
            const auto plane=planes[index];const float intercept=index==0?0:riseSlope*riseStart;
            if((plane.dot({p.x,p.y,low})+intercept)/plane.length()<radius-.08f)return false;
        }
        for(const auto& box:boxes) {
            const float dx=p.x-std::clamp(p.x,box.low.x,box.high.x),dy=p.y-std::clamp(p.y,box.low.y,box.high.y);
            const float dz=high<box.low.z?box.low.z-high:low>box.high.z?low-box.high.z:0;
            if(std::sqrt(dx*dx+dy*dy+dz*dz)<radius-.08f)return false;
        }
        return true;
    }
};
static Traversal freshEntry(){Traversal t;t.cfg.radius=31;t.cfg.gap=37;t.cfg.height=138;return t;}
static EntryWorld baseWorld(float cross,float rise) {
    EntryWorld w;w.crossSlope=cross;w.riseSlope=rise;w.boxes={{{-1000,94.5f,-1000},{1000,500,1000}}};return w;
}
static bool accepted(EntryWorld& world,Vec feet,bool grounded,Traversal& t) {
    world.rays=0;
    const bool result=t.attach(world,world.global(feet),world.direction({0,1,0}),1000,60,false,grounded);
    peakEntryRays=std::max(peakEntryRays,world.rays);return result;
}
struct MixedEntryWorld:AttachWorld {
    struct Triangle {Vec a,b,c;};std::vector<Triangle> triangles;
    unsigned rays{};
    explicit MixedEntryWorld(const char* file) {
        std::ifstream stream(file,std::ios::binary);std::uint32_t count{};stream.read(reinterpret_cast<char*>(&count),4);
        check(bool(stream)&&count>0&&count<10000,"read optional local collision witness");triangles.resize(count);
        stream.read(reinterpret_cast<char*>(triangles.data()),std::streamsize(count*sizeof(Triangle)));check(bool(stream),"complete diagnostic triangles");

        boxes={{{133436.66f,36000,-16000},{134000,39000,-9000}}};
    }
    std::optional<Hit> ray(Vec from,Vec to)override {
        ++rays;
        auto result=AttachWorld::ray(from,to);const auto d=to-from;
        for(const auto& tri:triangles) {
            const auto ab=tri.b-tri.a,ac=tri.c-tri.a,n=ab.cross(ac).unit();
            const double start=(from-tri.a).dot(n),denominator=d.dot(n);
            if(start<=.00001||denominator>=-.00001)continue;
            const double time=-start/denominator;if(time<=0||time>1)continue;
            const auto p=from+d*float(time),q=p-tri.a;
            const double aa=ab.dot(ab),bb=ac.dot(ac),cross=ab.dot(ac),qa=q.dot(ab),qb=q.dot(ac),det=aa*bb-cross*cross;
            if(det<=0)continue;const double u=(qa*bb-qb*cross)/det,v=(qb*aa-qa*cross)/det;
            if(u<-.00001||v<-.00001||u+v>1.00001)continue;
            if(!result||(p-from).length()<(result->point-from).length())result=Hit{p,n,true};
        }
        return result;
    }
    static Vec closest(Vec p,const Triangle& t) {
        const auto ab=t.b-t.a,ac=t.c-t.a,ap=p-t.a;const float d1=ab.dot(ap),d2=ac.dot(ap);
        if(d1<=0&&d2<=0)return t.a;
        const auto bp=p-t.b;const float d3=ab.dot(bp),d4=ac.dot(bp);
        if(d3>=0&&d4<=d3)return t.b;
        const float vc=d1*d4-d3*d2;if(vc<=0&&d1>=0&&d3<=0)return t.a+ab*(d1/(d1-d3));
        const auto cp=p-t.c;const float d5=ab.dot(cp),d6=ac.dot(cp);
        if(d6>=0&&d5<=d6)return t.c;
        const float vb=d5*d2-d1*d6;if(vb<=0&&d2>=0&&d6<=0)return t.a+ac*(d2/(d2-d6));
        const float va=d3*d6-d5*d4;if(va<=0&&(d4-d3)>=0&&(d5-d6)>=0)return t.b+(t.c-t.b)*((d4-d3)/((d4-d3)+(d5-d6)));
        const float inv=1/(va+vb+vc);return t.a+ab*(vb*inv)+ac*(vc*inv);
    }
    static float edgeDistance(Vec p,Vec q,Vec a,Vec b) {
        const auto d=q-p,e=b-a,r=p-a;const float aa=d.dot(d),ee=e.dot(e),f=e.dot(r);
        float s=0,t=0;
        if(aa<=1e-8f)t=std::clamp(f/ee,0.f,1.f);
        else {
            const float c=d.dot(r);
            if(ee<=1e-8f)s=std::clamp(-c/aa,0.f,1.f);
            else {const float dot=d.dot(e),den=aa*ee-dot*dot;
                if(den>1e-8f)s=std::clamp((dot*f-c*ee)/den,0.f,1.f);
                t=(dot*s+f)/ee;if(t<0){t=0;s=std::clamp(-c/aa,0.f,1.f);}else if(t>1){t=1;s=std::clamp((dot-c)/aa,0.f,1.f);}
            }
        }
        return (p+d*s-a-e*t).length();
    }
    float capsuleDistance(Vec feet) const {
        const auto p=feet+Vec{0,0,31},q=feet+Vec{0,0,107};float distance=133436.66f-feet.x;
        for(const auto& t:triangles) {
            distance=std::min({distance,(p-closest(p,t)).length(),(q-closest(q,t)).length(),
                edgeDistance(p,q,t.a,t.b),edgeDistance(p,q,t.b,t.c),edgeDistance(p,q,t.c,t.a)});
            const auto n=(t.b-t.a).cross(t.c-t.a).unit();const float den=(q-p).dot(n);
            if(std::abs(den)>.00001f) {const float f=(t.a-p).dot(n)/den;
                if(f>=0&&f<=1){const auto middle=p+(q-p)*f;if((middle-closest(middle,t)).length()<.02f)distance=0;}
            }
        }
        return distance;
    }
};
static void actualTerrainWitness(const char* file) {
    for(int fps:{30,60,120})for(Vec feet:{Vec{133342.17f,37712.16f,-12606.04f},Vec{133341.f,37696.54f,-12605.87f}}) {
        MixedEntryWorld world(file);auto t=freshEntry();const bool success=t.attach(world,feet,{1,0,0},1000,60,false,true);
        peakEntryRays=std::max(peakEntryRays,world.rays);
        const auto entryRays=world.rays;
        check(success&&t.entryLiftHeight()>0,"measured terrain/inferred-wall fixture accepts bounded lift");
        const float lift=t.entryLiftHeight();t.entry(Motion::reach,true);float minimum=world.capsuleDistance(t.position);
        while(t.state==State::approach) {
            const auto result=t.update(world,{},1.f/fps,1000);check(!result.released,"mixed fixture plays checked full curve");
            minimum=std::min(minimum,world.capsuleDistance(t.position));
        }
        check(minimum>=30.92f,"independent capsule-to-triangle distance clears actual terrain");
        std::cout<<"mixed terrain (inferred wall): fps="<<fps<<" lift="<<lift<<" minCapsuleDistance="<<minimum<<" entryRays="<<entryRays<<'\n';
    }
}
static void posedEntry(const Library& library,int fps,bool releaseDuringEntry,Motion entryMotion) {
    auto world=baseWorld(0,.625f);auto t=freshEntry();
    check(accepted(world,{0,0,0},true,t)&&t.entryLiftHeight()>0,"pose regression takes actual raised Core route");
    t.entry(entryMotion,true);SurfacePose surface;Pose previous;Motion last=Motion::none;
    const float dt=1.f/fps;bool catchSeen=false,runSeen=false,climbSeen=false;
    for(int frame=0;frame<fps*3;++frame) {
        Input input;input.y=1;input.run=releaseDuringEntry?frame<fps/6:frame<fps;
        const auto result=t.update(world,input,frame?dt:0,1000);
        check(!result.released,"actual entry/run/climb transition retains wall");
        const auto pose=surface.update(library,world,t,result.motion,frame?dt:0,1);
        check(pose.size()==99&&library.armBendValid(pose,0)&&library.armBendValid(pose,1),"raised entry keeps99bones and anatomical elbows");
        for(std::size_t bone=0;bone<pose.size();++bone) {
            check(pose[bone].t.finite()&&std::abs(pose[bone].q.dot(pose[bone].q)-1)<.004f,"all raised-entry transforms are finite normalized");
            if(!previous.empty())check(angleBetween(previous[bone].q,pose[bone].q)<=
                (runMotion(result.motion)||runMotion(last)?18.849556f:12.566371f)*dt+.016f,"raised catch/run/climb respects complete-bone angular budgets");
        }
        const auto paused=surface.update(library,world,t,result.motion,0,1);
        for(std::size_t bone=0;bone<pose.size();++bone)check((pose[bone].t-paused[bone].t).length()<.00001f&&
            angleBetween(pose[bone].q,paused[bone].q)<.00001f,"zero-time callback never refreshes raised entry pose");
        catchSeen|=result.motion==entryMotion;runSeen|=runMotion(result.motion);climbSeen|=result.motion==Motion::up;
        previous=pose;last=result.motion;
    }
    check(catchSeen&&climbSeen&&(releaseDuringEntry||runSeen),"requested modes appear after actual raised approach");
}
int main(int argc,char** argv){try {
    unsigned scenarios=0;
    for(Vec origin:{Vec{},Vec{133342.17f,37712.16f,-12606.04f}})for(float yaw:{0.f,.73f})for(int fps:{30,60,120}) {
        for(bool rising:{false,true}) {
            auto w=baseWorld(rising?0:(yaw==0?.235f:.4f),rising?.625f:0);w.floorSlope=rising?0:.02f;w.origin=origin;w.rotation=yaw;
            const Vec start{0,0,rising?0.f:(yaw==0?2.1f:3.7f)};auto t=freshEntry();
            check(w.actualCapsule(w.global(start),31,138),"starting physical rounded capsule is clear");
            const bool caught=accepted(w,start,true,t);
            check(caught,"grounded rounded/lifted entry succeeds");
            if(!t.roundedEntryPath())std::cerr<<"ordinary entry rising="<<rising<<" origin="<<origin.x<<" yaw="<<yaw<<"\n";
            check(t.roundedEntryPath(),"native lower capsule is explicitly selected");
            check(rising?t.entryLiftHeight()>0:t.entryLiftHeight()==0,"only the rising plinth needs raised endpoint");
            check(t.lastAttachDistance<=60.01f,"horizontal approach never exceeds60");
            const auto initial=t.position;check((initial-w.global(start)).length()<.01f,"no entry teleport");
            t.entry(Motion::reach,true);Vec previous=t.position;int frames=0;
            while(t.state==State::approach&&frames++<fps*2) {
                const auto result=t.update(w,{},1.f/fps,1000);
                check(!result.released,"preflight curve remains playable");
                check(w.actualCapsule(t.position,31,138),"independent rounded capsule stays outside floor/wall");
                check((t.position-previous).length()<650.f/fps+.1f,"entry moves continuously at bounded speed");previous=t.position;
            }
            check(t.state==State::wall&&frames>fps/4,"entry completes only after animated path");
            ++scenarios;
        }
        auto stairs=baseWorld(0,.625f);stairs.origin=origin;stairs.rotation=yaw;
        auto air=freshEntry();check(!accepted(stairs,{0,0,0},false,air),"unconfirmed/airborne zero-height entry gets no ground boost");
        auto disabled=freshEntry();disabled.cfg.groundJumpHeight=0;
        check(!accepted(stairs,{0,0,0},true,disabled),"zero jump budget leaves blocked raised target unavailable");
        auto capped=freshEntry();capped.cfg.groundJumpHeight=16;
        check(!accepted(stairs,{0,0,0},true,capped),"configured budget is respected");
        auto roof=stairs;roof.boxes.push_back({{-1000,-100,150},{1000,500,160},false});
        auto blocked=freshEntry();check(!accepted(roof,{0,0,0},true,blocked),"low ceiling blocks full lifted body route");
        auto distant=stairs;distant.boxes={{{-1000,104,-1000},{1000,500,1000}}};
        auto tooFar=freshEntry();check(!accepted(distant,{0,0,0},true,tooFar),"raised searches preserve horizontal reach/snap cap");
        auto t=freshEntry();check(accepted(stairs,{0,0,0},true,t),"dynamic obstacle fixture starts valid");t.entry(Motion::reach,true);
        stairs.boxes.push_back({{-1000,30,110},{1000,35,200},false});
        bool stopped=false;
        for(int frame=0;frame<fps&&t.active();++frame) {
            const auto before=t.position;const auto result=t.update(stairs,{},1.f/fps,1000);
            if(result.released){stopped=true;check((t.position-before).length()<.001f,"dynamic obstacle stops before crossing");}
        }
        check(stopped,"new obstruction is caught during actual playback");
        auto removed=baseWorld(0,.625f);auto changed=freshEntry();check(accepted(removed,{0,0,0},true,changed),"support change starts with real wall");
        changed.entry(Motion::reach,true);removed.boxes.clear();
        const auto before=changed.position;const auto result=changed.update(removed,{},1.f/fps,1000);
        check(result.released&&result.releaseVelocity.z<0&&(changed.position-before).length()<.001f,"vanished upper grip cannot finish an air catch and hands off to physical fall");
    }
    Library library;check(argc>=2&&library.load(argv[1]),"load actual motion library for ground-entry transitions");
    for(int fps:{30,60,120})for(bool releaseShift:{false,true})for(Motion entry:{Motion::reach,Motion::runLaunch})
        posedEntry(library,fps,releaseShift,entry);
    if(argc>=3)actualTerrainWitness(argv[2]);
    std::cout<<"Ground entry "<<scenarios<<" rotated/far/FPS cases passed";
    std::cout<<"; peak entry rays="<<peakEntryRays<<'\n';return 0;
}catch(const std::exception& error){std::cerr<<"Ground entry FAIL: "<<error.what()<<'\n';return 1;}}
