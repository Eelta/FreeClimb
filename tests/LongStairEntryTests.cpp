#define main fixtureMain
#include "AttachSurfaceTests.cpp"
#undef main
#ifdef FC_ENTRY_BASELINE
#include "NativeWalkableApproach.baseline.h"
#else
#include "NativeWalkableApproach.h"
#endif
struct BevelStep {float width,front,tail,base,rise,depth;};
struct BevelWorld:AttachWorld {
    std::vector<BevelStep> steps;float grade{};
    std::optional<Hit> ray(Vec from,Vec to) override {
        auto result=AttachWorld::ray(from,to);const auto a=local(from),d=local(to)-a;
        double nearest=result?(result->point-from).length()/std::max(.001f,(to-from).length()):2;
        for(const auto& step:steps) {
            const float k=step.rise/step.depth;
            const std::array<Vec,7> normals{{{1,0,0},{-1,0,0},{0,-1,0},{0,1,0},{0,0,-1},{0,-grade,1},{0,-k-grade,1}}};
            const std::array<float,7> limits{step.width,step.width,-step.front,step.tail,1000,step.base+step.rise,step.base-k*step.front};
            double entry=0,exit=1;Vec normal{};bool valid=true;
            for(unsigned j=0;j<normals.size();++j) {
                const auto n=normals[j];const double speed=n.dot(d),remaining=limits[j]-n.dot(a);
                if(std::abs(speed)<1e-8){if(remaining<0)valid=false;continue;}
                const double t=remaining/speed;
                if(speed<0){if(t>entry){entry=t;normal=n.unit();}}else exit=std::min(exit,t);
            }
            if(valid&&entry<=exit&&entry>1e-6&&entry<=1&&entry<nearest) {
                nearest=entry;result=Hit{global(a+d*float(entry)),direction(normal),true};
            }
        }
        return result;
    }
};
static Settings longEntryBody(){Settings c;c.radius=31;c.gap=37;c.height=138;return c;}
static BevelWorld stairs(float rise,float depth,float normalZ,int count=24) {
    BevelWorld w;w.boxes={{{-1000,-1000,-1000},{1000,3000,0}}};
    const float bevel=rise*normalZ/std::sqrt(1-normalZ*normalZ);
    for(int i=0;i<count;++i)w.steps.push_back({160,i*depth,count*depth,i*rise,rise,bevel});
    return w;
}
static GroundEntryExclusionResult classify(BevelWorld& w,Vec feet,Vec target,Vec normal,bool grounded) {
    const auto c=longEntryBody();const auto facing=w.direction({0,1,0});
    if(nativeWalkableApproach(w,feet,facing,c,grounded).walkable){GroundEntryExclusionResult r;r.reason=GroundEntryExclusion::staircase;return r;}
    auto r=groundedLowFace(w,feet,c,grounded,target,normal);
    if(!r.excludes()){r=groundedStepFace(w,feet,facing,c,grounded,target,normal);check(r.casts<=768,"step shape classifier stays in its finite query budget");}
    if(!r.excludes())r=groundedEntryExclusion(w,feet,facing,c,grounded,target,normal);
    return r;
}
int main(){try {
    unsigned checks=0,failures=0,actual=0,actualPositive=0;
    for(float rotation:{0.f,.73f,1.57f})for(Vec origin:{Vec{},Vec{132781.28f,38531.56f,-11554.62f}}) {
        auto inspect=[&](BevelWorld w,Vec feet,Vec target,Vec normal,bool expected,const char* label,bool grounded=true,bool useCandidate=false) {
            w.rotation=rotation;w.origin=origin;Traversal candidate;candidate.cfg=longEntryBody();
            Vec selected=w.global(target),surface=w.direction(normal);
            if(useCandidate&&candidate.attach(w,w.global(feet),w.direction({0,1,0}),1000,60,false,true)) {
                selected=candidate.entryTarget();surface=candidate.surfaceNormal;++actual;actualPositive+=expected;
            }
            const auto r=classify(w,w.global(feet),selected,surface,grounded);
            if(r.excludes()!=expected){++failures;std::cout<<"FAIL "<<label<<" far="<<(origin.x!=0)<<" angle="<<rotation<<" expected="<<expected<<" actual="<<r.excludes()<<" normalZ="<<surface.z<<" target="<<w.local(selected).y<<","<<w.local(selected).z<<" probes="<<r.casts<<'\n';}
            ++checks;
        };
        for(float nz:{.17f,.35f,.491f,.63f}) {
            const float rise=20,depth=32,k=std::sqrt(1-nz*nz)/nz;
            const Vec normal=Vec{0,-k,1}.unit();auto w=stairs(rise,depth,nz);
            inspect(w,{0,-50,0},{0,6/k-37,0},normal,true,"long stairs beveled first riser",true,true);
            inspect(w,{0,8*depth-12,8*rise+6},{0,8*depth+12/k-37,8*rise+6},normal,true,"already high on long stairs",true,true);
            inspect(w,{0,-50,0},{0,6/k-37,0},normal,false,"airborne grab of inclined staircase remains eligible",false);
        }
        auto sloped=stairs(20,32,.35f);sloped.grade=.3f;sloped.boxes.clear();
        sloped.steps.insert(sloped.steps.begin(),{1000,-1000,3000,0,0,1});
        const float gradedK=std::sqrt(1-.35f*.35f)/.35f+.3f;
        inspect(sloped,{0,-50,-15},{0,-9/gradedK-37,-15},Vec{0,-gradedK,1}.unit(),true,"long staircase with inclined walkable treads",true,true);
        const float midY=8*32-12,midZ=8*20+sloped.grade*midY+6;
        inspect(sloped,{0,midY,midZ},{0,8*32+(midZ+6-8*20-sloped.grade*8*32)/gradedK-37,midZ},
            Vec{0,-gradedK,1}.unit(),true,"inclined walkable treads high on long staircase",true,true);
        auto tall=stairs(80,180,.491f,1);const float k=std::sqrt(1-.491f*.491f)/.491f;
        inspect(tall,{0,-50,0},{0,6/k-37,0},Vec{0,-k,1}.unit(),false,"tall beveled wall is not a stair");
        BevelWorld ramp;ramp.boxes={{{-1000,-1000,-1000},{1000,3000,0}}};ramp.steps.push_back({160,0,2000,0,1000,1000/k});
        inspect(ramp,{0,-50,0},{0,6/k-37,0},Vec{0,-k,1}.unit(),false,"continuous steep surface has no upper tread");
        auto upper=stairs(20,32,.491f,1);upper.boxes.push_back({{-160,0,60},{160,200,1000}});
        inspect(upper,{0,-50,0},{0,-37,0},{0,-1,0},false,"selected upper high wall above small step",true,true);
    }
    check(actual>=12&&actualPositive>=6,"long stairs fixture must include actual positive and negative Core selections");
    std::cout<<"Long stair checks="<<checks<<" actualCoreSelections="<<actual<<" positiveCoreSelections="<<actualPositive<<" failures="<<failures<<'\n';return failures?1:0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
