#define main attachFixtureMain
#include "AttachSurfaceTests.cpp"
#undef main
#include "NativeWalkableApproach.h"
struct LowTiltWorld:AttachWorld {
    float tilt{},depth=1.25f,top=30,bottom=-1000,width=60;
    bool object=true;
    std::optional<Hit> ray(Vec from,Vec to)override {
        auto best=AttachWorld::ray(from,to);if(!object)return best;
        const auto a=local(from),d=local(to)-a;
        const float slope=tilt/std::sqrt(1-tilt*tilt);
        const std::array<Vec,6> normals{{{1,0,0},{-1,0,0},{0,-1,slope},{0,1,-slope},{0,0,1},{0,0,-1}}};
        const std::array<float,6> limits{width,width,0,depth,top,-bottom};
        double entry=0,leave=1;Vec normal{};bool valid=true;
        for(unsigned i=0;i<normals.size();++i) {
            const double remainder=limits[i]-normals[i].dot(a),speed=normals[i].dot(d);
            if(std::abs(speed)<1e-8){if(remainder<0)valid=false;continue;}
            const double phase=remainder/speed;
            if(speed<0){if(phase>entry){entry=phase;normal=normals[i].unit();}}else leave=std::min(leave,phase);
        }
        const float prior=best?(best->point-from).length()/std::max(.001f,(to-from).length()):2;
        if(valid&&entry<=leave&&entry>1e-6&&entry<=1&&entry<prior)
            return Hit{global(a+d*float(entry)),direction(normal),true};
        return best;
    }
};
static Settings footprintBody(){Settings cfg;cfg.radius=31;cfg.gap=37;cfg.height=138;return cfg;}
static LowTiltWorld footprintWorld(bool peripheral) {
    LowTiltWorld w;w.boxes={{{-1000,-1000,-1000},{1000,peripheral?-70.f:1000.f,0}}};return w;
}
int main(){try {
    unsigned checks=0,failed=0,actual=0,peak=0;
    for(float yaw:{0.f,.73f,1.57f})for(Vec origin:{Vec{},Vec{135749.17f,39271.41f,-12167.60f}}) {
        auto inspect=[&](LowTiltWorld w,float rootHeight,bool grounded,bool expected,const char* label) {
            w.rotation=yaw;w.origin=origin;const auto cfg=footprintBody();
            const auto feet=w.global({0,-50,rootHeight});const float slope=w.tilt/std::sqrt(1-w.tilt*w.tilt);
            const auto target=w.global({0,slope*(rootHeight+(w.tilt>=0?6.f:cfg.height))-cfg.gap,rootHeight});
            const auto normal=w.direction(Vec{0,-1,slope}.unit());
            Traversal candidate;candidate.cfg=cfg;
            const bool accepted=candidate.attach(w,feet,w.direction({0,1,0}),1000,60,false,true);
            if(accepted)++actual;
            const auto result=groundedLowFace(w,feet,cfg,grounded,target,normal);
            peak=std::max(peak,result.casts);++checks;
            if(result.excludes()!=expected) {
                ++failed;std::cout<<"FAIL "<<label<<" tilt="<<w.tilt<<" top="<<w.top<<" depth="<<w.depth
                    <<" yaw="<<yaw<<" far="<<(origin.x!=0)<<" root="<<rootHeight<<" grounded="<<grounded
                    <<" found="<<result.groundFound<<" samples="<<result.supportSamples<<" casts="<<result.casts<<'\n';
            }
            if(!grounded)check(result.casts==0,"confirmed airborne catch cannot invoke a ground exclusion");
            if(result.excludes())check(result.groundFound&&result.selectedRise<=48.02f,"exclusion preserves actual bounded ground and low face evidence");
            if(accepted&&expected) {
                const auto actualResult=groundedLowFace(w,feet,cfg,grounded,candidate.entryTarget(),candidate.surfaceNormal);
                if(!actualResult.excludes()){++failed;std::cout<<"FAIL actual Core candidate "<<label<<" tilt="<<candidate.surfaceNormal.z<<"\n";}
            }
        };
        for(bool peripheral:{false,true})for(float tilt:{-.12f,-.044f,.063f,.2f})for(float top:{16.f,30.f,48.f})for(float depth:{1.25f,60.f}) {
            auto w=footprintWorld(peripheral);w.tilt=tilt;w.top=top;w.depth=depth;
            inspect(w,0,true,true,"tilted low object with real support within the native footprint");
        }
        for(bool peripheral:{false,true}) {
            auto w=footprintWorld(peripheral);w.tilt=.063f;
            w.top=70;inspect(w,0,true,false,"actual higher ledge is not low");
            w.top=1000;inspect(w,0,true,false,"actual high wall retains climb eligibility");
            w.top=30;inspect(w,18,true,false,"ground height tolerance is unchanged");
            inspect(w,0,false,false,"falling low ledge regrab is unchanged");
            w.bottom=27;inspect(w,0,true,false,"floating thin top is not a grounded obstacle");
        }
        auto noFloor=footprintWorld(false);noFloor.boxes.clear();
        inspect(noFloor,0,true,false,"native grounded state does not invent an absent floor");
        auto isolated=noFloor;isolated.boxes={{{-2,-80,-1000},{2,-76,0}},{{-2,-24,-1000},{2,-20,0}}};
        inspect(isolated,0,true,false,"isolated outer footprint points do not invent a support plane");
    }
    for(unsigned mask=0;mask<32;++mask) {
        const bool allowed=groundEntryGeometryAllowed(mask&1,mask&2,mask&4,mask&8,mask&16);
        check(allowed==(mask==1),"ground-like unsupplied controller support permits only actual geometric verification; all air and jump ownership remains excluded");
    }
    std::cout<<"Entry footprint checks="<<checks<<" actualCandidates="<<actual<<" failures="<<failed<<" peakQueries="<<peak<<'\n';
    check(actual>24,"new cases include actual Core-selected low faces and legitimate walls");return failed?1:0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
