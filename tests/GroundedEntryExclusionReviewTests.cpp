#define main attachFixtureMain
#include "AttachSurfaceTests.cpp"
#undef main
#include "NativeWalkableApproach.h"

static Settings exclusionBody() {Settings cfg;cfg.radius=31;cfg.height=138;cfg.gap=37;return cfg;}
static AttachWorld exclusionFloor() {
    AttachWorld w;w.boxes.push_back({{-1000,-1000,-1000},{1000,1000,0}});return w;
}
static AttachWorld exclusionSteps(float rise=18,float depth=32) {
    auto w=exclusionFloor();
    for(int i=0;i<7;++i)w.boxes.push_back({{-200,float(i)*depth,-1000},{200,1000,float(i+1)*rise}});
    return w;
}
int main() {try {
    unsigned checks=0,group=0,peak=0,integrated=0;
    for(Vec origin:{Vec{},Vec{132498.4f,37835.1f,-12604.1f}})for(float yaw:{0.f,.73f,1.57f}) {
        auto run=[&](AttachWorld w,Vec feet,Vec faceFeet,Vec normal,bool grounded,bool expected,const std::string& label,Settings cfg=exclusionBody()) {
            w.origin=origin;w.rotation=yaw;
            const auto result=groundedEntryExclusion(w,w.global(feet),w.direction({0,1,0}),cfg,grounded,
                w.global(faceFeet),w.direction(normal));
            peak=std::max(peak,result.casts);++checks;
            if(result.excludes()!=expected)std::cerr<<label<<" origin="<<origin.x<<" yaw="<<yaw
                <<" reason="<<int(result.reason)<<" casts="<<result.casts<<'\n';
            check(result.excludes()==expected,label);
            check(result.casts<=3072,label+" query budget");
            if(!grounded)check(result.casts==0,label+" airborne must not probe/suppress");
            return result;
        };

        for(float height:{24.f,38.f,47.9f,48.f,48.1f,62.f}) {
            auto w=exclusionFloor();w.boxes.push_back({{-200,0,-1000},{200,300,height}});
            run(w,{0,-50,0},{0,-37,0},{0,-1,0},true,height<=48,"low-solid boundary "+std::to_string(height));
        }
        auto high=exclusionFloor();high.boxes.push_back(wall);
        run(high,{0,-50,0},{0,-37,0},{0,-1,0},true,false,"real high front wall retains grab");
        auto low=exclusionFloor();low.boxes.push_back({{-200,0,-1000},{200,300,38}});
        run(low,{0,-50,0},{0,-37,0},{0,-1,0},false,false,"airborne low-face catch not suppressed");
        auto missing=low;missing.boxes.erase(missing.boxes.begin());
        run(missing,{0,-50,0},{0,-37,0},{0,-1,0},true,false,"missing starting ground is not low-obstacle exemption");
        auto narrow=exclusionFloor();narrow.boxes.push_back({{-8,0,-1000},{8,300,38}});
        run(narrow,{0,-50,0},{0,-37,0},{0,-1,0},true,false,"thin fence top has no actual foot corridor");
        auto unsupported=exclusionFloor();unsupported.boxes.push_back({{-200,0,100},{200,300,102},false});
        run(unsupported,{0,-50,0},{0,-37,0},{0,-1,0},true,false,"thin overhead sheet is not a grounded low object");
        auto floatingLow=exclusionFloor();floatingLow.boxes.push_back({{-200,0,36},{200,300,38}});
        run(floatingLow,{0,-50,0},{0,-37,0},{0,-1,0},true,false,"low floating sheet has no solid riser from ground");

        for(float rise:{18.f,24.f}) {
            const auto result=run(exclusionSteps(rise),{0,-50,0},{0,-37,0},{0,-1,0},true,true,
                "two or more continuous real steps "+std::to_string(rise));
            check(result.reason!=GroundEntryExclusion::none,"stair reason is explicit");
            const auto third=run(exclusionSteps(rise),{0,-33,0},{0,27,0},{0,-1,0},true,true,
                "third selected riser needs actual staircase classification "+std::to_string(rise));
            check(third.reason==GroundEntryExclusion::staircase,"three real rises exceed low-object threshold");
        }
        run(exclusionSteps(24.1f),{0,-33,0},{0,27,0},{0,-1,0},true,false,"24.1 riser exceeds stair limit and total exceeds low limit");
        auto shortBody=exclusionBody();shortBody.height=100;
        run(low,{0,-50,0},{0,-37,0},{0,-1,0},true,false,"short body uses .35 height not fixed 48",shortBody);
        auto shortLow=exclusionFloor();shortLow.boxes.push_back({{-200,0,-1000},{200,300,35}});
        run(shortLow,{0,-50,0},{0,-37,0},{0,-1,0},true,true,"short body exact 35-unit threshold",shortBody);
        auto narrowStairs=exclusionSteps();
        for(std::size_t i=1;i<narrowStairs.boxes.size();++i){narrowStairs.boxes[i].low.x=-8;narrowStairs.boxes[i].high.x=8;}
        run(narrowStairs,{0,-50,0},{0,-37,0},{0,-1,0},true,false,"narrow stairs cannot invent full foot support");
        auto sideWall=narrowStairs;sideWall.boxes.push_back({{20,0,-1000},{200,300,1000}});
        run(sideWall,{10,-50,0},{40,-37,0},{0,-1,0},true,false,"narrow stairs beside chosen high wall cannot replace its face");
        auto coplanar=exclusionFloor();coplanar.boxes.push_back({{-200,0,-1000},{35,300,38}});
        coplanar.boxes.push_back({{40,0,-1000},{200,300,1000}});
        run(coplanar,{0,-50,0},{50,-37,0},{0,-1,0},true,false,
            "laterally selected high wall cannot borrow coplanar low object along original facing");
        auto gapSteps=exclusionFloor();gapSteps.boxes[0].high.y=8;
        gapSteps.boxes.push_back({{-200,0,-1000},{200,8,18}});
        gapSteps.boxes.push_back({{-200,36,-1000},{200,300,36}});
        run(gapSteps,{0,-50,0},{0,-37,0},{0,-1,0},true,false,"gap between step platforms is not continuous staircase");

        auto farWall=exclusionSteps();farWall.boxes.push_back({{-200,240,-1000},{200,260,1000}});
        run(farWall,{0,-50,0},{0,-37,0},{0,-1,0},true,true,"distant end wall does not undo near-stair exclusion");
        auto endWall=exclusionFloor();
        endWall.boxes.push_back({{-200,0,-1000},{200,1000,18}});
        endWall.boxes.push_back({{-200,32,-1000},{200,1000,36}});
        endWall.boxes.push_back({{-200,100,-1000},{200,120,1000}});
        run(endWall,{0,50,36},{0,63,36},{0,-1,0},true,false,"standing on final tread faces real high wall");
        run(endWall,{0,-50,0},{0,63,0},{0,-1,0},true,false,"selected end wall cannot borrow preceding low steps");

        for(const auto& item:std::vector<std::pair<AttachWorld,bool>>{{low,true},{high,false}}) {
            auto w=item.first;w.origin=origin;w.rotation=yaw;
            Traversal t;t.cfg=exclusionBody();
            const Vec feet=w.global({0,-50,0}),forward=w.direction({0,1,0});
            check(t.attach(w,feet,forward,1000,60,false,true),"fixture real entry candidate exists");
            const auto result=groundedEntryExclusion(w,feet,forward,t.cfg,true,t.entryTarget(),t.surfaceNormal);
            check(result.excludes()==item.second,"actual candidate exclusion matches selected low/high face");
            ++checks;++integrated;peak=std::max(peak,result.casts);
        }
        ++group;
    }
    std::cout<<"Grounded exclusion independent groups="<<group<<" checks="<<checks
        <<" actualCandidates="<<integrated<<" peakQueries="<<peak<<" PASS\n";
    return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
