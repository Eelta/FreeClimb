#include "Pose.h"
namespace fc {
#include "GripEdge.h"
}
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace fc;
static void check(bool condition,const char* reason) {if(!condition)throw std::runtime_error(reason);}

struct BoxWorld:World {
    struct Box {Vec low,high;bool climbable=true;};
    std::vector<Box> boxes;
    Vec origin{};
    float yaw{};
    unsigned queries{};
    Vec vector(Vec p) const {return {p.x*std::cos(yaw)-p.y*std::sin(yaw),p.x*std::sin(yaw)+p.y*std::cos(yaw),p.z};}
    Vec point(Vec p) const {return origin+vector(p);}
    Vec local(Vec p) const {p=p-origin;return {p.x*std::cos(yaw)+p.y*std::sin(yaw),-p.x*std::sin(yaw)+p.y*std::cos(yaw),p.z};}
    std::optional<Hit> ray(Vec worldA,Vec worldB) override {
        ++queries;const auto a=local(worldA),b=local(worldB),delta=b-a;
        std::optional<Hit> result;float nearest=2;
        for(const auto& box:boxes) {
            float enter=0,leave=1;Vec entryNormal{},exitNormal{};bool miss=false,inside=true;
            const float starts[]{a.x,a.y,a.z},directions[]{delta.x,delta.y,delta.z};
            const float lows[]{box.low.x,box.low.y,box.low.z},highs[]{box.high.x,box.high.y,box.high.z};
            for(int axis=0;axis<3;++axis) {
                inside&=starts[axis]>lows[axis]&&starts[axis]<highs[axis];
                if(std::abs(directions[axis])<1e-7f) {if(starts[axis]<lows[axis]||starts[axis]>highs[axis])miss=true;continue;}
                float first=(lows[axis]-starts[axis])/directions[axis],last=(highs[axis]-starts[axis])/directions[axis];
                Vec firstNormal{},lastNormal{};
                if(axis==0){firstNormal.x=-1;lastNormal.x=1;}else if(axis==1){firstNormal.y=-1;lastNormal.y=1;}
                else {firstNormal.z=-1;lastNormal.z=1;}
                if(first>last){std::swap(first,last);std::swap(firstNormal,lastNormal);}
                if(first>=enter){enter=first;entryNormal=firstNormal;}
                if(last<=leave){leave=last;exitNormal=lastNormal;}
                if(enter>leave)miss=true;
            }
            const float t=inside?leave:enter;const auto normal=inside?exitNormal:entryNormal;
            if(!miss&&t>=0&&t<=1&&normal.length()>.5f&&t<nearest) {
                nearest=t;result=Hit{point(a+delta*t),vector(normal),box.climbable};
            }
        }
        return result;
    }
};
static Settings settings() {Settings cfg;cfg.gap=37;cfg.radius=31;cfg.height=138;return cfg;}
static std::optional<GripEdge> find(BoxWorld& world,float low=100,float high=124) {
    const auto count=world.queries;
    const auto result=findGripEdge(world,world.point({0,-37,0}),world.vector({0,-1,0}),settings(),low,high);
    check(world.queries-count<=gripEdgeDetail::queryLimit,"grip search exceeds explicit query budget");
    return result;
}
static BoxWorld block() {BoxWorld world;world.boxes.push_back({{-100,0,-200},{100,100,112}});return world;}
static BoxWorld shelves() {
    BoxWorld world;world.boxes.push_back({{-100,8,-200},{100,100,500}});
    world.boxes.push_back({{-100,0,104},{100,8,112}});
    world.boxes.push_back({{-100,0,24},{100,8,32}});return world;
}
static void positiveGeometry() {
    unsigned cases=0;
    for(float angle:{0.f,.35f,1.5707963f,2.2f,-1.3f})for(Vec origin:{Vec{},Vec{75000,-68000,23000}}) {
        for(bool narrow:{false,true}) {
            auto world=narrow?shelves():block();world.yaw=angle;world.origin=origin;
            const auto edge=find(world);check(edge.has_value(),"actual horizontal lip must be found at arbitrary yaw/origin");
            check((edge->center-world.point({0,0,112})).length()<.08f,"center must be the front lip, not inset palm");
            check(edge->normal.dot(world.vector({0,-1,0}))>.999f,"edge outward heading");
            for(int hand=0;hand<2;++hand)check((edge->hands[hand]-world.point({hand==0?-16.f:16.f,3,112})).length()<.08f,"actual separated palms lie three units behind front lip");
            check(gripEdgeStillValid(world,*edge),"unchanged physical edge revalidates");
            if(narrow) {
                const auto lower=find(world,16,52);check(lower.has_value(),"lower edge independent of standing footprint");
                check(std::abs(lower->center.z-(origin.z+32))<.05f,"stacked search must choose requested lower band");
                check(std::abs((edge->center-lower->center).length()-80)<.08f,"lower edge preserves true eighty-unit separation");
            }
            ++cases;
        }
    }

    auto projecting=shelves();projecting.boxes[1].low.y=-6;projecting.boxes[1].high.y=0;
    const auto projection=find(projecting);check(projection&&std::abs(projection->center.y+6)<.01f,"projecting sill locates real front instead of cfg.gap");
    std::cout<<"positive geometric fixtures="<<cases+1<<'\n';
}
static void rejectGeometry() {
    auto world=block();world.boxes[0].high.z=500;check(!find(world),"uninterrupted plane has no top edge");
    world=block();world.boxes[0].high.x=7;check(!find(world),"one-palm shelf cannot provide pair of contacts");
    world=block();world.boxes[0].low.x=-7;check(!find(world),"opposite missing palm cannot provide pair");
    world=block();world.boxes[0].high.y=4;check(!find(world),"top narrower than six units is not a certified hand strip");
    world=block();world.boxes[0].climbable=false;check(!find(world),"nonclimbable surface cannot become grip edge");
    world=block();world.boxes[0].high.z=130;check(!find(world),"top above requested height cannot leak into hand range");
    world=block();world.boxes[0].high.z=98;check(!find(world),"top below requested height cannot leak into hand range");
    world=block();world.boxes[0].low.z=111;check(!find(world),"thin sheet without front below lip fails real front requirement");
    world.boxes={{{-100,0,-200},{0,100,112}},{{0,0,-200},{100,100,118}}};
    check(!find(world),"different height under other hand is not one level edge");
    world.boxes={{{-100,0,-200},{-10,100,112}},{{10,0,-200},{100,100,112}}};
    check(!find(world),"disconnected patches without centre edge are rejected");

    world.boxes={{{-100,0,-200},{100,1,500}},{{-100,1,-200},{100,100,112}}};
    check(!find(world),"floor hidden behind a continuing front wall cannot be a lip");
    world.boxes={{{-100,0,-200},{0,100,112}},{{0,4,-200},{100,100,112}}};
    check(!find(world),"stepped front under other hand cannot be called the same plane");
    std::cout<<"negative solid fixtures=12\n";
}
struct SlopedPlane:World {
    std::optional<Hit> ray(Vec a,Vec b) override {
        const Vec n=Vec{0,-1,.5f}.unit();const float da=a.dot(n),db=b.dot(n);
        if(da*db>0||std::abs(da-db)<1e-6f)return {};
        return Hit{a+(b-a)*(da/(da-db)),n,true};
    }
};
static void invalidation() {
    auto world=shelves();const auto edge=find(world);check(edge.has_value(),"invalidation baseline");
    world.boxes[1].high.x=7;check(!gripEdgeStillValid(world,*edge),"lost right palm invalidates stored target");
    world=shelves();world.boxes[1].high.z+=3;check(!gripEdgeStillValid(world,*edge),"moved top invalidates old palm anchors");
    world=shelves();world.boxes[1].low.y+=2;check(!gripEdgeStillValid(world,*edge),"moved front invalidates stored target");
    world=shelves();world.boxes[1].low.y=4;check(!gripEdgeStillValid(world,*edge),"lost strip invalidates stored hand contact");
    world=shelves();auto bad=*edge;bad.hands[0].z+=2;check(!gripEdgeStillValid(world,bad),"inconsistent serialized hand anchor is not accepted");
    bad=*edge;bad.normal={};check(!gripEdgeStillValid(world,bad),"invalid stored normal is rejected");
    SlopedPlane slope;check(!findGripEdge(slope,{0,-37,0},{0,-1,0},settings(),100,124),"smooth slope cannot produce building edge");
    world=block();const auto before=world.queries;
    check(!findGripEdge(world,{0,-37,0},{0,0,0},settings(),100,124),"invalid outward heading rejected");
    check(!findGripEdge(world,{0,-37,0},{0,-1,0},settings(),124,100),"reversed height range rejected");
    check(!findGripEdge(world,{0,-37,0},{0,-1,0},settings(),0,500),"unbounded scan rejected");
    check(world.queries==before,"invalid inputs cannot spend ray queries");
    std::cout<<"revalidation and input guards passed\n";
}
static void sourceHandSpans() {
    unsigned positive=0;
    for(float scale:{.5f,1.f,2.f})for(float yaw:{0.f,.75f,2.2f})for(Vec origin:{Vec{},Vec{75000,-68000,23000}}) {
        auto world=block();world.origin=origin;world.yaw=yaw;
        const float height=138.12f*scale,span=25.14f*scale;
        world.boxes[0].high.z=height;
        const auto before=world.queries;
        const auto edge=findGripEdge(world,world.point({0,-37,0}),world.vector({0,-1,0}),settings(),height-12,height+12,span);
        check(world.queries-before<=gripEdgeDetail::queryLimit,"source-span query remains bounded");
        check(edge.has_value(),"captured hand span scales with the source pose");
        check((edge->center-world.point({0,0,height})).length()<.08f,"scaled target remains actual front lip");
        for(int hand=0;hand<2;++hand) {
            const auto expected=world.point({hand==0?-span:span,3,height});
            check((edge->hands[hand]-expected).length()<.08f,"scaled hands use requested source span instead of fixed sixteen units");
        }
        check(gripEdgeStillValid(world,*edge),"scaled source contacts revalidate without serialized span");
        const Vec along=world.vector({1,0,0}),outward=world.vector({0,-1,0});
        auto forged=*edge;forged.hands[0]=forged.hands[0]+along*5;
        check(!gripEdgeStillValid(world,forged),"one forged lateral palm cannot change inferred source span");
        forged=*edge;forged.hands[0]=forged.hands[0]+along*4;forged.hands[1]=forged.hands[1]+along*4;
        check(!gripEdgeStillValid(world,forged),"both palms shifted away from their centre are asymmetric");
        forged=*edge;std::swap(forged.hands[0],forged.hands[1]);
        check(!gripEdgeStillValid(world,forged),"reversed hand ordering is rejected");
        forged=*edge;forged.hands[1]=forged.hands[1]+outward*2;
        check(!gripEdgeStillValid(world,forged),"forged front-plane inset cannot be accepted from lateral span alone");
        ++positive;
    }
    auto world=block();
    for(float span:{11.f,53.f}) {
        const auto before=world.queries;
        check(!findGripEdge(world,{0,-37,0},{0,-1,0},settings(),100,124,span),"unsupported source span is rejected");
        check(world.queries==before,"invalid source span cannot consume ray queries");
    }
    world.boxes[0].low.x=-20;world.boxes[0].high.x=20;
    check(find(world).has_value(),"narrow span fixture accepts default palms");
    check(!findGripEdge(world,{0,-37,0},{0,-1,0},settings(),100,124,25.14f),"actual narrow shelf cannot support wider captured palms");
    std::cout<<"source-scaled span fixtures="<<positive<<" and forged-contact guards passed\n";
}
static BoxWorld stackedSeedWorld(float height=128.222f) {
    BoxWorld world;world.boxes={{{-1000,8,-500},{1000,300,500}},
        {{-1000,0,-500},{1000,8,height}},{{-1000,0,height+18.5f},{1000,8,height+20}}};return world;
}
static void stackedSeedAcquisition() {
    constexpr float h=128.222f,span=20.43f;unsigned cases=0;
    for(float yaw:{0.f,.73f,1.5707963f})for(Vec origin:{Vec{},Vec{75000,-68000,23000}}) {
        auto world=stackedSeedWorld();world.yaw=yaw;world.origin=origin;
        const auto before=world.queries;
        const auto edge=findGripEdge(world,world.point({0,-37,0}),world.vector({0,-1,0}),settings(),h-18,h+18,span);
        check(world.queries-before<=gripEdgeDetail::queryLimit,"second seed stays inside shared ninety-six-query budget");
        check(edge&&(edge->center-world.point({0,0,h})).length()<.08f,"upper out-of-range solid cannot conceal a reachable lower lip");
        check(gripEdgeStillValid(world,*edge),"lower lip retains all original front, top and paired-hand checks");++cases;
    }
    auto world=stackedSeedWorld();
    world.boxes.push_back({{-1000,-1,h+1},{1000,8,h+17}});
    check(!findGripEdge(world,{0,-37,0},{0,-1,0},settings(),h-18,h+18,span),"solid over lower hands cannot be skipped by lower seed");
    world=stackedSeedWorld();
    world.boxes.push_back({{-1000,-10,h+1},{1000,-8,h+17},false});
    check(!findGripEdge(world,{0,-37,0},{0,-1,0},settings(),h-18,h+18,span),"unknown intervening solid blocks access to independent lower seed");
    world=stackedSeedWorld();world.boxes[1].high.x=10;
    check(!findGripEdge(world,{0,-37,0},{0,-1,0},settings(),h-18,h+18,span),"second seed does not invent missing second hand");
    std::cout<<"stacked independent seed fixtures="<<cases<<" and blocked-hand/access guards passed\n";
}
static void stackedTraversal(const Library& library) {
    auto cfg=settings();check(library.configureThreepeat(cfg),"production captured calibration available");
    cfg.threepeatAnimations=true;cfg.approachSeconds=.01f;
    const float h=cfg.threepeatHangHeight;
    unsigned cases=0;
    for(int fps:{30,60,120})for(int side:{-1,1})for(bool far:{false,true}) {
        auto world=stackedSeedWorld(h);
        if(far){world.origin={75000,-68000,23000};world.yaw=.73f;}
        Traversal t;t.cfg=cfg;check(t.attach(world,world.point({0,-45,0}),world.vector({0,1,0}),100,60),"real stacked geometry has clear attached body");
        t.update(world,{},.05f,1000);check(t.state==State::wall,"checked approach completes");
        Input hop{float(side),0};hop.hop=true;
        auto result=t.update(world,hop,1.f/fps,1000);
        check(t.preparingEdge()||threepeatHop(result.motion),"both real lower edges are acquired beneath the upper solid");
        check(t.threepeatStatus()>=7,"acquisition selected captured side leap rather than legacy fallback");
        SurfacePose pose;Pose previous;int frames=0,captured=0;
        while((t.preparingEdge()||t.state==State::action)&&frames++<fps*4) {
            result=t.update(world,{},1.f/fps,1000);check(!result.released,"complete live body route remains clear beneath upper solid");
            const auto current=pose.update(library,world,t,result.motion,1.f/fps,1);
            if(threepeatHop(result.motion))++captured;
            for(int hand=0;hand<2;++hand)check(library.armBendValid(current,hand),"stacked source retains legal actual captured elbows");
            if(!previous.empty())for(std::size_t bone=0;bone<current.size();++bone)
                check(angleBetween(previous[bone].q,current[bone].q)<=12.566371f/fps+.015f,"stacked-source transition keeps existing output angle budget");
            previous=current;
        }
        check(captured>fps/2&&t.state==State::wall&&t.threepeatStatus()==9,"full captured path and supported target complete");++cases;
    }
    auto headBlocked=stackedSeedWorld(h);
    headBlocked.boxes.back().low={-1000,-100,h+8};
    const auto edge=findGripEdge(headBlocked,{0,-37,0},{0,-1,0},cfg,h-18,h+18,cfg.threepeatHandHalfWidth);
    check(edge&&gripEdgeStillValid(headBlocked,*edge),"head-block fixture still has genuine hand contacts");
    Traversal blocked;blocked.cfg=cfg;
    check(!blocked.attach(headBlocked,{0,-45,0},{0,1,0},100,60),"upper solid intersecting capsule head must reject entry despite real hands");
    std::cout<<"stacked complete captured paths="<<cases<<" and real overhead body blocker rejected\n";
}
int main(int argc,char**argv) {try {positiveGeometry();rejectGeometry();invalidation();sourceHandSpans();stackedSeedAcquisition();
    check(argc==2,"supply production motion library for complete stacked route verification");Library library;
    check(library.load(argv[1]),"production motion library loads");stackedTraversal(library);return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
