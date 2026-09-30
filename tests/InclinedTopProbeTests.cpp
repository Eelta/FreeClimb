#define main eaveFixtureMain
#include "EaveBypassTests.cpp"
#undef main

static EaveWorld inclinedWorld(float yaw,Vec origin,bool overhang=false) {
    EaveWorld w;w.shapes.clear();w.yaw=yaw;w.origin=origin;
    w.shapes.push_back({box({-300,0,-500},{300,300,130})});
    const Vec n{0,-.520399f,.853923f};
    auto roof=box({-300,overhang?-10.f:0.f,overhang?-500.f:130.f},{300,300,400});
    if(overhang)roof.push_back({n*-1,-n.dot({0,0,128})});
    roof.push_back({n,n.dot({0,0,130})});w.shapes.push_back({roof});
    return w;
}
static Traversal inclinedActor(EaveWorld& w,float height=0) {
    Traversal t;t.cfg.radius=31;t.cfg.gap=37;t.cfg.height=138;t.cfg.approachSeconds=0;t.cfg.automaticClimbActions=false;
    check(t.attach(w,w.global({0,-37,height}),w.rotate({0,1,0},w.yaw),1000),"real front face attaches below the roof");return t;
}

static bool inclinedBodyClear(const EaveWorld& w,Vec feet) {
    const Vec p=w.local(feet);
    for(int iz=0;iz<=138;++iz) {
        const float z=float(iz),cap=z<31?31-z:z>107?z-107:0;
        const float radius=std::sqrt(std::max(0.f,31*31-cap*cap));
        for(int ring=0;ring<=64;++ring) {
            const float a=ring*6.28318530718f/64,r=ring==64?0.f:radius;
            const Vec v=p+Vec{r*std::cos(a),r*std::sin(a),z};
            for(const auto& shape:w.shapes) {
                bool inside=true;for(const auto& plane:shape.planes)inside&=v.dot(plane.n)<plane.d-.05f;
                if(inside)return false;
            }
        }
    }
    return true;
}
static void validSlope(int fps,float yaw,Vec origin) {
    auto w=inclinedWorld(yaw,origin);auto t=inclinedActor(w);w.casts=0;
    auto r=t.update(w,{0,1,false,true},1.f/fps,1000);
    if(t.state!=State::mantle||std::abs(t.topStart().z-origin.z)>.03f)std::cerr<<"inclined failure fps="<<fps<<" yaw="<<yaw<<" originZ="<<origin.z<<" state="<<int(t.state)<<" startZ="<<t.topStart().z<<" positionZ="<<t.position.z<<" reason="<<t.ledgeReason<<"\n";
    check(t.state==State::mantle&&std::abs(t.topStart().z-origin.z)<.03f,"walkable horizontal roof hit discovers a top before another climb step");
    check(w.casts<1400,"bounded five-candidate discovery and existing full mantle preflight");
    const auto stand=w.local(t.topTarget()),lip=w.local(t.topLip());
    check(stand.z>140&&stand.z<165&&lip.z>129&&lip.z<140,"higher feet retain a lower actual reachable lip");
    for(int hand=0;hand<2;++hand) {
        const Vec palm=w.local(t.topHand(hand));
        check(std::abs(palm.z-(130+.520399f/.853923f*palm.y))<.06f,"both palms lie on the actual pitched solid");
        const Vec anchor{hand==0?-18.f:18.f,-37,112};

        const Vec correctAnchor{palm.x<0?-18.f:18.f,anchor.y,anchor.z};
        check((palm-correctAnchor).length()<=61.05f,"original real hand reach remains unchanged");
    }
    bool complete=false;
    for(int frame=0;frame<fps*3&&t.active();++frame) {
        r=t.update(w,{0,1,false,true},1.f/fps,1000);
        check(inclinedBodyClear(w,t.position),"entire ascent, crossing and descent remain outside the real roof");
        check(!r.released||r.completed,"valid pitched top has no forced drop");complete|=r.completed;
    }
    check(complete&&!t.active(),"pitched roof top-out returns native movement");
}
static void rejectedSlope(float yaw,Vec origin) {

    auto over=inclinedWorld(yaw,origin,true);auto t=inclinedActor(over,-14);
    t.update(over,{0,1,false,true},1.f/60,1000);
    check(t.state!=State::mantle&&inclinedBodyClear(over,t.position),"real projecting eave is still a blocker");
    auto ceiling=inclinedWorld(yaw,origin);ceiling.shapes.push_back({box({-300,-100,149},{300,100,151})});
    auto low=inclinedActor(ceiling);low.update(ceiling,{0,1,false,true},1.f/60,1000);
    check(low.state!=State::mantle&&inclinedBodyClear(ceiling,low.position),"two-unit roof clearance blocker cannot be crossed");
    auto invalid=inclinedWorld(yaw,origin);auto bad=inclinedActor(invalid);invalid.invalidRoof=true;
    bad.update(invalid,{0,1,false,true},1.f/60,1000);
    check(bad.state!=State::mantle,"unclimbable inclined collision does not become an artificial floor");
    auto narrow=inclinedWorld(yaw,origin);narrow.shapes[0].planes.push_back({{0,1,0},18});narrow.shapes[1].planes.push_back({{0,1,0},18});
    auto shortTop=inclinedActor(narrow);shortTop.update(narrow,{0,1,false,true},1.f/60,1000);
    check(shortTop.state!=State::mantle,"missing uphill footprint support cannot be projected from the measured plane");
    auto dynamic=inclinedWorld(yaw,origin);auto live=inclinedActor(dynamic);
    live.update(dynamic,{0,1,false,true},1.f/60,1000);check(live.state==State::mantle,"dynamic blocker fixture plans the real slope");
    dynamic.shapes.push_back({box({-300,-100,149},{300,100,151})});bool aborted=false;
    for(int i=0;i<180&&live.active();++i){const auto r=live.update(dynamic,{0,1,false,true},1.f/60,1000);aborted|=r.released&&!r.completed;check(inclinedBodyClear(dynamic,live.position),"new obstacle aborts before crossing it");}
    check(aborted,"new collision still revalidates every mantle frame");
}
int main(){try {unsigned cases=0;
    for(float yaw:{0.f,.73f})for(Vec origin:{Vec{},Vec{131146.67f,38939.656f,-12205.819f}}) {
        for(int fps:{30,60,120}){validSlope(fps,yaw,origin);++cases;}
        rejectedSlope(yaw,origin);cases+=5;
    }
    std::cout<<"PASS real inclined-top discovery, complete body paths and negative geometry: "<<cases<<" scenarios\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
