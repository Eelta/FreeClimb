#define main attachSurfaceFixtureMain
#include "AttachSurfaceTests.cpp"
#undef main

static bool handoffCapsuleClear(const AttachWorld& w,Vec feet,float radius,float height) {
    const Vec p=w.local(feet);const float low=p.z+radius,high=p.z+height-radius;
    for(const auto& b:w.boxes) {
        const float x=p.x-std::clamp(p.x,b.low.x,b.high.x),y=p.y-std::clamp(p.y,b.low.y,b.high.y);
        const float z=high<b.low.z?b.low.z-high:low>b.high.z?low-b.high.z:0;
        if(std::sqrt(x*x+y*y+z*z)<radius-.035f)return false;
    }
    return true;
}
static AttachWorld handoffWorld(float yaw,Vec origin) {
    AttachWorld w;w.rotation=yaw;w.origin=origin;
    w.boxes={{{-1000,-1000,-1000},{1000,1000,0}},{{-1000,94.5f,-1000},{1000,500,1000}},
        {{22.5f,30,7},{60,85,8}}};
    return w;
}
static Traversal groundedHandoff(AttachWorld& w,int fps) {
    Traversal t;t.cfg.radius=31;t.cfg.gap=37;t.cfg.height=138;t.cfg.automaticClimbActions=false;
    check(t.attach(w,w.global({0,0,0}),w.direction({0,1,0}),1000,60,false,true),"rounded native ground entry is actually accepted");
    check(t.roundedEntryPath(),"fixture must exercise the verified rounded-ground path");
    t.entry(Motion::runLaunch,true);
    for(int frame=0;t.state==State::approach&&frame<fps*2;++frame) {
        const auto r=t.update(w,{},1.f/fps,1000);
        check(!r.released&&handoffCapsuleClear(w,t.position,31,138),"entry keeps exact radius31 height138 capsule outside every closed solid");
    }
    check(t.state==State::wall,"entry completes before ordinary wall movement");return t;
}
static void continuation(int fps,float yaw,Vec origin,bool side) {
    auto w=handoffWorld(yaw,origin);auto t=groundedHandoff(w,fps);const Vec start=t.position;
    for(int frame=0;frame<fps;++frame) {
        const auto r=t.update(w,side?Input{-1,0}:Input{0,1},1.f/fps,1000);
        if(r.released)std::cerr<<"released fps="<<fps<<" yaw="<<yaw<<" side="<<side<<" frame="<<frame<<" reason="<<r.reason<<" xyz="<<w.local(t.position).x<<","<<w.local(t.position).y<<","<<w.local(t.position).z<<"\n"; check(!r.released,"grounded handoff does not drop or disable controls");
        check(handoffCapsuleClear(w,t.position,31,138),"every continued wall frame clears the independent unchanged physical capsule"); if(frame==0)check((t.position-start).length()>.1f,"the first ordinary wall step must use a consistent body profile");
    }
    const Vec moved=w.local(t.position)-w.local(start);
    check(side?moved.x<-20:moved.z>40,"first supported climb/side movement must not freeze after rounded entry");

    check(t.entryLiftHeight()>0,"an incompatible zero-lift endpoint is replaced by an existing bounded ground jump");
}
static void blockedEntry(int fps,float yaw,Vec origin) {
    auto w=handoffWorld(yaw,origin);
    auto fresh=[](){Traversal t;t.cfg.radius=31;t.cfg.gap=37;t.cfg.height=138;return t;};
    auto air=fresh();check(!air.attach(w,w.global({0,0,0}),w.direction({0,1,0}),1000,60,true,false),
        "an airborne request cannot borrow rounded ground clearance or the ground-lift budget");
    auto capped=fresh();capped.cfg.groundJumpHeight=16;
    check(!capped.attach(w,w.global({0,0,0}),w.direction({0,1,0}),1000,60,false,true)&&!capped.active(),
        "insufficient configured lift budget rejects before taking native movement");
    w.boxes.push_back({{-1000,40,128},{1000,85,130}});
    check(handoffCapsuleClear(w,w.global({0,0,0}),31,138),"thin ceiling leaves the native starting capsule clear");
    auto roof=fresh();check(!roof.attach(w,w.global({0,0,0}),w.direction({0,1,0}),1000,60,false,true)&&!roof.active(),
        "a two-unit-thick ceiling through all permitted landing heights still blocks the full entry");
    auto plain=handoffWorld(yaw,origin);plain.boxes.pop_back();auto ordinary=fresh();
    check(ordinary.attach(plain,plain.global({0,0,0}),plain.direction({0,1,0}),1000,60,false,true)&&
        ordinary.entryLiftHeight()==0&&!ordinary.roundedEntryPath(),"unobstructed ordinary entry keeps its original path and duration");
}int main(){try {
    unsigned cases=0;
    for(int fps:{30,60,120})for(float yaw:{0.f,.73f})for(Vec origin:{Vec{},Vec{131089.3f,38954.9f,-12204.7f}}) {
        continuation(fps,yaw,origin,false);continuation(fps,yaw,origin,true);blockedEntry(fps,yaw,origin);cases+=3;
    }
    std::cout<<"PASS grounded rounded entry handoff and original entry exclusions: "<<cases<<" scenarios\n";return 0;
} catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
