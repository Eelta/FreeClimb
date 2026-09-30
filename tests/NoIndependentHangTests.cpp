#define main noIndependentThreepeatFixtureMain
#include "ThreepeatMotionTests.cpp"
#undef main

static void ordinaryIdle(const Library& lib,int fps,float scale,bool lip) {
    ThreepeatWorld world;world.boxes={{{-5000,0,-5000},{5000,1000,10000}}};
    Traversal t;t.cfg.gap=37*scale;t.cfg.radius=31*scale;t.cfg.height=138*scale;
    t.cfg.chest=70*scale;t.cfg.grip=112*scale;t.cfg.contextScale=scale;
    t.cfg.approachSeconds=.01f;t.cfg.surfaceActionVariants=true;t.cfg.automaticClimbActions=true;
    check(lib.configureThreepeat(t.cfg),"existing39-42 library remains enabled");t.cfg.threepeatAnimations=true;
    if(lip)world.boxes[0].high.z=t.cfg.threepeatHangHeight*scale;
    check(t.attach(world,{0,-45*scale,0},{0,1,0},1000,60*scale),"ordinary idle attaches through physical geometry");
    t.update(world,{},.05f,1000);const Vec start=t.position;
    for(int frame=0;frame<fps*5;++frame) {
        const auto result=t.update(world,{},1.f/fps,1000);
        check(t.active()&&!result.released,"releasing directions still means normal hanging, never dropping");
        check(result.motion==Motion::hang&&!t.preparingEdge()&&!t.usesEdgeTargets(result.motion),
            "neutral input never discovers or maintains the removed independent39 hang");
        check((t.position-start).length()<.001f,"removing independent39 also removes its automatic actor repositioning");
    }
    check(t.contextIdleCount()==0&&t.automaticActionCount()==0,"idle cannot count a special idle or random movement action");
    std::cout<<"NO_INDEPENDENT39 fps="<<fps<<" scale="<<scale<<" lip="<<lip<<'\n';
}
int main(int argc,char** argv) {
    try {check(argc==2,"motion path required");Library lib;check(lib.load(argv[1]),"unchanged42 motion library");
        for(int fps:{30,60,120})for(float scale:{.75f,1.f,1.3f})for(bool lip:{false,true})ordinaryIdle(lib,fps,scale,lip);
        std::cout<<"PASS no independently discovered39 idle, ordinary wall hanging and position retained\n";return 0;
    }catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}
