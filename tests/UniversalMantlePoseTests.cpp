#ifdef _WIN32
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#undef far
#undef near
#endif
#define main eaveFixtureMain
#include "EaveBypassTests.cpp"
#undef main

static EaveWorld topWorld(float height,float normalZ,bool distant) {
    EaveWorld world;world.shapes.clear();
    world.shapes.push_back({box({-1000,0,-1000},{1000,1000,height})});
    if(normalZ<1) {
        auto roof=box({-1000,0,height},{1000,1000,2000});
        const Vec normal{0,-std::sqrt(1-normalZ*normalZ),normalZ};
        roof.push_back({normal,normal.dot({0,0,height})});world.shapes.push_back({roof});
    }
    if(distant){world.origin={134559.219f,36994.7031f,-11691.1309f};world.yaw=.633f;}
    return world;
}
static Traversal topActor(const Library& library,EaveWorld& world,float scale) {
    Traversal traversal;traversal.cfg.contextScale=scale;traversal.cfg.gap=37;
    traversal.cfg.radius=31;traversal.cfg.height=138;traversal.cfg.approachSeconds=0;
    traversal.cfg.automaticClimbActions=false;
    check(library.configureThreepeat(traversal.cfg),"real HKX configures mantle metadata");
    traversal.cfg.threepeatAnimations=true;
    check(traversal.attach(world,world.global({0,-37,0}),world.rotate({0,1,0},world.yaw),1000),
        "actual wall geometry admits the initial climbing state");
    return traversal;
}
static float solidDepth(const EaveWorld& world,Vec point) {
    point=world.local(point);float depth=0;
    for(const auto& shape:world.shapes) {
        float inside=1.e10f;
        for(const auto& plane:shape.planes)inside=std::min(inside,plane.d-point.dot(plane.n));
        depth=std::max(depth,inside);
    }
    return depth;
}
static void topPose(const Library& library,int kind,int fps,bool distant,float scale) {
    const float height=kind==0?48.f:kind==3?96.f:kind==4?22.f:kind==5||kind==6?40.f:kind==7?64.f:130.f,normalZ=kind==1||kind==6?.8f:1.f,dt=1.f/fps;
    auto world=topWorld(height,normalZ,distant);auto traversal=topActor(library,world,scale);
    SurfacePose surface;Pose previous;std::array<Vec,4> previousEnds{};Motion prior=Motion::none;
    float angleExcess=0,endpointExcess=0,palmError=0,fingerDepth=0,footDepth=0,footPhase=0;int footBone=-1;
    unsigned mantleFrames=0;bool completed=false,selected=false;float firstAdvance=-1,selectedHeight=0;Vec mantleStart{};
    auto point=[&](Vec value) {
        return traversal.position+(Vec{-traversal.normal.y,traversal.normal.x,0}*value.x-
            traversal.normal*value.y+Vec{0,0,value.z})*scale;
    };
    for(int frame=-10;frame<fps*5&&traversal.active();++frame) {
        Result result;
        if(frame<0)result.motion=Motion::hang;
        else result=traversal.update(world,{0,1,false,true},dt,1000);
        check(!result.released||result.completed,"stable real platform completes without dropping");
        check(result.motion!=Motion::mantle&&result.motion!=Motion::step,"retired top-out slots are never selected");
        const Vec root=traversal.position;
        const auto pose=surface.update(library,world,traversal,result.motion,dt,scale);
        check((traversal.position-root).length()==0,"pose adaptation cannot alter the physical route");
        check(pose.size()==99,"complete Skyrim skeleton is retained");
        const auto body=library.world(pose);
        for(std::size_t bone=0;bone<pose.size();++bone) {
            check(pose[bone].t.finite()&&std::abs(pose[bone].q.dot(pose[bone].q)-1)<.002f,
                "finite normalized full pose");
            if(bone!=0&&bone!=4)check(std::abs(pose[bone].t.length()-library.rest[bone].t.length())<.002f,
                "fixed bone lengths remain unchanged");
            if(!previous.empty())angleExcess=std::max(angleExcess,
                angleBetween(previous[bone].q,pose[bone].q)-12.566371f*dt);
        }
        std::array<Vec,4> endpoints{};
        for(unsigned index=0;index<endpoints.size();++index) {
            endpoints[index]=point(body[std::array{8,11,38,39}[index]].t);
            if(!previous.empty())endpointExcess=std::max(endpointExcess,
                (endpoints[index]-previousEnds[index]).length()-(result.motion!=prior?600.f:750.f)*dt*scale);
        }
        for(int hand=0;hand<2;++hand)check(library.armBendValid(pose,hand),"elbows keep their legal bend branch");
        if(result.motion==Motion::contextMantle) {
            ++mantleFrames;
            if(!selected) {
                selected=true;mantleStart=traversal.position;selectedHeight=traversal.topLip().z-mantleStart.z;
                check(traversal.preciseTopContacts()==(kind==2),"geometry selects the precise or adaptive branch");
                check((traversal.topSampleBegin()>0)==(kind==0||kind==3||kind>=4),
                    "only low platforms use the released-hand portion of the same HKX");
            }
            if(kind==0||kind>=4) {
                check(traversal.lowTopStep()&&traversal.topPreparation()>=.999f,"a waist-low edge starts moving without a suspended grip preparation");
                check(surface.sampledPhase()>=.70f&&traversal.topHandWeight(0,surface.sampledPhase())<.001f&&traversal.topHandWeight(1,surface.sampledPhase())<.001f,
                    "the low step only plays the released-hand foot-placement portion of clip 42");
                if(firstAdvance<0&&(traversal.position-mantleStart).length()>.01f)firstAdvance=(mantleFrames-1)*dt;
            }
            if(traversal.topPreparation()>=.99f)palmError=std::max(palmError,surface.topPalmError);
            for(int hand=0;hand<2;++hand) {
                const int elbow=hand?32:29,wrist=hand?39:38,middle=hand?88:73;
                const float bend=std::acos(std::clamp((body[wrist].t-body[elbow].t).unit().dot(
                    (body[middle].t-body[wrist].t).unit()),-1.f,1.f));
                check(bend<=1.658063f+.001f,"mantle wrists remain inside the 95 degree limit");
                if(traversal.topPreparation()<.75f||traversal.topHandWeight(hand,surface.sampledPhase())<=.95f)continue;
                for(int digit=0;digit<5;++digit)for(int joint=0;joint<4;++joint) {
                    const int bone=(hand?82:67)+digit*3+std::min(joint,2);
                    Vec sample=body[bone].t;
                    if(joint==3)sample=sample+body[bone].q.rotate({0,0,library.rest[bone].t.length()*.75f});
                    fingerDepth=std::max(fingerDepth,solidDepth(world,point(sample)));
                }
            }
            for(int bone:{8,11,50,51}){const float depth=solidDepth(world,point(body[bone].t));if(depth>footDepth){footDepth=depth;footPhase=traversal.progress();footBone=bone;}}
        }
        completed|=result.completed;previous=pose;previousEnds=endpoints;prior=result.motion;
    }
    std::cout<<"universal kind="<<kind<<" fps="<<fps<<" distant="<<distant<<" scale="<<scale
        <<" mantle="<<mantleFrames<<" selectedHeight="<<selectedHeight<<" firstAdvance="<<firstAdvance<<" completed="<<completed<<" palm="<<palmError<<" finger="<<fingerDepth
        <<" foot="<<footDepth<<" footBone="<<footBone<<" footPhase="<<footPhase<<" angleExcess="<<angleExcess<<" endpointExcess="<<endpointExcess<<'\n';
    if(kind==0||kind>=4)check(firstAdvance>=0&&firstAdvance<.10f&&mantleFrames*dt<.65f,
        "a low edge continues its physical route promptly and finishes the short foot placement without the former preparation pause");
    check(completed&&selected&&!traversal.active(),"selected slot 42 completes and restores native control");
    check(palmError<5*scale,"loaded palms remain within five scaled units of real contacts");
    check(fingerDepth<=.15001f*scale,"loaded finger joints and tips remain outside the solid");
    if(kind!=2)check(footDepth<=.15001f*scale,"adaptive ankle and toe positions remain outside the solid");
    check(angleExcess<=.006f,"all output bones obey the existing angular speed limit");
    check(endpointExcess<=.03f,"actual world endpoints obey existing action and transition speed limits");
}
struct MissingSurface final:World {
    EaveWorld& source;std::optional<Vec> missing;unsigned removed{};
    explicit MissingSurface(EaveWorld& world):source(world){}
    std::optional<Hit> ray(Vec from,Vec to)override {
        auto hit=source.ray(from,to);
        if(hit&&missing&&(hit->point-*missing).length()<3.f){++removed;return {};}
        return hit;
    }
};
static void lostSupport(const Library& library,bool hand,int fps,bool distant,float scale) {
    auto world=topWorld(130,.8f,distant);auto traversal=topActor(library,world,scale);
    MissingSurface changed(world);const float dt=1.f/fps;
    for(int frame=0;frame<fps*3&&traversal.active();++frame) {
        traversal.update(changed,{0,1,false,true},dt,1000);
        if(traversal.state==State::mantle&&traversal.topPreparation()>=1&&traversal.progress()>=.08f)break;
    }
    check(traversal.state==State::mantle&&!traversal.preciseTopContacts(),"dynamic fixture reaches adaptive top-out");
    check(traversal.topHandWeight(0,traversal.topSamplePhase(traversal.progress()))>.95f,
        "dynamic removal occurs during actual hand loading");
    const auto floor=world.ray(traversal.topTarget()+Vec{0,0,8},traversal.topTarget()-Vec{0,0,40});
    check(floor.has_value(),"real standing support exists before removal");
    const Vec target=hand?traversal.topHand(0):floor->point;
    const Vec normal=hand?traversal.topHandNormal(0):floor->normal;
    check(world.ray(target+normal*3,target-normal*3).has_value(),"removed patch starts with a real collision hit");
    changed.missing=target;
    check(!changed.ray(target+normal*3,target-normal*3),"negative fixture only removes an existing collision patch");
    const Vec retained=hand?floor->point:traversal.topHand(0),retainedNormal=hand?floor->normal:traversal.topHandNormal(0);
    check(changed.ray(retained+retainedNormal*3,retained-retainedNormal*3).has_value(),
        "the other kind of support remains physically present");
    const Vec before=traversal.position;const unsigned previousRemoved=changed.removed;
    const auto result=traversal.update(changed,{0,1,false,true},dt,1000);
    check(changed.removed>previousRemoved,"live controller checks the removed support");
    check(result.released&&!result.completed&&!traversal.active(),"support loss aborts the adaptive mantle");
    check((traversal.position-before).length()<.03f,"support loss aborts before further root movement");
}
int main(int argc,char** argv) {
#ifdef _WIN32
    SetErrorMode(SEM_FAILCRITICALERRORS|SEM_NOGPFAULTERRORBOX|SEM_NOOPENFILEERRORBOX);
#endif
    try {
        check(argc==2,"provide the runtime HKX manifest");Library library;
        check(library.load(argv[1])&&library.hasThreepeat(),"load the actual runtime animation family");
        unsigned positive=0,negative=0;
        for(int kind:{0,1,2,3,4,5,6,7})for(int fps:{30,60,120})for(bool distant:{false,true})for(float scale:{1.f,1.03f}) {
            topPose(library,kind,fps,distant,scale);++positive;
        }
        for(bool hand:{false,true})for(int fps:{30,60,120})for(bool distant:{false,true})for(float scale:{1.f,1.03f}) {
            lostSupport(library,hand,fps,distant,scale);++negative;
        }
        std::cout<<"PASS universal mantle: "<<positive<<" pose cases and "<<negative<<" dynamic support cases\n";
    } catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}


