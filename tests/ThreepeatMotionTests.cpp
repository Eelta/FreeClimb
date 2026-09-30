#include "Pose.h"
#include "TraversalCapture.h"
#include <iostream>
#include <stdexcept>
#include <vector>
#include <memory>
#include <fstream>
#include <iomanip>
using namespace fc;
static void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
static std::string dumpDirectory;
static void writeVec(std::ostream& out,Vec v){out<<'['<<v.x<<','<<v.y<<','<<v.z<<']';}

struct ThreepeatWorld:World {
    struct Box {Vec low,high;};
    std::vector<Box> boxes;
    std::vector<Vec> missingTop;
    Vec origin{};float yaw{};
    Vec rotate(Vec p,float angle) const {return {p.x*std::cos(angle)-p.y*std::sin(angle),p.x*std::sin(angle)+p.y*std::cos(angle),p.z};}
    Vec point(Vec p) const{return origin+rotate(p,yaw);}
    Vec vector(Vec p) const{return rotate(p,yaw);}
    std::optional<Hit> ray(Vec a,Vec b) override {
        a=rotate(a-origin,-yaw);b=rotate(b-origin,-yaw);const auto d=b-a;
        std::optional<Hit> found;float nearest=2;
        const float begin[]{a.x,a.y,a.z},direction[]{d.x,d.y,d.z};
        for(const auto& box:boxes) {
            const float low[]{box.low.x,box.low.y,box.low.z},high[]{box.high.x,box.high.y,box.high.z};
            for(int axis=0;axis<3;++axis)for(int side=0;side<2;++side) {
                if(std::abs(direction[axis])<1e-7f)continue;
                const float time=((side?high[axis]:low[axis])-begin[axis])/direction[axis];
                if(time<0||time>1||time>=nearest)continue;
                const auto hit=a+d*time;const float p[]{hit.x,hit.y,hit.z};bool within=true;
                for(int other=0;other<3;++other)if(other!=axis)within&=p[other]>=low[other]-.0001f&&p[other]<=high[other]+.0001f;
                if(!within)continue;
                bool gap=false;
                if(axis==2&&side==1)for(auto missing:missingTop)
                    gap|=std::hypot(hit.x-missing.x,hit.y-missing.y)<.35f;
                if(gap)continue;
                Vec normal{};const float sign=side?1.f:-1.f;
                if(axis==0)normal.x=sign;else if(axis==1)normal.y=sign;else normal.z=sign;
                nearest=time;found=Hit{point(hit),vector(normal),true};
            }
        }
        return found;
    }
};
static Traversal attached(ThreepeatWorld& world,const Library& lib,bool enabled=true) {
    Traversal t;t.cfg.gap=37;t.cfg.radius=31;t.cfg.height=138;t.cfg.approachSeconds=.01f;
    check(lib.configureThreepeat(t.cfg),"new motion library must provide valid independent calibration");
    t.cfg.threepeatAnimations=enabled;
    check(t.attach(world,world.point({0,-45,0}),world.vector({0,1,0}),100,60),"test must attach through production geometry");
    t.update(world,{},.05f,100);check(t.state==State::wall,"checked approach must finish before action");return t;
}
static Vec posePoint(Vec p,const Traversal& t){return t.position+Vec{-t.normal.y,t.normal.x,0}*p.x-t.normal*p.y+Vec{0,0,p.z};}
static void clips(const Library& lib) {
    check(lib.hasThreepeat(),"the captured animation family must exist");
    for(auto motion:{Motion::contextHang,Motion::contextHopLeft,Motion::contextHopRight,Motion::contextMantle}) {
        const auto& clip=lib.clip(motion);check(clip.frames.size()>=40,"full new animation take must be present");
        for(int frame=0;frame<=240;++frame) {
            const float phase=frame/240.f;const auto pose=lib.sample(motion,phase);
            for(const auto& bone:pose)check(bone.t.finite()&&std::abs(bone.q.dot(bone.q)-1)<.002f,"new animation transforms must be finite normalized");
            for(int hand=0;hand<2;++hand) {
                if(!lib.armBendValid(pose,hand))std::cerr<<"source elbow motion="<<int(motion)<<" phase="<<phase<<" hand="<<hand<<'\n';
                check(lib.armBendValid(pose,hand),"source interpolation must preserve the anatomical elbow branch");
            }
        }
    }
    for(bool left:{false,true}) {
        check(std::abs(threepeatHopTravel(left,0))<.00001f&&std::abs(threepeatHopTravel(left,1)-1)<.00001f,"new root travel has exact endpoints");
        for(int i=0;i<=1000;++i)for(int hand=0;hand<2;++hand) {
            const auto phase=i/1000.f,a=threepeatSourceWeight(left,hand,phase),b=threepeatTargetWeight(left,hand,phase);
            check(a>=0&&a<=1&&b>=0&&b<=1&&a*b<.000001f,"one hand cannot hold the source and target simultaneously");
        }
    }
}
static void integrated(const Library& lib,int fps,int direction,bool far) {
    Settings calibration;check(lib.configureThreepeat(calibration),"configure test source height");
    ThreepeatWorld world;world.boxes={{{-1000,0,-500},{1000,300,calibration.threepeatHangHeight}}};
    if(far){world.origin={131316.86f,38643.36f,-11331.41f};world.yaw=.633f;}
    auto t=attached(world,lib);SurfacePose animator;Pose previous;std::array<Vec,4> oldEnds{};
    float worstAngle=0,worstStep=0,worstPalm=0,worstTopPalm=0,worstPhase=0;unsigned hopFrames=0,planted=0,mantleFrames=0;
    int worstBone=0,worstMotion=0,palmHand=0;float palmPhase=0;
    bool newIdle=false,newHop=false,newTop=false;const float dt=1.f/fps;
    std::ofstream dump;
    if(fps==60&&!far&&!dumpDirectory.empty()) {
        dump.open(dumpDirectory+"/threepeat-chain-"+(direction<0?"left":"right")+".json");
        check(bool(dump),"open diagnostic output inside specified project directory");
        dump<<std::setprecision(9)<<"{\"bones\":[";
        for(std::size_t bone=0;bone<lib.names.size();++bone)dump<<(bone?",":"")<<std::quoted(lib.names[bone]);
        dump<<"],\"parents\":[";
        for(std::size_t bone=0;bone<lib.parents.size();++bone)dump<<(bone?",":"")<<lib.parents[bone];
        dump<<"],\"fps\":60,\"frames\":[";
    }
    unsigned sampled=0;
    auto frame=[&](Input input) {
        const auto r=t.update(world,input,dt,100);
        const auto pose=animator.update(lib,world,t,r.motion,dt,1);const auto body=lib.world(pose);
        check(pose.size()==99,"actual SurfacePose must return every target bone");
        std::array<Vec,4> ends{posePoint(lib.palm(body,0),t),posePoint(lib.palm(body,1),t),posePoint(body[8].t,t),posePoint(body[11].t,t)};
        for(int hand=0;hand<2;++hand)check(lib.armBendValid(pose,hand),"output pose cannot reverse elbows at a source-family transition");
        if(threepeatMotion(r.motion))for(int hand=0;hand<2;++hand) {
            const int elbow=hand?32:29,wrist=hand?39:38,middle=hand?88:73;
            const auto forearm=(body[wrist].t-body[elbow].t).unit(),finger=(body[middle].t-body[wrist].t).unit();
            check(std::acos(std::clamp(forearm.dot(finger),-1.f,1.f))<=1.658063f+.001f,
                "final contact solve cannot bend either new-family wrist beyond 95 degrees");
        }
        if(dump.is_open()) {
            dump<<(sampled?",":"")<<"{\"motion\":"<<int(r.motion)<<",\"phase\":"<<t.progress()<<",\"position\":";
            writeVec(dump,t.position);dump<<",\"normal\":";writeVec(dump,t.normal);dump<<",\"local\":[";
            for(std::size_t bone=0;bone<pose.size();++bone) {
                dump<<(bone?",":"")<<"{\"t\":";writeVec(dump,pose[bone].t);
                const auto q=pose[bone].q;dump<<",\"q\":["<<q.x<<','<<q.y<<','<<q.z<<','<<q.w<<"]}";
            }
            dump<<"],\"edgeHands\":[";
            for(int hand=0;hand<2;++hand){if(hand)dump<<',';writeVec(dump,t.edgeHand(hand,true));}
            dump<<"],\"topHands\":[";
            for(int hand=0;hand<2;++hand){if(hand)dump<<',';writeVec(dump,t.topHand(hand));}
            dump<<"]}";++sampled;
        }
        if(!previous.empty()) {
            for(std::size_t bone=0;bone<pose.size();++bone) {
                const float angle=angleBetween(previous[bone].q,pose[bone].q);
                if(angle>worstAngle){worstAngle=angle;worstBone=int(bone);worstMotion=int(r.motion);worstPhase=t.progress();}
            }
            for(int limb=0;limb<4;++limb)worstStep=std::max(worstStep,(ends[limb]-oldEnds[limb]).length());
        }
        if(r.motion==Motion::contextHang)newIdle=true;
        if(threepeatHop(r.motion)) {
            newHop=true;++hopFrames;
            for(int hand=0;hand<2;++hand) {
                const float source=threepeatSourceWeight(direction<0,hand,t.actionProgress());
                const float target=threepeatTargetWeight(direction<0,hand,t.actionProgress());
                if(std::max(source,target)>.95f) {
                    const auto anchor=t.edgeHand(hand,target>source)+Vec{0,0,.8f};
                    const float error=(ends[hand]-anchor).length();
                    if(error>worstPalm){worstPalm=error;palmHand=hand;palmPhase=t.actionProgress();}++planted;
                }
            }
        }
        if(r.motion==Motion::contextMantle){newTop=true;++mantleFrames;worstTopPalm=std::max(worstTopPalm,animator.topPalmError);}
        const auto same=animator.update(lib,world,t,r.motion,0,1);
        for(std::size_t bone=0;bone<pose.size();++bone)check(angleBetween(same[bone].q,pose[bone].q)<.00001f&&(same[bone].t-pose[bone].t).length()==0,"duplicate output callbacks cannot advance animation");
        previous=pose;oldEnds=ends;return r;
    };
    for(int i=0;i<fps/2;++i)check(frame({}).motion==Motion::hang,
        "neutral lip contact remains ordinary hang without independently selecting39");
    Input hop{float(direction),0};hop.hop=true;const auto initial=frame(hop);
    std::cout<<"start fps="<<fps<<" side="<<direction<<" motion="<<int(initial.motion)<<" state="<<int(t.state)
        <<" prepare="<<t.preparingEdge()<<" status="<<t.contextReason()<<'\n';
    for(int i=0;i<fps*3&&(t.preparingEdge()||t.state==State::action);++i) {
        const auto r=frame({});check(!r.released,"new hop path and target must remain supported");
    }
    check(newIdle&&newHop&&hopFrames>fps/2&&t.state==State::wall,"real lip input must use the new side-hop preparation, complete capture and catch");
    unsigned catchFrames=0,ordinaryFrames=0;
    for(int i=0;i<fps/2;++i) {
        const auto motion=frame({}).motion;
        check(motion==Motion::contextHang||motion==Motion::hang,"completed side hop uses only its matching short catch then ordinary hang");
        if(motion==Motion::contextHang)++catchFrames;else ++ordinaryFrames;
    }
    check(catchFrames>0&&catchFrames<=unsigned(std::ceil(.18f*fps))+1&&ordinaryFrames>0,
        "matching39 catch expires after the existing cooldown instead of creating a permanent hang mode");

    auto capture=std::make_unique<TraversalCapture>();capture->begin(t,{},dt,100);TraversalCapture::RecordingWorld recording(world,*capture);
    auto result=t.update(recording,{},dt,100);capture->finish(t,result);
    auto loaded=std::make_unique<TraversalCapture>();std::string error;
    check(loaded->deserialize(capture->serialize(),error)&&loaded->replay().matched,"same-version replay must retain Threepeat state");
    Input mantle{0,1};mantle.mantle=true;frame(mantle);
    for(int i=0;i<fps*3&&t.active();++i)frame({});
    if(dump.is_open())dump<<"]}";
    std::cout<<"Threepeat fps="<<fps<<" side="<<direction<<" far="<<far<<" hop="<<hopFrames<<" top="<<mantleFrames
        <<" angle="<<worstAngle<<" bone="<<worstBone<<" motion="<<worstMotion<<" phase="<<worstPhase<<" step="<<worstStep<<" palm="<<worstPalm<<" palmHand="<<palmHand<<" palmPhase="<<palmPhase<<" topPalm="<<worstTopPalm<<'\n';
    check(newTop&&mantleFrames>fps/2&&!t.active(),"verified full platform must play new mantle through completion");
    check(planted>fps/5&&worstPalm<5&&worstTopPalm<5,"loaded source/target palms retain original five-unit bound");
    check(worstAngle<=12.566371f*dt+.015f,"entire new transition chain obeys the existing angular budget");
    check(worstStep<=750*dt+1.f,"new chain cannot teleport hands or feet");
}
static void negative(const Library& lib) {
    ThreepeatWorld world;world.boxes={{{-1000,0,-500},{1000,100,1000}}};
    auto enabled=attached(world,lib),disabled=attached(world,lib,false);
    for(int frame=0;frame<60;++frame) {
        Input input{1,0};if(frame==0)input.hop=true;
        auto a=enabled.update(world,input,1.f/60,100),b=disabled.update(world,input,1.f/60,100);
        check(a.motion==b.motion&&enabled.state==disabled.state&&(enabled.position-disabled.position).length()<.0001f,"without a real edge new family must preserve the legacy path");
    }
    Settings c;lib.configureThreepeat(c);world.boxes={{{-1000,0,-500},{1000,300,c.threepeatHangHeight}}};
    for(int side:{-1,0,1}) {
        auto t=attached(world,lib);Input run{float(side),side==0?1.f:0.f};run.run=run.hop=true;
        const auto r=t.update(world,run,1.f/60,100);
        check(!hopMotion(r.motion)&&!t.preparingEdge(),"wall run plus Space still cannot create a new or old hop");
    }
    auto t=attached(world,lib);Input release{0,-1};release.release=release.backDrop=true;
    const auto r=t.update(world,release,1.f/60,100);
    check(r.motion==Motion::dropBack||r.motion==Motion::backFlipOut,"S plus Space remains the independent wall departure");

    const Vec first{-c.threepeatMantleHalfWidth,3,c.threepeatHangHeight};
    const Vec second=first+Vec{c.threepeatMantleReplant[0].x,c.threepeatMantleReplant[0].y,0};
    world.missingTop={second};t=attached(world,lib);
    Input climb{0,1};climb.mantle=true;
    const auto fallback=t.update(world,climb,1.f/60,100);
    check(t.state==State::mantle&&fallback.motion==Motion::contextMantle&&!t.preciseTopContacts(),
        "missing actual second-press surface selects the adaptive42 route without claiming exact captured replant contacts");
    world.missingTop.clear();
    for(float phase:{.01f,.25f,.40f,.75f}) {
        t=attached(world,lib);check(t.update(world,climb,1.f/60,100).motion==Motion::contextMantle,
            "support invalidation regression must first start a real new mantle");
        while(t.progress()<phase)check(!t.update(world,{},1.f/60,100).released,"platform remains present before mutation");
        const auto geometry=world.boxes;world.boxes.clear();
        const auto lost=t.update(world,{},1.f/60,100);
        check(lost.released&&!lost.completed&&!t.active(),"removed platform during new mantle must fall rather than complete in air");
        world.boxes=geometry;
    }
    t=attached(world,lib);t.update(world,climb,1.f/60,100);
    while(t.progress()<.39f)t.update(world,{},1.f/60,100);
    world.missingTop={second};const auto lostSecond=t.update(world,{},1.f/60,100);
    check(lostSecond.released&&!lostSecond.completed,"losing only the newly loaded second palm cannot be hidden by a zero cosmetic weight");
    world.missingTop.clear();
}
int main(int argc,char** argv) {
    try {
        check(argc>=2&&argc<=4,"supply runtime motion library path, optional 38-clip baseline and diagnostic directory");Library lib;check(lib.load(argv[1]),"load old plus new source library");
        if(argc>=3) {
            Library old;check(old.load(argv[2])&&!old.hasThreepeat(),"the original 38-clip library must remain loadable");
            Settings oldConfig;oldConfig.threepeatAnimations=true;
            check(!old.configureThreepeat(oldConfig)&&!oldConfig.threepeatAnimations,"missing new source clips must disable the new action family");
            ThreepeatWorld w;w.boxes={{{-500,0,-500},{500,100,500}}};Traversal t;t.cfg=oldConfig;t.cfg.approachSeconds=.01f;
            check(t.attach(w,{0,-40,0},{0,1,0},100,60),"old data fallback keeps wall attachment functional");
            SurfacePose animator;
            for(int i=0;i<60;++i) {
                const auto result=t.update(w,{},1.f/60,100);const auto pose=animator.update(old,w,t,result.motion,1.f/60,1);
                check(!threepeatMotion(result.motion)&&pose.size()==99,"missing clips cannot emit new empty-pose animation IDs");
            }
        }
        if(argc==4)dumpDirectory=argv[3];
        clips(lib);negative(lib);
        for(int fps:{30,60,120})for(int side:{-1,1})for(bool far:{false,true})integrated(lib,fps,side,far);
        std::cout<<"Threepeat source, geometry, real pose transitions and controls passed\n";return 0;
    } catch(const std::exception& error){std::cerr<<"Threepeat failure: "<<error.what()<<'\n';return 1;}
}
