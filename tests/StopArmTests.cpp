#include "AnimationOverrides.h"
#include "Pose.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
static void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
struct ArmWall:World {
    Vec normal{0,-1,0};
    std::optional<Hit> ray(Vec a,Vec b) override {
        const float from=a.dot(normal),to=b.dot(normal);
        if(from<=0||to>=0)return {};
        return Hit{a+(b-a)*(from/(from-to)),normal,true};
    }
};

static void legacyPoleIK(const Library& lib,Pose& p,int a,int b,int c,Vec target,Vec pole) {
    auto w=lib.world(p);const Vec start=w[a].t,mid=w[b].t,end=w[c].t;
    const float upper=(mid-start).length(),lower=(end-mid).length();
    const Vec axis=(target-start).unit();
    const float distance=std::clamp((target-start).length(),std::abs(upper-lower)+.02f,upper+lower-.04f);
    Vec plane=pole-start;plane=plane-axis*plane.dot(axis);
    if(plane.length()<.01f){plane=axis.cross({0,1,0});if(plane.length()<.01f)plane=axis.cross({1,0,0});}
    const float along=(upper*upper-lower*lower+distance*distance)/(2*distance);
    const Vec joint=start+axis*along+plane.unit()*std::sqrt(std::max(0.f,upper*upper-along*along));
    lib.rotateWorld(p,a,Quat::between(mid-start,joint-start)*w[a].q);w=lib.world(p);
    lib.rotateWorld(p,b,Quat::between(w[c].t-w[b].t,start+axis*distance-w[b].t)*w[b].q);
}
static void capturedArmsRemainUnchanged(const Library& lib) {
    check(lib.armBends[0].valid&&lib.armBends[1].valid,"calibrate both elbow references from the loaded hang capture");
    int frames=0;float closest=10;
    for(const auto& clip:lib.clips)for(const auto& source:clip.frames) {
        auto guarded=source;
        check(lib.guardArmBends(guarded)==0,"legal captured arm poses must not be changed by the guard");
        for(int hand=0;hand<2;++hand){
            check(lib.armBendValid(source,hand),"all bundled captures use the calibrated anatomical half-space");
            closest=std::min(closest,lib.signedArmBend(source,hand));
        }
        for(std::size_t bone=0;bone<source.size();++bone)
            check(angleBetween(source[bone].q,guarded[bone].q)<1e-6f&&(source[bone].t-guarded[bone].t).length()==0,
                "source validation preserves every captured transform");
        ++frames;
    }
    check(closest>=0&&closest<.02f,"near-straight captured elbows remain legal instead of imposing a five-degree minimum");
    std::cout<<"unaltered captured frames="<<frames<<" closest signed elbow="<<closest<<'\n';
}
static float geometricElbowBend(const Library& lib,const Pose& pose,int hand) {
    const auto w=lib.world(pose);const int upper=hand?31:28,elbow=hand?32:29,wrist=hand?39:38;
    return std::acos(std::clamp((w[elbow].t-w[upper].t).unit().dot((w[wrist].t-w[elbow].t).unit()),-1.f,1.f));
}
static void repairedHingeHasMargin(const Library& lib) {
    float minimum=10,maximumLegacy=0;int cases=0;
    for(int hand:{0,1})for(float phase:{0.f,.4f,.9f})for(float backwards:{.03f,.4f,1.1f}) {
        auto p=lib.sample(Motion::runRight,phase);const int elbow=hand?32:29,wrist=hand?39:38,upper=hand?31:28;
        const auto& reference=lib.armBends[hand];
        const Vec reversed=reference.axis*std::cos(backwards)-reference.direction*std::sin(backwards);
        const Vec forearm=p[elbow].q.rotate(p[wrist].t).unit();
        p[elbow].q=(Quat::between(forearm,reversed)*p[elbow].q).unit();
        check(!lib.armBendValid(p,hand),"negative hinge fixture really crosses the calibrated bend half-space");
        const Pose before=p;const auto oldWorld=lib.world(before);

        auto legacy=before;const Vec bad=legacy[elbow].q.rotate(legacy[wrist].t).unit();
        const Vec projected=(bad+reference.direction*(.0001f-bad.dot(reference.direction))).unit();
        legacy[elbow].q=(Quat::between(bad,projected)*legacy[elbow].q).unit();
        const float legacyBend=geometricElbowBend(lib,legacy,hand);maximumLegacy=std::max(maximumLegacy,legacyBend);
        check(lib.armBendValid(legacy,hand)&&legacyBend<.01f,
            "negative control: the former legal half-space check accepts a nearly singular repaired elbow");
        check(lib.guardArmBend(p,hand),"a reversed hinge is actually repaired");
        const float bend=geometricElbowBend(lib,p,hand);minimum=std::min(minimum,bend);
        check(bend>5.9f*3.14159265f/180&&bend<6.1f*3.14159265f/180,
            "only an invalid pure hinge repairs to a measured six-degree positive bend rather than a singular straight arm");
        const auto solved=lib.world(p);
        check(std::abs((solved[elbow].t-solved[upper].t).length()-(oldWorld[elbow].t-oldWorld[upper].t).length())<.001f&&
            std::abs((solved[wrist].t-solved[elbow].t).length()-(oldWorld[wrist].t-oldWorld[elbow].t).length())<.001f,
            "repair keeps physical upper-arm and forearm lengths");
        for(std::size_t bone=0;bone<p.size();++bone)if(int(bone)!=elbow)
            check(angleBetween(before[bone].q,p[bone].q)<.000001f&&(before[bone].t-p[bone].t).length()==0,
                "the margin does not twist wrists, fingers, helpers or the other arm");
        const auto once=p;check(!lib.guardArmBend(p,hand),"repair is idempotent after reaching the safe side");
        for(std::size_t bone=0;bone<p.size();++bone)
            check(angleBetween(once[bone].q,p[bone].q)<.000001f,"repeated guard calls cannot accumulate another corrective bend");
        ++cases;
    }
    std::cout<<"repaired hinge cases="<<cases<<" min geometric bend="<<minimum<<" legacy maximum="<<maximumLegacy<<'\n';
}
static void inversePoleBranch(const Library& lib) {
    int oldFailures=0;
    for(auto test:{std::array<int,3>{2,81,1},{3,9,1},{4,67,0},{5,68,1},{26,7,1}}) {
        const int hand=test[2],a=hand?31:28,b=hand?32:29,c=hand?39:38;
        const auto& clip=lib.clip(Motion(test[0]));
        check(clip.frames.size()>std::size_t(test[1]),"captured inverse-branch fixture exists");
        const auto source=clip.frames[test[1]],w=lib.world(source);
        const auto target=w[c].t+w[a].q.rotate({0,-9,0});
        auto legacy=source;legacyPoleIK(lib,legacy,a,b,c,target,w[b].t);
        const bool oldAccepted=angleBetween(source[a].q,legacy[a].q)<=.65f&&angleBetween(source[b].q,legacy[b].q)<=.75f;
        if(oldAccepted&&lib.signedArmBend(legacy,hand)<-.02f)++oldFailures;
        auto solved=source;const float error=lib.ik(solved,a,b,c,target,w[b].t);
        const auto result=lib.world(solved);
        check(lib.armBendValid(solved,hand),"a reachable contact cannot select a reversed elbow branch");
        check(std::abs(error-(target-result[c].t).length())<.0001f,"IK reports actual residual after any anatomical repair");
        check(error<.06f,"the legal mirrored branch still reaches these known reachable targets");
        check(std::abs((result[b].t-result[a].t).length()-(w[b].t-w[a].t).length())<.001f&&
              std::abs((result[c].t-result[b].t).length()-(w[c].t-w[b].t).length())<.001f,
              "anatomical IK keeps both bone lengths fixed");
        for(int bone:{38,39,67,70,73,76,79,82,85,88,91,94})
            check(angleBetween(source[bone].q,solved[bone].q)<1e-6f,"arm branch repair does not twist wrists or fingers");
    }
    check(oldFailures==5,"negative controls reproduce inverse elbows that the former adaptation-angle checks accepted");
    std::cout<<"legacy accepted inverse branches="<<oldFailures<<'\n';
}
static void stoppedClimbing(const Library& lib) {
    int cases=0;
    for(float slope:{0.f,.65f})for(int fps:{30,60,120})
    for(Input input:{Input{0,1},Input{0,-1},Input{1,0},Input{-1,0}})for(int movingFrames=1;movingFrames<=90;movingFrames+=3) {
        ArmWall wall;wall.normal={0,-std::sqrt(1-slope*slope),slope};
        Traversal traversal;traversal.cfg.approachSeconds=0;
        traversal.cfg.gap=37;traversal.cfg.radius=31;traversal.cfg.height=138;
        check(traversal.attach(wall,{0,-37,0},{0,1,0},100),"actual stopped-climb fixture attaches");
        SurfacePose surface;const float dt=1.f/fps;
        Pose settled;
        for(int frame=0;frame<movingFrames+fps;++frame) {
            const auto result=traversal.update(wall,frame<movingFrames?input:Input{},dt,100);
            const auto p=surface.update(lib,wall,traversal,result.motion,dt,1);
            check(traversal.active()&&!result.released,"a cosmetic elbow constraint never drops a physically supported actor");
            for(int hand=0;hand<2;++hand)
                check(lib.armBendValid(p,hand),"real move-to-hang output stays in its anatomical bend half-space, including interrupted IK history");
            if(frame==movingFrames+fps-1)settled=p;
        }

        const auto result=traversal.update(wall,{},dt,100);
        const auto next=surface.update(lib,wall,traversal,result.motion,dt,1);
        for(int bone:{28,29,31,32}) {
            const float change=angleBetween(settled[bone].q,next[bone].q);
            if(change>=.02f)std::cerr<<"STOP slope="<<slope<<" fps="<<fps<<" input="<<input.x<<','<<input.y<<
                " movingFrames="<<movingFrames<<" bone="<<bone<<" delta="<<change<<'\n';
            check(change<.02f,"the stopped arms converge to a stable hang");
        }
        ++cases;
    }
    std::cout<<"actual stop sequences="<<cases<<'\n';
}
static void supportedIdleAfterStop(const Library& lib) {

    for(int fps:{30,60,120})for(bool sideways:{false,true}) {
        ArmWall wall;Traversal traversal;traversal.cfg.approachSeconds=0;
        traversal.cfg.gap=37;traversal.cfg.radius=31;traversal.cfg.height=138;
        check(traversal.attach(wall,{0,-37,0},{0,1,0},100),"long idle fixture attaches to a real plane");
        SurfacePose surface;const float dt=1.f/fps;
        const int movingFrames=sideways?(fps==30?40:fps==60?76:151):fps/2;
        Pose previous,rest;std::array<Vec,2> restingPalms{};Vec restPosition{};
        float palmError=0,palmTravel=0,armStep=0,chestTravel=0,minimumCOM=1e9f,maximumCOM=-1e9f;
        auto point=[&](Vec p){return traversal.position+Vec{-traversal.normal.y,traversal.normal.x,0}*p.x-
            traversal.normal*p.y+Vec{0,0,p.z};};
        for(int frame=0;frame<movingFrames+fps*9;++frame) {
            const Input input=frame<movingFrames?(sideways?Input{-1,0}:Input{0,1}):Input{};
            const auto result=traversal.update(wall,input,dt,100);
            const auto pose=surface.update(lib,wall,traversal,result.motion,dt,1),body=lib.world(pose);
            check(traversal.active()&&!result.released,"resting output cannot change supported traversal ownership");
            for(int hand=0;hand<2;++hand)check(lib.armBendValid(pose,hand),"long idle retains legal elbows");
            if(frame==movingFrames+fps) {
                rest=pose;restPosition=traversal.position;
                for(int hand=0;hand<2;++hand)restingPalms[hand]=point(lib.palm(body,hand));
            }
            if(!rest.empty()) {
                check((traversal.position-restPosition).length()<.001f,"resting output does not move the actor along the wall");
                minimumCOM=std::min(minimumCOM,pose[4].t.z);maximumCOM=std::max(maximumCOM,pose[4].t.z);
                chestTravel=std::max(chestTravel,angleBetween(rest[26].q,pose[26].q));
                for(int bone:{28,29,31,32})if(!previous.empty()) {
                    const float change=angleBetween(previous[bone].q,pose[bone].q);armStep=std::max(armStep,change);
                    check(change<.02f,"resting output must not restart a large stopped-arm correction");
                }
                for(int hand=0;hand<2;++hand) {
                    const Vec palm=point(lib.palm(body,hand));
                    palmError=std::max(palmError,std::abs(palm.y+.8f));
                    palmTravel=std::max(palmTravel,(palm-restingPalms[hand]).length());
                    check(std::abs(palm.y+.8f)<5.f,"resting output retains each actual palm within the existing five-unit contact envelope");
                    check((palm-restingPalms[hand]).length()<1.f,"resting output keeps planted hands stable throughout the full pause");
                }
            }
            previous=pose;
        }
        check(chestTravel<.0001f&&maximumCOM-minimumCOM<.0001f,
            "settled ordinary idle has no added chest rotation or body-lift cycle");
        std::cout<<"supported idle stop fps="<<fps<<" sideways="<<sideways<<" armStep="<<armStep<<" palm="<<palmError<<
            " drift="<<palmTravel<<" chest="<<chestTravel<<" comRange="<<maximumCOM-minimumCOM<<'\n';
    }
}
int main(int argc,char** argv){try {
    check(argc>=2&&argc<=3,"motion path and optional HKX directory required");Library lib;check(lib.load(argv[1]),"load current runtime motion library");
    if(argc==3) {
        const auto overrides=fc::loadHkxOverrides(lib,argv[2]);
        check(overrides.loaded==activeMotionCount&&overrides.rejected==0&&overrides.missing==0,
            "load every active HKX slot without missing or rejected clips");
        for(Motion motion:activeMotions)check(lib.hasAnimationOverride(motion),"every active slot installs its HKX override");
    }
    capturedArmsRemainUnchanged(lib);repairedHingeHasMargin(lib);inversePoleBranch(lib);stoppedClimbing(lib);supportedIdleAfterStop(lib);
    std::cout<<"PASS: source-preserving anatomical arm guards, inverse-pole negative controls, and actual move-to-hang outputs\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
