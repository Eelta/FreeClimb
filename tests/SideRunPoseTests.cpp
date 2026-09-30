#include "AnimationOverrides.h"
#include "Pose.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
static void require(bool value,const char* message) {if(!value)throw std::runtime_error(message);}
static bool close(Vec a,Vec b,float epsilon=.0001f) {return (a-b).length()<epsilon;}
struct JointDelta {float swing{},roll{};};

static JointDelta jointDelta(Quat authored,Quat actual,Vec boneAxis) {
    boneAxis=boneAxis.unit();
    const auto delta=(authored.inverse()*actual).unit();
    const auto actualAxis=delta.rotate(boneAxis);
    const auto swing=Quat::between(boneAxis,actualAxis);
    const auto roll=(swing.inverse()*delta).unit();
    return {std::acos(std::clamp(boneAxis.dot(actualAxis),-1.f,1.f)),angleBetween({},roll)};
}
static void samePose(const Pose& a,const Pose& b,const char* message) {
    require(a.size()==b.size(),message);
    for(std::size_t i=0;i<a.size();++i)
        require(angleBetween(a[i].q,b[i].q)<.00001f&&close(a[i].t,b[i].t),message);
}
struct WholeArmAudit {float localRoll{},chestRoll{},helperRoll{};};
static WholeArmAudit wholeArmAudit(const Library& lib,const Pose& pose,const Pose& world,const Pose& restWorld,int hand) {
    const int upper=hand?31:28,elbow=hand?32:29,first=hand?58:54;
    const Vec axis=lib.rest[elbow].t;
    WholeArmAudit result;
    result.localRoll=jointDelta(lib.rest[upper].q,pose[upper].q,axis).roll;
    const auto source=(restWorld[26].q.inverse()*restWorld[upper].q).unit();
    const auto actual=(world[26].q.inverse()*world[upper].q).unit();
    result.chestRoll=jointDelta(source,actual,axis).roll;
    for(int helper:{first,first+1}) {
        const Vec helperAxis=restWorld[helper].q.inverse().rotate(restWorld[elbow].t-restWorld[upper].t);
        const auto restFrame=(restWorld[26].q.inverse()*restWorld[helper].q).unit();
        const auto frame=(world[26].q.inverse()*world[helper].q).unit();
        result.helperRoll=std::max(result.helperRoll,jointDelta(restFrame,frame,helperAxis).roll);
    }
    return result;
}
static float signedAxialRoll(Quat rotation,Vec axis) {
    rotation=rotation.unit();if(rotation.w<0)rotation={-rotation.x,-rotation.y,-rotation.z,-rotation.w};
    return 2*std::atan2(Vec{rotation.x,rotation.y,rotation.z}.dot(axis.unit()),rotation.w);
}
static void skinHelpersFollowActualArm(const Library& lib,const Pose& pose,const Pose& world,int hand) {
    const int upper=hand?31:28,elbow=hand?32:29,wrist=hand?39:38,first=hand?56:52;
    const Vec humerus=(world[elbow].t-world[upper].t).unit();
    const Vec helperLine=world[first+3].t-world[first+2].t;
    require(helperLine.unit().dot(humerus)>.999f,
        "upper-arm skin helper chain stays aligned with the actual humerus rather than inheriting an incompatible source frame");
    const Vec axis=lib.rest[wrist].t.unit();
    const float wristTwist=signedAxialRoll(pose[wrist].q*lib.rest[wrist].q.inverse(),axis);
    for(int bone:{first,first+1}) {
        const auto delta=(pose[bone].q*lib.rest[bone].q.inverse()).unit();
        const float helperTwist=signedAxialRoll(delta,axis);
        const float fraction=lib.rest[bone].t.dot(axis)/lib.rest[wrist].t.length();
        require(std::abs(helperTwist-wristTwist*fraction)<.002f&&
            jointDelta({},delta,axis).swing<.002f,
            "forearm skin helpers distribute the actual wrist axial rotation along the limb without extra swing or stale captured twist");
    }
}
static void hiddenUpperTwistNegativeControl(const Library& lib) {
    const auto source=lib.rest,sourceWorld=lib.world(source);
    for(int hand:{0,1}) {
        const int upper=hand?31:28,elbow=hand?32:29,wrist=hand?39:38;
        const auto twist=Quat::axis(lib.rest[elbow].t,2.8f);
        auto bad=source;bad[upper].q=(bad[upper].q*twist).unit();bad[elbow].q=(twist.inverse()*bad[elbow].q).unit();
        const auto world=lib.world(bad);
        require((lib.palm(world,hand)-lib.palm(sourceWorld,hand)).length()<.001f&&
            angleBetween(world[wrist].q,sourceWorld[wrist].q)<.00001f,
            "negative control preserves the palm contact and wrist world orientation despite a twisted humerus");
        require(jointDelta(bad[upper].q,bad[upper].q,lib.rest[elbow].t).roll<.00001f,
            "negative control passes the old contact-versus-already-authored shoulder test");
        const auto audit=wholeArmAudit(lib,bad,world,sourceWorld,hand);
        require(audit.localRoll>2.7f&&audit.chestRoll>2.7f&&audit.helperRoll>2.7f,
            "independent whole-arm and skin-helper checks expose hidden twist despite unchanged palm and wrist-world checks");

        auto selfReferenced=source;selfReferenced[upper].q=(source[upper].q*twist).unit();
        const auto selfWorld=lib.world(selfReferenced);const auto authored=selfReferenced;
        SideRunFrame contact;contact.innerHand=hand;contact.normal=sideRunPalmNormal(selfWorld,hand)*-1;
        const float error=applySideRunPalm(lib,selfReferenced,contact,lib.palm(selfWorld,hand),1);
        require(error<.001f&&jointDelta(authored[upper].q,selfReferenced[upper].q,lib.rest[elbow].t).roll<.00001f&&
            jointDelta(source[elbow].q,selfReferenced[elbow].q,lib.rest[wrist].t).roll<.00001f&&
            angleBetween(source[wrist].q,selfReferenced[wrist].q)<.00001f,
            "negative control reproduces a perfectly facing contact with zero reported contact/forearm/wrist correction");
        require(wholeArmAudit(lib,selfReferenced,lib.world(selfReferenced),sourceWorld,hand).localRoll>2.7f,
            "the natural-body reference rejects the twisted starting gesture even when every former local correction test is green");
    }
}
static void absoluteBraceAnatomy(const Library& lib) {
    const auto restWorld=lib.world(lib.rest);float maximumLocal=0,maximumChest=0,maximumHelper=0,sourceChest=0,fromRunning=0;
    int failed=0,cases=0;float worstSlope=0,worstPhase=0;int worstHand=0;
    for(Motion motion:{Motion::runRight,Motion::sideBrace})for(const auto& source:lib.clip(motion).frames) {
        const auto world=lib.world(source);
        for(int hand:{0,1})sourceChest=std::max(sourceChest,wholeArmAudit(lib,source,world,restWorld,hand).chestRoll);
    }
    require(sourceChest<1.0473f,"the independent sixty-degree chest-space bound contains the actual unmodified captures");
    for(float slope:{0.f,.35f,.70f})for(float side:{-1.f,1.f})for(float up:{0.f,1.f})for(int frame=0;frame<39;++frame) {
        const float phase=frame/38.f;const auto basis=sideRunFrame(slope,{side,up,0});
        auto pose=lib.sample(Motion::runRight,phase);placeSideRun(lib,pose,basis,-30);
        const auto natural=pose;applySideRunBrace(lib,pose,basis,phase);
        auto world=lib.world(pose);const int hand=basis.innerHand,upper=hand?31:28,elbow=hand?32:29;
        auto target=lib.palm(world,hand);target=target+basis.normal*(-28-target.dot(basis.normal));
        applySideRunPalm(lib,pose,basis,target,1);lib.guardArmBends(pose);lib.forearmTwist(pose);world=lib.world(pose);
        const auto audit=wholeArmAudit(lib,pose,world,restWorld,hand);
        if(audit.localRoll>maximumLocal){maximumLocal=audit.localRoll;worstSlope=slope;worstPhase=phase;worstHand=hand;}
        maximumChest=std::max(maximumChest,audit.chestRoll);maximumHelper=std::max(maximumHelper,audit.helperRoll);
        fromRunning=std::max(fromRunning,jointDelta(natural[upper].q,pose[upper].q,lib.rest[elbow].t).roll);
        if(audit.localRoll>.7855f||audit.chestRoll>1.0473f||audit.helperRoll>1.0473f)++failed;
        skinHelpersFollowActualArm(lib,pose,world,hand);++cases;
    }
    std::cout<<"absolute brace anatomy cases="<<cases<<" failures="<<failed<<" upper/chest/helper="<<maximumLocal<<'/'<<maximumChest<<'/'<<maximumHelper
        <<" sourceChest="<<sourceChest<<" totalFromRunning="<<fromRunning<<" worst hand/slope/phase="<<worstHand<<'/'<<worstSlope<<'/'<<worstPhase<<'\n';
    require(failed==0,
        "complete side brace, contact, anatomical guard and helper output must stay within 45-degree absolute upper roll and 60-degree chest-space cumulative roll");
}
struct ArmSequenceWall:World {
    Vec normal{0,-1,0};
    std::optional<Hit> ray(Vec from,Vec to) override {
        const float a=from.dot(normal),b=to.dot(normal);
        if(a<=0||b>=0)return {};
        return Hit{from+(to-from)*(a/(a-b)),normal,true};
    }
};
static void actualSurfaceWholeArms(const Library& lib) {
    const auto restWorld=lib.world(lib.rest);int frames=0,settledSideFrames=0;
    float maximumLocal=0,maximumChest=0,maximumHelper=0,maxTransitionLocal=0,maxTransitionRoll=0,maxChestStep=0;
    struct Stage {Input input;float seconds;};
    const std::array<Stage,9> stages={Stage{{0,1},.6f},
        Stage{{1,1,false,false,false,false,true},1.2f},Stage{{1,0,false,false,false,false,true},.8f},
        Stage{{-1,1,false,false,false,false,true},1.2f},Stage{{-1,0,false,false,false,false,true},.8f},
        Stage{{0,1,false,false,false,false,true},.6f},Stage{{0,1},.8f},Stage{{},.8f},
        Stage{{1,1,false,false,false,false,true},1.2f}};
    for(float slope:{0.f,.35f,.65f})for(int fps:{30,60,120}) {
        ArmSequenceWall wall;wall.normal={0,-std::sqrt(1-slope*slope),slope};
        Traversal t;t.cfg.approachSeconds=0;t.cfg.gap=37;t.cfg.radius=31;t.cfg.height=138;
        require(t.attach(wall,{0,-37,0},{0,1,0},100),"whole-arm runtime sequence attaches to the real sloped plane");
        SurfacePose animator;Pose last;const float dt=1.f/fps;
        auto tick=[&](Input input,bool settledSide) {
            const auto result=t.update(wall,input,dt,100);
            require(t.active()&&!result.released,"arm adaptation cannot release physical wall support on a clear slope");
            const auto pose=animator.update(lib,wall,t,result.motion,dt,1);const auto world=lib.world(pose);
            for(const auto& bone:pose)require(bone.t.finite()&&std::isfinite(bone.q.dot(bone.q))&&std::abs(bone.q.dot(bone.q)-1)<.001f,
                "real arm switches retain finite normalized body and helper transforms");
            const auto previousWorld=last.empty()?Pose{}:lib.world(last);
            for(int hand:{0,1}) {
                const auto audit=wholeArmAudit(lib,pose,world,restWorld,hand);
                maxTransitionLocal=std::max(maxTransitionLocal,audit.localRoll);
                maxTransitionRoll=std::max(maxTransitionRoll,audit.chestRoll);

                require(audit.localRoll<1.3091f&&audit.chestRoll<1.2218f&&audit.helperRoll<1.2218f,
                    "actual climb/run transitions retain their captured arm range without an intermediate shoulder or skin-helper half-turn");
                skinHelpersFollowActualArm(lib,pose,world,hand);
                require(lib.armBendValid(pose,hand),"actual Surface contact/blend/guard sequence never returns a reversed elbow");
                if(!last.empty()) {
                    const int upper=hand?31:28;
                    const auto old=(previousWorld[26].q.inverse()*previousWorld[upper].q).unit();
                    const auto now=(world[26].q.inverse()*world[upper].q).unit();
                    const float step=angleBetween(old,now);maxChestStep=std::max(maxChestStep,step/dt);

                    require(step<=2*18.849556f*dt+.002f,
                        "combined shoulder motion cannot jump farther than the sum of its real joint budgets");
                }
            }
            if(settledSide&&result.motion>=Motion::runLeft&&result.motion<=Motion::runDiagonalRight) {
                const int hand=input.x>0?0:1;const auto audit=wholeArmAudit(lib,pose,world,restWorld,hand);
                maximumLocal=std::max(maximumLocal,audit.localRoll);maximumChest=std::max(maximumChest,audit.chestRoll);
                maximumHelper=std::max(maximumHelper,audit.helperRoll);++settledSideFrames;
                if(audit.localRoll>.7855f||audit.chestRoll>1.0473f||audit.helperRoll>1.0473f)
                    std::cerr<<"runtime whole arm failed fps="<<fps<<" slope="<<slope<<" direction="<<input.x<<','<<input.y
                        <<" local/chest/helper="<<audit.localRoll<<'/'<<audit.chestRoll<<'/'<<audit.helperRoll<<'\n';
                require(audit.localRoll<=.7855f&&audit.chestRoll<=1.0473f&&audit.helperRoll<=1.0473f,
                    "actual sustained lateral/diagonal output retains the complete natural upper-arm frame after Surface IK and smoothing");
            }
            if(frames%31==0) {
                const auto held=animator.update(lib,wall,t,result.motion,0,1);
                samePose(held,pose,"zero-time Surface callbacks cannot accumulate shoulder twist or reapply helper corrections");
            }
            last=pose;++frames;
        };
        for(const auto& stage:stages)for(int frame=0;frame<int(std::lround(stage.seconds*fps));++frame)
            tick(stage.input,frame*dt>=.5f&&stage.input.run&&std::abs(stage.input.x)>.1f);

        for(int switchIndex=0;switchIndex<16;++switchIndex) {
            const Input input=switchIndex%4==0?Input{1,1,false,false,false,false,true}:
                switchIndex%4==1?Input{-1,1,false,false,false,false,true}:
                switchIndex%4==2?Input{-1,0}:Input{};
            for(int frame=0;frame<std::max(1,fps/12);++frame)tick(input,false);
        }
        for(int frame=0;frame<fps*2;++frame)tick({-1,1,false,false,false,false,true},frame>fps/2);
    }
    require(settledSideFrames>800,"whole-arm test actually consumes many settled runtime side poses across directions and slopes");
    std::cout<<"actual whole-arm frames="<<frames<<" settledSide="<<settledSideFrames<<" upper/chest/helper="
        <<maximumLocal<<'/'<<maximumChest<<'/'<<maximumHelper<<" all-transition local/chest="<<maxTransitionLocal<<'/'<<maxTransitionRoll<<" chestSpeed="<<maxChestStep<<'\n';
}
int main(int argc,char** argv) {
    try {
        for(float z:{-.15f,0.f,.35f,.70f})for(float side:{-1.f,1.f})for(float up:{0.f,1.f}) {
            const auto f=sideRunFrame(z,{side,up,0});
            require(f.forward.finite()&&f.up.finite()&&f.right.finite(),"side-run basis finite");
            require(std::abs(f.forward.dot(f.normal))<.0001f,"heading tangent to real wall");
            require(std::abs(f.up.dot(f.forward))<.0001f&&std::abs(f.right.dot(f.up))<.0001f,"side-run frame orthonormal");
            require(close(f.rotation.rotate({0,1,0}),f.forward)&&close(f.rotation.rotate({0,0,1}),f.up),"native forward/up basis preserved");
            require(f.up.z>.50f,"banked runner keeps head above pelvis");
            const auto bank=std::asin(std::clamp(f.up.dot(f.normal),-1.f,1.f));
            require(bank>.47f&&bank<.51f,"visible outward bank stays about 28 degrees, including diagonal motion");
            require(f.innerHand==(side>0?0:1)&&f.right.cross(f.forward).dot(f.up)>.9999f,"inner hand and handedness preserved");
            if(z==0&&up>0) {
                require(f.up.z>.84f&&std::abs(f.forward.z)<.25f,
                    "diagonal travel cannot recline the complete torso by 45 degrees");
                require(f.travel.z>.70f&&f.travel.x*side>.70f,
                    "actual diagonal foot travel retains the requested 45-degree heading");
            }
        }
        for(Vec direction:{Vec{},Vec{0,1,0},Vec{0,-1,0}})
            require(sideRunFrame(0,direction).rotation.rotate({0,0,1}).finite(),"degenerate direction finite");

        for(Vec axis:{Vec{0,0,1},Vec{.2f,.5f,1}.unit()})for(float roll:{-3.f,-1.f,1.f,3.f})for(float swing:{-.9f,.9f,2.f}) {
            const auto authored=Quat::axis({1,2,3},.73f);
            const auto perpendicular=axis.cross({1,0,0}).unit();
            const auto desired=authored*Quat::axis(perpendicular,swing)*Quat::axis(axis,roll);
            const auto bounded=boundedSideRunJoint(authored,desired,axis,.61086524f,.20943951f);
            const auto measured=jointDelta(authored,bounded,axis);
            require(measured.swing<.611f&&measured.roll<.210f,"independent shoulder/forearm swing and roll limits");
        }
        Library lib;require(argc>1&&lib.load(argv[1]),"load actual bundled library");
        if(argc==3) {
            const auto overrides=fc::loadHkxOverrides(lib,argv[2]);
            require(overrides.loaded==activeMotionCount&&overrides.rejected==0&&overrides.missing==0,
                "load every active HKX slot without missing or rejected clips");
            for(Motion motion:activeMotions)require(lib.hasAnimationOverride(motion),"every active slot installs its HKX override");
        }
        hiddenUpperTwistNegativeControl(lib);absoluteBraceAnatomy(lib);actualSurfaceWholeArms(lib);
        const auto hang=lib.world(lib.sample(Motion::ledgeCatch,.25f));
        for(int hand=0;hand<2;++hand)
            require(sideRunPalmNormal(hang,hand).dot({0,1,0})>.85f,"anatomical palm sign independently matches captured contact");
        float maxShoulderSwing=0,maxShoulderRoll=0,maxElbowSwing=0,maxElbowRoll=0,maxWrist=0,maxCorrection=0;
        int changed=0,unchanged=0;float minDistance=1000,maxFacing=-1;
        for(Motion clip:{Motion::runRight,Motion::runLaunchLeft,Motion::runLaunchRight})
        for(float z:{0.f,.35f,.70f})for(float side:{-1.f,1.f})for(float up:{0.f,1.f})for(int sample=0;sample<39;++sample) {
            const auto frame=sideRunFrame(z,{side,up,0});
            auto pose=lib.sample(clip,sample/38.f);
            const auto authored=pose;
            const float plane=-30*std::sqrt(1-z*z);
            placeSideRun(lib,pose,frame,plane);
            const auto world=lib.world(pose);
            require(std::abs(world[4].t.dot(frame.normal)-plane-24)<.001f,"pelvis is positioned from real wall plane");
            for(int bone:{6,7,8,9,10,11,50,51})
                require(angleBetween(pose[bone].q,authored[bone].q)<.00001f&&close(pose[bone].t,authored[bone].t),"frame preserves source leg gait");
            Vec target=sideRunPalmProbe(lib,pose,frame);
            require(close(target,lib.palm(world,frame.innerHand)),"probe follows actual source hand rather than hanging target");
            target=target+frame.normal*(plane+2-target.dot(frame.normal));
            const int upper=frame.innerHand==0?28:31,elbow=frame.innerHand==0?29:32,wrist=frame.innerHand==0?38:39;
            const auto before=pose;
            const float beforeError=(target-lib.palm(world,frame.innerHand)).length();
            const float facing=sideRunPalmNormal(world,frame.innerHand).dot(frame.normal*-1);
            minDistance=std::min(minDistance,beforeError);maxFacing=std::max(maxFacing,facing);
            const float error=applySideRunPalm(lib,pose,frame,target,1);
            const auto solved=lib.world(pose);
            require(std::isfinite(error)&&error<=beforeError+.011f,"contact never worsens source hand reach");
            const auto shoulderChange=jointDelta(before[upper].q,pose[upper].q,lib.rest[elbow].t);
            const auto elbowChange=jointDelta(before[elbow].q,pose[elbow].q,lib.rest[wrist].t);
            maxShoulderSwing=std::max(maxShoulderSwing,shoulderChange.swing);maxShoulderRoll=std::max(maxShoulderRoll,shoulderChange.roll);
            maxElbowSwing=std::max(maxElbowSwing,elbowChange.swing);maxElbowRoll=std::max(maxElbowRoll,elbowChange.roll);
            maxWrist=std::max(maxWrist,angleBetween(before[wrist].q,pose[wrist].q));
            const auto correction=(lib.palm(solved,frame.innerHand)-lib.palm(world,frame.innerHand)).length();
            maxCorrection=std::max(maxCorrection,correction);
            if(correction>.001f)++changed;else ++unchanged;
            require(shoulderChange.swing<.611f&&elbowChange.swing<.611f,"neither shoulder nor elbow exceeds 35 degree source adjustment");
            require(shoulderChange.roll<.210f&&elbowChange.roll<.210f,"neither upper arm nor forearm exceeds 12 degree source roll adjustment");
            const float a=(world[elbow].t-world[upper].t).length(),b=(world[wrist].t-world[elbow].t).length();
            require(std::abs((solved[elbow].t-solved[upper].t).length()-a)<.001f&&std::abs((solved[wrist].t-solved[elbow].t).length()-b)<.001f,"fixed arm lengths");
            for(std::size_t bone=0;bone<pose.size();++bone)if(int(bone)!=upper&&int(bone)!=elbow)
                require(angleBetween(pose[bone].q,before[bone].q)<.00001f&&close(pose[bone].t,before[bone].t),"preserve captured wrist/fingers, outer arm and every unrelated bone");
            const auto valid=pose;
            applySideRunPalm(lib,pose,frame,target+frame.normal*300,1);
            samePose(pose,valid,"out of reach target preserves natural animation");
            pose=before;applySideRunPalm(lib,pose,frame,target,0);
            samePose(pose,before,"zero contact weight preserves source exactly");
        }
        std::cout<<"side bounded contact: shoulder swing/roll="<<maxShoulderSwing<<'/'<<maxShoulderRoll
            <<" forearm swing/roll="<<maxElbowSwing<<'/'<<maxElbowRoll<<" wrist deviation="<<maxWrist
            <<" max palm correction="<<maxCorrection<<" adapted="<<changed<<" source-only="<<unchanged<<" minDistance="<<minDistance<<" maxFacing="<<maxFacing<<'\n';

        int validAdjustments=0;
        for(int hand:{0,1})for(int sample=0;sample<39;++sample) {
            auto p=lib.sample(Motion::runRight,sample/38.f);const auto before=p,world=lib.world(p);
            SideRunFrame frame;frame.innerHand=hand;frame.normal=sideRunPalmNormal(world,hand)*-1;
            const auto target=lib.palm(world,hand)-frame.normal*4.f;
            const float error=applySideRunPalm(lib,p,frame,target,1);
            const int upper=hand==0?28:31,elbow=hand==0?29:32,wrist=hand==0?38:39;
            const auto shoulderChange=jointDelta(before[upper].q,p[upper].q,lib.rest[elbow].t);
            const auto forearmChange=jointDelta(before[elbow].q,p[elbow].q,lib.rest[wrist].t);
            require(error<=4.01f&&shoulderChange.swing<.611f&&forearmChange.swing<.611f,"facing palm receives only bounded reach correction");
            require(shoulderChange.roll<.210f&&forearmChange.roll<.210f,"valid contact cannot hide axial arm roll");
            require(angleBetween(p[wrist].q,before[wrist].q)<.00001f,"valid contact retains source wrist anatomy");
            if(error<3.9f)++validAdjustments;

            frame.normal=frame.normal*-1;p=before;
            applySideRunPalm(lib,p,frame,target,1);
            samePose(p,before,"back-facing hand is preserved instead of rotated onto wall");
        }
        std::cout<<"captured-palm fixture adapted="<<validAdjustments<<"/78\n";
        require(validAdjustments>10,"valid source-aligned contact actually performs positional IK");
        require(unchanged>100,"unreachable or unsuitable frames are not forced into a hanging pose");
        require(maxWrist<.00001f,"no procedural wrist bend");
        float braceError=0,braceFacing=1,braceSwing=0,braceRoll=0,referenceForearmRoll=0;
        int braceContacts=0;
        for(float z:{0.f,.35f,.70f})for(float side:{-1.f,1.f})for(float up:{0.f,1.f})
        for(int sample=0;sample<39;++sample) {
            const float phase=sample/38.f;const auto frame=sideRunFrame(z,{side,up,0});
            auto p=lib.sample(Motion::runRight,phase);placeSideRun(lib,p,frame,-30);
            const auto runningPose=p;
            applySideRunBrace(lib,p,frame,phase,0);
            samePose(p,runningPose,"zero brace influence retains the original running pose");
            applySideRunBrace(lib,p,frame,phase);
            const auto gesture=p,world=lib.world(p);
            const int hand=frame.innerHand,upper=hand==0?28:31,elbow=hand==0?29:32,wrist=hand==0?38:39,base=hand==0?67:82;
            const auto reference=lib.sample(Motion::sideBrace,.5f+.10f*std::sin(phase*6.283185307f));
            require(angleBetween(p[wrist].q,reference[wrist].q)<.00001f,
                "authored side gesture preserves its captured wall-contact wrist without wall-normal twisting");
            for(int finger=base;finger<base+15;++finger)
                require(angleBetween(p[finger].q,reference[finger].q)<.00001f,
                    "support gesture preserves captured finger anatomy");
            const int outerUpper=hand==0?31:28,outerElbow=hand==0?32:29,outerWrist=hand==0?39:38;
            for(int bone:{outerUpper,outerElbow,outerWrist,7,8,10,11,50,51})
                require(angleBetween(p[bone].q,runningPose[bone].q)<.00001f,
                    "free arm swing and source knee/ankle articulation remain in the running cycle");
            auto target=lib.palm(world,hand);target=target+frame.normal*(-28-target.dot(frame.normal));
            const float error=applySideRunPalm(lib,p,frame,target,1);const auto solved=lib.world(p);
            require(std::abs(error-(target-lib.palm(solved,hand)).length())<.011f,
                "support contact reports its actual residual even when anatomical constraints leave the palm short of the wall");
            const auto shoulder=jointDelta(gesture[upper].q,p[upper].q,lib.rest[elbow].t);
            const auto forearm=jointDelta(gesture[elbow].q,p[elbow].q,lib.rest[wrist].t);
            referenceForearmRoll=std::max(referenceForearmRoll,jointDelta(reference[elbow].q,p[elbow].q,lib.rest[wrist].t).roll);
            braceSwing=std::max({braceSwing,shoulder.swing,forearm.swing});braceRoll=std::max({braceRoll,shoulder.roll,forearm.roll});
            braceError=std::max(braceError,error);braceFacing=std::min(braceFacing,sideRunPalmNormal(solved,hand).dot(frame.normal*-1));
            require(shoulder.swing<.611f&&forearm.swing<.611f&&shoulder.roll<.210f&&forearm.roll<.210f,
                "actual contact is bounded against the natural wall-brace gesture independently for swing and roll");
            require(angleBetween(p[wrist].q,gesture[wrist].q)<.00001f,
                "contact correction cannot twist the reference wrist");
            const Vec wallUp=(Vec{0,0,1}-frame.normal*frame.normal.z).unit();
            require((solved[elbow].t-solved[upper].t).dot(wallUp)<-3,
                "side support keeps its elbow naturally below the shoulder rather than an overhead reach");
            const float bend=std::acos(std::clamp((solved[upper].t-solved[elbow].t).unit().dot((solved[wrist].t-solved[elbow].t).unit()),-1.f,1.f));
            require(bend>.35f&&bend<2.8f,"supporting elbow remains bent away from either singular straight/reversed limit");
            if(error<5&&sideRunPalmNormal(solved,hand).dot(frame.normal*-1)>.70f)++braceContacts;
        }
        std::cout<<"authored side brace: max palm error="<<braceError<<" min facing="<<braceFacing
            <<" contact frames="<<braceContacts<<" max swing/roll="<<braceSwing<<'/'<<braceRoll
            <<" forearm roll from captured reference="<<referenceForearmRoll<<'\n';

        require(std::isfinite(braceError)&&braceError>=0&&std::isfinite(braceFacing),
            "rejected cosmetic contact still has finite, honest geometry diagnostics");
        require(referenceForearmRoll<.211f,"authored gesture and contact together retain the captured forearm axial limit");
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
