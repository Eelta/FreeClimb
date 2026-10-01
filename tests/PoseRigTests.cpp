#include "Pose.h"
#include "PoseHandoff.h"
#include "CanonicalSkeleton.h"
#include <bit>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace fc;
static unsigned checks{};
static void check(bool ok,const char* message){++checks;if(!ok)throw std::runtime_error(message);}
static bool exact(float a,float b){return std::bit_cast<std::uint32_t>(a)==std::bit_cast<std::uint32_t>(b);}
static bool exact(Vec a,Vec b){return exact(a.x,b.x)&&exact(a.y,b.y)&&exact(a.z,b.z);}
static bool exact(Quat a,Quat b){return exact(a.x,b.x)&&exact(a.y,b.y)&&exact(a.z,b.z)&&exact(a.w,b.w);}
static bool exact(const Transform& a,const Transform& b){return exact(a.t,b.t)&&exact(a.q,b.q)&&exact(a.s,b.s);}
static bool exact(const Pose& a,const Pose& b){if(a.size()!=b.size())return false;for(std::size_t i=0;i<a.size();++i)if(!exact(a[i],b[i]))return false;return true;}
static bool near(Vec a,Vec b,float tolerance=.0003f){return (a-b).length()<tolerance;}
static bool near(const Transform& a,const Transform& b,float tolerance=.0003f){return near(a.t,b.t,tolerance)&&near(a.s,b.s,tolerance)&&angleBetween(a.q,b.q)<tolerance;}
static Pose canonical(){Pose result;for(const auto& v:canonicalBoneRest)result.push_back({{v[0],v[1],v[2]},{v[3],v[4],v[5],v[6]},{v[7],v[8],v[9]}});return result;}
struct Fixture {
    Pose source=canonical(),native=source;
    std::array<Transform,99> basis{};
    std::array<bool,99> mapped{};
    Fixture(){mapped.fill(true);for(int bone:{1,2,3,42,43,60,61,62,63,64,65,66,97,98})mapped[bone]=false;}
    void customize(){
        for(int bone:{7,8,28,29,31,32,38,39,68,83})native[bone].t=source[bone].t*1.23f+Vec{.3f,-.2f,.4f};
        native[7].s={1.11f,.93f,1.07f};native[29].s={.89f,1.15f,1.04f};native[38].s={1.04f,1.04f,1.04f};
        basis[7]={{2,-3,4},Quat::axis({1,2,3},.37f),{1.07f,.94f,1.13f}};
        basis[29]={{-.4f,.2f,1},Quat::axis({2,-1,3},-.22f),{.97f,1.08f,1.02f}};
    }
};
static Transform expectedAdapt(const Fixture& f,std::size_t bone,Transform value){
    if(!f.mapped[bone])return value;
    if(bone!=0&&bone!=4)value.t=f.native[bone].t+(value.t-f.source[bone].t);
    value.s={f.native[bone].s.x*(value.s.x/f.source[bone].s.x),f.native[bone].s.y*(value.s.y/f.source[bone].s.y),f.native[bone].s.z*(value.s.z/f.source[bone].s.z)};
    return compose(f.basis[bone],value);
}
static void inactivePreservesBits(){
    Fixture f;PoseRig<Pose> rig;
    for(std::size_t i=0;i<99;++i)f.native[i].q=Quat::axis({1,2,3},float(i)*.031f);
    f.native[0].t={17,23,-41};f.native[4].t={-12,41,91};
    check(rig.configure(f.source,f.native,f.basis,f.mapped)&&!rig.active(),"native animation rotations and root/COM motion alone do not activate structural retargeting");
    check(exact(rig.source(),f.source)&&exact(rig.reference(),f.source),"inactive canonical reference preserves every float bit");
    Pose input=f.source;input[7].t.x=-0.f;input[8].q={.2f,.3f,.4f,.5f};input[9].s.y=-0.f;
    const Pose original=input;rig.adapt(input);check(exact(input,original),"inactive adaptation does not normalize, rescale or rewrite signed zero");
    for(std::size_t i=0;i<=100;++i){const auto& local=original[i%99];check(exact(rig.toEffective(i,local),local)&&exact(rig.toLocal(i,local),local),"inactive coordinate conversion preserves all input bits");check(exact(rig.rotation(i,local.q),local.q),"inactive rotation conversion preserves quaternion bits");}
    for(int bone:{42,43,60,61,62,63,64,65,66,97,98}){f.native[bone].t={123,456,789};f.native[bone].s={2,3,4};f.basis[bone]={{7,8,9},Quat::axis({1,0,0},1),{2,2,2}};}
    check(rig.configure(f.source,f.native,f.basis,f.mapped)&&!rig.active(),"engine-owned equipment and camera differences do not activate or authorize adaptation");
    check(exact(rig.reference(),f.source),"unmapped equipment and cameras retain source tracks");
}
static void sceneRotationValidation(){
    using Rig=PoseRig<Pose>;
    for(int i=0;i<360;++i){
        const auto q=Quat::axis({1,2,3},float(i)*.01745329252f);
        check(Rig::validAxes(q.rotate({1,0,0}),q.rotate({0,1,0}),q.rotate({0,0,1})),"valid animated scene rotation remains accepted");
    }
    check(!Rig::validAxes(Vec{},Vec{},Vec{}),"zero matrix cannot normalize into an accepted scene rotation");
    check(!Rig::validAxes(Vec{1,0,0},Vec{1,0,0},Vec{0,0,1}),"parallel scene axes rejected");
    check(!Rig::validAxes(Vec{1,0,0},Vec{0,1,0},Vec{0,0,-1}),"reflection cannot be interpreted as a bone quaternion");
    check(!Rig::validAxes(Vec{2,0,0},Vec{0,1,0},Vec{0,0,1}),"matrix scale is not silently normalized");
    check(!Rig::validAxes(Vec{std::numeric_limits<float>::quiet_NaN(),0,0},Vec{0,1,0},Vec{0,0,1}),"nonfinite scene axes rejected");
}
static void structuralCapture(){
    const auto reference=canonical();
    for(std::size_t bone=0;bone<99;++bone)for(bool locked:{false,true}){
        Transform live{{13,-17,29},Quat::axis({1,2,3},.42f),{1.1f,.92f,1.18f}};
        const auto captured=PoseRig<Pose>::structuralReference(bone,live,reference[bone],locked);
        check(exact(captured.q,live.q)&&exact(captured.s,live.s),"structural capture retains native rotation and scale");
        check(exact(captured.t,(bone==0||bone==4||!locked)?reference[bone].t:live.t),"only locked non-root/COM translations define captured bone structure");
    }
}
static void animatedLocalsKeepStableStructure(){
    Fixture f;f.customize();PoseRig<Pose> rig;
    check(rig.configure(f.source,f.native,f.basis,f.mapped),"temporal preflight fixture captures customized proportions");
    const auto captured=f.source,proportions=f.native,source=rig.source(),effective=rig.reference();
    const auto bases=f.basis;
    unsigned callbacks=0;bool changedTranslation=false,changedBridge=false;
    for(int fps:{30,60,120})for(int frame=0;frame<fps*2;++frame)for(int phase:{0,1}) {
        const float time=(frame+phase*.5f)/fps;
        for(std::size_t bone=0;bone<99;++bone)if(f.mapped[bone]) {
            auto live=f.native[bone];
            live.t=live.t+Vec{std::sin(time*5+float(bone))*.23f,std::cos(time*3)*.19f,std::sin(time*7)*.31f};
            live.q=(Quat::axis({1,2,-1},time*.37f+phase*.021f)*live.q).unit();
            live.s=live.s*(1+std::sin(time*2)*.015f);
            check(PoseRig<Pose>::currentStructure(live,f.source[bone],captured[bone]),"native animation and IK between main update and scene callback preserve preflight validity");
            if(bone!=0&&bone!=4)changedTranslation|=(live.t-proportions[bone].t).length()>.002f;
            auto bridge=f.basis[bone];
            bridge.t=bridge.t+Vec{std::sin(time*4)*.12f,0,.03f};
            bridge.q=(Quat::axis({0,1,0},time*.11f+phase*.004f)*bridge.q).unit();
            bridge.s=bridge.s*(1+std::cos(time)*.012f);
            check(PoseRig<Pose>::validLocal(bridge),"animated intermediate adjustment nodes remain valid scene transforms");
            changedBridge|=angleBetween(bridge.q,bases[bone].q)>.0005f;
        }
        if(phase==1)++callbacks;
        check(exact(rig.source(),source)&&exact(rig.reference(),effective),"repeated scene validation never recaptures proportions from transient animated locals");
    }
    check(callbacks==420&&changedTranslation&&changedBridge,"preflight regression covers repeated callbacks beyond former translation and bridge-rotation rejection thresholds");
    check(exact(f.native,proportions)&&exact(f.source,captured),"validation leaves captured native proportions and hka references unchanged");
    for(std::size_t bone=0;bone<99;++bone)check(exact(f.basis[bone],bases[bone]),"validation preserves captured adjustment bases");
}
static void structuralValidationRejectsInvalidData(){
    const auto captured=canonical()[7];
    check(PoseRig<Pose>::validLocal(captured)&&PoseRig<Pose>::currentStructure(captured,captured,captured),"unchanged valid reference permits a scene callback");
    for(int kind=0;kind<11;++kind) {
        auto invalid=captured;
        if(kind==0)invalid.q={0,0,0,0};
        if(kind==1)invalid.q.x=std::numeric_limits<float>::quiet_NaN();
        if(kind==2)invalid.q.w=std::numeric_limits<float>::infinity();
        if(kind==3)invalid.q={0,0,0,.5f};
        if(kind==4)invalid.t.x=std::numeric_limits<float>::quiet_NaN();
        if(kind==5)invalid.t.y=std::numeric_limits<float>::infinity();
        if(kind==6)invalid.t={1000,0,0};
        if(kind==7)invalid.s.x=0;
        if(kind==8)invalid.s.y=5.001f;
        if(kind==9)invalid.s.z=std::numeric_limits<float>::quiet_NaN();
        if(kind==10)invalid.s.x=.099f;
        check(!PoseRig<Pose>::validLocal(invalid),"unsafe live or intermediate-node transform is rejected");
        check(!PoseRig<Pose>::currentStructure(invalid,captured,captured),"transient animation allowance never accepts invalid scene output");
        check(!PoseRig<Pose>::currentStructure(captured,invalid,invalid),"even an unchanged invalid hka reference is rejected");
    }
    for(int kind=0;kind<3;++kind) {
        auto reference=captured;
        if(kind==0)reference.t.x+=.0001f;
        if(kind==1)reference.q=(Quat::axis({1,0,0},.002f)*reference.q).unit();
        if(kind==2)reference.s.y+=.0001f;
        check(PoseRig<Pose>::validLocal(reference),"changed-reference fixture remains a valid transform");
        check(!PoseRig<Pose>::currentStructure(captured,reference,captured),"actual hka translation rotation or scale replacement invalidates the captured rig");
    }
}
static void adaptationAndBridges(){
    Fixture f;f.customize();f.source[29].s={.8f,1.4f,2};
    for(int bone:{42,43,60,61,62,63,64,65,66,97,98}){f.native[bone].t={111,-222,333};f.native[bone].s={3,2,4};f.basis[bone]={{9,8,7},Quat::axis({0,1,0},.83f),{2,2,2}};}
    f.native[0].t={80,-70,60};f.native[4].t={40,-30,120};f.native[0].s={1.04f,1.04f,1.04f};
    f.native[4].s={.96f,.96f,.96f};f.basis[4]={{1,2,3},Quat::axis({0,0,1},.13f),{1,1,1}};
    PoseRig<Pose> rig;check(rig.configure(f.source,f.native,f.basis,f.mapped)&&rig.active(),"custom locked lengths, scales and adjustment bridges activate adaptation");
    const auto reference=rig.reference();
    for(std::size_t i=0;i<99;++i)check(near(reference[i],expectedAdapt(f,i,f.source[i])),"effective rest includes the same actual structure and bridges as output");
    Pose authored=f.source;
    for(std::size_t i=0;i<99;++i){authored[i].q=Quat::axis({1,-2,3},float(i)*.017f);authored[i].s=authored[i].s*1.02f;}
    authored[0].t={-7,11,23};authored[4].t=authored[4].t+Vec{2,-5,8};authored[29].t=authored[29].t+Vec{.7f,-.3f,.2f};
    const Pose original=authored;rig.adapt(authored);
    for(std::size_t i=0;i<99;++i){
        check(near(authored[i],expectedAdapt(f,i,original[i])),"adapted output retains native structure plus authored local displacement");
        check(near(rig.toLocal(i,authored[i]).t,i==0||i==4||!f.mapped[i]?original[i].t:f.native[i].t+(original[i].t-f.source[i].t)),"inverse basis never reapplies or removes native length adaptation");
        const auto effective=rig.toEffective(i,f.native[i]);
        check(near(rig.toLocal(i,effective),f.native[i]),"native effective/local roundtrip preserves translation rotation and nonuniform scale");
        check(near(effective,f.mapped[i]?compose(f.basis[i],f.native[i]):f.native[i]),"native conversion composes only bridge basis and does not remap animated translation");
        const auto wanted=f.mapped[i]?(f.basis[i].q*original[i].q).unit():original[i].q;
        check(angleBetween(rig.rotation(i,original[i].q),wanted)<.0003f,"rotation override is expressed in the effective bridge frame");
        if(!f.mapped[i])check(exact(authored[i],original[i]),"unmapped helpers, equipment and camera tracks remain byte identical");
    }
    check(near(rig.toLocal(0,authored[0]).t,original[0].t)&&near(rig.toLocal(4,authored[4]).t,original[4].t),"authored root and COM displacement is preserved despite animated native positions");
    const Transform parent{{4,-8,12},Quat::axis({2,3,-1},.71f),{.7f,1.6f,1.2f}},child{{3,-5,7},Quat::axis({-1,4,2},-.53f),{1.2f,.8f,1.05f}};
    check(near(PoseRig<Pose>::relative(parent,compose(parent,child)),child),"relative transform inverts translated rotated nonuniform bridge composition");
    Pose shortPose(98);shortPose[1]=child;const auto shortBefore=shortPose;rig.adapt(shortPose);check(exact(shortPose,shortBefore),"wrong-sized output is untouched");
    check(exact(rig.toEffective(99,child),child)&&exact(rig.toLocal(99,child),child)&&exact(rig.rotation(99,child.q),child.q),"out-of-range conversion cannot access a bridge");
    rig.clear();check(!rig.active()&&rig.source().empty()&&rig.reference().empty(),"clear retires stored structure and reference");
    authored=original;rig.adapt(authored);check(exact(authored,original),"cleared rig immediately returns to exact pass-through");
}
static void invalidConfigurationIsTransactional(){
    Fixture f;f.customize();PoseRig<Pose> rig;check(rig.configure(f.source,f.native,f.basis,f.mapped),"valid transaction baseline");
    const auto original=rig.source(),reference=rig.reference();const auto converted=rig.toEffective(7,f.native[7]);
    auto rejected=[&](auto mutate){auto source=f.source,native=f.native;auto basis=f.basis;auto mapped=f.mapped;source[6].t.x+=2;mutate(source,native,basis,mapped);check(!rig.configure(source,native,basis,mapped),"malformed replacement is rejected");check(rig.active()&&exact(rig.source(),original)&&exact(rig.reference(),reference)&&exact(rig.toEffective(7,f.native[7]),converted),"rejected replacement preserves every prior mapping and effective transform");};
    rejected([](auto& s,auto&,auto&,auto&){s.pop_back();});
    rejected([](auto&,auto& n,auto&,auto&){n.push_back({});});
    for(int domain:{0,1,2})for(int kind=0;kind<9;++kind)rejected([=](auto& s,auto& n,auto& b,auto&){
        auto& t=domain==0?s[7]:domain==1?n[7]:b[7];
        if(kind==0)t.t.x=std::numeric_limits<float>::quiet_NaN();
        if(kind==1)t.t={1000,0,0};
        if(kind==2)t.q={0,0,0,0};
        if(kind==3)t.q.w=std::numeric_limits<float>::infinity();
        if(kind==4)t.q={0,0,0,.5f};
        if(kind==5)t.s.x=.099f;
        if(kind==6)t.s.y=5.001f;
        if(kind==7)t.s.z=std::numeric_limits<float>::quiet_NaN();
        if(kind==8)t.t.y=std::numeric_limits<float>::infinity();
    });
    rejected([](auto& s,auto&,auto&,auto& mapped){mapped[97]=false;s[97].s.z=0;});
    PoseRig<Pose> empty;auto invalid=f.native;invalid[7].s.x=0;check(!empty.configure(f.source,invalid,f.basis,f.mapped)&&!empty.active()&&empty.source().empty(),"first invalid configuration cannot leave a partial rig");
}
static Library libraryFixture(){
    Library library;library.rest=canonical();library.parents.assign(canonicalBoneParents.begin(),canonicalBoneParents.end());for(auto name:canonicalBoneNames)library.names.emplace_back(name);
    Pose a=library.rest,b=a;a[0].t={1,2,3};b[0].t={4,6,8};b[4].t=b[4].t+Vec{2,-1,5};
    for(std::size_t i=5;i<97;++i)b[i].q=(Quat::axis({1,2,-1},.15f)*b[i].q).unit();
    for(auto& clip:library.clips){clip.seconds=1;clip.frames={a,b};clip.contacts={{{1,1,1,1}},{{1,1,1,1}}};clip.travel={90,0,0};clip.height=80;}
    library.calibrateArmBends();return library;
}
static void libraryRestSamplingAndIK(){
    auto library=libraryFixture();const auto original=library;Fixture f;f.customize();
    check(library.configureRig(f.native,f.basis,f.mapped)&&library.rig.active(),"library installs valid actual structural rig");
    for(std::size_t i=0;i<99;++i)check(near(library.rest[i],expectedAdapt(f,i,f.source[i])),"library rest uses effective structure for all later IK and joint references");
    for(float phase:{0.f,.13f,.5f,.91f,1.f}){
        const auto authored=original.sample(Motion::hang,phase),sample=library.sample(Motion::hang,phase);
        for(std::size_t i=0;i<99;++i)check(near(sample[i],expectedAdapt(f,i,authored[i])),"library samples adapt exactly once from immutable authored clips");
    }
    check(exact(library.clip(Motion::hang).frames[0],original.clip(Motion::hang).frames[0])&&exact(library.clip(Motion::hang).frames[1],original.clip(Motion::hang).frames[1]),"rig setup never rewrites source clip frames");
    const auto once=library.rest;check(library.configureRig(f.native,f.basis,f.mapped)&&exact(library.rest,once),"repeated configure references original rest rather than compounding structural adaptation");
    auto invalid=f.native;invalid[7].s.y=0;check(!library.configureRig(invalid,f.basis,f.mapped)&&exact(library.rest,once),"library rejects invalid rig without replacing rest");
    auto& replacement=library.rotationOverrides[int(Motion::hang)-1];replacement.frames.resize(2);replacement.bones[7]=true;replacement.frames[0][7]=Quat::axis({0,1,0},.17f);replacement.frames[1][7]=Quat::axis({1,0,0},.41f);
    const auto overridden=library.sample(Motion::hang,.4f);check(angleBetween(overridden[7].q,(f.basis[7].q*blend(replacement.frames[0][7],replacement.frames[1][7],.4f)).unit())<.0003f,"replacement rotations retain effective intermediate-node basis");
    Settings cfg;library.configureThreepeat(cfg);const auto hangWorld=library.world(library.sampleBase(Motion::contextHang,0)),mantleWorld=library.world(library.sampleBase(Motion::contextMantle,0));
    const auto left=library.palm(hangWorld,0),right=library.palm(hangWorld,1);
    check(std::abs(cfg.threepeatHangHeight-(left.z+right.z)*.5f)<.0003f&&std::abs(cfg.threepeatHandHalfWidth-std::abs(right.x-left.x)*.5f)<.0003f,"threepeat contact dimensions use the adapted body rather than raw clip lengths");
    check(near(cfg.threepeatHangToes[0],hangWorld[50].t)&&near(cfg.threepeatHangToes[1],hangWorld[51].t),"threepeat toe references include actual structure");
    const auto replantWorld=library.world(library.sampleBase(Motion::contextMantle,library.threepeatProfile.replantSamplePhase));
    for(int hand=0;hand<2;++hand)check(near(cfg.threepeatMantleReplant[hand],library.palm(replantWorld,hand)-library.palm(mantleWorld,hand)),"mantle reference and replant use one effective rig");
    library.clearAnimationOverrides();library.clearRig();check(!library.rig.active()&&exact(library.rest,original.rest),"detached library clears back to exact original canonical rest");
    for(float phase:{0.f,.37f,1.f})check(exact(library.sample(Motion::hang,phase),original.sample(Motion::hang,phase)),"clearing rig restores bit-identical canonical sampling");
    Fixture identity;check(library.configureRig(identity.native,identity.basis,identity.mapped)&&!library.rig.active(),"canonical library remains inactive");
    check(exact(library.sample(Motion::hang,.37f),original.sample(Motion::hang,.37f)),"canonical library configuration has no sampling drift");
    library.clearRig();Fixture arm;arm.native[7].t=arm.source[7].t*1.23f;arm.native[8].t=arm.source[8].t*.84f;arm.native[7].s={1.04f,1.04f,1.04f};
    check(library.configureRig(arm.native,arm.basis,arm.mapped),"IK fixture installs noncanonical segment lengths and uniform actual scale");
    auto pose=library.sample(Motion::hang,.2f);const auto before=pose,world=library.world(pose);const Vec start=world[6].t;const float upper=(world[7].t-start).length(),lower=(world[8].t-world[7].t).length();
    const Vec target=start+Vec{.36f,.22f,-.28f}*(upper+lower);const float error=library.ik(pose,6,7,8,target,start+Vec{0,80,30});
    check(std::isfinite(error)&&error<.003f,"IK reaches a target using actual adapted lengths");
    const auto after=library.world(pose);check(std::abs((after[7].t-after[6].t).length()-upper)<.003f&&std::abs((after[8].t-after[7].t).length()-lower)<.003f,"IK preserves actual world-space segment lengths");
    for(std::size_t i=0;i<99;++i)check(exact(pose[i].t,before[i].t)&&exact(pose[i].s,before[i].s),"IK never compensates by restoring canonical translation or stretching scale");
}
static void handoffKeepsActualStructure(){
    Fixture f;f.customize();PoseRig<Pose> rig;check(rig.configure(f.source,f.native,f.basis,f.mapped),"handoff fixture rig");
    const auto verify=[&](const Pose& visible){check(visible.size()==99,"handoff has a complete rig");for(std::size_t i=0;i<99;++i)if(f.mapped[i]&&i!=0&&i!=4){const auto local=rig.toLocal(i,visible[i]);check(near(local.t,f.native[i].t)&&near(local.s,f.native[i].s),"entry and exit keep actual local bone length and scale instead of shrinking toward canonical");}};
    for(float fps:{30.f,60.f,120.f})for(int mode:{0,1,2}){
        PoseHandoff handoff;Library guardLibrary;const float dt=1/fps;float time=0;Pose native,authored;
        auto frame=[&](float sampleTime){native=f.native;authored=f.source;authored[0].t={sampleTime*3,-sampleTime,5};authored[4].t=authored[4].t+Vec{0,0,sampleTime*2};for(std::size_t i=0;i<99;++i){native[i].q=Quat::axis({1,2,3},sampleTime*.2f+float(i)*.01f);authored[i].q=Quat::axis({-1,3,2},sampleTime*.4f+float(i)*.02f);native[i]=rig.toEffective(i,native[i]);}rig.adapt(authored);};
        for(int step=0;step<=int(fps*.3f);++step){time=step*dt;frame(time);auto output=handoff.evaluate(native,authored,std::min(1.f,time/.18f),mode==2&&time>=.2f?1.f:0.f,time);verify(output.pose);check(handoff.consumed(output),"confirmed entry output becomes the next handoff source");}
        check(handoff.beginExit(mode==1,mode==2),"confirmed custom output can enter each native exit path");
        for(int step=0;step<=int(fps*.4f);++step){const float elapsed=step*dt;frame(time+elapsed);handoff.advanceExitSource(elapsed,guardLibrary);auto output=handoff.evaluate(native,authored,1-smooth(elapsed/.28f),0,time+elapsed);verify(output.pose);check(handoff.consumed(output),"confirmed exit output preserves handoff revision");}
    }
}
namespace fc {class TraversalCapture {
public:
    static void sample(Traversal& t,Motion motion,float phase,const Library& library) {
        t.state=runMotion(motion)||motion<=Motion::right||motion==Motion::contextHang?State::wall:State::action;
        t.actionTime=t.approachTime=t.mantleTime=phase;
        t.moveDirection={motion==Motion::left||motion==Motion::runLeft||motion==Motion::runDiagonalLeft?-1.f:motion==Motion::right||motion==Motion::runRight||motion==Motion::runDiagonalRight?1.f:0.f,motion==Motion::down?-1.f:1.f,0};
        t.actionDirection=t.moveDirection;t.running=runMotion(motion);
        if(motion==Motion::contextMantle) {
            t.state=State::mantle;t.threepeatMantle=true;t.mantleFrom={0,-37,0};
            t.mantleLip={0,0,t.cfg.threepeatMantlePalmHeight};t.mantleTo={0,library.clip(motion).travel.y,t.cfg.threepeatMantlePalmHeight};
        }
    }
};}
struct RigWall:World {
    unsigned hits{};
    std::optional<Hit> ray(Vec a,Vec b)override {
        if(std::abs(b.y-a.y)<1e-7f)return {};
        const float at=-a.y/(b.y-a.y);
        if(at<0||at>1)return {};
        ++hits;return Hit{a+(b-a)*at,{0,-1,0},true};
    }
};
static Fixture actualFixture(const Library& library,int variant) {
    Fixture f;f.source=f.native=library.rest;
    if(variant==0)return f;
    for(int bone:{7,8,10,11,29,32,38,39})f.native[bone].t=f.source[bone].t*(bone%2?1.12f:.91f);
    for(int bone:{6,9,28,31})f.native[bone].s={1.035f,1.035f,1.035f};
    for(int bone:{68,71,74,77,80,83,86,89,92,95})f.native[bone].t=f.source[bone].t*1.08f;
    if(variant==2) {
        f.basis[28]={{.6f,-.3f,.4f},Quat::axis({0,0,1},.04f),{1.02f,1.02f,1.02f}};
        f.basis[31]={{-.6f,-.3f,.4f},Quat::axis({0,0,1},-.04f),{1.02f,1.02f,1.02f}};
        f.basis[6]={{.3f,0,.2f},Quat::axis({0,1,0},.025f),{.98f,.98f,.98f}};
    }
    return f;
}
static void finitePose(const Pose& pose) {
    check(pose.size()==99,"complete rendered pose");
    for(const auto& bone:pose)check(bone.t.finite()&&bone.s.finite()&&std::abs(bone.q.dot(bone.q)-1)<.003f,"surface output remains finite and normalized");
}
static void unchangedStructure(const Library& lib,const Fixture& f,const Pose& output) {
    for(std::size_t i=0;i<99;++i)if(f.mapped[i]&&i!=0&&i!=4) {
        const auto local=lib.rig.toLocal(i,output[i]);
        if(!near(local.t,f.native[i].t,.001f))throw std::runtime_error("actual locked translation changed at bone "+std::to_string(i)+" by "+std::to_string((local.t-f.native[i].t).length()));
        check(near(local.s,f.native[i].s,.001f),"surface and transitions retain the actual bone scale");
    }
}
static void actualClipCoverage(const Library& source) {
    unsigned motions=0,samples=0;float oldLengthError=0;
    for(int variant=0;variant<3;++variant) {
        auto lib=source;const auto f=actualFixture(source,variant);
        check(lib.configureRig(f.native,f.basis,f.mapped),"actual HKX fixture configures captured player structure");
        check(lib.rig.active()==(variant!=0),"canonical remains inactive while changed body structure activates adaptation");
        for(int id=1;id<=motionCount;++id)if(isActiveMotion(Motion(id))) {
            ++motions;const auto motion=Motion(id);const auto& clip=source.clip(motion);
            check(clip.frames.size()>=2,"every active motion has independent real HKX frames");
            for(int step=0;step<=40;++step) {
                const float phase=step/40.f;const auto authored=source.sample(motion,phase),adapted=lib.sample(motion,phase);
                finitePose(adapted);unchangedStructure(lib,f,adapted);
                if(variant==0)check(exact(authored,adapted),"all35 canonical HKX clips preserve every sampled transform bit");
                else for(int bone:{7,8,29,32,38,39})oldLengthError=std::max(oldLengthError,(authored[bone].t-f.native[bone].t).length());
                const auto body=lib.world(adapted);
                Pose scene(99);
                for(int bone=0;bone<99;++bone) {
                    const auto local=lib.rig.toLocal(bone,adapted[bone]);
                    const auto parent=lib.parents[bone]<0?Transform{}:scene[lib.parents[bone]];
                    scene[bone]=compose(compose(parent,f.mapped[bone]?f.basis[bone]:Transform{}),local);
                    check(near(body[bone].t,scene[bone].t,.003f),"IK world and expanded native scene agree through adjustment bridges");
                }
                ++samples;
            }
            check(exact(lib.clip(motion).frames.front(),clip.frames.front())&&exact(lib.clip(motion).frames.back(),clip.frames.back()),"actual HKX source frames remain immutable");
        }
    }
    check(motions==activeMotionCount*3&&samples==activeMotionCount*3*41,"all35 clips cover canonical and two changed player structures");
    check(oldLengthError>2,"negative control exposes the old canonical translation override");
    std::cout<<"HKX_RIG motions="<<motions<<" samples="<<samples<<" oldLengthError="<<oldLengthError<<'\n';
}
static void surfaceCoverage(const Library& source) {
    unsigned samples=0;float oldEntryError=0;
    for(int variant=0;variant<3;++variant)for(int fps:{30,60,120}) {
        auto lib=source;const auto f=actualFixture(source,variant);check(lib.configureRig(f.native,f.basis,f.mapped),"surface fixture configures actual structure");
        for(int id=1;id<=motionCount;++id)if(isActiveMotion(Motion(id))) {
            const auto motion=Motion(id);SurfacePose surface,canonicalSurface;RigWall world,canonicalWorld;
            Traversal t;t.cfg.gap=37;t.position={0,-37,0};t.normal=t.surfaceNormal={0,-1,0};
            check(lib.configureThreepeat(t.cfg),"surface contact metadata fits the actual body");
            Traversal comparison=t;PoseHandoff handoff,oldHandoff;Pose latest;const float dt=1.f/fps;
            for(int frame=0;frame<=fps;++frame) {
                const float time=frame*dt;
                TraversalCapture::sample(t,motion,time,lib);TraversalCapture::sample(comparison,motion,time,source);
                auto native=f.native;
                for(int bone=0;bone<99;++bone)native[bone]=lib.rig.toEffective(bone,native[bone]);
                const auto posed=surface.update(lib,world,t,motion,dt,1);finitePose(posed);unchangedStructure(lib,f,posed);
                if(variant==0)check(exact(posed,canonicalSurface.update(source,canonicalWorld,comparison,motion,dt,1)),"actual SurfacePose canonical output is bit-identical throughout every motion");
                auto output=handoff.evaluate(native,posed,smooth(time/.18f),0,time);finitePose(output.pose);unchangedStructure(lib,f,output.pose);
                check(handoff.consumed(output),"actual surface entry and stable output acknowledged");latest=posed;
                if(variant==1) {
                    const auto wrong=oldHandoff.evaluate(native,source.sample(motion,time),smooth(time/.18f),0,time);
                    for(int bone:{7,8,29,32,38,39})oldEntryError=std::max(oldEntryError,(wrong.pose[bone].t-f.native[bone].t).length());
                    oldHandoff.consumed(wrong);
                }
                ++samples;
            }
            for(int mode=0;mode<3;++mode) {
                auto exit=handoff;check(exit.beginExit(mode==1,mode==2),"actual surface supports static, moving, and completed-top native exits");
                for(int frame=0;frame<=fps/2;++frame) {
                    const float elapsed=frame*dt;auto native=f.native;
                    for(int bone=0;bone<99;++bone)native[bone]=lib.rig.toEffective(bone,native[bone]);
                    exit.advanceExitSource(elapsed,lib);
                    const auto output=exit.evaluate(native,latest,1-smooth(elapsed/.28f),0,1+elapsed);
                    finitePose(output.pose);unchangedStructure(lib,f,output.pose);check(exit.consumed(output),"actual exit acknowledgements retain the captured skeleton");
                }
            }
        }
    }
    check(oldEntryError>2,"old handoff negative control visibly changes actual limb lengths on entry");
    std::cout<<"SURFACE_RIG samples="<<samples<<" oldEntryError="<<oldEntryError<<'\n';
}
static void supportedActualIK(const Library& source) {
    for(int variant:{1,2})for(int fps:{30,60,120}) {
        auto lib=source;const auto f=actualFixture(source,variant);check(lib.configureRig(f.native,f.basis,f.mapped),"supported wall uses actual body rig");
        RigWall world;Traversal t;t.cfg.gap=37;t.cfg.radius=31;t.cfg.height=138;t.cfg.approachSeconds=.01f;
        t.cfg.automaticClimbActions=false;check(lib.configureThreepeat(t.cfg),"real attachment uses adapted dimensions");
        check(t.attach(world,{0,-45,0},{0,1,0},1000,60),"actual rig enters through live traversal support geometry");
        SurfacePose surface;float maxPalm=0;unsigned contactFrames=0;
        for(int frame=0;frame<fps*2;++frame) {
            const auto result=t.update(world,{},1.f/fps,1000);check(!result.released&&t.active(),"actual supported rig stays attached");
            const auto pose=surface.update(lib,world,t,result.motion,1.f/fps,1),body=lib.world(pose);
            unchangedStructure(lib,f,pose);
            for(int hand=0;hand<2;++hand)check(lib.armBendValid(pose,hand),"actual supported IK retains the anatomical elbow branch");
            if(frame>fps)for(int hand=0;hand<2;++hand) {
                const auto palm=lib.palm(body,hand);const Vec worldPalm=t.position+Vec{palm.x,palm.y,palm.z};
                const auto contact=world.ray(worldPalm+t.normal*14,worldPalm-t.normal*20);
                check(contact.has_value(),"adapted actual palms have real wall support");
                maxPalm=std::max(maxPalm,(worldPalm-contact->point-contact->normal*.8f).length());
                ++contactFrames;
            }
        }
        check(maxPalm<5.001f,"actual proportions and bridge nodes keep the unchanged five-unit palm-contact envelope");
        check(contactFrames>unsigned(fps)&&world.hits>contactFrames,"support and IK were actually queried throughout real attached idle");
        std::cout<<"ACTUAL_IK variant="<<variant<<" fps="<<fps<<" palmError="<<maxPalm<<" contactFrames="<<contactFrames<<'\n';
    }
}
int main(int argc,char** argv)try{
    sceneRotationValidation();inactivePreservesBits();structuralCapture();animatedLocalsKeepStableStructure();structuralValidationRejectsInvalidData();adaptationAndBridges();invalidConfigurationIsTransactional();libraryRestSamplingAndIK();handoffKeepsActualStructure();
    check(argc==2,"runtime HKX pack path required");Library source;check(source.load(argv[1]),"load actual runtime HKX pack");
    actualClipCoverage(source);surfaceCoverage(source);supportedActualIK(source);
    std::cout<<"PASS PoseRig: "<<checks<<" checks for canonical bit identity, actual HKX clips, locked structural capture, actual lengths/scales, bridge scene/IK agreement, transactional rejection and entry/exit handoff\n";return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
