#include "PoseHandoff.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
static void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
static float distance(const Pose& a,const Pose& b){
    require(a.size()==99&&b.size()==99,"use the full actual skeleton");float worst=0;
    for(std::size_t i=0;i<a.size();++i)worst=std::max(worst,(a[i].t-b[i].t).length()+angleBetween(a[i].q,b[i].q)+(a[i].s-b[i].s).length());
    return worst;
}
static void finitePose(const Pose& pose){
    for(const auto& bone:pose)require(bone.t.finite()&&bone.s.finite()&&std::isfinite(bone.q.dot(bone.q))&&std::abs(bone.q.dot(bone.q)-1)<.002f,"native continuation stays finite and normalized");
}
static Pose nativeOutsideReference(const Library& library,const Pose& base,int hand){
    const int elbow=hand?32:29;
    for(const auto axis:{Vec{1,0,0},Vec{0,1,0},Vec{0,0,1}}){
        auto native=base;native[elbow].q=(Quat::axis(axis,3)*native[elbow].q).unit();
        if(!library.armBendValid(native,hand)){finitePose(native);return native;}
    }
    throw std::runtime_error("native fixture must differ from the climbing arm reference");
}
static Pose bend(const Library& library,Pose pose,int hand,float radians){
    const int elbow=hand?32:29,wrist=hand?39:38;const auto& reference=library.armBends[hand];
    require(reference.valid,"actual arm reference is calibrated");
    const Vec direction=reference.axis*std::cos(radians)+reference.direction*std::sin(radians);
    pose[elbow].q=(Quat::between(pose[elbow].q.rotate(pose[wrist].t),direction)*pose[elbow].q).unit();
    return pose;
}
static void record(PoseHandoff& handoff,const Pose& older,const Pose& current,const Pose& authored,float recovery){
    require(handoff.consumed(handoff.evaluate(older,authored,1,recovery,.98f)),"record older propagated output");
    require(handoff.consumed(handoff.evaluate(current,authored,1,recovery,1)),"record current propagated output");
}
static void stationaryNative(const Library& library,const Pose& base,int hand,bool initialZero,float recovery){
    const auto native=nativeOutsideReference(library,base,hand);PoseHandoff handoff;
    record(handoff,native,native,base,recovery);
    const auto shown=handoff.evaluate(native,base,1,recovery,1).pose;
    require(!library.armBendValid(shown,hand),"nearly recovered native output still differs from the climb reference");
    require(handoff.beginExit(false,true),"completed top starts native continuation");
    require(distance(handoff.evaluate(native,base,1).pose,shown)<.00001f,"zero-age exit preserves the actual displayed pose");
    if(initialZero)handoff.advanceExitSource(0,library);
    constexpr float epsilon=.0001f;handoff.advanceExitSource(epsilon,library);
    const auto first=handoff.evaluate(native,base,1).pose;
    const float firstStep=distance(first,shown);
    auto legacySource=shown;library.guardArmBends(legacySource);Pose legacy=shown;
    for(std::size_t i=0;i<legacy.size();++i)legacy[i]=blend(legacySource[i],shown[i],smooth(epsilon/PoseHandoff::nativeExitSeconds));
    const float legacyStep=distance(legacy,shown);
    std::cout<<"NATIVE_EXIT_ARM hand="<<hand<<" initialZero="<<initialZero<<" recovery="<<recovery<<" baselineStep="<<legacyStep<<" currentStep="<<firstStep<<'\n';
    require(legacyStep>.05f,"negative control reproduces the old unconditional native arm correction");
    require(firstStep<.00001f,"stationary native arm cannot jump into the climbing reference on the first positive exit sample");
    for(float age:{epsilon,.01f,.04f,.08f,.119f}){
        handoff.advanceExitSource(age,library);const auto shownAgain=handoff.evaluate(native,base,1).pose;
        auto expected=shown;for(std::size_t i=0;i<expected.size();++i)expected[i]=blend(shown[i],native[i],smooth(age/PoseHandoff::nativeExitSeconds));
        finitePose(shownAgain);require(distance(shownAgain,expected)<.00001f,"stationary native source follows one live-target blend without an added climbing arm correction");
    }
    handoff.advanceExitSource(PoseHandoff::nativeExitSeconds,library);
    require(distance(handoff.evaluate(native,base,0).pose,native)<.00001f,"completed bridge returns the exact live native pose");
}
static void protectedContinuation(const Library& library,const Pose& base,int guardedHand,bool initialZero){
    auto current=nativeOutsideReference(library,base,1-guardedHand);current=bend(library,current,guardedHand,.005f);
    const auto older=bend(library,current,guardedHand,.065f);
    require(library.armBendValid(current,guardedHand)&&library.armBendValid(older,guardedHand),"both observed guarded-side frames are legal");
    PoseContinuation prediction;prediction.begin(current,older,.02f);
    constexpr float elapsed=.02f;const auto unguarded=prediction.sample(elapsed);
    require(!library.armBendValid(unguarded,guardedHand),"bounded continuation crosses the arm limit before its first nonzero sample");
    PoseHandoff handoff;record(handoff,older,current,base,1);require(handoff.beginExit(false,true),"mixed native arm source starts exit");
    if(initialZero)handoff.advanceExitSource(0,library);
    handoff.advanceExitSource(elapsed,library);const auto output=handoff.evaluate(current,base,1).pose;
    finitePose(output);require(library.armBendValid(output,guardedHand),"initially legal arm stays guarded even if the first sampled continuation is already outside its limit");
    const int otherElbow=guardedHand?29:32;
    require(angleBetween(output[otherElbow].q,current[otherElbow].q)<.00001f,"guarding one side does not alter a stationary native arm on the other side");
    handoff.advanceExitSource(PoseHandoff::nativeExitSeconds,library);
    const auto next=nativeOutsideReference(library,base,guardedHand);
    require(handoff.consumed(handoff.evaluate(next,base,0))&&handoff.beginExit(false,true),"a restarted exit captures its new displayed native source");
    handoff.advanceExitSource(.000001f,library);
    require(distance(handoff.evaluate(next,base,1).pose,next)<.0001f,"restarting an exit cannot reuse the previous source arm classification");
}
static void physicalExitStillGuarded(const Library& library,const Pose& base,int hand){
    const auto source=nativeOutsideReference(library,base,hand);PoseHandoff handoff;
    require(handoff.consumed(handoff.evaluate(base,source,1,0,.98f)),"record older physical source");
    require(handoff.consumed(handoff.evaluate(base,source,1,0,1))&&handoff.beginExit(true,true),"physical exit takes priority over native takeover");
    require(!handoff.nativeExitActive(),"physical departure does not use native exit policy");handoff.advanceExitSource(.0001f,library);
    const auto output=handoff.evaluate(base,source,1).pose;
    require(library.armBendValid(output,0)&&library.armBendValid(output,1)&&distance(output,source)>.05f,"physical fall retains the original unconditional anatomical guards");
}
int main(int argc,char** argv){try{
    Library library;require(argc==2&&library.load(argv[1]),"load current animation pack");
    auto base=library.sample(Motion::contextMantle,1);library.guardArmBends(base);
    for(int hand=0;hand<2;++hand){
        for(bool zero:{false,true}){
            for(float recovery:{.999984f,1.f})stationaryNative(library,base,hand,zero,recovery);
            protectedContinuation(library,base,hand,zero);
        }
        physicalExitStillGuarded(library,base,hand);
    }
    std::cout<<"PASS native exit arms: actual rig, stationary native continuity, first-sample classification, restart and unchanged physical guards\n";return 0;
}catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}}
