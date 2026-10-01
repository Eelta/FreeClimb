#include "AnimationOverrides.h"
#include "PoseHandoff.h"
#include "PoseBlendEnvelope.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
static void require(bool value,const char* text){if(!value)throw std::runtime_error(text);}
static float distance(const Pose& a,const Pose& b){
    require(a.size()==99&&b.size()==99,"test uses the actual full 99-bone skeleton");float worst=0;
    for(std::size_t i=0;i<a.size();++i)worst=std::max(worst,(a[i].t-b[i].t).length()+angleBetween(a[i].q,b[i].q)+(a[i].s-b[i].s).length());
    return worst;
}
static float endpointDistance(const Library& library,const Pose& a,const Pose& b){
    const auto x=library.world(a),y=library.world(b);float worst=0;
    for(int bone:{4,8,11,26,36,38,39,75,90})worst=std::max(worst,(x[bone].t-y[bone].t).length());return worst;
}
static Pose nativeSurrogate(const Library& library,float time,bool aligned=false){

    auto p=library.sample(Motion::runUp,.19f+time/.93f);library.guardArmBends(p);
    if(aligned){
        const auto standing=library.sample(Motion::contextMantle,1);
        for(std::size_t i=0;i<p.size();++i){p[i].q=blend(standing[i].q,p[i].q,.35f);p[i].t=library.rest[i].t;p[i].s=library.rest[i].s;}
        p[0]=library.rest[0];p[4]=library.rest[4];library.guardArmBends(p);
        p[4].t=p[4].t+Vec{2+10*time,-1+4*time,.3f};
    }else p[4].t=p[4].t+Vec{8+16*time,-4+9*time,1};return p;
}
static Pose observedStanding(const Library& library,float time,bool aligned=false){
    auto p=library.sample(Motion::contextMantle,1);library.guardArmBends(p);
    if(aligned){for(std::size_t i=0;i<p.size();++i){p[i].t=library.rest[i].t;p[i].s=library.rest[i].s;}p[0]=library.rest[0];p[4]=library.rest[4];}
    p[4].t=p[4].t+Vec{35*time,0,0};p[36].q=(Quat::axis({0,0,1},1.4f*time)*p[36].q).unit();return p;
}
static void finitePose(const Pose& p){for(const auto& b:p)require(b.t.finite()&&b.s.finite()&&std::isfinite(b.q.dot(b.q))&&std::abs(b.q.dot(b.q)-1)<.002f,"native takeover stays finite and normalized");}
static void dynamicTakeover(const Library& library,int fps,float recovery,bool aligned=false){
    const float dt=1.f/fps;auto authored=library.sample(Motion::contextMantle,.98f);const auto stand=observedStanding(library,0,aligned),older=observedStanding(library,-dt,aligned);
    if(aligned){for(std::size_t i=0;i<authored.size();++i){authored[i].t=library.rest[i].t;authored[i].s=library.rest[i].s;}authored[0]=library.rest[0];authored[4]=library.rest[4];}
    PoseHandoff smoothExit,legacy;
    for(auto* h:{&smoothExit,&legacy}){
        const auto old=h->evaluate(older,authored,1,recovery,1-dt);require(h->consumed(old),"older successfully propagated frame recorded");
        const auto now=h->evaluate(stand,authored,1,recovery,1);
        for(int repeat=0;repeat<7;++repeat)require(h->consumed(now),"duplicate scene callbacks do not invent temporal samples");
        require(!h->consumed(old),"an older render cannot rewind actual displayed history");
    }
    const auto shown=smoothExit.evaluate(stand,authored,1,recovery,1).pose;
    const auto oldShown=legacy.evaluate(older,authored,1,recovery,1-dt).pose;
    auto unseen=stand;unseen[4].t.x+=500;
    const auto pending=smoothExit.evaluate(unseen,unseen,1,1,1+dt);
    require(smoothExit.beginExit(false,true)&&smoothExit.nativeExitActive(),"successful top opt-in starts a bounded native bridge even at recovery one");
    require(legacy.beginExit(),"legacy comparison begins through the unchanged default API");
    require(!smoothExit.consumed(pending),"unconsumed terminal publication cannot replace the actual exit origin");
    const auto native0=nativeSurrogate(library,0,aligned);
    smoothExit.advanceExitSource(0,library);
    const auto initial=smoothExit.evaluate(native0,unseen,1,0,1);
    require(distance(initial.pose,shown)<.0002f,"first new phase starts exactly at the last actually displayed 99-bone pose");
    require(endpointDistance(library,legacy.evaluate(native0,unseen,1).pose,shown)>3.f,"fixture contains a meaningful stand-to-locomotion endpoint jump");

    constexpr float epsilon=.0001f;smoothExit.advanceExitSource(epsilon,library);
    const auto first=smoothExit.evaluate(nativeSurrogate(library,epsilon,aligned),unseen,1).pose;
    const auto actualVelocity=(first[4].t-shown[4].t)/epsilon,oldVelocity=(shown[4].t-oldShown[4].t)/dt;
    require((actualVelocity-oldVelocity).length()<.12f,"initial COM derivative retains displayed motion instead of freezing or taking the new native velocity");
    const float angularSpeed=angleBetween(first[36].q,shown[36].q)/epsilon;
    require(angularSpeed>1.15f&&angularSpeed<1.65f,"actual head rotation continues at release rather than holding a frozen snapshot");
    require(smoothExit.consumed(initial),"new-phase output remains acknowledgeable after exit");

    float firstNew=0,firstOld=0,maxStep=0,maxAngle=0,maxWrist=0;Pose previous=shown;
    for(int frame=1;frame<=int(.30f*fps)+2;++frame){
        const float elapsed=frame*dt,weight=1-smooth(elapsed/PoseHandoff::nativeExitSeconds);auto native=nativeSurrogate(library,elapsed,aligned);
        smoothExit.advanceExitSource(elapsed,library);const auto output=smoothExit.evaluate(native,unseen,weight,0,1+elapsed);
        finitePose(output.pose);const float step=endpointDistance(library,output.pose,previous);maxStep=std::max(maxStep,step);
        float angle=0;for(std::size_t i=0;i<output.pose.size();++i)angle=std::max(angle,angleBetween(output.pose[i].q,previous[i].q));maxAngle=std::max(maxAngle,angle);
        if(aligned){
            require(library.armBendValid(output.pose,0)&&library.armBendValid(output.pose,1),"aligned actual-rig takeover preserves anatomical elbow hinges");
            const auto body=library.world(output.pose);
            for(int hand=0;hand<2;++hand){const int elbow=hand?32:29,wrist=hand?39:38,middle=hand?88:73;
                const float flex=std::acos(std::clamp((body[wrist].t-body[elbow].t).unit().dot((body[middle].t-body[wrist].t).unit()),-1.f,1.f));maxWrist=std::max(maxWrist,flex);
                require(flex<1.658064f+.002f,"aligned runtime surrogate retains original95degree wrist budget");}
            require(angle<=12.566371f*dt+.005f,"aligned native graph transition remains inside original ordinary output angular rate budget");
            require(step<=750.f*dt+.6f,"aligned top-out bridge endpoints remain inside original top-out speed budget");
        }
        if(frame==1){firstNew=step;firstOld=endpointDistance(library,legacy.evaluate(native,unseen,weight).pose,shown);
            require(firstNew<firstOld*.40f,"new first-frame full-body endpoint jump is substantially smaller than the old nearly-native release");}
        const auto duplicate=smoothExit.evaluate(native,unseen,weight,0,1+elapsed);
        require(distance(output.pose,duplicate.pose)<.00001f,"repeated render evaluation does not advance or recursively fade the bridge");
        require(smoothExit.consumed(output)&&smoothExit.consumed(duplicate),"new revision consumes duplicate output without restarting takeover");
        previous=output.pose;
        if(elapsed>=.28f)require(distance(output.pose,native)<.00001f,"all99 bones reach the changing live native target, without a stale captured endpoint");
    }
    require(!smoothExit.nativeExitActive(),"the short bridge retires independently of the old residual-custom fade");
    auto liveA=nativeSurrogate(library,.5f,aligned),liveB=nativeSurrogate(library,.65f,aligned);
    require(distance(smoothExit.evaluate(liveA,unseen,0).pose,liveA)<.00001f&&distance(smoothExit.evaluate(liveB,unseen,0).pose,liveB)<.00001f,"native animation remains live after the bridge ends");
    const auto stale=smoothExit.evaluate(liveB,unseen,0);smoothExit.clear();
    require(!smoothExit.consumed(stale)&&!smoothExit.hasOutput(),"old-revision callbacks cannot resurrect an ended top-out");
    std::cout<<"TOP_EXIT aligned="<<aligned<<" fps="<<fps<<" recovery="<<recovery<<" firstEndpointOld="<<firstOld<<" firstEndpointNew="<<firstNew<<" peakStep="<<maxStep<<" peakAngle="<<maxAngle<<" maxWrist="<<maxWrist<<" initialSpeed="<<actualVelocity.length()<<" initialHeadSpeed="<<angularSpeed<<'\n';
}
static void terminalDerivativeAndTarget(const Library& library){
    PoseHandoff h;const auto stand=observedStanding(library,0),authored=library.sample(Motion::contextMantle,.98f);
    h.consumed(h.evaluate(observedStanding(library,-.01f),authored,1,1,.99f));h.consumed(h.evaluate(stand,authored,1,1,1));require(h.beginExit(false,true),"complete recovered top can still bridge native graph discontinuity");
    const float end=PoseHandoff::nativeExitSeconds,epsilon=.0005f;
    h.advanceExitSource(end-epsilon,library);const auto targetBefore=nativeSurrogate(library,end-epsilon),before=h.evaluate(targetBefore,authored,1).pose;

    auto different=targetBefore;different[4].t.y+=3;const auto changed=h.evaluate(different,authored,1).pose;
    require((changed[4].t-before[4].t).length()>2.99f,"late bridge follows each live native callback rather than caching its first target");
    h.advanceExitSource(end,library);const auto targetEnd=nativeSurrogate(library,end),last=h.evaluate(targetEnd,authored,1).pose;
    require(distance(last,targetEnd)<.00001f&&!h.nativeExitActive(),"at exactly .12 seconds a fully recovered top is exactly live native");
    const Vec exitVelocity=(last[4].t-before[4].t)/epsilon,nativeVelocity=(targetEnd[4].t-targetBefore[4].t)/epsilon;
    require((exitVelocity-nativeVelocity).length()<.15f,"bridge endpoint derivative matches the continuously updating native target");
    require(endpointDistance(library,before,targetBefore)<.001f,"all important actual FK endpoints converge before ownership retires");
}
static void optInAndRevisionContracts(const Library& library){
    const auto a=observedStanding(library,0),b=nativeSurrogate(library,0),authored=library.sample(Motion::contextMantle,.94f);
    PoseHandoff empty;empty.evaluate(a,authored,1,1,1);require(!empty.beginExit(false,true),"published but never propagated output is not a valid native-bridge source");
    for(float recovery:{0.f,.6f,1.f}){
        PoseHandoff old;const auto shown=old.evaluate(a,authored,1,recovery,1);require(old.consumed(shown)&&old.beginExit(),"legacy release begins");
        require(!old.nativeExitActive(),"ordinary/mid-action release does not opt into completed-top ownership");
        const auto got=old.evaluate(b,authored,.5f).pose;Pose expected=b;
        for(std::size_t i=0;i<expected.size();++i)expected[i]=blend(authored[i],b[i],1-.5f*(1-recovery));
        require(distance(got,expected)<.0001f,"no-opt-in residual custom/live-native composition remains unchanged");
    }
    PoseHandoff fall;fall.consumed(fall.evaluate(a,authored,1,0,1));require(fall.beginExit(true,true)&&!fall.nativeExitActive(),"physical fall continuation takes priority over an accidental native-top flag");
}
static void acknowledgedRuntimeSchedule(const Library& library,int fps){
    const float dt=1.f/fps;const auto authored=library.sample(Motion::contextMantle,.98f),stand=observedStanding(library,0,true);
    PoseHandoff h;h.consumed(h.evaluate(observedStanding(library,-dt,true),authored,1,1,1-dt));h.consumed(h.evaluate(stand,authored,1,1,1));
    auto pending=h.evaluate(nativeSurrogate(library,0,true),authored,1,1,1+dt);
    require(h.beginExit(false,true),"acknowledged runtime schedule begins a completed top");
    PoseBlendEnvelope envelope;std::uint32_t applied=20;envelope.beginExit(applied,PoseHandoff::nativeExitSeconds);
    float elapsed=0;
    auto tick=[&](float step){elapsed+=step;envelope.advanceExit(applied,step);h.advanceExitSource(elapsed,library,envelope.elapsedExitSeconds());};
    for(int frame=0;frame<fps/2;++frame)tick(dt);
    require(envelope.elapsedExitSeconds()==0&&envelope.weight()==1&&h.nativeExitActive(),"half-second callback stall cannot consume the unseen .12-second bridge");
    require(!h.consumed(pending),"an old revision finishing after the stall cannot acknowledge native takeover");
    const auto resumed=h.evaluate(nativeSurrogate(library,elapsed,true),authored,envelope.weight(),0,1+elapsed);
    require(distance(resumed.pose,stand)<.00001f,"first successful callback after the stall still begins at the actual last displayed pose");
    require(h.consumed(resumed),"resumed valid output is acknowledged");++applied;tick(dt);
    require(std::abs(envelope.elapsedExitSeconds()-dt)<.00001f,"one confirmed display advances exactly one frame of bridge time");
    const auto fixedNative=nativeSurrogate(library,elapsed,true);const auto held=h.evaluate(fixedNative,authored,envelope.weight(),0,1+elapsed);
    const float before=envelope.elapsedExitSeconds();
    for(int frame=0;frame<fps/4;++frame)tick(dt);
    require(envelope.elapsedExitSeconds()==before&&distance(h.evaluate(fixedNative,authored,envelope.weight()).pose,held.pose)<.00001f,"additional missing callbacks preserve the visible bridge phase and continuation source");
    for(int pass=0;pass<7;++pass){require(h.consumed(held),"same publication may render in several scene passes");++applied;}
    tick(dt);require(std::abs(envelope.elapsedExitSeconds()-2*dt)<.00001f,"many same-frame scene passes advance one dt rather than seven");
    for(int frame=0;frame<fps;++frame){
        auto shown=h.evaluate(nativeSurrogate(library,elapsed,true),authored,envelope.weight(),0,1+elapsed);require(h.consumed(shown),"fresh native-bridge output renders");++applied;tick(dt);
    }
    require(envelope.weight()==0&&!h.nativeExitActive(),"acknowledged sequence reaches exact native and retires normally");
    require(distance(h.evaluate(fixedNative,authored,envelope.weight()).pose,fixedNative)<.00001f,"retired runtime bridge is the current live native pose");

    PoseHandoff fall;const auto older=observedStanding(library,-dt,true);
    fall.consumed(fall.evaluate(stand,older,1,0,1-dt));fall.consumed(fall.evaluate(stand,stand,1,0,1));require(fall.beginExit(true),"physical departure retains separate continuation");
    fall.advanceExitSource(.04f,library,0);const auto moving=fall.evaluate(stand,stand,1).pose;
    require((moving[4].t-stand[4].t).length()>.8f,"new acknowledged-native clock does not freeze pre-existing physical-fall extrapolation");
}
int main(int argc,char** argv){try{
    Library library;require(argc>=2&&argc<=3&&library.load(argv[1]),"load current runtime motion library");
    if(argc==3) {
        const auto overrides=fc::loadHkxOverrides(library,argv[2]);
        require(overrides.loaded==activeMotionCount&&overrides.rejected==0&&overrides.missing==0,
            "load every active HKX slot without missing or rejected clips");
        for(Motion motion:activeMotions)require(library.hasAnimationOverride(motion),"every active slot installs its HKX override");
    }
    for(bool aligned:{false,true})for(int fps:{30,60,120})for(float recovery:{.92f,.999f,1.f})dynamicTakeover(library,fps,recovery,aligned);
    terminalDerivativeAndTarget(library);optInAndRevisionContracts(library);
    for(int fps:{30,60,120})acknowledgedRuntimeSchedule(library,fps);
    std::cout<<"PASS completed top native takeover: actual99 bones, initial/final velocity, dynamic endpoint, delayed callbacks and legacy contracts\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
