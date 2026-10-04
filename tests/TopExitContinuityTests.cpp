#include "AnimationOverrides.h"
#include "PoseHandoff.h"
#include "PoseBlendEnvelope.h"
#include "PoseOutput.h"
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
    require(distance(last,targetEnd)<.00001f&&!h.nativeExitActive(),"at the configured endpoint a fully recovered top is exactly live native");
    const Vec exitVelocity=(last[4].t-before[4].t)/epsilon,nativeVelocity=(targetEnd[4].t-targetBefore[4].t)/epsilon;
    require((exitVelocity-nativeVelocity).length()<.15f,"bridge endpoint derivative matches the continuously updating native target");
    require(endpointDistance(library,before,targetBefore)<.001f,"all important actual FK endpoints converge before ownership retires");
}
static void singleLayerWeight(const Library& library){
    const auto source=library.sample(Motion::contextMantle,1);auto native=source;native[4].t.x+=20;
    PoseHandoff h;require(h.consumed(h.evaluate(native,source,1,0,.98f)),"single-stage fixture records the actual authored mantle");
    require(h.consumed(h.evaluate(native,source,1,0,1))&&h.beginExit(false,true),"full authored endpoint starts the native transition without pre-recovery");
    const float elapsed=PoseHandoff::nativeExitSeconds*.5f;h.advanceExitSource(elapsed,library);
    const auto output=h.evaluate(native,source,.5f);const float current=output.pose[4].t.x-source[4].t.x;
    const float legacy=(native[4].t.x-source[4].t.x)*smooth(.5f)*smooth(.5f);
    std::cout<<"SINGLE_LAYER baselineMiddle="<<legacy<<" currentMiddle="<<current<<" targetDelta=20\n";
    require(std::abs(current-10)<.0001f&&std::abs(output.recovery-.5f)<.00001f,"native midpoint has one half-weight blend rather than multiplying two fade weights");
    require(std::abs(current-legacy)>4.99f,"negative control contains the previous double-eased native delay");
}
struct NativeExitLedge:World {
    float height=120;
    std::optional<Hit> ray(Vec a,Vec b) override {
        std::optional<Hit> hit;float nearest=2;
        if(a.y<0&&b.y>=0){
            const float t=-a.y/(b.y-a.y);const Vec point=a+(b-a)*t;
            if(point.z<=height&&t<nearest){nearest=t;hit=Hit{point,{0,-1,0},true};}
        }
        if(a.z>height&&b.z<=height){
            const float t=(height-a.z)/(b.z-a.z);const Vec point=a+(b-a)*t;
            if(point.y>=0&&t<nearest)hit=Hit{point,{0,0,1},true};
        }
        return hit;
    }
};
static std::array<Pose,2> authoredFinish(const Library& library,int fps,float height){
    NativeExitLedge ledge;ledge.height=height;Traversal traversal;SurfacePose surface;
    require(library.configureThreepeat(traversal.cfg),"calibrate actual authored top path");traversal.cfg.gap=37;traversal.cfg.radius=31;
    require(traversal.attach(ledge,{0,-42,0},{0,1,0},100),"actual native-exit fixture attaches to a collision-checked ledge");
    std::array<Pose,2> poses;
    for(int frame=0;frame<fps*8&&traversal.active();++frame){
        const auto result=traversal.update(ledge,{0,1,false,true},1.f/fps,100);
        poses[0]=std::move(poses[1]);poses[1]=surface.update(library,ledge,traversal,result.motion,1.f/fps,1);
        if(result.completed){
            require(result.motion==Motion::contextMantle&&traversal.position.z>=height&&poses[0].size()==99&&poses[1].size()==99,"both final displayed samples follow the complete supported mantle path");
            return poses;
        }
    }
    throw std::runtime_error("actual top fixture must complete its collision-controlled route");
}
static void completeAuthoredTakeover(const Library& library,int fps,bool moving,float height){
    const float dt=1.f/fps;const auto finish=authoredFinish(library,fps,height);
    const auto& older=finish[0];const auto& source=finish[1];
    const auto target=[&](float elapsed){
        return moving?nativeSurrogate(library,elapsed,true):library.rest;
    };
    PoseHandoff h;
    require(h.consumed(h.evaluate(target(0),older,1,0,1-dt)),"full authored fixture records the penultimate actual mantle frame");
    require(h.consumed(h.evaluate(target(0),source,1,0,1)),"full authored fixture records the actual mantle endpoint");
    require(h.beginExit(false,true),"only the completed authored action starts native takeover");h.advanceExitSource(0,library);
    require(distance(h.evaluate(target(0),source,1).pose,source)<.00001f,"actual mantle endpoint has no discontinuity when native ownership begins");
    PoseContinuation continuation;continuation.begin(source,older,dt);constexpr float epsilon=.0001f;
    const auto predicted=continuation.sample(epsilon);h.advanceExitSource(epsilon,library);
    const auto first=h.evaluate(target(epsilon),source,1).pose;
    require(((first[4].t-source[4].t)-(predicted[4].t-source[4].t)).length()/epsilon<.12f,"full authored COM continues its bounded outgoing velocity at the initial native transition");
    float maxStep=0,maxAngle=0,maxWrist=0;Pose previous=source;
    for(int frame=1;frame<=int(std::ceil(PoseHandoff::nativeExitSeconds*fps));++frame){
        const float elapsed=std::min(frame*dt,PoseHandoff::nativeExitSeconds);h.advanceExitSource(elapsed,library);
        const auto native=target(elapsed);const auto output=h.evaluate(native,source,1-smooth(elapsed/PoseHandoff::nativeExitSeconds));finitePose(output.pose);
        const float step=endpointDistance(library,output.pose,previous);float angle=0;
        for(std::size_t i=0;i<output.pose.size();++i)angle=std::max(angle,angleBetween(output.pose[i].q,previous[i].q));
        maxStep=std::max(maxStep,step);maxAngle=std::max(maxAngle,angle);
        if(step>750.f*dt+.6f||angle>12.566371f*dt+.005f){
            std::cout<<"AUTHORED_TOP_LIMIT fps="<<fps<<" moving="<<moving<<" height="<<height<<" frame="<<frame<<" age="<<elapsed<<" step="<<step<<" angle="<<angle
                <<" sourceRoot="<<source[0].t.x<<','<<source[0].t.y<<','<<source[0].t.z<<" nativeRoot="<<native[0].t.x<<','<<native[0].t.y<<','<<native[0].t.z
                <<" sourceCOM="<<source[4].t.x<<','<<source[4].t.y<<','<<source[4].t.z<<" nativeCOM="<<native[4].t.x<<','<<native[4].t.y<<','<<native[4].t.z<<'\n';
            const auto beforeWorld=library.world(previous),afterWorld=library.world(output.pose);
            for(int bone:{0,4,8,11,26,36,38,39,75,90})std::cout<<"AUTHORED_TOP_BONE bone="<<bone<<" step="<<(afterWorld[bone].t-beforeWorld[bone].t).length()<<" angle="<<angleBetween(previous[bone].q,output.pose[bone].q)<<'\n';
        }
        const auto world=library.world(output.pose);
        for(int hand=0;hand<2;++hand){const int elbow=hand?32:29,wrist=hand?39:38,middle=hand?88:73;
            const float flex=std::acos(std::clamp((world[wrist].t-world[elbow].t).unit().dot((world[middle].t-world[wrist].t).unit()),-1.f,1.f));maxWrist=std::max(maxWrist,flex);
            require(flex<1.658064f+.002f,"full authored takeover retains the original ninety-five-degree wrist budget");}
        require(angle<=12.566371f*dt+.005f,"full authored takeover retains the original angular output rate");
        require(step<=750.f*dt+.6f,"full authored takeover retains the original top endpoint speed");
        require(h.consumed(output),"full authored native output records only successful propagation");previous=output.pose;
    }
    require(!h.nativeExitActive()&&distance(previous,target(PoseHandoff::nativeExitSeconds))<.00001f,"full authored finish reaches exact live native before its ownership can end");
    const float epsilonEnd=.0005f,end=PoseHandoff::nativeExitSeconds;
    PoseHandoff terminal;terminal.consumed(terminal.evaluate(target(0),older,1,0,1-dt));terminal.consumed(terminal.evaluate(target(0),source,1,0,1));terminal.beginExit(false,true);
    terminal.advanceExitSource(end-epsilonEnd,library);const auto before=terminal.evaluate(target(end-epsilonEnd),source,1).pose;
    terminal.advanceExitSource(end,library);const auto after=terminal.evaluate(target(end),source,1).pose;
    require(((after[4].t-before[4].t)-(target(end)[4].t-target(end-epsilonEnd)[4].t)).length()/epsilonEnd<.15f,"full authored transition ends at the current native COM velocity");
    std::cout<<"AUTHORED_TOP_EXIT fps="<<fps<<" moving="<<moving<<" height="<<height<<" maxStep="<<maxStep<<" maxAngle="<<maxAngle<<" maxWrist="<<maxWrist<<'\n';
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
    require(envelope.elapsedExitSeconds()==0&&envelope.weight()==1&&h.nativeExitActive(),"half-second callback stall cannot consume the unseen native bridge");
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
    tick(dt);require(std::abs(envelope.elapsedExitSeconds()-before-.05f)<.00001f,"render resumption consumes at most fifty milliseconds of accumulated updates regardless of scene-pass count");
    for(int frame=0;frame<fps;++frame){
        auto shown=h.evaluate(nativeSurrogate(library,elapsed,true),authored,envelope.weight(),0,1+elapsed);require(h.consumed(shown),"fresh native-bridge output renders");++applied;tick(dt);
    }
    require(envelope.weight()==0&&!h.nativeExitActive(),"acknowledged sequence reaches exact native and retires normally");
    require(distance(h.evaluate(fixedNative,authored,envelope.weight()).pose,fixedNative)<.00001f,"retired runtime bridge is the current live native pose");

    PoseHandoff fall;const auto older=observedStanding(library,-dt,true);
    fall.consumed(fall.evaluate(stand,older,1,0,1-dt));fall.consumed(fall.evaluate(stand,stand,1,0,1));require(fall.beginExit(true),"physical departure retains separate continuation");
    fall.advanceExitSource(.04f,library,0);const auto moving=fall.evaluate(stand,stand,1).pose;
    require(distance(moving,stand)<.00001f,"physical continuation cannot advance beyond the first acknowledged output phase");
    fall.advanceExitSource(.10f,library,.04f);const auto resumedFall=fall.evaluate(stand,stand,1).pose;
    require((resumedFall[4].t-stand[4].t).length()>.8f,"physical continuation resumes with the same confirmed clock as its native blend");
}
static unsigned sparseRuntimeSchedule(const Library& library,int updateFps,unsigned updatesPerRender,unsigned passes){
    const float dt=1.f/updateFps,renderInterval=dt*updatesPerRender;
    require(renderInterval<=.050001f,"normal sparse-render fixture stays within the bounded recovery interval");
    const auto authored=library.sample(Motion::contextMantle,.98f),stand=observedStanding(library,0,true);
    PoseHandoff h;require(h.consumed(h.evaluate(observedStanding(library,-renderInterval,true),authored,1,1,1-renderInterval)),"sparse schedule records older displayed pose");
    require(h.consumed(h.evaluate(stand,authored,1,1,1))&&h.beginExit(false,true),"sparse schedule starts at the actual completed top");
    PoseBlendEnvelope envelope;std::uint32_t applied=0;envelope.beginExit(applied,PoseHandoff::nativeExitSeconds);
    h.advanceExitSource(0,library,envelope.elapsedExitSeconds());
    const auto first=h.evaluate(nativeSurrogate(library,0,true),authored,envelope.weight(),0,1);
    require(distance(first.pose,stand)<.00001f,"first sparse callback keeps the displayed exit origin");
    for(unsigned pass=0;pass<passes;++pass){require(h.consumed(first),"initial scene pass completes before its next player update");++applied;}
    float zeroTime=-1,terminalTime=-1;unsigned weighted=1;
    for(unsigned frame=1;frame<=unsigned(updateFps);++frame){
        const float elapsed=frame*dt;
        if(envelope.exitComplete(applied)){
            require(zeroTime>=PoseHandoff::nativeExitSeconds&&terminalTime>=zeroTime&&elapsed>terminalTime,"sparse cleanup follows both elapsed transition and a propagated native frame");
            require(zeroTime<=PoseHandoff::nativeExitSeconds+renderInterval+.00001f,"ordinary sparse rendering cannot stretch the native bridge by the update-to-render ratio");
            require(elapsed<=zeroTime+renderInterval+.00001f,"native terminal acknowledgement adds only the normal callback and cleanup updates");
            require(weighted>2&&!h.nativeExitActive(),"sparse schedule shows intermediate poses before returning to live native");
            std::cout<<"SPARSE_EXIT updateFps="<<updateFps<<" renderFps="<<updateFps/updatesPerRender<<" passes="<<passes<<" zeroMs="<<zeroTime*1000<<" nativeMs="<<terminalTime*1000<<" cleanupMs="<<elapsed*1000<<'\n';
            return frame;
        }
        const auto previous=envelope.elapsedExitSeconds();envelope.advanceExit(applied,dt);
        h.advanceExitSource(elapsed,library,envelope.elapsedExitSeconds());
        require(envelope.elapsedExitSeconds()>=previous&&envelope.elapsedExitSeconds()-previous<=.050001f,"sparse clock remains monotonic with bounded output steps");
        if(envelope.weight()==0&&zeroTime<0){zeroTime=elapsed;require(!envelope.exitComplete(applied),"zero-weight update cannot acknowledge its own native output");}
        if(frame%updatesPerRender!=0)continue;
        const auto native=nativeSurrogate(library,elapsed,true);const auto shown=h.evaluate(native,authored,envelope.weight(),0,1+elapsed);
        finitePose(shown.pose);
        if(envelope.weight()==0){require(distance(shown.pose,native)<.00001f,"sparse terminal output is the current live native pose for all ninety-nine bones");terminalTime=elapsed;}
        else ++weighted;
        for(unsigned pass=0;pass<passes;++pass){
            const auto duplicate=h.evaluate(native,authored,envelope.weight(),0,1+elapsed);
            require(distance(duplicate.pose,shown.pose)<.00001f&&h.consumed(duplicate),"duplicate scene passes render the same phase without adding simulation time");
            envelope.acknowledgeExit(envelope.weight()==0);++applied;
        }
    }
    require(false,"normal sparse callbacks must retire the native bridge within one second");return 0;
}
static void entrySeedContinuation(const Library& library,int fps){
    const float dt=1.f/fps;const auto current=observedStanding(library,0,true),older=observedStanding(library,-dt,true);
    auto authored=library.sample(Motion::jumpCatch,.2f),changedNative=nativeSurrogate(library,.3f,true);changedNative[4].t.x+=100;
    PoseHandoff h;const auto stale=h.evaluate(changedNative,authored,.4f);h.beginEntry(current,older,dt);
    require(!h.consumed(stale)&&!h.hasOutput(),"preflight seed resets prior ownership without claiming unrendered output");
    h.advanceEntrySource(1,library);const auto initial=h.evaluate(changedNative,authored,0,0,1);
    require(distance(initial.pose,current)<.00001f,"native graph changes after acquisition cannot replace the preflight entry origin");
    require(endpointDistance(library,changedNative,current)>20,"entry fixture includes a meaningful native ownership discontinuity");
    constexpr float epsilon=.0001f;h.advanceEntrySource(1+epsilon,library);
    const auto advanced=h.evaluate(changedNative,authored,0,0,1+epsilon);
    const auto velocity=(advanced.pose[4].t-current[4].t)/epsilon;
    require((velocity-Vec{35,0,0}).length()<.2f,"entry continues the observed COM velocity instead of freezing its native snapshot");
    const float angularSpeed=angleBetween(advanced.pose[36].q,current[36].q)/epsilon;
    require(angularSpeed>1.15f&&angularSpeed<1.65f,"entry continues observed head rotation from the older preflight sample");
    for(float time:{1+epsilon,1.f,.5f,-1.f,NAN,INFINITY}){
        h.advanceEntrySource(time,library);
        require(distance(h.evaluate(changedNative,authored,0).pose,advanced.pose)<.00001f,"repeated, old and invalid sample times cannot advance or rewind the entry source");
    }
    PoseBlendEnvelope envelope;envelope.beginEntry();std::uint32_t applied=0;
    h.beginEntry(current,older,dt);h.advanceEntrySource(10,library);
    for(unsigned frame=0;frame<unsigned(fps);++frame){
        const float time=10+frame*dt;h.advanceEntrySource(time,library);
        const auto shown=h.evaluate(changedNative,authored,envelope.weight(),0,time);finitePose(shown.pose);
        const auto repeat=h.evaluate(changedNative,authored,envelope.weight(),0,time);
        require(distance(shown.pose,repeat.pose)<.00001f,"same-time scene passes cannot recursively advance entry");
        require(h.consumed(shown)&&h.consumed(repeat),"entry source becomes display history only after successful propagation");
        applied+=2;envelope.advanceEntry(applied,dt);
    }
    require(envelope.weight()==1&&distance(h.evaluate(changedNative,authored,1).pose,authored)<.00001f,"entry reaches the unchanged authored pose at full weight");
    const auto frozen=h.evaluate(changedNative,authored,0).pose;h.advanceEntrySource(1000,library);
    require(distance(h.evaluate(changedNative,authored,0).pose,frozen)<.00001f,"completed entry stops advancing its continuation source");
    h.beginEntry(current,older,dt);h.advanceEntrySource(0,library);h.advanceEntrySource(.02f,library);
    const auto beforeExit=h.evaluate(changedNative,authored,.2f,0,.02f);require(h.consumed(beforeExit)&&h.beginExit(),"observed partial entry can leave normally");
    const auto exit=h.evaluate(changedNative,authored,1).pose;h.advanceEntrySource(10,library);
    require(distance(h.evaluate(changedNative,authored,1).pose,exit)<.00001f,"entry continuation cannot modify an active exit source");
    h.clear();h.advanceEntrySource(100,library);
    require(distance(h.evaluate(changedNative,authored,0).pose,changedNative)<.00001f,"cleared entry returns to native fallback without resurrecting a seed");
}
static void entrySeedValidation(const Library& library){
    const auto current=observedStanding(library,0,true),older=observedStanding(library,-.01f,true),native=nativeSurrogate(library,.3f,true);
    const auto authored=library.sample(Motion::jumpCatch,.2f);
    for(unsigned fault=0;fault<6;++fault){
        auto bad=current;
        if(fault==0)bad.clear();else if(fault==1)bad.pop_back();else if(fault==2)bad.push_back(bad.back());
        else if(fault==3)bad[4].t.x=NAN;else if(fault==4)bad[36].q={0,0,0,0};else bad[38].s.x=INFINITY;
        PoseHandoff h;h.beginEntry(bad,older,.01f);h.advanceEntrySource(0,library);
        require(distance(h.evaluate(native,authored,0).pose,native)<.00001f,"invalid current seed safely retains the old native fallback");
    }
    for(unsigned fault=0;fault<5;++fault){
        auto bad=older;if(fault==0)bad.clear();else if(fault==1)bad.pop_back();else if(fault==2)bad.push_back(bad.back());
        else if(fault==3)bad[4].t.x=NAN;else bad[36].q={0,0,0,0};
        PoseHandoff h;h.beginEntry(current,bad,.01f);h.advanceEntrySource(0,library);h.advanceEntrySource(.04f,library);
        require(distance(h.evaluate(native,authored,0).pose,current)<.0002f,"invalid older seed removes extrapolation without discarding valid current pose");
    }
    for(float interval:{0.f,-1.f,.000001f,NAN,INFINITY}){
        PoseHandoff h;h.beginEntry(current,older,interval);h.advanceEntrySource(0,library);h.advanceEntrySource(.04f,library);
        require(distance(h.evaluate(native,authored,0).pose,current)<.0002f,"unusable sample intervals produce a stable finite entry source");
    }
    auto nativeSeed=current;bool outsideClimbReference=false;
    for(const auto axis:{Vec{1,0,0},Vec{0,1,0},Vec{0,0,1}}){
        nativeSeed=current;nativeSeed[29].q=(Quat::axis(axis,3)*nativeSeed[29].q).unit();
        if(!library.armBendValid(nativeSeed,0)){outsideClimbReference=true;break;}
    }
    require(outsideClimbReference,"native seed fixture contains a valid transform outside the climbing elbow reference");
    PoseHandoff stationary;stationary.beginEntry(nativeSeed,{},0);stationary.advanceEntrySource(0,library);stationary.advanceEntrySource(.04f,library);
    require(distance(stationary.evaluate(native,authored,0).pose,nativeSeed)<.00001f,"zero-velocity native seed is not abruptly corrected into a different climbing elbow pose");
    auto fast=older;fast[4].t.x-=100000;fast[36].q=(Quat::axis({0,0,1},-2)*current[36].q).unit();
    PoseHandoff bounded;bounded.beginEntry(current,fast,.0001f);bounded.advanceEntrySource(0,library);bounded.advanceEntrySource(.01f,library);
    const auto near=bounded.evaluate(native,authored,0).pose;finitePose(near);
    require((near[4].t-current[4].t).length()<=4.0001f&&angleBetween(near[36].q,current[36].q)<=.1201f,"excessive entry velocities retain existing400 linear and12 angular speed caps");
    bounded.advanceEntrySource(100,library);const auto far=bounded.evaluate(native,authored,0).pose;finitePose(far);
    require((far[4].t-current[4].t).length()<=26.001f&&angleBetween(far[36].q,current[36].q)<=.7801f,"entry continuation decays to its existing finite travel bounds");
}
static void terminalCallbackSchedule(const Library& library,int fps,unsigned mode,unsigned schedule){
    const float dt=1.f/fps;const auto authored=library.sample(Motion::contextMantle,.98f),stand=observedStanding(library,0,true);
    PoseHandoff h;require(h.consumed(h.evaluate(observedStanding(library,-dt,true),authored,1,mode==0?1.f:0.f,1-dt)),"terminal fixture records older output");
    require(h.consumed(h.evaluate(stand,authored,1,mode==0?1.f:0.f,1)),"terminal fixture records current output");
    require(h.beginExit(mode==1,mode==0),"runtime schedule starts the selected exit");
    const float seconds=mode==0?PoseHandoff::nativeExitSeconds:mode==1?PoseBlendEnvelope::fallExitSeconds:PoseBlendEnvelope::exitSeconds;
    PoseBlendEnvelope envelope;std::uint32_t applied=17;envelope.beginExit(applied,seconds);
    bool cleared=false,terminalSeen=false;unsigned terminalCallbacks=0,terminalWaits=0,weightedCallbacks=0;float elapsed=0;
    for(unsigned frame=0;frame<unsigned(fps*5)&&!cleared;++frame){
        const float step=schedule==2&&frame%7==0?.2f:dt;
        elapsed+=std::min(step,.05f);
        if(envelope.exitComplete(applied)){
            require(terminalSeen&&terminalCallbacks>0,"cleanup requires a previously completed fully native callback");
            h.clear();envelope.clear();cleared=true;break;
        }
        const auto before=envelope.elapsedExitSeconds();envelope.advanceExit(applied,step);
        h.advanceExitSource(elapsed,library,envelope.elapsedExitSeconds());
        require(envelope.elapsedExitSeconds()-before<=.050001f,"long ticks cannot skip the bounded exit clock");
        if(envelope.weight()==0){
            terminalSeen=true;
            require(!envelope.exitComplete(applied),"reaching zero weight in tick cannot acknowledge its own undrawn terminal frame");
            if(terminalWaits++<unsigned(fps/4))continue;
        }
        if(schedule==1&&(frame<unsigned(fps/3)||frame%3==1))continue;
        const auto native=nativeSurrogate(library,elapsed,true);
        for(unsigned pass=0;pass<(schedule==2?7u:1u);++pass){
            if(envelope.weight()==0){
                const auto shown=h.evaluate(native,authored,0,0,1+elapsed);
                require(distance(shown.pose,native)<.00001f,"terminal frame is exactly the current native result before cleanup");
                ++terminalCallbacks;
            }else{
                const auto shown=h.evaluate(native,authored,envelope.weight(),0,1+elapsed);
                require(h.consumed(shown),"weighted exit output completes propagation before acknowledgement");++weightedCallbacks;
            }
            envelope.acknowledgeExit(envelope.weight()==0);++applied;
        }
        require(!cleared,"callbacks finish before the following tick can retire ownership");
    }
    require(cleared&&terminalCallbacks>0&&weightedCallbacks>0,"every frame-rate schedule exits only after terminal native display");
    require(terminalWaits>unsigned(fps/4),"absent terminal callbacks preserve ownership across repeated zero-weight ticks");
    require(!envelope.exitComplete(applied),"cleared ownership cannot retain a stale terminal acknowledgement");
    std::cout<<"TERMINAL_CALLBACK fps="<<fps<<" mode="<<mode<<" schedule="<<schedule<<" weighted="<<weightedCallbacks<<" terminal="<<terminalCallbacks<<'\n';
}
static void terminalEnvelopeBoundaries(){
    PoseBlendEnvelope e;
    require(!e.exitComplete(1),"idle envelope cannot claim exit completion");e.beginEntry();
    for(std::uint32_t output=1;output<60;++output)e.advanceEntry(output,1.f/60);
    require(e.weight()==1&&!e.exitComplete(60),"completed entry is not a completed exit");
    e.beginExit(0xfffffffeu,.05f);e.advanceExit(0xffffffffu,.05f);
    require(e.weight()==0&&!e.exitComplete(0xffffffffu),"last weighted acknowledgement cannot retire a newly reached terminal frame");
    for(int i=0;i<60;++i){e.advanceExit(0xffffffffu,.05f);require(!e.exitComplete(0xffffffffu),"missing terminal callbacks cannot be replaced by elapsed time");}
    require(!e.exitComplete(0),"counter rollover alone cannot acknowledge a native terminal frame");
    e.acknowledgeExit(true);require(e.exitComplete(0),"wrapping callback counter with native confirmation acknowledges the terminal frame");
    e.clear();require(!e.exitComplete(0),"clearing removes terminal confirmation");
    e.beginExit(20,.05f);e.advanceExit(21,NAN);e.advanceExit(21,0);e.advanceExit(21,-1);
    require(e.weight()==1&&!e.exitComplete(21),"invalid frame times cannot prematurely reach terminal exit");
    e.advanceExit(21,.05f);require(!e.exitComplete(21)&&!e.exitComplete(22),"late weighted output after terminal tick cannot permit cleanup without native confirmation");
    e.acknowledgeExit(true);require(!e.exitComplete(21)&&e.exitComplete(22),"native confirmation and a completed callback are both required");
    e.beginEntry();require(!e.exitComplete(22),"reattachment cannot consume an earlier exit confirmation");
}
static void lateCallbackConfirmation(){
    PoseBlendEnvelope envelope;std::uint32_t applied=20;unsigned pending=1;
    envelope.beginExit(applied,.05f);envelope.acknowledgeExit(true);
    ++applied;envelope.advanceExit(applied,.05f);
    require(envelope.weight()==0&&!envelope.exitComplete(applied+1),"a native callback from before terminal phase cannot preconfirm the endpoint");
    envelope.acknowledgeExit(false);++applied;--pending;
    require(!envelope.exitComplete(applied),"a positive-weight callback finishing after terminal tick cannot masquerade as native output");
    pending=1;envelope.acknowledgeExit(true);++applied;
    bool cleared=false;
    if(pending==0&&envelope.exitComplete(applied))cleared=true;
    require(!cleared&&envelope.exitComplete(applied),"terminal confirmation cannot clear ownership while an older scene pass is still pending");
    for(int tick=0;tick<10;++tick){envelope.advanceExit(applied,.02f);require(envelope.exitComplete(applied),"terminal updates cannot swallow a confirmed native callback while an old scene pass remains pending");}
    envelope.acknowledgeExit(false);++applied;--pending;
    require(pending==0&&!envelope.exitComplete(applied),"late positive-weight completion revokes terminal confirmation instead of retiring stale output");
    envelope.advanceExit(applied,.02f);
    require(!envelope.exitComplete(applied),"time and old output counts cannot restore revoked native confirmation");
    envelope.acknowledgeExit(true);++applied;
    if(pending==0&&envelope.exitComplete(applied))cleared=true;
    require(cleared,"a fresh terminal native pass after every old pass permits safe retirement");
    envelope.beginExit(applied,.05f);++applied;envelope.advanceExit(applied,.05f);
    require(!envelope.exitComplete(applied+1),"a later exit cannot reuse the previous native terminal acknowledgement");
    pending=1;envelope.acknowledgeExit(true);++applied;
    for(int tick=0;tick<10;++tick)envelope.advanceExit(applied,.02f);
    --pending;
    require(pending==0&&envelope.exitComplete(applied),"draining a pending scene pass without new output retains the already confirmed native endpoint");
}
int main(int argc,char** argv){try{
    Library library;require(argc>=2&&argc<=3&&library.load(argv[1]),"load current runtime motion library");
    if(argc==3) {
        const auto overrides=fc::loadHkxOverrides(library,argv[2]);
        require(overrides.loaded==activeMotionCount&&overrides.rejected==0&&overrides.missing==0,
            "load every active HKX slot without missing or rejected clips");
        for(Motion motion:activeMotions)require(library.hasAnimationOverride(motion),"every active slot installs its HKX override");
    }
    singleLayerWeight(library);
    for(int fps:{30,40,60,120})for(bool moving:{false,true})for(float height:{45.f,80.f,120.f})completeAuthoredTakeover(library,fps,moving,height);
    for(bool aligned:{false,true})for(int fps:{30,60,120})for(float recovery:{.92f,.999f,1.f})dynamicTakeover(library,fps,recovery,aligned);
    terminalDerivativeAndTarget(library);optInAndRevisionContracts(library);
    for(int fps:{30,40,60,120})acknowledgedRuntimeSchedule(library,fps);
    for(int fps:{40,60,120})for(unsigned interval:{2u,3u,4u})if(float(interval)/fps<=.050001f)
        require(sparseRuntimeSchedule(library,fps,interval,1)==sparseRuntimeSchedule(library,fps,interval,7),"same-frame scene-pass multiplicity cannot change native exit duration");
    entrySeedValidation(library);
    for(int fps:{30,40,60,120})entrySeedContinuation(library,fps);
    terminalEnvelopeBoundaries();lateCallbackConfirmation();
    for(int fps:{30,40,60,120})for(unsigned mode=0;mode<3;++mode)for(unsigned schedule=0;schedule<3;++schedule)terminalCallbackSchedule(library,fps,mode,schedule);
    std::cout<<"PASS completed top native takeover: actual99 bones, initial/final velocity, dynamic endpoint, delayed callbacks and legacy contracts\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
