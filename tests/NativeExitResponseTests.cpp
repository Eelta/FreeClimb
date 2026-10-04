#include "PoseHandoff.h"
#include "PoseBlendEnvelope.h"
#include "PoseFrameClock.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
void require(bool value,const char* text){if(!value)throw std::runtime_error(text);}
struct ResponseLedge:World {
    std::optional<Hit> ray(Vec a,Vec b) override {
        std::optional<Hit> hit;float nearest=2;
        if(a.y<0&&b.y>=0){float t=-a.y/(b.y-a.y);Vec p=a+(b-a)*t;if(p.z<=130.88f&&t<nearest){nearest=t;hit=Hit{p,{0,-1,0},true};}}
        if(a.z>130.88f&&b.z<=130.88f){float t=(130.88f-a.z)/(b.z-a.z);Vec p=a+(b-a)*t;if(p.y>=0&&t<nearest)hit=Hit{p,{0,0,1},true};}
        return hit;
    }
};
float poseAngle(const Pose& a,const Pose& b){float peak=0;for(unsigned i=0;i<a.size();++i)peak=std::max(peak,angleBetween(a[i].q,b[i].q));return peak;}
float endpointDistance(const Library& l,const Pose& a,const Pose& b){auto x=l.world(a),y=l.world(b);float peak=0;for(int bone:{4,8,11,26,36,38,39,75,90})peak=std::max(peak,(x[bone].t-y[bone].t).length());return peak;}
std::array<Pose,2> finish(const Library& l,int fps){
    ResponseLedge world;Traversal t;SurfacePose surface;l.configureThreepeat(t.cfg);t.cfg.gap=37;t.cfg.radius=31;
    require(t.attach(world,{0,-42,0},{0,1,0},100),"checked response fixture attaches");std::array<Pose,2> frames;
    for(int frame=0;frame<8*fps&&t.active();++frame){auto result=t.update(world,{0,1,false,true},1.f/fps,100);frames[0]=frames[1];frames[1]=surface.update(l,world,t,result.motion,1.f/fps,1);if(result.completed)return frames;}
    throw std::runtime_error("complete checked mantle required");
}
struct Frame {unsigned wall,stamp,applied;float seconds;};
constexpr Frame first[]{{16,213686,197,0},{78,213717,198,.031f},{203,213860,216,.081f},{234,213894,217,.115f},{266,213924,218,.145f},{297,213954,219,.175f},{328,213985,220,.206f},{359,214018,221,.239f},{375,214047,222,.24f}};
constexpr Frame second[]{{15,232870,202,0},{62,232898,203,.028f},{187,233040,221,.078f},{218,233070,222,.108f},{250,233099,223,.137f},{281,233156,224,.187f},{343,233190,226,.221f},{375,233221,227,.24f}};
constexpr Frame physicalFirst[]{{16,220098,130,0},{63,220123,131,.025f},{188,220261,149,.075f},{203,220286,150,.1f},{235,220312,151,.126f},{250,220336,152,.15f},{282,220363,153,.16f}};
constexpr Frame physicalSecond[]{{16,225862,140,0},{62,225888,141,.026f},{187,226022,159,.076f},{203,226047,160,.101f},{234,226071,161,.125f},{250,226095,162,.149f},{281,226119,163,.16f}};
constexpr Frame syntheticDrop[]{{0,0,0,0},{20,20,1,.02f},{40,40,2,.04f},{60,60,3,.06f},{80,80,4,.08f},{100,100,5,.1f},{120,120,6,.12f},{140,140,7,.14f},{160,160,8,.16f}};
struct ResponseWall:World {
    const Library& library;
    explicit ResponseWall(const Library& value):library(value){}
    std::optional<Hit> ray(Vec a,Vec b)override{if(a.y>=0||b.y<0)return {};return Hit{a+(b-a)*(-a.y/(b.y-a.y)),{0,-1,0},true};}
    bool actionBodyClear(Motion motion,Vec from,Vec to,float begin,float end,Vec normal)override{return motion==Motion::backFlipOut&&backFlipBodyClear(*this,library,from,to,begin,end,normal,0,37,1);}
};
std::array<Pose,2> physicalFinish(const Library& l,int kind,int fps){
    ResponseWall world(l);Traversal t;SurfacePose surface;std::array<Pose,2> captured;
    t.cfg.approachSeconds=0;t.cfg.fancyJumps=kind==1;t.cfg.contextActions=false;t.cfg.radius=31;t.cfg.gap=37;t.cfg.height=138;
    require(t.attach(world,{0,-37,1000},{0,1,0},1000),"physical response fixture starts from supported attachment");float dt=1.f/fps;
    for(int frame=0;frame<fps;++frame){auto r=t.update(world,{},dt,1000);captured[0]=captured[1];captured[1]=surface.update(l,world,t,r.motion,dt,1);}
    const Vec start=t.position;
    for(int frame=0;frame<2*fps;++frame){Input input;if(frame==0){input.release=true;input.backDrop=kind!=2;if(kind!=2)input.y=-1;}
        auto r=t.update(world,input,dt,1000);const Motion expected=kind==2?Motion::drop:kind==1?Motion::backFlipOut:Motion::dropBack;
        require(r.motion==expected,"physical source preserves the actual action identity");auto p=surface.update(l,world,t,r.motion,dt,1);
        if(kind==2)require((t.position-start).length()<.003f&&r.releaseVelocity.length()==0,"in-place drop never acquires displacement or an outward impulse");
        if(r.released){require(t.actionProgress()==1&&!r.completed&&captured[0].size()==99&&captured[1].size()==99,"physical release follows complete checked action with two consumed sources");if(kind!=2)require(r.releaseVelocity.length()>100,"outward departure retains the checked physical impulse");return captured;}
        captured[0]=captured[1];captured[1]=std::move(p);
    }
    throw std::runtime_error("complete physical departure required");
}
Pose fallingTarget(const Library& l,float time){auto p=l.sample(Motion::drop,1);p[0]=l.rest[0];p[4]=l.rest[4];p[4].t=p[4].t+Vec{3*time,-2*time,2*std::sin(5*time)};p[6].q=(Quat::axis({1,0,0},.18f*std::sin(7*time))*p[6].q).unit();p[9].q=(Quat::axis({1,0,0},-.18f*std::sin(7*time))*p[9].q).unit();p[36].q=(Quat::axis({0,0,1},.7f*time)*p[36].q).unit();return p;}
template<std::size_t N> void physicalCadence(const Library& l,const Frame (&frames)[N],int fps,int kind){
    const auto captured=physicalFinish(l,kind,fps);const auto& source=captured[1];PoseHandoff h;h.consumed(h.evaluate(l.rest,captured[0],1,0,1-1.f/fps));h.consumed(h.evaluate(l.rest,source,1,0,1));require(h.beginExit(true),"actual physical source starts continuation");
    PoseContinuation continuation;continuation.begin(source,captured[0],1.f/fps);PoseBlendEnvelope envelope;PoseFrameClock clock;envelope.beginExit(frames[0].applied,.16f,true);
    auto previous=source,oldPrevious=source;float peakAngle=0,peakSpeed=0,oldPeakAngle=0,oldPeakSpeed=0;
    for(std::size_t i=0;i<N;++i){const auto& frame=frames[i];float dt=clock.sample(frame.stamp,.03f,.03f,false);envelope.advanceExit(frame.applied,dt);float phase=envelope.elapsedExitSeconds();require(std::abs(phase-frame.seconds)<.000001f,"physical response keeps recorded cadence and acknowledged clock cap");
        float time=float(frame.wall-frames[0].wall)*.001f;auto native=fallingTarget(l,time);h.advanceExitSource(time,l,phase);auto out=h.evaluate(native,source,envelope.weight());auto old=h.evaluate(native,source,1-smooth(phase/.16f));
        auto expected=continuation.sample(phase);l.guardArmBends(expected);require(poseAngle(out.source,expected)<.0001f&&endpointDistance(l,out.source,expected)<.001f,"physical continuation retains the acknowledged source velocity");
        require(l.armBendValid(out.source,0)&&l.armBendValid(out.source,1),"both physical source elbow guards remain active");
        for(unsigned bone=0;bone<out.pose.size();++bone){const auto& value=out.pose[bone];require(value.t.finite()&&value.s.finite()&&std::abs(value.q.dot(value.q)-1)<.002f,"physical handoff stays finite and normalized");if((source[bone].t-native[bone].t).length()<.0001f&&(source[bone].t-captured[0][bone].t).length()<.0001f)require((value.t-native[bone].t).length()<.0001f,"static shared structural offsets stay fixed throughout physical handoff");}
        if(i){float elapsed=phase-frames[i-1].seconds;peakAngle=std::max(peakAngle,poseAngle(out.pose,previous)/elapsed);peakSpeed=std::max(peakSpeed,endpointDistance(l,out.pose,previous)/elapsed);oldPeakAngle=std::max(oldPeakAngle,poseAngle(old.pose,oldPrevious)/elapsed);oldPeakSpeed=std::max(oldPeakSpeed,endpointDistance(l,old.pose,oldPrevious)/elapsed);}
        else require(poseAngle(out.pose,source)<.0001f&&endpointDistance(l,out.pose,source)<.001f,"first physical callback does not advance the source");
        require(h.consumed(out),"physical source uses acknowledged outputs");previous=out.pose;oldPrevious=old.pose;
        if(i==1){float oldRecovery=smooth(phase/.16f);require(out.recovery>oldRecovery*2&&out.recovery<.12f,"physical short shoulder has earlier bounded target response");std::cout<<"PHYSICAL_EARLY motion="<<kind<<" seconds="<<phase<<" old="<<oldRecovery<<" current="<<out.recovery<<'\n';}
        if(i+1==N){require(poseAngle(out.pose,native)<.00001f&&endpointDistance(l,out.pose,native)<.0001f,"physical bridge ends exactly at live falling pose");require(!envelope.exitComplete(frame.applied+1),"physical elapsed phase alone cannot clean up");envelope.acknowledgeExit(true);require(envelope.exitComplete(frame.applied+1),"physical cleanup requires native acknowledgement");}
    }
    std::cout<<"PHYSICAL_RESPONSE kind="<<kind<<" fps="<<fps<<" samples="<<N<<" peakAngular="<<peakAngle<<" oldAngular="<<oldPeakAngle<<" peakEndpoint="<<peakSpeed<<" oldEndpoint="<<oldPeakSpeed<<'\n';
    PoseHandoff boundary;boundary.consumed(boundary.evaluate(l.rest,captured[0],1,0,1-1.f/fps));boundary.consumed(boundary.evaluate(l.rest,source,1,0,1));boundary.beginExit(true);
    constexpr float epsilon=.0001f;auto firstNative=fallingTarget(l,epsilon);boundary.advanceExitSource(epsilon,l,epsilon);auto initial=boundary.evaluate(firstNative,source,1-PoseBlendEnvelope::exitRecovery(epsilon/.16f));auto expectedFirst=continuation.sample(epsilon);l.guardArmBends(expectedFirst);
    require((initial.pose[4].t-expectedFirst[4].t).length()/epsilon<.15f,"physical blend retains outgoing COM velocity at the boundary");
    constexpr float beforeEnd=.1595f;boundary.advanceExitSource(beforeEnd,l,beforeEnd);auto targetBefore=fallingTarget(l,beforeEnd);auto before=boundary.evaluate(targetBefore,source,1-PoseBlendEnvelope::exitRecovery(beforeEnd/.16f));boundary.advanceExitSource(.16f,l,.16f);auto targetEnd=fallingTarget(l,.16f);auto after=boundary.evaluate(targetEnd,source,0);
    require(((after.pose[4].t-before.pose[4].t)-(targetEnd[4].t-targetBefore[4].t)).length()/.0005f<.15f,"physical blend retains live target COM velocity at its endpoint");
}
template<std::size_t N> void observedCadence(const Library& l,const Frame (&frames)[N],int fps){
    const auto captured=finish(l,fps);const auto& source=captured[1];PoseHandoff h;
    require(h.consumed(h.evaluate(l.rest,captured[0],1,0,1-1.f/fps))&&h.consumed(h.evaluate(l.rest,source,1,0,1))&&h.beginExit(false,true),"actual final output starts native response");
    PoseBlendEnvelope envelope;PoseFrameClock clock;envelope.beginExit(frames[0].applied,.24f,true);
    auto previous=source;float peakAngle=0,peakSpeed=0;
    for(std::size_t i=0;i<N;++i){
        const auto& frame=frames[i];float dt=clock.sample(frame.stamp,.03f,.03f,false);envelope.advanceExit(frame.applied,dt);
        require(std::abs(envelope.elapsedExitSeconds()-frame.seconds)<.000001f,"logged scene cadence preserves acknowledged phase and cap");
        const float time=float(frame.wall-frames[0].wall)*.001f,rise=float(frames[1].wall-frames[0].wall)*.001f,fall=float(frames[2].wall-frames[1].wall)*.001f;
        auto native=l.rest;native[4].t.x+=10*time;native[36].q=(Quat::axis({0,0,1},.4f*time)*native[36].q).unit();
        const float angle=.843f*smooth(time/rise)-.812f*smooth((time-rise)/fall);native[31].q=(Quat::axis({1,0,0},angle)*native[31].q).unit();
        h.advanceExitSource(time,l,envelope.elapsedExitSeconds());const auto output=h.evaluate(native,source,envelope.weight());
        require(std::abs(output.recovery+envelope.weight()-1)<.000001f,"reported weight equals the single completed-native response");
        if(i){float elapsed=frame.seconds-frames[i-1].seconds;const float rotation=poseAngle(output.pose,previous),distance=endpointDistance(l,output.pose,previous);peakAngle=std::max(peakAngle,rotation/elapsed);peakSpeed=std::max(peakSpeed,distance/elapsed);
            require(rotation<=12.566371f*elapsed+.005f&&distance<=750*elapsed+.6f,"observed cadence surrogate retains existing output rate bounds");}
        else require(poseAngle(output.pose,source)<.00001f&&endpointDistance(l,output.pose,source)<.0001f,"first unconfirmed callback has zero source advance");
        require(h.consumed(output),"only acknowledged output feeds source history");previous=output.pose;
        if(i==1){float old=smooth(frame.seconds/.24f);require(output.recovery>old*3&&output.recovery<.09f,"short shoulder restores earlier response without full target gain");std::cout<<"EARLY_RESPONSE wall="<<frame.wall<<" seconds="<<frame.seconds<<" old="<<old<<" current="<<output.recovery<<'\n';}
        if(i+1==N){require(poseAngle(output.pose,native)<.00001f&&endpointDistance(l,output.pose,native)<.0001f,"terminal output is exactly live native");require(!envelope.exitComplete(frame.applied+1),"elapsed phase alone cannot clean up");envelope.acknowledgeExit(true);require(envelope.exitComplete(frame.applied+1),"validated terminal callback permits cleanup");}
    }
    std::cout<<"CADENCE_RESPONSE fps="<<fps<<" samples="<<N<<" peakAngular="<<peakAngle<<" peakEndpoint="<<peakSpeed<<'\n';
}
void profileContracts(const Library& l){
    constexpr float epsilon=.0001f;float previous=0;
    for(int i=0;i<=1000;++i){float x=i/1000.f,r=PoseBlendEnvelope::exitRecovery(x);require(r>=previous&&r>=0&&r<=1,"native response is bounded and monotone");previous=r;}
    require(PoseBlendEnvelope::exitRecovery(0)==0&&PoseBlendEnvelope::exitRecovery(1)==1&&PoseBlendEnvelope::exitRecovery(.5f)==.5f,"native response retains exact endpoints and one-layer midpoint");
    require(PoseBlendEnvelope::exitRecovery(epsilon)/epsilon<.00001f&&(1-PoseBlendEnvelope::exitRecovery(1-epsilon))/epsilon<.00001f,"native response starts and ends with zero blend derivative");
    PoseHandoff h;auto source=l.rest,older=source;older[4].t.x-=20*.02f;
    h.consumed(h.evaluate(l.rest,older,1,0,.98f));h.consumed(h.evaluate(l.rest,source,1,0,1));require(h.beginExit(false,true),"moving source starts native bridge");
    auto native=source;native[4].t.x+=30;h.advanceExitSource(epsilon,l,epsilon);auto start=h.evaluate(native,source,1).pose;
    require(std::abs((start[4].t.x-source[4].t.x)/epsilon-20)<.15f,"native response preserves captured initial COM velocity");
    h.advanceExitSource(.24f-.0005f,l,.24f-.0005f);native[4].t.x+=8*(.24f-.0005f);auto before=h.evaluate(native,source,1).pose;
    native[4].t.x+=8*.0005f;h.advanceExitSource(.24f,l,.24f);auto end=h.evaluate(native,source,1).pose;
    require(std::abs((end[4].t.x-before[4].t.x)/.0005f-8)<.15f,"native response preserves terminal live COM velocity");
    for(float seconds:{.16f,.28f}){PoseBlendEnvelope e;e.beginExit(10,seconds);e.advanceExit(11,.028f);require(std::abs(e.weight()-(1-smooth(.028f/seconds)))<.000001f,"non-opted exits retain their original curve");}
    PoseBlendEnvelope entry;entry.beginEntry();entry.advanceEntry(1,.028f);require(std::abs(entry.weight()-smooth(.028f/.18f))<.000001f,"entry retains its original curve");
}
int main(int argc,char** argv){try{Library l;require(argc==2&&l.load(argv[1]),"load actual animation pack");profileContracts(l);for(int fps:{20,30,40,60,120}){observedCadence(l,first,fps);observedCadence(l,second,fps);for(int kind:{0,1,2}){physicalCadence(l,physicalFirst,fps,kind);physicalCadence(l,physicalSecond,fps,kind);if(kind==2)physicalCadence(l,syntheticDrop,fps,kind);}}std::cout<<"PASS exit response: actual top, push, flip and drop sources; four logged cadences and synthetic in-place cadence; controlled native targets; explicit acknowledgement\n";return 0;}catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}}
