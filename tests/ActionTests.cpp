#include "Pose.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
void check(bool b,const char* message){if(!b)throw std::runtime_error(message);}
struct Facets:World {
    float angle=.57735f;
    std::optional<Hit> ray(Vec a,Vec b) override {
        std::optional<Hit> out;float best=2;
        for(int side=0;side<2;++side) {
            Vec normal=side?Vec{angle,-1,0}.unit():Vec{0,-1,0};
            float from=a.dot(normal),to=b.dot(normal);
            if(from<=0||to>=0)continue;
            float t=from/(from-to);Vec p=a+(b-a)*t;
            if((side?p.x>=0:p.x<=0)&&t<best){out=Hit{p,normal,true};best=t;}
        }
        return out;
    }
};
struct Slab:World {
    float bottom=80,top=120;
    std::optional<Hit> ray(Vec a,Vec b) override {
        std::optional<Hit> out;float best=2;
        if(a.y<0&&b.y>=0) {
            float t=-a.y/(b.y-a.y);auto p=a+(b-a)*t;
            if(p.z>=bottom&&p.z<=top){out=Hit{p,{0,-1,0},true};best=t;}
        }
        if(a.z>top&&b.z<=top) {
            float t=(top-a.z)/(b.z-a.z);auto p=a+(b-a)*t;
            if(p.y>=0&&t<best)out=Hit{p,{0,0,1},true};
        }
        return out;
    }
};

struct Exterior:World {
    Facets facets;
    Vec origin{110190,77709,2509};
    float yaw{};
    Vec rotate(Vec v,float angle)const {return {v.x*std::cos(angle)-v.y*std::sin(angle),v.x*std::sin(angle)+v.y*std::cos(angle),v.z};}
    Vec point(Vec local)const{return origin+rotate(local,yaw);}
    std::optional<Hit> ray(Vec a,Vec b) override {
        auto h=facets.ray(rotate(a-origin,-yaw),rotate(b-origin,-yaw));
        if(h){h->point=point(h->point);h->normal=rotate(h->normal,yaw);}return h;
    }
};
struct Mountain:World {
    Vec origin{111874,77319,2183};
    std::optional<Hit> ray(Vec a,Vec b) override {
        a=a-origin;b=b-origin;
        struct Segment{float low,high,y,slope;};
        std::optional<Hit> out;float best=2;
        for(auto s:{Segment{-1000,90,-400,.4f},Segment{90,180,36,-.1f},Segment{180,1000,27,.7f}}) {
            Vec n=Vec{0,-1,s.slope}.unit(),base{0,s.y,s.low};
            float from=(a-base).dot(n),to=(b-base).dot(n);
            if(from<=0||to>=0)continue;
            const float t=from/(from-to);const Vec p=a+(b-a)*t;
            if(p.z>=s.low&&p.z<=s.high&&t<best){out=Hit{p+origin,n,true};best=t;}
        }
        return out;
    }
};
struct JumpWall:World {
    std::optional<float> rear;
    std::optional<Hit> ray(Vec a,Vec b) override {
        std::optional<Hit> result;float nearest=2;
        if(a.y<0&&b.y>=0) {const float phase=-a.y/(b.y-a.y);nearest=phase;result=Hit{a+(b-a)*phase,{0,-1,0},true};}
        if(rear&&a.y>*rear&&b.y<=*rear) {
            const float phase=(*rear-a.y)/(b.y-a.y);
            if(phase<nearest)result=Hit{a+(b-a)*phase,{0,1,0},true};
        }
        return result;
    }
};
static void normalClimbingJumps() {
    int cases=0;float worstCompletionExcess=0;
    for(int fps:{20,30,48,60,120})
    for(Vec direction:{Vec{0,1,0},Vec{-1,0,0},Vec{1,0,0},Vec{-1,1,0},Vec{1,1,0}}) {
        JumpWall wall;Traversal t;t.cfg.approachSeconds=0;
        check(t.attach(wall,{0,-30,1000},{0,1,0},100),"ordinary action attach");
        const float dt=1.f/fps;Input moving{direction.x,direction.y};Result r;
        for(int frame=0;frame<fps;++frame)r=t.update(wall,moving,dt,100);
        const auto start=t.position;auto launch=moving;launch.hop=true;r=t.update(wall,launch,dt,100);
        check(t.state==State::action&&(r.motion==Motion::hopUp||r.motion==Motion::hopLeft||r.motion==Motion::hopRight)&&!t.runningAction(),
            "ordinary climbing retains its checked directional hop after running jumps are removed");
        const Motion action=r.motion;const float duration=t.actionDuration();
        check(duration>=.50f&&duration<=.55f&&duration==jumpActionSeconds(false),
            "ordinary hop retains its independent half-second playback window");
        float elapsed=0,cost=r.staminaCost,maxOutward=0;
        while(t.state==State::action&&elapsed<1.f) {
            const auto before=t.position;const float beforePhase=t.actionProgress();
            const auto duplicate=t.update(wall,moving,0,100);
            check(duplicate.motion==action&&duplicate.staminaCost==0&&t.actionProgress()==beforePhase&&(t.position-before).length()==0,
                "zero-time callbacks never advance a hop source or its collision path");
            r=t.update(wall,moving,dt,100);elapsed+=dt;cost+=r.staminaCost;
            check(t.active()&&!r.released&&r.motion==action,"ordinary hop retains supported ownership through landing");
            check(std::abs(t.actionProgress()-std::min(1.f,elapsed/duration))<.00002f,
                "published hop progress follows the same clock as collision-checked displacement");
            maxOutward=std::max(maxOutward,(t.position-start).dot(t.normal));
        }
        worstCompletionExcess=std::max(worstCompletionExcess,elapsed-duration);
        check(t.state!=State::action&&elapsed>=duration-.0001f&&elapsed-duration<=dt+.0001f&&std::abs(cost-15)<.001f,
            "ordinary hop completes within one update and charges its fifteen stamina exactly once");
        check(maxOutward>t.cfg.hopOut*.94f,"ordinary hop retains its visible checked outward arc");
        check(std::abs((t.position-start).dot(t.normal))<.01f,"ordinary hop returns to the verified wall");
        const auto landing=t.position;r=t.update(wall,moving,dt,100);
        check(!runMotion(r.motion)&&(t.position-landing).length()>.1f,
            "first landing update immediately resumes held ordinary climbing");
        ++cases;
    }
    std::cout<<"ordinary actions cases="<<cases<<" worstFrameQuantization="<<worstCompletionExcess<<'\n';
}
static void intermediateHopObstacle() {
    for(int fps:{20,30,48,60,120}) {
        JumpWall open;Traversal t;t.cfg.approachSeconds=0;
        check(t.attach(open,{0,-30,1000},{0,1,0},100),"intermediate hop clearance attach");
        const float dt=1.f/fps;const Input moving{0,1};
        for(int frame=0;frame<fps;++frame)t.update(open,moving,dt,100);
        auto jump=moving;jump.hop=true;const auto started=t.update(open,jump,dt,100);
        check(started.motion==Motion::hopUp&&started.staminaCost==15,"intermediate obstacle follows a valid full-hop preflight");
        const float duration=t.actionDuration();const int beforePeak=int(std::floor(.5f*duration/dt));
        for(int frame=0;frame<beforePeak;++frame)t.update(open,moving,dt,100);
        const Vec before=t.position;const float phase=t.actionProgress();
        check(phase<.5f&&phase+dt/duration>.5f,"fixture straddles the hop's outward maximum");
        auto atPeak=t;atPeak.update(open,moving,(.5f-phase)*duration,100);
        auto atEnd=t;atEnd.update(open,moving,dt,100);
        const float depth=t.cfg.radius;
        const float endpointBack=std::min(before.y,atEnd.position.y)-depth,peakBack=atPeak.position.y-depth;
        check(peakBack<endpointBack-.001f,"both frame endpoints are clear of the intermediate rear plane");
        JumpWall changed;changed.rear=(endpointBack+peakBack)*.5f;
        check(before.y-depth>*changed.rear&&atEnd.position.y-depth>*changed.rear,
            "negative control: checking only the endpoint capsules would approve the obstructed curve");
        const auto blocked=t.update(changed,moving,dt,100);
        check(blocked.released&&!t.active()&&std::string(blocked.reason)=="jump path changed"&&blocked.staminaCost==0,
            "ordinary hop still catches a new obstruction at an intermediate arc knot");
        check((t.position-before).length()<.0001f&&t.actionProgress()==phase,
            "neither movement nor source progress commits before the entire hop step is clear");
        std::cout<<"intermediate rear obstacle fps="<<fps<<" phase="<<phase<<"->"<<atEnd.actionProgress()<<" clearanceGap="<<endpointBack-peakBack<<'\n';
    }
}
static void continuousBackPush() {
    for(int fps:{20,30,48,60,120}) {
        JumpWall wall;Traversal t;t.cfg.approachSeconds=0;
        check(t.attach(wall,{0,-30,1000},{0,1,0},100),"continuous push attach");
        const Vec start=t.position,velocity=t.normal*220+Vec{0,0,90};
        const float dt=1.f/fps;float elapsed=0,previousSpeed=0;Vec before=t.position;Result result;
        for(int frame=0;frame<fps&&t.active();++frame) {
            const Input input=frame==0?Input{0,-1,true,false,false,true}:Input{};
            before=t.position;result=t.update(wall,input,dt,100);elapsed+=dt;
            const float phase=std::min(1.f,elapsed/.32f);
            const float travelTime=.32f/3*(phase*phase*phase)+std::max(0.f,elapsed-.32f);
            check((t.position-(start+velocity*travelTime)).length()<.003f,
                "checked push follows one accelerating path through the final update's subframe remainder");
            const float speed=(t.position-before).dot(velocity.unit())/dt;
            check(speed+.02f>=previousSpeed,"push accelerates into physical release instead of braking before its impulse");
            previousSpeed=speed;
            if(t.active()) {
                const Vec held=t.position;const float heldPhase=t.actionProgress();
                const auto duplicate=t.update(wall,{},0,100);
                check(!duplicate.released&&duplicate.releaseVelocity.length()==0&&t.actionProgress()==heldPhase&&(t.position-held).length()==0,
                    "duplicate callbacks cannot advance or emit the physical release early");
                check(result.releaseVelocity.length()==0,"only the completed push emits physical velocity");
            }
        }
        check(result.released&&!t.active()&&std::abs(t.actionProgress()-1)<.00001f&&elapsed>=.32f-.0001f&&elapsed-.32f<=dt+.0001f,
            "push source reaches its terminal phase in one action, within one update of .32 seconds");
        check((result.releaseVelocity-velocity).length()<.002f,"open departure retains the same checked terminal velocity");
        check(previousSpeed/velocity.length()>.90f&&previousSpeed/velocity.length()<1.001f,
            "actual final displayed position step is close to physical speed even when duration falls between updates");
        check(t.update(wall,{},dt,100).releaseVelocity.length()==0,"free fall does not replay the departure impulse");
        std::cout<<"continuous back push fps="<<fps<<" elapsed="<<elapsed<<" terminalStepRatio="<<previousSpeed/velocity.length()<<'\n';
    }
}

struct ExitWall:World {
    Vec outward{0,-1,0};bool approveSourceSweep=true;
    std::optional<float> rearDistance,rejectAfterPhase;
    unsigned bodyChecks{};float lastFromPhase{},lastToPhase{};
    std::optional<Hit> ray(Vec from,Vec to) override {
        const float a=from.dot(outward),b=to.dot(outward);
        if(a>0&&b<=0) {const float p=a/(a-b);return Hit{from+(to-from)*p,outward,true};}
        if(rearDistance&&a<*rearDistance&&b>=*rearDistance) {
            const float p=(*rearDistance-a)/(b-a);return Hit{from+(to-from)*p,outward*-1,false};
        }
        return {};
    }
    bool actionBodyClear(Motion motion,Vec from,Vec to,float a,float b,Vec normal) override {
        ++bodyChecks;lastFromPhase=a;lastToPhase=b;
        check(motion==Motion::backFlipOut&&a>=0&&b>=a&&b<=1&&b-a<=1.f/12+.00001f,
            "source-sweep callback receives bounded consecutive source intervals only for the dedicated exit");
        check(from.finite()&&to.finite()&&(normal-outward).length()<.0001f,
            "source-sweep callback follows the actual latched outward wall frame");
        return approveSourceSweep&&(!rejectAfterPhase||b<*rejectAfterPhase);
    }
};
static void checkedBackFlipExit() {
    const Input release{0,-1,true,false,false,true};
    for(int fps:{20,30,48,60,120})for(float yaw:{0.f,.8f,2.2f}) {
        ExitWall wall;wall.outward={std::sin(yaw),-std::cos(yaw),0};
        Traversal t;t.cfg.approachSeconds=0;
        check(t.attach(wall,wall.outward*30+Vec{0,0,1000},wall.outward*-1,100),"back flip attach actual yaw");
        const Vec start=t.position;const float dt=1.f/fps;float elapsed=0,maxLift=0;Vec terminalStep{};Result result;
        for(int frame=0;frame<fps*2&&t.active();++frame) {
            const Vec before=t.position;const unsigned checksBefore=wall.bodyChecks;
            result=t.update(wall,release,dt,100);elapsed+=dt;
            check(result.motion==Motion::backFlipOut&&t.actionDuration()>=.70f&&t.actionDuration()<=.78f,
                "S+Space selects the dedicated finite outward somersault only after runtime source approval");
            check(wall.bodyChecks-checksBefore<=(frame==0?14u:2u),
                "source-body clearance is preflighted once then bounded to crossed knots per update");
            check((t.position-backFlipExitPoint(start,wall.outward,elapsed/backFlipExitSeconds)).length()<.003f,
                "back flip body path and pose phase share the same complete clock including terminal remainder");
            check((t.position-before).dot(wall.outward)>0,"outward somersault never returns toward the wall or stalls at its peak");
            check(result.staminaCost==0,"manual detach does not charge a new wall-grab jump cost");
            maxLift=std::max(maxLift,t.position.z-start.z);terminalStep=(t.position-before)/dt;
            if(t.active()) {
                check(result.releaseVelocity.length()==0,"the complete source flip stays owned instead of being abandoned midair");
                const auto held=t.position;const auto phase=t.actionProgress();const auto checks=wall.bodyChecks;
                t.update(wall,{},0,100);
                check((t.position-held).length()==0&&phase==t.actionProgress()&&checks==wall.bodyChecks,
                    "duplicate callbacks neither advance nor rescan an already checked flip");
            }
        }
        const Vec expectedVelocity=wall.outward*240+Vec{0,0,-160};
        check(result.released&&!result.completed&&!t.active()&&std::string(result.reason)=="manual back flip"&&t.actionProgress()==1,
            "the final captured phase exits into ordinary fall, without a fabricated wall landing");
        check(elapsed>=.70f&&elapsed<=.78f&&maxLift>60&&maxLift<75&&(t.position-start).dot(wall.outward)>=179.99f,
            "the finite flip is prompt, visibly away from the wall, and uses its checked bounded lift");
        check((result.releaseVelocity-expectedVelocity).length()<.003f&&
            std::abs(terminalStep.dot(wall.outward)-240)<12&&std::abs(terminalStep.z+160)<30,
            "last sampled displacement joins the actual outward/downward release velocity without stopping");
        check(t.update(wall,release,dt,100).releaseVelocity.length()==0,"holding S+Space after release never repeats the impulse");
        std::cout<<"checked back flip fps="<<fps<<" yaw="<<yaw<<" elapsed="<<elapsed<<" lift="<<maxLift<<" terminalZ="<<terminalStep.z<<'\n';
    }
    for(int rejected=0;rejected<3;++rejected) {
        ExitWall wall;Traversal t;t.cfg.approachSeconds=0;
        check(t.attach(wall,{0,-30,1000},{0,1,0},100),"back flip refusal attach");
        if(rejected==0)t.cfg.fancyJumps=false;
        if(rejected==1)wall.approveSourceSweep=false;
        if(rejected==2)wall.rearDistance=120;
        const auto result=t.update(wall,release,.02f,100);
        check(result.motion==Motion::dropBack&&t.active()&&t.actionDuration()==.32f,
            "disabled fancy capture, rejected source sweep, or blocked full route falls back to the short visible push");
        check(rejected!=0||wall.bodyChecks==0,"disabled fancy jumps do not spend time preparing source-body sweeps");
    }
    for(int fps:{20,48,120})for(bool bodyObstacle:{false,true}) {
        ExitWall wall;Traversal t;t.cfg.approachSeconds=0;
        check(t.attach(wall,{0,-30,1000},{0,1,0},100),"dynamic back flip obstacle attach");
        const float dt=1.f/fps;auto result=t.update(wall,release,dt,100);
        while(t.actionProgress()<.35f)result=t.update(wall,{},dt,100);
        check(t.active()&&result.motion==Motion::backFlipOut,"dynamic change follows a fully approved departure");
        const Vec before=t.position;const float phase=t.actionProgress();
        if(bodyObstacle)wall.rejectAfterPhase=phase+.001f;
        else wall.rearDistance=before.dot(wall.outward)+t.cfg.radius+.05f;
        result=t.update(wall,{},dt,100);
        check(result.released&&!t.active()&&(t.position-before).length()==0&&t.actionProgress()==phase,
            "new capsule or animated-body obstruction aborts before committing any unchecked partial travel");
        if(bodyObstacle) {
            const float epsilon=.0001f;
            const Vec numericalTangent=(backFlipExitPoint({0,-30,1000},wall.outward,phase+epsilon)-
                backFlipExitPoint({0,-30,1000},wall.outward,phase-epsilon))/(2*epsilon*backFlipExitSeconds);
            check((result.releaseVelocity-numericalTangent).length()<1.f,
                "only a separately source-approved departure preserves the current tangent after a blocked rotation");
            check(wall.lastFromPhase==phase&&wall.lastToPhase==phase,
                "abort momentum has its own current-capture translation clearance check");
        } else check(result.releaseVelocity.length()==0,"a blocking capsule leaves no unchecked outward impulse on abort");
        check(std::string(result.reason)==(bodyObstacle?"back flip body clearance changed":"jump path changed"),
            "dynamic source-body rejection is distinguished from a changed controller path");
    }
    {
        ExitWall wall;Traversal t;t.cfg.approachSeconds=0;t.cfg.fancyJumps=false;
        check(t.attach(wall,{0,-30,1000},{0,1,0},100),"airborne release attach");
        const auto started=t.update(wall,{0,1,false,false,true},.02f,100);
        check(hopMotion(started.motion)&&t.state==State::action,"airborne release begins from an actual checked wall hop");
        for(int frame=0;frame<7;++frame)t.update(wall,{},.02f,100);
        t.cfg.fancyJumps=true;wall.bodyChecks=0;
        const auto cancelled=t.update(wall,release,.02f,100);
        check(cancelled.motion==Motion::dropBack&&t.state==State::action&&wall.bodyChecks==0,
            "S+Space cancels an airborne action through the short push without attempting another flip takeoff");
    }
}
#include "ManualHopTests.h"
int main(int argc,char**argv) {
 try {
    check(argc==2,"motion path");Library lib;check(lib.load(argv[1]),"FCM4 input");
    normalClimbingJumps();intermediateHopObstacle();continuousBackPush();checkedBackFlipExit();
    repeatedManualHops(lib);repeatedHopSafety();manualHopAfterContext(lib);
    {
        Mountain mountain;Traversal t;t.cfg.approachSeconds=0;t.cfg.radius=31;t.cfg.gap=37;t.cfg.height=138;
        check(t.attach(mountain,mountain.origin+Vec{0,-45,0},{0,1,0},100),"mountain base attach");
        float longest=0;
        for(int i=0;i<600;++i) {
            t.update(mountain,{0,1},1.f/60,100);longest=std::max(longest,t.stalledSeconds());
            check(t.active(),"mountain ridge must retain support");
        }
        std::cout<<"mountain rise="<<t.position.z-mountain.origin.z<<" stall="<<longest<<'\n';
        check(t.position.z-mountain.origin.z>400&&longest<.5f,"changing uphill normals and a mild protrusion must not strand the actor");
    }
    for(float yaw:{0.f,.5f,1.3f,2.7f})for(float angle:{-.57735f,0.f,.57735f}) {
        Exterior exterior;exterior.yaw=yaw;exterior.facets.angle=angle;
        Traversal t;t.cfg.approachSeconds=0;t.cfg.radius=31;t.cfg.gap=37;
        check(t.attach(exterior,exterior.point({-100,-45,200}),exterior.rotate({0,1,0},yaw),100),"exterior attach");
        SurfacePose animator;float longestStall=0;
        for(int frame=0;frame<360;++frame) {
            auto result=t.update(exterior,{1,0},1.f/60,100);
            auto pose=animator.update(lib,exterior,t,result.motion,1.f/60,1);
            const auto before=t.position;
            auto duplicate=t.update(exterior,{1,0},0,100);
            check(duplicate.motion==result.motion&&(t.position-before).length()==0&&duplicate.staminaCost==0,
                "zero-delta callbacks must preserve movement and position without stamina use");
            auto held=animator.update(lib,exterior,t,duplicate.motion,0,1);
            check(angleBetween(held[38].q,pose[38].q)<.002f,"zero delta must not restart pose blending");
            longestStall=std::max(longestStall,t.stalledSeconds());
        }
        const auto local=exterior.rotate(t.position-exterior.origin,-yaw);
        std::cout<<"exterior yaw="<<yaw<<" facet="<<angle<<" endX="<<local.x<<" stall="<<longestStall<<'\n';
        check(t.active()&&local.x>60&&longestStall<.5f,"exterior facet crossing must retain progress");
    }
    for(float angle:{-.57735f,.57735f}) {
        Facets w;w.angle=angle;Traversal t;t.cfg.approachSeconds=0;t.cfg.radius=31;t.cfg.gap=37;
        check(t.attach(w,{-100,-45,200},{0,1,0},100),"attach angled wall");
        float longestStall=0;SurfacePose pose;
        for(int frame=0;frame<360;++frame) {
            auto result=t.update(w,{1,0},1.f/60,100);
            check(t.active(),"minor facet seam must not lose the wall");
            auto p=pose.update(lib,w,t,result.motion,1.f/60,1);
            for(auto tr:p)check(tr.t.finite()&&std::isfinite(tr.q.dot(tr.q)),"finite pose on a corner");
            longestStall=std::max(longestStall,t.stalledSeconds());
        }
        std::cout<<"facet="<<angle<<" endX="<<t.position.x<<" stall="<<longestStall<<'\n';
        check(t.position.x>60&&longestStall<.5f,"30-degree seams must be traversable without hanging indefinitely");
    }
    Facets wall;wall.angle=0;
    for(auto direction:{Vec{-1,0,0},Vec{1,0,0},Vec{0,1,0}}) {
        Traversal t;t.cfg.approachSeconds=0;
        check(t.attach(wall,{0,-40,200},{0,1,0},100),"hop attach");
        SurfacePose animator;
        for(int frame=0;frame<30;++frame) {
            const auto moving=t.update(wall,{direction.x,direction.y},1.f/60,100);
            animator.update(lib,wall,t,moving.motion,1.f/60,1);
        }
        const auto start=t.position;
        auto r=t.update(wall,{direction.x,direction.y,false,false,true},1.f/60,100);
        check(t.state==State::action&&r.staminaCost==15,"hop requires a checked target and stamina");
        check(!t.runningAction(),"ordinary climbing does not acquire the running takeoff frame");
        float elapsed=0,cost=r.staminaCost;animator.update(lib,wall,t,r.motion,1.f/60,1);
        for(int i=0;i<70&&t.state==State::action;++i) {
            r=t.update(wall,{},1.f/60,100);auto pose=animator.update(lib,wall,t,r.motion,1.f/60,1);
            elapsed+=1.f/60;cost+=r.staminaCost;
            for(auto tr:pose)check(tr.t.finite()&&std::isfinite(tr.q.dot(tr.q)),"hop pose finite");
        }
        check(elapsed>=.45f&&elapsed<=.60f&&std::abs(cost-15)<.001f,
            "ordinary leap completes promptly and pays only its initial checked grab");
        check(t.active()&&t.state!=State::action&&(t.position-start).length()>45,"hop must land back on a verified wall");
        t.update(wall,{0,0,true},.02f,100);
        for(int i=0;i<20&&t.active();++i)t.update(wall,{},.02f,100);
        check(!t.active(),"release remains available after a hop");
    }
    {
        Traversal t;t.cfg.approachSeconds=0;check(t.attach(wall,{0,-40,200},{0,1,0},100),"abort attach");
        t.update(wall,{1,0,false,false,true},.02f,100);
        check(t.update(wall,{0,0,true},.02f,100).motion==Motion::drop,"release must interrupt a hop");
        for(int i=0;i<20&&t.active();++i)t.update(wall,{},.02f,100);
        check(!t.active(),"aborted hop restores ordinary falling");
    }
    {
        Slab slab;Traversal t;t.cfg.approachSeconds=0;
        check(t.attach(slab,{0,-40,0},{0,1,0},100),"ledge slab attach");
        const auto before=t.position;
        auto r=t.update(slab,{},.02f,100);
        check(r.motion==Motion::hang&&t.state!=State::action,"missing foot support retains ordinary stationary motion");
        for(int i=0;i<40;++i) {
            r=t.update(slab,{},.02f,100);
            check(r.motion==Motion::hang&&t.state!=State::action,"ledge-only support never starts a retired transfer");
        }
        check((t.position-before).length()<.00001f,"ordinary ledge idle has no positional drift");
        r=t.update(slab,{0,1,false,true},.02f,100);
        check(t.state==State::mantle,"ledge-only ordinary idle still allows top-out");
    }
    std::cout<<"PASS: convex/concave facet seams, three hops, landing, cancellation, ledge-only idle and top-out\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
