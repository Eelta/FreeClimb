#include "Pose.h"
#include "TraversalCapture.h"
#include "CornerTestWorld.h"
#include <iostream>
#include <memory>
#include <string>
using namespace fc;
using namespace fc_test;
namespace {
unsigned failures{},cases{},snapshots{};
void require(bool condition,const char* why){if(!condition){++failures;std::cerr<<"FAIL "<<why<<'\n';}}
void capture(CornerWorld& world,Traversal& traversal,Input input,float dt) {
    auto recorded=std::make_unique<TraversalCapture>(),loaded=std::make_unique<TraversalCapture>();
    TraversalCapture::RecordingWorld wrapper(world,*recorded);
    recorded->begin(traversal,input,dt,1000);const auto result=traversal.update(wrapper,input,dt,1000);recorded->finish(traversal,result);
    std::string error;
    require(recorded->complete(),"corner frame fits the capture ray capacity");
    require(loaded->deserialize(recorded->serialize(),error),"corner state, knots, progress and cooldown serialize exactly");
    const auto replay=loaded->replay();if(!replay.matched)std::cerr<<replay.error<<'\n';
    require(replay.matched,"same-version corner state reproduces geometry query order and output");++snapshots;
}
void traverse(const Library& library,bool convex,int fps,float side,bool run) {
    CornerWorld world;world.corner(convex);world.mirror=side;world.yaw=.73f;
    if(side<0)world.origin={131065.836f,41791.117f,-11204.176f};
    Traversal traversal;traversal.cfg=settings();
    const Vec start=world.global({-155,-37,0});
    require(traversal.attach(world,start,world.direction({0,1,0}),1000),"Core attaches on the physical source wall");
    SurfacePose surface;Pose previous;float maxAngle=0,minClear=10000,maxHeading=0;Vec previousNormal=traversal.normal;bool poseBounded=true;
    bool entered=false,finished=false,airborne=false;unsigned movingFrames=0,pauseFrames=0;Vec paused{};bool captured=false;
    const float dt=1.f/fps;
    for(int frame=0;frame<fps*5&&traversal.active();++frame) {
        Input input;input.x=side;input.run=run;
        if(traversal.turningCorner()&&movingFrames>unsigned(fps/4)&&pauseFrames<unsigned(fps/2)) {
            if(!pauseFrames)paused=traversal.position;
            input={};++pauseFrames;
        }
        if(traversal.turningCorner()&&!captured&&pauseFrames>3){capture(world,traversal,input,dt);captured=true;}
        const auto result=traversal.update(world,input,dt,1000);
        if(traversal.turningCorner()) {entered=true;if(input.x!=0)++movingFrames;}
        else if(entered){finished=true;}
        if(pauseFrames>0&&pauseFrames<=unsigned(fps/2)&&input.x==0)
            require((traversal.position-paused).length()<.025f,"stopping during an attached corner freezes the controller without jitter");
        airborne|=traversal.state==State::action||result.released;
        minClear=std::min(minClear,world.clearance(traversal.position,traversal.cfg));
        maxHeading=std::max(maxHeading,std::acos(std::clamp(previousNormal.dot(traversal.normal),-1.f,1.f)));previousNormal=traversal.normal;
        const auto pose=surface.update(library,world,traversal,result.motion,dt,1);
        require(pose.size()==99,"corner keeps the complete 99-bone pose");
        for(unsigned bone=0;bone<pose.size();++bone) {
            require(pose[bone].t.finite()&&std::isfinite(pose[bone].q.dot(pose[bone].q)),"corner pose remains finite");
            if(previous.size()==pose.size()) {
                const float angle=angleBetween(previous[bone].q,pose[bone].q);
                maxAngle=std::max(maxAngle,angle);
                poseBounded&=angle<=(runMotion(result.motion)?18.849556f:12.566371f)*dt+.016f;
            }
        }
        previous=pose;
        if(finished&&(world.local(traversal.position)-Vec{convex?37.f:-37.f,convex?100.f:-140.f,0}).length()<20)break;
    }
    ++cases;const Vec finish=world.local(traversal.position);
    std::cout<<"integrated corner convex="<<convex<<" fps="<<fps<<" side="<<side<<" run="<<run<<" started="<<entered<<" finished="<<finished
        <<" p="<<finish.x<<','<<finish.y<<" clear="<<minClear<<" boneStep="<<maxAngle<<" headingStep="<<maxHeading<<" poseFrames="<<movingFrames<<'\n';
    require(entered&&finished&&traversal.active(),"continuous A/D enters and finishes the complete Core corner without releasing");
    require(!airborne,"corner remains continuous wall movement with no hop or fall state");
    require(minClear>=traversal.cfg.radius-.05f,"independent solid distance rejects Core capsule penetration");
    require(poseBounded,"actual SurfacePose preserves the existing climb/run all-bone transition budgets through turn and pause");
    require(maxHeading<=8.2f*dt+.02f,"actual traversal heading respects the smooth corner pacing");
    require(convex?finish.x>35&&finish.y>20:finish.x<-35&&finish.y<-70,"Core continues beyond the adjacent face instead of stopping at the corner endpoint");
}
void heightAndDiagonal(bool convex,int fps) {
    CornerWorld world;world.corner(convex);world.yaw=.73f;world.origin={131065.836f,41791.117f,-11204.176f};
    Traversal traversal;traversal.cfg=settings();const float dt=1.f/fps;
    require(traversal.attach(world,world.global({-155,-37,0}),world.direction({0,1,0}),1000),"vertical corner fixture attaches");
    const Vec source=traversal.normal;
    for(unsigned frame=0;frame<unsigned(fps*3)&&traversal.normal.dot(source)>.76f;++frame)traversal.update(world,{1,0},dt,1000);
    require(traversal.turningCorner()&&traversal.normal.dot(source)<.8f,"height changes begin during the actual rotating arc");
    const Vec before=traversal.position,normal=traversal.normal;
    for(int frame=0;frame<fps/5;++frame) {
        const auto result=traversal.update(world,{0,1,false,true},dt,1000);
        require(traversal.turningCorner()&&result.motion==Motion::up&&!result.released,"W alone climbs during an unfinished corner without becoming stuck");
    }
    const Vec upper=traversal.position;
    require(std::abs(upper.x-before.x)<.025f&&std::abs(upper.y-before.y)<.025f&&upper.z-before.z>19.7f,"W changes height while preserving the measured lateral path");
    for(int frame=0;frame<fps/5;++frame) {
        const auto result=traversal.update(world,{0,-1},dt,1000);
        require(traversal.turningCorner()&&result.motion==Motion::down&&!result.released,"S alone descends during an unfinished corner");
    }
    require(upper.z-traversal.position.z>12.5f&&traversal.normal.dot(normal)>.99999f,"vertical reverse retains the actual corner heading");
    for(int frame=0;frame<fps/5;++frame) {
        const Vec previous=traversal.position;
        const auto result=traversal.update(world,{1,1,false,true},dt,1000);
        const Vec delta=traversal.position-previous;
        require(!result.released&&delta.z>.5f&&std::hypot(delta.x,delta.y)>.2f,"W+D progresses both vertical and lateral coordinates");
        require(delta.length()<=std::hypot(traversal.cfg.sideSpeed,traversal.cfg.climbSpeed)*.7071068f*dt+.05f,"mixed corner input preserves normalized diagonal speed");
        require(world.clearance(traversal.position,traversal.cfg)>=traversal.cfg.radius-.05f,"mixed movement preserves independent body clearance");
    }
    capture(world,traversal,{0,1},dt);capture(world,traversal,{1,1},dt);++cases;
}
void manualHopControls() {
    {
        CornerWorld world;world.corner(true);Traversal traversal;traversal.cfg=settings();
        require(traversal.attach(world,{-100,-37,0},{0,1,0},1000),"pre-corner Space fixture attaches");
        Input jump{1,0};jump.hop=true;const auto result=traversal.update(world,jump,1.f/60,1000);
        require(result.motion==Motion::hopRight&&traversal.state==State::action&&!traversal.turningCorner(),
            "a checked lateral Space hop wins before automatic corner acquisition on the same input frame");++cases;
    }

    for(bool convex:{false,true})for(bool middle:{false,true}) {
        CornerWorld world;world.corner(convex);Traversal traversal;traversal.cfg=settings();
        require(traversal.attach(world,{-85,-37,0},{0,1,0},1000),"corner hop fixture attaches");
        traversal.update(world,{1,0},1.f/60,1000);
        if(middle) {

            const float normalDot=convex?.18f:.78f;
            for(int frame=0;frame<180&&traversal.normal.dot({0,-1,0})>normalDot;++frame)
                traversal.update(world,{1,0},1.f/60,1000);
        }
        require(traversal.turningCorner(),"manual hop starts while corner traversal still owns the route");
        const Vec before=traversal.position;Input jump; jump.hop=true;
        const auto begin=traversal.update(world,jump,1.f/60,1000);
        require(begin.motion==Motion::hopUp&&traversal.state==State::action&&!traversal.turningCorner(),
            "validated Space hop wins over the corner handler without an unchecked projection");
        require((traversal.position-before).length()<.025f,"manual hop starts at the displayed corner position without snapping");
        bool released=false;
        for(int frame=0;frame<120&&traversal.state==State::action;++frame) {
            const auto result=traversal.update(world,{},1.f/60,1000);released|=result.released;
            require(world.clearance(traversal.position,traversal.cfg)>=traversal.cfg.radius-.05f,
                "corner-initiated ordinary hop retains independent physical clearance");
        }
        require(!released&&traversal.state==State::wall&&traversal.position.z>before.z+65,
            "reachable corner hop completes on its actual upper wall support");++cases;
    }
    {
        CornerWorld world;world.corner(true);Traversal traversal;traversal.cfg=settings();
        require(traversal.attach(world,{-85,-37,0},{0,1,0},1000),"blocked corner hop fixture attaches");
        traversal.update(world,{1,0},1.f/60,1000);const Vec before=traversal.position;
        world.boxes.push_back({{-400,-400,149},{400,400,160},false});
        Input jump;jump.hop=true;const auto result=traversal.update(world,jump,1.f/60,1000);
        require(traversal.turningCorner()&&traversal.state==State::wall&&!result.released&&(traversal.position-before).length()<.025f,
            "blocked Space cannot discard the live corner or start an unchecked hop");
        traversal.update(world,{1,0},1.f/60,1000);
        require(traversal.turningCorner()&&(traversal.position-before).length()>.5f,
            "lateral movement remains available after a physically blocked hop");++cases;
    }
    {
        CornerWorld world;world.corner(true);Traversal traversal;traversal.cfg=settings();
        require(traversal.attach(world,{-85,-37,0},{0,1,0},1000),"running corner Space fixture attaches");
        Input run{1,0};run.run=true;traversal.update(world,run,1.f/60,1000);run.hop=true;
        const auto result=traversal.update(world,run,1.f/60,1000);
        require(traversal.turningCorner()&&traversal.state==State::wall&&!hopMotion(result.motion)&&!traversal.preparingEdge(),
            "wall-run Space stays consumed during a corner");
        run.run=false;const auto releasedShift=traversal.update(world,run,1.f/60,1000);
        require(traversal.state==State::wall&&!hopMotion(releasedShift.motion)&&!traversal.preparingEdge(),
            "releasing Shift with Space cannot leak a corner wall-run jump");++cases;
    }
    {
        CornerWorld world;world.corner(true);Traversal traversal;traversal.cfg=settings();
        require(traversal.attach(world,{-85,-37,0},{0,1,0},1000),"removed-source corner hop fixture attaches");
        traversal.update(world,{1,0},1.f/60,1000);
        world.boxes[0].low.z=150;Input jump;jump.hop=true;
        const auto result=traversal.update(world,jump,1.f/60,1000);
        require(result.released&&!traversal.active()&&!hopMotion(result.motion),
            "a surviving upper landing cannot authorize a hop after the current source grips disappear");++cases;
    }
    {
        CornerWorld world;world.corner(true);Traversal traversal;traversal.cfg=settings();
        require(traversal.attach(world,{-85,-37,0},{0,1,0},1000),"edge-preparation corner fixture attaches");
        traversal.update(world,{1,0},1.f/60,1000);
        world.boxes.push_back({{-400,-8,143},{400,0,151}});
        world.boxes.push_back({{-400,-8,205},{400,0,213}});
        Input jump;jump.hop=true;const auto result=traversal.update(world,jump,1.f/60,1000);
        require(traversal.preparingEdge()&&!traversal.turningCorner()&&result.staminaCost==0,
            "successful measured-edge preparation acquires sole movement ownership before action commit");
        bool committed=false;
        for(int frame=0;frame<120&&!committed;++frame) {
            const auto action=traversal.update(world,{},1.f/60,1000);
            committed=traversal.state==State::action&&traversal.usesEdgeTargets(action.motion);
        }
        require(committed&&!traversal.turningCorner(),"latched corner Space finishes measured alignment without resuming its stale corner route");++cases;
    }
}
}
int main(int argc,char** argv) {
    if(argc!=2){std::cerr<<"motion path required\n";return 1;}
    Library library;if(!library.load(argv[1])){std::cerr<<"motion load failed\n";return 1;}
    for(bool convex:{false,true})for(int fps:{30,60,120})for(float side:{-1.f,1.f})for(bool run:{false,true})traverse(library,convex,fps,side,run);
    for(bool convex:{false,true})for(int fps:{30,60,120})heightAndDiagonal(convex,fps);
    manualHopControls();
    {
        CornerWorld world;world.corner(true);Traversal traversal;traversal.cfg=settings();
        require(traversal.attach(world,{-45,-37,0},{0,1,0},1000),"dynamic corner fixture attaches");
        capture(world,traversal,{1,0},1.f/60);
        for(int frame=0;frame<90&&!traversal.turningCorner();++frame)traversal.update(world,{1,0},1.f/60,1000);
        require(traversal.turningCorner(),"dynamic corner fixture begins turn");
        capture(world,traversal,{1,0},1.f/60);
        world.boxes.clear();const auto result=traversal.update(world,{1,0},1.f/60,1000);
        require(!traversal.turningCorner()&&(!traversal.active()||result.released),"removing all physical support cancels the corner and resumes gravity");
    }
    std::cout<<"CornerIntegrationTests cases="<<cases<<" snapshots="<<snapshots<<" failures="<<failures<<'\n';
    return failures?1:0;
}
