#include "AnimationPack.h"
#include "CornerTestWorld.h"
#include <iostream>
#include <stdexcept>

using namespace fc;
static unsigned checks{};
static void require(bool value,const char* text){++checks;if(!value)throw std::runtime_error(text);}
namespace fc {
class TraversalCapture {
public:
    static std::optional<Result> recover(Traversal& t,World& world,Input input){return t.trySupportRecovery(world,input,1000);}
};
}
struct RecoveryPoseWorld final:World {
    fc_test::CornerWorld source,target;
    bool sourceVisible=true;
    RecoveryPoseWorld(){source.boxes={{{-10000,0,-10000},{10000,10000,10000}}};}
    std::optional<Hit> ray(Vec from,Vec to)override {
        auto hit=sourceVisible?source.ray(from,to):std::optional<Hit>{};
        if(auto next=target.ray(from,to))if(!hit||(next->point-from).length()<(hit->point-from).length())hit=next;
        return hit;
    }
};
static float poseDistance(const Pose& a,const Pose& b) {
    require(a.size()==b.size()&&a.size()==99,"real animation pack supplies all 99 canonical bones");
    float worst=0;
    for(std::size_t bone=0;bone<a.size();++bone)
        worst=std::max(worst,(a[bone].t-b[bone].t).length()+angleBetween(a[bone].q,b[bone].q)+(a[bone].s-b[bone].s).length());
    return worst;
}
static void validPose(const Library& library,const Pose& pose) {
    for(const auto& bone:pose)require(bone.t.finite()&&bone.s.finite()&&std::isfinite(bone.q.dot(bone.q))&&
        std::abs(bone.q.dot(bone.q)-1)<.002f,"recovery transition keeps finite normalized bone transforms");
    for(int hand=0;hand<2;++hand)require(library.armBendValid(pose,hand),"recovery keeps existing arm hinge protection");
}
static void sameMotionReplan(const Library& library,float stopPhase) {
    RecoveryPoseWorld world;Traversal t;t.cfg=fc_test::settings();library.configureThreepeat(t.cfg);
    t.cfg.contextActions=false;t.cfg.automaticClimbActions=false;t.cfg.wallRunObstacleJumps=false;t.cfg.fancyJumps=false;
    require(t.attach(world,{0,-42,200},{0,1,0},1000),"pose fixture attaches to a real initial wall");
    SurfacePose surface;Pose previous;
    constexpr float dt=1.f/60;
    for(int frame=0;frame<30;++frame)previous=surface.update(library,world,t,Motion::hang,dt,1);
    world.sourceVisible=false;world.target.boxes={{{38,0,-1000},{500,100,1500}}};
    const auto planned=TraversalCapture::recover(t,world,{1,0});
    require(planned&&t.recoveringSupport()&&planned->motion==Motion::hopRight,"finite target creates a collision-checked rightward recovery");
    previous=surface.update(library,world,t,planned->motion,dt,1);
    for(unsigned frame=0;frame<100&&t.actionProgress()<stopPhase;++frame) {
        const auto result=t.update(world,{},dt,1000);
        require(t.recoveringSupport()&&!t.geometryHolding(),"initial recovery moves along its valid route");
        previous=surface.update(library,world,t,result.motion,dt,1);validPose(library,previous);
    }
    const Vec originalTarget=t.edgeTarget();const auto oldMotion=planned->motion;
    world.target.boxes.clear();auto stopped=t.update(world,{},dt,1000);
    require(t.geometryHolding()&&t.recoveringSupport()&&t.actionProgress()>0,"vanished destination freezes an in-progress action");
    const float heldPhase=t.actionProgress(),heldBlend=surface.blendProgress();const Vec heldRoot=t.position;
    for(unsigned frame=0;frame<90;++frame) {
        const auto result=t.update(world,{},frame%2?dt:0,1000);
        const auto shown=surface.update(library,world,t,result.motion,0,1);
        require(poseDistance(previous,shown)<.00001f&&surface.blendProgress()==heldBlend,
            "holding and paused pose time preserve the displayed pose and do not restart blending");
        require(t.actionProgress()==heldPhase&&(t.position-heldRoot).length()<.001f,"holding retains its checked root and original phase");
    }
    world.target.boxes={{{100,0,-1000},{500,100,1500}}};
    bool replanned=false;Result result;
    for(unsigned frame=0;frame<180;++frame) {
        result=t.update(world,{1,0},dt,1000);
        if(t.recoveringSupport()&&(t.edgeTarget()-originalTarget).length()>1) {replanned=true;break;}
        previous=surface.update(library,world,t,result.motion,t.geometryHolding()?0:dt,1);
    }
    require(replanned&&result.motion==oldMotion&&t.actionProgress()==0,"new real target restarts the same motion identifier from its current root");
    require((t.position-heldRoot).length()<.001f,"replacement planning does not move the character before playback");
    constexpr float firstDt=.00001f;
    const auto first=surface.update(library,world,t,result.motion,firstDt,1);validPose(library,first);
    require(surface.blendProgress()>0&&surface.blendProgress()<.001f,"same-motion replacement restarts the existing transition clock");
    require(surface.bridgeMotion()==Motion::none,"new support route never inherits an unrelated wall-run bridge");
    const float continuity=poseDistance(previous,first);
    require(continuity<.02f,"replacement blends from the last shown pose rather than jumping to the new clip phase");
    previous=first;float previousBlend=surface.blendProgress();unsigned continuing=0;
    for(unsigned frame=0;frame<160&&t.state==State::action;++frame) {
        result=t.update(world,{},dt,1000);
        require(!result.released&&!t.geometryHolding(),"replacement route keeps real support until landing");
        const auto shown=surface.update(library,world,t,result.motion,dt,1);validPose(library,shown);
        if(t.recoveringSupport()) {
            require(surface.blendProgress()>=previousBlend,"unchanged recovery endpoints cannot restart the transition every frame");
            previousBlend=surface.blendProgress();++continuing;
        }
        previous=shown;
    }
    require(continuing>5&&t.state==State::wall&&!t.recoveringSupport(),"replacement completes and returns to normal supported movement");
    for(unsigned frame=0;frame<25;++frame) {
        result=t.update(world,{0,1},dt,1000);previous=surface.update(library,world,t,result.motion,dt,1);validPose(library,previous);
    }
    require(surface.blendProgress()==1,"ordinary movement settles after recovery without a repeated transition");
    std::cout<<"same-motion recovery stop="<<stopPhase<<" first-pose delta="<<continuity<<" continuing="<<continuing<<'\n';
}
int main(int argc,char** argv) {
    try {
        require(argc>1,"animation pack path is required");Library library;
        require(library.load(argv[1]),"real bundled HKX animation pack loads");
        sameMotionReplan(library,.25f);sameMotionReplan(library,.55f);
        std::cout<<"support recovery pose checks="<<checks<<" failures=0\n";return 0;
    }catch(const std::exception& error){std::cerr<<"support recovery pose checks="<<checks<<" failure="<<error.what()<<'\n';return 1;}
}
