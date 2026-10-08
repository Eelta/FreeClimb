#include "Pose.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
static void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
struct Wall:World {
    unsigned calls{};
    std::optional<Hit> ray(Vec a,Vec b) override {
        ++calls;
        if(a.y<0&&b.y>=0)return Hit{a+(b-a)*(-a.y/(b.y-a.y)),{0,-1,0},true};
        return {};
    }
};
struct EmptyWorld:World {std::optional<Hit> ray(Vec,Vec) override{return {};}};
static constexpr std::array<Motion,5> directions{Motion::runUp,Motion::runLeft,Motion::runRight,Motion::runDiagonalLeft,Motion::runDiagonalRight};
static constexpr std::array<Vec,5> headings{Vec{0,1,0},Vec{-1,0,0},Vec{1,0,0},Vec{-1,1,0},Vec{1,1,0}};
static Motion launch(std::size_t index){return index==0?Motion::runLaunch:index==1||index==3?Motion::runLaunchLeft:Motion::runLaunchRight;}
static Input runInput(std::size_t index){return {headings[index].x,headings[index].y,false,false,false,false,true};}
static Traversal attached(const Library& library,Wall& wall) {
    Traversal t;require(library.configureThreepeat(t.cfg),"direction fixture retains canonical geometry");
    t.cfg.approachSeconds=0;
    require(t.attach(wall,{0,-30,200},{0,1,0},1000),"direction fixture attaches through the wall collision contract");
    return t;
}
static Library sequences(const Library& base,bool authored) {
    auto library=base;library.clearAnimationOverrides();
    for(std::size_t index=0;index<directions.size();++index) {
        library.wallRunSequenceValid[index]=true;
        library.wallRunLaunches[index]=base.clip(launch(index));library.wallRunCatches[index]=base.clip(Motion::runCatch);
        if(!authored)continue;
        for(bool caught:{false,true}) {
            auto& clip=caught?library.wallRunCatches[index]:library.wallRunLaunches[index];
            clip.authoredPlayback=true;clip.seconds=(caught?.35f:.45f)+float(index)*.07f;clip.stride=24;
            clip.frames.assign(33,base.clip(Motion::hang).frames.front());clip.contacts.assign(33,{});
            clip.trajectory.count=5;
            for(std::size_t knot=0;knot<5;++knot) {
                const float phase=float(knot)/4;
                clip.trajectory.knots[knot]={phase,{2*std::sin(phase*6.2831853f),0,24*phase}};
            }
            for(std::size_t frame=0;frame<clip.frames.size();++frame) {
                const float phase=float(frame)/float(clip.frames.size()-1);auto& pose=clip.frames[frame];
                pose[0].t=clip.trajectory.sample(phase);
                pose[56].q=(Quat::axis({0,0,1},float(index+1)*.13f+(caught?.05f:0)+phase*.08f)*pose[56].q).unit();
                pose[4].t.z+=phase*3;
            }
        }
    }
    return library;
}
static void lookup(const Library& base) {
    auto library=sequences(base,true);
    auto& override=library.rotationOverrides[int(Motion::runCatch)-1];override.frames.resize(2);override.bones[56]=true;
    for(auto& frame:override.frames)frame[56]=Quat::axis({1,0,0},1.5f);
    for(std::size_t index=0;index<directions.size();++index) {
        const auto direction=directions[index];
        require(wallRunDirectionIndex(direction)==int(index),"the five sequence indices follow the retained logical direction IDs");
        require(&library.clip(launch(index),direction)==&library.wallRunLaunches[index]&&
            &library.clip(Motion::runCatch,direction)==&library.wallRunCatches[index],"launch and catch select the same independent direction pair");
        require(&library.clip(direction,direction)==&library.clips[int(direction)-1],"Loop selects its own directional source clip");
        const auto* brace=index?&library.wallRunBraces[index]:&library.clips[int(Motion::sideBrace)-1];
        require(&library.clip(Motion::sideBrace,direction)==brace,"Lateral directions select privately owned support references");
        const auto raw=library.sampleBase(Motion::runCatch,.5f,direction),sample=library.sample(Motion::runCatch,.5f,direction);
        require(angleBetween(raw[56].q,sample[56].q)<.0001f,"directional source curves cannot acquire the unrelated shared catch rotation override");
        library.wallRunCatches[index].contacts.assign(2,{float(index+1)*.1f,.25f,0,0});
        require(std::abs(library.contactWeights(Motion::runCatch,.7f,direction)[0]-float(index+1)*.1f)<.00001f,"contact sampling uses the same directional source as pose sampling");
    }
    Settings cfg;library.configureThreepeat(cfg);
    require(cfg.authoredWallRunSequences&&cfg.authoredWallRunSequences->valid[4]&&!cfg.authoredMotions,"direction-only source packs carry authored runtime metadata without requiring a shared source slot");
    const auto seconds=cfg.authoredWallRunSequences->catches[4].seconds;
    library.wallRunCatches[4].seconds=10;
    require(cfg.authoredWallRunSequences->catches[4].seconds==seconds,"runtime metadata is an immutable copy independent of subsequent editor changes");
    library.wallRunCatches[2]={};
    require(!library.hasWallRunSequence(directions[2])&& &library.clip(Motion::runCatch,directions[2])==&library.clips[int(Motion::runCatch)-1],"incomplete direction pairs fall back without returning an empty clip");
    library.configureThreepeat(cfg);require(!cfg.authoredWallRunSequences->valid[2],"Core and Library agree on incomplete-pair fallback");
    library.clearAnimationOverrides();require(library.hasWallRunSequence(directions[0]),"clearing rotation edits preserves direction source clips");
}
static void completeAction(const Library& library,Wall& world,Traversal& t,SurfacePose& surface,Motion stage,std::size_t index,int fps) {
    EmptyWorld poseWorld;const float dt=1.f/fps,seconds=library.clip(stage,directions[index]).seconds;
    require(t.state==State::action&&t.actionProgress()==0&&t.actionDuration()==seconds,"direction stage starts on the complete source clock");
    float elapsed=0;unsigned frames=0;
    while(t.state==State::action&&frames<unsigned(fps*3)) {
        const auto before=t.actionProgress();auto input=runInput(frames>unsigned(fps*seconds/2)?index:(index+1)%directions.size());
        if(stage==Motion::runCatch)input.run=false;
        const auto result=t.update(world,input,dt,1000);elapsed+=dt;++frames;
        require(result.motion==stage&&!result.released&&t.wallRunDirection(stage)==directions[index],"input changes cannot replace the direction of an accepted launch or catch");
        require(std::abs(t.actionProgress()-std::min(1.f,before+dt/seconds))<.00001f,"directional actions keep the source one-times phase clock");
        const auto pose=surface.update(library,poseWorld,t,result.motion,dt,1),source=library.sample(stage,t.actionProgress(),directions[index]);
        require(std::abs(surface.sampledPhase()-t.actionProgress())<.00001f,"Surface uses the same full phase as Core for every directional stage");
        if(surface.blendProgress()>=1) {
            require(angleBetween(pose[56].q,source[56].q)<.0001f,"Surface retains the chosen directional source pose");
            require((pose[0].t-(source[0].t-t.authoredRoot(stage,t.actionProgress()))).length()<.001f,"directional Root displacement is consumed once");
        }
        for(const auto& bone:pose)require(bone.t.finite()&&std::abs(bone.q.dot(bone.q)-1)<.002f,"direction transitions remain finite and normalized");
    }
    require(t.state==State::wall&&t.actionProgress()==1&&std::abs(elapsed-seconds)<=dt*1.01f,"complete launch and catch retain their final source frame at the source duration");
    require(t.wallRunDirection(stage)==directions[index],"completion keeps the source direction until the final pose is sampled");
}
static void fullSequences(const Library& library,int fps) {
    EmptyWorld poseWorld;
    for(std::size_t index=0;index<directions.size();++index) {
        Wall world;auto t=attached(library,world);SurfacePose surface;
        surface.update(library,poseWorld,t,Motion::hang,1.f/fps,1);
        auto result=t.update(world,runInput(index),1.f/fps,1000);
        require(result.motion==launch(index),"Core selects the direction-specific starting stage");
        completeAction(library,world,t,surface,result.motion,index,fps);
        for(int frame=0;frame<fps;++frame) {
            result=t.update(world,runInput(index),1.f/fps,1000);surface.update(library,poseWorld,t,result.motion,1.f/fps,1);
        }
        require(result.motion==directions[index]&&t.state==State::wall,"launch returns to its real running direction loop");
        auto stopped=runInput((index+2)%directions.size());stopped.run=false;
        result=t.update(world,stopped,1.f/fps,1000);
        require(result.motion==Motion::runCatch&&t.wallRunDirection(result.motion)==directions[index],"catch uses the direction being left even when the next movement input points elsewhere");
        completeAction(library,world,t,surface,result.motion,index,fps);
    }
}
static void legacySequences(const Library& base) {
    const auto library=sequences(base,false);EmptyWorld poseWorld;
    for(std::size_t index=0;index<directions.size();++index) {
        Wall firstWorld,secondWorld;auto first=attached(base,firstWorld),second=attached(library,secondWorld);SurfacePose firstPose,secondPose;
        firstPose.update(base,poseWorld,first,Motion::hang,1.f/60,1);secondPose.update(library,poseWorld,second,Motion::hang,1.f/60,1);
        for(int frame=0;frame<60;++frame) {
            const auto input=frame<30?runInput(index):Input{};
            const auto a=first.update(firstWorld,input,1.f/60,1000),b=second.update(secondWorld,input,1.f/60,1000);
            require(a.motion==b.motion&&first.state==second.state&&firstWorld.calls==secondWorld.calls&&
                (first.position-second.position).length()<.0001f,"v1 direction references retain identical runtime timing, movement and collision queries");
            require(second.state!=State::action,"v1 direction references do not become full-duration Core actions");
            const auto pa=firstPose.update(base,poseWorld,first,a.motion,1.f/60,1),pb=secondPose.update(library,poseWorld,second,b.motion,1.f/60,1);
            for(std::size_t bone=0;bone<pa.size();++bone)require((pa[bone].t-pb[bone].t).length()<.0001f&&
                angleBetween(pa[bone].q,pb[bone].q)<.0001f,"v1 source reference variants preserve the original rig and pose treatment");
            if(frame==0||frame==30)require(secondPose.bridgeWallRunDirection()==directions[index],"legacy short launch and catch bridges preserve their own direction");
        }
    }
}
static void mixedSequences(const Library& base) {
    auto library=sequences(base,true);
    library.wallRunSequenceValid[0]=false;
    library.wallRunLaunches[2]=base.clip(Motion::runLaunchRight);library.wallRunCatches[2]=base.clip(Motion::runCatch);
    library.clips[int(Motion::runCatch)-1]=library.wallRunCatches[4];
    for(std::size_t index:{std::size_t(0),std::size_t(1),std::size_t(2)}) {
        Wall world;auto t=attached(library,world);auto result=t.update(world,runInput(index),1.f/60,1000);
        require((t.state==State::action)==(index==1),"mixed packages select complete source stages only where that direction is authored");
        for(int frame=0;frame<120;++frame)result=t.update(world,runInput(index),1.f/60,1000);
        result=t.update(world,{},1.f/60,1000);
        require((t.state==State::action)==(index!=2),"an explicit v1 direction suppresses the unrelated authored shared catch while a missing direction uses the fallback");
        if(t.state==State::action)require(t.actionDuration()==library.clip(Motion::runCatch,directions[index]).seconds,"mixed catch duration follows the same selection as Library");
    }
}
int main(int argc,char** argv) try {
    require(argc==2,"animation pack argument required");Library library;require(library.load(argv[1]),"load canonical animation library");
    lookup(library);legacySequences(library);mixedSequences(library);
    const auto authored=sequences(library,true);for(int fps:{30,60,120})fullSequences(authored,fps);
    std::cout<<"Wall-run direction selection, full clocks, legacy bridges and mixed sources passed\n";
    return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
