#define main contextHopOwnershipFixtureMain
#include "AutomaticVarietyTests.cpp"
#undef main

static void privateReferences(const Library& base) {
    Library library=base;
    for(int side=0;side<2;++side) {
        const auto direction=side?Motion::contextHopRight:Motion::contextHopLeft;
        library.contextHopSequenceValid[side]=true;
        for(bool recovery:{false,true}) {
            auto& clip=(recovery?library.contextHopRecoveries:library.contextHopPreparations)[side];
            clip=base.clip(Motion::contextHang,direction,recovery);
            for(auto& pose:clip.frames)pose[56].q=(Quat::axis({0,0,1},(side+1)*.15f+(recovery?.1f:0))*pose[56].q).unit();
            clip.contacts.assign(clip.frames.size(),{side?.7f:.3f,recovery?.8f:.4f,0,0});
        }
    }
    const auto left=library.sample(Motion::contextHang,.3f,Motion::contextHopLeft);
    const auto right=library.sample(Motion::contextHang,.3f,Motion::contextHopRight);
    const auto recovery=library.sample(Motion::contextHang,.3f,Motion::contextHopRight,true);
    require(angleBetween(left[56].q,right[56].q)>.1f&&angleBetween(right[56].q,recovery[56].q)>.08f,"Each side and preparation/recovery select independent owned data");
    Settings before;require(library.configureThreepeat(before),"Private reference calibration remains valid");
    require(before.contextHopReferences&&(*before.contextHopReferences)[0].valid&&(*before.contextHopReferences)[1].valid,"Both side calibrations are retained in immutable traversal metadata");
    library.clips[int(Motion::contextHang)-1].frames.assign(2,base.rest);
    for(auto& pose:library.contextHopPreparations[0].frames)pose[56].q=Quat::axis({1,0,0},1.8f);
    const auto independent=library.sample(Motion::contextHang,.3f,Motion::contextHopRight);
    require(angleBetween(right[56].q,independent[56].q)<.0001f,"Changing left and shared fallback cannot change right preparation");
    require(std::abs(library.contactWeights(Motion::contextHang,.3f,Motion::contextHopRight,true)[1]-.8f)<.00001f,"Recovery contacts belong to the selected side");
    auto& override=library.rotationOverrides[int(Motion::contextHang)-1];override.frames.resize(2);override.bones[56]=true;
    for(auto& frame:override.frames)frame[56]=Quat::axis({1,0,0},2.f);
    require(angleBetween(right[56].q,library.sample(Motion::contextHang,.3f,Motion::contextHopRight)[56].q)<.0001f,"Shared fallback edits cannot override an owned side reference");
}

static void oppositeSideRuntime(const Library& base,int fps) {
    Library changed=base;
    for(auto& pose:changed.contextHopPreparations[0].frames)pose[56].q=(Quat::axis({0,0,1},.2f)*pose[56].q).unit();
    for(auto& pose:changed.contextHopRecoveries[0].frames)pose[56].q=(Quat::axis({0,0,1},-.2f)*pose[56].q).unit();
    auto firstWorld=flat(),secondWorld=flat();auto first=attached(firstWorld,base),second=attached(secondWorld,changed);
    SurfacePose firstPose,secondPose;bool prepared=false,action=false,recovered=false;
    for(int frame=0;frame<fps*6;++frame) {
        const Input input=action?Input{}:Input{1,0};
        const auto a=first.update(firstWorld,input,1.f/fps,1000),b=second.update(secondWorld,input,1.f/fps,1000);
        require(a.motion==b.motion&&first.state==second.state&&(first.position-second.position).length()<.0001f,"Editing left cannot change right timing or collision route");
        const auto x=firstPose.update(base,firstWorld,first,a.motion,1.f/fps,1),y=secondPose.update(changed,secondWorld,second,b.motion,1.f/fps,1);
        for(std::size_t bone=0;bone<x.size();++bone)require((x[bone].t-y[bone].t).length()<.0001f&&angleBetween(x[bone].q,y[bone].q)<.0001f,"Editing left cannot change right live poses");
        prepared|=a.motion==Motion::contextHang&&first.holdsPreparedEdge(a.motion);
        action|=a.motion==Motion::contextHopRight;
        recovered|=a.motion==Motion::contextHang&&first.holdsDestinationEdge(a.motion);
    }
    require(prepared&&action&&recovered,"Independent right preparation, full leap and recovery remain reachable");
}

static void capturedReferences(const Library& library) {
    auto world=flat();auto traversal=attached(world,library);std::string error;
    auto capture=std::make_unique<TraversalCapture>(),decoded=std::make_unique<TraversalCapture>();
    capture->begin(traversal,{1,0},1.f/60,1000);TraversalCapture::RecordingWorld recording(world,*capture);
    const auto result=traversal.update(recording,{1,0},1.f/60,1000);capture->finish(traversal,result);
    require(decoded->deserialize(capture->serialize(),error)&&decoded->replay().matched,"Direction-owned geometry, support data and clocks survive capture/replay");
}

static void privatePlaybackFragments(const Library& base) {
    Library changed=base;
    for(Motion owner:{Motion::runLaunch,Motion::kickUp,Motion::kickLeft,Motion::kickRight}) {
        const std::array roles=owner==Motion::runLaunch?std::array{PlaybackReference::launchApproach,PlaybackReference::count,PlaybackReference::count,PlaybackReference::count}:
            std::array{PlaybackReference::kickLanding,PlaybackReference::kickRunLanding,PlaybackReference::kickRunBrace,owner==Motion::kickUp?PlaybackReference::count:PlaybackReference::kickTakeoff};
        for(const auto role:roles)if(role!=PlaybackReference::count) {
            require(base.hasReference(owner,role),"Default complete action carries its private playback reference");
            const auto expected=base.sampleReference(owner,role,.47f);
            const auto source=Library::referenceFallback(owner,role);
            for(auto& frame:changed.clips[int(source)-1].frames)frame[56].q=Quat::axis({1,0,0},1.3f);
            const auto actual=changed.sampleReference(owner,role,.47f);
            for(std::size_t bone=0;bone<actual.size();++bone)require((expected[bone].t-actual[bone].t).length()<.0001f&&
                angleBetween(expected[bone].q,actual[bone].q)<.0001f,"Replacing another action cannot alter an owned takeoff or landing fragment");
        }
    }
    const auto rightBefore=base.sampleReference(Motion::kickRight,PlaybackReference::kickLanding,.4f);
    for(auto& frame:changed.references[{Motion::kickLeft,PlaybackReference::kickLanding}].frames)frame[56].q=Quat::axis({1,0,0},1.1f);
    require(angleBetween(rightBefore[56].q,changed.sampleReference(Motion::kickRight,PlaybackReference::kickLanding,.4f)[56].q)<.0001f,
        "Private left landing edits cannot change right landing");
    for(Motion direction:{Motion::runLeft,Motion::runRight,Motion::runDiagonalLeft,Motion::runDiagonalRight}) {
        require(base.wallRunBraces[wallRunDirectionIndex(direction)].frames.size()>=2,"Each lateral running direction owns its support reference");
        const auto expected=base.sample(Motion::sideBrace,.5f,direction);
        for(auto& frame:changed.clips[int(Motion::sideBrace)-1].frames)frame[29].q=Quat::axis({1,0,0},1.2f);
        require(angleBetween(expected[29].q,changed.sample(Motion::sideBrace,.5f,direction)[29].q)<.0001f,
            "Shared compatibility slot cannot replace an independently owned brace");
    }
}

int main(int argc,char** argv) try {
    require(argc==2,"animation pack argument required");Library library;require(library.load(argv[1]),"Load actual default animation pack");
    require(library.hasContextHopSequence(Motion::contextHopLeft)&&library.hasContextHopSequence(Motion::contextHopRight),"Both default sides own preparation and recovery");
    privateReferences(library);capturedReferences(library);privatePlaybackFragments(library);
    for(int fps:{30,60,120})oppositeSideRuntime(library,fps);
    std::cout<<"Independent contextual side preparation, action, recovery and capture passed\n";return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
