#include "ConverterActionPreview.h"
#include "ConverterWallRunGroup.h"
#include "../src/MotionSlots.h"
#include "CornerTestWorld.h"
#include <iostream>
#include <stdexcept>

using namespace fc;
static void require(bool value,const char* reason){if(!value)throw std::runtime_error(reason);}
static float poseDifference(const Pose& a,const Pose& b) {
    require(a.size()==b.size(),"preview poses have matching skeletons");float difference=0;
    for(std::size_t bone=0;bone<a.size();++bone) {
        difference=std::max(difference,(a[bone].t-b[bone].t).length());
        difference=std::max(difference,angleBetween(a[bone].q,b[bone].q));
    }
    return difference;
}
static void verifyMantle(const ConverterEditorDocument& doc,const ConverterEditedAnimation& edited,const ConverterActionPreview& preview,const char* label) {
    auto library=doc.base;auto& clip=library.clips[int(Motion::contextMantle)-1];
    clip.frames=edited.clip.frames;clip.seconds=edited.clip.duration;clip.height=edited.config.at("height").get<float>();
    const auto travel=edited.config.at("travel").get<std::array<float,3>>();clip.travel={travel[0],travel[1],travel[2]};
    clip.contacts=edited.config.at("contacts").get<std::vector<std::array<float,4>>>();clip.authoredPlayback=edited.config.contains("authoredPlayback");clip.trajectory={};
    if(clip.authoredPlayback&&edited.config.at("authoredPlayback").contains("trajectory")) {
        const auto& path=edited.config.at("authoredPlayback").at("trajectory");clip.trajectory.count=std::uint32_t(path.size());
        for(std::size_t i=0;i<path.size();++i)clip.trajectory.knots[i]={path[i][0],{path[i][1],path[i][2],path[i][3]}};
    } else if(!clip.authoredPlayback) {
        auto& profile=library.threepeatProfile;profile.mantleUnplant=edited.config.at("unplant").get<std::array<float,2>>();
        profile.mantleReplant=edited.config.at("replant").get<std::array<float,2>>();profile.mantleRelease=edited.config.at("releaseHands").get<std::array<std::array<float,2>,2>>();
        profile.replantSamplePhase=edited.config.at("replantSamplePhase").get<float>();
    }
    Traversal traversal;library.configureThreepeat(traversal.cfg);traversal.cfg.approachSeconds=0;traversal.cfg.gap=37;traversal.cfg.radius=31;
    traversal.cfg.contextActions=traversal.cfg.automaticClimbActions=traversal.cfg.wallRunObstacleJumps=false;traversal.cfg.staminaEnabled=false;
    const auto height=std::clamp(traversal.cfg.threepeatMantlePalmHeight,90.f,175.f);
    fc_test::CornerWorld world;world.boxes={{{-10000,0,-10000},{10000,10000,height}}};
    require(traversal.attach(world,{0,-42,0},{0,1,0},1000),"independent solid AABB supports the preview fixture");
    SurfacePose surface;constexpr float dt=1.f/120;
    for(unsigned i=0;i<60;++i){const auto result=traversal.update(world,{},dt,1000);surface.update(library,world,traversal,result.motion,dt,1);}
    std::size_t sample=0;float maximumPose=0,maximumPosition=0,maximumStep=0;Pose previous;bool completed=false;
    for(unsigned i=0;i<1800;++i) {
        const auto result=traversal.update(world,{0,1,false,true},dt,1000);const auto pose=surface.update(library,world,traversal,result.motion,dt,1);
        require(!result.released||result.completed,"independent runtime mantle keeps a supported route");
        if(result.motion==Motion::contextMantle) {
            require(sample<preview.frames.size(),"preview retains every actual mantle sample");const auto& actual=preview.frames[sample++];
            maximumPose=std::max(maximumPose,poseDifference(pose,actual.pose));
            maximumPosition=std::max(maximumPosition,(actual.position-traversal.position+traversal.topStart()).length());
            require(std::abs(actual.phase-surface.sampledPhase())<.0001f,"preview source phase follows SurfacePose rather than elapsed-time normalization");
            require(preview.ledgeHeight&&std::abs(*preview.ledgeHeight-height+traversal.topStart().z)<.0001f,"visible ledge is the actual runtime contact plane");
            auto expectedContacts=library.contactWeights(Motion::contextMantle,actual.phase);
            if(!clip.authoredPlayback)for(int hand=0;hand<2;++hand)expectedContacts[hand]=traversal.topHandWeight(hand,actual.phase);
            for(unsigned hand=0;hand<4;++hand)require(std::abs(actual.contacts[hand]-expectedContacts[hand])<.0001f,"displayed contacts follow the actual edited mantle source phase");
            const auto body=library.world(placedConverterActionPreviewPose(actual));
            if(!previous.empty())for(int bone:{8,11,38,39})maximumStep=std::max(maximumStep,(body[bone].t-previous[bone].t).length());
            previous=body;
        }
        if(result.completed){completed=true;break;}
    }
    require(completed&&sample==preview.frames.size()&&preview.surfaceQueries>0&&preview.coreSimulated,"complete preview runs real collision-controlled mantle and posture output");
    require(maximumPose<.002f&&maximumPosition<.002f,"every preview frame matches independent runtime on a solid AABB");
    require(preview.frames.front().phase<.001f&&preview.frames.back().phase>.999f,"complete mantle source begins at zero and reaches the actual final sample");
    const auto last=library.world(placedConverterActionPreviewPose(preview.frames.back()));
    for(int foot:{8,11})require(last[foot].t.y>=preview.wallDistance-1&&last[foot].t.z-*preview.ledgeHeight>0&&last[foot].t.z-*preview.ledgeHeight<20,"terminal ankles stand just above the ledge instead of receiving a second mantle displacement");
    require(maximumStep<12,"complete default and source mantle endpoints remain continuous");
    std::cout<<label<<" seconds="<<preview.seconds<<" frames="<<preview.frames.size()<<" poseError="<<maximumPose<<" pathError="<<maximumPosition<<" maxEndpointStep="<<maximumStep<<'\n';
    for(float fraction:{0.f,.25f,.5f,.75f,1.f}) {
        const auto frame=sampleConverterActionPreview(preview,preview.seconds*fraction);const auto body=library.world(placedConverterActionPreviewPose(frame));
        std::cout<<label<<" at="<<fraction<<" phase="<<frame.phase<<" root=("<<frame.position.x<<','<<frame.position.y<<','<<frame.position.z<<") COM=("<<body[4].t.x<<','<<body[4].t.y<<','<<body[4].t.z<<") feetZ="<<body[8].t.z<<','<<body[11].t.z<<" palmsZ="<<library.palm(body,0).z<<','<<library.palm(body,1).z<<" ledge="<<*preview.ledgeHeight<<'\n';
    }
}
int main(int argc,char** argv)try {
    require(argc==2,"base pack argument required");const std::filesystem::path pack=argv[1];
    for(const auto motion:activeMotions) {
        const std::string slot(motionSlotNames[int(motion)-1]);std::cout<<"preview "<<slot<<std::endl;const auto doc=loadConverterBaseEditor(pack,slot);const auto edited=applyConverterEdits(doc,{});
        const auto original=edited.clip.frames;const auto preview=buildConverterActionPreview(doc,edited);
        require(preview.available()&&preview.coreSimulated==(motion==Motion::contextMantle),"mantles use the checked runtime while other single scenes remain illustrative");
        if(motion!=Motion::contextMantle)require(std::abs(preview.seconds-edited.clip.duration)<.0001f,"single-action scene preserves the edited source duration");
        else {
            verifyMantle(doc,edited,preview,"base mantle");
        }
        require(preview.frames.front().position.length()<.001f,"single-action route starts at the common preview origin");
        if(authoredMovingLoop(motion)||hopMotion(motion)||motion==Motion::contextMantle||motion==Motion::reach||motion==Motion::jumpCatch||motion==Motion::ledgeCatch||motion==Motion::drop||motion==Motion::dropBack||motion==Motion::backFlipOut)
            require(preview.frames.back().position.length()>2,"moving actions have visible scene displacement instead of a treadmill pose");
        if(authoredIdleLoop(motion)||motion==Motion::sideBrace)require(preview.frames.back().position.length()<.001f,"idle and reference clips are not given fabricated locomotion");
        for(const auto& frame:preview.frames) {
            require(frame.pose.size()==99&&frame.position.finite(),"all action-scene frames use finite canonical poses");
            const auto placed=placedConverterActionPreviewPose(frame);
            require((placed[0].t-frame.pose[0].t-frame.position).length()<.001f,"controller translation is composed once outside local Root");
            for(std::size_t bone=1;bone<placed.size();++bone)require((placed[bone].t-frame.pose[bone].t).length()<.0001f,"scene placement cannot alter local bone offsets");
        }
        for(std::size_t frame=0;frame<original.size();++frame)require(poseDifference(original[frame],edited.clip.frames[frame])<.001f,"scene preview cannot edit exported animation data");
    }
    {
        const auto baseMantle=loadConverterBaseEditor(pack,"contextMantle");auto rootMantle=baseMantle;rootMantle.authored=true;
        const auto originalPreview=buildConverterActionPreview(baseMantle,applyConverterEdits(baseMantle,{}));
        rootMantle.clip.duration=originalPreview.seconds;rootMantle.clip.frames.clear();
        for(const auto& frame:originalPreview.frames){auto pose=frame.pose;pose[0].t=pose[0].t+frame.position;rootMantle.clip.frames.push_back(std::move(pose));}
        ConverterEditOptions options;options.autoCalibration=false;
        const auto rootEdited=applyConverterEdits(rootMantle,options);
        const auto rootPreview=buildConverterActionPreview(rootMantle,rootEdited);
        verifyMantle(rootMantle,rootEdited,rootPreview,"Root mantle");
        auto inPlace=rootMantle;for(auto& pose:inPlace.clip.frames)pose[0].t={};
        const auto inPlaceEdited=applyConverterEdits(inPlace,options);const auto inPlacePreview=buildConverterActionPreview(inPlace,inPlaceEdited);
        require(!inPlaceEdited.config.at("authoredPlayback").contains("trajectory"),"in-place fixture has no invented source Root path");
        verifyMantle(inPlace,inPlaceEdited,inPlacePreview,"in-place mantle");
        ConverterEditOptions changes;changes.autoCalibration=false;changes.speed=1.3f;changes.windows={{"releaseLeft",.50f,.59f},{"releaseRight",.55f,.65f}};
        const auto changed=applyConverterEdits(baseMantle,changes);const auto changedPreview=buildConverterActionPreview(baseMantle,changed);
        verifyMantle(baseMantle,changed,changedPreview,"edited mantle");
        bool changedContacts=false;
        for(const auto& frame:changedPreview.frames)if(frame.phase>.57f&&frame.phase<.61f&&frame.contacts[1]>.1f)changedContacts=true;
        require(changedContacts&&changedPreview.seconds<originalPreview.seconds,"edited contact windows and speed reach actual preview clocks and support weights");
    }
    for(const auto direction:{"runUp","runLeft","runRight","runDiagonalLeft","runDiagonalRight"}) {
        const auto doc=loadConverterBaseEditor(pack,direction);const auto edited=applyConverterEdits(doc,{});
        ConverterActionPreviewOptions options;options.kind=ConverterActionPreviewKind::wallRun;options.direction=direction;
        const auto preview=buildConverterActionPreview(doc,edited,options),repeat=buildConverterActionPreview(doc,edited,options);
        require(preview.available()&&preview.coreSimulated&&preview.surfaceQueries>0,"complete wall-run preview executes Core and SurfacePose on the standard wall");
        require(preview.segments.size()==3&&preview.segments[0].stage==ConverterActionPreviewStage::launch&&preview.segments[1].stage==ConverterActionPreviewStage::loop&&preview.segments[2].stage==ConverterActionPreviewStage::catching,"complete wall run has real launch, loop and catch timeline sections");
        require(preview.frames.back().motion==Motion::hang,"complete wall run returns to climbing through its catch");
        require(preview.frames.back().position.length()>edited.config.at("stride").get<float>(),"complete sequence displays controller travel through multiple gait cycles");
        bool checkedWrap=false;
        for(std::size_t i=1;i<preview.frames.size();++i) {
            const auto& a=preview.frames[i-1];const auto& b=preview.frames[i];
            if(a.motion!=b.motion||!authoredMovingLoop(a.motion)||b.phase>=a.phase)continue;
            const auto middle=sampleConverterActionPreview(preview,(a.seconds+b.seconds)*.5f);
            const float expected=std::fmod((a.phase+b.phase+1.f)*.5f,1.f);
            require(std::abs(std::remainder(middle.phase-expected,1.f))<.001f,"wall-run source phase crosses the loop seam without jumping to mid-clip");checkedWrap=true;
        }
        require(checkedWrap,"complete wall-run fixtures exercise real cycle boundaries");
        const auto queryCount=preview.surfaceQueries;
        for(float phase:{0.f,.17f,.5f,.97f,1.f,.5f,.17f}) {
            const auto a=sampleConverterActionPreview(preview,preview.seconds*phase),b=sampleConverterActionPreview(repeat,repeat.seconds*phase);
            require(poseDifference(a.pose,b.pose)<.001f&&(a.position-b.position).length()<.001f,"cached full-action seek is deterministic in either timeline direction");
        }
        require(preview.surfaceQueries==queryCount,"scrubbing the cached sequence does not issue new collision queries");
        for(std::size_t i=1;i<preview.frames.size();++i)require((preview.frames[i].position-preview.frames[i-1].position).length()<4,"default sequence controller remains continuous across launch and catch");
        const auto stages=converter::wallRunStages(direction);
        auto launch=loadConverterBaseEditor(pack,std::string(stages[0]),direction),catching=loadConverterBaseEditor(pack,"runCatch",direction);
        launch.authored=catching.authored=true;launch.clip.duration=.43f;catching.clip.duration=.71f;
        launch.clip.frames.assign(53,doc.base.clip(Motion::hang).frames.front());catching.clip.frames.assign(87,doc.base.clip(Motion::hang).frames.front());
        ConverterEditOptions freeContacts;for(unsigned contact=0;contact<4;++contact)freeContacts.contacts.push_back({contact,0,1,0,0});
        const auto launchEdited=applyConverterEdits(launch,freeContacts),catchEdited=applyConverterEdits(catching,freeContacts);
        const auto sequence=buildConverterActionPreview(doc,edited,options,{{&launch,&launchEdited,direction},{&catching,&catchEdited,direction}});
        for(const auto& expected:{std::pair{stages[0],launchEdited.clip.duration},std::pair{std::string_view("runCatch"),catchEdited.clip.duration}}) {
            const auto at=std::find(motionSlotNames.begin(),motionSlotNames.end(),expected.first);const auto selected=Motion(int(at-motionSlotNames.begin())+1);
            float first=-1,last=-1,maxPhase=0,minPhase=1;
            for(const auto& frame:sequence.frames)if(frame.motion==selected){if(first<0)first=frame.seconds;last=frame.seconds;maxPhase=std::max(maxPhase,frame.phase);minPhase=std::min(minPhase,frame.phase);}
            require(first>=0&&std::abs(last-first-expected.second)<.018f,"authored launch and catch use their complete individual source clocks");
            require(minPhase<.001f&&maxPhase>.999f,"authored full-sequence transitions include their first and terminal source samples");
        }
        auto unrelated=catching;unrelated.direction=std::string_view(direction)=="runRight"?"runUp":"runRight";
        const auto isolated=buildConverterActionPreview(doc,edited,options,{{&unrelated,&catchEdited,unrelated.direction}});
        require(isolated.frames.size()==preview.frames.size(),"another direction catch cannot change this sequence duration");
        for(std::size_t frame=0;frame<preview.frames.size();frame+=17)
            require(poseDifference(isolated.frames[frame].pose,preview.frames[frame].pose)<.001f&&(isolated.frames[frame].position-preview.frames[frame].position).length()<.001f,"another direction catch cannot leak into the selected wall-run sequence");
        if(std::string_view(direction)=="runUp") {
            auto shortSource=doc;shortSource.authored=true;shortSource.clip.duration=.04f;shortSource.clip.frames.assign(6,doc.base.clip(Motion::hang).frames.front());
            auto shortLoop=applyConverterEdits(shortSource,freeContacts);shortLoop.config["stride"]=1.5f;
            auto shortOptions=options;shortOptions.cycles=1;
            const auto shortPreview=buildConverterActionPreview(shortSource,shortLoop,shortOptions,{{&launch,&launchEdited,direction},{&catching,&catchEdited,direction}});
            require(std::any_of(shortPreview.frames.begin(),shortPreview.frames.end(),[](const auto& frame){return frame.motion==Motion::runCatch&&frame.phase>.999f;}),"a very short loop still leaves enough settled runtime time to play the complete authored catch");
        }
        std::cout<<direction<<" sequence seconds="<<preview.seconds<<" frames="<<preview.frames.size()<<" displacement="<<preview.frames.back().position.length()<<'\n';
    }
    const auto base=loadConverterBaseEditor(pack,"hopRight");auto source=base;source.authored=true;source.clip.duration=1;
    source.clip.frames.assign(121,base.base.clip(Motion::hang).frames.front());
    for(std::size_t i=0;i<source.clip.frames.size();++i) {
        const float phase=float(i)/120;source.clip.frames[i][0].t={80*phase,0,5*std::sin(phase*3.14159265f)};
    }
    ConverterEditOptions edits;for(unsigned contact=0;contact<4;++contact)edits.contacts.push_back({contact,0,1,0,0});
    const auto authored=applyConverterEdits(source,edits);const auto preview=buildConverterActionPreview(source,authored);
    AuthoredTrajectory trajectory;const auto& knots=authored.config.at("authoredPlayback").at("trajectory");trajectory.count=std::uint32_t(knots.size());
    for(std::size_t i=0;i<knots.size();++i)trajectory.knots[i]={knots[i][0].get<float>(),{knots[i][1].get<float>(),knots[i][2].get<float>(),knots[i][3].get<float>()}};
    require(trajectory.valid(),"the Root-consumption fixture has a valid exported controller trajectory");
    float maximumResidual=0,maximumConsumptionError=0;
    for(float phase:{0.f,.25f,.5f,.75f,1.f}) {
        const auto frame=sampleConverterActionPreview(preview,phase);const auto raw=sampleConverterEditor(authored,phase);
        const auto consumed=trajectory.sample(phase);const auto residual=raw[0].t-consumed;
        maximumResidual=std::max(maximumResidual,residual.length());maximumConsumptionError=std::max(maximumConsumptionError,(frame.pose[0].t-residual).length());
        require((frame.pose[0].t-residual).length()<.015f,"authored Root trajectory is consumed exactly once by the illustrative controller");
        require((placedConverterActionPreviewPose(frame)[0].t-(frame.position+raw[0].t-consumed)).length()<.015f,"world scene adds only fitted controller travel and the unconsumed Root residual");
    }
    require(maximumResidual>.015f&&maximumResidual<=.25f,"Root trajectory approximation leaves a real bounded residual that the pose must retain");
    std::cout<<"Root residual="<<maximumResidual<<" consumptionError="<<maximumConsumptionError<<'\n';
    auto invalid=authored;invalid.clip.duration=std::numeric_limits<float>::quiet_NaN();bool rejected=false;
    try{buildConverterActionPreview(source,invalid);}catch(const std::exception&){rejected=true;}
    require(rejected,"non-finite source duration cannot reach preview sampling");
    const auto doc=loadConverterBaseEditor(pack,"runUp");const auto edited=applyConverterEdits(doc,{});ConverterActionPreviewOptions options;
    options.kind=ConverterActionPreviewKind::wallRun;options.direction="hopUp";rejected=false;
    try{buildConverterActionPreview(doc,edited,options);}catch(const std::exception&){rejected=true;}
    require(rejected,"full wall-run mode refuses unrelated directions");
    std::cout<<"All active action routes, complete wall-run transitions, Root composition and deterministic seeking passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
