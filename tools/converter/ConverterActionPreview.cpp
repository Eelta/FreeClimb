#include "ConverterActionPreview.h"
#include "ConverterRuntimePreview.h"
#include "ConverterWallRunGroup.h"
#include "../../src/MotionSlots.h"
#include <set>
#include <stdexcept>

namespace fc {
namespace {
constexpr float step=1.f/120;
struct PreviewWall final:World {
    std::size_t queries{};
    std::optional<Hit> ray(Vec a,Vec b) override {
        ++queries;
        if(std::abs(b.y-a.y)<.00001f)return {};
        const float phase=-a.y/(b.y-a.y);
        if(phase<0||phase>1)return {};
        return Hit{a+(b-a)*phase,{0,-1,0},true};
    }
};
struct PreviewLedge final:World {
    float height{};
    std::size_t queries{};
    std::optional<Hit> ray(Vec a,Vec b) override {
        ++queries;std::optional<Hit> hit;float nearest=2;
        auto offer=[&](float phase,Vec normal) {
            if(phase>=0&&phase<=1&&phase<nearest){nearest=phase;hit=Hit{a+(b-a)*phase,normal,true};}
        };
        if(a.y<0&&b.y>=0){const float phase=-a.y/(b.y-a.y);if((a+(b-a)*phase).z<=height)offer(phase,{0,-1,0});}
        if(a.z>height&&b.z<=height){const float phase=(height-a.z)/(b.z-a.z);if((a+(b-a)*phase).y>=0)offer(phase,{0,0,1});}
        return hit;
    }
};
Motion motionFor(std::string_view slot) {
    const auto at=std::find(motionSlotNames.begin(),motionSlotNames.end(),slot);
    if(slot.empty()||at==motionSlotNames.end())throw std::runtime_error("Invalid action preview slot");
    return Motion(int(at-motionSlotNames.begin())+1);
}
Vec headingFor(Motion motion) {
    if(motion==Motion::left||motion==Motion::runLeft)return {-1,0,0};
    if(motion==Motion::right||motion==Motion::runRight)return {1,0,0};
    if(motion==Motion::down)return {0,-1,0};
    if(motion==Motion::runDiagonalLeft)return Vec{-1,1,0}.unit();
    if(motion==Motion::runDiagonalRight)return Vec{1,1,0}.unit();
    return {0,1,0};
}
bool directionStage(Motion motion) {return motion==Motion::runLaunch||motion==Motion::runLaunchLeft||motion==Motion::runLaunchRight||motion==Motion::runCatch||motion==Motion::sideBrace;}
std::unique_ptr<Library> previewLibrary(const ConverterEditorDocument& doc,const ConverterEditedAnimation& edited,const ConverterActionPreviewOptions& options,const std::vector<ConverterEditorExport>& overlays) {
    auto storage=std::make_unique<Library>(doc.base);auto& library=*storage;library.clearAnimationOverrides();
    const std::string selectedDirection=options.kind==ConverterActionPreviewKind::wallRun?options.direction:doc.direction;
    auto apply=[&](std::string_view name,const ConverterEditedAnimation& value,std::string_view direction) {
        const auto motion=motionFor(name);
        if(!direction.empty()&&!converter::isWallRunPrimary(direction))throw std::runtime_error("Invalid action preview overlay direction");
        if(directionStage(motion)&&!direction.empty()&&!selectedDirection.empty()&&direction!=selectedDirection)return;
        if(value.clip.frames.size()<2||value.clip.frames.size()>1201||!std::isfinite(value.clip.duration)||value.clip.duration<=0||value.clip.duration>10)
            throw std::runtime_error("Invalid action preview frames or duration");
        Clip* selected=&library.clips[std::size_t(int(motion)-1)];
        if(directionStage(motion)&&!direction.empty()) {
            const auto directionMotion=motionFor(direction);const auto index=std::size_t(int(directionMotion)-int(Motion::runUp));
            if(!library.wallRunSequenceValid[index]) {
                const auto launch=directionMotion==Motion::runUp?Motion::runLaunch:(directionMotion==Motion::runLeft||directionMotion==Motion::runDiagonalLeft)?Motion::runLaunchLeft:Motion::runLaunchRight;
                library.wallRunLaunches[index]=library.clip(launch);library.wallRunCatches[index]=library.clip(Motion::runCatch);library.wallRunSequenceValid[index]=true;
            }
            selected=motion==Motion::sideBrace?&library.wallRunBraces[index]:motion==Motion::runCatch?&library.wallRunCatches[index]:&library.wallRunLaunches[index];
        }
        auto& target=*selected;target.frames=value.clip.frames;target.seconds=value.clip.duration;
        target.stride=value.config.at("stride").get<float>();target.height=value.config.at("height").get<float>();
        const auto& travel=value.config.at("travel");target.travel={travel[0].get<float>(),travel[1].get<float>(),travel[2].get<float>()};
        if(!std::isfinite(target.stride)||target.stride<0||!std::isfinite(target.height)||!target.travel.finite())throw std::runtime_error("Invalid action preview geometry");
        target.authoredPlayback=value.config.contains("authoredPlayback");target.trajectory={};target.contacts.clear();
        for(const auto& row:value.config.at("contacts"))target.contacts.push_back(row.get<std::array<float,4>>());
        if(motion==Motion::contextMantle&&!target.authoredPlayback) {
            auto& profile=library.threepeatProfile;
            profile.mantleUnplant=value.config.at("unplant").get<std::array<float,2>>();
            profile.mantleReplant=value.config.at("replant").get<std::array<float,2>>();
            profile.mantleRelease=value.config.at("releaseHands").get<std::array<std::array<float,2>,2>>();
            profile.replantSamplePhase=value.config.at("replantSamplePhase").get<float>();
            if(!validThreepeatProfile(profile))throw std::runtime_error("Invalid mantle preview contact windows");
        }
        if(target.authoredPlayback&&value.config.at("authoredPlayback").contains("trajectory")) {
            const auto& rows=value.config.at("authoredPlayback").at("trajectory");
            if(rows.size()>65)throw std::runtime_error("Invalid action preview Root trajectory");
            target.trajectory.count=std::uint32_t(rows.size());
            for(std::size_t i=0;i<rows.size();++i)target.trajectory.knots[i]={rows[i][0].get<float>(),{rows[i][1].get<float>(),rows[i][2].get<float>(),rows[i][3].get<float>()}};
            if(!target.trajectory.valid())throw std::runtime_error("Invalid action preview Root trajectory");
        }
    };
    const auto members=converter::actionStages(converter::actionPrimaryForSlot(doc.slot));
    if(overlays.size()>32)throw std::runtime_error("Too many action preview overlays");
    std::set<std::string> used;
    for(const auto& overlay:overlays) {
        std::error_code error;
        if(!overlay.document||!overlay.animation||!std::filesystem::equivalent(overlay.document->pack,doc.pack,error)||error||
            std::find(members.begin(),members.end(),overlay.document->slot)==members.end())
            throw std::runtime_error("Invalid action preview overlay base pack or group");
        const auto direction=overlay.direction.empty()?overlay.document->direction:overlay.direction;
        if(!used.insert(overlay.document->slot+"/"+direction).second)throw std::runtime_error("Duplicate action preview overlay");
        apply(overlay.document->slot,*overlay.animation,direction);
    }
    apply(doc.slot,edited,doc.direction);return storage;
}
void append(ConverterActionPreview& output,ConverterActionPreviewFrame frame,ConverterActionPreviewStage stage,std::string_view slot) {
    if(frame.pose.size()!=99||!frame.position.finite())throw std::runtime_error("Invalid action preview pose");
    for(const auto& bone:frame.pose)if(!bone.t.finite()||!bone.s.finite()||!std::isfinite(bone.q.dot(bone.q)))throw std::runtime_error("Non-finite action preview pose");
    if(output.segments.empty()||output.segments.back().stage!=stage||output.segments.back().slot!=slot) {
        const float begin=output.frames.empty()?frame.seconds:output.frames.back().seconds;
        output.segments.push_back({begin,frame.seconds,std::string(slot),stage});
    } else output.segments.back().end=frame.seconds;
    output.seconds=frame.seconds;output.frames.push_back(std::move(frame));
}
Vec singleTarget(const Clip& clip,Motion motion) {
    if(authoredMovingLoop(motion)) {const auto heading=headingFor(motion);return {heading.x*clip.stride,0,heading.y*clip.stride};}
    if(authoredIdleLoop(motion)||motion==Motion::sideBrace)return {};
    if(motion==Motion::reach)return {0,30,0};
    if(motion==Motion::jumpCatch)return {0,30,88};
    if(motion==Motion::ledgeCatch)return {0,30,-20};
    if(motion==Motion::drop)return {0,0,-60};
    if(motion==Motion::dropBack)return {0,-220.f*.32f/3,90.f*.32f/3};
    if(motion==Motion::backFlipOut)return backFlipExitPoint({},{0,-1,0},1);
    if(motion==Motion::contextMantle)return {0,clip.travel.y,clip.height};
    if(motion==Motion::hopLeft||motion==Motion::hopRight||threepeatHop(motion))return {clip.travel.x,0,clip.travel.z};
    if(motion==Motion::hopUp)return {0,0,std::abs(clip.travel.z)};
    if(motion==Motion::kickUp)return {0,0,std::max(std::abs(clip.travel.y),std::abs(clip.travel.z))};
    if(motion==Motion::kickLeft||motion==Motion::kickRight)return {clip.travel.x,0,clip.travel.z};
    return clip.travel;
}
ConverterActionPreview singlePreview(const ConverterEditorDocument& doc,const ConverterEditedAnimation& edited,const Library& library) {
    ConverterActionPreview output;const auto motion=motionFor(doc.slot);const auto direction=doc.direction.empty()?Motion::none:motionFor(doc.direction);const auto& clip=library.clip(motion,direction);
    const auto destination=singleTarget(clip,motion);const auto rootEnd=clip.trajectory.sample(1);
    const bool consumedRoot=clip.authoredPlayback&&!authoredIdleLoop(motion)&&rootEnd.length()>=2;
    ConverterRuntimePreview adapted;
    if(runMotion(motion)){auto adaptedDoc=std::make_unique<ConverterEditorDocument>(doc);adaptedDoc->base=library;adapted=buildConverterRuntimePreview(*adaptedDoc,edited);}
    const auto intervals=std::clamp(std::size_t(std::ceil(clip.seconds*120)),std::size_t(2),std::size_t(1200));
    output.wallDistance=(motion==Motion::reach||motion==Motion::jumpCatch||motion==Motion::ledgeCatch)?60.f:30.f;
    output.surfaceQueries=adapted.surfaceQueries;output.frames.reserve(intervals+1);
    for(std::size_t i=0;i<=intervals;++i) {
        const float phase=float(i)/float(intervals);ConverterActionPreviewFrame frame;
        frame.seconds=clip.seconds*phase;frame.phase=phase;frame.motion=motion;frame.contacts=library.contactWeights(motion,phase,direction);
        frame.pose=adapted.available()?sampleConverterRuntimePreview(adapted,frame.seconds):library.sample(motion,phase,direction);
        const auto root=authoredMovingLoop(motion)?rootEnd*phase:clip.trajectory.sample(phase);
        if(!adapted.available()&&consumedRoot)frame.pose[0].t=frame.pose[0].t-root;
        if(authoredMovingLoop(motion))frame.position=destination*phase;
        else if(consumedRoot){const float along=root.dot(rootEnd)/rootEnd.dot(rootEnd);frame.position=destination*along+root-rootEnd*along;}
        else if(motion==Motion::backFlipOut)frame.position=backFlipExitPoint({},{0,-1,0},phase);
        else {
            frame.position=destination*smooth(phase);
            if(hopMotion(motion))frame.position.y-=(motion>=Motion::kickUp&&motion<=Motion::kickRight?52.f:32.f)*std::sin(phase*3.14159265f);
        }
        append(output,std::move(frame),authoredMovingLoop(motion)?ConverterActionPreviewStage::loop:ConverterActionPreviewStage::action,doc.slot);
    }
    return output;
}
ConverterActionPreview mantlePreview(const Library& library) {
    PreviewLedge world;Traversal traversal;library.configureThreepeat(traversal.cfg);
    traversal.cfg.approachSeconds=0;traversal.cfg.gap=37;traversal.cfg.radius=31;
    traversal.cfg.contextActions=traversal.cfg.automaticClimbActions=traversal.cfg.wallRunObstacleJumps=false;
    traversal.cfg.staminaEnabled=false;
    world.height=std::clamp(traversal.cfg.threepeatMantlePalmHeight,90.f,175.f);
    if(!traversal.attach(world,{0,-42,0},{0,1,0},1000))throw std::runtime_error("Standard ledge cannot support the edited mantle entry");
    SurfacePose surface;
    for(unsigned frame=0;frame<60;++frame) {
        const auto result=traversal.update(world,{},step,1000);
        if(result.released)throw std::runtime_error("Standard ledge lost initial mantle support");
        surface.update(library,world,traversal,result.motion,step,1);
    }
    ConverterActionPreview output;output.coreSimulated=true;bool began=false;Vec origin{};unsigned samples=0;
    for(unsigned frame=0;frame<1800;++frame) {
        const auto result=traversal.update(world,{0,1,false,true},step,1000);
        if(result.released&&!result.completed)throw std::runtime_error("Edited mantle cannot complete the checked standard ledge route");
        auto pose=surface.update(library,world,traversal,result.motion,step,1);
        if(result.motion==Motion::contextMantle) {
            if(!began){origin=traversal.topStart();output.wallDistance=-origin.y;output.ledgeHeight=world.height-origin.z;began=true;}
            const float phase=surface.sampledPhase();auto contacts=library.contactWeights(result.motion,phase);
            if(!traversal.cfg.authoredMantle)for(int hand=0;hand<2;++hand)contacts[hand]=traversal.topHandWeight(hand,phase);
            append(output,{float(samples++)*step,std::move(pose),traversal.position-origin,Quat{},result.motion,phase,
                contacts},ConverterActionPreviewStage::action,"contextMantle");
        } else if(began)throw std::runtime_error("Standard mantle changed action before reaching its top");
        else if(frame>=120)throw std::runtime_error("Edited mantle cannot start on the checked standard ledge route");
        if(result.completed){output.surfaceQueries=world.queries;if(!output.available())throw std::runtime_error("Standard mantle produced no complete preview");return output;}
    }
    throw std::runtime_error("Edited mantle cannot find a checked standard ledge route");
}
ConverterActionPreview wallRunPreview(const Library& library,const ConverterActionPreviewOptions& options) {
    if(!converter::isWallRunPrimary(options.direction)||options.cycles<1||options.cycles>4)throw std::runtime_error("Invalid complete wall-run preview direction or cycle count");
    const auto motion=motionFor(options.direction);const auto stages=converter::wallRunStages(options.direction);
    const auto& loop=library.clip(motion);if(!std::isfinite(loop.stride)||loop.stride<=0)throw std::runtime_error("Wall-run preview needs a positive cycle stride");
    PreviewWall world;Traversal traversal;library.configureThreepeat(traversal.cfg);traversal.cfg.approachSeconds=0;
    traversal.cfg.contextActions=traversal.cfg.automaticClimbActions=traversal.cfg.wallRunObstacleJumps=false;
    traversal.cfg.staminaEnabled=false;traversal.cfg.diagonalRunMultiplier=1;
    if(!traversal.attach(world,{0,-30,200},{0,1,0},1000))throw std::runtime_error("Standard preview wall cannot support the selected base configuration");
    SurfacePose surface;Pose pose;
    for(unsigned i=0;i<240;++i) {
        const auto result=traversal.update(world,{},step,1000);
        if(result.released)throw std::runtime_error("Standard preview wall lost initial support");
        pose=surface.update(library,world,traversal,result.motion,step,1);
        if(i>=30&&traversal.state==State::wall)break;
        if(i==239)throw std::runtime_error("Standard preview wall could not finish attachment");
    }
    const auto origin=traversal.position;ConverterActionPreview output;output.coreSimulated=true;output.wallDistance=-origin.y;
    append(output,{0,pose,{},Quat{},Motion::hang,surface.sampledPhase(),library.contactWeights(Motion::hang,surface.sampledPhase())},ConverterActionPreviewStage::launch,stages[0]);
    const auto heading=headingFor(motion);Input input;input.x=heading.x;input.y=heading.y;input.run=true;
    float distance=0,loopElapsed=0,stopped=0;bool loopStarted=false,stopping=false;
    for(unsigned i=1;i<=9000;++i) {
        const auto before=traversal.position;const auto result=traversal.update(world,input,step,1000);
        if(result.released||!traversal.active()||!isActiveMotion(result.motion))throw std::runtime_error("Standard wall-run preview rejected its route");
        if(!stopping&&result.motion==Motion::hang&&i>24)throw std::runtime_error("Standard wall-run preview could not advance on the selected route");
        pose=surface.update(library,world,traversal,result.motion,step,1);
        if(!stopping&&result.motion==motion&&surface.blendProgress()>=.999f)loopStarted=true;
        const auto stage=stopping?ConverterActionPreviewStage::catching:loopStarted?ConverterActionPreviewStage::loop:ConverterActionPreviewStage::launch;
        const auto slot=stopping?stages[2]:loopStarted?stages[1]:stages[0];
        append(output,{float(i)*step,std::move(pose),traversal.position-origin,Quat{},result.motion,surface.sampledPhase(),library.contactWeights(result.motion,surface.sampledPhase(),traversal.wallRunDirection(result.motion))},stage,slot);
        if(loopStarted&&!stopping) {
            distance+=(traversal.position-before).length();loopElapsed+=step;
            if(distance>=loop.stride*float(options.cycles)&&loopElapsed>=.20f){stopping=true;input={};}
        } else if(stopping) {
            stopped+=step;
            if(traversal.state==State::wall&&result.motion==Motion::hang&&surface.blendProgress()>=.999f&&stopped>=.25f)break;
        }
        if(i==9000)throw std::runtime_error("Standard wall-run preview exceeded its bounded duration");
    }
    output.surfaceQueries=world.queries;return output;
}
}
ConverterActionPreview buildConverterActionPreview(const ConverterEditorDocument& doc,const ConverterEditedAnimation& edited,const ConverterActionPreviewOptions& options,const std::vector<ConverterEditorExport>& overlays) {
    if(!edited.contextGroup.empty()){
        const auto& main=selectAnimationClipConfig(edited.contextGroup,doc.slot);const auto range=animationFrameRange(main,edited.clip.frames.size());const float first=float(range[0])/float(edited.clip.frames.size()-1),last=float(range[1])/float(edited.clip.frames.size()-1);const auto motion=motionFor(doc.slot);const auto destination=singleTarget(doc.base.clip(motion),motion);ConverterActionPreview output;output.wallDistance=30;
        for(std::size_t i=0;i<edited.clip.frames.size();++i){const float phase=float(i)/float(edited.clip.frames.size()-1),action=std::clamp((phase-first)/(last-first),0.f,1.f);ConverterActionPreviewFrame frame;frame.seconds=edited.clip.duration*phase;frame.phase=phase;frame.motion=motion;frame.pose=sampleConverterEditor(edited,frame.seconds);frame.contacts=sampleConverterContacts(edited,frame.seconds);frame.position=destination*smooth(action);frame.position.y-=32*std::sin(action*3.14159265f);append(output,std::move(frame),ConverterActionPreviewStage::action,doc.slot);}return output;
    }
    const auto storage=previewLibrary(doc,edited,options,overlays);const auto& library=*storage;
    if(options.kind==ConverterActionPreviewKind::wallRun) {
        if(converter::actionPrimaryForSlot(doc.slot)!="wallRun")throw std::runtime_error("Complete wall-run preview requires a wall-run group");
        return wallRunPreview(library,options);
    }
    if(doc.slot=="contextMantle")return mantlePreview(library);
    return singlePreview(doc,edited,library);
}
ConverterActionPreviewFrame sampleConverterActionPreview(const ConverterActionPreview& preview,float seconds) {
    if(!preview.available()||!std::isfinite(seconds))return {};
    seconds=std::clamp(seconds,0.f,preview.seconds);
    const auto upper=std::upper_bound(preview.frames.begin(),preview.frames.end(),seconds,[](float time,const auto& frame){return time<frame.seconds;});
    if(upper==preview.frames.begin())return preview.frames.front();
    if(upper==preview.frames.end())return preview.frames.back();
    const auto& a=*(upper-1);const auto& b=*upper;const float phase=(seconds-a.seconds)/(b.seconds-a.seconds);
    auto frame=a;frame.seconds=seconds;frame.position=a.position+(b.position-a.position)*phase;frame.orientation=blend(a.orientation,b.orientation,phase);
    for(std::size_t bone=0;bone<frame.pose.size();++bone)frame.pose[bone]=blend(a.pose[bone],b.pose[bone],phase);
    if(a.motion==b.motion){const bool wrap=b.phase<a.phase&&(authoredMovingLoop(a.motion)||authoredIdleLoop(a.motion));frame.phase=a.phase+(b.phase-a.phase+(wrap?1.f:0.f))*phase;if(wrap)frame.phase=std::fmod(frame.phase,1.f);}
    for(unsigned i=0;i<4;++i)frame.contacts[i]=a.contacts[i]+(b.contacts[i]-a.contacts[i])*phase;
    return frame;
}
Pose placedConverterActionPreviewPose(const ConverterActionPreviewFrame& frame) {
    auto pose=frame.pose;if(pose.empty())return pose;
    pose[0].t=frame.position+frame.orientation.rotate(pose[0].t);pose[0].q=(frame.orientation*pose[0].q).unit();return pose;
}
}
