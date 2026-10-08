#include "ConverterRuntimePreview.h"
#include "ConverterWallRunGroup.h"
#include "../../src/MotionSlots.h"
#include <stdexcept>

namespace fc {
namespace {
struct PreviewWall final:World {
    float y{};
    std::size_t queries{};
    std::optional<Hit> ray(Vec a,Vec b) override {
        ++queries;const float delta=b.y-a.y;
        if(std::abs(delta)<.00001f)return {};
        const float phase=(y-a.y)/delta;
        if(phase<0||phase>1)return {};
        return Hit{a+(b-a)*phase,{0,-1,0},true};
    }
};
Vec direction(Motion motion) {
    if(motion==Motion::runLeft)return {-1,0,0};
    if(motion==Motion::runRight)return {1,0,0};
    if(motion==Motion::runDiagonalLeft)return Vec{-1,1,0}.unit();
    if(motion==Motion::runDiagonalRight)return Vec{1,1,0}.unit();
    return {0,1,0};
}
}
bool converterRuntimePreviewSupported(std::string_view slot) {
    return slot=="runUp"||slot=="runLeft"||slot=="runRight"||slot=="runDiagonalLeft"||slot=="runDiagonalRight";
}
ConverterRuntimePreview buildConverterRuntimePreview(const ConverterEditorDocument& doc,const ConverterEditedAnimation& edited,const std::vector<ConverterEditorExport>& overlays) {
    ConverterRuntimePreview output;
    if(!converterRuntimePreviewSupported(doc.slot))return output;
    const auto found=std::find(motionSlotNames.begin(),motionSlotNames.end(),doc.slot);
    if(found==motionSlotNames.end()||edited.clip.frames.size()<2||edited.clip.frames.size()>1201||edited.clip.duration<=0)
        throw std::runtime_error("Invalid wall-run preview data");
    const auto motion=Motion(int(found-motionSlotNames.begin())+1);auto storage=std::make_unique<Library>(doc.base);auto& library=*storage;library.clearAnimationOverrides();
    auto applyClip=[&](std::string_view name,const ConverterEditedAnimation& value,std::string_view owner={}){
        const auto at=std::find(motionSlotNames.begin(),motionSlotNames.end(),name);if(at==motionSlotNames.end())throw std::runtime_error("Invalid preview overlay slot");
        auto& target=name=="sideBrace"&&converter::isWallRunPrimary(owner)?library.wallRunBraces[wallRunDirectionIndex(Motion(std::find(motionSlotNames.begin(),motionSlotNames.end(),owner)-motionSlotNames.begin()+1))]:library.clips[std::size_t(at-motionSlotNames.begin())];target.frames=value.clip.frames;target.seconds=value.clip.duration;
        target.stride=value.config.at("stride").get<float>();target.height=value.config.at("height").get<float>();
        const auto& travel=value.config.at("travel");target.travel={travel[0].get<float>(),travel[1].get<float>(),travel[2].get<float>()};
        target.authoredPlayback=value.config.contains("authoredPlayback");target.trajectory={};target.contacts.clear();
        for(const auto& row:value.config.at("contacts"))target.contacts.push_back(row.get<std::array<float,4>>());
        if(target.authoredPlayback&&value.config.at("authoredPlayback").contains("trajectory")) {
            const auto& rows=value.config.at("authoredPlayback").at("trajectory");if(rows.size()>65)throw std::runtime_error("Invalid preview Root trajectory");
            target.trajectory.count=std::uint32_t(rows.size());for(std::size_t i=0;i<rows.size();++i)target.trajectory.knots[i]={rows[i][0].get<float>(),{rows[i][1].get<float>(),rows[i][2].get<float>(),rows[i][3].get<float>()}};
        }
    };
    const auto members=converter::actionStages("wallRun");if(overlays.size()>members.size())throw std::runtime_error("Too many preview overlays");
    for(const auto& overlay:overlays){std::error_code error;if(!overlay.document||!overlay.animation||!std::filesystem::equivalent(overlay.document->pack,doc.pack,error)||error||std::find(members.begin(),members.end(),overlay.document->slot)==members.end())throw std::runtime_error("Invalid preview overlay base pack or group");applyClip(overlay.document->slot,*overlay.animation,overlay.direction.empty()?overlay.document->direction:overlay.direction);}
    applyClip(doc.slot,edited);const auto& clip=library.clips[std::size_t(int(motion)-1)];
    PreviewWall world;Traversal traversal;library.configureThreepeat(traversal.cfg);traversal.cfg.approachSeconds=0;
    traversal.cfg.contextActions=traversal.cfg.automaticClimbActions=traversal.cfg.wallRunObstacleJumps=false;
    traversal.cfg.staminaEnabled=false;traversal.cfg.diagonalRunMultiplier=1;
    if(!traversal.attach(world,{0,-30,0},{0,1,0},100))throw std::runtime_error("Standard preview wall cannot support the selected base configuration");
    const auto heading=direction(motion);Input input;input.x=heading.x;input.y=heading.y;input.run=true;input.modeBlend=1;
    traversal.update(world,input,1.f/120,100);traversal.state=State::wall;traversal.position={};world.y=traversal.cfg.gap;
    const auto intervals=std::clamp(std::size_t(std::ceil(clip.seconds*120)),std::size_t(120),std::size_t(1200));
    const float dt=clip.seconds/float(intervals);const Vec travelPerFrame{heading.x*clip.stride/float(intervals),0,heading.y*clip.stride/float(intervals)};
    SurfacePose surface;output.seconds=clip.seconds;output.wallDistance=world.y;output.frames.reserve(intervals+1);
    for(std::size_t frame=0;frame<=intervals*3;++frame) {
        if(frame)traversal.position=traversal.position+travelPerFrame;
        auto pose=surface.update(library,world,traversal,motion,dt,1);
        if(frame>=intervals*2)output.frames.push_back(std::move(pose));
    }
    output.surfaceQueries=world.queries;return output;
}
Pose sampleConverterRuntimePreview(const ConverterRuntimePreview& preview,float seconds) {
    if(!preview.available()||!std::isfinite(seconds))return {};
    const float at=std::clamp(seconds/preview.seconds,0.f,1.f)*float(preview.frames.size()-1);
    const auto index=std::min(std::size_t(at),preview.frames.size()-2);Pose result=preview.frames[index];
    for(std::size_t bone=0;bone<result.size();++bone)result[bone]=blend(result[bone],preview.frames[index+1][bone],at-float(index));
    return result;
}
}

