#include "ConverterActionAuthoring.h"
#include "ConverterTimeline.h"
#include "ConverterWallRunGroup.h"

namespace fc {
namespace {
void requireAction(bool accepted,const char* message){if(!accepted)throw std::runtime_error(message);}
nlohmann::json sourceMetadata(const ConverterEditorDocument& document,std::string_view slot,std::string_view direction){
    const auto found=std::find_if(document.snapshots.begin(),document.snapshots.end(),[&](const auto& value){return value.path==document.pack;});requireAction(found!=document.snapshots.end(),"Complete action requires a loaded base pack");
    const auto selected=converter::isWallRunPrimary(direction)?direction:slot;
    const auto manifest=nlohmann::json::parse(found->bytes);for(const auto candidate:{selected,slot})for(const auto& entry:manifest.at("motions"))if(entry.at("slot").get<std::string>()==candidate){
        const auto path=std::filesystem::weakly_canonical(document.pack.parent_path()/entry.at("config").get<std::string>());
        const auto saved=std::find_if(document.snapshots.begin(),document.snapshots.end(),[&](const auto& value){return value.path==path;});requireAction(saved!=document.snapshots.end(),"Complete-action metadata is missing from the document snapshot");
        const auto metadata=nlohmann::json::parse(saved->bytes);if(metadata.value("format",std::string{})=="FreeClimbClip"&&metadata.value("slot",std::string{})!=slot)continue;
        return selectAnimationClipConfig(metadata,slot,direction);
    }throw std::runtime_error("Complete-action stage is absent from the base pack");
}
ConverterInputClip savedClip(const ConverterEditorDocument& document){
    const auto saved=std::find_if(document.snapshots.begin(),document.snapshots.end(),[&](const auto& value){return value.path==document.input;});requireAction(saved!=document.snapshots.end(),"Complete action HKX is missing from the document snapshot");
    ConverterInputClip result;std::string error;const auto member=document.templateConfig.value("member",std::string{});
    const bool decoded=decodeConverterInput(saved->bytes,result,error,member)||(member.empty()&&decodeConverterInput(saved->bytes,result,error,document.slot));requireAction(decoded,error.c_str());
    requireAction(result.frames.size()>=2&&result.frames.size()<=1201&&result.boneIndices.size()==99&&result.trackNames.size()==99,"Complete base action must contain canonical 99-bone frames");
    for(unsigned bone=0;bone<99;++bone)requireAction(result.boneIndices[bone]==int(bone),"Complete base action has a reordered binding");return result;
}
bool unchanged(const ConverterEditorDocument& document,const ConverterEditOptions& edits){
    return edits.trimIn==0&&(edits.trimOut==-1||edits.trimOut==document.clip.duration)&&edits.speed==1&&edits.yawDegrees==0&&edits.rootOffset.length()==0&&edits.comOffset.length()==0&&edits.makeInPlaceAxes.length()==0&&edits.bones.empty()&&edits.contacts.empty()&&edits.windows.empty()&&!edits.geometry;
}
ConverterEditedAnimation savedAnimation(const ConverterEditorDocument& document){
    ConverterEditedAnimation result;result.clip=document.clip;result.config=document.templateConfig;result.warnings=document.warnings;result.confidence=1;result.phaseMap={{0,0},{1,1}};return result;
}
}
ConverterWallRunDocument loadConverterWallRunEditor(const std::filesystem::path& manifest,const std::string& direction){
    requireAction(converter::isWallRunPrimary(direction),"Choose a wall-running direction");ConverterWallRunDocument result;const auto roles=converter::wallRunStages(direction);std::vector<ConverterInputClip> timelines;
    for(unsigned stage=0;stage<3;++stage){
        auto document=loadConverterBaseEditor(manifest,std::string(roles[stage]),direction);auto timeline=savedClip(document);document.clip=sliceConverterTimeline(timeline,document.templateConfig);document.preserveTemplateTiming=true;
        if(!result.stages.empty()){const auto& snapshots=result.stages.front().document.snapshots;requireAction(snapshots.size()==document.snapshots.size(),"Base pack changed while loading the complete action");for(const auto& saved:document.snapshots){const auto before=std::find_if(snapshots.begin(),snapshots.end(),[&](const auto& value){return value.path==saved.path;});requireAction(before!=snapshots.end()&&before->bytes==saved.bytes,"Base pack changed while loading the complete action");}}
        ConverterEditOptions options;options.autoCalibration=false;auto animation=savedAnimation(document);result.stages.emplace_back(std::move(document),std::move(animation),std::move(options));timelines.push_back(std::move(timeline));
    }
    result.document=result.stages[1].document;bool shared=true;
    for(const auto& stage:result.stages)shared=shared&&stage.document.input==result.document.input&&stage.document.templateConfig.contains("frameRange")&&stage.document.templateConfig.value("member",std::string{})==result.document.templateConfig.value("member",std::string{});
    if(shared){
        result.document.clip=std::move(timelines[1]);
        for(unsigned stage=0;stage<3;++stage){const auto& config=result.stages[stage].animation.config;result.ranges[stage]=config.at("frameRange").get<std::array<std::size_t,2>>();if(config.contains("rootShift")){const auto shift=config.at("rootShift").get<std::array<float,3>>();result.rootShifts[stage]={shift[0],shift[1],shift[2]};}}
    }else{
        std::vector<ConverterTimelinePart> parts;for(const auto& stage:result.stages)parts.push_back({stage.document.slot,stage.animation.clip});const auto joined=composeConverterTimeline(parts);result.document.clip=joined.clip;
        for(unsigned stage=0;stage<3;++stage){result.ranges[stage]=joined.ranges[stage].frames;result.rootShifts[stage]=joined.ranges[stage].rootShift;}result.document.warnings.insert(result.document.warnings.end(),joined.warnings.begin(),joined.warnings.end());
    }
    const auto count=result.document.clip.frames.size();requireAction(result.ranges[0][0]==0&&result.ranges[2][1]==count-1&&result.ranges[0][1]<=result.ranges[1][0]&&result.ranges[1][1]<=result.ranges[2][0],"Complete action stages are not ordered on one timeline");
    const float interval=result.document.clip.duration/float(count-1);result.cuts={interval*float(result.ranges[1][0]),interval*float(result.ranges[1][1])};
    result.document.authored=std::any_of(result.stages.begin(),result.stages.end(),[](const auto& stage){return stage.document.authored;});result.document.preserveTemplateTiming=true;result.document.templateConfig.erase("frameRange");result.document.templateConfig.erase("rootShift");result.document.templateConfig.erase("authoredPlayback");result.document.templateConfig["version"]=1;
    auto& contacts=result.document.templateConfig["contacts"];contacts=nlohmann::json::array();
    for(std::size_t frame=0;frame<count;++frame){
        std::array<float,4> row{};bool assigned=false;
        for(unsigned stage=0;stage<3;++stage){const auto range=result.ranges[stage];if(frame>=range[0]&&frame<=range[1]){row=sampleConverterContacts(result.stages[stage].animation,result.stages[stage].animation.clip.duration*float(frame-range[0])/float(range[1]-range[0]));assigned=true;break;}}
        if(!assigned)for(unsigned stage=1;stage<3;++stage)if(frame>result.ranges[stage-1][1]&&frame<result.ranges[stage][0]){const auto a=sampleConverterContacts(result.stages[stage-1].animation,result.stages[stage-1].animation.clip.duration),b=sampleConverterContacts(result.stages[stage].animation,0);const float phase=float(frame-result.ranges[stage-1][1])/float(result.ranges[stage][0]-result.ranges[stage-1][1]);for(unsigned limb=0;limb<4;++limb)row[limb]=std::lerp(a[limb],b[limb],phase);break;}
        contacts.push_back(row);
    }
    if(direction!="runUp"){
        auto brace=loadConverterBaseEditor(manifest,"sideBrace",direction);ConverterEditOptions options;options.autoCalibration=false;result.stages.emplace_back(brace,savedAnimation(brace),options);
    }return result;
}
bool applyConverterWallRunEdits(const ConverterWallRunDocument& source,const ConverterEditOptions& edits,ConverterWallRunCuts cuts,ConverterWallRunSequence& output,std::string& error){
    if(source.stages.empty())return authorWallRunSequence(source.document,edits,source.document.direction.empty()?source.document.slot:source.document.direction,cuts,output,error);
    output={};error.clear();try{
        requireAction((source.stages.size()==3||source.stages.size()==4)&&converter::isWallRunPrimary(source.document.slot),"Complete action requires three validated source stages and its optional private reference");requireAction(edits.windows.empty(),"Complete wall-running actions use the Contacts timeline");
        const auto& document=source.document;const float begin=edits.trimIn,end=edits.trimOut==-1?document.clip.duration:edits.trimOut;
        requireAction(std::isfinite(cuts.loopBegin)&&std::isfinite(cuts.loopEnd)&&begin<cuts.loopBegin&&cuts.loopBegin<cuts.loopEnd&&cuts.loopEnd<end,"Mark a nonempty start, loop and ending inside the selected trim range");
        const bool sameCuts=std::abs(cuts.loopBegin-source.cuts.loopBegin)<.000001f&&std::abs(cuts.loopEnd-source.cuts.loopEnd)<.000001f;
        ConverterWallRunSequence result;result.primary=document.slot;result.actualCuts=cuts;
        if(sameCuts&&unchanged(document,edits)){result.whole=savedAnimation(document);result.stages=source.stages;result.warnings=document.warnings;for(unsigned stage=0;stage<3;++stage)result.sourceRanges[stage]={document.clip.duration*float(source.ranges[stage][0])/float(document.clip.frames.size()-1),document.clip.duration*float(source.ranges[stage][1])/float(document.clip.frames.size()-1)};output=std::move(result);return true;}
        auto wholeOptions=edits;wholeOptions.geometry.reset();auto wholeDocument=std::make_unique<ConverterEditorDocument>(document);wholeDocument->authored=false;result.whole=applyConverterEdits(*wholeDocument,wholeOptions);result.warnings=result.whole.warnings;
        const auto count=result.whole.clip.frames.size();const float sourceInterval=(end-begin)/float(count-1),originalInterval=document.clip.duration/float(document.clip.frames.size()-1);
        const auto original=savedAnimation(document);auto& wholeContacts=result.whole.config["contacts"];wholeContacts=nlohmann::json::array();
        for(std::size_t frame=0;frame<count;++frame){const float phase=float(frame)/float(count-1);auto row=sampleConverterContacts(original,begin+(end-begin)*phase);for(const auto& contact:edits.contacts){float weight=0;if(phase>=contact.beginPhase&&phase<=contact.endPhase)weight=contact.fadePhase==0?1:std::clamp(smooth(std::min((phase-contact.beginPhase)/contact.fadePhase,(contact.endPhase-phase)/contact.fadePhase)),0.f,1.f);row[contact.contact]=std::lerp(row[contact.contact],contact.weight,weight);}wholeContacts.push_back(row);}
        auto index=[&](float seconds){return std::min(count-1,std::size_t(std::llround(std::clamp((seconds-begin)/sourceInterval,0.f,float(count-1)))));};
        std::array<std::array<std::size_t,2>,3> ranges;
        if(sameCuts){for(unsigned stage=0;stage<3;++stage)ranges[stage]={index(originalInterval*float(source.ranges[stage][0])),index(originalInterval*float(source.ranges[stage][1]))};}
        else{const auto first=index(cuts.loopBegin),last=index(cuts.loopEnd);ranges={{{0,first},{first,last},{last,count-1}}};}
        requireAction(ranges[0][0]==0&&ranges[2][1]==count-1,"Trim must retain a nonempty start and ending");for(const auto range:ranges)requireAction(range[0]<range[1],"Each marked section must contain at least two animation frames");
        result.actualCuts={begin+sourceInterval*float(ranges[1][0]),begin+sourceInterval*float(ranges[1][1])};
        const auto facing=Quat::axis({0,0,1},edits.yawDegrees*3.14159265358979323846f/180);
        for(unsigned stage=0;stage<3;++stage){
            result.sourceRanges[stage]={begin+sourceInterval*float(ranges[stage][0]),begin+sourceInterval*float(ranges[stage][1])};
            auto editedDocument=source.stages[stage].document;editedDocument.preserveTemplateTiming=true;const auto range=ranges[stage];const Vec shift=sameCuts?facing.rotate(source.rootShifts[stage]):result.whole.clip.frames[range[0]][0].t-edits.rootOffset-facing.rotate(source.stages[stage].animation.clip.frames.front()[0].t);
            editedDocument.clip=sliceConverterTimeline(result.whole.clip,{{"frameRange",range},{"rootShift",{shift.x,shift.y,shift.z}}});auto& config=editedDocument.templateConfig;config.erase("frameRange");config.erase("rootShift");config["contacts"]=nlohmann::json::array();
            for(std::size_t frame=range[0];frame<=range[1];++frame)config["contacts"].push_back(sampleConverterContacts(result.whole,result.whole.clip.duration*float(frame)/float(count-1)));
            ConverterEditOptions stageOptions;stageOptions.autoCalibration=false;if(editedDocument.authored)stageOptions.rootOffset=editedDocument.clip.frames.front()[0].t;if(stage==1&&edits.geometry)stageOptions.geometry=edits.geometry;
            auto animation=editedDocument.authored||stageOptions.geometry?applyConverterEdits(editedDocument,stageOptions):savedAnimation(editedDocument);editedDocument.templateConfig=animation.config;
            result.stages.emplace_back(std::move(editedDocument),std::move(animation),std::move(stageOptions));
        }
        if(source.stages.size()==4){auto brace=source.stages[3];auto options=edits;options.trimIn=0;options.trimOut=-1;options.speed=1;options.geometry.reset();options.contacts.clear();options.windows.clear();options.autoCalibration=false;brace.animation=applyConverterEdits(brace.document,options);result.stages.push_back(std::move(brace));}
        if(std::abs(result.actualCuts.loopBegin-cuts.loopBegin)>.00001f||std::abs(result.actualCuts.loopEnd-cuts.loopEnd)>.00001f)result.warnings.push_back("Loop markers were aligned to the edited timeline's nearest animation frames.");
        output=std::move(result);return true;
    }catch(const std::exception& failure){error=failure.what();return false;}
}
bool authorWallRunSequence(const ConverterEditorDocument& source,const ConverterEditOptions& edits,std::string_view primary,
    ConverterWallRunCuts cuts,ConverterWallRunSequence& output,std::string& error){
    output={};error.clear();try{
        requireAction(converter::isWallRunPrimary(primary),"Choose a wall-running direction");const float begin=edits.trimIn,end=edits.trimOut==-1?source.clip.duration:edits.trimOut;
        requireAction(std::isfinite(cuts.loopBegin)&&std::isfinite(cuts.loopEnd)&&begin<cuts.loopBegin&&cuts.loopBegin<cuts.loopEnd&&cuts.loopEnd<end,"Mark a nonempty start, loop and ending inside the selected trim range");
        requireAction(edits.windows.empty(),"A complete imported action uses Contacts rather than base action windows");
        auto wholeStorage=std::make_unique<ConverterEditorDocument>(source);auto& wholeDocument=*wholeStorage;wholeDocument.slot=std::string(primary);wholeDocument.direction=std::string(primary);wholeDocument.authored=true;wholeDocument.templateConfig=sourceMetadata(source,primary,primary);
        auto whole=applyConverterEdits(wholeDocument,edits);const auto count=whole.clip.frames.size();const float sourceInterval=(end-begin)/float(count-1);
        const auto loopFirst=std::size_t(std::llround((cuts.loopBegin-begin)/sourceInterval)),loopLast=std::size_t(std::llround((cuts.loopEnd-begin)/sourceInterval));
        requireAction(loopFirst>0&&loopLast>loopFirst&&loopLast<count-1,"Each marked section must contain at least two animation frames");
        ConverterWallRunSequence result;result.primary=std::string(primary);result.actualCuts={begin+float(loopFirst)*sourceInterval,begin+float(loopLast)*sourceInterval};
        result.whole=whole;result.warnings=whole.warnings;if(std::abs(result.actualCuts.loopBegin-cuts.loopBegin)>.00001f||std::abs(result.actualCuts.loopEnd-cuts.loopEnd)>.00001f)result.warnings.push_back("Loop markers were aligned to the nearest source preview frames; all three sections retain one continuous frame grid.");
        const auto roles=converter::wallRunStages(primary);const std::array<std::array<std::size_t,2>,3> ranges{{{0,loopFirst},{loopFirst,loopLast},{loopLast,count-1}}};
        for(std::size_t stage=0;stage<3;++stage){
            result.sourceRanges[stage]={begin+sourceInterval*float(ranges[stage][0]),begin+sourceInterval*float(ranges[stage][1])};
            auto stageStorage=std::make_unique<ConverterEditorDocument>(wholeDocument);auto& document=*stageStorage;document.slot=std::string(roles[stage]);document.templateConfig=sourceMetadata(source,roles[stage],primary);document.preserveTemplateTiming=true;
            const auto range=ranges[stage];const auto origin=whole.clip.frames[range[0]][0].t-edits.rootOffset;
            document.clip=sliceConverterTimeline(whole.clip,{{"frameRange",range},{"rootShift",{origin.x,origin.y,origin.z}}});
            document.templateConfig.erase("frameRange");document.templateConfig.erase("rootShift");document.templateConfig["contacts"]=nlohmann::json::array();
            for(std::size_t frame=range[0];frame<=range[1];++frame)document.templateConfig["contacts"].push_back(sampleConverterContacts(whole,whole.clip.duration*float(frame)/float(count-1)));
            ConverterEditOptions options;options.autoCalibration=false;options.rootOffset=edits.rootOffset;
            if(stage==1&&edits.geometry)options.geometry=edits.geometry;
            auto animation=applyConverterEdits(document,options);document.templateConfig=animation.config;
            result.stages.emplace_back(std::move(document),std::move(animation),std::move(options));
        }
        const auto& loop=result.stages[1].animation.clip;float seam=0;for(unsigned bone=4;bone<97;++bone)if(converterEditableBone(source.base,bone))seam=std::max(seam,angleBetween(loop.frames.front()[bone].q,loop.frames.back()[bone].q));
        if(seam>15.f*3.14159265358979323846f/180)result.warnings.push_back("The selected loop has a visible end-to-start pose difference. Adjust its two markers or edit the source loop; the tool does not invent a new gait.");
        result.warnings.push_back("This replaces the selected direction's complete start, loop and ending. Other directions and the local brace reference retain their current actions.");
        output=std::move(result);return true;
    }catch(const std::exception& failure){error=failure.what();return false;}
}
}
