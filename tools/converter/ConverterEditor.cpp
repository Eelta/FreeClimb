#include "ConverterEditor.h"
#include "ConverterCalibration.h"
#include "ConverterSourceMotion.h"
#include "ConverterTimeline.h"
#include "../../src/AnimationPack.h"
#include "../../src/SceneBinding.h"
#include <fstream>
#include <set>

namespace fc {
namespace {
using Json=nlohmann::json;
void require(bool ok,const std::string& text){if(!ok)throw std::runtime_error(text);}
std::vector<std::uint8_t> read(const std::filesystem::path& path,std::size_t limit){
    require(std::filesystem::is_regular_file(path),"Editor input is not a regular file");const auto size=std::filesystem::file_size(path);
    require(size&&size<=limit,"Editor input is empty or exceeds its size limit");std::vector<std::uint8_t> bytes(std::size_t(size),0);
    std::ifstream stream(path,std::ios::binary);require(bool(stream.read(reinterpret_cast<char*>(bytes.data()),std::streamsize(bytes.size())))&&stream.peek()==std::char_traits<char>::eof(),"Editor input changed while being read");return bytes;
}
std::filesystem::path packPath(const std::filesystem::path& root,const Json& value){
    require(value.is_string(),"Expected a relative pack path");const auto text=value.get<std::string>();
    require(!text.empty()&&text.size()<=240&&text.find(':')==std::string::npos&&text.find('\0')==std::string::npos,"Invalid editor pack path");
    const std::filesystem::path child(std::u8string(reinterpret_cast<const char8_t*>(text.data()),text.size()));
    require(!child.is_absolute()&&!child.has_root_path(),"Editor pack paths must be relative");for(const auto& part:child)require(part!=".."&&part!=".","Unsafe editor pack path");
    const auto result=std::filesystem::weakly_canonical(root/child),relative=result.lexically_relative(root);
    require(!relative.empty()&&!relative.is_absolute(),"Editor pack path escapes its directory");for(const auto& part:relative)require(part!="..","Editor pack symlink escapes its directory");return result;
}
Json json(const std::vector<std::uint8_t>& bytes){
    std::vector<std::set<std::string>> keys;auto callback=[&](int depth,Json::parse_event_t event,Json& value){
        require(depth<=32,"Editor metadata nesting exceeds limit");if(event==Json::parse_event_t::object_start)keys.emplace_back();
        else if(event==Json::parse_event_t::key)require(!keys.empty()&&keys.back().insert(value.get<std::string>()).second,"Duplicate editor metadata key");
        else if(event==Json::parse_event_t::object_end)keys.pop_back();return true;
    };auto result=Json::parse(bytes.begin(),bytes.end(),callback);require(result.is_object(),"Expected editor metadata object");return result;
}
bool finite(Vec v){return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);}
float phaseAt(const ConverterInputClip& clip,float seconds){require(std::isfinite(seconds)&&clip.duration>0,"Invalid preview time");return std::clamp(seconds/clip.duration,0.f,1.f);}
Pose sample(const ConverterInputClip& clip,float phase){
    require(clip.frames.size()>=2&&clip.frames.size()<=1201,"Editor clip has no valid frames");phase=std::clamp(phase,0.f,1.f);
    const float at=phase*float(clip.frames.size()-1);const auto a=std::min(std::size_t(at),clip.frames.size()-2);const auto& first=clip.frames[a];const auto& next=clip.frames[a+1];
    require(first.size()==99&&next.size()==99,"Preview requires a canonical 99-bone clip");Pose result(99);for(std::size_t bone=0;bone<99;++bone)result[bone]=blend(first[bone],next[bone],at-float(a));return result;
}
std::array<float,4> contacts(const Json& config,float phase){
    const auto& rows=config.at("contacts");require(rows.is_array()&&rows.size()>=2&&rows.size()<=1201,"Invalid editor contact samples");
    const float at=std::clamp(phase,0.f,1.f)*float(rows.size()-1);const auto a=std::min(std::size_t(at),rows.size()-2);std::array<float,4> result{};
    for(int limb=0;limb<4;++limb){require(rows[a].size()==4&&rows[a+1].size()==4,"Contact samples require four weights");const float x=rows[a][limb].get<float>(),y=rows[a+1][limb].get<float>();require(std::isfinite(x)&&std::isfinite(y)&&x>=0&&x<=1&&y>=0&&y<=1,"Contact weight is outside 0..1");result[limb]=std::lerp(x,y,at-float(a));}return result;
}
float envelope(float phase,float begin,float end,float fade){
    if(phase<begin||phase>end)return 0;if(fade==0)return 1;return std::clamp(smooth(std::min((phase-begin)/fade,(end-phase)/fade)),0.f,1.f);
}
void editWindow(float begin,float end,float fade){require(std::isfinite(begin)&&std::isfinite(end)&&std::isfinite(fade)&&begin>=0&&end<=1&&end-begin>=.001f&&fade>=0&&fade<=.5f*(end-begin),"Edit phase window or fade is outside its bounds");}
std::array<float,2> window(const Json& value){require(value.is_array()&&value.size()==2,"Special action requires a phase window");const float a=value[0].get<float>(),b=value[1].get<float>();editWindow(a,b,0);return {a,b};}
bool hop(std::string_view slot){return slot=="contextHopLeft"||slot=="contextHopRight";}
bool movingCycle(std::string_view slot){constexpr std::array<std::string_view,9> names{"up","down","left","right","runUp","runLeft","runRight","runDiagonalLeft","runDiagonalRight"};return std::find(names.begin(),names.end(),slot)!=names.end();}
void validateSpecial(const Json& config,std::string_view slot){
    if(hop(slot)){
        const auto& path=config.at("path");require(path.is_array()&&path.size()>=2&&path.size()<=64,"Side-action paths require 2..64 knots");
        float previous=-1;for(const auto& row:path){require(row.is_array()&&row.size()==4,"Path knots require four components");const float phase=row[0].get<float>(),progress=row[1].get<float>(),lift=row[2].get<float>(),out=row[3].get<float>();
            require(std::isfinite(phase)&&phase>=0&&phase<=1&&phase-previous>=.001f&&std::isfinite(progress)&&progress>=-.25f&&progress<=1.5f&&std::isfinite(lift)&&lift>=-32&&lift<=96&&std::isfinite(out)&&out>=0&&out<=48,"Path knot exceeds its bounds or does not advance");previous=phase;}
        const auto& first=path.front();const auto& last=path.back();require(first[0]==0&&last[0]==1&&std::abs(first[1].get<float>())<.0001f&&std::abs(last[1].get<float>()-1)<.0001f&&std::abs(first[2].get<float>())<.0001f&&std::abs(last[2].get<float>())<.0001f&&std::abs(first[3].get<float>())<.0001f&&std::abs(last[3].get<float>())<.0001f,"Path endpoints must remain at their verified anchors");
        for(const auto* key:{"sourceHands","targetHands"})require(config.at(key).is_array()&&config.at(key).size()==2,"Two special hand windows are required");
        float released=0,acquired=1;for(int hand=0;hand<2;++hand){const auto a=window(config.at("sourceHands")[hand]),b=window(config.at("targetHands")[hand]);require(a[1]<=b[0],"A hand cannot hold both action endpoints simultaneously");released=std::max(released,a[1]);acquired=std::min(acquired,b[0]);}
        const auto vertical=window(config.at("verticalBlend"));require(vertical[0]>=released&&vertical[1]<=acquired,"Vertical movement must remain between source release and destination acquisition");
    }else if(slot=="contextMantle"){
        const auto unplant=window(config.at("unplant")),replant=window(config.at("replant"));const auto& release=config.at("releaseHands");require(release.is_array()&&release.size()==2,"Two mantle hand windows are required");
        const auto left=window(release[0]),right=window(release[1]);const float at=config.at("replantSamplePhase").get<float>();
        require(std::isfinite(at)&&at>=replant[0]&&at<=replant[1]&&replant[1]<=left[0]&&replant[1]<=right[0]&&unplant[1]<=replant[0],"Mantle release, replant and calibration windows are inconsistent");
    }
}
void applyWindows(Json& config,std::string_view slot,const std::vector<ConverterWindowEdit>& edits){
    require(edits.size()<=12,"Too many special timing edits");std::set<std::string> seen;
    for(const auto& edit:edits){require(seen.insert(edit.role).second,"Duplicate special timing role");
        if(slot=="contextMantle"&&edit.role=="replantSample"){require(std::isfinite(edit.beginPhase)&&edit.beginPhase>=0&&edit.beginPhase<=1,"Invalid replant sample phase");config["replantSamplePhase"]=edit.beginPhase;continue;}
        editWindow(edit.beginPhase,edit.endPhase,0);const Json value={edit.beginPhase,edit.endPhase};bool accepted=false;
        if(hop(slot)){
            if(edit.role=="vertical"){config["verticalBlend"]=value;accepted=true;}
            for(int hand=0;hand<2;++hand){const std::string side=hand==0?"Left":"Right";if(edit.role=="source"+side){config["sourceHands"][hand]=value;accepted=true;}if(edit.role=="target"+side){config["targetHands"][hand]=value;accepted=true;}}
        }else if(slot=="contextMantle"){
            if(edit.role=="unplant"||edit.role=="replant"){config[edit.role]=value;accepted=true;}
            for(int hand=0;hand<2;++hand)if(edit.role==(hand==0?"releaseLeft":"releaseRight")){config["releaseHands"][hand]=value;accepted=true;}
        }
        require(accepted,"This special timing role is unavailable for the selected slot");
    }validateSpecial(config,slot);
}
std::array<float,4> referenceSample(const ConverterReferenceFrame& reference,float phase){
    require(reference.samples.size()>=2,"Invalid extracted root samples");const float at=std::clamp(phase,0.f,1.f)*float(reference.samples.size()-1);const auto a=std::min(std::size_t(at),reference.samples.size()-2);std::array<float,4> result{};
    for(int k=0;k<4;++k)result[k]=reference.samples[a][k]+(reference.samples[a+1][k]-reference.samples[a][k])*(at-float(a));return result;
}
void addWarnings(std::vector<std::string>& to,const std::vector<std::string>& from){for(const auto& text:from)if(std::find(to.begin(),to.end(),text)==to.end())to.push_back(text);}
}
ConverterEditorDocument loadConverterEditor(const std::filesystem::path& input,const std::filesystem::path& pack,const std::string& slot,const std::string& direction){
    ConverterEditorDocument doc;doc.input=std::filesystem::weakly_canonical(input);doc.pack=std::filesystem::weakly_canonical(pack);doc.slot=slot;doc.direction=direction;
    require(doc.pack.filename()=="pack.json","Select the complete FreeClimb pack.json");const auto found=std::find(motionSlotNames.begin(),motionSlotNames.end(),slot);
    require(!slot.empty()&&found!=motionSlotNames.end()&&isActiveMotion(Motion(int(found-motionSlotNames.begin())+1)),"Unknown active editor slot");
    std::set<std::filesystem::path> seen;std::size_t total=0;auto capture=[&](const std::filesystem::path& path,std::size_t limit){
        const auto bytes=read(path,limit);if(seen.insert(path).second){require(bytes.size()<=256*1024*1024-total,"Editor document exceeds 256 MiB");total+=bytes.size();doc.snapshots.push_back({path,bytes});}return bytes;
    };
    const auto raw=capture(doc.input,64*1024*1024);const auto manifest=json(capture(doc.pack,65536));const auto root=doc.pack.parent_path();
    capture(packPath(root,manifest.at("skeleton")),256*1024);require(manifest.at("motions").is_array()&&manifest.at("motions").size()==activeMotionCount,"Editor base requires all active slots");
    const auto owner=direction.empty()?slot:direction;
    for(const auto& entry:manifest.at("motions")){const auto metadata=selectAnimationClipConfig(json(capture(packPath(root,entry.at("config")),256*1024)),entry.at("slot").get<std::string>());capture(packPath(root,metadata.at("file")),64*1024*1024);if(entry.at("slot")==owner)doc.templateConfig=selectAnimationClipConfig(json(capture(packPath(root,entry.at("config")),256*1024)),slot,direction);}
    require(!doc.templateConfig.empty(),"Selected editor slot is missing");const auto loaded=loadAnimationPack(doc.base,doc.pack);require(loaded.committed,"Editor base pack rejected: "+loaded.error);
    const auto member=doc.input==packPath(root,doc.templateConfig.at("file"))?doc.templateConfig.value("member",std::string{}):std::string{};
    ConverterInputClip source;std::string error;const bool decoded=decodeConverterInput(raw,source,error,member)||
        (member.empty()&&decodeConverterInput(raw,source,error,slot));require(decoded,error);
    if(doc.input==packPath(root,doc.templateConfig.at("file"))){
        if(hop(slot)&&!doc.templateConfig.contains("authoredPlayback"))for(const auto& entry:manifest.at("motions"))if(entry.at("slot")==slot){const auto group=json(capture(packPath(root,entry.at("config")),256*1024));if(group.value("format",std::string{})=="FreeClimbActionGroup"&&group.value("version",0)==2&&group.value("group",std::string{})=="contextHop")doc.contextGroup=group;}
        if(doc.contextGroup.empty())source=sliceConverterTimeline(source,doc.templateConfig);
    }
    if(!doc.contextGroup.empty())doc.clip=source;else{const auto canonical=writeConverterHkx(source,doc.base,doc.warnings);require(decodeConverterInput(canonical,doc.clip,error),"Canonical editor clip failed decoding: "+error);}
    doc.authored=doc.input!=packPath(root,doc.templateConfig.at("file"))||doc.templateConfig.contains("authoredPlayback");
    if(!doc.contextGroup.empty()){
        const auto& sequence=doc.contextGroup.at("sequences").at(0);const std::array<Json,3> parts{sequence.at("prepare"),selectAnimationClipConfig(doc.contextGroup,slot),sequence.at("catch")};auto& weights=doc.templateConfig["contacts"];weights=Json::array();
        for(std::size_t frame=0;frame<doc.clip.frames.size();++frame){std::array<float,4> row{};for(const auto& part:parts){const auto range=animationFrameRange(part,doc.clip.frames.size());if(frame<range[0])break;row=contacts(part,std::clamp(float(frame-range[0])/float(range[1]-range[0]),0.f,1.f));}weights.push_back(row);}
        doc.templateConfig.erase("frameRange");doc.templateConfig.erase("rootShift");doc.preserveTemplateTiming=true;
    }
    for(const auto& saved:doc.snapshots)require(read(saved.path,64*1024*1024)==saved.bytes,"Input or base pack changed while loading editor");return doc;
}
ConverterEditorDocument loadConverterBaseEditor(const std::filesystem::path& manifest,const std::string& slot,const std::string& direction){
    const auto pack=std::filesystem::weakly_canonical(manifest),root=pack.parent_path();const auto metadata=json(read(pack,65536));require(metadata.at("motions").is_array()&&metadata.at("motions").size()==activeMotionCount,"Editor base requires all active slots");
    for(const auto& entry:metadata.at("motions"))if(entry.at("slot")== (direction.empty()?slot:direction)){const auto config=selectAnimationClipConfig(json(read(packPath(root,entry.at("config")),256*1024)),slot,direction);const auto input=packPath(root,config.at("file"));auto doc=loadConverterEditor(input,pack,slot,direction);require(packPath(root,doc.templateConfig.at("file"))==input,"Base slot changed while loading editor");return doc;}
    throw std::runtime_error("Selected base slot is missing");
}
bool converterEditableBone(const Library& base,unsigned bone){return base.names.size()==99&&base.parents.size()==99&&bone<97&&(bone==0||bone>=4)&&!engineOwnedTrack(bone,base.names,base.parents);}
bool converterEditableStride(const ConverterEditorDocument& doc,const ConverterEditedAnimation& edited){return doc.authored&&movingCycle(doc.slot)&&edited.clip.frames.size()>=2&&(edited.clip.frames.back()[0].t-edited.clip.frames.front()[0].t).length()<2;}
ConverterEditedAnimation applyConverterEdits(const ConverterEditorDocument& doc,const ConverterEditOptions& options){
    if(!doc.authored&&!doc.contextGroup.empty()&&options.trimIn==0&&(options.trimOut==-1||options.trimOut==doc.clip.duration)&&options.speed==1&&options.yawDegrees==0&&options.rootOffset.length()==0&&options.comOffset.length()==0&&options.makeInPlaceAxes.length()==0&&options.bones.empty()&&options.contacts.empty()&&options.windows.empty()&&!options.geometry){ConverterEditedAnimation result;result.clip=doc.clip;result.config=doc.templateConfig;result.warnings=doc.warnings;result.confidence=1;result.phaseMap={{0,0},{1,1}};result.contextGroup=doc.contextGroup;return result;}
    const auto slot=std::find(motionSlotNames.begin(),motionSlotNames.end(),doc.slot);require(!doc.slot.empty()&&slot!=motionSlotNames.end()&&isActiveMotion(Motion(int(slot-motionSlotNames.begin())+1)),"Unknown active editor slot");
    auto original=doc.clip;bakeConverterSourceMotion(original);const float begin=options.trimIn,end=options.trimOut==-1?original.duration:options.trimOut;
    require(std::isfinite(begin)&&std::isfinite(end)&&begin>=0&&end<=original.duration&&end>begin,"Trim range must lie inside the source animation");
    require(std::isfinite(options.speed)&&options.speed>=.1f&&options.speed<=4,"Speed must be within 0.1..4");const float duration=(end-begin)/options.speed;require(duration>0&&duration<=10,"Edited animation duration must be within 0..10 seconds");
    require(std::isfinite(options.yawDegrees)&&std::abs(options.yawDegrees)<=360&&finite(options.rootOffset)&&finite(options.comOffset)&&options.rootOffset.length()<=200&&options.comOffset.length()<=200,"Facing or position correction exceeds its bounds");
    require(finite(options.makeInPlaceAxes)&&(options.makeInPlaceAxes.x==0||options.makeInPlaceAxes.x==1)&&(options.makeInPlaceAxes.y==0||options.makeInPlaceAxes.y==1)&&(options.makeInPlaceAxes.z==0||options.makeInPlaceAxes.z==1),"In-place axes must be disabled or enabled");
    require(options.bones.size()<=32&&options.contacts.size()<=32,"Too many bone or contact corrections");
    std::set<unsigned> corrected;for(const auto& edit:options.bones){require(converterEditableBone(doc.base,edit.bone)&&corrected.insert(edit.bone).second&&finite(edit.eulerDegrees)&&std::abs(edit.eulerDegrees.x)<=45&&std::abs(edit.eulerDegrees.y)<=45&&std::abs(edit.eulerDegrees.z)<=45,"Bone correction targets an engine-owned/protected bone or exceeds its axis bounds");editWindow(edit.beginPhase,edit.endPhase,edit.fadePhase);}
    require(options.contacts.empty()||doc.authored||!doc.contextGroup.empty()||(!hop(doc.slot)&&doc.slot!="contextMantle"),"Special actions use source/target or mantle timing markers; generic contact overrides are unavailable");
    for(const auto& edit:options.contacts){require(edit.contact<4&&std::isfinite(edit.weight)&&edit.weight>=0&&edit.weight<=1,"Invalid contact correction");editWindow(edit.beginPhase,edit.endPhase,edit.fadePhase);}
    ConverterEditedAnimation result;result.clip=original;result.clip.duration=duration;result.warnings=doc.warnings;const float firstPhase=begin/original.duration,lastPhase=end/original.duration;
    auto intervals=std::max(std::size_t(1),std::size_t(std::ceil((lastPhase-firstPhase)*float(original.frames.size()-1))));
    if(begin>0||end<original.duration)intervals=std::max(intervals,std::min(std::size_t(1200),std::size_t(std::ceil((end-begin)*120))));
    if(!options.bones.empty()||!options.contacts.empty()){intervals=std::max(intervals,std::size_t(64));auto represent=[&](float start,float finish,float fade){const float detail=fade>0?std::min(fade,finish-start):finish-start;intervals=std::max(intervals,std::size_t(std::min(1200.f,std::ceil(4/detail))));};
        for(const auto& edit:options.bones)represent(edit.beginPhase,edit.endPhase,edit.fadePhase);for(const auto& edit:options.contacts)represent(edit.beginPhase,edit.endPhase,edit.fadePhase);
        auto visible=[&](float start,float finish,float fade){float peak=0;for(std::size_t frame=0;frame<=intervals;++frame)peak=std::max(peak,envelope(float(frame)/float(intervals),start,finish,fade));require(peak>=.5f,"Edit window is too narrow for the bounded sample count; widen its interval or fade");};
        for(const auto& edit:options.bones)visible(edit.beginPhase,edit.endPhase,edit.fadePhase);for(const auto& edit:options.contacts)visible(edit.beginPhase,edit.endPhase,edit.fadePhase);
    }
    if(begin>0||end<original.duration){const double sourceIntervals=double(original.frames.size()-1),span=double(lastPhase-firstPhase)*sourceIntervals,offset=double(firstPhase)*sourceIntervals;
        for(std::size_t multiple=1;multiple<=1200;++multiple){const double count=span*double(multiple);if(count>1200.001)break;const auto aligned=std::size_t(std::llround(count));if(aligned<intervals)continue;
            if(std::abs(count-double(aligned))<.002&&std::abs(offset*double(multiple)-std::round(offset*double(multiple)))<.002){intervals=aligned;break;}}
    }
    require(intervals<1201,"Edited clip has too many samples");result.clip.frames.resize(intervals+1);
    const Vec rootOrigin=doc.authored?sample(original,firstPhase)[0].t:Vec{};
    const float radians=options.yawDegrees*(3.14159265358979323846f/180);const auto facing=Quat::axis({0,0,1},radians);
    for(std::size_t frame=0;frame<=intervals;++frame){const float phase=float(frame)/float(intervals);auto pose=sample(original,firstPhase+(lastPhase-firstPhase)*phase);
        pose[0].t=facing.rotate(pose[0].t-rootOrigin)+options.rootOffset;pose[0].q=(facing*pose[0].q).unit();pose[4].t=pose[4].t+options.comOffset;
        for(const auto& edit:options.bones){const auto delta=(Quat::axis({1,0,0},edit.eulerDegrees.x*3.14159265358979323846f/180)*Quat::axis({0,1,0},edit.eulerDegrees.y*3.14159265358979323846f/180)*Quat::axis({0,0,1},edit.eulerDegrees.z*3.14159265358979323846f/180)).unit();require(angleBetween({},delta)<=3.14159265358979323846f/3+.00001f,"Combined bone rotation exceeds 60 degrees");pose[edit.bone].q=(pose[edit.bone].q*blend({},delta,envelope(phase,edit.beginPhase,edit.endPhase,edit.fadePhase))).unit();}
        result.clip.frames[frame]=std::move(pose);
    }
    if(options.makeInPlaceAxes.length()>0){const auto first=doc.base.world(result.clip.frames.front()),last=doc.base.world(result.clip.frames.back());const auto net=multiply(last[4].t-first[4].t,options.makeInPlaceAxes);
        for(std::size_t frame=0;frame<=intervals;++frame)result.clip.frames[frame][0].t=result.clip.frames[frame][0].t-net*(float(frame)/float(intervals));result.warnings.push_back("Selected linear net body-motion axes were removed from Root. Nonlinear motion remains; FreeClimb slot movement is still controlled by the calibrated configuration.");
    }
    result.clip.rotations.resize(result.clip.frames.size());for(std::size_t frame=0;frame<result.clip.frames.size();++frame){auto& row=result.clip.rotations[frame];row.resize(99);for(int bone=0;bone<99;++bone)row[bone]=result.clip.frames[frame][bone].q;}
    result.clip.annotations.clear();for(const auto& event:original.annotations)if(event.text==converterSourceMotionMarker||(event.time>=begin-.00005f&&event.time<=end+.00005f)){auto edited=event;edited.time=event.text==converterSourceMotionMarker?0:(std::clamp(event.time,begin,end)-begin)/options.speed;result.clip.annotations.push_back(std::move(edited));}
    if(result.clip.annotations.size()!=original.annotations.size())result.warnings.push_back(std::to_string(original.annotations.size()-result.clip.annotations.size())+" annotations outside the trimmed interval were removed.");
    if(!original.referenceFrame.samples.empty()){
        auto& reference=result.clip.referenceFrame;reference.up=facing.rotate(original.referenceFrame.up);reference.forward=facing.rotate(original.referenceFrame.forward);
        if(begin==0&&end==original.duration){reference.duration=original.referenceFrame.duration/options.speed;for(auto& row:reference.samples){const auto position=facing.rotate({row[0],row[1],row[2]});row[0]=position.x;row[1]=position.y;row[2]=position.z;}}
        else {reference.duration=duration;reference.samples.resize(intervals+1);const auto origin=referenceSample(original.referenceFrame,firstPhase);
            for(std::size_t frame=0;frame<=intervals;++frame){auto row=referenceSample(original.referenceFrame,firstPhase+(lastPhase-firstPhase)*float(frame)/float(intervals));const auto position=facing.rotate({row[0]-origin[0],row[1]-origin[1],row[2]-origin[2]});row={position.x,position.y,position.z,row[3]-origin[3]};reference.samples[frame]=row;}
        }
    }
    if(options.autoCalibration&&!doc.preserveTemplateTiming){auto calibration=calibrateConverterClip(doc.base,doc.slot,result.clip,doc.templateConfig);result.config=std::move(calibration.config);result.confidence=calibration.confidence;result.phaseMap=std::move(calibration.phaseMap);addWarnings(result.warnings,calibration.warnings);}
    else {result.config=doc.templateConfig;result.phaseMap={{0,0},{1,1}};result.warnings.push_back("Automatic timing calibration is disabled. Target-slot phase timing was retained and requires verification.");}
    require(result.config.is_object()&&result.config.at("slot")==doc.slot&&result.config.at("file")==doc.templateConfig.at("file"),"Calibration changed the slot or file identity");
    if(doc.authored){
        result.config.erase("references");
        std::erase_if(result.warnings,[](const std::string& text){return text.starts_with("This animation differs substantially")||text.starts_with("The suggested timing was too compressed")||text.starts_with("Automatic timing calibration is disabled");});
        result.config["version"]=2;result.config["authoredPlayback"]={{"version",1}};
        if(!doc.preserveTemplateTiming&&(result.confidence<.9f||!options.autoCalibration)){result.config["contacts"]=Json::array({{0,0,0,0},{0,0,0,0}});result.warnings.push_back("Reference contacts do not reliably match this action. Source poses are preserved without inherited grip locks; Contacts can add bounded hand/foot support.");}
        const auto origin=result.clip.frames.front()[0].t;std::vector<Vec> points;points.reserve(result.clip.frames.size());for(const auto& pose:result.clip.frames)points.push_back(pose[0].t-origin);
        const auto endPoint=points.back();const float net=endPoint.length();
        if(movingCycle(doc.slot)&&net>=2){require(net<=500,"Source cycle displacement exceeds the supported stride");result.config["stride"]=net;}
        if(std::any_of(points.begin(),points.end(),[](Vec value){return value.length()>.0001f;})){
            std::vector<std::size_t> keys{0,points.size()-1};while(keys.size()<65){float worst=.15f;std::size_t selected=0,segment=0;for(std::size_t i=1;i<keys.size();++i)for(auto at=keys[i-1]+1;at<keys[i];++at){const float phase=float(at-keys[i-1])/float(keys[i]-keys[i-1]);const float error=(points[at]-(points[keys[i-1]]+(points[keys[i]]-points[keys[i-1]])*phase)).length();if(error>worst){worst=error;selected=at;segment=i;}}if(!selected)break;keys.insert(keys.begin()+std::ptrdiff_t(segment),selected);}
            for(std::size_t i=1;i<keys.size();++i)for(auto at=keys[i-1]+1;at<keys[i];++at){const float phase=float(at-keys[i-1])/float(keys[i]-keys[i-1]);require((points[at]-(points[keys[i-1]]+(points[keys[i]]-points[keys[i-1]])*phase)).length()<=.25f,"Source trajectory is too detailed for the bounded playback path. Use a shorter or smoother clip.");}
            Json trajectory=Json::array();for(const auto at:keys){const auto v=points[at];trajectory.push_back({float(at)/float(points.size()-1),v.x,v.y,v.z});}
            result.config["authoredPlayback"]["basis"]="root";result.config["authoredPlayback"]["trajectory"]=std::move(trajectory);
            if(net>40&&hop(doc.slot))result.config["travel"][0]=doc.slot=="contextHopLeft"?-net:net;
            if(net>=2&&doc.slot=="contextMantle"){result.config["height"]=endPoint.z;result.config["travel"]={endPoint.x,endPoint.y,endPoint.z};}
            result.warnings.push_back("Source Root movement was recorded. The action controller adapts compatible travel to its checked route; unconsumed local movement remains in the pose.");
        }else {
            result.warnings.push_back("This is an in-place source. Its complete animation timing is retained; movement uses the selected action's collision-checked route and reference distances.");
        }
    }
    if(options.geometry){const auto& geometry=*options.geometry;
        require(std::isfinite(geometry.stride)&&geometry.stride>=0&&geometry.stride<=500&&std::isfinite(geometry.height)&&std::abs(geometry.height)<=500&&finite(geometry.travel)&&std::abs(geometry.travel.x)<=500&&std::abs(geometry.travel.y)<=500&&std::abs(geometry.travel.z)<=500,"Movement geometry exceeds the supported metadata bounds");
        if(doc.authored){require(converterEditableStride(doc,result),"Only an imported movement cycle without net Root travel has an editable stride");const auto travel=result.config.at("travel");require(geometry.height==result.config.at("height").get<float>()&&geometry.travel.x==travel[0].get<float>()&&geometry.travel.y==travel[1].get<float>()&&geometry.travel.z==travel[2].get<float>()&&geometry.path.empty(),"Imported in-place cycles allow only stride edits, not unrelated route geometry");}
        require(!hop(doc.slot)||(doc.slot=="contextHopLeft"?geometry.travel.x<-40:geometry.travel.x>40),"Side-action travel must exceed 40 units toward its selected side");
        require(geometry.path.empty()||hop(doc.slot),"Only side-action slots provide an editable movement path");result.config["stride"]=geometry.stride;result.config["height"]=geometry.height;result.config["travel"]={geometry.travel.x,geometry.travel.y,geometry.travel.z};if(!geometry.path.empty())result.config["path"]=geometry.path;
        result.warnings.push_back("Movement geometry was explicitly edited. Preview shows authored bone motion; verify collision-controlled reach and support in-game.");
    }
    require(!doc.authored||options.windows.empty(),"Imported source playback uses per-limb Contacts, not the base action timing windows");
    if(!doc.authored)applyWindows(result.config,doc.slot,options.windows);
    Json weights=Json::array();for(std::size_t frame=0;frame<=intervals;++frame){const float phase=float(frame)/float(intervals);auto row=contacts(result.config,phase);for(const auto& edit:options.contacts){const float at=envelope(phase,edit.beginPhase,edit.endPhase,edit.fadePhase);row[edit.contact]=std::lerp(row[edit.contact],edit.weight,at);}weights.push_back(row);}result.config["contacts"]=std::move(weights);
    if(!doc.authored&&!doc.contextGroup.empty()){
        require(options.windows.empty(),"Complete side leaps use the Contacts timeline");result.contextGroup=doc.contextGroup;auto& sequence=result.contextGroup.at("sequences").at(0);
        auto& clips=result.contextGroup.at("clips");auto& main=*std::find_if(clips.begin(),clips.end(),[&](const auto& clip){return clip.at("slot")==doc.slot;});const auto located=main;main=result.config;for(const auto key:{"member","frameRange","rootShift"})main[key]=located.at(key);
        std::array<Json*,3> parts{&sequence.at("prepare"),&main,&sequence.at("catch")};
        for(auto* part:parts){const auto originalRange=animationFrameRange(*part,original.frames.size());const float a=original.duration*float(originalRange[0])/float(original.frames.size()-1),b=original.duration*float(originalRange[1])/float(original.frames.size()-1);
            const auto first=std::size_t(std::llround(std::clamp((a-begin)/(end-begin),0.f,1.f)*float(intervals))),last=std::size_t(std::llround(std::clamp((b-begin)/(end-begin),0.f,1.f)*float(intervals)));require(first<last,"Trim must retain preparation, leap and recovery; import a new full source to replace their structure");
            Vec shift{};if(part->contains("rootShift")){const auto& value=part->at("rootShift");shift=facing.rotate({value[0].get<float>(),value[1].get<float>(),value[2].get<float>()});}(*part)["frameRange"]={first,last};(*part)["rootShift"]={shift.x,shift.y,shift.z};
            auto& stageWeights=(*part)["contacts"];stageWeights=Json::array();for(auto frame=first;frame<=last;++frame)stageWeights.push_back(result.config.at("contacts").at(frame));
        }
        *std::find_if(clips.begin(),clips.end(),[](const auto& clip){return clip.at("slot")=="contextHang";})=sequence.at("prepare");
    }return result;
}
Pose sampleConverterEditor(const ConverterEditedAnimation& edited,float seconds){return sample(edited.clip,phaseAt(edited.clip,seconds));}
std::array<float,4> sampleConverterContacts(const ConverterEditedAnimation& edited,float seconds){
    const float phase=phaseAt(edited.clip,seconds);auto result=contacts(edited.config,phase);const auto slot=edited.config.at("slot").get<std::string>();
    auto weight=[&](const Json& value){const auto bounds=window(value);return threepeatEase((phase-bounds[0])/(bounds[1]-bounds[0]));};
    if(edited.config.contains("authoredPlayback")||!edited.contextGroup.empty())return result;
    if(hop(slot))for(int hand=0;hand<2;++hand)result[hand]=std::max(1-weight(edited.config.at("sourceHands")[hand]),weight(edited.config.at("targetHands")[hand]));
    else if(slot=="contextMantle")for(int hand=0;hand<2;++hand)result[hand]=(1-weight(edited.config.at("unplant"))+weight(edited.config.at("replant")))*(1-weight(edited.config.at("releaseHands")[hand]));return result;
}
nlohmann::json exportConverterEditor(const ConverterEditorDocument& doc,const ConverterEditedAnimation& edited,const std::filesystem::path& output,bool overwrite){
    require(!doc.snapshots.empty(),"Editor export requires a loaded document");ConverterPreparedClip prepared{edited.clip,edited.config,edited.warnings,doc.snapshots,doc.direction,edited.contextGroup};return convertHkx({doc.input,doc.pack,output,{},doc.slot,overwrite},prepared);
}
nlohmann::json exportConverterEditorGroup(const std::vector<ConverterEditorExport>& items,const std::filesystem::path& output,bool overwrite){
    require(!items.empty()&&items.front().document&&items.front().animation,"No edited action group");const auto& first=*items.front().document;
    std::vector<ConverterPreparedClip> prepared;prepared.reserve(items.size());
    for(const auto& item:items){require(item.document&&item.animation&&!item.document->snapshots.empty()&&item.document->pack==first.pack,"Action group must use one loaded base pack");const auto& doc=*item.document;const auto& edit=*item.animation;require(edit.config.at("slot")==doc.slot,"Action group stage mismatch");prepared.push_back({edit.clip,edit.config,edit.warnings,doc.snapshots,item.direction.empty()?doc.direction:item.direction,edit.contextGroup});}
    return convertHkxGroup({first.input,first.pack,output,{},first.slot,overwrite},prepared);
}
}
