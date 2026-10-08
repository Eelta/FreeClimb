#include "../tools/converter/ConverterEditor.h"
#include "../tools/converter/ConverterSourceMotion.h"
#include "../tools/converter/ConverterTimeline.h"
#include "../src/AnimationPack.h"
#include "../src/AnimationConfig.h"
#include "../src/MotionSlots.h"
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>

namespace {
using Json=nlohmann::json;
using Bytes=std::vector<std::uint8_t>;
std::size_t checks{};
void require(bool value,const std::string& message){++checks;if(!value)throw std::runtime_error(message);}
Bytes read(const std::filesystem::path& path){std::ifstream input(path,std::ios::binary);require(bool(input),"Read fixture");return {std::istreambuf_iterator<char>(input),{}};}
void write(const std::filesystem::path& path,const Bytes& value){std::filesystem::create_directories(path.parent_path());std::ofstream output(path,std::ios::binary);require(bool(output.write(reinterpret_cast<const char*>(value.data()),std::streamsize(value.size()))),"Write fixture");}
template<class T>T word(const Bytes& bytes,std::size_t at){require(at<=bytes.size()&&sizeof(T)<=bytes.size()-at,"ZIP field bounds");T result;std::memcpy(&result,bytes.data()+at,sizeof result);return result;}
std::map<std::string,Bytes> unpack(const std::filesystem::path& path){
    const auto bytes=read(path);require(bytes.size()>=22,"ZIP length");const auto end=bytes.size()-22;require(word<std::uint32_t>(bytes,end)==0x06054b50u&&word<std::uint16_t>(bytes,end+10)==2,"Two-file override");
    std::size_t at=word<std::uint32_t>(bytes,end+16);std::map<std::string,Bytes> entries;
    for(int i=0;i<2;++i){require(word<std::uint32_t>(bytes,at)==0x02014b50u&&word<std::uint16_t>(bytes,at+10)==0,"Stored ZIP entry");const auto length=word<std::uint16_t>(bytes,at+28),extra=word<std::uint16_t>(bytes,at+30),comment=word<std::uint16_t>(bytes,at+32);const auto size=word<std::uint32_t>(bytes,at+24),offset=word<std::uint32_t>(bytes,at+42);require(at+46+length<=bytes.size(),"ZIP name bounds");const std::string name(reinterpret_cast<const char*>(bytes.data()+at+46),length);const auto data=offset+30+word<std::uint16_t>(bytes,offset+26)+word<std::uint16_t>(bytes,offset+28);require(data+size<=bytes.size(),"ZIP data bounds");require(entries.emplace(name,Bytes(bytes.begin()+data,bytes.begin()+data+size)).second,"Unique ZIP entry");at+=46+length+extra+comment;}
    require(at==end,"ZIP directory end");return entries;
}
void poseEqual(const fc::Pose& a,const fc::Pose& b,const std::string& context){
    require(a.size()==99&&b.size()==99,context+": bone count");for(std::size_t bone=0;bone<99;++bone){require((a[bone].t-b[bone].t).length()<.0001f,context+": translation");require(fc::angleBetween(a[bone].q,b[bone].q)<.0001f,context+": rotation");require((a[bone].s-b[bone].s).length()<.0001f,context+": scale");}
}
fc::Pose sample(const fc::ConverterInputClip& clip,float phase){
    const float at=std::clamp(phase,0.f,1.f)*float(clip.frames.size()-1);const auto first=std::min(std::size_t(at),clip.frames.size()-2);auto pose=clip.frames[first];
    for(std::size_t bone=0;bone<pose.size();++bone)pose[bone]=fc::blend(pose[bone],clip.frames[first+1][bone],at-float(first));return pose;
}
void timelineEqual(const fc::ConverterInputClip& timeline,const Json& config,const fc::ConverterInputClip& source,const std::string& slot){
    const auto range=fc::animationFrameRange(config,timeline.frames.size());
    require(config.at("frameRange").size()==2&&range[0]<range[1]&&range[1]<timeline.frames.size(),"Export locates a nonempty complete stage: "+slot);
    const auto& value=config.at("rootShift");require(value.is_array()&&value.size()==3,"Timeline Root placement has three axes: "+slot);
    const fc::Vec shift{value[0].get<float>(),value[1].get<float>(),value[2].get<float>()};require(shift.finite(),"Timeline Root placement is finite: "+slot);
    const auto decoded=fc::sliceConverterTimeline(timeline,config);
    if(slot=="contextHang"){
        const float interval=timeline.duration/float(timeline.frames.size()-1);require(std::abs(decoded.duration-source.duration)<=std::min({interval*.5f,source.duration*.01f,1.f/120})+.000001f,"Private helper timing stays within the bounded timeline grid");
        for(std::size_t frame=0;frame<decoded.frames.size();++frame){const auto expected=sample(source,float(frame)/float(decoded.frames.size()-1));for(unsigned bone=0;bone<99;++bone){require((decoded.frames[frame][bone].t-expected[bone].t).length()<=.02f,"Private helper keeps original positional fidelity bound");require(fc::angleBetween(decoded.frames[frame][bone].q,expected[bone].q)<=.00174533f,"Private helper keeps original angular fidelity bound");require((decoded.frames[frame][bone].s-expected[bone].s).length()<=.00001f,"Private helper never changes bone scale");}}return;
    }
    const float clockError=4*std::numeric_limits<float>::epsilon()*std::max(1.f,source.duration);
    require(std::abs(decoded.duration-source.duration)<=clockError,"Complete source timing survives timeline location: "+slot);
    require(decoded.frames.size()>=source.frames.size(),"Timeline does not discard original source keys: "+slot);
    for(std::size_t frame=0;frame<decoded.frames.size();++frame){
        auto located=timeline.frames[range[0]+frame];located[0].t=located[0].t-shift;
        poseEqual(decoded.frames[frame],located,"Timeline location removes Root placement once "+slot);
        poseEqual(decoded.frames[frame],sample(source,float(frame)/float(decoded.frames.size()-1)),"Timeline preserves every sampled source pose "+slot);
    }
    for(std::size_t frame=0;frame<source.frames.size();++frame){
        const double at=double(frame)*double(decoded.frames.size()-1)/double(source.frames.size()-1);const auto index=std::size_t(std::llround(at));
        require(std::abs(at-double(index))<.0001,"Timeline retains the exact source key grid: "+slot);
        poseEqual(decoded.frames[index],source.frames[frame],"Timeline retains the original source key "+slot);
    }
}
bool cycle(std::string_view slot){constexpr std::array<std::string_view,9> names{"up","down","left","right","runUp","runLeft","runRight","runDiagonalLeft","runDiagonalRight"};return std::find(names.begin(),names.end(),slot)!=names.end();}
fc::Vec endpoint(std::string_view slot){
    if(slot=="hang"||slot=="contextHang")return {};
    if(slot=="drop"||slot=="backFlipOut")return {0,-35,-50};
    if(slot=="hopLeft"||slot=="contextHopLeft")return {-60,0,15};
    if(slot=="hopRight")return {60,0,15};
    if(slot=="runUp")return {0,80,0};
    if(slot=="contextMantle")return {0,45,100};
    return {0,10,65};
}
int run(const std::filesystem::path& manifest,const std::filesystem::path& output){
    const auto folder=output/("run-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));std::filesystem::create_directories(folder);
    const auto metadata=Json::parse(read(manifest));std::map<std::filesystem::path,Bytes> sourceFiles;sourceFiles.emplace(manifest,read(manifest));const auto skeleton=manifest.parent_path()/metadata.at("skeleton").get<std::string>();sourceFiles.emplace(skeleton,read(skeleton));
    for(const auto& entry:metadata.at("motions")){const auto config=manifest.parent_path()/entry.at("config").get<std::string>();sourceFiles.emplace(config,read(config));const auto clip=manifest.parent_path()/fc::selectAnimationClipConfig(Json::parse(sourceFiles.at(config)),entry.at("slot").get<std::string>()).at("file").get<std::string>();sourceFiles.emplace(clip,read(clip));}
    Json report=Json::array();
    const auto inspectExport=[&](const fc::ConverterEditorDocument& doc,const fc::ConverterEditedAnimation& animation,const std::filesystem::path& destination){
        const auto result=fc::exportConverterEditor(doc,animation,destination);require(result.at("loaded")==fc::activeMotionCount,"Full pack accepted: "+doc.slot);const auto files=unpack(destination);const std::string prefix="meshes/actors/character/animations/FreeClimb/";
        const auto selected=std::find_if(metadata.at("motions").begin(),metadata.at("motions").end(),[&](const auto& entry){return entry.at("slot")==doc.slot;});require(selected!=metadata.at("motions").end(),"Selected slot remains present");const auto configPath=prefix+selected->at("config").template get<std::string>();const auto config=fc::selectAnimationClipConfig(Json::parse(files.at(configPath)),doc.slot);
        require(config.at("version")==2&&config.at("authoredPlayback").at("version")==1,"Explicit authored format: "+doc.slot);auto expectedConfig=animation.config,sourceConfig=config;expectedConfig["file"]=config.at("file");if(config.contains("member"))expectedConfig["member"]=config.at("member");else expectedConfig.erase("member");
        for(const auto key:{"frameRange","rootShift"}){sourceConfig.erase(key);expectedConfig.erase(key);}require(sourceConfig==expectedConfig,"Export source configuration matches preview: "+doc.slot);
        fc::ConverterInputClip decoded;std::string error;require(fc::decodeConverterInput(files.at(prefix+config.at("file").get<std::string>()),decoded,error,config.value("member",std::string{})),"Decode exported action: "+error);
        if(config.contains("frameRange")){timelineEqual(decoded,config,animation.clip,doc.slot);decoded=fc::sliceConverterTimeline(decoded,config);}
        else{require(!config.contains("rootShift"),"A standalone action cannot have timeline Root placement");require(decoded.duration==animation.clip.duration&&decoded.frames.size()==animation.clip.frames.size(),"Complete edited source timing: "+doc.slot);for(std::size_t frame=0;frame<decoded.frames.size();++frame)poseEqual(decoded.frames[frame],animation.clip.frames[frame],"Export equals preview "+doc.slot);}
        std::vector<std::string> warnings;const auto secondBytes=fc::writeConverterHkx(decoded,doc.base,warnings);fc::ConverterInputClip second;require(fc::decodeConverterInput(secondBytes,second,error),"Reimport exported action");require(second.annotations.size()==decoded.annotations.size(),"Reimport retains annotation count");for(std::size_t frame=0;frame<decoded.frames.size();++frame)poseEqual(second.frames[frame],decoded.frames[frame],"Reimport does not duplicate Root");return files;
    };
    for(const auto motion:fc::activeMotions){
        const std::string slot(fc::motionSlotNames[int(motion)-1]);const auto base=fc::loadConverterBaseEditor(manifest,slot);require(!base.authored,"Bundled base stays legacy: "+slot);const auto baseAnimation=fc::applyConverterEdits(base,{});require(baseAnimation.config.at("version")==1&&!baseAnimation.config.contains("authoredPlayback"),"Base configuration unchanged: "+slot);
        const auto input=folder/(slot+"-external.hkx");std::vector<std::string> warnings;write(input,fc::writeConverterHkx(base.clip,base.base,warnings));const auto original=read(input);const auto doc=fc::loadConverterEditor(input,manifest,slot);require(doc.authored,"External input opts into authored playback: "+slot);
        require(doc.clip.duration==base.clip.duration&&doc.clip.frames.size()==base.clip.frames.size(),"External fixture contains the selected complete stage rather than its multi-stage member: "+slot);
        for(std::size_t frame=0;frame<base.clip.frames.size();++frame)poseEqual(doc.clip.frames[frame],base.clip.frames[frame],"External fixture retains the located source stage "+slot);
        const auto edited=fc::applyConverterEdits(doc,{});require(edited.clip.duration==doc.clip.duration&&edited.clip.frames.size()==doc.clip.frames.size(),"No shortening of source action: "+slot);
        const auto origin=doc.clip.frames.front()[0].t;for(std::size_t frame=0;frame<edited.clip.frames.size();++frame){auto expected=doc.clip.frames[frame];expected[0].t=expected[0].t-origin;poseEqual(edited.clip.frames[frame],expected,"Source local pose preserved "+slot);}
        if(slot=="contextMantle"){
            bool rejected=false;try{fc::exportConverterEditor(doc,edited,folder/(slot+"-ambiguous-com.zip"));}catch(const std::exception& error){rejected=std::string(error.what()).find("COM")!=std::string::npos;}
            require(rejected,"A large non-Root carrier is rejected without inventing a route");require(read(input)==original,"Rejected source remains unchanged");report.push_back({{"slot",slot},{"authored",true},{"ambiguousComRejected",true}});continue;
        }
        const auto exported=inspectExport(doc,edited,folder/(slot+"-gui.zip"));const auto cli=folder/(slot+"-cli.zip");fc::convertHkx({input,manifest,cli,{},slot,false});require(unpack(cli)==exported,"CLI and GUI use identical adaptation: "+slot);require(read(input)==original,"Original HKX unchanged: "+slot);
        report.push_back({{"slot",slot},{"sourceFrames",doc.clip.frames.size()},{"sourceSeconds",doc.clip.duration},{"authored",true},{"sourceTrajectory",edited.config["authoredPlayback"].contains("trajectory")},{"guiCliEqual",true}});
    }
    constexpr std::array<std::string_view,12> representatives{"hang","up","runUp","contextHang","jumpCatch","hopLeft","kickUp","hopRight","contextHopLeft","contextMantle","backFlipOut","drop"};
    for(const auto name:representatives)for(const bool moved:{false,true}){
        const std::string slot(name);const auto base=fc::loadConverterBaseEditor(manifest,slot);auto source=base.clip;source.referenceFrame={};source.annotations.clear();const fc::Vec start{4,-3,2},finish=endpoint(slot);
        for(std::size_t frame=0;frame<source.frames.size();++frame){const float phase=float(frame)/float(source.frames.size()-1);source.frames[frame][0].t=start;if(slot=="contextMantle")source.frames[frame][4].t=source.frames.front()[4].t;
            if(moved)source.frames[frame][0].t=source.frames[frame][0].t+finish*phase+fc::Vec{(slot=="hang"||slot=="contextHang")?2.f*std::sin(phase*6.283185307f):0,0,(slot=="hang"||slot=="contextHang")?0:6.f*std::sin(phase*3.141592654f)};}
        std::vector<std::string> warnings;const auto input=folder/(slot+(moved?"-root.hkx":"-in-place.hkx"));write(input,fc::writeConverterHkx(source,base.base,warnings));const auto doc=fc::loadConverterEditor(input,manifest,slot);const auto edited=fc::applyConverterEdits(doc,{});
        require(edited.config["authoredPlayback"].contains("trajectory")==moved,"Real Root and in-place sources are distinguished: "+slot);require(edited.clip.frames.front()[0].t.length()<.0001f,"Authored Root starts at zero: "+slot);
        if(cycle(slot)){const float expected=moved&&finish.length()>=2?finish.length():base.templateConfig["stride"].get<float>();require(std::abs(edited.config["stride"].get<float>()-expected)<.001f,"Stride follows net source travel or explicit fallback: "+slot);}
        const auto suffix=moved?"-root.zip":"-in-place.zip";inspectExport(doc,edited,folder/(slot+suffix));
        if(cycle(slot)){
            require(fc::converterEditableStride(doc,edited)==!moved,"Stride editing is limited to in-place movement cycles");
            const auto travel=edited.config.at("travel");fc::ConverterEditOptions custom;custom.geometry=fc::ConverterGeometryEdit{137,edited.config.at("height").get<float>(),{travel[0].get<float>(),travel[1].get<float>(),travel[2].get<float>()}};
            if(!moved){const auto adjusted=fc::applyConverterEdits(doc,custom);require(adjusted.config.at("stride")==137,"In-place stride applies to runtime metadata");const auto zip=folder/(slot+"-stride.zip");const auto files=inspectExport(doc,adjusted,zip);const auto replacement=folder/(slot+"-stride-pack");for(const auto& [path,bytes]:sourceFiles){const auto relative=path.lexically_relative(manifest.parent_path());write(replacement/relative,bytes);}for(const auto& [name,bytes]:files)write(replacement/name.substr(std::string("meshes/actors/character/animations/FreeClimb/").size()),bytes);
                const auto reloaded=fc::loadConverterBaseEditor(replacement/"pack.json",slot);const auto again=fc::applyConverterEdits(reloaded,{});require(again.config.at("stride")==137&&fc::converterEditableStride(reloaded,again),"Exported authored stride survives loading the replacement pack");
                custom.geometry->height+=1;bool rejected=false;try{fc::applyConverterEdits(doc,custom);}catch(const std::exception&){rejected=true;}require(rejected,"In-place stride does not unlock unrelated geometry");
            }else {bool rejected=false;try{fc::applyConverterEdits(doc,custom);}catch(const std::exception&){rejected=true;}require(rejected,"Real source travel keeps automatic stride");}
        }
        if(moved&&slot=="contextHopLeft"){auto shortTravel=doc;const auto origin=shortTravel.clip.frames.front()[0].t;for(auto& frame:shortTravel.clip.frames)frame[0].t=origin+(frame[0].t-origin)*.1f;const auto shortAction=fc::applyConverterEdits(shortTravel,{});require(shortAction.config.at("travel")==base.templateConfig.at("travel"),"Small source side leap keeps usable target-search distances");require(shortAction.config["authoredPlayback"].contains("trajectory"),"Small Root path remains represented");inspectExport(shortTravel,shortAction,folder/"short-side-travel.zip");}
        if(moved){fc::ConverterEditOptions trim;trim.trimIn=doc.clip.duration*.2f;trim.trimOut=doc.clip.duration*.85f;trim.speed=1.5f;trim.rootOffset={1,2,3};const auto cropped=fc::applyConverterEdits(doc,trim);require((cropped.clip.frames.front()[0].t-trim.rootOffset).length()<.0001f,"Trim reanchors before explicit offset: "+slot);require(std::abs(cropped.clip.duration-(trim.trimOut-trim.trimIn)/trim.speed)<.0001f,"Trim and speed retain full selected interval: "+slot);inspectExport(doc,cropped,folder/(slot+"-trimmed.zip"));}
    }
    for(const auto& [path,bytes]:sourceFiles)require(read(path)==bytes,"Complete source pack remains byte-identical");
    const Json result{{"ok",true},{"slots",report},{"rootAndInPlaceClasses",representatives.size()},{"checks",checks},{"sourcePackUnchanged",true}};std::ofstream(folder/"report.json")<<result.dump(2);std::cout<<"30 bundled-source imports passed; COM-carrier mantle rejected; all slot classes covered by Root/in-place fixtures; checks="<<checks<<'\n';return 0;
}
}
int main(int argc,char** argv)try{require(argc==3,"pack.json and output directory required");return run(argv[1],argv[2]);}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
