#include "ConverterEditor.h"
#include "ConverterWallRunGroup.h"
#include "ConverterTimeline.h"
#include "../../src/AnimationPack.h"
#include <fstream>
#include <iostream>
#include <map>
#include <set>

namespace {
using Json=nlohmann::json;
using Bytes=std::vector<std::uint8_t>;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
void put(const std::filesystem::path& path,const Bytes& bytes){std::filesystem::create_directories(path.parent_path());std::ofstream file(path,std::ios::binary);require(bool(file.write(reinterpret_cast<const char*>(bytes.data()),std::streamsize(bytes.size()))),"Could not write independent action");}
void put(const std::filesystem::path& path,const Json& value){const auto text=value.dump(2)+"\n";put(path,Bytes(text.begin(),text.end()));}
void files(Json& value,const std::string& name){if(value.is_object()){if(value.value("format",std::string{})=="FreeClimbClip")value["file"]=name;for(auto& [key,item]:value.items())if(item.is_structured())files(item,name);}else if(value.is_array())for(auto& item:value)files(item,name);}
fc::ConverterPreparedClip raw(const std::filesystem::path& pack,const std::string& slot,const std::string& direction={}){
    const auto document=fc::loadConverterBaseEditor(pack,slot,direction);const auto saved=std::find_if(document.snapshots.begin(),document.snapshots.end(),[&](const auto& item){return item.path==document.input;});require(saved!=document.snapshots.end(),"Missing source snapshot");fc::ConverterInputClip clip;std::string error;require(fc::decodeConverterInput(saved->bytes,clip,error,document.templateConfig.value("member",std::string{})),error.c_str());clip=fc::sliceConverterTimeline(clip,document.templateConfig);return {clip,document.templateConfig,{},document.snapshots,direction};
}
fc::ConverterPreparedClip window(fc::ConverterPreparedClip source,float seconds){
    const auto original=source.clip;const unsigned intervals=unsigned(std::lround(seconds*300));source.clip.duration=seconds;source.clip.frames.resize(intervals+1);source.clip.rotations.clear();source.clip.annotations.clear();source.clip.referenceFrame={};
    for(unsigned frame=0;frame<=intervals;++frame){const float at=seconds*float(frame)/float(intervals)/original.duration*float(original.frames.size()-1);const auto first=std::min(std::size_t(at),original.frames.size()-2);auto& pose=source.clip.frames[frame];pose.resize(99);for(unsigned bone=0;bone<99;++bone)pose[bone]=fc::blend(original.frames[first][bone],original.frames[first+1][bone],at-float(first));}
    source.config["contacts"]={{1,1,0,0},{1,1,0,0}};source.config.erase("member");source.config.erase("frameRange");source.config.erase("rootShift");return source;
}
int independent(const std::filesystem::path& pack,const std::filesystem::path& output){
    require(!std::filesystem::exists(output),"Independent migration destination must not exist");fc::Library library;const auto loaded=fc::loadAnimationPack(library,pack);require(loaded.committed,loaded.error.c_str());
    std::ifstream stream(pack);Json manifest=Json::parse(stream);const auto root=pack.parent_path();std::set<std::filesystem::path> originals{pack,root/manifest.at("skeleton").get<std::string>()};
    for(const auto& entry:manifest.at("motions")){const auto file=root/entry.at("config").get<std::string>();originals.insert(file);std::ifstream config(file);const auto metadata=Json::parse(config);originals.insert(root/fc::selectAnimationClipConfig(metadata,entry.at("slot").get<std::string>()).at("file").get<std::string>());}
    for(const auto& file:originals){const auto relative=file.lexically_relative(root);if(relative=="contextHop.hkx"||relative=="sideBrace.hkx"||relative=="configs/contextHop.json"||relative=="configs/sideBrace.json")continue;std::ifstream input(file,std::ios::binary);put(output/relative,Bytes(std::istreambuf_iterator<char>(input),{}));}
    std::map<std::string,fc::ConverterPreparedClip> sources;auto get=[&](const std::string& name)->fc::ConverterPreparedClip{if(!sources.contains(name))sources[name]=raw(pack,name);return sources.at(name);};
    auto own=[&](fc::ConverterPreparedClip& action,const std::string& role,const std::string& source,fc::PlaybackReference reference){auto content=get(source);auto config=content.config;config.erase("references");config.erase("frameRange");config.erase("rootShift");config["member"]=action.config.at("slot").get<std::string>()+"_"+role;action.config["references"][role]=config;const auto owner=std::find(fc::motionSlotNames.begin(),fc::motionSlotNames.end(),action.config.at("slot").get<std::string>());const auto fallback=std::find(fc::motionSlotNames.begin(),fc::motionSlotNames.end(),source);library.references[{fc::Motion(owner-fc::motionSlotNames.begin()+1),reference}]=library.clip(fc::Motion(fallback-fc::motionSlotNames.begin()+1));};
    auto save=[&](const std::string& name,fc::ConverterWallRunPack content){files(content.config,name+".hkx");put(output/(name+".hkx"),content.hkx);put(output/"configs"/(name+".json"),content.config);};
    for(const auto direction:{"runUp","runLeft","runRight","runDiagonalLeft","runDiagonalRight"}){const auto roles=fc::converter::wallRunStages(direction);std::vector<fc::ConverterPreparedClip> parts;for(unsigned i=0;i<3;++i)parts.push_back(raw(pack,std::string(roles[i]),direction));if(std::string_view(direction)=="runUp")own(parts[0],"launchApproach","runUp",fc::PlaybackReference::launchApproach);else parts.push_back(get("sideBrace"));save(direction,fc::composeConverterWallRunDirection(library,direction,parts));}
    for(const auto direction:{"contextHopLeft","contextHopRight"}){std::vector<fc::ConverterPreparedClip> parts{window(get("contextHang"),.26f),get(direction),window(get("contextHang"),.18f)};save(direction,fc::composeConverterContextHopDirection(library,direction,parts));}
    for(const auto direction:{"kickUp","kickLeft","kickRight"}){auto action=get(direction);const std::string owner=direction;own(action,"kickLanding",owner=="kickLeft"?"hopLeft":owner=="kickRight"?"hopRight":"hopUp",fc::PlaybackReference::kickLanding);own(action,"kickRunLanding","runUp",fc::PlaybackReference::kickRunLanding);own(action,"kickRunBrace","sideBrace",fc::PlaybackReference::kickRunBrace);if(owner!="kickUp")own(action,"kickTakeoff","kickUp",fc::PlaybackReference::kickTakeoff);auto metadata=action.config;metadata["member"]=owner;metadata.erase("frameRange");metadata.erase("rootShift");std::vector<std::pair<std::string,Bytes>> members{{owner,fc::writeCanonicalConverterHkx(action.clip,library)}};for(auto& [role,reference]:metadata["references"].items()){auto source=get(reference.at("slot").get<std::string>());members.emplace_back(reference.at("member").get<std::string>(),fc::writeCanonicalConverterHkx(source.clip,library));}files(metadata,owner+".hkx");put(output/(owner+".hkx"),fc::bundleConverterHkx(members));put(output/"configs"/(owner+".json"),metadata);}
    for(auto& entry:manifest["motions"]){const auto slot=entry.at("slot").get<std::string>();if(slot=="contextHang")entry["config"]="configs/contextHopLeft.json";else if(slot=="contextHopLeft"||slot=="contextHopRight")entry["config"]="configs/"+slot+".json";else if(slot=="sideBrace")entry["config"]="configs/runLeft.json";}
    put(output/"pack.json",manifest);fc::Library candidate;const auto validation=fc::loadAnimationPack(candidate,output/"pack.json");if(!validation.committed){std::string error=validation.error;for(const auto& slot:validation.slots)if(slot.status==fc::OverrideStatus::rejected)error+="; "+slot.file+": "+slot.reason;throw std::runtime_error(error);}std::cout<<Json({{"ok",true},{"loaded",validation.loaded},{"output",output.string()}}).dump()<<'\n';return 0;
}
}

int main(int argc,char** argv){
    try{
        if(argc==4&&std::string_view(argv[3])=="--independent-actions")return independent(std::filesystem::absolute(argv[1]),std::filesystem::absolute(argv[2]));
        if(argc!=3&&argc!=4)throw std::runtime_error("Usage: FreeClimbComposeWallRun pack.json output-directory [--split-directions]");
        const bool split=argc==4&&std::string_view(argv[3])=="--split-directions";
        if(argc==4&&!split)throw std::runtime_error("Unknown migration option");
        const auto pack=std::filesystem::absolute(argv[1]),output=std::filesystem::absolute(argv[2]);
        const auto hkx=output/"wallRun.hkx",config=output/"configs"/"wallRun.json";
        if(std::filesystem::exists(hkx)||std::filesystem::exists(config))throw std::runtime_error("Migration output already exists");
        fc::Library library;const auto loaded=fc::loadAnimationPack(library,pack);if(!loaded.committed)throw std::runtime_error(loaded.error);
        if(split) {
            std::ifstream source(pack);auto manifest=nlohmann::json::parse(source);
            std::map<std::filesystem::path,std::vector<std::uint8_t>> files;std::vector<std::string> warnings;
            auto encode=[&](const std::string& name,fc::ConverterWallRunPack value) {
                for(auto& clip:value.config["clips"])clip["file"]=name+".hkx";
                for(auto& sequence:value.config["sequences"])for(const auto key:{"launch","catch"})sequence[key]["file"]=name+".hkx";
                fc::selectAnimationClipConfig(value.config,name);files.emplace(output/(name+".hkx"),std::move(value.hkx));
                const auto text=value.config.dump(2)+"\n";files.emplace(output/"configs"/(name+".json"),std::vector<std::uint8_t>(text.begin(),text.end()));
                warnings.insert(warnings.end(),value.warnings.begin(),value.warnings.end());
            };
            for(const auto direction:{"runUp","runLeft","runRight","runDiagonalLeft","runDiagonalRight"}) {
                const auto roles=fc::converter::wallRunStages(direction);std::vector<fc::ConverterPreparedClip> stages;
                for(std::size_t part=0;part<3;++part) {
                    const auto doc=fc::loadConverterBaseEditor(pack,std::string(roles[part]),direction);
                    const auto saved=std::find_if(doc.snapshots.begin(),doc.snapshots.end(),[&](const auto& value){return value.path==doc.input;});
                    if(saved==doc.snapshots.end())throw std::runtime_error("Captured base animation is missing");
                    fc::ConverterInputClip clip;std::string error;if(!fc::decodeConverterInput(saved->bytes,clip,error,doc.templateConfig.value("member",std::string{})))throw std::runtime_error(error);
                    stages.push_back({fc::sliceConverterTimeline(clip,doc.templateConfig),doc.templateConfig,{},doc.snapshots,direction});
                }
                const auto loop=fc::loadConverterBaseEditor(pack,direction,direction);fc::ConverterWallRunPack result;
                if(loop.templateConfig.contains("frameRange")) {
                    const auto saved=std::find_if(loop.snapshots.begin(),loop.snapshots.end(),[&](const auto& value){return value.path==loop.input;});
                    fc::ConverterInputClip clip;std::string error;if(saved==loop.snapshots.end()||!fc::decodeConverterInput(saved->bytes,clip,error,loop.templateConfig.value("member",std::string{})))throw std::runtime_error(error);
                    result.hkx=fc::bundleConverterHkx({{direction,fc::writeCanonicalConverterHkx(clip,library)}});
                    result.config={{"format","FreeClimbActionGroup"},{"version",2},{"group","wallRun"},{"direction",direction},{"clips",nlohmann::json::array()},{"sequences",nlohmann::json::array()}};
                    for(const auto& stage:stages)result.config["clips"].push_back(stage.config);
                    result.config["sequences"].push_back({{"slot",direction},{"launch",stages[0].config},{"catch",stages[2].config}});
                }else result=fc::composeConverterWallRunDirection(library,direction,stages);
                encode(direction,std::move(result));
            }
            const auto brace=fc::loadConverterBaseEditor(pack,"sideBrace");const auto saved=std::find_if(brace.snapshots.begin(),brace.snapshots.end(),[&](const auto& value){return value.path==brace.input;});
            fc::ConverterInputClip clip;std::string error;if(saved==brace.snapshots.end()||!fc::decodeConverterInput(saved->bytes,clip,error,brace.templateConfig.value("member",std::string{})))throw std::runtime_error(error);
            auto braceConfig=brace.templateConfig;clip=fc::sliceConverterTimeline(clip,braceConfig);braceConfig["file"]="sideBrace.hkx";braceConfig.erase("member");braceConfig.erase("frameRange");braceConfig.erase("rootShift");
            files.emplace(output/"sideBrace.hkx",fc::writeCanonicalConverterHkx(clip,library));const auto braceText=braceConfig.dump(2)+"\n";files.emplace(output/"configs"/"sideBrace.json",std::vector<std::uint8_t>(braceText.begin(),braceText.end()));
            for(auto& entry:manifest["motions"]) {
                const auto name=entry.at("slot").get<std::string>();std::string owner;
                if(fc::converter::isWallRunPrimary(name)||name=="sideBrace")owner=name;
                else if(name=="runLaunch"||name=="runCatch")owner="runUp";
                else if(name=="runLaunchLeft")owner="runLeft";
                else if(name=="runLaunchRight")owner="runRight";
                if(!owner.empty())entry["config"]="configs/"+owner+".json";
            }
            const auto text=manifest.dump(2)+"\n";files.emplace(output/"pack.json",std::vector<std::uint8_t>(text.begin(),text.end()));
            for(const auto& [path,bytes]:files)if(std::filesystem::exists(path))throw std::runtime_error("Migration output already exists");
            for(const auto& [path,bytes]:files){std::filesystem::create_directories(path.parent_path());std::ofstream stream(path,std::ios::binary);stream.write(reinterpret_cast<const char*>(bytes.data()),std::streamsize(bytes.size()));stream.close();if(!stream)throw std::runtime_error("Could not write direction migration");}
            std::cout<<nlohmann::json({{"ok",true},{"directions",5},{"files",files.size()},{"warnings",warnings}}).dump()<<'\n';return 0;
        }
        std::vector<fc::ConverterPreparedClip> stages;
        auto append=[&](const std::string& slot,const std::string& direction){
            const auto doc=fc::loadConverterBaseEditor(pack,slot,direction);const auto found=std::find_if(doc.snapshots.begin(),doc.snapshots.end(),[&](const auto& value){return value.path==doc.input;});
            if(found==doc.snapshots.end())throw std::runtime_error("Captured base animation is missing");
            fc::ConverterInputClip clip;std::string error;if(!fc::decodeConverterInput(found->bytes,clip,error,doc.templateConfig.value("member",std::string{})))throw std::runtime_error(error);
            clip=fc::sliceConverterTimeline(clip,doc.templateConfig);stages.push_back({std::move(clip),doc.templateConfig,{},doc.snapshots,direction});
        };
        for(const auto direction:{"runUp","runLeft","runRight","runDiagonalLeft","runDiagonalRight"}){
            const auto roles=fc::converter::wallRunStages(direction);for(std::size_t stage=0;stage<3;++stage)append(std::string(roles[stage]),direction);
        }
        append("sideBrace",{});
        auto result=fc::composeConverterWallRunPack(library,stages);for(auto& clip:result.config["clips"])clip["file"]="wallRun.hkx";
        for(auto& sequence:result.config["sequences"])for(const auto key:{"launch","catch"})sequence[key]["file"]="wallRun.hkx";
        fc::selectAnimationClipConfig(result.config,"runUp");std::filesystem::create_directories(config.parent_path());
        std::ofstream binary(hkx,std::ios::binary);binary.write(reinterpret_cast<const char*>(result.hkx.data()),std::streamsize(result.hkx.size()));binary.close();if(!binary)throw std::runtime_error("Timeline HKX could not be written");
        std::ofstream metadata(config,std::ios::binary);metadata<<result.config.dump(2)<<'\n';metadata.close();if(!metadata)throw std::runtime_error("Timeline configuration could not be written");
        std::cout<<nlohmann::json({{"ok",true},{"directions",5},{"members",6},{"logicalStages",10},{"bytes",result.hkx.size()},{"warnings",result.warnings}}).dump()<<'\n';return 0;
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
