#include "ConverterEditor.h"
#include "AnimationPack.h"
#include "HkxAnimation.h"
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>

using Bytes=std::vector<std::uint8_t>;
using Json=nlohmann::json;
static void check(bool value,const std::string& why){if(!value)throw std::runtime_error(why);}
static Bytes read(const std::filesystem::path& p){std::ifstream f(p,std::ios::binary);check(bool(f),"Read "+p.string());return {std::istreambuf_iterator<char>(f),{}};}
static void write(const std::filesystem::path& p,const Bytes& bytes){std::filesystem::create_directories(p.parent_path());std::ofstream f(p,std::ios::binary);check(bool(f.write(reinterpret_cast<const char*>(bytes.data()),std::streamsize(bytes.size()))),"Write fixture");}
template<class T>static T get(const Bytes& bytes,std::size_t at){check(at<=bytes.size()&&sizeof(T)<=bytes.size()-at,"Field bounds");T v;std::memcpy(&v,bytes.data()+at,sizeof v);return v;}
template<class T>static void put(Bytes& bytes,std::size_t at,T value){check(at<=bytes.size()&&sizeof(T)<=bytes.size()-at,"Write bounds");std::memcpy(bytes.data()+at,&value,sizeof value);}
static std::map<std::string,Bytes> unpack(const std::filesystem::path& p){
    const auto bytes=read(p);const auto end=bytes.size()-22;check(get<std::uint32_t>(bytes,end)==0x06054b50u,"ZIP end");std::size_t at=get<std::uint32_t>(bytes,end+16);std::map<std::string,Bytes> result;
    for(unsigned i=0;i<get<std::uint16_t>(bytes,end+10);++i){check(get<std::uint32_t>(bytes,at)==0x02014b50u,"ZIP central");const auto count=get<std::uint16_t>(bytes,at+28),extra=get<std::uint16_t>(bytes,at+30),comment=get<std::uint16_t>(bytes,at+32);const auto offset=get<std::uint32_t>(bytes,at+42),size=get<std::uint32_t>(bytes,at+24);const auto data=offset+30+get<std::uint16_t>(bytes,offset+26)+get<std::uint16_t>(bytes,offset+28);check(data+size<=bytes.size()&&at+46+count<=bytes.size(),"ZIP bounds");result.emplace(std::string(reinterpret_cast<const char*>(bytes.data()+at+46),count),Bytes(bytes.begin()+data,bytes.begin()+data+size));at+=46+count+extra+comment;}
    check(at==end,"ZIP boundary");return result;
}
static void equal(const fc::HkxClip& a,const fc::HkxClip& b){check(a.duration==b.duration&&a.boneIndices==b.boneIndices&&a.frames.size()==b.frames.size(),"Member metadata preserved");for(std::size_t f=0;f<a.frames.size();++f)check(std::memcmp(a.frames[f].data(),b.frames[f].data(),a.frames[f].size()*sizeof(fc::Transform))==0,"Member poses remain bit exact");}
int main(int argc,char** argv)try{
    check(argc==3,"Pack and output required");const std::filesystem::path pack=argv[1],work=std::filesystem::path(argv[2])/std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());std::filesystem::create_directories(work);
    const auto load=[](const std::filesystem::path& path,const std::string& name){return std::make_unique<fc::ConverterEditorDocument>(fc::loadConverterBaseEditor(path,name));};
    const auto leftStorage=load(pack,"runLeft"),launchStorage=load(pack,"runLaunchLeft");const auto& left=*leftStorage;const auto& launch=*launchStorage;std::vector<std::string> warnings;
    const auto leftBytes=fc::writeConverterHkx(left.clip,left.base,warnings),launchBytes=fc::writeConverterHkx(launch.clip,launch.base,warnings);
    const auto bundle=fc::bundleConverterHkx({{"runLeft",leftBytes},{"runLaunchLeft",launchBytes}});std::string error;fc::HkxClip baseline,selected;fc::ConverterInputClip input;
    check(!fc::decodeHkxAnimation(bundle,selected,error),"Ambiguous group requires member");
    check(!fc::decodeHkxAnimation(bundle,selected,error,"absent"),"Unknown member rejected");
    std::vector<std::pair<std::string,fc::HkxClip>> decodedMembers;
    check(!fc::decodeHkxAnimationMembers(bundle,decodedMembers,error,1)&&decodedMembers.empty(),"Group memory budget rejects transactionally");
    for(const auto& member:std::vector<std::pair<std::string,Bytes>>{{"runLeft",leftBytes},{"runLaunchLeft",launchBytes}}){check(fc::decodeHkxAnimation(member.second,baseline,error),error);check(fc::decodeHkxAnimation(bundle,selected,error,member.first),error);equal(baseline,selected);check(fc::decodeConverterInput(bundle,input,error,member.first),error);check(input.frames.size()==selected.frames.size()&&input.duration==selected.duration,"Tool resolves selected member");}
    bool duplicate=false;try{fc::bundleConverterHkx({{"runLeft",leftBytes},{"runLeft",launchBytes}});}catch(...){duplicate=true;}check(duplicate,"Duplicate member rejected by writer");
    const auto begin=get<std::uint32_t>(bundle,180),local=get<std::uint32_t>(bundle,184),global=get<std::uint32_t>(bundle,188),virtuals=get<std::uint32_t>(bundle,192);std::map<std::uint32_t,std::uint32_t> pointers;std::map<std::uint32_t,std::size_t> localRows;
    for(auto p=local;p+8<=global;p+=8){const auto from=get<std::uint32_t>(bundle,begin+p);if(from!=0xffffffffu){pointers[from]=get<std::uint32_t>(bundle,begin+p+4);localRows[from]=begin+p+4;}}
    for(auto p=global;p+12<=virtuals;p+=12){const auto from=get<std::uint32_t>(bundle,begin+p);if(from!=0xffffffffu)pointers[from]=get<std::uint32_t>(bundle,begin+p+8);}
    const auto variants=pointers.at(0),container=pointers.at(variants+16),animations=pointers.at(container+32),second=pointers.at(animations+8);
    auto corrupt=bundle;put(corrupt,begin+second+20,std::uint32_t(0x7fc00000));check(!fc::decodeHkxAnimation(corrupt,selected,error,"runLeft"),"Bad unselected member rejects group transactionally");check(!fc::decodeConverterInput(corrupt,input,error,"runLeft"),"Tool also rejects bad unselected member");
    auto duplicateName=bundle;put(duplicateName,localRows.at(variants+48),pointers.at(variants+24));check(!fc::decodeHkxAnimation(duplicateName,selected,error,"runLeft"),"Duplicate root member rejected");
    fc::ConverterEditOptions options;options.speed=1.1f;const auto a=fc::applyConverterEdits(left,options),b=fc::applyConverterEdits(launch,{});const auto zip=work/"run.zip";const auto report=fc::exportConverterEditorGroup({{&left,&a},{&launch,&b}},zip);
    check(report.at("loaded")==fc::activeMotionCount&&report.at("files").size()==2&&report.at("slots").size()==4,"Editing two stages exports one complete wall-run HKX and one configuration");const auto archive=unpack(zip);std::size_t hkxCount=0;for(const auto& [name,bytes]:archive)hkxCount+=std::filesystem::path(name).extension()==".hkx";check(hkxCount==1&&archive.size()==2,"Complete action has one HKX and one JSON without pack or skeleton");
    const auto replacementConfig=Json::parse(archive.at("meshes/actors/character/animations/FreeClimb/configs/runLeft.json"));check(replacementConfig.at("format")=="FreeClimbActionGroup"&&replacementConfig.at("clips").size()==4,"Complete action configuration includes every direction and helper");
    for(const auto motion:{fc::Motion::runLaunchLeft,fc::Motion::runLeft,fc::Motion::runCatch}){const std::string name(fc::motionSlotNames[int(motion)-1]);if(name=="runLeft"||name=="runLaunchLeft")continue;const auto docStorage=load(pack,name);const auto& doc=*docStorage;auto original=doc.templateConfig,current=fc::selectAnimationClipConfig(replacementConfig,name);original.erase("file");original.erase("member");original.erase("frameRange");original.erase("rootShift");current.erase("file");current.erase("member");current.erase("frameRange");current.erase("rootShift");check(original==current,"Unedited stage configuration is exactly preserved");}
    const auto root=pack.parent_path(),stage=work/"pack";const auto metadata=Json::parse(read(pack));write(stage/"pack.json",read(pack));write(stage/metadata.at("skeleton").get<std::string>(),read(root/metadata.at("skeleton").get<std::string>()));
    for(const auto& entry:metadata.at("motions")){const auto file=entry.at("config").get<std::string>();const auto bytes=read(root/file);write(stage/file,bytes);const auto hkx=fc::selectAnimationClipConfig(Json::parse(bytes),entry.at("slot").get<std::string>()).at("file").get<std::string>();write(stage/hkx,read(root/hkx));}
    const std::string prefix="meshes/actors/character/animations/FreeClimb/";for(const auto& [name,bytes]:archive){check(name.starts_with(prefix),"Archive namespace");write(stage/name.substr(prefix.size()),bytes);}
    auto mergedStorage=std::make_unique<fc::Library>();auto& merged=*mergedStorage;const auto loaded=fc::loadAnimationPack(merged,stage/"pack.json");check(loaded.committed,"Merged group loads");
    fc::AnimationOverrideLimits exact;exact.totalOutputBytes=loaded.outputBytes;auto limitedStorage=std::make_unique<fc::Library>();auto& limited=*limitedStorage;check(fc::loadAnimationPack(limited,stage/"pack.json",exact).committed,"Cache memory does not reduce the final output allowance");--exact.totalOutputBytes;check(!fc::loadAnimationPack(limited,stage/"pack.json",exact).committed,"Final output budget still enforced");
    for(const auto m:fc::activeMotions){if(m==fc::Motion::runLeft||m==fc::Motion::runLaunchLeft)continue;const auto& x=left.base.clip(m);const auto& y=merged.clip(m);check(std::abs(x.seconds-y.seconds)<.000001f&&x.frames.size()==y.frames.size(),"Unedited stage timing preserved");for(std::size_t i=0;i<x.frames.size();++i)check(std::memcmp(x.frames[i].data(),y.frames[i].data(),99*sizeof(fc::Transform))==0,"Unedited stages unchanged");}
    const auto reloadedStorage=load(stage/"pack.json","runLaunchLeft");const auto& reloaded=*reloadedStorage;check(std::abs(reloaded.clip.duration-b.clip.duration)<.000001f,"Grouped export reopens selected stage");
    {
        const auto rightStorage=load(pack,"runRight");const auto& right=*rightStorage;fc::ConverterEditOptions rightOptions;rightOptions.speed=1.1f;
        const auto rightEdit=fc::applyConverterEdits(right,rightOptions);const auto rightReport=fc::exportConverterEditor(right,rightEdit,work/"right.zip");
        check(rightReport.at("files").size()==2&&rightReport.at("slots").size()==3,"A second direction exports only its own complete animation");
        const auto overlay=work/"independent-directions";std::filesystem::copy(stage,overlay,std::filesystem::copy_options::recursive);
        for(const auto& [name,bytes]:unpack(work/"right.zip")){check(name!=prefix+"configs/runLeft.json","Direction overrides never share a configuration path");write(overlay/name.substr(prefix.size()),bytes);}
        auto combined=std::make_unique<fc::Library>();check(fc::loadAnimationPack(*combined,overlay/"pack.json").committed,"Independent direction packs can be overlaid together");
        for(const auto motion:{fc::Motion::runLeft,fc::Motion::runLaunchLeft}) {
            const auto& x=merged.clip(motion);const auto& y=combined->clip(motion);check(x.seconds==y.seconds&&x.frames.size()==y.frames.size(),"Right override preserves left timing");
            for(std::size_t frame=0;frame<x.frames.size();++frame)check(std::memcmp(x.frames[frame].data(),y.frames[frame].data(),99*sizeof(fc::Transform))==0,"Right override preserves every left pose");
        }
        check(std::abs(combined->clip(fc::Motion::runRight).seconds-rightEdit.clip.duration)<.000001f,"Second direction retains its own edit");
        for(unsigned index:{0u,1u,3u,4u}) {
            check(combined->wallRunLaunches[index].seconds==merged.wallRunLaunches[index].seconds&&combined->wallRunCatches[index].seconds==merged.wallRunCatches[index].seconds,"Other directions retain their own start and end clocks");
        }
    }
    const auto single=fc::exportConverterEditor(launch,b,work/"single.zip");check(single.at("files").size()==2&&single.at("slots").size()==4,"Single-stage edits also export the complete group");for(const auto& [name,bytes]:unpack(work/"single.zip"))write(stage/name.substr(prefix.size()),bytes);check(fc::loadAnimationPack(merged,stage/"pack.json").committed,"Complete action replacement loads");check(std::abs(merged.clip(fc::Motion::runLeft).seconds-left.clip.duration)<.000001f,"Unedited sibling comes from the selected base pack rather than a prior replacement");
    for(bool entireGroup:{false,true}) {
        const auto aliasRoot=work/(entireGroup?"generated-case-pack":"member-case-pack"),aliasOutput=work/(entireGroup?"generated-case.zip":"member-case.zip");std::filesystem::copy(stage,aliasRoot,std::filesystem::copy_options::recursive);
        const auto configPath=aliasRoot/"configs/runLeft.json";auto groupConfig=Json::parse(read(configPath));const auto originalName=fc::selectAnimationClipConfig(groupConfig,"runLaunchLeft").at("file").get<std::string>();auto aliasName=originalName;
        const auto filename=aliasName.find_last_of('/');for(std::size_t i=filename==std::string::npos?0:filename+1;i<aliasName.size();++i)if(aliasName[i]>='a'&&aliasName[i]<='z')aliasName[i]=char(aliasName[i]-'a'+'A');check(aliasName!=originalName,"Case fixture changes the animation spelling");
        if(!std::filesystem::exists(aliasRoot/aliasName))write(aliasRoot/aliasName,read(aliasRoot/originalName));const bool sameFile=std::filesystem::equivalent(aliasRoot/originalName,aliasRoot/aliasName);
        if(entireGroup){for(auto& clip:groupConfig.at("clips"))clip["file"]=aliasName;if(groupConfig.contains("sequences"))for(auto& sequence:groupConfig["sequences"])for(const auto key:{"launch","catch","brace"})if(sequence.contains(key))sequence[key]["file"]=aliasName;const auto text=groupConfig.dump(2);write(configPath,Bytes(text.begin(),text.end()));}
        else{const auto entry=std::find_if(metadata.at("motions").begin(),metadata.at("motions").end(),[](const auto& value){return value.at("slot")=="hang";});check(entry!=metadata.at("motions").end(),"Independent case fixture slot exists");const auto file=aliasRoot/entry->at("config").get<std::string>();auto config=Json::parse(read(file));const auto selected=fc::selectAnimationClipConfig(groupConfig,"runLeft");config["file"]=aliasName;config["member"]="runLeft";if(selected.contains("frameRange")){config["frameRange"]=selected["frameRange"];config["rootShift"]=selected["rootShift"];}const auto text=config.dump(2);write(file,Bytes(text.begin(),text.end()));}
        auto beforeStorage=std::make_unique<fc::Library>();auto& before=*beforeStorage;const auto beforeReport=fc::loadAnimationPack(before,aliasRoot/"pack.json");std::string reason=beforeReport.error;for(const auto& slot:beforeReport.slots)if(slot.status==fc::OverrideStatus::rejected)reason+="; "+slot.file+": "+slot.reason;check(beforeReport.committed,"Case fixture remains a valid complete runtime pack: "+reason);
        fc::ConverterRequest request;request.pack=aliasRoot/"pack.json";request.output=aliasOutput;request.slot="runLaunchLeft";fc::ConverterPreparedClip edit{b.clip,fc::selectAnimationClipConfig(groupConfig,request.slot),b.warnings,{}};
        if(!sameFile){bool rejected=false;try{fc::convertHkxGroup(request,{edit});}catch(const std::exception& failure){rejected=std::string(failure.what()).find("Ambiguously cased animation path")!=std::string::npos;}check(rejected&&!std::filesystem::exists(aliasOutput),"Case-sensitive distinct HKX files cannot be treated as one shared snapshot");continue;}
        const auto caseReport=fc::convertHkxGroup(request,{edit});check(caseReport.at("files").size()==2&&caseReport.at("slots").size()==4,"Equivalent case aliases export a complete group");const auto caseArchive=unpack(aliasOutput);
        check(caseArchive.at(prefix+originalName)==read(stage/originalName),"Repeated export resolves an existing differently cased content-addressed file without changing its bytes");
        for(const auto& [name,bytes]:caseArchive)write(aliasRoot/name.substr(prefix.size()),bytes);auto afterStorage=std::make_unique<fc::Library>();auto& after=*afterStorage;check(fc::loadAnimationPack(after,aliasRoot/"pack.json").committed,"Case-normalized group replacement remains loadable");
        for(const auto motion:fc::activeMotions){if(motion==fc::Motion::runLaunchLeft)continue;const auto& x=before.clip(motion);const auto& y=after.clip(motion);check(std::abs(x.seconds-y.seconds)<.000001f&&x.frames.size()==y.frames.size(),"Case aliases preserve sibling timing");for(std::size_t frame=0;frame<x.frames.size();++frame)check(std::memcmp(x.frames[frame].data(),y.frames[frame].data(),99*sizeof(fc::Transform))==0,"Case aliases preserve every unedited sibling pose exactly");}
    }
    const auto hopStorage=load(pack,"contextHopLeft"),rightHopStorage=load(pack,"contextHopRight");const auto& hop=*hopStorage;const auto& rightHop=*rightHopStorage;
    const auto hopEdit=fc::applyConverterEdits(hop,{});const auto contextReport=fc::exportConverterEditor(hop,hopEdit,work/"hop.zip");check(contextReport.at("files").size()==2&&contextReport.at("slots").size()==2,"Left contextual leap exports one independent complete timeline and its private metadata");
    fc::ConverterEditOptions hopOptions;hopOptions.speed=1.1f;const auto rightHopEdit=fc::applyConverterEdits(rightHop,hopOptions);fc::exportConverterEditor(rightHop,rightHopEdit,work/"hop-right.zip");
    const auto leftHopArchive=unpack(work/"hop.zip"),rightHopArchive=unpack(work/"hop-right.zip");for(const auto& [name,bytes]:leftHopArchive)check(!rightHopArchive.contains(name),"Left and right contextual leap exports never overwrite the same path");
    const auto contextPack=work/"independent-context-hops";std::filesystem::copy(stage,contextPack,std::filesystem::copy_options::recursive);
    for(const auto& archive:{leftHopArchive,rightHopArchive})for(const auto& [name,bytes]:archive)write(contextPack/name.substr(prefix.size()),bytes);
    auto combinedContext=std::make_unique<fc::Library>();check(fc::loadAnimationPack(*combinedContext,contextPack/"pack.json").committed,"Both contextual leap overrides load together");
    const auto leftReload=load(contextPack/"pack.json","contextHopLeft"),rightReload=load(contextPack/"pack.json","contextHopRight");check(leftReload->clip.frames.size()==hop.clip.frames.size()&&std::abs(leftReload->clip.duration-hop.clip.duration)<.000001f,"Right replacement cannot alter the complete left timeline");
    for(std::size_t frame=0;frame<hop.clip.frames.size();++frame)check(std::memcmp(leftReload->clip.frames[frame].data(),hop.clip.frames[frame].data(),99*sizeof(fc::Transform))==0,"Independent contextual leap preserves every opposite pose");
    check(std::abs(rightReload->clip.duration-rightHop.clip.duration/1.1f)<.000001f,"Right complete timeline retains its independent speed edit");
    {
        auto fullSource=hop.clip;fullSource.duration=10.0f;const auto sourcePath=work/"full-context-source.hkx";write(sourcePath,fc::writeCanonicalConverterHkx(fullSource,hop.base));
        const auto imported=fc::loadConverterEditor(sourcePath,pack,"contextHopLeft");const auto edited=fc::applyConverterEdits(imported,{});fc::exportConverterEditor(imported,edited,work/"full-context.zip");
        const auto archive=unpack(work/"full-context.zip");check(archive.size()==2,"Full imported side leap keeps one HKX and its configuration");
        const auto config=Json::parse(archive.at(prefix+"configs/contextHopLeft.json"));const auto main=fc::selectAnimationClipConfig(config,"contextHopLeft");check(main.contains("authoredPlayback")&&!main.contains("frameRange")&&main.at("member")=="contextHopLeft","Full source side leap remains one complete authored clip");
        const auto& sequence=config.at("sequences").at(0);for(const auto key:{"prepare","catch"})check(sequence.at(key).at("file")==main.at("file")&&sequence.at(key).at("member")!=main.at("member"),"Private calibration does not append playback phases to imported source");
        const auto mergedPath=work/"full-context-pack";std::filesystem::copy(contextPack,mergedPath,std::filesystem::copy_options::recursive);for(const auto& [name,bytes]:archive)write(mergedPath/name.substr(prefix.size()),bytes);
        auto loaded=std::make_unique<fc::Library>();check(fc::loadAnimationPack(*loaded,mergedPath/"pack.json").committed,"Maximum-length full side leap imports without exceeding timeline limits");check(std::abs(loaded->clip(fc::Motion::contextHopLeft).seconds-10.0f)<.00001f,"Full imported side leap retains its complete source duration");
        const auto& before=combinedContext->clip(fc::Motion::contextHopRight);const auto& after=loaded->clip(fc::Motion::contextHopRight);check(before.frames.size()==after.frames.size()&&before.seconds==after.seconds,"Full left import preserves opposite direction timing");for(std::size_t frame=0;frame<before.frames.size();++frame)check(std::memcmp(before.frames[frame].data(),after.frames[frame].data(),99*sizeof(fc::Transform))==0,"Full left import preserves opposite direction poses");
    }
    bool mixed=false;try{fc::exportConverterEditorGroup({{&left,&a},{&hop,&hopEdit}},work/"mixed.zip");}catch(...){mixed=true;}check(mixed&&!std::filesystem::exists(work/"mixed.zip"),"Unrelated families rejected before publication");
    std::cout<<"Group bytes, selectors, malformed members, complete-action export, exact unedited stages and context grouping passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
