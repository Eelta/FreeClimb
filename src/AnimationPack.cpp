#include "AnimationPack.h"
#include "AnimationConfig.h"
#include "CanonicalSkeleton.h"
#include "HkxAnimation.h"
#include "../external/nlohmann/json.hpp"
#include <chrono>
#include <map>
#include <set>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

namespace fc {
namespace {
using Json=nlohmann::json;
struct LogicalPathLess {
    bool operator()(const std::filesystem::path& a,const std::filesystem::path& b) const {
#ifdef _WIN32
        return CompareStringOrdinal(a.c_str(),int(a.native().size()),b.c_str(),int(b.native().size()),TRUE)==CSTR_LESS_THAN;
#else
        return a<b;
#endif
    }
};
struct CachedHkx {
    std::vector<std::pair<std::string,HkxClip>> members;
    std::string failure;
    std::size_t bytes{};
    std::uint64_t contentId{};
    bool missing{};
};
void requirePack(bool valid,const std::string& message) {if(!valid)throw std::runtime_error(message);}
float scalar(const Json& j,float low,float high) {
    requirePack(j.is_number(),"Expected numeric metadata");
    const auto value=j.get<float>();
    requirePack(std::isfinite(value)&&value>=low&&value<=high,"Numeric metadata is outside supported bounds");return value;
}
Vec vector(const Json& j,float low,float high) {
    requirePack(j.is_array()&&j.size()==3,"Expected a three-component vector");
    return {scalar(j[0],low,high),scalar(j[1],low,high),scalar(j[2],low,high)};
}
std::array<float,2> window(const Json& j) {
    requirePack(j.is_array()&&j.size()==2,"Expected a phase window");
    std::array<float,2> result{scalar(j[0],0,1),scalar(j[1],0,1)};
    requirePack(result[1]-result[0]>=.001f,"Phase windows must increase by at least .001");return result;
}
Json readJson(const std::filesystem::path& path,std::size_t limit,std::size_t& total,std::size_t totalLimit) {
    const auto size=std::filesystem::file_size(path);
    requirePack(size>0&&size<=limit&&size<=totalLimit-total,"JSON file or total input exceeds limit");
    total+=std::size_t(size);std::ifstream stream(path,std::ios::binary);
    requirePack(bool(stream),"Cannot open JSON file");
    std::string bytes(std::size_t(size),'\0');
    requirePack(bool(stream.read(bytes.data(),std::streamsize(bytes.size())))&&stream.peek()==std::char_traits<char>::eof(),
        "JSON file changed while being read");
    std::vector<std::set<std::string>> keys;
    auto callback=[&](int depth,Json::parse_event_t event,Json& value) {
        requirePack(depth<=32,"JSON nesting exceeds limit");
        if(event==Json::parse_event_t::object_start)keys.emplace_back();
        else if(event==Json::parse_event_t::key)requirePack(!keys.empty()&&keys.back().insert(value.get<std::string>()).second,"Duplicate JSON key");
        else if(event==Json::parse_event_t::object_end)keys.pop_back();
        return true;
    };
    auto result=Json::parse(bytes,callback,true,false);
    requirePack(result.is_object(),"JSON document must be an object");return result;
}
void ordinaryPath(const std::filesystem::path& path) {
#ifdef _WIN32
    const auto attributes=GetFileAttributesW(path.c_str());
    if(attributes==INVALID_FILE_ATTRIBUTES) {
        const auto error=GetLastError();
        requirePack(error==ERROR_FILE_NOT_FOUND||error==ERROR_PATH_NOT_FOUND,"Cannot inspect animation path");return;
    }
    requirePack(!(attributes&FILE_ATTRIBUTE_REPARSE_POINT),"Animation paths cannot use links or reparse points");
#else
    std::error_code error;const auto status=std::filesystem::symlink_status(path,error);
    requirePack(!error||error==std::errc::no_such_file_or_directory,"Cannot inspect animation path");
    requirePack(!std::filesystem::is_symlink(status),"Animation paths cannot use links");
#endif
}
std::filesystem::path contained(const std::filesystem::path& root,const Json& value) {
    requirePack(value.is_string(),"Expected a relative file path");
    const auto text=value.get<std::string>();
    requirePack(!text.empty()&&text.size()<=240&&text.find(':')==std::string::npos&&text.find('\0')==std::string::npos,"Invalid animation file path");
    const std::filesystem::path child(text);
    requirePack(!child.is_absolute()&&!child.has_root_path(),"Animation paths must be relative");
    auto path=root;
    for(const auto& part:child) {
        requirePack(part!="..","Animation paths cannot escape their pack");
        if(part==".")continue;
        const auto component=part.native();
        requirePack(!component.empty()&&component.back()!='.'&&component.back()!=' ',"Invalid animation path component");
        path/=part;ordinaryPath(path);
    }
    return path;
}
void format(const Json& j,const char* expected) {
    const bool authored=std::string_view(expected)=="FreeClimbClip"&&j.contains("authoredPlayback");
    requirePack(j.at("format")==expected&&j.at("version")==int(authored?2:1),"Unsupported animation metadata format or version");
}
void skeleton(Library& result,const Json& j) {
    format(j,"FreeClimbSkeleton");const auto& bones=j.at("bones");
    requirePack(bones.is_array()&&bones.size()==99,"Skeleton requires exactly 99 canonical bones");
    for(std::size_t i=0;i<99;++i) {
        const auto& b=bones[i];const auto name=b.at("name").get<std::string>();const auto parent=b.at("parent").get<int>();
        requirePack(name==canonicalBoneNames[i]&&parent==canonicalBoneParents[i],"Skeleton bone names or hierarchy do not match the canonical rig");
        Transform t;t.t=vector(b.at("t"),-1000,1000);t.s=vector(b.at("s"),.1f,5);
        const auto& q=b.at("q");requirePack(q.is_array()&&q.size()==4,"Skeleton quaternion requires four components");
        t.q={scalar(q[0],-1,1),scalar(q[1],-1,1),scalar(q[2],-1,1),scalar(q[3],-1,1)};
        const auto& c=canonicalBoneRest[i];const Transform reference{{c[0],c[1],c[2]},{c[3],c[4],c[5],c[6]},{c[7],c[8],c[9]}};
        requirePack(std::abs(t.q.dot(t.q)-1)<.001f&&(t.t-reference.t).length()<=.002f&&
            (t.s-reference.s).length()<=.0002f&&angleBetween(t.q,reference.q)<=.0002f,
            "Skeleton bind transforms differ from the supported canonical rig");
        result.names.push_back(name);result.parents.push_back(parent);result.rest.push_back(t);
    }
}
void profile(Library& result,std::size_t slot,const Json& j) {
    auto& p=result.threepeatProfile;
    if(slot==39||slot==40) {
        const std::size_t side=slot-39;const auto& path=j.at("path");
        requirePack(path.is_array()&&path.size()>=2&&path.size()<=64,"Side-action paths require 2..64 knots");
        p.pathCounts[side]=std::uint32_t(path.size());
        for(std::size_t k=0;k<path.size();++k) {
            const auto& row=path[k];requirePack(row.is_array()&&row.size()==4,"Path knots require phase, travel, lift and outward components");
            auto& knot=p.paths[side][k];knot={scalar(row[0],0,1),scalar(row[1],-.25f,1.5f),scalar(row[2],-32,96),scalar(row[3],0,48)};
            if(k)requirePack(knot.phase-p.paths[side][k-1].phase>=.001f,"Path knot phases must strictly increase");
        }
        const auto a=p.paths[side][0],b=p.paths[side][path.size()-1];
        requirePack(a.phase==0&&b.phase==1&&std::abs(a.travel)<.0001f&&std::abs(b.travel-1)<.0001f&&
            std::abs(a.lift)<.0001f&&std::abs(b.lift)<.0001f&&std::abs(a.out)<.0001f&&std::abs(b.out)<.0001f,
            "Action paths must begin and end at their verified anchors");
        for(const auto* key:{"sourceHands","targetHands"})requirePack(j.at(key).is_array()&&j.at(key).size()==2,"Two hand windows are required");
        for(int hand=0;hand<2;++hand) {
            p.source[side][hand]=window(j.at("sourceHands")[hand]);p.target[side][hand]=window(j.at("targetHands")[hand]);
            requirePack(p.source[side][hand][1]<=p.target[side][hand][0],"A hand cannot load both action endpoints at once");
        }
        p.rise[side]=window(j.at("verticalBlend"));
    } else if(slot==41) {
        requirePack(j.at("releaseHands").is_array()&&j.at("releaseHands").size()==2,"Two mantle release windows are required");
        for(int hand=0;hand<2;++hand)p.mantleRelease[hand]=window(j.at("releaseHands")[hand]);
        p.mantleUnplant=window(j.at("unplant"));p.mantleReplant=window(j.at("replant"));
        p.replantSamplePhase=scalar(j.at("replantSamplePhase"),0,1);
        requirePack(p.mantleUnplant[1]<=p.mantleReplant[0]&&p.replantSamplePhase>=p.mantleReplant[0]&&
            p.replantSamplePhase<=p.mantleRelease[0][0]&&p.replantSamplePhase<=p.mantleRelease[1][0],
            "Mantle release, replant and calibration windows are inconsistent");
    }
}
void installClip(Library& result,std::size_t slot,const Json& j,const HkxClip& source) {
    auto& clip=result.clips[slot];clip.seconds=source.duration;
    clip.stride=scalar(j.at("stride"),0,500);clip.height=scalar(j.at("height"),-500,500);clip.travel=vector(j.at("travel"),-500,500);
    if(j.contains("authoredPlayback")) {
        requirePack(isActiveMotion(Motion(slot+1)),"Authored playback requires an active motion slot");
        const auto& authored=j.at("authoredPlayback");
        requirePack(authored.is_object()&&authored.at("version")==1,"Unsupported authored playback contract");
        clip.authoredPlayback=true;
        if(authored.contains("trajectory")) {
            requirePack(authored.value("basis",std::string{})=="root","Authored trajectory must use Root displacement");
            const auto& path=authored.at("trajectory");
            requirePack(path.is_array()&&(path.empty()||(path.size()>=2&&path.size()<=65)),"Authored trajectory requires 2..65 knots");
            clip.trajectory.count=std::uint32_t(path.size());float distance=0;
            for(std::size_t i=0;i<path.size();++i) {
                const auto& row=path[i];requirePack(row.is_array()&&row.size()==4,"Authored knots require phase, x, y and z");
                auto& knot=clip.trajectory.knots[i];knot.phase=scalar(row[0],0,1);
                knot.displacement={scalar(row[1],-500,500),scalar(row[2],-500,500),scalar(row[3],-500,500)};
                if(i) {
                    requirePack(knot.phase-clip.trajectory.knots[i-1].phase>=.0001f,"Authored trajectory phases must strictly increase");
                    distance+=(knot.displacement-clip.trajectory.knots[i-1].displacement).length();
                }
            }
            if(!path.empty())requirePack(path.front()[0]==0&&path.back()[0]==1&&clip.trajectory.knots[0].displacement.length()<.001f&&distance<=2000,
                "Authored trajectory requires normalized endpoints and a bounded path");
        }
    }
    std::array<int,99> mapping;mapping.fill(-1);
    for(std::size_t track=0;track<source.boneIndices.size();++track) {
        const auto bone=source.boneIndices[track];if(bone>=99)continue;
        requirePack(source.trackNames[track].empty()||source.trackNames[track]==result.names[bone],"HKX named track does not match the canonical skeleton mapping");
        if(source.identityMapping&&source.boneIndices.size()==126)requirePack(!source.trackNames[track].empty(),"126-track identity bindings require canonical names");
        mapping[bone]=int(track);
    }
    for(int track:mapping)requirePack(track>=0,"Full-pose animation requires all 99 canonical tracks");
    clip.frames.resize(source.frames.size(),Pose(99));
    for(std::size_t frame=0;frame<source.frames.size();++frame)for(std::size_t bone=0;bone<99;++bone) {
        const auto& t=source.frames[frame][mapping[bone]];const auto& rest=result.rest[bone];
        requirePack(t.t.length()<500,"HKX display translation exceeds the supported local range");
        requirePack((t.s-rest.s).length()<=.0002f,"Animated bone scale is unsupported; preserve canonical bone lengths");
        if(bone!=0&&bone!=4)requirePack((t.t-rest.t).length()<=.02f,"Animated joint translation is unsupported outside Root and COM");
        if((bone>=1&&bone<=3)||bone>=97)requirePack(angleBetween(t.q,rest.q)<=.002f,"HKX modifies a protected control or camera rotation");
        clip.frames[frame][bone]=t;
    }
    if(clip.authoredPlayback&&clip.trajectory.count) {
        const auto origin=clip.frames.front()[0].t;
        for(std::uint32_t i=0;i<clip.trajectory.count;++i) {
            const auto& knot=clip.trajectory.knots[i];const float at=knot.phase*float(clip.frames.size()-1);
            const auto index=std::min(std::size_t(at),clip.frames.size()-2);auto pose=clip.frames[index];
            for(std::size_t bone=0;bone<pose.size();++bone)pose[bone]=blend(pose[bone],clip.frames[index+1][bone],at-float(index));
            requirePack((pose[0].t-origin-knot.displacement).length()<.05f,
                "Authored trajectory does not match the animation's Root displacement");
        }
        for(std::size_t frame=0;frame<clip.frames.size();++frame) {
            const auto displacement=clip.frames[frame][0].t-origin;
            requirePack((displacement-clip.trajectory.sample(float(frame)/float(clip.frames.size()-1))).length()<=.25f,
                "Authored trajectory is too coarse for the animation's Root curve");
        }
    }
    if(clip.authoredPlayback&&!clip.trajectory.count)for(const auto& frame:clip.frames)
        requirePack((frame[0].t-clip.frames.front()[0].t).length()<=.25f,"Authored Root movement requires its matching trajectory");
    if(clip.authoredPlayback&&!authoredIdleLoop(Motion(slot+1))) {
        auto localCom=[&](const Pose& frame){return frame[0].q.inverse().rotate(result.world(frame)[4].t-frame[0].t);};
        const auto origin=localCom(clip.frames.front());
        for(const auto& frame:clip.frames)requirePack((localCom(frame)-origin).length()<=96,
            "Action COM moves too far relative to Root; put travel motion on Root");
        requirePack((localCom(clip.frames.back())-origin).length()<=64,
            "Action COM has excessive net travel; put travel motion on Root");
    }
    if(clip.authoredPlayback&&authoredIdleLoop(Motion(slot+1))) {
        const auto origin=result.world(clip.frames.front())[4].t;
        for(const auto& frame:clip.frames)requirePack((result.world(frame)[4].t-origin).length()<=24,
            "Wall idle moves too far from its anchor; use an in-place idle animation");
        requirePack((result.world(clip.frames.back())[4].t-origin).length()<=12,
            "Wall idle has excessive end-to-start motion; use a looping in-place idle animation");
    }
    const auto& samples=j.at("contacts");requirePack(samples.is_array()&&samples.size()>=2&&samples.size()<=1201,"Contacts require 2..1201 uniformly spaced weight samples");
    std::vector<std::array<float,4>> weights(samples.size());
    for(std::size_t i=0;i<samples.size();++i) {
        requirePack(samples[i].is_array()&&samples[i].size()==4,"Contact samples require four weights");
        for(int limb=0;limb<4;++limb)weights[i][limb]=scalar(samples[i][limb],0,1);
    }
    clip.contacts.resize(clip.frames.size());
    if(weights.size()==clip.contacts.size())clip.contacts=std::move(weights);
    else for(std::size_t i=0;i<clip.contacts.size();++i) {
        const float at=float(i)*float(weights.size()-1)/float(clip.contacts.size()-1);
        const auto a=std::min(std::size_t(at),weights.size()-2);const float alpha=at-float(a);
        for(int limb=0;limb<4;++limb)clip.contacts[i][limb]=weights[a][limb]+(weights[a+1][limb]-weights[a][limb])*alpha;
    }
    profile(result,slot,j);
}
HkxClip timelineSlice(const HkxClip& source,const Json& config) {
    const auto range=animationFrameRange(config,source.frames.size());
    HkxClip clip;
    clip.duration=source.duration*float(range[1]-range[0])/float(source.frames.size()-1);
    clip.identityMapping=source.identityMapping;clip.skeletonName=source.skeletonName;
    clip.boneIndices=source.boneIndices;clip.trackNames=source.trackNames;
    clip.frames.assign(source.frames.begin()+range[0],source.frames.begin()+range[1]+1);
    if(!source.rotations.empty()) {
        requirePack(source.rotations.size()==source.frames.size(),"Timeline rotation frames do not match transforms");
        clip.rotations.assign(source.rotations.begin()+range[0],source.rotations.begin()+range[1]+1);
    }
    if(config.contains("rootShift")) {
        const auto shift=vector(config.at("rootShift"),-10000,10000);
        const auto root=std::find(clip.boneIndices.begin(),clip.boneIndices.end(),0);
        requirePack(root!=clip.boneIndices.end(),"Timeline requires a Root track");
        const auto index=std::size_t(root-clip.boneIndices.begin());
        for(auto& frame:clip.frames){requirePack(index<frame.size(),"Timeline Root track is missing");frame[index].t=frame[index].t-shift;}
    }
    return clip;
}
}
AnimationPackReport loadAnimationPack(Library& library,const std::filesystem::path& manifest,AnimationOverrideLimits limits) {
    AnimationPackReport report;report.slots.resize(activeMotionCount);
    for(std::size_t i=0;i<activeMotions.size();++i)report.slots[i].motion=activeMotions[i];
    try {
        requirePack(limits.totalBytes>0&&limits.totalOutputBytes>0,"Animation pack limits are invalid");
        const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(limits.totalMilliseconds);
        const auto logicalManifest=std::filesystem::absolute(manifest).lexically_normal();
        const auto root=logicalManifest.parent_path();ordinaryPath(root);ordinaryPath(logicalManifest);
        const auto document=readJson(logicalManifest,65536,report.inputBytes,limits.totalBytes);format(document,"FreeClimbAnimationPack");
        Library staged;skeleton(staged,readJson(contained(root,document.at("skeleton")),256*1024,report.inputBytes,limits.totalBytes));
        const auto& motions=document.at("motions");requirePack(motions.is_array()&&motions.size()==activeMotionCount,"Pack must provide all 31 active animation slots");
        std::array<const Json*,motionCount> entries{};
        for(const auto& entry:motions) {
            const auto name=entry.at("slot").get<std::string>();const auto found=std::find(motionSlotNames.begin(),motionSlotNames.end(),name);
            requirePack(!name.empty()&&found!=motionSlotNames.end(),"Pack contains an unknown animation slot");
            const auto index=std::size_t(found-motionSlotNames.begin());requirePack(!entries[index],"Pack contains a duplicate animation slot");entries[index]=&entry;
        }
        std::map<std::filesystem::path,CachedHkx,LogicalPathLess> files;
        std::map<std::filesystem::path,Json,LogicalPathLess> configs;
        std::size_t decodedBytes=0;constexpr std::size_t decodedLimit=200*1024*1024;
        for(auto& slot:report.slots) {
            const auto i=std::size_t(int(slot.motion)-1);slot.file=std::string(motionSlotNames[i])+".hkx";
            try {
                requirePack(std::chrono::steady_clock::now()<deadline,"Animation pack load time budget exceeded");
                requirePack(entries[i]!=nullptr,"Pack is missing an active animation slot");
                const auto configPath=contained(root,entries[i]->at("config"));
                auto configFile=configs.find(configPath);
                if(configFile==configs.end())configFile=configs.emplace(configPath,readJson(configPath,2*1024*1024,report.inputBytes,limits.totalBytes)).first;
                const auto& config=selectAnimationClipConfig(configFile->second,motionSlotNames[i]);format(config,"FreeClimbClip");
                requirePack(config.at("slot").get<std::string>()==motionSlotNames[i],"Clip configuration is assigned to the wrong slot");
                slot.file=config.at("file").get<std::string>();const auto path=contained(root,config.at("file"));
                const auto utf8=path.u8string();slot.path.assign(utf8.begin(),utf8.end());
                std::string member;
                if(config.contains("member")) {
                    requirePack(config.at("member").is_string(),"HKX member selector must be a string");member=config.at("member").get<std::string>();
                    requirePack(!member.empty()&&member.size()<=64&&std::all_of(member.begin(),member.end(),[](unsigned char c){
                        return (c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='_';}),"HKX member selector is invalid");
                }
                auto [found,first]=files.try_emplace(path);auto& cached=found->second;
                if(first)try {
                    if(!std::filesystem::is_regular_file(path))cached.missing=true;
                    else {
                        const auto bytes=std::filesystem::file_size(path);
                        requirePack(bytes>=208&&bytes<=limits.fileBytes&&bytes<=limits.totalBytes-report.inputBytes,"HKX input or total size exceeds limit");
                        report.inputBytes+=std::size_t(bytes);std::vector<std::uint8_t> data(std::size_t(bytes),0);
                        std::ifstream file(path,std::ios::binary);requirePack(bool(file.read(reinterpret_cast<char*>(data.data()),std::streamsize(data.size())))&&
                            file.peek()==std::char_traits<char>::eof(),"HKX file changed while being read");
                        cached.bytes=data.size();cached.contentId=14695981039346656037ull;
                        for(const auto byte:data)cached.contentId=(cached.contentId^byte)*1099511628211ull;
                        std::string failure;const auto start=std::chrono::steady_clock::now();
                        requirePack(decodeHkxAnimationMembers(data,cached.members,failure,decodedLimit-decodedBytes),failure);
                        requirePack(std::chrono::steady_clock::now()-start<=std::chrono::milliseconds(limits.fileMilliseconds),"HKX decode time budget exceeded");
                        for(const auto& item:cached.members)decodedBytes+=item.second.frames.size()*(item.second.boneIndices.size()*(sizeof(Transform)+sizeof(Quat))+sizeof(Pose)+sizeof(std::vector<Quat>));
                    }
                } catch(const std::exception& e) {cached.members.clear();cached.failure=e.what();}
                slot.bytes=cached.bytes;slot.contentId=cached.contentId;
                if(cached.missing){slot.reason="HKX file is missing";++report.missing;continue;}
                requirePack(cached.failure.empty(),cached.failure);
                requirePack(!member.empty()||cached.members.size()==1,"HKX contains multiple animations; select a named member");
                const auto selected=member.empty()?cached.members.begin():std::find_if(cached.members.begin(),cached.members.end(),[&](const auto& item){return item.first==member;});
                requirePack(selected!=cached.members.end(),"HKX selected member is missing");
                const auto range=animationFrameRange(config,selected->second.frames.size());
                const auto samples=range[1]-range[0]+1,output=samples*(99*sizeof(Transform)+sizeof(std::array<float,4>));
                requirePack(samples<=limits.frames&&output<=limits.totalOutputBytes-report.outputBytes,"Animation output memory exceeds limit");
                std::optional<HkxClip> sliced;if(config.contains("frameRange"))sliced=timelineSlice(selected->second,config);
                const auto& clip=sliced?*sliced:selected->second;
                installClip(staged,i,config,clip);report.outputBytes+=output;
                if(config.contains("references")) {
                    const auto& references=config.at("references");requirePack(references.is_object()&&references.size()<=5,"Invalid private animation references");
                    constexpr std::array<std::string_view,5> roles{"launchApproach","kickTakeoff","kickLanding","kickRunLanding","kickRunBrace"};
                    const auto owner=Motion(i+1);
                    for(const auto& [key,metadata]:references.items()) {
                        const auto roleIterator=std::find(roles.begin(),roles.end(),key);
                        requirePack(roleIterator!=roles.end(),"Unknown private animation reference");const auto role=PlaybackReference(roleIterator-roles.begin());
                        requirePack(role==PlaybackReference::launchApproach?wallRunLaunch(owner):
                            (owner>=Motion::kickUp&&owner<=Motion::kickRight),"Private animation reference is assigned to the wrong action");
                        format(metadata,"FreeClimbClip");
                        requirePack(metadata.at("file")==config.at("file")&&!metadata.contains("references"),"Private animation reference must remain inside its owning action file");
                        const auto referenceMotion=Library::referenceFallback(owner,role);
                        requirePack(metadata.at("slot").get<std::string>()==motionSlotNames[int(referenceMotion)-1],"Private reference contains the wrong playback role");
                        const auto name=metadata.at("member").get<std::string>();
                        const auto binding=std::find_if(cached.members.begin(),cached.members.end(),[&](const auto& value){return value.first==name;});
                        requirePack(binding!=cached.members.end(),"Private animation reference binding is missing");
                        const auto referenceRange=animationFrameRange(metadata,binding->second.frames.size());
                        const auto referenceSamples=referenceRange[1]-referenceRange[0]+1,referenceBytes=referenceSamples*(99*sizeof(Transform)+sizeof(std::array<float,4>));
                        requirePack(referenceSamples<=limits.frames&&referenceBytes<=limits.totalOutputBytes-report.outputBytes,"Private animation reference exceeds output limits");
                        const auto reference=timelineSlice(binding->second,metadata);const auto target=int(referenceMotion)-1;
                        auto saved=std::move(staged.clips[target]);staged.clips[target]={};installClip(staged,target,metadata,reference);
                        staged.references[{owner,role}]=std::move(staged.clips[target]);staged.clips[target]=std::move(saved);report.outputBytes+=referenceBytes;
                    }
                }
                slot.status=OverrideStatus::loaded;slot.samples=clip.frames.size();slot.seconds=clip.duration;++report.loaded;
            } catch(const std::exception& e) {staged.clips[i]=Clip{};slot.status=OverrideStatus::rejected;slot.reason=e.what();++report.rejected;}
        }
        requirePack(report.loaded==activeMotionCount,"Animation pack rejected transactionally: all 31 active slots must load successfully");
        for(const auto& [configPath,group]:configs)if(group.value("format",std::string{})=="FreeClimbActionGroup"&&group.value("version",0)==2) {
            const bool contextHop=group.at("group")=="contextHop";
            constexpr std::array<std::string_view,5> directions{"runUp","runLeft","runRight","runDiagonalLeft","runDiagonalRight"};
            for(const auto& sequence:group.at("sequences")) {
                requirePack(std::chrono::steady_clock::now()<deadline,"Animation pack load time budget exceeded");
                const auto direction=sequence.at("slot").get<std::string>();
                const auto found=std::find(directions.begin(),directions.end(),direction);
                requirePack(contextHop?(direction=="contextHopLeft"||direction=="contextHopRight"):found!=directions.end(),"Unknown complete-action sequence direction");
                const auto index=contextHop?std::size_t(direction=="contextHopRight"):std::size_t(found-directions.begin());
                requirePack(!(contextHop?staged.contextHopSequenceValid[index]:staged.wallRunSequenceValid[index]),"Duplicate complete-action sequence definition");
                for(unsigned part=0;part<(sequence.contains("brace")?3u:2u);++part) {
                    const auto& config=sequence.at(part==2?"brace":part?"catch":contextHop?"prepare":"launch");format(config,"FreeClimbClip");
                    const auto file=files.find(contained(root,config.at("file")));
                    requirePack(file!=files.end()&&file->second.failure.empty()&&!file->second.missing,"Wall-run sequence file was not validated");
                    const auto member=config.at("member").get<std::string>();const auto& members=file->second.members;
                    const auto selected=std::find_if(members.begin(),members.end(),[&](const auto& item){return item.first==member;});
                    requirePack(selected!=members.end(),"Wall-run sequence member is missing");
                    const auto range=animationFrameRange(config,selected->second.frames.size());
                    const auto samples=range[1]-range[0]+1,output=samples*(99*sizeof(Transform)+sizeof(std::array<float,4>));
                    requirePack(samples<=limits.frames&&output<=limits.totalOutputBytes-report.outputBytes,"Wall-run sequence output memory exceeds limit");
                    const auto source=timelineSlice(selected->second,config);
                    const auto name=config.at("slot").get<std::string>();const auto stage=std::find(motionSlotNames.begin(),motionSlotNames.end(),name);
                    requirePack(stage!=motionSlotNames.end(),"Wall-run sequence stage is invalid");const auto slot=std::size_t(stage-motionSlotNames.begin());
                    auto saved=std::move(staged.clips[slot]);staged.clips[slot]={};installClip(staged,slot,config,source);
                    auto& target=contextHop?(part?staged.contextHopRecoveries[index]:staged.contextHopPreparations[index]):
                        (part==2?staged.wallRunBraces[index]:part?staged.wallRunCatches[index]:staged.wallRunLaunches[index]);
                    target=std::move(staged.clips[slot]);staged.clips[slot]=std::move(saved);
                    report.outputBytes+=output;
                }
                (contextHop?staged.contextHopSequenceValid[index]:staged.wallRunSequenceValid[index])=true;
            }
        }
        requirePack(validThreepeatProfile(staged.threepeatProfile),"Captured-action path and support phase profile is invalid");
        staged.calibrateArmBends();staged.animationPack=true;staged.sourceValidated=true;
        Settings calibration;requirePack(staged.configureThreepeat(calibration),"Captured-action geometry calibration is outside supported bounds");
        if(library.animationPack)for(std::size_t i=0;i<99;++i)requirePack((staged.rest[i].t-library.rest[i].t).length()<=.0001f&&
            (staged.rest[i].s-library.rest[i].s).length()<=.00001f&&angleBetween(staged.rest[i].q,library.rest[i].q)<=.00001f,
            "Reload cannot change the installed canonical skeleton reference");
        library=std::move(staged);report.committed=true;
    } catch(const std::exception& e) {report.error=e.what();}
    return report;
}
bool loadAnimationPackFile(Library& library,const std::string& path,std::string& error) {
    const auto report=loadAnimationPack(library,path);error=report.error;return report.committed;
}
}

