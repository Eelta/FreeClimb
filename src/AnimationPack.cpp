#include "AnimationPack.h"
#include "CanonicalSkeleton.h"
#include "HkxAnimation.h"
#include "../external/nlohmann/json.hpp"
#include <chrono>
#include <set>

namespace fc {
namespace {
using Json=nlohmann::json;
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
std::filesystem::path contained(const std::filesystem::path& root,const Json& value) {
    requirePack(value.is_string(),"Expected a relative file path");
    const auto text=value.get<std::string>();
    requirePack(!text.empty()&&text.size()<=240&&text.find(':')==std::string::npos&&text.find('\0')==std::string::npos,"Invalid animation file path");
    const std::filesystem::path child(text);
    requirePack(!child.is_absolute()&&!child.has_root_path(),"Animation paths must be relative");
    for(const auto& part:child)requirePack(part!="..","Animation paths cannot escape their pack");
    const auto absolute=std::filesystem::weakly_canonical(root/child);
    const auto relative=absolute.lexically_relative(root);
    requirePack(!relative.empty()&&!relative.is_absolute(),"Animation path is outside its pack");
    for(const auto& part:relative)requirePack(part!="..","Animation path is outside its pack");
    return absolute;
}
void format(const Json& j,const char* expected) {
    requirePack(j.at("format")==expected&&j.at("version")==1,"Unsupported animation metadata format or version");
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
}
AnimationPackReport loadAnimationPack(Library& library,const std::filesystem::path& manifest,AnimationOverrideLimits limits) {
    AnimationPackReport report;report.slots.resize(activeMotionCount);
    for(std::size_t i=0;i<activeMotions.size();++i)report.slots[i].motion=activeMotions[i];
    try {
        requirePack(limits.totalBytes>0&&limits.totalOutputBytes>0,"Animation pack limits are invalid");
        const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(limits.totalMilliseconds);
        const auto root=std::filesystem::weakly_canonical(manifest.parent_path());
        const auto document=readJson(manifest,65536,report.inputBytes,limits.totalBytes);format(document,"FreeClimbAnimationPack");
        Library staged;skeleton(staged,readJson(contained(root,document.at("skeleton")),256*1024,report.inputBytes,limits.totalBytes));
        const auto& motions=document.at("motions");requirePack(motions.is_array()&&motions.size()==activeMotionCount,"Pack must provide all 35 active animation slots");
        std::array<const Json*,motionCount> entries{};
        for(const auto& entry:motions) {
            const auto name=entry.at("slot").get<std::string>();const auto found=std::find(motionSlotNames.begin(),motionSlotNames.end(),name);
            requirePack(!name.empty()&&found!=motionSlotNames.end(),"Pack contains an unknown animation slot");
            const auto index=std::size_t(found-motionSlotNames.begin());requirePack(!entries[index],"Pack contains a duplicate animation slot");entries[index]=&entry;
        }
        for(auto& slot:report.slots) {
            const auto i=std::size_t(int(slot.motion)-1);slot.file=std::string(motionSlotNames[i])+".hkx";
            try {
                requirePack(std::chrono::steady_clock::now()<deadline,"Animation pack load time budget exceeded");
                requirePack(entries[i]!=nullptr,"Pack is missing an active animation slot");
                const auto config=readJson(contained(root,entries[i]->at("config")),256*1024,report.inputBytes,limits.totalBytes);format(config,"FreeClimbClip");
                requirePack(config.at("slot").get<std::string>()==motionSlotNames[i],"Clip configuration is assigned to the wrong slot");
                slot.file=config.at("file").get<std::string>();const auto path=contained(root,config.at("file"));
                if(!std::filesystem::is_regular_file(path)){slot.reason="HKX file is missing";++report.missing;continue;}
                const auto bytes=std::filesystem::file_size(path);
                requirePack(bytes>=208&&bytes<=limits.fileBytes&&bytes<=limits.totalBytes-report.inputBytes,"HKX input or total size exceeds limit");
                report.inputBytes+=std::size_t(bytes);std::vector<std::uint8_t> data(std::size_t(bytes),0);
                std::ifstream file(path,std::ios::binary);requirePack(bool(file.read(reinterpret_cast<char*>(data.data()),std::streamsize(data.size())))&&
                    file.peek()==std::char_traits<char>::eof(),"HKX file changed while being read");
                HkxClip clip;std::string failure;const auto start=std::chrono::steady_clock::now();
                requirePack(decodeHkxAnimation(data,clip,failure),failure);
                requirePack(std::chrono::steady_clock::now()-start<=std::chrono::milliseconds(limits.fileMilliseconds),"HKX decode time budget exceeded");
                const auto output=clip.frames.size()*(99*sizeof(Transform)+sizeof(std::array<float,4>));
                requirePack(clip.frames.size()<=limits.frames&&output<=limits.totalOutputBytes-report.outputBytes,"Animation output memory exceeds limit");
                installClip(staged,i,config,clip);report.outputBytes+=output;
                slot.status=OverrideStatus::loaded;slot.samples=clip.frames.size();slot.seconds=clip.duration;++report.loaded;
            } catch(const std::exception& e) {staged.clips[i]=Clip{};slot.status=OverrideStatus::rejected;slot.reason=e.what();++report.rejected;}
        }
        requirePack(report.loaded==activeMotionCount,"Animation pack rejected transactionally: all 35 active slots must load successfully");
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

