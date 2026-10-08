#include "Converter.h"
#include "ConverterEditor.h"
#include "ConverterSourceMotion.h"
#include "ConverterWallRunGroup.h"
#include "ConverterTimeline.h"
#include "../../src/AnimationPack.h"
#include "../../src/CanonicalSkeleton.h"
#include "../../src/HkxAnimation.h"
#include <chrono>
#include <cstring>
#include <fstream>
#include <map>
#include <set>
#include <system_error>
#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#endif

namespace fc {
namespace {
using Json=nlohmann::json;
using Bytes=std::vector<std::uint8_t>;
constexpr std::string_view prefix="meshes/actors/character/animations/FreeClimb/";
void check(bool valid,const std::string& message) {if(!valid)throw std::runtime_error(message);}
std::string pathText(const std::filesystem::path& p) {const auto text=p.u8string();return {reinterpret_cast<const char*>(text.data()),text.size()};}
std::filesystem::path utf8Path(std::string_view text) {return std::filesystem::path(std::u8string(reinterpret_cast<const char8_t*>(text.data()),text.size()));}
Bytes readFile(const std::filesystem::path& path,std::size_t limit) {
    check(std::filesystem::is_regular_file(path),"Input is not a regular file: "+pathText(path));
    const auto size=std::filesystem::file_size(path);check(size&&size<=limit,"Input is empty or exceeds its size limit");
    std::ifstream file(path,std::ios::binary);Bytes bytes(std::size_t(size),0);
    check(bool(file.read(reinterpret_cast<char*>(bytes.data()),std::streamsize(bytes.size())))&&file.peek()==std::char_traits<char>::eof(),"Input changed while being read");return bytes;
}
Json readJson(const Bytes& bytes) {
    std::vector<std::set<std::string>> keys;
    auto callback=[&](int depth,Json::parse_event_t event,Json& value) {
        check(depth<=32,"JSON nesting exceeds limit");
        if(event==Json::parse_event_t::object_start)keys.emplace_back();
        else if(event==Json::parse_event_t::key)check(!keys.empty()&&keys.back().insert(value.get<std::string>()).second,"Duplicate JSON key");
        else if(event==Json::parse_event_t::object_end)keys.pop_back();return true;
    };
    auto doc=Json::parse(bytes.begin(),bytes.end(),callback);check(doc.is_object(),"Expected a JSON object");return doc;
}
bool containedPath(const std::filesystem::path& root,const std::filesystem::path& path) {
    const auto canonicalRoot=root.lexically_normal(),canonicalPath=path.lexically_normal();auto child=canonicalPath.begin();
    for(auto parent=canonicalRoot.begin();parent!=canonicalRoot.end();++parent,++child) {
        if(child==canonicalPath.end())return false;
#ifdef _WIN32
        const auto a=parent->native(),b=child->native();if(CompareStringOrdinal(a.c_str(),int(a.size()),b.c_str(),int(b.size()),TRUE)!=CSTR_EQUAL)return false;
#else
        if(*parent!=*child)return false;
#endif
    }
    return true;
}
std::string relativeName(const Json& value) {
    check(value.is_string(),"Expected relative pack path");auto name=value.get<std::string>();
    check(!name.empty()&&name.size()<=240&&name.find(':')==std::string::npos&&name.find('\0')==std::string::npos,"Invalid pack file path");
    std::replace(name.begin(),name.end(),'\\','/');check(name.find("//")==std::string::npos,"Pack paths cannot contain repeated separators");const auto path=utf8Path(name);
    check(!path.is_absolute()&&!path.has_root_path(),"Pack path must be relative");
    for(const auto& part:path)check(part!=".."&&part!="."&&!part.empty()&&part.native().back()!='.'&&part.native().back()!=' ',"Unsafe pack path");
    const auto normalizedPath=path.generic_u8string();check(std::string(reinterpret_cast<const char*>(normalizedPath.data()),normalizedPath.size())==name,"Pack paths must use canonical relative spelling");return name;
}
struct CanonicalPathLess {
    bool operator()(const std::filesystem::path& a,const std::filesystem::path& b) const {
#ifdef _WIN32
        const auto& left=a.native();const auto& right=b.native();return CompareStringOrdinal(left.c_str(),int(left.size()),right.c_str(),int(right.size()),TRUE)==CSTR_LESS_THAN;
#else
        return a.native()<b.native();
#endif
    }
};
std::string lower(std::string value) {for(char& c:value)if(c>='A'&&c<='Z')c=char(c-'A'+'a');return value;}
const Bytes& snapshotBytes(const std::map<std::string,Bytes>& snapshot,const std::filesystem::path& root,const std::string& name) {
    auto found=snapshot.find(name);if(found!=snapshot.end())return found->second;
    found=std::find_if(snapshot.begin(),snapshot.end(),[&](const auto& item){return lower(item.first)==lower(name);});std::error_code error;
    check(found!=snapshot.end()&&std::filesystem::equivalent(root/utf8Path(name),root/utf8Path(found->first),error)&&!error,"Ambiguously cased animation path");return found->second;
}
std::string normalized(std::string_view name) {if(name.starts_with("x_"))name.remove_prefix(2);return std::string(name);}
bool boneAlias(std::string_view name,std::string_view canonical) {
    const auto source=normalized(name),target=normalized(canonical);
    if(source==target)return true;
    if((target=="Shield"&&source=="SHIELD")||(target=="Weapon"&&source=="WEAPON")||(target=="Quiver"&&source=="QUIVER"))return true;
    if(!target.starts_with("NPC L ")&&!target.starts_with("NPC R "))return false;
    const char side=target[4];const auto bracket=target.find('[');
    if(bracket==std::string::npos||target[bracket+1]!=side)return false;
    auto alias=target;alias.erase(bracket+1,1);alias.erase(4,2);alias+='.';alias+=side;return source==alias;
}
struct DataWriter {
    Bytes bytes;
    std::vector<std::pair<std::uint32_t,std::uint32_t>> links;
    std::vector<std::pair<std::uint32_t,std::uint32_t>> globalLinks;
    std::vector<std::pair<std::uint32_t,std::string>> objects;
    void align(std::size_t multiple=16,std::uint8_t fill=0) {while(bytes.size()%multiple)bytes.push_back(fill);}
    std::uint32_t allocate(std::size_t count) {align();const auto at=std::uint32_t(bytes.size());bytes.resize(bytes.size()+count);return at;}
    template<class T>void put(std::size_t at,T value) {check(at<=bytes.size()&&sizeof(T)<=bytes.size()-at,"Writer bounds exceeded");std::memcpy(bytes.data()+at,&value,sizeof(T));}
    std::uint32_t text(std::string_view s) {const auto at=std::uint32_t(bytes.size());bytes.insert(bytes.end(),s.begin(),s.end());bytes.push_back(0);return at;}
    void link(std::uint32_t from,std::uint32_t to) {links.emplace_back(from,to);}
    void classLink(std::uint32_t from,std::uint32_t to) {globalLinks.emplace_back(from,to);}
    void array(std::uint32_t from,std::uint32_t to,std::uint32_t count) {if(count)link(from,to);put(from+8,count);put(from+12,count|0x80000000u);}
    std::uint32_t object(std::size_t size,std::string name) {const auto at=allocate(size);objects.emplace_back(at,std::move(name));return at;}
};
void putVec(DataWriter& out,std::uint32_t at,Vec v) {out.put(at,v.x);out.put(at+4,v.y);out.put(at+8,v.z);}
Bytes packfile(DataWriter& data) {
    const std::map<std::string,std::uint32_t> signatures{{"hkRootLevelContainer",0x2772c11e},{"hkaAnimationContainer",0x8dc20333},{"hkaAnimationBinding",0x66eac971},{"hkaInterleavedUncompressedAnimation",0x930af031},{"hkaDefaultAnimatedReferenceFrame",0x6d85e445}};
    DataWriter names;std::map<std::string,std::uint32_t> offsets;
    for(const auto& [name,signature]:signatures) {
        const auto at=std::uint32_t(names.bytes.size());names.bytes.resize(at+5);names.put(at,signature);names.bytes[at+4]=9;offsets[name]=names.text(name);
    }
    names.align(16,0xff);data.align();const auto local=std::uint32_t(data.bytes.size());
    for(const auto [from,to]:data.links) {const auto at=data.bytes.size();data.bytes.resize(at+8);data.put(at,from);data.put(at+4,to);}
    data.align(16,0xff);const auto global=std::uint32_t(data.bytes.size());
    for(const auto [from,to]:data.globalLinks) {const auto at=data.bytes.size();data.bytes.resize(at+12);data.put(at,from);data.put(at+4,2u);data.put(at+8,to);}
    data.align(16,0xff);const auto virtuals=std::uint32_t(data.bytes.size());
    for(const auto& [at,name]:data.objects) {const auto p=data.bytes.size();data.bytes.resize(p+12);data.put(p,at);data.put(p+4,0u);data.put(p+8,offsets.at(name));}
    data.align(16,0xff);const auto end=std::uint32_t(data.bytes.size());DataWriter file;file.bytes.resize(208);
    file.put(0,0x57e0e057u);file.put(4,0x10c0c010u);file.put(8,0xffffffffu);file.put(12,8u);
    file.bytes[16]=8;file.bytes[17]=1;file.bytes[19]=1;file.put(20,3u);file.put(24,2u);file.put(28,0u);file.put(32,0u);file.put(36,offsets.at("hkRootLevelContainer"));
    std::memcpy(file.bytes.data()+40,"hk_2010.2.0-r1",14);file.put(60,std::uint16_t(0xffff));file.put(62,std::uint16_t(0xffff));
    const auto nameBegin=208u,dataBegin=208u+std::uint32_t(names.bytes.size());
    for(std::size_t i=0;i<3;++i) {
        const auto h=64+i*48;const char* tag=i==0?"__classnames__":i==1?"__types__":"__data__";std::memcpy(file.bytes.data()+h,tag,std::strlen(tag));
        file.bytes[h+19]=0xff;
        file.put(h+20,i==0?nameBegin:dataBegin);
        if(i==0)for(std::size_t k=24;k<=44;k+=4)file.put(h+k,std::uint32_t(names.bytes.size()));
        if(i==2) {file.put(h+24,local);file.put(h+28,global);file.put(h+32,virtuals);for(std::size_t k=36;k<=44;k+=4)file.put(h+k,end);}
    }
    file.bytes.insert(file.bytes.end(),names.bytes.begin(),names.bytes.end());file.bytes.insert(file.bytes.end(),data.bytes.begin(),data.bytes.end());return file.bytes;
}
std::uint32_t crc32(const Bytes& data) {
    std::uint32_t result=0xffffffffu;for(auto byte:data) {result^=byte;for(int n=0;n<8;++n)result=(result>>1)^((result&1u)?0xedb88320u:0u);}return ~result;
}
Bytes zip(const std::vector<std::pair<std::string,Bytes>>& files) {
    DataWriter out;struct Entry {std::string name;std::uint32_t crc,size,offset;};std::vector<Entry> entries;
    for(const auto& [name,data]:files) {
        const auto offset=std::uint32_t(out.bytes.size()),crc=crc32(data),size=std::uint32_t(data.size());const auto at=out.bytes.size();out.bytes.resize(at+30);
        out.put(at,0x04034b50u);out.put(at+4,std::uint16_t(20));out.put(at+6,std::uint16_t(0x800));out.put(at+14,crc);out.put(at+18,size);out.put(at+22,size);out.put(at+26,std::uint16_t(name.size()));
        out.bytes.insert(out.bytes.end(),name.begin(),name.end());out.bytes.insert(out.bytes.end(),data.begin(),data.end());entries.push_back({name,crc,size,offset});
    }
    const auto central=std::uint32_t(out.bytes.size());
    for(const auto& entry:entries) {
        const auto at=out.bytes.size();out.bytes.resize(at+46);out.put(at,0x02014b50u);out.put(at+4,std::uint16_t(20));out.put(at+6,std::uint16_t(20));out.put(at+8,std::uint16_t(0x800));
        out.put(at+16,entry.crc);out.put(at+20,entry.size);out.put(at+24,entry.size);out.put(at+28,std::uint16_t(entry.name.size()));out.put(at+42,entry.offset);out.bytes.insert(out.bytes.end(),entry.name.begin(),entry.name.end());
    }
    const auto length=std::uint32_t(out.bytes.size())-central;const auto at=out.bytes.size();out.bytes.resize(at+22);out.put(at,0x06054b50u);out.put(at+8,std::uint16_t(entries.size()));out.put(at+10,std::uint16_t(entries.size()));out.put(at+12,length);out.put(at+16,central);return out.bytes;
}
void writeFile(const std::filesystem::path& path,const Bytes& bytes) {
    std::filesystem::create_directories(path.parent_path());std::ofstream file(path,std::ios::binary|std::ios::trunc);
    check(bool(file.write(reinterpret_cast<const char*>(bytes.data()),std::streamsize(bytes.size()))),"Cannot write candidate file");file.close();check(bool(file),"Cannot close candidate file");
}
struct Temporary {
    std::filesystem::path path,owner;
    explicit Temporary(const std::filesystem::path& parent) {
        check(std::filesystem::is_directory(parent),"Work/output parent directory does not exist");
        owner=std::filesystem::weakly_canonical(parent);
        const auto stamp=std::chrono::high_resolution_clock::now().time_since_epoch().count();
        for(unsigned i=0;i<100;++i) {auto candidate=owner/(".freeclimb-convert-"+std::to_string(stamp)+"-"+std::to_string(i));if(std::filesystem::create_directory(candidate)){path=std::move(candidate);return;}}
        throw std::runtime_error("Cannot create unique work directory");
    }
    ~Temporary() {
        if(path.empty())return;std::error_code ec;const auto resolved=std::filesystem::weakly_canonical(path,ec);
        if(!ec&&resolved.is_absolute()&&resolved!=owner&&containedPath(owner,resolved)&&path.filename().native().starts_with(std::filesystem::path(".freeclimb-convert-").native()))std::filesystem::remove_all(path,ec);
    }
};
void publish(const std::filesystem::path& temporary,const std::filesystem::path& destination,bool overwrite) {
#ifdef _WIN32
    check(MoveFileExW(temporary.c_str(),destination.c_str(),MOVEFILE_WRITE_THROUGH|(overwrite?MOVEFILE_REPLACE_EXISTING:0))!=0,"Cannot publish ZIP; output exists, is locked or cannot be written");
#else
    check(overwrite||!std::filesystem::exists(destination),"Output exists; use --overwrite explicitly");std::filesystem::rename(temporary,destination);
#endif
}
}

Bytes writeCanonicalConverterHkx(const ConverterInputClip& canonical,const Library& library) {
    check(canonical.frames.size()>=2&&canonical.frames.size()<=1201&&library.names.size()==99&&std::all_of(canonical.frames.begin(),canonical.frames.end(),[](const auto& frame){return frame.size()==99;})&&std::all_of(canonical.annotations.begin(),canonical.annotations.end(),[](const auto& event){return event.track<99;}),"Canonical group stage dimensions are invalid");
    DataWriter out;const auto root=out.object(16,"hkRootLevelContainer"),container=out.object(96,"hkaAnimationContainer"),animation=out.object(88,"hkaInterleavedUncompressedAnimation"),binding=out.object(72,"hkaAnimationBinding");
    check(root==0,"Unexpected root offset");const auto variant=out.allocate(24);out.array(root,variant,1);out.link(variant,out.text("Merged Animation Container"));out.link(variant+8,out.text("hkaAnimationContainer"));out.classLink(variant+16,container);
    for(auto offset:{16u,64u,80u})out.array(container+offset,0,0);const auto animations=out.allocate(8),bindings=out.allocate(8);out.classLink(animations,animation);out.classLink(bindings,binding);out.array(container+32,animations,1);out.array(container+48,bindings,1);
    out.put(animation+16,1u);out.put(animation+20,canonical.duration);out.put(animation+24,99u);out.link(binding+16,out.text("NPC Root [Root]"));out.classLink(binding+24,animation);
    const auto indices=out.allocate(99*2);for(int i=0;i<99;++i)out.put(indices+i*2,std::int16_t(i));out.array(binding+32,indices,99);out.array(binding+48,0,0);out.array(animation+72,0,0);
    const auto annotations=out.allocate(99*24);out.array(animation+40,annotations,99);std::array<std::vector<const ConverterAnnotation*>,99> events;
    for(const auto& event:canonical.annotations)events[event.track].push_back(&event);
    for(int bone=0;bone<99;++bone) {
        const auto at=annotations+bone*24;out.link(at,out.text(library.names[bone]));const auto count=std::uint32_t(events[bone].size()),rows=count?out.allocate(count*16):0u;out.array(at+8,rows,count);
        for(std::size_t i=0;i<count;++i) {out.put(rows+i*16,events[bone][i]->time);out.link(rows+std::uint32_t(i*16)+8,out.text(events[bone][i]->text));}
    }
    const auto transforms=out.allocate(canonical.frames.size()*99*48);out.array(animation+56,transforms,std::uint32_t(canonical.frames.size()*99));
    for(std::size_t frame=0;frame<canonical.frames.size();++frame)for(int bone=0;bone<99;++bone) {
        const auto& value=canonical.frames[frame][bone];
        const auto at=transforms+std::uint32_t((frame*99+bone)*48);putVec(out,at,value.t);out.put(at+16,value.q.x);out.put(at+20,value.q.y);out.put(at+24,value.q.z);out.put(at+28,value.q.w);putVec(out,at+32,value.s);
    }
    if(!canonical.referenceFrame.samples.empty()) {
        const auto reference=out.object(80,"hkaDefaultAnimatedReferenceFrame");out.classLink(animation+32,reference);putVec(out,reference+16,canonical.referenceFrame.up);putVec(out,reference+32,canonical.referenceFrame.forward);out.put(reference+48,canonical.referenceFrame.duration);
        const auto samples=out.allocate(canonical.referenceFrame.samples.size()*16);out.array(reference+56,samples,std::uint32_t(canonical.referenceFrame.samples.size()));
        for(std::size_t i=0;i<canonical.referenceFrame.samples.size();++i)for(int k=0;k<4;++k)out.put(samples+std::uint32_t(i*16+k*4),canonical.referenceFrame.samples[i][k]);

    }
    auto bytes=packfile(out);HkxClip verified;std::string error;check(decodeHkxAnimation(bytes,verified,error),"Generated HKX failed the formal decoder: "+error);
    check(verified.duration==canonical.duration&&verified.frames.size()==canonical.frames.size()&&verified.boneIndices.size()==99,"Generated HKX round-trip metadata mismatch");return bytes;
}

std::vector<std::uint8_t> writeConverterHkx(const ConverterInputClip& source,const Library& library,std::vector<std::string>& warnings) {
    const auto tracks=source.boneIndices.size();
    check(source.skeletonName=="NPC Root [Root]"&&source.frames.size()>=2&&source.frames.size()<=1201&&
        tracks>0&&tracks<=256&&source.trackNames.size()==tracks&&library.rest.size()==99&&library.names.size()==99,
        "Invalid converter source or canonical skeleton");
    check(std::all_of(source.frames.begin(),source.frames.end(),[&](const auto& frame){return frame.size()==tracks;}),"Input transform track count is inconsistent");
    const auto& annotationNames=source.annotationTrackNames;
    bool trailingCameraNames=source.identityMapping&&tracks==97&&annotationNames.size()==99;
    if(trailingCameraNames)for(std::size_t bone=0;bone<99;++bone)if(!boneAlias(annotationNames[bone],library.names[bone])){trailingCameraNames=false;break;}
    const bool named=trailingCameraNames||(!source.partialAnnotationNames&&std::all_of(source.trackNames.begin(),source.trackNames.end(),[](const auto& name){return !name.empty();}));
    check(!source.identityMapping||named||tracks==97||tracks==99||tracks==126||tracks==156,
        "Unknown unnamed identity layout; use standard 97/99/126/156 tracks, an explicit standard binding map, or complete bone names");
    std::array<int,99> mapping;mapping.fill(-1);std::array<bool,256> seenIndices{};bool labels=false;std::size_t ignored=0;
    for(std::size_t track=0;track<tracks;++track) {
        const auto bone=source.boneIndices[track];check(bone>=0&&bone<256&&!seenIndices[bone],"Input binding map contains an invalid or duplicate bone index");seenIndices[bone]=true;
        const auto& name=trailingCameraNames?annotationNames[track]:source.trackNames[track];int target=-1;
        if(!name.empty())for(std::size_t i=0;i<99;++i)if(boneAlias(name,library.names[i])){target=int(i);break;}
        if(named) {
            check(source.identityMapping||target<0||target==bone,"Named bone disagrees with explicit standard binding index; custom retargeting is unsupported");
        } else {
            check(target<0||target==bone,"Partial bone labels disagree with standard identity or explicit binding indices");
            labels|=!name.empty()&&target<0;
            target=bone<99?bone:-1;
        }
        if(target>=0){check(mapping[target]<0,"Two input tracks map to the same canonical bone");mapping[target]=int(track);}else ++ignored;
    }
    if(named&&ignored)check(std::all_of(mapping.begin(),mapping.begin()+97,[](int track){return track>=0;}),
        "Unrecognized bone labels in an incomplete humanoid layout; provide complete standard body names instead of event labels");
    const auto animated=std::count_if(mapping.begin()+5,mapping.begin()+97,[](int track){return track>=0;});check(animated>0,"No supported body animation tracks");

    if(source.identityMapping&&!named)warnings.push_back("Unnamed identity tracks use the known Skyrim "+std::to_string(tracks)+"-track humanoid convention. An unnamed file cannot prove its skeleton layout; custom hierarchies are not retargeted.");
    if(named&&source.identityMapping)warnings.push_back("Complete bone names determine the track mapping, including reordered tracks. Local bone axes and hierarchy must already match Skyrim; this is not skeleton retargeting.");
    if(trailingCameraNames)warnings.push_back("The complete 99-name standard annotation layout identifies 97 body tracks; its two trailing camera labels do not represent transform tracks.");
    if(labels)warnings.push_back("Unidentified partial annotation-track labels were treated as event labels, not bone identities. Standard binding indices determine the pose.");
    if(source.partialAnnotationNames)warnings.push_back("Partial annotation-track names were treated as event labels; standard bone indices determine the pose. Their events were retained on the canonical root track.");
    if(source.ignoredFloatTracks)warnings.push_back(std::to_string(source.ignoredFloatTracks)+" float animation tracks were not exported. FreeClimb does not use Havok float channels.");
    const auto missing=std::count(mapping.begin(),mapping.end(),-1);if(missing)warnings.push_back(std::to_string(missing)+" absent canonical tracks were filled with the reference pose.");
    if(ignored)warnings.push_back(std::to_string(ignored)+" extension tracks outside the 99-bone FreeClimb rig were not applied.");
    inspectConverterSourceMotion(source);
    ConverterInputClip canonical=source;canonical.boneIndices.resize(99);canonical.trackNames=library.names;canonical.annotationTrackNames=library.names;canonical.identityMapping=true;canonical.partialAnnotationNames=false;
    bool translation=false,scale=false,protectedRotation=false;
    for(std::size_t frame=0;frame<source.frames.size();++frame){canonical.frames[frame]=library.rest;for(int bone=0;bone<99;++bone){
        canonical.boneIndices[bone]=bone;auto& value=canonical.frames[frame][bone];if(mapping[bone]<0)continue;const auto& authored=source.frames[frame][mapping[bone]];value.q=authored.q.unit();
        if(bone==0||bone==4)value.t=authored.t;else translation|=(authored.t-value.t).length()>.02f;scale|=(authored.s-value.s).length()>.0002f;
        if((bone>=1&&bone<=3)||bone>=97){protectedRotation|=angleBetween(value.q,library.rest[bone].q)>.002f;value.q=library.rest[bone].q;}
    }}
    std::vector<int> inverse(source.boneIndices.size(),-1);for(int bone=0;bone<99;++bone)if(mapping[bone]>=0)inverse[mapping[bone]]=bone;
    for(auto& event:canonical.annotations){check(source.partialAnnotationNames||event.track<inverse.size(),"Annotation track is invalid");const auto bone=source.partialAnnotationNames?0:inverse[event.track];event.track=std::size_t(std::max(bone,0));}
    const auto movement=bakeConverterSourceMotion(canonical);for(const auto& warning:movement.warnings)if(std::find(warnings.begin(),warnings.end(),warning)==warnings.end())warnings.push_back(warning);
    if(movement.present&&!movement.alreadyBaked)warnings.push_back("Source AMR/extracted root movement was baked once into the animation Root for preview and FreeClimb playback.");
    if(translation)warnings.push_back("Animated joint translations were normalized to canonical bone lengths; Root and COM translations were retained.");
    if(scale)warnings.push_back("Animated bone scales were replaced by canonical reference scales, as required by FreeClimb.");
    if(protectedRotation)warnings.push_back("Look/control/camera rotations were reset to the protected reference pose.");
    if(!canonical.annotations.empty())warnings.push_back("HKX annotations were retained. FreeClimb does not dispatch their native behavior events; extension-track events were moved to the root track.");
    return writeCanonicalConverterHkx(canonical,library);
}

std::vector<std::uint8_t> bundleConverterHkx(const std::vector<std::pair<std::string,Bytes>>& members) {
    check(!members.empty()&&members.size()<=35,"An animation group requires 1..35 members");
    DataWriter out;const auto root=out.object(16,"hkRootLevelContainer"),container=out.object(96,"hkaAnimationContainer");
    const auto variants=out.allocate((members.size()+1)*24),animations=out.allocate(members.size()*8),bindings=out.allocate(members.size()*8);
    out.array(root,variants,std::uint32_t(members.size()+1));out.link(variants,out.text("Merged Animation Container"));out.link(variants+8,out.text("hkaAnimationContainer"));out.classLink(variants+16,container);
    out.array(container+32,animations,std::uint32_t(members.size()));out.array(container+48,bindings,std::uint32_t(members.size()));
    std::set<std::string> names;std::size_t total=0;
    for(std::size_t member=0;member<members.size();++member) {
        const auto& [name,bytes]=members[member];
        check(!name.empty()&&name.size()<=64&&std::all_of(name.begin(),name.end(),[](unsigned char c){return (c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='_';})&&names.insert(name).second,"Invalid or duplicate animation member name");
        check(bytes.size()<=64*1024*1024-total,"Combined animation input exceeds 64 MiB");total+=bytes.size();
        HkxClip decoded;std::string error;check(decodeHkxAnimation(bytes,decoded,error),"Group member must be a supported single animation: "+error);
        auto word=[&](std::size_t at){check(at<=bytes.size()&&4<=bytes.size()-at,"Group source bounds exceeded");std::uint32_t value;std::memcpy(&value,bytes.data()+at,4);return value;};
        check(word(24)==2,"Group source data section is unsupported");
        const auto begin=word(180),local=word(184),global=word(188),virtuals=word(192),end=word(196);
        check(begin<=bytes.size()&&end<=bytes.size()-begin,"Group source section exceeds input");
        std::map<std::uint32_t,std::uint32_t> pointers;std::map<std::uint32_t,std::string> classes;
        const auto offset=out.allocate(local);std::copy_n(bytes.begin()+begin,local,out.bytes.begin()+offset);
        for(std::uint32_t at=local;at+8<=global;at+=8) {
            const auto from=word(begin+at);if(from==0xffffffffu)continue;const auto to=word(begin+at+4);
            check(from<=local&&8<=local-from&&to<local&&pointers.emplace(from,to).second,"Invalid source local pointer");out.link(offset+from,offset+to);
        }
        for(std::uint32_t at=global;at+12<=virtuals;at+=12) {
            const auto from=word(begin+at);if(from==0xffffffffu)continue;const auto section=word(begin+at+4),to=word(begin+at+8);
            check(section==2&&from<=local&&8<=local-from&&to<local&&pointers.emplace(from,to).second,"Invalid source global pointer");out.classLink(offset+from,offset+to);
        }
        for(std::uint32_t at=virtuals;at+12<=end;at+=12) {
            const auto from=word(begin+at);if(from==0xffffffffu)continue;const auto section=word(begin+at+4),to=word(begin+at+8);
            check(section<3&&from<local,"Invalid source object reference");const auto textAt=std::size_t(word(64+section*48+20))+to;
            check(textAt<bytes.size(),"Invalid source class name");std::string type;
            for(std::size_t p=textAt;p<bytes.size()&&bytes[p]&&type.size()<128;++p)type.push_back(char(bytes[p]));
            check(!type.empty()&&type.size()<128&&textAt+type.size()<bytes.size()&&!bytes[textAt+type.size()],"Invalid source class name");
            check(type=="hkRootLevelContainer"||type=="hkaAnimationContainer"||type=="hkaAnimationBinding"||type=="hkaInterleavedUncompressedAnimation"||type=="hkaDefaultAnimatedReferenceFrame","Group writer requires canonical interleaved clips; import the source animation first");
            classes.emplace(from,type);out.objects.emplace_back(offset+from,std::move(type));
        }
        auto pointer=[&](std::uint32_t at){const auto found=pointers.find(at);check(found!=pointers.end(),"Missing source group pointer");return found->second;};
        const auto sourceRoot=word(28),sourceVariants=pointer(sourceRoot);std::uint32_t sourceContainer=0;bool foundContainer=false;
        const auto count=word(begin+sourceRoot+8);
        for(std::uint32_t i=0;i<count;++i){const auto target=pointer(sourceVariants+i*24+16);if(classes.contains(target)&&classes.at(target)=="hkaAnimationContainer"){sourceContainer=target;foundContainer=true;}}
        check(foundContainer,"Source animation container is missing");const auto animation=pointer(pointer(sourceContainer+32)),binding=pointer(pointer(sourceContainer+48));
        out.classLink(animations+std::uint32_t(member*8),offset+animation);out.classLink(bindings+std::uint32_t(member*8),offset+binding);
        const auto variant=variants+std::uint32_t((member+1)*24);out.link(variant,out.text(name));out.link(variant+8,out.text("hkaAnimationBinding"));out.classLink(variant+16,offset+binding);
    }
    auto result=packfile(out);check(result.size()<=64*1024*1024,"Combined animation exceeds 64 MiB");return result;
}

static void appendReferences(Json& config,const Library& library,std::vector<std::pair<std::string,Bytes>>& members){
    if(!config.contains("references"))return;const auto ownerName=config.at("slot").get<std::string>();const auto owner=Motion(std::find(motionSlotNames.begin(),motionSlotNames.end(),ownerName)-motionSlotNames.begin()+1);
    for(auto& [name,metadata]:config["references"].items()){
        const auto role=name=="launchApproach"?PlaybackReference::launchApproach:name=="kickTakeoff"?PlaybackReference::kickTakeoff:name=="kickLanding"?PlaybackReference::kickLanding:name=="kickRunLanding"?PlaybackReference::kickRunLanding:name=="kickRunBrace"?PlaybackReference::kickRunBrace:PlaybackReference::count;
        check(role!=PlaybackReference::count&&library.hasReference(owner,role),"Private action reference is unavailable");const auto& source=library.reference(owner,role);ConverterInputClip clip;clip.duration=source.seconds;clip.frames=source.frames;clip.trackNames=library.names;clip.boneIndices.resize(99);for(int i=0;i<99;++i)clip.boneIndices[i]=i;
        const auto member=ownerName+"_"+name;metadata["member"]=member;metadata.erase("frameRange");metadata.erase("rootShift");members.emplace_back(member,writeCanonicalConverterHkx(clip,library));
    }
}
static void assignClipFile(Json& config,const std::string& file){config["file"]=file;if(config.contains("references"))for(auto& [name,reference]:config["references"].items())reference["file"]=file;}

ConverterWallRunPack composeConverterWallRunDirection(const Library& library,std::string_view direction,const std::vector<ConverterPreparedClip>& stages) {
    check(converter::isWallRunPrimary(direction)&&(stages.size()==3||stages.size()==4),"One wall run requires its start, loop and ending with an optional private brace");
    const auto roles=converter::wallRunStages(direction);std::vector<ConverterTimelinePart> parts;std::vector<Json> metadata;
    ConverterWallRunPack result;result.config={{"format","FreeClimbActionGroup"},{"version",2},{"group","wallRun"},{"direction",direction},{"clips",Json::array()},{"sequences",Json::array()}};
    for(std::size_t i=0;i<3;++i) {
        const auto found=std::find_if(stages.begin(),stages.end(),[&](const auto& stage){return stage.config.at("slot").get<std::string>()==roles[i];});
        check(found!=stages.end()&&(found->direction.empty()||found->direction==direction),"Direction animation has a missing or unrelated section");
        parts.push_back({std::string(roles[i]),found->clip});metadata.push_back(found->config);result.warnings.insert(result.warnings.end(),found->warnings.begin(),found->warnings.end());
    }
    const auto timeline=composeConverterTimeline(parts);result.warnings.insert(result.warnings.end(),timeline.warnings.begin(),timeline.warnings.end());
    for(std::size_t i=0;i<3;++i) {
        const auto& range=timeline.ranges[i];metadata[i]["member"]=direction;metadata[i]["frameRange"]={range.frames[0],range.frames[1]};metadata[i]["rootShift"]={range.rootShift.x,range.rootShift.y,range.rootShift.z};
        result.config["clips"].push_back(metadata[i]);
    }
    std::vector<std::pair<std::string,Bytes>> members{{std::string(direction),writeCanonicalConverterHkx(timeline.clip,library)}};
    for(unsigned index=0;index<3;++index){appendReferences(metadata[index],library,members);result.config["clips"][index]=metadata[index];}
    result.config["sequences"].push_back({{"slot",direction},{"launch",metadata[0]},{"catch",metadata[2]}});
    if(stages.size()==4){check(direction!="runUp"&&stages[3].config.at("slot")=="sideBrace","Only side wall runs own a brace reference");auto brace=stages[3].config;brace["member"]=std::string(direction)+"Brace";brace.erase("frameRange");brace.erase("rootShift");result.config["sequences"][0]["brace"]=brace;if(direction=="runLeft")result.config["clips"].push_back(brace);members.emplace_back(std::string(direction)+"Brace",writeCanonicalConverterHkx(stages[3].clip,library));}
    result.hkx=bundleConverterHkx(members);return result;
}

ConverterWallRunPack composeConverterContextHopDirection(const Library& library,std::string_view direction,const std::vector<ConverterPreparedClip>& stages) {
    check(converter::isContextHop(direction)&&stages.size()==3,"One contextual leap requires its own preparation, leap and recovery");
    const std::array<std::string_view,3> roles{"prepare",direction,"catch"};std::vector<ConverterTimelinePart> parts;std::array<Json,3> metadata;
    ConverterWallRunPack result;result.config={{"format","FreeClimbActionGroup"},{"version",2},{"group","contextHop"},{"direction",direction}};
    for(std::size_t i=0;i<3;++i){check(stages[i].config.at("slot")==std::string(i==1?direction:"contextHang"),"Contextual leap contains an unrelated stage");parts.push_back({std::string(roles[i]),stages[i].clip});metadata[i]=stages[i].config;result.warnings.insert(result.warnings.end(),stages[i].warnings.begin(),stages[i].warnings.end());}
    const auto timeline=composeConverterTimeline(parts);result.warnings.insert(result.warnings.end(),timeline.warnings.begin(),timeline.warnings.end());
    for(std::size_t i=0;i<3;++i){const auto& range=timeline.ranges[i];metadata[i]["member"]=direction;metadata[i]["frameRange"]={range.frames[0],range.frames[1]};metadata[i]["rootShift"]={range.rootShift.x,range.rootShift.y,range.rootShift.z};}
    result.config["clips"]={metadata[0],metadata[1]};result.config["sequences"]={{{"slot",direction},{"prepare",metadata[0]},{"catch",metadata[2]}}};
    result.hkx=bundleConverterHkx({{std::string(direction),writeCanonicalConverterHkx(timeline.clip,library)}});return result;
}

ConverterWallRunPack composeConverterWallRunPack(const Library& library,const std::vector<ConverterPreparedClip>& stages) {
    check(stages.size()==16,"Complete wall-running pack requires five launch/cycle/catch sequences and one brace reference");
    std::map<std::string,const ConverterPreparedClip*> selected;
    for(const auto& stage:stages){const auto slot=stage.config.at("slot").get<std::string>();const auto key=stage.direction.empty()?slot:stage.direction+"/"+slot;check(selected.emplace(key,&stage).second,"Duplicate wall-running sequence stage");}
    ConverterWallRunPack result;result.config={{"format","FreeClimbActionGroup"},{"version",2},{"group","wallRun"},{"clips",Json::array()},{"sequences",Json::array()}};
    std::vector<std::pair<std::string,Bytes>> members;std::map<std::string,Json> defaults;
    for(const auto direction:{"runUp","runLeft","runRight","runDiagonalLeft","runDiagonalRight"}){
        const auto roles=converter::wallRunStages(direction);std::vector<ConverterTimelinePart> parts;std::vector<Json> metadata;
        for(const auto role:std::span(roles.data(),3)){const auto found=selected.find(std::string(direction)+"/"+std::string(role));check(found!=selected.end(),"Missing wall-running sequence stage");const auto& stage=*found->second;parts.push_back({std::string(role),stage.clip});metadata.push_back(stage.config);metadata.back().erase("references");result.warnings.insert(result.warnings.end(),stage.warnings.begin(),stage.warnings.end());}
        const auto timeline=composeConverterTimeline(parts);result.warnings.insert(result.warnings.end(),timeline.warnings.begin(),timeline.warnings.end());members.emplace_back(direction,writeCanonicalConverterHkx(timeline.clip,library));
        for(std::size_t i=0;i<3;++i){const auto& range=timeline.ranges[i];metadata[i]["member"]=direction;metadata[i]["frameRange"]={range.frames[0],range.frames[1]};metadata[i]["rootShift"]={range.rootShift.x,range.rootShift.y,range.rootShift.z};}
        defaults.emplace(direction,metadata[1]);if(std::string_view(direction)=="runUp"){defaults.emplace("runLaunch",metadata[0]);defaults.emplace("runCatch",metadata[2]);}
        if(std::string_view(direction)=="runLeft")defaults.emplace("runLaunchLeft",metadata[0]);if(std::string_view(direction)=="runRight")defaults.emplace("runLaunchRight",metadata[0]);
        result.config["sequences"].push_back({{"slot",direction},{"launch",metadata[0]},{"catch",metadata[2]}});
    }
    const auto brace=selected.find("sideBrace");check(brace!=selected.end(),"Missing local brace reference");auto braceConfig=brace->second->config;braceConfig["member"]="sideBrace";braceConfig.erase("frameRange");braceConfig.erase("rootShift");
    members.emplace_back("sideBrace",writeCanonicalConverterHkx(brace->second->clip,library));defaults.emplace("sideBrace",std::move(braceConfig));
    for(const auto motion:actionGroupMotions("wallRun"))result.config["clips"].push_back(defaults.at(std::string(motionSlotNames[int(motion)-1])));
    result.hkx=bundleConverterHkx(members);return result;
}

static nlohmann::json convertPrepared(const ConverterRequest& request,const ConverterPreparedClip* prepared) {
    if(prepared&&(!actionGroupMotions(converter::actionPrimaryForSlot(request.slot)).empty()||converter::isContextHop(request.slot)||request.slot=="contextHang")) {
        const auto manifest=readJson(readFile(request.pack,65536));
        const auto entry=std::find_if(manifest.at("motions").begin(),manifest.at("motions").end(),[&](const auto& value){return value.at("slot")==request.slot;});
        check(entry!=manifest.at("motions").end(),"Selected slot is missing from pack");
        const auto metadata=readJson(readFile(request.pack.parent_path()/relativeName(entry->at("config")),2*1024*1024));
        if(metadata.value("format",std::string{})=="FreeClimbActionGroup")return convertHkxGroup(request,{*prepared});
    }
    const auto input=std::filesystem::weakly_canonical(request.input),pack=std::filesystem::weakly_canonical(request.pack),root=pack.parent_path(),output=std::filesystem::absolute(request.output).lexically_normal();
    check(lower(pathText(pack.filename()))=="pack.json","Select a complete FreeClimb pack.json");check(lower(pathText(input.extension()))==".hkx","Input must be an HKX file");check(lower(pathText(output.extension()))==".zip","Output must be a ZIP file");
    check(!containedPath(root,std::filesystem::weakly_canonical(output))&&output!=input,"Output cannot overwrite input or live inside the base pack");check(std::filesystem::is_directory(output.parent_path()),"Output directory does not exist");
    check(request.overwrite||!std::filesystem::exists(output),"Output already exists; use --overwrite explicitly");
    if(prepared)for(const auto& snapshot:prepared->snapshots)check(readFile(snapshot.path,64*1024*1024)==snapshot.bytes,"Editor input or base pack changed; reload it before export");
    const auto slot=std::find(motionSlotNames.begin(),motionSlotNames.end(),request.slot);check(!request.slot.empty()&&slot!=motionSlotNames.end()&&isActiveMotion(Motion(int(slot-motionSlotNames.begin())+1)),"Unknown active animation slot");
    auto libraryStorage=std::make_unique<Library>();auto& library=*libraryStorage;const auto initial=loadAnimationPack(library,pack);check(initial.committed,"Base animation pack failed validation: "+initial.error);
    std::map<std::string,Bytes> snapshot;std::map<std::filesystem::path,std::size_t,CanonicalPathLess> uses;std::set<std::string> names;std::size_t total=0;
    auto add=[&](const std::string& name,bool sharedHkx=false) {
        const auto path=std::filesystem::weakly_canonical(root/utf8Path(name));check(containedPath(root,path),"Pack symlink escapes its directory");
        if(!names.insert(lower(name)).second){if(sharedHkx&&!snapshot.contains(name)){const auto previous=std::find_if(snapshot.begin(),snapshot.end(),[&](const auto& item){return lower(item.first)==lower(name);});check(previous!=snapshot.end()&&std::filesystem::equivalent(path,root/utf8Path(previous->first)),"Ambiguously cased animation path");}else check((sharedHkx||lower(pathText(path.extension()))==".json")&&snapshot.contains(name),"Duplicate or ambiguously cased pack path");return;}
        auto data=readFile(path,lower(pathText(path.extension()))==".hkx"?64*1024*1024:256*1024);check(data.size()<=256*1024*1024-total,"Base pack exceeds 256 MiB");total+=data.size();snapshot[name]=std::move(data);
    };
    add("pack.json");auto manifest=readJson(snapshot.at("pack.json"));check(manifest.at("format")=="FreeClimbAnimationPack"&&manifest.at("version")==1,"Unsupported base manifest");add(relativeName(manifest.at("skeleton")));
    std::string configName,hkxName;Json config;
    for(const auto& entry:manifest.at("motions")) {
        const auto name=relativeName(entry.at("config"));add(name);const auto metadata=selectAnimationClipConfig(readJson(snapshot.at(name)),entry.at("slot").get<std::string>());const auto hkx=relativeName(metadata.at("file"));add(hkx,true);++uses[std::filesystem::weakly_canonical(root/utf8Path(hkx))];
        if(entry.at("slot")==request.slot){configName=name;hkxName=hkx;config=metadata;}
    }
    check(!configName.empty(),"Selected slot is missing from pack");const auto sourceBytes=readFile(input,64*1024*1024);ConverterInputClip source;std::string error;
    if(prepared) {
        check(prepared->config.is_object()&&prepared->config.at("file")==config.at("file")&&prepared->config.at("slot")==request.slot&&
            prepared->config.at("format")==config.at("format")&&(prepared->config.at("version")==config.at("version")||(prepared->config.at("version")==2&&prepared->config.contains("authoredPlayback"))),"Edited metadata must retain the selected slot and file identity");
        source=prepared->clip;config=prepared->config;
    } else check(decodeConverterInput(sourceBytes,source,error),error);
    config.erase("member");config.erase("frameRange");config.erase("rootShift");std::vector<std::string> warnings=prepared?prepared->warnings:std::vector<std::string>{};auto hkx=writeConverterHkx(source,library,warnings);
    if(config.contains("references")){config["member"]=request.slot;std::vector<std::pair<std::string,Bytes>> members{{request.slot,std::move(hkx)}};appendReferences(config,library,members);hkx=bundleConverterHkx(members);}
    if(!prepared)warnings.push_back("Target-slot contact timing, movement and special-action phases were reused. Check hand/foot grab-release timing in-game before publishing.");
    bool separate=false;
    if(uses.at(std::filesystem::weakly_canonical(root/utf8Path(hkxName)))>1) {
        for(unsigned index=0;index<100;++index) {
            const auto name="converted/"+request.slot+(index?"-"+std::to_string(index):"")+".hkx";
            if(!names.contains(lower(name))&&!std::filesystem::exists(root/utf8Path(name))){hkxName=name;config["file"]=hkxName;separate=true;break;}
        }
        check(separate,"Cannot allocate a unique animation filename for the shared slot");warnings.push_back("The base HKX is shared by multiple slots. A separate selected-slot HKX was created so other slots remain unchanged.");
    }
    assignClipFile(config,hkxName);const auto text=config.dump(2)+"\n";const Bytes configBytes(text.begin(),text.end());const auto work=request.work.empty()?output.parent_path():std::filesystem::weakly_canonical(request.work);
    check(!containedPath(root,work),"Work directory cannot be inside the base pack");Temporary stage(work);
    for(const auto& [name,data]:snapshot)writeFile(stage.path/utf8Path(name),name==hkxName?hkx:name==configName?configBytes:data);
    if(separate)writeFile(stage.path/utf8Path(hkxName),hkx);
    auto candidateStorage=std::make_unique<Library>();auto& candidate=*candidateStorage;const auto validation=loadAnimationPack(candidate,stage.path/"pack.json");
    if(!validation.committed) {std::string message=validation.error;for(const auto& result:validation.slots)if(result.status==OverrideStatus::rejected)message+="; "+result.file+": "+result.reason;throw std::runtime_error("Converted pack failed validation: "+message);}
    for(const auto& [name,data]:snapshot)check(readFile(root/utf8Path(name),64*1024*1024)==data,"Base pack changed during conversion; nothing published");check(readFile(input,64*1024*1024)==sourceBytes,"Input changed during conversion; nothing published");
    if(prepared)for(const auto& saved:prepared->snapshots)check(readFile(saved.path,64*1024*1024)==saved.bytes,"Editor input or base pack changed during export; nothing published");
    const auto archive=zip({{std::string(prefix)+hkxName,hkx},{std::string(prefix)+configName,configBytes}});Temporary publishDirectory(output.parent_path());const auto temporary=publishDirectory.path/"output.zip";writeFile(temporary,archive);publish(temporary,output,request.overwrite);
    std::vector<std::string> uniqueWarnings;for(auto& warning:warnings)if(std::find(uniqueWarnings.begin(),uniqueWarnings.end(),warning)==uniqueWarnings.end())uniqueWarnings.push_back(std::move(warning));warnings=std::move(uniqueWarnings);
    return {{"ok",true},{"output",pathText(output)},{"slot",request.slot},{"duration",source.duration},{"frames",source.frames.size()},{"warnings",warnings},{"error",""},{"files",{std::string(prefix)+hkxName,std::string(prefix)+configName}},{"loaded",validation.loaded}};
}
nlohmann::json convertHkx(const ConverterRequest& request) {
    const auto doc=loadConverterEditor(request.input,request.pack,request.slot);const auto edited=applyConverterEdits(doc,{});
    const ConverterPreparedClip prepared{edited.clip,edited.config,edited.warnings,doc.snapshots,doc.direction,edited.contextGroup};return convertPrepared(request,&prepared);
}
nlohmann::json convertHkx(const ConverterRequest& request,const ConverterPreparedClip& prepared) {return convertPrepared(request,&prepared);}
nlohmann::json convertHkxGroup(const ConverterRequest& request,const std::vector<ConverterPreparedClip>& prepared) {
    check(!prepared.empty()&&prepared.size()<=16,"An action group requires 1..16 edited stages");
    const auto pack=std::filesystem::weakly_canonical(request.pack),root=pack.parent_path(),output=std::filesystem::absolute(request.output).lexically_normal();
    check(pack.filename()=="pack.json"&&lower(pathText(output.extension()))==".zip","Select a complete pack and a ZIP output");
    check(!containedPath(root,std::filesystem::weakly_canonical(output))&&std::filesystem::is_directory(output.parent_path()),"Output must be outside the base pack in an existing directory");
    check(request.overwrite||!std::filesystem::exists(output),"Output already exists; use overwrite explicitly");
    for(const auto& edit:prepared)for(const auto& saved:edit.snapshots) {
        check(!containedPath(saved.path,std::filesystem::weakly_canonical(output)),"Output cannot overwrite a source file");
        check(readFile(saved.path,64*1024*1024)==saved.bytes,"Editor input or base pack changed; reload before export");
    }
    auto libraryStorage=std::make_unique<Library>();auto& library=*libraryStorage;const auto initial=loadAnimationPack(library,pack);check(initial.committed,"Base animation pack rejected: "+initial.error);
    std::map<std::string,Bytes> snapshot;std::map<std::string,std::string> configs;std::map<std::string,Json> originals;std::set<std::string> paths;std::size_t total=0;
    auto capture=[&](const std::string& name) {
        const auto path=std::filesystem::weakly_canonical(root/utf8Path(name));check(containedPath(root,path),"Pack path escapes its directory");
        if(!paths.insert(lower(name)).second){if(!snapshot.contains(name)&&lower(pathText(path.extension()))==".hkx"){const auto previous=std::find_if(snapshot.begin(),snapshot.end(),[&](const auto& item){return lower(item.first)==lower(name);});check(previous!=snapshot.end()&&std::filesystem::equivalent(path,root/utf8Path(previous->first)),"Ambiguously cased animation path");}else check(snapshot.contains(name),"Ambiguously cased pack path");return;}
        auto bytes=readFile(path,lower(pathText(path.extension()))==".hkx"?64*1024*1024:256*1024);check(bytes.size()<=256*1024*1024-total,"Base pack exceeds 256 MiB");total+=bytes.size();snapshot.emplace(name,std::move(bytes));
    };
    capture("pack.json");const auto manifest=readJson(snapshot.at("pack.json"));capture(relativeName(manifest.at("skeleton")));
    for(const auto& entry:manifest.at("motions")) {
        const auto name=entry.at("slot").get<std::string>(),file=relativeName(entry.at("config"));capture(file);
        const auto config=selectAnimationClipConfig(readJson(snapshot.at(file)),name);configs.emplace(name,file);originals.emplace(name,config);capture(relativeName(config.at("file")));
    }
    const auto firstSlot=prepared.front().config.at("slot").get<std::string>();const bool contextual=converter::isContextHop(firstSlot)||firstSlot=="contextHang";const std::string group=contextual?"contextHop":std::string(converter::actionPrimaryForSlot(firstSlot));const auto required=actionGroupMotions(group);
    check(!required.empty(),"Unknown complete action group");
    std::string selectedDirection=prepared.front().direction;
    if(contextual)selectedDirection=converter::isContextHop(firstSlot)?firstSlot:converter::isContextHop(selectedDirection)?selectedDirection:"contextHopLeft";
    if(group=="wallRun"&&selectedDirection.empty())selectedDirection=std::string(converter::wallRunPrimaryForSlot(firstSlot,"runUp"));
    const auto configName=configs.at(group=="wallRun"&&converter::isWallRunPrimary(selectedDirection)?selectedDirection:firstSlot);
    check(readJson(snapshot.at(configName)).value("format",std::string{})=="FreeClimbActionGroup","Complete-action export requires a current base pack with grouped configurations");
    std::map<std::string,const ConverterPreparedClip*> edits;std::vector<std::pair<std::string,Bytes>> members;std::vector<Json> editedConfigs;std::vector<std::string> warnings;
    const auto originalGroup=readJson(snapshot.at(configName));
    const bool separate=originalGroup.value("version",0)==2&&!originalGroup.value("direction",std::string{}).empty();
    if(separate)check(originalGroup.at("direction")==selectedDirection,"Selected wall-run direction differs from its base file");
    for(const auto& edit:prepared) {
        const auto name=edit.config.at("slot").get<std::string>();
        check(contextual?(name==selectedDirection||name=="contextHang"):converter::actionPrimaryForSlot(name)==group,"Mixed action group stages");
        if(separate)check(edit.direction.empty()||edit.direction==selectedDirection,"Independent wall-run export accepts only one direction");
        if(!edit.direction.empty()){const auto roles=converter::wallRunStages(edit.direction);check(contextual?edit.direction==selectedDirection:group=="wallRun"&&std::find(roles.begin(),roles.end(),name)!=roles.end(),"Invalid complete-action stage direction");}
        const auto key=edit.direction.empty()?name:edit.direction+"/"+name;check(edits.emplace(key,&edit).second,"Duplicate action stage");
        const auto& original=selectAnimationClipConfig(originalGroup,name,edit.direction);check(edit.config.at("format")==original.at("format")&&edit.config.at("file")==original.at("file")&&
            (edit.config.at("version")==original.at("version")||(edit.config.at("version")==2&&edit.config.contains("authoredPlayback"))),"Edited stage identity changed");
    }
    auto getStage=[&](const std::string& name,const std::string& direction){
        if(!separate)check(configs.at(name)==configName,"Complete-action stages must share one configuration in the base pack");
        auto found=edits.find(direction.empty()?name:direction+"/"+name);if(found==edits.end())found=edits.find(name);
        ConverterPreparedClip stage;stage.direction=direction;std::string error;
        if(found!=edits.end()){
            const auto& edit=*found->second;stage.config=edit.config;stage.warnings=edit.warnings;
            bool canonical=edit.clip.trackNames==library.names&&edit.clip.boneIndices.size()==99;
            if(canonical)for(int bone=0;bone<99;++bone)canonical=canonical&&edit.clip.boneIndices[bone]==bone;
            const auto bytes=canonical?writeCanonicalConverterHkx(edit.clip,library):writeConverterHkx(edit.clip,library,stage.warnings);
            check(decodeConverterInput(bytes,stage.clip,error),"Edited stage serialization failed: "+error);
        }else{
            stage.config=selectAnimationClipConfig(originalGroup,name,direction);
            check(decodeConverterInput(snapshotBytes(snapshot,root,relativeName(stage.config.at("file"))),stage.clip,error,stage.config.value("member",std::string{})),"Unedited base stage cannot be read: "+error);
            stage.clip=sliceConverterTimeline(stage.clip,stage.config);
        }
        check(stage.clip.boneIndices.size()==99&&stage.clip.trackNames.size()==99,"Base stage must retain canonical tracks");for(int bone=0;bone<99;++bone)check(stage.clip.boneIndices[bone]==bone,"Base stage has a reordered binding");
        warnings.insert(warnings.end(),stage.warnings.begin(),stage.warnings.end());return stage;
    };
    Json slots=Json::array(),files=Json::array(),groupConfig;Bytes bundle;
    if(group=="wallRun"){
        std::vector<ConverterPreparedClip> stages;
        ConverterWallRunPack composed;
        if(separate) {
            const auto roles=converter::wallRunStages(selectedDirection);for(std::size_t i=0;i<3;++i)stages.push_back(getStage(std::string(roles[i]),selectedDirection));
            if(selectedDirection!="runUp"&&originalGroup.at("sequences").at(0).contains("brace"))stages.push_back(getStage("sideBrace",selectedDirection));
            composed=composeConverterWallRunDirection(library,selectedDirection,stages);
        }else {
            for(const auto direction:{"runUp","runLeft","runRight","runDiagonalLeft","runDiagonalRight"}){const auto roles=converter::wallRunStages(direction);for(std::size_t i=0;i<3;++i)stages.push_back(getStage(std::string(roles[i]),direction));}
            stages.push_back(getStage("sideBrace",{}));composed=composeConverterWallRunPack(library,stages);
        }
        bundle=std::move(composed.hkx);groupConfig=std::move(composed.config);
        warnings.insert(warnings.end(),composed.warnings.begin(),composed.warnings.end());
    }else if(contextual&&separate){
        const auto& edit=prepared.front();
        if(!edit.contextGroup.empty()){
            check(prepared.size()==1&&edit.contextGroup.at("direction")==selectedDirection,"Complete side-leap edits must target exactly one direction");groupConfig=edit.contextGroup;
            bundle=bundleConverterHkx({{selectedDirection,writeCanonicalConverterHkx(edit.clip,library)}});
        }else if(firstSlot=="contextHang"||!edit.config.contains("authoredPlayback")){
            std::vector<ConverterPreparedClip> stages{getStage("contextHang",{}),getStage(selectedDirection,{})};auto recovery=stages.front();recovery.config=originalGroup.at("sequences").at(0).at("catch");std::string error;check(decodeConverterInput(snapshotBytes(snapshot,root,relativeName(recovery.config.at("file"))),recovery.clip,error,recovery.config.value("member",std::string{})),error);recovery.clip=sliceConverterTimeline(recovery.clip,recovery.config);stages.push_back(std::move(recovery));auto composed=composeConverterContextHopDirection(library,selectedDirection,stages);groupConfig=std::move(composed.config);bundle=std::move(composed.hkx);warnings.insert(warnings.end(),composed.warnings.begin(),composed.warnings.end());
        }else{
            auto main=getStage(firstSlot,{});main.config["member"]=selectedDirection;main.config.erase("frameRange");main.config.erase("rootShift");
            std::vector<std::pair<std::string,Bytes>> parts{{selectedDirection,writeCanonicalConverterHkx(main.clip,library)}};std::array<Json,2> boundaries;
            for(unsigned index=0;index<2;++index){auto metadata=originalGroup.at("sequences").at(0).at(index?"catch":"prepare");std::string error;ConverterInputClip original;check(decodeConverterInput(snapshotBytes(snapshot,root,relativeName(metadata.at("file"))),original,error,metadata.value("member",std::string{})),error);original=sliceConverterTimeline(original,metadata);metadata.erase("frameRange");metadata.erase("rootShift");metadata.erase("authoredPlayback");metadata["version"]=1;metadata["contacts"]={{1,1,0,0},{1,1,0,0}};original.frames={original.frames.front(),original.frames.front()};original.duration=1.f/60;original.rotations.clear();original.annotations.clear();original.referenceFrame={};const auto member=selectedDirection+(index?"Catch":"Prepare");metadata["member"]=member;boundaries[index]=std::move(metadata);parts.emplace_back(member,writeCanonicalConverterHkx(original,library));}
            groupConfig={{"format","FreeClimbActionGroup"},{"version",2},{"group","contextHop"},{"direction",selectedDirection},{"clips",{boundaries[0],main.config}},{"sequences",{{{"slot",selectedDirection},{"prepare",boundaries[0]},{"catch",boundaries[1]}}}}};bundle=bundleConverterHkx(parts);

        }
    }else{
        groupConfig={{"format","FreeClimbActionGroup"},{"version",1},{"group",group},{"clips",Json::array()}};
        for(const auto motion:required){const std::string name(motionSlotNames[int(motion)-1]);auto stage=getStage(name,{});stage.config["member"]=name;stage.config.erase("frameRange");stage.config.erase("rootShift");members.emplace_back(name,writeCanonicalConverterHkx(stage.clip,library));groupConfig["clips"].push_back(std::move(stage.config));}
        bundle=bundleConverterHkx(members);
    }
    std::uint64_t hash=14695981039346656037ull;for(const auto byte:bundle)hash=(hash^byte)*1099511628211ull;
    const auto hkxName="converted/"+(separate?selectedDirection:group)+"-"+std::to_string(hash)+".hkx";
    check(!paths.contains(lower(hkxName))||snapshotBytes(snapshot,root,hkxName)==bundle,"Generated animation path conflicts with different base content");
    std::map<std::string,Bytes> replacements;replacements.emplace(hkxName,bundle);
    for(auto& config:groupConfig["clips"]){assignClipFile(config,hkxName);slots.push_back(config.at("slot"));}
    if(groupConfig.contains("sequences"))for(auto& sequence:groupConfig["sequences"])for(const auto key:{"launch","prepare","catch","brace"})if(sequence.contains(key))assignClipFile(sequence[key],hkxName);
    selectAnimationClipConfig(groupConfig,firstSlot);const auto text=groupConfig.dump(2)+"\n";replacements.emplace(configName,Bytes(text.begin(),text.end()));
    const auto work=request.work.empty()?output.parent_path():std::filesystem::weakly_canonical(request.work);check(!containedPath(root,work),"Work directory cannot be inside the base pack");Temporary stage(work);
    for(const auto& [name,data]:snapshot)writeFile(stage.path/utf8Path(name),replacements.contains(name)?replacements.at(name):data);
    for(const auto& [name,data]:replacements)if(!snapshot.contains(name))writeFile(stage.path/utf8Path(name),data);
    auto candidateStorage=std::make_unique<Library>();auto& candidate=*candidateStorage;const auto validation=loadAnimationPack(candidate,stage.path/"pack.json");
    if(!validation.committed){std::string why=validation.error;for(const auto& slot:validation.slots)if(slot.status==OverrideStatus::rejected)why+="; "+slot.file+": "+slot.reason;throw std::runtime_error("Edited action group rejected: "+why);}
    for(const auto& [name,data]:snapshot)check(readFile(root/utf8Path(name),64*1024*1024)==data,"Base pack changed during export");
    for(const auto& edit:prepared)for(const auto& saved:edit.snapshots)check(readFile(saved.path,64*1024*1024)==saved.bytes,"Editor input changed during export");
    std::vector<std::pair<std::string,Bytes>> archiveFiles;for(const auto& [name,data]:replacements){archiveFiles.emplace_back(std::string(prefix)+name,data);files.push_back(std::string(prefix)+name);}
    Temporary publishing(output.parent_path());const auto temporary=publishing.path/"output.zip";writeFile(temporary,zip(archiveFiles));publish(temporary,output,request.overwrite);
    std::sort(warnings.begin(),warnings.end());warnings.erase(std::unique(warnings.begin(),warnings.end()),warnings.end());
    return {{"ok",true},{"output",pathText(output)},{"slot",firstSlot},{"duration",prepared.front().clip.duration},{"frames",prepared.front().clip.frames.size()},{"slots",slots},{"files",files},{"loaded",validation.loaded},{"warnings",warnings},{"error",""}};
}
}

