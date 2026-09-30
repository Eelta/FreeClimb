#include "HkxAnimation.h"
#include "HkxSpline.h"
#include <chrono>
#include <cstring>
#include <map>
#include <limits>
#include <stdexcept>

namespace fc {
namespace {
struct InvalidHkx : std::runtime_error {using std::runtime_error::runtime_error;};
struct Section {
    std::size_t begin{},local{},global{},virtuals{},exports{},imports{},end{};
};
struct ArrayView {std::size_t at{},count{};};
class Packfile {
    std::span<const std::uint8_t> bytes;
    std::array<Section,3> sections{};
    std::map<std::size_t,std::size_t> pointers;
    std::map<std::size_t,std::string> classes;
    std::chrono::steady_clock::time_point deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
    std::size_t object{};
    void require(bool valid,const char* message) const {if(!valid)throw InvalidHkx(message);}
    void bounded(std::size_t at,std::size_t size) const {require(at<=bytes.size()&&size<=bytes.size()-at,"HKX byte range is outside file");}
    void timed() const {require(std::chrono::steady_clock::now()<deadline,"HKX decode time budget exceeded");}
    template<class T>T raw(std::size_t at) const {bounded(at,sizeof(T));T result;std::memcpy(&result,bytes.data()+at,sizeof(T));return result;}
    const Section& containing(std::size_t at,std::size_t size) const {
        for(const auto& s:sections)if(at>=s.begin&&at-s.begin<=s.local&&size<=s.local-(at-s.begin))return s;
        throw InvalidHkx("HKX object or array exceeds section payload");
    }
    template<class T>T read(std::size_t at) const {containing(at,sizeof(T));return raw<T>(at);}
    std::size_t address(std::uint32_t section,std::uint32_t offset,std::size_t size=1) const {
        require(section<sections.size(),"HKX fixup section index is invalid");
        const auto& s=sections[section];require(offset<=s.local&&size<=s.local-offset,"HKX fixup target exceeds section payload");
        return s.begin+offset;
    }
    std::string stringAt(std::size_t at,std::size_t limit=256) const {
        const auto& s=containing(at,1);const auto maximum=std::min(limit,s.begin+s.local-at);
        std::size_t n=0;while(n<maximum&&bytes[at+n])++n;
        require(n<maximum,"HKX string is missing a bounded terminator");
        return std::string(reinterpret_cast<const char*>(bytes.data()+at),n);
    }
    std::size_t pointer(std::size_t at,bool optional=false) const {
        require(read<std::uint64_t>(at)==0,"HKX serialized pointer must be zero");
        const auto found=pointers.find(at);
        if(found==pointers.end()){require(optional,"HKX required pointer fixup is missing");return 0;}
        return found->second;
    }
    std::string pointerString(std::size_t at,bool optional=false) const {
        const auto target=pointer(at,optional);return target?stringAt(target):std::string{};
    }
    ArrayView array(std::size_t at,std::size_t stride,std::size_t maximum) const {
        containing(at,16);const auto count=read<std::int32_t>(at+8);
        require(count>=0&&std::size_t(count)<=maximum,"HKX array count exceeds supported bounds");
        const auto capacity=read<std::uint32_t>(at+12)&0x3fffffffu;
        require(capacity>=std::uint32_t(count),"HKX array capacity is smaller than count");
        const auto target=pointer(at,count==0);
        require(stride&&std::size_t(count)<=std::numeric_limits<std::size_t>::max()/stride,"HKX array byte count overflow");
        if(count)containing(target,std::size_t(count)*stride);
        return {target,std::size_t(count)};
    }
    std::string classAt(std::size_t at) const {
        const auto found=classes.find(at);require(found!=classes.end(),"HKX object class fixup is missing");return found->second;
    }
    void classIs(std::size_t at,std::string_view name,std::size_t bytesRequired) const {
        require(classAt(at)==name,"HKX object class does not match animation layout");containing(at,bytesRequired);
    }
    void fixups(std::size_t index,std::size_t begin,std::size_t end,std::size_t width,int kind) {
        const auto& s=sections[index];std::size_t cursor=begin;
        while(cursor<end) {
            timed();const auto absolute=s.begin+cursor;
            if(end-cursor<width||raw<std::uint32_t>(absolute)==0xffffffffu) {
                for(;cursor<end;++cursor) {
                    if((cursor&4095)==0)timed();
                    require(bytes[s.begin+cursor]==0xff,"HKX fixup padding is malformed");
                }
                break;
            }
            const auto source=raw<std::uint32_t>(absolute);
            require((source%8)==0,"HKX fixup source is not pointer aligned");
            const auto key=address(std::uint32_t(index),source,kind==2?1:8);
            std::size_t target{};
            if(kind==0)target=address(std::uint32_t(index),raw<std::uint32_t>(absolute+4));
            else target=address(raw<std::uint32_t>(absolute+4),raw<std::uint32_t>(absolute+8));
            if(kind==2)require(classes.emplace(key,stringAt(target,128)).second,"Duplicate HKX class fixup");
            else {
                require(raw<std::uint64_t>(key)==0,"HKX fixup points at nonzero serialized pointer");
                require(pointers.emplace(key,target).second,"Duplicate HKX pointer fixup");
            }
            require(classes.size()+pointers.size()<=50000,"HKX fixup count exceeds limit");cursor+=width;
        }
    }
    std::vector<std::uint32_t> words(std::size_t at,std::size_t maximum) const {
        const auto values=array(at,4,maximum);std::vector<std::uint32_t> out(values.count);
        for(std::size_t i=0;i<values.count;++i)out[i]=read<std::uint32_t>(values.at+i*4);
        return out;
    }
public:
    explicit Packfile(std::span<const std::uint8_t> data):bytes(data) {
        require(bytes.size()>=208&&bytes.size()<=64*1024*1024,"HKX file size is outside supported bounds");
        require(raw<std::uint32_t>(0)==0x57e0e057u&&raw<std::uint32_t>(4)==0x10c0c010u,"Not a Havok binary packfile");
        require(raw<std::uint32_t>(12)==8&&bytes[16]==8&&bytes[17]==1&&bytes[18]==0&&bytes[19]==1,
            "Only Skyrim SE 64-bit little-endian 2010 packfiles are supported");
        require(raw<std::uint32_t>(20)==3&&raw<std::int16_t>(62)==-1,"Unsupported HKX section layout");
        require(std::memcmp(bytes.data()+40,"hk_2010.2.0-r1",14)==0,"Unsupported Havok contents version");
        for(std::size_t i=0;i<sections.size();++i) {
            const std::size_t h=64+i*48;auto& s=sections[i];
            s.begin=raw<std::uint32_t>(h+20);s.local=raw<std::uint32_t>(h+24);s.global=raw<std::uint32_t>(h+28);
            s.virtuals=raw<std::uint32_t>(h+32);s.exports=raw<std::uint32_t>(h+36);s.imports=raw<std::uint32_t>(h+40);s.end=raw<std::uint32_t>(h+44);
            require(s.begin>=208&&s.local<=s.global&&s.global<=s.virtuals&&s.virtuals<=s.exports&&s.exports<=s.imports&&s.imports<=s.end,"HKX section offsets are not ordered");
            bounded(s.begin,s.end);require(s.exports==s.imports&&s.imports==s.end,"HKX external imports and exports are unsupported");
        }
        for(std::size_t i=0;i<3;++i)for(std::size_t j=i+1;j<3;++j) {
            const auto& a=sections[i];const auto& b=sections[j];
            require(a.end==0||b.end==0||a.begin+a.end<=b.begin||b.begin+b.end<=a.begin,"HKX sections overlap");
        }
        for(std::size_t i=0;i<3;++i) {
            const auto& s=sections[i];fixups(i,s.local,s.global,8,0);fixups(i,s.global,s.virtuals,12,1);fixups(i,s.virtuals,s.exports,12,2);
        }
        object=address(raw<std::uint32_t>(24),raw<std::uint32_t>(28),16);
        const auto name=stringAt(address(raw<std::uint32_t>(32),raw<std::uint32_t>(36)),128);
        require(name=="hkRootLevelContainer","HKX root must be an animation root container");
        classIs(object,"hkRootLevelContainer",16);
    }
    HkxClip decode() {
        const auto variants=array(object,24,2);require(variants.count>=1,"HKX animation container variant is missing");
        std::size_t container=0;bool resourceSeen=false;
        for(std::size_t i=0;i<variants.count;++i) {
            const auto variant=variants.at+i*24;
            pointerString(variant,true);
            const auto type=pointerString(variant+8);const auto target=pointer(variant+16);
            if(type=="hkaAnimationContainer") {
                require(container==0,"HKX contains duplicate animation containers");container=target;
                classIs(container,"hkaAnimationContainer",96);
            } else if(type=="hkMemoryResourceContainer") {
                require(!resourceSeen,"HKX contains duplicate resource containers");resourceSeen=true;
                classIs(target,"hkMemoryResourceContainer",80);
                for(std::size_t offset:{16,32,48}) {
                    const auto items=array(target+offset,8,0);
                    require(items.count==0&&items.at==0,"HKX resource container is not empty");
                }
                require(pointer(target+64,true)==0&&read<std::uint64_t>(target+72)==0,
                    "HKX resource container has unsupported name or metadata");
            } else throw InvalidHkx("HKX contains an unsupported root variant");
        }
        require(container!=0,"HKX animation container variant is missing");
        require(array(container+16,8,1).count==0&&array(container+64,8,1).count==0&&array(container+80,8,1).count==0,
            "HKX must be an animation-only file without embedded skeletons or meshes");
        const auto animations=array(container+32,8,1),bindings=array(container+48,8,1);
        require(animations.count==1&&bindings.count==1,"HKX must contain one animation and one binding");
        const auto animation=pointer(animations.at),binding=pointer(bindings.at);classIs(binding,"hkaAnimationBinding",72);
        require(pointer(binding+24)==animation,"HKX binding references a different animation");
        require(read<std::uint8_t>(binding+64)==0,"Additive HKX bindings are unsupported");
        require(array(binding+48,2,0).count==0,"HKX float bindings are unsupported");
        HkxClip result;result.skeletonName=pointerString(binding+16);
        require(result.skeletonName=="NPC Root [Root]","HKX binding must target NPC Root [Root]");
        containing(animation,56);const auto type=read<std::uint32_t>(animation+16);
        result.duration=read<float>(animation+20);const auto tracks=read<std::int32_t>(animation+24);
        require(std::isfinite(result.duration)&&result.duration>0&&result.duration<=10,"HKX duration must be finite and between zero and ten seconds");
        require(tracks>=1&&tracks<=256,"HKX transform track count must be 1..256");
        require(read<std::int32_t>(animation+28)==0,"HKX float animation tracks are unsupported");
        const auto mapping=array(binding+32,2,256);
        require(mapping.count==std::size_t(tracks)||(mapping.count==0&&(tracks==99||tracks==126)),"HKX binding map must cover every track; identity requires 99 or named 126 tracks");
        result.identityMapping=mapping.count==0;
        std::array<bool,256> seen{};result.boneIndices.resize(tracks);
        for(int i=0;i<tracks;++i) {
            const int bone=mapping.count?read<std::int16_t>(mapping.at+std::size_t(i)*2):i;
            require(bone>=0&&bone<256&&!seen[bone],"HKX bone map contains an invalid or duplicate index");seen[bone]=true;result.boneIndices[i]=bone;
        }
        const auto annotations=array(animation+40,24,256);
        require(annotations.count==0||annotations.count==std::size_t(tracks),"HKX track names do not match track count");
        result.trackNames.resize(tracks);
        std::size_t annotationCount=0;
        for(std::size_t i=0;i<annotations.count;++i) {
            timed();
            result.trackNames[i]=pointerString(annotations.at+i*24,true);
            const auto events=array(annotations.at+i*24+8,16,4096);
            require(events.count<=8192-annotationCount,"HKX annotation count exceeds limit");annotationCount+=events.count;
            for(std::size_t e=0;e<events.count;++e) {
                if((e&255)==0)timed();
                const float t=read<float>(events.at+e*16);require(std::isfinite(t),"HKX annotation time is invalid");
                pointerString(events.at+e*16+8,true);
            }
        }
        if(type==1) {
            classIs(animation,"hkaInterleavedUncompressedAnimation",88);
            const auto transforms=array(animation+56,48,std::size_t(tracks)*1201);
            require(array(animation+72,4,0).count==0,"HKX interleaved float data is unsupported");
            require(transforms.count%tracks==0&&transforms.count/std::size_t(tracks)>=2,"HKX interleaved frame count is invalid");
            result.rotations.assign(transforms.count/std::size_t(tracks),std::vector<Quat>(tracks));
            result.frames.assign(result.rotations.size(),Pose(tracks));
            for(std::size_t frame=0;frame<result.rotations.size();++frame) {
                timed();for(int track=0;track<tracks;++track) {
                    const auto at=transforms.at+(frame*tracks+track)*48;
                    for(int k=0;k<12;++k)require(std::isfinite(read<float>(at+k*4)),"HKX transform contains a nonfinite value");
                    for(int k=0;k<3;++k)require(std::abs(read<float>(at+k*4))<100000&&read<float>(at+32+k*4)>0&&read<float>(at+32+k*4)<=5,"HKX translation or scale is outside supported bounds");
                    Quat q{read<float>(at+16),read<float>(at+20),read<float>(at+24),read<float>(at+28)};
                    require(std::abs(q.dot(q)-1)<.01f,"HKX rotation quaternion is not normalized");result.rotations[frame][track]=q.unit();
                    result.frames[frame][track]={{read<float>(at),read<float>(at+4),read<float>(at+8)},q,
                        {read<float>(at+32),read<float>(at+36),read<float>(at+40)}};
                }
            }
        } else if(type==5) {
            classIs(animation,"hkaSplineCompressedAnimation",176);
            require(read<std::uint32_t>(animation+168)==0,"HKX spline stream endian must be zero");
            HkxSplineData source;source.tracks=std::uint32_t(tracks);
            source.numFrames=read<std::uint32_t>(animation+56);source.numBlocks=read<std::uint32_t>(animation+60);
            source.maxFramesPerBlock=read<std::uint32_t>(animation+64);source.maskAndQuantizationSize=read<std::uint32_t>(animation+68);
            source.blockDuration=read<float>(animation+72);source.blockInverseDuration=read<float>(animation+76);source.frameDuration=read<float>(animation+80);
            require(source.numFrames>=2&&source.numFrames<=1201&&source.numBlocks>=1&&source.numBlocks<=1201,"HKX spline frame/block count is invalid");
            require(std::isfinite(source.frameDuration)&&std::abs(source.frameDuration*float(source.numFrames-1)-result.duration)<.002f,
                "HKX spline frame timing disagrees with animation duration");
            source.blockOffsets=words(animation+88,1201);source.floatBlockOffsets=words(animation+104,1201);
            source.transformOffsets=words(animation+120,1201*256);source.floatOffsets=words(animation+136,1201);
            const auto data=array(animation+152,1,64*1024*1024);source.data.assign(bytes.begin()+data.at,bytes.begin()+data.at+data.count);
            std::string failure;require(decodeHkxSplineTransforms(source,result.frames,failure),failure.c_str());timed();
        } else throw InvalidHkx("HKX animation type is unsupported; use interleaved or spline compressed");
        if(result.rotations.empty()) {
            result.rotations.resize(result.frames.size());
            for(std::size_t frame=0;frame<result.frames.size();++frame)
                for(const auto& transform:result.frames[frame])result.rotations[frame].push_back(transform.q);
        }
        for(const auto& frame:result.frames)for(const auto& transform:frame) {
            require(transform.t.finite()&&transform.s.finite()&&transform.t.length()<100000,
                "Decoded HKX translation is invalid");
            require(transform.s.x>0&&transform.s.y>0&&transform.s.z>0&&transform.s.x<=5&&transform.s.y<=5&&transform.s.z<=5,
                "Decoded HKX scale is invalid");
        }
        require(result.rotations.size()>=2&&result.rotations.size()<=1201,"Decoded HKX frame count is invalid");
        for(const auto& frame:result.rotations) {
            require(frame.size()==std::size_t(tracks),"Decoded HKX track count is invalid");
            for(const auto& q:frame)require(std::isfinite(q.dot(q))&&std::abs(q.dot(q)-1)<.01f,"Decoded HKX rotation is invalid");
        }
        return result;
    }
};
}
bool decodeHkxAnimation(std::span<const std::uint8_t> bytes,HkxClip& clip,std::string& error) {
    clip={};error.clear();
    try {auto result=Packfile(bytes).decode();clip=std::move(result);return true;}
    catch(const std::exception& failure){error=failure.what();return false;}
}
}
