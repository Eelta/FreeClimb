#include "Converter.h"
#include "ConverterEditor.h"
#include "ConverterSpline.h"
#include "ConverterSourceMotion.h"
#include "../../src/AnimationPack.h"
#include "../../src/HkxAnimation.h"
#include "../../src/CanonicalSkeleton.h"
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <tuple>
#include "../../tests/HkxFixtures.h"

namespace {
using Bytes=std::vector<std::uint8_t>;
std::size_t checks{};
void expect(bool valid,const std::string& message) {++checks;if(!valid)throw std::runtime_error(message);}
Bytes read(const std::filesystem::path& p) {std::ifstream f(p,std::ios::binary);return Bytes(std::istreambuf_iterator<char>(f),{});}
void write(const std::filesystem::path& p,const Bytes& bytes) {std::filesystem::create_directories(p.parent_path());std::ofstream file(p,std::ios::binary);expect(bool(file.write(reinterpret_cast<const char*>(bytes.data()),bytes.size())),"fixture write");}
void writeJson(const std::filesystem::path& p,const nlohmann::json& value) {const auto text=value.dump(2);write(p,Bytes(text.begin(),text.end()));}
template<class T>T word(const Bytes& b,std::size_t at) {expect(at<=b.size()&&sizeof(T)<=b.size()-at,"archive field bounds");T value;std::memcpy(&value,b.data()+at,sizeof value);return value;}
std::size_t objectAt(const Bytes& bytes,std::string_view name) {
    const auto classes=word<std::uint32_t>(bytes,84),data=word<std::uint32_t>(bytes,180),begin=word<std::uint32_t>(bytes,192),end=word<std::uint32_t>(bytes,196);
    for(std::size_t cursor=data+begin;cursor+12<=data+end;cursor+=12) {
        if(word<std::uint32_t>(bytes,cursor)==0xffffffffu)break;
        const auto at=classes+word<std::uint32_t>(bytes,cursor+8);expect(at<bytes.size(),"class-name bounds");const auto terminator=std::find(bytes.begin()+at,bytes.end(),0);expect(terminator!=bytes.end(),"class-name terminator");
        if(std::string_view(reinterpret_cast<const char*>(bytes.data()+at),std::size_t(terminator-(bytes.begin()+at)))==name)return data+word<std::uint32_t>(bytes,cursor);
    }
    throw std::runtime_error("Fixture object not found");
}
std::uint32_t crc(const Bytes& b) {std::uint32_t value=0xffffffffu;for(auto x:b){value^=x;for(int i=0;i<8;++i)value=(value>>1)^((value&1u)?0xedb88320u:0u);}return ~value;}
std::map<std::string,Bytes> unzip(const std::filesystem::path& p) {
    const auto b=read(p);expect(b.size()>=22,"ZIP header length");const auto end=b.size()-22;expect(word<std::uint32_t>(b,end)==0x06054b50u,"ZIP end signature");expect(word<std::uint16_t>(b,end+8)==2&&word<std::uint16_t>(b,end+10)==2,"ZIP contains exactly two entries");
    std::map<std::string,Bytes> files;std::size_t central=word<std::uint32_t>(b,end+16);
    for(int entry=0;entry<2;++entry) {
        expect(word<std::uint32_t>(b,central)==0x02014b50u,"ZIP central signature");expect(word<std::uint16_t>(b,central+10)==0,"ZIP stored method");
        const auto size=word<std::uint32_t>(b,central+24),local=word<std::uint32_t>(b,central+42);const auto nameSize=word<std::uint16_t>(b,central+28),extra=word<std::uint16_t>(b,central+30),comment=word<std::uint16_t>(b,central+32);
        expect(central+46+nameSize<=b.size(),"ZIP filename bounds");const std::string name(reinterpret_cast<const char*>(b.data()+central+46),nameSize);
        expect(name.starts_with("meshes/actors/character/animations/FreeClimb/")&&name.find("..") == std::string::npos,"MO2 relative path");expect(word<std::uint32_t>(b,local)==0x04034b50u,"ZIP local signature");
        const auto dataAt=local+30+word<std::uint16_t>(b,local+26)+word<std::uint16_t>(b,local+28);expect(dataAt+size<=b.size(),"ZIP data bounds");
        Bytes data(b.begin()+dataAt,b.begin()+dataAt+size);expect(crc(data)==word<std::uint32_t>(b,central+16)&&crc(data)==word<std::uint32_t>(b,local+14),"ZIP CRC");expect(files.emplace(name,std::move(data)).second,"ZIP duplicate entry");central+=46+nameSize+extra+comment;
    }
    expect(central==end,"ZIP central length");return files;
}
void rejected(const fc::ConverterRequest& request,const std::string& name) {
    bool failure=false;try{fc::convertHkx(request);}catch(const std::exception&){failure=true;}expect(failure,name);expect(!std::filesystem::exists(request.output),name+" did not publish output");
}
std::string normalized(std::string s) {return s.starts_with("x_")?s.substr(2):s;}
template<class T>void put(Bytes& bytes,std::size_t at,T value) {expect(at<=bytes.size()&&sizeof(value)<=bytes.size()-at,"fixture mutation bounds");std::memcpy(bytes.data()+at,&value,sizeof(value));}
std::size_t pointerAt(const Bytes& bytes,std::size_t field) {
    const auto data=word<std::uint32_t>(bytes,180),begin=word<std::uint32_t>(bytes,184),end=word<std::uint32_t>(bytes,188);
    for(std::size_t at=data+begin;at+8<=data+end;at+=8)if(word<std::uint32_t>(bytes,at)==field-data)return data+word<std::uint32_t>(bytes,at+4);
    throw std::runtime_error("Fixture pointer not found");
}
void repoint(Bytes& bytes,std::size_t field,std::size_t target) {
    const auto data=word<std::uint32_t>(bytes,180),begin=word<std::uint32_t>(bytes,184),end=word<std::uint32_t>(bytes,188);
    for(std::size_t at=data+begin;at+8<=data+end;at+=8)if(word<std::uint32_t>(bytes,at)==field-data){put(bytes,at+4,std::uint32_t(target-data));return;}
    throw std::runtime_error("Fixture fixup not found");
}
struct MetadataFixture {
    Bytes header,names,data;
    std::vector<std::array<std::uint32_t,2>> local;
    std::vector<std::array<std::uint32_t,3>> global,objects;
    explicit MetadataFixture(const Bytes& seed) {
        const auto classes=word<std::uint32_t>(seed,84),payload=word<std::uint32_t>(seed,180);
        const auto begin=word<std::uint32_t>(seed,184),middle=word<std::uint32_t>(seed,188),virtuals=word<std::uint32_t>(seed,192),end=word<std::uint32_t>(seed,196);
        header.assign(seed.begin(),seed.begin()+208);names.assign(seed.begin()+classes,seed.begin()+payload);while(!names.empty()&&names.back()==255)names.pop_back();
        data.assign(seed.begin()+payload,seed.begin()+payload+begin);
        for(std::size_t at=payload+begin;at+8<=payload+middle;at+=8){const auto source=word<std::uint32_t>(seed,at);if(source==0xffffffffu)break;local.push_back({source,word<std::uint32_t>(seed,at+4)});}
        for(auto [first,last,rows]:{std::tuple{middle,virtuals,&global},std::tuple{virtuals,end,&objects}})
            for(std::size_t at=payload+first;at+12<=payload+last;at+=12){const auto source=word<std::uint32_t>(seed,at);if(source==0xffffffffu)break;rows->push_back({source,word<std::uint32_t>(seed,at+4),word<std::uint32_t>(seed,at+8)});}
    }
    std::uint32_t allocate(std::size_t size) {while(data.size()%16)data.push_back(0);const auto at=std::uint32_t(data.size());data.resize(data.size()+size);return at;}
    std::uint32_t text(const std::string& value) {const auto at=std::uint32_t(data.size());data.insert(data.end(),value.begin(),value.end());data.push_back(0);return at;}
    std::uint32_t type(const std::string& value) {const auto at=names.size();names.resize(at+5);names[at+4]=9;const auto offset=std::uint32_t(names.size());names.insert(names.end(),value.begin(),value.end());names.push_back(0);return offset;}
    Bytes finish() {
        while(names.size()%16)names.push_back(255);while(data.size()%16)data.push_back(0);const auto begin=std::uint32_t(data.size());
        auto rows=[&](const auto& values){for(const auto& row:values)for(auto value:row){const auto at=data.size();data.resize(at+4);put(data,at,value);}while(data.size()%16)data.push_back(255);};
        rows(local);const auto middle=std::uint32_t(data.size());rows(global);const auto virtuals=std::uint32_t(data.size());rows(objects);const auto end=std::uint32_t(data.size());
        const auto payload=208u+std::uint32_t(names.size());put(header,84,208u);put(header,132,payload);put(header,180,payload);
        for(std::size_t field=88;field<=108;field+=4)put(header,field,std::uint32_t(names.size()));
        put(header,184,begin);put(header,188,middle);put(header,192,virtuals);for(std::size_t field=196;field<=204;field+=4)put(header,field,end);
        auto result=header;result.insert(result.end(),names.begin(),names.end());result.insert(result.end(),data.begin(),data.end());return result;
    }
};
Bytes resourceFixture(const Bytes& seed,const std::string& label,unsigned fault=0) {
    MetadataFixture out(seed);const auto payload=word<std::uint32_t>(seed,180),root=std::uint32_t(objectAt(seed,"hkRootLevelContainer")-payload);
    const auto old=std::uint32_t(pointerAt(seed,payload+root)-payload),resource=out.allocate(64),variants=out.allocate(48);
    std::memcpy(out.data.data()+variants,out.data.data()+old,24);put(out.data,root+8,2u);put(out.data,root+12,0x80000002u);
    for(auto& row:out.local)if(row[0]==root)row[1]=variants;
    const auto count=out.local.size();for(std::size_t i=0;i<count;++i){const auto row=out.local[i];if(row[0]==old||row[0]==old+8)out.local.push_back({variants+row[0]-old,row[1]});}
    const auto prior=std::find_if(out.global.begin(),out.global.end(),[&](const auto& row){return row[0]==old+16;});expect(prior!=out.global.end(),"fixture animation variant reference");
    out.global.push_back({variants+16,(*prior)[1],(*prior)[2]});out.local.push_back({variants+24,out.text("Resource Data")});out.local.push_back({variants+32,out.text("hkMemoryResourceContainer")});out.global.push_back({variants+40,2,resource});
    out.objects.push_back({resource,0,out.type("hkMemoryResourceContainer")});
    const auto name=out.text(fault==1?std::string(256,'x'):label);out.local.push_back({resource+16,name});
    if(fault==2)out.local.push_back({resource+24,resource});
    if(fault==3||fault==4){const auto offset=resource+(fault==3?32u:48u);put(out.data,offset+8,1u);put(out.data,offset+12,0x80000001u);out.local.push_back({offset,resource});}
    auto bytes=out.finish();
    if(fault==5){const auto data=word<std::uint32_t>(bytes,180),local=word<std::uint32_t>(bytes,184),global=word<std::uint32_t>(bytes,188);for(std::size_t at=data+local;at+8<=data+global;at+=8)if(word<std::uint32_t>(bytes,at)==resource+16){put(bytes,at+4,local);break;}}
    if(fault==6)put(bytes,200,word<std::uint32_t>(bytes,196)+1u);
    return bytes;
}
void resourceCases(const Bytes& seed) {
    for(const std::string label:{std::string{},std::string{"Export Metadata"}}) {
        fc::ConverterInputClip clip;std::string error;expect(fc::decodeConverterInput(resourceFixture(seed,label),clip,error),"empty resource with bounded label decodes: "+error);
        fc::ConverterInputClip baseline;expect(fc::decodeConverterInput(seed,baseline,error),"resource fixture baseline decodes");expect(clip.frames.size()==baseline.frames.size(),"resource metadata does not affect animation frames");
    }
    for(unsigned fault=1;fault<=6;++fault) {
        fc::ConverterInputClip clip;std::string error;expect(!fc::decodeConverterInput(resourceFixture(seed,"",fault),clip,error)&&clip.frames.empty()&&!error.empty(),"invalid resource metadata rejected: "+std::to_string(fault));
    }
}
Bytes floatFixture(const Bytes& seed,unsigned fault=0) {
    MetadataFixture out(seed);const auto payload=word<std::uint32_t>(seed,180),animation=std::uint32_t(objectAt(seed,"hkaInterleavedUncompressedAnimation")-payload),binding=std::uint32_t(objectAt(seed,"hkaAnimationBinding")-payload);
    const auto frames=word<std::uint32_t>(seed,payload+animation+64)/99;put(out.data,animation+28,4u);
    const auto count=fault==1?frames*4-1:frames*4,values=out.allocate(std::size_t(count)*4),indices=out.allocate(8);
    put(out.data,animation+80,count);put(out.data,animation+84,0x80000000u|count);out.local.push_back({animation+72,values});
    put(out.data,binding+56,4u);put(out.data,binding+60,0x80000004u);out.local.push_back({binding+48,indices});
    for(unsigned track=0;track<4;++track)put(out.data,indices+track*2,std::int16_t(fault==3?0:track));
    for(unsigned value=0;value<count;++value)put(out.data,values+value*4,float(value)*.03125f);
    if(fault==2)put(out.data,values,std::numeric_limits<float>::quiet_NaN());
    if(fault==4)put(out.data,binding+56,3u);
    return out.finish();
}
void floatCases(const fc::Library& library,const Bytes& seed) {
    fc::ConverterInputClip clip;std::string error;expect(fc::decodeConverterInput(floatFixture(seed),clip,error)&&clip.ignoredFloatTracks==4,"bounded finite float channels coexist with body transforms: "+error);
    std::vector<std::string> warnings;const auto bytes=fc::writeConverterHkx(clip,library,warnings);fc::HkxClip output;
    expect(fc::decodeHkxAnimation(bytes,output,error)&&output.frames.size()==clip.frames.size(),"ignored float channels do not alter body output");
    expect(std::any_of(warnings.begin(),warnings.end(),[](const auto& value){return value.find("4 float animation tracks")!=std::string::npos;}),"float channel loss is reported explicitly");
    for(unsigned fault=1;fault<=4;++fault)expect(!fc::decodeConverterInput(floatFixture(seed,fault),clip,error)&&clip.frames.empty(),"malformed float data or binding is rejected: "+std::to_string(fault));
}
Bytes layoutFixture(const Bytes& seed,const std::vector<int>& order,bool identity,bool names) {
    const auto animation=objectAt(seed,"hkaInterleavedUncompressedAnimation"),binding=objectAt(seed,"hkaAnimationBinding");
    const auto data=pointerAt(seed,animation+56),labels=pointerAt(seed,animation+40),indices=pointerAt(seed,binding+32);
    const auto frames=word<std::uint32_t>(seed,animation+64)/99;expect(!order.empty()&&order.size()<=99,"bounded synthetic layout");auto bytes=seed;
    put(bytes,animation+24,std::uint32_t(order.size()));
    put(bytes,animation+64,std::uint32_t(frames*order.size()));put(bytes,animation+68,0x80000000u|std::uint32_t(frames*order.size()));
    put(bytes,animation+48,std::uint32_t(order.size()));put(bytes,animation+52,0x80000000u|std::uint32_t(order.size()));
    put(bytes,binding+40,identity?0u:std::uint32_t(order.size()));put(bytes,binding+44,0x80000000u|(identity?0u:std::uint32_t(order.size())));
    for(std::size_t track=0;track<order.size();++track) {
        expect(order[track]>=0&&order[track]<99,"fixture uses canonical source bones");
        for(std::size_t frame=0;frame<frames;++frame)std::memcpy(bytes.data()+data+(frame*order.size()+track)*48,seed.data()+data+(frame*99+order[track])*48,48);
        const auto label=pointerAt(seed,labels+std::size_t(order[track])*24);repoint(bytes,labels+track*24,label);
        if(!names)bytes[label]=0;
        put(bytes,indices+track*2,std::int16_t(order[track]));
    }
    return bytes;
}
void layoutCases(const fc::Library& library,const Bytes& seed) {
    auto convert=[&](const Bytes& bytes) {
        fc::ConverterInputClip clip;std::string error;expect(fc::decodeConverterInput(bytes,clip,error),"synthetic layout decodes: "+error);
        std::vector<std::string> warnings;const auto converted=fc::writeConverterHkx(clip,library,warnings);fc::HkxClip output;
        expect(fc::decodeHkxAnimation(converted,output,error),"synthetic output is readable: "+error);return output;
    };
    auto fail=[&](const Bytes& bytes,const std::string& reason) {
        fc::ConverterInputClip clip;std::string error;expect(fc::decodeConverterInput(bytes,clip,error),"invalid mapping fixture is structurally readable");
        bool rejected=false;try{std::vector<std::string> warnings;fc::writeConverterHkx(clip,library,warnings);}catch(const std::exception&){rejected=true;}expect(rejected,reason);
    };
    std::vector<int> order(99);for(int i=0;i<99;++i)order[i]=i;
    for(int count:{97,99}) {
        const std::vector<int> subset(order.begin(),order.begin()+count);const auto output=convert(layoutFixture(seed,subset,true,false));
        expect(output.frames.front().size()==99,"known unnamed identity output remains full pose");
        for(int bone=97;bone<99;++bone)expect(fc::angleBetween(output.frames.front()[bone].q,library.rest[bone].q)<.0001f,"missing camera tracks use protected reference pose");
    }
    fail(layoutFixture(seed,std::vector<int>(order.begin(),order.begin()+98),true,false),"unknown unnamed 98-track identity is rejected");
    fail(layoutFixture(seed,std::vector<int>(order.begin(),order.begin()+95),true,false),"unknown unnamed 95-track identity is rejected");
    fail(layoutFixture(seed,std::vector<int>(order.begin(),order.begin()+6),true,false),"anonymous partial identity cannot silently use first-N bones");
    auto trailing=layoutFixture(seed,std::vector<int>(order.begin(),order.begin()+97),true,true);
    const auto trailingAnimation=objectAt(trailing,"hkaInterleavedUncompressedAnimation");put(trailing,trailingAnimation+48,99u);put(trailing,trailingAnimation+52,0x80000063u);
    convert(trailing);
    std::reverse(order.begin(),order.end());
    for(bool identity:{false,true}) {
        const auto bytes=layoutFixture(seed,order,identity,true);const auto output=convert(bytes);fc::ConverterInputClip clip;std::string error;expect(fc::decodeConverterInput(bytes,clip,error),"reordered reader fixture");
        for(int track=0;track<99;++track) {
            const auto bone=order[track];if((bone>=1&&bone<=3)||bone>=97)continue;
            for(std::size_t frame=0;frame<clip.frames.size();++frame)expect(fc::angleBetween(output.frames[frame][bone].q,clip.frames[frame][track].q)<.0001f,"reordered named local rotations remain on the correct bone");
        }
    }
    const auto sparse=convert(layoutFixture(seed,{38,29,28},false,false));expect(sparse.frames.front().size()==99,"explicit sparse standard bindings fill absent tracks safely");
    convert(layoutFixture(seed,{38,29,28},true,true));
    auto conflict=layoutFixture(seed,order,false,true);const auto binding=objectAt(conflict,"hkaAnimationBinding"),indices=pointerAt(conflict,binding+32);
    for(int track=0;track<99;++track)put(conflict,indices+track*2,std::int16_t(track));fail(conflict,"explicit map and named bone disagreement is rejected");
    auto duplicate=layoutFixture(seed,order,true,true);const auto animation=objectAt(duplicate,"hkaInterleavedUncompressedAnimation"),labels=pointerAt(duplicate,animation+40);
    repoint(duplicate,labels+24,pointerAt(duplicate,labels));fail(duplicate,"duplicate named bones cannot silently overwrite one another");
    auto mixed=layoutFixture(seed,order,true,true);mixed[pointerAt(mixed,labels+48)]=0;fail(mixed,"partly named reordered identities cannot mix indexing conventions");
    fc::ConverterInputClip expanded;std::string error;expect(fc::decodeConverterInput(seed,expanded,error),"expanded fixture canonical source");expanded.identityMapping=true;
    for(int bone=99;bone<156;++bone) {
        expanded.boneIndices.push_back(bone);expanded.trackNames.push_back("NPC Converter Test Extension "+std::to_string(bone));
        for(auto& frame:expanded.frames)frame.push_back(library.rest[0]);
    }
    std::vector<std::string> warnings;const auto extended=fc::writeConverterHkx(expanded,library,warnings);fc::HkxClip decoded;
    expect(fc::decodeHkxAnimation(extended,decoded,error)&&decoded.boneIndices.size()==99,"complete named humanoid with extra tracks converts");
    expect(std::any_of(warnings.begin(),warnings.end(),[](const auto& value){return value.find("57 extension tracks")!=std::string::npos;}),"named extensions are reported explicitly");
    auto unnamed=expanded;std::fill(unnamed.trackNames.begin(),unnamed.trackNames.end(),std::string{});warnings.clear();
    const auto inferred=fc::writeConverterHkx(unnamed,library,warnings);expect(fc::decodeHkxAnimation(inferred,decoded,error),"known unnamed 156-track extended convention converts");
    expect(std::any_of(warnings.begin(),warnings.end(),[](const auto& value){return value.find("156-track humanoid convention")!=std::string::npos;}),"unnamed extended mapping inference is explicitly reported");
    for(int bone=5;bone<97;++bone)for(std::size_t frame=0;frame<unnamed.frames.size();++frame)
        expect(fc::angleBetween(decoded.frames[frame][bone].q,unnamed.frames[frame][bone].q)<.0001f,"known extended convention retains canonical body rotations");
    unnamed.boneIndices.pop_back();unnamed.trackNames.pop_back();for(auto& frame:unnamed.frames)frame.pop_back();bool uncertain=false;
    try{fc::writeConverterHkx(unnamed,library,warnings);}catch(const std::exception&){uncertain=true;}expect(uncertain,"unknown unnamed 155-track convention is not inferred");
    expanded.trackNames[0]="Unknown Retarget Bone";bool unknown=false;try{fc::writeConverterHkx(expanded,library,warnings);}catch(const std::exception&){unknown=true;}expect(unknown,"unknown named body cannot masquerade as an extension");
    fc::ConverterInputClip aliases;expect(fc::decodeConverterInput(seed,aliases,error),"alias fixture canonical source");
    aliases.identityMapping=true;
    for(auto& name:aliases.trackNames) {
        if(name=="Shield")name="SHIELD";else if(name=="Weapon")name="WEAPON";else if(name=="Quiver")name="QUIVER";
        else if(name.starts_with("NPC L ")||name.starts_with("NPC R ")) {
            const char side=name[4];const auto bracket=name.find('[');
            if(bracket!=std::string::npos&&name[bracket+1]==side){name.erase(bracket+1,1);name.erase(4,2);name+='.';name+=side;}
        }
    }
    const auto aliasOutput=fc::writeConverterHkx(aliases,library,warnings);expect(fc::decodeHkxAnimation(aliasOutput,decoded,error),"finite Blender naming aliases convert");
    for(int bone=5;bone<97;++bone)for(std::size_t frame=0;frame<aliases.frames.size();++frame)
        expect(fc::angleBetween(decoded.frames[frame][bone].q,aliases.frames[frame][bone].q)<.0001f,"Blender aliases preserve each bone's local rotation");
    aliases.trackNames[28]="NPC UpperArm [Uar].l";unknown=false;try{fc::writeConverterHkx(aliases,library,warnings);}catch(const std::exception&){unknown=true;}expect(unknown,"unrecognized suffix is not arbitrarily normalized");
}
void splineCases() {
    fc::HkxSplineData source;source.tracks=1;source.numFrames=5;source.numBlocks=1;source.maxFramesPerBlock=5;
    source.maskAndQuantizationSize=4;source.blockDuration=2;source.blockInverseDuration=.5f;source.frameDuration=.5f;source.blockOffsets={0};
    source.data={20,0,180,0,1,0,1,0,0,4,4,0};
    for(float value:{0.f,0.f,0.f,1.f,.7071067812f,0.f,0.f,.7071067812f}) {
        const auto at=source.data.size();source.data.resize(at+4);put(source.data,at,value);
    }
    const auto bytes=source.data;std::vector<fc::Pose> frames,reference;std::string error;
    expect(fc::decodeConverterSplineTransforms(source,frames,error),"disjoint quaternion components decode: "+error);
    auto plain=source;plain.data[2]=176;expect(fc::decodeHkxSplineTransforms(plain,reference,error),"independent unchanged decoder reference");
    expect(frames.size()==reference.size()&&source.data==bytes,"converter normalization does not alter the source");
    for(std::size_t frame=0;frame<frames.size();++frame) {
        expect(fc::angleBetween(frames[frame][0].q,reference[frame][0].q)<.000001f,"disjoint static component preserves quaternion controls");
        const auto t=float(frame)*.25f;const fc::Quat expected{.7071067812f*t,0,0,1-t+.7071067812f*t};
        expect(fc::angleBetween(frames[frame][0].q,expected.unit())<.000001f,"disjoint spline analytic quaternion oracle");
    }
    source.numFrames=9;source.numBlocks=2;source.blockOffsets.push_back(std::uint32_t(bytes.size()));source.data.insert(source.data.end(),bytes.begin(),bytes.end());
    expect(fc::decodeConverterSplineTransforms(source,frames,error)&&frames.size()==9,"each spline block normalizes its own masks");
    source.data[source.blockOffsets.back()+2]=177;
    expect(!fc::decodeConverterSplineTransforms(source,frames,error)&&frames.empty()&&!error.empty(),"overlapping quaternion components reject without partial frames");
    source.data[source.blockOffsets.back()+2]=180;source.blockOffsets.back()=std::uint32_t(source.data.size()-2);
    expect(!fc::decodeConverterSplineTransforms(source,frames,error)&&frames.empty(),"truncated block masks reject before mutation");
    source.blockOffsets.clear();expect(!fc::decodeConverterSplineTransforms(source,frames,error)&&frames.empty(),"missing block offset table rejects safely");
    hkx_fixture::Spline spline;spline.data=bytes;auto spec=hkx_fixture::procedural({28},5);const auto packed=hkx_fixture::pack(spec,&spline);auto legacy=packed.bytes;
    put(legacy,packed.animation+16,3u);fc::ConverterInputClip clip;expect(fc::decodeConverterInput(legacy,clip,error)&&clip.frames.size()==5,"legacy type 3 with the exact spline class decodes: "+error);
    put(legacy,packed.animation+16,1u);expect(!fc::decodeConverterInput(legacy,clip,error),"animation enum and actual class disagreement remains rejected");
    auto body=bytes;body.insert(body.begin()+4,4,0);const auto floatBegin=std::uint32_t(body.size());body.resize(body.size()+16,0);
    spline.frames=9;spline.blocks=2;spline.maskBytes=8;spline.blockOffsets={0,std::uint32_t(body.size())};spline.floatBlockOffsets={floatBegin,floatBegin};spline.data=body;spline.data.insert(spline.data.end(),body.begin(),body.end());
    spec=hkx_fixture::procedural({28},9);spec.duration=4;const auto floated=hkx_fixture::pack(spec,&spline);auto floatedBytes=floated.bytes;put(floatedBytes,floated.animation+28,1u);
    expect(fc::decodeConverterInput(floatedBytes,clip,error)&&clip.frames.size()==9&&clip.ignoredFloatTracks==1,"per-block relative float boundaries preserve two transform blocks: "+error);
    auto floatFail=[&](const Bytes& bad,const std::string& why){expect(!fc::decodeConverterInput(bad,clip,error)&&clip.frames.empty()&&!error.empty(),why);};
    for(unsigned count:{0u,1u}){auto bad=floatedBytes;put(bad,floated.animation+112,count);floatFail(bad,"missing float boundary table cannot be inferred");}
    const auto offsetData=pointerAt(floatedBytes,floated.animation+104);
    for(unsigned value:{4u,std::uint32_t(body.size()+4),floatBegin-1}){auto bad=floatedBytes;put(bad,offsetData,value);floatFail(bad,"float boundary must be aligned and inside its own block after masks");}
    auto second=floatedBytes;put(second,offsetData+4,std::uint32_t(body.size()+4));floatFail(second,"each block validates its relative float boundary independently");
    spline.transformOffsets={std::uint32_t(spline.data.size()+4)};const auto aux=hkx_fixture::pack(spec,&spline);auto outside=aux.bytes;put(outside,aux.animation+28,1u);floatFail(outside,"ignored spline auxiliary offsets remain bounded before stripping floats");
    source={};source.tracks=1;source.numFrames=5;source.numBlocks=1;source.maxFramesPerBlock=5;source.maskAndQuantizationSize=4;source.blockDuration=2;source.blockInverseDuration=.5f;source.frameDuration=.5f;source.blockOffsets={0};
    source.data={4,0,15,0,0x51,0xfa,0x58,0x47,0x3e,0,0,0};const fc::Quat sdk{.20518877827792997f,-.21555185029619486f,.5554605648115687f,.7764654055667326f};
    expect(fc::decodeConverterSplineTransforms(source,frames,error),"SDK 40-bit quaternion fixture decodes: "+error);
    for(const auto& frame:frames)expect(fc::angleBetween(frame[0].q,sdk)<.00001f,"40-bit quantization matches independent Havok SDK quaternion");
    source.numFrames=330;source.numBlocks=2;source.maxFramesPerBlock=256;source.frameDuration=1.f/60;source.blockDuration=255*source.frameDuration;source.blockInverseDuration=1/source.blockDuration;
    source.data={20,0,15,0};for(float value:{0.f,0.f,0.f,1.f}){const auto at=source.data.size();source.data.resize(at+4);put(source.data,at,value);}
    source.blockOffsets={0,std::uint32_t(source.data.size())};source.data.insert(source.data.end(),{20,0,15,0});
    for(float value:{0.f,0.f,.7071067812f,.7071067812f}){const auto at=source.data.size();source.data.resize(at+4);put(source.data,at,value);}
    expect(fc::decodeConverterSplineTransforms(source,frames,error)&&frames.size()==330,"standard overlapping blocks with distinct constant poses decode without guessing");
    for(unsigned frame:{0u,200u,254u})expect(fc::angleBetween(frames[frame][0].q,fc::Quat{})<.000001f,"first Havok block retains samples before its shared endpoint");
    for(unsigned frame:{255u,273u,329u})expect(fc::angleBetween(frames[frame][0].q,fc::Quat{0,0,.7071067812f,.7071067812f})<.000001f,"second Havok block begins at maxFramesPerBlock minus one");
    source.numFrames=512;expect(!fc::decodeConverterSplineTransforms(source,frames,error)&&frames.empty(),"separate nonoverlapping block convention is not silently substituted for Havok timing");
}
int dumpPoses(const std::filesystem::path& path) {
    fc::ConverterInputClip source;std::string error;if(!fc::decodeConverterInput(read(path),source,error))throw std::runtime_error(error);
    nlohmann::json frames=nlohmann::json::array();
    for(const auto& frame:source.frames) {
        nlohmann::json transforms=nlohmann::json::array();
        for(const auto& value:frame)transforms.push_back({value.t.x,value.t.y,value.t.z,value.q.x,value.q.y,value.q.z,value.q.w,value.s.x,value.s.y,value.s.z});
        frames.push_back(std::move(transforms));
    }
    std::cout<<nlohmann::json{{"duration",source.duration},{"tracks",source.boneIndices.size()},{"frames",std::move(frames)}}.dump()<<'\n';return 0;
}
int inspectBindings(const std::filesystem::path& path) {
    fc::ConverterInputClip source;std::string error;if(!fc::decodeConverterInput(read(path),source,error))throw std::runtime_error(error);
    nlohmann::json rows=nlohmann::json::array();
    for(std::size_t bone=5;bone<std::min(std::size_t{97},source.boneIndices.size());++bone) {
        const auto& rest=fc::canonicalBoneRest[bone];const fc::Vec target{rest[0],rest[1],rest[2]};float difference=0,variation=0;
        for(const auto& frame:source.frames){difference=std::max(difference,(frame[bone].t-target).length());variation=std::max(variation,(frame[bone].t-source.frames.front()[bone].t).length());}
        const auto t=source.frames.front()[bone].t;rows.push_back({{"bone",bone},{"name",fc::canonicalBoneNames[bone]},{"firstT",{t.x,t.y,t.z}},{"maxCanonicalDifference",difference},{"maxAnimatedVariation",variation}});
    }
    const auto encoded=path.u8string();const std::string utf8(reinterpret_cast<const char*>(encoded.data()),encoded.size());
    std::cout<<nlohmann::json{{"path",utf8},{"skeleton",source.skeletonName},{"tracks",source.boneIndices.size()},{"identity",source.identityMapping},{"frames",source.frames.size()},{"joints",rows}}.dump(2)<<'\n';return 0;
}
int run(const std::filesystem::path& manifest,const std::filesystem::path& native,const std::filesystem::path& parent) {
    splineCases();
    fc::ConverterInputClip movement;
    movement.annotations={{0,0,"SoundPlay.Test"},{0,0,"animmotionCustom 0 1 2"},{0,0,"note animrotation 90"}};
    expect(!fc::converterExternalMovement(movement),"unrelated events do not imply movement commands");
    movement.annotations.push_back({0,0,"animmotion 0 40 100"});expect(fc::converterExternalMovement(movement),"AMR displacement is distinguished from bone curves");
    movement.annotations={{0,0,"animrotation\t90"}};expect(fc::converterExternalMovement(movement),"AMR rotation is distinguished from bone curves");
    movement.annotations.clear();movement.referenceFrame.samples={{0,0,0,0},{0,40,100,0}};expect(fc::converterExternalMovement(movement),"extracted root samples are distinguished from bone curves");
    const auto outputs=parent/("run-"+std::to_string(std::chrono::high_resolution_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(parent);expect(std::filesystem::create_directory(outputs),"unique test output directory");fc::Library library;const auto loaded=fc::loadAnimationPack(library,manifest);expect(loaded.committed,"base pack is valid");const auto baseline=read(manifest);
    const auto pack=nlohmann::json::parse(baseline);std::map<std::string,std::pair<std::string,std::string>> paths;
    for(const auto& entry:pack.at("motions")) {const auto config=entry.at("config").get<std::string>();const auto path=std::filesystem::path(std::u8string(reinterpret_cast<const char8_t*>(config.data()),config.size()));const auto bytes=read(manifest.parent_path()/path);paths[entry.at("slot").get<std::string>()]={fc::selectAnimationClipConfig(nlohmann::json::parse(bytes),entry.at("slot").get<std::string>()).at("file").get<std::string>(),config};}
    std::size_t clips=0,events=0,referenceSamples=0;
    for(const auto motion:fc::activeMotions) {
        const auto slot=std::string(fc::motionSlotNames[int(motion)-1]);auto input=native/(slot+".hkx");
        if(std::filesystem::is_regular_file(native/"pack.json")){
            const auto doc=fc::loadConverterBaseEditor(native/"pack.json",slot);if(!std::filesystem::exists(input)||doc.templateConfig.contains("member")){std::vector<std::string> notes;input=outputs/(slot+"-source.hkx");write(input,fc::writeConverterHkx(doc.clip,doc.base,notes));}
        }
        const auto original=read(input);fc::ConverterInputClip source;std::string error;expect(fc::decodeConverterInput(original,source,error),"actual native input "+slot+": "+error);
        const auto sourceMotion=fc::inspectConverterSourceMotion(source);const bool bake=sourceMotion.present&&!sourceMotion.alreadyBaked;
        std::vector<std::string> warnings;const auto bytes=fc::writeConverterHkx(source,library,warnings);fc::HkxClip formal;expect(fc::decodeHkxAnimation(bytes,formal,error),"formal decoder "+slot+": "+error);
        expect(formal.duration==source.duration&&formal.frames.size()==source.frames.size()&&formal.boneIndices.size()==99,"exact source duration and frame count "+slot);
        for(std::size_t track=0;track<source.boneIndices.size();++track) {
            int bone=-1;const auto& name=source.trackNames[track];if(name.empty())bone=source.boneIndices[track];else for(int k=0;k<99;++k)if(normalized(name)==normalized(library.names[k])){bone=k;break;}
            if(bone<0||bone>=99)continue;
            for(std::size_t frame=0;frame<source.frames.size();++frame) {
                const auto protectedBone=(bone>=1&&bone<=3)||bone>=97;const auto originalRoot=bone==0&&bake?fc::compose(sourceMotion.samples[frame],source.frames[frame][track]):source.frames[frame][track];const auto expected=protectedBone?library.rest[bone].q:originalRoot.q.unit();
                expect(fc::angleBetween(formal.frames[frame][bone].q,expected)<.0001f,"retained canonical local rotation "+slot);
                expect((formal.frames[frame][bone].s-library.rest[bone].s).length()<.00001f,"fixed canonical scale");
                if(bone!=0&&bone!=4)expect((formal.frames[frame][bone].t-library.rest[bone].t).length()<.00001f,"fixed canonical joint length");
                else expect((formal.frames[frame][bone].t-originalRoot.t).length()<.00001f,bone==0&&bake?"source motion composed into Root exactly once":"unbaked Root and COM translations retained");
            }
        }
        fc::ConverterInputClip roundTrip;expect(fc::decodeConverterInput(bytes,roundTrip,error),"output extended reader "+error);expect(roundTrip.annotations.size()==source.annotations.size()+std::size_t(bake),"all annotation events and only the required bake marker retained");
        std::array<std::vector<std::pair<float,std::string>>,99> expectedEvents,actualEvents;
        for(const auto& event:source.annotations){int bone=0;if(!source.partialAnnotationNames){const auto& name=source.trackNames.at(event.track);if(name.empty())bone=source.boneIndices.at(event.track);else{bone=-1;for(int k=0;k<99;++k)if(normalized(name)==normalized(library.names[k])){bone=k;break;}}}if(bone<0||bone>=99)bone=0;expectedEvents[bone].push_back({event.time,event.text});}
        if(bake)expectedEvents[0].push_back({0,std::string(fc::converterSourceMotionMarker)});for(const auto& event:roundTrip.annotations){expect(event.track<99,"export annotation track is canonical");actualEvents[event.track].push_back({event.time,event.text});}
        expect(actualEvents==expectedEvents,"every annotation content, time, multiplicity and per-track order retained");
        expect(roundTrip.referenceFrame.samples==source.referenceFrame.samples,"reference root samples retained exactly");if(!source.referenceFrame.samples.empty())expect(roundTrip.referenceFrame.duration==source.referenceFrame.duration&&(roundTrip.referenceFrame.up-source.referenceFrame.up).length()<.00001f&&(roundTrip.referenceFrame.forward-source.referenceFrame.forward).length()<.00001f,"reference motion duration and basis retained");events+=source.annotations.size();referenceSamples+=source.referenceFrame.samples.size();
        std::vector<std::string> secondWarnings;const auto secondBytes=fc::writeConverterHkx(roundTrip,library,secondWarnings);fc::ConverterInputClip second;expect(fc::decodeConverterInput(secondBytes,second,error),"second conversion readable");expect(second.annotations.size()==roundTrip.annotations.size(),"second conversion adds no marker");
        for(std::size_t event=0;event<second.annotations.size();++event)expect(second.annotations[event].track==roundTrip.annotations[event].track&&second.annotations[event].time==roundTrip.annotations[event].time&&second.annotations[event].text==roundTrip.annotations[event].text,"second conversion retains exact annotation sequence");
        for(std::size_t frame=0;frame<second.frames.size();++frame)for(int bone=0;bone<99;++bone)expect((second.frames[frame][bone].t-roundTrip.frames[frame][bone].t).length()<.00001f&&fc::angleBetween(second.frames[frame][bone].q,roundTrip.frames[frame][bone].q)<.00001f,"second conversion does not reapply displacement or rotation");
        fc::ConverterRequest request{input,manifest,outputs/(slot+".zip"),{},slot,false};const auto report=fc::convertHkx(request);expect(report.at("ok")==true&&report.at("loaded")==fc::activeMotionCount,"full converted pack validation "+slot);
        const auto archive=unzip(request.output);const std::string prefix="meshes/actors/character/animations/FreeClimb/";
        expect(archive.contains(prefix+paths.at(slot).second),"selected metadata only");const auto outputConfig=fc::selectAnimationClipConfig(nlohmann::json::parse(archive.at(prefix+paths.at(slot).second)),slot);expect(archive.contains(prefix+outputConfig.at("file").get<std::string>()),"selected HKX only");
        expect(read(input)==original&&read(manifest)==baseline,"input and base manifest untouched");const auto published=read(request.output);bool collision=false;try{fc::convertHkx(request);}catch(const std::exception&){collision=true;}expect(collision&&read(request.output)==published,"existing ZIP is protected");
        if(slot=="up") {request.overwrite=true;expect(fc::convertHkx(request).at("ok")==true,"explicit overwrite succeeds");expect(read(request.output)==published,"repeated conversion is deterministic");}
        ++clips;
    }
    fc::ConverterInputClip layoutSource;std::string layoutError;expect(fc::decodeConverterInput(read(native/"up.hkx"),layoutSource,layoutError),"canonical layout fixture source");
    std::vector<std::string> layoutWarnings;const auto fixture=fc::writeConverterHkx(layoutSource,library,layoutWarnings);layoutCases(library,fixture);resourceCases(fixture);floatCases(library,fixture);
    auto request=fc::ConverterRequest{native/"up.hkx",manifest,outputs/"unknown.zip",{},"unknown",false};rejected(request,"unknown slot rejected");
    for(const auto removed:{"sprintCatch","flipUp","flipLeft","flipRight"}){request.slot=removed;request.output=outputs/(std::string(removed)+"-removed.zip");rejected(request,"removed slot cannot be converted or exported");expect(!std::filesystem::exists(request.output),"removed slot writes no override");}
    request.slot="up";request.output=manifest.parent_path()/"forbidden.zip";rejected(request,"base path is protected");
    const auto broken=outputs/"broken.hkx";{std::ofstream f(broken,std::ios::binary);f<<"not hkx";}request.input=broken;request.output=outputs/"broken.zip";rejected(request,"malformed input rejected");
    auto altered=read(native/"up.hkx");altered[16]=4;{std::ofstream f(broken,std::ios::binary|std::ios::trunc);f.write(reinterpret_cast<const char*>(altered.data()),altered.size());}rejected(request,"32-bit input rejected");
    auto partial=read(native/"up.hkx");std::size_t animation{};try{animation=objectAt(partial,"hkaSplineCompressedAnimation");}catch(const std::exception&){animation=objectAt(partial,"hkaInterleavedUncompressedAnimation");}
    const std::uint32_t one=1,capacity=0x80000001u;std::memcpy(partial.data()+animation+48,&one,sizeof(one));std::memcpy(partial.data()+animation+52,&capacity,sizeof(capacity));const auto partialPath=outputs/"partial.hkx";write(partialPath,partial);fc::ConverterInputClip partialClip;std::string partialError;
    expect(fc::decodeConverterInput(partial,partialClip,partialError)&&partialClip.partialAnnotationNames,"single event track is accepted independently of transform count");expect(std::all_of(partialClip.trackNames.begin(),partialClip.trackNames.end(),[](const auto& name){return name.empty();}),"partial event labels are not bone identities");
    request.input=partialPath;request.output=outputs/"partial.zip";const auto partialReport=fc::convertHkx(request);expect(partialReport.at("ok")==true&&!partialReport.at("warnings").empty(),"partial event track converts with explicit warning");
    auto additive=read(native/"up.hkx");additive[objectAt(additive,"hkaAnimationBinding")+64]=1;write(broken,additive);request.input=broken;request.output=outputs/"additive.zip";rejected(request,"additive animation rejected");
    auto floats=read(native/"up.hkx");std::memcpy(floats.data()+animation+28,&one,sizeof(one));write(broken,floats);request.output=outputs/"floats.zip";rejected(request,"declared float channel without its payload is rejected");
    fc::ConverterInputClip custom;std::string error;expect(fc::decodeConverterInput(read(native/"up.hkx"),custom,error),"custom case baseline");custom.trackNames[0]="Unknown Retarget Bone";std::vector<std::string> warnings;bool unsupported=false;try{fc::writeConverterHkx(custom,library,warnings);}catch(const std::exception&){unsupported=true;}expect(unsupported,"unknown body track does not silently retarget");
    const auto shared=outputs/"shared-base";std::filesystem::create_directory(shared);std::set<std::string> baseNames{"pack.json",pack.at("skeleton").get<std::string>()};
    for(const auto& [slot,path]:paths){baseNames.insert(path.first);baseNames.insert(path.second);}
    for(const auto& name:baseNames)write(shared/name,read(manifest.parent_path()/name));auto sharedConfig=nlohmann::json::parse(read(shared/paths.at("up").second));sharedConfig["file"]=paths.at("down").first;writeJson(shared/paths.at("up").second,sharedConfig);
    fc::Library sharedLibrary;expect(fc::loadAnimationPack(sharedLibrary,shared/"pack.json").committed,"two-slot shared-HKX fixture is valid");const auto sharedInput=read(shared/paths.at("down").first),sharedMetadata=read(shared/paths.at("up").second);
    request={native/"up.hkx",shared/"pack.json",outputs/"shared.zip",{},"up",false};const auto sharedReport=fc::convertHkx(request);const auto sharedArchive=unzip(request.output);
    expect(sharedArchive.contains("meshes/actors/character/animations/FreeClimb/converted/up.hkx"),"shared animation receives a distinct selected-slot file");const auto generatedConfig=nlohmann::json::parse(sharedArchive.at("meshes/actors/character/animations/FreeClimb/"+paths.at("up").second));expect(generatedConfig.at("file")=="converted/up.hkx","only target configuration selects the new file");
    expect(read(shared/paths.at("down").first)==sharedInput&&read(shared/paths.at("up").second)==sharedMetadata,"shared input pack remains untouched");expect(sharedReport.at("loaded")==fc::activeMotionCount,"shared target full-pack validation");
    auto aliasPack=pack;for(auto& entry:aliasPack.at("motions"))if(entry.at("slot")=="up")entry["config"]="configs//up.json";writeJson(shared/"pack.json",aliasPack);request.output=outputs/"alias.zip";rejected(request,"noncanonical relative path is rejected safely");writeJson(shared/"pack.json",pack);
#ifdef _WIN32
    sharedConfig["file"]="DOWN.HKX";writeJson(shared/paths.at("up").second,sharedConfig);request.output=outputs/"shared-case.zip";const auto caseReport=fc::convertHkx(request);expect(caseReport.at("ok")==true&&unzip(request.output).contains("meshes/actors/character/animations/FreeClimb/converted/up.hkx"),"case aliases use canonical shared-file identity");
#endif
    expect(read(manifest)==baseline,"base remains unchanged after failures");const auto folder=outputs.u8string();std::cout<<"pass: "<<clips<<" HKX clips, "<<events<<" events, "<<referenceSamples<<" root samples, "<<checks<<" assertions; output="<<std::string(reinterpret_cast<const char*>(folder.data()),folder.size())<<'\n';return 0;
}
}
#ifdef _WIN32
int wmain(int argc,wchar_t** argv) {try{if(argc==3&&std::wstring_view(argv[1])==L"--inspect-bindings")return inspectBindings(argv[2]);if(argc==3&&std::wstring_view(argv[1])==L"--dump-poses")return dumpPoses(argv[2]);if(argc!=4)throw std::runtime_error("Need pack.json native-directory output-directory");return run(argv[1],argv[2],argv[3]);}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
#else
int main(int argc,char** argv) {try{if(argc==3&&std::string_view(argv[1])=="--inspect-bindings")return inspectBindings(argv[2]);if(argc==3&&std::string_view(argv[1])=="--dump-poses")return dumpPoses(argv[2]);if(argc!=4)throw std::runtime_error("Need pack.json native-directory output-directory");return run(argv[1],argv[2],argv[3]);}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
#endif
