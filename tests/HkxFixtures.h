#pragma once
#include "Pose.h"
#include <cstring>
#include <filesystem>
#include <map>
#include <numeric>
#include <stdexcept>

namespace hkx_fixture {
using Bytes=std::vector<std::uint8_t>;
template<class T> void put(Bytes& b,std::size_t at,T value) {
    if(at+sizeof(T)>b.size())throw std::runtime_error("fixture write range");
    std::memcpy(b.data()+at,&value,sizeof(T));
}
template<class T> void append(Bytes& b,T value) {auto p=b.size();b.resize(p+sizeof(T));put(b,p,value);}
inline void align(Bytes& b,std::size_t n,std::uint8_t fill=0) {while(b.size()%n)b.push_back(fill);}
struct Spec {
    std::string skeleton="NPC Root [Root]";
    std::vector<int> indices{28};
    std::vector<std::string> names;
    std::vector<fc::Pose> frames;
    float duration=2;
    bool identityMap=false;
    unsigned blendHint=0;
};
struct Spline {
    std::uint32_t tracks=1,frames=5,blocks=1,maxFrames=5,maskBytes=4;
    float blockDuration=2,inverseDuration=.5f,frameDuration=.5f;
    std::vector<std::uint32_t> blockOffsets{0},floatBlockOffsets,transformOffsets,floatOffsets;
    Bytes data;
};
struct Packed {
    Bytes bytes;
    std::size_t payload{},binding{},animation{},transforms{},mapping{},annotations{},localFixups{},globalFixups{},classFixups{};
};
class Writer {
    Bytes data,classes;
    std::vector<std::array<std::uint32_t,2>> local;
    std::vector<std::array<std::uint32_t,3>> global,virtuals;
    std::map<std::string,std::uint32_t> classNames;
    std::size_t alloc(std::size_t n,std::size_t alignment=16) {align(data,alignment);auto p=data.size();data.resize(p+n);return p;}
    std::size_t string(std::string_view s) {auto p=alloc(s.size()+1,1);std::memcpy(data.data()+p,s.data(),s.size());return p;}
    void pointer(std::size_t src,std::size_t target,bool object=false) {
        if(object)global.push_back({std::uint32_t(src),2,std::uint32_t(target)});
        else local.push_back({std::uint32_t(src),std::uint32_t(target)});
    }
    void text(std::size_t src,std::string_view value) {pointer(src,string(value));}
    void array(std::size_t src,std::size_t target,std::size_t count) {
        put(data,src+8,std::uint32_t(count));put(data,src+12,std::uint32_t(count)|0x80000000u);
        if(count)pointer(src,target);
    }
    std::uint32_t className(const std::string& name,std::uint32_t signature) {
        if(auto i=classNames.find(name);i!=classNames.end())return i->second;
        append(classes,signature);classes.push_back(9);auto offset=std::uint32_t(classes.size());
        classes.insert(classes.end(),name.begin(),name.end());classes.push_back(0);classNames[name]=offset;return offset;
    }
    void object(std::size_t at,const std::string& name,std::uint32_t signature) {
        virtuals.push_back({std::uint32_t(at),0,className(name,signature)});
    }
    template<class T> void values(std::size_t src,const std::vector<T>& values) {
        auto at=alloc(values.size()*sizeof(T));
        if(!values.empty())std::memcpy(data.data()+at,values.data(),values.size()*sizeof(T));
        array(src,at,values.size());
    }
public:
    Packed build(const Spec& spec,const Spline* spline=nullptr) {
        Packed p;const std::size_t tracks=spec.indices.size();
        auto root=alloc(16),variants=alloc(24),container=alloc(96),animation=alloc(spline?176:88),binding=alloc(72);
        object(root,"hkRootLevelContainer",0x2772c11e);object(container,"hkaAnimationContainer",0x8dc20333);
        object(animation,spline?"hkaSplineCompressedAnimation":"hkaInterleavedUncompressedAnimation",spline?0x792ee0bb:0x930af031);
        object(binding,"hkaAnimationBinding",0x66eac971);
        array(root,variants,1);text(variants,"Original FreeClimb fixture");text(variants+8,"hkaAnimationContainer");pointer(variants+16,container,true);
        auto animations=alloc(8),bindings=alloc(8);
        array(container+16,0,0);array(container+32,animations,1);array(container+48,bindings,1);array(container+64,0,0);array(container+80,0,0);
        pointer(animations,animation,true);pointer(bindings,binding,true);
        text(binding+16,spec.skeleton);pointer(binding+24,animation,true);
        auto mapping=alloc(spec.indices.size()*2);p.mapping=mapping;
        for(std::size_t i=0;i<spec.indices.size();++i)put(data,mapping+2*i,std::int16_t(spec.indices[i]));
        array(binding+32,mapping,spec.identityMap?0:spec.indices.size());array(binding+48,0,0);put(data,binding+64,std::uint8_t(spec.blendHint));
        put(data,animation+16,std::uint32_t(spline?5:1));put(data,animation+20,spec.duration);put(data,animation+24,std::uint32_t(tracks));
        auto annotations=alloc(spec.names.size()*24);p.annotations=annotations;array(animation+40,annotations,spec.names.size());
        for(std::size_t i=0;i<spec.names.size();++i){text(annotations+24*i,spec.names[i]);array(annotations+24*i+8,0,0);}
        if(spline) {
            put(data,animation+56,spline->frames);put(data,animation+60,spline->blocks);put(data,animation+64,spline->maxFrames);put(data,animation+68,spline->maskBytes);
            put(data,animation+72,spline->blockDuration);put(data,animation+76,spline->inverseDuration);put(data,animation+80,spline->frameDuration);
            values(animation+88,spline->blockOffsets);values(animation+104,spline->floatBlockOffsets);values(animation+120,spline->transformOffsets);values(animation+136,spline->floatOffsets);values(animation+152,spline->data);
        } else {
            auto transforms=alloc(spec.frames.size()*tracks*48);p.transforms=transforms;array(animation+56,transforms,spec.frames.size()*tracks);array(animation+72,0,0);
            for(std::size_t f=0;f<spec.frames.size();++f) {
                if(spec.frames[f].size()!=tracks)throw std::runtime_error("fixture track size");
                for(std::size_t t=0;t<tracks;++t) {
                    const auto& v=spec.frames[f][t];const auto at=transforms+(f*tracks+t)*48;
                    put(data,at,v.t.x);put(data,at+4,v.t.y);put(data,at+8,v.t.z);
                    put(data,at+16,v.q.x);put(data,at+20,v.q.y);put(data,at+24,v.q.z);put(data,at+28,v.q.w);
                    put(data,at+32,v.s.x);put(data,at+36,v.s.y);put(data,at+40,v.s.z);
                }
            }
        }
        align(data,16);const auto localBegin=data.size();for(auto v:local)for(auto n:v)append(data,n);align(data,16,255);
        const auto globalBegin=data.size();for(auto v:global)for(auto n:v)append(data,n);align(data,16,255);
        const auto virtualBegin=data.size();for(auto v:virtuals)for(auto n:v)append(data,n);align(data,16,255);
        align(classes,16,255);Bytes file(208);const std::size_t classStart=208,dataStart=classStart+classes.size();
        put(file,0,0x57e0e057u);put(file,4,0x10c0c010u);put(file,12,8u);file[16]=8;file[17]=1;file[19]=1;put(file,20,3u);put(file,24,2u);
        put(file,36,classNames["hkRootLevelContainer"]);std::memcpy(file.data()+40,"hk_2010.2.0-r1",14);file[54]=file[55]=255;put(file,60,std::int16_t(-1));put(file,62,std::int16_t(-1));
        auto section=[&](int index,const char* name,std::size_t start,std::array<std::size_t,6> offsets) {
            const auto at=64+48*index;std::memcpy(file.data()+at,name,std::strlen(name));file[at+19]=255;put(file,at+20,std::uint32_t(start));
            for(int i=0;i<6;++i)put(file,at+24+i*4,std::uint32_t(offsets[i]));
        };
        section(0,"__classnames__",classStart,{classes.size(),classes.size(),classes.size(),classes.size(),classes.size(),classes.size()});
        section(1,"__types__",dataStart,{0,0,0,0,0,0});section(2,"__data__",dataStart,{localBegin,globalBegin,virtualBegin,data.size(),data.size(),data.size()});
        file.insert(file.end(),classes.begin(),classes.end());file.insert(file.end(),data.begin(),data.end());
        p.bytes=std::move(file);p.payload=dataStart;p.binding=dataStart+binding;p.animation=dataStart+animation;p.transforms+=dataStart;p.mapping+=dataStart;p.annotations+=dataStart;
        p.localFixups=dataStart+localBegin;p.globalFixups=dataStart+globalBegin;p.classFixups=dataStart+virtualBegin;return p;
    }
};
inline Packed pack(const Spec& spec,const Spline* spline=nullptr){return Writer{}.build(spec,spline);}
inline void save(const std::filesystem::path& path,const Bytes& bytes) {
    std::filesystem::create_directories(path.parent_path());std::ofstream out(path,std::ios::binary);out.write(reinterpret_cast<const char*>(bytes.data()),bytes.size());
    if(!out)throw std::runtime_error("fixture file output failed");
}
inline Spec procedural(std::vector<int> bones={28},std::size_t count=5) {
    Spec spec;spec.indices=std::move(bones);
    for(std::size_t f=0;f<count;++f){fc::Pose frame;for(int bone:spec.indices)frame.push_back({{float(bone),2,3},fc::Quat::axis({0,0,1},.1f+float(f)*.13f+bone*.001f),{1,1,1}});spec.frames.push_back(frame);}return spec;
}
}
