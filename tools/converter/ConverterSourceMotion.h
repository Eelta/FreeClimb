#pragma once
#include "ConverterInput.h"
#include <charconv>
#include <numbers>
#include <optional>

namespace fc {
inline constexpr std::string_view converterSourceMotionMarker="FreeClimb.SourceMotionBaked:v1";
struct ConverterSourceMotion {
    bool present{},alreadyBaked{},amrTranslation{},amrRotation{},referenceTranslation{},referenceRotation{};
    std::size_t translationKeys{},rotationKeys{},referenceSamples{};
    std::vector<Transform> samples;
    std::vector<std::string> warnings;
};
namespace converter_source_motion {
inline void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
inline bool space(char c){return c==' '||c=='\t'||c=='\r'||c=='\n';}
inline std::string_view token(std::string_view& text){
    while(!text.empty()&&space(text.front()))text.remove_prefix(1);
    const auto end=text.find_first_of(" \t\r\n");const auto result=text.substr(0,end);text.remove_prefix(result.size());return result;
}
inline float number(std::string_view& text,float limit){
    auto part=token(text);require(!part.empty(),"Source motion annotation has missing values");
    if(part.front()=='+')part.remove_prefix(1);require(!part.empty(),"Source motion annotation has an invalid number");
    float value{};const auto parsed=std::from_chars(part.data(),part.data()+part.size(),value);
    require(parsed.ec==std::errc{}&&parsed.ptr==part.data()+part.size()&&std::isfinite(value)&&std::abs(value)<=limit,"Source motion annotation contains an invalid or unsupported value");return value;
}
template<class T>struct Key {float time{};T value{};};
inline bool equal(Vec a,Vec b){return (a-b).length()<=.0001f;}
inline bool equal(Quat a,Quat b){return angleBetween(a,b)<=.0001f;}
template<class T>inline void order(std::vector<Key<T>>& keys){
    std::stable_sort(keys.begin(),keys.end(),[](const auto& a,const auto& b){return a.time<b.time;});
    std::size_t at=0;for(const auto& key:keys){if(at&&key.time==keys[at-1].time)require(equal(key.value,keys[at-1].value),"Source motion contains conflicting values at the same time");else keys[at++]=key;}keys.resize(at);
}
inline Vec interpolate(Vec a,Vec b,float t){return a+(b-a)*t;}
inline Quat interpolate(Quat a,Quat b,float t){return blend(a,b,t);}
template<class T>inline T sample(const std::vector<Key<T>>& keys,float seconds){
    if(keys.empty())return T{};const auto next=std::upper_bound(keys.begin(),keys.end(),seconds,[](float time,const auto& key){return time<key.time;});
    if(next==keys.end())return keys.back().value;
    const float begin=next==keys.begin()?0:std::prev(next)->time;const T first=next==keys.begin()?T{}:std::prev(next)->value;
    return interpolate(first,next->value,std::clamp((seconds-begin)/(next->time-begin),0.f,1.f));
}
inline std::array<float,4> reference(const ConverterReferenceFrame& data,float seconds){
    const float at=std::clamp(seconds/data.duration,0.f,1.f)*float(data.samples.size()-1);const auto a=std::min(std::size_t(at),data.samples.size()-2);std::array<float,4> result{};
    for(int axis=0;axis<4;++axis)result[axis]=std::lerp(data.samples[a][axis],data.samples[a+1][axis],at-float(a));return result;
}
}
inline bool converterSourceMotionBaked(const ConverterInputClip& clip){
    return std::any_of(clip.annotations.begin(),clip.annotations.end(),[](const auto& item){return item.text==converterSourceMotionMarker;});
}
inline ConverterSourceMotion inspectConverterSourceMotion(const ConverterInputClip& clip){
    using namespace converter_source_motion;ConverterSourceMotion result;
    require(clip.annotations.size()<=8192,"Source motion annotation count exceeds its limit");
    for(const auto& event:clip.annotations)if(std::string_view(event.text).starts_with("FreeClimb.SourceMotionBaked:")){
        require(event.text==converterSourceMotionMarker,"Unsupported source motion bake marker");result.alreadyBaked=true;
    }
    if(result.alreadyBaked){result.present=true;return result;}
    std::vector<Key<Vec>> translations;std::vector<Key<Quat>> rotations;std::optional<std::size_t> track;
    for(const auto& event:clip.annotations){
        std::string_view text=event.text;const auto command=token(text);if(command!="animmotion"&&command!="animrotation")continue;
        require(event.text.size()<=256,"Source motion annotation text exceeds its limit");
        require(std::isfinite(clip.duration)&&clip.duration>0&&std::isfinite(event.time)&&event.time>=-.00005f&&event.time<=clip.duration+.00005f,"Source motion annotation time lies outside the animation");const float time=std::clamp(event.time,0.f,clip.duration);
        require(!track||*track==event.track,"Source motion annotations occur on multiple tracks; select one motion track before conversion");track=event.track;
        if(command=="animmotion"){Vec value;value.x=number(text,2000);value.y=number(text,2000);value.z=number(text,2000);translations.push_back({time,value});}
        else rotations.push_back({time,Quat::axis({0,0,1},number(text,3600)*std::numbers::pi_v<float>/180)});
        require(token(text).empty(),"Source motion annotation contains extra values");
    }
    const auto& extracted=clip.referenceFrame;result.present=!translations.empty()||!rotations.empty()||!extracted.samples.empty();if(!result.present)return result;
    require(std::isfinite(clip.duration)&&clip.duration>0&&clip.duration<=10&&clip.frames.size()>=2&&clip.frames.size()<=1201,"Source motion requires a valid bounded animation duration and frame count");
    order(translations);order(rotations);result.translationKeys=translations.size();result.rotationKeys=rotations.size();result.referenceSamples=extracted.samples.size();
    result.amrTranslation=!translations.empty();result.amrRotation=!rotations.empty();
    if(!extracted.samples.empty()){
        require(extracted.samples.size()>=2&&extracted.samples.size()<=1201&&std::isfinite(extracted.duration)&&std::abs(extracted.duration-clip.duration)<=.00001f,"Extracted source motion has inconsistent duration or sample count");
        require(extracted.up.finite()&&extracted.forward.finite()&&std::abs(extracted.up.length()-1)<.0001f&&std::abs(extracted.forward.length()-1)<.0001f&&std::abs(extracted.up.dot(extracted.forward))<.0001f,"Extracted source motion basis is invalid");
        for(const auto& row:extracted.samples)for(int axis=0;axis<4;++axis)require(std::isfinite(row[axis])&&std::abs(row[axis])<=(axis==3?6.283186f:2000.f),"Extracted source motion exceeds its supported range");
        result.referenceTranslation=!result.amrTranslation;result.referenceRotation=!result.amrRotation;
        if(result.amrTranslation||result.amrRotation)result.warnings.push_back("AMR translation/rotation takes priority over the matching extracted-root channel; the two sources are not added together.");
    }
    const auto at=[&](float seconds){
        Transform value;const auto row=extracted.samples.empty()?std::array<float,4>{}:reference(extracted,seconds);
        value.t=result.amrTranslation?sample(translations,seconds):Vec{row[0],row[1],row[2]};
        value.q=result.amrRotation?sample(rotations,seconds):Quat::axis(extracted.samples.empty()?Vec{0,0,1}:extracted.up,row[3]);return value;
    };
    const auto origin=at(0);result.samples.reserve(clip.frames.size());
    for(std::size_t frame=0;frame<clip.frames.size();++frame){auto value=at(clip.duration*float(frame)/float(clip.frames.size()-1));value.t=value.t-origin.t;value.q=(origin.q.inverse()*value.q).unit();result.samples.push_back(value);}
    return result;
}
inline ConverterSourceMotion bakeConverterSourceMotion(ConverterInputClip& clip){
    using namespace converter_source_motion;auto result=inspectConverterSourceMotion(clip);if(!result.present||result.alreadyBaked)return result;
    require(clip.boneIndices.size()==99,"Source motion baking requires the canonical 99-bone layout");
    for(std::size_t bone=0;bone<99;++bone)require(clip.boneIndices[bone]==int(bone),"Source motion baking requires identity canonical bone indices");
    require(clip.annotations.size()<8192,"Source motion bake marker would exceed the annotation limit");
    auto frames=clip.frames;for(std::size_t frame=0;frame<frames.size();++frame){
        require(frames[frame].size()==99,"Source motion frame has an invalid canonical bone count");auto& root=frames[frame][0];
        require(root.t.finite()&&root.s.finite()&&std::isfinite(root.q.dot(root.q))&&std::abs(root.q.dot(root.q)-1)<.01f,"Source root transform is invalid");
        root=compose(result.samples[frame],root);require(root.t.finite()&&root.t.length()<100000,"Baked source root displacement exceeds its supported range");
    }
    std::vector<std::vector<Quat>> rotations(frames.size());for(std::size_t frame=0;frame<frames.size();++frame){rotations[frame].reserve(99);for(const auto& bone:frames[frame])rotations[frame].push_back(bone.q);}
    auto annotations=clip.annotations;annotations.push_back({0,0,std::string(converterSourceMotionMarker)});
    clip.frames=std::move(frames);clip.rotations=std::move(rotations);clip.annotations=std::move(annotations);return result;
}
}
