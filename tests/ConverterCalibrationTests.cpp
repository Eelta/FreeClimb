#include "../tools/converter/ConverterCalibration.h"
#include "AnimationPack.h"
#include "AnimationConfig.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>

using namespace fc;
using Json=nlohmann::json;
namespace {
unsigned checks{},identities{},warps{},rejections{};
void check(bool value,const std::string& message){++checks;if(!value)throw std::runtime_error(message);}
Json readJson(const std::filesystem::path& path){std::ifstream stream(path);return Json::parse(stream);}
ConverterInputClip input(const Clip& clip) {
    ConverterInputClip result;result.duration=clip.seconds;result.frames=clip.frames;return result;
}
Pose sample(const std::vector<Pose>& frames,float phase) {
    const float f=std::clamp(phase,0.f,1.f)*float(frames.size()-1);
    const auto a=std::min(std::size_t(f),frames.size()-2);auto pose=frames[a];
    for(std::size_t i=0;i<pose.size();++i)pose[i]=blend(pose[i],frames[a+1][i],f-float(a));
    return pose;
}
float mapped(const ConverterCalibration& result,float phase) {
    const float f=phase*float(result.phaseMap.size()-1);const auto a=std::min(std::size_t(f),result.phaseMap.size()-2);
    return result.phaseMap[a][1]+(result.phaseMap[a+1][1]-result.phaseMap[a][1])*(f-float(a));
}
float warp(float phase){return phase+.15f*std::sin(3.14159265359f*phase);}
float inverseWarp(float phase) {
    float low=0,high=1;
    for(unsigned i=0;i<28;++i){const float middle=(low+high)*.5f;if(warp(middle)<phase)low=middle;else high=middle;}
    return (low+high)*.5f;
}
void fixedGeometry(const Json& actual,const Json& original) {
    for(const auto key:{"format","version","slot","file","stride","height","travel"})
        check(actual.at(key)==original.at(key),std::string("automatic timing preserves fixed field ")+key);
    if(original.contains("path")) {
        check(actual.at("path").size()==original.at("path").size(),"timing keeps every route knot");
        for(std::size_t i=0;i<original.at("path").size();++i)for(int component=1;component<4;++component)
            check(actual.at("path")[i][component]==original.at("path")[i][component],"timing never changes route geometry");
    }
}
void validMetadata(const Library& base,std::string_view slot,const ConverterCalibration& result,std::size_t frames) {
    const auto& contacts=result.config.at("contacts");check(contacts.size()==frames,"contacts follow edited frame count");
    for(const auto& row:contacts) {
        check(row.is_array()&&row.size()==4,"all four limb tracks remain present");
        for(const auto& value:row){const auto weight=value.get<float>();check(std::isfinite(weight)&&weight>=0&&weight<=1,"all contact weights remain bounded");}
    }
    check(result.phaseMap.size()==65&&result.phaseMap.front()==std::array<float,2>{0,0}&&result.phaseMap.back()==std::array<float,2>{1,1},"mapping preserves exact playback endpoints");
    for(std::size_t i=1;i<result.phaseMap.size();++i)
        check(std::isfinite(result.phaseMap[i][1])&&result.phaseMap[i][1]>result.phaseMap[i-1][1],"mapping cannot reverse or freeze animation time");
    auto profile=base.threepeatProfile;
    const auto window=[](const Json& value){return std::array<float,2>{value.at(0).get<float>(),value.at(1).get<float>()};};
    if(slot=="contextHopLeft"||slot=="contextHopRight") {
        const int side=slot=="contextHopLeft"?0:1;const auto& path=result.config.at("path");
        profile.pathCounts[side]=std::uint32_t(path.size());
        for(std::size_t i=0;i<path.size();++i)profile.paths[side][i]={path[i][0].get<float>(),path[i][1].get<float>(),path[i][2].get<float>(),path[i][3].get<float>()};
        for(int hand=0;hand<2;++hand){profile.source[side][hand]=window(result.config.at("sourceHands")[hand]);profile.target[side][hand]=window(result.config.at("targetHands")[hand]);}
        profile.rise[side]=window(result.config.at("verticalBlend"));
    } else if(slot=="contextMantle") {
        for(int hand=0;hand<2;++hand)profile.mantleRelease[hand]=window(result.config.at("releaseHands")[hand]);
        profile.mantleUnplant=window(result.config.at("unplant"));profile.mantleReplant=window(result.config.at("replant"));
        profile.replantSamplePhase=result.config.at("replantSamplePhase").get<float>();
    }
    check(validThreepeatProfile(profile),"remapped special action satisfies the actual runtime validator");
}
void identity(const Library& base,const std::map<std::string,Json>& configs) {
    for(const auto motion:activeMotions) {
        const auto slot=std::string(motionSlotNames[int(motion)-1]);const auto& reference=base.clip(motion);const auto& config=configs.at(slot);
        const auto result=calibrateConverterClip(base,slot,input(reference),config);
        check(result.confidence>.99999f,"identical real animation has full matching confidence: "+slot);
        fixedGeometry(result.config,config);validMetadata(base,slot,result,reference.frames.size());
        for(unsigned i=0;i<=128;++i)check(std::abs(mapped(result,i/128.f)-i/128.f)<.000001f,"identity retains animation time: "+slot);
        const auto& before=config.at("contacts");const auto& after=result.config.at("contacts");
        for(std::size_t i=0;i<after.size();++i) {
            const float at=float(i)*float(before.size()-1)/float(after.size()-1);const auto a=std::min(std::size_t(at),before.size()-2);
            for(int limb=0;limb<4;++limb){const float low=before[a][limb].get<float>(),high=before[a+1][limb].get<float>();
                check(std::abs(after[i][limb].get<float>()-(low+(high-low)*(at-float(a))))<.00001f,"identity preserves interpolated actual contact track: "+slot);}
        }
        for(const auto key:{"sourceHands","targetHands","releaseHands","verticalBlend","unplant","replant","replantSamplePhase","path"})
            if(config.contains(key)) {
                if(key!=std::string_view("path")) {
                    const auto near=[](const auto& self,const Json& a,const Json& b)->bool {
                        if(a.is_array()){if(!b.is_array()||a.size()!=b.size())return false;for(std::size_t i=0;i<a.size();++i)if(!self(self,a[i],b[i]))return false;return true;}
                        return a.is_number()&&b.is_number()&&std::abs(a.get<double>()-b.get<double>())<.000001;
                    };
                    check(near(near,result.config.at(key),config.at(key)),"identity preserves special support timing: "+slot);
                }
            }
        ++identities;
    }
}
void temporalWarp(const Library& base,const std::map<std::string,Json>& configs) {
    for(const auto slot:{"contextMantle","contextHopLeft","contextHopRight"}) {
        const auto found=std::find(motionSlotNames.begin(),motionSlotNames.end(),std::string_view(slot));const auto& reference=base.clips[std::size_t(found-motionSlotNames.begin())];
        auto edited=input(reference);edited.frames.clear();
        for(unsigned i=0;i<=160;++i)edited.frames.push_back(sample(reference.frames,warp(i/160.f)));
        const auto result=calibrateConverterClip(base,slot,edited,configs.at(slot));
        fixedGeometry(result.config,configs.at(slot));validMetadata(base,slot,result,edited.frames.size());
        double error=0,baseline=0;
        for(unsigned i=1;i<128;++i){const float phase=i/128.f,expected=warp(phase);error+=std::pow(mapped(result,phase)-expected,2);baseline+=std::pow(phase-expected,2);}
        const auto rms=std::sqrt(error/127),initial=std::sqrt(baseline/127);
        std::cout<<slot<<" warp RMS="<<rms<<" identity="<<initial<<" confidence="<<result.confidence<<'\n';
        check(result.confidence>=.55f&&rms<initial*.85,"real nonuniform retiming improves known phase correspondence: "+std::string(slot));
        error=baseline=0;unsigned markers=0;
        for(const auto key:{"sourceHands","targetHands","releaseHands","verticalBlend","unplant","replant"})if(configs.at(slot).contains(key)) {
            const auto& original=configs.at(slot).at(key);const auto& actual=result.config.at(key);
            const auto accumulate=[&](const Json& a,const Json& b) {for(int i=0;i<2;++i){const float phase=a[i].get<float>(),expected=inverseWarp(phase);error+=std::pow(b[i].get<float>()-expected,2);baseline+=std::pow(phase-expected,2);++markers;}};
            if(original[0].is_array())for(std::size_t i=0;i<original.size();++i)accumulate(original[i],actual[i]);else accumulate(original,actual);
        }
        check(markers>0&&error<baseline*.85,"hand release and acquisition timing improves after real retiming: "+std::string(slot));++warps;
    }
}
template<class Function> void rejects(Function function,const char* message) {
    bool rejected=false;try{function();}catch(const std::exception&){rejected=true;}
    check(rejected,message);++rejections;
}
void negative(const Library& base,const std::map<std::string,Json>& configs) {
    const std::string slot="contextMantle";const auto& config=configs.at(slot);const auto original=input(base.clip(Motion::contextMantle));
    auto empty=original;empty.frames.clear();rejects([&]{calibrateConverterClip(base,slot,empty,config);},"empty source cannot calibrate");
    auto one=original;one.frames.resize(1);rejects([&]{calibrateConverterClip(base,slot,one,config);},"single frame source cannot calibrate");
    auto shortPose=original;shortPose.frames[1].resize(98);rejects([&]{calibrateConverterClip(base,slot,shortPose,config);},"noncanonical frame cannot calibrate");
    auto nan=original;nan.frames[0][28].q.x=std::numeric_limits<float>::quiet_NaN();rejects([&]{calibrateConverterClip(base,slot,nan,config);},"nonfinite source rotation cannot calibrate");
    auto infinity=original;infinity.frames[0][4].t.z=std::numeric_limits<float>::infinity();rejects([&]{calibrateConverterClip(base,slot,infinity,config);},"nonfinite source position cannot calibrate");
    auto zero=original;zero.frames[0][29].q={0,0,0,0};rejects([&]{calibrateConverterClip(base,slot,zero,config);},"zero source quaternion cannot silently become an identity rotation");
    auto nonunit=original;nonunit.frames[0][29].q={0,0,0,2};rejects([&]{calibrateConverterClip(base,slot,nonunit,config);},"nonunit source quaternion cannot distort timing features");
    auto wrongBase=base;wrongBase.parents.resize(98);rejects([&]{calibrateConverterClip(wrongBase,slot,original,config);},"noncanonical base cannot calibrate");
    auto absent=base;absent.clips[int(Motion::contextMantle)-1].frames.clear();rejects([&]{calibrateConverterClip(absent,slot,original,config);},"missing reference cannot calibrate");
    auto parentLoop=base;parentLoop.parents[28]=28;rejects([&]{calibrateConverterClip(parentLoop,slot,original,config);},"self-parent cannot enter unchecked forward kinematics");
    auto shortReference=base;shortReference.clips[int(Motion::contextMantle)-1].frames[0].resize(98);rejects([&]{calibrateConverterClip(shortReference,slot,original,config);},"noncanonical reference frame cannot enter feature sampling");
    auto badReference=base;badReference.clips[int(Motion::contextMantle)-1].frames[0][29].q={0,0,0,0};rejects([&]{calibrateConverterClip(badReference,slot,original,config);},"invalid reference quaternion cannot calibrate");
    rejects([&]{calibrateConverterClip(base,"unregistered",original,config);},"unknown slot cannot calibrate");
    auto missing=config;missing.erase("contacts");rejects([&]{calibrateConverterClip(base,slot,original,missing);},"missing contact data cannot calibrate");
    auto badContacts=config;badContacts["contacts"]=Json::array({Json::array({1,1,0,0})});rejects([&]{calibrateConverterClip(base,slot,original,badContacts);},"single contact row cannot calibrate");
    auto negativeContact=config;negativeContact["contacts"][0][0]=-.1f;rejects([&]{calibrateConverterClip(base,slot,original,negativeContact);},"negative contact weight cannot be silently clamped");
    auto excessiveContact=config;excessiveContact["contacts"][0][0]=1.1f;rejects([&]{calibrateConverterClip(base,slot,original,excessiveContact);},"excessive contact weight cannot be silently clamped");
    auto nanContact=config;nanContact["contacts"][0][0]=std::numeric_limits<float>::quiet_NaN();rejects([&]{calibrateConverterClip(base,slot,original,nanContact);},"nonfinite contact weight cannot reach serialized metadata");
    auto extraContact=config;extraContact["contacts"][0].push_back(1);rejects([&]{calibrateConverterClip(base,slot,original,extraContact);},"extra contact dimensions cannot be silently discarded");
    auto constant=original;constant.frames.assign(original.frames.size(),base.rest);
    const auto result=calibrateConverterClip(base,slot,constant,config);
    check(result.confidence<.55f&&!result.warnings.empty(),"unrelated constant standing pose clearly warns and falls back");
    for(const auto& row:result.phaseMap)check(row[0]==row[1],"constant pose cannot invent support timing");
    fixedGeometry(result.config,config);validMetadata(base,slot,result,constant.frames.size());
}
}
int main(int argc,char** argv) {
    try {
        check(argc==2,"arguments: pack.json");const std::filesystem::path pack=argv[1];Library base;
        const auto report=loadAnimationPack(base,pack);check(report.committed,"load actual animation pack: "+report.error);
        const auto manifest=readJson(pack);std::map<std::string,Json> configs;
        for(const auto& item:manifest.at("motions"))configs.emplace(item.at("slot").get<std::string>(),selectAnimationClipConfig(readJson(pack.parent_path()/item.at("config").get<std::string>()),item.at("slot").get<std::string>()));
        identity(base,configs);temporalWarp(base,configs);negative(base,configs);
        std::cout<<checks<<" calibration checks, "<<identities<<" real identity slots, "<<warps<<" temporal warps, "<<rejections<<" invalid input rejections passed\n";return 0;
    } catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
