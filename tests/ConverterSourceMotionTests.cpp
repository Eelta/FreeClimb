#include "../tools/converter/ConverterSourceMotion.h"
#include "../external/nlohmann/json.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>

namespace {
std::size_t checks{};
void expect(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
fc::ConverterInputClip clip(){
    fc::ConverterInputClip result;result.duration=2;result.frames.assign(5,fc::Pose(99));result.boneIndices.resize(99);
    for(int bone=0;bone<99;++bone)result.boneIndices[bone]=bone;return result;
}
void near(fc::Vec actual,fc::Vec expected,const char* message){expect((actual-expected).length()<.0002f,message);}
void rejected(fc::ConverterInputClip input,const char* message){
    const auto frames=input.frames;const auto annotations=input.annotations.size();bool failed=false;
    try{fc::bakeConverterSourceMotion(input);}catch(const std::runtime_error&){failed=true;}expect(failed,message);expect(input.annotations.size()==annotations,"Rejected bake keeps annotations");
    expect(input.frames.size()==frames.size(),"Rejected bake keeps frame count");for(std::size_t frame=0;frame<frames.size();++frame)for(std::size_t bone=0;bone<frames[frame].size();++bone){near(input.frames[frame][bone].t,frames[frame][bone].t,"Rejected bake keeps translations");expect(fc::angleBetween(input.frames[frame][bone].q,frames[frame][bone].q)<.00001f,"Rejected bake keeps rotations");}
}
}
int main(int argc,char** argv){try{
    auto empty=clip();empty.annotations={{0,0,"SoundPlay.Test"},{1,0,"animmotionCustom 10 20 30"}};const auto noMotion=fc::bakeConverterSourceMotion(empty);
    expect(!noMotion.present&&empty.annotations.size()==2&&empty.rotations.empty(),"Motionless input remains completely unchanged");
    auto translated=clip();translated.annotations={{2,0,"animmotion 10 40 100"},{1,0,"animmotion +2\t+10 20"}};
    translated.frames[0][0].t={1,2,3};const auto translation=fc::bakeConverterSourceMotion(translated);
    expect(translation.amrTranslation&&translation.translationKeys==2,"AMR keys are parsed and sorted");near(translation.samples[1].t,{1,5,10},"Late first key interpolates from zero");near(translated.frames[4][0].t,{10,40,100},"Translation reaches authored endpoint");near(translated.frames[0][0].t,{1,2,3},"Existing root offset is retained");
    expect(translated.annotations.size()==3&&fc::converterSourceMotionBaked(translated),"Bake preserves annotations and records marker");
    const auto preserved=translated.frames;const auto repeated=fc::bakeConverterSourceMotion(translated);expect(repeated.alreadyBaked&&translated.annotations.size()==3,"Repeated bake is idempotent");for(std::size_t frame=0;frame<5;++frame)near(translated.frames[frame][0].t,preserved[frame][0].t,"Repeated bake does not double translation");
    auto combined=clip();for(auto& frame:combined.frames)frame[0].t={0,2,0};combined.annotations={{0,0,"animmotion 10 20 30"},{2,0,"animmotion 20 40 60"},{0,0,"animrotation 45"},{2,0,"animrotation -45"}};
    const auto mixed=fc::bakeConverterSourceMotion(combined);near(mixed.samples.back().t,{10,20,30},"Position uses independent source origin");near(combined.frames.back()[0].t,{12,20,30},"Negative yaw rotates existing root and then adds translation");expect(fc::angleBetween(combined.frames.back()[0].q,fc::Quat::axis({0,0,1},-std::numbers::pi_v<float>/2))<.00001f,"Rotation uses independent source origin");
    auto shortest=clip();shortest.annotations={{0,0,"animrotation 170"},{2,0,"animrotation -170"}};const auto arc=fc::bakeConverterSourceMotion(shortest);expect(fc::angleBetween(arc.samples[2].q,fc::Quat::axis({0,0,1},10*std::numbers::pi_v<float>/180))<.00001f,"AMR rotation follows the shortest quaternion arc");
    auto reference=clip();reference.referenceFrame={2,{0,0,1},{0,1,0},{{10,20,30,.5f},{30,60,130,1.5f}}};const auto extracted=fc::bakeConverterSourceMotion(reference);
    expect(extracted.referenceTranslation&&extracted.referenceRotation,"Extracted motion supplies both channels");near(extracted.samples[2].t,{10,20,50},"Extracted position interpolates and rebases");expect(fc::angleBetween(extracted.samples[4].q,fc::Quat::axis({0,0,1},1))<.00001f,"Extracted rotation interpolates and rebases");
    auto priority=clip();priority.referenceFrame={2,{0,0,1},{0,1,0},{{0,0,0,0},{100,200,300,1}}};priority.annotations={{2,0,"animmotion 1 2 3"}};const auto preferred=fc::bakeConverterSourceMotion(priority);
    near(preferred.samples.back().t,{1,2,3},"AMR replaces matching extracted channel without double counting");expect(preferred.referenceRotation&&!preferred.referenceTranslation&&!preferred.warnings.empty(),"Nonoverridden rotation and explicit source-priority notice retained");
    auto duplicate=clip();duplicate.annotations={{0,0,"animmotion 0 0 0"},{2,0,"animmotion 1 2 3"},{2,0,"animmotion 1 2 3"}};expect(fc::bakeConverterSourceMotion(duplicate).translationKeys==2,"Identical duplicate keys collapse safely");
    for(const std::string text:{"animmotion 0 0","animmotion 0 0 0 0","animmotion nan 0 0","animmotion inf 0 0","animmotion 2001 0 0","animmotion 1e99 0 0","animrotation -3601","animrotation 90oops","animrotation 90 0"}){auto invalid=clip();invalid.annotations={{1,0,text}};rejected(invalid,"Malformed or out-of-range movement is rejected");}
    auto invalid=clip();invalid.annotations={{1,0,"animmotion 1 2 3"},{1,0,"animmotion 1 2 4"}};rejected(invalid,"Conflicting duplicate translation is rejected");
    invalid=clip();invalid.annotations={{1,0,"animrotation 30"},{1,0,"animrotation 40"}};rejected(invalid,"Conflicting duplicate rotation is rejected");
    invalid=clip();invalid.annotations={{1,0,"animmotion 1 2 3"},{1,1,"animrotation 40"}};rejected(invalid,"Ambiguous multiple motion tracks are rejected");
    for(float time:{-1.f,-.0001f,2.0001f,2.1f,std::numeric_limits<float>::infinity()}){invalid=clip();invalid.annotations={{time,0,"animmotion 1 2 3"}};rejected(invalid,"Out-of-clip annotation time is rejected");}
    auto rounded=clip();rounded.duration=1.466666698f;rounded.annotations={{1.466699958f,0,"animmotion 1 20 70"}};const auto roundedTime=rounded.annotations[0].time;fc::bakeConverterSourceMotion(rounded);near(rounded.frames.back()[0].t,{1,20,70},"Four-decimal boundary rounding reaches the source endpoint");expect(rounded.annotations[0].time==roundedTime,"Source boundary annotation is preserved during baking");
    invalid=clip();invalid.annotations={{2,0,"animmotion 1 2 3"},{2.00003f,0,"animmotion 1 2 4"}};rejected(invalid,"Rounded boundary cannot conceal conflicting endpoint values");
    invalid=clip();invalid.annotations={{0,0,"FreeClimb.SourceMotionBaked:v2"}};rejected(invalid,"Unknown bake marker is rejected");
    invalid=clip();invalid.annotations={{1,0,"animmotion 1 2 3"}};invalid.frames.clear();rejected(invalid,"Empty motion-bearing clip is rejected");
    invalid=clip();invalid.referenceFrame={2,{0,0,1},{0,1,0},{{0,0,0,0}}};rejected(invalid,"Single extracted sample is rejected");
    invalid=clip();invalid.referenceFrame={1,{0,0,1},{0,1,0},{{0,0,0,0},{1,2,3,0}}};rejected(invalid,"Extracted duration mismatch is rejected");
    invalid=clip();invalid.referenceFrame={2,{0,0,0},{0,1,0},{{0,0,0,0},{1,2,3,0}}};rejected(invalid,"Invalid extracted basis is rejected");
    invalid=clip();invalid.referenceFrame={2,{0,0,1},{0,1,0},{{0,0,0,0},{1,2,3,7}}};rejected(invalid,"Unsupported extracted angle is rejected");
    if(argc>1){
        std::ifstream input(std::filesystem::u8path(argv[1]),std::ios::binary);const std::vector<std::uint8_t> bytes(std::istreambuf_iterator<char>(input),{});fc::ConverterInputClip source;std::string error;expect(fc::decodeConverterInput(bytes,source,error),error.c_str());const auto report=fc::inspectConverterSourceMotion(source);
        expect(report.amrTranslation&&report.translationKeys==75&&report.referenceSamples==75,"ClimbMedium AMR and extracted source identified");near(report.samples.back().t,{1.331f,47.034f,152.132f},"ClimbMedium full source movement retained");
        const auto first=source.frames.front()[0].t,last=source.frames.back()[0].t;fc::bakeConverterSourceMotion(source);near(source.frames.back()[0].t,last+report.samples.back().t,"ClimbMedium root receives source endpoint exactly");
        nlohmann::json value={{"input",argv[1]},{"sourceFrameCount",source.frames.size()},{"translationKeys",report.translationKeys},{"referenceSamples",report.referenceSamples},{"sourceMotionEnd",{report.samples.back().t.x,report.samples.back().t.y,report.samples.back().t.z}},{"rootBefore",{last.x,last.y,last.z}},{"rootAfter",{source.frames.back()[0].t.x,source.frames.back()[0].t.y,source.frames.back()[0].t.z}},{"checks",checks}};
        if(argc>2){std::ofstream output(std::filesystem::u8path(argv[2]));output<<value.dump(2)<<'\n';}else std::cout<<value.dump(2)<<'\n';
    }
    std::cout<<checks<<" source-motion checks passed\n";return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
