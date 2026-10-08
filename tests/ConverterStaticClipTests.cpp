#include "../tools/converter/ConverterInput.h"
#include "../tools/converter/ConverterSpline.h"
#include "HkxFixtures.h"
#include <iostream>
#include <limits>

namespace {
std::size_t checks{};
void expect(bool valid,const std::string& message) {
    ++checks;if(!valid)throw std::runtime_error(message);
}
fc::HkxSplineData staticSpline() {
    fc::HkxSplineData source;source.tracks=1;source.numFrames=1;source.numBlocks=1;
    source.maxFramesPerBlock=256;source.maskAndQuantizationSize=4;source.blockOffsets={0};
    source.frameDuration=source.blockDuration=std::numeric_limits<float>::infinity();source.blockInverseDuration=0;
    source.data={69,7,15,0};
    for(float value:{12.f,2.f,3.f})hkx_fixture::append(source.data,value);
    source.data.insert(source.data.end(),{0x51,0xfa,0x58,0x47,0x3e});hkx_fixture::align(source.data,4);
    return source;
}
hkx_fixture::Spline packedSpline(const fc::HkxSplineData& source) {
    hkx_fixture::Spline target;target.tracks=source.tracks;target.frames=source.numFrames;
    target.blocks=source.numBlocks;target.maxFrames=source.maxFramesPerBlock;
    target.maskBytes=source.maskAndQuantizationSize;target.blockDuration=source.blockDuration;
    target.inverseDuration=source.blockInverseDuration;target.frameDuration=source.frameDuration;
    target.blockOffsets=source.blockOffsets;target.floatBlockOffsets=source.floatBlockOffsets;
    target.data=source.data;return target;
}
void same(const fc::Pose& a,const fc::Pose& b) {
    expect(a.size()==b.size(),"expanded static track counts are identical");
    for(std::size_t track=0;track<a.size();++track) {
        expect((a[track].t-b[track].t).length()==0,"expanded static translations are identical");
        expect(a[track].q.dot(b[track].q)>.999999f,"expanded static rotations are identical");
        expect((a[track].s-b[track].s).length()==0,"expanded static scales are identical");
    }
}
void run() {
    const auto source=staticSpline();std::vector<fc::Pose> frames;std::string error;
    expect(fc::decodeConverterSplineTransforms(source,frames,error)&&frames.size()==1,"static Havok clock decodes one source frame: "+error);
    const fc::Quat sdk{.20518877827792997f,-.21555185029619486f,.5554605648115687f,.7764654055667326f};
    expect(fc::angleBetween(frames[0][0].q,sdk)<.00001f,"single-frame compressed quaternion retains SDK value");
    auto spec=hkx_fixture::procedural({28},1);spec.duration=1.f/60;
    auto spline=packedSpline(source);const auto packed=hkx_fixture::pack(spec,&spline);const auto original=packed.bytes;fc::ConverterInputClip clip;
    expect(fc::decodeConverterInput(packed.bytes,clip,error)&&clip.frames.size()==2&&clip.rotations.size()==2,"single static frame expands for the existing formal sampler: "+error);
    expect(clip.duration==spec.duration,"static expansion preserves authored duration");same(clip.frames[0],clip.frames[1]);
    expect(fc::angleBetween(clip.frames[0][0].q,sdk)<.00001f,"expanded source quaternion is unchanged");
    expect(packed.bytes==original,"static expansion does not mutate input bytes");
    auto finite=source;finite.frameDuration=spec.duration;finite.blockDuration=255*finite.frameDuration;finite.blockInverseDuration=1/finite.blockDuration;
    spline=packedSpline(finite);expect(fc::decodeConverterInput(hkx_fixture::pack(spec,&spline).bytes,clip,error)&&clip.frames.size()==2,"finite consistent single-frame clock also expands");
    const auto interleaved=hkx_fixture::pack(spec);
    expect(fc::decodeConverterInput(interleaved.bytes,clip,error)&&clip.frames.size()==2,"interleaved single frame also expands");same(clip.frames[0],clip.frames[1]);
    expect((clip.frames.front()[0].t-spec.frames.front()[0].t).length()==0,"interleaved authored transform is retained");
    auto badInterleaved=interleaved.bytes;hkx_fixture::put(badInterleaved,interleaved.transforms+28,2.f);
    expect(!fc::decodeConverterInput(badInterleaved,clip,error)&&clip.frames.empty(),"invalid single-frame quaternion is not hidden by duplication");
    const auto reject=[&](const fc::HkxSplineData& bad,const std::string& why) {
        expect(!fc::decodeConverterSplineTransforms(bad,frames,error)&&frames.empty()&&!error.empty(),why);
        const auto data=packedSpline(bad);expect(!fc::decodeConverterInput(hkx_fixture::pack(spec,&data).bytes,clip,error)&&clip.frames.empty()&&!error.empty(),why+" through bounded packfile reader");
    };
    for(unsigned channel:{1u,2u,3u}) {
        auto bad=source;bad.data[channel]|=0x10;reject(bad,"single-frame dynamic channel is not guessed: "+std::to_string(channel));
    }
    auto bad=source;bad.numBlocks=2;bad.blockOffsets.push_back(0);reject(bad,"single frame cannot claim multiple blocks");
    bad=source;bad.numFrames=0;reject(bad,"zero frames remain rejected");
    bad=source;bad.numFrames=2;reject(bad,"infinite clocks remain rejected for moving clips");
    bad=source;bad.frameDuration=std::numeric_limits<float>::quiet_NaN();reject(bad,"NaN clock is never accepted");
    bad=source;bad.blockDuration=-std::numeric_limits<float>::infinity();reject(bad,"negative infinite block duration is rejected");
    bad=source;bad.frameDuration=-std::numeric_limits<float>::infinity();reject(bad,"negative infinite frame duration is rejected");
    bad=source;bad.blockInverseDuration=1;reject(bad,"infinite clock requires exact zero inverse");
    bad=source;bad.frameDuration=1.f/60;reject(bad,"mixed finite and infinite clocks remain rejected");
    bad=source;bad.data.resize(5);reject(bad,"truncated static transforms remain rejected");
    bad=source;hkx_fixture::put(bad.data,4,std::numeric_limits<float>::infinity());reject(bad,"nonfinite static translation remains rejected");
    bad=source;bad.blockOffsets[0]=1;reject(bad,"unaligned single-block offset remains rejected");
    spline=packedSpline(finite);spec.duration=.5f;
    expect(!fc::decodeConverterInput(hkx_fixture::pack(spec,&spline).bytes,clip,error)&&clip.frames.empty(),"finite static frame interval must agree with authored duration");
    expect(source.data==staticSpline().data,"all rejected inputs leave baseline source untouched");
    std::cout<<"pass: "<<checks<<" static-clip checks\n";
}
}
int main() {
    try {run();return 0;}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
