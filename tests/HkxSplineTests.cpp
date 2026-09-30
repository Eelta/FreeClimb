#include "HkxAnimation.h"
#include "HkxFixtures.h"
#include "HkxOracleFixtures.h"
#include <iostream>
#include <limits>

namespace {
unsigned checks=0;
void check(bool value,const std::string& why){++checks;if(!value)throw std::runtime_error(why);}
using Bytes=hkx_fixture::Bytes;
fc::HkxSplineData simple(Bytes bytes) {
    fc::HkxSplineData s;s.tracks=1;s.numFrames=5;s.numBlocks=1;s.maxFramesPerBlock=5;s.maskAndQuantizationSize=4;
    s.blockDuration=2;s.blockInverseDuration=.5f;s.frameDuration=.5f;s.blockOffsets={0};s.data=std::move(bytes);return s;
}
std::vector<std::vector<fc::Quat>> decode(const fc::HkxSplineData& s) {
    std::vector<std::vector<fc::Quat>> out;std::string error;
    check(fc::decodeHkxSpline(s,out,error),"valid spline rejected: "+error);check(error.empty(),"success error retained");return out;
}
void reject(const fc::HkxSplineData& s,const std::string& name) {
    std::vector<std::vector<fc::Quat>> out{{{}}};std::string error;
    check(!fc::decodeHkxSpline(s,out,error),"invalid spline accepted: "+name);check(out.empty()&&!error.empty(),"partial spline escaped: "+name);
}
void vecCurve(Bytes& b,int bits,float lo,float hi) {
    hkx_fixture::append(b,std::uint16_t(1));b.insert(b.end(),{1,0,0,4,4});hkx_fixture::align(b,4);
    hkx_fixture::append(b,lo);hkx_fixture::append(b,hi);
    if(bits==8)b.insert(b.end(),{0,255});else {hkx_fixture::append(b,std::uint16_t(0));hkx_fixture::append(b,std::uint16_t(65535));}
    hkx_fixture::align(b,4);
}
}
int main() try {
    bool oracleGood=true;
    for(const auto& c:hkx_oracle::cases()) {
        auto out=decode(c.source);check(out.size()==c.expected.size(),"oracle frame count");
        float worst=0;unsigned wf=0,wt=0;
        for(unsigned f=0;f<out.size();++f)for(unsigned t=0;t<out[f].size();++t) {
            const float error=fc::angleBetween(out[f][t],c.expected[f][t]);++checks;
            if(error>worst){worst=error;wf=f;wt=t;}
        }
        std::cout<<c.name<<" oracle max radians="<<worst<<" frame="<<wf<<" track="<<wt<<'\n';
        oracleGood=oracleGood&&worst<.00002f;
        fc::HkxClip clip;std::string error;
        check(fc::decodeHkxAnimation(c.file,clip,error),"official full packfile rejected: "+c.name+": "+error);
        check(clip.boneIndices==std::vector<int>({28,39,75}),"official binding changed");
        check(clip.rotations.size()==out.size(),"full/direct spline mismatch");
    }
    const Bytes q40{4,0,15,0,1,24,128,1,56,0,0,0};
    const Bytes q48{8,0,15,0,255,191,255,191,255,63,0,0};
    for(const auto& bytes:{q40,q48})for(const auto& f:decode(simple(bytes)))check(fc::angleBetween(f[0],{})<1e-6f,"packed identity oracle");
    Bytes q128{20,0,15,0};for(float v:{.5f,.5f,.5f,.5f})hkx_fixture::append(q128,v);
    for(const auto& f:decode(simple(q128)))check(fc::angleBetween(f[0],{.5f,.5f,.5f,.5f})<1e-6f,"float quaternion oracle");
    Bytes linear{20,0,240,0};hkx_fixture::append(linear,std::uint16_t(1));linear.insert(linear.end(),{1,0,0,4,4});hkx_fixture::align(linear,4);
    for(float v:{0.f,0.f,0.f,1.f,0.f,0.f,.7071067812f,.7071067812f})hkx_fixture::append(linear,v);
    auto values=decode(simple(linear));
    for(unsigned f=0;f<5;++f) {float t=f*.25f;fc::Quat expected{0,0,.7071067812f*t,1-t+.7071067812f*t};check(fc::angleBetween(values[f][0],expected.unit())<1e-6f,"linear Bezier analytic oracle");}
    auto overlap=simple(linear);overlap.numFrames=9;overlap.numBlocks=2;overlap.blockOffsets.push_back(std::uint32_t(linear.size()));
    auto next=linear;const auto controls=next.size()-32;
    for(unsigned j=0;j<8;++j){const std::array<float,8> q{0,0,.7071067812f,.7071067812f,0,0,1,0};hkx_fixture::put(next,controls+j*4,q[j]);}
    overlap.data.insert(overlap.data.end(),next.begin(),next.end());auto overlapping=decode(overlap);
    for(unsigned f=0;f<9;++f){
        const float t=(f<4?f:f-4)*.25f;
        const fc::Quat a=f<4?fc::Quat{}:fc::Quat{0,0,.7071067812f,.7071067812f};
        const fc::Quat b=f<4?fc::Quat{0,0,.7071067812f,.7071067812f}:fc::Quat{0,0,1,0};
        const fc::Quat q{a.x*(1-t)+b.x*t,a.y*(1-t)+b.y*t,a.z*(1-t)+b.z*t,a.w*(1-t)+b.w*t};
        check(fc::angleBetween(overlapping[f][0],q.unit())<1e-6f,"shared endpoint overlap layout");
    }
    auto ambiguous=simple(q128);ambiguous.numFrames=9;ambiguous.numBlocks=2;ambiguous.blockOffsets.push_back(std::uint32_t(q128.size()));
    ambiguous.data.insert(ambiguous.data.end(),q128.begin(),q128.end());
    check(decode(ambiguous).size()==9,"equivalent static block layouts rejected");
    hkx_fixture::put(ambiguous.data,q128.size()+4,0.f);hkx_fixture::put(ambiguous.data,q128.size()+8,0.f);
    hkx_fixture::put(ambiguous.data,q128.size()+12,0.f);hkx_fixture::put(ambiguous.data,q128.size()+16,1.f);
    reject(ambiguous,"different static blocks have ambiguous layout");
    for(int bits:{8,16}) {
        Bytes b{std::uint8_t(20+(bits==16?1:0)+(bits==16?64:0)),16,15,16};vecCurve(b,bits,0,10);
        for(float v:{.5f,.5f,.5f,.5f})hkx_fixture::append(b,v);vecCurve(b,bits,1,2);
        for(const auto& f:decode(simple(b)))check(fc::angleBetween(f[0],{.5f,.5f,.5f,.5f})<1e-6f,"vector stream alignment oracle");
    }
    auto mutation=[&](const std::string& name,auto fn){auto s=simple(linear);fn(s);reject(s,name);};
    mutation("count",[](auto& s){s.tracks=100;});mutation("frames",[](auto& s){s.numFrames=1202;});
    mutation("blocks",[](auto& s){s.numBlocks=65;});mutation("mask",[](auto& s){s.maskAndQuantizationSize=3;});
    mutation("timing",[](auto& s){s.blockInverseDuration=2;});mutation("NaN",[](auto& s){s.frameDuration=std::numeric_limits<float>::quiet_NaN();});
    mutation("offset",[](auto& s){s.blockOffsets={4};});mutation("aux",[](auto& s){s.floatOffsets={0xffffffff};});
    mutation("degree",[](auto& s){s.data[6]=4;});mutation("knots",[](auto& s){s.data[8]=5;});
    mutation("mixed rotation mask",[](auto& s){s.data[2]=255;});mutation("control count",[](auto& s){s.data[4]=255;s.data[5]=255;});
    mutation("missing data",[](auto& s){s.data.clear();});
    for(std::size_t n=0;n<linear.size();++n){auto s=simple(linear);s.data.resize(n);reject(s,"truncation");}
    check(oracleGood,"independent PyNifly spline oracle mismatch");
    std::cout<<"HKX spline: "<<checks<<" checks passed\n";return 0;
} catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
