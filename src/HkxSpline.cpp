#include "HkxSpline.h"
#include <bit>
#include <chrono>
#include <cstring>
#include <span>
#include <stdexcept>

namespace fc {
namespace {
using Value=std::array<float,4>;
struct Reader {
    std::span<const std::uint8_t> bytes;
    std::size_t at{},end{};
    void need(std::size_t count)const {
        if(at>end||count>end-at)throw std::runtime_error("truncated spline block");
    }
    std::uint8_t u8(){need(1);return bytes[at++];}
    std::uint16_t u16(){const auto lo=u8();return std::uint16_t(lo|std::uint16_t(u8())<<8);}
    float real(){need(4);float value;std::memcpy(&value,bytes.data()+at,4);at+=4;
        if(!std::isfinite(value)||std::abs(value)>1000000)throw std::runtime_error("invalid spline scalar");return value;}
    void align(std::size_t n){const auto skip=(n-at%n)%n;need(skip);at+=skip;}
};
struct Curve {
    unsigned degree{};
    std::vector<std::uint8_t> knots;
    std::vector<Value> values;
    Value sample(float time,bool rotation)const {
        if(values.size()==1)return values.front();
        const auto n=values.size()-1;
        const float u=std::clamp(time,float(knots[degree]),float(knots[n+1]));
        std::size_t span=n;
        if(u<float(knots[n+1])) {
            const auto it=std::upper_bound(knots.begin()+degree,knots.begin()+n+2,u);
            span=std::clamp<std::size_t>(std::size_t(it-knots.begin()-1),degree,n);
        }
        std::array<Value,4> points{};
        for(unsigned j=0;j<=degree;++j) {
            points[j]=values[span-degree+j];
            if(rotation&&j) {
                float dot=0;for(unsigned c=0;c<4;++c)dot+=points[0][c]*points[j][c];
                if(dot<0)for(auto& v:points[j])v=-v;
            }
        }
        for(unsigned r=1;r<=degree;++r)for(int j=int(degree);j>=int(r);--j) {
            const auto i=span-degree+std::size_t(j);
            const float left=knots[i],right=knots[i+degree-r+1];
            const float a=right>left?std::clamp((u-left)/(right-left),0.f,1.f):0.f;
            for(unsigned c=0;c<4;++c)points[j][c]=points[j-1][c]*(1-a)+points[j][c]*a;
        }
        return points[degree];
    }
};
void splineHeader(Reader& r,Curve& curve) {
    const auto count=std::size_t(r.u16())+1;
    curve.degree=r.u8();
    if(curve.degree>3||count<=curve.degree||count>512)throw std::runtime_error("unsupported spline degree or control count");
    curve.knots.resize(count+curve.degree+1);
    for(auto& k:curve.knots)k=r.u8();
    if(!std::is_sorted(curve.knots.begin(),curve.knots.end())||
       (count>1&&curve.knots[curve.degree]>=curve.knots[count]))throw std::runtime_error("invalid spline knots");
    curve.values.resize(count);
}
Value normalized(Value q) {
    float squared=0;for(float v:q){if(!std::isfinite(v))throw std::runtime_error("nonfinite spline quaternion");squared+=v*v;}
    if(squared<.25f||squared>1.75f)throw std::runtime_error("invalid spline quaternion length");
    const float inverse=1/std::sqrt(squared);for(auto& v:q)v*=inverse;return q;
}
Value unpackQuaternion(Reader& r,unsigned type) {
    if(type==5) {
        r.align(4);Value q{};float length=0;
        for(auto& v:q){v=r.real();length+=v*v;}
        if(std::abs(length-1)>.01f)throw std::runtime_error("uncompressed spline quaternion is not normalized");
        return normalized(q);
    }
    std::array<float,3> small{};unsigned omitted=0;bool negative=false;
    if(type==1) {
        std::uint64_t bits=0;for(unsigned b=0;b<5;++b)bits|=std::uint64_t(r.u8())<<(8*b);
        for(unsigned c=0;c<3;++c)small[c]=float(int((bits>>(12*c))&4095)-2049)*.000345436f;
        omitted=unsigned((bits>>36)&3);negative=((bits>>38)&1)!=0;
    } else if(type==2) {
        r.align(2);std::array<std::uint16_t,3> bits{};for(auto& b:bits)b=r.u16();
        for(unsigned c=0;c<3;++c)small[c]=float(int(bits[c]&32767)-16383)/(16383.f*std::sqrt(2.f));
        omitted=((bits[0]>>15)&1)|((bits[1]>>14)&2);negative=(bits[2]&32768)!=0;
    } else throw std::runtime_error("unsupported HKX rotation quantization (supported: 40, 48, 128 bit)");
    const float squared=small[0]*small[0]+small[1]*small[1]+small[2]*small[2];
    if(squared>1.001f)throw std::runtime_error("invalid packed quaternion components");
    Value result{};unsigned input=0;
    for(unsigned c=0;c<4;++c)result[c]=c==omitted?(negative?-1.f:1.f)*std::sqrt(std::max(0.f,1-squared)):small[input++];
    return normalized(result);
}
Curve readVector(Reader& r,unsigned mask,unsigned quantization,float fallback) {
    if(quantization>1||(mask&0x80)||((mask&7)&((mask>>4)&7)))throw std::runtime_error("unsupported vector mask or quantization");
    const unsigned dynamic=(mask>>4)&7;
    Curve curve;
    if(dynamic)splineHeader(r,curve);else curve.values.resize(1);
    r.align(4);
    Value value{fallback,fallback,fallback,fallback};
    std::array<std::array<float,2>,3> ranges{};
    for(unsigned c=0;c<3;++c) {
        if(mask&(1u<<c))value[c]=r.real();
        else if(dynamic&(1u<<c)) {
            ranges[c]={r.real(),r.real()};
            if(ranges[c][1]<ranges[c][0])throw std::runtime_error("reversed vector quantization range");
        }
    }
    if(mask&8)value[3]=r.real();
    if(dynamic)r.align(2);
    for(auto& point:curve.values) {
        point=value;
        for(unsigned c=0;c<3;++c)if(dynamic&(1u<<c)) {
            const float q=quantization?float(r.u16())/65535.f:float(r.u8())/255.f;
            point[c]=ranges[c][0]+(ranges[c][1]-ranges[c][0])*q;
        }
    }
    r.align(4);return curve;
}
Curve readRotation(Reader& r,unsigned mask,unsigned type) {
    if((mask&0xf0)&&(mask&0xf))throw std::runtime_error("spline rotation cannot be both static and dynamic");
    if(mask&&type!=1&&type!=2&&type!=5)throw std::runtime_error("unsupported HKX rotation quantization (supported: 40, 48, 128 bit)");
    Curve curve;
    if(mask&0xf0) {
        splineHeader(r,curve);
        for(auto& q:curve.values)q=unpackQuaternion(r,type);
    } else if(mask&0xf)curve.values.push_back(unpackQuaternion(r,type));
    else curve.values.push_back({0,0,0,1});
    r.align(4);return curve;
}
struct Track {Curve position,rotation,scale;};
}
bool decodeHkxSplineTransforms(const HkxSplineData& source,std::vector<Pose>& frames,std::string& error) {
    frames.clear();error.clear();
    try {
        const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
        const auto timed=[&](){if(std::chrono::steady_clock::now()>=deadline)throw std::runtime_error("spline decode time budget exceeded");};
        if(source.tracks<1||source.tracks>256||source.numFrames<2||source.numFrames>1201||
           source.numBlocks<1||source.numBlocks>64||source.maxFramesPerBlock<2||source.maxFramesPerBlock>256||
           source.blockOffsets.size()!=source.numBlocks||source.data.size()>64*1024*1024||
           source.maskAndQuantizationSize<source.tracks*4||source.maskAndQuantizationSize>4096)
            throw std::runtime_error("invalid spline dimensions");
        if(!std::isfinite(source.frameDuration)||source.frameDuration<=0||source.frameDuration>1||
           !std::isfinite(source.blockDuration)||source.blockDuration<=0||
           !std::isfinite(source.blockInverseDuration)||source.blockInverseDuration<=0||
           std::abs(source.blockDuration*source.blockInverseDuration-1)>.001f||
           std::abs(source.blockDuration-source.frameDuration*(source.maxFramesPerBlock-1))>.002f)
            throw std::runtime_error("invalid spline timing");
        const auto overlapStride=source.maxFramesPerBlock-1;
        const auto needed=(source.numFrames-2)/overlapStride+1;
        bool overlap=source.numBlocks==needed||(source.numBlocks==needed+1&&(source.numFrames-1)%overlapStride==0);
        bool separate=source.numBlocks==(source.numFrames-1)/source.maxFramesPerBlock+1;
        if(!overlap&&!separate)
            throw std::runtime_error("spline block coverage mismatch");
        for(const auto* offsets:{&source.floatBlockOffsets,&source.transformOffsets,&source.floatOffsets})
            for(auto offset:*offsets)if(offset>source.data.size())throw std::runtime_error("spline auxiliary offset out of range");
        const auto readBlock=[&](unsigned b) {
            timed();
            const auto start=std::size_t(source.blockOffsets[b]);
            const auto end=b+1<source.numBlocks?std::size_t(source.blockOffsets[b+1]):source.data.size();
            if(start>=end||end>source.data.size()||(start&3)||end-start<source.maskAndQuantizationSize)
                throw std::runtime_error("invalid spline block offsets");
            Reader reader{source.data,start+source.maskAndQuantizationSize,end};
            std::vector<Track> block;block.reserve(source.tracks);
            for(unsigned t=0;t<source.tracks;++t) {
                timed();
                const auto* mask=source.data.data()+start+4*t;
                auto position=readVector(reader,mask[1],mask[0]&3,0.f);
                auto rotation=readRotation(reader,mask[2],(mask[0]>>2)&15);
                auto scale=readVector(reader,mask[3],mask[0]>>6,1.f);
                for(const auto& value:scale.values)for(unsigned c=0;c<3;++c)
                    if(value[c]<=.01f||value[c]>100)throw std::runtime_error("invalid animation scale");
                block.push_back({std::move(position),std::move(rotation),std::move(scale)});
            }
            return block;
        };
        std::array<Quat,256> constant{};std::array<std::array<Value,2>,256> constantVectors{};bool constantRotations=true;
        for(unsigned b=0;b<source.numBlocks;++b) {
            const auto block=readBlock(b);
            for(unsigned t=0;t<source.tracks;++t) {
                for(const auto* curve:{&block[t].position,&block[t].rotation,&block[t].scale}) {
                    if(curve->values.size()<2)continue;
                    const auto domain=unsigned(curve->knots[curve->values.size()]);
                    const auto matches=[&](unsigned step) {
                        const auto first=b*step;
                        return first<source.numFrames&&domain==std::min(source.maxFramesPerBlock,source.numFrames-first)-1;
                    };
                    overlap=overlap&&matches(overlapStride);
                    separate=separate&&matches(source.maxFramesPerBlock);
                }
                for(unsigned component=0;component<2;++component) {
                    const auto& curve=component==0?block[t].position:block[t].scale;
                    for(std::size_t k=0;k<curve.values.size();++k) {
                        if(b==0&&k==0)constantVectors[t][component]=curve.values[k];
                        else for(unsigned axis=0;axis<3;++axis)
                            if(std::abs(constantVectors[t][component][axis]-curve.values[k][axis])>1e-6f)constantRotations=false;
                    }
                }
                for(std::size_t k=0;k<block[t].rotation.values.size();++k) {
                    const auto q=normalized(block[t].rotation.values[k]);const Quat rotation{q[0],q[1],q[2],q[3]};
                    if(b==0&&k==0)constant[t]=rotation;
                    else if(angleBetween(constant[t],rotation)>1e-6f)constantRotations=false;
                }
            }
        }
        if(!overlap&&!separate)throw std::runtime_error("spline curve domains disagree with block coverage");
        if(source.numBlocks>1&&overlap&&separate&&!constantRotations)
            throw std::runtime_error("spline block timing is ambiguous; export explicit dynamic block endpoints or uncompressed HKX");
        const auto stride=overlap?overlapStride:source.maxFramesPerBlock;
        std::vector<Pose> result(source.numFrames,Pose(source.tracks));
        for(unsigned b=0;b<source.numBlocks;++b) {
            const auto block=readBlock(b);
            const unsigned first=b*stride;
            const unsigned last=b+1<source.numBlocks?std::min(first+stride,source.numFrames):source.numFrames;
            for(unsigned frame=first;frame<last;++frame) {
                timed();const float local=float(frame-first);
                for(unsigned t=0;t<source.tracks;++t) {
                    auto q=normalized(block[t].rotation.sample(local,true));
                    const auto p=block[t].position.sample(local,false),s=block[t].scale.sample(local,false);
                    result[frame][t]={{p[0],p[1],p[2]},{q[0],q[1],q[2],q[3]},{s[0],s[1],s[2]}};
                }
            }
        }
        frames=std::move(result);return true;
    } catch(const std::exception& exception) {error=exception.what();return false;}
}
bool decodeHkxSpline(const HkxSplineData& source,std::vector<std::vector<Quat>>& rotations,std::string& error) {
    rotations.clear();std::vector<Pose> frames;
    if(!decodeHkxSplineTransforms(source,frames,error))return false;
    rotations.resize(frames.size());
    for(std::size_t i=0;i<frames.size();++i) {
        rotations[i].reserve(frames[i].size());
        for(const auto& transform:frames[i])rotations[i].push_back(transform.q);
    }
    return true;
}
}
