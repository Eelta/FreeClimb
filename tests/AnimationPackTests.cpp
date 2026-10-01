#include "AnimationPack.h"
#include "HkxAnimation.h"
#include "HkxSpline.h"
#include "HkxFixtures.h"
#include "../external/nlohmann/json.hpp"
#include <chrono>
#include <iostream>

using namespace fc;
using Json=nlohmann::json;
namespace {
std::size_t checks{};
void check(bool result,const std::string& message) {++checks;if(!result)throw std::runtime_error(message);}
Json json(const std::filesystem::path& path) {std::ifstream f(path);return Json::parse(f);}
void write(const std::filesystem::path& path,const Json& value) {std::ofstream f(path);f<<value.dump(2);check(bool(f),"write fixture JSON");}
struct Temp {
    std::filesystem::path path=std::filesystem::current_path()/("animation-pack-test-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    Temp(){check(std::filesystem::create_directory(path),"create private test directory");}
    ~Temp(){std::error_code ec;std::filesystem::remove_all(path,ec);}
};
float translationError{},rotationError{},scaleError{};
void samePose(const Pose& a,const Pose& b) {
    check(a.size()==99&&b.size()==99,"canonical pose size");
    for(std::size_t i=0;i<99;++i) {
        translationError=std::max(translationError,(a[i].t-b[i].t).length());
        rotationError=std::max(rotationError,angleBetween(a[i].q,b[i].q));
        scaleError=std::max(scaleError,(a[i].s-b[i].s).length());
        check((a[i].t-b[i].t).length()<.0002f,"lossless migration translation");
        check(angleBetween(a[i].q,b[i].q)<.00002f,"lossless migration corrected quaternion");
        check((a[i].s-b[i].s).length()<.000001f,"lossless migration scale");
    }
}
void fullSpline() {
    for(int bits:{8,16}) {
        hkx_fixture::Bytes bytes{std::uint8_t(bits==16?85:20),16,15,16};
        const auto curve=[&](float low,float high) {
            hkx_fixture::append(bytes,std::uint16_t(1));bytes.insert(bytes.end(),{1,0,0,4,4});hkx_fixture::align(bytes,4);
            hkx_fixture::append(bytes,low);hkx_fixture::append(bytes,high);
            if(bits==8)bytes.insert(bytes.end(),{0,255});else {hkx_fixture::append(bytes,std::uint16_t(0));hkx_fixture::append(bytes,std::uint16_t(65535));}
            hkx_fixture::align(bytes,4);
        };
        curve(-8,12);for(float q:{.5f,.5f,.5f,.5f})hkx_fixture::append(bytes,q);curve(1,2);
        HkxSplineData source;source.tracks=1;source.numFrames=5;source.numBlocks=1;source.maxFramesPerBlock=5;
        source.maskAndQuantizationSize=4;source.blockDuration=2;source.blockInverseDuration=.5f;source.frameDuration=.5f;
        source.blockOffsets={0};source.data=bytes;std::vector<Pose> frames;std::string error;
        check(decodeHkxSplineTransforms(source,frames,error),"full spline vector decode: "+error);
        for(int i=0;i<5;++i) {
            check((frames[i][0].t-Vec{-8+5.f*i,0,0}).length()<.000001f,"analytic spline translation");
            check((frames[i][0].s-Vec{1+.25f*i,1,1}).length()<.000001f,"analytic spline scale");
            check(angleBetween(frames[i][0].q,{.5f,.5f,.5f,.5f})<.000001f,"analytic spline rotation");
        }
    }
    hkx_fixture::Bytes bytes{0,1,0,0};hkx_fixture::append(bytes,1.f);
    HkxSplineData source;source.tracks=1;source.numFrames=9;source.numBlocks=2;source.maxFramesPerBlock=5;
    source.maskAndQuantizationSize=4;source.blockDuration=2;source.blockInverseDuration=.5f;source.frameDuration=.5f;
    source.blockOffsets={0,std::uint32_t(bytes.size())};source.data=bytes;
    hkx_fixture::put(bytes,4,2.f);source.data.insert(source.data.end(),bytes.begin(),bytes.end());
    std::vector<Pose> frames;std::string error;check(!decodeHkxSplineTransforms(source,frames,error)&&frames.empty(),"ambiguous position timing must reject despite constant rotations");
}
}
int main(int argc,char** argv) try {
    fullSpline();
    check(argc==2||argc==3,"arguments: [legacy.motion] pack.json");Library legacy,loaded;
    const bool migration=argc==3;const auto packPath=argv[argc-1];
    if(migration)check(legacy.loadLegacy(argv[1]),"load legacy comparison source");
    auto report=loadAnimationPack(loaded,packPath);
    check(report.committed&&report.loaded==activeMotionCount&&report.rejected==0&&report.missing==0,"load all 35 active HKX slots: "+report.error);
    check(report.slots.size()==activeMotions.size(),"Report includes only active slots");
    for(std::size_t i=0;i<activeMotions.size();++i) {
        check(report.slots[i].motion==activeMotions[i]&&report.slots[i].status==OverrideStatus::loaded,"Report preserves each original numeric motion ID");
        check(report.slots[i].samples==loaded.clips[int(activeMotions[i])-1].frames.size(),"Report index maps to the actual sparse library index");
    }
    if(!migration)legacy=loaded;
    check(loaded.animationPack&&!loaded.hasLegacyThreepeatFingerBasis(),"pack is independent from legacy finger cache");
    check(loaded.sourceBytes==0&&loaded.sourceFingerprint==0,"pack has no legacy blob identity");
    check(legacy.names==loaded.names&&legacy.parents==loaded.parents,"canonical names and parents");
    samePose(legacy.rest,loaded.rest);
    std::size_t frameSamples=0;
    for(const auto m:activeMotions) {
        const auto& a=legacy.clip(m);const auto& b=loaded.clip(m);
        check(a.frames.size()==b.frames.size()&&a.seconds==b.seconds,"HKX frame count and duration");
        check(a.stride==b.stride&&a.height==b.height&&(a.travel-b.travel).length()==0&&a.contacts==b.contacts,"editable metadata preserves default controller behavior");
        for(std::size_t i=0;i<a.frames.size();++i) {
            const float phase=float(i)/float(a.frames.size()-1);samePose(legacy.sampleBase(m,phase),loaded.sample(m,phase));++frameSamples;
        }
        for(int i=0;i<=120;++i) {
            const float phase=i/120.f;samePose(legacy.sampleBase(m,phase),loaded.sample(m,phase));
            check(legacy.contactWeights(m,phase)==loaded.contactWeights(m,phase),"normalized contact envelope");
        }
    }
    for(int id:{6,7,12,13,14,33,38}) {
        check(!isActiveMotion(Motion(id))&&motionSlotNames[id-1].empty(),"Retired ID has no public slot name");
        check(loaded.clips[id-1].frames.empty()&&loaded.clips[id-1].contacts.empty(),"Retired pack ID has no allocated animation data");
        check(legacy.clips[id-1].frames.empty()&&legacy.clips[id-1].contacts.empty(),"Retired legacy ID is validated and discarded");
        check(loaded.clip(Motion(id)).frames.empty()&&!loaded.hasAnimationOverride(Motion(id)),"Retired ID cannot alias a live clip");
        samePose(loaded.sample(Motion(id),.4f),loaded.rest);
        check(loaded.contactWeights(Motion(id),.4f)==std::array<float,4>{},"Retired ID has no contact loads");
    }
    for(int id:{0,-1,motionCount+1,1000000}) {
        samePose(loaded.sample(Motion(id),.4f),loaded.rest);
        check(loaded.clip(Motion(id)).frames.empty()&&!loaded.hasAnimationOverride(Motion(id)),"Out-of-range IDs cannot clamp into active clips");
    }
    const auto& p=loaded.threepeatProfile;
    for(int i=0;i<=10000;++i) {
        const float phase=i/10000.f;
        for(bool left:{false,true}) {
            check(std::abs(threepeatHopTravel(left,phase)-threepeatHopTravel(left,phase,p))<.000002f,"config travel equivalence");
            check(std::abs(threepeatHopLift(left,phase)-threepeatHopLift(left,phase,p))<.00002f,"config lift equivalence");
            check(std::abs(threepeatHopOut(left,phase)-threepeatHopOut(left,phase,p))<.000002f,"config outside equivalence");
            for(int hand=0;hand<2;++hand) {
                check(std::abs(threepeatSourceWeight(left,hand,phase)-threepeatSourceWeight(left,hand,phase,p))<.000003f,"source-window equivalence");
                check(std::abs(threepeatTargetWeight(left,hand,phase)-threepeatTargetWeight(left,hand,phase,p))<.000003f,"target-window equivalence");
            }
        }
        for(int hand=0;hand<2;++hand)check(std::abs(threepeatMantleWeight(hand,phase)-threepeatMantleWeight(hand,phase,p))<.000003f,"mantle-window equivalence");
    }
    Temp temp;const auto source=std::filesystem::path(packPath).parent_path();
    std::filesystem::copy(source,temp.path,std::filesystem::copy_options::recursive|std::filesystem::copy_options::overwrite_existing);
    const auto manifest=temp.path/"pack.json";const auto originalManifest=json(manifest);
    check(originalManifest.at("motions").size()==activeMotionCount,"Real pack manifest contains exactly 35 active records");
    for(const char* name:{"mantle","step","toFree","toBraced","freeHang","runDown","dropCatch"}) {
        check(!std::filesystem::exists(temp.path/(std::string(name)+".hkx"))&&
            !std::filesystem::exists(temp.path/"configs"/(std::string(name)+".json")),"Retired assets are absent from the real pack");
    }
    auto transaction=[&](const std::string& label) {
        const auto pointer=loaded.clips[0].frames.data();const auto reference=loaded.sample(Motion::contextMantle,.47f);
        const auto rejected=loadAnimationPack(loaded,manifest);
        check(!rejected.committed&&!rejected.error.empty(),label+" must reject");
        check(pointer==loaded.clips[0].frames.data(),label+" must retain previous allocation");
        samePose(reference,loaded.sample(Motion::contextMantle,.47f));
        return rejected;
    };
    auto j=originalManifest;
    std::reverse(j["motions"].begin(),j["motions"].end());write(manifest,j);
    Library reversed;const auto reordered=loadAnimationPack(reversed,manifest);
    check(reordered.committed&&reordered.slots.size()==activeMotionCount,"Manifest order is independent of numeric motion IDs");
    for(const auto motion:activeMotions) {
        samePose(reversed.sample(motion,.37f),loaded.sample(motion,.37f));
        check(reversed.clip(motion).contacts==loaded.clip(motion).contacts,"Reordered manifest preserves per-ID contact samples");
    }
    j=originalManifest;j["motions"].erase(j["motions"].begin());write(manifest,j);transaction("missing active slot");
    j=originalManifest;j["motions"][1]=j["motions"][0];write(manifest,j);transaction("duplicate slot");write(manifest,originalManifest);
    for(const char* name:{"mantle","step","toFree","toBraced","freeHang","runDown","dropCatch","contextRegrab","","unknownMotion"}) {
        j=originalManifest;j["motions"].back()["slot"]=name;write(manifest,j);
        const auto rejected=transaction("unknown or retired manifest slot");
        check(rejected.error.find("unknown animation slot")!=std::string::npos,"Retired and empty names are rejected before any sparse-slot dereference");
        check(rejected.slots.size()==activeMotionCount,"Invalid manifest reports no retired rows");
    }
    j=originalManifest;j["motions"].push_back({{"slot","dropCatch"},{"config","configs/dropCatch.json"}});write(manifest,j);transaction("extra retired slot");write(manifest,originalManifest);
    const auto hangConfig=temp.path/"configs/hang.json";const auto hangOriginal=json(hangConfig);
    j=hangOriginal;j["file"]="../outside.hkx";write(hangConfig,j);transaction("path escape");
    j=hangOriginal;j["file"]="missing.hkx";write(hangConfig,j);auto missing=transaction("missing HKX");check(missing.missing==1&&missing.slots[0].status==OverrideStatus::missing,"missing file report");
    j=hangOriginal;j["contacts"][0][0]=2;write(hangConfig,j);transaction("contact weight bounds");write(hangConfig,hangOriginal);
    const auto skeletonFile=temp.path/"skeleton.json";const auto skeletonOriginal=json(skeletonFile);
    j=skeletonOriginal;j["bones"][28]["t"][0]=45;write(skeletonFile,j);transaction("modified bind lengths");write(skeletonFile,skeletonOriginal);
    const auto leftConfig=temp.path/"configs/contextHopLeft.json";const auto leftOriginal=json(leftConfig);
    j=leftOriginal;j["path"][4][0]=j["path"][3][0];write(leftConfig,j);transaction("nonincreasing path phases");write(leftConfig,leftOriginal);
    {std::ofstream f(manifest);f<<"{\"format\":\"FreeClimbAnimationPack\",\"format\":\"FreeClimbAnimationPack\",\"version\":1}";}transaction("duplicate JSON keys");write(manifest,originalManifest);
    AnimationOverrideLimits limits;limits.totalOutputBytes=1;check(!loadAnimationPack(loaded,manifest,limits).committed,"allocation budget rejects transaction");
    hkx_fixture::Spec spec;spec.indices.resize(99);std::iota(spec.indices.begin(),spec.indices.end(),0);spec.names=loaded.names;spec.duration=3.25f;
    spec.frames={loaded.clip(Motion::hang).frames.front(),loaded.clip(Motion::hang).frames.back()};
    spec.frames[0][0].t.x+=3;spec.frames[1][0].t.x+=8;spec.frames[0][4].t.z+=2;spec.frames[1][4].t.z+=4;
    spec.frames[1][28].q=(spec.frames[1][28].q*Quat::axis({1,0,0},.15f)).unit();
    auto fixture=hkx_fixture::pack(spec);hkx_fixture::save(temp.path/"hang.hkx",fixture.bytes);
    report=loadAnimationPack(loaded,manifest);check(report.committed,"custom full transform HKX reload: "+report.error);
    check(loaded.clip(Motion::hang).seconds==3.25f&&loaded.clip(Motion::hang).frames.size()==2,"HKX controls timing and sampling density");
    samePose(loaded.sample(Motion::hang,0),spec.frames[0]);samePose(loaded.sample(Motion::hang,1),spec.frames[1]);
    check(loaded.clip(Motion::hang).contacts.front()==legacy.clip(Motion::hang).contacts.front()&&loaded.clip(Motion::hang).contacts.back()==legacy.clip(Motion::hang).contacts.back(),"contact retiming preserves endpoints");
    auto bad=spec;bad.frames[0][28].t.z+=1;hkx_fixture::save(temp.path/"hang.hkx",hkx_fixture::pack(bad).bytes);transaction("bone stretching");
    bad=spec;bad.frames[0][4].s.x=1.1f;hkx_fixture::save(temp.path/"hang.hkx",hkx_fixture::pack(bad).bytes);transaction("animated scale");
    bad=spec;bad.frames[0][97].q=Quat::axis({0,0,1},.4f);hkx_fixture::save(temp.path/"hang.hkx",hkx_fixture::pack(bad).bytes);transaction("camera track mutation");
    bad=spec;bad.indices.pop_back();bad.names.pop_back();for(auto& frame:bad.frames)frame.pop_back();hkx_fixture::save(temp.path/"hang.hkx",hkx_fixture::pack(bad).bytes);transaction("missing canonical track");
    auto truncated=fixture.bytes;truncated.resize(truncated.size()/2);hkx_fixture::save(temp.path/"hang.hkx",truncated);transaction("truncated HKX");
    Library dispatch;check(dispatch.load(packPath)&&dispatch.animationPack,"Library JSON dispatch");
    std::cout<<"PASS "<<checks<<" checks; mode="<<(migration?"legacy-migration":"standalone-pack")<<"; source frames="<<frameSamples<<"; max translation="<<translationError<<" quaternion radians="<<rotationError<<" scale="<<scaleError<<'\n';
    return 0;
} catch(const std::exception& e) {std::cerr<<"FAIL "<<checks<<": "<<e.what()<<'\n';return 1;}
