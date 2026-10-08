#include "AnimationPack.h"
#include "AnimationConfig.h"
#include "HkxAnimation.h"
#include "HkxSpline.h"
#include "HkxFixtures.h"
#include "../external/nlohmann/json.hpp"
#include <chrono>
#include <iostream>
#include <set>

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
    check(report.committed&&report.loaded==activeMotionCount&&report.rejected==0&&report.missing==0,"load all active HKX slots: "+report.error);
    check(report.slots.size()==activeMotions.size(),"Report includes only active slots");
    for(std::size_t i=0;i<activeMotions.size();++i) {
        check(report.slots[i].motion==activeMotions[i]&&report.slots[i].status==OverrideStatus::loaded,"Report preserves each original numeric motion ID");
        check(report.slots[i].samples==loaded.clips[int(activeMotions[i])-1].frames.size(),"Report index maps to the actual sparse library index");
        const auto expected=(std::filesystem::absolute(packPath).lexically_normal().parent_path()/report.slots[i].file).lexically_normal();
        check(std::filesystem::path(report.slots[i].path)==expected,"Report retains the logical animation read path");
        check(report.slots[i].bytes==std::filesystem::file_size(expected)&&report.slots[i].contentId!=0,"Report identifies the bytes actually read");
    }
    const auto baselineReport=report;
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
    for(int id:{6,7,12,13,14,17,30,31,32,33,38}) {
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
    check(originalManifest.at("motions").size()==activeMotionCount,"Real pack manifest contains exactly the active slot records");
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
    for(const char* name:{"mantle","step","toFree","toBraced","freeHang","runDown","dropCatch","contextRegrab","sprintCatch","flipUp","flipLeft","flipRight","","unknownMotion"}) {
        j=originalManifest;j["motions"].back()["slot"]=name;write(manifest,j);
        const auto rejected=transaction("unknown or retired manifest slot");
        check(rejected.error.find("unknown animation slot")!=std::string::npos,"Retired and empty names are rejected before any sparse-slot dereference");
        check(rejected.slots.size()==activeMotionCount,"Invalid manifest reports no retired rows");
    }
    j=originalManifest;j["motions"].push_back({{"slot","dropCatch"},{"config","configs/dropCatch.json"}});write(manifest,j);transaction("extra retired slot");write(manifest,originalManifest);
    {
        const auto entry=std::find_if(originalManifest.at("motions").begin(),originalManifest.at("motions").end(),[](const auto& item){return item.at("slot")=="runUp";});
        check(entry!=originalManifest.at("motions").end(),"wall-run group has a public running stage");
        const auto path=temp.path/entry->at("config").get<std::string>();const auto original=json(path);
        check(original.at("format")=="FreeClimbActionGroup"&&original.at("clips").size()==3,"default wall-run stages share one complete action configuration");
        j=original;auto& left=selectAnimationClipConfig(j,"runUp");const float stride=left.at("stride").get<float>();left["stride"]=stride*1.1f;write(path,j);
        Library changed;const auto changedReport=loadAnimationPack(changed,manifest);
        check(changedReport.committed&&changed.clip(Motion::runUp).stride==stride*1.1f,"editing one grouped stage changes its own runtime metadata");
        check(changed.clip(Motion::runRight).stride==loaded.clip(Motion::runRight).stride,"a grouped stage edit preserves its sibling's runtime metadata");
        samePose(changed.sample(Motion::runRight,.37f),loaded.sample(Motion::runRight,.37f));write(path,original);
        j=original;j["clips"].push_back(j["clips"].front());write(path,j);transaction("duplicate grouped stage");
        j=original;j["clips"].erase(j["clips"].begin());write(path,j);transaction("missing grouped stage");
        j=original;j["group"]="contextHop";write(path,j);transaction("mismatched group membership");
        j=original;selectAnimationClipConfig(j,"runUp")["file"]="../outside.hkx";write(path,j);transaction("one unsafe grouped stage rejects the complete pack");
        write(path,original);
    }
    const auto hangConfig=temp.path/"configs/hang.json";const auto hangOriginal=json(hangConfig);
    {
        for(const Json& selector:std::vector<Json>{Json{},Json(0),Json(true),Json(""),Json("../hang"),Json(std::string(65,'x')),Json("hang")}) {
            j=hangOriginal;j["member"]=selector;write(hangConfig,j);
            const auto rejected=transaction("invalid or missing member selector");
            check(rejected.rejected==1&&rejected.slots[0].reason.find("member")!=std::string::npos,"Member selection never silently falls back to the first animation");
        }
        write(hangConfig,hangOriginal);
        const auto upConfig=temp.path/"configs/up.json";const auto upOriginal=json(upConfig);
        j=upOriginal;j["file"]=hangOriginal.at("file");j.erase("member");write(upConfig,j);
        Library shared;const auto sharedReport=loadAnimationPack(shared,manifest);
        check(sharedReport.committed,"Shared legacy file remains compatible: "+sharedReport.error);
        std::size_t expectedBytes=std::filesystem::file_size(manifest)+std::filesystem::file_size(temp.path/originalManifest.at("skeleton").get<std::string>());
        std::set<std::filesystem::path> uniqueFiles,uniqueConfigs;
        for(const auto& entry:originalManifest.at("motions")) {
            const auto path=temp.path/entry.at("config").get<std::string>();
            if(uniqueConfigs.insert(path).second)expectedBytes+=std::filesystem::file_size(path);
        }
        for(const auto& slot:sharedReport.slots)if(uniqueFiles.insert(slot.path).second)expectedBytes+=slot.bytes;
        check(sharedReport.inputBytes==expectedBytes,"Shared HKX and action configurations are counted only once per transaction");
        AnimationOverrideLimits sharedLimits;sharedLimits.totalBytes=expectedBytes;
        check(loadAnimationPack(shared,manifest,sharedLimits).committed,"Shared file reuse fits the exact unique input budget");
        samePose(shared.sample(Motion::up,.37f),shared.sample(Motion::hang,.37f));
        check(shared.clip(Motion::up).contacts!=shared.clip(Motion::hang).contacts,"Shared animation data keeps independent per-slot metadata");
        write(upConfig,upOriginal);
    }
    for(const std::string& path:std::vector<std::string>{"../outside.hkx","configs/../../outside.hkx",".. /outside.hkx",".. ./outside.hkx",
        "configs /hang.hkx","configs./hang.hkx","hang.hkx ","hang.hkx.","/outside.hkx","//server/share/outside.hkx",
        "C:/outside.hkx","C:outside.hkx","hang.hkx:stream",std::string("hang.hkx\0extra",14)}) {
        j=hangOriginal;j["file"]=path;write(hangConfig,j);const auto rejected=transaction("invalid animation path");
        check(rejected.rejected==1&&rejected.missing==0&&rejected.slots[0].status==OverrideStatus::rejected,
            "Invalid paths are rejected before any missing-file lookup");
    }
#ifdef _WIN32
    for(const char* path:{"\\outside.hkx","\\\\server\\share\\outside.hkx","configs\\..\\outside.hkx","configs\\.. \\outside.hkx"}) {
        j=hangOriginal;j["file"]=path;write(hangConfig,j);const auto rejected=transaction("invalid Windows animation path");
        check(rejected.rejected==1&&rejected.missing==0,"Windows root and traversal paths cannot reach a file lookup");
    }
#endif
    j=hangOriginal;j["file"]="./hang.hkx";write(hangConfig,j);
    j=originalManifest;j["motions"][0]["config"]="configs/./hang.json";j["skeleton"]="./skeleton.json";write(manifest,j);
    Library dotted;const auto dottedReport=loadAnimationPack(dotted,manifest);
    check(dottedReport.committed,"Dot components remain valid inside the logical pack");
    for(std::size_t i=0;i<dottedReport.slots.size();++i) {
        check(dottedReport.slots[i].bytes==baselineReport.slots[i].bytes&&dottedReport.slots[i].contentId==baselineReport.slots[i].contentId,
            "Equivalent logical paths identify identical animation contents");
        check(std::filesystem::path(dottedReport.slots[i].path)==temp.path/baselineReport.slots[i].file,
            "Equivalent logical paths preserve the selected pack root");
    }
    write(manifest,originalManifest);write(hangConfig,hangOriginal);
    {
        Temp outside;std::filesystem::copy_file(temp.path/"hang.hkx",outside.path/"outside.hkx");
        auto linkTest=[&](bool directory) {
            const auto link=temp.path/(directory?"linked-directory":"linked.hkx");std::error_code error;
            if(directory)std::filesystem::create_directory_symlink(outside.path,link,error);
            else std::filesystem::create_symlink(outside.path/"outside.hkx",link,error);
            if(error) {
                check(error==std::errc::permission_denied||error==std::errc::operation_not_permitted||error==std::errc::function_not_supported||
                    error.value()==1314,"Link fixture failed for an unexpected reason: "+error.message());
                std::cout<<"SKIP "<<(directory?"directory":"file")<<" symlink escape: "<<error.message()<<'\n';return;
            }
            j=hangOriginal;j["file"]=directory?"linked-directory/outside.hkx":"linked.hkx";write(hangConfig,j);
            const auto rejected=transaction(directory?"directory symlink escape":"file symlink escape");
            check(rejected.rejected==1&&rejected.missing==0&&rejected.slots[0].reason.find("links")!=std::string::npos,
                "Physical links cannot redirect animation reads outside the pack");
            write(hangConfig,hangOriginal);check(std::filesystem::remove(link),"remove link fixture without following its target");
        };
        linkTest(false);linkTest(true);
    }
    j=hangOriginal;j["file"]="missing.hkx";write(hangConfig,j);auto missing=transaction("missing HKX");
    check(missing.missing==1&&missing.rejected==0&&missing.slots[0].status==OverrideStatus::missing,"missing file report");
    check(missing.slots[0].bytes==0&&missing.slots[0].contentId==0&&std::filesystem::path(missing.slots[0].path)==temp.path/"missing.hkx",
        "Missing files retain their logical path without fabricated content identity");
    j=hangOriginal;j["contacts"][0][0]=2;write(hangConfig,j);transaction("contact weight bounds");write(hangConfig,hangOriginal);
    {
        auto authored=hangOriginal;authored["version"]=2;authored["authoredPlayback"]={{"version",1}};
        write(hangConfig,authored);Library replacement;auto accepted=loadAnimationPack(replacement,manifest);
        check(accepted.committed&&replacement.clip(Motion::hang).authoredPlayback,"Optional authored idle is accepted by the matching metadata version");
        check(!replacement.clip(Motion::up).authoredPlayback&&replacement.clip(Motion::hang).trajectory.count==0,
            "Authored playback remains per slot and does not change other clips");
        samePose(replacement.sample(Motion::hang,.31f),loaded.sample(Motion::hang,.31f));
        for(int version:{1,3}){j=authored;j["version"]=version;write(hangConfig,j);transaction("authored clip requires version two");}
        j=hangOriginal;j["version"]=2;write(hangConfig,j);transaction("version two requires an explicit authored contract");
        j=authored;j["authoredPlayback"]["version"]=2;write(hangConfig,j);transaction("unknown authored contract");
        for(const auto& trajectory:std::vector<Json>{Json{{0,0,0,0}},Json{{0,0,0,0},{0,0,0,0}},
            Json{{0,1,0,0},{1,0,0,0}},Json{{0,0,0,0},{1,0,0,501}},Json{{0,0,0,0},{1,0,0,200}}}) {
            j=authored;j["authoredPlayback"]["basis"]="root";j["authoredPlayback"]["trajectory"]=trajectory;write(hangConfig,j);transaction("invalid or inconsistent authored trajectory");
        }
        write(hangConfig,hangOriginal);
        const auto upFile=temp.path/"configs/up.json";const auto upOriginal=json(upFile);
        j=upOriginal;j["version"]=2;j["authoredPlayback"]={{"version",1}};write(upFile,j);
        accepted=loadAnimationPack(replacement,manifest);
        check(accepted.committed&&replacement.clip(Motion::up).authoredPlayback,"Authored movement slots use the same validated metadata contract");
        write(upFile,upOriginal);
        const auto mantleFile=temp.path/"configs/contextMantle.json";const auto mantleOriginal=json(mantleFile);
        j=mantleOriginal;j["version"]=2;j["authoredPlayback"]={{"version",1}};
        write(mantleFile,j);const auto unrooted=transaction("in-place fallback cannot conceal excessive COM travel");
        check(std::any_of(unrooted.slots.begin(),unrooted.slots.end(),[](const auto& slot){return slot.reason.find("COM")!=std::string::npos;}),"legacy COM-carried travel needs an explicit Root authoring correction");
        hkx_fixture::Spec moving;moving.indices.resize(99);std::iota(moving.indices.begin(),moving.indices.end(),0);
        moving.names=loaded.names;moving.duration=2;moving.frames.assign(3,loaded.clip(Motion::contextMantle).frames.front());
        moving.frames[1][0].t=moving.frames[0][0].t+Vec{0,0,90};moving.frames[2][0].t=moving.frames[0][0].t+Vec{0,40,80};
        moving.frames[2][4].t.z+=12;
        hkx_fixture::save(temp.path/"contextMantle.hkx",hkx_fixture::pack(moving).bytes);
        const auto origin=moving.frames[0][0].t;j["authoredPlayback"]["basis"]="root";
        j["authoredPlayback"]["trajectory"]=Json::array();
        for(int sample=0;sample<=2;++sample) {
            const float phase=float(sample)/2;
            const auto displacement=moving.frames[sample][0].t-origin;
            j["authoredPlayback"]["trajectory"].push_back({phase,displacement.x,displacement.y,displacement.z});
        }
        write(mantleFile,j);accepted=loadAnimationPack(replacement,manifest);
        check(accepted.committed&&replacement.clip(Motion::contextMantle).authoredPlayback,"A matching full-animation mantle trajectory loads transactionally");
        Settings authoredSettings;check(replacement.configureThreepeat(authoredSettings)&&authoredSettings.authoredMantle&&
            authoredSettings.authoredMantleTrajectory.count==3,"Loaded author timing and displacement reach the collision controller");
        samePose(replacement.sample(Motion::contextMantle,0),moving.frames.front());samePose(replacement.sample(Motion::contextMantle,1),moving.frames.back());
        auto spun=moving;
        for(int frame=0;frame<3;++frame) {
            spun.frames[frame]=moving.frames.front();
            spun.frames[frame][0].q=(Quat::axis({1,0,0},3.14159265359f*frame)*spun.frames[frame][0].q).unit();
        }
        auto inPlace=j;inPlace["authoredPlayback"]={{"version",1}};write(mantleFile,inPlace);
        hkx_fixture::save(temp.path/"contextMantle.hkx",hkx_fixture::pack(spun).bytes);
        accepted=loadAnimationPack(replacement,manifest);
        check(accepted.committed,"Legitimate 180 and 360 degree Root turns do not become false COM translation failures");
        spun.frames.back()[4].t.z+=100;
        hkx_fixture::save(temp.path/"contextMantle.hkx",hkx_fixture::pack(spun).bytes);
        const auto badCom=transaction("COM-carried travel in a Root-rotating action");
        check(std::any_of(badCom.slots.begin(),badCom.slots.end(),[](const auto& slot){return slot.reason.find("COM")!=std::string::npos;}),"Root-local COM travel is rejected without suppressing legitimate whole-body turns");
        hkx_fixture::save(temp.path/"contextMantle.hkx",hkx_fixture::pack(moving).bytes);write(mantleFile,j);
        const auto rows=j["authoredPlayback"]["trajectory"];
        j["authoredPlayback"]["trajectory"]=Json{rows.front(),rows.back()};write(mantleFile,j);transaction("coarse authored trajectory");
        write(mantleFile,mantleOriginal);
        std::filesystem::copy_file(source/"contextMantle.hkx",temp.path/"contextMantle.hkx",std::filesystem::copy_options::overwrite_existing);
        hkx_fixture::Spec drifting;drifting.indices.resize(99);std::iota(drifting.indices.begin(),drifting.indices.end(),0);
        drifting.names=loaded.names;drifting.duration=1;drifting.frames.assign(3,loaded.clip(Motion::hang).frames.front());
        write(hangConfig,authored);drifting.frames[1][0].t.x+=25;
        hkx_fixture::save(temp.path/"hang.hkx",hkx_fixture::pack(drifting).bytes);transaction("idle excursions stay near the collider");
        drifting.frames[1]=drifting.frames[0];drifting.frames[2][0].t.x+=13;
        hkx_fixture::save(temp.path/"hang.hkx",hkx_fixture::pack(drifting).bytes);transaction("idle loop cannot progressively move away from its anchor");
        std::filesystem::copy_file(source/"hang.hkx",temp.path/"hang.hkx",std::filesystem::copy_options::overwrite_existing);write(hangConfig,hangOriginal);
    }
    const auto skeletonFile=temp.path/"skeleton.json";const auto skeletonOriginal=json(skeletonFile);
    j=skeletonOriginal;j["bones"][28]["t"][0]=45;write(skeletonFile,j);transaction("modified bind lengths");write(skeletonFile,skeletonOriginal);
    const auto leftEntry=std::find_if(originalManifest.at("motions").begin(),originalManifest.at("motions").end(),[](const auto& entry){return entry.at("slot")=="contextHopLeft";});
    check(leftEntry!=originalManifest.at("motions").end(),"contextual left stage is declared in the pack");
    const auto leftConfig=temp.path/leftEntry->at("config").get<std::string>();const auto leftOriginal=json(leftConfig);
    if(leftOriginal.value("version",0)==2) {
        check(leftOriginal.at("direction")=="contextHopLeft"&&leftOriginal.at("clips").size()==2,"Left action owns only its preparation and leap");
        j=leftOriginal;j["direction"]="contextHopRight";write(leftConfig,j);transaction("cross-direction side-hop group");
        j=leftOriginal;j["sequences"][0]["catch"]["file"]="contextHopRight.hkx";write(leftConfig,j);transaction("side-hop recovery cannot reference the other file");
        j=leftOriginal;j["sequences"][0]["prepare"]["height"]=170;write(leftConfig,j);transaction("prepare metadata cannot differ from its declared stage");
        j=leftOriginal;j["sequences"][0]["catch"]["frameRange"]={0,1};write(leftConfig,j);transaction("recovery cannot precede the leap");
        write(leftConfig,leftOriginal);
        const auto rightEntry=std::find_if(originalManifest.at("motions").begin(),originalManifest.at("motions").end(),[](const auto& entry){return entry.at("slot")=="contextHopRight";});
        check(rightEntry!=originalManifest.at("motions").end()&&rightEntry->at("config")!=leftEntry->at("config"),"Right side is a physically independent override target");
        const auto rightBefore=loaded.sample(Motion::contextHang,.4f,Motion::contextHopRight,true);
        j=leftOriginal;selectAnimationClipConfig(j,"contextHopLeft")["stride"]=120;write(leftConfig,j);
        Library changed;check(loadAnimationPack(changed,manifest).committed,"Left-only override loads transactionally");
        samePose(rightBefore,changed.sample(Motion::contextHang,.4f,Motion::contextHopRight,true));
        check(changed.clip(Motion::contextHopRight).seconds==loaded.clip(Motion::contextHopRight).seconds,"Left override leaves right action duration unchanged");
        write(leftConfig,leftOriginal);
    }
    j=leftOriginal;auto& leftStage=selectAnimationClipConfig(j,"contextHopLeft");leftStage["path"][4][0]=leftStage["path"][3][0];
    write(leftConfig,j);transaction("nonincreasing grouped stage path phases");write(leftConfig,leftOriginal);
    {std::ofstream f(manifest);f<<"{\"format\":\"FreeClimbAnimationPack\",\"format\":\"FreeClimbAnimationPack\",\"version\":1}";}transaction("duplicate JSON keys");write(manifest,originalManifest);
    AnimationOverrideLimits limits;limits.totalOutputBytes=1;check(!loadAnimationPack(loaded,manifest,limits).committed,"allocation budget rejects transaction");
    hkx_fixture::Spec spec;spec.indices.resize(99);std::iota(spec.indices.begin(),spec.indices.end(),0);spec.names=loaded.names;spec.duration=3.25f;
    spec.frames={loaded.clip(Motion::hang).frames.front(),loaded.clip(Motion::hang).frames.back()};
    spec.frames[0][0].t.x+=3;spec.frames[1][0].t.x+=8;spec.frames[0][4].t.z+=2;spec.frames[1][4].t.z+=4;
    spec.frames[1][28].q=(spec.frames[1][28].q*Quat::axis({1,0,0},.15f)).unit();
    auto fixture=hkx_fixture::pack(spec);hkx_fixture::save(temp.path/"hang.hkx",fixture.bytes);
    report=loadAnimationPack(loaded,manifest);check(report.committed,"custom full transform HKX reload: "+report.error);
    check(loaded.clip(Motion::hang).seconds==3.25f&&loaded.clip(Motion::hang).frames.size()==2,"HKX controls timing and sampling density");
    check(report.slots[0].bytes==fixture.bytes.size()&&report.slots[0].contentId!=0&&report.slots[0].contentId!=baselineReport.slots[0].contentId,
        "Animation replacement changes the reported content identity");
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
