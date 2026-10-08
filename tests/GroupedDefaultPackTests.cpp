#include "AnimationPack.h"
#include "AnimationConfig.h"
#include "HkxAnimation.h"
#include "json.hpp"
#include <bit>
#include <iostream>
#include <map>
#include <set>
using namespace fc;
static void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
static std::vector<std::uint8_t> read(const std::filesystem::path& file){std::ifstream in(file,std::ios::binary);check(bool(in),"read grouped asset");return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(in),{});}
static std::uint64_t fingerprint(const HkxClip& clip) {
    std::uint64_t value=14695981039346656037ull;
    auto add=[&](float v){const auto word=std::bit_cast<std::uint32_t>(v);for(int k=0;k<4;++k)value=(value^((word>>(k*8))&255u))*1099511628211ull;};
    add(clip.duration);
    for(const auto& pose:clip.frames)for(const auto& t:pose)
        for(float v:{t.t.x,t.t.y,t.t.z,t.q.x,t.q.y,t.q.z,t.q.w,t.s.x,t.s.y,t.s.z})add(v);
    return value;
}
int main(int argc,char** argv)try {
    check(argc==2,"pack argument required");const std::filesystem::path pack=argv[1],root=pack.parent_path();
    struct Original {const char* slot;const char* group;std::size_t frames;std::uint64_t hash;};
    const std::array sources={
        Original{"runUp","runUp.hkx",39,7931351570731944531ull},
        Original{"runLeft","runLeft.hkx",39,7931351570731944531ull},
        Original{"runRight","runRight.hkx",39,7931351570731944531ull},
        Original{"runDiagonalLeft","runDiagonalLeft.hkx",39,7931351570731944531ull},
        Original{"runDiagonalRight","runDiagonalRight.hkx",39,7931351570731944531ull},
        Original{"runLaunch","runUp.hkx",19,16403947290542353928ull},
        Original{"runCatch","runUp.hkx",14,13563822882909519012ull},
        Original{"runLaunchLeft","runLeft.hkx",13,539864479105119469ull},
        Original{"runLaunchRight","runRight.hkx",13,5161255709131506276ull},
        Original{"sideBrace","runLeft.hkx",4,13299759289905414484ull},
        Original{"contextHang","contextHopLeft.hkx",79,5147081827906125498ull},
        Original{"contextHopLeft","contextHopLeft.hkx",391,5548413346695530091ull},
        Original{"contextHopRight","contextHopRight.hkx",391,13242776804076299365ull}};
    const auto manifest=nlohmann::json::parse(read(pack));Library library;const auto report=loadAnimationPack(library,pack);
    check(report.committed&&report.loaded==31,"grouped default pack loads all retained slots");
    std::set<std::string> hkx,configs;std::map<std::string,nlohmann::json> selected;unsigned slots=0;
    for(const auto& entry:manifest.at("motions")) {
        const auto slot=entry.at("slot").get<std::string>(),path=entry.at("config").get<std::string>();
        const auto document=nlohmann::json::parse(read(root/path));const auto& config=selectAnimationClipConfig(document,slot);
        configs.insert(path);selected.emplace(slot,config);
        hkx.insert(config.at("file").get<std::string>());
        const auto index=std::find(motionSlotNames.begin(),motionSlotNames.end(),slot)-motionSlotNames.begin();
        check(index<motionSlotNames.size()&&!library.clips[index].authoredPlayback,"bundled source metadata stays default v1");++slots;
    }
    check(slots==31&&hkx.size()==25&&configs.size()==25,"31 logical clips use 25 complete action resources and configurations");
    unsigned files=0;for(const auto& entry:std::filesystem::recursive_directory_iterator(root))if(entry.is_regular_file())++files;
    check(files==52,"the default animation directory contains only 25 HKX, 25 action configurations, pack and skeleton");
    for(const auto& expected:sources) {
        const auto& config=selected.at(expected.slot);
        check(config.at("file")==expected.group,"every grouped slot selects its complete action file and explicit timeline");


        HkxClip decoded;std::string error;const auto bytes=read(root/expected.group);
        check(decodeHkxAnimation(bytes,decoded,error,config.value("member",std::string{})),"group timeline decodes");
        if(config.contains("frameRange")){
            const auto range=animationFrameRange(config,decoded.frames.size());const float interval=decoded.duration/float(decoded.frames.size()-1);decoded.duration=interval*float(range[1]-range[0]);
            decoded.frames=std::vector<Pose>(decoded.frames.begin()+range[0],decoded.frames.begin()+range[1]+1);const auto shift=config.at("rootShift").get<std::array<float,3>>();for(auto& pose:decoded.frames)pose[0].t=pose[0].t-Vec{shift[0],shift[1],shift[2]};
            const std::string slot=expected.slot;const float originalSeconds=slot=="contextHang"?.26f:slot=="contextHopLeft"||slot=="contextHopRight"?1.3f:float(expected.frames-1)/60;
            check(std::abs(decoded.duration-originalSeconds)<.000001f,"Timeline reconstruction preserves source stage duration within float precision");decoded.duration=originalSeconds;
        }
        check(decoded.frames.size()==expected.frames&&decoded.boneIndices.size()==99&&fingerprint(decoded)==expected.hash,
            "Canonical grouped timelines retain their exact per-bone transform fingerprints");
        const bool context=std::string(expected.group).starts_with("contextHop");
        check(decodeHkxAnimation(bytes,decoded,error)==context,"Single-binding side leaps decode directly; private reference bindings require explicit selection");
    }
    for(const char* name:{"sprintCatch","flipUp","flipLeft","flipRight"}) {
        check(!selected.contains(name),"removed actions cannot reappear through grouped configurations");
        check(!std::filesystem::exists(root/(std::string(name)+".hkx"))&&!std::filesystem::exists(root/"configs"/(std::string(name)+".json")),"removed action files are absent");
    }
    std::cout<<"13 grouped canonical timelines retain exact fingerprints; 31 slots in 25 complete HKX and JSON action resources\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
