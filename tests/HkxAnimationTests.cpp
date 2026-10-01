#include "HkxAnimation.h"
#include "HkxFixtures.h"
#include <iostream>
#include <limits>
#include <functional>

using namespace hkx_fixture;
namespace {
unsigned checks=0;
void check(bool ok,const std::string& why) {++checks;if(!ok)throw std::runtime_error(why);}
fc::HkxClip decode(const Bytes& bytes) {
    fc::HkxClip clip;std::string error;
    check(fc::decodeHkxAnimation(bytes,clip,error),"valid fixture rejected: "+error);
    check(error.empty(),"success retained error");return clip;
}
void reject(const Bytes& bytes,const std::string& name) {
    fc::HkxClip clip;clip.duration=1;clip.boneIndices={3};clip.rotations={{{}}};std::string error;
    check(!fc::decodeHkxAnimation(bytes,clip,error),"invalid fixture accepted: "+name);
    check(!error.empty()&&clip.rotations.empty()&&clip.boneIndices.empty()&&clip.duration==0,"nontransactional failure: "+name);
}
}
int main() try {
    const auto spec=procedural({40,5,96,28},7);const auto packed=pack(spec);auto out=decode(packed.bytes);
    check(out.boneIndices==spec.indices&&out.rotations.size()==7,"sparse unordered mapping changed");
    for(std::size_t f=0;f<spec.frames.size();++f)for(std::size_t t=0;t<spec.indices.size();++t)
        check(fc::angleBetween(out.rotations[f][t],spec.frames[f][t].q)<1e-5f,"interleaved quaternion mismatch");
    std::vector<int> all(99);std::iota(all.begin(),all.end(),0);auto identity=procedural(all,2);identity.identityMap=true;
    check(decode(pack(identity).bytes).boneIndices==all,"99-track identity binding failed");
    auto named=spec;for(int i:named.indices)named.names.push_back("original-track-"+std::to_string(i));
    check(decode(pack(named).bytes).trackNames==named.names,"annotation names changed");
    auto bad=[&](const std::string& name,const std::function<void(Bytes&)>& change){auto b=packed.bytes;change(b);reject(b,name);};
    bad("magic",[](auto& b){b[0]^=1;});
    bad("32-bit",[](auto& b){b[16]=4;});
    bad("big endian",[](auto& b){b[17]=0;});
    bad("reuse padding",[](auto& b){b[18]=1;});
    bad("packfile version",[](auto& b){put(b,12,9u);});
    bad("contents version",[](auto& b){b[43]='9';});
    bad("section count",[](auto& b){put(b,20,4u);});
    bad("section offset overflow",[](auto& b){put(b,180,0xfffffff0u);});
    bad("section order",[](auto& b){put(b,184,0xffffffffu);});
    bad("external imports",[](auto& b){put(b,200,1u);});
    bad("root section",[](auto& b){put(b,24,3u);});
    bad("root class",[](auto& b){put(b,36,0u);});
    bad("unaligned fixup",[&](auto& b){put(b,packed.localFixups,3u);});
    bad("duplicate fixup",[&](auto& b){std::memcpy(b.data()+packed.localFixups+8,b.data()+packed.localFixups,8);});
    bad("fixup target overflow",[&](auto& b){put(b,packed.localFixups+4,0xfffffff0u);});
    bad("fixup section",[&](auto& b){put(b,packed.globalFixups+4,3u);});
    bad("class fixup",[&](auto& b){put(b,packed.classFixups+4,3u);});
    bad("nonzero pointer",[&](auto& b){put(b,packed.binding+24,std::uint64_t(1));});
    bad("additive",[&](auto& b){b[packed.binding+64]=1;});
    bad("zero duration",[&](auto& b){put(b,packed.animation+20,0.f);});
    bad("NaN duration",[&](auto& b){put(b,packed.animation+20,std::numeric_limits<float>::quiet_NaN());});
    bad("negative tracks",[&](auto& b){put(b,packed.animation+24,-1);});
    bad("too many tracks",[&](auto& b){put(b,packed.animation+24,257);});
    bad("float tracks",[&](auto& b){put(b,packed.animation+28,1);});
    bad("negative mapping",[&](auto& b){put(b,packed.mapping,std::int16_t(-1));});
    bad("duplicate mapping",[&](auto& b){put(b,packed.mapping+2,std::int16_t(spec.indices[0]));});
    bad("large mapping",[&](auto& b){put(b,packed.mapping,std::int16_t(256));});
    bad("partial identity",[&](auto& b){put(b,packed.binding+40,0u);});
    bad("mapping capacity",[&](auto& b){put(b,packed.binding+44,0x80000001u);});
    bad("unsupported animation",[&](auto& b){put(b,packed.animation+16,7u);});
    bad("nonintegral frames",[&](auto& b){put(b,packed.animation+64,27u);});
    bad("single frame",[&](auto& b){put(b,packed.animation+64,4u);});
    bad("excessive frames",[&](auto& b){put(b,packed.animation+64,4u*1202);put(b,packed.animation+68,0x80000000u|4u*1202);});
    for(int i=0;i<12;++i)bad("nonfinite transform "+std::to_string(i),[&,i](auto& b){put(b,packed.transforms+i*4,std::numeric_limits<float>::infinity());});
    bad("zero quaternion",[&](auto& b){for(int i=0;i<4;++i)put(b,packed.transforms+16+i*4,0.f);});
    bad("bad scale",[&](auto& b){put(b,packed.transforms+32,-1.f);});
    bad("large translation",[&](auto& b){put(b,packed.transforms,100001.f);});
    const auto small=pack(procedural()).bytes;
    for(std::size_t count=0;count<small.size();++count)reject(Bytes(small.begin(),small.begin()+count),"truncated "+std::to_string(count));
    Bytes huge(64u*1024*1024+1);reject(huge,"file allocation bound");
    std::cout<<"HKX packfile independent fixtures: "<<checks<<" checks passed\n";return 0;
} catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
