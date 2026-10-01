#include "Pose.h"
#include <filesystem>
#include <iostream>
#include <stdexcept>
using namespace fc;
static void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
static constexpr unsigned frozenMotionCount=42;
static constexpr std::array<int,10> tips{69,72,75,78,81,84,87,90,93,96};
static bool tip(int bone){return std::find(tips.begin(),tips.end(),bone)!=tips.end();}
static void writeLibrary(const Library& library,const std::filesystem::path& path,unsigned count,bool corrected,
    const std::array<Quat,10>& offsets) {
    std::ofstream out(path,std::ios::binary);
    auto write=[&](const auto& value){out.write(reinterpret_cast<const char*>(&value),sizeof(value));};
    auto transform=[&](Transform value){write(value.t);write(value.q);write(value.s);};
    write(std::uint32_t{0x344D4346});write(std::uint32_t{2});write(std::uint32_t{99});write(std::uint32_t{count});
    for(int bone=0;bone<99;++bone) {
        write(std::int32_t{library.parents[bone]});write(std::uint32_t(library.names[bone].size()));
        out.write(library.names[bone].data(),library.names[bone].size());transform(library.rest[bone]);
    }
    for(unsigned clip=0;clip<count;++clip) {
        const auto& value=isActiveMotion(Motion(clip+1))?library.clips[clip]:library.clip(Motion::hang);
        write(value.seconds);write(std::uint32_t(value.frames.size()));
        write(value.stride);write(value.height);write(value.travel);
        for(std::size_t frame=0;frame<value.frames.size();++frame) {
            auto pose=value.frames[frame];
            if(corrected&&clip>=legacyMotionCount)for(int index=0;index<10;++index)
                pose[tips[index]].q=(pose[tips[index]].q*offsets[index]).unit();
            for(const auto& bone:pose)transform(bone);
            write(value.contacts[frame]);
        }
    }
    check(bool(out),"diagnostic library output succeeds");
}
int main(int argc,char** argv) {
    try {
        check(argc==2,"motion path required");
        const auto source=std::filesystem::absolute(argv[1]);
        const auto directory=std::filesystem::current_path()/"FreeClimbFingerCalibrationTests-output";
        std::filesystem::create_directories(directory);
        Library library;check(library.load(source.string()),"original runtime library loads");
        check(library.hasLegacyThreepeatFingerBasis(),"only the exact published old42 bytes match the compatibility gate");
        auto raw=library;raw.sourceValidated=false;
        std::array<Quat,10> offsets{};
        const auto initialRaw=raw.sample(Motion::contextHang,0),initial=library.sample(Motion::contextHang,0);
        for(int i=0;i<10;++i) {
            offsets[i]=(initialRaw[tips[i]].q.inverse()*initial[tips[i]].q).unit();
            check(angleBetween(offsets[i],{})>1,"every old terminal basis receives its measured nontrivial correction");
        }
        float otherDelta=0,pointDelta=0,basisDelta=0,timeDelta=0;
        unsigned samples=0;
        for(int motion=1;motion<=frozenMotionCount;++motion) {
            if(!isActiveMotion(Motion(motion)))continue;
            Pose previousRaw,previous;
            for(int frame=0;frame<=240;++frame) {
                const auto before=raw.sample(Motion(motion),frame/240.f),after=library.sample(Motion(motion),frame/240.f);
                const auto worldBefore=raw.world(before),worldAfter=library.world(after);++samples;
                for(int bone=0;bone<99;++bone) {
                    check((before[bone].t-after[bone].t).length()==0&&(before[bone].s-after[bone].s).length()==0,
                        "compatibility changes no bone length, local translation or scale");
                    pointDelta=std::max(pointDelta,(worldBefore[bone].t-worldAfter[bone].t).length());
                    if(motion<=legacyMotionCount||!tip(bone))otherDelta=std::max(otherDelta,angleBetween(before[bone].q,after[bone].q));
                }
                if(motion>legacyMotionCount)for(int index=0;index<10;++index) {
                    const int bone=tips[index];const auto expected=(before[bone].q*offsets[index]).unit();
                    basisDelta=std::max(basisDelta,angleBetween(expected,after[bone].q));
                    if(!previous.empty()) {
                        const auto a=(before[bone].q*previousRaw[bone].q.inverse()).unit();
                        const auto b=(after[bone].q*previous[bone].q.inverse()).unit();
                        timeDelta=std::max(timeDelta,angleBetween(a,b));
                    }
                }
                previousRaw=before;previous=after;
            }
        }
        check(otherDelta<.000001f,"all active legacy clips and all89 nonterminal bones in39-42 are unchanged");
        check(pointDelta<.00001f,"all99 joint positions remain identical");
        check(basisDelta<.000002f&&timeDelta<.000002f,"one static right basis preserves every sampled angular increment");
        const auto corrected=directory/"gate-corrected.motion",legacy=directory/"gate-legacy38.motion",invalid=directory/"gate-invalid.motion";
        writeLibrary(library,corrected,frozenMotionCount,true,offsets);
        writeLibrary(library,legacy,legacyMotionCount,false,offsets);
        Library future,old;
        check(future.load(corrected.string())&&!future.hasLegacyThreepeatFingerBasis(),"correctly converted new42 library is not corrected twice");
        check(old.load(legacy.string())&&!old.hasLegacyThreepeatFingerBasis()&&!old.hasThreepeat(),"old38 library remains a supported uncorrected input");
        float futureDelta=0,legacyDelta=0;
        for(int motion=1;motion<=frozenMotionCount;++motion)if(isActiveMotion(Motion(motion)))for(float at:{0.f,.031f,.239f,.517f,.923f,1.f}) {
            const auto expected=library.sample(Motion(motion),at),newPose=future.sample(Motion(motion),at);
            for(int bone=0;bone<99;++bone)futureDelta=std::max(futureDelta,angleBetween(expected[bone].q,newPose[bone].q));
            if(motion<=legacyMotionCount) {
                const auto oldPose=old.sample(Motion(motion),at);
                for(int bone=0;bone<99;++bone)legacyDelta=std::max(legacyDelta,angleBetween(expected[bone].q,oldPose[bone].q));
            }
        }
        check(futureDelta<.000002f&&legacyDelta<.000001f,"corrected and legacy files play once with the same captured timing");
        std::filesystem::copy_file(source,invalid,std::filesystem::copy_options::overwrite_existing);
        {std::ofstream out(invalid,std::ios::binary|std::ios::app);out.put('x');}
        check(!library.load(invalid.string())&&!library.hasLegacyThreepeatFingerBasis(),"invalid trailing byte cannot leave an exact-prefix match enabled");
        check(library.load(source.string())&&library.hasLegacyThreepeatFingerBasis(),"reuse can subsequently load the known valid old library");
        check(!library.load((directory/"missing-finger-library.motion").string())&&!library.hasLegacyThreepeatFingerBasis(),
            "failed object reuse clears the previous correction flag");
        std::cout<<"PASS finger compatibility samples="<<samples<<" otherBones="<<otherDelta<<" points="<<pointDelta<<
            " staticBasis="<<basisDelta<<" temporalDelta="<<timeDelta<<" future="<<futureDelta<<" legacy38="<<legacyDelta<<'\n';
        return 0;
    }catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}
