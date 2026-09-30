#include "AnimationOverrides.h"
#include "HkxAnimation.h"
#include <chrono>
#include <fstream>

namespace fc {
AnimationOverrideReport loadHkxOverrides(Library& library,const std::filesystem::path& directory,AnimationOverrideLimits limits) {
    AnimationOverrideReport report;library.clearAnimationOverrides();
    const auto started=std::chrono::steady_clock::now();std::size_t totalBytes=0,totalOutputBytes=0;
    limits.fileBytes=std::min(limits.fileBytes,std::size_t(64*1024*1024));
    limits.totalBytes=std::min(limits.totalBytes,std::size_t(256*1024*1024));limits.frames=std::min(limits.frames,std::size_t(1201));
    limits.totalOutputBytes=std::min(limits.totalOutputBytes,std::size_t(128*1024*1024));
    limits.fileMilliseconds=std::min(limits.fileMilliseconds,3000u);limits.totalMilliseconds=std::min(limits.totalMilliseconds,30000u);
    for(const auto motion:activeMotions) {
        const int index=int(motion)-1;
        AnimationOverrideResult item;item.motion=motion;item.file=std::string(motionSlotNames[index])+".hkx";
        const auto path=directory/item.file;std::error_code ec;const auto status=std::filesystem::status(path,ec);
        if(ec||!std::filesystem::exists(status)) {
            if(ec&&ec!=std::errc::no_such_file_or_directory){item.status=OverrideStatus::rejected;item.reason="Cannot inspect HKX file: "+ec.message();}
            if(item.status==OverrideStatus::rejected)++report.rejected;else ++report.missing;
            report.slots.push_back(std::move(item));continue;
        }
        item.status=OverrideStatus::rejected;
        try {
            auto require=[](bool valid,const char* error){if(!valid)throw std::runtime_error(error);};
            require(std::filesystem::is_regular_file(status),"HKX slot path is not a regular file");
            require(library.sourceValidated&&library.rest.size()==99&&library.names.size()==99,"Canonical base motion library is unavailable");
            const auto now=std::chrono::steady_clock::now();
            require(limits.fileMilliseconds>0,"HKX slot load time budget is zero");
            require(now-started<std::chrono::milliseconds(limits.totalMilliseconds),"Total HKX load time budget exceeded");
            const auto size=std::filesystem::file_size(path,ec);require(!ec,"Cannot read HKX file size");
            require(size>=208&&size<=limits.fileBytes,"HKX file exceeds slot byte limit or is truncated");
            require(totalBytes<=limits.totalBytes&&size<=limits.totalBytes-totalBytes,"Total HKX byte budget exceeded");totalBytes+=std::size_t(size);
            std::ifstream input(path,std::ios::binary);require(bool(input),"Cannot open HKX slot");
            std::vector<std::uint8_t> bytes(std::size_t(size),0);
            require(bool(input.read(reinterpret_cast<char*>(bytes.data()),std::streamsize(bytes.size()))),"HKX file read was incomplete");
            require(input.peek()==std::char_traits<char>::eof(),"HKX file changed while being read");
            HkxClip decoded;std::string error;require(decodeHkxAnimation(bytes,decoded,error),error.c_str());
            require(std::chrono::steady_clock::now()-now<std::chrono::milliseconds(limits.fileMilliseconds),"HKX slot load time budget exceeded");
            require(std::chrono::steady_clock::now()-started<std::chrono::milliseconds(limits.totalMilliseconds),"Total HKX load time budget exceeded");
            require(decoded.rotations.size()<=limits.frames,"HKX source frame budget exceeded");
            RotationOverride staged;std::size_t accepted=0;
            for(std::size_t track=0;track<decoded.boneIndices.size();++track) {
                const auto bone=decoded.boneIndices[track];
                if(bone<99) {
                    require(decoded.trackNames[track].empty()||decoded.trackNames[track]==library.names[bone],"HKX track name conflicts with its canonical bone index");
                    require(!decoded.identityMapping||decoded.boneIndices.size()!=126||decoded.trackNames[track]==library.names[bone],
                        "Implicit 126-track HKX requires complete canonical names for the first 99 bones");
                }
                if(bone>=5&&bone<97){staged.bones[bone]=true;++accepted;}
            }
            require(accepted>0,"HKX slot contains only protected root, control or camera tracks");
            const auto count=library.clips[index].frames.size();require(count>=2&&count<=limits.frames,"Base slot is missing or exceeds frame budget");
            const auto outputBytes=count*sizeof(std::array<Quat,99>);
            require(totalOutputBytes<=limits.totalOutputBytes&&outputBytes<=limits.totalOutputBytes-totalOutputBytes,
                "Total HKX output memory budget exceeded");
            staged.frames.resize(count);
            for(std::size_t frame=0;frame<count;++frame) {
                const float position=float(frame)*float(decoded.rotations.size()-1)/float(count-1);
                const auto first=std::min(std::size_t(position),decoded.rotations.size()-2);const float blendWeight=position-float(first);
                for(std::size_t track=0;track<decoded.boneIndices.size();++track) {
                    const auto bone=decoded.boneIndices[track];if(bone<99&&staged.bones[bone])staged.frames[frame][bone]=blend(decoded.rotations[first][track],decoded.rotations[first+1][track],blendWeight);
                }
            }
            require(std::chrono::steady_clock::now()-now<std::chrono::milliseconds(limits.fileMilliseconds),"HKX slot load time budget exceeded");
            require(std::chrono::steady_clock::now()-started<std::chrono::milliseconds(limits.totalMilliseconds),"Total HKX load time budget exceeded");
            library.rotationOverrides[index]=std::move(staged);totalOutputBytes+=outputBytes;item.samples=count;item.status=OverrideStatus::loaded;
            item.reason="Canonical local rotations loaded; base timing, contacts, root motion and controls retained";++report.loaded;
        } catch(const std::exception& failure){item.reason=failure.what();}
        if(item.status==OverrideStatus::rejected)++report.rejected;
        report.slots.push_back(std::move(item));
    }
    report.inputBytes=totalBytes;report.outputBytes=totalOutputBytes;
    return report;
}
}
