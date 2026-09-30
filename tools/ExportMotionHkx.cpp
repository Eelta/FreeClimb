#include "Pose.h"
#include "MotionSlots.h"
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
constexpr std::array<int,7> protectedBones{{0,1,2,3,4,97,98}};

void quoted(std::ostream& out,const std::string& value) {
    out<<'"';
    for(unsigned char c:value) {
        if(c=='"'||c=='\\')out<<'\\'<<char(c);
        else if(c<32)out<<"\\u00"<<"0123456789abcdef"[c>>4]<<"0123456789abcdef"[c&15];
        else out<<char(c);
    }
    out<<'"';
}

void vec(std::ostream& out,fc::Vec value) {
    out<<'['<<value.x<<','<<value.y<<','<<value.z<<']';
}

template<class T> fc::Pose sourcePose(const T& library,fc::Motion motion,float phase) {
    if constexpr(requires {library.sampleBase(motion,phase);})return library.sampleBase(motion,phase);
    else return library.sample(motion,phase);
}

std::ofstream output(const std::filesystem::path& path) {
    std::ofstream out(path,std::ios::binary|std::ios::trunc);
    out.exceptions(std::ios::failbit|std::ios::badbit);
    out<<std::setprecision(std::numeric_limits<float>::max_digits10);
    return out;
}
}

int main(int argc,char** argv) {
    try {
        if(argc!=3)throw std::runtime_error("Usage: FreeClimbExportMotionHkx animation-library output-directory");
        const std::filesystem::path input=std::filesystem::absolute(argv[1]);
        const std::filesystem::path directory=std::filesystem::absolute(argv[2]);
        fc::Library library;
        if(!library.load(input.string())||!library.hasThreepeat()||library.names.size()!=99)
            throw std::runtime_error("A complete validated active animation library is required");
        for(const auto motion:fc::activeMotions)if(library.clip(motion).frames.size()<2)
            throw std::runtime_error("An active animation slot is missing");
        std::filesystem::create_directories(directory);
        auto manifest=output(directory/"export.json");
        manifest<<"{\"schema\":1,\"source\":";quoted(manifest,input.generic_string());
        manifest<<",\"format\":\"SkyrimSE/hk_2010.2.0-r1/interleaved/64bitLE\",\"skeleton\":\"NPC Root [Root]\",\"legacyFingerBasisApplied\":"
            <<(library.hasLegacyThreepeatFingerBasis()?"true":"false")<<",\"protectedBones\":[";
        for(std::size_t i=0;i<protectedBones.size();++i) {
            if(i)manifest<<',';
            const int bone=protectedBones[i];
            manifest<<"{\"index\":"<<bone<<",\"name\":";quoted(manifest,library.names[bone]);manifest<<'}';
        }
        manifest<<"],\"bones\":[";
        for(std::size_t i=0;i<library.names.size();++i) {
            if(i)manifest<<',';
            manifest<<"{\"index\":"<<i<<",\"parent\":"<<library.parents[i]<<",\"name\":";
            quoted(manifest,library.names[i]);manifest<<'}';
        }
        manifest<<"],\"clips\":[";
        std::size_t totalFrames{};
        for(std::size_t slot=0;slot<fc::activeMotions.size();++slot) {
            const auto motion=fc::activeMotions[slot];const auto name=fc::motionSlotNames[int(motion)-1];
            const auto& clip=library.clip(motion);
            const std::string json=std::string(name)+".json",hkx=std::string(name)+".hkx";
            auto out=output(directory/json);
            out<<"{\"skeleton\":\"NPC Root [Root]\",\"bones\":[";
            for(std::size_t i=0;i<library.names.size();++i) {if(i)out<<',';quoted(out,library.names[i]);}
            out<<"],\"indices\":[";
            for(std::size_t i=0;i<library.names.size();++i) {if(i)out<<',';out<<i;}
            out<<"],\"duration\":"<<clip.seconds<<",\"frames\":[";
            for(std::size_t frame=0;frame<clip.frames.size();++frame) {
                if(frame)out<<',';
                out<<'[';
                const auto pose=sourcePose(library,motion,float(frame)/float(clip.frames.size()-1));
                if(pose.size()!=99)throw std::runtime_error("Unexpected source pose size");
                for(std::size_t bone=0;bone<pose.size();++bone) {
                    if(bone)out<<',';
                    const auto& t=pose[bone];
                    if(!t.t.finite()||!t.s.finite()||!std::isfinite(t.q.dot(t.q)))throw std::runtime_error("Nonfinite source pose");
                    out<<"{\"t\":";vec(out,t.t);
                    out<<",\"q\":["<<t.q.x<<','<<t.q.y<<','<<t.q.z<<','<<t.q.w<<"],\"s\":";vec(out,t.s);out<<'}';
                }
                out<<']';
            }
            out<<"]}\n";out.close();
            if(slot)manifest<<',';
            manifest<<"{\"id\":"<<int(motion)<<",\"name\":";quoted(manifest,std::string(name));
            manifest<<",\"json\":";quoted(manifest,json);manifest<<",\"hkx\":";quoted(manifest,hkx);
            manifest<<",\"seconds\":"<<clip.seconds<<",\"frames\":"<<clip.frames.size()
                <<",\"sampleFPS\":"<<float(clip.frames.size()-1)/clip.seconds
                <<",\"stride\":"<<clip.stride<<",\"height\":"<<clip.height<<",\"travel\":";vec(manifest,clip.travel);
            manifest<<",\"controllerMetadata\":\"retained in baseline; not inferred from HKX\"}";
            totalFrames+=clip.frames.size();
        }
        manifest<<"],\"totalFrames\":"<<totalFrames<<"}\n";manifest.close();
        std::cout<<"Exported "<<fc::activeMotions.size()<<" slots / "<<totalFrames<<" frames / 99 bones; legacy finger basis "
            <<(library.hasLegacyThreepeatFingerBasis()?"applied":"not required")<<'\n';
        return 0;
    }catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
