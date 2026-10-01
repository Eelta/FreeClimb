#include "AnimationPack.h"
#include "../external/nlohmann/json.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>

using Json=nlohmann::json;
int main(int argc,char** argv) {
    try {
        if(argc!=4)throw std::runtime_error("Usage: MigrateAnimationPack legacy.motion lossless-hkx-directory output-directory");
        fc::Library source;if(!source.loadLegacy(argv[1])||!source.hasThreepeat())throw std::runtime_error("Invalid legacy 42-motion source");
        const std::filesystem::path input=std::filesystem::absolute(argv[2]),output=std::filesystem::absolute(argv[3]);
        if(std::filesystem::exists(output)&&(!std::filesystem::is_directory(output)||!std::filesystem::is_empty(output)))
            throw std::runtime_error("Output directory must be empty; existing pack files will not be replaced");
        for(const auto motion:fc::activeMotions) {
            const auto name=std::string(fc::motionSlotNames[int(motion)-1]);
            if(source.clip(motion).frames.size()<2||!std::filesystem::is_regular_file(input/(name+".hkx")))
                throw std::runtime_error("Missing original active animation: "+name);
        }
        std::filesystem::create_directories(output/"configs");
        const auto write=[&](const auto& path,const Json& value){std::ofstream f(path);f<<value.dump(2)<<'\n';if(!f)throw std::runtime_error("Metadata write failed");};
        const auto vec=[](fc::Vec v){return Json::array({v.x,v.y,v.z});};
        Json skeleton={{"format","FreeClimbSkeleton"},{"version",1},{"bones",Json::array()}};
        for(std::size_t i=0;i<99;++i) {
            const auto& t=source.rest[i];skeleton["bones"].push_back({{"name",source.names[i]},{"parent",source.parents[i]},
                {"t",vec(t.t)},{"q",{t.q.x,t.q.y,t.q.z,t.q.w}},{"s",vec(t.s)}});
        }
        write(output/"skeleton.json",skeleton);
        Json pack={{"format","FreeClimbAnimationPack"},{"version",1},{"name","FreeClimb default"},{"skeleton","skeleton.json"},{"motions",Json::array()}};
        const auto& p=source.threepeatProfile;
        for(const auto motion:fc::activeMotions) {
            const int i=int(motion)-1;
            const auto name=std::string(fc::motionSlotNames[i]);const auto& clip=source.clips[i];
            const auto config="configs/"+name+".json",file=name+".hkx";
            Json j={{"format","FreeClimbClip"},{"version",1},{"slot",name},{"file",file},
                {"stride",clip.stride},{"height",clip.height},{"travel",vec(clip.travel)},{"contacts",clip.contacts}};
            if(i==39||i==40) {
                const auto side=i-39;j["path"]=Json::array();
                for(std::size_t k=0;k<p.pathCounts[side];++k) {const auto row=p.paths[side][k];j["path"].push_back({row.phase,row.travel,row.lift,row.out});}
                j["sourceHands"]=p.source[side];j["targetHands"]=p.target[side];j["verticalBlend"]=p.rise[side];
            } else if(i==41) {
                j["releaseHands"]=p.mantleRelease;j["unplant"]=p.mantleUnplant;j["replant"]=p.mantleReplant;
                j["replantSamplePhase"]=p.replantSamplePhase;
            }
            std::filesystem::copy_file(input/file,output/file,std::filesystem::copy_options::overwrite_existing);
            write(output/config,j);pack["motions"].push_back({{"slot",name},{"config",config}});
        }
        write(output/"pack.json",pack);
        fc::Library check;const auto report=fc::loadAnimationPack(check,output/"pack.json");
        for(const auto& slot:report.slots)if(slot.status!=fc::OverrideStatus::loaded)std::cerr<<int(slot.motion)<<' '<<slot.reason<<'\n';
        if(!report.committed)throw std::runtime_error(report.error);
        std::cout<<"Migrated "<<report.loaded<<" full-pose HKX clips; metadata and canonical skeleton validated\n";
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
