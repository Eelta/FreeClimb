#include "AnimationPack.h"
#include "AnimationConfig.h"
#include <chrono>
#include <iostream>

using Json=nlohmann::json;
static void check(bool valid,const char* message){if(!valid)throw std::runtime_error(message);}
static Json read(const std::filesystem::path& path){std::ifstream stream(path);return Json::parse(stream);}
static void write(const std::filesystem::path& path,const Json& value){std::ofstream stream(path);stream<<value.dump();check(bool(stream),"write test metadata");}
struct Workspace {
    std::filesystem::path path=std::filesystem::current_path()/("timeline-loader-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    Workspace(){check(std::filesystem::create_directory(path),"create test workspace");}
    ~Workspace(){std::error_code error;std::filesystem::remove_all(path,error);}
};
int main(int argc,char** argv)try {
    check(argc==2,"animation pack argument required");const std::filesystem::path manifest=argv[1];
    fc::Library original;const auto baseline=fc::loadAnimationPack(original,manifest);check(baseline.committed,"default animation pack loads");
    const auto group=read(manifest.parent_path()/"configs/runUp.json");check(group.at("version")==2,"default wall runs use complete direction timelines");
    for(unsigned i=0;i<5;++i)check(original.wallRunSequenceValid[i]&&original.wallRunLaunches[i].frames.size()>=2&&original.wallRunCatches[i].frames.size()>=2,"every direction has independent start and end data");
    Workspace work;std::filesystem::copy(manifest.parent_path(),work.path,std::filesystem::copy_options::recursive|std::filesystem::copy_options::overwrite_existing);
    const auto config=work.path/"configs/runUp.json";const auto pack=work.path/"pack.json";
    auto fails=[&](Json changed){write(config,changed);fc::Library target=original;const auto result=fc::loadAnimationPack(target,pack);check(!result.committed,"invalid timeline rejected transactionally");check(target.clip(fc::Motion::runUp).seconds==original.clip(fc::Motion::runUp).seconds&&target.wallRunCatches[4].seconds==original.wallRunCatches[4].seconds,"invalid transaction leaves installed clips intact");write(config,group);};
    auto changed=group;changed["sequences"].erase(0);fails(changed);
    changed=group;changed["sequences"].push_back(changed["sequences"][0]);fails(changed);
    changed=group;changed["sequences"][0]["launch"]["frameRange"]={0,0};fails(changed);
    changed=group;changed["sequences"][0]["launch"]["frameRange"]={false,8};fails(changed);
    changed=group;changed["sequences"][0]["catch"]["frameRange"]={1199,1200};fails(changed);
    changed=group;changed["sequences"][0]["launch"]["rootShift"]={0,0,10001};fails(changed);
    changed=group;changed["sequences"][0]["launch"]["member"]="runLeft";fails(changed);
    changed=group;changed["sequences"][0]["catch"]["slot"]="hang";fails(changed);
    changed=group;fc::selectAnimationClipConfig(changed,"runUp")["rootShift"]={0,0,800};fails(changed);
    fc::AnimationOverrideLimits limits;limits.totalOutputBytes=baseline.outputBytes;fc::Library budgeted;
    check(fc::loadAnimationPack(budgeted,manifest,limits).committed,"exact total decoded output budget includes all direction stages");
    --limits.totalOutputBytes;check(!fc::loadAnimationPack(budgeted,manifest,limits).committed,"direction stage allocation cannot bypass output budget");
    std::cout<<"Complete direction timelines, malformed ranges, per-direction data and transactional budgets passed\n";return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
