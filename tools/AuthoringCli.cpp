#include "AnimationPack.h"
#include "HkxAnimation.h"
#include "../external/nlohmann/json.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string_view>

namespace {
using Json=nlohmann::json;

int output(const Json& result) {
    std::cout<<result.dump(-1,' ',true,Json::error_handler_t::replace)<<'\n';
    return result.value("ok",false)?0:1;
}

int failure(std::string_view error) {
    return output({{"ok",false},{"error",error}});
}

Json inspect(const std::filesystem::path& path) {
    if(!std::filesystem::is_regular_file(path))throw std::runtime_error("HKX input is not a regular file");
    const auto size=std::filesystem::file_size(path);
    if(size<208||size>64*1024*1024)throw std::runtime_error("HKX input is truncated or exceeds the 64 MiB file limit");
    std::ifstream stream(path,std::ios::binary);
    if(!stream)throw std::runtime_error("Cannot open HKX input");
    std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size),0);
    if(!stream.read(reinterpret_cast<char*>(bytes.data()),static_cast<std::streamsize>(bytes.size()))||
        stream.peek()!=std::char_traits<char>::eof()||stream.bad())throw std::runtime_error("HKX input changed or failed while being read");
    fc::HkxClip clip;std::string error;
    if(!fc::decodeHkxAnimation(bytes,clip,error))throw std::runtime_error(error);
    return {{"ok",true},{"duration",clip.duration},{"frames",clip.frames.size()},{"tracks",clip.boneIndices.size()}};
}

Json validate(const std::filesystem::path& path) {
    fc::Library library;const auto report=fc::loadAnimationPack(library,path);
    Json result{{"ok",report.committed},{"loaded",report.loaded},{"error",report.error},{"slots",Json::array()}};
    for(const auto& entry:report.slots) {
        const auto id=int(entry.motion);
        const char* status=entry.status==fc::OverrideStatus::loaded?"loaded":entry.status==fc::OverrideStatus::rejected?"rejected":"missing";
        result["slots"].push_back({{"slot",fc::motionSlotNames[id-1]},{"id",id},{"status",status},{"file",entry.file},{"reason",entry.reason}});
    }
    return result;
}

int run(std::string_view command,const std::filesystem::path& path) {
    try {
        return output(command=="inspect"?inspect(path):validate(path));
    } catch(const std::exception& error) {
        return failure(error.what());
    }
}
}

#ifdef _WIN32
int wmain(int argc,wchar_t** argv) {
    try {
        if(argc!=3)return failure("Usage: FreeClimbAuthoring inspect <animation.hkx> | validate <pack.json>");
        const std::wstring_view command=argv[1];
        if(command!=L"inspect"&&command!=L"validate")return failure("Unknown command; use inspect or validate");
        return run(command==L"inspect"?"inspect":"validate",std::filesystem::path(argv[2]));
    } catch(const std::exception& error) {
        return failure(error.what());
    }
}
#else
int main(int argc,char** argv) {
    try {
        if(argc!=3)return failure("Usage: FreeClimbAuthoring inspect <animation.hkx> | validate <pack.json>");
        const std::string_view command=argv[1];
        if(command!="inspect"&&command!="validate")return failure("Unknown command; use inspect or validate");
        return run(command,std::filesystem::path(argv[2]));
    } catch(const std::exception& error) {
        return failure(error.what());
    }
}
#endif
