#include "AnimationPack.h"
#include "HkxAnimation.h"
#include "converter/Converter.h"
#include "converter/ConverterWallRunGroup.h"
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

std::vector<std::uint8_t> read(const std::filesystem::path& path,std::uintmax_t maximum=64*1024*1024) {
    if(!std::filesystem::is_regular_file(path))throw std::runtime_error("Input is not a regular file");
    const auto size=std::filesystem::file_size(path);
    if(!size||size>maximum)throw std::runtime_error("Input is empty or exceeds its size limit");
    std::ifstream stream(path,std::ios::binary);
    if(!stream)throw std::runtime_error("Cannot open HKX input");
    std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size),0);
    if(!stream.read(reinterpret_cast<char*>(bytes.data()),static_cast<std::streamsize>(bytes.size()))||
        stream.peek()!=std::char_traits<char>::eof()||stream.bad())throw std::runtime_error("HKX input changed or failed while being read");
    return bytes;
}

Json inspect(const std::filesystem::path& path) {
    const auto bytes=read(path);fc::HkxClip clip;std::string error;
    if(!fc::decodeHkxAnimation(bytes,clip,error))throw std::runtime_error(error);
    return {{"ok",true},{"duration",clip.duration},{"frames",clip.frames.size()},{"tracks",clip.boneIndices.size()}};
}

Json exportGroup(const std::filesystem::path& path) {
    const auto bytes=read(path,256*1024);const auto job=Json::parse(bytes);
    auto file=[&](const char* key){const auto text=job.at(key).get<std::string>();return std::filesystem::path(std::u8string(reinterpret_cast<const char8_t*>(text.data()),text.size()));};
    const auto input=file("input"),pack=file("pack"),output=file("output"),work=file("work");
    const auto source=read(input);fc::ConverterPreparedClip prepared;std::string error;
    if(!fc::decodeConverterInput(source,prepared.clip,error))throw std::runtime_error(error);
    prepared.config=job.at("config");prepared.snapshots.push_back({input,source});prepared.snapshots.push_back({path,bytes});
    const auto member=prepared.config.value("member",std::string{});
    if(prepared.config.contains("frameRange")&&fc::converter::isWallRunPrimary(member))prepared.direction=member;
    return fc::convertHkxGroup({input,pack,output,work,prepared.config.at("slot").get<std::string>(),true},{prepared});
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
        return output(command=="inspect"?inspect(path):command=="export-group"?exportGroup(path):validate(path));
    } catch(const std::exception& error) {
        return failure(error.what());
    }
}
}

#ifdef _WIN32
int wmain(int argc,wchar_t** argv) {
    try {
        if(argc!=3)return failure("Usage: FreeClimbAuthoring inspect <animation.hkx> | validate <pack.json> | export-group <job.json>");
        const std::wstring_view command=argv[1];
        if(command!=L"inspect"&&command!=L"validate"&&command!=L"export-group")return failure("Unknown command; use inspect, validate or export-group");
        return run(command==L"inspect"?"inspect":command==L"export-group"?"export-group":"validate",std::filesystem::path(argv[2]));
    } catch(const std::exception& error) {
        return failure(error.what());
    }
}
#else
int main(int argc,char** argv) {
    try {
        if(argc!=3)return failure("Usage: FreeClimbAuthoring inspect <animation.hkx> | validate <pack.json> | export-group <job.json>");
        const std::string_view command=argv[1];
        if(command!="inspect"&&command!="validate"&&command!="export-group")return failure("Unknown command; use inspect, validate or export-group");
        return run(command,std::filesystem::path(argv[2]));
    } catch(const std::exception& error) {
        return failure(error.what());
    }
}
#endif
