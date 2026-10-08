#pragma once
#include <array>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace fc::converter {
inline constexpr std::array<std::wstring_view,31> slots{
    L"hang",L"up",L"down",L"left",L"right",L"reach",L"hopLeft",L"hopRight",L"hopUp",L"drop",
    L"jumpCatch",L"dropBack",L"ledgeCatch",L"runUp",L"runLeft",L"runRight",
    L"runDiagonalLeft",L"runDiagonalRight",L"runLaunch",L"runCatch",L"kickUp",L"kickLeft",L"kickRight",
    L"runLaunchLeft",L"runLaunchRight",L"sideBrace",L"backFlipOut",
    L"contextHang",L"contextHopLeft",L"contextHopRight",L"contextMantle"
};
inline constexpr std::array<std::wstring_view,20> primarySlots{
    L"hang",L"up",L"down",L"left",L"right",L"reach",L"hopLeft",L"hopRight",L"hopUp",L"drop",
    L"jumpCatch",L"dropBack",L"ledgeCatch",L"wallRun",L"kickUp",L"kickLeft",L"kickRight",L"backFlipOut",L"contextHop",L"contextMantle"
};
inline std::wstring quoteArgument(std::wstring_view value) {
    std::wstring output=L"\"";std::size_t slashes=0;
    for(wchar_t c:value) {
        if(c==L'\\'){++slashes;continue;}
        if(c==L'\"')output.append(slashes*2+1,L'\\');else output.append(slashes,L'\\');
        slashes=0;output.push_back(c);
    }
    output.append(slashes*2,L'\\');output.push_back(L'\"');return output;
}
struct Request {std::wstring executable,input,slot,pack,output;bool overwrite{};};
inline std::optional<std::wstring> commandLine(const Request& request) {
    bool known=false;for(const auto slot:slots)known=known||slot==request.slot;
    if(!known)return {};
    const std::array<std::wstring_view,10> args{request.executable,L"convert",L"--input",request.input,
        L"--slot",request.slot,L"--pack",request.pack,L"--output",request.output};
    std::wstring result;
    for(const auto arg:args) {
        if(arg.empty()||arg.find(L'\0')!=std::wstring_view::npos)return {};
        if(!result.empty())result.push_back(L' ');result+=quoteArgument(arg);
    }
    if(request.overwrite)result+=L" --overwrite";
    if(result.size()>=32767)return {};return result;
}
inline bool absolutePaths(Request& request) {
    for(auto* value:{&request.executable,&request.input,&request.pack,&request.output}) {
        if(value->empty()||value->find(L'\0')!=std::wstring::npos)return false;
        std::error_code error;const auto path=std::filesystem::absolute(std::filesystem::path(*value),error);
        if(error)return false;*value=path.lexically_normal().wstring();
    }
    return true;
}
template<class Exists> std::optional<std::filesystem::path> discoverPack(std::filesystem::path executable,Exists&& exists) {
    auto directory=executable.parent_path();
    const auto relative=std::filesystem::path(L"meshes")/L"actors"/L"character"/L"animations"/L"FreeClimb"/L"pack.json";
    for(unsigned depth=0;depth<6&&!directory.empty();++depth) {
        for(const auto candidate:{directory/L"runtime"/relative,directory/relative,directory/L"pack.json"})
            if(exists(candidate))return candidate;
        const auto parent=directory.parent_path();if(parent==directory)break;directory=parent;
    }
    return {};
}
}


