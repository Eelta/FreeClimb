#include "RuntimeLog.h"
#include <array>
#include <iostream>
#include <iterator>

static unsigned checks{};
static void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
static std::string contents(const std::filesystem::path& path) {
    std::ifstream file(path,std::ios::binary);
    return {std::istreambuf_iterator<char>(file),std::istreambuf_iterator<char>()};
}

int main(int argc,char** argv) {
    try {
        check(argc==2,"scratch directory argument required");
        const auto scratch=std::filesystem::absolute(argv[1]);
        std::filesystem::create_directories(scratch);
        std::array<wchar_t,32768> executable{};
        const auto count=GetModuleFileNameW(nullptr,executable.data(),static_cast<DWORD>(executable.size()));
        check(count>0&&count<executable.size(),"test process path is available");
        const auto expected=std::filesystem::path(std::wstring(executable.data(),count)).parent_path()/L"Data"/L"SKSE"/L"FreeClimb.log";
        check(fc::runtimeLogPath()==expected,"log path follows process executable");
        const auto dataDirectory=expected.parent_path().parent_path();
        check(fc::runtimeDataDirectory()==dataDirectory,"runtime data directory follows process executable");
        const auto translations=dataDirectory/L"Interface"/L"Translations";
        const auto originalDirectory=std::filesystem::current_path();
        std::filesystem::current_path(scratch);
        check(fc::runtimeLogPath()==expected,"working directory does not relocate log");
        check(fc::runtimeDataDirectory()/L"Interface"/L"Translations"==translations,"working directory does not relocate translation discovery");
        std::filesystem::current_path(originalDirectory);
        const auto log=scratch/L"日志 测试"/L"Data"/L"SKSE"/L"FreeClimb.log";
        check(fc::initializeLogging(log),"Unicode directory created and opened");
        spdlog::info("first session: 攀爬日志");
        check(contents(log).find("first session: 攀爬日志")!=std::string::npos,"UTF-8 message flushed to Unicode path");
        check(!fc::initializeLogging(std::filesystem::path(L"relative.log")),"relative destination rejected");
        spdlog::info("discarded after relative-path failure");
        const auto blocker=scratch/L"blocked-parent";
        std::ofstream(blocker,std::ios::binary|std::ios::trunc)<<"not a directory";
        check(!fc::initializeLogging(blocker/L"SKSE"/L"FreeClimb.log"),"unwritable destination returns without aborting");
        spdlog::info("discarded after directory creation failure");
        check(contents(blocker)=="not a directory","failed setup preserves unrelated file");
        check(fc::initializeLogging(log),"logging can initialize after failure");
        spdlog::info("second session");
        const auto current=contents(log);
        check(current.find("second session")!=std::string::npos,"second session flushed");
        check(current.find("first session")==std::string::npos,"new session replaces old contents");
        check(current.find("discarded")==std::string::npos,"fallback does not buffer unrelated records");
        spdlog::set_default_logger(std::make_shared<spdlog::logger>("test-end",std::make_shared<spdlog::sinks::null_sink_mt>()));
        std::cout<<checks<<" runtime logging checks passed\n";
        return 0;
    } catch(const std::exception& error) {
        std::cerr<<error.what()<<'\n';return 1;
    }
}
