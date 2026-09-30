#pragma once
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/null_sink.h>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <string>

namespace fc {

inline std::filesystem::path runtimeDataDirectory() {
    std::wstring executable(512,L'\0');
    for(;;) {
        const auto count=GetModuleFileNameW(nullptr,executable.data(),static_cast<DWORD>(executable.size()));
        if(!count)throw std::runtime_error("Cannot locate the game executable");
        if(count<executable.size()) {
            executable.resize(count);
            const std::filesystem::path path(executable);
            if(!path.is_absolute())throw std::runtime_error("Game executable path is not absolute");
            return path.parent_path()/L"Data";
        }
        if(executable.size()>=32768)throw std::runtime_error("Game executable path is too long");
        executable.resize(executable.size()*2);
    }
}

inline std::filesystem::path runtimeLogPath() {
    return runtimeDataDirectory()/L"SKSE"/L"FreeClimb.log";
}

class RuntimeFileSink final:public spdlog::sinks::base_sink<std::mutex> {
    std::ofstream file;
protected:
    void sink_it_(const spdlog::details::log_msg& message)override {
        spdlog::memory_buf_t formatted;
        formatter_->format(message,formatted);
        file.write(formatted.data(),static_cast<std::streamsize>(formatted.size()));
    }
    void flush_()override {file.flush();}
public:
    explicit RuntimeFileSink(const std::filesystem::path& path) {
        if(!path.is_absolute())throw std::runtime_error("Log path is not absolute");
        std::filesystem::create_directories(path.parent_path());
        file.exceptions(std::ios::badbit|std::ios::failbit);
        file.open(path,std::ios::binary|std::ios::out|std::ios::trunc);
    }
};

inline bool initializeLogging(const std::filesystem::path& path={}) {
    auto logger=std::make_shared<spdlog::logger>("FreeClimb",std::make_shared<spdlog::sinks::null_sink_mt>());
    bool opened=false;
    try {
        logger->sinks().front()=std::make_shared<RuntimeFileSink>(path.empty()?runtimeLogPath():path);
        opened=true;
    } catch(const std::exception&) {
        OutputDebugStringW(L"FreeClimb: cannot open Data\\SKSE\\FreeClimb.log; file logging is disabled.\n");
    }
    logger->set_level(spdlog::level::info);
    logger->flush_on(spdlog::level::info);
    spdlog::set_default_logger(std::move(logger));
    return opened;
}

}
