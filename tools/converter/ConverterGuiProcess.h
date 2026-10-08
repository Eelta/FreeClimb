#pragma once
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <shellapi.h>
#include "ConverterGuiContract.h"
#include "../../external/nlohmann/json.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <vector>

namespace {
using fc::converter::Request;
using Json=nlohmann::json;
struct Handle {
    HANDLE value{};
    Handle()=default;explicit Handle(HANDLE v):value(v){}
    Handle(const Handle&)=delete;Handle& operator=(const Handle&)=delete;
    ~Handle(){close();}
    void close(){if(value&&value!=INVALID_HANDLE_VALUE)CloseHandle(value);value=nullptr;}
    explicit operator bool()const{return value&&value!=INVALID_HANDLE_VALUE;}
};
struct RunResult {DWORD exit{1};std::string output,diagnostic;std::wstring failure;};
std::wstring systemError(DWORD code) {
    wchar_t* text{};const auto size=FormatMessageW(FORMAT_MESSAGE_ALLOCATE_BUFFER|FORMAT_MESSAGE_FROM_SYSTEM|FORMAT_MESSAGE_IGNORE_INSERTS,
        nullptr,code,0,reinterpret_cast<wchar_t*>(&text),0,nullptr);
    std::wstring value=size?std::wstring(text,size):L"Windows error "+std::to_wstring(code);if(text)LocalFree(text);return value;
}
std::wstring fromUtf8(const std::string& text) {
    if(text.empty())return {};
    const int count=MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,text.data(),int(text.size()),nullptr,0);
    if(!count)return L"Invalid UTF-8 output.";std::wstring result(count,L'\0');
    MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,text.data(),int(text.size()),result.data(),count);return result;
}
bool pipe(Handle& read,Handle& write,SECURITY_ATTRIBUTES& security) {
    HANDLE a{},b{};if(!CreatePipe(&a,&b,&security,0))return false;read.value=a;write.value=b;
    return SetHandleInformation(read.value,HANDLE_FLAG_INHERIT,0)!=0;
}
std::size_t drain(Handle& reader,std::string& destination,std::size_t maximum,bool& truncated) {
    std::size_t total=0;
    for(unsigned attempt=0;attempt<64;++attempt) {
        DWORD available{};if(!PeekNamedPipe(reader.value,nullptr,0,nullptr,&available,nullptr)||!available)break;
        std::array<char,4096> bytes{};DWORD read{};
        if(!ReadFile(reader.value,bytes.data(),std::min<DWORD>(available,DWORD(bytes.size())),&read,nullptr)||!read)break;
        const auto kept=std::min<std::size_t>(read,maximum-destination.size());destination.append(bytes.data(),kept);
        truncated=truncated||kept<read;total+=read;
    }
    return total;
}
RunResult run(const Request& request) {
    RunResult result;const auto arguments=fc::converter::commandLine(request);
    if(!arguments){result.failure=L"Invalid converter arguments.";return result;}
    SECURITY_ATTRIBUTES security{sizeof(security),nullptr,TRUE};Handle outputRead,outputWrite,errorRead,errorWrite;
    if(!pipe(outputRead,outputWrite,security)||!pipe(errorRead,errorWrite,security)) {result.failure=systemError(GetLastError());return result;}
    Handle input(CreateFileW(L"NUL",GENERIC_READ,FILE_SHARE_READ|FILE_SHARE_WRITE,&security,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr));
    Handle job(CreateJobObjectW(nullptr,nullptr));JOBOBJECT_EXTENDED_LIMIT_INFORMATION limits{};
    limits.BasicLimitInformation.LimitFlags=JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
    if(!input||!job||!SetInformationJobObject(job.value,JobObjectExtendedLimitInformation,&limits,sizeof(limits))) {
        result.failure=systemError(GetLastError());return result;
    }
    STARTUPINFOEXW startup{};startup.StartupInfo.cb=sizeof(startup);startup.StartupInfo.dwFlags=STARTF_USESTDHANDLES;
    startup.StartupInfo.hStdInput=input.value;startup.StartupInfo.hStdOutput=outputWrite.value;startup.StartupInfo.hStdError=errorWrite.value;
    SIZE_T attributeBytes{};InitializeProcThreadAttributeList(nullptr,1,0,&attributeBytes);
    if(!attributeBytes){result.failure=systemError(GetLastError());return result;}
    std::vector<std::byte> attributes(attributeBytes);startup.lpAttributeList=reinterpret_cast<LPPROC_THREAD_ATTRIBUTE_LIST>(attributes.data());
    if(!InitializeProcThreadAttributeList(startup.lpAttributeList,1,0,&attributeBytes)){result.failure=systemError(GetLastError());return result;}
    const std::array<HANDLE,3> inherited{input.value,outputWrite.value,errorWrite.value};
    const bool inheritedSet=UpdateProcThreadAttribute(startup.lpAttributeList,0,PROC_THREAD_ATTRIBUTE_HANDLE_LIST,
        const_cast<HANDLE*>(inherited.data()),sizeof(inherited),nullptr,nullptr)!=0;
    PROCESS_INFORMATION processInfo{};auto command=*arguments;const auto directory=std::filesystem::path(request.executable).parent_path();
    const bool created=inheritedSet&&CreateProcessW(request.executable.c_str(),command.data(),nullptr,nullptr,TRUE,
        CREATE_NO_WINDOW|CREATE_SUSPENDED|EXTENDED_STARTUPINFO_PRESENT,nullptr,directory.c_str(),&startup.StartupInfo,&processInfo);
    const DWORD createError=created?0:GetLastError();DeleteProcThreadAttributeList(startup.lpAttributeList);
    if(!created){result.failure=systemError(createError);return result;}
    Handle process(processInfo.hProcess),thread(processInfo.hThread);
    if(!AssignProcessToJobObject(job.value,process.value)||ResumeThread(thread.value)==DWORD(-1)) {
        const auto error=GetLastError();TerminateProcess(process.value,1);result.failure=systemError(error);return result;
    }
    outputWrite.close();errorWrite.close();input.close();thread.close();bool truncated=false;const auto started=GetTickCount64();
    for(;;) {
        drain(outputRead,result.output,1024*1024,truncated);drain(errorRead,result.diagnostic,64*1024,truncated);
        const auto wait=WaitForSingleObject(process.value,20);if(wait==WAIT_OBJECT_0)break;
        if(wait==WAIT_FAILED){result.failure=systemError(GetLastError());return result;}
        if(GetTickCount64()-started>10*60*1000){TerminateJobObject(job.value,1);result.failure=L"The conversion exceeded the time limit.";return result;}
    }
    drain(outputRead,result.output,1024*1024,truncated);drain(errorRead,result.diagnostic,64*1024,truncated);
    if(!GetExitCodeProcess(process.value,&result.exit)){result.failure=systemError(GetLastError());return result;}
    if(truncated)result.failure=L"The converter returned too much output.";return result;
}
}
