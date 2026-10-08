#include "ConverterGuiProcess.h"
#include "ConverterTheme.h"
#include "../../src/MotionSlots.h"
#include <chrono>
#include <future>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {
unsigned checks{};
unsigned reusedHandleValues{};
void check(bool value,const char* label) {++checks;if(!value)throw std::runtime_error(label);}
std::string utf8(std::wstring_view input) {
    const int count=WideCharToMultiByte(CP_UTF8,WC_ERR_INVALID_CHARS,input.data(),int(input.size()),nullptr,0,nullptr,nullptr);
    if(!count)throw std::runtime_error("UTF-8 conversion failed");std::string result(count,'\0');
    WideCharToMultiByte(CP_UTF8,WC_ERR_INVALID_CHARS,input.data(),int(input.size()),result.data(),count,nullptr,nullptr);return result;
}
std::vector<std::wstring> split(std::wstring value) {
    int count{};auto* args=CommandLineToArgvW(value.c_str(),&count);if(!args)throw std::runtime_error("Argv split failed");
    std::vector<std::wstring> result;for(int i=0;i<count;++i)result.emplace_back(args[i]);LocalFree(args);return result;
}
int backend(int count,wchar_t** args) {
    if(count!=10&&count!=11)return 2;
    const std::wstring mode(args[3]);
    if(mode==L"large") {std::cout<<std::string(2*1024*1024,'x');return 1;}
    if(mode==L"delay")Sleep(160);
    Json values=Json::array();for(int i=0;i<count;++i)values.push_back(utf8(args[i]));
    if(mode==L"error")std::cerr<<"isolated diagnostic";
    bool candidateValid=false,signalSucceeded=false;
    if(mode==L"inherit"||mode==L"reuse") {
        const auto candidate=reinterpret_cast<HANDLE>(std::stoull(args[9]));DWORD flags{};
        std::vector<std::unique_ptr<Handle>> localEvents;
        for(unsigned attempt=0;mode==L"reuse"&&attempt<2048&&!GetHandleInformation(candidate,&flags);++attempt) {
            auto event=std::make_unique<Handle>(CreateEventW(nullptr,FALSE,FALSE,nullptr));if(!*event)return 1;
            localEvents.push_back(std::move(event));
        }
        candidateValid=GetHandleInformation(candidate,&flags)!=0;signalSucceeded=SetEvent(candidate)!=0;
    }
    const bool ok=mode!=L"error";
    std::cout<<Json{{"ok",ok},{"output",utf8(args[9])},{"slot",utf8(args[5])},{"duration",1.0},{"frames",61},
        {"warnings",Json::array()},{"error",ok?"":"controlled failure"},{"args",values},{"candidateValid",candidateValid},{"signalSucceeded",signalSucceeded},
        {"console",GetConsoleWindow()!=nullptr}}.dump();return ok?0:1;
}
void contractTests() {
    for(const auto color:{fc::converter::theme::canvas,fc::converter::theme::panel,fc::converter::theme::field,fc::converter::theme::hover,fc::converter::theme::border,fc::converter::theme::text,fc::converter::theme::muted,fc::converter::theme::disabled,fc::converter::theme::accent})check(GetRValue(color)==GetGValue(color)&&GetGValue(color)==GetBValue(color),"author interface palette is neutral grey without a blue tint");
    for(const int dpi:{96,120,144,192})for(const SIZE work:std::array<SIZE,3>{{{1920,1080},{2560,1440},{3840,2160}}}){
        const auto size=fc::converter::theme::initialSize(dpi,work.cx,work.cy);check(size.cx>0&&size.cy>0&&size.cx<work.cx&&size.cy<work.cy,"default author window stays inside work area at every DPI");
        check(size.cx<=MulDiv(1440,dpi,96)&&size.cy<=MulDiv(1000,dpi,96),"default author window uses larger bounded logical dimensions");
    }
    std::size_t index=0;
    for(const auto name:fc::motionSlotNames)if(!name.empty()) {
        check(index<fc::converter::slots.size(),"slot overflow");
        const std::wstring expected(name.begin(),name.end());check(expected==fc::converter::slots[index++],"slot parity");
    }
    check(index==31,"31 active slots");
    constexpr std::array<std::wstring_view,11> paths{L"E:\\动作库\\攀爬 测试.hkx",L"E:\\space dir\\filename.hkx",L"\\\\server\\share\\test.hkx",
        L"C:\\trail\\",L"name\"quotes\".hkx",L"name\\\"end\\",L"&whoami|>file$(`thing`)",L"literal\nnewline",L"",L"a\\\\b\\",L"a\"\\\\\"b"};
    for(const auto a:paths)for(const auto b:paths) {
        const auto decoded=split(L"dummy "+fc::converter::quoteArgument(a)+L" "+fc::converter::quoteArgument(b));
        check(decoded.size()==3,"quoted argument count");check(decoded[1]==a&&decoded[2]==b,"quoted argument roundtrip");
    }
    Request request{L"E:\\Tools Test\\FreeClimbHKXConverter.exe",L"E:\\动作库\\向上.hkx",L"contextMantle",L"E:\\Base\\pack.json",L"E:\\Output Test\\mod.zip"};
    for(const auto slot:fc::converter::slots) {
        request.slot=slot;const auto command=fc::converter::commandLine(request);check(command.has_value(),"valid request");
        const auto args=split(*command);check(args.size()==10,"request count");check(args[0]==request.executable&&args[3]==request.input&&args[5]==slot&&args[7]==request.pack&&args[9]==request.output,"request argv");
    }
    request.overwrite=true;const auto args=split(*fc::converter::commandLine(request));check(args.size()==11&&args[10]==L"--overwrite","explicit overwrite");
    for(const auto removed:{L"freeHang",L"sprintCatch",L"flipUp",L"flipLeft",L"flipRight"}){request.slot=removed;check(!fc::converter::commandLine(request),"removed slot is absent from tool commands");}
    request.slot=L"backFlipOut";check(fc::converter::commandLine(request).has_value(),"wall departure backflip remains available");request.slot=L"hang";
    request.input.clear();check(!fc::converter::commandLine(request),"empty path rejected");request.input=std::wstring(L"a\0b",3);
    check(!fc::converter::commandLine(request),"NUL path rejected");request.input.assign(33000,L'a');check(!fc::converter::commandLine(request),"oversized path rejected");
    request.input=L"input.hkx";request.pack=L"runtime/pack.json";request.output=L"out.zip";request.executable=L"converter.exe";
    check(fc::converter::absolutePaths(request),"absolute paths");check(std::filesystem::path(request.input).is_absolute()&&std::filesystem::path(request.output).is_absolute(),"all paths absolute");
    request.input.clear();check(!fc::converter::absolutePaths(request),"absolute empty rejected");
    const std::filesystem::path root=L"E:\\源码 测试\\FreeClimb";
    const auto expected=root/L"runtime"/L"meshes"/L"actors"/L"character"/L"animations"/L"FreeClimb"/L"pack.json";
    const auto found=fc::converter::discoverPack(root/L"tools"/L"converter"/L"bin"/L"FreeClimbConverter.exe",[&](const auto& path){return path==expected;});
    check(found&&*found==expected,"source runtime discovery");
    const auto unpacked=root/L"meshes"/L"actors"/L"character"/L"animations"/L"FreeClimb"/L"pack.json";
    check(fc::converter::discoverPack(root/L"FreeClimbConverter.exe",[&](const auto& path){return path==unpacked;})==unpacked,"unpacked discovery");
    check(!fc::converter::discoverPack(root/L"tools"/L"converter.exe",[](const auto&){return false;}),"missing base discovery");
    check(fromUtf8(utf8(L"测试 斜跑 🧗"))==L"测试 斜跑 🧗","UTF-8 roundtrip");check(fromUtf8(std::string("\xff",1))==L"Invalid UTF-8 output.","invalid UTF-8 rejected");
}
void drainTests() {
    SECURITY_ATTRIBUTES security{sizeof(security),nullptr,TRUE};HANDLE a{},b{};
    check(CreatePipe(&a,&b,&security,1024*1024)!=0,"burst pipe created");Handle reader(a),writer(b);
    const std::string burst(512*1024,'x');
    auto writing=std::async(std::launch::async,[&]{DWORD written{};return WriteFile(writer.value,burst.data(),DWORD(burst.size()),&written,nullptr)&&written==burst.size();});
    const bool buffered=writing.wait_for(std::chrono::seconds(1))==std::future_status::ready;
    std::string output;bool truncated=false;
    if(!buffered)while(writing.wait_for(std::chrono::milliseconds(1))!=std::future_status::ready)drain(reader,output,1024*1024,truncated);
    check(writing.get(),"burst pipe filled");check(buffered,"burst buffered for bounded-read fixture");
    const auto first=drain(reader,output,4096,truncated);check(first==64*4096,"single drain read budget");
    check(output.size()==4096&&truncated,"drain output cap");DWORD available{};
    check(PeekNamedPipe(reader.value,nullptr,0,nullptr,&available,nullptr)&&available==burst.size()-first,"remaining burst retained");
    const auto second=drain(reader,output,4096,truncated);check(second==64*4096,"next drain continues burst");
    check(output.size()==4096,"discarded excess does not grow output");
    check(PeekNamedPipe(reader.value,nullptr,0,nullptr,&available,nullptr)&&available==0,"outer iterations fully drain burst");
}
void inheritancePositiveControl(const Request& request,HANDLE event) {
    SECURITY_ATTRIBUTES security{sizeof(security),nullptr,TRUE};
    Handle sink(CreateFileW(L"NUL",GENERIC_READ|GENERIC_WRITE,FILE_SHARE_READ|FILE_SHARE_WRITE,&security,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr));
    check(bool(sink),"positive inheritance sink created");
    STARTUPINFOW startup{};startup.cb=sizeof(startup);startup.dwFlags=STARTF_USESTDHANDLES;
    startup.hStdInput=sink.value;startup.hStdOutput=sink.value;startup.hStdError=sink.value;
    PROCESS_INFORMATION info{};auto arguments=fc::converter::commandLine(request);
    check(arguments.has_value(),"positive inheritance arguments valid");auto command=*arguments;
    check(CreateProcessW(request.executable.c_str(),command.data(),nullptr,nullptr,TRUE,CREATE_NO_WINDOW,nullptr,nullptr,&startup,&info)!=0,"positive inheritance child created");
    Handle process(info.hProcess),thread(info.hThread);const auto wait=WaitForSingleObject(process.value,5000);
    if(wait!=WAIT_OBJECT_0){TerminateProcess(process.value,1);WaitForSingleObject(process.value,5000);}
    check(wait==WAIT_OBJECT_0,"positive inheritance child bounded");DWORD exit{};
    check(GetExitCodeProcess(process.value,&exit)&&exit==0,"positive inheritance child success");
    check(WaitForSingleObject(event,0)==WAIT_OBJECT_0,"actual inherited object detected");
}
void processTests(const std::filesystem::path& executable) {
    Request request{executable.wstring(),L"E:\\动作库\\向上 &测试.hkx",L"up",L"E:\\完整 基础包\\pack.json",L"E:\\我的 覆盖包\\动作.zip"};
    auto result=run(request);check(result.failure.empty()&&result.exit==0,"backend success");
    const auto report=Json::parse(result.output);check(report["args"].size()==10,"actual argv count");
    check(fromUtf8(report["args"][3].get<std::string>())==request.input&&fromUtf8(report["args"][7].get<std::string>())==request.pack&&fromUtf8(report["args"][9].get<std::string>())==request.output,"actual Unicode argument roundtrip");
    check(!report["console"].get<bool>(),"child console hidden");check(result.diagnostic.empty(),"stdout separated");
    request.overwrite=true;result=run(request);check(Json::parse(result.output)["args"][10]=="--overwrite","actual overwrite arg");
    request.input=L"error";result=run(request);check(result.exit==1&&result.failure.empty(),"conversion failure code");
    check(!Json::parse(result.output)["ok"].get<bool>()&&result.diagnostic=="isolated diagnostic","stderr isolated from JSON");
    request.input=L"large";result=run(request);check(!result.failure.empty()&&result.output.size()==1024*1024,"bounded output drained");
    request.input=L"delay";auto pending=std::async(std::launch::async,[&]{return run(request);});unsigned freeWork=0;
    while(pending.wait_for(std::chrono::milliseconds(1))!=std::future_status::ready)++freeWork;
    check(freeWork>2&&pending.get().exit==0,"caller remains responsive");
    SECURITY_ATTRIBUTES security{sizeof(security),nullptr,TRUE};Handle event(CreateEventW(&security,FALSE,FALSE,nullptr));
    check(bool(event),"test inherited event created");request.input=L"inherit";request.output=std::to_wstring(reinterpret_cast<std::uintptr_t>(event.value));
    for(unsigned attempt=0;attempt<8;++attempt) {
        result=run(request);check(result.failure.empty()&&result.exit==0,"inheritance probe completed");
        const auto probe=Json::parse(result.output);check(probe["candidateValid"].is_boolean()&&probe["signalSucceeded"].is_boolean(),"inheritance probe report valid");
        check(WaitForSingleObject(event.value,0)==WAIT_TIMEOUT,"unrelated inheritable object excluded");
        reusedHandleValues+=probe["candidateValid"].get<bool>();
    }
    inheritancePositiveControl(request,event.value);
    request.input=L"reuse";result=run(request);check(result.failure.empty()&&result.exit==0,"reused local handle probe completed");
    check(Json::parse(result.output)["candidateValid"].get<bool>(),"same numeric child handle value reproduced");
    check(WaitForSingleObject(event.value,0)==WAIT_TIMEOUT,"reused child handle is not the parent object");++reusedHandleValues;
    request.slot=L"nonexistent";result=run(request);check(!result.failure.empty(),"invalid request fails before launch");
    request.slot=L"hang";request.executable=(executable.parent_path()/L"backend-missing.exe").wstring();result=run(request);check(!result.failure.empty(),"missing executable rejected");
}
void editorStateTests(const std::filesystem::path& executable) {
    const auto gui=executable.parent_path()/L"FreeClimbConverter.exe";
    const auto pack=fc::converter::discoverPack(executable,[](const auto& path){return std::filesystem::is_regular_file(path);});
    check(pack.has_value()&&std::filesystem::is_regular_file(gui),"state regression requires actual GUI and complete base pack");
    const auto directory=executable.parent_path()/L"gui-state-tests"/(L"run-"+std::to_wstring(std::chrono::high_resolution_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(directory);const auto report=directory/L"report.json";
    auto command=fc::converter::quoteArgument(gui.wstring())+L" --verify-state "+fc::converter::quoteArgument(pack->wstring())+L" "+fc::converter::quoteArgument(report.wstring());
    STARTUPINFOW startup{sizeof(startup)};PROCESS_INFORMATION info{};
    check(CreateProcessW(gui.c_str(),command.data(),nullptr,nullptr,FALSE,CREATE_NO_WINDOW,nullptr,directory.c_str(),&startup,&info)!=0,"actual hidden editor state regression started");
    Handle process(info.hProcess),thread(info.hThread);const auto wait=WaitForSingleObject(process.value,180000);
    if(wait!=WAIT_OBJECT_0){TerminateProcess(process.value,1);WaitForSingleObject(process.value,5000);}
    check(wait==WAIT_OBJECT_0,"hidden editor state regression completed");DWORD exit{};check(GetExitCodeProcess(process.value,&exit)!=0,"hidden editor state exit available");
    std::ifstream file(report,std::ios::binary);Json result;file>>result;
    if(exit||!result.value("ok",false))throw std::runtime_error("Editor state regression failed: "+result.value("error",std::string("unknown")));
    check(result.value("hiddenWindow",false)&&!result.value("guiLaunched",true)&&result.value("checks",0)>=15,"real editor identity, recovery and export checks passed");
    std::cout<<"Editor state report: "<<utf8(report.wstring())<<'\n';
}
}
int wmain(int count,wchar_t** args) {
    if(count>1&&std::wstring_view(args[1])==L"convert")return backend(count,args);
    try {
        contractTests();drainTests();std::vector<wchar_t> path(32768);const auto length=GetModuleFileNameW(nullptr,path.data(),DWORD(path.size()));
        check(length>0&&length<path.size(),"test executable path");const std::filesystem::path original(path.data());
        const auto directory=original.parent_path()/L"Unicode 路径 测试";std::filesystem::create_directories(directory);
        const auto copy=directory/L"Backend 测试.exe";std::filesystem::copy_file(original,copy,std::filesystem::copy_options::overwrite_existing);
        processTests(copy);editorStateTests(original);std::cout<<Json{{"ok",true},{"checks",checks},{"reusedHandleValues",reusedHandleValues},{"guiLaunched",false},{"usesPython",false},{"usesDotNet",false}}.dump()<<'\n';return 0;
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
