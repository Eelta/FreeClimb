#include "Converter.h"
#include <iostream>
#include <map>

namespace {
int run(const std::vector<std::filesystem::path>& arguments) {
    using Json=nlohmann::json;
    try {
        if(arguments.empty()||arguments[0]!="convert") {std::cout<<Json{{"ok",false},{"error","Use convert --input <HKX> --slot <name> --pack <pack.json> --output <ZIP> [--overwrite]"}}.dump()<<'\n';return 2;}
        fc::ConverterRequest request;std::map<std::string,std::filesystem::path> options;
        for(std::size_t i=1;i<arguments.size();++i) {
            const auto text=arguments[i].u8string();const std::string key(reinterpret_cast<const char*>(text.data()),text.size());
            if(key=="--overwrite") {if(request.overwrite)throw std::invalid_argument("Duplicate --overwrite");request.overwrite=true;continue;}
            if((key!="--input"&&key!="--slot"&&key!="--pack"&&key!="--output")||i+1>=arguments.size()||!options.emplace(key,arguments[++i]).second)throw std::invalid_argument("Unknown, missing or duplicate argument");
        }
        for(const auto* key:{"--input","--slot","--pack","--output"})if(!options.contains(key))throw std::invalid_argument(std::string("Missing ")+key);
        request.input=options.at("--input");request.pack=options.at("--pack");request.output=options.at("--output");const auto slot=options.at("--slot").u8string();request.slot={reinterpret_cast<const char*>(slot.data()),slot.size()};
        std::cout<<fc::convertHkx(request).dump(-1,' ',true,Json::error_handler_t::replace)<<'\n';return 0;
    } catch(const std::invalid_argument& error) {std::cout<<Json{{"ok",false},{"error",error.what()}}.dump()<<'\n';return 2;}
    catch(const std::exception& error) {std::cout<<Json{{"ok",false},{"error",error.what()}}.dump(-1,' ',true,Json::error_handler_t::replace)<<'\n';return 1;}
}
}
#ifdef _WIN32
int wmain(int argc,wchar_t** argv) {std::vector<std::filesystem::path> arguments;for(int i=1;i<argc;++i)arguments.emplace_back(argv[i]);return run(arguments);}
#else
int main(int argc,char** argv) {std::vector<std::filesystem::path> arguments;for(int i=1;i<argc;++i)arguments.emplace_back(argv[i]);return run(arguments);}
#endif
