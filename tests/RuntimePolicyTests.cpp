#include "RuntimePolicy.h"
#include <iostream>
#include <stdexcept>

int main() {
    using namespace fc::runtime;
    const auto require=[](bool ok,const char* message){if(!ok)throw std::runtime_error(message);};
    try {
        require(supportedVersions.size()==13,"explicit public runtime list");
        for(std::size_t i=0;i<supportedVersions.size();++i) {
            const auto version=supportedVersions[i];
            require(supported(version)&&family(version)!=Family::unsupported,"listed runtime is classified");
            if(i)require(supportedVersions[i-1]<version,"runtime list must be unique and sorted");
        }
        require(family(pack(1,5,97))==Family::se&&addressFormat(pack(1,5,97))==1,"SE branch and address database");
        require(family(pack(1,6,318))==Family::ae&&family(pack(1,6,353))==Family::ae,"old AE structure branch");
        require(family(pack(1,6,629))==Family::ae629&&family(pack(1,6,1170))==Family::ae629,"post-629 structure branch");
        require(addressFormat(pack(1,6,1179))==2,"GOG AE address database");
        require(family(pack(1,7,99))==Family::ae17&&addressFormat(pack(1,7,99))==5,"new AE format 5");
        for(const auto version:{pack(1,4,15),pack(1,5,80),pack(1,6,117),pack(1,6,628),pack(1,6,999),
            pack(1,7,98),pack(1,7,104),pack(1,7,99,1),pack(2,5,97),0u})
            require(!supported(version)&&family(version)==Family::unsupported&&addressFormat(version)==0,
                "unknown, truncated, VR and future versions must not acquire engine hooks");
        std::cout<<"PASS: 13 explicit runtimes, SE / AE / post-629 / format-5 families and rejected unknown versions\n";
        return 0;
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
