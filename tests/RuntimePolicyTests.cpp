#include "RuntimePolicy.h"
#include <iostream>
#include <stdexcept>

int main() {
    using namespace fc::runtime;
    const auto require=[](bool ok,const char* message){if(!ok)throw std::runtime_error(message);};
    try {
        require(supportedVersions.size()==14,"explicit public runtime list");
        require(supportedSKSEVersions.size()==supportedVersions.size(),"loader versions map one-to-one to executable versions");
        require(pack(1,6,1179,1)==0x010649B1u&&pack(1,6,659,1)==0x01062931u,"official GOG packed runtime identifiers");
        for(std::size_t i=0;i<supportedVersions.size();++i) {
            const auto version=supportedVersions[i];
            require(supported(version)&&family(version)!=Family::unsupported,"listed runtime is classified");
            if(i)require(supportedVersions[i-1]<version,"runtime list must be unique and sorted");
            const auto encoded=skseVersion(version);
            const bool gog=version==pack(1,6,659)||version==pack(1,6,1179);
            require(supportedSKSEVersions[i]==encoded&&supportedSKSE(encoded),"each executable has an accepted exact loader encoding");
            require(gameVersionFromSKSE(encoded)==version,"accepted loader encoding resolves to the executable version");
            require(encoded==(version|(gog?1u:0u)),"only the two known GOG releases use a platform bit");
            if(i)require(supportedSKSEVersions[i-1]<encoded,"loader version list must be unique and sorted");
            for(unsigned platform=0;platform<16;++platform){
                const auto raw=version|platform;
                require(supportedSKSE(raw)==(platform==(gog?1u:0u)),"unknown loader platform variants are rejected without masking");
                require(gameVersionFromSKSE(raw)==(gog&&platform==1?version:raw),"normalization only recognizes exact known GOG loader values");
                require(supported(raw)==(platform==0),"file version support never accepts loader platform bits");
            }
        }
        require(family(pack(1,5,97))==Family::se&&addressFormat(pack(1,5,97))==1,"SE branch and address database");
        require(family(pack(1,6,318))==Family::ae&&family(pack(1,6,353))==Family::ae,"old AE structure branch");
        require(family(pack(1,6,629))==Family::ae629&&family(pack(1,6,1170))==Family::ae629,"post-629 structure branch");
        require(addressFormat(pack(1,6,1179))==2,"GOG AE address database");
        for(const auto raw:{0x01062931u,0x010649B1u}){
            const auto game=gameVersionFromSKSE(raw);
            require(supportedSKSE(raw)&&!supported(raw),"GOG loader encoding is distinct from the executable whitelist");
            require(supported(game)&&!supportedSKSE(game),"GOG executable spelling cannot substitute for its loader encoding");
            require(family(raw)==Family::unsupported&&addressFormat(raw)==0,"loader platform bit cannot reach structure or database selection");
            require(family(game)==Family::ae629&&addressFormat(game)==2,"normalized GOG version selects post-629 AE and format 2");
        }
        require(family(pack(1,7,99))==Family::ae17&&addressFormat(pack(1,7,99))==5,"new AE format 5");
        require(supported(pack(1,7,104))&&family(pack(1,7,104))==Family::ae17&&
            addressFormat(pack(1,7,104))==5,"Steam 1.7.104 uses AE format 5");
        for(const auto version:{pack(1,4,15),pack(1,5,80),pack(1,6,117),pack(1,6,628),pack(1,6,999),
            pack(1,7,98),pack(1,7,100),pack(1,7,103),pack(1,7,105),pack(1,7,99,1),pack(1,7,104,1),pack(2,5,97),0u})
            require(!supported(version)&&family(version)==Family::unsupported&&addressFormat(version)==0,
                "unknown, truncated, VR and future versions must not acquire engine hooks");
        for(const auto raw:{pack(1,4,15),pack(1,4,15,1),pack(1,5,80),pack(1,6,117),pack(1,6,1178,1),
            pack(1,6,1180,1),pack(1,7,105,1),pack(1,7,999,2),pack(2,6,1179,1),0u}){
            require(!supportedSKSE(raw),"unknown executable releases remain rejected by loader policy");
            require(gameVersionFromSKSE(raw)==raw&&skseVersion(raw)==raw,"unsupported values are never rewritten into a known runtime");
        }
        std::cout<<"PASS: 14 executable and loader runtime pairs, GOG platform encoding, structure/database families and rejected unknown versions\n";
        return 0;
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
