#include "AnimationSkeletonBinding.h"
#include "CanonicalSkeleton.h"
#include <algorithm>
#include <array>
#include <iostream>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>
using namespace fc;
constexpr std::array<int,9> equipmentTracks{42,43,60,61,62,63,64,65,66};
constexpr std::array<int,4> magicTracks{46,47,48,49};
constexpr std::array<int,13> attachmentTracks{42,43,46,47,48,49,60,61,62,63,64,65,66};
constexpr std::array<int,18> optionalTracksList{1,2,3,42,43,46,47,48,49,60,61,62,63,64,65,66,97,98};
static bool unownedTrack(int i){return i>=97||std::find(attachmentTracks.begin(),attachmentTracks.end(),i)!=attachmentTracks.end();}
static unsigned checks{};
static void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
static std::vector<std::string> canonicalNames(){std::vector<std::string> result;for(auto name:canonicalBoneNames)result.emplace_back(name);return result;}
static std::vector<int> canonicalParents(){return {canonicalBoneParents.begin(),canonicalBoneParents.end()};}
struct Fixture {
    std::vector<std::string> names=canonicalNames();
    std::vector<int> parents=canonicalParents(),ids;
    Fixture(){ids.resize(names.size());std::iota(ids.begin(),ids.end(),0);}
    int index(int id)const{auto found=std::find(ids.begin(),ids.end(),id);return found==ids.end()?-1:int(found-ids.begin());}
    void alias(int id){const auto i=index(id);check(i>=0&&id>=1&&id<=3,"alias fixture selects a known helper");names[i].erase(0,2);}
    void extend(std::size_t count){while(names.size()<count){const auto i=names.size();names.push_back("Extension "+std::to_string(i));parents.push_back(i==99?index(0):int(i-1));ids.push_back(-1);}}
    void remove(int id){
        const int at=index(id);check(at>=0,"removed fixture track must exist");
        check(std::find(parents.begin(),parents.end(),at)==parents.end(),"fixture removes only leaves");
        names.erase(names.begin()+at);parents.erase(parents.begin()+at);ids.erase(ids.begin()+at);
        for(auto& parent:parents)if(parent>at)--parent;
    }
    void reorder(std::mt19937& random){
        std::vector<int> order(names.size()),inverse(names.size());std::iota(order.begin(),order.end(),0);std::shuffle(order.begin(),order.end(),random);
        for(std::size_t i=0;i<order.size();++i)inverse[order[i]]=int(i);
        const auto oldNames=names;const auto oldParents=parents,oldIds=ids;
        for(std::size_t i=0;i<order.size();++i){const auto old=order[i];names[i]=oldNames[old];parents[i]=oldParents[old]<0?-1:inverse[oldParents[old]];ids[i]=oldIds[old];}
    }
    AnimationSkeletonBinding bind(const std::vector<std::string>& expected=canonicalNames(),const std::vector<int>& hierarchy=canonicalParents())const{
        std::vector<AnimationSkeletonBone> bones;for(std::size_t i=0;i<names.size();++i)bones.push_back({names[i],parents[i]});
        return bindAnimationSkeleton(expected,hierarchy,bones);
    }
};
static void verify(const Fixture& fixture,const AnimationSkeletonBinding& binding){
    check(bool(binding),"compatible skeleton binds");
    std::size_t remapped=0,aliases=0,missing=0;
    std::vector<std::array<float,4>> references(fixture.names.size());std::vector<bool> locked(fixture.names.size());
    for(std::size_t i=0;i<fixture.ids.size();++i){const float id=float(fixture.ids[i]);references[i]={id*1.25f,id*-2.5f,id+17.f,id*.5f};locked[i]=fixture.ids[i]>=0&&fixture.ids[i]%3!=0;}
    for(int id=0;id<99;++id){
        const int expected=unownedTrack(id)?-1:fixture.index(id);
        check(binding.indices[id]==expected,"canonical track maps to its actual runtime bone");
        if(expected>=0){
            check(binding.parents[id]==fixture.parents[expected],"mapped parent snapshot uses actual runtime indices");check(binding.current(id,{fixture.names[expected],fixture.parents[expected]}),"captured runtime names and parents remain current");
            const auto& reference=references[binding.indices[id]];const float canonical=float(id);check(reference==std::array<float,4>{canonical*1.25f,canonical*-2.5f,canonical+17.f,canonical*.5f},"reference pose components read through mapping preserve canonical bone identity");check(locked[binding.indices[id]]==(id%3!=0),"lock translation flags read through mapping preserve canonical bone identity");
            remapped+=expected!=id;
            aliases+=id>=1&&id<=3&&fixture.names[expected]!=canonicalBoneNames[id];
        }else{check(!binding.current(id,{"Unbound",-1}),"missing helpers and engine-owned tracks cannot validate live output state");if(id>=1&&id<=3)++missing;}
    }
    check(binding.remapped==remapped,"remap diagnostic counts only bound reordered tracks");
    check(binding.aliases==aliases,"alias diagnostic counts only permitted helper aliases");
    check(binding.missingOptional==missing,"missing diagnostic excludes engine-owned attachments and cameras");
}
static void canonicalAndReport(){
    for(std::size_t count:{99u,116u,126u}){
        Fixture fixture;fixture.extend(count);verify(fixture,fixture.bind());
        fixture.alias(1);const auto reported=fixture.bind();verify(fixture,reported);
        check(reported.indices[1]==1&&reported.aliases==1,"reported NPC LookNode helper spelling binds without changing its parent");
        fixture.alias(2);fixture.alias(3);verify(fixture,fixture.bind());
    }
}
static void optionalTracks(){
    const auto& optional=optionalTracksList;
    for(int removed:optional){Fixture fixture;fixture.remove(removed);verify(fixture,fixture.bind());}
    Fixture minimal;for(int removed:optional)minimal.remove(removed);check(minimal.names.size()==81,"minimal fixture preserves all 81 required body tracks");verify(minimal,minimal.bind());
    for(int id=0;id<99;++id){
        if(std::find(optional.begin(),optional.end(),id)!=optional.end())continue;
        Fixture fixture;fixture.names[id]="Missing required "+std::to_string(id);const auto rejected=fixture.bind();
        check(!rejected,"every required canonical body track must be present");check(rejected.track==id,"missing required track identifies canonical index");
    }
    for(int id:equipmentTracks){Fixture fixture;fixture.names[id]="x_"+fixture.names[id];const auto result=fixture.bind();check(bool(result)&&result.indices[id]==-1&&result.aliases==0&&result.missingOptional==0,"unowned equipment names are not mistaken for missing optional helpers");}
    for(int id:{0,4,28,38,68}){Fixture fixture;fixture.names[id]="x_"+fixture.names[id];check(!fixture.bind(),"prefix aliases never apply to required body bones");}
    Fixture wrongCase;wrongCase.names[1]="NPC lookNode [Look]";const auto result=wrongCase.bind();check(bool(result)&&result.indices[1]==-1&&result.aliases==0,"helper spelling is case-sensitive and narrowly defined");
}
static void ambiguity(){
    for(int id:{0,1,2,3,28,38,96}){
        Fixture fixture;fixture.names.push_back(fixture.names[id]);fixture.parents.push_back(fixture.parents[id]);fixture.ids.push_back(-1);
        const auto duplicate=fixture.bind();check(!duplicate,"duplicate exact required or optional names are ambiguous");check(duplicate.track==id,"ambiguous match identifies canonical track");
    }
    for(int id:{1,2,3}){
        Fixture fixture;fixture.names.push_back(fixture.names[id].substr(2));fixture.parents.push_back(fixture.parents[id]);fixture.ids.push_back(-1);
        check(!fixture.bind(),"exact helper plus alias cannot choose an arbitrary match");
        fixture.alias(id);check(!fixture.bind(),"duplicate helper aliases cannot choose an arbitrary match");
    }
}
static void invalidTopology(){
    for(int id:{1,4,28,38,96}){Fixture fixture;fixture.parents[id]=fixture.parents[id]==0?4:0;const auto result=fixture.bind();check(!result,"mapped body and optional helpers retain their exact canonical parent");}
    for(int parent:{-2,99,1024}){Fixture fixture;fixture.parents[28]=parent;check(!fixture.bind(),"graph parent bounds are validated");}
    Fixture self;self.parents[28]=28;check(!self.bind(),"self-parent cycle rejected");
    Fixture bodyCycle;bodyCycle.parents[28]=29;bodyCycle.parents[29]=28;check(!bodyCycle.bind(),"required bone cycle rejected");
    Fixture extraCycle;extraCycle.extend(116);extraCycle.parents[99]=115;check(!extraCycle.bind(),"unmapped extension cycle rejected");
    Fixture cameraCycle;cameraCycle.parents[97]=98;cameraCycle.parents[98]=97;check(!cameraCycle.bind(),"engine-owned camera cycle still invalidates graph structure");
    Fixture extraBounds;extraBounds.extend(116);extraBounds.parents[115]=116;check(!extraBounds.bind(),"unmapped extension parent bounds checked");
    Fixture intermediate;intermediate.names.push_back("Additional parent");intermediate.parents.push_back(28);intermediate.ids.push_back(-1);intermediate.parents[29]=99;check(!intermediate.bind(),"unproven additional hka parent cannot silently change reference pose coordinates");
    Fixture wrongRoot;wrongRoot.parents[0]=97;check(!wrongRoot.bind(),"required root cannot acquire an engine-owned camera parent");
    Fixture helperBranch;auto parents=canonicalParents();parents[33]=1;helperBranch.parents[33]=1;helperBranch.alias(1);check(!helperBranch.bind(canonicalNames(),parents),"helper aliases require a canonical animation-only leaf");
}
static void equipmentTopology(){
    for(int id:equipmentTracks){
        for(int parent:{-1,0,24,26,38,39}){
            Fixture fixture;fixture.parents[id]=parent;verify(fixture,fixture.bind());
        }
        Fixture duplicate;duplicate.names.push_back(duplicate.names[id]);duplicate.parents.push_back(26);duplicate.ids.push_back(-1);
        verify(duplicate,duplicate.bind());
        duplicate.parents[duplicate.index(id)]=38;verify(duplicate,duplicate.bind());
        Fixture accessory;accessory.extend(116);accessory.parents[99]=id;verify(accessory,accessory.bind());
        Fixture invalid;invalid.parents[id]=1024;check(!invalid.bind(),"engine-owned equipment retains graph bounds checks");
        invalid.parents[id]=id;check(!invalid.bind(),"engine-owned equipment self-cycle rejected");
        invalid.parents[id]=28;invalid.parents[28]=id;check(!invalid.bind(),"mixed equipment and anatomy cycle rejected");
        auto names=canonicalNames();names[id]="Unknown equipment";Fixture canonicalMismatch;
        check(!canonicalMismatch.bind(names),"unknown canonical equipment identity remains mandatory");
        auto hierarchy=canonicalParents();hierarchy[id]=0;
        check(!canonicalMismatch.bind(canonicalNames(),hierarchy),"altered canonical equipment parent cannot bypass hierarchy checks");
        hierarchy=canonicalParents();hierarchy[96]=id;canonicalMismatch.remove(id);
        check(!canonicalMismatch.bind(canonicalNames(),hierarchy),"equipment required as an anatomical ancestor cannot disappear");
    }
    Fixture anatomy;anatomy.parents[38]=61;check(!anatomy.bind(),"required hand cannot acquire an engine-owned equipment parent");
    Fixture reported;reported.parents[61]=26;const auto binding=reported.bind();verify(reported,binding);
    check(binding.indices[61]==-1&&binding.indices[5]==5&&binding.indices[26]==26,"reported WeaponAxe spine reparenting leaves pelvis and spine mappings intact");
    Fixture allDuplicate;for(int id:equipmentTracks){allDuplicate.names.push_back(allDuplicate.names[id]);allDuplicate.parents.push_back(38);allDuplicate.ids.push_back(-1);}
    verify(allDuplicate,allDuplicate.bind());
}
static void magicTopology(){
    for(int id:magicTracks){
        for(int parent:{-1,0,24,26,36,38,39,61}){
            Fixture fixture;fixture.parents[id]=parent;verify(fixture,fixture.bind());
        }
        Fixture alias;alias.names[id]="x_"+alias.names[id];const auto ignored=alias.bind();verify(alias,ignored);
        check(ignored.indices[id]==-1&&ignored.aliases==0&&ignored.missingOptional==0,"renamed effect sockets stay unowned without becoming helper aliases");
        Fixture duplicate;duplicate.names.push_back(duplicate.names[id]);duplicate.parents.push_back(26);duplicate.ids.push_back(-1);
        verify(duplicate,duplicate.bind());
        Fixture effects;effects.extend(116);effects.parents[99]=id;verify(effects,effects.bind());
        for(int parent:{-2,99,1024}){Fixture bounds;bounds.parents[id]=parent;check(!bounds.bind(),"unowned magic sockets retain live parent bounds checks");}
        Fixture cycle;cycle.parents[id]=id;check(!cycle.bind(),"unowned magic socket self-cycle rejected");
        cycle.parents[id]=28;cycle.parents[28]=id;check(!cycle.bind(),"magic socket and body cycle rejected");
        Fixture body;body.parents[38]=id;check(!body.bind(),"required hand cannot inherit through an unowned magic socket");
        auto names=canonicalNames();names[id]="x_"+names[id];Fixture schema;
        check(!schema.bind(names),"canonical magic socket names are exact and cannot acquire prefix aliases");
        auto parents=canonicalParents();parents[id]=0;
        check(!schema.bind(canonicalNames(),parents),"changed canonical magic socket parent cannot bypass hierarchy checks");
        parents=canonicalParents();parents[96]=id;schema.remove(id);
        check(!schema.bind(canonicalNames(),parents),"a magic socket required as a body ancestor cannot disappear");
    }
    Fixture cycle;cycle.parents[46]=49;cycle.parents[49]=46;check(!cycle.bind(),"cycle wholly within unowned magic sockets remains invalid");
    Fixture reported;reported.parents[49]=26;const auto binding=reported.bind();verify(reported,binding);
    check(binding.indices[49]==-1&&binding.indices[36]==36&&binding.indices[26]==26,"Head MagicNode reparenting preserves the actual head and spine bindings");
    Fixture missing;for(int id:magicTracks)missing.remove(id);verify(missing,missing.bind());
}
static void capturedIdentity(){
    Fixture fixture;fixture.alias(1);fixture.extend(116);std::mt19937 random(0xb731u);fixture.reorder(random);const auto binding=fixture.bind();verify(fixture,binding);
    const int helper=fixture.index(1),hand=fixture.index(38);
    check(!binding.current(1,{canonicalBoneNames[1],fixture.parents[helper]}),"helper spelling changes invalidate the captured exact identity");
    check(!binding.current(38,{fixture.names[hand],fixture.index(0)}),"same-storage parent changes invalidate captured binding");
    check(!binding.current(38,{fixture.names[fixture.index(39)],fixture.parents[hand]}),"same-storage name changes invalidate captured binding");
    for(std::size_t id:{42u,43u,46u,47u,48u,49u,60u,61u,62u,63u,64u,65u,66u,97u,98u,99u,1024u})check(!binding.current(id,{"Camera Control",0}),"unowned and out-of-range tracks never become valid output bindings");
    const auto savedName=binding.names[38];fixture.names[hand]="Replaced live hand";check(binding.names[38]==savedName,"captured name owns storage independently of mutable live name memory");
}
static void malformedInputs(){
    Fixture fixture;auto names=canonicalNames();names.pop_back();check(!fixture.bind(names),"canonical name count must match fixed pose track count");
    auto parents=canonicalParents();parents.pop_back();check(!fixture.bind(canonicalNames(),parents),"canonical parent count must match pose track count");
    Fixture empty;empty.names.clear();empty.parents.clear();empty.ids.clear();check(!empty.bind(),"empty live animation skeleton rejected");
    Fixture missingName;missingName.names[38].clear();check(!missingName.bind(),"empty required live name rejected");
    Fixture upperBound;upperBound.extend(1024);verify(upperBound,upperBound.bind());upperBound.extend(1025);check(!upperBound.bind(),"oversized live skeleton rejected before unbounded traversal");
}
static void reorderedProperties(){
    std::mt19937 random(0x7fc2a519u);const auto& optional=optionalTracksList;
    for(std::size_t count:{99u,116u,126u})for(unsigned sample=0;sample<64;++sample){
        Fixture fixture;fixture.extend(count);
        for(int id=1;id<=3;++id)if(sample&(1u<<id))fixture.alias(id);
        for(std::size_t i=0;i<optional.size();++i)if((sample*37u+unsigned(i)*13u)%5==0)fixture.remove(optional[i]);
        for(int id:attachmentTracks)if(fixture.index(id)>=0)fixture.parents[fixture.index(id)]=fixture.index(sample%2?26:38);
        fixture.reorder(random);verify(fixture,fixture.bind());
        const int forearm=fixture.index(29),wrongParent=fixture.index(30);fixture.parents[forearm]=wrongParent;check(!fixture.bind(),"reordering never weakens anatomical hierarchy validation");
    }
}
int main()try{canonicalAndReport();optionalTracks();ambiguity();invalidTopology();equipmentTopology();magicTopology();capturedIdentity();malformedInputs();reorderedProperties();std::cout<<"Animation skeleton binding checks: "<<checks<<'\n';return 0;}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}


