#include "AnimationSkeletonLayout.h"
#include "AnimationSkeletonBinding.h"
#include "CanonicalSkeleton.h"
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
using namespace fc;
static unsigned checks{};
static void require(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
struct Bone {std::string_view name;bool locked{};};
using Reference=std::array<float,12>;
using Layout=AnimationSkeletonLayout<Bone,Reference>;
template<class T> struct Array {
    const T* pointer{};int count{};
    const T* data()const{return pointer;}
    int size()const{return count;}
    const T& operator[](int index)const{return pointer[index];}
};
struct Skeleton {Array<Bone> bones;Array<std::int16_t> parentIndices;Array<Reference> referencePose;};
struct Fixture {
    std::vector<std::string> names;
    std::vector<Bone> bones;
    std::vector<std::int16_t> parents;
    std::vector<Reference> references;
    Fixture(int count=116,int parentCount=126,int referenceCount=126):names(count),bones(count),parents(parentCount),references(referenceCount){
        for(int i=0;i<count;++i){
            names[i]=i<99?std::string(canonicalBoneNames[i]):"Extension "+std::to_string(i);
            bones[i]={names[i],i%3!=0};parents[i]=std::int16_t(i<99?canonicalBoneParents[i]:0);
            for(std::size_t j=0;j<references[i].size();++j)references[i][j]=float(i*16+int(j));
        }
        for(int i=count;i<parentCount;++i)parents[i]=std::numeric_limits<std::int16_t>::max();
        for(int i=count;i<referenceCount;++i)references[i].fill(std::numeric_limits<float>::quiet_NaN());
    }
    Layout layout()const{return animationSkeletonLayout(Skeleton{{bones.data(),int(bones.size())},{parents.data(),int(parents.size())},{references.data(),int(references.size())}});}
};
static std::vector<std::string> canonicalNames(){std::vector<std::string> result;for(auto name:canonicalBoneNames)result.emplace_back(name);return result;}
static AnimationSkeletonBinding bind(const Layout& layout){
    std::vector<AnimationSkeletonBone> input;input.reserve(layout.boneCount);
    for(int i=0;i<layout.boneCount;++i)input.push_back({layout.bones[i].name,layout.parents[i]});
    return bindAnimationSkeleton(canonicalNames(),canonicalBoneParents,input);
}
static void declaredReadability(const Layout& layout){
    const std::array expected{
        std::pair{reinterpret_cast<std::uintptr_t>(layout.bones),std::size_t(layout.boneCount)*sizeof(Bone)},
        std::pair{reinterpret_cast<std::uintptr_t>(layout.parents),std::size_t(layout.parentCount)*sizeof(std::int16_t)},
        std::pair{reinterpret_cast<std::uintptr_t>(layout.references),std::size_t(layout.referenceCount)*sizeof(Reference)}};
    std::vector<std::pair<std::uintptr_t,std::size_t>> actual;
    require(layout.valid([&](std::uintptr_t at,std::size_t bytes){actual.emplace_back(at,bytes);return true;}),"compatible metadata validates all three declared storage ranges");
    require(actual.size()==expected.size(),"all three arrays must have an independent readability check");
    for(std::size_t i=0;i<expected.size();++i)require(actual[i]==expected[i],"readability covers the declared array size with its own element width");
    for(std::size_t denied=0;denied<expected.size();++denied){
        require(!layout.valid([&](std::uintptr_t at,std::size_t bytes){return std::pair{at,bytes}!=expected[denied];}),"an unreadable declared bone, parent or reference range rejects the layout");
    }
}
static void reportedLayouts(){
    for(const auto dimensions:{std::array{99,99,99},std::array{116,126,126},std::array{116,116,126},std::array{116,126,116},std::array{126,126,126},std::array{1024,1024,1024}}){
        Fixture fixture(dimensions[0],dimensions[1],dimensions[2]);const auto layout=fixture.layout();
        require(layout.boneCount==dimensions[0]&&layout.parentCount==dimensions[1]&&layout.referenceCount==dimensions[2],"typed extraction preserves each independent signed array length");
        declaredReadability(layout);const auto binding=bind(layout);require(bool(binding),"named valid bones bind with equal or longer parent and reference arrays");
        for(std::size_t track=0;track<99;++track){
            const int index=binding.indices[track];if(index<0)continue;
            require(index<layout.boneCount,"every mapped reference index remains inside the named bone domain");
            for(std::size_t component=0;component<12;++component)
                require(std::isfinite(layout.references[index][component])&&layout.references[index][component]==float(index*16+int(component)),"mapped reference values use the named bone prefix without touching poisoned trailing poses");
            require(layout.bones[index].locked==(index%3!=0),"lock translation identity follows the same mapped bone index");
        }
        for(int i=layout.boneCount;i<layout.parentCount;++i)require(layout.parents[i]==std::numeric_limits<std::int16_t>::max(),"trailing parent poison remains unused and unchanged");
        for(int i=layout.boneCount;i<layout.referenceCount;++i)for(float value:layout.references[i])require(std::isnan(value),"trailing reference poison remains unused and unchanged");
    }
    Fixture reported;const auto layout=reported.layout();
    const bool oldStrict=layout.parentCount==layout.boneCount&&layout.referenceCount==layout.boneCount;
    require(!oldStrict&&layout.valid([](std::uintptr_t,std::size_t){return true;}),"the reported 116/126/126 layout reproduces the old rejection and passes the new production policy");
}
static void malformedMetadata(){
    Fixture fixture;const auto baseline=fixture.layout();
    const auto rejected=[&](Layout changed){unsigned reads=0;require(!changed.valid([&](std::uintptr_t,std::size_t){++reads;return true;}),"invalid metadata is rejected before native storage access");require(reads==0,"bad dimensions and null arrays never reach the readability probe");};
    for(int value:{std::numeric_limits<int>::min(),-1,0,1025,std::numeric_limits<int>::max()}){auto changed=baseline;changed.boneCount=value;rejected(changed);}
    for(int value:{std::numeric_limits<int>::min(),-1,0,115,1025,std::numeric_limits<int>::max()}){
        auto changed=baseline;changed.parentCount=value;rejected(changed);changed=baseline;changed.referenceCount=value;rejected(changed);
    }
    auto changed=baseline;changed.bones=nullptr;rejected(changed);changed=baseline;changed.parents=nullptr;rejected(changed);changed=baseline;changed.references=nullptr;rejected(changed);
}
static void namedDomainGuards(){
    for(int track:{28,97,115})for(int parent:{-2,116,125,126,1024}){
        Fixture fixture;fixture.parents[track]=std::int16_t(parent);const auto result=bind(fixture.layout());
        require(!result&&std::string_view(result.error)=="animation parent outside skeleton","body, engine-owned and extension parents cannot point into unnamed trailing storage");
    }
    for(int track:{28,97,115}){Fixture fixture;fixture.parents[track]=std::int16_t(track);require(!bind(fixture.layout()),"cycles inside the valid named prefix remain rejected with longer arrays");}
    Fixture missing;missing.bones[36].name="Missing actual head";require(!bind(missing.layout()),"larger companion arrays cannot substitute for a missing required named bone");
    Fixture duplicate;duplicate.bones[115].name=canonicalBoneNames[36];require(!bind(duplicate.layout()),"duplicate required names remain ambiguous inside the active prefix");
    Fixture topology;topology.parents[38]=26;require(!bind(topology.layout()),"larger companion arrays do not relax required body parent matching");
    Fixture headMagic;headMagic.parents[49]=26;const auto mapped=bind(headMagic.layout());
    require(bool(mapped)&&mapped.indices[49]<0&&mapped.indices[36]==36,"the existing head magic attachment exclusion remains compatible with asymmetric storage");
}
static void capturedStorage(){
    Fixture fixture,other;const auto captured=fixture.layout();
    require(captured==fixture.layout(),"an unchanged set of typed arrays matches the captured layout");
    auto changed=captured;changed.bones=other.bones.data();require(changed!=captured,"bone storage replacement invalidates the captured identity");
    changed=captured;changed.parents=other.parents.data();require(changed!=captured,"parent storage replacement invalidates the captured identity");
    changed=captured;changed.references=other.references.data();require(changed!=captured,"reference storage replacement invalidates the captured identity");
    for(int which=0;which<3;++which)for(int direction:{-1,1}){
        changed=captured;if(which==0)changed.boneCount+=direction;else if(which==1)changed.parentCount+=direction;else changed.referenceCount+=direction;
        require(changed!=captured,"any independent array length change invalidates the captured layout even when it remains numerically plausible");
    }
    fixture.parents.back()=0;fixture.references.back().fill(0);
    require(captured==fixture.layout()&&bool(bind(fixture.layout())),"unused tail content changes do not invent storage replacement or affect named bone mapping");
}
int main()try{reportedLayouts();malformedMetadata();namedDomainGuards();capturedStorage();std::cout<<"Animation skeleton layout checks: "<<checks<<'\n';return 0;}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
