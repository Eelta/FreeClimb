#pragma once
#include "SceneBinding.h"
#include "ScenePropagation.h"
#include "RuntimeSupport.h"
#include <cstring>
#include <unordered_set>
namespace fc {
using FlatBoneEntry=RE::BSFlattenedBoneTree::BoneEntry;
inline RE::BSFlattenedBoneTree* flatTree(RE::NiAVObject* root) {
    const auto* rtti=root?root->GetRTTI():nullptr;
    return rtti&&std::string_view(rtti->GetName())=="BSFlattenedBoneTree"?static_cast<RE::BSFlattenedBoneTree*>(root):nullptr;
}
inline std::optional<std::span<FlatBoneEntry>> flatEntries(RE::NiAVObject* root) {
    auto* tree=flatTree(root);
    if(!tree)return std::span<FlatBoneEntry>{};
    auto& data=tree->GetRuntimeData();
    if(data.numBones>4096||(!data.boneEntries&&data.numBones))return std::nullopt;
    if(data.numBones&&!runtime::readable(reinterpret_cast<std::uintptr_t>(data.boneEntries),data.numBones*sizeof(FlatBoneEntry)))return std::nullopt;
    std::span<FlatBoneEntry> entries(data.boneEntries,data.numBones);
    return validFlatParents(entries)?std::optional(entries):std::nullopt;
}
using FlatWorldRefresh=void(*)(RE::NiAVObject*);
inline FlatWorldRefresh flatWorldRefresh{};
inline bool initializeFlatWorldRefresh() {
    flatWorldRefresh=nullptr;
    if(!runtime::supported())return false;
    if(!runtime::isSE())return true;
    const auto address=REL::ID(69363).address();
    constexpr std::string_view signature="\x40\x56\x48\x83\xec\x60\x8b\x81\x28\x01\x00\x00\x48\x8b\xf1\x85\xc0\x0f\x84\xe8\x00\x00\x00"sv;
    if(address!=REL::Module::get().base()+0xC6A010||!runtime::executable(address)||!runtime::readable(address,signature.size())||
        std::memcmp(reinterpret_cast<const void*>(address),signature.data(),signature.size())!=0) {
        SKSE::log::error("SE flattened world refresh ABI mismatch; pose hooks not installed");return false;
    }
    flatWorldRefresh=reinterpret_cast<FlatWorldRefresh>(address);
    return true;
}
inline bool refreshFlatWorld(RE::NiAVObject* root) {
    if(!flatTree(root))return true;
    const auto entries=flatEntries(root);
    if(!entries)return false;
    if(flatWorldRefresh){flatWorldRefresh(root);return true;}
    return synchronizeFlatEntries(*entries,root->world,[](const auto& parent,const auto& local){return parent*local;});
}
static_assert(sizeof(FlatBoneEntry)==0x80);
static_assert(offsetof(FlatBoneEntry,node)==0x70&&offsetof(FlatBoneEntry,nodeName)==0x78);
struct SceneSlot {
    RE::NiPointer<RE::NiAVObject> node;
    RE::NiTransform* flat{};
    RE::NiTransform* flatWorld{};
    RE::NiTransform* flatParentWorld{};
    explicit operator bool()const{return node.get()!=nullptr||flat!=nullptr;}
    RE::NiTransform& local()const{return flat?*flat:node->local;}
    const RE::NiTransform& world()const{return node?node->world:*flatWorld;}
    RE::NiTransform expectedWorld(const RE::NiTransform& authored)const {
        if(node)return node->parent?node->parent->world*authored:authored;
        return flatParentWorld?*flatParentWorld*authored:authored;
    }
    bool cacheMatches()const {
        if(!node||!flatWorld)return true;

        return *flatWorld==node->world;
    }
};
inline RE::NiAVObject* existingNode(RE::NiAVObject* root,std::string_view name,int depth=0) {
    if(!root||depth>128)return nullptr;
    if(root->name.c_str()&&name==root->name.c_str())return root;
    if(auto n=root->AsNode())for(auto& child:n->GetChildren())
        if(auto found=existingNode(child.get(),name,depth+1))return found;
    return nullptr;
}
inline SceneBinding<SceneSlot> bindRuntimeScene(RE::NiAVObject* root,
    std::span<const std::string> names,std::span<const int> parents) {
    const auto entries=flatEntries(root);
    if(!entries){SceneBinding<SceneSlot> invalid;invalid.missing=0;return invalid;}
    return bindScene<SceneSlot>(names,parents,[&](const std::string& name) {
        if(root&&name==root->name.c_str())return SceneSlot{RE::NiPointer<RE::NiAVObject>(root),nullptr};
        for(auto& entry:*entries)if(entry.nodeName.c_str()&&name==entry.nodeName.c_str()) {
            auto* parentWorld=entry.parentIndex>=0&&std::size_t(entry.parentIndex)<entries->size()?&(*entries)[entry.parentIndex].world:&root->world;
            return entry.node?SceneSlot{RE::NiPointer<RE::NiAVObject>(entry.node),nullptr,&entry.world,parentWorld}:
                SceneSlot{{},&entry.local,&entry.world,parentWorld};
        }
        return SceneSlot{RE::NiPointer<RE::NiAVObject>(existingNode(root,name)),nullptr};
    });
}
inline std::string sceneDiagnosticText(const char* text) {
    if(!text)return "<empty>";
    const auto address=reinterpret_cast<std::uintptr_t>(text);
    constexpr std::size_t limit=80;
    const bool whole=runtime::readable(address,limit);
    std::string result;
    for(std::size_t i=0;i<limit;++i) {
        if(!whole&&(i>std::numeric_limits<std::uintptr_t>::max()-address||!runtime::readable(address+i,1)))return result+"<unreadable>";
        const auto c=static_cast<unsigned char>(text[i]);
        if(!c)return result.empty()?"<empty>":result;
        result+=c<32||c==127?'?':char(c);
    }
    return result+"...";
}
inline std::string sceneDiagnosticName(const RE::BSFixedString& value) {
    static_assert(sizeof(value)==sizeof(const char*));
    const char* text{};std::memcpy(&text,&value,sizeof(text));
    return sceneDiagnosticText(text);
}
inline std::string sceneDiagnosticClass(RE::NiAVObject* object) {
    if(!runtime::readable(reinterpret_cast<std::uintptr_t>(object),sizeof(RE::NiAVObject)))return "<unreadable>";
    std::uintptr_t table{};std::memcpy(&table,object,sizeof(table));
    if(!runtime::hookSite(table,2))return "<invalid-vtable>";
    const auto* rtti=object->GetRTTI();
    if(!runtime::readable(reinterpret_cast<std::uintptr_t>(rtti),sizeof(RE::NiRTTI)))return "<invalid-rtti>";
    return sceneDiagnosticText(rtti->GetName());
}
struct FlatSceneDiagnostic {
    std::string state="not-flat",samples;
    std::size_t count{},populated{};
};
inline FlatSceneDiagnostic describeFlatScene(RE::NiAVObject* root,std::string_view type,bool samples) {
    FlatSceneDiagnostic result;
    if(type!="BSFlattenedBoneTree")return result;
    result.state="invalid";
    if(!runtime::readable(reinterpret_cast<std::uintptr_t>(root),sizeof(RE::BSFlattenedBoneTree)))return result;
    const auto& data=static_cast<RE::BSFlattenedBoneTree*>(root)->GetRuntimeData();
    result.count=data.numBones;result.populated=data.numPopulatedBones;
    if(data.numBones>4096||(!data.boneEntries&&data.numBones))return result;
    if(data.numBones&&!runtime::readable(reinterpret_cast<std::uintptr_t>(data.boneEntries),data.numBones*sizeof(FlatBoneEntry)))return result;
    const std::span entries(data.boneEntries,data.numBones);
    if(!validFlatParents(entries))return result;
    result.state="valid";
    if(samples)for(std::size_t i=0,named=0;i<entries.size()&&named<6;++i) {
        const auto name=sceneDiagnosticName(entries[i].nodeName);
        if(name=="<empty>")continue;
        if(named++)result.samples+="; ";
        result.samples+=std::to_string(i)+":"+name;
    }
    return result;
}
struct SceneLookupDiagnostic {
    std::string actorClass,rootClass,rootName,ancestorFlatTrees;
    FlatSceneDiagnostic flat;
    std::size_t scanned{},inside{},outside{};
    bool incomplete{},parentChainIncomplete{};
};
inline SceneLookupDiagnostic describeSceneLookup(RE::NiAVObject* actorRoot,RE::NiAVObject* root,std::string_view missing) {
    SceneLookupDiagnostic result;
    result.actorClass=sceneDiagnosticClass(actorRoot);result.rootClass=sceneDiagnosticClass(root);
    const bool rootReadable=runtime::readable(reinterpret_cast<std::uintptr_t>(root),sizeof(RE::NiAVObject));
    result.rootName=rootReadable?sceneDiagnosticName(root->name):"<unreadable>";
    result.flat=describeFlatScene(root,result.rootClass,true);
    struct Pending {RE::NiAVObject* node;unsigned depth;bool inside;};
    std::vector<Pending> pending{{actorRoot,0,false}};
    std::unordered_set<RE::NiAVObject*> visited;visited.reserve(128);
    std::size_t edges=0;
    while(!pending.empty()&&result.scanned<4096) {
        auto current=pending.back();pending.pop_back();
        if(!current.node)continue;
        if(!visited.insert(current.node).second){result.incomplete=true;continue;}
        ++result.scanned;
        if(!runtime::readable(reinterpret_cast<std::uintptr_t>(current.node),sizeof(RE::NiAVObject))){result.incomplete=true;continue;}
        current.inside|=current.node==root;
        if(sceneDiagnosticName(current.node->name)==missing)++(current.inside?result.inside:result.outside);
        std::uintptr_t table{};std::memcpy(&table,current.node,sizeof(table));
        if(!runtime::hookSite(table,3)){result.incomplete=true;continue;}
        auto* node=current.node->AsNode();
        if(!node)continue;
        if(!runtime::readable(reinterpret_cast<std::uintptr_t>(node),sizeof(RE::NiNode))){result.incomplete=true;continue;}
        const auto& children=node->GetChildren();const auto count=children.capacity();
        if(count>4096||(count&&!runtime::readable(reinterpret_cast<std::uintptr_t>(children.begin()),count*sizeof(*children.begin())))) {
            result.incomplete=true;continue;
        }
        if(current.depth>=128){result.incomplete|=count!=0;continue;}
        for(std::size_t i=0;i<count;++i) {
            if(++edges>8192||pending.size()>=4096){result.incomplete=true;break;}
            const auto& child=children.begin()[i];
            if(child)pending.push_back({child.get(),current.depth+1,current.inside});
        }
        if(edges>8192)break;
    }
    result.incomplete|=!pending.empty();visited.clear();
    if(rootReadable) {
        visited.insert(root);
        auto* parent=root->parent;unsigned depth=0,trees=0;
        while(parent&&depth<128) {
            if(!visited.insert(parent).second||!runtime::readable(reinterpret_cast<std::uintptr_t>(parent),sizeof(RE::NiAVObject))) {
                result.parentChainIncomplete=true;break;
            }
            ++depth;
            const auto type=sceneDiagnosticClass(parent);
            if(type=="BSFlattenedBoneTree") {
                if(trees++<6) {
                    const auto flat=describeFlatScene(parent,type,false);
                    if(!result.ancestorFlatTrees.empty())result.ancestorFlatTrees+="; ";
                    result.ancestorFlatTrees+=std::to_string(depth)+":"+sceneDiagnosticName(parent->name)+
                        "("+flat.state+","+std::to_string(flat.count)+")";
                } else result.parentChainIncomplete=true;
            }
            parent=parent->parent;
        }
        result.parentChainIncomplete|=parent!=nullptr;
    } else result.parentChainIncomplete=true;
    return result;
}
}
