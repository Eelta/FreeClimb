#pragma once
#include "SceneBinding.h"
#include "ScenePropagation.h"
#include "RuntimeSupport.h"
#include <cstring>
#include <bit>
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
struct ExistingSceneLookup {
    RE::NiAVObject* node{};
    std::size_t scanned{},edges{};
    bool ambiguous{},incomplete{};
    RE::NiAVObject* conflict{};
    RE::NiAVObject* stoppedAt{};
    std::size_t nonNullEdges{},matches{};
    int maxDepth{};
    const char* limit{};
    RE::NiAVObject* selected()const{return ambiguous||incomplete?nullptr:node;}
    const char* reason()const{return ambiguous?"ambiguous":incomplete?limit:node?"unique":"not-found";}
};
inline void findExistingSceneNodes(RE::NiAVObject* root,std::string_view name,int depth,ExistingSceneLookup& result,bool stopAtMatch=false) {
    if(!root||result.ambiguous||result.incomplete)return;
    result.maxDepth=std::max(result.maxDepth,depth);
    if(depth>128){result.incomplete=true;result.limit="depth-limit";result.stoppedAt=root;return;}
    if(++result.scanned>4096){result.incomplete=true;result.limit="node-limit";result.stoppedAt=root;return;}
    if(root->name.c_str()&&sameSceneBoneName(name,root->name.c_str())) {
        ++result.matches;
        if(result.node&&result.node!=root){result.ambiguous=true;result.conflict=root;return;}
        result.node=root;
        if(stopAtMatch)return;
    }
    if(auto node=root->AsNode())for(auto& child:node->GetChildren()) {
        if(++result.edges>8192){result.incomplete=true;result.limit="child-slot-limit";result.stoppedAt=root;return;}
        auto* next=child.get();result.nonNullEdges+=next!=nullptr;
        findExistingSceneNodes(next,name,depth+1,result,stopAtMatch);
        if(result.ambiguous||result.incomplete)return;
    }
}
inline ExistingSceneLookup lookupExistingNode(RE::NiAVObject* root,std::string_view name,int depth=0) {
    ExistingSceneLookup result;findExistingSceneNodes(root,name,depth,result);
    return result;
}
inline RE::NiAVObject* existingNode(RE::NiAVObject* root,std::string_view name,int depth=0) {
    return lookupExistingNode(root,name,depth).selected();
}
inline ExistingSceneLookup lookupSkeletonRoot(RE::NiAVObject* actorRoot,std::string_view name) {
    ExistingSceneLookup result;findExistingSceneNodes(actorRoot,name,0,result,true);
    if(!result.selected())return result;
    if(result.node->AsNode()) {
        auto* ancestor=result.node;
        for(int depth=0;ancestor&&depth<=128;++depth) {
            if(ancestor==actorRoot)return result;
            if(!runtime::readable(reinterpret_cast<std::uintptr_t>(ancestor),sizeof(RE::NiAVObject)))break;
            ancestor=ancestor->parent;
        }
        result.limit="root-outside-actor";
    }else result.limit="invalid-root-node";
    result.incomplete=true;result.stoppedAt=result.node;
    return result;
}
struct FlatMapSlot {
    const char* name{};
    std::int32_t index{};
    std::uint32_t padding{};
    std::uintptr_t next{};
};
struct FlatMapStorage {
    std::array<std::byte,12> padding{};
    std::uint32_t capacity{},free{},good{};
    std::uintptr_t sentinel{},allocator{};
    const FlatMapSlot* slots{};
};
static_assert(sizeof(FlatMapSlot)==0x18&&sizeof(FlatMapStorage)==sizeof(RE::BSTHashMap<RE::BSFixedString,std::int32_t>));
static_assert(offsetof(FlatMapStorage,capacity)==0x0C&&offsetof(FlatMapStorage,sentinel)==0x18&&offsetof(FlatMapStorage,slots)==0x28);
inline const char* flatRawName(const RE::BSFixedString& name) {
    const char* value{};std::memcpy(&value,&name,sizeof(value));return value;
}
inline bool flatReadName(const char* text,std::string& result) {
    result.clear();if(!text)return true;
    constexpr std::size_t limit=256;
    const auto address=reinterpret_cast<std::uintptr_t>(text);
    const bool whole=runtime::readable(address,limit);
    for(std::size_t i=0;i<limit;++i) {
        if(!whole&&(i>std::numeric_limits<std::uintptr_t>::max()-address||!runtime::readable(address+i,1)))return false;
        if(!text[i])return true;
        result+=text[i];
    }
    return false;
}
struct FlatNameEntry {
    std::string name,directName,nodeName;
    FlatBoneEntry* entry{};
    RE::NiAVObject* node{};
    const char* rawName{};
    const char* rawNodeName{};
    int parentIndex=-1;
    bool verifyNodeName{};
    std::vector<std::size_t> mapSlots;
};
struct FlatNameSnapshot {
    bool valid{},mapUsed{},flattened{};
    const char* issue="uninitialized";
    FlatBoneEntry* data{};
    std::size_t count{};
    FlatMapStorage map{};
    std::vector<FlatMapSlot> slots;
    std::vector<std::string> mapNames;
    std::vector<FlatNameEntry> entries;
    std::size_t caseAliases{};
    bool exact(std::size_t index,std::string_view name)const {
        if(index>=entries.size())return false;
        const auto& entry=entries[index];
        if(entry.directName==name||entry.nodeName==name)return true;
        for(auto slot:entry.mapSlots)if(mapNames[slot]==name)return true;
        return false;
    }
    int find(std::string_view name)const {
        int result=-1;
        const auto accept=[&](std::size_t index) {
            if(result>=0&&result!=int(index)){result=-2;return false;}
            result=int(index);return true;
        };
        for(std::size_t i=0;i<entries.size();++i) {
            const auto& entry=entries[i];
            if(sameSceneBoneName(entry.directName,name)||sameSceneBoneName(entry.nodeName,name)) {
                if((!entry.directName.empty()&&!sameSceneBoneName(entry.directName,name))||(!entry.nodeName.empty()&&!sameSceneBoneName(entry.nodeName,name))||!accept(i))return -2;
            }
        }
        for(std::size_t i=0;i<slots.size();++i)if(slots[i].next&&sameSceneBoneName(mapNames[i],name)) {
            if(slots[i].index<0||std::size_t(slots[i].index)>=entries.size())return -2;
            const auto index=std::size_t(slots[i].index);const auto& entry=entries[index];
            if((!entry.directName.empty()&&!sameSceneBoneName(entry.directName,name))||(!entry.nodeName.empty()&&!sameSceneBoneName(entry.nodeName,name))||!accept(index))return -2;
        }
        return result;
    }
    bool attachment(std::size_t index,std::span<const std::string> names,std::span<const int> parents)const {
        if(index>=entries.size())return false;
        const auto& entry=entries[index];
        if(engineOwnedAttachmentName(entry.directName,names,parents)||engineOwnedAttachmentName(entry.nodeName,names,parents))return true;
        for(auto slot:entry.mapSlots)if(engineOwnedAttachmentName(mapNames[slot],names,parents))return true;
        return false;
    }
    bool currentStorage(RE::NiAVObject* root)const {
        if(!valid)return false;
        const auto* tree=flatTree(root);
        if(bool(tree)!=flattened)return false;
        if(!tree)return count==0&&entries.empty();
        const auto& current=tree->GetRuntimeData();
        return current.boneEntries==data&&current.numBones==count&&std::memcmp(&current.boneMap,&map,sizeof(map))==0;
    }
    bool currentEntryIdentity(std::size_t index,bool nodeName)const {
        const auto& cached=entries[index];const auto& live=data[index];
        if(live.node!=cached.node||flatRawName(live.nodeName)!=cached.rawName||
            (nodeName&&live.node&&flatRawName(live.node->name)!=cached.rawNodeName))return false;
        for(auto slot:cached.mapSlots)if(std::memcmp(map.slots+slot,&slots[slot],sizeof(FlatMapSlot))!=0)return false;
        return true;
    }
    bool currentEntry(RE::NiAVObject* root,std::size_t index)const {
        return currentStorage(root)&&index<entries.size()&&currentEntryIdentity(index,true);
    }
    bool current(RE::NiAVObject* root)const {
        if(!currentStorage(root))return false;
        if(count&&!runtime::readable(reinterpret_cast<std::uintptr_t>(data),count*sizeof(FlatBoneEntry)))return false;
        if(!slots.empty()&&(!runtime::readable(reinterpret_cast<std::uintptr_t>(map.slots),slots.size()*sizeof(FlatMapSlot))||
            std::memcmp(map.slots,slots.data(),slots.size()*sizeof(FlatMapSlot))!=0))return false;
        for(std::size_t i=0;i<entries.size();++i)if(!currentEntryIdentity(i,entries[i].verifyNodeName))return false;
        return true;
    }
};
inline FlatNameSnapshot captureFlatNames(RE::NiAVObject* root) {
    FlatNameSnapshot result;
    const auto storage=flatEntries(root);
    if(!storage){result.issue="invalid-flat-storage";return result;}
    result.data=storage->data();result.count=storage->size();
    result.entries.resize(result.count);
    for(std::size_t i=0;i<result.count;++i) {
        const auto& source=(*storage)[i];auto& target=result.entries[i];
        target.entry=&(*storage)[i];target.node=source.node;target.rawName=flatRawName(source.nodeName);
        target.parentIndex=source.parentIndex;
        if(!flatReadName(target.rawName,target.directName)){result.issue="unreadable-entry-name";return result;}
        if(source.node) {
            if(!runtime::readable(reinterpret_cast<std::uintptr_t>(source.node),sizeof(RE::NiAVObject))){result.issue="unreadable-entry-node";return result;}
            target.rawNodeName=flatRawName(source.node->name);
            if(!flatReadName(target.rawNodeName,target.nodeName)){result.issue="unreadable-node-name";return result;}
        }
        target.name=target.directName.empty()?target.nodeName:target.directName;
    }
    const auto* tree=flatTree(root);
    if(!tree){result.valid=true;result.issue="not-flat";return result;}
    result.flattened=true;
    std::memcpy(&result.map,&tree->GetRuntimeData().boneMap,sizeof(result.map));
    const auto& map=result.map;
    if(map.capacity>8192||map.free>map.capacity||(map.capacity&&(!std::has_single_bit(map.capacity)||map.good>=map.capacity||!map.sentinel||!map.slots))||
        (!map.capacity&&map.slots)) {result.issue="invalid-map-header";return result;}
    if(!map.capacity){result.valid=true;result.issue="empty-map";return result;}
    const auto base=reinterpret_cast<std::uintptr_t>(map.slots);
    if(!runtime::readable(base,map.capacity*sizeof(FlatMapSlot))){result.issue="unreadable-map-storage";return result;}
    result.slots.resize(map.capacity);std::memcpy(result.slots.data(),map.slots,map.capacity*sizeof(FlatMapSlot));
    result.mapNames.resize(map.capacity);std::vector<unsigned char> colors(map.capacity);
    std::size_t populated=0;
    for(std::size_t i=0;i<result.slots.size();++i) {
        const auto& slot=result.slots[i];if(!slot.next)continue;++populated;
        if(slot.next!=map.sentinel&&(slot.next<base||slot.next-base>=map.capacity*sizeof(FlatMapSlot)||
            (slot.next-base)%sizeof(FlatMapSlot))) {result.issue="invalid-map-chain";return result;}
        if(!flatReadName(slot.name,result.mapNames[i])||result.mapNames[i].empty()){result.issue="invalid-map-name";return result;}
        if(slot.index>=0&&std::size_t(slot.index)<result.count) {
            auto& entry=result.entries[slot.index];entry.mapSlots.push_back(i);
            if(entry.name.empty())entry.name=result.mapNames[i];
        }
    }
    if(populated!=map.capacity-map.free){result.issue="invalid-map-size";return result;}
    for(std::size_t i=0;i<result.slots.size();++i) {
        if(!result.slots[i].next||colors[i]==2)continue;
        std::size_t index=i;
        while(colors[index]!=2) {
            if(colors[index]==1||!result.slots[index].next){result.issue="invalid-map-chain";return result;}
            colors[index]=1;const auto next=result.slots[index].next;
            if(next==map.sentinel)break;
            index=(next-base)/sizeof(FlatMapSlot);
        }
        index=i;
        while(colors[index]==1) {
            colors[index]=2;const auto next=result.slots[index].next;
            if(next==map.sentinel)break;
            index=(next-base)/sizeof(FlatMapSlot);
        }
    }
    result.mapUsed=populated!=0;result.valid=true;result.issue=populated?"valid-map":"empty-map";
    return result;
}
inline SceneBinding<SceneSlot> bindRuntimeScene(RE::NiAVObject* root,
    std::span<const std::string> names,std::span<const int> parents,FlatNameSnapshot* captured=nullptr) {
    auto snapshot=captureFlatNames(root);
    if(!snapshot.valid){if(captured)*captured=std::move(snapshot);SceneBinding<SceneSlot> invalid;invalid.missing=0;return invalid;}
    const std::span entries(snapshot.data,snapshot.count);
    std::array<int,99> selected{};selected.fill(-1);std::size_t selectedCount=0;
    auto result=bindScene<SceneSlot>(names,parents,[&](const std::string& name) {
        if(root&&sameSceneBoneName(name,root->name.c_str())) {
            snapshot.caseAliases+=name!=root->name.c_str();
            return SceneSlot{RE::NiPointer<RE::NiAVObject>(root),nullptr};
        }
        const int index=snapshot.find(name);
        if(index==-2){snapshot.issue="ambiguous-flat-name";return SceneSlot{};}
        if(index>=0) {
            if(snapshot.attachment(std::size_t(index),names,parents)){snapshot.issue="engine-owned-flat-alias";return SceneSlot{};}
            if(std::find(selected.begin(),selected.begin()+selectedCount,index)!=selected.begin()+selectedCount){snapshot.issue="duplicate-flat-binding";return SceneSlot{};}
            selected[selectedCount++]=index;auto& entry=entries[index];
            snapshot.caseAliases+=!snapshot.exact(std::size_t(index),name);
            snapshot.entries[index].verifyNodeName=entry.node!=nullptr;
            auto* parentWorld=entry.parentIndex>=0&&std::size_t(entry.parentIndex)<entries.size()?&entries[entry.parentIndex].world:&root->world;
            return entry.node?SceneSlot{RE::NiPointer<RE::NiAVObject>(entry.node),nullptr,&entry.world,parentWorld}:
                SceneSlot{{},&entry.local,&entry.world,parentWorld};
        }
        auto* node=existingNode(root,name);
        if(node)snapshot.caseAliases+=name!=node->name.c_str();
        return SceneSlot{RE::NiPointer<RE::NiAVObject>(node),nullptr};
    });
    if(captured)*captured=std::move(snapshot);
    return result;
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
inline std::string describeExistingSceneLookup(RE::NiAVObject* actorRoot,const ExistingSceneLookup& lookup) {
    const auto describe=[](RE::NiAVObject* node) {
        if(!node)return std::string("<null>");
        std::string result="address="+std::to_string(reinterpret_cast<std::uintptr_t>(node));
        if(!runtime::readable(reinterpret_cast<std::uintptr_t>(node),sizeof(RE::NiAVObject)))return result+" <unreadable>";
        return result+" name='"+sceneDiagnosticName(node->name)+"' class='"+sceneDiagnosticClass(node)+"'";
    };
    const auto path=[&](RE::NiAVObject* node) {
        std::string result;std::array<RE::NiAVObject*,8> visited{};std::size_t count=0;
        while(node&&count<visited.size()) {
            if(!result.empty())result+=" <- ";
            if(std::find(visited.begin(),visited.begin()+count,node)!=visited.begin()+count){result+="<cycle>";return result;}
            visited[count++]=node;result+=describe(node);
            if(!runtime::readable(reinterpret_cast<std::uintptr_t>(node),sizeof(RE::NiAVObject)))return result;
            node=node->parent;
        }
        if(node)result+=" <- <parent-limit>";
        return result.empty()?std::string("<null>"):result;
    };
    std::string result="reason="+std::string(lookup.reason())+" scanned="+std::to_string(lookup.scanned)+
        " childSlots="+std::to_string(lookup.edges)+" nonNullChildren="+std::to_string(lookup.nonNullEdges)+
        " matches="+std::to_string(lookup.matches)+" maxDepth="+std::to_string(lookup.maxDepth)+
        " first=["+path(lookup.node)+"] conflict=["+path(lookup.conflict)+"] stoppedAt=["+path(lookup.stoppedAt)+
        "] actor=["+describe(actorRoot)+"] topChildren=[";
    if(!runtime::readable(reinterpret_cast<std::uintptr_t>(actorRoot),sizeof(RE::NiAVObject)))return result+"<unreadable>]";
    std::uintptr_t table{};std::memcpy(&table,actorRoot,sizeof(table));
    if(!runtime::hookSite(table,3))return result+"<invalid-vtable>]";
    auto* node=actorRoot->AsNode();
    if(!node)return result+"<not-node>]";
    if(!runtime::readable(reinterpret_cast<std::uintptr_t>(node),sizeof(RE::NiNode)))return result+"<unreadable-node>]";
    const auto& children=node->GetChildren();const auto count=std::min<std::size_t>(children.capacity(),8);
    if(count&&!runtime::readable(reinterpret_cast<std::uintptr_t>(children.begin()),count*sizeof(*children.begin())))return result+"<unreadable-children>]";
    for(std::size_t i=0;i<count;++i) {
        if(i)result+="; ";
        result+=std::to_string(i)+":"+describe(children.begin()[i].get());
    }
    return result+"] childrenCapacity="+std::to_string(children.capacity());
}
inline std::string sceneDiagnosticRawName(std::string_view name) {
    constexpr char digits[]="0123456789ABCDEF";
    const auto limit=std::min<std::size_t>(name.size(),80);
    std::string raw,hex;raw.reserve(limit);hex.reserve(limit*2);
    for(std::size_t i=0;i<limit;++i) {
        const auto value=static_cast<unsigned char>(name[i]);raw+=value<32||value==127?'?':char(value);
        hex+=digits[value>>4];hex+=digits[value&15];
    }
    if(limit<name.size()){raw+="...";hex+="...";}
    return "'"+raw+"' len="+std::to_string(name.size())+" hex="+hex;
}
inline bool sceneDiagnosticRelatedName(std::string_view name) {
    for(auto part:{std::string_view("thigh"),std::string_view("lthg")})
        for(std::size_t i=0;i+part.size()<=name.size();++i)if(sameSceneBoneName(name.substr(i,part.size()),part))return true;
    return false;
}
inline std::string sceneDiagnosticFlatCandidate(const FlatNameSnapshot& names,std::size_t index,std::string_view source,std::string_view name) {
    const auto& entry=names.entries[index];
    return std::string(source)+"="+std::to_string(index)+" "+sceneDiagnosticRawName(name)+
        " parent="+std::to_string(entry.parentIndex)+" node="+(entry.node?"true":"false");
}
struct FlatSceneDiagnostic {
    std::string state="not-flat",samples;
    std::size_t count{},populated{};
    std::string mapState="not-flat",missingEntryName;
    std::size_t mapCount{};
    int missingIndex=-1;
    bool missingNode{};
    std::size_t caseOnlyCount{};
    std::string caseOnlyCandidates,nearbyEntries,relatedNames;
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
    std::size_t caseOnlyInside{},caseOnlyOutside{};
    std::string actualCaseCandidates;
    bool incomplete{},parentChainIncomplete{};
};
inline SceneLookupDiagnostic describeSceneLookup(RE::NiAVObject* actorRoot,RE::NiAVObject* root,std::string_view missing,const FlatNameSnapshot* captured=nullptr) {
    SceneLookupDiagnostic result;
    result.actorClass=sceneDiagnosticClass(actorRoot);result.rootClass=sceneDiagnosticClass(root);
    const bool rootReadable=runtime::readable(reinterpret_cast<std::uintptr_t>(root),sizeof(RE::NiAVObject));
    result.rootName=rootReadable?sceneDiagnosticName(root->name):"<unreadable>";
    result.flat=describeFlatScene(root,result.rootClass,true);
    if(result.flat.state=="valid") {
        std::optional<FlatNameSnapshot> owned;
        if(!captured){owned.emplace(captureFlatNames(root));captured=&*owned;}
        const auto& names=*captured;result.flat.mapState=names.issue;
        result.flat.mapCount=names.map.free<=names.map.capacity?names.map.capacity-names.map.free:0;
        if(names.valid) {
            result.flat.missingIndex=names.find(missing);
            if(result.flat.missingIndex>=0) {
                const auto& entry=names.entries[result.flat.missingIndex];
                result.flat.missingEntryName=entry.name;result.flat.missingNode=entry.node!=nullptr;
            }
            std::size_t cases=0,related=0;
            const auto append=[](std::string& text,const std::string& candidate){if(!text.empty())text+="; ";text+=candidate;};
            for(std::size_t i=0;i<names.entries.size();++i) {
                const auto& entry=names.entries[i];bool caseOnly=false;
                const auto inspect=[&](std::string_view source,std::string_view name) {
                    if(name.empty())return;
                    if(name!=missing&&sameSceneBoneName(name,missing)) {
                        caseOnly=true;if(cases++<6)append(result.flat.caseOnlyCandidates,sceneDiagnosticFlatCandidate(names,i,source,name));
                    }
                    if(sceneDiagnosticRelatedName(name)&&related++<6)append(result.flat.relatedNames,sceneDiagnosticFlatCandidate(names,i,source,name));
                };
                inspect("entry",entry.directName);inspect("node",entry.nodeName);
                for(auto slot:entry.mapSlots)inspect("map",names.mapNames[slot]);
                result.flat.caseOnlyCount+=caseOnly;
            }
            if(result.flat.missingIndex<0)for(std::size_t i=6;i<std::min<std::size_t>(names.entries.size(),14);++i)
                append(result.flat.nearbyEntries,sceneDiagnosticFlatCandidate(names,i,"row",names.entries[i].name));
        }
    }
    struct Pending {RE::NiAVObject* node;unsigned depth;bool inside;};
    std::vector<Pending> pending{{actorRoot,0,false}};
    std::unordered_set<RE::NiAVObject*> visited;visited.reserve(128);
    std::size_t edges=0,actualCases=0;
    while(!pending.empty()&&result.scanned<4096) {
        auto current=pending.back();pending.pop_back();
        if(!current.node)continue;
        if(!visited.insert(current.node).second){result.incomplete=true;continue;}
        ++result.scanned;
        if(!runtime::readable(reinterpret_cast<std::uintptr_t>(current.node),sizeof(RE::NiAVObject))){result.incomplete=true;continue;}
        current.inside|=current.node==root;
        const auto name=sceneDiagnosticName(current.node->name);
        if(name==missing)++(current.inside?result.inside:result.outside);
        else if(sameSceneBoneName(name,missing)) {
            ++(current.inside?result.caseOnlyInside:result.caseOnlyOutside);
            if(actualCases++<6) {
                if(!result.actualCaseCandidates.empty())result.actualCaseCandidates+="; ";
                result.actualCaseCandidates+=(current.inside?"inside ":"outside ")+sceneDiagnosticRawName(name);
            }
        }
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
