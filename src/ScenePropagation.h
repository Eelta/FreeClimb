#pragma once
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <optional>
#include <span>
#include <vector>

namespace fc {
enum class ScenePass : unsigned { downward, selected, rigid, transformOnly };
constexpr std::size_t scenePassSlot(ScenePass pass) {
    return pass==ScenePass::transformOnly?0x31:0x2C+static_cast<unsigned>(pass);
}
constexpr std::size_t sceneWorldSlot() {return 0x30;}
template<class Entry,std::size_t Extent> bool validFlatParents(std::span<Entry,Extent> entries) {
    if(entries.size()>4096)return false;
    for(std::size_t i=0;i<entries.size();++i)
        if(!entries[i].node&&entries[i].parentIndex>=0&&std::size_t(entries[i].parentIndex)>=i)return false;
    return true;
}
template<class Entry,std::size_t Extent,class Transform,class Compose>
bool synchronizeFlatEntries(std::span<Entry,Extent> entries,const Transform& rootWorld,Compose compose) {
    if(!validFlatParents(entries))return false;
    for(auto& entry:entries) {
        if(entry.node)entry.world=entry.node->world;
        else entry.world=compose(entry.parentIndex>=0?entries[entry.parentIndex].world:rootWorld,entry.local);
    }
    return true;
}
constexpr bool overlaysPose(ScenePass pass,bool owned) {
    return owned&&(pass==ScenePass::downward||pass==ScenePass::selected||
        pass==ScenePass::rigid||pass==ScenePass::transformOnly);
}
constexpr bool refreshFlatAfterPass(ScenePass pass,bool owned) {
    return owned&&pass==ScenePass::transformOnly;
}
constexpr std::uint32_t ownedSceneNodeFlags(std::uint32_t flags) {
    return (flags|std::uint32_t{6})&~std::uint32_t{16};
}

template<class Node,class Parent> std::optional<std::vector<Node>> sceneBridgeNodes(
    std::span<const Node> mapped,Node root,Parent parent) {
    if(!root)return std::nullopt;
    std::vector<Node> bridges;
    for(auto node:mapped) {
        if(!node)continue;
        unsigned depth=0;
        while(node&&node!=root&&depth++<256) {
            node=parent(node);
            if(node&&node!=root&&std::find(mapped.begin(),mapped.end(),node)==mapped.end()&&
                std::find(bridges.begin(),bridges.end(),node)==bridges.end())bridges.push_back(node);
        }
        if(node!=root)return std::nullopt;
    }
    return bridges;
}
constexpr std::uint32_t effectiveUpdateDataFlags(ScenePass pass,std::uint32_t flags,bool owned) {

    return owned&&pass==ScenePass::selected?flags&~std::uint32_t{2}:flags;
}
}
