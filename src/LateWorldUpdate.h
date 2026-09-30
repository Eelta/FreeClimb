#pragma once
#include <optional>
#include <unordered_map>

namespace fc {

template<class T> class ScopedLocalOverride {
    T& target;
    T saved;
public:
    ScopedLocalOverride(T& value,const T& replacement):target(value),saved(value){target=replacement;}
    ~ScopedLocalOverride(){target=saved;}
    ScopedLocalOverride(const ScopedLocalOverride&)=delete;
    ScopedLocalOverride& operator=(const ScopedLocalOverride&)=delete;
};

template<class Node,class Transform,class Parent,class Local,class Compose>
std::optional<Transform> ownedSkinWorld(Node node,Node root,const Transform& rootWorld,
    Parent parent,Local local,Compose compose,std::unordered_map<Node,std::optional<Transform>>& cache,unsigned depth=0) {
    if(!node||depth>256)return {};
    if(node==root)return rootWorld;
    if(const auto it=cache.find(node);it!=cache.end())return it->second;
    cache.emplace(node,std::nullopt);
    const auto here=local(node);
    if(!here)return {};
    const auto above=ownedSkinWorld(parent(node),root,rootWorld,parent,local,compose,cache,depth+1);
    if(!above)return {};
    return cache[node]=compose(*above,*here);
}
}
