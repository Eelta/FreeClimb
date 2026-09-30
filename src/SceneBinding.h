#pragma once
#include <algorithm>
#include <array>
#include <span>
#include <string>
#include <string_view>

namespace fc {

inline bool engineOwnedCameraTrack(std::size_t i,std::span<const std::string> names,std::span<const int> parents) {
    if(names.size()!=99||parents.size()!=99||i>=99||std::find(parents.begin(),parents.end(),int(i))!=parents.end())return false;
    return (i==97&&names[i]=="Camera3rd [Cam3]"&&parents[i]==-1)||
        (i==98&&names[i]=="Camera Control"&&parents[i]==0);
}

inline bool animationOnlyLeaf(std::size_t i,std::span<const std::string> names,std::span<const int> parents) {
    struct Leaf {std::size_t index;std::string_view name;int parent;};
    constexpr std::array<Leaf,6> optional={Leaf{1,"x_NPC LookNode [Look]",0},{2,"x_NPC Translate [Pos ]",0},
        {3,"x_NPC Rotate [Rot ]",0},{42,"Shield",38},{43,"Weapon",39},{60,"Quiver",26}};
    if(names.size()!=99||parents.size()!=99||i>=99||std::find(parents.begin(),parents.end(),int(i))!=parents.end())return false;
    return std::any_of(optional.begin(),optional.end(),[&](const Leaf& leaf){return i==leaf.index&&names[i]==leaf.name&&parents[i]==leaf.parent;});
}
template<class Node> struct SceneBinding {
    std::array<Node,99> nodes{};
    int missing=-1;
    std::size_t count{},virtualLeaves{},unownedTracks{};
    explicit operator bool() const {return missing<0&&nodes[0]&&count+virtualLeaves+unownedTracks==nodes.size();}
    template<class Function> void each(Function function) const {
        for(std::size_t i=0;i<nodes.size();++i)if(nodes[i])function(i,nodes[i]);
    }
};
template<class Node,class Lookup> SceneBinding<Node> bindScene(
    std::span<const std::string> names,std::span<const int> parents,Lookup lookup) {
    SceneBinding<Node> result;
    if(names.size()!=99||parents.size()!=99){result.missing=0;return result;}
    for(std::size_t i=0;i<99;++i) {
        if(engineOwnedCameraTrack(i,names,parents)){++result.unownedTracks;continue;}
        result.nodes[i]=lookup(names[i]);
        if(result.nodes[i])++result.count;
        else if(animationOnlyLeaf(i,names,parents))++result.virtualLeaves;
        else {result.missing=int(i);return result;}
    }
    return result;
}
}
