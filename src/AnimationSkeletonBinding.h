#pragma once
#include "SceneBinding.h"
#include <array>
#include <span>
#include <string>
#include <string_view>

namespace fc {
struct AnimationSkeletonBone {std::string_view name;int parent=-1;};
struct AnimationSkeletonBinding {
    std::array<int,99> indices{},parents{};
    std::array<std::string,99> names;
    const char* error{};
    int track=-1,actual=-1;
    std::size_t remapped{},aliases{},missingOptional{};
    AnimationSkeletonBinding(){indices.fill(-1);parents.fill(-1);}
    explicit operator bool()const{return !error;}
    bool current(std::size_t i,AnimationSkeletonBone bone)const {
        return i<indices.size()&&indices[i]>=0&&names[i]==bone.name&&parents[i]==bone.parent;
    }
};
inline AnimationSkeletonBinding bindAnimationSkeleton(std::span<const std::string> names,
    std::span<const int> parents,std::span<const AnimationSkeletonBone> bones) {
    AnimationSkeletonBinding result;
    const auto fail=[&](const char* error,int track=-1,int actual=-1){result.error=error;result.track=track;result.actual=actual;return result;};
    if(names.size()!=99||parents.size()!=99||bones.empty()||bones.size()>1024)return fail("invalid animation skeleton dimensions");
    for(std::size_t i=0;i<99;++i) {
        if(names[i].empty()||parents[i]<-1||parents[i]>=int(i))return fail("invalid canonical bone",int(i));
        for(std::size_t j=0;j<i;++j)if(names[j]==names[i])return fail("duplicate canonical bone",int(i));
    }
    for(std::size_t i=0;i<bones.size();++i)
        if(bones[i].parent<-1||bones[i].parent>=int(bones.size()))return fail("animation parent outside skeleton",-1,int(i));
    std::array<unsigned char,1024> visited{};
    for(std::size_t i=0;i<bones.size();++i) {
        int cursor=int(i);
        while(cursor>=0&&!visited[cursor]){visited[cursor]=1;cursor=bones[cursor].parent;}
        if(cursor>=0&&visited[cursor]==1)return fail("cyclic animation skeleton",-1,cursor);
        cursor=int(i);
        while(cursor>=0&&visited[cursor]==1){visited[cursor]=2;cursor=bones[cursor].parent;}
    }
    for(std::size_t i=0;i<99;++i) {
        if(engineOwnedTrack(i,names,parents))continue;
        const bool optional=animationOnlyLeaf(i,names,parents);
        const auto alias=optional&&i>=1&&i<=3?std::string_view(names[i]).substr(2):std::string_view{};
        for(std::size_t j=0;j<bones.size();++j) {
            if(bones[j].name!=names[i]&&(alias.empty()||bones[j].name!=alias))continue;
            if(result.indices[i]>=0)return fail("ambiguous animation bone",int(i),int(j));
            result.indices[i]=int(j);
        }
        const int index=result.indices[i];
        if(index<0) {
            if(!optional)return fail("required animation bone missing",int(i));
            ++result.missingOptional;continue;
        }
        result.parents[i]=bones[index].parent;result.names[i]=bones[index].name;
        result.remapped+=index!=int(i);result.aliases+=bones[index].name!=names[i];
    }
    for(std::size_t i=0;i<99;++i) {
        if(result.indices[i]<0)continue;
        const int parent=parents[i]<0?-1:result.indices[parents[i]];
        if((parents[i]>=0&&parent<0)||result.parents[i]!=parent)
            return fail("animation bone parent mismatch",int(i),result.indices[i]);
    }
    return result;
}
}
