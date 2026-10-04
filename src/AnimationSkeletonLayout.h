#pragma once
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace fc {
template<class Bone,class Reference> struct AnimationSkeletonLayout {
    const Bone* bones{};
    const std::int16_t* parents{};
    const Reference* references{};
    int boneCount{},parentCount{},referenceCount{};
    template<class Readable> bool valid(Readable readable) const {
        if(!bones||!parents||!references||boneCount<=0||boneCount>1024||
            parentCount<boneCount||parentCount>1024||referenceCount<boneCount||referenceCount>1024)return false;
        return readable(reinterpret_cast<std::uintptr_t>(bones),std::size_t(boneCount)*sizeof(Bone))&&
            readable(reinterpret_cast<std::uintptr_t>(parents),std::size_t(parentCount)*sizeof(std::int16_t))&&
            readable(reinterpret_cast<std::uintptr_t>(references),std::size_t(referenceCount)*sizeof(Reference));
    }
    bool operator==(const AnimationSkeletonLayout&) const=default;
};
template<class Skeleton> auto animationSkeletonLayout(const Skeleton& skeleton) {
    using Bone=std::remove_cvref_t<decltype(skeleton.bones[0])>;
    using Reference=std::remove_cvref_t<decltype(skeleton.referencePose[0])>;
    return AnimationSkeletonLayout<Bone,Reference>{skeleton.bones.data(),skeleton.parentIndices.data(),skeleton.referencePose.data(),
        skeleton.bones.size(),skeleton.parentIndices.size(),skeleton.referencePose.size()};
}
}
