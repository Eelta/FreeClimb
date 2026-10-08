#pragma once
#include "../../src/Pose.h"
#include <span>

namespace fc {
struct ConverterAnnotation {float time{};std::size_t track{};std::string text;};
struct ConverterReferenceFrame {
    float duration{};
    Vec up{},forward{};
    std::vector<std::array<float,4>> samples;
};
struct ConverterInputClip {
    float duration{};
    unsigned ignoredFloatTracks{};
    bool identityMapping{};
    bool partialAnnotationNames{};
    std::string skeletonName;
    std::vector<int> boneIndices;
    std::vector<std::string> trackNames;
    std::vector<std::string> annotationTrackNames;
    std::vector<std::vector<Quat>> rotations;
    std::vector<Pose> frames;
    std::vector<ConverterAnnotation> annotations;
    ConverterReferenceFrame referenceFrame;
};
inline bool converterExternalMovement(const ConverterInputClip& clip) {
    if(!clip.referenceFrame.samples.empty())return true;
    for(const auto& event:clip.annotations)for(const auto command:{std::string_view{"animmotion"},std::string_view{"animrotation"}}) {
        const std::string_view text=event.text;
        if(text.starts_with(command)&&text.size()>command.size()&&(text[command.size()]==' '||text[command.size()]=='\t'))return true;
    }
    return false;
}
bool decodeConverterInput(std::span<const std::uint8_t>,ConverterInputClip&,std::string&);
bool decodeConverterInput(std::span<const std::uint8_t>,ConverterInputClip&,std::string&,std::string_view member);
}
