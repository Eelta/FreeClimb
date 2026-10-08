#pragma once
#include "ConverterEditor.h"

namespace fc {
enum class ConverterActionPreviewKind { single,wallRun };
enum class ConverterActionPreviewStage { action,launch,loop,catching };
struct ConverterActionPreviewOptions {
    ConverterActionPreviewKind kind=ConverterActionPreviewKind::single;
    std::string direction="runUp";
    unsigned cycles=2;
};
struct ConverterActionPreviewFrame {
    float seconds{};
    Pose pose;
    Vec position;
    Quat orientation;
    Motion motion=Motion::none;
    float phase{};
    std::array<float,4> contacts{};
};
struct ConverterActionPreviewSegment {
    float begin{},end{};
    std::string slot;
    ConverterActionPreviewStage stage=ConverterActionPreviewStage::action;
};
struct ConverterActionPreview {
    std::vector<ConverterActionPreviewFrame> frames;
    std::vector<ConverterActionPreviewSegment> segments;
    float seconds{},wallDistance{30};
    std::optional<float> ledgeHeight;
    std::size_t surfaceQueries{};
    bool coreSimulated{};
    bool available() const {return frames.size()>=2&&seconds>0;}
};
ConverterActionPreview buildConverterActionPreview(const ConverterEditorDocument&,const ConverterEditedAnimation&,const ConverterActionPreviewOptions& options={},const std::vector<ConverterEditorExport>& overlays={});
ConverterActionPreviewFrame sampleConverterActionPreview(const ConverterActionPreview&,float seconds);
Pose placedConverterActionPreviewPose(const ConverterActionPreviewFrame&);
}
