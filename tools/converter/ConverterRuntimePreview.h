#pragma once
#include "ConverterEditor.h"

namespace fc {
struct ConverterRuntimePreview {
    std::vector<Pose> frames;
    float seconds{},wallDistance{30};
    std::size_t surfaceQueries{};
    bool available() const {return frames.size()>=2&&seconds>0;}
};
bool converterRuntimePreviewSupported(std::string_view slot);
ConverterRuntimePreview buildConverterRuntimePreview(const ConverterEditorDocument&,const ConverterEditedAnimation&,const std::vector<ConverterEditorExport>& overlays={});
Pose sampleConverterRuntimePreview(const ConverterRuntimePreview&,float seconds);
}
