#pragma once
#include "ConverterEditor.h"

namespace fc {
struct ConverterWallRunCuts {
    float loopBegin{},loopEnd{};
};
struct ConverterAuthoredStage {
    ConverterEditorDocument document;
    ConverterEditedAnimation animation;
    ConverterEditOptions options;
};
struct ConverterWallRunSequence {
    std::string primary;
    ConverterWallRunCuts actualCuts;
    std::vector<ConverterAuthoredStage> stages;
    std::vector<std::string> warnings;
    ConverterEditedAnimation whole;
    std::array<std::array<float,2>,3> sourceRanges{};
};
struct ConverterWallRunDocument {
    ConverterEditorDocument document;
    ConverterWallRunCuts cuts;
    std::vector<ConverterAuthoredStage> stages;
    std::array<std::array<std::size_t,2>,3> ranges{};
    std::array<Vec,3> rootShifts{};
};
ConverterWallRunDocument loadConverterWallRunEditor(const std::filesystem::path&,const std::string&);
bool applyConverterWallRunEdits(const ConverterWallRunDocument&,const ConverterEditOptions&,
    ConverterWallRunCuts,ConverterWallRunSequence&,std::string&);
bool authorWallRunSequence(const ConverterEditorDocument&,const ConverterEditOptions&,std::string_view,
    ConverterWallRunCuts,ConverterWallRunSequence&,std::string&);
}
