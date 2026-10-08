#pragma once
#include "Converter.h"
#include <optional>

namespace fc {
struct ConverterBoneEdit {
    unsigned bone{};
    Vec eulerDegrees{};
    float beginPhase{},endPhase{1},fadePhase{.1f};
};
struct ConverterContactEdit {
    unsigned contact{};
    float beginPhase{},endPhase{1},fadePhase{.05f},weight{1};
};
struct ConverterWindowEdit {
    std::string role;
    float beginPhase{},endPhase{1};
};
struct ConverterGeometryEdit {
    float stride{},height{};
    Vec travel;
    std::vector<std::array<float,4>> path;
};
struct ConverterEditOptions {
    float trimIn{},trimOut{-1},speed{1},yawDegrees{};
    Vec rootOffset{},comOffset{};
    Vec makeInPlaceAxes{};
    bool autoCalibration{true};
    std::vector<ConverterBoneEdit> bones;
    std::vector<ConverterContactEdit> contacts;
    std::vector<ConverterWindowEdit> windows;
    std::optional<ConverterGeometryEdit> geometry;
};
struct ConverterEditorDocument {
    std::filesystem::path input,pack;
    std::string slot;
    bool authored{};
    Library base;
    ConverterInputClip clip;
    nlohmann::json templateConfig;
    std::vector<std::string> warnings;
    std::vector<ConverterFileSnapshot> snapshots;
    std::string direction;
    bool preserveTemplateTiming{};
    nlohmann::json contextGroup;
};
struct ConverterEditedAnimation {
    ConverterInputClip clip;
    nlohmann::json config;
    std::vector<std::string> warnings;
    float confidence{};
    std::vector<std::array<float,2>> phaseMap;
    nlohmann::json contextGroup;
};
ConverterEditorDocument loadConverterEditor(const std::filesystem::path&,const std::filesystem::path&,const std::string&,const std::string& direction={});
ConverterEditorDocument loadConverterBaseEditor(const std::filesystem::path&,const std::string&,const std::string& direction={});
bool converterEditableBone(const Library&,unsigned);
bool converterEditableStride(const ConverterEditorDocument&,const ConverterEditedAnimation&);
ConverterEditedAnimation applyConverterEdits(const ConverterEditorDocument&,const ConverterEditOptions&);
Pose sampleConverterEditor(const ConverterEditedAnimation&,float seconds);
std::array<float,4> sampleConverterContacts(const ConverterEditedAnimation&,float seconds);
nlohmann::json exportConverterEditor(const ConverterEditorDocument&,const ConverterEditedAnimation&,const std::filesystem::path&,bool overwrite=false);
struct ConverterEditorExport {
    const ConverterEditorDocument* document{};
    const ConverterEditedAnimation* animation{};
    std::string direction;
};
nlohmann::json exportConverterEditorGroup(const std::vector<ConverterEditorExport>&,const std::filesystem::path&,bool overwrite=false);
}
