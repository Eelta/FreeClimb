#pragma once
#include "MotionSlots.h"
#include "../external/nlohmann/json.hpp"
#include <span>
#include <cmath>
#include <set>
#include <stdexcept>

namespace fc {
inline std::array<std::size_t,2> animationFrameRange(const nlohmann::json& clip,std::size_t count) {
    if(count<2)throw std::runtime_error("Animation requires at least two frames");
    if(!clip.contains("frameRange")) {
        if(clip.contains("rootShift"))throw std::runtime_error("Root shift requires a timeline frame range");
        return {0,count-1};
    }
    const auto& range=clip.at("frameRange");
    if(!range.is_array()||range.size()!=2)throw std::runtime_error("Timeline range requires two frame indices");
    for(const auto& value:range)if(!value.is_number_integer()||value<0||value>=count)
        throw std::runtime_error("Timeline frame index is outside the animation");
    const std::array<std::size_t,2> result{range[0].get<std::size_t>(),range[1].get<std::size_t>()};
    if(result[0]>=result[1])throw std::runtime_error("Timeline range must contain at least two frames");
    if(clip.contains("rootShift")) {
        const auto& shift=clip.at("rootShift");
        if(!shift.is_array()||shift.size()!=3)throw std::runtime_error("Timeline root shift requires three components");
        for(const auto& value:shift)if(!value.is_number()||!std::isfinite(value.get<double>())||std::abs(value.get<double>())>10000)
            throw std::runtime_error("Timeline root shift exceeds its bounds");
    }
    return result;
}
inline std::string_view wallRunLaunchSlot(std::string_view direction) {
    if(direction=="runUp")return "runLaunch";
    if(direction=="runLeft"||direction=="runDiagonalLeft")return "runLaunchLeft";
    if(direction=="runRight"||direction=="runDiagonalRight")return "runLaunchRight";
    throw std::runtime_error("Invalid wall-run sequence direction");
}
inline std::span<const Motion> actionGroupMotions(std::string_view group) {
    static constexpr std::array wallRun{Motion::runUp,Motion::runLeft,Motion::runRight,Motion::runDiagonalLeft,
        Motion::runDiagonalRight,Motion::runLaunch,Motion::runCatch,Motion::runLaunchLeft,Motion::runLaunchRight,Motion::sideBrace};
    static constexpr std::array contextHop{Motion::contextHang,Motion::contextHopLeft,Motion::contextHopRight};
    if(group=="wallRun")return wallRun;
    if(group=="contextHop")return contextHop;
    return {};
}
inline const nlohmann::json& selectAnimationClipConfig(const nlohmann::json& document,std::string_view slot) {
    const auto reject=[](bool accepted,const char* message){if(!accepted)throw std::runtime_error(message);};
    const auto found=std::find(motionSlotNames.begin(),motionSlotNames.end(),slot);
    reject(!slot.empty()&&found!=motionSlotNames.end(),"Unknown or retired animation slot");
    if(document.value("format",std::string{})=="FreeClimbClip") {
        reject(document.at("slot").get<std::string>()==slot,"Clip configuration is assigned to the wrong slot");
        return document;
    }
    const bool timeline=document.value("version",0)==2;
    reject(document.value("format",std::string{})=="FreeClimbActionGroup"&&(document.at("version")==1||timeline),"Unsupported complete-action configuration");
    const bool contextHop=timeline&&document.at("group")=="contextHop";
    reject(!timeline||document.at("group")=="wallRun"||contextHop,"Unsupported timeline action group");
    const auto layoutDirection=document.value("direction",std::string{});
    const bool separate=timeline&&!layoutDirection.empty();
    const bool ownsBrace=!contextHop&&separate&&layoutDirection=="runLeft"&&document.at("clips").size()==4;
    std::array<Motion,4> directionMembers{};
    if(contextHop) {
        reject(layoutDirection=="contextHopLeft"||layoutDirection=="contextHopRight","Invalid side-hop sequence direction");
        directionMembers[0]=Motion::contextHang;directionMembers[1]=layoutDirection=="contextHopLeft"?Motion::contextHopLeft:Motion::contextHopRight;
    } else if(separate) {
        const auto launch=wallRunLaunchSlot(layoutDirection);
        const std::array<std::string_view,3> names{launch,layoutDirection,"runCatch"};
        for(std::size_t i=0;i<names.size();++i)directionMembers[i]=Motion(std::find(motionSlotNames.begin(),motionSlotNames.end(),names[i])-motionSlotNames.begin()+1);
        if(ownsBrace)directionMembers[3]=Motion::sideBrace;
    }
    const auto members=separate?std::span<const Motion>(directionMembers.data(),contextHop?2:ownsBrace?4:3):actionGroupMotions(document.at("group").get<std::string>());
    reject(!members.empty(),"Unknown complete-action group");
    const auto& clips=document.at("clips");
    reject(clips.is_array()&&clips.size()==members.size(),"Complete action must include every required stage");
    std::array<bool,motionCount> seen{};
    const nlohmann::json* selected=nullptr;
    std::string sharedFile;
    std::vector<std::string> bindings;
    for(const auto& clip:clips) {
        reject(clip.is_object()&&clip.value("format",std::string{})=="FreeClimbClip"&&
            (clip.at("version")==1||clip.at("version")==2),"Invalid complete-action stage");
        const auto name=clip.at("slot").get<std::string>();
        const auto item=std::find(motionSlotNames.begin(),motionSlotNames.end(),name);
        reject(!name.empty()&&item!=motionSlotNames.end(),"Unknown complete-action stage slot");
        const auto index=std::size_t(item-motionSlotNames.begin());
        reject(std::find(members.begin(),members.end(),Motion(index+1))!=members.end()&&!seen[index],"Duplicate or unrelated complete-action stage");
        seen[index]=true;
        const auto file=clip.at("file").get<std::string>(),binding=clip.at("member").get<std::string>();
        reject(!file.empty()&&!binding.empty()&&binding.size()<=64&&
            std::all_of(binding.begin(),binding.end(),[](unsigned char c){return (c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='_';}),"Invalid complete-action HKX binding");
        if(sharedFile.empty())sharedFile=file;
        reject(sharedFile==file&&(timeline||std::find(bindings.begin(),bindings.end(),binding)==bindings.end()),"Complete-action stages require unique bindings in one HKX");
        if(timeline)animationFrameRange(clip,1201);
        bindings.push_back(binding);
        if(name==slot)selected=&clip;
    }
    if(timeline) {
        const auto& sequences=document.at("sequences");
        reject(sequences.is_array()&&sequences.size()==(separate?1:5),"Wall-running sequence count does not match its direction layout");
        std::set<std::string> directions;
        for(const auto& sequence:sequences) {
            reject(sequence.is_object(),"Invalid wall-run sequence");
            const auto direction=sequence.at("slot").get<std::string>();
            reject(!separate||document.at("direction")==direction,"Wall-running sequence must match its direction file");
            const auto launchSlot=contextHop?std::string_view("contextHang"):wallRunLaunchSlot(direction);
            reject(directions.insert(direction).second,"Duplicate wall-run sequence direction");
            const auto cycle=std::find_if(clips.begin(),clips.end(),[&](const auto& value){return value.at("slot")==direction;});
            const bool completeSource=contextHop&&cycle!=clips.end()&&cycle->contains("authoredPlayback")&&!cycle->contains("frameRange");
            reject(cycle!=clips.end()&&cycle->at("member")==direction&&(cycle->contains("frameRange")||completeSource),"Complete action requires its own playback range");
            const auto loopRange=animationFrameRange(*cycle,1201);
            std::array<std::array<std::size_t,2>,2> ranges{};
            for(unsigned index=0;index<2;++index) {
                const auto& part=sequence.at(index?"catch":contextHop?"prepare":"launch");
                reject(part.is_object()&&part.value("format",std::string{})=="FreeClimbClip"&&
                    (part.at("version")==1||part.at("version")==2)&&part.at("slot")==std::string(contextHop?"contextHang":index?"runCatch":launchSlot)&&
                    part.at("file")==sharedFile&&part.at("member")==direction+(completeSource?(index?"Catch":"Prepare"):"")&&
                    (part.contains("frameRange")||completeSource),"Invalid direction-specific complete-action stage");
                ranges[index]=animationFrameRange(part,1201);
                if(separate&&(!contextHop||index==0)) {
                    const auto stored=std::find_if(clips.begin(),clips.end(),[&](const auto& value){return value.at("slot")==part.at("slot");});
                    reject(stored!=clips.end()&&*stored==part,"Direction stage metadata must match its timeline section");
                }
            }
            reject(completeSource||(ranges[0][1]<=loopRange[0]&&loopRange[1]<=ranges[1][0]),"Action start, movement and end ranges must occur in order");
            if(sequence.contains("brace")) {
                const auto& brace=sequence.at("brace");
                reject(!contextHop&&direction!="runUp"&&brace.is_object()&&brace.value("format",std::string{})=="FreeClimbClip"&&
                    (brace.at("version")==1||brace.at("version")==2)&&brace.at("slot")=="sideBrace"&&brace.at("file")==sharedFile&&
                    brace.at("member")==direction+"Brace","Invalid private wall-run support reference");
                animationFrameRange(brace,1201);
                if(ownsBrace) {
                    const auto stored=std::find_if(clips.begin(),clips.end(),[](const auto& value){return value.at("slot")=="sideBrace";});
                    reject(stored!=clips.end()&&*stored==brace,"Stored wall-run support reference must match its sequence");
                }
            } else reject(!ownsBrace,"Owned support reference requires matching sequence metadata");
        }
    }
    reject(selected!=nullptr,"Complete action does not contain the requested stage");
    return *selected;
}
inline nlohmann::json& selectAnimationClipConfig(nlohmann::json& document,std::string_view slot) {
    return const_cast<nlohmann::json&>(selectAnimationClipConfig(static_cast<const nlohmann::json&>(document),slot));
}
inline const nlohmann::json& selectAnimationClipConfig(const nlohmann::json& document,std::string_view slot,std::string_view direction) {
    if(slot=="sideBrace"&&!direction.empty()&&document.value("format",std::string{})=="FreeClimbActionGroup"&&document.value("version",0)==2&&document.at("group")=="wallRun") {
        selectAnimationClipConfig(document,direction);
        for(const auto& sequence:document.at("sequences"))if(sequence.at("slot").get<std::string>()==direction&&sequence.contains("brace"))return sequence.at("brace");
        if(!document.contains("direction"))return selectAnimationClipConfig(document,slot);
        throw std::runtime_error("Wall-run direction does not own a support reference");
    }
    const auto& fallback=selectAnimationClipConfig(document,slot);
    if(direction.empty()||document.value("format",std::string{})!="FreeClimbActionGroup"||document.value("version",0)!=2)return fallback;
    if(document.at("group")=="contextHop") {
        if(slot!="contextHang")return fallback;
        for(const auto& sequence:document.at("sequences"))if(sequence.at("slot").get<std::string>()==direction)return sequence.at("prepare");
        throw std::runtime_error("Side-hop direction is missing from the action group");
    }
    const auto launch=wallRunLaunchSlot(direction);
    if(slot!="runCatch"&&slot!=launch)return fallback;
    for(const auto& sequence:document.at("sequences"))if(sequence.at("slot").get<std::string>()==direction)return sequence.at(slot=="runCatch"?"catch":"launch");
    throw std::runtime_error("Wall-run direction is missing from the action group");
}
inline nlohmann::json& selectAnimationClipConfig(nlohmann::json& document,std::string_view slot,std::string_view direction) {
    return const_cast<nlohmann::json&>(selectAnimationClipConfig(static_cast<const nlohmann::json&>(document),slot,direction));
}
}
