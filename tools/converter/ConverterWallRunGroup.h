#pragma once
#include <algorithm>
#include <string_view>
#include <vector>

namespace fc::converter {
inline constexpr bool isWallRunHelper(std::string_view name){return name=="runLaunch"||name=="runLaunchLeft"||name=="runLaunchRight"||name=="runCatch"||name=="sideBrace";}
inline constexpr bool isWallRunPrimary(std::string_view name){return name=="runUp"||name=="runLeft"||name=="runRight"||name=="runDiagonalLeft"||name=="runDiagonalRight";}
inline std::vector<std::string_view> wallRunStages(std::string_view primary){
    if(primary=="runUp")return {"runLaunch","runUp","runCatch"};
    if(primary=="runLeft")return {"runLaunchLeft","runLeft","runCatch","sideBrace"};
    if(primary=="runRight")return {"runLaunchRight","runRight","runCatch","sideBrace"};
    if(primary=="runDiagonalLeft")return {"runLaunchLeft","runDiagonalLeft","runCatch","sideBrace"};
    if(primary=="runDiagonalRight")return {"runLaunchRight","runDiagonalRight","runCatch","sideBrace"};
    return {};
}
inline std::string_view wallRunPrimaryForSlot(std::string_view name,std::string_view preferred){
    if(!isWallRunHelper(name))return name;
    const auto stages=wallRunStages(preferred);if(std::find(stages.begin(),stages.end(),name)!=stages.end())return preferred;
    if(name=="runLaunch")return "runUp";
    if(name=="runLaunchLeft")return "runLeft";
    if(name=="runLaunchRight")return "runRight";
    return "runLeft";
}
inline constexpr bool isActionHelper(std::string_view name){return isWallRunHelper(name)||name=="contextHang";}
inline constexpr bool isContextHop(std::string_view name){return name=="contextHopLeft"||name=="contextHopRight";}
inline constexpr bool isActionGroupPrimary(std::string_view name){return name=="wallRun"||name=="contextHop";}
inline constexpr bool isActionGroupMember(std::string_view name){return isWallRunPrimary(name)||isActionHelper(name)||name=="contextHopLeft"||name=="contextHopRight";}
inline std::vector<std::string_view> actionStages(std::string_view primary){
    if(primary=="contextHop")return {"contextHopLeft","contextHopRight","contextHang"};
    if(primary=="wallRun")return {"runUp","runLeft","runRight","runDiagonalLeft","runDiagonalRight","runLaunch","runLaunchLeft","runLaunchRight","runCatch","sideBrace"};
    return {};
}
inline std::string_view actionPrimaryForSlot(std::string_view name,std::string_view={}){
    if(isWallRunPrimary(name)||isWallRunHelper(name))return "wallRun";
    return name;
}
}
