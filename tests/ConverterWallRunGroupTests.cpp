#include "../tools/converter/ConverterWallRunGroup.h"
#include "../src/MotionSlots.h"
#include <iostream>
#include <set>
#include <stdexcept>

namespace {
std::size_t checks{};
void require(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
}
int main(){try{
    using namespace fc::converter;
    const std::array<std::string_view,5> primaries{"runUp","runLeft","runRight","runDiagonalLeft","runDiagonalRight"};
    const std::array<std::string_view,5> helpers{"runLaunch","runLaunchLeft","runLaunchRight","runCatch","sideBrace"};
    const std::array<std::vector<std::string_view>,5> expected{{
        {"runLaunch","runUp","runCatch"},{"runLaunchLeft","runLeft","runCatch","sideBrace"},
        {"runLaunchRight","runRight","runCatch","sideBrace"},{"runLaunchLeft","runDiagonalLeft","runCatch","sideBrace"},
        {"runLaunchRight","runDiagonalRight","runCatch","sideBrace"}
    }};
    std::set<std::string_view> active;for(auto motion:fc::activeMotions)active.insert(fc::motionSlotNames[int(motion)-1]);
    require(active.size()==31,"Internal active slots exclude four deleted actions");
    for(const auto removed:{"sprintCatch","flipUp","flipLeft","flipRight"})require(!active.contains(removed)&&actionStages(removed).empty(),"Removed actions cannot be reintroduced as grouped helpers");
    require(active.contains("backFlipOut"),"Wall departure backflip remains active");
    for(std::size_t i=0;i<primaries.size();++i){const auto primary=primaries[i];require(isWallRunPrimary(primary)&&!isWallRunHelper(primary),"Primary classification");const auto stages=wallRunStages(primary);require(stages==expected[i],"Ordered wall-run stages");require(std::set<std::string_view>(stages.begin(),stages.end()).size()==stages.size(),"No duplicate stage in group");
        for(const auto stage:stages){require(active.contains(stage),"Every stage retains an active runtime slot");require(wallRunPrimaryForSlot(stage,primary)==primary,"Shared helper retains compatible preferred group");}
        for(const auto helper:helpers){const auto chosen=wallRunPrimaryForSlot(helper,primary);const auto destination=wallRunStages(chosen);require(isWallRunPrimary(chosen)&&std::find(destination.begin(),destination.end(),helper)!=destination.end(),"Every helper resolves to a group containing it");}
    }
    const std::array<std::string_view,5> defaults{"runUp","runLeft","runRight","runLeft","runLeft"};
    for(std::size_t i=0;i<helpers.size();++i){require(isWallRunHelper(helpers[i])&&!isWallRunPrimary(helpers[i]),"Helper classification");require(wallRunStages(helpers[i]).empty(),"Helper is not a group primary");for(const auto preference:{std::string_view{},std::string_view{"unknown"},std::string_view{"hang"}})require(wallRunPrimaryForSlot(helpers[i],preference)==defaults[i],"Helper fallback is deterministic");}
    require(wallRunPrimaryForSlot("runCatch","runUp")=="runUp","Upward catch retains upward group");require(wallRunPrimaryForSlot("sideBrace","runUp")=="runLeft","Upward group has no side brace");
    require(wallRunPrimaryForSlot("runLaunchLeft","runDiagonalLeft")=="runDiagonalLeft","Left diagonal retains shared launch");require(wallRunPrimaryForSlot("runLaunchRight","runDiagonalRight")=="runDiagonalRight","Right diagonal retains shared launch");
    std::set<std::string_view> visible;std::size_t helperCount=0,primaryCount=0;
    for(const auto slot:active){helperCount+=isWallRunHelper(slot);primaryCount+=isWallRunPrimary(slot);if(!isWallRunHelper(slot)){visible.insert(slot);require(wallRunPrimaryForSlot(slot,"runDiagonalRight")==slot,"Nonhelpers retain their own slot");}else require(!visible.contains(slot),"Helpers are not main choices");}
    require(helperCount==5&&primaryCount==5&&visible.size()==26,"31 runtime slots become 26 wall-run-filtered choices");
    const std::array<std::string_view,2> grouped{"wallRun","contextHop"};
    for(const auto primary:grouped){require(isActionGroupPrimary(primary)&&!isActionHelper(primary),"Complete group classification");const auto stages=actionStages(primary);require(stages.size()==(primary=="wallRun"?10:3),"Every complete action group contains all directions and helpers");
        require(std::set<std::string_view>(stages.begin(),stages.end()).size()==stages.size(),"Complete action has no duplicate stage");
        for(const auto stage:stages){require(active.contains(stage)&&isActionGroupMember(stage),"Every complete-action stage retains an active runtime slot");require(actionPrimaryForSlot(stage)==(primary=="wallRun"?primary:stage),"Side directions remain independently selected actions");}
    }
    require(actionStages("runLeft").empty()&&actionStages("contextHopLeft").empty(),"Directions are stages rather than separate exported groups");
    for(const auto preference:{"contextHopLeft","contextHopRight","wallRun","unknown"})require(actionPrimaryForSlot("contextHang",preference)=="contextHang","Internal preparation cannot select or export both side actions");
    std::set<std::string_view> mainChoices;std::size_t allHelpers=0;
    for(const auto slot:active){allHelpers+=isActionHelper(slot);mainChoices.insert(actionPrimaryForSlot(slot));}
    require(active.size()==31&&allHelpers==6&&mainChoices.size()==22&&mainChoices.contains("wallRun")&&mainChoices.contains("contextHopLeft")&&mainChoices.contains("contextHopRight"),"Thirty-one internal stages form twenty complete author actions");
    for(const auto name:{"","unknown","runDown","runLeftExtra","RunUp","runlaunch","hang"}){require(!isWallRunPrimary(name)&&!isWallRunHelper(name),"Classification uses exact known names");require(wallRunStages(name).empty(),"Non-wall-run names have no group");require(wallRunPrimaryForSlot(name,"runUp")==name,"Ordinary and unknown names are preserved");require(!isActionHelper(name)&&!isActionGroupPrimary(name)&&actionStages(name).empty(),"Generic classification uses exact known names");require(actionPrimaryForSlot(name,"contextHopRight")==name,"Generic unknown names are preserved");}
    std::cout<<checks<<" wall-run group checks passed\n";return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
