#pragma once
#include <string_view>

namespace fc {
struct MenuInputState {bool pausesGame{},itemMenu{},menuContext{};};
constexpr bool menuBlocksTraversal(std::string_view name,MenuInputState state={}) {
    return state.pausesGame||state.itemMenu||state.menuContext||name=="Console"||
        name=="Dialogue Menu"||name=="Loading Menu"||name=="TweenMenu";
}
constexpr bool menuInterruptsTraversal(std::string_view name,bool opening,MenuInputState state={}) {
    return opening&&menuBlocksTraversal(name,state);
}
}
