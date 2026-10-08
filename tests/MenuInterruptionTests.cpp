#include "GamepadInput.h"
#include "MenuInterruption.h"
#include "TraversalSuspension.h"
#include <array>
#include <iostream>
#include <stdexcept>
#include <string_view>
using namespace fc;
static unsigned checks{};
static void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
static void classification() {
    struct Case {std::string_view name;bool alwaysBlocks;};
    constexpr std::array cases{
        Case{"Console",true},Case{"Dialogue Menu",true},Case{"Loading Menu",true},Case{"TweenMenu",true},
        Case{"HUD Menu",false},Case{"Cursor Menu",false},Case{"NotificationOverlay",false},
        Case{"CustomWidget",false},Case{"InventoryMenu",false},Case{"FavoritesMenu",false},
        Case{"MapMenu",false},Case{"Journal Menu",false},Case{"",false},Case{"ConsoleOverlay",false},
        Case{"Overlay Dialogue Menu",false},Case{"TweenMenuWidget",false}
    };
    for(const auto& value:cases)for(unsigned bits=0;bits<8;++bits) {
        const MenuInputState state{bool(bits&1),bool(bits&2),bool(bits&4)};
        const bool expected=value.alwaysBlocks||bits!=0;
        check(menuBlocksTraversal(value.name,state)==expected,"menu name and actual input flags classify independently");
        check(menuInterruptsTraversal(value.name,true,state)==expected,"opening event agrees with current blocking state");
        check(!menuInterruptsTraversal(value.name,false,state),"closing any menu never creates a new interruption");
    }
    check(!menuBlocksTraversal("UnknownOverlay"),"an unclassified overlay without blocking evidence stays harmless");
    check(menuBlocksTraversal("",{true,false,false}),"pause evidence does not depend on a valid menu name");
    check(menuBlocksTraversal("",{false,true,false}),"item-menu evidence does not depend on a valid menu name");
    check(menuBlocksTraversal("",{false,false,true}),"menu-input context does not depend on a valid menu name");
}
struct Session {
    InputState keyboard;
    GamepadState gamepad;
    GamepadSettings settings;
    WallRunEntryGate runGate;
    ClimbEntryIntent entry;
    JumpGrabGate request;
    EntryPreparationGrace preparation;
    bool fromGamepad{},attached{true};
    TraversalSuspension suspension;
    unsigned releases{};
    explicit Session(bool pad):fromGamepad(pad) {
        for(unsigned scan:{0x11u,0x1eu,0x20u,0x39u,0x2au})keyboard.set(scan,true);
        gamepad.sampleXInput(0x8100,255,255,0,32767,settings);
        check(entry.sample(keys()).fresh,"fixture begins with a complete entry combination");
        runGate.begin(keys());request.request();preparation.arm(fromGamepad);
    }
    Keys keys()const{return fromGamepad?gamepad.keys(settings.bindings):mapKeys(keyboard,InputBindings{});}
    void interrupt() {
        keyboard.reset();gamepad.blockUntilButtonsReleased();runGate.reset();
        suspension.suspend();entry.blockUntilRelease();request.cancel();preparation.cancel();
    }
    void event(std::string_view name,bool opening,MenuInputState state={}) {
        if(menuInterruptsTraversal(name,opening,state))interrupt();
    }
};
static void harmlessEventsPreserveInput() {
    for(bool pad:{false,true}) {
        Session session(pad);
        for(const auto name:{"HUD Menu","Cursor Menu","NotificationOverlay","CustomWidget"}) {
            session.event(name,true);session.event(name,false);
            check(session.attached&&session.releases==0,"nonblocking overlay changes preserve traversal");
            const auto keys=session.keys();
            check(keys.w&&keys.space&&keys.shift&&keys.entry,"harmless events retain complete keyboard and gamepad input");
            check(!session.runGate.filter(keys).shift,"harmless events cannot reset the entry modifier release gate");
            check(!session.gamepad.waitingForButtonsRelease(),"harmless events do not suspend the controller");
            check(session.request.pending(),"harmless events do not cancel a pending grab");
            check(session.preparation.sample(.01f,pad,keys),"harmless events preserve pending pose preparation");
            const auto continued=session.entry.sample(keys);
            check(continued.requested&&!continued.fresh,"harmless events preserve the held entry gesture");
        }
        session.event("Console",false,{true,true,true});
        check(session.attached&&!session.gamepad.waitingForButtonsRelease(),"closing a blocking menu cannot reset retained input");
        check(!session.runGate.filter(session.keys()).shift,"closing a menu cannot enable a held wall-run modifier");
    }
}
static void blockingMenusPauseSafely() {
    struct Case {std::string_view name;MenuInputState state;};
    constexpr std::array cases{
        Case{"Console",{}},Case{"Dialogue Menu",{}},Case{"Loading Menu",{}},Case{"TweenMenu",{}},
        Case{"PausedCustomMenu",{true,false,false}},Case{"InventoryMenu",{false,true,false}},
        Case{"CustomControlMenu",{false,false,true}}
    };
    for(bool pad:{false,true})for(const auto& value:cases) {
        Session session(pad);
        session.event(value.name,true,value.state);
        check(session.attached&&session.releases==0&&session.suspension.active(),"a blocking menu holds traversal without a release");
        check(!mapKeys(session.keyboard,InputBindings{}).entry,"blocking menus clear stale keyboard input");
        check(session.gamepad.waitingForButtonsRelease()&&!session.gamepad.keys(session.settings.bindings).entry,
            "blocking menus suspend controller actions without inventing releases");
        check(!session.request.pending()&&!session.preparation.sample(.001f,pad,session.keys()),
            "blocking menus cancel pending grabs and pose preparation");
        check(session.entry.waitingForRelease(),"blocking menus require a fresh entry gesture");
        session.event(value.name,false,value.state);
        check(session.attached&&session.releases==0&&session.suspension.active(),"closing events wait for the complete live menu stack before resuming");
        check(!session.gamepad.resumeIfButtonsReleased(),"closing the menu with held entry buttons remains blocked");
        session.gamepad.sampleXInput(0,255,0,0,32767,session.settings);
        check(!session.gamepad.resumeIfButtonsReleased(),"a held left trigger prevents premature controller recovery");
        session.gamepad.sampleXInput(0,0,255,0,32767,session.settings);
        check(!session.gamepad.resumeIfButtonsReleased(),"a held right trigger prevents premature controller recovery");
        session.gamepad.sampleXInput(0,0,0,0,32767,session.settings);
        check(session.gamepad.resumeIfButtonsReleased()&&!session.gamepad.neutral(),
            "releasing all buttons rearms the controller while the forward stick remains held");
        check(session.gamepad.keys(session.settings.bindings).w,"controller recovery retains physical movement direction");
        session.entry.sample(session.keys());
        if(pad)session.gamepad.sampleXInput(0x8100,0,0,0,32767,session.settings);
        else for(unsigned scan:{0x11u,0x1eu,0x20u,0x39u})session.keyboard.set(scan,true);
        check(!session.entry.sample(session.keys(),session.attached).requested,"an already attached character does not reattach after a menu");
    }
}
static void syntheticOverlayTimingRegression() {
    constexpr std::array<unsigned,36> delaysMs{92,5,205,83,290,205,204,307,167,369,6,142,81,69,54,221,
        279,245,134,34,266,140,6,296,31,210,159,96,149,32,206,267,338,125,30,361};
    unsigned interruptions{},legacyInterruptions{};
    const auto legacyPolicy=[](std::string_view name,bool opening){return opening&&name!="HUD Menu"&&name!="Cursor Menu";};
    for(unsigned index=0;index<delaysMs.size();++index) {
        Session session(index<28),legacy(index<28);
        for(unsigned ms=0;ms<delaysMs[index];++ms)session.request.tick(.001f);
        session.event("SyntheticNotificationOverlay",true);
        if(legacyPolicy("SyntheticNotificationOverlay",true)){legacy.interrupt();legacy.attached=false;++legacy.releases;}
        interruptions+=session.releases;legacyInterruptions+=legacy.releases;
        check(session.attached&&session.keys().entry&&session.request.pending(),
            "synthetic early overlay openings retain attachment and input across both devices");
        check(!session.runGate.filter(session.keys()).shift,"synthetic overlay bursts retain modifier-release gating");
        session.event("SyntheticNotificationOverlay",false);
        check(session.attached,"synthetic overlay closing events preserve attachment");
    }
    check(interruptions==0&&legacyInterruptions==36,"harmless synthetic events reproduce only the old unconditional drop policy");
}
int main()try {
    classification();harmlessEventsPreserveInput();blockingMenusPauseSafely();syntheticOverlayTimingRegression();
    std::cout<<"Menu interruption checks passed: "<<checks<<'\n';return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
