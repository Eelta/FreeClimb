#include "SettingsMenuEdits.h"
#include "SettingsMenuHelp.h"
#include "TranslationDefaults.h"
#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>

using namespace fc;
namespace {
unsigned checks{};
void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
BindingCaptureSnapshot keys(std::initializer_list<KeyCode> values={}) {
    BindingCaptureSnapshot snapshot;snapshot.keyboardValid=true;
    for(const auto value:values)snapshot.keyboard.set(value,true);
    return snapshot;
}
BindingCaptureSnapshot buttons(GamepadChord values=0) {
    BindingCaptureSnapshot snapshot;snapshot.gamepadValid=true;snapshot.gamepad=values;return snapshot;
}
}
int main(int argc,char** argv) {
    try {
        SettingsMenuHelp help;TranslationCatalog catalog{std::span<const TranslationEntry>{translationDefaults}};
        check(help.helpKey.empty(),"an idle menu starts with an empty help area");
        for(const auto& entry:settingsHelpEntries) {
            check(std::string_view(catalog.text("english",entry.label))!=entry.label&&
                std::string_view(catalog.text("english",entry.help))!=entry.help,"every contextual option and help key has a fallback translation");
            check(settingsHelpFor(entry.label)==entry.help,"each option resolves its own explanation");
            help.observe(entry.label,entry.help,true,false);
            check(help.labelKey==entry.label&&help.helpKey==entry.help,"hovering replaces the previous option explanation");
        }
        help.reset();help.observe("focused","focus help",false,true);
        check(help.helpKey=="focus help","keyboard or gamepad focus supplies contextual help without a mouse");
        help.observe("hovered","hover help",true,false);help.observe("later focus","later help",false,true);
        check(help.labelKey=="hovered"&&help.helpKey=="hover help","mouse hover takes priority regardless of focused item order");
        help.reset();help.observe("idle","idle help",false,false);help.observe("unmapped",{},true,false);
        check(help.helpKey.empty()&&help.labelKey.empty(),"next frame and page switches cannot retain stale help or show idle paragraphs");
        const auto base=argc>1?std::filesystem::path(argv[1]):std::filesystem::current_path();
        const auto directory=base/("settings-menu-edits-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        std::filesystem::create_directories(directory);const auto path=directory/"FreeClimb.ini";
        {std::ofstream file(path);file<<"[OtherMod]\nKeep=unchanged\n";}
        UserSettings draft;SettingsMenuEdits edits;edits.initialize(draft);
        unsigned writes{};UserSettings requested;
        const auto submit=[&](const UserSettings& value) {
            std::string error;check(saveUserSettings(path,value,error),"valid automatic submission writes the actual INI");
            ++writes;requested=value;
        };
        const auto saved=[&] {
            const auto loaded=loadUserSettings(path);
            check(loaded.found&&loaded.warnings.empty()&&userSettingsIni(loaded.settings)==userSettingsIni(draft),"automatic commit persists exactly the accepted values");
            check(userSettingsIni(requested)==userSettingsIni(draft),"runtime queue receives the same values written to disk");
        };
        for(unsigned i=0;i<2400;++i)check(edits.apply(draft,submit)==SettingsEditResult::unchanged,"unchanged renders do not enqueue settings or rewrite disk");
        check(writes==0,"opening the menu alone does not write settings");
        const std::array<std::function<void(UserSettings&)>,30> changes{{
            [](auto& u){u.enabled=false;},[](auto& u){u.notifications=false;},[](auto& u){u.lowStaminaNotifications=false;},
            [](auto& u){u.autoMantle=false;},[](auto& u){u.upSpeed=110;},[](auto& u){u.downSpeed=68;},
            [](auto& u){u.sideSpeed=71;},[](auto& u){u.wallRunEnabled=false;},[](auto& u){u.wallRunSpeed=401;},[](auto& u){u.diagonalRunMultiplier=1.2f;},
            [](auto& u){u.automaticClimbActions=false;},[](auto& u){u.wallRunObstacleJumps=false;},
            [](auto& u){u.contextualMantleEnabled=false;},[](auto& u){u.autoActionMinSeconds=2;u.autoActionMaxSeconds=2;},
            [](auto& u){u.autoActionMaxSeconds=4;},[](auto& u){u.automaticSideWeights[0]=.3f;},
            [](auto& u){u.automaticSideWeights[1]=.6f;},[](auto& u){u.staminaEnabled=false;},
            [](auto& u){u.movingPerSecond=23;},[](auto& u){u.hangingPerSecond=7;},[](auto& u){u.requiredToGrab=26;},
            [](auto& u){u.audioEnabled=false;},[](auto& u){u.audioVolume=.3f;},[](auto& u){u.gamepad.enabled=false;},
            [](auto& u){u.gamepad.deadzone=.36f;},[](auto& u){u.gamepad.triggerThreshold=.68f;},
            [](auto& u){u.diagnostics=true;},[](auto& u){u.language="chinese";},
            [](auto& u){u.language="english";},[](auto& u){u.language="chinese";}
        }};
        for(const auto& change:changes) {
            const auto before=writes;change(draft);
            check(edits.apply(draft,submit)==SettingsEditResult::queued&&writes==before+1,"each accepted visible setting applies and saves automatically once");saved();
            for(unsigned i=0;i<5;++i)check(edits.apply(draft,submit)==SettingsEditResult::unchanged,"idle frames after a change do not repeat its save");
        }
        draft.audioVolume=.9f;draft.audioEnabled=true;const auto beforeBatch=writes;
        check(edits.apply(draft,submit)==SettingsEditResult::queued&&writes==beforeBatch+1,"same-render changes are coalesced into one full commit");saved();
        draft.audioVolume=.8f;draft.audioVolume=.9f;
        check(edits.apply(draft,submit)==SettingsEditResult::unchanged,"no net change does not rewrite disk");
        std::string error;BindingCapture capture;capture.begin(BindingCaptureDevice::keyboard);capture.sample(keys());
        const auto prior=userSettingsIni(draft);const auto beforeCapture=writes;
        for(const auto snapshot:{keys({0x2a}),keys({0x2a,0x11}),keys({0x11})}) {
            const auto result=capture.sample(snapshot);
            check(!applyCompletedBinding(draft,result,4,error)&&userSettingsIni(draft)==prior,"partial keyboard capture cannot mutate or save a partial chord");
            check(edits.apply(draft,submit)==SettingsEditResult::unchanged&&writes==beforeCapture,"recording preview never reaches the runtime or disk");
        }
        check(applyCompletedBinding(draft,capture.sample(keys()),4,error),"full released keyboard capture commits after validation");
        check(serializeKeyChord(draft.bindings.entry)=="LShift+W","completed chord keeps exact physical modifier identity");
        check(edits.apply(draft,submit)==SettingsEditResult::queued&&writes==beforeCapture+1,"completed keyboard capture saves without a Save button");saved();
        BindingCaptureResult conflict;conflict.status=BindingCaptureStatus::captured;conflict.device=BindingCaptureDevice::keyboard;conflict.keyboard=draft.bindings.backward;
        const auto retained=userSettingsIni(draft);const auto beforeConflict=writes;
        check(!applyCompletedBinding(draft,conflict,0,error)&&!error.empty()&&userSettingsIni(draft)==retained,"conflicting captured chord retains the previous complete binding");
        check(edits.apply(draft,submit)==SettingsEditResult::unchanged&&writes==beforeConflict,"invalid captured binding never queues or saves");
        capture.begin(BindingCaptureDevice::gamepad);capture.sample(buttons());capture.sample(buttons(*parseGamepadChord("RB+X")));
        check(!applyCompletedBinding(draft,capture.result(),0,error),"held gamepad preview is not a completed binding");
        check(applyCompletedBinding(draft,capture.sample(buttons()),0,error),"released gamepad chord is accepted");
        check(edits.apply(draft,submit)==SettingsEditResult::queued,"gamepad completed capture applies and saves automatically");saved();
        conflict.device=BindingCaptureDevice::gamepad;conflict.gamepad=draft.gamepad.bindings.hop;
        const auto gamepadPrior=draft.gamepad.bindings;
        check(!applyCompletedBinding(draft,conflict,3,error)&&draft.gamepad.bindings==gamepadPrior,"gamepad hop/drop conflict leaves old binding intact");
        conflict.status=BindingCaptureStatus::cancelled;
        check(!applyCompletedBinding(draft,conflict,0,error)&&draft.gamepad.bindings==gamepadPrior,"cancelled capture never commits its old candidate");
        conflict.status=BindingCaptureStatus::captured;
        check(!applyCompletedBinding(draft,conflict,99,error),"out-of-range capture target is rejected");
        auto invalid=draft;invalid.bindings.forward=invalid.bindings.backward;
        check(edits.apply(invalid,submit)==SettingsEditResult::invalid,"automatic commit does not sanitize conflicting bindings into unexpected defaults");
        for(const auto page:{SettingsPage::general,SettingsPage::movement,SettingsPage::automatic,SettingsPage::stamina,SettingsPage::audio,SettingsPage::keys,SettingsPage::diagnostics}) {
            const auto expected=restoreSettingsPage(page,draft);const auto before=writes;draft=expected;
            check(edits.apply(draft,submit,true)==SettingsEditResult::queued&&writes==before+1,"Restore Defaults automatically applies and writes only the selected page");
            check(userSettingsIni(draft)==userSettingsIni(expected)&&draft.language=="chinese","restoring a page preserves all other settings and selected language");saved();
            check(edits.apply(draft,submit)==SettingsEditResult::unchanged,"render following page restore does not save twice");
        }
        check(edits.apply(draft,submit,true)==SettingsEditResult::queued,"explicit Restore Defaults can retry writing unchanged values");
        std::ifstream file(path);const std::string content{std::istreambuf_iterator<char>(file),{}};
        check(content.find("Keep=unchanged")!=std::string::npos,"automatic saving preserves unrelated INI entries");
        std::cout<<checks<<" settings automatic apply/save checks passed\n";return 0;
    } catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
