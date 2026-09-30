#include "SettingsMenu.h"
#include "TranslationDefaults.h"
#include "RuntimeLog.h"
#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <cstring>
#include <utility>
#include <RE/Skyrim.h>
#include "vendor/SKSEMenuFramework/SKSEMenuFramework.h"

namespace fc {
namespace {
namespace ui=ImGuiMCP;
SettingsMenuCallbacks callbacks;
UserSettings draft;
SettingsPage activePage=SettingsPage::general;
bool initialized=false,registered=false,dirty=false;
std::string notice,detail;
std::array<std::array<char,96>,7> bindingText{};
constexpr std::array<KeyChord InputBindings::*,7> bindingMembers{
    &InputBindings::forward,&InputBindings::backward,&InputBindings::left,&InputBindings::right,
    &InputBindings::entry,&InputBindings::runModifier,&InputBindings::hop
};
TranslationCatalog translations{std::span<const TranslationEntry>{translationDefaults}};
const char* tr(std::string_view key){return translations.text(draft.language,key);}
std::string label(std::string_view key,const char* id){return std::string(tr(key))+"###"+id;}
void text(std::string_view key){ui::TextWrapped("%s",tr(key));}
bool check(std::string_view key,const char* id,bool& value){const bool changed=ui::Checkbox(label(key,id).c_str(),&value);dirty|=changed;return changed;}
bool slider(std::string_view key,const char* id,float& value,float low,float high,const char* format="%.2f") {
    const bool changed=ui::SliderFloat(label(key,id).c_str(),&value,low,high,format);dirty|=changed;return changed;
}
bool button(std::string_view key,const char* id){return ui::Button(label(key,id).c_str());}
void reloadTranslations() {
    try {
        const auto directory=runtimeDataDirectory()/L"Interface"/L"Translations";
        const bool loaded=translations.reload(directory);
        const auto encoded=directory.u8string();
        std::string available;
        for(const auto& language:translations.languages()) {
            if(!available.empty())available+=", ";
            available+=language.id;
        }
        SKSE::log::info("Menu translations: directory={}, loaded={}, available={}",
            std::string(encoded.begin(),encoded.end()),loaded,available);
        for(const auto& warning:translations.warnings())SKSE::log::warn("Menu translation: {}",warning);
    } catch(const std::exception& error) {
        SKSE::log::warn("Menu translation loading failed: {}",error.what());
    }
}
void syncBindings() {
    for(std::size_t i=0;i<bindingMembers.size();++i) {
        const auto value=serializeKeyChord(draft.bindings.*bindingMembers[i]);
        bindingText[i].fill(0);std::copy_n(value.data(),std::min(value.size(),bindingText[i].size()-1),bindingText[i].data());
    }
}
bool acceptBindings() {
    InputBindings value=draft.bindings;
    for(std::size_t i=0;i<bindingMembers.size();++i) {
        auto parsed=parseKeyChord(bindingText[i].data());
        if(!parsed){notice="binding-invalid";detail=bindingText[i].data();return false;}
        value.*bindingMembers[i]=*parsed;
    }
    if(auto validation=validateBindings(value);!validation.valid){notice="binding-conflict";detail=validation.message;return false;}
    draft.bindings=value;return true;
}
void saveSettings() {
    draft=sanitizeUserSettings(draft);syncBindings();callbacks.requestSave(draft);notice="save-queued";detail.clear();dirty=false;
}
void restoreDefaults() {
    const auto saved=restoreSettingsPage(activePage,callbacks.getSettings());
    draft=restoreSettingsPage(activePage,draft);
    if(activePage==SettingsPage::keys)syncBindings();
    callbacks.requestSave(saved);notice="save-queued";detail.clear();
    dirty=userSettingsIni(draft)!=userSettingsIni(saved);
    for(std::size_t i=0;i<bindingMembers.size();++i)
        dirty|=bindingText[i].data()!=serializeKeyChord(draft.bindings.*bindingMembers[i]);
}
const char* translatedNotice() {
    if(notice=="binding-invalid")return tr("$FC_NOTICE_BINDING_INVALID");
    if(notice=="binding-conflict")return tr("$FC_NOTICE_BINDING_CONFLICT");
    if(notice=="save-queued")return tr("$FC_NOTICE_SAVE_QUEUED");
    if(notice=="reload-queued")return tr("$FC_NOTICE_RELOAD_QUEUED");
    return "";
}
const char* runtimeStatus(std::string_view value) {
    if(value=="ready")return tr("$FC_STATUS_READY");
    if(value=="pending")return tr("$FC_STATUS_PENDING");
    if(value=="reloaded")return tr("$FC_STATUS_RELOADED");
    if(value=="reload_failed")return tr("$FC_STATUS_RELOAD_FAILED");
    if(value=="save_failed")return tr("$FC_STATUS_SAVE_FAILED");
    if(value=="pack_unavailable")return tr("$FC_STATUS_PACK_UNAVAILABLE");
    if(value=="not_ready")return tr("$FC_STATUS_NOT_READY");
    if(value=="applied")return tr("$FC_STATUS_APPLIED");
    return tr("$FC_STATUS_DETAILS");
}
const char* majorReason(const MenuAnimationSlot& slot) {
    if(slot.status==1)return tr("$FC_SLOT_VALIDATED");
    if(slot.status==0)return tr("$FC_SLOT_MISSING");
    const auto& value=slot.reason;
    if(value.find("skeleton")!=value.npos||value.find("bone")!=value.npos||value.find("track")!=value.npos||value.find("rig")!=value.npos)
        return tr("$FC_SLOT_SKELETON_ERROR");
    if(value.find("limit")!=value.npos||value.find("budget")!=value.npos||value.find("large")!=value.npos||value.find("bounds")!=value.npos)
        return tr("$FC_SLOT_LIMIT_ERROR");
    if(value.find("open")!=value.npos||value.find("read")!=value.npos||value.find("inspect")!=value.npos||value.find("missing")!=value.npos)
        return tr("$FC_SLOT_READ_ERROR");
    if(value.find("JSON")!=value.npos||value.find("json")!=value.npos||value.find("profile")!=value.npos||value.find("pack")!=value.npos)
        return tr("$FC_SLOT_PROFILE_ERROR");
    if(value.find("finite")!=value.npos||value.find("rotation")!=value.npos||value.find("quaternion")!=value.npos)
        return tr("$FC_SLOT_POSE_ERROR");
    return tr("$FC_SLOT_INVALID");
}
const char* slotTitle(std::string_view name) {
    struct Name {const char* name;const char* key;};
    static constexpr Name names[]{
        {"hang","$FC_SLOT_HANG"},{"up","$FC_SLOT_UP"},{"down","$FC_SLOT_DOWN"},
        {"left","$FC_SLOT_LEFT"},{"right","$FC_SLOT_RIGHT"},
        {"reach","$FC_SLOT_REACH"},{"hopLeft","$FC_SLOT_HOP_LEFT"},
        {"hopRight","$FC_SLOT_HOP_RIGHT"},{"hopUp","$FC_SLOT_HOP_UP"},
        {"drop","$FC_SLOT_DROP"},
        {"jumpCatch","$FC_SLOT_JUMP_CATCH"},{"sprintCatch","$FC_SLOT_SPRINT_CATCH"},{"dropBack","$FC_SLOT_DROP_BACK"},
        {"ledgeCatch","$FC_SLOT_LEDGE_CATCH"},{"runUp","$FC_SLOT_RUN_UP"},{"runLeft","$FC_SLOT_RUN_LEFT"},
        {"runRight","$FC_SLOT_RUN_RIGHT"},{"runDiagonalLeft","$FC_SLOT_RUN_DIAGONAL_LEFT"},
        {"runDiagonalRight","$FC_SLOT_RUN_DIAGONAL_RIGHT"},{"runLaunch","$FC_SLOT_RUN_LAUNCH"},
        {"runCatch","$FC_SLOT_RUN_CATCH"},{"kickUp","$FC_SLOT_KICK_UP"},
        {"kickLeft","$FC_SLOT_KICK_LEFT"},{"kickRight","$FC_SLOT_KICK_RIGHT"},
        {"flipUp","$FC_SLOT_FLIP_UP"},{"flipLeft","$FC_SLOT_FLIP_LEFT"},
        {"flipRight","$FC_SLOT_FLIP_RIGHT"},
        {"runLaunchLeft","$FC_SLOT_RUN_LAUNCH_LEFT"},{"runLaunchRight","$FC_SLOT_RUN_LAUNCH_RIGHT"},
        {"sideBrace","$FC_SLOT_SIDE_BRACE"},{"backFlipOut","$FC_SLOT_BACK_FLIP_OUT"},
        {"contextHang","$FC_SLOT_CONTEXT_HANG"},{"contextHopLeft","$FC_SLOT_CONTEXT_HOP_LEFT"},
        {"contextHopRight","$FC_SLOT_CONTEXT_HOP_RIGHT"},{"contextMantle","$FC_SLOT_CONTEXT_MANTLE"}
    };
    for(const auto& item:names)if(name==item.name)return tr(item.key);
    return tr("$FC_SLOT_UNKNOWN");
}
void basic() {
    check("$FC_ENABLED","enabled",draft.enabled);
    check("$FC_NOTIFICATIONS","notifications",draft.notifications);
    check("$FC_LOW_STAMINA_NOTIFICATIONS","lowStamina",draft.lowStaminaNotifications);
    check("$FC_AUTO_MANTLE","autoMantle",draft.autoMantle);
    text("$FC_ENTRY_HELP");
}
void movement() {
    slider("$FC_UP_SPEED","upSpeed",draft.upSpeed,10,140,"%.0f");
    slider("$FC_DOWN_SPEED","downSpeed",draft.downSpeed,10,140,"%.0f");
    slider("$FC_SIDE_SPEED","sideSpeed",draft.sideSpeed,10,120,"%.0f");
    slider("$FC_WALL_RUN_SPEED","runSpeed",draft.wallRunSpeed,10,450,"%.1f");
    slider("$FC_DIAGONAL_MULTIPLIER","diagonal",draft.diagonalRunMultiplier,1,1.3f,"%.2fx");
    text("$FC_MOVEMENT_HELP");
}
void automaticActions() {
    check("$FC_AUTO_SIDE_ACTIONS","autoActions",draft.automaticClimbActions);
    check("$FC_WALL_RUN_OBSTACLES","obstacleJumps",draft.wallRunObstacleJumps);
    check("$FC_CONTEXT_MANTLE","contextMantle",draft.contextualMantleEnabled);
    slider("$FC_ATTEMPT_INTERVAL_MIN","intervalMin",draft.autoActionMinSeconds,.65f,20);
    draft.autoActionMaxSeconds=std::max(draft.autoActionMinSeconds,draft.autoActionMaxSeconds);
    slider("$FC_ATTEMPT_INTERVAL_MAX","intervalMax",draft.autoActionMaxSeconds,draft.autoActionMinSeconds,30);
    slider("$FC_LEFT_OPPORTUNITY","leftWeight",draft.automaticSideWeights[0],0,1,"%.2f");
    slider("$FC_RIGHT_OPPORTUNITY","rightWeight",draft.automaticSideWeights[1],0,1,"%.2f");
    text("$FC_OPPORTUNITY_HELP");
    text("$FC_ATTEMPT_HELP");
}
void stamina() {
    check("$FC_STAMINA_ENABLED","staminaEnabled",draft.staminaEnabled);
    text("$FC_STAMINA_HELP");
    ui::BeginDisabled(!draft.staminaEnabled);
    slider("$FC_CLIMB_STAMINA","movingDrain",draft.movingPerSecond,0,50,"%.1f");
    ui::Text("%s: %.1f",tr("$FC_WALL_RUN_STAMINA"),draft.movingPerSecond*2);
    slider("$FC_REST_STAMINA","hangDrain",draft.hangingPerSecond,0,30,"%.1f");
    slider("$FC_GRAB_STAMINA","grabStamina",draft.requiredToGrab,0,100,"%.0f");
    ui::EndDisabled();
}
void audio() {
    bool changed=check("$FC_AUDIO_ENABLED","audioEnabled",draft.audioEnabled);
    changed|=slider("$FC_AUDIO_VOLUME","audioVolume",draft.audioVolume,0,1,"%.2f");
    if(changed&&callbacks.requestAudio)callbacks.requestAudio(draft.audioEnabled,draft.audioVolume);
    text("$FC_AUDIO_HELP");
}
void bindings() {
    const char* keys[]{"$FC_BIND_FORWARD","$FC_BIND_BACKWARD","$FC_BIND_LEFT","$FC_BIND_RIGHT","$FC_BIND_ENTRY","$FC_BIND_RUN","$FC_BIND_HOP"};
    const char* ids[]{"bindForward","bindBackward","bindLeft","bindRight","bindEntry","bindRun","bindHop"};
    for(std::size_t i=0;i<bindingText.size();++i)dirty|=ui::InputText(label(keys[i],ids[i]).c_str(),bindingText[i].data(),bindingText[i].size());
    text("$FC_BINDING_HELP");
    text("$FC_BINDING_DERIVED_HELP");
    if(button("$FC_VALIDATE_BINDINGS","validateBindings")) {
        if(acceptBindings()){notice.clear();detail.clear();syncBindings();}
    }
}
void diagnostics(const SettingsMenuSnapshot& snapshot) {
    check("$FC_DIAGNOSTICS_ENABLED","diagnostics",draft.diagnostics);
    text("$FC_DIAGNOSTICS_HELP");
    ui::Text("%s: %s",tr("$FC_RUNTIME"),snapshot.ready?tr("$FC_STATUS_READY"):tr("$FC_STATUS_UNAVAILABLE"));
    ui::Text("%s: %s",tr("$FC_AUDIO_ASSETS"),snapshot.audioReady?tr("$FC_STATUS_READY"):tr("$FC_STATUS_UNAVAILABLE"));
    if(snapshot.movementPending)text("$FC_MOVEMENT_PENDING");
    if(snapshot.reloadPending)text("$FC_RELOAD_PENDING");
    if(!snapshot.packName.empty())ui::Text("%s: %s",tr("$FC_ANIMATION_PACK"),snapshot.packName.c_str());
    ui::Text("%s: %llu / %llu / %llu",tr("$FC_AUTOMATIC_STATS"),
        static_cast<unsigned long long>(snapshot.automaticAttempts),static_cast<unsigned long long>(snapshot.automaticActions),static_cast<unsigned long long>(snapshot.wallRunObstacleJumps));
    ui::BeginDisabled(snapshot.reloadPending);
    if(button("$FC_RELOAD_ANIMATIONS","reloadAnimations")&&callbacks.requestReloadAnimations) {
        callbacks.requestReloadAnimations();notice="reload-queued";detail.clear();
    }
    ui::EndDisabled();
    text("$FC_RELOAD_HELP");
    text("$FC_SLOT_REPORT_HELP");
    for(const auto& slot:snapshot.slots) {
        const auto title=std::string(slotTitle(slot.name))+" ("+slot.name+")###slot_"+slot.name;
        if(!ui::CollapsingHeader(title.c_str()))continue;
        ui::TextWrapped("%s: %s",tr("$FC_STATUS"),majorReason(slot));
        ui::TextWrapped("%s: %s",tr("$FC_FILE"),slot.file.c_str());
        if(snapshot.diagnosticsEnabled)
            ui::Text("%s: %llu   %s: %llu",tr("$FC_SELECTED"),static_cast<unsigned long long>(slot.triggers),tr("$FC_OBSERVED_OUTPUT"),static_cast<unsigned long long>(slot.observed));
        else {
            ui::Text("%s: %llu   %s: %s",tr("$FC_SELECTED"),static_cast<unsigned long long>(slot.triggers),tr("$FC_OBSERVED_OUTPUT"),tr("$FC_SAMPLING_OFF"));
            if(slot.observed)ui::Text("%s: %llu",tr("$FC_PREVIOUS_OBSERVED"),static_cast<unsigned long long>(slot.observed));
        }
        ui::Text("%s: %zu   %s: %.3f",tr("$FC_SAMPLES"),slot.samples,tr("$FC_SECONDS"),slot.seconds);
        if(!slot.reason.empty())ui::TextWrapped("%s: %s",tr("$FC_TECHNICAL_DETAILS"),slot.reason.c_str());
    }
    if(snapshot.slots.empty())text("$FC_NO_ANIMATION_REPORT");
    text("$FC_STATS_HELP");
    text("$FC_OBSERVED_HELP");
}
void languageControls() {
    std::vector<std::string> languageIds,languageNames;
    int selected=-1;
    for(const auto& language:translations.languages()) {
        if(language.id==draft.language)selected=static_cast<int>(languageIds.size());
        languageIds.push_back(language.id);languageNames.push_back(language.name);
    }
    const bool unavailable=selected<0;
    if(unavailable) {
        selected=static_cast<int>(languageIds.size());
        languageIds.push_back(draft.language);languageNames.push_back(draft.language);
    }
    std::vector<const char*> names;names.reserve(languageNames.size());
    for(const auto& name:languageNames)names.push_back(name.c_str());
    if(ui::Combo(label("$FC_LANGUAGE","language").c_str(),&selected,names.data(),static_cast<int>(names.size()))) {
        draft.language=languageIds[static_cast<std::size_t>(selected)];reloadTranslations();dirty=true;
    }
}
void __stdcall render() {
    if(!initialized){draft=callbacks.getSettings();syncBindings();reloadTranslations();initialized=true;}
    languageControls();
    const auto snapshot=callbacks.snapshot?callbacks.snapshot():SettingsMenuSnapshot{};
    if(ui::BeginTabBar("FreeClimbSettingsTabs")) {
        if(ui::BeginTabItem(label("$FC_TAB_GENERAL","generalTab").c_str())){activePage=SettingsPage::general;basic();ui::EndTabItem();}
        if(ui::BeginTabItem(label("$FC_TAB_MOVEMENT","movementTab").c_str())){activePage=SettingsPage::movement;movement();ui::EndTabItem();}
        if(ui::BeginTabItem(label("$FC_TAB_AUTOMATIC","actionsTab").c_str())){activePage=SettingsPage::automatic;automaticActions();ui::EndTabItem();}
        if(ui::BeginTabItem(label("$FC_TAB_STAMINA","staminaTab").c_str())){activePage=SettingsPage::stamina;stamina();ui::EndTabItem();}
        if(ui::BeginTabItem(label("$FC_TAB_AUDIO","audioTab").c_str())){activePage=SettingsPage::audio;audio();ui::EndTabItem();}
        if(ui::BeginTabItem(label("$FC_TAB_KEYS","keysTab").c_str())){activePage=SettingsPage::keys;bindings();ui::EndTabItem();}
        if(ui::BeginTabItem(label("$FC_TAB_DIAGNOSTICS","diagnosticsTab").c_str())){activePage=SettingsPage::diagnostics;diagnostics(snapshot);ui::EndTabItem();}
        ui::EndTabBar();
    }
    ui::Separator();
    if(button("$FC_SAVE_SETTINGS","saveSettings")&&acceptBindings())saveSettings();
    ui::SameLine();
    if(button("$FC_RESTORE_DEFAULTS","restoreDefaults"))restoreDefaults();
    if(dirty)text("$FC_UNSAVED_CHANGES");
    if(!notice.empty())ui::TextWrapped("%s",translatedNotice());
    if(!detail.empty())ui::TextWrapped("%s: %s",tr("$FC_DETAILS"),detail.c_str());
    if(!snapshot.error.empty()) {
        text("$FC_RUNTIME_ERROR_HELP");
        ui::TextWrapped("%s",snapshot.error.c_str());
    }
    if(!snapshot.status.empty())ui::TextWrapped("%s: %s",tr("$FC_RUNTIME_STATUS"),runtimeStatus(snapshot.status));
}
}
bool registerSettingsMenu(SettingsMenuCallbacks value) {
    if(registered)return true;
    const auto module=GetMenuFrameworkModule();
    if(!module||!value.getSettings||!value.requestSave)return false;
    constexpr const char* required[]{"AddSectionItem","igTextWrappedV","igTextV","igCheckbox","igSliderFloat","igButton","igSameLine",
        "igCombo_Str_arr","igInputText","igBeginDisabled","igEndDisabled","igCollapsingHeader_TreeNodeFlags",
        "igBeginTabBar","igEndTabBar","igBeginTabItem","igEndTabItem","igSeparator",
        "IsAnyBlockingWindowOpened"};
    for(const auto name:required)if(!GetProcAddress(module,name))return false;
    callbacks=std::move(value);SKSEMenuFramework::SetSection("FreeClimb");
    SKSEMenuFramework::AddSectionItem("Settings",render);registered=true;return true;
}
bool settingsMenuBlocking() {return registered&&SKSEMenuFramework::IsAnyBlockingWindowOpened();}
}
