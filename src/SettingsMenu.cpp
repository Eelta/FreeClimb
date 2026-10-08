#include "SettingsMenu.h"
#include "SettingsMenuEdits.h"
#include "SettingsMenuHelp.h"
#include "TranslationDefaults.h"
#include "RuntimeLog.h"
#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <chrono>
#include <mutex>
#include <utility>
#include <RE/Skyrim.h>
#include "vendor/SKSEMenuFramework/SKSEMenuFramework.h"

namespace fc {
namespace {
namespace ui=ImGuiMCP;
SettingsMenuCallbacks callbacks;
UserSettings draft;
SettingsMenuEdits edits;
SettingsMenuHelp help;
SettingsPage activePage=SettingsPage::general;
bool initialized=false,registered=false,dirty=false;
std::string notice,detail;
struct CaptureSession {
    std::mutex mutex;
    BindingCapture capture;
    BindingCaptureSnapshot input;
    BindingCaptureOwnership ownership;
    std::atomic<bool> filtering{},cancelRequested{},observing{};
    std::atomic<float> triggerThreshold{.5f};
    std::size_t index{};
    bool active{},rendered{},uiLocked{},unavailable{};
    std::chrono::steady_clock::time_point started;
} recording;
bool captureNeutral() {
    return std::none_of(recording.input.keyboard.down.begin(),recording.input.keyboard.down.end(),[](bool down){return down;})&&
        !recording.input.gamepad&&!(GetAsyncKeyState(VK_LBUTTON)&0x8000)&&!(GetAsyncKeyState(VK_RBUTTON)&0x8000);
}
void captureFilterState() {
    recording.filtering.store(recording.active||recording.ownership.pending());
}
void advanceCapture() {
    if(!recording.active)return;
    recording.input.activationHeld=!captureNeutral();
    recording.capture.sample(recording.input);
}
void captureUI(bool lock) {
    if(recording.uiLocked==lock)return;
    auto* io=ui::GetIO();
    ui::ImGuiIOManager::ClearEventsQueue(io);
    ui::ImGuiIOManager::ClearInputKeys(io);
    ui::ImGuiIOManager::SetAppAcceptingEvents(io,!lock);
    recording.uiLocked=lock;
}
void stopCapture() {
    std::scoped_lock lock(recording.mutex);
    recording.capture.cancel();recording.active=false;captureFilterState();
}
void cancelCapture() {stopCapture();captureUI(false);}
void __stdcall captureEvent(SKSEMenuFramework::Model::EventType event) {
    if(event==SKSEMenuFramework::Model::kBeforeRender) {
        recording.rendered=false;
        DWORD process=0;GetWindowThreadProcessId(GetForegroundWindow(),&process);
        if(recording.cancelRequested.exchange(false)||process!=GetCurrentProcessId())cancelCapture();
    } else if(event==SKSEMenuFramework::Model::kCloseMenu) {
        recording.observing.store(false);stopCapture();recording.cancelRequested.store(true);
    } else if(event==SKSEMenuFramework::Model::kAfterRender&&!recording.rendered) {
        recording.observing.store(false);cancelCapture();
    }
}
TranslationCatalog translations{std::span<const TranslationEntry>{translationDefaults}};
const char* tr(std::string_view key){return translations.text(draft.language,key);}
std::string label(std::string_view key,const char* id){return std::string(tr(key))+"###"+id;}
void itemHelp(std::string_view key,std::string_view description={}) {
    help.observe(key,description.empty()?settingsHelpFor(key):description,
        ui::IsItemHovered(ui::ImGuiHoveredFlags_AllowWhenDisabled|ui::ImGuiHoveredFlags_NoNavOverride),ui::IsItemFocused());
}
void text(std::string_view key){ui::TextWrapped("%s",tr(key));itemHelp(key);}
bool check(std::string_view key,const char* id,bool& value){const bool changed=ui::Checkbox(label(key,id).c_str(),&value);itemHelp(key);dirty|=changed;return changed;}
bool slider(std::string_view key,const char* id,float& value,float low,float high,const char* format="%.2f") {
    const bool changed=ui::SliderFloat(label(key,id).c_str(),&value,low,high,format);itemHelp(key);dirty|=changed;return changed;
}
bool button(std::string_view key,const char* id){const bool clicked=ui::Button(label(key,id).c_str());itemHelp(key);return clicked;}
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
bool acceptBindings() {
    for(const auto& validation:{validateBindings(draft.bindings),validateGamepadBindings(draft.gamepad.bindings)})
        if(!validation.valid){notice="binding-conflict";detail=validation.message;return false;}
    return true;
}
void applyEdits(bool force=false) {
    if(!dirty&&!force)return;
    if(!acceptBindings())return;
    const auto result=edits.apply(draft,callbacks.requestSave,force);dirty=false;
    if(result==SettingsEditResult::queued){notice="settings-queued";detail.clear();}
}
void restoreDefaults() {
    draft=restoreSettingsPage(activePage,draft);applyEdits(true);
}
void beginCapture(BindingCaptureDevice device,std::size_t index) {
    {
        std::scoped_lock lock(recording.mutex);
        recording.index=index;recording.unavailable=false;recording.input={};
        recording.capture.begin(device);recording.started=std::chrono::steady_clock::now();
        recording.active=true;captureFilterState();
    }
    notice.clear();detail.clear();captureUI(true);
}
void finishCapture() {
    BindingCaptureResult result;
    std::size_t index{};bool finished=false,unavailable=false;
    {
        std::scoped_lock lock(recording.mutex);
        if(recording.active) {
            if(std::chrono::steady_clock::now()-recording.started>std::chrono::seconds(20))recording.capture.cancel();
            if(!recording.capture.active()&&captureNeutral()) {
                result=recording.capture.result();index=recording.index;unavailable=recording.unavailable;
                recording.active=false;captureFilterState();finished=true;
            }
        }
    }
    if(!finished)return;
    captureUI(false);
    if(result.status==BindingCaptureStatus::captured) {
        if(applyCompletedBinding(draft,result,index,detail)){dirty=true;notice.clear();detail.clear();}
        else notice="binding-conflict";
    } else notice=unavailable?"$FC_CAPTURE_UNAVAILABLE":
        result.status==BindingCaptureStatus::error?"$FC_CAPTURE_INVALID":"$FC_CAPTURE_TIMEOUT";
}
void bindingButton(BindingCaptureDevice device,std::size_t index,const char* key,const char* id) {
    const auto value=device==BindingCaptureDevice::keyboard?serializeKeyChord(draft.bindings.*bindingFields[index].member):
        serializeGamepadChord(draft.gamepad.bindings.*gamepadBindingFields[index].member);
    if(ui::Button((value+"###"+id).c_str()))beginCapture(device,index);
    const auto description=device==BindingCaptureDevice::keyboard?"$FC_BINDING_HELP":"$FC_GAMEPAD_BINDING_HELP";
    itemHelp(key,description);ui::SameLine();text(key);itemHelp(key,description);
}
void capturePrompt() {
    BindingCaptureResult result;bool active=false;
    {
        std::scoped_lock lock(recording.mutex);
        active=recording.active;result=recording.capture.result();
    }
    if(!active)return;
    const auto position=ui::GetWindowPos(),size=ui::GetWindowSize();
    ui::SetNextWindowPos({position.x+size.x*.5f,position.y+size.y*.5f},ui::ImGuiCond_Always,{.5f,.5f});
    ui::SetNextWindowSize({std::clamp(size.x*.8f,280.f,680.f),0},ui::ImGuiCond_Always);
    if(ui::Begin(label("$FC_CAPTURE_TITLE","FreeClimbBindingCapture").c_str(),nullptr,
        ui::ImGuiWindowFlags_AlwaysAutoResize|ui::ImGuiWindowFlags_NoCollapse|ui::ImGuiWindowFlags_NoSavedSettings|
        ui::ImGuiWindowFlags_NoMove|ui::ImGuiWindowFlags_NoNav)) {
        text(result.device==BindingCaptureDevice::keyboard?"$FC_KEYBOARD_SECTION":"$FC_GAMEPAD_SECTION");
        text(result.status==BindingCaptureStatus::armed?"$FC_CAPTURE_READY":"$FC_CAPTURE_WAITING");
        if(!result.preview.empty())ui::Text("%s",result.preview.c_str());
        text("$FC_CAPTURE_HELP");
    }
    ui::End();
}
const char* translatedNotice() {
    if(notice=="binding-invalid")return tr("$FC_NOTICE_BINDING_INVALID");
    if(notice=="gamepad-binding-invalid")return tr("$FC_NOTICE_GAMEPAD_BINDING_INVALID");
    if(notice=="binding-conflict")return tr("$FC_NOTICE_BINDING_CONFLICT");
    if(notice=="settings-queued")return tr("$FC_NOTICE_SETTINGS_QUEUED");
    if(notice=="reload-queued")return tr("$FC_NOTICE_RELOAD_QUEUED");
    return notice.starts_with("$FC_")?tr(notice):"";
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
        {"jumpCatch","$FC_SLOT_JUMP_CATCH"},{"dropBack","$FC_SLOT_DROP_BACK"},
        {"ledgeCatch","$FC_SLOT_LEDGE_CATCH"},{"runUp","$FC_SLOT_RUN_UP"},{"runLeft","$FC_SLOT_RUN_LEFT"},
        {"runRight","$FC_SLOT_RUN_RIGHT"},{"runDiagonalLeft","$FC_SLOT_RUN_DIAGONAL_LEFT"},
        {"runDiagonalRight","$FC_SLOT_RUN_DIAGONAL_RIGHT"},{"runLaunch","$FC_SLOT_RUN_LAUNCH"},
        {"runCatch","$FC_SLOT_RUN_CATCH"},{"kickUp","$FC_SLOT_KICK_UP"},
        {"kickLeft","$FC_SLOT_KICK_LEFT"},{"kickRight","$FC_SLOT_KICK_RIGHT"},
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
}
void movement() {
    slider("$FC_UP_SPEED","upSpeed",draft.upSpeed,10,140,"%.0f");
    slider("$FC_DOWN_SPEED","downSpeed",draft.downSpeed,10,140,"%.0f");
    slider("$FC_SIDE_SPEED","sideSpeed",draft.sideSpeed,10,120,"%.0f");
    check("$FC_WALL_RUN_ENABLED","wallRunEnabled",draft.wallRunEnabled);
    ui::BeginDisabled(!draft.wallRunEnabled);
    slider("$FC_WALL_RUN_SPEED","runSpeed",draft.wallRunSpeed,10,450,"%.1f");
    slider("$FC_DIAGONAL_MULTIPLIER","diagonal",draft.diagonalRunMultiplier,1,1.3f,"%.2fx");
    ui::EndDisabled();
}
void automaticActions() {
    check("$FC_AUTO_SIDE_ACTIONS","autoActions",draft.automaticClimbActions);
    check("$FC_WALL_RUN_OBSTACLES","obstacleJumps",draft.wallRunObstacleJumps);
    check("$FC_CONTEXT_MANTLE","contextMantle",draft.contextualMantleEnabled);
    ui::BeginDisabled(!draft.automaticClimbActions);
    slider("$FC_ATTEMPT_INTERVAL_MIN","intervalMin",draft.autoActionMinSeconds,.65f,20);
    draft.autoActionMaxSeconds=std::max(draft.autoActionMinSeconds,draft.autoActionMaxSeconds);
    slider("$FC_ATTEMPT_INTERVAL_MAX","intervalMax",draft.autoActionMaxSeconds,draft.autoActionMinSeconds,30);
    slider("$FC_LEFT_OPPORTUNITY","leftWeight",draft.automaticSideWeights[0],0,1,"%.2f");
    slider("$FC_RIGHT_OPPORTUNITY","rightWeight",draft.automaticSideWeights[1],0,1,"%.2f");
    ui::EndDisabled();
}
void stamina() {
    check("$FC_STAMINA_ENABLED","staminaEnabled",draft.staminaEnabled);
    ui::BeginDisabled(!draft.staminaEnabled);
    slider("$FC_CLIMB_STAMINA","movingDrain",draft.movingPerSecond,0,50,"%.1f");
    ui::Text("%s: %.1f",tr("$FC_WALL_RUN_STAMINA"),draft.movingPerSecond*2);
    itemHelp("$FC_WALL_RUN_STAMINA");
    slider("$FC_REST_STAMINA","hangDrain",draft.hangingPerSecond,0,30,"%.1f");
    slider("$FC_GRAB_STAMINA","grabStamina",draft.requiredToGrab,0,100,"%.0f");
    ui::EndDisabled();
}
void audio() {
    check("$FC_AUDIO_ENABLED","audioEnabled",draft.audioEnabled);
    slider("$FC_AUDIO_VOLUME","audioVolume",draft.audioVolume,0,1,"%.2f");
}
void bindings() {
    text("$FC_KEYBOARD_SECTION");
    const char* keys[]{"$FC_BIND_FORWARD","$FC_BIND_BACKWARD","$FC_BIND_LEFT","$FC_BIND_RIGHT","$FC_BIND_ENTRY","$FC_BIND_RUN","$FC_BIND_HOP"};
    const char* ids[]{"bindForward","bindBackward","bindLeft","bindRight","bindEntry","bindRun","bindHop"};
    for(std::size_t i=0;i<bindingFields.size();++i)bindingButton(BindingCaptureDevice::keyboard,i,keys[i],ids[i]);
    ui::Separator();
    text("$FC_GAMEPAD_SECTION");
    check("$FC_GAMEPAD_ENABLED","gamepadEnabled",draft.gamepad.enabled);
    slider("$FC_GAMEPAD_DEADZONE","gamepadDeadzone",draft.gamepad.deadzone,.1f,.8f);
    slider("$FC_GAMEPAD_TRIGGER_THRESHOLD","gamepadTriggerThreshold",draft.gamepad.triggerThreshold,.1f,.95f);
    const char* gamepadKeys[]{"$FC_BIND_ENTRY","$FC_BIND_RUN","$FC_BIND_HOP","$FC_GAMEPAD_DROP"};
    const char* gamepadIds[]{"gamepadEntry","gamepadRun","gamepadHop","gamepadDrop"};
    recording.triggerThreshold.store(draft.gamepad.triggerThreshold);
    for(std::size_t i=0;i<gamepadBindingFields.size();++i)bindingButton(BindingCaptureDevice::gamepad,i,gamepadKeys[i],gamepadIds[i]);
    if(button("$FC_VALIDATE_BINDINGS","validateBindings")) {
        if(acceptBindings()){notice.clear();detail.clear();}
    }
}
void diagnostics(const SettingsMenuSnapshot& snapshot) {
    check("$FC_DIAGNOSTICS_ENABLED","diagnostics",draft.diagnostics);
    ui::Text("%s: %s",tr("$FC_RUNTIME"),snapshot.ready?tr("$FC_STATUS_READY"):tr("$FC_STATUS_UNAVAILABLE"));
    ui::Text("%s: %s",tr("$FC_AUDIO_ASSETS"),snapshot.audioReady?tr("$FC_STATUS_READY"):tr("$FC_STATUS_UNAVAILABLE"));
    if(snapshot.movementPending)text("$FC_MOVEMENT_PENDING");
    if(snapshot.reloadPending)text("$FC_RELOAD_PENDING");
    if(!snapshot.packName.empty()){ui::Text("%s: %s",tr("$FC_ANIMATION_PACK"),snapshot.packName.c_str());itemHelp("$FC_ANIMATION_PACK");}
    ui::Text("%s: %llu / %llu / %llu",tr("$FC_AUTOMATIC_STATS"),
        static_cast<unsigned long long>(snapshot.automaticAttempts),static_cast<unsigned long long>(snapshot.automaticActions),static_cast<unsigned long long>(snapshot.wallRunObstacleJumps));
    itemHelp("$FC_AUTOMATIC_STATS");
    ui::BeginDisabled(snapshot.reloadPending);
    if(button("$FC_RELOAD_ANIMATIONS","reloadAnimations")&&callbacks.requestReloadAnimations) {
        callbacks.requestReloadAnimations();notice="reload-queued";detail.clear();
    }
    ui::EndDisabled();
    const bool showSlots=ui::CollapsingHeader(label("$FC_ANIMATION_PACK","animationSlotReports").c_str());itemHelp("$FC_ANIMATION_PACK");
    if(showSlots)for(const auto& slot:snapshot.slots) {
        const auto title=std::string(slotTitle(slot.name))+" ("+slot.name+")###slot_"+slot.name;
        const bool expanded=ui::CollapsingHeader(title.c_str());itemHelp("$FC_ANIMATION_PACK");
        if(!expanded)continue;
        ui::TextWrapped("%s: %s",tr("$FC_STATUS"),majorReason(slot));
        ui::TextWrapped("%s: %s",tr("$FC_FILE"),slot.file.c_str());
        if(snapshot.diagnosticsEnabled) {
            ui::Text("%s: %llu   %s: %llu",tr("$FC_SELECTED"),static_cast<unsigned long long>(slot.triggers),tr("$FC_OBSERVED_OUTPUT"),static_cast<unsigned long long>(slot.observed));
            itemHelp("$FC_OBSERVED_OUTPUT");
        } else {
            ui::Text("%s: %llu   %s: %s",tr("$FC_SELECTED"),static_cast<unsigned long long>(slot.triggers),tr("$FC_OBSERVED_OUTPUT"),tr("$FC_SAMPLING_OFF"));
            itemHelp("$FC_OBSERVED_OUTPUT");
            if(slot.observed){ui::Text("%s: %llu",tr("$FC_PREVIOUS_OBSERVED"),static_cast<unsigned long long>(slot.observed));itemHelp("$FC_OBSERVED_OUTPUT");}
        }
        ui::Text("%s: %zu   %s: %.3f",tr("$FC_SAMPLES"),slot.samples,tr("$FC_SECONDS"),slot.seconds);
        if(!slot.reason.empty())ui::TextWrapped("%s: %s",tr("$FC_TECHNICAL_DETAILS"),slot.reason.c_str());
    }
    if(snapshot.slots.empty())text("$FC_NO_ANIMATION_REPORT");
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
    itemHelp("$FC_LANGUAGE");
}
void __stdcall render() {
    help.reset();
    recording.rendered=true;recording.observing.store(true);finishCapture();
    if(!initialized){draft=callbacks.getSettings();edits.initialize(draft);reloadTranslations();initialized=true;}
    bool capturing=false;
    {std::scoped_lock lock(recording.mutex);capturing=recording.active;}
    ui::BeginDisabled(capturing);
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
    if(button("$FC_RESTORE_DEFAULTS","restoreDefaults"))restoreDefaults();
    ui::EndDisabled();applyEdits();capturePrompt();
    if(!notice.empty())ui::TextWrapped("%s",translatedNotice());
    if(!detail.empty())ui::TextWrapped("%s: %s",tr("$FC_DETAILS"),detail.c_str());
    if(!snapshot.error.empty()) {
        text("$FC_RUNTIME_ERROR_HELP");
        ui::TextWrapped("%s",snapshot.error.c_str());
    }
    if(!snapshot.status.empty())ui::TextWrapped("%s: %s",tr("$FC_RUNTIME_STATUS"),runtimeStatus(snapshot.status));
    if(ui::BeginChild("FreeClimbContextHelp",{0,ui::GetTextLineHeightWithSpacing()*5.5f},0,ui::ImGuiWindowFlags_NoSavedSettings)) {
        if(!capturing&&!help.helpKey.empty())ui::TextWrapped("%s: %s",tr(help.labelKey),tr(help.helpKey));
    }
    ui::EndChild();
}
}
bool registerSettingsMenu(SettingsMenuCallbacks value) {
    if(registered)return true;
    const auto module=GetMenuFrameworkModule();
    if(!module||!value.getSettings||!value.requestSave)return false;
    constexpr const char* required[]{"AddSectionItem","igTextWrappedV","igTextV","igCheckbox","igSliderFloat","igButton","igSameLine",
        "igCombo_Str_arr","igGetIO","ImGuiIO_SetAppAcceptingEvents","ImGuiIO_ClearEventsQueue","ImGuiIO_ClearInputKeys",
        "igBegin","igEnd","igGetWindowPos","igGetWindowSize","igSetNextWindowPos","igSetNextWindowSize",
        "RegisterEventPriority","igBeginDisabled","igEndDisabled","igCollapsingHeader_TreeNodeFlags",
        "igBeginTabBar","igEndTabBar","igBeginTabItem","igEndTabItem","igSeparator",
        "igIsItemHovered","igIsItemFocused","igBeginChild_Str","igEndChild","igGetTextLineHeightWithSpacing",
        "IsAnyBlockingWindowOpened"};
    for(const auto name:required)if(!GetProcAddress(module,name))return false;
    callbacks=std::move(value);SKSEMenuFramework::SetSection("FreeClimb");
    SKSEMenuFramework::AddSectionItem("Settings",render);
    static SKSEMenuFramework::Model::Event events(captureEvent,0.f);
    registered=true;return true;
}
void settingsMenuKeyboardSample(const std::uint8_t* keys) {
    if(!recording.observing.load())return;
    std::scoped_lock lock(recording.mutex);
    for(unsigned i=0;i<256;++i)recording.input.keyboard.down[i]=(keys[i]&0x80)!=0;
    recording.input.keyboardValid=true;advanceCapture();
}
void settingsMenuGamepadSample(bool available,std::uint16_t buttons,std::uint8_t leftTrigger,std::uint8_t rightTrigger) {
    if(!recording.observing.load())return;
    std::scoped_lock lock(recording.mutex);
    recording.input.gamepad=0;recording.input.gamepadValid=available;
    if(available) {
        for(unsigned i=0;i<14;++i)if(buttons&(1u<<(i<10?i:i+2)))recording.input.gamepad|=gamepadButtonMask(i);
        const float threshold=recording.triggerThreshold.load()*255.f;
        if(leftTrigger>=threshold)recording.input.gamepad|=gamepadButtonMask(14);
        if(rightTrigger>=threshold)recording.input.gamepad|=gamepadButtonMask(15);
    } else if(recording.active&&recording.capture.result().device==BindingCaptureDevice::gamepad) {
        recording.unavailable=true;recording.capture.cancel();
    }
    advanceCapture();
}
bool settingsMenuFilterInput(RE::InputEvent* event) {
    if(!event||!recording.filtering.load())return false;
    std::scoped_lock lock(recording.mutex);
    const auto device=event->GetDevice();
    if(device!=RE::INPUT_DEVICE::kKeyboard&&device!=RE::INPUT_DEVICE::kGamepad)return false;
    auto* button=event->AsButtonEvent();
    if(!button)return recording.active;
    const auto code=device==RE::INPUT_DEVICE::kKeyboard?button->GetIDCode():
        SKSE::InputMap::GamepadMaskToKeycode(button->GetIDCode())-SKSE::InputMap::kMacro_GamepadOffset;
    const bool consume=recording.ownership.filter(device==RE::INPUT_DEVICE::kKeyboard?
        BindingCaptureDevice::keyboard:BindingCaptureDevice::gamepad,code,button->IsDown(),button->IsUp(),recording.active);
    captureFilterState();
    return consume;
}
bool settingsMenuBlocking() {return registered&&SKSEMenuFramework::IsAnyBlockingWindowOpened();}
}
