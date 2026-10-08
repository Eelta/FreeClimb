#pragma once
#include "BindingCapture.h"
#include "UserSettings.h"
#include <utility>

namespace fc {
enum class SettingsEditResult {invalid,unchanged,queued};
class SettingsMenuEdits {
    std::string submitted;
public:
    void initialize(const UserSettings& settings){submitted=userSettingsIni(settings);}
    template<class Submit> SettingsEditResult apply(UserSettings& settings,Submit&& submit,bool force=false) {
        if(!validateBindings(settings.bindings).valid||!validateGamepadBindings(settings.gamepad.bindings).valid)
            return SettingsEditResult::invalid;
        auto normalized=sanitizeUserSettings(settings);auto encoded=userSettingsIni(normalized);
        settings=std::move(normalized);
        if(!force&&encoded==submitted)return SettingsEditResult::unchanged;
        submit(settings);submitted=std::move(encoded);return SettingsEditResult::queued;
    }
};
inline bool applyCompletedBinding(UserSettings& settings,const BindingCaptureResult& result,std::size_t index,std::string& error) {
    error.clear();
    if(result.status!=BindingCaptureStatus::captured){error="Binding recording is incomplete";return false;}
    auto candidate=settings;
    if(result.device==BindingCaptureDevice::keyboard&&index<bindingFields.size())candidate.bindings.*bindingFields[index].member=result.keyboard;
    else if(result.device==BindingCaptureDevice::gamepad&&index<gamepadBindingFields.size())candidate.gamepad.bindings.*gamepadBindingFields[index].member=result.gamepad;
    else {error="Binding recording target is invalid";return false;}
    for(const auto& validation:{validateBindings(candidate.bindings),validateGamepadBindings(candidate.gamepad.bindings)})
        if(!validation.valid){error=validation.message;return false;}
    settings=std::move(candidate);return true;
}
}
