#pragma once
#include <string_view>

namespace fc {
struct SettingsHelpEntry {std::string_view label,help;};
inline constexpr SettingsHelpEntry settingsHelpEntries[]{
    {"$FC_LANGUAGE","$FC_LANGUAGE_HELP"},
    {"$FC_ENABLED","$FC_ENTRY_HELP"},
    {"$FC_NOTIFICATIONS","$FC_NOTIFICATIONS_HELP"},
    {"$FC_LOW_STAMINA_NOTIFICATIONS","$FC_NOTIFICATIONS_HELP"},
    {"$FC_AUTO_MANTLE","$FC_AUTO_MANTLE_HELP"},
    {"$FC_UP_SPEED","$FC_MOVEMENT_HELP"},
    {"$FC_DOWN_SPEED","$FC_MOVEMENT_HELP"},
    {"$FC_SIDE_SPEED","$FC_MOVEMENT_HELP"},
    {"$FC_WALL_RUN_ENABLED","$FC_WALL_RUN_HELP"},
    {"$FC_WALL_RUN_SPEED","$FC_MOVEMENT_HELP"},
    {"$FC_DIAGONAL_MULTIPLIER","$FC_DIAGONAL_HELP"},
    {"$FC_AUTO_SIDE_ACTIONS","$FC_ATTEMPT_HELP"},
    {"$FC_WALL_RUN_OBSTACLES","$FC_WALL_RUN_OBSTACLES_HELP"},
    {"$FC_CONTEXT_MANTLE","$FC_CONTEXT_MANTLE_HELP"},
    {"$FC_ATTEMPT_INTERVAL_MIN","$FC_INTERVAL_HELP"},
    {"$FC_ATTEMPT_INTERVAL_MAX","$FC_INTERVAL_HELP"},
    {"$FC_LEFT_OPPORTUNITY","$FC_OPPORTUNITY_HELP"},
    {"$FC_RIGHT_OPPORTUNITY","$FC_OPPORTUNITY_HELP"},
    {"$FC_STAMINA_ENABLED","$FC_STAMINA_HELP"},
    {"$FC_CLIMB_STAMINA","$FC_STAMINA_RATE_HELP"},
    {"$FC_WALL_RUN_STAMINA","$FC_STAMINA_RATE_HELP"},
    {"$FC_REST_STAMINA","$FC_STAMINA_RATE_HELP"},
    {"$FC_GRAB_STAMINA","$FC_GRAB_STAMINA_HELP"},
    {"$FC_AUDIO_ENABLED","$FC_AUDIO_HELP"},
    {"$FC_AUDIO_VOLUME","$FC_AUDIO_HELP"},
    {"$FC_KEYBOARD_SECTION","$FC_BINDING_DERIVED_HELP"},
    {"$FC_GAMEPAD_SECTION","$FC_GAMEPAD_CONTROL_HELP"},
    {"$FC_GAMEPAD_ENABLED","$FC_GAMEPAD_CONTROL_HELP"},
    {"$FC_GAMEPAD_DEADZONE","$FC_GAMEPAD_THRESHOLD_HELP"},
    {"$FC_GAMEPAD_TRIGGER_THRESHOLD","$FC_GAMEPAD_THRESHOLD_HELP"},
    {"$FC_VALIDATE_BINDINGS","$FC_VALIDATE_BINDINGS_HELP"},
    {"$FC_DIAGNOSTICS_ENABLED","$FC_DIAGNOSTICS_HELP"},
    {"$FC_AUTOMATIC_STATS","$FC_STATS_HELP"},
    {"$FC_RELOAD_ANIMATIONS","$FC_RELOAD_HELP"},
    {"$FC_ANIMATION_PACK","$FC_SLOT_REPORT_HELP"},
    {"$FC_SELECTED","$FC_STATS_HELP"},
    {"$FC_OBSERVED_OUTPUT","$FC_OBSERVED_HELP"},
    {"$FC_RESTORE_DEFAULTS","$FC_RESTORE_DEFAULTS_HELP"}
};
constexpr std::string_view settingsHelpFor(std::string_view label) {
    for(const auto& entry:settingsHelpEntries)if(entry.label==label)return entry.help;
    return {};
}
struct SettingsMenuHelp {
    std::string_view labelKey{},helpKey{};
    unsigned priority{};
    void reset(){*this={};}
    void observe(std::string_view label,std::string_view help,bool hovered,bool focused) {
        const unsigned rank=hovered?2:focused?1:0;
        if(rank&&rank>=priority&&!help.empty()){labelKey=label;helpKey=help;priority=rank;}
    }
};
}
