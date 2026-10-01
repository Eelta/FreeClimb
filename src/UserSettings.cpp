#include "UserSettings.h"
#include "TranslationCatalog.h"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <system_error>

namespace fc {
namespace {
std::string trim(std::string_view input) {
    const auto begin=input.find_first_not_of(" \t\r\n");
    if(begin==input.npos)return {};
    return std::string(input.substr(begin,input.find_last_not_of(" \t\r\n")-begin+1));
}
std::string lower(std::string text) {
    for(char& c:text)if(c>='A'&&c<='Z')c=char(c-'A'+'a');
    return text;
}
using Values=std::map<std::string,std::string>;
std::string key(std::string_view section,std::string_view name) {return lower(std::string(section)+"/"+std::string(name));}
Values parse(std::istream& stream) {
    Values result;std::string line,section;
    while(std::getline(stream,line)) {
        if(line.starts_with("\xef\xbb\xbf"))line.erase(0,3);
        const auto text=trim(line);
        if(text.empty()||text[0]==';'||text[0]=='#')continue;
        if(text.front()=='['&&text.back()==']'){section=trim(std::string_view(text).substr(1,text.size()-2));continue;}
        const auto equals=text.find('=');
        if(equals!=text.npos)result[key(section,trim(std::string_view(text).substr(0,equals)))]=trim(std::string_view(text).substr(equals+1));
    }
    return result;
}
struct Field {const char* section;const char* name;float UserSettings::* member;float low,high;};
constexpr Field fields[]{
    {"Movement","UpSpeed",&UserSettings::upSpeed,10,140},{"Movement","DownSpeed",&UserSettings::downSpeed,10,140},
    {"Movement","SideSpeed",&UserSettings::sideSpeed,10,120},{"Movement","WallRunSpeed",&UserSettings::wallRunSpeed,0,450},
    {"Movement","DiagonalRunMultiplier",&UserSettings::diagonalRunMultiplier,1,1.3f},
    {"Movement","AutoActionMinSeconds",&UserSettings::autoActionMinSeconds,.65f,20},
    {"Movement","AutoActionMaxSeconds",&UserSettings::autoActionMaxSeconds,.65f,30},
    {"Movement","HopOutDistance",&UserSettings::hopOut,24,55},{"Movement","KickOutDistance",&UserSettings::kickOut,36,75},
    {"Audio","Volume",&UserSettings::audioVolume,0,1},{"Detection","Reach",&UserSettings::reach,40,160},
    {"Detection","GrabMaxSnap",&UserSettings::grabMaxSnap,5,60},{"Detection","GroundJumpHeight",&UserSettings::groundJumpHeight,0,88},
    {"Detection","MaxNormalZ",&UserSettings::maxNormalZ,.2f,.75f},{"Stamina","MovingPerSecond",&UserSettings::movingPerSecond,0,50},
    {"Stamina","HangingPerSecond",&UserSettings::hangingPerSecond,0,30},{"Stamina","RequiredToGrab",&UserSettings::requiredToGrab,0,100},
};
struct Flag {const char* section;const char* name;bool UserSettings::* member;};
constexpr Flag flags[]{
    {"General","Enabled",&UserSettings::enabled},{"General","Notifications",&UserSettings::notifications},
    {"General","LowStaminaNotifications",&UserSettings::lowStaminaNotifications},{"General","JumpToAttach",&UserSettings::jumpToAttach},
    {"General","AutoMantle",&UserSettings::autoMantle},{"General","ContextActions",&UserSettings::contextActions},
    {"General","ThreepeatAnimations",&UserSettings::threepeatAnimations},{"General","AutomaticClimbActions",&UserSettings::automaticClimbActions},
    {"General","LegacyAutomaticHops",&UserSettings::legacyAutomaticHops},{"General","SurfaceActionVariants",&UserSettings::surfaceActionVariants},
    {"General","WallRunObstacleJumps",&UserSettings::wallRunObstacleJumps},{"General","Diagnostics",&UserSettings::diagnostics},
    {"AutomaticActions","ContextualMantleEnabled",&UserSettings::contextualMantleEnabled},
    {"Movement","FancyJumps",&UserSettings::fancyJumps},
    {"Audio","Enabled",&UserSettings::audioEnabled},{"Stamina","Enabled",&UserSettings::staminaEnabled}
};
struct IniBindingField {const char* name;KeyChord InputBindings::* member;};
constexpr IniBindingField iniBindingFields[]{
    {"Forward",&InputBindings::forward},{"Backward",&InputBindings::backward},{"Left",&InputBindings::left},
    {"Right",&InputBindings::right},{"Entry",&InputBindings::entry},
    {"RunModifier",&InputBindings::runModifier},{"Hop",&InputBindings::hop}
};
std::optional<float> number(std::string text) {
    text=trim(text.substr(0,text.find_first_of(";#")));
    float value{};const auto parsed=std::from_chars(text.data(),text.data()+text.size(),value);
    if(parsed.ec!=std::errc{}||parsed.ptr!=text.data()+text.size()||!std::isfinite(value))return {};
    return value;
}
Values valuesOf(const UserSettings& source) {
    const auto settings=sanitizeUserSettings(source);Values values;
    auto put=[&](const char* section,const char* name,float value) {
        std::ostringstream out;out<<std::setprecision(7)<<value;values[key(section,name)]=out.str();
    };
    for(const auto& f:fields)put(f.section,f.name,settings.*f.member);
    for(const auto& f:flags)put(f.section,f.name,settings.*f.member?1.f:0.f);
    values[key("Menu","Language")]=settings.language;
    put("AutomaticActions","LeftWeight",settings.automaticSideWeights[0]);
    put("AutomaticActions","RightWeight",settings.automaticSideWeights[1]);
    for(const auto& f:iniBindingFields)values[key("Controls",f.name)]=serializeKeyChord(settings.bindings.*f.member);
    put("Gamepad","Enabled",settings.gamepad.enabled?1.f:0.f);
    put("Gamepad","Deadzone",settings.gamepad.deadzone);
    put("Gamepad","TriggerThreshold",settings.gamepad.triggerThreshold);
    for(const auto& f:gamepadBindingFields)values[key("Gamepad",f.name)]=serializeGamepadChord(settings.gamepad.bindings.*f.member);
    return values;
}
std::string mergeIni(std::istream& existing,const UserSettings& settings) {
    const auto all=valuesOf(settings);auto remaining=all;std::ostringstream out;std::string line,section;
    while(std::getline(existing,line)) {
        if(!line.empty()&&line.back()=='\r')line.pop_back();
        if(line.starts_with("\xef\xbb\xbf"))line.erase(0,3);
        const auto text=trim(line);
        if(text.size()>1&&text.front()=='['&&text.back()==']')section=trim(std::string_view(text).substr(1,text.size()-2));
        else if(!text.empty()&&text[0]!=';'&&text[0]!='#') {
            const auto equal=text.find('=');
            if(equal!=text.npos) {
                const auto name=trim(std::string_view(text).substr(0,equal));
                const auto id=key(section,name);
                if(id=="compatibility/requireskyparkour"||id=="controls/entrymodifier"||
                    id=="controls/holdseconds"||id=="animation/idlebreathing") {
                    const auto comment=line.find_first_of(";#",line.find('=')+1);
                    if(comment!=line.npos)out<<line.substr(0,line.find_first_not_of(" \t"))<<line.substr(comment)<<'\n';
                    continue;
                }
                if(auto found=remaining.find(id);found!=remaining.end()) {
                    out<<name<<'='<<found->second<<'\n';remaining.erase(found);continue;
                }
                if(all.contains(id))continue;
            }
        }
        out<<line<<'\n';
    }
    std::string active;
    for(const auto& [id,value]:remaining) {
        const auto slash=id.find('/');const auto group=id.substr(0,slash);
        if(group!=active){active=group;out<<"\n["<<group<<"]\n";}
        out<<id.substr(slash+1)<<'='<<value<<'\n';
    }
    return out.str();
}
}
UserSettings sanitizeUserSettings(UserSettings settings) {
    const UserSettings defaults;
    for(const auto& f:fields)settings.*f.member=std::isfinite(settings.*f.member)?std::clamp(settings.*f.member,f.low,f.high):defaults.*f.member;
    settings.autoActionMaxSeconds=std::max(settings.autoActionMinSeconds,settings.autoActionMaxSeconds);
    if(settings.wallRunSpeed==0)settings.wallRunSpeed=defaults.wallRunSpeed;
    settings.language=normalizeLanguage(settings.language);
    settings.legacyAutomaticHops=false;
    for(auto& weight:settings.automaticSideWeights)weight=std::isfinite(weight)?std::clamp(weight,0.f,1.f):1.f;
    if(!validateBindings(settings.bindings).valid)settings.bindings=InputBindings{};
    settings.gamepad=sanitizeGamepadSettings(settings.gamepad);
    return settings;
}
UserSettings restoreSettingsPage(SettingsPage page,const UserSettings& current) {
    UserSettings settings=current;
    const UserSettings defaults;
    switch(page) {
    case SettingsPage::general:
        settings.enabled=defaults.enabled;settings.notifications=defaults.notifications;
        settings.lowStaminaNotifications=defaults.lowStaminaNotifications;settings.autoMantle=defaults.autoMantle;
        break;
    case SettingsPage::movement:
        settings.upSpeed=defaults.upSpeed;settings.downSpeed=defaults.downSpeed;settings.sideSpeed=defaults.sideSpeed;
        settings.wallRunSpeed=defaults.wallRunSpeed;settings.diagonalRunMultiplier=defaults.diagonalRunMultiplier;
        break;
    case SettingsPage::automatic:
        settings.automaticClimbActions=defaults.automaticClimbActions;settings.wallRunObstacleJumps=defaults.wallRunObstacleJumps;
        settings.contextualMantleEnabled=defaults.contextualMantleEnabled;
        settings.autoActionMinSeconds=defaults.autoActionMinSeconds;settings.autoActionMaxSeconds=defaults.autoActionMaxSeconds;
        settings.automaticSideWeights=defaults.automaticSideWeights;
        break;
    case SettingsPage::stamina:
        settings.staminaEnabled=defaults.staminaEnabled;settings.movingPerSecond=defaults.movingPerSecond;
        settings.hangingPerSecond=defaults.hangingPerSecond;settings.requiredToGrab=defaults.requiredToGrab;
        break;
    case SettingsPage::audio:
        settings.audioEnabled=defaults.audioEnabled;settings.audioVolume=defaults.audioVolume;
        break;
    case SettingsPage::keys:
        settings.bindings=defaults.bindings;settings.gamepad=defaults.gamepad;
        break;
    case SettingsPage::diagnostics:
        settings.diagnostics=defaults.diagnostics;
        break;
    }
    return settings;
}
SettingsLoadResult loadUserSettings(const std::filesystem::path& path) {
    SettingsLoadResult result;std::ifstream stream(path);
    if(!stream){result.warnings.push_back("Settings file unavailable; using defaults");return result;}
    result.found=true;const auto values=parse(stream);
    auto read=[&](const char* section,const char* name,float fallback) {
        const auto it=values.find(key(section,name));if(it==values.end())return fallback;
        if(auto parsed=number(it->second))return *parsed;
        const auto word=lower(trim(it->second.substr(0,it->second.find_first_of(";#"))));if(word=="true")return 1.f;if(word=="false")return 0.f;
        result.warnings.push_back(std::string(section)+"/"+name+": invalid value; using default");return fallback;
    };
    for(const auto& f:fields)result.settings.*f.member=read(f.section,f.name,result.settings.*f.member);
    for(const auto& f:flags)result.settings.*f.member=read(f.section,f.name,result.settings.*f.member?1.f:0.f)>0;
    if(const auto it=values.find(key("Menu","Language"));it!=values.end())
        result.settings.language=normalizeLanguage(trim(it->second.substr(0,it->second.find_first_of(";#"))));
    result.settings.automaticSideWeights[0]=read("AutomaticActions","LeftWeight",1);
    result.settings.automaticSideWeights[1]=read("AutomaticActions","RightWeight",1);
    for(const auto& f:iniBindingFields)if(const auto it=values.find(key("Controls",f.name));it!=values.end()) {
        if(auto chord=parseKeyChord(it->second.substr(0,it->second.find_first_of(";#"))))result.settings.bindings.*f.member=*chord;
        else result.warnings.push_back(std::string("Controls/")+f.name+": invalid chord; using default");
    }
    result.settings.gamepad.enabled=read("Gamepad","Enabled",result.settings.gamepad.enabled?1.f:0.f)>0;
    result.settings.gamepad.deadzone=read("Gamepad","Deadzone",result.settings.gamepad.deadzone);
    result.settings.gamepad.triggerThreshold=read("Gamepad","TriggerThreshold",result.settings.gamepad.triggerThreshold);
    for(const auto& f:gamepadBindingFields)if(const auto it=values.find(key("Gamepad",f.name));it!=values.end()) {
        if(auto chord=parseGamepadChord(it->second.substr(0,it->second.find_first_of(";#"))))result.settings.gamepad.bindings.*f.member=*chord;
        else result.warnings.push_back(std::string("Gamepad/")+std::string(f.name)+": invalid chord; using default");
    }
    if(const auto validation=validateGamepadBindings(result.settings.gamepad.bindings);!validation.valid)
        result.warnings.push_back("Gamepad: "+validation.message+"; using default bindings");
    if(!values.contains(key("Controls","Entry"))) {
        if(const auto it=values.find(key("Controls","EntryModifier"));it!=values.end()) {
            const auto old=parseKeyChord(it->second.substr(0,it->second.find_first_of(";#")));
            const auto combined=old?combineKeyChords(result.settings.bindings.forward,*old):std::nullopt;
            if(combined) {
                if(*combined!=*parseKeyChord("Shift+W"))result.settings.bindings.entry=*combined;
            } else result.warnings.push_back("Controls/EntryModifier: invalid legacy chord; using default entry");
        }
    }
    if(const auto validation=validateBindings(result.settings.bindings);!validation.valid)result.warnings.push_back(validation.message+"; using default bindings");
    result.settings=sanitizeUserSettings(result.settings);return result;
}
std::string userSettingsIni(const UserSettings& settings) {
    std::istringstream empty;return mergeIni(empty,settings);
}
bool saveUserSettings(const std::filesystem::path& path,const UserSettings& settings,std::string& error) {
    error.clear();
    if(!validateBindings(settings.bindings).valid){error="Invalid or conflicting key bindings";return false;}
    if(!validateGamepadBindings(settings.gamepad.bindings).valid){error="Invalid or conflicting gamepad bindings";return false;}
    std::error_code ec;const auto parent=path.parent_path();
    if(!parent.empty())std::filesystem::create_directories(parent,ec);
    if(ec){error=ec.message();return false;}
    std::ifstream source(path);const auto content=mergeIni(source,settings);source.close();
    auto temporary=path;temporary+=".freeclimb.tmp";auto backup=path;backup+=".freeclimb.bak";
    if(std::filesystem::exists(temporary,ec)||std::filesystem::exists(backup,ec)){error="Previous settings transaction requires recovery";return false;}
    {std::ofstream output(temporary,std::ios::binary);output<<content;output.flush();if(!output){error="Unable to write settings";output.close();std::filesystem::remove(temporary,ec);return false;}}
    const bool previous=std::filesystem::exists(path,ec);
    if(previous){std::filesystem::rename(path,backup,ec);if(ec){error=ec.message();std::filesystem::remove(temporary,ec);return false;}}
    std::filesystem::rename(temporary,path,ec);
    if(ec) {
        error=ec.message();if(previous)std::filesystem::rename(backup,path,ec);
        std::filesystem::remove(temporary,ec);return false;
    }
    if(previous)std::filesystem::remove(backup,ec);
    return true;
}
}
