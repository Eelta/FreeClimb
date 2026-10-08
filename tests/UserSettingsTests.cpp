#include "UserSettings.h"
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>

namespace {
int failures=0,checks=0;
void expect(bool value,const char* message){++checks;if(!value){++failures;std::cerr<<message<<'\n';}}
void write(const std::filesystem::path& path,std::string_view value){std::ofstream out(path,std::ios::binary);out<<value;}
std::string read(const std::filesystem::path& path){std::ifstream in(path);return {std::istreambuf_iterator<char>(in),{}};}
std::map<std::string,std::string> iniFields(const fc::UserSettings& settings) {
    std::map<std::string,std::string> fields;
    std::istringstream input(fc::userSettingsIni(settings));std::string line,section;
    while(std::getline(input,line)) {
        if(line.empty())continue;
        if(line.front()=='['&&line.back()==']'){section=line.substr(1,line.size()-2);continue;}
        const auto equal=line.find('=');
        if(equal!=line.npos)fields[section+"/"+line.substr(0,equal)]=line.substr(equal+1);
    }
    return fields;
}
}
int main() {
    using namespace fc;
    const auto stamp=std::chrono::steady_clock::now().time_since_epoch().count();
    const auto directory=std::filesystem::current_path()/("settings-tests-"+std::to_string(stamp));
    std::filesystem::create_directories(directory);const auto path=directory/"FreeClimb.ini";
    auto missing=loadUserSettings(path);
    expect(!missing.found&&missing.settings.language=="english","Missing INI must use English defaults");
    expect(missing.settings.wallRunEnabled,"Missing INI preserves enabled wall running");
    expect(!missing.settings.climbSneakEnabled,"Missing INI disables automatic climbing sneak by default");
    expect(missing.settings.downSpeed==78&&missing.settings.wallRunSpeed==379.5f,"Runtime defaults must match shipped settings");
    expect(!UserSettings{}.diagnostics&&!missing.settings.diagnostics,"Detailed diagnostics must default off when configuration is absent");
    write(path,"[General]\nDiagnostics=1\n");
    const auto explicitDiagnostics=loadUserSettings(path);
    expect(explicitDiagnostics.settings.wallRunEnabled,"Old INI without a wall run master switch preserves existing behavior");
    expect(!explicitDiagnostics.settings.climbSneakEnabled,"Old INI without the climb sneak switch uses the disabled default");
    expect(explicitDiagnostics.settings.diagnostics,"Existing explicit Diagnostics=1 must remain enabled when read");
    expect(explicitDiagnostics.warnings.empty()&&explicitDiagnostics.settings.gamepad==GamepadSettings{},"Old INI files without a gamepad section retain keyboard behavior and adopt gamepad defaults");
    std::string diagnosticsError;
    expect(saveUserSettings(path,explicitDiagnostics.settings,diagnosticsError),"Explicit diagnostic setting should save");
    expect(loadUserSettings(path).settings.diagnostics,"Saving an explicit Diagnostics=1 must preserve the user's choice");
    for(const bool enabled:{false,true}) {
        write(path,std::string("[General]\nClimbSneakEnabled=")+(enabled?"1":"0")+"\n[FutureMod]\nCustom=keep\n");
        const auto sneak=loadUserSettings(path);
        expect(sneak.warnings.empty()&&sneak.settings.climbSneakEnabled==enabled,"The climb sneak switch loads both explicit values");
        expect(saveUserSettings(path,sneak.settings,diagnosticsError),"The climb sneak switch saves successfully");
        expect(loadUserSettings(path).settings.climbSneakEnabled==enabled&&read(path).find("Custom=keep")!=std::string::npos,
            "Saving and reloading preserves the climb sneak choice and unrelated settings");
    }
    write(path,"[General]\nClimbSneakEnabled=invalid\n");
    const auto invalidSneak=loadUserSettings(path);
    expect(!invalidSneak.settings.climbSneakEnabled&&invalidSneak.warnings.size()==1,"An invalid climb sneak switch warns and retains the disabled default");
    write(path,"\xef\xbb\xbf[Movement]\r\nUpSpeed=nan\r\nDownSpeed=999\r\nSideSpeed=bad\r\nWallRunSpeed=0\r\nAutoActionMinSeconds=5\r\nAutoActionMaxSeconds=1\r\n[Stamina]\r\nEnabled=0\r\nMovingPerSecond=21\r\nRequiredToGrab=-3\r\n[Menu]\r\nLanguage=1\r\n[General]\r\nLegacyAutomaticHops=1\r\n[AutomaticActions]\r\nLeftWeight=-1\r\nRightWeight=2\r\n[Controls]\r\nHoldSeconds=99\r\n");
    const auto loaded=loadUserSettings(path);
    expect(loaded.found&&loaded.settings.language=="chinese","BOM and CRLF settings should load and migrate legacy Chinese language");
    expect(loaded.settings.upSpeed==100&&loaded.settings.sideSpeed==82,"Invalid numbers fall back per field");
    expect(loaded.settings.downSpeed==140&&loaded.settings.wallRunSpeed==379.5f,"Clamp bounds and preserve zero wallrun fallback");
    expect(loaded.settings.autoActionMinSeconds==5&&loaded.settings.autoActionMaxSeconds==5,"Interval order remains valid");
    expect(!loaded.settings.staminaEnabled&&loaded.settings.movingPerSecond==21&&loaded.settings.requiredToGrab==0,"Zero-consumption mode preserves tuned costs");
    expect(serializeKeyChord(loaded.settings.bindings.entry)=="W+A+D+Space","Retired hold threshold cannot change deliberate default entry");
    expect(!loaded.settings.legacyAutomaticHops,"Old automatic hops must remain disabled");
    expect(loaded.settings.automaticSideWeights[0]==0&&loaded.settings.automaticSideWeights[1]==1,"Direction opportunity rates bounded");
    write(path,"[Controls]\nForward=Q\nBackward=Q\nHop=Space\n");
    auto conflict=loadUserSettings(path);
    expect(serializeKeyChord(conflict.settings.bindings.forward)=="W"&&!conflict.warnings.empty(),"Conflicting INI bindings must fall back coherently");
    UserSettings settings;settings.language="chinese";settings.staminaEnabled=false;settings.audioVolume=.35f;
    settings.bindings.hop=*parseKeyChord("Ctrl+Space");settings.automaticSideWeights={.2f,.7f};
    settings.gamepad.enabled=false;settings.gamepad.deadzone=.35f;settings.gamepad.triggerThreshold=.7f;
    settings.gamepad.bindings.entry=*parseGamepadChord("RB+X");settings.gamepad.bindings.runModifier=*parseGamepadChord("RB");
    settings.gamepad.bindings.hop=*parseGamepadChord("X");settings.gamepad.bindings.drop=*parseGamepadChord("A");
    expect(validateGamepadBindings(settings.gamepad.bindings).valid,"Test custom gamepad combinations must be valid");
    expect(validateBindings(settings.bindings).valid,"Test custom combination is valid");
    write(path,"; preserve my settings\n[FutureMod]\nCustom=keep\n[Audio]\nVolume=.1\nVolume=.9\n");
    std::string error;expect(saveUserSettings(path,settings,error),"Valid settings should save");
    auto roundtrip=loadUserSettings(path);
    expect(roundtrip.settings.language=="chinese"&&!roundtrip.settings.staminaEnabled,"Language and no-stamina option persist");
    expect(std::abs(roundtrip.settings.audioVolume-.35f)<.0001f,"Last duplicate must not override saved value");
    expect(serializeKeyChord(roundtrip.settings.bindings.hop)=="Ctrl+Space","Combination persists");
    expect(roundtrip.settings.automaticSideWeights==settings.automaticSideWeights,"Opportunity rates persist");
    expect(roundtrip.warnings.empty()&&roundtrip.settings.gamepad==settings.gamepad,"Gamepad enable, thresholds and every binding survive an INI round trip");
    expect(read(path).find("Custom=keep")!=std::string::npos&&read(path).find("; preserve my settings")!=std::string::npos,"Unknown sections and comments survive saving");
    const auto before=read(path);settings.bindings.forward=settings.bindings.backward;
    expect(!saveUserSettings(path,settings,error)&&read(path)==before,"Invalid menu bindings must not overwrite INI");
    settings=roundtrip.settings;settings.gamepad.bindings.drop=settings.gamepad.bindings.hop;
    expect(!saveUserSettings(path,settings,error)&&!error.empty()&&read(path)==before,"Invalid gamepad bindings must reject save without modifying the INI");
    settings=roundtrip.settings;write(std::filesystem::path(path.string()+".freeclimb.bak"),"recovery");
    expect(!saveUserSettings(path,settings,error)&&read(path)==before,"Unresolved prior save must preserve both files");
    std::filesystem::remove(std::filesystem::path(path.string()+".freeclimb.bak"));
    settings.upSpeed=std::numeric_limits<float>::infinity();
    expect(sanitizeUserSettings(settings).upSpeed==100,"In-memory NaN and infinity do not reach runtime");
    write(path,"; legacy compatibility comment\n[Compatibility]\nRequireSkyParkour=1\n  rEqUiReSkYpArKoUr = 1 ; retained inline comment\nRequireSkyParkourExtra=untouched\nOtherCompatibilityOption=keep\n# retained section comment\n[OtherSection]\nRequireSkyParkour=keep_other_section\n");
    const auto legacy=loadUserSettings(path);
    expect(legacy.found&&legacy.warnings.empty()&&userSettingsIni(legacy.settings)==userSettingsIni(UserSettings{}),"Deprecated dependency setting must not affect loaded settings or introduce a prerequisite");
    expect(saveUserSettings(path,legacy.settings,error),"Legacy settings migration should save");
    const auto migrated=read(path);
    expect(migrated.find("RequireSkyParkour=1")==std::string::npos&&migrated.find("rEqUiReSkYpArKoUr")==std::string::npos,"Saving must remove all case-insensitive instances of the retired compatibility key");
    expect(migrated.find("[Compatibility]")!=std::string::npos&&migrated.find("RequireSkyParkourExtra=untouched")!=std::string::npos&&migrated.find("OtherCompatibilityOption=keep")!=std::string::npos,"Migration must retain the compatibility section and other unknown keys");
    expect(migrated.find("; legacy compatibility comment")!=std::string::npos&&migrated.find("; retained inline comment")!=std::string::npos&&migrated.find("# retained section comment")!=std::string::npos,"Migration must preserve standalone and inline comments");
    expect(migrated.find("[OtherSection]")!=std::string::npos&&migrated.find("RequireSkyParkour=keep_other_section")!=std::string::npos,"Migration must only remove the exact retired section/key pair");
    expect(userSettingsIni(UserSettings{}).find("compatibility")==std::string::npos,"New default settings must not recreate the retired section");
    struct EntryCase {const char* controls;const char* entry;bool warning;};
    const EntryCase entries[]{
        {"", "W+A+D+Space", false},
        {"EntryModifier=Shift\nHoldSeconds=0.6\n", "W+A+D+Space", false},
        {"EntryModifier=RShift\n", "RShift+W", false},
        {"Forward=Up\nEntryModifier=Ctrl\n", "Ctrl+Up", false},
        {"Forward=Ctrl+W\nEntryModifier=RCtrl+Alt\n", "RCtrl+Alt+W", false},
        {"Entry=G\nEntryModifier=Ctrl\n", "G", false},
        {"Entry=Ctrl+Alt+Shift+G\n", "Shift+Ctrl+Alt+G", false},
        {"Entry=Space+A+W+D ; entry note\n", "W+A+D+Space", false},
        {"Entry=NoSuchKey\nEntryModifier=Ctrl\n", "W+A+D+Space", true},
        {"EntryModifier=Ctrl+Alt+Shift+G\n", "W+A+D+Space", true},
        {"Forward=F4\nEntryModifier=Alt\n", "W+A+D+Space", true}
    };
    for(const auto& entry:entries) {
        write(path,std::string("[Controls]\n")+entry.controls+
            "HoldSeconds=0.1\n[Animation]\nIdleBreathing=1\nOther=keep\n[Other]\nEntryModifier=unchanged\nIdleBreathing=unchanged\n[Menu]\nLanguage=chinese\n[General]\nDiagnostics=1\n");
        const auto value=loadUserSettings(path);
        expect(serializeKeyChord(value.settings.bindings.entry)==entry.entry,"Entry migration must preserve custom chords and replace only the old default");
        expect(value.warnings.empty()!=entry.warning,"Entry migration reports malformed or unsafe legacy bindings");
        expect(value.settings.language=="chinese"&&value.settings.diagnostics,"Entry migration retains local language and diagnostics");
        expect(saveUserSettings(path,value.settings,error),"Migrated entry settings should save atomically");
        const auto text=read(path);
        expect(text.find("HoldSeconds=")==std::string::npos&&text.find("IdleBreathing=1")==std::string::npos,"Save removes retired hold and breathing options");
        expect(text.find("EntryModifier=unchanged")!=std::string::npos&&text.find("IdleBreathing=unchanged")!=std::string::npos&&text.find("Other=keep")!=std::string::npos,"Migration retains unrelated options and same-name keys in other sections");
        const auto restored=loadUserSettings(path);
        expect(restored.warnings.empty()&&serializeKeyChord(restored.settings.bindings.entry)==entry.entry,"Migrated explicit entry survives another load without legacy fallback");
    }
    expect(userSettingsIni(UserSettings{}).find("idlebreathing")==std::string::npos&&
        userSettingsIni(UserSettings{}).find("holdseconds")==std::string::npos&&
        userSettingsIni(UserSettings{}).find("entrymodifier")==std::string::npos,"New settings omit all removed controls and breathing fields");
    struct LanguageCase {const char* input;const char* expected;};
    const LanguageCase languages[]{
        {"0","english"},{"1","chinese"},{" English ","english"},{" CHINESE # old preference","chinese"},
        {"  FRENCH \t ; custom translation","french"},{"portuguese_br","portuguese_br"},
        {"pt-BR","pt-br"},{"../chinese","english"},{"","english"},{"2","english"}
    };
    for(const auto& language:languages) {
        write(path,std::string("[Menu]\nLanguage=")+language.input+"\n[Audio]\nVolume=0.35\n[FutureMod]\nKeep=unchanged\n");
        const auto selected=loadUserSettings(path);
        expect(selected.settings.language==language.expected,"Legacy and custom language IDs must normalize without checking installed files");
        expect(saveUserSettings(path,selected.settings,error),"Normalized language should save");
        const auto restored=loadUserSettings(path);
        expect(restored.settings.language==language.expected&&read(path).find(std::string("Language=")+language.expected+"\n")!=std::string::npos,"Save must persist canonical language rather than a legacy number or comment");
        expect(std::abs(restored.settings.audioVolume-.35f)<.0001f&&read(path).find("Keep=unchanged")!=std::string::npos,"Language migration must preserve other settings and unknown fields");
    }
    for(const auto value:{"false ; disabled", "FALSE # disabled", "0 ; disabled", "0 # disabled"}) {
        write(path,std::string("[Gamepad]\nEnabled=")+value+"\n[Audio]\nEnabled=TrUe ; enabled\nVolume=.35 # volume\n[Menu]\nLanguage=chinese ; local\n");
        const auto commented=loadUserSettings(path);
        expect(commented.warnings.empty()&&!commented.settings.gamepad.enabled&&commented.settings.audioEnabled&&
            commented.settings.audioVolume==.35f&&commented.settings.language=="chinese",
            "Boolean words and numeric flags accept inline comments without changing numbers or language parsing");
    }
    write(path,"[Gamepad]\nEnabled=bad\nDeadzone=nan\nTriggerThreshold=inf\nEntry=Start\nHop=NoSuchButton\n");
    const auto invalidGamepad=loadUserSettings(path);
    expect(invalidGamepad.settings.gamepad==GamepadSettings{}&&invalidGamepad.warnings.size()==5,
        "Malformed gamepad flags, numbers and unsupported buttons warn and use defaults");
    write(path,"[Gamepad]\nDeadzone=-1\nTriggerThreshold=10\n");
    const auto boundedGamepad=loadUserSettings(path);
    expect(boundedGamepad.settings.gamepad.deadzone==.1f&&boundedGamepad.settings.gamepad.triggerThreshold==.95f,
        "Gamepad thresholds clamp to the same bounds as the menu");
    write(path,"[Gamepad]\nDeadzone=4\nTriggerThreshold=-1\n");
    const auto oppositeBounds=loadUserSettings(path);
    expect(oppositeBounds.settings.gamepad.deadzone==.8f&&oppositeBounds.settings.gamepad.triggerThreshold==.1f,
        "Both gamepad threshold ranges have bounded lower and upper limits");
    for(const auto controls:{"Entry=LB+B\n", "RunModifier=Y\n", "Hop=B\n", "Drop=LB\n"}) {
        write(path,std::string("[Gamepad]\nEnabled=0\nDeadzone=.4\n")+controls+"[Controls]\nEntry=G\n[Menu]\nLanguage=chinese\n[General]\nDiagnostics=1\n");
        const auto value=loadUserSettings(path);
        expect(!value.warnings.empty()&&value.settings.gamepad.bindings==GamepadBindings{},
            "Conflicting gamepad actions must fall back to a coherent default binding set");
        expect(!value.settings.gamepad.enabled&&value.settings.gamepad.deadzone==.4f&&
            serializeKeyChord(value.settings.bindings.entry)=="G"&&value.settings.language=="chinese"&&value.settings.diagnostics,
            "Gamepad binding fallback preserves other gamepad, keyboard and local settings");
    }
    write(path,"[gAmEpAd]\neNaBlEd=false\ndEaDzOnE=.3 ; stick\ntRiGgErThReShOlD=.6 # trigger\n"
        "eNtRy=LT+RT ; entry\nrUnMoDiFiEr=LB\nhOp=Y\ndRoP=B\nFutureOption=keep\n"
        "[Other]\nEntry=unrelated\n[Menu]\nLanguage=chinese\n[General]\nDiagnostics=1\n");
    const auto namedGamepad=loadUserSettings(path);
    expect(namedGamepad.warnings.empty()&&!namedGamepad.settings.gamepad.enabled&&
        namedGamepad.settings.gamepad.bindings.entry==*parseGamepadChord("LT+RT"),
        "Case-insensitive INI names, comments and trigger chords are supported");
    expect(saveUserSettings(path,namedGamepad.settings,error),"Custom gamepad settings should merge into existing INI");
    const auto savedGamepad=loadUserSettings(path);
    expect(savedGamepad.settings.gamepad==namedGamepad.settings.gamepad&&savedGamepad.settings.language=="chinese"&&
        savedGamepad.settings.diagnostics&&read(path).find("FutureOption=keep")!=std::string::npos&&
        read(path).find("Entry=unrelated")!=std::string::npos,
        "Gamepad saves retain unknown fields and the author's language and diagnostic preferences");
    auto nonfiniteGamepad=namedGamepad.settings;
    nonfiniteGamepad.gamepad.deadzone=std::numeric_limits<float>::quiet_NaN();
    nonfiniteGamepad.gamepad.triggerThreshold=std::numeric_limits<float>::infinity();
    const auto sanitizedGamepad=sanitizeUserSettings(nonfiniteGamepad);
    expect(sanitizedGamepad.gamepad.deadzone==GamepadSettings{}.deadzone&&
        sanitizedGamepad.gamepad.triggerThreshold==GamepadSettings{}.triggerThreshold,
        "Non-finite in-memory gamepad thresholds return to defaults before runtime use");
    settings.language=" CHINESE ";
    expect(sanitizeUserSettings(settings).language=="chinese","In-memory language must canonicalize before applying");
    write(path,
        "[Menu]\nLanguage=french\n"
        "[General]\nEnabled=0\nNotifications=0\nLowStaminaNotifications=0\nAutoMantle=0\nClimbSneakEnabled=1\n"
        "AutomaticClimbActions=0\nWallRunObstacleJumps=0\nDiagnostics=1\nJumpToAttach=0\n"
        "ContextActions=0\nThreepeatAnimations=0\nSurfaceActionVariants=0\n"
        "[Movement]\nWallRunEnabled=0\nUpSpeed=41\nDownSpeed=42\nSideSpeed=43\nWallRunSpeed=244\nDiagonalRunMultiplier=1.23\n"
        "AutoActionMinSeconds=7\nAutoActionMaxSeconds=9\nHopOutDistance=40\nKickOutDistance=60\nFancyJumps=0\n"
        "[AutomaticActions]\nContextualMantleEnabled=0\nLeftWeight=0.25\nRightWeight=0.5\n"
        "[Stamina]\nEnabled=0\nMovingPerSecond=23\nHangingPerSecond=12\nRequiredToGrab=31\n"
        "[Audio]\nEnabled=0\nVolume=0.2\n"
        "[Detection]\nReach=120\nGrabMaxSnap=25\nGroundJumpHeight=40\nMaxNormalZ=0.5\n"
        "[Controls]\nForward=Up\nBackward=Down\nLeft=Left\nRight=Right\n"
        "Entry=Ctrl+Up\nRunModifier=Alt\nHop=Enter\n"
        "[Gamepad]\nEnabled=0\nDeadzone=.35\nTriggerThreshold=.7\nEntry=RB+X\nRunModifier=RB\nHop=X\nDrop=A\n");
    const auto customized=loadUserSettings(path);
    expect(!customized.settings.wallRunEnabled,"Explicit movement wall run disable is read independently of obstacle jumps");
    expect(customized.warnings.empty()&&validateBindings(customized.settings.bindings).valid,"Page reset fixture must have valid non-default settings");
    const auto originalFields=iniFields(customized.settings),defaultFields=iniFields(UserSettings{});
    struct PageCase {SettingsPage page;std::set<std::string> fields;};
    const PageCase pages[]{
        {SettingsPage::general,{"general/enabled","general/notifications","general/lowstaminanotifications","general/automantle","general/climbsneakenabled"}},
        {SettingsPage::movement,{"movement/wallrunenabled","movement/upspeed","movement/downspeed","movement/sidespeed","movement/wallrunspeed","movement/diagonalrunmultiplier"}},
        {SettingsPage::automatic,{"general/automaticclimbactions","general/wallrunobstaclejumps","automaticactions/contextualmantleenabled",
            "movement/autoactionminseconds","movement/autoactionmaxseconds","automaticactions/leftweight","automaticactions/rightweight"}},
        {SettingsPage::stamina,{"stamina/enabled","stamina/movingpersecond","stamina/hangingpersecond","stamina/requiredtograb"}},
        {SettingsPage::audio,{"audio/enabled","audio/volume"}},
        {SettingsPage::keys,{"controls/forward","controls/backward","controls/left","controls/right","controls/entry",
            "controls/runmodifier","controls/hop","gamepad/enabled","gamepad/deadzone","gamepad/triggerthreshold",
            "gamepad/entry","gamepad/runmodifier","gamepad/hop","gamepad/drop"}},
        {SettingsPage::diagnostics,{"general/diagnostics"}}
    };
    for(const auto& page:pages) {
        const auto restored=restoreSettingsPage(page.page,customized.settings);
        const auto actual=iniFields(restored);auto expected=originalFields;
        for(const auto& key:page.fields) {
            expect(originalFields.at(key)!=defaultFields.at(key),"Every displayed page field must differ from its default in the fixture");
            expected[key]=defaultFields.at(key);
        }
        expect(actual==expected,"Page reset must change only that tab's displayed settings and preserve all other INI fields");
        expect(restored.language=="french","Resetting any page must retain the selected custom language");
        expect(iniFields(restoreSettingsPage(page.page,restored))==actual,"Repeated page reset must be idempotent");
        write(path,"; preserve unrelated content\n[FutureMod]\nCustom=keep\n");
        expect(saveUserSettings(path,restored,error),"Page defaults should save without requiring a second apply action");
        const auto persisted=loadUserSettings(path);
        expect(persisted.warnings.empty()&&iniFields(persisted.settings)==expected,"Saved page defaults and all unaffected settings must survive reload");
        expect(read(path).find("Custom=keep")!=std::string::npos&&read(path).find("; preserve unrelated content")!=std::string::npos,
            "Saving page defaults must retain unknown INI fields and comments");
        auto invalidDraft=customized.settings;
        invalidDraft.language=" FRENCH ";invalidDraft.reach=999;invalidDraft.bindings.forward=invalidDraft.bindings.backward;
        invalidDraft.gamepad.bindings.drop=invalidDraft.gamepad.bindings.hop;invalidDraft.gamepad.deadzone=5;
        const auto retained=restoreSettingsPage(page.page,invalidDraft);
        expect(retained.language==invalidDraft.language&&retained.reach==999,"Page reset must not normalize unrelated editor values");
        expect(page.page==SettingsPage::keys?validateBindings(retained.bindings).valid:
            serializeKeyChord(retained.bindings.forward)==serializeKeyChord(invalidDraft.bindings.forward),
            "Only resetting the keys page may replace unrelated invalid binding drafts");
        expect(page.page==SettingsPage::keys?retained.gamepad==GamepadSettings{}:retained.gamepad==invalidDraft.gamepad,
            "Only the keys page resets gamepad controls and replaces invalid gamepad drafts");
    }
    std::filesystem::remove(path);std::filesystem::remove(directory);
    std::cout<<checks<<" settings checks, "<<failures<<" failures\n";return failures?1:0;
}
