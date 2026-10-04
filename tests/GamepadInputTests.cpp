#include "GamepadInput.h"
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace fc;
static unsigned checks{};
static void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
static GamepadChord chord(std::string_view text){auto result=parseGamepadChord(text);check(result.has_value(),"valid fixture gamepad chord");return *result;}
static void press(GamepadState& state,GamepadChord buttons,bool down=true) {
    for(unsigned index=0;index<16;++index)if(buttons&gamepadButtonMask(index))state.setButton(index,down?1.f:0.f,.5f);
}
static bool emptyActions(Keys keys) {
    return !keys.w&&!keys.a&&!keys.s&&!keys.d&&!keys.shift&&!keys.space&&!keys.entry&&!keys.letGo;
}
static void parsingAndBindings() {
    check(GamepadBindings{}==GamepadBindings::defaults(),"default binding construction is stable");
    check(serializeGamepadChord(GamepadBindings{}.entry)=="LB+Y","default entry uses LB and Y");
    check(chord(" y + lb ")==chord("LB+Y"),"case whitespace and order normalize");
    for(const auto& named:gamepadButtonNames) {
        auto parsed=parseGamepadChord(named.name);
        if(named.index==4||named.index==5)check(!parsed,"Start and Back remain reserved for native UI");
        else check(parsed&&*parsed==gamepadButtonMask(named.index),"button names use the exact SKSE index");
    }
    for(const auto text:{""," ","+","LB+","+Y","LB++Y","LB+lb","Start","Back","LB+Start","Guide","Mouse1","Y+Z","A+B+X+Y+LB"})
        check(!parseGamepadChord(text),"invalid reserved duplicate and oversized chords are rejected");
    check(!parseGamepadChord(std::string(129,'A')),"parser rejects oversized text");
    check(chord("RT+LS+DPadLeft+X")==static_cast<GamepadChord>((1u<<15)|(1u<<6)|(1u<<2)|(1u<<12)),"four independent buttons are supported");
    for(unsigned value=0;value<=0xffff;++value) {
        const auto candidate=static_cast<GamepadChord>(value);
        const bool valid=value&&!(value&0x30)&&std::popcount(value)<=4;
        check(validGamepadChord(candidate)==valid,"every mask respects reserved buttons and four-button capacity");
        const auto text=serializeGamepadChord(candidate);
        if(valid)check(parseGamepadChord(text)==candidate,"every valid mask survives serialization");
        else check(text.empty(),"invalid in-memory mask is never serialized");
    }
    check(validateGamepadBindings(GamepadBindings{}).valid,"default gamepad bindings are valid");
    for(const auto& field:gamepadBindingFields) {
        GamepadBindings bindings;bindings.*field.member=0;
        const auto validation=validateGamepadBindings(bindings);
        check(!validation.valid&&validation.first==field.name,"invalid action identifies its field");
    }
    GamepadBindings bindings;
    bindings.entry=chord("RB+X");bindings.runModifier=chord("RB");bindings.hop=chord("X");bindings.drop=chord("A");
    check(validateGamepadBindings(bindings).valid,"entry can share separately configured run and hop buttons");
    bindings.hop=chord("RB+X");
    auto validation=validateGamepadBindings(bindings);
    check(!validation.valid&&validation.first=="runModifier"&&validation.second=="hop","subset run and hop bindings conflict");
    bindings=GamepadBindings{};bindings.drop=bindings.hop;
    check(!validateGamepadBindings(bindings).valid,"hop and drop cannot trigger together from the same button");
    bindings=GamepadBindings{};bindings.entry=chord("LB+B");
    validation=validateGamepadBindings(bindings);
    check(!validation.valid&&validation.first=="entry"&&validation.second=="drop","entry cannot simultaneously trigger drop");
    bindings=GamepadBindings{};bindings.entry=chord("LS+RS+LT+RT");
    check(validateGamepadBindings(bindings).valid,"entry may independently use four buttons");
    GamepadSettings settings;settings.enabled=false;settings.bindings=bindings;
    check(sanitizeGamepadSettings(settings)==settings,"valid custom settings are preserved including disabled state");
    settings.deadzone=-10;settings.triggerThreshold=10;
    auto clean=sanitizeGamepadSettings(settings);
    check(clean.deadzone==.1f&&clean.triggerThreshold==.95f,"thresholds are clamped to menu bounds");
    settings.deadzone=10;settings.triggerThreshold=-10;clean=sanitizeGamepadSettings(settings);
    check(clean.deadzone==.8f&&clean.triggerThreshold==.1f,"opposite threshold limits are bounded");
    for(float invalid:{std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()}) {
        settings.deadzone=settings.triggerThreshold=invalid;settings.bindings.drop=settings.bindings.hop;
        clean=sanitizeGamepadSettings(settings);
        check(clean.deadzone==.25f&&clean.triggerThreshold==.5f&&clean.bindings==GamepadBindings{}&&!clean.enabled,"nonfinite values and invalid bindings safely default");
    }
}
static void completeChordsAndRearming() {
    for(const auto text:{"X","LB+Y","LS+RB+X","DPadUp+LS+RB+X"}) {
        GamepadBindings bindings;bindings.entry=chord(text);
        std::array<unsigned,4> order{};unsigned count{};
        for(unsigned index=0;index<16;++index)if(bindings.entry&gamepadButtonMask(index))order[count++]=index;
        do {
            GamepadState state;ClimbEntryIntent intent;
            for(unsigned i=0;i<count;++i) {
                state.setButton(order[i],1,.5f);
                const auto keys=state.keys(bindings);const auto request=intent.sample(keys);
                check(keys.bindingsMapped&&!keys.w&&!keys.a&&!keys.s&&!keys.d,"entry does not fabricate movement directions");
                check(request.requested==(i+1==count)&&request.fresh==(i+1==count),"each chord order waits for its complete final button");
            }
            for(unsigned frame=0;frame<120;++frame) {
                const auto request=intent.sample(state.keys(bindings));
                check(request.requested&&!request.fresh,"held complete chord retries without inventing fresh presses");
            }
            intent.blockUntilRelease();
            check(!intent.sample(state.keys(bindings)).requested,"detachment blocks reacquisition while entry remains held");
            for(unsigned i=0;i<count;++i) {
                state.setButton(order[i],0,.5f);
                check(!intent.sample(state.keys(bindings)).requested&&!intent.waitingForRelease(),"release of any entry member disarms and rearms correctly");
                state.setButton(order[i],1,.5f);
                check(intent.sample(state.keys(bindings)).fresh,"repressing the missing member starts one fresh request");
                intent.blockUntilRelease();
            }
        }while(std::next_permutation(order.begin(),order.begin()+count));
    }
    GamepadState state;GamepadBindings bindings;WallRunEntryGate gate;
    press(state,bindings.entry);gate.begin(state.keys(bindings));state.setStick(0,1,.25f);
    check(!wallInput(gate.filter(state.keys(bindings)),true,true,true).run,"run modifier held during entry cannot start wall running");
    check(!wallInput(gate.filter(state.keys(bindings)),true,true,true).hop,"entry hop cannot become an attachment-frame jump");
    state.setButton(8,0,.5f);gate.filter(state.keys(bindings));state.setButton(8,1,.5f);
    check(wallInput(gate.filter(state.keys(bindings)),false).run,"release and repress enables the independent run modifier");
}
static void stickAndTriggerFiltering() {
    const GamepadBindings bindings;
    struct Direction {float x,y;bool w,a,s,d;};
    for(const auto value:std::array<Direction,8>{{{0,1,true,false,false,false},{1,1,true,false,false,true},
        {1,0,false,false,false,true},{1,-1,false,false,true,true},{0,-1,false,false,true,false},
        {-1,-1,false,true,true,false},{-1,0,false,true,false,false},{-1,1,true,true,false,false}}}) {
        GamepadState state;state.setStick(value.x,value.y,.25f);const auto keys=state.keys(bindings);
        check(keys.w==value.w&&keys.a==value.a&&keys.s==value.s&&keys.d==value.d,"left stick maps all eight directions correctly");
        check(!state.neutral(),"nonzero direction keeps physical state nonneutral");
    }
    GamepadState state;
    for(float x:{-.15f,0.f,.15f})for(float y:{-.15f,0.f,.15f}) {
        state.setStick(x,y,.25f);
        check(emptyActions(state.keys(bindings))&&state.neutral(),"radial deadzone removes centered stick noise");
    }
    state.setStick(.2f,.2f,.25f);
    check(state.keys(bindings).w&&state.keys(bindings).d,"diagonal radius outside the deadzone is retained even when each axis is below it");
    state.setStick(0,1,.25f);state.setStick(.4f,std::sqrt(1.f-.4f*.4f),.25f);
    check(!state.keys(bindings).d&&state.keys(bindings).w,"near-diagonal noise does not prematurely enable a new axis");
    state.setStick(.46f,std::sqrt(1.f-.46f*.46f),.25f);
    check(state.keys(bindings).d&&state.keys(bindings).w,"crossing the enter threshold enables diagonal motion");
    for(unsigned i=0;i<100;++i) {
        const float x=i%2?.44f:.36f;state.setStick(x,std::sqrt(1.f-x*x),.25f);
        check(state.keys(bindings).d&&state.keys(bindings).w,"hysteresis preserves a diagonal across the threshold noise band");
    }
    state.setStick(.34f,std::sqrt(1.f-.34f*.34f),.25f);
    check(!state.keys(bindings).d&&state.keys(bindings).w,"crossing the lower exit threshold returns to a cardinal direction");
    state.setStick(-1,0,.25f);check(state.keys(bindings).a&&!state.keys(bindings).d,"direct opposite input changes sign without conflicting directions");
    for(int x=-20;x<=20;++x)for(int y=-20;y<=20;++y) {
        state.setStick(x*.1f,y*.1f,.25f);const auto keys=state.keys(bindings);
        check(!(keys.a&&keys.d)&&!(keys.w&&keys.s),"no stick position creates opposing directions simultaneously");
        check(!keys.letGo,"stick movement cannot manufacture the drop button");
    }
    state.setStick(0,0,.25f);
    for(unsigned index=0;index<16;++index) {
        state.setButton(index,.49f,.5f);
        check(state.held(index)==(index<14),"only triggers use the configurable analog threshold");
        state.setButton(index,.5f,.5f);check(state.held(index),"trigger threshold includes the exact configured boundary");
        state.setButton(index,0,.5f);check(!state.held(index),"button release clears its held state");
    }
    state.setButton(16,1,.5f);state.setButton(std::numeric_limits<unsigned>::max(),1,.5f);
    check(state.neutral()&&!state.held(16)&&!state.held(std::numeric_limits<unsigned>::max()),"out-of-range button indices cannot corrupt state");
    for(float invalid:{std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()}) {
        state.setStick(1,1,.25f);state.setStick(invalid,1,.25f);
        check(state.neutral()&&emptyActions(state.keys(bindings)),"nonfinite horizontal axis clears stick output");
        state.setStick(1,invalid,.25f);check(state.neutral(),"nonfinite vertical axis clears stick output");
        state.setButton(13,1,.5f);state.setButton(13,invalid,.5f);check(!state.held(13),"nonfinite button values cannot retain an action");
        state.setButton(14,.49f,invalid);check(!state.held(14),"nonfinite trigger threshold uses the safe default");
        state.setStick(.2f,0,invalid);check(state.neutral(),"nonfinite deadzone uses the safe default");
    }
    state.setButton(14,.95f,.99f);check(state.held(14),"trigger thresholds use the sanitized upper bound");
    state.setButton(14,-1,.5f);check(!state.held(14),"negative trigger values release instead of underflowing");
}
static void completeXInputSnapshots() {
    GamepadSettings settings;
    GamepadState state;
    for(unsigned bit=0;bit<16;++bit) {
        state.sampleXInput(static_cast<std::uint16_t>(1u<<bit),0,0,0,0,settings);
        const unsigned expected=bit<10?bit:bit>=12?bit-2:16;
        for(unsigned index=0;index<16;++index)
            check(state.held(index)==(index==expected),"XInput digital masks map to their exact SKSE indices and skip the two reserved gaps");
    }
    state.sampleXInput(0xFFFF,255,255,-32768,32767,settings);
    for(unsigned index=0;index<16;++index)check(state.held(index),"complete full-press snapshot includes all fourteen buttons and both triggers");
    auto keys=state.keys(settings.bindings);
    check(keys.a&&keys.w&&!keys.d&&!keys.s,"signed minimum and positive maximum axes retain the full upper-left diagonal");
    state.sampleXInput(0,0,0,32767,-32768,settings);keys=state.keys(settings.bindings);
    check(keys.d&&keys.s&&!keys.a&&!keys.w&&!keys.space&&!keys.shift&&!keys.entry&&!keys.letGo,
        "the next complete snapshot releases every old button and reverses both axes");
    state.sampleXInput(0,0,0,0,0,settings);
    check(state.neutral()&&emptyActions(state.keys(settings.bindings)),"complete neutral snapshot clears state without requiring release events");
    settings.deadzone=.5f;
    state.sampleXInput(0,0,0,-16384,0,settings);
    check(state.neutral(),"negative half-range uses division by 32768 at the exact radial deadzone boundary");
    state.sampleXInput(0,0,0,-16385,0,settings);
    check(state.keys(settings.bindings).a,"one negative raw step outside the configured deadzone enables movement");
    state.sampleXInput(0,0,0,16383,0,settings);
    check(state.neutral(),"positive raw value below half range remains in the radial deadzone");
    state.sampleXInput(0,0,0,16384,0,settings);
    check(state.keys(settings.bindings).d,"positive half-range uses division by 32767 without a second engine deadzone");
    settings.deadzone=.1f;
    state.sampleXInput(0,0,0,4000,0,settings);
    check(state.keys(settings.bindings).d,"small raw stick input obeys the configured low deadzone directly");
    state.sampleXInput(0,127,128,0,0,settings);
    check(!state.held(14)&&state.held(15),"raw trigger bytes are normalized by 255 before the configured threshold");
    state.sampleXInput(0,128,127,0,0,settings);
    check(state.held(14)&&!state.held(15),"new snapshot replaces previous trigger state independently on both sides");
    settings.triggerThreshold=128.f/255.f;
    state.sampleXInput(0,128,128,0,0,settings);
    check(state.held(14)&&state.held(15),"normalized raw trigger exactly on the threshold is held");
    settings.triggerThreshold=.1f;
    state.sampleXInput(0,25,26,0,0,settings);
    check(!state.held(14)&&state.held(15),"custom low trigger threshold does not inherit the engine trigger filter");
    settings=GamepadSettings{};
    state.sampleXInput(0x8100,0,0,0,0,settings);
    check(state.keys(settings.bindings).entry,"raw LB and Y map to the complete default entry chord");
    state.blockUntilButtonsReleased();
    state.sampleXInput(0x8100,0,0,0,0,settings);
    check(!state.resumeIfButtonsReleased()&&emptyActions(state.keys(settings.bindings)),"complete still-held reconnect snapshot cannot rearm actions");
    state.sampleXInput(0,0,0,0,32767,settings);
    check(state.waitingForButtonsRelease()&&state.resumeIfButtonsReleased()&&!state.neutral()&&state.keys(settings.bindings).w,"complete button release rearms while retaining a held forward stick");
    state.sampleXInput(0,0,0,0,0,settings);
    check(!state.waitingForButtonsRelease()&&state.resumeIfButtonsReleased(),"later neutral snapshot keeps rearmed input available");
    state.sampleXInput(0x8100,0,0,0,0,settings);
    check(state.keys(settings.bindings).entry,"fresh complete chord works after snapshot-verified button release");
    state.reset();state.sampleXInput(0x8000,0,0,0,0,settings);
    check(!state.resumeIfButtonsReleased()&&state.held(13)&&emptyActions(state.keys(settings.bindings)),"disconnect reset cannot treat a held first reconnect sample as neutral");
    state.sampleXInput(0,0,0,0,0,settings);
    check(state.resumeIfButtonsReleased()&&state.neutral(),"later complete release snapshot safely rearms after disconnect");
}
static void suspensionAndDisconnect() {
    GamepadState state;GamepadBindings bindings;
    press(state,bindings.entry);state.setStick(1,1,.25f);state.blockUntilButtonsReleased();
    check(state.waitingForButtonsRelease()&&state.held(8)&&state.held(13)&&emptyActions(state.keys(bindings)),"suspension retains physical state while suppressing all actions");
    check(!state.resumeIfButtonsReleased(),"still-held chord cannot resume after a menu");
    press(state,bindings.entry,false);check(state.resumeIfButtonsReleased()&&!state.waitingForButtonsRelease()&&state.keys(bindings).w&&state.keys(bindings).d,"button release rearms while retaining diagonal stick input");
    press(state,bindings.entry);check(state.keys(bindings).entry,"a new entry works after button release rearming");
    state.reset();
    check(state.neutral()&&state.waitingForButtonsRelease()&&!state.held(8)&&emptyActions(state.keys(bindings)),"disconnect clears all physical and mapped input and blocks reuse");
    state.setStick(0,0,.25f);press(state,bindings.entry);
    check(!state.resumeIfButtonsReleased()&&emptyActions(state.keys(bindings)),"neutral stick received before held reconnect buttons cannot rearm mid-batch");
    press(state,bindings.entry,false);state.setButton(4,1,.5f);
    check(!state.resumeIfButtonsReleased(),"native menu button still held prevents rearming");
    state.setButton(4,0,.5f);check(state.resumeIfButtonsReleased(),"released reconnect buttons can resume");
    press(state,bindings.entry);check(state.keys(bindings).entry,"reconnected fresh chord works after verified button release");
}
static void rearmWithHeldMovement() {
    const GamepadSettings settings;
    for(std::int16_t x:{std::int16_t{0},std::int16_t{32767},std::int16_t{-32768}})for(bool disconnected:{false,true}) {
        GamepadState state;ClimbEntryIntent intent;
        state.sampleXInput(0x8100,0,0,x,32767,settings);
        if(disconnected)state.reset();else state.blockUntilButtonsReleased();
        state.sampleXInput(0x8100,0,0,x,32767,settings);
        check(!state.resumeIfButtonsReleased()&&emptyActions(state.keys(settings.bindings)),"held entry buttons remain blocked after menu suspension or reconnect");
        check(!intent.sample(state.keys(settings.bindings)).requested,"blocked snapshot cannot start attachment");
        state.sampleXInput(0,0,0,x,32767,settings);
        check(state.resumeIfButtonsReleased()&&!state.waitingForButtonsRelease(),"complete released-button snapshot rearms without centering the moving stick");
        const auto moving=state.keys(settings.bindings);
        check(moving.w&&moving.a==(x<0)&&moving.d==(x>0)&&!moving.s&&!moving.entry&&!state.neutral(),"rearming preserves cardinal or diagonal movement without inventing an entry request");
        check(!intent.sample(moving).requested,"held movement alone is not a climb entry");
        state.sampleXInput(0x0100,0,0,x,32767,settings);
        check(!intent.sample(state.keys(settings.bindings)).requested,"partial freshly pressed LB chord cannot start entry");
        state.sampleXInput(0x8100,0,0,x,32767,settings);
        auto request=intent.sample(state.keys(settings.bindings));
        check(request.requested&&request.fresh,"fresh LB and Y enter while the left stick remains held forward or diagonally");
        request=intent.sample(state.keys(settings.bindings));
        check(request.requested&&!request.fresh,"held complete entry retries without another fresh press");
    }
    for(unsigned index=0;index<16;++index) {
        GamepadState state;state.reset();
        const auto bits=index<14?static_cast<std::uint16_t>(1u<<(index<10?index:index+2)):std::uint16_t{};
        state.sampleXInput(bits,index==14?255:0,index==15?255:0,32767,32767,settings);
        check(state.held(index)&&!state.resumeIfButtonsReleased()&&emptyActions(state.keys(settings.bindings)),"every held digital button or trigger blocks resuming despite a moving stick");
        state.sampleXInput(0,0,0,32767,32767,settings);
        check(state.resumeIfButtonsReleased()&&state.keys(settings.bindings).w&&state.keys(settings.bindings).d,"releasing the last button or trigger rearms with diagonal movement intact");
    }
    GamepadState triggers;triggers.reset();
    triggers.sampleXInput(0,128,255,0,32767,settings);
    check(!triggers.resumeIfButtonsReleased(),"held triggers at or above threshold prevent rearming");
    triggers.sampleXInput(0,0,128,0,32767,settings);
    check(!triggers.resumeIfButtonsReleased(),"releasing only one trigger does not clear the other held trigger");
    triggers.sampleXInput(0,0,0,0,32767,settings);
    check(triggers.resumeIfButtonsReleased()&&triggers.keys(settings.bindings).w,"releasing both triggers permits rearming without centering the stick");
}
static void eventOwnership() {
    GamepadBindings bindings;
    for(unsigned index=0;index<16;++index)for(bool attachedAtRelease:{false,true}) {
        GamepadState state;GamepadOwnership ownership;auto before=state;
        state.setButton(index,1,.5f);
        check(!ownership.filter(index,true,false,false,before,state,bindings),"every ground button down remains native");
        check(ownership.nativeDown(index)&&ownership.startedNativeJump(index),"native down history tracks the actual configured jump index");
        before=state;state.setButton(index,0,.5f);
        check(!ownership.filter(index,false,true,attachedAtRelease,before,state,bindings),"native button up always reaches the game even after attachment");
        check(!ownership.startedNativeJump(index),"native jump history ends on button up");
    }
    for(unsigned index:{8u,11u,13u}) {
        GamepadState state;GamepadOwnership ownership;auto before=state;state.setButton(index,1,.5f);
        check(ownership.filter(index,true,false,true,before,state,bindings)&&!ownership.startedNativeJump(index),"attached action down belongs to traversal");
        for(unsigned i=0;i<10;++i)check(ownership.filter(index,false,false,false,state,state,bindings),"held action stays consumed after leaving the wall");
        before=state;state.setButton(index,0,.5f);
        check(ownership.filter(index,false,true,false,before,state,bindings),"traversal-owned release stays consumed after detachment");
        before=state;state.setButton(index,1,.5f);
        check(!ownership.filter(index,true,false,false,before,state,bindings),"a subsequent ground press regains ordinary native behavior");
    }
    for(unsigned index:{0u,4u,5u,10u,12u,15u}) {
        GamepadState state;GamepadOwnership ownership;auto before=state;state.setButton(index,1,.5f);
        check(!ownership.filter(index,true,false,true,before,state,bindings),"unbound and reserved attached buttons retain their native routing");
    }
    bindings.entry=chord("LS+RS");bindings.runModifier=chord("LT+LB");bindings.hop=chord("RB+Y");
    GamepadState state;GamepadOwnership ownership;auto before=state;state.setButton(9,1,.5f);
    check(!ownership.filter(9,true,false,true,before,state,bindings),"partial custom action chord retains native down");
    before=state;state.setButton(13,1,.5f);
    check(ownership.filter(13,true,false,true,before,state,bindings),"custom action chord completion is consumed");
    check(ownership.filter(9,false,false,true,state,state,bindings),"held earlier native member becomes consumed when the chord is active");
    before=state;state.setButton(13,0,.5f);check(ownership.filter(13,false,true,false,before,state,bindings),"owned chord completion releases without native leakage");
    check(ownership.filter(9,false,false,false,state,state,bindings),"broken chord remains owned until its already-held member releases");
    before=state;state.setButton(9,0,.5f);
    check(!ownership.filter(9,false,true,false,before,state,bindings),"earlier native chord member still receives its paired native up");
    bindings=GamepadBindings{};state=GamepadState{};before=state;state.setButton(13,1,.5f);
    check(ownership.filter(13,true,false,true,before,state,bindings),"old hop starts owned before a settings change");
    bindings.hop=chord("X");state.reset();
    check(ownership.filter(13,false,true,false,state,state,bindings),"old binding ownership survives state reset and rebind until release");
    state.resumeIfButtonsReleased();before=state;state.setButton(12,1,.5f);
    check(ownership.filter(12,true,false,true,before,state,bindings),"new custom hop has independent ownership");
    state.reset();
    check(!ownership.filter(12,true,false,false,state,state,bindings),"fresh native down supersedes stale ownership after a lost release");
    check(!ownership.filter(12,false,true,false,state,state,bindings),"replacement native press receives its release");
    before=state;state.setButton(10,1,.5f);ownership.filter(10,true,false,false,before,state,bindings);ownership.reset();
    check(!ownership.startedNativeJump(10)&&!ownership.filter(10,false,true,false,state,state,bindings),"disconnect can fully clear native ownership history");
    check(!ownership.filter(99,true,false,true,state,state,bindings)&&!ownership.startedNativeJump(99),"unknown events never claim native ownership");
}
static void entryWithHeldStickAndExtraButtons() {
    const GamepadSettings settings;
    for(std::int16_t x:{std::int16_t{-32768},std::int16_t{0},std::int16_t{32767}})
        for(std::int16_t y:{std::int16_t{0},std::int16_t{32767}})
            for(unsigned extra:{0u,1u,2u,3u,6u,7u,9u,10u,12u,14u,15u,16u}) {
        GamepadState state;ClimbEntryIntent intent;state.reset();
        state.sampleXInput(0,0,0,x,y,settings);
        check(state.resumeIfButtonsReleased(),"complete released snapshot rearms with a forward or side stick held");
        auto bits=std::uint16_t{0x8100};
        if(extra<14)bits=static_cast<std::uint16_t>(bits|(1u<<(extra<10?extra:extra+2)));
        state.sampleXInput(bits,extra==14?255:0,extra==15?255:0,x,y,settings);
        const auto keys=state.keys(settings.bindings);
        const auto request=intent.sample(keys);
        check(keys.entry&&!keys.s&&!keys.letGo&&approachIntent(keys)&&request.requested&&request.began,
            "snapshot alone starts LB+Y with non-cancel extra buttons or triggers and moving stick without native events");
        for(int frame=0;frame<4;++frame) {
            const auto held=intent.sample(state.keys(settings.bindings));
            check(held.requested&&!held.fresh,"held snapshot retries without requiring button edges or stick neutrality");
        }
    }
    for(bool backward:{false,true}) {
        GamepadState state;ClimbEntryIntent intent;
        state.sampleXInput(backward?0x8100:0xA100,0,0,16000,backward?-32768:32767,settings);
        const auto blocked=state.keys(settings.bindings);
        check(!approachIntent(blocked)&&!intent.sample(blocked).requested&&intent.waitingForRelease(),
            "backward movement or independent B drop prevents a grab-and-immediate-release cycle");
        state.sampleXInput(0x8100,0,0,16000,32767,settings);
        check(!intent.sample(state.keys(settings.bindings)).requested,"clearing only backward or B does not silently rearm the held chord");
        state.sampleXInput(0x0100,0,0,16000,32767,settings);
        check(!intent.sample(state.keys(settings.bindings)).requested&&!intent.waitingForRelease(),"releasing Y rearms with LB and moving stick retained");
        state.sampleXInput(0x8100,0,0,16000,32767,settings);
        check(intent.sample(state.keys(settings.bindings)).fresh,"fresh LB+Y works after deliberate rearming without centering the stick");
    }
}
static void wallActionPriority() {
    GamepadBindings bindings;
    for(unsigned mask=0;mask<32;++mask)for(int x=-1;x<=1;++x)for(int y=-1;y<=1;++y) {
        GamepadState state;state.setStick(static_cast<float>(x),static_cast<float>(y),.25f);
        state.setButton(8,mask&1?1.f:0.f,.5f);state.setButton(13,mask&2?1.f:0.f,.5f);state.setButton(11,1,.5f);
        const auto keys=state.keys(bindings);const auto input=wallInput(keys,mask&2,mask&4,mask&8,mask&16);
        check(keys.letGo&&!(keys.a&&keys.d),"drop remains a separate action without fabricating opposing directions");
        check(input.release&&!input.backDrop&&!input.hop&&!input.run&&!input.mantle&&input.x==0&&input.y==0,"in-place drop has priority over every direction run hop and mantle state");
        check(wallInput(keys,false).release,"held drop remains stable without requiring a hop edge");
    }
    for(bool shifted:{false,true})for(bool wasRunning:{false,true}) {
        GamepadState state;state.setStick(0,1,.25f);state.setButton(13,1,.5f);state.setButton(8,shifted?1.f:0.f,.5f);
        auto input=wallInput(state.keys(bindings),true,true,false,wasRunning);
        check(input.hop==(!shifted&&!wasRunning),"manual hop is disabled during current or just-finished wall running");
        check(!wallInput(state.keys(bindings),true,true,true,wasRunning).hop,"attachment frame cannot turn the held entry hop into a jump");
        state.setStick(0,-1,.25f);input=wallInput(state.keys(bindings),true,true,false,wasRunning);
        check(input.release&&input.backDrop&&!input.hop&&!input.run,"backward hop retains outward departure instead of wall running");
        check(!wallInput(state.keys(bindings),false).release,"backward departure requires a fresh hop edge");
    }
}
int main(){try {
    parsingAndBindings();completeChordsAndRearming();stickAndTriggerFiltering();completeXInputSnapshots();suspensionAndDisconnect();rearmWithHeldMovement();eventOwnership();entryWithHeldStickAndExtraButtons();wallActionPriority();
    std::cout<<"PASS: "<<checks<<" synthetic gamepad binding, direction, suspension, ownership and wall action checks\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
