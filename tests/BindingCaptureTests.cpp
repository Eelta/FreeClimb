#include "BindingCapture.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
static unsigned checks{};
static void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
static BindingCaptureSnapshot keyboard(std::initializer_list<KeyCode> keys={}) {
    BindingCaptureSnapshot snapshot;snapshot.keyboardValid=true;
    for(auto key:keys)snapshot.keyboard.set(key,true);
    return snapshot;
}
static BindingCaptureSnapshot gamepad(GamepadChord keys=0) {
    BindingCaptureSnapshot snapshot;snapshot.gamepadValid=true;snapshot.gamepad=keys;return snapshot;
}
static void arm(BindingCapture& capture,BindingCaptureDevice device) {
    check(capture.begin(device).status==BindingCaptureStatus::waiting,"begin waits for activation release");
    const auto snapshot=device==BindingCaptureDevice::keyboard?keyboard():gamepad();
    check(capture.sample(snapshot).status==BindingCaptureStatus::armed,"complete neutral target snapshot arms capture");
}
static void emptyTerminal(const BindingCaptureResult& result,BindingCaptureStatus status) {
    check(result.status==status&&result.keyboard.count==0&&result.gamepad==0&&result.preview.empty(),"terminal rejection contains no old binding or partial candidate");
}
static void activationAndSnapshots() {
    BindingCapture capture;
    check(!capture.active()&&capture.result().status==BindingCaptureStatus::idle,"new capture is idle");
    check(capture.sample(keyboard({0x1e})).status==BindingCaptureStatus::idle,"snapshots cannot begin capture");
    capture.begin(BindingCaptureDevice::keyboard);
    check(capture.sample(keyboard({0x1c})).status==BindingCaptureStatus::waiting,"held keyboard confirmation is not recorded");
    check(capture.sample({}).status==BindingCaptureStatus::waiting,"missing keyboard snapshot is not neutral");
    auto other=gamepad(gamepadButtonMask(10));other.keyboardValid=true;other.activationHeld=true;
    check(capture.sample(other).status==BindingCaptureStatus::waiting,"gamepad confirmation can retain keyboard activation barrier");
    other.gamepad=0;
    check(capture.sample(other).status==BindingCaptureStatus::waiting,"held mouse activation can retain barrier with neutral devices");
    other.activationHeld=false;
    check(capture.sample(other).status==BindingCaptureStatus::armed&&capture.result().preview.empty(),"releasing activation arms without capturing confirmation");
    auto current=keyboard({0x1e});current.activationHeld=true;
    check(capture.sample(current).preview=="A","activation barrier only gates initial arming");
    current.keyboardValid=false;current.keyboard.reset();
    check(capture.sample(current).status==BindingCaptureStatus::armed&&capture.result().preview=="A","unavailable full snapshot never releases held candidate");
    const auto result=capture.sample(keyboard());
    check(result.status==BindingCaptureStatus::captured&&result.preview=="A"&&!capture.active(),"complete released snapshot commits once");
    check(capture.sample(keyboard({0x01})).preview=="A"&&capture.result().status==BindingCaptureStatus::captured,"finished capture remains stable until explicitly restarted");
    capture.begin(BindingCaptureDevice::gamepad);
    check(capture.result().preview.empty()&&capture.result().keyboard.count==0,"restart clears previous result");
    auto pad=gamepad(gamepadButtonMask(10));pad.activationHeld=true;
    check(capture.sample(pad).status==BindingCaptureStatus::waiting,"gamepad activation button must release");
    pad.gamepad=0;pad.keyboard=keyboard({0x1c}).keyboard;pad.keyboardValid=true;
    check(capture.sample(pad).status==BindingCaptureStatus::waiting,"keyboard confirmation can retain gamepad activation barrier");
    pad.keyboard.reset();pad.activationHeld=false;
    check(capture.sample(pad).status==BindingCaptureStatus::armed,"all activation controls released arms selected gamepad");
    pad.gamepad=gamepadButtonMask(11);capture.sample(pad);pad.gamepad=0;pad.gamepadValid=false;
    check(capture.sample(pad).status==BindingCaptureStatus::armed&&capture.result().preview=="B","missing gamepad snapshot cannot finish recording");
    check(capture.sample(gamepad()).status==BindingCaptureStatus::captured&&capture.result().gamepad==gamepadButtonMask(11),"B is a configurable button");
    capture.reset();check(capture.result().status==BindingCaptureStatus::idle&&!capture.active()&&capture.result().preview.empty(),"reset clears completed capture");
}
static void keyboardChords() {
    std::array<KeyCode,4> order{0x11,0x1e,0x20,0x39};
    do {
        std::array<KeyCode,4> release{0x11,0x1e,0x20,0x39};
        do {
            BindingCapture capture;arm(capture,BindingCaptureDevice::keyboard);auto snapshot=keyboard();
            for(unsigned i=0;i<order.size();++i) {
                snapshot.keyboard.set(order[i],true);const auto& result=capture.sample(snapshot);
                check(result.status==BindingCaptureStatus::armed&&result.keyboard.count==i+1&&!result.preview.empty(),"overlapping keys immediately update preview regardless of press order");
            }
            const auto expected=*parseKeyChord("W+A+D+Space");
            check(capture.result().keyboard==expected,"captured keyboard order is canonical");
            for(unsigned i=0;i<release.size();++i) {
                snapshot.keyboard.set(release[i],false);const auto& result=capture.sample(snapshot);
                check(result.status==(i+1==release.size()?BindingCaptureStatus::captured:BindingCaptureStatus::armed),"commit waits for every recorded key to release");
                check(result.keyboard==expected&&result.preview==serializeKeyChord(expected),"partial release preserves full simultaneous combination");
            }
        } while(std::next_permutation(release.begin(),release.end()));
    } while(std::next_permutation(order.begin(),order.end()));
    for(const auto& named:keyNames)if(named.code<256) {
        BindingCapture capture;arm(capture,BindingCaptureDevice::keyboard);
        capture.sample(keyboard({named.code}));const auto result=capture.sample(keyboard());
        check(result.status==BindingCaptureStatus::captured&&result.keyboard==*parseKeyChord(named.name),"every bindable physical keyboard key is recordable");
    }
    BindingCapture capture;arm(capture,BindingCaptureDevice::keyboard);
    capture.sample(keyboard({0x36,0x1e,0x9d}));
    check(capture.result().preview=="RShift+RCtrl+A"&&capture.result().keyboard==*parseKeyChord("A+RCtrl+RShift"),"physical modifier sides are retained and sorted first");
    arm(capture,BindingCaptureDevice::keyboard);capture.sample(keyboard({0x1e,0x30}));capture.sample(keyboard({0x1e}));
    capture.sample(keyboard({0x1e,0x30,0x2e}));
    check(capture.sample(keyboard()).keyboard==*parseKeyChord("A+B+C"),"additional keys are allowed when a complete simultaneous superset is observed");
}
static void deviceIsolationAndCancellation() {
    BindingCapture capture;arm(capture,BindingCaptureDevice::keyboard);
    auto snapshot=keyboard();snapshot.gamepadValid=true;snapshot.gamepad=gamepadButtonMask(11)|gamepadButtonMask(10);
    check(capture.sample(snapshot).status==BindingCaptureStatus::armed&&capture.result().preview.empty(),"nonselected regular gamepad buttons do not enter or cancel keyboard capture");
    snapshot.keyboard.set(0x1e,true);capture.sample(snapshot);snapshot.keyboard.reset();
    check(capture.sample(snapshot).preview=="A"&&capture.result().gamepad==0,"other device remains excluded from committed keyboard chord");
    arm(capture,BindingCaptureDevice::gamepad);snapshot=gamepad();snapshot.keyboardValid=true;snapshot.keyboard.set(0x1c,true);
    check(capture.sample(snapshot).preview.empty(),"nonselected keyboard key cannot enter gamepad capture");
    snapshot.gamepad=gamepadButtonMask(11);capture.sample(snapshot);snapshot.gamepad=0;
    check(capture.sample(snapshot).status==BindingCaptureStatus::captured&&capture.result().preview=="B","keyboard confirmation is not part of gamepad capture");
    for(auto device:{BindingCaptureDevice::keyboard,BindingCaptureDevice::gamepad}) {
        for(bool armed:{false,true}) {
            capture.begin(device);if(armed)capture.sample(device==BindingCaptureDevice::keyboard?keyboard():gamepad());
            const auto result=capture.sample(keyboard({0x01}));emptyTerminal(result,BindingCaptureStatus::cancelled);
            check(result.error==BindingCaptureError::none&&!capture.active(),"Escape universally cancels even without selected-device snapshot");
            check(capture.sample(device==BindingCaptureDevice::keyboard?keyboard():gamepad()).status==BindingCaptureStatus::cancelled,"release after cancellation cannot commit");
        }
    }
    for(auto device:{BindingCaptureDevice::keyboard,BindingCaptureDevice::gamepad})for(bool armed:{false,true}) {
        capture.begin(device);if(armed)capture.sample(device==BindingCaptureDevice::keyboard?keyboard():gamepad());
        emptyTerminal(capture.sample(gamepad(gamepadButtonMask(5)|gamepadButtonMask(11))),BindingCaptureStatus::cancelled);
        check(!capture.active(),"Back universally cancels even without a selected-device snapshot");
    }
    arm(capture,BindingCaptureDevice::keyboard);snapshot=keyboard({0x1e});snapshot.gamepad=gamepadButtonMask(5);
    check(capture.sample(snapshot).preview=="A","invalid gamepad snapshot cannot fabricate a Back cancellation");
    arm(capture,BindingCaptureDevice::gamepad);snapshot=gamepad(gamepadButtonMask(11));snapshot.keyboard.set(0x01,true);
    check(capture.sample(snapshot).preview=="B","invalid keyboard snapshot cannot fabricate an Escape cancellation");
    arm(capture,BindingCaptureDevice::keyboard);capture.sample(keyboard({0x1e}));emptyTerminal(capture.cancel(),BindingCaptureStatus::cancelled);
    check(capture.sample(keyboard()).status==BindingCaptureStatus::cancelled,"caller focus-loss cancellation prevents pending commit");
    arm(capture,BindingCaptureDevice::gamepad);capture.sample(gamepad(gamepadButtonMask(13)));emptyTerminal(capture.cancel(),BindingCaptureStatus::cancelled);
    check(capture.sample(gamepad()).status==BindingCaptureStatus::cancelled,"caller disconnection cancellation prevents pending commit");
}
static void rejection() {
    BindingCapture capture;
    for(const auto snapshot:{keyboard({0x00}),keyboard({0xff}),keyboard({0x38,0x0f}),keyboard({0x38,0x3e}),keyboard({0x38,0x1d,0xd3})}) {
        arm(capture,BindingCaptureDevice::keyboard);capture.sample(keyboard({0x1e}));const auto result=capture.sample(snapshot);
        emptyTerminal(result,BindingCaptureStatus::error);check(result.error==BindingCaptureError::invalid,"unsupported keys and unsafe operating-system chords reject entire recording");
    }
    arm(capture,BindingCaptureDevice::keyboard);capture.sample(keyboard({0x1e}));const auto tooMany=capture.sample(keyboard({0x1e,0x30,0x2e,0x20,0x12}));
    emptyTerminal(tooMany,BindingCaptureStatus::error);check(tooMany.error==BindingCaptureError::tooMany,"five simultaneous keys never commit an earlier candidate");
    check(capture.sample(keyboard()).status==BindingCaptureStatus::error,"release after error cannot commit old candidate");
    for(bool partial:{false,true}) {
        arm(capture,BindingCaptureDevice::keyboard);capture.sample(keyboard(partial?std::initializer_list<KeyCode>{0x1e,0x30}:std::initializer_list<KeyCode>{0x1e}));
        const auto result=capture.sample(keyboard(partial?std::initializer_list<KeyCode>{0x30,0x2e}:std::initializer_list<KeyCode>{0x30}));
        emptyTerminal(result,BindingCaptureStatus::error);check(result.error==BindingCaptureError::notSimultaneous,"rolling keyboard input cannot accumulate a chord that never overlapped");
    }
    arm(capture,BindingCaptureDevice::gamepad);capture.sample(gamepad(gamepadButtonMask(8)|gamepadButtonMask(13)));
    const auto rolling=capture.sample(gamepad(gamepadButtonMask(13)|gamepadButtonMask(9)));
    emptyTerminal(rolling,BindingCaptureStatus::error);check(rolling.error==BindingCaptureError::notSimultaneous,"rolling gamepad buttons never accumulate independent sequences");
    arm(capture,BindingCaptureDevice::gamepad);const auto start=capture.sample(gamepad(gamepadButtonMask(4)));
    emptyTerminal(start,BindingCaptureStatus::error);check(start.error==BindingCaptureError::invalid,"Start remains unbindable");
}
static void allGamepadMasks() {
    for(unsigned raw=0;raw<=0xffff;++raw) {
        BindingCapture capture;arm(capture,BindingCaptureDevice::gamepad);
        const auto mask=static_cast<GamepadChord>(raw);const auto result=capture.sample(gamepad(mask));
        if(mask&gamepadButtonMask(5))emptyTerminal(result,BindingCaptureStatus::cancelled);
        else if(std::popcount(mask)>4) {
            emptyTerminal(result,BindingCaptureStatus::error);check(result.error==BindingCaptureError::tooMany,"oversized gamepad masks are rejected");
        } else if(mask&gamepadButtonMask(4)) {
            emptyTerminal(result,BindingCaptureStatus::error);check(result.error==BindingCaptureError::invalid,"reserved Start masks are rejected");
        } else if(mask) {
            check(result.status==BindingCaptureStatus::armed&&result.preview==serializeGamepadChord(mask)&&result.keyboard.count==0,"every legal gamepad mask has canonical live preview");
            check(capture.sample(gamepad()).status==BindingCaptureStatus::captured&&capture.result().gamepad==mask,"all legal masks commit exactly after release");
        } else check(result.status==BindingCaptureStatus::armed&&result.preview.empty(),"neutral snapshots never produce an empty binding");
    }
    std::array<unsigned,4> order{0,6,9,12};
    do {
        std::array<unsigned,4> release{0,6,9,12};
        do {
            BindingCapture capture;arm(capture,BindingCaptureDevice::gamepad);GamepadChord held{};
            for(auto index:order) {
                held=static_cast<GamepadChord>(held|gamepadButtonMask(index));
                check(capture.sample(gamepad(held)).gamepad==held,"gamepad preview grows only through overlapping snapshots");
            }
            const auto expected=held;
            for(unsigned i=0;i<release.size();++i) {
                held=static_cast<GamepadChord>(held&~gamepadButtonMask(release[i]));const auto result=capture.sample(gamepad(held));
                check(result.status==(i+1==release.size()?BindingCaptureStatus::captured:BindingCaptureStatus::armed)&&result.gamepad==expected,"every gamepad press and release permutation preserves complete chord");
            }
        } while(std::next_permutation(release.begin(),release.end()));
    } while(std::next_permutation(order.begin(),order.end()));
}
static void pairedOwnership() {
    for(auto device:{BindingCaptureDevice::keyboard,BindingCaptureDevice::gamepad}) {
        const unsigned limit=device==BindingCaptureDevice::keyboard?256u:16u;
        for(unsigned code=0;code<limit;++code) {
            BindingCaptureOwnership ownership;
            check(!ownership.pending()&&!ownership.owns(device,code),"new ownership has no captured input");
            check(!ownership.filter(device,code,true,false,false),"activation down passes before recording");
            check(ownership.filter(device,code,false,false,true)&&!ownership.owns(device,code),"waiting consumes held input without claiming an external down");
            check(!ownership.filter(device,code,false,true,true)&&!ownership.pending(),"activation up is delivered to its external down");
            check(ownership.filter(device,code,true,false,true)&&ownership.pending()&&ownership.owns(device,code),"recording down is consumed and owned");
            check(ownership.filter(device,code,false,false,true),"recorded held input remains consumed");
            check(ownership.filter(device,code,false,false,false)&&ownership.pending(),"cancellation still drains already captured held input");
            check(ownership.filter(device,code,false,true,false)&&!ownership.pending(),"recorded up remains consumed after cancellation");
            check(!ownership.filter(device,code,false,true,false),"a duplicate up never gains ownership");
            ownership.filter(device,code,true,false,true);
            check(!ownership.filter(device,code,true,false,false)&&!ownership.pending(),"fresh native down clears stale ownership from an interrupted capture");
            check(!ownership.filter(device,code,false,false,false)&&!ownership.filter(device,code,false,true,false),"new external press retains its held and up events");
            ownership.filter(device,code,true,false,true);
            check(ownership.filter(device,code,false,true,true)&&!ownership.pending(),"recording down and up are both consumed while active");
            ownership.filter(device,code,true,false,true);ownership.reset();
            check(!ownership.pending()&&!ownership.owns(device,code),"explicit reset releases every owned input");
        }
    }
    BindingCaptureOwnership ownership;
    const auto keyboardDevice=BindingCaptureDevice::keyboard,gamepadDevice=BindingCaptureDevice::gamepad;
    ownership.filter(keyboardDevice,10,true,false,true);ownership.filter(gamepadDevice,10,true,false,true);
    check(ownership.filter(keyboardDevice,10,false,true,false)&&ownership.pending()&&ownership.owns(gamepadDevice,10),"keyboard and gamepad indices cannot alias");
    check(ownership.filter(gamepadDevice,10,false,true,false)&&!ownership.pending(),"pending clears only when both devices finish paired release");
    for(auto device:{keyboardDevice,gamepadDevice}) {
        const auto invalid=device==keyboardDevice?256u:16u;
        for(unsigned code:{invalid,65535u,0xffffffffu}) {
            check(ownership.filter(device,code,true,false,true)&&!ownership.pending()&&!ownership.owns(device,code),"invalid active input is suppressed without indexing owned storage");
            check(!ownership.filter(device,code,true,false,false)&&!ownership.filter(device,code,false,true,false),"invalid inactive input remains native");
        }
    }
    BindingCapture capture;arm(capture,keyboardDevice);
    ownership.filter(keyboardDevice,0x1e,true,false,true);capture.sample(keyboard({0x1e}));capture.cancel();
    check(ownership.pending()&&ownership.filter(keyboardDevice,0x1e,false,true,false),"capture cancellation does not discard pending keyboard release ownership");
    arm(capture,gamepadDevice);ownership.filter(gamepadDevice,11,true,false,true);capture.sample(gamepad(gamepadButtonMask(11)));
    capture.cancel();check(ownership.filter(gamepadDevice,11,false,false,false)&&ownership.filter(gamepadDevice,11,false,true,false),"disconnection cancellation drains captured gamepad input until its real release");
    check(!ownership.pending(),"all cancellation-owned input has drained");
}
int main(){try {
    activationAndSnapshots();keyboardChords();deviceIsolationAndCancellation();rejection();allGamepadMasks();pairedOwnership();
    std::cout<<"PASS: "<<checks<<" binding capture snapshot, overlap, cancellation, isolation and ownership checks\n";
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
