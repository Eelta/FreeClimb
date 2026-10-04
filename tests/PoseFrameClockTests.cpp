#include "PoseFrameClock.h"
#include "PoseBlendEnvelope.h"
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>
using namespace fc;
static void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
static bool near(float a,float b){return std::abs(a-b)<.000001f;}
static void frameBoundaries(){
    PoseFrameClock clock;
    require(clock.sample(100,.02f,.02f,false)==0,"the first observed scene frame establishes time without advancing an unseen pose");
    require(near(clock.sample(120,.02f,.02f,false),.02f),"an adjacent scene frame advances by its elapsed game time");
    for(int pass=0;pass<7;++pass)require(clock.sample(120,.02f,.02f,false)==0,"multiple scene passes in the same engine frame add no time");
    require(near(clock.sample(160,.02f,.02f,false),.04f),"missing one rendered frame retains the actual interval instead of one callback dt");
    require(near(clock.sample(1000,.02f,.02f,false),.05f),"a long render gap recovers by at most fifty milliseconds");
    require(near(clock.sample(1020,.02f,.02f,false),.02f),"a capped gap cannot leak old elapsed time into subsequent frames");
    require(clock.sample(900,.02f,.02f,false)==0,"a backward engine clock resets its origin instead of wrapping into a large advance");
    require(near(clock.sample(920,.02f,.02f,false),.02f),"normal progression resumes from the corrected engine clock");
    clock.clear();require(clock.sample(0xfffffff0u,.02f,.02f,false)==0,"a first sample near counter rollover establishes the current origin");
    require(near(clock.sample(0x10u,.02f,.02f,false),.032f),"unsigned engine-millisecond rollover preserves the real thirty-two millisecond interval");
    require(clock.sample(0x10u,.02f,.02f,false)==0,"duplicate callbacks remain deduplicated immediately after rollover");
}
static void scaledAndSuspended(){
    PoseFrameClock clock;clock.sample(0,.01f,.02f,false);
    require(near(clock.sample(20,.01f,.02f,false),.01f),"half-speed time advances half as much game time between scene frames");
    require(near(clock.sample(40,.04f,.02f,false),.04f),"double-speed time advances twice as much game time between scene frames");
    require(clock.sample(40,.02f,.02f,false)==0,"a multiplier change within the same engine frame does not add a second elapsed interval");
    require(clock.sample(60,0,.02f,false)==0,"a frozen game clock cannot advance a visible pose");
    require(clock.sample(1000,.02f,.02f,false)==0,"resuming after a zero-speed interval does not accumulate the hidden freeze");
    require(near(clock.sample(1020,.02f,.02f,false),.02f),"a resumed game clock advances normally after its new origin");
    require(clock.sample(1040,.02f,.02f,true)==0,"paused scenes do not advance pose time");
    require(clock.sample(5000,.02f,.02f,false)==0,"the first unpaused scene discards time spent paused without callbacks");
    require(near(clock.sample(5020,.02f,.02f,false),.02f),"unpaused time starts from the first resumed scene");
    clock.clear();require(clock.sample(9000,.02f,.02f,false)==0,"reset cannot inherit a previous exit interval");
    require(near(clock.sample(9020,std::numeric_limits<float>::max(),std::numeric_limits<float>::denorm_min(),false),.05f),"an extreme finite multiplier cannot overflow or exceed the output cap");
}
static void invalidInputs(){
    for(float bad:{-1.f,NAN,INFINITY})for(bool invalidGame:{false,true}){
        PoseFrameClock clock;clock.sample(0,.02f,.02f,false);
        require(clock.sample(20,invalidGame?bad:.02f,invalidGame?.02f:bad,false)==0,"nonfinite or negative clock inputs cannot advance pose time");
        require(clock.sample(1000,.02f,.02f,false)==0,"invalid data cannot leave an old interval waiting for the next valid callback");
        require(near(clock.sample(1020,.02f,.02f,false),.02f),"valid adjacent scene time recovers after an invalid sample");
    }
    PoseFrameClock clock;clock.sample(0,.02f,.02f,false);
    require(clock.sample(20,.02f,0,false)==0&&clock.sample(1000,.02f,.02f,false)==0,"a missing real-time denominator resets the time origin without dividing by zero");
}
static std::vector<float> renderCadence(int renderFps,int playerFps,unsigned passes){
    PoseFrameClock clock;std::vector<float> samples;float total=0;unsigned playerUpdates=0;
    for(int frame=0;frame<=renderFps;++frame){
        const auto milliseconds=std::uint32_t(std::lround(1000.0*frame/renderFps));
        while(playerUpdates<unsigned(double(milliseconds)*playerFps/1000.0))++playerUpdates;
        float current=0;
        for(unsigned pass=0;pass<passes;++pass)current+=clock.sample(milliseconds,1.f/renderFps,1.f/renderFps,false);
        total+=current;samples.push_back(current);
    }
    require(playerUpdates==unsigned(playerFps)&&near(total,1.f),"scene time covers one game second independently of lower-frequency actor updates");
    return samples;
}
static void acknowledgedExit(unsigned passes){
    PoseFrameClock clock;PoseBlendEnvelope envelope;std::uint32_t applied=0;
    envelope.beginExit(applied,.12f);bool native=false;unsigned pending=0;
    for(std::uint32_t milliseconds=0;milliseconds<500&&!native;milliseconds+=17){
        const float elapsed=clock.sample(milliseconds,.017f,.017f,false);envelope.advanceExit(applied,elapsed);
        const float weight=envelope.weight();
        if(milliseconds==0)require(weight==1,"the first displayed exit frame retains its exact source pose");
        if(weight==0){
            require(!envelope.exitComplete(applied),"reaching the endpoint cannot confirm the current undrawn native output");
            native=true;pending=1;
        }
        for(unsigned pass=0;pass<passes;++pass){
            const float duplicateElapsed=clock.sample(milliseconds,.017f,.017f,false);envelope.advanceExit(applied,duplicateElapsed);
            require(duplicateElapsed==0&&envelope.weight()==weight,"repeated scene callbacks cannot advance the current exit weight");
            envelope.acknowledgeExit(native);++applied;
        }
    }
    require(native&&envelope.exitComplete(applied)&&pending==1,"confirmed native output retains ownership until all pending scene passes finish");
    for(std::uint32_t milliseconds:{600u,620u,640u})envelope.advanceExit(applied,clock.sample(milliseconds,.02f,.02f,false));
    require(envelope.exitComplete(applied),"later unique engine frames cannot swallow an acknowledged native endpoint");
    envelope.acknowledgeExit(false);++applied;--pending;
    require(!envelope.exitComplete(applied)&&pending==0,"late weighted scene completion revokes native confirmation even after the output clock finishes");
    envelope.acknowledgeExit(true);++applied;
    require(envelope.exitComplete(applied)&&pending==0,"a newly confirmed native pass permits cleanup after the old scene pass drains");
}
static void failedSceneOutputs(){
    PoseFrameClock clock;PoseBlendEnvelope envelope;std::uint32_t applied=0;envelope.beginExit(applied);
    envelope.advanceExit(applied,clock.sample(0,.02f,.02f,false));++applied;
    envelope.advanceExit(applied,clock.sample(20,.02f,.02f,false));
    require(near(envelope.elapsedExitSeconds(),.02f),"the first confirmed scene output starts the visible fade clock");
    for(std::uint32_t milliseconds=40;milliseconds<=1000;milliseconds+=20){
        envelope.advanceExit(applied,clock.sample(milliseconds,.02f,.02f,false));
        require(near(envelope.elapsedExitSeconds(),.02f),"unique scene frames with failed propagation cannot advance beyond the last acknowledged pose phase");
    }
    ++applied;envelope.advanceExit(applied,clock.sample(1020,.02f,.02f,false));
    require(near(envelope.elapsedExitSeconds(),.07f),"recovering successful propagation consumes at most fifty milliseconds of pending scene time");
    ++applied;envelope.advanceExit(applied,clock.sample(1040,.02f,.02f,false));
    require(near(envelope.elapsedExitSeconds(),.09f),"discarded failed-output time cannot leak into the next valid scene frame");
    ++applied;envelope.advanceExit(applied,clock.sample(1060,NAN,.02f,false));
    envelope.advanceExit(applied,clock.sample(2000,.02f,.02f,false));
    require(near(envelope.elapsedExitSeconds(),.09f),"invalid scene time and its first resumed frame cannot consume a pending output acknowledgement");
    envelope.advanceExit(applied,clock.sample(2020,.02f,.02f,false));
    require(near(envelope.elapsedExitSeconds(),.11f),"a later valid adjacent scene frame consumes the preserved acknowledgement once");
}
int main(){try{
    frameBoundaries();scaledAndSuspended();invalidInputs();
    for(int fps:{30,40,60,120})require(renderCadence(fps,18,1)==renderCadence(fps,60,7),"one or seven scene passes produce identical game-time samples despite different actor update cadence");
    acknowledgedExit(1);acknowledgedExit(7);failedSceneOutputs();
    std::cout<<"PASS scene frame clock: sparse actor updates, duplicate render passes, time scaling, stalls, suspension, rollover and terminal output acknowledgement\n";return 0;
}catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}}
