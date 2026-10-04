#include "ExitPoseTrace.h"
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <type_traits>
using namespace fc;
static unsigned checks{};
static void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
static bool near(float a,float b){return std::abs(a-b)<.00001f;}
static ExitPoseTrace::Frame frame(unsigned value,bool terminal=false,bool timer=true) {
    ExitPoseTrace::Frame result;
    result.wallMs=value*17;result.nativeFrame=value;result.applied=value*3;result.stamp=100+value*17;
    result.delta=.016f;result.realDelta=.017f;result.clockDt=.016f;result.seconds=value*.016f;
    result.weight=.7f;result.recovery=.3f;result.pass=1;result.timer=timer;result.stable=true;result.terminal=terminal;
    return result;
}
static Pose fixture(float phase=0) {
    Pose result(99);
    for(std::size_t i=0;i<result.size();++i)result[i]={{float(i)+phase,float(i)*.5f,-float(i)},Quat::axis({1,2,3},phase),{1.1f,.9f,1.f}};
    return result;
}
static void disabledAndReadOnly() {
    ExitPoseTrace trace;auto native=fixture(),output=fixture(.2f);
    const auto initialNative=native,initialOutput=output;
    check(!trace.capture(frame(0),native,output).valid,"disabled tracing cannot take a pose snapshot");
    check(!trace.finish()&&trace.samples().empty(),"disabled tracing cannot emit a summary");
    trace.reset(true);
    const auto sample=trace.capture(frame(0),native,output);
    check(sample.valid&&trace.samples().empty(),"capture is a deferred pre-update snapshot rather than an acknowledged output");
    check(trace.commit(sample,0,0),"a matching post-update observation commits the first frame");
    check(!trace.samples()[0].previous&&trace.samples()[0].changes[0].nativeAngle==0,"first frame has no fabricated pose velocity");
    check(std::memcmp(native.data(),initialNative.data(),99*sizeof(Transform))==0&&std::memcmp(output.data(),initialOutput.data(),99*sizeof(Transform))==0,"capture and commit leave every input pose byte unchanged");
    check(!trace.capture(frame(1),Pose(98),output).valid&&!trace.capture(frame(1),native,Pose(100)).valid,"incomplete or oversized rigs do not produce misleading body samples");
    trace.reset(false);
    check(!trace.commit(sample,0,0)&&trace.samples().empty(),"turning diagnostics off discards pending snapshots");
}
static void frameAndTerminalDeduplication() {
    for(bool timer:{false,true}) {
        ExitPoseTrace trace;trace.reset(true);
        auto native=fixture(),output=fixture(.1f);
        const auto initial=frame(0,false,timer);
        check(trace.commit(trace.capture(initial,native,output),0,0),"first available scene pass is recorded");
        auto duplicate=initial;duplicate.applied+=5;duplicate.pass=2;duplicate.wallMs+=1;
        if(timer)++duplicate.nativeFrame;else ++duplicate.stamp;
        check(!trace.commit(trace.capture(duplicate,fixture(.6f),output),0,0),"multiple passes use the selected scene clock rather than callback or unrelated clock counters");
        auto next=frame(1,false,timer);
        auto snapshot=trace.capture(next,fixture(.2f),fixture(.4f));
        check(trace.commit(snapshot,2,3,.1f,2.f,.2f,4.f),"post-update audit metrics stay attached to their own captured pose");
        const auto& record=trace.samples()[1];
        check(record.frame.stamp==next.stamp&&record.frame.nativeFrame==next.nativeFrame&&record.frame.clockDt==next.clockDt&&record.overwritten==2&&record.worldMismatches==3,"captured time and callback identity are not taken from a later update");
        check(record.previous&&record.previousWallMs==initial.wallMs&&near(record.changes[0].nativeAngle,.2f)&&near(record.changes[0].nativeDistance,.2f)&&near(record.changes[0].outputAngle,.3f),"differences use the previous distinct scene frame");
        next.terminal=true;
        check(trace.commit(trace.capture(next,fixture(.25f),fixture(.45f)),0,1),"a terminal pass replaces an earlier weighted pass in the same frame");
        check(trace.samples().size()==2&&trace.frames()==2&&near(trace.samples()[1].changes[0].nativeAngle,.25f),"terminal replacement is compared with the preceding distinct frame");
        check(trace.commit(trace.capture(next,fixture(.3f),fixture(.5f)),0,0),"a validated terminal pass replaces a failed same-frame propagation audit");
        check(!trace.commit(trace.capture(next,native,output),0,0),"repeated validated terminal callbacks do not append duplicate records");
        check(!trace.commit(trace.capture(initial,native,output),0,0),"an older concurrent callback cannot replace newer scene evidence");
        check(trace.finish()&&!trace.finish(),"summary is emitted once per exit even if terminal propagation repeats");
        check(!trace.capture(frame(2),native,output).valid,"finished traces stop sampling");
    }
}
static void boundedStorageAndReset() {
    static_assert(std::is_trivially_destructible_v<ExitPoseTrace>);
    static_assert(sizeof(ExitPoseTrace)<16384);
    ExitPoseTrace trace;trace.reset(true);
    auto native=fixture(),output=fixture();
    for(unsigned i=0;i<1000;++i) {
        const auto sample=trace.capture(frame(i),native,output);
        check(trace.commit(sample,i%3,i%2),"unique frame records remain accepted after the retention limit");
        check(trace.samples().size()<=ExitPoseTrace::capacity,"sample memory remains bounded during a stalled exit");
    }
    check(trace.frames()==1000&&trace.samples().size()==24,"total observed frames and retained records are distinguished");
    check(trace.samples()[0].frame.stamp==frame(0).stamp&&trace.samples()[22].frame.stamp==frame(22).stamp&&trace.samples()[23].frame.stamp==frame(999).stamp,"overflow retains the first twenty-three records and the latest record");
    const auto end=trace.capture(frame(1000,true),native,output);
    check(trace.commit(end,0,0)&&trace.samples().back().frame.terminal,"terminal evidence is retained even after capacity is exhausted");
    check(trace.samples().back().previousWallMs==frame(999).wallMs,"retained tail identifies its actual preceding sample interval");
    trace.reset(true);
    check(!trace.commit(end,0,0)&&trace.samples().empty(),"a new exit rejects a pending sample from the previous exit");
    check(trace.finish()&&!trace.finish(),"a callback timeout can emit one empty summary");
    trace.reset(true);
    auto first=frame(1);first.paused=true;first.stable=false;first.delta=0;first.clockDt=0;
    auto sample=trace.capture(first,native,output);
    check(trace.commit(sample,0,0)&&trace.samples()[0].frame.paused&&!trace.samples()[0].frame.stable&&trace.samples()[0].frame.clockDt==0,"invalid or paused clock evidence is retained without advancing any pose");
}
int main()try {
    disabledAndReadOnly();frameAndTerminalDeduplication();boundedStorageAndReset();
    std::cout<<"PASS ExitPoseTrace: "<<checks<<" checks for bounded deferred scene samples, terminal retention, deduplication, reset, disabled tracing and unchanged poses\n";
    return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
