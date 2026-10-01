#include "Core.h"
#include "SearchRetry.h"
#include <iostream>
#include <limits>
#include <stdexcept>
#include <type_traits>
using namespace fc;
static unsigned checks{};
static void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
static bool same(Vec a,Vec b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
static void invalidation() {
    SearchRetry<Vec> retry;
    const Vec p{0,0,0},n{0,-1,0},intent{0,1,0};
    check(retry.ready(p,n,intent,0),"new retry state cannot suppress a search");
    check(!retry.valid&&retry.remaining==0&&!retry.standingPath&&retry.mode==0,"default state contains no retained search history");
    retry.defer(p,n,intent,3,.2f,true);
    check(retry.valid&&retry.remaining==.2f&&retry.standingPath&&retry.mode==3&&
        same(retry.position,p)&&same(retry.normal,n)&&same(retry.input,intent),"defer retains the exact failed-search context and standing path flag");
    check(!retry.ready(p,n,intent,3),"unchanged failed search waits for its retry interval");
    check(!retry.ready({.25f,0,0},n,intent,3),"position threshold includes its exact boundary");
    check(retry.ready({.2501f,0,0},n,intent,3),"position outside the threshold immediately permits a fresh search");
    check(retry.ready({.18f,.18f,0},n,intent,3),"diagonal displacement uses vector distance rather than separate axis thresholds");
    check(!retry.ready(p,{.01f,-1,0},intent,3),"normal threshold includes its exact boundary");
    check(retry.ready(p,{.0101f,-1,0},intent,3),"changed wall normal immediately invalidates a failed search");
    check(!retry.ready(p,n,{.01f,1,0},3),"input threshold includes its exact boundary");
    check(retry.ready(p,n,{.0101f,1,0},3),"changed input immediately invalidates a failed search");
    check(retry.ready(p,n,{0,-1,0},3)&&retry.ready(p,n,{1,0,0},3),"reversal and lateral direction changes do not inherit the old retry delay");
    check(retry.ready(p,n,intent,4),"changing traversal mode immediately permits a search");
    check(!retry.ready(p,n,intent,3)&&retry.remaining==.2f,"readiness queries do not mutate the retained search context");
    retry.defer({4,5,6},{1,0,0},{-1,0,0},7,.12f);
    check(!retry.standingPath&&retry.mode==7&&same(retry.position,{4,5,6})&&
        !retry.ready({4,5,6},{1,0,0},{-1,0,0},7),"new failure replaces context and clears the optional standing flag by default");
}
static void expirationAndClamping() {
    const Vec p{1,2,3},n{0,-1,0},intent{0,1,0};
    SearchRetry<Vec> retry;
    retry.defer(p,n,intent,0,.12f,true);
    retry.tick(.05f);retry.tick(.05f);
    check(retry.remaining>0&&!retry.ready(p,n,intent,0)&&retry.standingPath,"retry remains deferred until its complete interval elapses");
    retry.tick(.05f);
    check(retry.remaining==0&&retry.ready(p,n,intent,0)&&retry.standingPath,"expiration permits a fresh search without losing the standing-path history");
    retry.tick(20);
    check(retry.remaining==0,"expired timers cannot become negative");
    retry.defer(p,n,intent,0,10);
    check(retry.remaining==.25f,"failed-search deferral cannot exceed a quarter second");
    retry.tick(10);
    check(std::abs(retry.remaining-.2f)<.000001f&&!retry.ready(p,n,intent,0),"large frame duration advances at most fifty milliseconds");
    const float previous=retry.remaining;
    for(float dt:{0.f,-.1f,std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()}) {
        retry.tick(dt);
        check(retry.remaining==previous,"zero negative and nonfinite frame durations never age the timer");
    }
    for(float seconds:{0.f,-1.f}) {
        retry.defer(p,n,intent,0,seconds);
        check(retry.remaining==0&&retry.ready(p,n,intent,0),"nonpositive retry durations cannot suppress a search");
    }
    for(int fps:{30,60,120}) {
        retry.defer(p,n,intent,0,.2f);
        float elapsed=0;
        while(!retry.ready(p,n,intent,0)&&elapsed<.3f) {retry.tick(1.f/fps);elapsed+=1.f/fps;}
        check(retry.ready(p,n,intent,0)&&elapsed>=.2f-.000001f&&elapsed<=.2f+1.f/fps+.000001f,
            "failure retries expire at the same elapsed interval across supported frame rates");
    }
}
static void malformedInputs() {
    const Vec p{1,2,3},n{0,-1,0},intent{0,1,0};
    for(float value:{std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()}) {
        const Vec invalid{value,0,0};
        SearchRetry<Vec> retry;retry.defer(p,n,intent,0,.2f);
        check(retry.ready(invalid,n,intent,0)&&retry.ready(p,invalid,intent,0)&&retry.ready(p,n,invalid,0),
            "invalid incoming position normal or input permits safe reevaluation instead of suppressing it");
        retry.position=invalid;check(retry.ready(p,n,intent,0),"invalid retained position never blocks a safe retry");
        retry.defer(p,n,intent,0,.2f);retry.normal=invalid;
        check(retry.ready(p,n,intent,0),"invalid retained normal never blocks a safe retry");
        retry.defer(p,n,intent,0,.2f);retry.input=invalid;
        check(retry.ready(p,n,intent,0),"invalid retained direction never blocks a safe retry");
        retry.defer(p,n,intent,0,.2f);retry.remaining=value;
        check(retry.ready(p,n,intent,0),"nonfinite retained duration never creates a permanent failure cache");
        retry.defer(p,n,intent,0,value,true);
        check(!retry.valid&&retry.remaining==0&&retry.ready(p,n,intent,0)&&retry.standingPath,
            "invalid requested duration leaves no active delay while retaining reported standing-path state");
        for(unsigned field=0;field<3;++field) {
            retry.defer(field==0?invalid:p,field==1?invalid:n,field==2?invalid:intent,0,.2f);
            check(!retry.valid&&retry.remaining==0&&retry.ready(p,n,intent,0),"malformed failed-search context cannot defer later valid inputs");
        }
    }
    SearchRetry<Vec> retry;retry.defer(p,n,intent,0,.2f);retry.valid=false;
    check(retry.ready(p,n,intent,0),"invalidated history cannot suppress a search even with positive remaining time");
}
static void resetState() {
    SearchRetry<Vec> retry;retry.defer({1,2,3},{0,-1,0},{1,1,0},9,.2f,true);retry.reset();
    check(retry.remaining==0&&same(retry.position,{})&&same(retry.normal,{})&&same(retry.input,{})&&
        retry.mode==0&&!retry.valid&&!retry.standingPath,"reset clears every serialized retry field");
    check(retry.ready({1,2,3},{0,-1,0},{1,1,0},9),"reset immediately rearms the former search context");
}
static_assert(std::is_trivially_copyable_v<SearchRetry<Vec>>);
int main(){try {
    invalidation();expirationAndClamping();malformedInputs();resetState();
    std::cout<<"PASS: "<<checks<<" failed-search retry timing, context invalidation and safety checks\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
