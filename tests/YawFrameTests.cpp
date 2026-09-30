#include "YawFrame.h"
#include "PoseHandoff.h"
#include "PoseBlendEnvelope.h"
#include <iostream>
#include <stdexcept>

using namespace fc;
static void check(bool ok,const char* reason) {if(!ok)throw std::runtime_error(reason);}
static void same(Transform a,Transform b,const char* reason) {
    check((a.t-b.t).length()<.001f&&angleBetween(a.q,b.q)<.001f&&
        (a.s-b.s).length()<.0001f,reason);
}
static Transform parentAt(float yaw) {
    return {{19,-27,41},Quat::axis({0,0,1},-yaw),{1.03f,1.03f,1.03f}};
}
static void cylinderFacing() {
    constexpr float pi=3.14159265359f;
    float previous=0;
    for(int step=0;step<=720;++step) {
        const float angle=float(step)*2*pi/360;
        const auto yaw=facingWallYaw({-std::sin(angle),-std::cos(angle),.25f});
        check(yaw&&*yaw>=0&&*yaw<2*pi,"facing yaw is normalized throughout two complete cylinder circuits");
        check(std::abs(yawDifference(*yaw,angle))<.00001f,"horizontal wall normal determines the actual facing direction");
        if(step)check(std::abs(yawDifference(*yaw,previous)-2*pi/360)<.00001f,
            "crossing the yaw wrap cannot create a long turn");
        previous=*yaw;
        const auto ahead=Quat::axis({0,0,1},-*yaw).rotate({0,1,0});
        check((ahead-Vec{std::sin(angle),std::cos(angle),0}).length()<.00001f,
            "Skyrim clockwise yaw and pose rotation have matching signs");
    }
    check(!facingWallYaw({0,0,1})&&!facingWallYaw({NAN,0,0}),"a missing horizontal wall direction cannot authorize a facing change");
    check(normalizeYaw(-1e-9f)<2*pi,"a near-zero negative heading cannot round outside the normalized range");
    check(std::abs(yawDifference(.01f,2*pi-.01f)-.02f)<.00001f,"shortest actor rotation survives the zero crossing");
}
static void delayedParentFrame() {
    const Transform authored{{-7,16,4},
        Quat::axis({0,1,0},.48f)*Quat::axis({1,0,0},1.5707963f),{1,1,1}};
    for(float desired:{0.f,.3f,1.4f,3.2f,6.27f})for(float delayed:{0.f,1.2f,5.9f}) {
        auto parent=parentAt(delayed);

        parent.q=(parent.q*Quat::axis({1,.3f,0},.17f)).unit();
        const WallYawFrame frame(parent.q,desired);
        Transform expectedParent=parent;expectedParent.q=Quat::axis({0,0,1},-desired);
        const auto actual=compose(parent,frame.toParent(authored));
        const auto expected=compose(expectedParent,authored);
        same(actual,expected,"a delayed scene parent cannot rotate a wall run away from the current wall");
        const auto rendered=frameYaw(actual.q*authored.q.inverse());
        check(rendered&&std::abs(yawDifference(*rendered,desired))<.00001f,
            "render diagnostics recover the wall frame even when the run pose is pitched vertically");
        const Transform native{{3,-2,6},Quat::axis({1,2,3},.79f),{1.12f,1.12f,1.12f}};
        same(frame.toParent(frame.toWall(native)),native,
            "a live native root passes through wall-space composition without translation or rotation changes");
    }
}
static void releasingWhileParentTurns() {
    const float consumedYaw=1.1f;
    const auto parentA=parentAt(.8f),parentB=parentAt(1.25f);
    const WallYawFrame frameA(parentA.q,consumedYaw),frameB(parentB.q,consumedYaw);
    Pose nativeA(99),nativeB(99),authored(99);
    nativeA[0]={{3,5,1},Quat::axis({1,0,0},.11f),{1,1,1}};
    nativeB[0]={{-4,9,7},Quat::axis({0,1,0},-.3f),{1,1,1}};
    authored[0]={{6,18,12},Quat::axis({1,0,0},1.1f),{1,1,1}};
    authored[20].q=Quat::axis({0,0,1},.65f);
    const auto actualNativeB=nativeB;
    nativeA[0]=frameA.toWall(nativeA[0]);nativeB[0]=frameB.toWall(nativeB[0]);
    PoseHandoff handoff;
    const auto shown=handoff.evaluate(nativeA,authored,.62f);
    handoff.consumed(shown);
    auto unpublished=authored;unpublished[0].q=Quat::axis({0,0,1},2.1f);
    handoff.evaluate(nativeB,unpublished,1);
    check(handoff.beginExit(),"a displayed partial pose is a valid exit source");
    const auto firstExit=handoff.evaluate(nativeB,unpublished,1);
    same(compose(parentB,frameB.toParent(firstExit.pose[0])),
        compose(parentA,frameA.toParent(shown.pose[0])),
        "release retains the consumed source's world orientation when the parent catches up, without adopting an unseen publication");
    const auto finished=handoff.evaluate(nativeB,unpublished,0);
    same(frameB.toParent(finished.pose[0]),actualNativeB[0],
        "fall exit reaches the current native root rather than the entry or release snapshot");
    handoff.clear();
    const auto recovered=handoff.evaluate(nativeB,authored,1,1);handoff.consumed(recovered);
    check(handoff.beginExit()&&handoff.exitContribution()==0,"fully consumed top recovery has no residual custom contribution");
    auto nextNative=nativeB;nextNative[0].t=nextNative[0].t+Vec{8,0,0};
    const auto continued=handoff.evaluate(nextNative,authored,1);
    same(frameB.toParent(continued.pose[0]),frameB.toParent(nextNative[0]),
        "completed top-out keeps live native movement while using the same coordinate conversion");
}
static void fallExitEnvelope() {
    PoseBlendEnvelope fall,ordinary;
    fall.beginExit(20,PoseBlendEnvelope::fallExitSeconds);ordinary.beginExit(20);
    for(int i=0;i<200;++i)fall.advanceExit(20,.01f);
    check(fall.weight()==1,"the shorter fall fade still waits for actual rendered output");
    float previous=1,firstChange=0,lastChange=0;
    for(std::uint32_t i=1;i<=16;++i) {
        fall.advanceExit(20+i,0);
        check(fall.weight()==previous,"zero time cannot consume a pending fall output acknowledgement");
        fall.advanceExit(20+i,.01f);ordinary.advanceExit(20+i,.01f);
        const float current=fall.weight();
        check(current<=previous+.000001f&&current>=0,"fall handoff fades monotonically");
        if(i==1)firstChange=previous-current;
        if(i==16)lastChange=previous-current;
        previous=current;
    }
    check(fall.weight()==0&&ordinary.weight()>.1f,"back-jump handoff ends in 0.16 seconds while ordinary exits retain their existing duration");
    check(firstChange<.003f&&lastChange<.003f,"the faster fall handoff still eases its first and last displayed frames");
}
int main() {try {
    cylinderFacing();delayedParentFrame();releasingWhileParentTurns();fallExitEnvelope();
    std::cout<<"PASS: cylinder yaw, delayed parent frame, native round trip, consumed-source release, acknowledged fall exit\n";
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
