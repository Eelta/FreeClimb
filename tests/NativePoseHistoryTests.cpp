#include "NativePoseHistory.h"
#include "PoseHandoff.h"
#include <bit>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace fc;
static unsigned checks{};
static void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
static bool exact(float a,float b){return std::bit_cast<std::uint32_t>(a)==std::bit_cast<std::uint32_t>(b);}
static bool exact(Vec a,Vec b){return exact(a.x,b.x)&&exact(a.y,b.y)&&exact(a.z,b.z);}
static bool exact(Quat a,Quat b){return exact(a.x,b.x)&&exact(a.y,b.y)&&exact(a.z,b.z)&&exact(a.w,b.w);}
static bool exact(Transform a,Transform b){return exact(a.t,b.t)&&exact(a.q,b.q)&&exact(a.s,b.s);}
static bool near(Transform a,Transform b){return (a.t-b.t).length()<.001f&&angleBetween(a.q,b.q)<.001f&&(a.s-b.s).length()<.00001f;}
static Pose fixture(float phase=0) {
    Pose result(99);
    for(std::size_t i=0;i<result.size();++i) {
        const float n=float(i);
        result[i]={{n*.1f+phase,-n*.07f,n*.13f},Quat::axis({1,2,-3},phase+n*.003f),{1.1f,.95f,1.04f}};
    }
    result[0].t={3+phase,-7,11};result[21].t.x=-0.f;
    return result;
}
static void unchangedLocals(const Pose& actual,const Pose& native) {
    check(actual.size()==99,"history always returns the complete rig");
    for(std::size_t i=1;i<99;++i)check(exact(actual[i],native[i]),"effective non-root locals retain every float bit");
    check(exact(actual[0].s,native[0].s),"root scale stays unchanged across coordinate conversion");
}
static void timestamps() {
    NativePoseHistory history;
    check(!history.seed(0,0),"empty history cannot seed a handoff");
    const auto a=fixture(),b=fixture(.1f),c=fixture(.2f);
    check(history.capture(a,{},0),"timestamp zero is a valid first sample");
    auto seed=history.seed(0,0);check(seed&&seed->seconds==0,"one sample seeds without fabricated velocity");
    check(near(seed->current[0],a[0])&&near(seed->older[0],a[0]),"one sample supplies itself as current and older");
    unchangedLocals(seed->current,a);unchangedLocals(seed->older,a);
    check(history.seed(0,250).has_value()&&!history.seed(0,251),"freshness includes250ms and excludes251ms");
    check(history.capture(b,{},16),"later sample advances history");
    seed=history.seed(0,16);
    check(seed&&std::abs(seed->seconds-.016f)<1e-7f&&near(seed->current[0],b[0])&&near(seed->older[0],a[0]),"two frames retain their actual millisecond interval");
    check(history.capture(c,{},16),"same-millisecond current sample may be replaced");
    seed=history.seed(0,16);
    check(seed&&std::abs(seed->seconds-.016f)<1e-7f&&near(seed->current[0],c[0])&&near(seed->older[0],a[0]),"same-time replacement never pushes current into older");
    check(!history.capture(a,{},15)&&!history.seed(0,15),"reverse capture and reverse seed timestamps are rejected");
    seed=history.seed(0,16);check(seed&&near(seed->current[0],c[0])&&near(seed->older[0],a[0]),"rejected reverse time preserves the last valid pair");
    check(history.capture(b,{},266),"exactly250ms capture interval is retained");
    seed=history.seed(0,266);check(seed&&seed->seconds==.25f&&near(seed->older[0],c[0]),"maximum accepted interval keeps the real older pose");
    check(history.capture(a,{},517),"a capture after a gap still records the new valid pose");
    seed=history.seed(0,517);check(seed&&seed->seconds==0&&near(seed->older[0],a[0]),"gap greater than250ms retires old velocity history");
    history.clear();check(!history.seed(0,517),"binding reset removes every preparatory sample");
    check(history.capture(c,{},1),"after clear a new binding may begin with an earlier clock");
    seed=history.seed(0,1);check(seed&&seed->seconds==0&&near(seed->older[0],c[0]),"new binding cannot inherit another skeleton's older pose");
    history.clear();const auto end=std::numeric_limits<std::uint64_t>::max();
    check(history.capture(a,{},end-300)&&history.capture(b,{},end-100),"high monotonic timestamps do not overflow");
    seed=history.seed(0,end);check(seed&&std::abs(seed->seconds-.2f)<1e-7f,"timestamp arithmetic uses bounded differences near uint64 limit");
    check(!history.seed(0,0)&&!history.capture(c,{},0),"wrapped timestamps cannot masquerade as fresh samples");
}
static void animationFrames() {
    NativePoseHistory history;const auto a=fixture(),b=fixture(.1f),c=fixture(.2f),d=fixture(.3f);
    const auto oldParent=Quat::axis({0,0,1},.2f),currentParent=Quat::axis({1,2,3},.11f);
    check(history.capture(a,oldParent,100,60)&&history.capture(b,currentParent,116,61),"distinct animation frame tokens advance native history");
    const auto before=history.seed(.4f,116);
    check(before&&std::abs(before->seconds-.016f)<1e-7f,"animation-frame interval uses the first captured sample times");
    for(std::uint64_t ms=117;ms<=122;++ms) {
        check(history.capture(c,currentParent,ms,61),"later scene passes in the same animation frame can replace the current pose");
        const auto after=history.seed(.4f,ms);
        check(after&&exact(after->seconds,before->seconds),"cross-millisecond scene passes cannot shrink the animation interval");
        for(std::size_t bone=0;bone<99;++bone)check(exact(after->older[bone],before->older[bone]),"same animation frame cannot evict the real older frame or fabricate velocity");
        unchangedLocals(after->current,c);
    }
    check(!history.capture(d,currentParent,123,60),"a reversed frame token is rejected even if wall-clock time advanced");
    check(!history.capture(d,currentParent,121,61),"a timestamp older than the latest scene pass is rejected within the same frame");
    check(!history.seed(.4f,121),"seeding before the latest captured pass is rejected");
    const auto replaced=history.seed(.4f,122);
    check(replaced&&near(WallYawFrame(currentParent,.4f).toParent(replaced->current[0]),c[0]),"rejected captures retain the newest accepted native pose");
    check(history.seed(.4f,372).has_value()&&!history.seed(.4f,373),"freshness counts from the last valid pass rather than the first pass in that frame");
    check(history.capture(d,currentParent,133,62),"the next genuine animation frame advances after multiple scene passes");
    auto next=history.seed(.4f,133);
    check(next&&std::abs(next->seconds-.017f)<1e-7f,"the next frame interval uses the prior frame's original116ms timestamp");
    for(std::size_t bone=0;bone<99;++bone)check(exact(next->older[bone],replaced->current[bone]),"new animation frame retains the final native pose of the preceding frame");
    check(history.capture(a,{},133,63),"two distinct animation frames may share the same millisecond");
    next=history.seed(0,133);check(next&&next->seconds==0,"same-millisecond distinct frames yield zero velocity interval");
    check(history.capture(b,{},384,64),"a new animation frame after a long gap is accepted");
    next=history.seed(0,384);check(next&&next->seconds==0&&near(next->current[0],next->older[0]),"a long frame gap retires older motion prediction");
    history.clear();check(history.capture(a,{},10,0),"clearing for a binding change also resets frame monotonicity");
    check(history.capture(b,{},20,0),"frame token zero supports repeated passes");
    next=history.seed(0,20);check(next&&next->seconds==0&&near(next->current[0],b[0]),"a repeated initial frame has no manufactured older sample");
}
static void rejectedSamples() {
    NativePoseHistory history;const auto original=fixture();check(history.capture(original,{},100),"invalid-input baseline");
    for(std::size_t bone=0;bone<99;++bone)for(int kind=0;kind<9;++kind) {
        auto invalid=original;
        if(kind==0)invalid[bone].t.x=std::numeric_limits<float>::quiet_NaN();
        if(kind==1)invalid[bone].t.z=std::numeric_limits<float>::infinity();
        if(kind==2)invalid[bone].q.x=std::numeric_limits<float>::quiet_NaN();
        if(kind==3)invalid[bone].q={0,0,0,0};
        if(kind==4)invalid[bone].q={0,0,0,2};
        if(kind==5)invalid[bone].s.x=std::numeric_limits<float>::infinity();
        if(kind==6)invalid[bone].s.y=std::numeric_limits<float>::quiet_NaN();
        if(kind==7)invalid[bone].s.z=0;
        if(kind==8)invalid[bone].s.x=-1;
        check(!history.capture(invalid,{},101),"every malformed root or non-root transform is rejected");
        const auto seed=history.seed(0,100);
        check(seed&&seed->seconds==0&&near(seed->current[0],original[0]),"rejected sample does not partially replace the prior snapshot");
        unchangedLocals(seed->current,original);
    }
    for(std::size_t size:{0u,1u,98u,100u})check(!history.capture(Pose(size),{},101),"only a complete99-bone pose is accepted");
    for(Quat parent:{Quat{0,0,0,0},Quat{0,0,0,2},Quat{0,0,0,std::numeric_limits<float>::infinity()},Quat{std::numeric_limits<float>::quiet_NaN(),0,0,1}})
        check(!history.capture(original,parent,101),"invalid parent rotation cannot enter native history");
    for(float yaw:{std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()})
        check(!history.seed(yaw,100),"invalid target wall frame cannot seed pose output");
    auto copy=history.seed(0,100);copy->current[0].t={1000,2000,3000};copy->older.clear();
    const auto intact=history.seed(0,100);check(intact&&near(intact->current[0],original[0])&&intact->older.size()==99,"returned snapshots cannot mutate retained history");
}
static void finiteAfterConversion() {
    const float large=std::numeric_limits<float>::max()*.75f;
    auto pose=fixture();pose[0].t={large,large,0};
    NativePoseHistory history;
    check(history.capture(pose,{},10,1),"large but finite native translations are representable before rotation");
    check(!history.seed(.785398163f,10),"a wall-frame conversion cannot publish overflowed root translation");
    check(history.seed(0,10).has_value(),"rejecting one wall-frame conversion does not damage valid stored history");
    NativePoseHistory captureOverflow;
    check(!captureOverflow.capture(pose,Quat::axis({0,0,1},.785398163f),10,1),"overflow during parent-world conversion rejects the whole capture");
    check(!captureOverflow.seed(0,10),"overflowing first capture leaves no partial pose");
    NativePoseHistory olderOverflow;
    check(olderOverflow.capture(pose,{},10,1)&&olderOverflow.capture(fixture(),{},26,2),"overflow fixture can retain a finite older native pose");
    check(!olderOverflow.seed(.785398163f,26),"converted older root must also be finite before velocity history can seed entry");
}
static void parentAndWallFrames() {
    constexpr float pi=3.14159265358979323846f;
    NativePoseHistory explicitCase;auto native=fixture();native[0].t={2,3,5};native[0].q={};
    check(explicitCase.capture(native,Quat::axis({0,0,1},pi/2),10),"quarter-turn parent fixture captures");
    const auto world=explicitCase.seed(0,10);
    check(world&&(world->current[0].t-Vec{-3,2,5}).length()<.00001f,"root translation is rotated into world axes without world-position translation");
    const auto opposite=explicitCase.seed(pi/2,10);
    check(opposite&&(opposite->current[0].t-Vec{-2,-3,5}).length()<.00001f,"Skyrim clockwise wall yaw rotates both captured root translation axes consistently");
    for(float yaw:{-7.f,-.4f,0.f,.9f,3.14f,6.27f})for(float oldYaw:{-.2f,1.3f,5.8f}) {
        const auto old=fixture(.03f),current=fixture(.08f);
        const auto parentOld=(Quat::axis({0,0,1},oldYaw)*Quat::axis({1,2,0},.23f)).unit();
        const auto parentCurrent=(Quat::axis({0,0,1},oldYaw+.21f)*Quat::axis({2,-1,0},-.13f)).unit();
        NativePoseHistory history;check(history.capture(old,parentOld,30)&&history.capture(current,parentCurrent,50),"two independently oriented parent frames are captured");
        const auto seed=history.seed(yaw,60);check(seed&&std::abs(seed->seconds-.02f)<1e-7f,"wall conversion preserves sample timing");
        for(int which=0;which<2;++which) {
            const auto& input=which?current:old;const auto& output=which?seed->current:seed->older;const auto parent=which?parentCurrent:parentOld;
            unchangedLocals(output,input);
            const auto reconstructed=compose(Transform{{},Quat::axis({0,0,1},-yaw),{1,1,1}},output[0]);
            const auto expected=compose(Transform{{},parent,{1,1,1}},input[0]);
            check(near(reconstructed,expected),"new wall frame reconstructs each native root's actual parent-world rotation and translation");
            check(near(WallYawFrame(parent,yaw).toParent(output[0]),input[0]),"each converted root returns to its original parent-local pose");
        }
        PoseHandoff handoff;handoff.beginEntry(seed->current,seed->older,seed->seconds);
        auto authored=fixture(.8f);const auto output=handoff.evaluate(authored,authored,0,0,0);
        check(near(output.pose[0],seed->current[0]),"handoff begins from the prepared wall-space native root instead of the new authored pose");
        for(std::size_t bone=1;bone<99;++bone)check(near(output.pose[bone],current[bone]),"handoff consumes the complete preparatory native skeleton");
    }
}
int main()try {
    timestamps();animationFrames();rejectedSamples();finiteAfterConversion();parentAndWallFrames();
    std::cout<<"PASS NativePoseHistory: "<<checks<<" checks for distinct animation frames, timestamps, transactional rejection, finite conversion, binding clear and wall-space root transforms\n";
    return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
