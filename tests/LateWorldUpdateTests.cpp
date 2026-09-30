#include "Pose.h"
#include "YawFrame.h"
#include "LateWorldUpdate.h"
#include <array>
#include <iostream>
#include <stdexcept>
#include <type_traits>

using namespace fc;
static void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
static bool same(const Transform& a,const Transform& b) {
    return (a.t-b.t).length()<.0002f&&angleBetween(a.q,b.q)<.0001f&&(a.s-b.s).length()<.0001f;
}
struct Data {std::uint32_t flags=0xAu;};
struct Node {
    Node* parent{};
    const char* name="NPC bone";
    Transform local{},world{},flatWorld{},observedLocal{};
    Data* observedData{};
    unsigned originalCalls{};
};
struct Scene {

    std::array<Node,8> nodes{};
    std::array<Transform,3> accepted{{
        {{-5,7,12},Quat::axis({1,0,0},.67f),{1,1,1}},
        {{0,0,63},Quat::axis({0,1,0},.81f),{1,1,1}},
        {{15,2,21},Quat::axis({0,1,0},-.93f),{1,1,1}}
    }};
    float wallYaw=1.1f;
    Scene() {
        nodes[0].world={{110,220,330},Quat::axis({0,0,1},-.35f),{1.03f,1.03f,1.03f}};
        nodes[1].parent=&nodes[0];nodes[1].name="NPC Root [Root]";
        nodes[1].local={{1,2,3},Quat::axis({1,0,0},.08f),{1.01f,1.01f,1.01f}};
        nodes[2].parent=&nodes[1];nodes[2].local={{0,0,63},Quat::axis({0,1,0},.04f),{1.06f,1.06f,1.06f}};
        nodes[3].parent=&nodes[2];nodes[3].name="CME hand adjustment";
        nodes[3].local={{3,-2,5},Quat::axis({0,1,0},.19f),{1.08f,1.08f,1.08f}};
        nodes[4].parent=&nodes[3];nodes[4].local={{15,2,21},Quat::axis({0,1,0},.07f),{.94f,.94f,.94f}};
        nodes[5].parent=&nodes[0];nodes[5].name="NPC Root [Root]";nodes[5].local.t={90,0,0};
        nodes[6].parent=&nodes[5];nodes[6].local.t={0,0,70};
        nodes[7].parent=&nodes[1];nodes[7].name="Camera Control";nodes[7].local.t={0,12,14};
        for(int i=1;i<8;++i){original(&nodes[i],nullptr);nodes[i].flatWorld=nodes[i].world;nodes[i].originalCalls=0;}
    }
    void original(Node* node,Data* data) {
        ++node->originalCalls;node->observedData=data;node->observedLocal=node->local;
        node->world=node->parent?compose(node->parent->world,node->local):node->local;
    }
    int track(Node* node) const {
        return node==&nodes[1]?0:node==&nodes[2]?1:node==&nodes[4]?2:-1;
    }
    Transform replacement(Node* node) const {
        const int index=track(node);check(index>=0,"replacement requested only for exact mapped pointers");
        auto desired=accepted[index];
        if(index==0)desired=WallYawFrame(node->parent->world.q,wallYaw).toParent(desired);

        desired.s=node->local.s;return desired;
    }
    void wrapped(Node* node,Data* data) {
        if(track(node)>=0) {
            const auto desired=replacement(node);
            ScopedLocalOverride<Transform> temporary(node->local,desired);
            original(node,data);node->flatWorld=node->world;
        }else {
            original(node,data);
            if(node==&nodes[3])node->flatWorld=node->world;
        }
    }
    std::array<Transform,4> expected() const {
        auto parent=nodes[0].world;parent.q=Quat::axis({0,0,1},-wallYaw);
        auto root=accepted[0];root.s=nodes[1].local.s;
        auto body=accepted[1];body.s=nodes[2].local.s;
        auto hand=accepted[2];hand.s=nodes[4].local.s;
        std::array<Transform,4> result;
        result[0]=compose(parent,root);result[1]=compose(result[0],body);
        result[2]=compose(result[1],nodes[3].local);result[3]=compose(result[2],hand);return result;
    }
    void publish() {
        const auto worlds=expected();
        for(int i=1;i<=4;++i){nodes[i].world=worlds[i-1];nodes[i].flatWorld=worlds[i-1];}
    }
};

static void nativeLatePassNegativeAndOwnedFix() {
    Scene old,current;Data data;
    const auto oldExpected=old.expected(),expected=current.expected();old.publish();current.publish();
    std::array<Transform,8> native{};for(std::size_t i=0;i<native.size();++i)native[i]=current.nodes[i].local;

    for(int i=2;i<=4;++i)old.original(&old.nodes[i],&data);
    check(same(old.nodes[1].world,oldExpected[0])&&!same(old.nodes[4].world,oldExpected[3]),
        "negative control: root remains custom while late per-bone native locals overwrite the displayed limb");
    check(!same(old.nodes[4].flatWorld,old.nodes[4].world),"negative control also exposes stale flattened aliases after direct per-node updates");
    for(Data* pointer:{static_cast<Data*>(nullptr),&data}) {
        for(int i=2;i<=4;++i) {
            const auto calls=current.nodes[i].originalCalls;current.wrapped(&current.nodes[i],pointer);
            check(current.nodes[i].originalCalls==calls+1&&current.nodes[i].observedData==pointer,
                "every direct world call forwards the original data pointer, including null, exactly once");
            check(same(current.nodes[i].world,expected[i-1])&&same(current.nodes[i].flatWorld,current.nodes[i].world),
                "accepted authored locals protect each owned late world and refresh its flat alias");
            check(same(current.nodes[i].local,native[i]),"native local is restored immediately after each original world update");
        }
    }
    check(data.flags==0xAu,"late world protection does not rewrite caller update-data flags");
    check(same(current.nodes[3].observedLocal,native[3]),"bridge translation, rotation and scale are passed through unchanged");
    for(int i:{2,4})check(same(current.nodes[i].observedLocal,current.replacement(&current.nodes[i])),
        "mapped original sees exactly the accepted local with its native scale");
    std::cout<<"late 0x30: native overwrite reproduced below an unchanged custom root; exact body wrappers retain authored worlds\n";
}

static void exactOwnershipAndRepeatedUpdates() {
    Scene scene;scene.publish();Data data;
    for(int i:{5,6,7}) {
        const auto local=scene.nodes[i].local,flat=scene.nodes[i].flatWorld;
        const auto expected=compose(scene.nodes[i].parent->world,local);const auto count=scene.nodes[i].originalCalls;
        scene.wrapped(&scene.nodes[i],&data);
        check(scene.nodes[i].originalCalls==count+1&&same(scene.nodes[i].observedLocal,local)&&same(scene.nodes[i].local,local)&&same(scene.nodes[i].world,expected),
            "same-name NPC nodes and an inside-root camera receive their unchanged native update");
        check(same(scene.nodes[i].flatWorld,flat),"unowned callbacks do not synchronize or alter unrelated flat cache fields");
    }
    const auto expected=scene.expected();

    for(int repeat=0;repeat<12;++repeat)for(int i=1;i<=4;++i) {
        scene.wrapped(&scene.nodes[i],repeat%2?&data:nullptr);
        check(same(scene.nodes[i].world,expected[i-1]),"same-frame repeated calls cannot blend the accepted output back toward restored native locals");
    }
    for(int i=1;i<=4;++i)check(scene.nodes[i].originalCalls==12,"same-frame callbacks call each original once without replaying controllers or parent passes");
    std::cout<<"ownership/repeats: same-name NPC and camera remain native; accepted body pose does not drift over 12 direct updates\n";
}

static void currentParentYawAndNativeAdjustments() {
    Scene scene;scene.publish();
    for(float parentYaw:{.05f,1.6f,3.5f,6.25f}) {
        scene.nodes[0].world.t=scene.nodes[0].world.t+Vec{3,-2,1};
        scene.nodes[0].world.q=Quat::axis({0,0,1},-parentYaw)*Quat::axis({1,0,0},.11f);
        scene.nodes[3].local.q=Quat::axis({0,1,0},parentYaw*.08f);
        scene.nodes[3].local.s={1.12f,1.12f,1.12f};
        const auto expected=scene.expected();
        std::array<Transform,4> native;for(int i=1;i<=4;++i)native[i-1]=scene.nodes[i].local;
        for(int i=1;i<=4;++i)scene.wrapped(&scene.nodes[i],nullptr);
        for(int i=1;i<=4;++i) {
            check(same(scene.nodes[i].world,expected[i-1]),"late root update rebases wall pose using the current parent yaw/pitch, translation and scale");
            check(same(scene.nodes[i].local,native[i-1]),"live native adjustment locals survive yaw rebasing and direct world propagation");
        }
        const auto rendered=frameYaw(scene.nodes[1].world.q*scene.accepted[0].q.inverse());
        check(rendered&&std::abs(yawDifference(*rendered,scene.wallYaw))<.00001f,
            "changing native parent orientation does not turn the accepted climbing body away from the wall");
    }
    std::cout<<"current parent: wall frame rebases through yaw wrap while bridge adjustment and native scales stay live\n";
}

static void scopedRestorationOnUnwind() {
    static_assert(!std::is_copy_constructible_v<ScopedLocalOverride<Transform>>);
    Transform native{{1,2,3},Quat::axis({0,1,0},.2f),{1.03f,1.03f,1.03f}};
    const auto saved=native;const Transform accepted{{7,8,9},Quat::axis({1,0,0},.7f),{1.03f,1.03f,1.03f}};
    try {
        ScopedLocalOverride<Transform> outer(native,accepted);
        check(same(native,accepted),"scope installs the exact accepted local");
        {ScopedLocalOverride<Transform> inner(native,saved);check(same(native,saved),"nested scope uses its own local");}
        check(same(native,accepted),"nested scope restores the immediately enclosing accepted local");
        throw std::runtime_error("simulated callback unwind");
    }catch(const std::runtime_error& error){check(std::string(error.what())=="simulated callback unwind","scope assertions must not be swallowed by the unwind test");}
    check(same(native,saved),"exception unwinding restores the original engine local");
}
static void finalSkinAuditExpectation() {
    Scene scene;const auto expected=scene.expected();
    std::unordered_map<Node*,std::optional<Transform>> cache;
    unsigned localReads=0;
    auto local=[&](Node* node)->std::optional<Transform> {
        ++localReads;if(node==&scene.nodes[7])return {};
        if(scene.track(node)>=0)return scene.replacement(node);
        return node->local;
    };
    auto resolve=[&](Node* node) {
        return ownedSkinWorld<Node*,Transform>(node,&scene.nodes[1],expected[0],
            [](Node* current){return current->parent;},local,
            [](const Transform& parent,const Transform& child){return compose(parent,child);},cache);
    };

    const auto staleHand=scene.nodes[4].world;
    const auto hand=resolve(&scene.nodes[4]);
    check(hand&&same(*hand,expected[3])&&!same(*hand,staleHand),
        "final skin audit detects native child worlds even when their entire native parent chain is self-consistent");
    check(localReads==3,"expected-world audit caches the actual body and native intermediate bridge once");
    for(int i=1;i<=4;++i){const auto value=resolve(&scene.nodes[i]);check(value&&same(*value,expected[i-1]),"read-only expected world retains native bridge local and scale");}
    check(localReads==3,"root endpoint and cached ancestors do not repeat local reads");
    for(int i:{0,5,6,7})check(!resolve(&scene.nodes[i]),"skin audit refuses outer actor, sibling NPC and engine-owned camera inputs");
    Node cameraChild;cameraChild.parent=&scene.nodes[7];
    check(!resolve(&cameraChild),"camera rejection also rejects descendants rather than inventing a body-space expectation");
    Node first,second;first.parent=&second;second.parent=&first;
    check(!resolve(&first),"cyclic skin ancestry cannot escape the owned root restriction");
    check(same(scene.nodes[4].world,staleHand),"final skin audit is read-only and cannot repair or hide actual input corruption");
    std::cout<<"final skin audit: authored expectation differs from self-consistent stale native worlds; foreign/camera/cyclic ancestry rejected\n";
}
int main() {
    try {
        nativeLatePassNegativeAndOwnedFix();exactOwnershipAndRepeatedUpdates();currentParentYawAndNativeAdjustments();scopedRestorationOnUnwind();finalSkinAuditExpectation();
        std::cout<<"PASS: late single-bone world protection, exact ownership, flat aliases, native restoration and current yaw\n";
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
