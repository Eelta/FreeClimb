#include "Pose.h"
#include "ScenePropagation.h"
#include <array>
#include <iostream>
#include <stdexcept>
using namespace fc;

static void check(bool condition,const char* message) {
    if(!condition)throw std::runtime_error(message);
}
static bool same(const Transform& a,const Transform& b) {
    return (a.t-b.t).length()<.0001f&&angleBetween(a.q,b.q)<.0001f&&(a.s-b.s).length()<.0001f;
}
using Chain=std::array<Transform,3>;
static bool same(const Chain& a,const Chain& b) {
    for(std::size_t i=0;i<a.size();++i)if(!same(a[i],b[i]))return false;
    return true;
}

struct SceneModel {
    Transform actor{{110190,77709,2509},Quat::axis({0,0,1},.63f),{1,1,1}};
    Chain native{{
        {{2,-3,4},Quat::axis({1,0,0},.11f),{1,1,1}},
        {{0,0,65},Quat::axis({0,1,0},-.18f),{1,1,1}},
        {{16,0,22},Quat::axis({0,0,1},.24f),{1,1,1}}
    }};
    Chain authored{{
        {{-5,7,12},Quat::axis({1,0,0},1.12f),{1,1,1}},
        {{0,0,65},Quat::axis({0,1,0},.78f),{1,1,1}},
        {{16,0,22},Quat::axis({0,0,1},-.91f),{1,1,1}}
    }};
    Chain local=native,world{};
    std::uint32_t dataFlags=0xAu,observedFlags{};
    unsigned originalCalls{};

    SceneModel() {world=expected(native);}
    Chain expected(const Chain& pose) const {
        Chain result;
        result[0]=compose(actor,pose[0]);
        result[1]=compose(result[0],pose[1]);
        result[2]=compose(result[1],pose[2]);
        return result;
    }
    void original(ScenePass pass) {
        ++originalCalls;observedFlags=dataFlags;
        if(pass==ScenePass::selected&&(dataFlags&0x2u))return;
        world=expected(local);
    }
};

static void propagate(SceneModel& model,ScenePass pass,bool owned,bool legacy=false) {
    const auto saved=model.local;
    const auto callerFlags=model.dataFlags;

    const bool overlay=legacy?(owned&&pass!=ScenePass::transformOnly):overlaysPose(pass,owned);
    if(overlay)model.local=model.authored;
    model.dataFlags=legacy?callerFlags:effectiveUpdateDataFlags(pass,callerFlags,owned);
    model.original(pass);
    model.local=saved;
    model.dataFlags=callerFlags;
}

static void selectedSuppression() {
    SceneModel old,current;
    const auto nativeWorld=old.world;
    propagate(old,ScenePass::selected,true,true);
    check(old.originalCalls==1&&same(old.world,nativeWorld),
        "negative control: selected suppression silently preserves the old native world");
    check(!same(old.world,old.expected(old.authored)),
        "negative control must expose missing custom world propagation despite temporary authored locals");

    propagate(current,ScenePass::selected,true);
    check(current.originalCalls==1&&same(current.world,current.expected(current.authored)),
        "owned selected pass propagates the authored rotations and translations through all descendants");
    check(current.observedFlags==0x8u&&current.dataFlags==0xAu,
        "only selected suppression is cleared for the call; unrelated and caller flags are preserved");
    check(same(current.local,current.native),
        "successful selected propagation restores native locals for the next animation sample");
    std::cout<<"selected-suppress: legacy native/stale world reproduced; owned policy produces authored world\n";
}

static void laterTransformPass() {
    SceneModel old,current;
    propagate(old,ScenePass::downward,true,true);
    propagate(current,ScenePass::downward,true);
    check(same(old.world,old.expected(old.authored))&&same(current.world,current.expected(current.authored)),
        "both paths initially display the authored pose in the ordinary downward pass");
    check(same(old.local,old.native)&&same(current.local,current.native),
        "native locals are restored before the later transform-only pass");

    old.actor.t=old.actor.t+Vec{3,-4,2};current.actor=old.actor;
    old.actor.q=Quat::axis({0,0,1},.83f);current.actor.q=old.actor.q;
    propagate(old,ScenePass::transformOnly,true,true);
    propagate(current,ScenePass::transformOnly,true);
    check(old.originalCalls==2&&same(old.world,old.expected(old.native))&&!same(old.world,old.expected(old.authored)),
        "negative control: the unhooked transform-only pass replaces custom worlds with restored native locals");
    check(current.originalCalls==2&&same(current.world,current.expected(current.authored)),
        "owned transform-only pass reapplies authored locals before rebuilding current descendant worlds");
    check(same(current.local,current.native)&&current.dataFlags==0xAu&&current.observedFlags==0xAu,
        "transform-only handling restores native locals and never rewrites its caller update flags");
    std::cout<<"later-transform-only: legacy native reset reproduced; owned policy preserves authored content under moved parent\n";
}

static void passCoverageAndUnownedActors() {
    constexpr std::array<ScenePass,4> passes={ScenePass::downward,ScenePass::selected,ScenePass::rigid,ScenePass::transformOnly};
    constexpr std::array<std::size_t,4> verifiedSlots={0x2C,0x2D,0x2E,0x31};
    for(std::size_t i=0;i<passes.size();++i) {
        const auto pass=passes[i];
        check(scenePassSlot(pass)==verifiedSlots[i],"hook policy maps the four verified SE scene slots, including nonconsecutive 0x31");
        SceneModel owned;
        propagate(owned,pass,true);
        check(owned.originalCalls==1&&same(owned.world,owned.expected(owned.authored))&&same(owned.local,owned.native),
            "each owned pass calls original once, produces the custom descendant pose and restores native locals");
        for(std::uint32_t flags:{0u,0x2u,0xAu,0xFFFFFFFFu}) {
            SceneModel npc,reference;npc.dataFlags=reference.dataFlags=flags;

            reference.original(pass);
            propagate(npc,pass,false);
            check(npc.originalCalls==1&&same(npc.world,reference.world)&&same(npc.local,npc.native),
                "an unowned NPC or inactive player follows the unchanged original update exactly once");
            check(npc.dataFlags==flags&&npc.observedFlags==flags,
                "unowned scene calls preserve all update flags including suppression");
        }
    }
}

struct FlatSceneModel:SceneModel {
    Transform virtualNative{{8,1,14},Quat::axis({0,1,0},.12f),{1,1,1}};
    Transform virtualAuthored{{-11,4,9},Quat::axis({1,0,0},-.76f),{1,1,1}};
    Transform virtualLocal=virtualNative;
    std::array<Transform,4> cachedWorld{};
    unsigned cacheRefreshes{},controllerTicks{};

    FlatSceneModel() {refreshCache();cacheRefreshes=0;}
    void refreshCache() {
        ++cacheRefreshes;
        for(std::size_t i=0;i<world.size();++i)cachedWorld[i]=world[i];
        cachedWorld[3]=compose(cachedWorld[1],virtualLocal);
    }
    void original(ScenePass pass) {
        SceneModel::original(pass);

        if(pass!=ScenePass::transformOnly){++controllerTicks;refreshCache();}
    }
    Transform expectedVirtual() const {return compose(expected(authored)[1],virtualAuthored);}
    bool materializedCacheMatches() const {
        for(std::size_t i=0;i<world.size();++i)if(!same(cachedWorld[i],world[i]))return false;
        return true;
    }
};

enum class CacheOrder { correct, omitted, afterRestore };
static void propagateFlat(FlatSceneModel& model,ScenePass pass,bool owned,CacheOrder order=CacheOrder::correct) {
    const auto saved=model.local;
    const auto savedVirtual=model.virtualLocal;
    const auto callerFlags=model.dataFlags;
    if(overlaysPose(pass,owned)){model.local=model.authored;model.virtualLocal=model.virtualAuthored;}
    model.dataFlags=effectiveUpdateDataFlags(pass,callerFlags,owned);
    model.original(pass);
    const bool refresh=refreshFlatAfterPass(pass,owned);
    if(refresh&&order==CacheOrder::correct)model.refreshCache();
    model.local=saved;model.virtualLocal=savedVirtual;model.dataFlags=callerFlags;
    if(refresh&&order==CacheOrder::afterRestore)model.refreshCache();
}

static void flatCacheAndVirtualChild() {
    FlatSceneModel initial;
    propagateFlat(initial,ScenePass::downward,true);
    check(initial.materializedCacheMatches()&&same(initial.cachedWorld[3],initial.expectedVirtual()),
        "ordinary flattened downward wrapper fills both materialized and virtual authored worlds");
    check(initial.originalCalls==1&&initial.controllerTicks==1&&initial.cacheRefreshes==1,
        "ordinary downward pass performs its existing controller and cache work once");

    initial.actor.t=initial.actor.t+Vec{-6,8,3};
    initial.authored[1].q=Quat::axis({0,1,0},-.62f);
    initial.virtualAuthored.q=Quat::axis({0,0,1},1.03f);
    auto omitted=initial,late=initial,current=initial;
    propagateFlat(omitted,ScenePass::transformOnly,true,CacheOrder::omitted);
    check(same(omitted.world,omitted.expected(omitted.authored))&&!omitted.materializedCacheMatches(),
        "negative control: correct node worlds still leave stale flattened caches when 0x31 refresh is omitted");
    propagateFlat(late,ScenePass::transformOnly,true,CacheOrder::afterRestore);
    check(late.materializedCacheMatches()&&!same(late.cachedWorld[3],late.expectedVirtual()),
        "negative control: refreshing after local restoration copies nodes correctly but corrupts the virtual child pose");
    check(same(late.cachedWorld[3],compose(late.cachedWorld[1],late.virtualNative)),
        "late refresh specifically combines the custom parent cache with a restored native virtual local");

    propagateFlat(current,ScenePass::transformOnly,true);
    check(current.materializedCacheMatches()&&same(current.cachedWorld[3],current.expectedVirtual()),
        "transform-only refresh before restore produces current authored node caches and the virtual child world");
    check(same(current.local,current.native)&&same(current.virtualLocal,current.virtualNative)&&current.dataFlags==0xAu,
        "both materialized and flat-only locals and caller flags are restored after cache publication");
    check(current.originalCalls==2&&current.controllerTicks==1&&current.cacheRefreshes==2,
        "0x31 calls its original once and refreshes flat worlds without replaying a downward animation controller");
    std::cout<<"flat-cache: omitted refresh leaves stale materialized cache; late refresh produces native virtual child; ordered refresh passes\n";

    for(ScenePass pass:{ScenePass::downward,ScenePass::selected,ScenePass::rigid,ScenePass::transformOnly}) {
        FlatSceneModel model;propagateFlat(model,pass,true);
        check(model.originalCalls==1&&model.cacheRefreshes==1&&model.controllerTicks==(pass==ScenePass::transformOnly?0u:1u),
            "every owned pass executes original/controller semantics once with exactly one flat cache update");
        check(model.materializedCacheMatches()&&same(model.cachedWorld[3],model.expectedVirtual()),
            "all owned passes publish authored materialized and flat-only descendant worlds");
        FlatSceneModel npc,reference;
        npc.actor.t=npc.actor.t+Vec{5,0,2};reference.actor=npc.actor;
        reference.original(pass);propagateFlat(npc,pass,false);
        check(npc.originalCalls==reference.originalCalls&&npc.controllerTicks==reference.controllerTicks&&npc.cacheRefreshes==reference.cacheRefreshes,
            "unowned flattened actors retain the original controller and cache update counts");
        for(std::size_t i=0;i<npc.cachedWorld.size();++i)check(same(npc.cachedWorld[i],reference.cachedWorld[i]),
            "unowned flattened actors do not receive an extra cache refresh or authored virtual pose");
    }
}

struct FakeSceneNode {
    Transform local{},world{};
    FakeSceneNode* parent{};
    std::uint32_t flags=0x6u;
    unsigned controllerTicks{};
};

static void unmappedNativeBridge() {
    for(bool forceBridge:{false,true}) {
        SceneModel source;
        FakeSceneNode outer;outer.local=outer.world=source.actor;outer.flags=0x401Au;
        std::array<FakeSceneNode,4> nodes;
        nodes[0].local=source.native[0];nodes[0].parent=&outer;
        nodes[1].local={{0,3,9},Quat::axis({0,0,1},.37f),{1.08f,1.08f,1.08f}};
        nodes[1].flags=0x401Au;
        nodes[2].local=source.native[1];nodes[3].local=source.native[2];
        for(std::size_t i=1;i<nodes.size();++i)nodes[i].parent=&nodes[i-1];
        std::array<Transform,4> savedLocal;
        std::array<std::uint32_t,4> savedFlags;
        for(std::size_t i=0;i<nodes.size();++i) {
            nodes[i].world=compose(nodes[i].parent->world,nodes[i].local);
            savedLocal[i]=nodes[i].local;savedFlags[i]=nodes[i].flags;
        }
        std::array<FakeSceneNode*,3> mapped={&nodes[0],&nodes[2],&nodes[3]};
        const auto bridges=sceneBridgeNodes<FakeSceneNode*>(mapped,&nodes[0],[](auto* node){return node->parent;});
        check(bridges&&bridges->size()==1&&bridges->front()==&nodes[1],
            "production bridge discovery finds the single existing unmapped ancestor and excludes the actor outer root");
        for(std::size_t i=0;i<mapped.size();++i) {
            mapped[i]->local=source.authored[i];
            mapped[i]->flags=ownedSceneNodeFlags(mapped[i]->flags);
        }
        if(forceBridge)for(auto* bridge:*bridges)bridge->flags=ownedSceneNodeFlags(bridge->flags);
        check(same(nodes[1].local,savedLocal[1])&&same(outer.local,source.actor)&&outer.flags==0x401Au,
            "ancestor forcing never changes a native bridge local/scale or touches the actor outer root");
        if(forceBridge)check(nodes[1].flags==0x400Eu,
            "owned bridge enables transforms and clears rigid while preserving controller and unrelated flags");

        const auto updateFlags=effectiveUpdateDataFlags(ScenePass::selected,0xAu,true);
        unsigned originalCalls=0;
        ++originalCalls;
        for(auto& node:nodes) {
            if(node.flags&0x8u)++node.controllerTicks;
            if((node.flags&0x4u)&&!(updateFlags&0x2u))node.world=compose(node.parent->world,node.local);
        }
        bool mappedAudit=true;
        for(auto* node:mapped)mappedAudit&=same(node->world,compose(node->parent->world,node->local));
        const bool bridgeAudit=same(nodes[1].world,compose(nodes[0].world,nodes[1].local));
        auto expected=compose(outer.world,source.authored[0]);
        expected=compose(expected,savedLocal[1]);
        expected=compose(expected,source.authored[1]);
        expected=compose(expected,source.authored[2]);
        const bool actualChain=same(nodes[3].world,expected);
        check(mappedAudit,"mapped-only parent.world-times-local audit passes in both positive and stale-bridge fixtures");
        if(forceBridge)check(bridgeAudit&&actualChain,
            "forcing discovered native ancestors propagates the complete authored chain through the preserved bridge transform");
        else check(!bridgeAudit&&!actualChain,
            "negative control: a stale unmapped ancestor makes the mapped-only audit falsely pass while the full pose remains wrong");
        check(originalCalls==1&&nodes[1].controllerTicks==1,
            "forcing ancestor transform flags does not replay the native bridge controller");
        for(std::size_t i=0;i<nodes.size();++i) {
            nodes[i].local=savedLocal[i];nodes[i].flags=savedFlags[i];
            check(same(nodes[i].local,savedLocal[i])&&nodes[i].flags==savedFlags[i],
                "every mapped local and original mapped/bridge flags are restored after the pass");
        }
    }

    FakeSceneNode outer,root,bridge,a,b,foreign;
    root.parent=&outer;bridge.parent=&root;a.parent=&bridge;b.parent=&a;foreign.parent=&outer;
    std::array<FakeSceneNode*,3> mapped={&root,&a,&b};
    auto ancestors=sceneBridgeNodes<FakeSceneNode*>(mapped,&root,[](auto* node){return node->parent;});
    check(ancestors&&ancestors->size()==1&&ancestors->front()==&bridge,
        "shared mapped-to-root paths deduplicate the native bridge without owning mapped ancestors twice");
    mapped[2]=&foreign;
    check(!sceneBridgeNodes<FakeSceneNode*>(mapped,&root,[](auto* node){return node->parent;}),
        "a same-named candidate in another skeleton branch cannot authorize ancestor flag ownership");
    mapped[2]=&b;bridge.parent=&a;
    check(!sceneBridgeNodes<FakeSceneNode*>(mapped,&root,[](auto* node){return node->parent;}),
        "a parent cycle fails bounded bridge discovery instead of hanging or crossing the owned root");
    std::cout<<"native-bridge: stale parent falsely passes mapped-only audit; discovered-ancestor propagation and full audit pass\n";
}

static void typedFlatSynchronization() {
    struct Node {Transform world;};
    struct Entry {Transform local,world;std::int16_t parentIndex=-1;Node* node{};};
    const Transform root{{112233,-88776,333},Quat::axis({0,0,1},.72f),{1.2f,1.2f,1.2f}};
    Node materialized{compose(root,Transform{{4,3,20},Quat::axis({0,1,0},.2f),{.9f,.9f,.9f}})};
    std::array<Entry,5> entries{{
        {{{0,0,2},Quat::axis({1,0,0},.12f),{1,1,1}},{},-1,nullptr},
        {{{0,0,7},Quat::axis({0,1,0},.23f),{1.1f,1.1f,1.1f}},{},0,nullptr},
        {{{123,456,789},{},{1,1,1}},{},-1,&materialized},
        {{{5,0,2},Quat::axis({0,0,1},-.19f),{1,1,1}},{},2,nullptr},
        {{{3,-2,1},{},{1,1,1}},{},-1,&materialized}
    }};
    const auto original=entries;
    unsigned compositions=0;
    const auto multiply=[&](const auto& parent,const auto& local){++compositions;return compose(parent,local);};
    check(synchronizeFlatEntries(std::span(entries),root,multiply),"typed flat entries accept a real parent-ordered mixed tree");
    check(compositions==3,"flat refresh composes virtual entries only, without node/controller calls");
    check(same(entries[0].world,compose(root,entries[0].local))&&
        same(entries[1].world,compose(entries[0].world,entries[1].local)),"root-relative and multi-level virtual entries inherit the full transform and scale");
    check(same(entries[2].world,materialized.world)&&same(entries[4].world,materialized.world)&&
        same(entries[3].world,compose(materialized.world,entries[3].local)),"materialized aliases copy actual node worlds before virtual descendants compose");
    for(std::size_t i=0;i<entries.size();++i)check(same(entries[i].local,original[i].local),"flat synchronization never changes source locals");
    const auto first=entries;
    check(synchronizeFlatEntries(std::span(entries),root,multiply),"second identical pass is valid");
    for(std::size_t i=0;i<entries.size();++i)check(same(entries[i].world,first[i].world),"cache synchronization is idempotent rather than accumulating transform drift");
    for(int invalid:{1,4,99}) {
        auto bad=entries;bad[1].parentIndex=std::int16_t(invalid);const auto saved=bad;const auto beforeCalls=compositions;
        check(!synchronizeFlatEntries(std::span(bad),root,multiply),"self, forward and out-of-range virtual parents reject before writing");
        check(compositions==beforeCalls,"rejected tree cannot partially enter the composition loop");
        for(std::size_t i=0;i<bad.size();++i)check(same(bad[i].world,saved[i].world),"malformed tree leaves every cache unchanged");
    }
    std::vector<Entry> tooMany(4097);
    check(!synchronizeFlatEntries(std::span(tooMany),root,multiply),"flat refresh rejects unbounded entry counts");
    check(synchronizeFlatEntries(std::span<Entry>{},root,multiply),"empty valid cache is a no-op");
    check(scenePassSlot(ScenePass::downward)==0x2c&&scenePassSlot(ScenePass::selected)==0x2d&&
        scenePassSlot(ScenePass::rigid)==0x2e&&scenePassSlot(ScenePass::transformOnly)==0x31&&sceneWorldSlot()==0x30,
        "CommonLib flat SE and AE NiAVObject virtual slots retain their documented signatures");
    std::cout<<"typed-flat: mixed materialized/virtual transforms, exact aliases, scales, no local mutation and fail-before-write verified\n";
}
int main() {
    try {
        selectedSuppression();laterTransformPass();passCoverageAndUnownedActors();flatCacheAndVirtualChild();unmappedNativeBridge();typedFlatSynchronization();
        std::cout<<"Scene propagation policy regressions passed; native game rendering still requires runtime verification.\n";
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
