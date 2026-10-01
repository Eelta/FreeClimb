#pragma once
#include "Pose.h"
#include "AnimationPack.h"
#include "TrackView.h"
#include "SceneBinding.h"
#include "AnimationSkeletonBinding.h"
#include "PoseHealth.h"
#include "PoseBlendEnvelope.h"
#include "PoseHandoff.h"
#include "YawFrame.h"
#include "FlatSkeleton.h"
#include "ScenePropagation.h"
#include "LateWorldUpdate.h"
#include <type_traits>
#include "SkinAuditBudget.h"
#include <mutex>
#include <atomic>
#include <chrono>
#include <memory>
#include <intrin.h>

namespace fc {

class PoseRuntime {
public:
    struct Orientation {
        float targetYaw{},parentYaw{},renderedYaw{};
        bool parentValid{},renderedValid{};
        std::uint64_t sampledAt{};
    };
    struct OutputAudit {
        Motion motion=Motion::none;
        float layerWeight{},nativeRecovery{},propagationMs{},callbackMs{};
        std::array<float,2> upperRollDegrees{};
        std::uint32_t overwrittenBones{};
        int firstOverwritten=-1;
        float overwrittenAngle{},overwrittenDistance{};
        std::uint32_t overwriteFrames{};
        int lastOverwritten=-1;
        float peakOverwrittenAngle{},peakOverwrittenDistance{};
        std::uint32_t worldMismatches{},worldMismatchFrames{},transformPasses{},selectedFlagRepairs{};
        float worldAngle{},worldDistance{};
        int firstWorldMismatch=-1;
        ScenePass pass=ScenePass::downward;
        std::uint64_t sampledAt{};
    };
private:
    struct OutputRig {
        RE::NiPointer<RE::NiAVObject> root;
        std::unordered_map<RE::NiAVObject*,std::size_t> slots;
        std::unordered_map<const RE::NiTransform*,RE::NiAVObject*> worldNodes;
        std::unordered_map<RE::NiAVObject*,std::vector<RE::NiTransform*>> flatAliases;

        std::vector<RE::NiPointer<RE::NiAVObject>> nodes;
        std::vector<RE::NiAVObject*> parents;
        FlatBoneEntry* flatData{};
        std::size_t flatCount{};
    };
    struct AcceptedScene {
        std::shared_ptr<const OutputRig> rig;
        std::array<RE::NiTransform,99> locals;
        Transform wallRoot;
        float yaw{};
        bool wallFrame{};
        std::uint64_t epoch{};
    };
    std::atomic<std::shared_ptr<const AcceptedScene>> acceptedScene;
    SkinAuditBudget skinAuditBudget;
    struct Binding {
        RE::BSTSmartPointer<RE::BShkbAnimationGraph> graph;
        RE::NiPointer<RE::NiAVObject> root;
        SceneBinding<SceneSlot> scene;
        std::vector<RE::NiPointer<RE::NiAVObject>> bridges;
        std::shared_ptr<const OutputRig> outputRig;
        RE::NiPointer<RE::NiAVObject> actorRoot;
        RE::hkRefPtr<const RE::hkaSkeleton> skeleton;
        const RE::hkaBone* bones{};
        const std::int16_t* parents{};
        const RE::hkQsTransform* references{};
        int boneCount{};
        AnimationSkeletonBinding animation;
        FlatBoneEntry* flatData{};
        std::size_t flatCount{};
        std::array<RE::NiAVObject*,99> nodeParents{};
        std::array<int,99> flatIndices{},flatParents{};
        std::array<std::vector<SceneSlot>,99> basisPaths;
        std::unordered_map<RE::NiAVObject*,RE::NiAVObject*> bridgeParents;
        std::unordered_map<int,int> flatBridgeParents;
        Pose reference,animationReference;
        std::array<Transform,99> basis{};
        std::array<bool,99> selected{},locked{};
        std::uint8_t footIK{};
        bool footOwned{};
    };
    std::vector<Binding> bindings;
    std::mutex mutex;
    std::atomic<bool> rigInvalidated{};
    enum class RigIssue:std::uint32_t {none,graph,arrays,storage,model,hierarchy,reference,transform};
    std::atomic<std::uint32_t> rigIssue{};
    bool rejectRig(RigIssue issue,int bone=-1) {
        rigIssue.store((std::uint32_t(issue)<<16)|std::uint32_t(bone+1));return false;
    }
    std::uint32_t reusedBindings{};
    Pose published;
    PoseHandoff handoff;
    TopRecoveryGate topEndpoint;
    float recovery{},exitAge{},sampleTime{};
    float publishedYaw{},displayedYaw{};
    bool hasWallFrame{},hasDisplayedYaw{};
    Orientation orientation;
    OutputAudit outputAudit;
    Motion publishedMotion=Motion::none;
    std::uint64_t generation{},lastSuccessfulOutput{};
    PoseBlendEnvelope envelope;
    bool fading{},ownsPose{},nativeFootIK{};
    std::atomic<std::uint64_t> lastCallback{};
    static inline PoseRuntime* instance{};
    static inline thread_local bool inside{};
    std::atomic<const RE::NiAVObject*> target{};
    static bool preflightHookSites() {
        constexpr std::array<std::size_t,5> slots{scenePassSlot(ScenePass::downward),scenePassSlot(ScenePass::selected),
            scenePassSlot(ScenePass::rigid),scenePassSlot(ScenePass::transformOnly),sceneWorldSlot()};
        for(unsigned tableIndex=0;tableIndex<4;++tableIndex) {
            REL::Relocation<std::uintptr_t> table{tableIndex==0?RE::VTABLE_NiNode[0]:tableIndex==1?RE::VTABLE_BSFadeNode[0]:
                tableIndex==2?RE::VTABLE_BSFlattenedBoneTree[0]:RE::VTABLE_NiAVObject[0]};
            for(const auto slot:slots) {
                if(tableIndex==3&&slot!=sceneWorldSlot())continue;
                if(!runtime::hookSite(table.address(),slot)) {
                    SKSE::log::error("Pose virtual hook preflight failed: table={} slot={}; no pose hooks installed",tableIndex,slot);return false;
                }
            }
        }
        return true;
    }
    using Downward=void(*)(RE::NiNode*,RE::NiUpdateData&,std::uint32_t);
    template<int Table,int Pass> struct Hook {
        static inline REL::Relocation<Downward> original;
        static void call(RE::NiNode* node,RE::NiUpdateData& data,std::uint32_t flags) {
            auto* self=instance;
            if(!self||self->target.load()!=node||inside){original(node,data,flags);return;}
            inside=true;
            self->apply(node,data,static_cast<ScenePass>(Pass),[&](RE::NiUpdateData& update){original(node,update,flags);});inside=false;
        }
        static void install() {

            REL::Relocation<std::uintptr_t> table{Table==0?RE::VTABLE_NiNode[0]:
                Table==1?RE::VTABLE_BSFadeNode[0]:RE::VTABLE_BSFlattenedBoneTree[0]};
            original=table.write_vfunc(scenePassSlot(static_cast<ScenePass>(Pass)),call);
        }
    };
    using TransformOnly=void(*)(RE::NiNode*,RE::NiUpdateData&);
    template<int Table> struct TransformHook {
        static inline REL::Relocation<TransformOnly> original;
        static void call(RE::NiNode* node,RE::NiUpdateData& data) {
            auto* self=instance;
            if(!self||self->target.load()!=node||inside){original(node,data);return;}
            inside=true;
            self->apply(node,data,ScenePass::transformOnly,[&](RE::NiUpdateData& update){original(node,update);});inside=false;
        }
        static void install() {
            REL::Relocation<std::uintptr_t> table{Table==0?RE::VTABLE_NiNode[0]:
                Table==1?RE::VTABLE_BSFadeNode[0]:RE::VTABLE_BSFlattenedBoneTree[0]};
            original=table.write_vfunc(scenePassSlot(ScenePass::transformOnly),call);
        }
    };
    using WorldData=void(*)(RE::NiAVObject*,RE::NiUpdateData*);
    template<int Table> struct WorldHook {
        static inline REL::Relocation<WorldData> original;
        static void call(RE::NiAVObject* node,RE::NiUpdateData* data) {
            auto* self=instance;
            if(!self||inside||!self->target.load()){original(node,data);return;}
            self->lateWorld(node,data,original.get(),_ReturnAddress());
        }
        static void install() {
            REL::Relocation<std::uintptr_t> table{Table==0?RE::VTABLE_NiNode[0]:
                Table==1?RE::VTABLE_BSFadeNode[0]:Table==2?RE::VTABLE_BSFlattenedBoneTree[0]:RE::VTABLE_NiAVObject[0]};
            original=table.write_vfunc(sceneWorldSlot(),call);
        }
    };
    using SkinGenerator=void(*)(RE::NiSkinInstance*,const RE::NiTransform*);
    struct SkinHook {
        static inline REL::Relocation<SkinGenerator> original;
        static inline const std::uint32_t* frame{};
        static void call(RE::NiSkinInstance* skin,const RE::NiTransform* geometryWorld) {
            auto* self=instance;
            if(!self){original(skin,geometryWorld);return;}
            self->observeSkin(skin,geometryWorld,original.get(),*frame);
        }
        static bool install() {
            if(!runtime::isSE()) {
                SKSE::log::info("Final skin observer disabled for this runtime; scene output and late-world protection remain active");return false;
            }
            const auto base=REL::Module::get().base();
            const auto function=REL::ID(75655).address(),callsite=REL::ID(101335).address()+0x7B;
            const auto frameAddress=REL::ID(525008).address();
            constexpr std::string_view signature="\x48\x89\x54\x24\x10\x55\x53\x56\x57\x41\x54\x41\x55\x41\x56\x41\x57\x48\x8d\x6c\x24\xe1\x48\x81\xec\xa8\x00\x00\x00"sv;
            constexpr std::string_view callBytes="\xe8\xc0\x58\xa5\xff"sv;
            if(function!=base+0xD74F70||callsite!=base+0x131F6AB||frameAddress!=base+0x302C8DC||
                !runtime::executable(function)||!runtime::readable(function,signature.size())||!runtime::readable(callsite,callBytes.size())||
                !runtime::readable(frameAddress,sizeof(std::uint32_t))||
                std::memcmp(reinterpret_cast<const void*>(function),signature.data(),signature.size())!=0||
                std::memcmp(reinterpret_cast<const void*>(callsite),callBytes.data(),callBytes.size())!=0) {
                SKSE::log::warn("Final skin observer unavailable: signature changed or callsite already owned; bone update protection remains active");return false;
            }
            frame=reinterpret_cast<const std::uint32_t*>(frameAddress);
            SKSE::AllocTrampoline(32);
            original=SKSE::GetTrampoline().write_call<5>(callsite,call);
            return true;
        }
    };
    static Transform read(const RE::NiTransform& tr) {
        const auto& m=tr.rotate.entry;
        const Quat q=Quat::boneFrame({m[0][2],m[1][2],m[2][2]},{m[0][0],m[1][0],m[2][0]});
        return {{tr.translate.x,tr.translate.y,tr.translate.z},q,{tr.scale,tr.scale,tr.scale}};
    }
    static void write(RE::NiTransform& tr,const Transform& value) {
        const auto x=value.q.rotate({1,0,0}),y=value.q.rotate({0,1,0}),z=value.q.rotate({0,0,1});
        tr.rotate.entry[0][0]=x.x;tr.rotate.entry[1][0]=x.y;tr.rotate.entry[2][0]=x.z;
        tr.rotate.entry[0][1]=y.x;tr.rotate.entry[1][1]=y.y;tr.rotate.entry[2][1]=y.z;
        tr.rotate.entry[0][2]=z.x;tr.rotate.entry[1][2]=z.y;tr.rotate.entry[2][2]=z.z;
        tr.translate={value.t.x,value.t.y,value.t.z};

    }
    static Transform read(const RE::hkQsTransform& tr) {
        alignas(16) float t[4],q[4],s[4];
        _mm_store_ps(t,tr.translation.quad);_mm_store_ps(q,tr.rotation.vec.quad);_mm_store_ps(s,tr.scale.quad);
        return {{t[0],t[1],t[2]},{q[0],q[1],q[2],q[3]},{s[0],s[1],s[2]}};
    }
    static bool validSceneTransform(const RE::NiTransform& value) {
        const auto& m=value.rotate.entry;
        return PoseRig<Pose>::validAxes(Vec{m[0][0],m[1][0],m[2][0]},Vec{m[0][1],m[1][1],m[2][1]},Vec{m[0][2],m[1][2],m[2][2]})&&
            PoseRig<Pose>::validLocal(read(value));
    }
    static bool sameFlatStorage(RE::NiAVObject* root,const FlatBoneEntry* data,std::size_t count) {
        if(const auto* tree=flatTree(root)) {
            const auto& current=tree->GetRuntimeData();return current.boneEntries==data&&current.numBones==count;
        }
        return count==0;
    }
    bool captureRig(Binding& binding) {
        const auto entries=flatEntries(binding.root.get());
        if(!entries){bindingFailure="invalid flattened scene storage";return false;}
        binding.flatData=entries->data();binding.flatCount=entries->size();
        binding.flatIndices.fill(-1);binding.flatParents.fill(-1);
        binding.reference=library.rig.source().empty()?library.rest:library.rig.source();
        binding.animationReference=binding.reference;
        for(std::size_t i=0;i<99;++i)if(binding.animation.indices[i]>=0)
            binding.animationReference[i]=read(binding.references[binding.animation.indices[i]]);
        const auto equipment=[&](const RE::BSFixedString& name) {
            return name.c_str()&&engineOwnedEquipmentName(name.c_str(),library.names,library.parents);
        };
        bool valid=true;int failed=-1;const char* reason="unsupported structural transform";
        const auto fail=[&](std::size_t i,const char* message){valid=false;failed=int(i);reason=message;};
        binding.scene.each([&](std::size_t i,const auto& slot) {
            if(!valid)return;
            const int index=binding.animation.indices[i];
            if(index<0||index>=binding.boneCount){fail(i,"scene bone has no animation mapping");return;}
            binding.selected[i]=true;binding.locked[i]=binding.bones[index].lockTranslation;
            binding.reference[i]=PoseRig<Pose>::structuralReference(i,read(slot.local()),read(binding.references[index]),binding.locked[i]);
            binding.nodeParents[i]=slot.node?slot.node->parent:nullptr;
            if(slot.flat)for(std::size_t j=0;j<entries->size();++j)if(&(*entries)[j].local==slot.flat) {
                binding.flatIndices[i]=int(j);binding.flatParents[i]=(*entries)[j].parentIndex;break;
            }
            if(i==0)return;
            const auto parentIndex=library.parents[i];
            if(parentIndex<0||parentIndex>=99||!binding.scene.nodes[parentIndex]){fail(i,"canonical parent is not mapped");return;}
            const auto& parent=binding.scene.nodes[parentIndex];
            auto& path=binding.basisPaths[i];
            if(slot.node) {
                if(!parent.node){fail(i,"node parent requires unsupported dynamic flattened reparenting");return;}
                auto* ancestor=slot.node->parent;
                for(unsigned depth=0;ancestor&&ancestor!=parent.node.get()&&depth<128;++depth) {
                    if(ancestor==binding.root.get()){fail(i,"mapped parent is not an actual ancestor");return;}
                    if(equipment(ancestor->name)){fail(i,"equipment attachment is an ancestor of a body bone");return;}
                    bool mapped=false;
                    binding.scene.each([&](std::size_t,const auto& entry){mapped|=entry.node.get()==ancestor;});
                    if(mapped){fail(i,"actual ancestor order differs from animation hierarchy");return;}
                    path.push_back({RE::NiPointer<RE::NiAVObject>(ancestor),nullptr});ancestor=ancestor->parent;
                }
                if(!ancestor||ancestor!=parent.node.get()){fail(i,"mapped parent is outside bounded ancestor chain");return;}
            } else {
                int index=binding.flatParents[i];bool found=false;
                for(unsigned depth=0;depth<128;++depth) {
                    if(index<0){found=parent.node==binding.root;break;}
                    if(std::size_t(index)>=entries->size()){fail(i,"flattened parent index is invalid");return;}
                    auto& entry=(*entries)[index];
                    if(equipment(entry.nodeName)||(entry.node&&equipment(entry.node->name))) {
                        fail(i,"equipment attachment is an ancestor of a body bone");return;
                    }
                    if((entry.node&&entry.node==parent.node.get())||&entry.local==parent.flat){found=true;break;}
                    path.push_back(entry.node?SceneSlot{RE::NiPointer<RE::NiAVObject>(entry.node),nullptr}:
                        SceneSlot{{},&entry.local,&entry.world});
                    index=entry.parentIndex;
                }
                if(!found){fail(i,"mapped parent is outside flattened ancestor chain");return;}
            }
            for(const auto& item:path) {
                if(item.node)binding.bridgeParents.emplace(item.node.get(),item.node->parent);
                else for(std::size_t j=0;j<entries->size();++j)if(&(*entries)[j].local==item.flat) {
                    binding.flatBridgeParents.emplace(int(j),(*entries)[j].parentIndex);break;
                }
            }
            for(auto it=path.rbegin();it!=path.rend();++it)binding.basis[i]=compose(binding.basis[i],read(it->local()));
        });
        if(valid&&library.configureRig(binding.reference,binding.basis,binding.selected))return true;
        bindingFailure=reason;
        const int parent=failed>=0?library.parents[failed]:-1;
        SKSE::log::warn("Pose rig rejected: track={} name='{}' parent='{}' storage={} reason={}",failed,
            failed>=0?library.names[failed]:"<profile>",parent>=0?library.names[parent]:"<none>",
            failed>=0?(binding.scene.nodes[failed].node?"node":"flat"):"profile",reason);
        return false;
    }
    bool currentRig(const Binding& binding,bool verifyMembers=false) {
        if(!binding.graph->characterInstance.setup||binding.graph->characterInstance.setup->animationSkeleton.get()!=binding.skeleton.get())return rejectRig(RigIssue::graph);
        const auto* skeleton=binding.skeleton.get();
        if(skeleton->bones.data()!=binding.bones||skeleton->bones.size()!=binding.boneCount||
            skeleton->parentIndices.data()!=binding.parents||skeleton->parentIndices.size()!=binding.boneCount||
            skeleton->referencePose.data()!=binding.references||skeleton->referencePose.size()!=binding.boneCount)return rejectRig(RigIssue::arrays);
        if(!sameFlatStorage(binding.root.get(),binding.flatData,binding.flatCount))return rejectRig(RigIssue::storage);
        if(auto* player=RE::PlayerCharacter::GetSingleton();!player||player->Get3D(false)!=binding.actorRoot.get())return rejectRig(RigIssue::model);
        auto* ancestor=binding.root.get();
        for(unsigned depth=0;ancestor&&ancestor!=binding.actorRoot.get()&&depth<128;++depth)ancestor=ancestor->parent;
        if(ancestor!=binding.actorRoot.get())return rejectRig(RigIssue::model);
        for(const auto& [node,parent]:binding.bridgeParents) {
            if(node->parent!=parent)return rejectRig(RigIssue::hierarchy);
            if(!validSceneTransform(node->local))return rejectRig(RigIssue::transform);
        }
        for(const auto& [index,parent]:binding.flatBridgeParents) {
            if(binding.flatData[index].parentIndex!=parent)return rejectRig(RigIssue::hierarchy);
            if(!validSceneTransform(binding.flatData[index].local))return rejectRig(RigIssue::transform);
        }
        bool valid=true;
        binding.scene.each([&](std::size_t i,const auto& slot) {
            if(!valid)return;
            const auto fail=[&](RigIssue issue){valid=rejectRig(issue,int(i));};
            const int index=binding.animation.indices[i];
            if(index<0||index>=binding.boneCount||binding.parents[index]!=binding.animation.parents[i]||
                binding.bones[index].lockTranslation!=binding.locked[i]){fail(RigIssue::hierarchy);return;}
            if(verifyMembers&&(!binding.bones[index].name.c_str()||
                !binding.animation.current(i,{binding.bones[index].name.c_str(),binding.parents[index]}))){fail(RigIssue::hierarchy);return;}
            if(slot.node) {
                if(slot.node->parent!=binding.nodeParents[i]){fail(RigIssue::hierarchy);return;}
                if(verifyMembers&&slot.node->parent) {
                    bool present=false;
                    for(const auto& child:slot.node->parent->GetChildren())present|=child.get()==slot.node.get();
                    if(!present){fail(RigIssue::hierarchy);return;}
                }
            } else if(binding.flatIndices[i]<0||binding.flatData[binding.flatIndices[i]].parentIndex!=binding.flatParents[i]){fail(RigIssue::hierarchy);return;}
            if(!validSceneTransform(slot.local())){fail(RigIssue::transform);return;}
            if(!PoseRig<Pose>::currentStructure(read(slot.local()),read(binding.references[index]),binding.animationReference[i]))fail(RigIssue::reference);
        });
        return valid;
    }
    void invalidateRig() {
        acceptedScene.store(nullptr);target=nullptr;rigInvalidated=true;
    }
    static std::shared_ptr<const OutputRig> makeOutputRig(const Binding& binding) {
        auto rig=std::make_shared<OutputRig>();rig->root=binding.root;
        rig->parents.resize(99+binding.bridges.size());rig->flatData=binding.flatData;rig->flatCount=binding.flatCount;
        const auto add=[&](RE::NiAVObject* node,std::size_t slot) {
            rig->slots.emplace(node,slot);rig->nodes.emplace_back(node);
            rig->parents[slot]=node->parent;
            rig->worldNodes.emplace(&node->world,node);
        };
        binding.scene.each([&](std::size_t i,const auto& bone){if(bone.node)add(bone.node.get(),i);});
        for(std::size_t i=0;i<binding.bridges.size();++i)add(binding.bridges[i].get(),99+i);
        if(const auto entries=flatEntries(binding.root.get())) {
            for(auto& entry:*entries) {
                if(entry.node&&rig->slots.contains(entry.node)) {
                    rig->flatAliases[entry.node].push_back(&entry.world);
                    rig->worldNodes.emplace(&entry.world,entry.node);
                }
            }
        }
        return rig;
    }
    static RE::NiTransform acceptedLocal(const AcceptedScene& output,RE::NiAVObject* node,std::size_t slot) {
        auto local=output.locals[slot];
        local.scale=node->local.scale;
        if(slot==0&&output.wallFrame) {
            const auto parent=node->parent?read(node->parent->world).q:Quat{};
            write(local,WallYawFrame(parent,output.yaw).toParent(output.wallRoot));
        }
        return local;
    }
    void lateWorld(RE::NiAVObject* node,RE::NiUpdateData* data,WorldData original,void* caller) {
        const auto output=acceptedScene.load();
        if(!output){original(node,data);return;}
        const auto found=output->rig->slots.find(node);
        if(found==output->rig->slots.end()){original(node,data);return;}
        if(rigInvalidated.load()){original(node,data);return;}
        if(!sameFlatStorage(output->rig->root.get(),output->rig->flatData,output->rig->flatCount)) {
            rejectRig(RigIssue::storage);invalidateRig();original(node,data);return;
        }
        if(node->parent!=output->rig->parents[found->second]||!validSceneTransform(node->local)) {
            rejectRig(node->parent!=output->rig->parents[found->second]?RigIssue::hierarchy:RigIssue::transform,int(found->second));
            invalidateRig();original(node,data);return;
        }

        if(found->second<99) {
            const auto desired=acceptedLocal(*output,node,found->second);
            ScopedLocalOverride guard(node->local,desired);
            original(node,data);
        } else original(node,data);
        if(!sameFlatStorage(output->rig->root.get(),output->rig->flatData,output->rig->flatCount)) {
            rejectRig(RigIssue::storage);invalidateRig();return;
        }
        if(const auto aliases=output->rig->flatAliases.find(node);aliases!=output->rig->flatAliases.end())
            for(auto* alias:aliases->second)*alias=node->world;
        if(lateWorldUpdates.fetch_add(1)==0&&traceOutput) {
            HMODULE module{};wchar_t filename[MAX_PATH]{};
            if(GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                reinterpret_cast<LPCWSTR>(caller),&module))GetModuleFileNameW(module,filename,MAX_PATH);
            SKSE::log::info("Late world update protected: bone='{}' caller={}+0x{:X}; exact bound node; native local restored",
                node->name.c_str(),std::filesystem::path(filename).filename().string(),
                reinterpret_cast<std::uintptr_t>(caller)-reinterpret_cast<std::uintptr_t>(module));
        }
        if(found->second==0)++lateRootUpdates;
    }
    void observeSkin(RE::NiSkinInstance* skin,const RE::NiTransform* geometryWorld,SkinGenerator original,std::uint32_t frame) {
        if(!traceOutput||!skin||!target.load(std::memory_order_relaxed)){original(skin,geometryWorld);return;}
        skinCalls.fetch_add(1,std::memory_order_relaxed);

        if(!skinAuditBudget.trySample(frame)) {
            skinAuditSkipped.fetch_add(1,std::memory_order_relaxed);original(skin,geometryWorld);return;
        }
        skinAuditSamples.fetch_add(1,std::memory_order_relaxed);
        const auto auditStart=std::chrono::steady_clock::now();
        const auto output=acceptedScene.load();
        SkinAuditWork work;
        std::uint32_t owned=0,body=0,bodyErrors=0,bridgeErrors=0,extraErrors=0;
        float peakBodyDistance=0;int firstBody=-1;
        bool cacheHit=false;
        if(output) {

            static_assert(sizeof(skin->lock)==sizeof(CRITICAL_SECTION));
            auto* lock=reinterpret_cast<CRITICAL_SECTION*>(&skin->lock);
            if(::TryEnterCriticalSection(lock)) {
                struct Unlock {CRITICAL_SECTION* lock;~Unlock(){::LeaveCriticalSection(lock);}} guard{lock};
                cacheHit=skin->frameID==frame;

                if(!cacheHit&&skin->skinData&&skin->boneWorldTransforms&&skin->skinData->bones>0&&skin->skinData->bones<=4096) {
                    auto* root=output->rig->root.get();
                    const auto local=acceptedLocal(*output,root,0);
                    const auto rootWorld=root->parent?root->parent->world*local:local;
                    std::unordered_map<RE::NiAVObject*,std::optional<RE::NiTransform>> cache;
                    cache.reserve(SkinAuditBudget::ancestorsPerSample);
                    const auto parent=[](RE::NiAVObject* node)->RE::NiAVObject* {return node->parent;};
                    const auto localFor=[&](RE::NiAVObject* node)->std::optional<RE::NiTransform> {
                        if(!work.ancestor())return {};
                        if(node->name.c_str()&&(std::string_view(node->name.c_str())=="Camera3rd [Cam3]"||
                            std::string_view(node->name.c_str())=="Camera Control"))return {};
                        const auto slot=output->rig->slots.find(node);
                        return slot!=output->rig->slots.end()&&slot->second<99?acceptedLocal(*output,node,slot->second):node->local;
                    };
                    const auto compose=[](const RE::NiTransform& a,const RE::NiTransform& b){return a*b;};
                    const std::uint32_t count=skin->skinData->bones;
                    const auto start=std::uint32_t((std::uint64_t(frame)*SkinAuditBudget::inputsPerSample)%count);
                    for(std::uint32_t checked=0;checked<count&&work.input();++checked) {
                        const auto i=(start+checked)%count;
                        const auto* actual=skin->boneWorldTransforms[i];
                        if(!actual)continue;
                        RE::NiAVObject* node=skin->bones?skin->bones[i]:nullptr;
                        if(const auto known=output->rig->worldNodes.find(actual);known!=output->rig->worldNodes.end())node=known->second;
                        if(!node)continue;
                        const auto expected=ownedSkinWorld(node,root,rootWorld,parent,localFor,compose,cache);

                        if(!expected)continue;
                        ++owned;
                        const auto slot=output->rig->slots.find(node);
                        const auto kind=skinInputKind(slot==output->rig->slots.end()?std::nullopt:std::optional<std::size_t>(slot->second));
                        if(kind==SkinInputKind::body)++body;
                        float matrixDelta=0;bool finite=true;
                        for(int row=0;row<3;++row)for(int column=0;column<3;++column) {
                            const float delta=std::abs(expected->rotate.entry[row][column]-actual->rotate.entry[row][column]);
                            finite=finite&&std::isfinite(delta);matrixDelta=std::max(matrixDelta,delta);
                        }
                        const float distance=(Vec{expected->translate.x,expected->translate.y,expected->translate.z}-
                            Vec{actual->translate.x,actual->translate.y,actual->translate.z}).length();
                        if(!finite||!std::isfinite(distance)||matrixDelta>.01f||distance>.15f) {
                            if(kind==SkinInputKind::body){++bodyErrors;peakBodyDistance=std::max(peakBodyDistance,distance);if(firstBody<0)firstBody=int(slot->second);}
                            else if(kind==SkinInputKind::bridge)++bridgeErrors;
                            else ++extraErrors;
                        }
                    }
                }
            }else skinAuditLockSkips.fetch_add(1,std::memory_order_relaxed);
        }
        if(owned)ownedSkinCalls.fetch_add(1,std::memory_order_relaxed);
        skinBodyInputs.fetch_add(body,std::memory_order_relaxed);skinOwnedInputs.fetch_add(owned,std::memory_order_relaxed);
        if(cacheHit)skinCacheHits.fetch_add(1,std::memory_order_relaxed);
        skinAuditInputs.fetch_add(work.inputs,std::memory_order_relaxed);skinAuditAncestors.fetch_add(work.ancestors,std::memory_order_relaxed);
        if(work.exhausted)skinAuditBudgetStops.fetch_add(1,std::memory_order_relaxed);
        skinBodyMismatches.fetch_add(bodyErrors,std::memory_order_relaxed);skinBridgeMismatches.fetch_add(bridgeErrors,std::memory_order_relaxed);
        skinExtraMismatches.fetch_add(extraErrors,std::memory_order_relaxed);
        if(bodyErrors&&skinMismatchCalls.fetch_add(1,std::memory_order_relaxed)==0)
            SKSE::log::warn("Sampled final BODY skin inputs differ: checkedBody={} mismatchedBody={} firstTrack={} distancePeak={:.3f}; bridgeDifferences={} extraPhysicsDifferences={}; read-only bounded observer",
                body,bodyErrors,firstBody,peakBodyDistance,bridgeErrors,extraErrors);
        const auto micros=std::uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-auditStart).count());
        skinAuditMicros.fetch_add(micros,std::memory_order_relaxed);
        auto peak=skinAuditPeakMicros.load(std::memory_order_relaxed);
        while(peak<micros&&!skinAuditPeakMicros.compare_exchange_weak(peak,micros,std::memory_order_relaxed)){}
        original(skin,geometryWorld);
    }
    template<class Original> void apply(RE::NiNode* node,RE::NiUpdateData& data,ScenePass pass,Original original) {
        const auto applyStarted=std::chrono::steady_clock::now();
        ++callbacks;
        std::unique_lock lock(mutex);
        if(bindings.empty()||bindings[0].root.get()!=node){lock.unlock();original(data);return;}

        if(rigInvalidated.load()||!currentRig(bindings[0])) {
            invalidateRig();lock.unlock();original(data);return;
        }
        lastCallback=GetTickCount64();
        const float weight=envelope.weight();
        if(!overlaysPose(pass,weight>0&&published.size()==99)){lock.unlock();original(data);return;}
        auto& binding=bindings[0];++matched;
        std::array<RE::NiTransform,99> saved;
        std::array<RE::NiTransform,99> written;
        using NodeFlags=std::remove_cvref_t<decltype(node->GetFlags())>;
        std::array<NodeFlags,99> savedFlags;

        Pose native=library.rest;
        bool valid=true;
        binding.scene.each([&](std::size_t i,const auto& bone) {
            saved[i]=bone.local();native[i]=library.rig.toEffective(i,read(saved[i]));
            if(bone.node)savedFlags[i]=bone.node->GetFlags();
            if(!native[i].t.finite()||!std::isfinite(native[i].q.dot(native[i].q)))valid=false;
        });
        const auto parentRotation=node->parent?read(node->parent->world).q:Quat{};
        if(!std::isfinite(parentRotation.dot(parentRotation)))valid=false;
        if(!valid){++rejected;lock.unlock();original(data);return;}
        const bool outputHasWallFrame=hasWallFrame;
        const float outputYaw=publishedYaw;
        const WallYawFrame frame(parentRotation,outputYaw);

        if(outputHasWallFrame)native[0]=frame.toWall(native[0]);
        const auto output=handoff.evaluate(native,published,weight,recovery,sampleTime);
        const auto outputMotion=publishedMotion;
        const auto& desired=output.pose;
        const std::array<float,2> outputUpperRoll{
            sideRunUpperRoll(library,desired,0)*57.2957795f,
            sideRunUpperRoll(library,desired,1)*57.2957795f};
        binding.scene.each([&](std::size_t i,const auto& bone) {
            const auto local=library.rig.toLocal(i,desired[i]);
            write(bone.local(),i==0&&outputHasWallFrame?frame.toParent(local):local);
            written[i]=bone.local();
            if(bone.node) {
                bone.node->GetFlags()=static_cast<RE::NiAVObject::Flag>(ownedSceneNodeFlags(bone.node->GetFlags().underlying()));
            }
        });
        const auto scene=binding.scene;
        const auto lockedTranslations=binding.locked;
        const auto outputRig=binding.outputRig;
        const auto bridges=binding.bridges;
        std::vector<NodeFlags> bridgeFlags;
        bridgeFlags.reserve(bridges.size());
        for(const auto& bridge:bridges) {
            bridgeFlags.push_back(bridge->GetFlags());
            bridge->GetFlags()=static_cast<RE::NiAVObject::Flag>(ownedSceneNodeFlags(bridge->GetFlags().underlying()));
        }
        const auto outputGeneration=generation;

        const auto sceneRoot=binding.root;
        lock.unlock();
        const auto propagationStarted=std::chrono::steady_clock::now();

        RE::NiUpdateData update=data;
        update.flags=static_cast<RE::NiUpdateData::Flag>(effectiveUpdateDataFlags(pass,data.flags.underlying(),true));
        const bool repairedSelectedFlag=update.flags!=data.flags;
        original(update);
        if(!sameFlatStorage(sceneRoot.get(),outputRig->flatData,outputRig->flatCount)) {
            scene.each([&](std::size_t i,const auto& bone){if(bone.node){bone.node->local=saved[i];bone.node->GetFlags()=savedFlags[i];}});
            for(std::size_t i=0;i<bridges.size();++i)bridges[i]->GetFlags()=bridgeFlags[i];
            lock.lock();if(outputGeneration==generation){rejectRig(RigIssue::storage);invalidateRig();}else ++retiredOutputs;
            return;
        }

        const bool flatRefreshed=!refreshFlatAfterPass(pass,true)||refreshFlatWorld(sceneRoot.get());
        OutputAudit audit;
        if(!flatRefreshed){audit.worldMismatches=1;audit.firstWorldMismatch=0;}
        audit.pass=pass;
        audit.motion=outputMotion;audit.layerWeight=weight;audit.nativeRecovery=output.recovery;
        audit.upperRollDegrees=outputUpperRoll;
        audit.propagationMs=std::chrono::duration<float,std::milli>(std::chrono::steady_clock::now()-propagationStarted).count();
        if(traceOutput)scene.each([&](std::size_t i,const auto& bone) {
            const auto& expected=written[i];const auto& actual=bone.local();
            float matrixDelta=0;
            for(int row=0;row<3;++row)for(int column=0;column<3;++column)
                matrixDelta=std::max(matrixDelta,std::abs(expected.rotate.entry[row][column]-actual.rotate.entry[row][column]));
            const float distance=(Vec{expected.translate.x,expected.translate.y,expected.translate.z}-
                Vec{actual.translate.x,actual.translate.y,actual.translate.z}).length();

            const float angle=matrixDelta>.002f?angleBetween(read(expected).q,read(actual).q):0.f;
            if(angle>.01f||distance>.10f) {
                if(!audit.overwrittenBones)audit.firstOverwritten=int(i);
                ++audit.overwrittenBones;
                audit.overwrittenAngle=std::max(audit.overwrittenAngle,angle);
                audit.overwrittenDistance=std::max(audit.overwrittenDistance,distance);
            }
        });

        const auto auditWorld=[&](int boneIndex,const RE::NiTransform& expected,const RE::NiTransform& actual,bool cacheMatches) {
            float matrixDelta=0;
            bool finite=true;
            for(int row=0;row<3;++row)for(int column=0;column<3;++column)
            {
                const float delta=std::abs(expected.rotate.entry[row][column]-actual.rotate.entry[row][column]);
                finite=finite&&std::isfinite(delta);matrixDelta=std::max(matrixDelta,delta);
            }
            const float distance=(Vec{expected.translate.x,expected.translate.y,expected.translate.z}-
                Vec{actual.translate.x,actual.translate.y,actual.translate.z}).length();
            const float angle=matrixDelta>.002f?angleBetween(read(expected).q,read(actual).q):0.f;
            if(!finite||!std::isfinite(distance)||!std::isfinite(angle)||angle>.01f||distance>.10f||!cacheMatches) {
                if(!audit.worldMismatches)audit.firstWorldMismatch=boneIndex;
                ++audit.worldMismatches;
                audit.worldAngle=std::max(audit.worldAngle,angle);
                audit.worldDistance=std::max(audit.worldDistance,distance);
            }
        };
        scene.each([&](std::size_t i,const auto& bone){auditWorld(int(i),bone.expectedWorld(written[i]),bone.world(),bone.cacheMatches());});
        for(std::size_t i=0;i<bridges.size();++i) {
            const auto& bridge=bridges[i];
            auditWorld(int(99+i),bridge->parent?bridge->parent->world*bridge->local:bridge->local,bridge->world,true);
        }
        const auto renderedBasis=frameYaw(read(sceneRoot->world).q*desired[0].q.inverse());
        bool structureChanged=false;
        scene.each([&](std::size_t i,const auto& bone){
            const auto current=bone.local();auto restored=saved[i];
            if(!validSceneTransform(current)){rejectRig(RigIssue::transform,int(i));structureChanged=true;}
            else {
                restored.scale=current.scale;
                if(i!=0&&i!=4&&lockedTranslations[i]) {
                    const Vec actual{current.translate.x,current.translate.y,current.translate.z},expected{written[i].translate.x,written[i].translate.y,written[i].translate.z};
                    const Vec previous{saved[i].translate.x,saved[i].translate.y,saved[i].translate.z};
                    if((actual-expected).length()>.002f&&(actual-previous).length()>.002f)restored.translate=current.translate;
                }
            }
            bone.local()=restored;if(bone.node)bone.node->GetFlags()=savedFlags[i];
        });
        for(std::size_t i=0;i<bridges.size();++i)bridges[i]->GetFlags()=bridgeFlags[i];
        lock.lock();

        if(outputGeneration!=generation){++retiredOutputs;return;}
        if(structureChanged||rigInvalidated.load()){invalidateRig();return;}
        audit.worldMismatchFrames=outputAudit.worldMismatchFrames+(audit.worldMismatches?1:0);
        audit.transformPasses=outputAudit.transformPasses+(pass==ScenePass::transformOnly?1:0);
        audit.selectedFlagRepairs=outputAudit.selectedFlagRepairs+(repairedSelectedFlag?1:0);
        audit.callbackMs=std::chrono::duration<float,std::milli>(std::chrono::steady_clock::now()-applyStarted).count();
        audit.overwriteFrames=outputAudit.overwriteFrames+(audit.overwrittenBones?1:0);
        audit.lastOverwritten=audit.overwrittenBones?audit.firstOverwritten:outputAudit.lastOverwritten;
        audit.peakOverwrittenAngle=std::max(outputAudit.peakOverwrittenAngle,audit.overwrittenAngle);
        audit.peakOverwrittenDistance=std::max(outputAudit.peakOverwrittenDistance,audit.overwrittenDistance);
        audit.sampledAt=lastSuccessfulOutput;
        if(audit.worldMismatches) {
            if(!outputAudit.worldMismatchFrames)SKSE::log::warn("Scene output mismatch: pass={} bones={} firstBone={} angle={:.4f} distance={:.3f}; output not acknowledged",
                unsigned(pass),audit.worldMismatches,audit.firstWorldMismatch,audit.worldAngle,audit.worldDistance);
            ++rejected;outputAudit=audit;return;
        }
        if(!handoff.consumed(output)){++retiredOutputs;return;}
        auto accepted=std::make_shared<AcceptedScene>();
        accepted->rig=outputRig;accepted->locals=written;accepted->wallRoot=desired[0];
        accepted->yaw=outputYaw;accepted->wallFrame=outputHasWallFrame;accepted->epoch=outputGeneration;
        acceptedScene.store(std::move(accepted));
        displayedYaw=outputYaw;hasDisplayedYaw=outputHasWallFrame;
        lastSuccessfulOutput=GetTickCount64();
        audit.sampledAt=lastSuccessfulOutput;outputAudit=audit;
        const auto parentHeading=frameYaw(parentRotation);
        orientation={outputYaw,parentHeading.value_or(0),renderedBasis.value_or(0),
            parentHeading.has_value(),renderedBasis.has_value(),lastSuccessfulOutput};
        if(applied.fetch_add(1)==0&&traceOutput)SKSE::log::info("First final skeleton pose applied; {} mapped nodes / {} non-body virtual leaves / {} engine-owned equipment/camera tracks; native locals restored",scene.count,scene.virtualLeaves,scene.unownedTracks);
    }
    void restoreFootIKLocked() {
        for(auto& b:bindings)if(b.footOwned) {
            b.graph->doFootIK=b.footIK;b.footOwned=false;
        }
        nativeFootIK=true;
    }
    void clearLocked() {
        acceptedScene.store(nullptr);
        ++generation;lastSuccessfulOutput=0;
        target=nullptr;
        restoreFootIKLocked();
        bindings.clear();published.clear();handoff.clear();recovery=exitAge=sampleTime=0;reusedBindings=0;
        if(!library.rig.source().empty())library.clearRig();
        topEndpoint.clear();
        publishedYaw=displayedYaw=0;hasWallFrame=hasDisplayedYaw=false;orientation={};
        publishedMotion=Motion::none;outputAudit={};
        envelope.clear();fading=ownsPose=nativeFootIK=false;lastCallback=0;
    }
    bool bindLocked(RE::PlayerCharacter* player,bool* changed=nullptr) {
        const auto bindingStarted=std::chrono::steady_clock::now();
        RE::BSTSmartPointer<RE::BSAnimationGraphManager> manager;
        if(!player->GetAnimationGraphManager(manager)||!manager){bindingFailure="animation graph unavailable";return false;}
        auto* actorRoot=player->Get3D(false);
        if(!actorRoot){bindingFailure="third-person model unavailable";return false;}
        bindingFailure="no eligible third-person animation graph";
        for(auto& g:manager->graphs) {
            if(!g||g->holder!=player||!g->characterInstance.setup||!g->characterInstance.behaviorGraph)continue;
            if(std::string(g->projectName.c_str()).find("FirstPerson")!=std::string::npos)continue;
            auto* skeleton=g->characterInstance.setup->animationSkeleton.get();
            if(!skeleton||!runtime::readable(reinterpret_cast<std::uintptr_t>(skeleton),sizeof(*skeleton)))continue;
            const auto* bones=skeleton->bones.data();
            const int count=skeleton->bones.size();
            const auto* parents=skeleton->parentIndices.data();
            const auto* references=skeleton->referencePose.data();
            if(!bones||!parents||!references||count<=0||count>1024||skeleton->parentIndices.size()!=count||skeleton->referencePose.size()!=count||
                !runtime::readable(reinterpret_cast<std::uintptr_t>(bones),count*sizeof(*bones))||
                !runtime::readable(reinterpret_cast<std::uintptr_t>(parents),count*sizeof(*parents))||
                !runtime::readable(reinterpret_cast<std::uintptr_t>(references),count*sizeof(*references))) {
                bindingFailure="invalid animation skeleton layout";
                SKSE::log::warn("Pose binding rejected: {} (project={}, bones={}, parents={}, references={}, boneData={}, parentData={}, referenceData={})",
                    bindingFailure,g->projectName.c_str(),count,skeleton->parentIndices.size(),skeleton->referencePose.size(),
                    bool(bones),bool(parents),bool(references));continue;
            }
            if(!bindings.empty()&&bindings[0].graph==g&&bindings[0].actorRoot.get()==actorRoot&&
                bindings[0].root.get()==existingNode(actorRoot,library.names[0])&&currentRig(bindings[0],true)) {
                ++reusedBindings;bindingFailure="none";return true;
            }
            if(ownsPose||fading){invalidateRig();bindingFailure="character rig changed during traversal";return false;}
            Binding next;next.graph=g;next.actorRoot=RE::NiPointer<RE::NiAVObject>(actorRoot);
            next.skeleton=g->characterInstance.setup->animationSkeleton;next.bones=bones;next.parents=parents;next.references=references;next.boneCount=count;
            std::vector<AnimationSkeletonBone> animationBones;animationBones.reserve(count);
            for(int i=0;i<count;++i)animationBones.push_back({bones[i].name.c_str()?bones[i].name.c_str():"",parents[i]});
            next.animation=bindAnimationSkeleton(library.names,library.parents,animationBones);
            if(!next.animation) {
                bindingFailure=next.animation.error;
                const int track=next.animation.track,index=next.animation.actual;
                SKSE::log::warn("Pose binding rejected: {} (project={}, bones={}, track={}, expected='{}', actualIndex={}, actual='{}', actualParent={})",
                    bindingFailure,g->projectName.c_str(),count,track,track>=0?library.names[track]:"<none>",index,
                    index>=0?animationBones[index].name:std::string_view("<none>"),index>=0?animationBones[index].parent:-1);
                continue;
            }
            next.root=RE::NiPointer<RE::NiAVObject>(existingNode(actorRoot,library.names[0]));
            if(!next.root){bindingFailure="existing NPC Root unavailable";continue;}
            next.scene=bindRuntimeScene(next.root.get(),library.names,library.parents);
            if(!next.scene) {
                bindingFailure="required model bone missing";
                SKSE::log::warn("Pose binding rejected: {}: index={} name='{}' actorRoot='{}' project='{}' mapped={}",
                    bindingFailure,next.scene.missing,library.names[next.scene.missing],actorRoot->name.c_str(),g->projectName.c_str(),next.scene.count);
                continue;
            }
            for(std::size_t i=0;i<99;++i)if(next.animation.indices[i]<0&&next.scene.nodes[i]) {
                next.scene.nodes[i]={};--next.scene.count;++next.scene.virtualLeaves;
            }
            std::vector<RE::NiAVObject*> mapped;
            next.scene.each([&](std::size_t,const auto& slot){if(slot.node)mapped.push_back(slot.node.get());});
            const auto bridges=sceneBridgeNodes<RE::NiAVObject*>(mapped,next.root.get(),
                [](RE::NiAVObject* bone)->RE::NiAVObject* {return bone->parent;});
            if(!bridges) {bindingFailure="mapped bone outside owned skeleton root";continue;}
            for(auto* bridge:*bridges)next.bridges.emplace_back(bridge);
            next.footIK=g->doFootIK;
            bindingFailure="none";
            if(!captureRig(next))continue;
            next.outputRig=makeOutputRig(next);
            for(auto& old:bindings) {
                if(old.graph==g&&old.footOwned)next.footIK=old.footIK;
                if(old.footOwned)old.graph->doFootIK=old.footIK;
            }
            if(ownsPose&&!nativeFootIK){g->doFootIK=0;next.footOwned=true;}
            ++generation;lastSuccessfulOutput=0;
            acceptedScene.store(nullptr);
            orientation={};
            outputAudit={};
            target=nullptr;lastCallback=0;callbacks=0;
            bindings.clear();bindings.push_back(std::move(next));reusedBindings=0;
            rigIssue=0;target=bindings[0].root.get();
            if(changed)*changed=true;
            if(traceOutput) {
                SKSE::log::info("Final pose root bound: project={}, animationBones={}, mappedNodes={}, virtualLeaves={}, engineOwnedTracks={}, node={}",
                    g->projectName.c_str(),count,bindings[0].scene.count,bindings[0].scene.virtualLeaves,bindings[0].scene.unownedTracks,bindings[0].root->name.c_str());
                SKSE::log::info("Animation skeleton mapping: remapped={} aliases={} optionalMissing={}",
                    bindings[0].animation.remapped,bindings[0].animation.aliases,bindings[0].animation.missingOptional);
                SKSE::log::info("Pose scene class: {}",bindings[0].root->GetRTTI()->name);
                std::size_t flat=0;bindings[0].scene.each([&](std::size_t,const auto& slot){if(slot.flat)++flat;});
                SKSE::log::info("Pose storage: {} animated flat transforms; {} existing nodes; no nodes created",flat,bindings[0].scene.count-flat);
                SKSE::log::info("Scene propagation owns {} existing intermediate ancestors; locals preserved",bindings[0].bridges.size());
                SKSE::log::info("Character rig captured: adapted={} mapped={} bridges={} lockedTranslations={} bindingMs={:.3f}",
                    library.rig.active(),bindings[0].scene.count,bindings[0].bridges.size(),
                    std::count(bindings[0].locked.begin(),bindings[0].locked.end(),true),
                    std::chrono::duration<float,std::milli>(std::chrono::steady_clock::now()-bindingStarted).count());
            }
            return true;
        }
        return false;
    }
public:
    bool consumeRigInvalidated(){
        std::scoped_lock lock(mutex);
        if(!rigInvalidated.exchange(false))return false;
        constexpr std::array reasons{"unspecified","animation graph replaced","skeleton arrays replaced","scene storage replaced",
            "character model replaced","bone hierarchy changed","reference pose changed","invalid live transform"};
        const auto issue=rigIssue.load();const int bone=int(issue&0xFFFF)-1;
        SKSE::log::warn("Character rig invalidated: reason={} track={} name='{}'; releasing before rebinding",
            reasons[std::min<std::size_t>(issue>>16,reasons.size()-1)],bone,
            bone>=0&&bone<99?library.names[bone]:"<scene>");
        clearLocked();return true;
    }
    const char* bindingFailure="none";
    Library library;
    SurfacePose surface;
    std::atomic<bool> traceOutput{false};
    std::atomic<std::uint32_t> applied{},rejected{},callbacks{},matched{},childCallbacks{};
    std::atomic<std::uint32_t> retiredOutputs{};
    std::atomic<std::uint32_t> lateWorldUpdates{},lateRootUpdates{},skinCalls{},ownedSkinCalls{};
    std::atomic<std::uint32_t> skinBodyInputs{},skinOwnedInputs{},skinCacheHits{},skinMismatchCalls{};
    std::atomic<std::uint64_t> skinAuditSamples{},skinAuditSkipped{},skinAuditLockSkips{},skinAuditInputs{},skinAuditAncestors{},skinAuditBudgetStops{};
    std::atomic<std::uint64_t> skinBodyMismatches{},skinBridgeMismatches{},skinExtraMismatches{},skinAuditMicros{},skinAuditPeakMicros{};
    Orientation lastOrientation() {std::scoped_lock lock(mutex);return orientation;}
    OutputAudit lastOutputAudit() {std::scoped_lock lock(mutex);return outputAudit;}
    AnimationPackReport packReport;
    bool canReloadPack() {
        std::scoped_lock lock(mutex);return !ownsPose&&!fading&&target.load()==nullptr;
    }
    bool reloadPack() {
        if(!canReloadPack())return false;
        Library candidate;
        {
            std::scoped_lock lock(mutex);
            candidate.names=library.names;candidate.parents=library.parents;candidate.rest=library.rig.source().empty()?library.rest:library.rig.source();
            candidate.animationPack=library.animationPack;
        }
        auto report=loadAnimationPack(candidate,"Data/meshes/actors/character/animations/FreeClimb/pack.json");
        for(const auto& slot:report.slots)if(traceOutput||slot.status!=OverrideStatus::loaded)
            SKSE::log::info("HKX pack slot={} file={} status={} frames={} seconds={} reason={}",int(slot.motion),slot.file,int(slot.status),slot.samples,slot.seconds,slot.reason);
        if(!report.committed) {
            SKSE::log::error("Animation pack rejected; previous library preserved: {}",report.error);
            packReport=std::move(report);return false;
        }
        std::scoped_lock lock(mutex);
        if(ownsPose||fading||target.load()!=nullptr) {
            report.committed=false;report.error="Animation ownership changed during reload; retry after leaving the wall";
            packReport=std::move(report);return false;
        }
        clearLocked();library=std::move(candidate);surface.reset();packReport=std::move(report);
        SKSE::log::info("Animation pack committed: {} HKX clips; no legacy motion library",packReport.loaded);
        return true;
    }
    bool install() {
        if(!reloadPack())return false;
        if(!initializeFlatWorldRefresh()||!preflightHookSites())return false;
        instance=this;
        Hook<0,0>::install();Hook<0,1>::install();Hook<0,2>::install();
        Hook<1,0>::install();Hook<1,1>::install();Hook<1,2>::install();
        Hook<2,0>::install();Hook<2,1>::install();Hook<2,2>::install();
        TransformHook<0>::install();TransformHook<1>::install();TransformHook<2>::install();
        WorldHook<0>::install();WorldHook<1>::install();WorldHook<2>::install();WorldHook<3>::install();
        const bool skinObserver=SkinHook::install();
        SKSE::log::info("Single-bone world update protection installed; final skin observer={}",skinObserver);
        return true;
    }
    enum class Preparation { waiting, ready, rejected };
    Preparation prepare(RE::PlayerCharacter* player) {
        std::scoped_lock lock(mutex);

        if(fading)return Preparation::waiting;
        if(rigInvalidated.load())return Preparation::waiting;
        if(!bindLocked(player))return Preparation::rejected;
        return recentPoseCallback(GetTickCount64(),lastCallback.load())?Preparation::ready:Preparation::waiting;
    }
    void cancelPreparation() {
        std::scoped_lock lock(mutex);if(!ownsPose&&!fading)clearLocked();
    }
    bool attach(RE::PlayerCharacter* player) {
        std::scoped_lock lock(mutex);
        if(fading||rigInvalidated.load())return false;

        if(!bindLocked(player)||!recentPoseCallback(GetTickCount64(),lastCallback.load()))return false;
        ++generation;lastSuccessfulOutput=0;
        acceptedScene.store(nullptr);
        lateWorldUpdates=lateRootUpdates=skinCalls=ownedSkinCalls=0;
        skinBodyInputs=skinOwnedInputs=skinCacheHits=skinMismatchCalls=0;
        skinAuditSamples=skinAuditSkipped=skinAuditLockSkips=skinAuditInputs=skinAuditAncestors=skinAuditBudgetStops=0;
        skinBodyMismatches=skinBridgeMismatches=skinExtraMismatches=skinAuditMicros=skinAuditPeakMicros=0;
        surface.reset();published.clear();handoff.clear();recovery=exitAge=sampleTime=0;
        topEndpoint.clear();
        publishedYaw=displayedYaw=0;hasWallFrame=hasDisplayedYaw=false;orientation={};
        publishedMotion=Motion::none;outputAudit={};
        applied=0;rejected=0;matched=0;retiredOutputs=0;envelope.beginEntry();
        fading=nativeFootIK=false;ownsPose=true;
        for(auto& b:bindings){b.footIK=b.graph->doFootIK;b.graph->doFootIK=0;b.footOwned=true;}
        if(traceOutput)SKSE::log::info("Character rig attached: adapted={} validatedBindingReuses={}",library.rig.active(),reusedBindings);
        return true;
    }
    bool refresh(RE::PlayerCharacter* player) {
        std::scoped_lock lock(mutex);if(fading||published.empty())return false;
        bool changed=false;return bindLocked(player,&changed)&&changed;
    }
    void update(World& world,const Traversal& t,Motion motion,float dt,float actorScale) {
        {
            std::scoped_lock lock(mutex);
            if(bindings.empty()||rigInvalidated.load())return;
            if(!currentRig(bindings[0])){invalidateRig();return;}
        }
        auto pose=surface.update(library,world,t,motion,dt,actorScale);
        std::scoped_lock lock(mutex);
        if(rigInvalidated.load())return;
        published=std::move(pose);
        if(std::isfinite(dt)&&dt>0)sampleTime+=std::min(dt,.05f);
        publishedMotion=motion;
        if(const auto yaw=facingWallYaw(t.normal)){publishedYaw=*yaw;hasWallFrame=true;}
        recovery=topEndpoint.weight(motion,t.progress());
        envelope.advanceEntry(applied.load(),dt);fading=false;
    }
    bool requestTopRecovery(State state,float progress,float seconds) {
        std::scoped_lock lock(mutex);return ownsPose&&!fading&&topEndpoint.request(state,progress,seconds);
    }
    bool topRecoveryReady() {std::scoped_lock lock(mutex);return topEndpoint.ready();}
    void resolveTopRecovery(bool accepted) {
        std::scoped_lock lock(mutex);
        topEndpoint.resolve(accepted);

        if(topEndpoint.ready())restoreFootIKLocked();
    }
    void release(bool fade,bool physicalFall=false,bool completedTop=false) {
        std::scoped_lock lock(mutex);
        if(rigInvalidated.load()){clearLocked();return;}
        restoreFootIKLocked();
        const bool nativeTakeover=completedTop&&topEndpoint.ready()&&!physicalFall;
        if(fade&&envelope.weight()>0&&handoff.beginExit(physicalFall,nativeTakeover)) {
            const float remaining=handoff.exitContribution();

            publishedYaw=displayedYaw;hasWallFrame=hasDisplayedYaw;
            const float seconds=nativeTakeover?PoseHandoff::nativeExitSeconds:
                physicalFall?PoseBlendEnvelope::fallExitSeconds:PoseBlendEnvelope::exitSeconds;
            if(traceOutput)SKSE::log::info("Pose release: consumed recovery={:.6f}; remaining custom={:.6f}; live native continuation={}; physicalFall={}; fadeSeconds={:.2f}; consumedYaw={:.3f}; completedTopBridge={}",
                1-remaining,remaining,remaining<1.f,physicalFall,seconds,publishedYaw,nativeTakeover);
            if(remaining<=0&&!handoff.nativeExitActive())clearLocked();
            else {envelope.beginExit(applied.load(),seconds);exitAge=0;fading=true;}
        } else clearLocked();
    }
    void tick(float dt) {
        std::scoped_lock lock(mutex);if(!fading||rigInvalidated.load())return;
        if(std::isfinite(dt)&&dt>0)exitAge+=std::min(dt,.05f);

        if(exitAge>.75f&&!recentPoseCallback(GetTickCount64(),lastSuccessfulOutput)){clearLocked();return;}
        envelope.advanceExit(applied.load(),dt);

        handoff.advanceExitSource(exitAge,library,envelope.elapsedExitSeconds());
        if(envelope.weight()<=0)clearLocked();
    }
};
}
