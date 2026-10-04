#include "PCH.h"
#include "RuntimeSupport.h"
#include "RuntimeLog.h"
#include "Core.h"
#include "Controls.h"
#include "InputBindings.h"
#include "GamepadInput.h"
#include "MenuInterruption.h"
#include <SKSE/InputMap.h>
#include "UserSettings.h"
#include "SettingsMenu.h"
#include "ControllerGravityLease.h"
#include "GroundMotionProbe.h"
#include "NativeShapeWitness.h"
#include "NativeWalkableApproach.h"
#include "TraversalCapture.h"
#include "PoseRuntime.h"
#include "TraversalAudioRuntime.h"
#include "ViewHeading.h"
#include "CameraHeading.h"
#include "FilteredRayCollector.h"
#include "RayFilterPolicy.h"
#include "RayCandidateCache.h"
#include "vendor/TrueDirectionalMovementAPI.h"
#include <chrono>

namespace {
fc::Traversal traversal;
fc::JumpGrabGate jumpGrab;
fc::ClimbEntryIntent climbEntry;
fc::NativeJumpIntent nativeJumpIntent;
fc::EntryPreparationGrace entryPreparationGrace;
fc::InputState inputState;
fc::InputOwnership inputOwnership;
fc::GamepadState gamepadState;
fc::GamepadOwnership gamepadOwnership;
bool gamepadAvailable{},gamepadPreferred{},gamepadOwned{},gamepadLost{},gamepadHookReady{};
fc::WallRunEntryGate wallRunEntryGate;
fc::PoseRuntime poses;
fc::TraversalAudioRuntime traversalAudio;
fc::ViewHeading viewHeading;
fc::GroundMotionProbe groundMotionProbe;
fc::TraversalCapture geometryCapture;
fc::TraversalCapture::SessionGate geometryCaptureGate;
RE::TESGlobal* animationState{};
RE::NiPointer<RE::bhkCharacterController> ownedController;
RE::TESObjectCELL* climbingCell{};
RE::TESWorldSpace* climbingWorldspace{};
TDM_API::IVTDM3* tdm{};
bool yawOwned{},directionOwned{},syncOwned{};
std::atomic<bool> ready{};
bool savedRunning{},runningSaved{},idleRunning=true,idleRunningKnown{};
fc::ControllerGravityLease<RE::bhkCharacterController> controllerGravity;
fc::PoseHealth poseHealth;
std::uint64_t preparationStarted{};
bool preparationChangedView{};
fc::Motion lastMotion=fc::Motion::none;
bool lastHop{};
float grabProbeCooldown{};
bool notifications=true,enabled=true;
bool jumpToAttach=true,autoMantle=true,diagnostics=false;
float grabMaxSnap=60,diagnosticCooldown{},attachedTime{};
float approachSpeed{},stallReportTime{},poseRefreshTime{},wallRunSpeedOverride{},yawReportTime{};
float outputReportTime{},peakTraversalMs{},peakPublishMs{};
int peakFrameCasts{};
std::uint64_t peakFrameRayCandidates{},peakFrameRayClassifications{};
unsigned lastContextStatus{},lastThreepeatStatus{};
bool lastCornerReported{},lastEdgeReported{};
float contextReportTime{};
unsigned contextHops{},contextCorners{},contextEaves{};
unsigned lastAutomaticAction{},automaticActionBase{},lastObstacleJump{},obstacleJumpBase{};
unsigned lastAutomaticAttempt{},automaticAttemptBase{};
unsigned opportunityBase{},surfaceActionBase{},idleActionBase{};
fc::Motion lastRenderedMotion=fc::Motion::none;
std::uint64_t lastRenderedSample{};
std::array<unsigned,fc::motionCount+1> renderedUses{};
std::array<float,fc::motionCount+1> selectedSeconds{},observedSpanSeconds{};
bool observedSpanContinuous{};

std::array<float,6> inputSeconds{};
unsigned attachmentsSinceLoad{};
unsigned nativeShapeBaselineSamples{},nativeShapeBaselineAttempts{};
float nativeShapeBaselineAge{},nativeShapeBaselineRetry{};
float crestAuditTime=-1;
unsigned crestAuditPhase{};
bool crestAuditSupported{};
fc::Vec crestAuditStart{};
bool lowStaminaNoted{};
bool lowStaminaNotifications=true,dataLoaded{},menuRegistered{},audioReady{};
std::mutex settingsMutex;
fc::UserSettings activeSettings,desiredSettings;
std::optional<fc::UserSettings> pendingSettings;
std::optional<std::pair<bool,float>> pendingAudio;
bool pendingSave{},pendingReload{};
std::uint64_t settingsRevision{},reloadRevision{};
fc::SettingsMenuSnapshot menuSnapshot;
std::string settingsStatus="not_ready",settingsError;
std::array<std::uint64_t,fc::motionCount+1> totalMotionUses{},totalObservedUses{};
std::uint64_t nextMenuSnapshot{};
void serviceSettings();
void initializeRuntime();
void refreshMenuSnapshot();
std::array<std::uint32_t,fc::motionCount+1> motionUses{};
fc::HookPublication<void(*)(RE::PlayerCharacter*,float)> originalUpdate;

bool gamepadSelected() {
    if(traversal.active())return gamepadOwned;
    if(!gamepadHookReady||!gamepadAvailable||!activeSettings.gamepad.enabled)return false;
    if(fc::entryChord(fc::mapKeys(inputState,activeSettings.bindings)))return false;
    return gamepadState.keys(activeSettings.gamepad.bindings).entry||gamepadPreferred;
}
fc::Keys keys() {
    return gamepadSelected()?gamepadState.keys(activeSettings.gamepad.bindings):fc::mapKeys(inputState,activeSettings.bindings);
}
unsigned gamepadButtonIndex(std::uint32_t mask) {
    if(!RE::ControlMap::GetSingleton())return 16;
    const auto code=SKSE::InputMap::GamepadMaskToKeycode(mask);
    return code>=SKSE::InputMap::kMacro_GamepadOffset?code-SKSE::InputMap::kMacro_GamepadOffset:16;
}
bool nativeJumpHeld() {
    const auto* controls=RE::ControlMap::GetSingleton();
    if(!controls)return false;
    const auto keyboard=controls->GetMappedKey("Jump",RE::INPUT_DEVICE::kKeyboard);
    const auto gamepad=gamepadButtonIndex(controls->GetMappedKey("Jump",RE::INPUT_DEVICE::kGamepad));
    return (keyboard<256&&inputState.held(static_cast<fc::KeyCode>(keyboard))&&inputOwnership.nativeDown(keyboard))||
        (gamepadAvailable&&gamepadState.held(gamepad)&&gamepadOwnership.startedNativeJump(gamepad));
}

RE::NiPoint3 ni(fc::Vec v) { return {v.x,v.y,v.z}; }
fc::Vec vec(RE::NiPoint3 v) { return {v.x,v.y,v.z}; }
bool graph(RE::Actor* p,const char* name) { bool result=false; p->GetGraphVariableBool(name,result); return result; }
void note(const char* text) { if(notifications) RE::SendHUDMessage::ShowHUDMessage(text); }
bool foreground() { DWORD pid=0; GetWindowThreadProcessId(GetForegroundWindow(),&pid); return pid==GetCurrentProcessId(); }


struct ExitNativeDiagnostics {
    std::uint64_t started{};
    unsigned next{};
    RE::FormID cell{};
    fc::Vec origin{};
    void reset(){started=0;}
    void begin(RE::PlayerCharacter* player) {
        reset();
        if(!diagnostics||!ready||!enabled||!player||player!=RE::PlayerCharacter::GetSingleton())return;
        const auto* currentCell=player->GetParentCell();
        const auto position=vec(player->GetPosition());
        if(!currentCell||!player->GetCharController()||!position.finite())return;
        cell=currentCell->GetFormID();origin=position;next=0;started=GetTickCount64();
    }
    void sample(RE::PlayerCharacter* player,float dt,bool newEntry) {
        if(!started)return;
        if(!diagnostics||!ready||!enabled||traversal.active()||newEntry||!player||player!=RE::PlayerCharacter::GetSingleton()) {reset();return;}
        const auto now=GetTickCount64();
        const auto elapsed=now-started;
        const auto* currentCell=player->GetParentCell();
        const auto* controller=player->GetCharController();
        const auto* controls=RE::PlayerControls::GetSingleton();
        const auto position=vec(player->GetPosition());
        const float scale=RE::bhkWorld::GetWorldScale();
        if(elapsed>750||!currentCell||currentCell->GetFormID()!=cell||!controller||!controls||!position.finite()||
            !std::isfinite(dt)||dt<=0||!std::isfinite(scale)||scale<=0) {reset();return;}
        constexpr std::array<std::uint64_t,4> times{0,100,250,500};
        if(elapsed<times[next])return;
        const auto displacement=position-origin;
        const auto& velocity=controller->outVelocity.quad;
        SKSE::log::info("Top native resume: sample={} wallMs={} dt={:.5f} position=({:.2f},{:.2f},{:.2f}) displacement=({:.2f},{:.2f},{:.2f}) move=({:.3f},{:.3f}) velocity=({:.2f},{:.2f},{:.2f}) current={} wanted={} support={} synced={} animationDriven={}",
            next,elapsed,dt,position.x,position.y,position.z,displacement.x,displacement.y,displacement.z,
            controls->data.moveInputVec.x,controls->data.moveInputVec.y,velocity.m128_f32[0]/scale,velocity.m128_f32[1]/scale,velocity.m128_f32[2]/scale,
            int(controller->context.currentState),int(controller->wantState),int(controller->surfaceInfo.supportedState.get()),
            graph(player,"bIsSynced"),player->IsAnimationDriven());
        while(++next<times.size()&&elapsed>=times[next]){}
        if(next==times.size())reset();
    }
} exitNativeDiagnostics;


struct EntryLookDiagnostics {
    bool tracking{};
    float elapsed{},next{},peakProbeMs{};
    unsigned probes{};
    float beforeActorYaw{},beforeCameraYaw{},beforeLookX{},beforeLookY{};
    void beforeNative(RE::PlayerCharacter* player) {
        if(!diagnostics||(!tracking&&!fc::entryChord(keys())))return;
        beforeActorYaw=player->GetAngleZ();
        const auto* camera=RE::PlayerCamera::GetSingleton();
        const auto* state=camera?camera->currentState.get():nullptr;
        const auto* third=state&&state->id==RE::CameraState::kThirdPerson?static_cast<const RE::ThirdPersonState*>(state):nullptr;
        beforeCameraYaw=third?third->currentYaw:0.f;
        if(const auto* controls=RE::PlayerControls::GetSingleton()){beforeLookX=controls->data.lookInputVec.x;beforeLookY=controls->data.lookInputVec.y;}
    }
    std::array<std::atomic<std::uint64_t>,2> mouseEvents{};
    std::array<std::atomic<std::int64_t>,2> mouseX{},mouseY{};
    void queue(RE::InputEvent* first,unsigned stage) {
        if(!diagnostics)return;
        unsigned visited=0;
        for(auto* event=first;event&&visited++<2048;event=event->next)if(const auto* mouse=event->AsMouseMoveEvent()) {
            mouseEvents[stage].fetch_add(1,std::memory_order_relaxed);
            mouseX[stage].fetch_add(mouse->mouseInputX,std::memory_order_relaxed);
            mouseY[stage].fetch_add(mouse->mouseInputY,std::memory_order_relaxed);
        }
    }
    void sample(RE::PlayerCharacter* player,fc::Keys keys,float dt) {
        if(!diagnostics){tracking=false;return;}
        const bool held=fc::entryChord(keys),begin=held&&!tracking,end=!held&&tracking;
        if(begin){tracking=true;elapsed=next=peakProbeMs=0;probes=0;}
        if(!tracking)return;
        elapsed+=std::clamp(dt,0.f,.05f);
        if(!begin&&!end&&elapsed<next)return;
        const auto* camera=RE::PlayerCamera::GetSingleton();
        const auto* controls=RE::PlayerControls::GetSingleton();
        const auto* map=RE::ControlMap::GetSingleton();
        const auto* state=camera?camera->currentState.get():nullptr;
        const auto* third=state&&state->id==RE::CameraState::kThirdPerson?static_cast<const RE::ThirdPersonState*>(state):nullptr;
        SKSE::log::info("Entry look: event={} t={:.2f} held={} state={} camera={} actorYaw={:.4f} cameraYaw={:.4f} free=({:.4f},{:.4f}) freeEnabled={} lookEnabled={} input=({:.5f},{:.5f}); beforeNativeActorYaw={:.4f} beforeNativeCameraYaw={:.4f} beforeNativeInput=({:.5f},{:.5f}); ownsController={} ownsYaw={} ownsDirection={} ownsSync={} synced={} animationDriven={} preparing={} tdmMode={} tdmOwner={}; probes={} peakMs={:.3f}; queueMouseBefore={} ({},{}) after={} ({},{})",
            begin?"begin":end?"end":"held",elapsed,held,int(traversal.state),state?int(state->id):-1,player->GetAngleZ(),
            third?third->currentYaw:0.f,third?third->freeRotation.x:0.f,third?third->freeRotation.y:0.f,third&&third->freeRotationEnabled,
            map&&map->IsLookingControlsEnabled(),controls?controls->data.lookInputVec.x:0.f,controls?controls->data.lookInputVec.y:0.f,
            beforeActorYaw,beforeCameraYaw,beforeLookX,beforeLookY,bool(ownedController),yawOwned,directionOwned,syncOwned,graph(player,"bIsSynced"),player->IsAnimationDriven(),preparationStarted!=0,
            tdm?int(tdm->GetDirectionalMovementMode()):-1,tdm?tdm->GetDisableDirectionalMovementOwner():0,
            probes,peakProbeMs,mouseEvents[0].exchange(0),mouseX[0].exchange(0),mouseY[0].exchange(0),
            mouseEvents[1].exchange(0),mouseX[1].exchange(0),mouseY[1].exchange(0));
        next=elapsed+1;peakProbeMs=0;probes=0;
        if(end)tracking=false;
    }
    struct Probe {
        EntryLookDiagnostics& owner;
        bool enabled;
        std::chrono::steady_clock::time_point start;
        explicit Probe(EntryLookDiagnostics& value):owner(value),enabled(diagnostics),start(std::chrono::steady_clock::now()){}
        ~Probe(){if(enabled){++owner.probes;owner.peakProbeMs=std::max(owner.peakProbeMs,std::chrono::duration<float,std::milli>(std::chrono::steady_clock::now()-start).count());}}
    };
} entryLookDiagnostics;

fc::MenuInputState menuInputState(const RE::IMenu& menu) {
    using Context=RE::UserEvents::INPUT_CONTEXT_ID;
    bool input=menu.UsesMenuContext();
    switch(menu.inputContext.get()) {
    case Context::kMenuMode: case Context::kConsole: case Context::kItemMenu: case Context::kInventory:
    case Context::kFavorites: case Context::kMap: case Context::kStats: case Context::kBook:
    case Context::kJournal: case Context::kLockpicking: input=true;break;
    default: break;
    }
    return {menu.PausesGame(),menu.InventoryItemMenu(),input};
}
bool grabInputSuspended() {
    const auto ui=RE::UI::GetSingleton();
    if(!foreground()||!ui||fc::settingsMenuBlocking())return true;
    RE::BSSpinLockGuard lock(ui->processMessagesLock);
    if(ui->GameIsPaused()||ui->numItemMenus>0||ui->IsMenuOpen("Console")||ui->IsMenuOpen("Dialogue Menu")||
        ui->IsMenuOpen("Loading Menu")||ui->IsMenuOpen("TweenMenu"))return true;
    for(const auto& menu:ui->menuStack)if(menu&&fc::menuBlocksTraversal({},menuInputState(*menu)))return true;
    return false;
}
void cancelEntryPreparation() {
    entryPreparationGrace.cancel();poses.cancelPreparation();preparationStarted=0;
    if(preparationChangedView) {
        if(auto* camera=RE::PlayerCamera::GetSingleton();camera&&camera->IsInThirdPerson())camera->ForceFirstPerson();
        preparationChangedView=false;
    }
}
void cancelGrabRequest() {

    if(jumpGrab.pending()||preparationStarted||preparationChangedView)cancelEntryPreparation();
    entryPreparationGrace.cancel();jumpGrab.tick(0,true);preparationStarted=0;
    climbEntry.blockUntilRelease();grabProbeCooldown=0;
}

void stopLocomotion() {
    if(auto controls=RE::PlayerControls::GetSingleton()) {
        controls->data.moveInputVec={0,0};
        controls->data.prevMoveVec={0,0};
    }
    if(ownedController) {
        const RE::hkVector4 zero(0,0,0,0);
        controllerGravity.suppress();
        ownedController->outVelocity=zero;
        ownedController->initialVelocity=zero;
        ownedController->velocityMod=zero;
        ownedController->SetLinearVelocityImpl(zero);
    }
}

void takeController(RE::bhkCharacterController* controller,bool replacing) {
    controllerGravity.take(controller);
    ownedController=RE::NiPointer<RE::bhkCharacterController>(controller);
    if(replacing) {
        SKSE::log::info("Continued climbing with replacement controller; restored gravity={:.3f}",controllerGravity.savedGravity());
        stopLocomotion();
    }
}

void fitBody(RE::PlayerCharacter* p) {
    traversal.cfg.runSpeed=wallRunSpeedOverride>0?wallRunSpeedOverride:379.5f;
    const float scale=p->GetScale();
    traversal.cfg.contextScale=std::isfinite(scale)?std::clamp(scale,.5f,2.f):1.f;
    const auto& extents=p->GetCharController()->collisionBound.extents;
    if(std::isfinite(extents.x)&&std::isfinite(extents.y)&&std::isfinite(extents.z)&&extents.z>30&&extents.z<120) {
        traversal.cfg.radius=std::clamp(std::max(extents.x,extents.y),22.0f,45.0f);
        traversal.cfg.gap=traversal.cfg.radius+6;
        traversal.cfg.height=std::clamp(extents.z*2+3,125.0f,200.0f);
    }
}

struct GameWorld final:fc::World {
    RE::PlayerCharacter* player;
    RE::hkRefPtr<RE::hkpShapePhantom> playerPhantom;
    fc::RayCollectorValidationCache<RE::NiPointer<RE::bhkWorld>,fc::FilteredRayCollector::NativeAdd> rayCollectorValidation;
    std::uint64_t rayCandidates{},rayClassifications{};
    int casts{},hits{},selfHits{},controllerSelfHits{},firstLayer=-1;
    std::uint32_t firstReference{},firstBaseReference{},firstFilterInfo{};
    int firstFormType=-1,firstBroadphase=-1,firstMotionType=-1,firstResponseType=-1;
    fc::Vec firstNormal{},firstHitPoint{};
    std::string firstStaticModel;
    float firstDistance{};
    int ignoredTriggerHits{},firstTriggerLayer=-1,firstTriggerBroadphase=-1;
    int firstTriggerMotionType=-1,firstTriggerResponseType=-1,firstTriggerFormType=-1;
    std::uint32_t firstTriggerReference{},firstTriggerBaseReference{},firstTriggerFilterInfo{};
    fc::Vec firstTriggerHitPoint{};
    float firstTriggerDistance{};
    bool firstTriggerPrimitive{};
    explicit GameWorld(RE::PlayerCharacter* p):player(p){

        if(auto* controller=skyrim_cast<RE::bhkCharProxyController*>(p->GetCharController()))
            if(auto* proxy=controller->GetCharacterProxy())playerPhantom=RE::hkRefPtr<RE::hkpShapePhantom>(proxy->shapePhantom);
    }
    struct Candidate {
        const RE::TESObjectREFR* reference{};
        bool ownController{};
        fc::RayDecision decision=fc::RayDecision::keep;
    };
    Candidate classify(const RE::hkpCollidable& body) {
        ++rayClassifications;
        const auto* ref=RE::TESHavokUtilities::FindCollidableRef(body);
        const bool ownController=playerPhantom&&playerPhantom->GetCollidable()==&body;
        fc::RayHitFacts facts;
        facts.exactPlayer=ref==player||ownController;
        facts.actorZone=body.GetCollisionLayer()==RE::COL_LAYER::kActorZone;
        facts.actor=ref&&ref->Is(RE::FormType::ActorCharacter);
        if(facts.actorZone&&!facts.actor&&!facts.exactPlayer) {
            facts.entity=body.broadPhaseHandle.type==static_cast<int>(RE::hkpWorldObject::BroadPhaseType::kEntity);
            facts.phantom=body.broadPhaseHandle.type==static_cast<int>(RE::hkpWorldObject::BroadPhaseType::kPhantom);
            if(facts.entity) {
                const auto* entity=body.GetOwner<RE::hkpEntity>();
                if(entity)switch(*entity->material.responseType) {
                case RE::hkpMaterial::ResponseType::kSimpleContact:facts.response=fc::RayResponse::simpleContact;break;
                case RE::hkpMaterial::ResponseType::kReporting:facts.response=fc::RayResponse::reporting;break;
                case RE::hkpMaterial::ResponseType::kNone:facts.response=fc::RayResponse::none;break;
                default:break;
                }
            } else if(facts.phantom&&ref) {

                const auto* base=ref->GetBaseObject();
                const auto* acti=base?base->As<RE::TESObjectACTI>():nullptr;
                const auto* primitive=ref->extraList.GetByType<RE::ExtraPrimitive>();
                const char* model=acti?acti->GetModel():nullptr;
                facts.primitiveActivatorWithoutModel=acti&&primitive&&primitive->primitive&&(!model||!*model);
            }
        }
        return {ref,ownController,fc::rayHitDecision(facts)};
    }
    struct RayContext {
        GameWorld& owner;
        const RE::hkpCollidable* firstTrigger{};
        float firstTriggerFraction{};
        std::optional<fc::RayCandidateCache<Candidate>> candidates;
        const Candidate& classify(const RE::hkpCollidable& body) {
            if(!candidates)candidates.emplace();
            return candidates->resolve(&body,[&]{return owner.classify(body);});
        }
    };
    static bool keepRayHit(void* opaque,const RE::hkpCollidable& body,float fraction) {
        auto& context=*static_cast<RayContext*>(opaque);
        auto& owner=context.owner;
        ++owner.rayCandidates;
        const auto& candidate=context.classify(body);
        const auto decision=candidate.decision;
        const bool accepted=decision==fc::RayDecision::keep;
        if(decision==fc::RayDecision::ignorePlayer) {
            ++owner.selfHits;owner.controllerSelfHits+=candidate.ownController;
        } else if(decision==fc::RayDecision::ignoreTrigger) {
            ++owner.ignoredTriggerHits;
            if(owner.firstTriggerLayer<0&&!context.firstTrigger) {
                context.firstTrigger=&body;context.firstTriggerFraction=fraction;
            }
        }
        return accepted;
    }
    void recordIgnoredTrigger(RayContext& context,fc::Vec from,fc::Vec to) {
        const auto* body=context.firstTrigger;
        if(!body||firstTriggerLayer>=0)return;
        firstTriggerLayer=static_cast<int>(body->GetCollisionLayer());
        firstTriggerFilterInfo=body->broadPhaseHandle.collisionFilterInfo.filter;
        firstTriggerBroadphase=static_cast<int>(body->broadPhaseHandle.type);
        firstTriggerHitPoint=from+(to-from)*context.firstTriggerFraction;
        firstTriggerDistance=(firstTriggerHitPoint-from).length();
        const auto* ref=context.classify(*body).reference;
        if(ref) {
            firstTriggerReference=ref->GetFormID();
            if(const auto* base=ref->GetBaseObject()) {
                firstTriggerBaseReference=base->GetFormID();firstTriggerFormType=static_cast<int>(base->GetFormType());
            }
            const auto* primitive=ref->extraList.GetByType<RE::ExtraPrimitive>();
            firstTriggerPrimitive=primitive&&primitive->primitive;
        }
        if(firstTriggerBroadphase==static_cast<int>(RE::hkpWorldObject::BroadPhaseType::kEntity)) {
            if(const auto* entity=body->GetOwner<RE::hkpEntity>()) {
                firstTriggerMotionType=static_cast<int>(*entity->motion.type);
                firstTriggerResponseType=static_cast<int>(*entity->material.responseType);
            }
        }
    }
    bool actionBodyClear(fc::Motion motion,fc::Vec from,fc::Vec to,float fromPhase,float toPhase,fc::Vec outward) override {
        return motion==fc::Motion::backFlipOut&&fc::backFlipBodyClear(*this,poses.library,
            from,to,fromPhase,toPhase,outward,traversal.surfaceNormal.z,traversal.cfg.gap,player->GetScale());
    }
    std::optional<fc::Hit> ray(fc::Vec from,fc::Vec to) override {
        auto cell=player->GetParentCell();
        auto world=cell?cell->GetbhkWorld():nullptr;
        rayCollectorValidation.bind(world);
        if(!world) return fc::Hit{from,{0,0,1},false};
        const auto scale=RE::bhkWorld::GetWorldScale();
        RE::CFilter filter{}; player->GetCollisionFilterInfo(filter);
        RE::BSReadLockGuard lock(world->worldLock);
        {
            RE::bhkPickData pick{};
            pick.rayOutput.Reset();
            pick.rayInput.from=RE::hkVector4(ni(from)*scale);
            pick.rayInput.to=RE::hkVector4(ni(to)*scale);
            pick.rayInput.filterInfo.filter=(filter.filter&0xFFFF0000)|static_cast<std::uint32_t>(RE::COL_LAYER::kCharController);
            RayContext context{*this};
            const auto table=*reinterpret_cast<const std::uintptr_t*>(world);
            const auto nativeAdd=rayCollectorValidation.resolve(table,[](std::uintptr_t value) {
                return reinterpret_cast<const std::uintptr_t*>(value)[0x33];
            },fc::verifiedRayCollectorAdd);
            fc::FilteredRayCollector collector(&context,keepRayHit,nativeAdd);
            if(nativeAdd)pick.closestRayHitCollector=collector.enginePrefix();
            ++casts;
            const bool hit=world->PickObject(pick);

            recordIgnoredTrigger(context,from,to);
            if(collector.malformed)return fc::Hit{from,{0,0,1},false};
            if(!hit||!pick.rayOutput.HasHit()) return {};
            const auto& r=pick.rayOutput;
            const auto point=from+(to-from)*r.hitFraction;
            const auto* ref=nativeAdd?context.classify(*r.rootCollidable).reference:
                RE::TESHavokUtilities::FindCollidableRef(*r.rootCollidable);

            const bool ownController=playerPhantom&&playerPhantom->GetCollidable()==r.rootCollidable;
            if(ref==player||ownController) {
                ++selfHits;
                controllerSelfHits+=ownController;
                return fc::Hit{point,{0,0,1},false};
            }
            const auto layer=r.rootCollidable->GetCollisionLayer();
            const bool tree=layer==RE::COL_LAYER::kTrees;
            bool stable=layer==RE::COL_LAYER::kStatic||layer==RE::COL_LAYER::kGround||layer==RE::COL_LAYER::kTerrain;

            if(!stable&&(tree||layer==RE::COL_LAYER::kClutter||layer==RE::COL_LAYER::kWard||layer==RE::COL_LAYER::kProps)&&
                (tree||(ref&&ref->GetBaseObject()&&ref->GetBaseObject()->Is(RE::FormType::Static)))&&
                r.rootCollidable->broadPhaseHandle.type==static_cast<int>(RE::hkpWorldObject::BroadPhaseType::kEntity)) {
                const auto entity=r.rootCollidable->GetOwner<RE::hkpEntity>();
                stable=entity&&entity->motion.type==RE::hkpMotion::MotionType::kFixed&&
                    (!tree||entity->material.responseType==RE::hkpMaterial::ResponseType::kSimpleContact);
            }
            const auto& n=r.normal.quad;
            const fc::Vec normal{n.m128_f32[0],n.m128_f32[1],n.m128_f32[2]};
            if(hits++==0) {
                firstLayer=static_cast<int>(layer);firstNormal=normal;firstHitPoint=point;
                firstDistance=(point-from).length();firstReference=ref?ref->GetFormID():0;
                firstFilterInfo=r.rootCollidable->broadPhaseHandle.collisionFilterInfo.filter;
                firstBroadphase=static_cast<int>(r.rootCollidable->broadPhaseHandle.type);
                const auto* base=ref?ref->GetBaseObject():nullptr;
                if(base){firstBaseReference=base->GetFormID();firstFormType=static_cast<int>(base->GetFormType());}
                if(firstBroadphase==static_cast<int>(RE::hkpWorldObject::BroadPhaseType::kEntity)) {
                    if(const auto* entity=r.rootCollidable->GetOwner<RE::hkpEntity>()) {
                        firstMotionType=static_cast<int>(*entity->motion.type);
                        firstResponseType=static_cast<int>(*entity->material.responseType);

                        if(base&&entity->motion.type==RE::hkpMotion::MotionType::kFixed)
                            if(const auto* object=base->As<RE::TESObjectSTAT>())
                                if(const auto* model=object->GetModel())firstStaticModel=model;
                    }
                }
            }
            return fc::Hit{point,normal,stable};
        }
    }
};

void reportIgnoredTrigger(const GameWorld& world) {
    if(!diagnostics||world.ignoredTriggerHits<=0)return;
    SKSE::log::info("Ignored non-solid trigger: hits={} reference={:08X} base={:08X} layer={} formType={} broadphase={} motion={} response={} primitive={} filter={:08X}; point=({:.2f},{:.2f},{:.2f}) distance={:.2f}",
        world.ignoredTriggerHits,world.firstTriggerReference,world.firstTriggerBaseReference,world.firstTriggerLayer,
        world.firstTriggerFormType,world.firstTriggerBroadphase,world.firstTriggerMotionType,world.firstTriggerResponseType,
        world.firstTriggerPrimitive,world.firstTriggerFilterInfo,world.firstTriggerHitPoint.x,world.firstTriggerHitPoint.y,
        world.firstTriggerHitPoint.z,world.firstTriggerDistance);
}

void setMotion(RE::PlayerCharacter*,fc::Motion m) {

    if(animationState) animationState->value=100;
    if(m!=lastMotion) {
        ++motionUses[std::clamp(int(m),0,fc::motionCount)];
        ++totalMotionUses[std::clamp(int(m),0,fc::motionCount)];
        if(diagnostics)SKSE::log::info("Motion {} -> {}",int(lastMotion),int(m));
    }
    lastMotion=m;
}

void release(RE::PlayerCharacter* p,const char* reason,bool fade=false,bool physicalFall=false,bool completedTop=false) {
    exitNativeDiagnostics.reset();
    const auto releaseStarted=diagnostics?std::chrono::steady_clock::now():std::chrono::steady_clock::time_point{};
    wallRunEntryGate.reset();
    if(preparationChangedView)cancelEntryPreparation();
    if(fade)traversalAudio.resetTiming();else traversalAudio.stop();
    entryPreparationGrace.cancel();preparationStarted=0;poseHealth.reset();
    viewHeading.reset();
    poses.release(fade,physicalFall,completedTop);
    const auto poseReleased=diagnostics?std::chrono::steady_clock::now():std::chrono::steady_clock::time_point{};
    const bool wasOwned=ownedController.get()!=nullptr;
    jumpGrab.cancel();climbEntry.blockUntilRelease();grabProbeCooldown=0;
    const auto gravityRelease=controllerGravity.release(p?p->GetCharController():nullptr);
    if(gravityRelease.replacement)SKSE::log::info("Restored inherited gravity on an unobserved replacement controller during release: {}",reason);
    ownedController.reset();
    if(p && syncOwned) p->SetGraphVariableBool("bIsSynced",false);
    if(tdm && yawOwned) tdm->ReleaseYawControl(SKSE::GetPluginHandle());
    if(tdm && directionOwned) tdm->ReleaseDisableDirectionalMovement(SKSE::GetPluginHandle());
    syncOwned=yawOwned=directionOwned=false;
    if(runningSaved) {
        if(auto controls=RE::PlayerControls::GetSingleton()) controls->data.running=savedRunning;
        runningSaved=false;
    }
    traversal.stop(); gamepadOwned=false; climbingCell=nullptr; climbingWorldspace=nullptr;
    if(animationState) animationState->value=0;
    if(p&&wasOwned) {

        if(!physicalFall)p->NotifyAnimationGraph("IdleForceDefaultState");
        const auto controlsReleased=diagnostics?std::chrono::steady_clock::now():std::chrono::steady_clock::time_point{};
        SKSE::log::info("Released: {}",reason);
        if(diagnostics)for(int i=1;i<=fc::motionCount;++i)if(motionUses[i])
            SKSE::log::info("Session action {} entries={} selectedSeconds={:.3f}",i,motionUses[i],selectedSeconds[i]);
        if(diagnostics)SKSE::log::info("Session measured routes: edgeHop={} corner={} eave={} automaticClimbActions={} automaticAttempts={} wallRunObstacleJumps={}",
            contextHops,contextCorners,contextEaves,lastAutomaticAction-automaticActionBase,lastAutomaticAttempt-automaticAttemptBase,lastObstacleJump-obstacleJumpBase);
        if(diagnostics) {
            SKSE::log::info("Session captured variety: opportunityScans={} surfaceActions={} idleSettles={}",
                traversal.automaticOpportunityCount()-opportunityBase,traversal.surfaceActionCount()-surfaceActionBase,traversal.contextIdleCount()-idleActionBase);
            SKSE::log::info("Session wall input seconds: up={:.3f} side={:.3f} diagonalUp={:.3f} down={:.3f} run={:.3f} idle={:.3f}; request exposure, not accepted travel",
                inputSeconds[0],inputSeconds[1],inputSeconds[2],inputSeconds[3],inputSeconds[4],inputSeconds[5]);
            for(unsigned motion=39;motion<=fc::motionCount;++motion)SKSE::log::info("Session rendered new action: motion={} starts={} observedSpanSeconds={:.3f}",motion,renderedUses[motion],observedSpanSeconds[motion]);
            SKSE::log::info("Release timing: poseMs={:.3f} controlsMs={:.3f} loggingMs={:.3f}; fade={} physicalFall={} completedTop={}",
                std::chrono::duration<float,std::milli>(poseReleased-releaseStarted).count(),
                std::chrono::duration<float,std::milli>(controlsReleased-poseReleased).count(),
                std::chrono::duration<float,std::milli>(std::chrono::steady_clock::now()-controlsReleased).count(),fade,physicalFall,completedTop);
        }
    }
    lastMotion=fc::Motion::none;
    if(wasOwned&&fade&&completedTop&&!physicalFall)exitNativeDiagnostics.begin(p);
}

const char* yawResultName(TDM_API::APIResult result) {
    switch(result) {
    case TDM_API::APIResult::OK:return "OK";
    case TDM_API::APIResult::NotOwner:return "NotOwner";
    case TDM_API::APIResult::MustKeep:return "MustKeep";
    case TDM_API::APIResult::AlreadyGiven:return "AlreadyGiven";
    case TDM_API::APIResult::AlreadyTaken:return "AlreadyTaken";
    case TDM_API::APIResult::BadThread:return "BadThread";
    }
    return "unknown";
}

bool faceWall(RE::PlayerCharacter* p,float dt) {
    const auto desired=fc::facingWallYaw(traversal.normal);
    if(!desired) {SKSE::log::error("Invalid wall-facing normal; restoring native control");return false;}
    const float before=p->GetAngleZ();
    if(!std::isfinite(before)){SKSE::log::error("Invalid actor yaw; restoring native control");return false;}
    if(!viewHeading.ready())viewHeading.begin(before);
    const float referenceYaw=viewHeading.advance(*desired,dt);
    auto result=TDM_API::APIResult::OK;
    if(tdm&&yawOwned) {
        result=tdm->SetPlayerYaw(SKSE::GetPluginHandle(),referenceYaw);
        if(result!=TDM_API::APIResult::OK) {
            SKSE::log::error("TDM yaw update rejected: {}; target={:.3f} actor={:.3f}; thread={} TDMThread={}",
                yawResultName(result),*desired,before,GetCurrentThreadId(),tdm->GetTDMThreadId());

            return false;
        }
    }

    bool cameraPreserved=false;
    if(std::abs(fc::yawDifference(referenceYaw,before))>.001f) {
        auto* camera=RE::PlayerCamera::GetSingleton();
        const auto cameraState=camera?camera->currentState:RE::BSTSmartPointer<RE::TESCameraState>{};
        auto* third=cameraState&&cameraState->id==RE::CameraState::kThirdPerson?
            static_cast<RE::ThirdPersonState*>(cameraState.get()):nullptr;
        const auto view=third&&third->freeRotationEnabled&&third->applyOffsets&&camera->cameraTarget.get().get()==p?
            fc::CameraHeading::capture(before,third->freeRotation.x):std::nullopt;
        p->SetHeading(referenceYaw);
        if(view&&camera->currentState.get()==cameraState.get()&&camera->cameraTarget.get().get()==p&&
            third->freeRotationEnabled&&third->applyOffsets) {
            if(const auto relative=view->relativeTo(p->GetAngleZ())) {
                third->freeRotation.x=*relative;cameraPreserved=true;
            }
        }
    }
    if(diagnostics&&attachedTime>=yawReportTime) {
        yawReportTime=attachedTime+1;
        const auto pose=poses.lastOrientation();
        std::optional<float> actorRootYaw;
        if(const auto* root=p->Get3D(false)) {
            const auto& m=root->world.rotate.entry;
            actorRootYaw=fc::headingYaw({m[0][1],m[1][1],m[2][1]});
        }
        SKSE::log::info("Wall facing: target={:.3f} viewReference={:.3f} viewRate={:.3f} actorBefore={:.3f} actorAfter={:.3f} actor3D={:.3f}/{}; renderedTarget={:.3f} parent={:.3f}/{} frame={:.3f}/{} ageMs={}; TDM={} cameraPreserved={}",
            *desired,referenceYaw,viewHeading.speed(),before,p->GetAngleZ(),actorRootYaw.value_or(0),actorRootYaw.has_value(),pose.targetYaw,
            pose.parentYaw,pose.parentValid,pose.renderedYaw,pose.renderedValid,
            pose.sampledAt?GetTickCount64()-pose.sampledAt:0,tdm?yawResultName(result):"absent",cameraPreserved);
    }
    return true;
}

void reportOutput(RE::PlayerCharacter* p,fc::Motion motion,float coreMs,float publishMs,int casts,std::uint64_t rayCandidates,std::uint64_t rayClassifications,bool finishing) {
    if(!diagnostics)return;
    peakTraversalMs=std::max(peakTraversalMs,coreMs);peakPublishMs=std::max(peakPublishMs,publishMs);
    peakFrameCasts=std::max(peakFrameCasts,casts);
    peakFrameRayCandidates=std::max(peakFrameRayCandidates,rayCandidates);
    peakFrameRayClassifications=std::max(peakFrameRayClassifications,rayClassifications);
    const auto output=poses.lastOutputAudit();
    const bool observedOutputValid=output.sampledAt&&output.worldMismatches==0&&output.layerWeight>.95f&&GetTickCount64()-output.sampledAt<250;
    if(!observedOutputValid)observedSpanContinuous=false;
    if(output.sampledAt>lastRenderedSample&&observedOutputValid) {

        if(observedSpanContinuous&&lastRenderedSample&&output.motion==lastRenderedMotion&&output.sampledAt-lastRenderedSample<=100&&
            int(output.motion)>0&&int(output.motion)<=fc::motionCount)
            observedSpanSeconds[int(output.motion)]+=float(output.sampledAt-lastRenderedSample)*.001f;
        lastRenderedSample=output.sampledAt;
        observedSpanContinuous=true;
        if(output.motion!=lastRenderedMotion) {
            lastRenderedMotion=output.motion;
            if(int(output.motion)>0&&int(output.motion)<=fc::motionCount)++totalObservedUses[int(output.motion)];
            if(int(output.motion)>=39&&int(output.motion)<=fc::motionCount) {
                ++renderedUses[int(output.motion)];
                SKSE::log::info("New action output observed: motion={} starts={} sampleAgeMs={}",int(output.motion),renderedUses[int(output.motion)],GetTickCount64()-output.sampledAt);
            }
        }
    }
    if(!finishing&&attachedTime<outputReportTime)return;
    outputReportTime=attachedTime+1;
    SKSE::log::info("Pose output audit: publishedMotion={} renderedMotion={} layerWeight={:.4f} nativeRecovery={:.4f} synced={} ageMs={}; applied={} retired={} overwriteFrames={} lastOverwriteBone={} anglePeak={:.4f} distancePeak={:.3f}; worldMismatchFrames={} worldMismatches={} worldAngle={:.4f} worldDistance={:.3f} pass={} transformPasses={} selectedFlagRepairs={}; upperRollDeg={:.2f}/{:.2f}; coreMs={:.3f}/{:.3f} publishMs={:.3f}/{:.3f} callbackMs={:.3f} propagationMs={:.3f} casts={}/{} rayCandidates={}/{} rayClassifications={}/{}",
        int(motion),int(output.motion),output.layerWeight,output.nativeRecovery,graph(p,"bIsSynced"),
        output.sampledAt?GetTickCount64()-output.sampledAt:0,poses.applied.load(),poses.retiredOutputs.load(),
        output.overwriteFrames,output.lastOverwritten,output.peakOverwrittenAngle,output.peakOverwrittenDistance,
        output.worldMismatchFrames,output.worldMismatches,output.worldAngle,output.worldDistance,unsigned(output.pass),output.transformPasses,output.selectedFlagRepairs,
        output.upperRollDegrees[0],output.upperRollDegrees[1],
        coreMs,peakTraversalMs,publishMs,peakPublishMs,output.callbackMs,output.propagationMs,casts,peakFrameCasts,rayCandidates,peakFrameRayCandidates,rayClassifications,peakFrameRayClassifications);
    SKSE::log::info("Surface placement audit: motion={} support={} samples={} gap={:.2f}/{:.2f}/{:.2f} palmPlaneGap={:.2f}/{:.2f} contacts={} reachError={:.2f} pos=({:.2f},{:.2f},{:.2f}) normal=({:.3f},{:.3f},{:.3f})",
        int(motion),poses.surface.surfaceGapValid,poses.surface.surfaceSamples,traversal.cfg.gap,
        poses.surface.measuredSurfaceGap,poses.surface.appliedSurfaceGap,poses.surface.surfacePalmGaps[0],poses.surface.surfacePalmGaps[1],
        poses.surface.contactCount,poses.surface.maxReachError,traversal.position.x,traversal.position.y,traversal.position.z,
        traversal.surfaceNormal.x,traversal.surfaceNormal.y,traversal.surfaceNormal.z);
    peakTraversalMs=peakPublishMs=0;peakFrameCasts=0;peakFrameRayCandidates=peakFrameRayClassifications=0;
    SKSE::log::info("Final display audit (sampled): lateWorldUpdates={} lateRootUpdates={} skinCalls={} ownedSkinSamples={} bodyInputs={} ownedInputs={} cachedSamples={} bodyMismatchSamples={}",
        poses.lateWorldUpdates.load(),poses.lateRootUpdates.load(),poses.skinCalls.load(),poses.ownedSkinCalls.load(),
        poses.skinBodyInputs.load(),poses.skinOwnedInputs.load(),poses.skinCacheHits.load(),poses.skinMismatchCalls.load());
    SKSE::log::info("Skin audit budget: sampled={} skipped={} lockSkips={} inputs={} ancestors={} incomplete={} bodyDiffs={} bridgeDiffs={} extraDiffs={} totalUs={} peakSampleUs={}; max 4 samples/frame, 64 inputs+64 ancestors/sample; excludes original draw time",
        poses.skinAuditSamples.load(),poses.skinAuditSkipped.load(),poses.skinAuditLockSkips.load(),poses.skinAuditInputs.load(),
        poses.skinAuditAncestors.load(),poses.skinAuditBudgetStops.load(),poses.skinBodyMismatches.load(),
        poses.skinBridgeMismatches.load(),poses.skinExtraMismatches.load(),poses.skinAuditMicros.load(),poses.skinAuditPeakMicros.load());
}

bool allowed(RE::PlayerCharacter* p) {
    if(!p||!p->Is3DLoaded()||!p->GetCharController()||!p->GetParentCell()||p->IsDead()||p->IsInKillMove()||p->IsOnMount()) return false;
    auto state=p->AsActorState();
    return !state->IsSwimming()&&!state->IsWeaponDrawn()&&
        state->GetKnockState()==RE::KNOCK_STATE_ENUM::kNormal&&state->GetSitSleepState()==RE::SIT_SLEEP_STATE::kNormal;
}

bool observeNativeContacts(RE::PlayerCharacter* p,const char* reason="near-stop",bool contacts=true) {
    const RE::NiPointer<RE::bhkCharacterController> controller(p?p->GetCharController():nullptr);
    auto* cell=p?p->GetParentCell():nullptr;auto* world=cell?cell->GetbhkWorld():nullptr;
    if(!controller||!world)return false;
    auto* proxyController=skyrim_cast<RE::bhkCharProxyController*>(controller.get());
    if(!proxyController)return false;
    const float scale=RE::bhkWorld::GetWorldScale();
    if(!std::isfinite(scale)||scale<=0)return false;
    RE::BSReadLockGuard lock(world->worldLock);
    const auto* proxy=proxyController->GetCharacterProxy();
    if(!proxy||!proxy->shapePhantom||proxy->shapePhantom->world!=world->referencedObject.get())return false;
    RE::hkVector4 feet{},centre{},velocity{};
    controller->GetPosition(feet,true);controller->GetPosition(centre,false);controller->GetLinearVelocityImpl(velocity);
    const auto point=[&](const RE::hkVector4& v){return fc::Vec{v.quad.m128_f32[0],v.quad.m128_f32[1],v.quad.m128_f32[2]}/scale;};
    const auto actualFeet=point(feet),actualCentre=point(centre),actualVelocity=point(velocity),delta=actualFeet-vec(p->GetPosition());
    const auto bounds=controller->collisionBound.extents,bumper=controller->bumperCollisionBound.extents;
    const auto* ownBody=proxy->shapePhantom->GetCollidable();
    const bool shapeRecorded=fc::NativeShapeWitness::LogLocked(ownBody->shape,1.f/scale,reason,
        attachmentsSinceLoad,&proxy->shapePhantom->motionState.transform);
    SKSE::log::info("Native physics witness: feetDelta=({:.2f},{:.2f},{:.2f}) centre=({:.2f},{:.2f},{:.2f}) resolvedVelocity=({:.2f},{:.2f},{:.2f}) bounds=({:.2f},{:.2f},{:.2f}) bumper=({:.2f},{:.2f},{:.2f}) flags={:08X} filter={:08X} contacts={} slopeCos={:.3f} keepDistance={:.2f}; read-only",
        delta.x,delta.y,delta.z,actualCentre.x,actualCentre.y,actualCentre.z,actualVelocity.x,actualVelocity.y,actualVelocity.z,
        bounds.x,bounds.y,bounds.z,bumper.x,bumper.y,bumper.z,controller->flags.underlying(),ownBody->broadPhaseHandle.collisionFilterInfo.filter,
        proxy->manifold.size(),proxy->maxSlopeCosine,proxy->keepDistance/scale);

    if(!contacts||proxy->manifold.size()>256)return shapeRecorded;
    for(std::uint32_t index=0;index<std::min<std::uint32_t>(proxy->manifold.size(),12);++index) {
        const auto& contact=proxy->manifold[index];
        const auto* a=contact.rootCollidableA;const auto* b=contact.rootCollidableB;
        const auto* refA=a?RE::TESHavokUtilities::FindCollidableRef(*a):nullptr;
        const auto* refB=b?RE::TESHavokUtilities::FindCollidableRef(*b):nullptr;
        const bool ownA=a==ownBody||refA==p,ownB=b==ownBody||refB==p;
        const auto* other=ownA?b:ownB?a:nullptr;
        const auto* ref=ownA?refB:ownB?refA:nullptr;
        const auto* base=ref?ref->GetBaseObject():nullptr;
        const auto position=point(contact.contact.position);
        const auto& n=contact.contact.separatingNormal.quad;
        const char* model="";
        if(base)if(const auto* object=base->As<RE::TESObjectSTAT>())if(const auto* path=object->GetModel())model=path;
        SKSE::log::info("Native contact: index={} playerSide={} exactPhantom={}/{} samePlayer={}/{} reference={:08X} base={:08X} layer={} formType={} point=({:.2f},{:.2f},{:.2f}) separatingNormal=({:.3f},{:.3f},{:.3f}) distance={:.2f} shapeA={} shapeB={} model='{}'",
            index,ownA?"A":ownB?"B":"unknown",a==ownBody,b==ownBody,refA==p,refB==p,ref?ref->GetFormID():0,base?base->GetFormID():0,
            other?int(other->GetCollisionLayer()):-1,base?int(base->GetFormType()):-1,
            position.x,position.y,position.z,n.m128_f32[0],n.m128_f32[1],n.m128_f32[2],n.m128_f32[3]/scale,
            contact.shapeKeyA,contact.shapeKeyB,model);
    }
    return shapeRecorded;
}

void observeNativeShapeBaseline(RE::PlayerCharacter* p,float dt) {
    if(!diagnostics||traversal.active()||attachmentsSinceLoad||nativeShapeBaselineSamples>=3||nativeShapeBaselineAttempts>=16)return;
    nativeShapeBaselineAge+=std::min(dt,.05f);
    constexpr std::array times{0.f,.5f,2.f};
    if(nativeShapeBaselineAge<times[nativeShapeBaselineSamples]||nativeShapeBaselineAge<nativeShapeBaselineRetry)return;
    nativeShapeBaselineRetry=nativeShapeBaselineAge+.25f;
    ++nativeShapeBaselineAttempts;

    if(observeNativeContacts(p,"pre-climb-baseline",false))++nativeShapeBaselineSamples;
    else if(nativeShapeBaselineAttempts==16)SKSE::log::info("Native shape baseline unavailable after bounded retries; samples={}",nativeShapeBaselineSamples);
}

void observeNativeMovement(RE::PlayerCharacter* p,const fc::Keys& held,float dt) {
    if(!p||!p->Is3DLoaded()||!p->GetCharController()||!p->GetParentCell()||p->IsDead()||p->IsOnMount()) {
        groundMotionProbe.suspend();return;
    }
    const auto* controls=RE::ControlMap::GetSingleton();
    const float yaw=p->GetAngleZ();
    const fc::Vec forward{std::sin(yaw),std::cos(yaw),0},right{forward.y,-forward.x,0};
    const auto intent=forward*float(int(held.w)-int(held.s))+right*float(int(held.d)-int(held.a));
    if(!groundMotionProbe.sample(vec(p->GetPosition()),intent,dt,
        diagnostics&&!traversal.active()&&controls&&controls->IsMovementControlsEnabled()))return;
    const auto* controller=p->GetCharController();
    const auto& velocity=controller->outVelocity.quad;
    const float worldScale=RE::bhkWorld::GetWorldScale();
    const auto position=vec(p->GetPosition());
    SKSE::log::info("Native movement near-stop: pos=({:.2f},{:.2f},{:.2f}) gravity={:.3f} current={} wanted={} supported={} synced={} animationDriven={} SkyParkour={}/{} controllerOwned={} Shift={} W={} A={} S={} D={}; velocity=({:.2f},{:.2f},{:.2f}); diagnostic only",
        position.x,position.y,position.z,controller->gravity,int(controller->context.currentState),int(controller->wantState),
        int(controller->surfaceInfo.supportedState.get()),graph(p,"bIsSynced"),p->IsAnimationDriven(),
        graph(p,"SkyParkourOngoing"),graph(p,"SkyParkourSliding"),ownedController.get()!=nullptr,
        held.shift,held.w,held.a,held.s,held.d,velocity.m128_f32[0]/worldScale,velocity.m128_f32[1]/worldScale,velocity.m128_f32[2]/worldScale);
    SKSE::log::info("Native entry history: attachmentsSinceLoad={} pending={} waitingForKeyRelease={} preparing={} changedView={}",
        attachmentsSinceLoad,jumpGrab.pending(),climbEntry.waitingForRelease(),preparationStarted!=0,preparationChangedView);
    observeNativeContacts(p);

    if(!std::isfinite(controller->collisionBound.extents.x))return;
    const float radius=std::clamp(controller->collisionBound.extents.x,22.f,45.f);
    for(float height:{12.f,70.f,125.f}) {
        GameWorld witness(p);const auto from=position+fc::Vec{0,0,height};
        const auto hit=witness.ray(from,from+intent.unit()*(radius+30));
        SKSE::log::info("Native near-stop ray: height={:.0f} hit={} distance={:.2f} reference={:08X} base={:08X} layer={} formType={} response={} model='{}' normal=({:.3f},{:.3f},{:.3f})",
            height,hit.has_value(),witness.firstDistance,witness.firstReference,witness.firstBaseReference,witness.firstLayer,
            witness.firstFormType,witness.firstResponseType,witness.firstStaticModel,
            witness.firstNormal.x,witness.firstNormal.y,witness.firstNormal.z);
    }
}

fc::GrabFlight entryFlight(RE::PlayerCharacter* p,bool nativeJump) {
    const auto* controller=p?p->GetCharController():nullptr;
    if(!controller)return {};
    const auto current=controller->context.currentState,wanted=controller->wantState;
    const bool inAir=current==RE::hkpCharacterStateType::kInAir||wanted==RE::hkpCharacterStateType::kInAir;
    const bool jumping=current==RE::hkpCharacterStateType::kJumping||wanted==RE::hkpCharacterStateType::kJumping;
    const float speed=controller->outVelocity.quad.m128_f32[2]/RE::bhkWorld::GetWorldScale();
    return fc::grabFlight(inAir,jumping,graph(p,"bInJumpState"),nativeJump,speed);
}

bool acquire(RE::PlayerCharacter* p) {
    const auto acquireStarted=diagnostics?std::chrono::steady_clock::now():std::chrono::steady_clock::time_point{};
    if(graph(p,"bIsSynced")||graph(p,"SkyParkourOngoing")||graph(p,"SkyParkourSliding")||p->IsAnimationDriven()) return false;
    if(tdm) {
        if(tdm->GetTargetLockState()) return false;
        const auto yaw=tdm->RequestYawControl(SKSE::GetPluginHandle(),0);
        yawOwned=yaw==TDM_API::APIResult::OK||yaw==TDM_API::APIResult::AlreadyGiven;
        if(!yawOwned) return false;
        const auto direction=tdm->RequestDisableDirectionalMovement(SKSE::GetPluginHandle());
        directionOwned=direction==TDM_API::APIResult::OK||direction==TDM_API::APIResult::AlreadyGiven;
        if(!directionOwned) { release(p,"TDM busy"); return false; }
    }
    if(!p->SetGraphVariableBool("bIsSynced",true)) { release(p,"unsupported animation graph"); return false; }
    syncOwned=true;
    const auto attachStarted=diagnostics?std::chrono::steady_clock::now():std::chrono::steady_clock::time_point{};
    if(!poses.attach(p)) {
        SKSE::log::error("Cannot attach pose layer: {}",poses.bindingFailure);
        release(p,"pose binding failed");note("FreeClimb: pose binding failed; see log");return false;
    }
    const auto poseAttached=diagnostics?std::chrono::steady_clock::now():std::chrono::steady_clock::time_point{};
    takeController(p->GetCharController(),false);
    climbingCell=p->GetParentCell();
    climbingWorldspace=p->GetWorldspace();
    if(auto controls=RE::PlayerControls::GetSingleton()) { savedRunning=idleRunningKnown?idleRunning:controls->data.running; runningSaved=true; }
    attachedTime=0;poseHealth.reset();geometryCaptureGate.reset();preparationStarted=0;stallReportTime=0;poseRefreshTime=0;yawReportTime=0;lowStaminaNoted=false;motionUses.fill(0);
    outputReportTime=peakTraversalMs=peakPublishMs=0;peakFrameCasts=0;peakFrameRayCandidates=peakFrameRayClassifications=0;
    lastContextStatus=lastThreepeatStatus=0;lastCornerReported=lastEdgeReported=false;contextReportTime=0;
    contextHops=contextCorners=contextEaves=0;
    automaticActionBase=lastAutomaticAction=traversal.automaticActionCount();
    automaticAttemptBase=lastAutomaticAttempt=traversal.automaticAttemptCount();
    opportunityBase=traversal.automaticOpportunityCount();surfaceActionBase=traversal.surfaceActionCount();idleActionBase=traversal.contextIdleCount();
    renderedUses={};selectedSeconds={};observedSpanSeconds={};inputSeconds={};
    observedSpanContinuous=false;
    lastRenderedMotion=fc::Motion::none;lastRenderedSample=0;
    obstacleJumpBase=lastObstacleJump=traversal.obstacleJumpCount();
    const auto controlsAcquired=diagnostics?std::chrono::steady_clock::now():std::chrono::steady_clock::time_point{};
    SKSE::log::info("Attached at ({:.1f},{:.1f},{:.1f}); normal=({:.2f},{:.2f},{:.2f}); actorScale={:.2f}; controllerBounds=({:.2f},{:.2f},{:.2f}); actorHeight={:.2f}",
        traversal.position.x,traversal.position.y,traversal.position.z,traversal.normal.x,traversal.normal.y,traversal.normal.z,
        p->GetScale(),ownedController->collisionBound.extents.x,ownedController->collisionBound.extents.y,
        ownedController->collisionBound.extents.z,ownedController->actorHeight);
    const auto& b=activeSettings.bindings;
    const auto& g=activeSettings.gamepad.bindings;
    const auto help=gamepadOwned?"FreeClimb: "+fc::serializeGamepadChord(g.runModifier)+" wall run | "+fc::serializeGamepadChord(g.hop)+" climb hop | "+
        fc::serializeGamepadChord(g.drop)+" let go":"FreeClimb: "+fc::serializeKeyChord(b.runModifier)+" wall run | "+fc::serializeKeyChord(b.hop)+" climb hop | "+
        fc::serializeKeyChord(b.backward)+"+"+fc::serializeKeyChord(b.hop)+" leave wall";
    note(help.c_str());
    SKSE::log::info("Traversal input: {}",gamepadOwned?"controller":"keyboard");
    ++attachmentsSinceLoad;
    if(diagnostics)SKSE::log::info("Attach timing: ownershipMs={:.3f} poseMs={:.3f} controlsMs={:.3f} loggingMs={:.3f}",
        std::chrono::duration<float,std::milli>(attachStarted-acquireStarted).count(),
        std::chrono::duration<float,std::milli>(poseAttached-attachStarted).count(),
        std::chrono::duration<float,std::milli>(controlsAcquired-poseAttached).count(),
        std::chrono::duration<float,std::milli>(std::chrono::steady_clock::now()-controlsAcquired).count());
    return true;
}

void update(RE::PlayerCharacter* p,float dt) {
    const auto prior=originalUpdate.get();
    if(!prior)return;
    if(!ready){prior(p,dt);return;}
    if(gamepadOwned) {
        auto* manager=RE::BSInputDeviceManager::GetSingleton();
        if(gamepadLost||!gamepadAvailable||!manager||!manager->IsGamepadEnabled()) {
            release(p,"controller disconnected",false,true);
            gamepadState.reset();gamepadOwnership.reset();gamepadAvailable=false;gamepadPreferred=false;
        }
    }
    gamepadLost=false;
    entryLookDiagnostics.beforeNative(p);
    if(traversal.active()) stopLocomotion();
    prior(p,dt);
    if(poses.consumeRigInvalidated()) {
        if(traversal.active())release(p,"character rig changed");
        else {cancelEntryPreparation();cancelGrabRequest();}
    }
    if(std::isfinite(dt)&&dt>1e-6f)poses.tick(dt);
    serviceSettings();
    if(!ready||!enabled||!animationState) {exitNativeDiagnostics.reset();return;}

    if(grabInputSuspended()) {
        exitNativeDiagnostics.reset();
        groundMotionProbe.suspend();
        inputState.reset();gamepadState.blockUntilButtonsReleased();wallRunEntryGate.reset();traversalAudio.stop();

        if(traversal.active())release(p,"input suspended");
        cancelGrabRequest();
        return;
    }

    if(!std::isfinite(dt)||dt<=1e-6f){exitNativeDiagnostics.reset();entryPreparationGrace.cancel();return;}
    traversalAudio.observe(dt);
    observeNativeShapeBaseline(p,dt);
    const bool fromGamepad=gamepadSelected();
    const auto heldKeys=keys();
    entryLookDiagnostics.sample(p,heldKeys,dt);
    if(crestAuditTime>=0) {
        if(traversal.active())crestAuditTime=-1;
        else if(const auto* controller=p->GetCharController()) {
            crestAuditTime+=dt;
            if(crestAuditTime>.12f)crestAuditSupported|=controller->surfaceInfo.supportedState==RE::hkpSurfaceInfo::SupportedState::kSupported;
            if(crestAuditTime>=.25f+.5f*crestAuditPhase) {
                const auto delta=vec(p->GetPosition())-crestAuditStart;
                SKSE::log::info("Roof crest native follow-up: t={:.2f} supportedSeen={} supportedState={} current={} wanted={} displacement=({:.2f},{:.2f},{:.2f}) velocityZ={:.2f} W={} A={} S={} D={}",
                    crestAuditTime,crestAuditSupported,int(controller->surfaceInfo.supportedState.get()),int(controller->context.currentState),int(controller->wantState),
                    delta.x,delta.y,delta.z,controller->outVelocity.quad.m128_f32[2]/RE::bhkWorld::GetWorldScale(),heldKeys.w,heldKeys.a,heldKeys.s,heldKeys.d);
                if(++crestAuditPhase==3)crestAuditTime=-1;
            }
        } else crestAuditTime=-1;
    }
    jumpGrab.tick(dt);
    const bool nativeJump=nativeJumpIntent.sample(nativeJumpHeld(),dt);
    const auto initialFlight=entryFlight(p,nativeJump);
    const auto grabIntent=climbEntry.sample(heldKeys,traversal.active(),false,dt,initialFlight.confirmedAirborne);
    if(grabIntent.began)gamepadPreferred=fromGamepad;
    const bool preparedRetry=entryPreparationGrace.sample(dt,fromGamepad,heldKeys);
    const bool grab=grabIntent.requested||preparedRetry,grabPressed=grabIntent.fresh;
    if(diagnostics&&!traversal.active()&&!grab&&activeSettings.gamepad.enabled&&gamepadAvailable&&
        gamepadState.heldChord(activeSettings.gamepad.bindings.entry)) {
        static ULONGLONG nextReport{};
        const auto now=GetTickCount64();
        if(now>=nextReport) {
            nextReport=now+2000;
            SKSE::log::info("Controller entry waiting: selected={} releaseButtons={} releaseChord={} backward={} drop={} preparing={} probeCooldown={:.3f}",
                fromGamepad,gamepadState.waitingForButtonsRelease(),climbEntry.waitingForRelease(),
                heldKeys.s,heldKeys.letGo,preparationStarted!=0,grabProbeCooldown);
        }
    }
    exitNativeDiagnostics.sample(p,dt,grab);
    const bool hop=heldKeys.space,hopPressed=hop&&!lastHop;
    lastHop=hop;
    if(!traversal.active()&&!grab){jumpGrab.cancel();cancelEntryPreparation();grabProbeCooldown=0;}
    if(!traversal.active()) if(auto controls=RE::PlayerControls::GetSingleton()) {
        idleRunning=controls->data.running;idleRunningKnown=true;
    }
    diagnosticCooldown=std::max(0.0f,diagnosticCooldown-std::clamp(dt,0.0f,0.05f));
    observeNativeMovement(p,heldKeys,dt);
    if(!allowed(p)) { cancelGrabRequest(); if(traversal.active())release(p,"actor state"); return; }
    if(!traversal.active()) {
        const auto& v=p->GetCharController()->outVelocity.quad;
        const float yaw=p->GetAngleZ();
        approachSpeed=(v.m128_f32[0]*std::sin(yaw)+v.m128_f32[1]*std::cos(yaw))/RE::bhkWorld::GetWorldScale();
    }
    traversal.tickCooldown(dt);
    if(traversal.active()) {
        const auto actual=vec(p->GetPosition());
        const float error=(actual-traversal.position).length();
        if(p->GetParentCell()!=climbingCell&&error<=40&&climbingWorldspace&&p->GetWorldspace()==climbingWorldspace&&
            p->GetParentCell()->GetbhkWorld()) {
            climbingCell=p->GetParentCell();
            SKSE::log::info("Continued climbing across an exterior cell boundary");
        }
        if(p->GetParentCell()!=climbingCell||error>150) {
            SKSE::log::info("Climb interrupted after {:.2f}s: controllerChanged={}, cellChanged={}, positionError={:.1f}; actual=({:.1f},{:.1f},{:.1f}); expected=({:.1f},{:.1f},{:.1f})",
                attachedTime,p->GetCharController()!=ownedController.get(),p->GetParentCell()!=climbingCell,error,
                actual.x,actual.y,actual.z,traversal.position.x,traversal.position.y,traversal.position.z);
            release(p,"controller, cell or teleport change"); return;
        }
        if(p->GetCharController()!=ownedController.get()) {

            if(error>40) { release(p,"replacement controller outside climb position"); return; }
            if(diagnostics) {
                const auto* replacement=p->GetCharController();
                SKSE::log::info("Replacement after {:.3f}s: positionError={:.2f}, nativeJump={}, currentState={}, wantState={}, velocityZ={:.2f}",
                    attachedTime,error,graph(p,"bInJumpState"),int(replacement->context.currentState),int(replacement->wantState),
                    replacement->outVelocity.quad.m128_f32[2]/RE::bhkWorld::GetWorldScale());
            }
            takeController(p->GetCharController(),true);
        }
        attachedTime+=std::clamp(dt,0.0f,0.05f);
    }
    GameWorld world(p);
    bool attachedThisFrame=false;
    const float stamina=p->AsActorValueOwner()->GetActorValue(RE::ActorValue::kStamina);
    if(!traversal.active()) {
        const auto controls=RE::ControlMap::GetSingleton();
        if(!jumpToAttach||!controls||!controls->IsMovementControlsEnabled()||p->IsSneaking()||
            graph(p,"bIsSynced")||graph(p,"SkyParkourOngoing")||graph(p,"SkyParkourSliding")||p->IsAnimationDriven()||
            (tdm&&tdm->GetTargetLockState())) {
            cancelGrabRequest();return;
        }

        if(!grab){jumpGrab.cancel();cancelEntryPreparation();return;}
        const float actorYaw=p->GetAngleZ();
        jumpGrab.hold({std::sin(actorYaw),std::cos(actorYaw),0},nativeJump,grabIntent.airborneAtBegin,grabPressed);
        if(grabPressed&&diagnostics)SKSE::log::info("Grab request: device={} entry chord={}; airborne={} descending={} velocityZ={:.2f}; releaseCooldown={:.3f}; nativeJumpPending={} airborneAtBegin={}",
            fromGamepad?"gamepad":"keyboard",fromGamepad?fc::serializeGamepadChord(activeSettings.gamepad.bindings.entry):fc::serializeKeyChord(activeSettings.bindings.entry),initialFlight.airborne,initialFlight.descending,
            initialFlight.verticalSpeed,traversal.cooldown,nativeJump,grabIntent.airborneAtBegin);
        const auto flight=entryFlight(p,jumpGrab.startedNativeJump());
        const bool explicitAirCatch=jumpGrab.explicitAirCatch(flight);
        if(!jumpGrab.permitted(traversal.cooldown,flight))return;
        if(!fc::entryProbeReady(grabProbeCooldown,dt,grabPressed))return;
        EntryLookDiagnostics::Probe probeTimer(entryLookDiagnostics);
        fitBody(p);

        const auto facing=jumpGrab.facing();
        const float yaw=std::atan2(facing.x,facing.y);
        auto candidate=traversal;
        if(!candidate.attach(world,vec(p->GetPosition()),facing,stamina,grabMaxSnap,explicitAirCatch,!flight.airborne)) {
            if(diagnostics&&diagnosticCooldown<=0&&world.hits>0) {
                SKSE::log::info("Climb entry rejected: {}; airborne={} descending={} snap={:.2f}/{}; pos=({:.1f},{:.1f},{:.1f}); yaw={:.2f}; casts={} hits={} firstRayDistance={:.1f}; normal=({:.2f},{:.2f},{:.2f})",
                    fc::name(candidate.lastFailure),flight.airborne,flight.descending,candidate.lastAttachDistance,grabMaxSnap,
                    p->GetPositionX(),p->GetPositionY(),p->GetPositionZ(),yaw,world.casts,world.hits,world.firstDistance,
                    world.firstNormal.x,world.firstNormal.y,world.firstNormal.z);
                reportIgnoredTrigger(world);diagnosticCooldown=2;
                SKSE::log::info("Grab collision: reference={:08X} base={:08X} formType={} broadphase={} motion={} response={} filter={:08X}; point=({:.2f},{:.2f},{:.2f}); fixedStaticModel='{}'",
                    world.firstReference,world.firstBaseReference,world.firstFormType,world.firstBroadphase,world.firstMotionType,world.firstResponseType,
                    world.firstFilterInfo,world.firstHitPoint.x,world.firstHitPoint.y,world.firstHitPoint.z,world.firstStaticModel);
            }
            if(preparationStarted||preparationChangedView)cancelEntryPreparation();
            return;
        }
        float nativeSlope=.707107f;
        if(auto* proxyController=skyrim_cast<RE::bhkCharProxyController*>(p->GetCharController()))
            if(const auto* proxy=proxyController->GetCharacterProxy();proxy&&std::isfinite(proxy->maxSlopeCosine)&&proxy->maxSlopeCosine>=0&&proxy->maxSlopeCosine<=1)
                nativeSlope=std::max(nativeSlope,proxy->maxSlopeCosine);
        const auto* entryController=p->GetCharController();
        const bool grounded=fc::groundEntryGeometryAllowed(
            entryController->context.currentState==RE::hkpCharacterStateType::kOnGround,
            entryController->wantState==RE::hkpCharacterStateType::kJumping,
            jumpGrab.startedNativeJump(),flight.airborne,flight.confirmedAirborne);
        const auto lowTop=fc::groundedLowTopFallback(traversal.cfg,grounded,candidate.entryFallbackTopRise());
        const auto nativePath=lowTop.excludes()?fc::NativeWalkableResult{}:
            fc::nativeWalkableApproach(world,vec(p->GetPosition()),facing,traversal.cfg,grounded,16.f,nativeSlope);
        auto lowEntry=lowTop.excludes()?lowTop:nativePath.walkable?fc::GroundEntryExclusionResult{}:
            fc::groundedLowFace(world,vec(p->GetPosition()),traversal.cfg,grounded,
                candidate.entryTarget(),candidate.surfaceNormal,nativeSlope);
        auto mergeEntryProbe=[&](const fc::GroundEntryExclusionResult& probe) {
            lowEntry.casts+=probe.casts;
            if(probe.groundFound) {
                lowEntry.groundFound=true;lowEntry.baseHeight=probe.baseHeight;lowEntry.supportSamples=probe.supportSamples;
            }
            if(probe.excludes()){lowEntry.reason=probe.reason;lowEntry.selectedRise=probe.selectedRise;}
        };
        if(!nativePath.walkable&&!lowEntry.excludes())
            mergeEntryProbe(fc::groundedStepFace(world,vec(p->GetPosition()),facing,traversal.cfg,grounded,
                candidate.entryTarget(),candidate.surfaceNormal,nativeSlope));
        if(!nativePath.walkable&&!lowEntry.excludes())
            mergeEntryProbe(fc::groundedEntryExclusion(world,vec(p->GetPosition()),facing,traversal.cfg,grounded,
                candidate.entryTarget(),candidate.surfaceNormal,nativeSlope));
        if(nativePath.walkable||lowEntry.excludes()) {
            if(preparationStarted||preparationChangedView)cancelEntryPreparation();
            if(diagnostics&&diagnosticCooldown<=0) {
                const auto delta=candidate.entryTarget()-vec(p->GetPosition());
                SKSE::log::info("Climb entry kept native: reason={}; nativeWalkable={} nativeProbes={} lowProbes={}; selectedTopFallback={} selectedTopRise={:.2f}; targetDelta=({:.2f},{:.2f},{:.2f}); pos=({:.2f},{:.2f},{:.2f})",
                    nativePath.walkable?"continuous walkable floor and body corridor":fc::name(lowEntry.reason),
                    nativePath.walkable,nativePath.casts,lowEntry.casts,candidate.entryFallbackTopRise().has_value(),
                    candidate.entryFallbackTopRise().value_or(0.f),delta.x,delta.y,delta.z,p->GetPositionX(),p->GetPositionY(),p->GetPositionZ());
                diagnosticCooldown=2;
            }

            return;
        }
        auto* camera=RE::PlayerCamera::GetSingleton();
        if(camera->IsInFirstPerson()){preparationChangedView=true;camera->ForceThirdPerson();}
        const auto prepareStarted=diagnostics?std::chrono::steady_clock::now():std::chrono::steady_clock::time_point{};
        const auto preparation=poses.prepare(p);
        if(diagnostics&&(preparation!=fc::PoseRuntime::Preparation::waiting||!preparationStarted))
            SKSE::log::info("Pose preflight timing: state={} bindingMs={:.3f} changedView={}",int(preparation),
                std::chrono::duration<float,std::milli>(std::chrono::steady_clock::now()-prepareStarted).count(),preparationChangedView);
        if(preparation!=fc::PoseRuntime::Preparation::ready) {
            if(preparation==fc::PoseRuntime::Preparation::waiting&&grabIntent.requested)entryPreparationGrace.arm(fromGamepad);
            const auto now=GetTickCount64();
            if(!preparationStarted)preparationStarted=now;
            if(preparation==fc::PoseRuntime::Preparation::rejected||now-preparationStarted>=1000) {
                SKSE::log::error("Pose preflight failed without taking player control: reason={}, callbacks={}",
                    preparation==fc::PoseRuntime::Preparation::rejected?poses.bindingFailure:"no scene callback",
                    poses.callbacks.load());
                cancelGrabRequest();
                note("FreeClimb: animation unavailable; normal movement retained");
            }
            return;
        }
        entryPreparationGrace.cancel();preparationStarted=0;
        candidate.cfg.threepeatAnimations=activeSettings.threepeatAnimations;
        poses.library.configureThreepeat(candidate.cfg);
        traversal=std::move(candidate);gamepadOwned=fromGamepad;
        const bool nativeSpace=jumpGrab.startedNativeJump();
        reportIgnoredTrigger(world);
        const auto entry=fc::grabEntryMotion(flight);
        if(!acquire(p)) {
            traversal.stop();gamepadOwned=false;
            cancelEntryPreparation();
            if(diagnosticCooldown<=0) {
                SKSE::log::info("Attach blocked by animation/movement ownership; synced={}, parkour={}, sliding={}, animationDriven={}, targetLock={}",
                    graph(p,"bIsSynced"),graph(p,"SkyParkourOngoing"),graph(p,"SkyParkourSliding"),p->IsAnimationDriven(),tdm&&tdm->GetTargetLockState());
                diagnosticCooldown=2;
            }
            if(grabPressed) note("FreeClimb: another animation or movement mod is busy");
            return;
        }
        preparationChangedView=false;
        if(diagnostics&&preparedRetry&&!grabIntent.requested)SKSE::log::info("Completed validated entry preparation after chord release within 150ms");
        wallRunEntryGate.begin(heldKeys);
        traversal.entry(entry,!flight.airborne);jumpGrab.cancel();climbEntry.blockUntilRelease();attachedThisFrame=true;
        if(diagnostics)SKSE::log::info("Entry={} preEntryForwardSpeed={:.1f} airborne={} descending={} nativeSpace={} velocityZ={:.2f} entryLift={} explicitAirCatch={} snap={:.2f} raisedTarget={:.1f} roundedCapsule={} duration={:.3f}; groundSupported={} supportedState={} nativeProbes={} lowProbes={}; target=({:.2f},{:.2f},{:.2f}) targetDelta=({:.2f},{:.2f},{:.2f})",
            int(entry),approachSpeed,flight.airborne,flight.descending,nativeSpace,flight.verticalSpeed,!flight.airborne,
            explicitAirCatch,traversal.lastAttachDistance,traversal.entryLiftHeight(),traversal.roundedEntryPath(),traversal.entryDuration(),
            grounded,int(p->GetCharController()->surfaceInfo.supportedState.get()),nativePath.casts,lowEntry.casts,
            traversal.entryTarget().x,traversal.entryTarget().y,traversal.entryTarget().z,
            traversal.entryTarget().x-p->GetPositionX(),traversal.entryTarget().y-p->GetPositionY(),traversal.entryTarget().z-p->GetPositionZ());
        if(diagnostics)SKSE::log::info("Entry ground evidence: found={} samples={} rootAboveFloor={:.2f} selectedRise={:.2f}",
            lowEntry.groundFound,lowEntry.supportSamples,lowEntry.groundFound?p->GetPositionZ()-lowEntry.baseHeight:0.f,lowEntry.selectedRise);
        if(diagnostics)SKSE::log::info("Entry collision witness (first query hit): reference={:08X} base={:08X} layer={} point=({:.2f},{:.2f},{:.2f}) normal=({:.3f},{:.3f},{:.3f}) model='{}'",
            world.firstReference,world.firstBaseReference,world.firstLayer,world.firstHitPoint.x,world.firstHitPoint.y,world.firstHitPoint.z,
            world.firstNormal.x,world.firstNormal.y,world.firstNormal.z,world.firstStaticModel);
    }
    fc::Input input=fc::wallInput(wallRunEntryGate.filter(heldKeys),hopPressed,autoMantle,attachedThisFrame,traversal.wallRunning());
    const float contactSpeed=poses.surface.movementScale();
    input.x*=contactSpeed;input.y*=contactSpeed;

    poseHealth.sample(poses.applied.load(),dt);
    if(poseHealth.failed()) {
        SKSE::log::error("Pose output timeout: callbacks={}, matched={}, applied={}, rejected={}; restoring player controls",
            poses.callbacks.load(),poses.matched.load(),poses.applied.load(),poses.rejected.load());
        release(p,"animation output timeout");
        note("FreeClimb: animation stopped; normal movement restored");return;
    }
    const bool animationReady=poseHealth.ready();
    if(input.release&&!animationReady&&attachedTime>.12f) {release(p,"manual drop while animation output unavailable",false,true);return;}
    const auto stateBefore=traversal.state;
    const bool runningBefore=traversal.wallRunning();
    const float effectiveDt=animationReady?dt:0.f;
    const bool captureFrame=diagnostics&&animationReady&&geometryCaptureGate.arm(traversal,input,attachedTime);
    const auto traversalStarted=std::chrono::steady_clock::now();
    fc::Result result;
    if(captureFrame) {
        geometryCapture.begin(traversal,input,effectiveDt,stamina);
        fc::TraversalCapture::RecordingWorld recorded(world,geometryCapture);
        result=traversal.update(recorded,input,effectiveDt,stamina);
        geometryCapture.finish(traversal,result);
    } else result=traversal.update(world,input,effectiveDt,stamina);
    const float traversalMs=std::chrono::duration<float,std::milli>(std::chrono::steady_clock::now()-traversalStarted).count();
    if(diagnostics) {
        const float seconds=std::clamp(effectiveDt,0.f,.05f);
        if(int(result.motion)>0&&int(result.motion)<=fc::motionCount)selectedSeconds[int(result.motion)]+=seconds;
        if(stateBefore==fc::State::wall||stateBefore==fc::State::ledge) {
            const bool sideways=std::abs(input.x)>.1f,moving=sideways||std::abs(input.y)>.1f;
            const unsigned mode=!moving?5u:input.run&&input.y>=0?4u:input.y<-.1f?3u:
                sideways?(input.y>.1f?2u:1u):0u;
            inputSeconds[mode]+=seconds;
        }
    }
    if(captureFrame) {

        try {SKSE::log::info("{}",geometryCapture.serialize());}
        catch(const std::exception& e){SKSE::log::warn("Geometry capture unavailable: {}",e.what());}
        catch(...){SKSE::log::warn("Geometry capture unavailable");}
    }
    if(diagnostics) {
        if(traversal.state==fc::State::mantle&&stateBefore!=fc::State::mantle) {
            const auto from=traversal.topStart(),target=traversal.topTarget(),lip=traversal.topLip();
            SKSE::log::info("Mantle selected: motion={} preciseContacts={} sourceBegin={:.3f} reason={}; height={:.2f} forward={:.2f} surfaceZ={:.3f} wasRunning={} preparation={:.3f}",
                int(result.motion),traversal.preciseTopContacts(),traversal.topSampleBegin(),traversal.topSelectionReason(),
                lip.z-from.z,(target-from).dot(traversal.normal*-1),traversal.surfaceNormal.z,runningBefore,traversal.topPreparation());
        }
        if(traversal.automaticAttemptCount()!=lastAutomaticAttempt) {
            lastAutomaticAttempt=traversal.automaticAttemptCount();
            SKSE::log::info("Automatic climbing attempt: count={} motion={} preparing={} edge={} threepeatReason={} contextReason={} time={:.2f} input=({:.2f},{:.2f}) surfaceZ={:.3f} running={} state={}",
                lastAutomaticAttempt-automaticAttemptBase,int(result.motion),traversal.preparingEdge(),traversal.usesEdgeTargets(result.motion),
                traversal.threepeatReason(),traversal.contextReason(),attachedTime,input.x,input.y,traversal.surfaceNormal.z,
                traversal.wallRunning(),int(traversal.state));
        }
        if(traversal.obstacleJumpCount()!=lastObstacleJump) {
            lastObstacleJump=traversal.obstacleJumpCount();
            SKSE::log::info("Wall-run obstacle jump committed: count={} motion={} duration={:.3f} time={:.2f}",
                lastObstacleJump-obstacleJumpBase,int(result.motion),traversal.actionDuration(),attachedTime);
        }
        if(traversal.automaticActionCount()!=lastAutomaticAction) {
            lastAutomaticAction=traversal.automaticActionCount();
            SKSE::log::info("Automatic climbing action committed: count={} motion={} edge={} time={:.2f} contactMode={}",
                lastAutomaticAction-automaticActionBase,int(result.motion),traversal.usesEdgeTargets(result.motion),attachedTime,
                traversal.usesWallTargets(result.motion)?"wall-patch":traversal.usesEdgeTargets(result.motion)?"ledge":"ordinary");
        }
        const unsigned contextStatus=traversal.contextStatus(),threepeatStatus=traversal.threepeatStatus();
        const bool corner=traversal.turningCorner(),edge=traversal.usesEdgeTargets(result.motion);
        const bool committed=edge&&traversal.state==fc::State::action&&stateBefore!=fc::State::action;
        const bool eave=std::string_view(traversal.blockedReason)=="checked eave bypass";
        if(corner&&!lastCornerReported)++contextCorners;
        if(eave)++contextEaves;
        if(committed) {
            ++contextHops;
            const auto a=traversal.edgeStart(),b=traversal.edgeTarget();
            const auto source=(traversal.edgeHand(0,false)+traversal.edgeHand(1,false))*.5f;
            const auto destination=(traversal.edgeHand(0,true)+traversal.edgeHand(1,true))*.5f;
            const auto delta=destination-source;
            SKSE::log::info("Measured contact committed: motion={} mode={} contactDelta=({:.2f},{:.2f},{:.2f}) bodyFrom=({:.2f},{:.2f},{:.2f}) bodyTo=({:.2f},{:.2f},{:.2f})",
                int(result.motion),traversal.usesWallTargets(result.motion)?"wall-patch":"ledge",delta.x,delta.y,delta.z,a.x,a.y,a.z,b.x,b.y,b.z);
        }

        if((contextStatus!=lastContextStatus&&(contextStatus>=7||attachedTime>=contextReportTime))||
            (threepeatStatus!=lastThreepeatStatus&&(threepeatStatus>=7||attachedTime>=contextReportTime))||
            corner!=lastCornerReported||edge!=lastEdgeReported||committed||eave) {
            SKSE::log::info("Context traversal: time={:.2f} status={} reason={} threepeatStatus={} threepeatReason={} preparing={} edge={} corner={} motion={} state={} pos=({:.2f},{:.2f},{:.2f}) route={}",
                attachedTime,contextStatus,traversal.contextReason(),threepeatStatus,traversal.threepeatReason(),traversal.preparingEdge(),edge,corner,
                int(result.motion),int(traversal.state),traversal.position.x,traversal.position.y,traversal.position.z,traversal.blockedReason);
            lastContextStatus=contextStatus;lastThreepeatStatus=threepeatStatus;lastCornerReported=corner;lastEdgeReported=edge;
            contextReportTime=attachedTime+.5f;
        }
    }
    if(diagnostics&&traversal.stalledSeconds()>.35f&&attachedTime>=stallReportTime) {
        stallReportTime=attachedTime+1;
        SKSE::log::info("Blocked movement: time={:.2f} dt={:.5f} input=({:.1f},{:.1f}) pos=({:.2f},{:.2f},{:.2f}) surface=({:.3f},{:.3f},{:.3f}) state={} casts={} hits={} rayCandidates={} rayClassifications={} reason={} top={}",
            traversal.stalledSeconds(),dt,input.x,input.y,traversal.position.x,traversal.position.y,traversal.position.z,
            traversal.surfaceNormal.x,traversal.surfaceNormal.y,traversal.surfaceNormal.z,int(traversal.state),world.casts,world.hits,world.rayCandidates,world.rayClassifications,traversal.blockedReason,traversal.ledgeReason);
        if(traversal.blockedHit) {
            const auto& hit=*traversal.blockedHit;const auto a=traversal.blockedFrom,b=traversal.blockedTo;
            SKSE::log::info("Clearance obstruction: from=({:.2f},{:.2f},{:.2f}) to=({:.2f},{:.2f},{:.2f}) hit=({:.2f},{:.2f},{:.2f}) normal=({:.3f},{:.3f},{:.3f}) climbable={}",
                a.x,a.y,a.z,b.x,b.y,b.z,hit.point.x,hit.point.y,hit.point.z,hit.normal.x,hit.normal.y,hit.normal.z,hit.climbable);
        }
        SKSE::log::info("Collision identity: firstReference={:08X} firstLayer={} self={} exactPlayerPhantom={}",
            world.firstReference,world.firstLayer,world.selfHits,world.controllerSelfHits);
        reportIgnoredTrigger(world);
    }

    if(ownedController) {
        stopLocomotion();
        ownedController->fallTime=0;
        ownedController->fallStartHeight=traversal.position.z;
        if((vec(p->GetPosition())-traversal.position).length()>.025f)p->SetPosition(ni(traversal.position),true);
        if(!faceWall(p,dt)) {release(p,"wall-facing ownership or normal invalid",true);return;}
    }
    if(result.staminaCost>0) p->AsActorValueOwner()->ModActorValue(RE::ACTOR_VALUE_MODIFIER::kDamage,RE::ActorValue::kStamina,-result.staminaCost);
    if(traversal.cfg.staminaEnabled&&lowStaminaNotifications&&stamina<=20&&!lowStaminaNoted){note("FreeClimb: low stamina - stop to rest or climb down");lowStaminaNoted=true;}
    if(stamina>30)lowStaminaNoted=false;
    const auto publishStarted=std::chrono::steady_clock::now();
    poses.update(world,traversal,result.motion,dt,p->GetScale());
    traversalAudio.update(poses.library,traversal,result,poses.surface.sampledPhase(),effectiveDt,animationReady);
    const float publishMs=std::chrono::duration<float,std::milli>(std::chrono::steady_clock::now()-publishStarted).count();
    reportOutput(p,result.motion,traversalMs,publishMs,world.casts,world.rayCandidates,world.rayClassifications,result.released);
    if(poseHealth.stale()>.25f&&attachedTime>=poseRefreshTime) {
        poseRefreshTime=attachedTime+.5f;
        if(poses.refresh(p)) {
            SKSE::log::info("Refreshed replaced animation graph without restarting traversal");

            poseHealth.invalidate();
        }
    }
    if(result.released) {
        if(diagnostics&&result.completed&&std::string_view(result.reason)=="roof crest reached") {
            crestAuditTime=0;crestAuditPhase=0;crestAuditSupported=false;crestAuditStart=traversal.position;
        }
        SKSE::log::info("Traversal ended after {:.2f}s; stamina={:.1f}; motion={}; state={}->{}; reason={}; casts={}; hits={}; layer={}; firstHitNormal=({:.2f},{:.2f},{:.2f}); facingNormal=({:.2f},{:.2f},{:.2f}); surfaceNormal=({:.2f},{:.2f},{:.2f})",
            attachedTime,stamina,static_cast<int>(result.motion),int(stateBefore),int(traversal.state),result.reason,
            world.casts,world.hits,world.firstLayer,world.firstNormal.x,world.firstNormal.y,world.firstNormal.z,
            traversal.normal.x,traversal.normal.y,traversal.normal.z,traversal.surfaceNormal.x,traversal.surfaceNormal.y,traversal.surfaceNormal.z);
        if(diagnostics)SKSE::log::info("Pose frames={}, rejected={}, contactCount={}, reachError={:.2f}, completed={}",poses.applied.load(),poses.rejected.load(),poses.surface.contactCount,poses.surface.maxReachError,result.completed);
        const bool physicalExit=result.motion==fc::Motion::drop||result.motion==fc::Motion::dropBack||result.motion==fc::Motion::backFlipOut||
            (!result.completed&&result.releaseVelocity.length()>.01f)||
            (stateBefore==fc::State::action&&(lastMotion==fc::Motion::dropBack||lastMotion==fc::Motion::backFlipOut));
        release(p,result.completed?(std::string_view(result.reason)!="none"?result.reason:"top-out complete"):
            result.motion==fc::Motion::drop?"manual drop":result.reason,true,physicalExit,result.completed);

        if(result.releaseVelocity.finite()&&result.releaseVelocity.length()>.01f) {
            if(auto* controller=p->GetCharController()) {
                const auto v=result.releaseVelocity*RE::bhkWorld::GetWorldScale();
                const RE::hkVector4 impulse(v.x,v.y,v.z,0),zero(0,0,0,0);
                controller->outVelocity=controller->initialVelocity=impulse;
                controller->velocityMod=zero;
                controller->wantState=RE::hkpCharacterStateType::kInAir;
                controller->SetLinearVelocityImpl(impulse);
                if(diagnostics)SKSE::log::info("Physical back jump velocity=({:.1f},{:.1f},{:.1f}); gravity={:.3f}",
                    result.releaseVelocity.x,result.releaseVelocity.y,result.releaseVelocity.z,controller->gravity);
            }
        }
        return;
    }
    setMotion(p,result.motion);
}

template<class Handler,int Index> struct InputGuard {
    static inline fc::HookPublication<bool(*)(Handler*,RE::InputEvent*)> original;
    static bool canProcess(Handler* self,RE::InputEvent* event) {
        const auto prior=original.get();
        if(!prior)return false;
        if(!ready)return prior(self,event);
        if(traversal.active()) {

            if(event) if(auto button=event->AsButtonEvent();button&&button->IsUp()) return prior(self,event);
            return false;
        }
        return prior(self,event);
    }
    static bool install(REL::VariantID table) {
        REL::Relocation<std::uintptr_t> vtable{table};
        return fc::runtime::installVfunc(vtable.address(),fc::runtime::inputFilterSlot,canProcess,original);
    }
};

struct AttachedSpaceGuard {
    static inline fc::HookPublication<void(*)(RE::BSWin32KeyboardDevice*,float)> original;
    static void process(RE::BSWin32KeyboardDevice* keyboard,float dt) {
        const auto prior=original.get();
        if(!prior)return;
        prior(keyboard,dt);
        if(!ready)return;
        fc::settingsMenuKeyboardSample(keyboard->GetRuntimeData().curState);
        const bool suspended=grabInputSuspended();
        if(suspended) {
            cancelGrabRequest();inputState.reset();wallRunEntryGate.reset();
        }
        auto* queue=RE::BSInputEventQueue::GetSingleton();
        if(!queue)return;
        entryLookDiagnostics.queue(queue->GetQueueHead(),0);
        std::array<std::uint32_t,4> nativeMovementKeys{0xffffffffu,0xffffffffu,0xffffffffu,0xffffffffu};
        if(const auto* controls=RE::ControlMap::GetSingleton();!suspended&&controls&&controls->IsMovementControlsEnabled()) {
            const std::array<std::string_view,4> events{"Forward","Back","Strafe Left","Strafe Right"};
            for(std::size_t i=0;i<events.size();++i)nativeMovementKeys[i]=controls->GetMappedKey(events[i],RE::INPUT_DEVICE::kKeyboard);
        }
        fc::removeInputEvents(queue->GetQueueHead(),queue->GetQueueTail(),[suspended,nativeMovementKeys](RE::InputEvent* event) {
            if(fc::settingsMenuFilterInput(event))return true;
            auto* button=event->AsButtonEvent();
            if(!button||event->GetDevice()!=RE::INPUT_DEVICE::kKeyboard)return false;
            const auto scan=button->GetIDCode();
            const auto before=inputState;
            if(!suspended)inputState.set(scan,button->IsPressed());
            if(button->IsDown()&&!suspended&&!traversal.active())gamepadPreferred=false;
            if(traversal.active()&&!gamepadOwned)wallRunEntryGate.filter(fc::mapKeys(inputState,activeSettings.bindings));
            return inputOwnership.filter(scan,button->IsDown(),button->IsUp(),
                !suspended&&traversal.active()&&!gamepadOwned,before,inputState,activeSettings.bindings,
                !suspended&&std::find(nativeMovementKeys.begin(),nativeMovementKeys.end(),scan)!=nativeMovementKeys.end());
        });
        entryLookDiagnostics.queue(queue->GetQueueHead(),1);
    }
    static bool install() {
        REL::Relocation<std::uintptr_t> table{RE::VTABLE_BSWin32KeyboardDevice[0]};
        return fc::runtime::installVfunc(table.address(),fc::runtime::keyboardProcessSlot,process,original);
    }
};

struct GamepadGuard {
    static inline fc::HookPublication<void(*)(RE::BSPCGamepadDeviceHandler*,float)> original;
    static RE::BSWin32GamepadDevice* device(RE::BSPCGamepadDeviceHandler* handler) {
        auto* delegate=handler?handler->GetRuntimeData().currentPCGamePadDelegate:nullptr;
        return delegate&&delegate->IsEnabled()?skyrim_cast<RE::BSWin32GamepadDevice*>(delegate):nullptr;
    }
    static void sample(const RE::BSWin32GamepadDevice& pad) {
        const auto& raw=pad.GetRuntimeData().currentState.gamepad;
        gamepadState.sampleXInput(raw.buttons,raw.leftTrigger,raw.rightTrigger,raw.thumbLX,raw.thumbLY,activeSettings.gamepad);
    }
    static void process(RE::BSPCGamepadDeviceHandler* handler,float dt) {
        const auto prior=original.get();
        if(!prior)return;
        prior(handler,dt);
        if(!ready)return;
        auto* pad=device(handler);
        const bool available=pad!=nullptr;
        if(available!=gamepadAvailable) {
            SKSE::log::info("XInput controller available={}",available);
            gamepadLost|=gamepadOwned&&!available;
            gamepadState.reset();gamepadOwnership.reset();gamepadPreferred=false;
            gamepadAvailable=available;
        }
        if(!pad){fc::settingsMenuGamepadSample(false);return;}
        const auto& raw=pad->GetRuntimeData().currentState.gamepad;
        fc::settingsMenuGamepadSample(true,raw.buttons,raw.leftTrigger,raw.rightTrigger);
        const bool suspended=grabInputSuspended()||!enabled||!activeSettings.gamepad.enabled;
        if(suspended)gamepadState.blockUntilButtonsReleased();
        const auto before=gamepadState;
        sample(*pad);
        auto* queue=RE::BSInputEventQueue::GetSingleton();
        if(!queue)return;
        fc::removeInputEvents(queue->GetQueueHead(),queue->GetQueueTail(),[suspended,&before](RE::InputEvent* event) {
            if(fc::settingsMenuFilterInput(event))return true;
            if(event->GetDevice()!=RE::INPUT_DEVICE::kGamepad)return false;
            if(auto* button=event->AsButtonEvent()) {
                const auto index=gamepadButtonIndex(button->GetIDCode());
                if(index>=16)return false;
                if(!suspended&&!traversal.active()&&button->IsDown())gamepadPreferred=true;
                return gamepadOwnership.filter(index,button->IsDown(),button->IsUp(),
                    !suspended&&traversal.active()&&gamepadOwned,before,gamepadState,activeSettings.gamepad.bindings);
            }
            if(const auto* stick=event->AsThumbstickEvent();stick&&stick->IsLeft()&&!suspended&&!traversal.active()&&
                std::hypot(stick->xValue,stick->yValue)>activeSettings.gamepad.deadzone)gamepadPreferred=true;
            return false;
        });
        if(!suspended)gamepadState.resumeIfButtonsReleased();
        if(traversal.active()&&gamepadOwned)wallRunEntryGate.filter(gamepadState.keys(activeSettings.gamepad.bindings));
    }
    static void install() {
        REL::Relocation<std::uintptr_t> table{RE::VTABLE_BSPCGamepadDeviceHandler[0]};
        if(!fc::runtime::installVfunc(table.address(),fc::runtime::gamepadPollSlot,process,original)) {
            SKSE::log::error("Controller input hook unavailable; keyboard controls retained");return;
        }
        gamepadHookReady=true;
    }
};

struct MenuListener:RE::BSTEventSink<RE::MenuOpenCloseEvent> {
    std::uint64_t nextOverlayLog{};
    RE::BSEventNotifyControl ProcessEvent(const RE::MenuOpenCloseEvent* e,RE::BSTEventSource<RE::MenuOpenCloseEvent>*) override {
        if(!e||!e->opening)return RE::BSEventNotifyControl::kContinue;
        fc::MenuInputState state;
        std::uint32_t flags{},context{};
        bool available=false;
        if(auto* ui=RE::UI::GetSingleton()) {
            RE::BSSpinLockGuard lock(ui->processMessagesLock);
            if(const auto menu=ui->GetMenu(e->menuName)) {
                state=menuInputState(*menu);flags=menu->menuFlags.underlying();context=menu->inputContext.underlying();available=true;
            }
        }
        const bool blocking=fc::menuInterruptsTraversal(e->menuName.c_str(),e->opening,state);
        const bool relevant=traversal.active()||preparationStarted||jumpGrab.pending();
        const auto now=GetTickCount64();
        if(relevant&&(blocking||(diagnostics&&now>=nextOverlayLog))) {
            SKSE::log::info("Climb menu event: name='{}' available={} flags=0x{:X} context={} blocking={} attached={}",
                e->menuName.c_str(),available,flags,context,blocking,traversal.active());
            if(!blocking)nextOverlayLog=now+250;
        }
        if(blocking) {
            exitNativeDiagnostics.reset();
            traversalAudio.stop();
            inputState.reset();gamepadState.blockUntilButtonsReleased();wallRunEntryGate.reset();
            if(traversal.active())release(RE::PlayerCharacter::GetSingleton(),"menu opened");
            cancelGrabRequest();
        }
        return RE::BSEventNotifyControl::kContinue;
    }
} menuListener;

bool runtimeHooksReady() {
    if(!fc::runtime::supported())return false;
    const auto check=[](REL::VariantID id,std::size_t slot) {
        REL::Relocation<std::uintptr_t> table{id};
        if(fc::runtime::hookSite(table.address(),slot))return true;
        SKSE::log::error("Runtime hook target unavailable: table={:X}, slot={:X}; hooks not installed",table.address(),slot);
        return false;
    };
    if(!check(RE::VTABLE_PlayerCharacter[0],fc::runtime::actorUpdateSlot)||
        !check(RE::VTABLE_BSWin32KeyboardDevice[0],fc::runtime::keyboardProcessSlot))return false;
    for(const auto id:{RE::VTABLE_MovementHandler[0],RE::VTABLE_JumpHandler[0],RE::VTABLE_SneakHandler[0],
        RE::VTABLE_SprintHandler[0],RE::VTABLE_ReadyWeaponHandler[0],RE::VTABLE_AttackBlockHandler[0],
        RE::VTABLE_ActivateHandler[0],RE::VTABLE_TogglePOVHandler[0],RE::VTABLE_ShoutHandler[0],
        RE::VTABLE_AutoMoveHandler[0],RE::VTABLE_RunHandler[0],RE::VTABLE_ToggleRunHandler[0]})
        if(!check(id,fc::runtime::inputFilterSlot))return false;
    return true;
}

void applyRuntimeSettings(const fc::UserSettings& value) {
    const auto oldGamepad=activeSettings.gamepad;
    activeSettings=fc::sanitizeUserSettings(value);
    if(activeSettings.gamepad!=oldGamepad)gamepadState.blockUntilButtonsReleased();
    const auto& u=activeSettings;
    enabled=u.enabled;notifications=u.notifications;lowStaminaNotifications=u.lowStaminaNotifications;
    jumpToAttach=u.jumpToAttach;autoMantle=u.autoMantle;diagnostics=u.diagnostics;
    grabMaxSnap=u.grabMaxSnap;
    poses.traceOutput=diagnostics;lowStaminaNoted=false;
    if(!diagnostics)crestAuditTime=-1;
    auto& c=traversal.cfg;
    c.contextActions=u.contextActions;c.threepeatAnimations=u.threepeatAnimations;
    c.automaticClimbActions=u.automaticClimbActions;c.legacyAutomaticHops=false;
    c.surfaceActionVariants=u.surfaceActionVariants;c.wallRunObstacleJumps=u.wallRunObstacleJumps;
    c.contextualMantleEnabled=u.contextualMantleEnabled;c.automaticSideWeights=u.automaticSideWeights;
    c.climbSpeed=u.upSpeed;c.downSpeed=u.downSpeed;c.sideSpeed=u.sideSpeed;
    wallRunSpeedOverride=u.wallRunSpeed;c.runSpeed=u.wallRunSpeed>0?u.wallRunSpeed:379.5f;
    c.diagonalRunMultiplier=u.diagonalRunMultiplier;
    c.autoActionMinSeconds=u.autoActionMinSeconds;c.autoActionMaxSeconds=u.autoActionMaxSeconds;
    c.fancyJumps=u.fancyJumps;c.hopOut=u.hopOut;c.kickOut=u.kickOut;
    c.reach=u.reach;c.groundJumpHeight=u.groundJumpHeight;c.maxNormalZ=u.maxNormalZ;
    c.staminaEnabled=u.staminaEnabled;c.drain=u.movingPerSecond;c.hangDrain=u.hangingPerSecond;c.startStamina=u.requiredToGrab;
    traversalAudio.enabled=u.audioEnabled;traversalAudio.volume=u.audioVolume;
    if(!u.audioEnabled)traversalAudio.stop();
    cancelGrabRequest();wallRunEntryGate.reset();
    lastHop=keys().space;
    if(ready)poses.library.configureThreepeat(c);
}

void refreshMenuSnapshot() {
    fc::SettingsMenuSnapshot snapshot;
    snapshot.ready=ready;snapshot.traversalActive=traversal.active();
    snapshot.diagnosticsEnabled=diagnostics;
    snapshot.audioReady=audioReady;
    snapshot.status=settingsStatus;snapshot.error=settingsError;
    snapshot.packName="meshes/actors/character/animations/FreeClimb/pack.json";
    snapshot.automaticAttempts=traversal.automaticAttemptCount();
    snapshot.automaticActions=traversal.automaticActionCount();snapshot.wallRunObstacleJumps=traversal.obstacleJumpCount();
    for(const auto& slot:poses.packReport.slots) {
        const auto id=static_cast<unsigned>(slot.motion);
        if(id<1||id>fc::motionCount)continue;
        snapshot.slots.push_back({std::string(fc::motionSlotNames[id-1]),slot.file,slot.reason,int(slot.status),
            totalMotionUses[id],totalObservedUses[id],slot.samples,slot.seconds});
    }
    std::scoped_lock lock(settingsMutex);
    snapshot.movementPending=pendingSettings.has_value();snapshot.reloadPending=pendingReload;
    menuSnapshot=std::move(snapshot);
}

void serviceSettings() {
    std::optional<fc::UserSettings> requested;
    std::optional<std::pair<bool,float>> audio;
    bool save=false,reload=false;
    std::uint64_t revision=0,reloadRequest=0;
    {
        std::scoped_lock lock(settingsMutex);
        requested=pendingSettings;audio=pendingAudio;pendingAudio.reset();
        save=pendingSave;pendingSave=false;reload=pendingReload;
        revision=settingsRevision;reloadRequest=reloadRevision;
    }
    if(audio) {
        traversalAudio.enabled=audio->first;traversalAudio.volume=audio->second;
        activeSettings.audioEnabled=audio->first;activeSettings.audioVolume=audio->second;
        if(!audio->first)traversalAudio.stop();
    }
    if(save&&requested) {
        std::string error;
        if(!fc::saveUserSettings("Data/SKSE/Plugins/FreeClimb.ini",*requested,error)) {
            settingsStatus="save_failed";settingsError=std::move(error);
            SKSE::log::error("Settings save failed: {}",settingsError);
        } else {settingsStatus="pending";settingsError.clear();}
    }
    if(requested) {
        notifications=requested->notifications;lowStaminaNotifications=requested->lowStaminaNotifications;
        traversalAudio.enabled=requested->audioEnabled;traversalAudio.volume=requested->audioVolume;
        if(!requested->audioEnabled)traversalAudio.stop();
        if(!requested->enabled&&traversal.active())release(RE::PlayerCharacter::GetSingleton(),"disabled in settings",true);
        if(!traversal.active())cancelEntryPreparation();
        if(!traversal.active()&&poses.canReloadPack()) {
            applyRuntimeSettings(*requested);
            {std::scoped_lock lock(settingsMutex);if(settingsRevision==revision)pendingSettings.reset();}
            if(settingsStatus!="save_failed"){settingsStatus="applied";settingsError.clear();}
        } else if(settingsStatus!="save_failed")settingsStatus="pending";
    }
    if(reload&&dataLoaded&&!traversal.active()) {
        cancelEntryPreparation();
        if(poses.canReloadPack()) {
            {std::scoped_lock lock(settingsMutex);if(reloadRevision==reloadRequest)pendingReload=false;}
            if(!ready)initializeRuntime();
            else if(poses.reloadPack()) {
                poses.library.configureThreepeat(traversal.cfg);cancelGrabRequest();
                settingsStatus="reloaded";settingsError.clear();
            } else {settingsStatus="reload_failed";settingsError=poses.packReport.error;}
        }
    }
    if(requested||audio||reload||GetTickCount64()>=nextMenuSnapshot) {
        refreshMenuSnapshot();nextMenuSnapshot=GetTickCount64()+200;
    }
}

void registerMenu() {
    if(menuRegistered)return;
    fc::SettingsMenuCallbacks callbacks;
    callbacks.getSettings=[] {std::scoped_lock lock(settingsMutex);return desiredSettings;};
    callbacks.snapshot=[] {std::scoped_lock lock(settingsMutex);return menuSnapshot;};
    callbacks.requestSave=[](fc::UserSettings value) {
        {
            std::scoped_lock lock(settingsMutex);
            desiredSettings=fc::sanitizeUserSettings(value);pendingSettings=desiredSettings;pendingSave=true;++settingsRevision;
        }
        SKSE::GetTaskInterface()->AddTask([] {serviceSettings();});
    };
    callbacks.requestAudio=[](bool enabled,float volume) {
        {std::scoped_lock lock(settingsMutex);pendingAudio=std::pair{enabled,std::clamp(volume,0.f,1.f)};}
        SKSE::GetTaskInterface()->AddTask([] {serviceSettings();});
    };
    callbacks.requestReloadAnimations=[] {
        {std::scoped_lock lock(settingsMutex);pendingReload=true;++reloadRevision;}
        SKSE::GetTaskInterface()->AddTask([] {serviceSettings();});
    };
    menuRegistered=fc::registerSettingsMenu(std::move(callbacks));
    SKSE::log::info("Optional settings menu registered={}; language={}",menuRegistered,desiredSettings.language);
}

void loadSettings() {
    const auto loaded=fc::loadUserSettings("Data/SKSE/Plugins/FreeClimb.ini");
    desiredSettings=loaded.settings;applyRuntimeSettings(desiredSettings);
    for(const auto& warning:loaded.warnings)SKSE::log::warn("Settings: {}",warning);
    SKSE::log::info("FreeClimb {}; standalone HKX framework; climb entry={}; wall-run modifier={}; stamina enabled={}",
        SKSE::PluginDeclaration::GetSingleton()->GetVersion().string("."),
        fc::serializeKeyChord(activeSettings.bindings.entry),fc::serializeKeyChord(activeSettings.bindings.runModifier),traversal.cfg.staminaEnabled);
    SKSE::log::info("Experimental controller: enabled={} entry={} run={} hop={} drop={} deadzone={:.2f} trigger={:.2f}",
        activeSettings.gamepad.enabled,fc::serializeGamepadChord(activeSettings.gamepad.bindings.entry),
        fc::serializeGamepadChord(activeSettings.gamepad.bindings.runModifier),fc::serializeGamepadChord(activeSettings.gamepad.bindings.hop),
        fc::serializeGamepadChord(activeSettings.gamepad.bindings.drop),activeSettings.gamepad.deadzone,activeSettings.gamepad.triggerThreshold);
}

void initializeRuntime() {
        static std::mutex initializationMutex;
        std::scoped_lock initializationLock(initializationMutex);
        if(ready)return;
        if(!runtimeHooksReady())return;
        animationState=RE::TESDataHandler::GetSingleton()->LookupForm<RE::TESGlobal>(0x800,"FreeClimb.esp");
        if(!animationState) { SKSE::log::error("FreeClimb.esp missing; climbing disabled"); return; }
        audioReady=traversalAudio.install();
        SKSE::log::info("Independent HKX framework; configurable climb-entry combination, wall-run obstacle jumps, native ground input preserved");
        if(!poses.install()) {settingsStatus="pack_unavailable";settingsError=poses.packReport.error;refreshMenuSnapshot();return;}
        poses.library.configureThreepeat(traversal.cfg);
        SKSE::log::info("Threepeat animations: library={}, enabled={}, surfaceVariants={}",poses.library.hasThreepeat(),traversal.cfg.threepeatAnimations,traversal.cfg.surfaceActionVariants);
        REL::Relocation<std::uintptr_t> vtable{RE::VTABLE_PlayerCharacter[0]};
        if(!fc::runtime::installVfunc(vtable.address(),fc::runtime::actorUpdateSlot,update,originalUpdate)||
            !AttachedSpaceGuard::install()) {
            SKSE::log::error("Player or keyboard hook unavailable; native input retained");return;
        }
        GamepadGuard::install();
        if(!InputGuard<RE::MovementHandler,0>::install(RE::VTABLE_MovementHandler[0])||
            !InputGuard<RE::JumpHandler,1>::install(RE::VTABLE_JumpHandler[0])||
            !InputGuard<RE::SneakHandler,2>::install(RE::VTABLE_SneakHandler[0])||
            !InputGuard<RE::SprintHandler,3>::install(RE::VTABLE_SprintHandler[0])||
            !InputGuard<RE::ReadyWeaponHandler,4>::install(RE::VTABLE_ReadyWeaponHandler[0])||
            !InputGuard<RE::AttackBlockHandler,5>::install(RE::VTABLE_AttackBlockHandler[0])||
            !InputGuard<RE::ActivateHandler,6>::install(RE::VTABLE_ActivateHandler[0])||
            !InputGuard<RE::TogglePOVHandler,7>::install(RE::VTABLE_TogglePOVHandler[0])||
            !InputGuard<RE::ShoutHandler,8>::install(RE::VTABLE_ShoutHandler[0])||
            !InputGuard<RE::AutoMoveHandler,9>::install(RE::VTABLE_AutoMoveHandler[0])||
            !InputGuard<RE::RunHandler,11>::install(RE::VTABLE_RunHandler[0])||
            !InputGuard<RE::ToggleRunHandler,12>::install(RE::VTABLE_ToggleRunHandler[0])) {
            SKSE::log::error("Native input guard hook unavailable; climbing disabled");return;
        }
        RE::UI::GetSingleton()->AddEventSink(&menuListener);
        settingsStatus="ready";settingsError.clear();ready=true;refreshMenuSnapshot();SKSE::log::info("Hooks installed; data ready");
}

void onMessage(SKSE::MessagingInterface::Message* m) {
    switch(m->type) {
    case SKSE::MessagingInterface::kPostLoad:
        registerMenu();
        tdm=static_cast<TDM_API::IVTDM3*>(TDM_API::RequestPluginAPI());
        SKSE::log::info("TDM API available: {}",tdm!=nullptr);
        break;
    case SKSE::MessagingInterface::kDataLoaded: {
        dataLoaded=true;registerMenu();initializeRuntime();break;
    }
    case SKSE::MessagingInterface::kPreLoadGame:
        release(RE::PlayerCharacter::GetSingleton(),"pre-load"); break;
    case SKSE::MessagingInterface::kNewGame:
    case SKSE::MessagingInterface::kPostLoadGame: {
        auto p=RE::PlayerCharacter::GetSingleton();

        if(animationState&&animationState->value!=0&&p) p->SetGraphVariableBool("bIsSynced",false);
        release(p,"new game / loaded"); traversal.reset();
        attachmentsSinceLoad=0;
        nativeShapeBaselineSamples=nativeShapeBaselineAttempts=0;nativeShapeBaselineAge=nativeShapeBaselineRetry=0;
        jumpGrab.cancel();nativeJumpIntent={};inputState.reset();inputOwnership.reset();
        gamepadState.reset();gamepadOwnership.reset();gamepadPreferred=false;gamepadLost=false;
        totalMotionUses.fill(0);totalObservedUses.fill(0);
        climbEntry.blockUntilRelease();lastHop=false;refreshMenuSnapshot();break;
    }
    case SKSE::MessagingInterface::kSaveGame:
        release(RE::PlayerCharacter::GetSingleton(),"save"); break;
    }
}
}

SKSEPluginLoad(const SKSE::LoadInterface* skse) {

    fc::initializeLogging();
    if(!skse){SKSE::log::error("Plugin load rejected: missing SKSE interface");return false;}
    const auto reportedVersion=skse->RuntimeVersion();
    const auto expectedVersion=REL::Version::unpack(fc::runtime::gameVersionFromSKSE(reportedVersion.pack()));
    SKSE::log::info("Plugin load: FreeClimb={} game={} SKSE={} loaderRuntime={}",
        SKSE::PluginDeclaration::GetSingleton()->GetVersion().string("."),expectedVersion.string("."),
        REL::Version::unpack(skse->SKSEVersion()).string("."),reportedVersion.string("."));
    if(!fc::runtime::supportedSKSE(reportedVersion.pack())) {
        SKSE::log::error("Plugin load rejected: SKSE runtime {} is outside the supported loader table",reportedVersion.string("."));
        return false;
    }
    SKSE::log::info("Initializing SKSE; expected Address Library: Data/SKSE/Plugins/{}-{}.bin",
        fc::runtime::addressFormat(expectedVersion.pack())==1?"version":"versionlib",expectedVersion.string("-"));
    SKSE::Init(skse,SKSE::InitInfo{.log=false});
    const auto gameVersion=REL::Module::get().version();
    if(gameVersion!=expectedVersion||!fc::runtime::supported(gameVersion.pack())) {
        SKSE::log::error("Plugin load rejected: executable {} does not match SKSE runtime {}",gameVersion.string("."),reportedVersion.string("."));
        return false;
    }
    SKSE::log::info("Runtime {}: family={}, addressLibraryFormat={}, explicit support table; AE builds have not been tested in-game",
        gameVersion.string("."),static_cast<int>(fc::runtime::family(gameVersion.pack())),fc::runtime::addressFormat(gameVersion.pack()));
    loadSettings();
    auto serialization=SKSE::GetSerializationInterface();
    serialization->SetUniqueID(0x46434C4D);
    serialization->SetSaveCallback([](SKSE::SerializationInterface*) { release(RE::PlayerCharacter::GetSingleton(),"serialize"); });
    serialization->SetRevertCallback([](SKSE::SerializationInterface*) { release(RE::PlayerCharacter::GetSingleton(),"revert"); });
    const bool registered=SKSE::GetMessagingInterface()->RegisterListener(onMessage);
    if(registered)SKSE::log::info("Plugin load complete; waiting for game data");
    else SKSE::log::error("Plugin load failed: SKSE message listener registration failed");
    return registered;
}
