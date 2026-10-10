#pragma once
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include "TopCandidateSearch.h"
#include "SearchRetry.h"
#include "TraversalQueryBudget.h"
#include "ThreepeatMotion.h"

namespace fc {
struct Vec {
    float x{}, y{}, z{};
    Vec operator+(Vec b) const { return {x+b.x,y+b.y,z+b.z}; }
    Vec operator-(Vec b) const { return {x-b.x,y-b.y,z-b.z}; }
    Vec operator*(float f) const { return {x*f,y*f,z*f}; }
    Vec operator/(float f) const { return *this*(1/f); }
    float dot(Vec b) const { return x*b.x+y*b.y+z*b.z; }
    Vec cross(Vec b) const { return {y*b.z-z*b.y,z*b.x-x*b.z,x*b.y-y*b.x}; }
    float length() const { return std::sqrt(dot(*this)); }
    Vec unit() const { const float l=length(); return l>0.0001f ? *this*(1/l) : Vec{}; }
    bool finite() const { return std::isfinite(x)&&std::isfinite(y)&&std::isfinite(z); }
};
struct Hit { Vec point, normal; bool climbable{}; };
struct AuthoredTrajectory {
    struct Knot {float phase{};Vec displacement{};};
    std::array<Knot,65> knots{};
    std::uint32_t count{};
    bool valid() const {
        if(!count)return true;if(count<2||count>knots.size())return false;
        if(knots[0].phase!=0||knots[count-1].phase!=1||knots[0].displacement.length()>=.001f)return false;
        float distance=0;
        for(std::uint32_t i=0;i<count;++i) {
            const auto& knot=knots[i];const auto value=knot.displacement;
            if(!std::isfinite(knot.phase)||knot.phase<0||knot.phase>1||!value.finite()||
                std::abs(value.x)>500||std::abs(value.y)>500||std::abs(value.z)>500)return false;
            if(i) {
                if(knot.phase-knots[i-1].phase<.0001f)return false;
                distance+=(value-knots[i-1].displacement).length();
            }
        }
        return distance<=2000;
    }
    Vec sample(float phase) const {
        if(count<2||count>knots.size())return {};
        phase=std::clamp(phase,0.f,1.f);
        std::uint32_t index=0;while(index+2<count&&knots[index+1].phase<phase)++index;
        const auto& a=knots[index];const auto& b=knots[index+1];
        return a.displacement+(b.displacement-a.displacement)*((phase-a.phase)/(b.phase-a.phase));
    }
    float nextPhase(float phase,float limit) const {
        for(std::uint32_t i=1;i<std::min(count,std::uint32_t(knots.size()));++i)if(knots[i].phase>phase&&knots[i].phase<limit)return knots[i].phase;
        return limit;
    }
};
struct AuthoredMotion {
    bool enabled{};
    float seconds{},stride{};
    AuthoredTrajectory trajectory;
    std::array<std::array<float,2>,65> contacts{};
};
struct AuthoredWallRunSequences {
    std::array<AuthoredMotion,5> launches,catches,braces;
    std::array<bool,5> valid{};
};
struct ContextHopReference {
    AuthoredMotion preparation,recovery;
    float height{},halfWidth{},forward{};
    std::array<Vec,2> toes{};
    bool valid{};
};
enum class Motion : int;
struct World {
    virtual ~World()=default;
    virtual std::optional<Hit> ray(Vec from,Vec to)=0;

    virtual bool actionBodyClear(Motion,Vec,Vec,float,float,Vec) {return false;}
    virtual bool actionBodyPathClear(Motion m,Vec a,Vec b,float p,float q,Vec n,Vec) {return actionBodyClear(m,a,b,p,q,n);}
};
using AuthoredQueryWorld=TraversalQueryBudget<World,Vec,Hit,Motion>;
struct Settings {
    float reach=110, gap=30, radius=22, height=125, chest=70, grip=112;
    float climbSpeed=100, sideSpeed=82, downSpeed=64, maxNormalZ=0.70f;
    float drain=10, hangDrain=0, startStamina=12, mantleCost=12, mantleSeconds=1.25f;
    float approachSeconds=.32f, groundJumpHeight=88.f, runSpeed=330, diagonalRunMultiplier=1.15f;
    float hopOut=32, kickOut=52;
    bool fancyJumps=true;
    bool contextActions=true;
    bool staminaEnabled=true,contextualMantleEnabled=true;
    std::array<float,2> automaticSideWeights{1.f,1.f};
    ThreepeatProfile threepeatProfile=defaultThreepeatProfile();
    bool automaticClimbActions=false;
    bool legacyAutomaticHops=false;
    bool surfaceActionVariants=false;
    bool wallRunEnabled=true,wallRunObstacleJumps=false;
    float autoActionMinSeconds=2.f,autoActionMaxSeconds=3.5f;
    float contextScale=1;

    bool threepeatAnimations=false;
    float threepeatHangHeight{},threepeatHandHalfWidth{},threepeatHangForward{};
    std::array<Vec,2> threepeatHangToes{};
    std::array<float,2> threepeatHopDistance{},threepeatHopSeconds{};
    float threepeatMantleHeight{},threepeatMantleSeconds{},threepeatMantlePalmHeight{},threepeatMantleHalfWidth{},threepeatMantleForward{};
    std::array<Vec,2> threepeatMantleReplant{};
    bool authoredMantle{};
    AuthoredTrajectory authoredMantleTrajectory;
    std::array<std::array<float,2>,65> authoredMantleContacts{};
    std::shared_ptr<const std::array<AuthoredMotion,42>> authoredMotions;
    std::shared_ptr<const AuthoredWallRunSequences> authoredWallRunSequences;
    std::shared_ptr<const std::array<ContextHopReference,2>> contextHopReferences;
};

inline std::optional<Vec> threepeatReplantContact(World& world,Vec initial,Vec outward,Vec offset,float scale) {
    const Vec right{-outward.y,outward.x,0};
    const Vec tangent=(right*offset.x-outward*offset.y)*scale;
    if(!tangent.finite()||tangent.length()>24*scale)return {};
    const Vec guess=initial+tangent;
    const auto hit=world.ray(guess+Vec{0,0,8*scale},guess-Vec{0,0,8*scale});
    if(!hit||!hit->climbable||hit->normal.z<.95f||(hit->point-guess).length()>=3*scale)return {};
    return hit->point+hit->normal*(.8f*scale);
}

#include "GripEdge.h"
#include "CornerTraversal.h"
enum class State { idle, approach, wall, ledge, mantle, action };
enum class Motion : int { none=0, hang=1, up=2, down=3, left=4, right=5, mantle=6, step=7, reach=8,
    hopLeft=9, hopRight=10, hopUp=11, drop=15,
    jumpCatch=16, dropBack=18, ledgeCatch=19,
    runUp=20, runLeft=21, runRight=22, runDiagonalLeft=23, runDiagonalRight=24,
    runLaunch=25, runCatch=26, kickUp=27, kickLeft=28, kickRight=29, runLaunchLeft=34, runLaunchRight=35, sideBrace=36, backFlipOut=37,
    contextHang=39,contextHopLeft=40,contextHopRight=41,contextMantle=42 };

inline constexpr int legacyMotionCount=38,motionCount=42,activeMotionCount=31;
inline constexpr bool isActiveMotion(Motion motion) {
    const int id=static_cast<int>(motion);
    return id>=1&&id<=motionCount&&id!=6&&id!=7&&id!=12&&id!=13&&id!=14&&id!=17&&id!=30&&id!=31&&id!=32&&id!=33&&id!=38;
}
inline bool threepeatHop(Motion m) {return m==Motion::contextHopLeft||m==Motion::contextHopRight;}
inline bool threepeatMotion(Motion m) {return m>=Motion::contextHang&&m<=Motion::contextMantle;}
inline bool authoredMovingLoop(Motion m) {return (m>=Motion::up&&m<=Motion::right)||(m>=Motion::runUp&&m<=Motion::runDiagonalRight);}
inline bool authoredIdleLoop(Motion m) {return m==Motion::hang||m==Motion::contextHang;}

inline constexpr float capturedWallPalmOffset=7.f;
inline bool runMotion(Motion m) {return m>=Motion::runUp&&m<=Motion::runDiagonalRight;}
inline int wallRunDirectionIndex(Motion m) {return runMotion(m)?int(m)-int(Motion::runUp):-1;}
inline bool wallRunLaunch(Motion m) {return m==Motion::runLaunch||m==Motion::runLaunchLeft||m==Motion::runLaunchRight;}
inline bool hopMotion(Motion m) {return (m>=Motion::hopLeft&&m<=Motion::hopUp)||(m>=Motion::kickUp&&m<=Motion::kickRight)||threepeatHop(m);}
#include "EdgePlan.h"

inline constexpr float jumpActionSeconds(bool running) {return running?.44f:.52f;}
inline constexpr float wallRunObstacleSeconds=.40f;
inline float wallRunObstacleDuration(float distance,float speed,float outward) {

    return std::max({wallRunObstacleSeconds,1.5f*distance/(850.f+.5f*speed),3.14159265f*outward/680.f});
}
inline constexpr float backFlipExitSeconds=.74f;
inline constexpr int backFlipExitSegments=12;
inline Vec backFlipExitVelocity(Vec outward) {return outward*240.f+Vec{0,0,-160};}
inline Vec backFlipExitTangent(Vec outward,float phase) {
    const float p=std::clamp(phase,0.f,1.f),slope=6*p*(1-p),terminalSlope=3*p*p-2*p;
    return outward*((180*slope+240*backFlipExitSeconds*terminalSlope)/backFlipExitSeconds)+
        Vec{0,0,(20*slope-160*backFlipExitSeconds*terminalSlope+1280*p*(1-p)*(1-2*p))/backFlipExitSeconds};
}
inline Vec backFlipExitPoint(Vec start,Vec outward,float phase) {
    if(phase>=1)return start+outward*180.f+Vec{0,0,20}+backFlipExitVelocity(outward)*((phase-1)*backFlipExitSeconds);
    const float p=std::max(0.f,phase),p2=p*p,p3=p2*p,smooth=3*p2-2*p3;

    const float outwardTravel=180*smooth+240*backFlipExitSeconds*(p3-p2);
    const float lift=20*smooth-160*backFlipExitSeconds*(p3-p2)+640*p2*(1-p)*(1-p);
    return start+outward*outwardTravel+Vec{0,0,lift};
}

inline float actionPushPhase(float p) {return std::clamp(p/.20f,0.f,1.f)*.25f;}
inline float actionReturnPhase(float p) {const float start=.60f;return std::clamp((p-start)/(1-start),0.f,1.f);}

enum class AttachFailure { none, unavailable, stamina, noWall, surface, support, tooFar, clearance };
inline const char* name(AttachFailure reason) {
    switch(reason) {
    case AttachFailure::none: return "attached";
    case AttachFailure::unavailable: return "cooldown or invalid actor position";
    case AttachFailure::stamina: return "insufficient stamina";
    case AttachFailure::noWall: return "no wall ray hit";
    case AttachFailure::surface: return "surface layer, slope or facing rejected";
    case AttachFailure::support: return "surface too narrow or discontinuous";
    case AttachFailure::tooFar: return "outside jump-grab distance";
    case AttachFailure::clearance: return "body clearance blocked";
    }
    return "unknown";
}
struct Input { float x{},y{}; bool release{}, mantle{},hop{},backDrop{},run{}; float modeBlend=-1; };
struct Result { bool released{}; Motion motion=Motion::none; float staminaCost{}; bool completed{}; const char* reason="none"; Vec releaseVelocity{}; };

class Traversal {
    friend class TraversalCapture;
public:
    Settings cfg;
    State state=State::idle;
    Vec position{}, normal{}, surfaceNormal{};
    float cooldown{};
    float lastAttachDistance{};
    AttachFailure lastFailure=AttachFailure::none;
    mutable std::optional<Hit> blockedHit;
    mutable Vec blockedFrom{},blockedTo{};
    const char* blockedReason="none";
    mutable const char* ledgeReason="not checked";
    mutable bool authoredQueryBudgetExhausted{};
    bool wallRunning() const {return running;}
    bool geometryHolding() const {return geometryHeld;}
    bool geometryRetryDue(float dt) const {return geometryHeld&&std::isfinite(dt)&&dt>0&&dt>=geometryRetry;}
    bool recoveringSupport() const {return supportRecoveryAction;}
    bool resting() const {return staminaResting;}
    Vec direction() const {return moveDirection;}
    bool active() const { return state!=State::idle; }
    Motion wallRunDirection(Motion m) const {return runMotion(m)?m:!entryPose&&m==actionMotion&&(wallRunLaunch(m)||m==Motion::runCatch||m==Motion::sideBrace)?actionWallRunDirection:Motion::none;}
    Motion contextHopDirection() const {return edgePreparation.active&&threepeatHop(edgePreparation.motion)?edgePreparation.motion:threepeatHop(actionMotion)?actionMotion:Motion::none;}
    Motion poseDirection(Motion m) const {return m==Motion::contextHang?contextHopDirection():wallRunDirection(m);}
    const ContextHopReference* contextHopReference(Motion direction=Motion::none) const {
        if(direction==Motion::none)direction=contextHopDirection();
        if(!threepeatHop(direction)||!cfg.contextHopReferences)return nullptr;
        const auto& reference=(*cfg.contextHopReferences)[direction==Motion::contextHopLeft?0:1];
        return reference.valid?&reference:nullptr;
    }
    float contextHangHeight(Motion direction=Motion::none) const {const auto* r=contextHopReference(direction);return r?r->height:cfg.threepeatHangHeight;}
    float contextHangHalfWidth(Motion direction=Motion::none) const {const auto* r=contextHopReference(direction);return r?r->halfWidth:cfg.threepeatHandHalfWidth;}
    float contextHangForward() const {const auto* r=contextHopReference();return r?r->forward:cfg.threepeatHangForward;}
    const std::array<Vec,2>& contextHangToes() const {const auto* r=contextHopReference();return r?r->toes:cfg.threepeatHangToes;}
    float contextHangPhase() const {
        const auto* reference=contextHopReference();if(!reference)return 0;
        return std::clamp(edgeSettled?(.18f-actionCooldown)/std::max(.001f,reference->recovery.seconds):
            edgePreparation.settle/std::max(.001f,reference->preparation.seconds),0.f,1.f);
    }
    bool authored(Motion m,Motion direction=Motion::none) const {return authoredMotion(m,direction).enabled;}
    const AuthoredMotion& authoredMotion(Motion m,Motion direction=Motion::none) const {
        static const AuthoredMotion empty{};
        if(!isActiveMotion(m))return empty;
        if(m==Motion::contextHang)if(const auto* reference=contextHopReference(direction))return edgeSettled?reference->recovery:reference->preparation;
        const int index=wallRunDirectionIndex(direction==Motion::none?wallRunDirection(m):direction);
        if(index>=0&&cfg.authoredWallRunSequences&&cfg.authoredWallRunSequences->valid[index]) {
            if(wallRunLaunch(m))return cfg.authoredWallRunSequences->launches[index];
            if(m==Motion::runCatch)return cfg.authoredWallRunSequences->catches[index];
            if(m==Motion::sideBrace&&cfg.authoredWallRunSequences->braces[index].seconds>0)return cfg.authoredWallRunSequences->braces[index];
        }
        return cfg.authoredMotions?(*cfg.authoredMotions)[std::size_t(int(m)-1)]:empty;
    }
    float motionDuration(Motion m,float fallback) const {return authored(m)?std::max(.001f,authoredMotion(m).seconds):fallback;}
    bool authoredPath(Motion m,Motion direction=Motion::none) const {
        if(!authored(m,direction)||authoredMovingLoop(m)||authoredIdleLoop(m))return false;
        const auto& path=authoredMotion(m,direction).trajectory;
        for(std::uint32_t i=0;i<std::min(path.count,std::uint32_t(path.knots.size()));++i)
            if(path.knots[i].displacement.dot(path.knots[i].displacement)>=4.f)return true;
        return false;
    }
    Vec authoredRoot(Motion m,float phase) const {
        if(!authored(m)||authoredIdleLoop(m))return {};
        const auto& path=authoredMotion(m).trajectory;const auto end=path.sample(1);
        if(authoredMovingLoop(m))return end.length()>=2.f?end*std::clamp(phase,0.f,1.f):Vec{};
        return authoredPath(m)?path.sample(phase):Vec{};
    }
    float authoredPathProgress(Motion m,float phase) const {
        const auto& path=authoredMotion(m).trajectory;const auto end=path.sample(1);
        return end.dot(end)>=4.f?path.sample(phase).dot(end)/end.dot(end):ease(std::clamp(phase,0.f,1.f));
    }
    float motionHandWeight(Motion m,int hand,float phase) const {
        const auto& samples=authoredMotion(m).contacts;const float at=std::clamp(phase,0.f,1.f)*64;
        const auto index=std::min(std::size_t(at),std::size_t(63));
        return samples[index][hand]+(samples[index+1][hand]-samples[index][hand])*(at-float(index));
    }
    Vec authoredMotionPoint(Motion m,Vec from,Vec to,Vec outward,float phase) const {
        const auto& path=authoredMotion(m).trajectory;const auto end=path.sample(1);
        const float clamped=std::clamp(phase,0.f,1.f);const auto sample=path.sample(clamped);
        const float along=authoredPathProgress(m,clamped);
        const Vec residual=sample-end*along,right{-outward.y,outward.x,0};
        Vec result=from+(to-from)*along+(right*residual.x-outward*residual.y+Vec{0,0,residual.z})*std::clamp(cfg.contextScale,.5f,2.f);
        if(phase>1) {
            const float step=.002f;
            const auto before=authoredMotionPoint(m,from,to,outward,1-step);
            result=result+(result-before)*((phase-1)/step);
        }
        return result;
    }
    float authoredPhase(Motion m) const {return entryPose&&m==entryMotion?approachTime:m==Motion::contextMantle?std::max(0.f,mantleTime):actionTime;}
    Vec actionPathPoint(float phase) const {return actionPoint(phase);}
    void stop() { geometryHeld=staminaResting=supportRecoveryAction=false;geometryRetry=supportRecoveryRetry=0;supportRecoveryCursor=0;topSearchRetry.reset();hopSearchRetry.reset();state=State::idle; cooldown=0.6f;cornerActive=false;edgePreparation.active=false;automaticPreparation=false;resetAutomaticClock(); }
    void reset() { stop(); cooldown=0;mantleSelectionStatus=0; }
    void tickCooldown(float dt) { cooldown=std::max(0.0f,cooldown-std::clamp(dt,0.0f,0.05f)); }
    float progress() const { return state==State::approach ? approachTime : state==State::action ? actionTime : std::max(0.f,mantleTime); }
    bool preciseTopContacts() const {return threepeatMantle;}
    bool crestTop() const {return mantleCrest;}
    bool lowTopStep() const {
        return !threepeatMantle&&(mantleLip.z-mantleFrom.z)<=70.f*std::clamp(cfg.contextScale,.5f,2.f);
    }
    float topSampleBegin() const {
        if(cfg.authoredMantle)return 0.f;
        if(threepeatMantle)return 0.f;
        if(lowTopStep())return std::max(.72f,std::max(cfg.threepeatProfile.mantleRelease[0][1],cfg.threepeatProfile.mantleRelease[1][1]));
        const float scale=std::clamp(cfg.contextScale,.5f,2.f);
        const float height=(mantleLip.z-mantleFrom.z)/scale;
        const float palmHeight=cfg.threepeatMantlePalmHeight>0?cfg.threepeatMantlePalmHeight:128.24f;
        return height<palmHeight-24.f?std::max(cfg.threepeatProfile.mantleRelease[0][1],cfg.threepeatProfile.mantleRelease[1][1]):0.f;
    }
    float topSamplePhase(float phase) const {const float begin=topSampleBegin();return begin+(1-begin)*std::max(0.f,phase);}
    float topSeconds() const {if(cfg.authoredMantle)return std::max(.001f,cfg.threepeatMantleSeconds);return std::max(lowTopStep()?.50f:.25f,cfg.threepeatMantleSeconds>0?cfg.threepeatMantleSeconds*(1-topSampleBegin()):cfg.mantleSeconds);}
    float topPreparation() const {if(cfg.authoredMantle||lowTopStep())return 1.f;return std::clamp(1.f+mantleTime*topSeconds()/.60f,0.f,1.f);}
    bool topReplanted(float phase) const {return !cfg.authoredMantle&&threepeatMantle&&phase>=cfg.threepeatProfile.mantleReplant[0];}
    Vec topReplantOffset(int hand) const {return cfg.threepeatMantleReplant[hand];}
    float topHandWeight(int hand,float phase) const {
        if(!cfg.authoredMantle)return threepeatMantleWeight(hand,phase,cfg.threepeatProfile);
        const float at=std::clamp(phase,0.f,1.f)*64;const auto index=std::min(std::size_t(at),std::size_t(63));
        return cfg.authoredMantleContacts[index][hand]+(cfg.authoredMantleContacts[index+1][hand]-cfg.authoredMantleContacts[index][hand])*(at-float(index));
    }
    float topRiseProgress(float phase) const {return topSampleBegin()>.5f?1.f:ease((phase-.04f)/.88f);}
    float topForwardProgress(float phase) const {return topSampleBegin()>.5f?1.f:ease((phase-.32f)/.60f);}
    float topClearanceProgress(float phase) const {return ease((phase-.45f)/.15f);}
    float actionProgress() const { return actionTime; }
    bool runningAction() const {return actionBeganRunning;}
    Vec actionHeading() const {return actionDirection;}
    float actionDuration() const {return actionSeconds;}
    float actionRouteDistance() const {
        const Vec route=actionTo-actionFrom;const float distance=route.length();
        return distance*hopAlongPhase(actionTime)+(actionTime>=1?(position-actionTo).dot(route.unit()):0.f);
    }
    float stalledSeconds() const { return stalled; }
    void entry(Motion m,bool jump=false) {
        if(!isActiveMotion(m))return;
        if(m!=entryMotion)entrySeconds=approachLift>0?std::max(cfg.approachSeconds,.28f+approachLift/260.f):cfg.approachSeconds;
        entryMotion=m;entryPose=true;jumpEntry=jump||approachLift>0;entrySeconds=motionDuration(m,entrySeconds);
    }
    bool sameAuthoredEntryPath(const Traversal& checked) const {
        auto same=[](Vec a,Vec b){return a.x==b.x&&a.y==b.y&&a.z==b.z;};
        if(!authoredPath(entryMotion)||!checked.authoredPath(checked.entryMotion)||entryMotion!=checked.entryMotion||
            !entryPose||!checked.entryPose||approachTime!=0||checked.approachTime!=0||state!=checked.state||
            !same(position,checked.position)||!same(normal,checked.normal)||!same(surfaceNormal,checked.surfaceNormal)||
            !same(approachFrom,checked.approachFrom)||!same(approachTo,checked.approachTo)||
            jumpEntry!=checked.jumpEntry||approachLift!=checked.approachLift||approachRounded!=checked.approachRounded||
            entrySeconds!=checked.entrySeconds||cfg.radius!=checked.cfg.radius||cfg.height!=checked.cfg.height||
            cfg.chest!=checked.cfg.chest||cfg.gap!=checked.cfg.gap||cfg.contextScale!=checked.cfg.contextScale)return false;
        const auto& a=authoredMotion(entryMotion).trajectory;const auto& b=checked.authoredMotion(entryMotion).trajectory;
        if(a.count!=b.count||!a.valid()||!b.valid())return false;
        for(std::uint32_t i=0;i<a.count;++i)
            if(a.knots[i].phase!=b.knots[i].phase||!same(a.knots[i].displacement,b.knots[i].displacement))return false;
        return true;
    }
    bool entry(World& w,Motion m,bool jump=false,const Traversal* checked=nullptr) {
        const bool changed=m!=entryMotion;
        entry(m,jump);
        if(!authored(m))return !changed||entryPathClear(w,approachFrom,approachTo,jumpEntry,0,1,approachFrom,approachLift,approachRounded);
        const bool prechecked=checked&&checked!=this&&checked->lastFailure==AttachFailure::none&&sameAuthoredEntryPath(*checked);
        Vec previous=approachFrom;float phase=0;
        while(!prechecked&&phase<1) {
            phase=authoredMotion(m).trajectory.nextPhase(phase,std::min(1.f,phase+1.f/32));
            const auto point=entryPathPoint(phase);
            if(!clearPath(w,previous,point,approachRounded))return false;
            previous=point;
        }
        if(state==State::wall){state=State::approach;position=approachFrom;approachTime=0;}
        return true;
    }
    Vec entryPathPoint(float phase) const {
        return authoredPath(entryMotion)?authoredMotionPoint(entryMotion,approachFrom,approachTo,normal,phase):entryPoint(approachFrom,approachTo,phase,jumpEntry,approachLift);
    }
    float reachProgress() const { return approachTime; }
    Motion entrySelection() const {return entryMotion;}
    float entryLiftHeight() const {return approachLift;}
    float entryDuration() const {return entrySeconds;}
    bool roundedEntryPath() const {return approachRounded;}
    Vec entryTarget() const {return approachTo;}
    std::optional<float> entryFallbackTopRise() const {return entryUsedTopFallback?std::optional<float>{entryTopRise}:std::nullopt;}
    Vec topTarget() const { return mantleTo; }
    Vec topLip() const { return mantleLip; }
    Vec topStart() const { return mantleFrom; }
    bool roundedTopPath() const {return roundedMantle;}
    const char* topRouteKind() const {return !cfg.authoredMantle?"calibrated":mantleRoute.count?"corridor":"source";}
    Vec topPathPoint(float phase) const {
        if(cfg.authoredMantle&&authoredPath(Motion::contextMantle))return authoredMantlePoint(phase,mantleFrom,mantleTo);
        if(!cfg.authoredMantle&&lowTopStep())phase=.22f+.78f*std::clamp(phase,0.f,1.f);
        return roundedMantle?roundedMantlePoint(phase,mantleFrom,mantleApex,mantleTo):rawMantlePoint(phase,mantleFrom,mantleApex,mantleTo);
    }
    Motion topMotion() const {return Motion::contextMantle;}
    Vec topHand(int hand) const {return mantleHands[hand];}
    Vec topHandNormal(int hand) const {return mantleHandNormals[hand];}
    bool usesEdgeTargets(Motion motion) const {
        return (edgeAction&&(motion==actionMotion||(edgeSettled&&(motion==Motion::hang||motion==Motion::contextHang))))||
            (motion==Motion::contextHang&&edgePreparation.active&&
                threepeatHop(edgePreparation.motion));
    }
    bool holdsDestinationEdge(Motion motion) const {return edgeSettled&&(motion==Motion::hang||motion==Motion::contextHang);}
    bool holdsPreparedEdge(Motion motion) const {return edgePreparation.active&&
        (motion==Motion::contextHang&&threepeatHop(edgePreparation.motion));}
    float preparedEdgeWeight(Motion motion) const {

        return motion==Motion::contextHang&&holdsPreparedEdge(motion)?
            ease((edgePreparation.settle-.18f)/.08f):1.f;
    }
    float preparedEdgeClearance(Motion motion) const {

        return motion==Motion::contextHang&&holdsPreparedEdge(motion)?
            4.f*(1-ease((edgePreparation.settle-.10f)/.12f)):0.f;
    }
    Vec edgeHand(int hand,bool destination) const {
        if(edgePreparation.active&&threepeatHop(edgePreparation.motion))return edgePreparation.source.hands[hand];
        return (destination?actionTargetEdge:actionSourceEdge).hands[hand];
    }
    bool usesWallTargets(Motion motion) const {
        return usesEdgeTargets(motion)&&(edgePreparation.active?edgePreparation.source.wallPatch:actionSourceEdge.wallPatch);
    }
    Vec edgeContactNormal(int,bool destination) const {
        const auto& contact=edgePreparation.active?edgePreparation.source:(destination?actionTargetEdge:actionSourceEdge);
        return contact.wallPatch?contact.normal:Vec{0,0,1};
    }
    Vec edgeStart() const {return actionFrom;}
    Vec edgeTarget() const {return actionTo;}
    bool turningCorner() const {return cornerActive;}
    bool preparingEdge() const {return edgePreparation.active;}
    const char* contextReason() const {return edgePlanReason(edgePreparation.status);}
    unsigned contextStatus() const {return edgePreparation.status;}
    const char* threepeatReason() const {return edgePlanReason(threepeatPlanStatus);}
    unsigned threepeatStatus() const {return threepeatPlanStatus;}
    const char* topSelectionReason() const {
        constexpr const char* reasons[]={"not checked","disabled","sloped surface or crest","feet unsupported","height mismatch","travel mismatch","edge mismatch","replant unsupported","selected"};
        return reasons[std::min(mantleSelectionStatus,8u)];
    }
    unsigned automaticActionCount() const {return automaticActions;}
    unsigned automaticAttemptCount() const {return automaticAttempts;}
    unsigned automaticOpportunityCount() const {return automaticOpportunities;}
    unsigned surfaceActionCount() const {return surfaceActions;}
    unsigned contextIdleCount() const {return contextIdles;}
    bool obstacleJumpActive() const {return obstacleJump;}
    unsigned obstacleJumpCount() const {return obstacleJumps;}
    float automaticActionPendingSeconds() const {return automaticInterval>0?std::max(0.f,automaticInterval-automaticElapsed):cfg.autoActionMinSeconds;}

    bool attach(World& world,Vec feet,Vec facing,float stamina,float maxSnap=1000,bool explicitAirCatch=false,bool groundJump=false,Motion selectedEntry=Motion::reach,Motion nearEntry=Motion::none,float nearEntryDistance=48) {
        lastFailure=AttachFailure::unavailable;
        lastAttachDistance=0;entryUsedTopFallback=false;entryTopRise=0;

        if(active()||(cooldown>0&&!explicitAirCatch)||!feet.finite()) return false;
        if(cfg.staminaEnabled&&stamina<cfg.startStamina) { lastFailure=AttachFailure::stamina; return false; }
        facing.z=0; facing=facing.unit();
        if(facing.length()<0.9f) return false;
        struct EntryBudget final:World {
            World& source;unsigned count{};bool exhausted{};
            explicit EntryBudget(World& value):source(value){}
            std::optional<Hit> ray(Vec from,Vec to) override {
                if(count>=12288){exhausted=true;return Hit{from,(from-to).unit(),false};}
                ++count;return source.ray(from,to);
            }
        } sourceBudget(world);
        World& w=authoredPath(selectedEntry)||authoredPath(nearEntry)?static_cast<World&>(sourceBudget):world;
        Motion acceptedEntry=selectedEntry;
        Vec target{};bool found=false;float selectedLift=0;bool selectedRounded=false;std::optional<float> selectedTopRise;
        AttachFailure bestFailure=AttachFailure::noWall;float rejectedDistance=0;bool rejectedCandidate=false;
        auto candidateFailure=[&](AttachFailure failure,float distance) {

            auto depth=[](AttachFailure reason){return reason==AttachFailure::support?3:reason==AttachFailure::clearance?2:1;};
            if(!rejectedCandidate||distance<rejectedDistance||
                (distance==rejectedDistance&&depth(failure)>depth(bestFailure))) {
                rejectedCandidate=true;rejectedDistance=distance;bestFailure=failure;
            }
        };

        World* entryWorld=&w;
        auto findEntry=[&](Vec origin,bool rounded,bool jump,bool upperOnly)->std::optional<Vec> {
        auto& entryGeometry=*entryWorld;
        for(float angle:{0.f,-.35f,.35f,-.7f,.7f,-1.05f,1.05f}) {
            const Vec search{facing.x*std::cos(angle)-facing.y*std::sin(angle),
                facing.x*std::sin(angle)+facing.y*std::cos(angle),0};
            AttachFailure failure=AttachFailure::noWall;
            const auto h=wall(entryGeometry,origin,search,cfg.reach,&failure);
            if(!h){if(!rejectedCandidate&&(bestFailure==AttachFailure::noWall||failure==AttachFailure::support))bestFailure=failure;continue;}
            normal=horizontal(h->normal);surfaceNormal=h->normal.unit();
            if(normal.dot(facing)>-.35f){if(!rejectedCandidate)bestFailure=AttachFailure::surface;continue;}
            for(bool alignToHit:{false,true}) {
                if(alignToHit&&angle==0)continue;
                Vec candidateFeet=origin;
                if(alignToHit) {
                    const Vec side{-normal.y,normal.x,0};
                    candidateFeet=candidateFeet+side*(h->point-origin).dot(side);
                }
                const Vec candidate=offsetFromSurface(candidateFeet,*h);
                const float distance=(candidate-origin).length();
                if(distance>maxSnap){candidateFailure(AttachFailure::tooFar,distance);continue;}
                const Motion candidateEntry=isActiveMotion(nearEntry)&&std::isfinite(nearEntryDistance)&&nearEntryDistance>=0&&
                    (candidate-feet).length()<=nearEntryDistance?nearEntry:selectedEntry;
                if(authoredPath(candidateEntry)) {
                    Vec previous=feet;float phase=0;bool clear=true;
                    while(phase<1) {
                        phase=authoredMotion(candidateEntry).trajectory.nextPhase(phase,std::min(1.f,phase+1.f/32));
                        const auto point=authoredMotionPoint(candidateEntry,feet,candidate,normal,phase);
                        if(!clearPath(entryGeometry,previous,point,rounded)){clear=false;break;}previous=point;
                    }
                    if(!clear){candidateFailure(AttachFailure::clearance,distance);continue;}
                } else if(!clearPath(entryGeometry,origin,candidate,rounded)){candidateFailure(AttachFailure::clearance,distance);continue;}

                if(rounded&&(!clearPath(entryGeometry,candidate,candidate)||!entrySideClear(entryGeometry,candidate))) {
                    candidateFailure(AttachFailure::clearance,distance);continue;
                }

                std::optional<float> fallbackTopRise;
                if(!gripSupport(entryGeometry,candidate,*h,false)) {
                    if(upperOnly){candidateFailure(AttachFailure::support,distance);continue;}
                    const auto savedPosition=position;position=candidate;
                    const auto lowTop=findLedge(entryGeometry);position=savedPosition;
                    if(!lowTop){candidateFailure(AttachFailure::support,distance);continue;}
                    fallbackTopRise=lowTop->lip.z-feet.z;
                }

                if(!authoredPath(candidateEntry)&&jump&&cfg.approachSeconds>0&&
                    !entryPathClear(entryGeometry,origin,candidate,true,0,1,origin,0,rounded)) {
                    candidateFailure(AttachFailure::clearance,distance);continue;
                }
                lastAttachDistance=distance;selectedTopRise=fallbackTopRise;acceptedEntry=candidateEntry;return candidate;
            }
        }
        return {};
        };
        if(const auto ordinary=findEntry(feet,false,groundJump,false)){target=*ordinary;found=true;}

        if(!found&&groundJump&&cfg.approachSeconds>0&&entryFootprint(w,feet,facing)) {
            struct GroundBudget final:World {
                World& source;unsigned count{};bool exhausted{};
                explicit GroundBudget(World& value):source(value){}
                std::optional<Hit> ray(Vec from,Vec to)override {
                    if(count>=6144){exhausted=true;return Hit{from,(from-to).unit(),false};}
                    ++count;return source.ray(from,to);
                }
            } limited(w);
            entryWorld=&limited;
            if(const auto rounded=findEntry(feet,true,true,false)) {
                if(!limited.exhausted){target=*rounded;found=true;selectedRounded=true;}
            }

            if(!found&&std::isfinite(cfg.groundJumpHeight))for(float lift:{24.f,40.f,56.f,72.f,88.f}) {
                if(limited.exhausted||lift>std::clamp(cfg.groundJumpHeight,0.f,88.f))break;
                const auto raised=findEntry(feet+Vec{0,0,lift},true,false,true);
                if(!raised)continue;
                if((!authoredPath(acceptedEntry)&&!entryPathClear(limited,feet,*raised,true,0,1,feet,lift,true))||limited.exhausted) {
                    candidateFailure(AttachFailure::clearance,lastAttachDistance);continue;
                }
                target=*raised;found=true;selectedRounded=true;selectedLift=lift;break;
            }
        }
        if(sourceBudget.exhausted){lastFailure=AttachFailure::clearance;lastAttachDistance=rejectedDistance;return false;}
        if(!found){lastFailure=bestFailure;lastAttachDistance=rejectedDistance;return false;}
        approachFrom=feet; approachTo=target; approachTime=0;entryPose=true;
        approachLift=selectedLift;approachRounded=selectedRounded;
        entryUsedTopFallback=selectedTopRise.has_value();entryTopRise=selectedTopRise.value_or(0.f);
        entrySeconds=selectedLift>0?std::max(cfg.approachSeconds,.28f+selectedLift/260.f):cfg.approachSeconds;
        stalled=0;detourProbe=0;actionTime=0;actionCooldown=0;runBlend=diagonalRunBlend=0;entryMotion=acceptedEntry;
        entrySeconds=motionDuration(entryMotion,entrySeconds);
        roofTransfer=false;roofProbeCooldown=0;actionStartSurface=actionTargetSurface={};mantleCrest=threepeatMantle=false;
        edgeAction=edgeSettled=false;edgeProbeCooldown=0;actionSourceEdge=actionTargetEdge={};
        cornerActive=false;cornerProbeCooldown=0;cornerRoute={};edgePreparation={};threepeatPlanStatus=mantleSelectionStatus=0;
        stableMotion=Motion::hang;actionWallRunDirection=Motion::none;clearanceMargin=0;missingSurface=0;runClearanceCooldown=0;running=false;jumpEntry=groundJump;moveDirection={};
        roundedMantle=false;mantleRoute={};automaticPreparation=false;resetAutomaticClock();
        geometryHeld=staminaResting=false;geometryRetry=0;
        supportRecoveryAction=false;supportRecoveryRetry=0;supportRecoveryCursor=0;supportRecoveryOrigin=position;supportRecoveryInput={};
        obstacleJump=false;obstacleProbeCooldown=0;actionRunSpeed=cfg.runSpeed;
        topSearchRetry.reset();hopSearchRetry.reset();
        position=cfg.approachSeconds>0 ? feet : target;
        state=cfg.approachSeconds>0 ? State::approach : State::wall;

        lastFailure=AttachFailure::none;
        return true;
    }

    Result update(World& w,Input in,float dt,float stamina) {
        if(!cfg.authoredMotions&&!cfg.authoredWallRunSequences) {
            auto result=updateWithStamina(w,in,dt,cfg.staminaEnabled?stamina:1000000.f);
            if(!cfg.staminaEnabled)result.staminaCost=0;
            return result;
        }
        const auto savedState=state;const auto savedPosition=position,savedNormal=normal,savedSurface=surfaceNormal;
        const bool savedEdge=edgeAction,savedSettled=edgeSettled,savedCorner=cornerActive;
        const auto savedSource=actionSourceEdge,savedTarget=actionTargetEdge;const auto savedPreparation=edgePreparation;
        const bool savedRunning=running;const auto savedStable=stableMotion;const float savedRunBlend=runBlend,savedDiagonalBlend=diagonalRunBlend;
        const auto savedWallRunDirection=actionWallRunDirection;
        const auto savedAutomatic=automaticActions,savedSurfaceActions=surfaceActions,savedObstacles=obstacleJumps;
        authoredActionRejected=false;authoredBackFlipPrechecked=false;authoredQueryBudgetExhausted=false;
        auto result=updateWithStamina(w,in,dt,cfg.staminaEnabled?stamina:1000000.f);
        if(authoredActionRejected||(!geometryHeld&&!staminaResting&&state==State::action&&actionTime==0&&authored(actionMotion)&&!checkAuthoredAction(w))) {
            position=savedState==State::action?actionFrom:savedPosition;normal=savedNormal;surfaceNormal=savedSurface;
            state=savedState==State::action?State::wall:savedState;
            edgeAction=savedEdge;edgeSettled=savedSettled;cornerActive=savedCorner;
            actionSourceEdge=savedSource;actionTargetEdge=savedTarget;edgePreparation=savedPreparation;
            running=savedRunning;stableMotion=savedStable;runBlend=savedRunBlend;diagonalRunBlend=savedDiagonalBlend;
            actionWallRunDirection=savedWallRunDirection;
            automaticActions=savedAutomatic;surfaceActions=savedSurfaceActions;obstacleJumps=savedObstacles;
            obstacleJump=roofTransfer=detour=false;actionCooldown=.15f;
            authoredActionChecked=authoredActionRejected=false;
            result={};result.motion=isActiveMotion(savedStable)?savedStable:Motion::hang;
            result.reason=authoredQueryBudgetExhausted?"authored action query budget exhausted":"authored action path blocked";blockedReason=result.reason;
        }
        if(!cfg.staminaEnabled)result.staminaCost=0;
        return result;
    }
    Result updateWithStamina(World& w,Input in,float dt,float stamina) {
        Result out;
        if(!active()) return out;
        dt=std::clamp(dt,0.0f,0.05f);
        if(dt<=1e-6f&&!in.release&&stamina>0) {
            out.motion=state==State::approach?entryMotion:state==State::mantle?topMotion():
                state==State::action?actionMotion:stableMotion;
            return out;
        }
        topSearchRetry.tick(dt);hopSearchRetry.tick(dt);
        supportRecoveryRetry=std::max(0.f,supportRecoveryRetry-dt);
        actionCooldown=std::max(0.f,actionCooldown-dt);
        edgeProbeCooldown=std::max(0.f,edgeProbeCooldown-dt);
        cornerProbeCooldown=std::max(0.f,cornerProbeCooldown-dt);
        roofProbeCooldown=std::max(0.f,roofProbeCooldown-dt);
        obstacleProbeCooldown=std::max(0.f,obstacleProbeCooldown-dt);
        blockedHit.reset();
        blockedReason="none";
        runClearanceCooldown=std::max(0.f,runClearanceCooldown-dt);
        const bool wasResting=staminaResting;
        if(in.release&&(state!=State::action||(actionMotion!=Motion::drop&&
            (!in.backDrop||(actionMotion!=Motion::dropBack&&actionMotion!=Motion::backFlipOut))))) {
            geometryHeld=staminaResting=false;geometryRetry=0;
            Vec target=position;

            const bool settled=state==State::wall||state==State::ledge;
            const bool backFlip=in.backDrop&&settled&&cfg.fancyJumps&&checkedBackFlip(w);
            if(in.backDrop&&!backFlip) {

                const auto velocity=backJumpVelocity(w,normal*220.f+Vec{0,0,90});
                const auto candidate=position+velocity*(.32f/3.f);
                if(clearPath(w,position,candidate))target=candidate;
            }
            if(backFlip){beginAction(Motion::backFlipOut,position,backFlipExitPoint(position,normal,1),backFlipExitSeconds);authoredActionChecked=authoredBackFlipPrechecked;}
            else beginAction(in.backDrop?Motion::dropBack:Motion::drop,position,target,in.backDrop?.32f:.16f);
        }
        const bool recoveringWall=geometryHeld&&(state==State::wall||state==State::ledge);
        const bool departing=state==State::action&&(actionMotion==Motion::drop||actionMotion==Motion::dropBack||actionMotion==Motion::backFlipOut);
        if(!departing) {
            staminaResting=stamina<=0||(wasResting&&stamina<std::max(1.f,cfg.startStamina));
            if(staminaResting) {
                running=false;runBlend=diagonalRunBlend=0;moveDirection={};resetAutomaticClock();
                edgePreparation.active=automaticPreparation=false;
                out.motion=heldMotion();if(state==State::wall||state==State::ledge)stableMotion=Motion::hang;
                out.reason=blockedReason="resting until stamina recovers";return out;
            }
            if(geometryHeld) {
                geometryRetry=std::max(0.f,geometryRetry-dt);
                if(geometryRetry>0)return heldResult("waiting for traversal geometry");
                geometryHeld=false;
            }
        }
        if(edgeSettled&&(std::abs(in.x)+std::abs(in.y)>.1f||in.hop||in.release||
            !gripEdgeStillValid(w,actionTargetEdge)||
            ((threepeatHop(actionMotion)||actionMotion==Motion::contextHang)&&
                (actionCooldown<=0||!idle39FeetSupported(w,position,actionTargetEdge)))))edgeAction=edgeSettled=false;

        const bool wallRunControls=in.y>=0&&((cfg.wallRunEnabled&&in.run)||running||runBlend>.5f||runMotion(stableMotion));
        if(wallRunControls)in.hop=false;
        if(!cfg.wallRunEnabled)in.run=false;
        running=in.run&&in.y>=0&&(std::abs(in.x)+std::abs(in.y)>.1f)&&runClearanceCooldown<=0;
        if(running&&!(state==State::action&&obstacleJump)&&!runwayClear(w,
            state==State::approach&&entryMotion==Motion::runLaunch?approachTo:position)) {
            running=false;runClearanceCooldown=.45f;
        }
        if(in.y<0)runBlend=diagonalRunBlend=0;
        moveDirection={in.x,in.y,0};
        runBlend+=std::clamp((running?1.f:0.f)-runBlend,-dt/.32f,dt/.36f);

        const bool diagonalRun=running&&in.y>.1f&&std::abs(in.x)>.1f;
        diagonalRunBlend+=std::clamp((diagonalRun?1.f:0.f)-diagonalRunBlend,-dt/.24f,dt/.24f);
        const float diagonalEase=diagonalRunBlend*diagonalRunBlend*(3-2*diagonalRunBlend);
        const float runSpeed=cfg.runSpeed*(1+(std::clamp(cfg.diagonalRunMultiplier,1.f,1.3f)-1)*diagonalEase);
        if(!automaticInputAllowed(in,wallRunControls)||
            (automaticDirection.length()>.1f&&automaticDirection.dot(Vec{in.x,in.y,0}.unit())<.45f))resetAutomaticClock();
        else automaticDirection=Vec{in.x,in.y,0}.unit();
        if(!recoveringWall&&(state==State::wall||state==State::ledge)&&!in.release&&!in.hop&&!edgePreparation.active&&!cornerActive&&actionCooldown<=0) {
            const Motion wanted=running?(std::abs(in.x)<.1f?Motion::runUp:std::abs(in.y)<.1f?
                (in.x<0?Motion::runLeft:Motion::runRight):(in.x<0?Motion::runDiagonalLeft:Motion::runDiagonalRight)):Motion::hang;
            Motion bridge=Motion::none;
            if(running&&!runMotion(stableMotion))bridge=std::abs(in.x)>.1f?(in.x<0?Motion::runLaunchLeft:Motion::runLaunchRight):Motion::runLaunch;
            else if(!running&&runMotion(stableMotion))bridge=Motion::runCatch;
            else if(running&&runMotion(stableMotion)&&wanted!=stableMotion&&
                ((stableMotion==Motion::runLeft||stableMotion==Motion::runDiagonalLeft)&&(wanted==Motion::runRight||wanted==Motion::runDiagonalRight)||
                 (stableMotion==Motion::runRight||stableMotion==Motion::runDiagonalRight)&&(wanted==Motion::runLeft||wanted==Motion::runDiagonalLeft)))bridge=Motion::sideBrace;
            const Motion bridgeDirection=bridge==Motion::runCatch?stableMotion:wanted;
            if(authored(bridge,bridgeDirection)) {
                Vec destination=position;bool supported=true;
                if(authoredPath(bridge,bridgeDirection)&&std::hypot(in.x,in.y)>.1f) {
                    const Vec direction=(Vec{-normal.y,normal.x,0}*in.x+Vec{0,0,in.y}).unit();
                    const auto proposed=position+direction*std::min(96.f,authoredMotion(bridge,bridgeDirection).trajectory.sample(1).length()*cfg.contextScale);
                    const auto grip=support(w,proposed,normal*-1);
                    supported=grip.has_value();
                    if(grip)destination=offsetFromSurface(proposed,*grip);
                }
                if(supported) {
                    beginAction(bridge,position,destination,.2f,bridgeDirection);stableMotion=wanted;
                    out.motion=bridge;out.reason="authored wall-run transition";return out;
                }
            }
        }


        if(state==State::action) {
            out.motion=actionMotion;
            if(authored(actionMotion)&&!checkAuthoredAction(w)) {
                if(departing) {
                    stop();out.motion=Motion::drop;out.released=true;out.reason="manual release: authored exit path blocked";return out;
                }
                authoredActionRejected=true;out.motion=Motion::hang;out.reason="authored action path blocked";return out;
            }
            const float rawNextTime=actionTime+dt/actionSeconds;
            const float nextTime=std::min(1.f,rawNextTime);
            Vec target=position,previous=position;float previousPhase=actionTime;
            const auto landingSupport=nextTime>=1&&hopMotion(actionMotion)&&!supportRecoveryAction?
                support(w,actionPoint(1),actionLandingNormal*-1):std::optional<Hit>{};
            std::optional<Hit> recoverySupport;
            if(supportRecoveryAction) {
                recoverySupport=recoveryLanding(w,actionTo,actionTargetSurface);
                if(!recoverySupport) {
                    if(auto retried=retrySupportRecovery(w,in,stamina))return *retried;
                    return holdGeometry("recovery destination support changed");
                }
            }
            if(obstacleJump||roofTransfer) {
                const auto destination=support(w,actionTo,actionLandingNormal*-1);
                const bool targetPresent=destination&&horizontal(destination->normal).dot(actionLandingNormal)>.95f&&
                    (offsetFromSurface(actionTo,*destination)-actionTo).length()<2&&
                    (!roofTransfer||(destination->normal.unit().dot(actionTargetSurface.unit())>.95f&&
                        gripSupport(w,actionTo,*destination,true)));
                const Vec sourceNormal=roofTransfer?horizontal(actionStartSurface):actionLandingNormal;
                const auto source=actionTime<.12f?support(w,actionFrom,sourceNormal*-1):std::optional<Hit>{};
                if(!targetPresent||(actionTime<.12f&&!source)) {
                    return holdGeometry(roofTransfer?"obstacle bypass support changed":"wall-run jump support changed");
                }
            }

            const bool sourceNeeded=authored(actionMotion)?std::max(motionHandWeight(actionMotion,0,actionTime),motionHandWeight(actionMotion,1,actionTime))>.05f:threepeatHop(actionMotion)?std::max(threepeatSourceWeight(actionMotion==Motion::contextHopLeft,0,actionTime,cfg.threepeatProfile),
                    threepeatSourceWeight(actionMotion==Motion::contextHopLeft,1,actionTime,cfg.threepeatProfile))>.05f:actionTime<.18f;
            const bool sourcePresent=!edgeAction||!sourceNeeded||gripEdgeStillValid(w,actionSourceEdge);
            if(edgeAction&&(!sourcePresent||!gripEdgeStillValid(w,actionTargetEdge))) {
                if(actionTime<.12f&&gripEdgeStillValid(w,actionSourceEdge)) {
                    state=State::wall;edgeAction=false;actionCooldown=.3f;
                    out.motion=stableMotion=Motion::hang;out.reason="planned edge changed before release";
                } else {
                    return holdGeometry("planned edge disappeared");
                }
                return out;
            }
            const int divisions=authored(actionMotion)?32:actionMotion==Motion::backFlipOut?backFlipExitSegments:
                hopMotion(actionMotion)||supportRecoveryAction?32:1;
            float scanPhase=actionTime;
            for(int knot=int(actionTime*divisions)+1;;++knot) {
                const float phase=authored(actionMotion)?authoredMotion(actionMotion).trajectory.nextPhase(scanPhase,std::min(nextTime,scanPhase+1.f/divisions)):std::min(nextTime,float(knot)/divisions);

                const bool exitMotion=actionMotion==Motion::dropBack||actionMotion==Motion::backFlipOut;
                target=actionPoint(exitMotion&&phase>=1&&!authored(actionMotion)?rawNextTime:phase);
                const Vec targetNormal=phase>=1&&landingSupport?horizontal(landingSupport->normal):roofTransfer||supportRecoveryAction?roofFacing(phase):normal;
                const Vec sourceNormal=roofTransfer||supportRecoveryAction?roofFacing(previousPhase):normal;
                if(!clearPath(w,previous,target,false,targetNormal,sourceNormal)||(roofTransfer&&!roofPathClear(w,previous,target))) {
                    if(supportRecoveryAction)if(auto retried=retrySupportRecovery(w,in,stamina))return *retried;
                    if(!departing)return holdGeometry("jump path changed");
                    if(actionMotion==Motion::backFlipOut)out.releaseVelocity=checkedBackFlipAbortVelocity(w);
                    if(edgeAction){out.motion=actionMotion;out.releaseVelocity={0,0,-30};edgeAction=false;}
                    stop();out.released=true;out.reason="jump path changed";return out;
                }
                const bool bodyClear=actionMotion!=Motion::backFlipOut||(authored(actionMotion)?
                    w.actionBodyPathClear(actionMotion,previous,target,previousPhase,phase,actionLandingNormal,actionPoint((previousPhase+phase)*.5f)):
                    w.actionBodyClear(actionMotion,previous,target,previousPhase,phase,actionLandingNormal));
                if(!bodyClear) {
                    out.releaseVelocity=checkedBackFlipAbortVelocity(w);
                    stop();out.released=true;out.reason="back flip body clearance changed";return out;
                }
                if(actionMotion>=Motion::kickUp&&actionMotion<=Motion::kickRight&&!kickClearance(w,target)) {
                    return holdGeometry("kick body clearance changed");
                }
                if(phase>=nextTime)break;
                previous=target;previousPhase=phase;scanPhase=phase;
            }
            const auto landed=nextTime>=1&&!departing?(supportRecoveryAction?recoverySupport:
                hopMotion(actionMotion)?landingSupport:support(w,target,actionLandingNormal*-1)):std::optional<Hit>{};
            std::optional<Vec> standingLanding;
            if(nextTime>=1&&!departing) {
                if(edgeAction&&(!landed||std::abs(landed->normal.unit().z)>.12f||
                    horizontal(landed->normal).dot(actionTargetEdge.normal)<.985f))
                    return holdGeometry("edge catch body support changed");
                if(hopMotion(actionMotion)||supportRecoveryAction) {
                    if(!landed) {
                        standingLanding=standingFloor(w,target,12,12);
                        if(!standingLanding||!clearPath(w,target,*standingLanding,true))
                            return holdGeometry("jump landing support lost");
                    } else {
                        const auto settled=offsetFromSurface(target,*landed);
                        if((settled-target).length()>8)return holdGeometry("jump landing moved beyond reach");
                        if(!clearPath(w,target,settled,false,horizontal(landed->normal))) {
                            if(supportRecoveryAction)if(auto retried=retrySupportRecovery(w,in,stamina))return *retried;
                            return holdGeometry("jump landing clearance blocked");
                        }
                    }
                }
            }
            actionTime=nextTime;
            position=target;out.motion=actionMotion;out.staminaCost=departing?0.f:obstacleJump?cfg.drain*(running?2.f:1.f)*dt:cfg.hangDrain*dt;
            if(roofTransfer||supportRecoveryAction) {

                const float amount=ease((actionTime-.25f)/.65f);
                normal=roofFacing(actionTime);
                surfaceNormal=(actionStartSurface*(1-amount)+actionTargetSurface*amount).unit();
            }
            if(actionTime>=1) {
                if(actionMotion==Motion::drop||actionMotion==Motion::dropBack||actionMotion==Motion::backFlipOut) {
                    if(authored(actionMotion)&&actionMotion!=Motion::drop) {
                        auto velocity=(actionPoint(1)-actionPoint(.998f))/(actionSeconds*.002f);
                        if(velocity.length()>600)velocity=velocity.unit()*600;
                        out.releaseVelocity=backJumpVelocity(w,velocity);
                    }
                    else if(actionMotion==Motion::dropBack)
                        out.releaseVelocity=backJumpVelocity(w,(actionTo-actionFrom)*(3.f/actionSeconds));
                    if(actionMotion==Motion::backFlipOut&&!authored(actionMotion))
                        out.releaseVelocity=backJumpVelocity(w,backFlipExitVelocity(actionLandingNormal));
                    stop();out.released=true;out.reason=actionMotion==Motion::backFlipOut?"manual back flip":
                        actionMotion==Motion::dropBack?"manual back push":"manual drop";
                }
                else {

                    if(hopMotion(actionMotion)||supportRecoveryAction) {
                        if(!landed) {
                            position=*standingLanding;stop();out.released=out.completed=true;out.reason="jump reached standing surface";
                        } else {
                            const auto settled=offsetFromSurface(position,*landed);
                            {
                                position=settled;normal=horizontal(landed->normal);surfaceNormal=landed->normal.unit();

                                state=State::wall;actionCooldown=.18f;stalled=missingSurface=0;roofTransfer=supportRecoveryAction=false;
                                edgeSettled=edgeAction&&!authored(actionMotion);
                                if(obstacleJump) {

                                    const float remainder=std::max(0.f,(rawNextTime-1)*actionSeconds);
                                    const Vec heading=(actionTo-actionFrom).unit();
                                    const Vec wanted=position+heading*(actionRunSpeed*remainder);
                                    if(remainder>0)if(const auto onward=support(w,wanted,normal*-1)) {
                                        const Vec next=offsetFromSurface(wanted,*onward);
                                        if((next-wanted).length()<2&&horizontal(onward->normal).dot(normal)>.95f&&
                                            clearPath(w,position,next)&&(!running||runwayClear(w,next)))position=next;
                                    }
                                    obstacleProbeCooldown=.22f;
                                }
                            }
                        }
                    } else {state=State::wall;actionCooldown=.18f;stalled=0;}
                }
            }
            return out;
        }
        if(state==State::approach) {
            const float nextPhase=std::min(1.0f,approachTime+dt/std::max(.01f,entrySeconds));
            const auto next=entryPathPoint(nextPhase);
            if(approachLift>0) {
                const auto actual=support(w,approachTo,normal*-1);
                if(!actual||(offsetFromSurface(approachTo,*actual)-approachTo).length()>2||
                    !gripSupport(w,approachTo,*actual,false)) {
                    out.releaseVelocity={0,0,-30};stop();out.released=true;out.reason="ground jump destination changed";return out;
                }
            }

            bool pathClear=true;
            if(authored(entryMotion)) {
                Vec previous=position;float phase=approachTime;
                while(phase<nextPhase) {
                    phase=authoredMotion(entryMotion).trajectory.nextPhase(phase,std::min(nextPhase,phase+1.f/32));
                    const auto point=entryPathPoint(phase);
                    if(!clearPath(w,previous,point,approachRounded)){pathClear=false;break;}previous=point;
                }
            } else pathClear=entryPathClear(w,approachFrom,approachTo,jumpEntry,approachTime,nextPhase,position,approachLift,approachRounded);
            if((approachRounded&&!entrySideClear(w,approachTo))||!pathClear) {
                if(approachLift>0)out.releaseVelocity={0,0,-30};
                stop();out.released=true;out.reason="entry path blocked";return out;
            }
            approachTime=nextPhase;position=next;out.motion=entryMotion;
            if(approachTime>=1)state=State::wall;
            return out;
        }
        if(state==State::mantle) {
            float next=std::min(1.0f,mantleTime+dt/topSeconds());
            const float release=std::max(cfg.threepeatProfile.mantleRelease[0][1],cfg.threepeatProfile.mantleRelease[1][1]);
            const bool tail=!cfg.authoredMantle&&cfg.threepeatMantleSeconds>0&&!lowTopStep()&&!mantleCrest&&topSamplePhase(next)>release;
            if(tail) {
                auto rate=[&](float phase){return 1.f+ease((topSamplePhase(phase)-release)/.14f);};
                const float step=dt/(8.f*topSeconds());next=mantleTime;
                for(int sample=0;sample<8;++sample)next+=step*rate(next+step*.5f*rate(next));
                next=std::min(1.f,next);
            }
            if(threepeatMantle&&((mantleTime<0&&(!support(w,position,normal*-1)||!edgeFeetSupported(w,position,normal)))||!threepeatTopStillValid(w,next))) {
                return holdGeometry("new mantle support changed");
            }
            if(!threepeatMantle&&!adaptiveTopStillValid(w,next)) {
                return holdGeometry("mantle support changed");
            }
            if(mantleCrest&&next>=.60f) {
                const auto support=crestStanding(w,mantleLip);
                if(!support||(*support-mantleTo).length()>1) {
                    return holdGeometry("roof crest support changed");
                }
            }

            Vec target=position;float phase=std::max(0.f,mantleTime);
            do {
                float point=tail||cfg.authoredMantle?std::min(next,phase+1.f/32.f):next;
                if(cfg.authoredMantle)point=authoredMantleNextPhase(phase,point);
                if(tail)for(float knot:{.60f,.88f})if(phase<knot&&point>knot)point=knot;
                const Vec checked=topPathPoint(point);
                if(!clearPath(w,target,checked,true))return holdGeometry("top-out path changed");
                target=checked;phase=point;
            }while(phase<next);
            position=target; mantleTime=next; out.motion=topMotion();
            if(next>=1) { stop(); out.released=true; out.completed=true;out.reason=mantleCrest?"roof crest reached":"top-out complete"; }
            return out;
        }
        if(edgePreparation.active)if(auto prepared=updateEdgePreparation(w,in,dt,stamina))return *prepared;
        bool manualHopTried=false;

        if(!recoveringWall&&(cornerActive||std::abs(in.x)>.5f)&&!wallRunControls&&!running&&in.hop&&actionCooldown<=0&&stamina>=15.f) {
            const bool sourceReady=cornerActive?
                cornerJoinValid(w,cfg,cornerRoute)&&cornerGrip(w,cfg,cornerRoute,position,surfaceNormal):
                support(w,position,normal*-1).has_value();
            if(sourceReady) {
                manualHopTried=true;
                const bool allowEdge=cfg.contextActions&&!in.run&&!runMotion(stableMotion)&&std::abs(surfaceNormal.z)<.1f;
                if(auto hop=tryManualHop(w,in,allowEdge))return *hop;
            }
        }

        if(!recoveringWall&&!cornerActive&&!edgePreparation.active&&std::abs(in.x)>.5f&&
            std::abs(surfaceNormal.z)<=cfg.maxNormalZ&&cornerProbeCooldown<=0) {
            cornerProbeCooldown=.12f;
            if(auto route=findCornerRoute(w,cfg,position,surfaceNormal,in.x,
                [&](Vec a,Vec b){return clearPath(w,a,b);})) {
                cornerRoute=*route;cornerActive=true;edgeAction=edgeSettled=false;
            }
        }
        if(cornerActive) {
            resetAutomaticClock();
            const bool inclinedCorner=std::abs(cornerRoute.sourceNormal.z)>.12f||std::abs(cornerRoute.targetNormal.z)>.12f;
            Vec input{std::clamp(in.x,-1.f,1.f),std::clamp(in.y,-1.f,1.f),0};
            if(input.length()>1)input=input.unit();
            const float mode=input.y<0?0.f:in.modeBlend<0?runBlend*runBlend*(3-2*runBlend):std::clamp(in.modeBlend,0.f,1.f);
            const float requested=cfg.sideSpeed+(runSpeed-cfg.sideSpeed)*mode;
            const float travel=cornerSpeedLimit(cornerRoute,requested)*input.x*cornerRoute.sideSign*dt;
            const float vertical=input.y>0?cfg.climbSpeed:cfg.downSpeed;
            const float rise=input.y*(vertical+(runSpeed-vertical)*mode)*dt;
            auto clear=[&](Vec a,Vec b){return clearPath(w,a,b);};

            CornerRoute trial=cornerRoute,verticalRoute=cornerRoute;
            std::optional<CornerStep> verticalStep;
            if(std::abs(rise)>.00001f) {
                verticalStep=shiftCornerRoute(w,cfg,trial,rise,clear);
                if(verticalStep)verticalRoute=trial;
            }

            const bool verticalFallback=std::abs(rise)>.00001f&&!verticalStep&&std::abs(input.x)<.1f;
            const bool checkedVertical=verticalStep&&travel==0.f&&std::isfinite(trial.distance)&&
                trial.distance>.001f&&trial.distance<trial.length-.001f;
            auto step=verticalFallback?std::optional<CornerStep>{}:checkedVertical?verticalStep:advanceCornerRoute(w,cfg,trial,travel,clear);
            bool shifted=verticalStep.has_value();
            bool lateral=step&&std::abs(trial.distance-cornerRoute.distance)>.00001f;
            if(step&&(!clearPath(w,position,step->position,false,horizontal(step->normal))||
                (shifted&&lateral&&!cornerBodyClear(w,cfg,position,step->position,inclinedCorner,true)))) {

                if(std::abs(input.y)>std::abs(input.x)) {
                    trial=verticalRoute;step=verticalStep;lateral=false;
                } else {
                    trial=cornerRoute;step=advanceCornerRoute(w,cfg,trial,travel,clear);shifted=false;
                    lateral=step&&std::abs(trial.distance-cornerRoute.distance)>.00001f;
                }
                if(step&&!clearPath(w,position,step->position,false,horizontal(step->normal)))step.reset();
            }
            if(!step&&verticalStep&&!verticalFallback&&clearPath(w,position,verticalStep->position,false,horizontal(verticalStep->normal))) {
                trial=verticalRoute;step=verticalStep;shifted=true;lateral=false;
            }
            if(step) {
                cornerRoute=trial;
                position=step->position;surfaceNormal=step->normal;normal=horizontal(surfaceNormal);
                state=State::wall;stalled=missingSurface=0;
                if(running&&!runwayClear(w,position)){running=false;runBlend=0;runClearanceCooldown=.45f;}
                moveDirection={lateral?input.x:0.f,shifted?input.y:0.f,0};
                const bool moving=moveDirection.length()>.00001f;
                Motion requestedMotion=std::abs(moveDirection.y)>std::abs(moveDirection.x)?
                    (moveDirection.y>0?Motion::up:Motion::down):(moveDirection.x<0?Motion::left:Motion::right);
                if(running&&moving)requestedMotion=std::abs(moveDirection.x)<.1f?Motion::runUp:
                    std::abs(moveDirection.y)<.1f?(moveDirection.x<0?Motion::runLeft:Motion::runRight):
                    (moveDirection.x<0?Motion::runDiagonalLeft:Motion::runDiagonalRight);
                out.motion=stableMotion=moving?requestedMotion:Motion::hang;
                out.staminaCost=moving?cfg.drain*(running?2:1)*dt:cfg.hangDrain*dt;
                blockedReason="checked lateral corner";
                if(lateral&&((travel>0&&step->complete)||(travel<0&&step->reversed))) {
                    cornerActive=false;

                    const float tail=cornerRoute.count>1?(cornerRoute.points[cornerRoute.count-1]-cornerRoute.points[cornerRoute.count-2]).length():18.f;
                    cornerProbeCooldown=tail<17.9f?0.f:.18f;clearanceMargin=0;
                }
                return out;
            }
            if(!verticalFallback&&cornerJoinValid(w,cfg,cornerRoute)&&
                cornerGrip(w,cfg,cornerRoute,position,surfaceNormal)&&clearPath(w,position,position)&&cornerBodyClear(w,cfg,position,position,inclinedCorner)) {

                state=State::wall;moveDirection={};out.motion=stableMotion=Motion::hang;
                stalled+=dt;blockedReason="corner route blocked";return out;
            }
            cornerActive=false;cornerProbeCooldown=.25f;
            if(!support(w,position,normal*-1)) {
                return holdGeometry("corner support changed");
            }
        }

        if(in.mantle) {
            for(float advance:{0.f,12.f,24.f}) {

                if(advance>0&&surfaceNormal.z<=.68f)continue;
                const auto ground=standingFloor(w,position-normal*advance,38,12);
                if(!ground)continue;

                bool wallAbove=false;

                for(float height:{6.f,32.f,cfg.chest,cfg.grip}) {
                    const auto from=position+Vec{0,0,height};
                    const auto hit=w.ray(from,from-normal*(cfg.gap+24));
                    if(hit&&hit->climbable&&hit->normal.z<.70f&&horizontal(hit->normal).dot(normal)>.35f){wallAbove=true;break;}
                }
                if(!wallAbove&&clearPath(w,position,*ground,true)) {
                    const auto delta=*ground-position;const float step=cfg.climbSpeed*dt;
                    if(delta.length()>step) {

                        position=position+delta.unit()*step;running=false;runBlend=0;
                        stalled=missingSurface=0;out.motion=stableMotion=Motion::up;
                        out.staminaCost=cfg.drain*dt;blockedReason="settling onto supported summit";return out;
                    }
                    position=*ground;stop();out.released=out.completed=true;
                    out.reason=surfaceNormal.z>.68f?"walkable summit reached":"standing surface reached";return out;
                }
            }
        }
        bool standingTopPath=false;
        std::optional<Ledge> ledge;
        const Vec searchInput{in.x,in.y,0};
        const unsigned searchMode=(in.mantle?1u:0u)|(wallRunControls?2u:0u);
        if(topSearchRetry.ready(position,surfaceNormal,searchInput,searchMode)) {
            ledge=findLedge(w,&standingTopPath);
            if(ledge)topSearchRetry.reset();
            else topSearchRetry.defer(position,surfaceNormal,searchInput,searchMode,.12f,standingTopPath);
        } else {
            standingTopPath=topSearchRetry.standingPath;
            ledgeReason="top search retry pending";
        }
        if(!ledge&&in.mantle&&in.y>0&&std::abs(in.x)<.6f&&roofProbeCooldown<=0&&
            surfaceNormal.z>=.30f&&(stalled>.12f||crestOpportunity(w))) {
            roofProbeCooldown=.25f;
            ledge=findCrestLedge(w);
        }
        state=ledge ? State::ledge : State::wall;
        if(in.mantle && ledge && stamina>=cfg.mantleCost) {
            mantleFrom=position; mantleApex={position.x,position.y,std::max(position.z,
                ledge->frontEdge?std::max(ledge->stand.z+4,ledge->lip.z+8):ledge->stand.z+4)};
            mantleTo=ledge->stand; mantleLip=ledge->lip; mantleTime=0;

            roundedMantle=(!cfg.authoredMantle||!authoredPath(Motion::contextMantle))&&roundedMantleClear(w,mantleFrom,mantleApex,mantleTo);
            if(cfg.authoredMantle&&!selectAuthoredMantlePath(w)) {
                topSearchRetry.defer(position,surfaceNormal,searchInput,searchMode,.12f,standingTopPath);
                ledgeReason=authoredQueryBudgetExhausted?"authored mantle query budget exhausted":"authored mantle path blocked";
                ledge.reset();state=State::wall;
            } else {
                mantleHands=ledge->hands;mantleHandNormals=ledge->normals;mantleCrest=ledge->crest;
                ledgePull=(mantleLip.z-position.z)>=110;
                threepeatMantle=!cfg.authoredMantle&&selectThreepeatMantle(w);
                mantleTime=cfg.authoredMantle||lowTopStep()?0.f:-.60f/topSeconds();
                state=State::mantle; out.motion=topMotion(); out.staminaCost=cfg.mantleCost;
                return out;
            }
        }
        auto current=support(w,position,normal*-1,ledge.has_value());
        bool directSupport=current.has_value();
        if(!current) {
            missingSurface=std::min(2.f,missingSurface+dt);blockedReason="surface temporarily missing";

            for(float z:{18.f,-18.f,-36.f}) {
                current=support(w,position+Vec{0,0,z},normal*-1);
                if(current)break;
            }
            if(!current) {
                if(auto recovery=trySupportRecovery(w,in,stamina))return *recovery;
                return holdGeometry("wall support unavailable; holding position");
            }
        }
        if(directSupport)missingSurface=0;

        in.x=std::clamp(in.x,-1.0f,1.0f); in.y=std::clamp(in.y,-1.0f,1.0f);
        const float magnitude=std::sqrt(in.x*in.x+in.y*in.y);
        if(magnitude>1) { in.x/=magnitude; in.y/=magnitude; }
        const float hopCost=15.f;

        const bool canPlanEdge=cfg.contextActions&&!in.run&&!wallRunControls&&!running&&!runMotion(stableMotion)&&
            actionCooldown<=0&&std::abs(surfaceNormal.z)<.1f;
        if(!manualHopTried&&!wallRunControls&&!running&&in.hop&&actionCooldown<=0&&stamina>=hopCost)
            if(auto hop=tryManualHop(w,in,canPlanEdge))return *hop;
        if(magnitude<.1f) {

            stalled=0;detourProbe=0;moveDirection={};out.motion=stationaryMotion();
            stableMotion=out.motion;out.staminaCost=cfg.hangDrain*dt;return out;
        }

        if(ledge && in.y>0 && !in.mantle) in.y=0;
        const Vec plane=current->normal.unit(),planeHorizontal=horizontal(plane);
        const Vec right{-planeHorizontal.y,planeHorizontal.x,0};
        const Vec tangent=(Vec{0,0,1}-plane*plane.z).unit();
        const float mode=in.y<0?0.f:in.modeBlend<0?runBlend*runBlend*(3-2*runBlend):std::clamp(in.modeBlend,0.f,1.f);
        const float vertical=in.y>0?cfg.climbSpeed:cfg.downSpeed;


        if(cfg.wallRunObstacleJumps&&running&&runBlend>.85f&&runMotion(stableMotion)&&
            actionCooldown<=0&&obstacleProbeCooldown<=0&&stamina>=30.f&&directSupport) {
            obstacleProbeCooldown=.10f;
            const Vec heading=(right*in.x+tangent*in.y).unit();
            if(auto jump=tryWallRunObstacle(w,heading,runSpeed,in,stamina))return *jump;
        }
        if(directSupport&&in.y>=0&&in.y<.1f&&std::abs(in.x)>=.6f&&
            actionCooldown<=0&&roofProbeCooldown<=0&&(!wallRunControls||cfg.wallRunObstacleJumps)&&
            stamina>=(wallRunControls?30.f:hopCost)&&eaveObstacleAhead(w,in)) {
            roofProbeCooldown=.25f;
            if(tryEaveTransfer(w,in,wallRunControls,runSpeed,stamina)) {
                out.motion=actionMotion;out.staminaCost=wallRunControls?30.f:hopCost;
                out.reason=blockedReason=wallRunControls?"checked wall-run obstacle bypass":"checked eave bypass";
                return out;
            }
        }
        Vec next=position+right*(in.x*(cfg.sideSpeed+(runSpeed-cfg.sideSpeed)*mode)*dt)+
            tangent*(in.y*(vertical+(runSpeed-vertical)*mode)*dt);
        bool moving=false,accepted=false,runwayRejected=false,hasDestinationSupport=false;

        blockedHit.reset();
        blockedReason="destination surface missing";

        for(float fraction:{1.f,.5f,.25f}) {
            if(accepted)break;
            const Vec candidate=position+(next-position)*fraction;
            if(auto h=support(w,candidate,planeHorizontal*-1)) {
                hasDestinationSupport=true;
                const auto newNormal=horizontal(h->normal);
                if(newNormal.dot(normal)>-.05f) {
                    const float turn=std::acos(std::clamp(normal.dot(newNormal),-1.f,1.f));
                    const float amount=turn>12*dt?12*dt/turn:1.f;
                    const auto nextNormal=(normal*(1-amount)+newNormal*amount).unit();
                    auto accept=[&](Vec target,float margin) {
                        if(!clearPath(w,position,target,false,nextNormal)||
                            ((newNormal-nextNormal).length()>.00001f&&!clearPath(w,target,target,false,newNormal,nextNormal,true,true)))return false;
                        if(running&&!runwayClear(w,target)) {
                            runwayRejected=true;runClearanceCooldown=.45f;return false;
                        }
                        moving=(target-position).dot(next-position)>.00001f;
                        position=target;missingSurface=0;clearanceMargin=margin;accepted=true;
                        normal=nextNormal;
                        surfaceNormal=(surfaceNormal*(1-std::min(1.f,dt*8))+h->normal.unit()*std::min(1.f,dt*8)).unit();
                        return true;
                    };
                    std::optional<Vec> checkedPoint;
                    bool checkedSupport{};
                    for(float outward:{std::max(0.f,clearanceMargin-dt*2),4.f,9.f,14.f}) {
                        Vec corrected=offsetFromSurface(candidate,*h)+newNormal*outward;

                        const Vec correction=corrected-candidate;
                        const float limit=std::max(2.f,dt*(running?std::max(150.f,cfg.runSpeed):150.f));
                        if(correction.length()>limit)corrected=candidate+correction.unit()*limit;

                        const bool needsSupport=correction.length()>limit||outward>0;
                        if(checkedPoint&&checkedSupport==needsSupport&&checkedPoint->x==corrected.x&&
                            checkedPoint->y==corrected.y&&checkedPoint->z==corrected.z)continue;
                        checkedPoint=corrected;checkedSupport=needsSupport;
                        if(needsSupport&&!support(w,corrected,newNormal*-1))continue;
                        blockedReason="body clearance";
                        if(accept(corrected,outward))break;
                    }
                    if(!accepted&&!runwayRejected) {
                        const float before=(position-offsetFromSurface(position,*h)).dot(newNormal);
                        const float after=(candidate-offsetFromSurface(candidate,*h)).dot(newNormal);
                        if(before>.05f&&after>=-.01f&&after<=before+.01f)
                            accept(candidate,std::max(0.f,after));
                    }
                } else blockedReason="corner exceeds turn limit";
            }
        }
        if(runwayRejected&&!accepted) {
            running=false;runBlend=0;blockedReason="wall-run clearance; switching to climb";
        }

        if(!accepted&&!runwayRejected&&!hasDestinationSupport&&standingTopPath&&in.y>0&&clearanceMargin>0) {
            const float recovery=std::min(clearanceMargin,std::max(0.f,cfg.climbSpeed*dt));
            const Vec candidate=position-normal*recovery;
            if(recovery>0)if(auto h=support(w,candidate,normal*-1)) {
                const auto facing=horizontal(h->normal);

                if((offsetFromSurface(candidate,*h)-candidate).dot(facing)<=.05f&&
                    clearPath(w,position,candidate)) {
                    position=candidate;clearanceMargin=std::max(0.f,clearanceMargin-recovery);
                    missingSurface=0;accepted=true;moving=true;
                }
            }
        }

        if(!accepted&&!runwayRejected&&!(standingTopPath&&in.y>0)&&magnitude>.1f&&clearanceMargin<14) {
            const Vec escape=position+normal*std::min(3.f,14-clearanceMargin);
            if(support(w,escape,normal*-1)&&clearPath(w,position,escape)) {
                position=escape;clearanceMargin+=std::min(3.f,14-clearanceMargin);
            }
        }
        stalled=magnitude>.1f&&!moving?stalled+dt:0;
        if(!accepted&&!hasDestinationSupport&&directSupport&&stalled>.18f&&actionCooldown<=0&&roofProbeCooldown<=0&&
            in.y>.5f&&std::abs(in.x)<.5f&&(!wallRunControls||cfg.wallRunObstacleJumps)&&stamina>=(wallRunControls?30.f:hopCost)) {
            if(tryRecessedWallTransfer(w,in,wallRunControls,runSpeed,stamina)) {
                roofProbeCooldown=.25f;
                out.motion=actionMotion;out.staminaCost=wallRunControls?30.f:hopCost;out.reason=blockedReason;return out;
            }
        }


        if(wallRunControls&&cfg.wallRunObstacleJumps&&!accepted&&stalled>.18f&&
            actionCooldown<=0&&roofProbeCooldown<=0&&stamina>=30.f&&in.y>=0&&magnitude>.1f) {
            roofProbeCooldown=.25f;
            if(tryEaveTransfer(w,in,true,runSpeed,stamina)) {
                out.motion=actionMotion;out.staminaCost=30.f;
                out.reason=blockedReason="checked wall-run obstacle bypass";return out;
            }
        }
        if(!wallRunControls&&!running&&!accepted&&!runwayRejected&&stalled>.12f&&actionCooldown<=0&&
            stamina>=hopCost&&magnitude>.1f&&in.y>=0&&hopSearchRetry.ready(position,surfaceNormal,searchInput,searchMode)) {
            constexpr float rises[]={52.f,96.f,144.f,176.f},clearances[]={18.f,30.f,44.f};
            struct Prefix {bool checked{},clear{};std::optional<Hit> hit;Vec from{},to{};};
            std::array<Prefix,3> prefixes{};
            const bool sideways=std::abs(in.x)>.6f;
            const auto recoveryMotion=sideways?(in.x<0?Motion::hopLeft:Motion::hopRight):Motion::hopUp;
            AuthoredQueryWorld bounded(w);
            World& recoveryWorld=authored(recoveryMotion)?static_cast<World&>(bounded):w;
            for(unsigned candidate=0;candidate<(sideways?1u:4u);++candidate) {
                if(bounded.exhausted)break;
                const float rise=rises[candidate];
                const Vec advance=sideways?right*(in.x<0?-48.f:48.f):Vec{0,0,rise};
                const auto wanted=position+advance;
                const auto destination=support(recoveryWorld,wanted,normal*-1,false,true);
                if(!destination)continue;
                const auto anchor=landingAnchor(recoveryWorld,wanted,*destination);
                if(!anchor||horizontal(anchor->hit.normal).dot(normal)<=.6f)continue;
                const auto target=anchor->position;
                for(std::size_t index=0;index<prefixes.size();++index) {
                    if(bounded.exhausted)break;
                    const float clearance=clearances[index];
                    const Vec a=position+normal*clearance,b=target+normal*clearance;
                    const auto motion=recoveryMotion;
                    if(authored(motion)) {
                        if(commitAuthoredRoute(recoveryWorld,motion,target,.55f+rise*.002f,horizontal(anchor->hit.normal),true,a,b)) {
                            out.motion=actionMotion;out.staminaCost=hopCost;blockedReason="checked obstacle hop";return out;
                        }
                        continue;
                    }
                    auto& prefix=prefixes[index];
                    if(!prefix.checked) {
                        prefix.clear=clearPath(w,position,a);prefix.checked=true;
                        if(!prefix.clear){prefix.hit=blockedHit;prefix.from=blockedFrom;prefix.to=blockedTo;}
                    } else if(!prefix.clear) {
                        blockedHit=prefix.hit;blockedFrom=prefix.from;blockedTo=prefix.to;
                    }
                    if(prefix.clear&&clearPath(w,a,b)&&clearPath(w,b,target,false,horizontal(anchor->hit.normal))) {
                        beginAction(motion,position,target,.55f+rise*.002f);
                        actionLandingNormal=horizontal(anchor->hit.normal);
                        detour=true;detourOut=a;detourOver=b;out.motion=actionMotion;out.staminaCost=hopCost;
                        blockedReason="checked obstacle hop";return out;
                    }
                }
            }
            authoredQueryBudgetExhausted|=bounded.exhausted;
            if(bounded.exhausted)blockedReason="authored obstacle query budget exhausted";
            hopSearchRetry.defer(position,surfaceNormal,searchInput,searchMode,.20f);
        }
        if(!wallRunControls&&!running&&!accepted&&!runwayRejected&&in.y>=0&&magnitude>.1f&&
            stalled>.18f&&actionCooldown<=0&&roofProbeCooldown<=0&&stamina>=hopCost) {
            roofProbeCooldown=.25f;
            if(in.y>0&&std::abs(in.x)<.5f&&tryRoofTransfer(w)) {
                out.motion=actionMotion;out.staminaCost=hopCost;blockedReason="checked steep roof transfer";return out;
            }
            if(tryEaveTransfer(w,in,false,0,stamina)) {
                out.motion=actionMotion;out.staminaCost=hopCost;blockedReason="checked eave bypass";return out;
            }
        }
        if(!accepted&&!runwayRejected&&stalled>.35f&&(!directSupport||(ledge&&in.y<-.1f&&!gripSupport(w,position,*current,true)))&&(!wallRunControls||cfg.wallRunObstacleJumps))
            if(auto recovery=trySupportRecovery(w,in,stamina))return *recovery;
        if(moving){detourProbe=0;blockedHit.reset();blockedReason="none";supportRecoveryCursor=0;supportRecoveryInput={};}

        if(moving&&accepted&&automaticInputAllowed(in,wallRunControls)&&stamina>=hopCost+cfg.drain*dt) {
            if(automaticInterval<=0) {
                const float low=std::clamp(cfg.autoActionMinSeconds,.65f,20.f);
                const float high=std::clamp(cfg.autoActionMaxSeconds,low,30.f);
                automaticInterval=low+(high-low)*automaticRandom();
                automaticDirection=Vec{in.x,in.y,0}.unit();
            }
            automaticElapsed+=dt;
            automaticBlocked=0;
            automaticRetry=std::max(0.f,automaticRetry-dt);
            automaticOpportunityRetry=std::max(0.f,automaticOpportunityRetry-dt);

            if(cfg.surfaceActionVariants&&canPlanEdge&&capturedSideIntent({in.x,in.y,0})&&
                automaticElapsed>=.35f&&automaticOpportunityRetry<=0) {
                ++automaticOpportunities;automaticOpportunityRetry=.08f;
                if(acceptAutomaticSide(in))if(auto captured=tryAutomaticCaptured(w,in,false)) {
                    captured->staminaCost+=cfg.drain*dt;return *captured;
                }
            }
            if(automaticElapsed+1.e-6f>=automaticInterval&&automaticRetry<=0) {
                ++automaticAttempts;
                automaticRetry=.30f;
                if(cfg.surfaceActionVariants&&canPlanEdge&&acceptAutomaticSide(in))if(auto captured=tryAutomaticCaptured(w,in,true)) {
                    captured->staminaCost+=cfg.drain*dt;return *captured;
                }

                if(cfg.legacyAutomaticHops)if(auto varied=in.y<0?std::optional<Result>{}:tryManualHop(w,in,false)) {
                    if(edgePreparation.active)automaticPreparation=true;
                    else if(state==State::action)++automaticActions;
                    varied->staminaCost+=cfg.drain*dt;
                    varied->reason="automatic checked climbing action";
                    blockedReason=threepeatHop(actionMotion)?"automatic measured edge hop":"automatic checked climb hop";
                    return *varied;
                }
                automaticElapsed=std::min(automaticElapsed,automaticInterval);
            }
        } else if(automaticInputAllowed(in,wallRunControls)&&directSupport&&stamina>=hopCost+cfg.drain*dt) {
            automaticBlocked+=dt;
            if(automaticBlocked>.65f)resetAutomaticClock();
        } else resetAutomaticClock();
        Motion requested=std::abs(in.y)>std::abs(in.x) ?
            (in.y>0?Motion::up:Motion::down) : (in.x<0?Motion::left:Motion::right);
        if(running) requested=std::abs(in.x)<.1f?Motion::runUp:
            std::abs(in.y)<.1f?(in.x<0?Motion::runLeft:Motion::runRight):
            (in.x<0?Motion::runDiagonalLeft:Motion::runDiagonalRight);
        const bool briefBlock=magnitude>.1f&&stalled<.18f&&stableMotion==requested;
        out.motion=moving?requested:briefBlock?stableMotion:Motion::hang;
        stableMotion=out.motion;
        out.staminaCost=moving?cfg.drain*(running?2:1)*dt:0;
        if(in.y<0) {
            auto ground=w.ray(position+Vec{0,0,5},position-Vec{0,0,6});
            if(ground && ground->normal.z>0.7f) { stop(); out.released=true;out.reason="descending reached ground"; }
        }
        return out;
    }

private:
    Motion heldMotion() const {
        return state==State::approach?entryMotion:state==State::mantle?topMotion():
            state==State::action?actionMotion:Motion::hang;
    }
    Result heldResult(const char* reason) {
        Result out;out.motion=heldMotion();out.reason=blockedReason=reason;return out;
    }
    Result holdGeometry(const char* reason) {
        geometryHeld=true;geometryRetry=.15f;moveDirection={};resetAutomaticClock();
        if(state==State::wall||state==State::ledge) {
            running=false;runBlend=diagonalRunBlend=0;stableMotion=Motion::hang;
            cornerActive=edgeAction=edgeSettled=edgePreparation.active=automaticPreparation=false;
        }
        return heldResult(reason);
    }
    Vec roofFacing(float phase) const {
        const Vec from=horizontal(actionStartSurface),to=horizontal(actionTargetSurface);
        const float angle=std::atan2(from.x*to.y-from.y*to.x,from.dot(to))*ease((phase-.25f)/.65f);
        return {from.x*std::cos(angle)-from.y*std::sin(angle),from.x*std::sin(angle)+from.y*std::cos(angle),0};
    }
    Vec mantleFrom{},mantleApex{},mantleTo{},mantleLip{},approachFrom{},approachTo{};
    std::array<Vec,2> mantleHands{},mantleHandNormals{};
    float mantleTime{},approachTime{},approachLift{},entrySeconds=.32f;
    bool approachRounded{},entryUsedTopFallback{};float entryTopRise{};
    bool ledgePull{},roundedMantle{},mantleCrest{},threepeatMantle{};
    AuthoredTrajectory mantleRoute;
    Vec actionFrom{},actionTo{};
    float actionTime{},actionSeconds=1,actionCooldown{},stalled{},clearanceMargin{},runBlend{},diagonalRunBlend{},hopDistance{};
    Motion actionMotion=Motion::none,entryMotion=Motion::reach;
    Motion actionWallRunDirection=Motion::none;
    Motion stableMotion=Motion::hang;
    bool running{},jumpEntry{},detour{},actionBeganRunning{};
    Vec actionDirection{},actionLandingNormal{};
    float missingSurface{},runClearanceCooldown{},geometryRetry{};
    bool geometryHeld{},staminaResting{};
    float supportRecoveryRetry{};
    Vec supportRecoveryOrigin{},supportRecoveryInput{};
    unsigned supportRecoveryCursor{};
    bool supportRecoveryAction{};
    Vec moveDirection{},detourOut{},detourOver{};
    unsigned detourProbe{};
    bool roofTransfer{};
    Vec actionStartSurface{},actionTargetSurface{};
    float roofProbeCooldown{};
    SearchRetry<Vec> topSearchRetry,hopSearchRetry;
    bool edgeAction{},edgeSettled{};
    GripEdge actionSourceEdge{},actionTargetEdge{};
    float edgeProbeCooldown{};
    EdgePreparation edgePreparation{};
    unsigned threepeatPlanStatus{},mantleSelectionStatus{};
    CornerRoute cornerRoute{};
    bool cornerActive{};
    float cornerProbeCooldown{};

    std::uint32_t automaticRandomState=0x6D2B79F5u;
    unsigned automaticActions{},automaticAttempts{},automaticOpportunities{},surfaceActions{},contextIdles{};
    float automaticElapsed{},automaticInterval{},automaticBlocked{},automaticRetry{},automaticOpportunityRetry{};
    Vec automaticDirection{};
    bool automaticPreparation{};
    bool obstacleJump{};
    bool authoredActionChecked{},authoredActionRejected{},authoredBackFlipPrechecked{},entryPose{};
    unsigned obstacleJumps{};
    float obstacleProbeCooldown{},actionRunSpeed{};
    void resetAutomaticClock() {automaticElapsed=automaticInterval=automaticBlocked=automaticRetry=automaticOpportunityRetry=0;automaticDirection={};}
    float automaticRandom() {
        automaticRandomState^=automaticRandomState<<13;
        automaticRandomState^=automaticRandomState>>17;
        automaticRandomState^=automaticRandomState<<5;
        return float(automaticRandomState>>8)*(1.f/16777216.f);
    }
    bool acceptAutomaticSide(Input input) {
        if(!capturedSideIntent({input.x,input.y,0}))return false;
        const float weight=cfg.automaticSideWeights[input.x<0?0:1];
        if(!std::isfinite(weight)||weight<=0)return false;
        return weight>=1||automaticRandom()<weight;
    }
    bool automaticInputAllowed(Input input,bool wallRunControls) const {
        return cfg.automaticClimbActions&&!input.run&&!running&&!wallRunControls&&runBlend<=.05f&&
            !runMotion(stableMotion)&&(state==State::wall||state==State::ledge)&&!cornerActive&&
            !edgePreparation.active&&!input.hop&&!input.release&&!input.backDrop&&
            input.y>=0&&std::hypot(input.x,input.y)>.1f&&actionCooldown<=0;
    }
    void beginAction(Motion motion,Vec from,Vec to,float seconds,Motion wallRunDirection=Motion::none) {
        supportRecoveryAction=false;
        topSearchRetry.reset();hopSearchRetry.reset();
        resetAutomaticClock();automaticPreparation=false;
        actionBeganRunning=running||runMotion(stableMotion);
        obstacleJump=false;actionRunSpeed=cfg.runSpeed;
        actionDirection=moveDirection;
        if(actionDirection.length()<.1f)actionDirection={0,1,0};
        actionLandingNormal=normal;
        authoredActionChecked=authoredActionRejected=false;entryPose=false;
        const int directionIndex=wallRunDirectionIndex(wallRunDirection);
        actionWallRunDirection=directionIndex>=0&&cfg.authoredWallRunSequences&&cfg.authoredWallRunSequences->valid[directionIndex]&&
            (wallRunLaunch(motion)||motion==Motion::runCatch||motion==Motion::sideBrace)?wallRunDirection:Motion::none;
        actionMotion=motion;actionFrom=from;actionTo=to;actionSeconds=motionDuration(motion,seconds);actionTime=0;state=State::action;detour=false;roofTransfer=false;edgeAction=edgeSettled=false;
        cornerActive=false;edgePreparation.active=false;
    }
    bool commitAuthoredRoute(World& w,Motion motion,Vec target,float seconds,Vec landing={},bool routed=false,Vec outside={},Vec over={},bool roof=false,Vec targetSurface={},float outward=0,float sourceSpeed=0) {
        auto candidate=*this;candidate.beginAction(motion,position,target,seconds);
        candidate.actionLandingNormal=landing.length()>.9f?landing:normal;
        candidate.detour=routed;candidate.detourOut=outside;candidate.detourOver=over;candidate.roofTransfer=roof;
        candidate.actionStartSurface=surfaceNormal;candidate.actionTargetSurface=targetSurface.length()>.9f?targetSurface:surfaceNormal;
        candidate.hopDistance=outward;
        if(sourceSpeed>0){candidate.actionRunSpeed=sourceSpeed;candidate.actionBeganRunning=true;}
        if(!candidate.authoredActionClear(w)){authoredQueryBudgetExhausted|=candidate.authoredQueryBudgetExhausted;return false;}
        candidate.authoredActionChecked=true;*this=std::move(candidate);return true;
    }

    std::optional<Result> trySupportRecovery(World& world,Input input,float stamina) {
        if(!std::isfinite(input.x)||!std::isfinite(input.y)||std::hypot(input.x,input.y)<=.1f)return {};
        const Vec intent=Vec{std::clamp(input.x,-1.f,1.f),std::clamp(input.y,-1.f,1.f),0}.unit();
        if(intent.length()<.5f||input.release||actionCooldown>0||stamina<15.f||
            (state!=State::wall&&state!=State::ledge))return {};
        if(supportRecoveryInput.length()<.5f||supportRecoveryInput.dot(intent)<.7f||
            (position-supportRecoveryOrigin).length()>4) {
            supportRecoveryInput=intent;supportRecoveryOrigin=position;
            supportRecoveryCursor=0;supportRecoveryRetry=0;
        }
        if(supportRecoveryRetry>0)return {};
        supportRecoveryRetry=.4f;
        const Vec facing=horizontal(surfaceNormal),right{-facing.y,facing.x,0};
        const Vec vertical=(Vec{0,0,1}-surfaceNormal*surfaceNormal.z).unit();
        const Vec direction=(right*intent.x+vertical*intent.y).unit();
        const Vec cross=(right*-intent.y+vertical*intent.x).unit();
        const bool descending=intent.y<-.5f&&std::abs(intent.x)<.5f;
        const Motion motion=descending?Motion::down:std::abs(intent.x)>.5f?
            (intent.x<0?Motion::hopLeft:Motion::hopRight):Motion::hopUp;
        const unsigned count=descending?6u:12u;
        constexpr std::array<float,4> distances{24.f,48.f,80.f,112.f};
        constexpr std::array<float,3> deviations{0.f,16.f,-16.f};
        constexpr std::array<float,5> angles{0.f,-.7f,.7f,-1.3f,1.3f};
        AuthoredQueryWorld bounded(world,4096);
        for(unsigned attempt=0;attempt<4&&!bounded.exhausted;++attempt) {
            const unsigned index=supportRecoveryCursor%count;
            supportRecoveryCursor=(index+1)%count;
            const Vec probe=position+direction*distances[index/3]+cross*deviations[index%3];
            std::optional<Anchor> anchor;
            for(float angle:angles) {
                const Vec forward{-(facing.x*std::cos(angle)-facing.y*std::sin(angle)),
                    -(facing.x*std::sin(angle)+facing.y*std::cos(angle)),0};
                for(float height:{cfg.chest,cfg.grip,32.f}) {
                    const Vec center=probe+Vec{0,0,height};
                    const auto hit=bounded.ray(center-forward*16.f,center+forward*cfg.reach);
                    if(!hit||!hit->climbable||!hit->point.finite()||!hit->normal.finite()||
                        hit->normal.length()<.5f)continue;
                    const auto n=hit->normal.unit();
                    if(n.z<-.45f||n.z>std::max(.94f,cfg.maxNormalZ)||horizontal(n).dot(normal)<-.05f)continue;
                    const auto next=landingAnchor(bounded,probe,*hit);
                    if(!next)continue;
                    const Vec delta=next->position-position;
                    if(delta.length()>144.f||delta.dot(direction)<8.f||
                        delta.dot(direction)<delta.length()*.4f||
                        (next->position-probe).length()>cfg.reach||
                        (descending&&(delta.z>-.5f||delta.length()>64.f)))continue;
                    if(!recoveryLanding(bounded,next->position,next->hit.normal))continue;
                    anchor=next;break;
                }
                if(anchor||bounded.exhausted)break;
            }
            if(!anchor)continue;
            const Vec landing=horizontal(anchor->hit.normal);
            for(float outward:{0.f,12.f,24.f}) {
                auto candidate=*this;
                candidate.beginAction(motion,position,anchor->position,
                    std::clamp((anchor->position-position).length()/160.f,.28f,.80f));
                candidate.supportRecoveryAction=true;candidate.geometryHeld=false;candidate.geometryRetry=0;
                candidate.running=false;candidate.runBlend=candidate.diagonalRunBlend=0;
                candidate.actionLandingNormal=landing;candidate.actionStartSurface=surfaceNormal;
                candidate.actionTargetSurface=anchor->hit.normal.unit();candidate.actionDirection=intent;
                candidate.detour=true;candidate.detourOut=position+normal*outward;
                candidate.detourOver=anchor->position+landing*outward;
                if(!candidate.authoredActionClear(bounded)) {
                    if(bounded.exhausted)break;
                    continue;
                }
                if(bounded.exhausted)break;
                candidate.authoredActionChecked=true;candidate.stalled=candidate.missingSurface=0;
                candidate.supportRecoveryCursor=0;candidate.supportRecoveryRetry=.4f;
                candidate.blockedReason="checked nearby support recovery";
                *this=std::move(candidate);
                Result result;result.motion=actionMotion;result.staminaCost=15.f;
                result.reason=blockedReason;return result;
            }
        }
        if(bounded.exhausted)blockedReason="support recovery query budget exhausted";
        return {};
    }

    std::optional<Result> retrySupportRecovery(World& world,Input input,float stamina) {
        if(!supportRecoveryAction||!std::isfinite(input.x)||!std::isfinite(input.y)||
            std::hypot(input.x,input.y)<=.1f||input.release||stamina<15.f)return {};
        const Vec intent=Vec{std::clamp(input.x,-1.f,1.f),std::clamp(input.y,-1.f,1.f),0}.unit();
        if(supportRecoveryRetry>0&&(position-supportRecoveryOrigin).length()<=4&&
            supportRecoveryInput.dot(intent)>=.7f)return {};
        AuthoredQueryWorld bounded(world,4096);
        if(const auto current=recoveryLanding(bounded,position,surfaceNormal)) {
            const auto facing=horizontal(current->normal);
            if(facing.dot(normal)>=.95f&&clearPath(bounded,position,position,false,facing)&&!bounded.exhausted) {
                state=State::wall;supportRecoveryAction=geometryHeld=false;geometryRetry=0;
                edgeAction=edgeSettled=cornerActive=detour=roofTransfer=false;
                edgePreparation.active=automaticPreparation=false;
                running=false;runBlend=diagonalRunBlend=0;moveDirection={};stableMotion=Motion::hang;
                normal=facing;surfaceNormal=current->normal.unit();stalled=missingSurface=0;
                actionCooldown=.18f;supportRecoveryRetry=.4f;supportRecoveryCursor=0;
                supportRecoveryOrigin=position;supportRecoveryInput=intent;
                blockedReason="recovery returned to verified current support";
                Result result;result.motion=Motion::hang;result.reason=blockedReason;return result;
            }
        }
        auto candidate=*this;candidate.state=State::wall;candidate.geometryHeld=false;
        candidate.supportRecoveryAction=false;candidate.actionCooldown=0;
        const auto result=candidate.trySupportRecovery(bounded,input,stamina);
        if(result&&!bounded.exhausted) {*this=std::move(candidate);return result;}
        supportRecoveryRetry=candidate.supportRecoveryRetry;supportRecoveryCursor=candidate.supportRecoveryCursor;
        supportRecoveryOrigin=candidate.supportRecoveryOrigin;supportRecoveryInput=candidate.supportRecoveryInput;
        return {};
    }

    struct ProbeRayCache final:World {
        struct Entry {Vec from,to;std::optional<Hit> hit;bool valid{};};
        World& source;std::array<Entry,256> entries{};
        explicit ProbeRayCache(World& world):source(world){}
        std::optional<Hit> ray(Vec from,Vec to) override {
            std::uint32_t hash=2166136261u;
            for(float value:{from.x,from.y,from.z,to.x,to.y,to.z})hash=(hash^std::bit_cast<std::uint32_t>(value))*16777619u;
            hash^=hash>>16;hash*=0x7feb352du;hash^=hash>>15;
            auto& entry=entries[hash&(entries.size()-1)];
            auto equal=[](Vec a,Vec b){return a.x==b.x&&a.y==b.y&&a.z==b.z;};
            if(entry.valid&&equal(entry.from,from)&&equal(entry.to,to))return entry.hit;
            const auto result=source.ray(from,to);entry={from,to,result,true};return result;
        }
        bool actionBodyClear(Motion motion,Vec from,Vec to,float begin,float end,Vec normal) override {
            return source.actionBodyClear(motion,from,to,begin,end,normal);
        }
        bool actionBodyPathClear(Motion motion,Vec from,Vec to,float begin,float end,Vec normal,Vec middle) override {
            return source.actionBodyPathClear(motion,from,to,begin,end,normal,middle);
        }
    };

    std::optional<Result> tryWallRunObstacle(World& world,Vec heading,float speed,Input input,float stamina) {

        struct BudgetWorld final:World {
            World& source;unsigned count{},limit{};bool exhausted{};
            BudgetWorld(World& value,unsigned maximum):source(value),limit(maximum){}
            std::optional<Hit> ray(Vec a,Vec b)override {
                if(count>=limit){exhausted=true;return Hit{a,(a-b).unit(),false};}
                ++count;return source.ray(a,b);
            }
        } budget(world,authored(std::abs(input.x)<.1f?Motion::kickUp:input.x<0?Motion::kickLeft:Motion::kickRight)?12288:4096);
        ProbeRayCache w(budget);
        const float lookAhead=110.f+std::abs(input.x)*12.f;
        if(heading.length()<.9f||clearPath(w,position,position+heading*lookAhead)||!blockedHit)return {};
        const Hit obstacle=*blockedHit;
        const Motion selected=std::abs(input.x)<.1f?Motion::kickUp:input.x<0?Motion::kickLeft:Motion::kickRight;
        const float beyond=(obstacle.point-position).dot(heading)+cfg.radius+16;
        if(!kickClearance(w,position))return {};
        const Vec savedFrom=actionFrom,savedTo=actionTo;
        const float savedDistance=hopDistance,savedSeconds=actionSeconds,savedSpeed=actionRunSpeed;
        const bool savedRunning=actionBeganRunning;
        auto restore=[&] {
            actionFrom=savedFrom;actionTo=savedTo;hopDistance=savedDistance;actionSeconds=savedSeconds;
            actionRunSpeed=savedSpeed;actionBeganRunning=savedRunning;
        };
        for(float distance:{224.f,320.f,416.f}) {
            if(distance<=beyond)continue;
            const Vec wanted=position+heading*distance;
            const auto destination=support(w,wanted,normal*-1,false,true);
            if(!destination||budget.exhausted)continue;
            const auto anchor=landingAnchor(w,wanted,*destination);
            if(!anchor||horizontal(anchor->hit.normal).dot(normal)<.985f||
                std::abs(anchor->hit.normal.unit().z-surfaceNormal.z)>.10f)continue;
            const Vec target=anchor->position;

            if(input.y>.1f&&obstacle.normal.z<-.5f&&target.z<obstacle.point.z+cfg.radius+16)continue;
            if((target-wanted).length()>8||!clearPath(w,target,target,false,horizontal(anchor->hit.normal))||!runwayClear(w,target)||!kickClearance(w,target))continue;
            bool sourceChecked=false;
            for(float outward:{std::max(52.f,cfg.kickOut),78.f,104.f}) {
                const float seconds=wallRunObstacleDuration((target-position).length(),speed,outward);
                if(stamina<30.f+2*cfg.drain*(motionDuration(selected,seconds)+.05f))continue;
                if(authored(selected)) {
                    if(sourceChecked&&authoredPath(selected))break;
                    sourceChecked=true;
                    const bool valid=commitAuthoredRoute(w,selected,target,seconds,horizontal(anchor->hit.normal),false,{},{},false,{},outward,speed);
                    if(budget.exhausted){obstacleProbeCooldown=.25f;return {};}
                    if(!valid)continue;
                    actionRunSpeed=speed;actionBeganRunning=true;obstacleJump=true;++obstacleJumps;obstacleProbeCooldown=.5f;
                    blockedReason="automatic checked wall-run obstacle jump";
                    Result result;result.motion=selected;result.staminaCost=30.f;result.reason=blockedReason;return result;
                }
                actionFrom=position;actionTo=target;hopDistance=outward;
                actionBeganRunning=true;actionSeconds=seconds;actionRunSpeed=speed;
                Vec previous=position;bool valid=true;
                for(int knot=1;knot<=32;++knot) {
                    const Vec point=hopPoint(knot/32.f);
                    if(!clearPath(w,previous,point,false,knot==32?horizontal(anchor->hit.normal):normal)||budget.exhausted){valid=false;break;}
                    previous=point;
                }
                if(valid)for(int knot=1;knot<=32;++knot)
                    if(!kickClearance(w,hopPoint(knot/32.f))||budget.exhausted){valid=false;break;}
                restore();
                if(budget.exhausted){obstacleProbeCooldown=.25f;return {};}
                if(!valid)continue;
                const Motion motion=std::abs(input.x)<.1f?Motion::kickUp:
                    input.x<0?Motion::kickLeft:Motion::kickRight;
                beginAction(motion,position,target,seconds);
                hopDistance=outward;actionRunSpeed=speed;obstacleJump=true;++obstacleJumps;
                actionLandingNormal=horizontal(anchor->hit.normal);obstacleProbeCooldown=.5f;
                blockedReason="automatic checked wall-run obstacle jump";
                Result result;result.motion=motion;result.staminaCost=30.f;result.reason=blockedReason;return result;
            }
            if(budget.exhausted)break;
        }
        restore();return {};
    }
    std::optional<Result> tryManualHop(World& originalWorld,Input input,bool allowEdge) {
        input.x=std::clamp(input.x,-1.f,1.f);input.y=std::clamp(input.y,-1.f,1.f);
        const float magnitude=std::hypot(input.x,input.y);
        if(magnitude>1){input.x/=magnitude;input.y/=magnitude;}
        const Vec right{-normal.y,normal.x,0};
        const bool sideways=std::abs(input.x)>.1f;
        const Vec delta=sideways?right*(input.x<0?-90.f:90.f)+Vec{0,0,input.y>0?55.f:0.f}:Vec{0,0,70.f};
        const auto motion=sideways?(input.x<0?Motion::hopLeft:Motion::hopRight):Motion::hopUp;
        Result result;
        if(allowEdge&&prepareEdgeAction(originalWorld,motion,{input.x,input.y,0})) {
            cornerActive=false;
            Result value;value.motion=actionMotion;value.staminaCost=state==State::action?15.f:0;
            blockedReason="measured edge-to-edge hop";return value;
        }
        AuthoredQueryWorld w(originalWorld,authored(motion)?AuthoredQueryWorld::authoredLimit:4096);
        const Vec wanted=position+delta;
        const auto grab=support(w,wanted,normal*-1,false,true);
        const auto anchor=grab?landingAnchor(w,wanted,*grab):std::nullopt;
        if(w.exhausted||!anchor||!(horizontal(anchor->hit.normal).dot(normal)>.7f))return {};
        auto candidate=*this;
        candidate.beginAction(motion,position,anchor->position,jumpActionSeconds(false));
        candidate.actionLandingNormal=horizontal(anchor->hit.normal);candidate.hopDistance=cfg.hopOut;
        if(!candidate.authoredActionClear(w)||w.exhausted) {
            authoredQueryBudgetExhausted|=w.exhausted||candidate.authoredQueryBudgetExhausted;
            if(authoredQueryBudgetExhausted)blockedReason="authored hop query budget exhausted";
            return {};
        }
        candidate.authoredActionChecked=true;*this=std::move(candidate);
        result.motion=actionMotion;result.staminaCost=15.f;return result;
    }
#include "EdgeActions.h"
#include "EaveTraversal.h"
#include "RecessedWallTransfer.h"
    static const std::array<Vec,16>& bodyRingDirections() {
        static const auto directions=[] {
            std::array<Vec,16> values{};
            for(std::size_t index=0;index<values.size();++index) {
                const float angle=index*(6.28318530718f/16);
                values[index]={std::cos(angle),std::sin(angle),0};
            }
            return values;
        }();
        return directions;
    }
    bool roofPathClear(World& w,Vec from,Vec to) const {

        const float radius=cfg.radius+4;
        const bool moving=(to-from).length()>.001f;
        for(const auto direction:bodyRingDirections()) {
            const Vec offset=direction*radius;
            for(float height:{6.f,cfg.chest,cfg.height}) {
                if(moving) {
                    const Vec a=from+offset+Vec{0,0,height},b=to+offset+Vec{0,0,height};
                    if(auto hit=w.ray(a,b)){blockedHit=hit;blockedFrom=a;blockedTo=b;return false;}
                }
            }
            const Vec a=to+offset+Vec{0,0,6},b=to+offset+Vec{0,0,cfg.height};
            if(auto hit=w.ray(a,b)){blockedHit=hit;blockedFrom=a;blockedTo=b;return false;}
        }
        return true;
    }
    bool tryRoofTransfer(World& world) {
        if(std::abs(surfaceNormal.z)>.25f)return false;
        AuthoredQueryWorld w(world,authored(Motion::hopUp)?AuthoredQueryWorld::authoredLimit:2048);
        for(float inset:{12.f,18.f}) {
            const Vec over=position-normal*(cfg.gap+clearanceMargin+inset);
            const auto seed=w.ray(over+Vec{0,0,cfg.grip+28},over-Vec{0,0,16});
            if(!seed||!seed->climbable||!seed->point.finite()||!seed->normal.finite()||seed->normal.length()<.5f)continue;
            const Vec roof=seed->normal.unit(),heading=horizontal(roof);

            if(roof.z<.2f||roof.z>=.70f||heading.dot(normal)<-.174f||heading.dot(normal)>.7072f||
                (seed->point-position).length()>cfg.reach)continue;
            const float apexZ=std::max(position.z+112,seed->point.z+42);
            const Vec apex{position.x,position.y,apexZ};
            std::optional<bool> prefixClear;
            for(float height:{26.f,-52.f}) {

                const Vec candidate=offsetFromSurface(seed->point+Vec{0,0,height},*seed);
                if(candidate.z<position.z+12)continue;
                const auto anchor=landingAnchor(w,candidate,*seed);
                if(!anchor||anchor->hit.normal.unit().dot(roof)<.9f||
                    (anchor->position-position).length()>cfg.reach+32||!gripSupport(w,anchor->position,anchor->hit,true))continue;
                const Vec target=anchor->position;
                const Vec above{target.x,target.y,apexZ};
                if(authored(Motion::hopUp)) {
                    if(commitAuthoredRoute(w,Motion::hopUp,target,.80f,horizontal(anchor->hit.normal),true,apex,above,true,anchor->hit.normal.unit()))return true;
                    continue;
                }
                if(!prefixClear.has_value())prefixClear=clearPath(w,position,apex)&&roofPathClear(w,position,apex);
                if(!*prefixClear)break;
                if(!clearPath(w,apex,above)||!roofPathClear(w,apex,above)||
                    !clearPath(w,above,target)||!roofPathClear(w,above,target))continue;
                auto planned=*this;
                planned.beginAction(Motion::hopUp,position,target,.80f);
                planned.detour=true;planned.detourOut=apex;planned.detourOver=above;planned.roofTransfer=true;
                planned.actionStartSurface=surfaceNormal;planned.actionTargetSurface=anchor->hit.normal.unit();
                planned.actionLandingNormal=horizontal(planned.actionTargetSurface);
                if(!planned.authoredActionClear(w,true)||w.exhausted)continue;
                *this=std::move(planned);
                return true;
            }
        }
        return false;
    }

    Vec backJumpVelocity(World& w,Vec impulse) const {
        if(!impulse.finite()||impulse.length()<.001f)return {};
        for(float fraction:{1.f,.5f,.25f})
            if(clearPath(w,position,position+impulse*(.12f*fraction)))return impulse*fraction;
        const Vec outward{impulse.x,impulse.y,0};
        if(clearPath(w,position,position+outward*.12f))return outward;
        return {};
    }
    Vec actionPoint(float phase) const {
        if(authoredPath(actionMotion)) {
            const auto point=authoredMotionPoint(actionMotion,actionFrom,actionTo,actionLandingNormal,phase);
            if(!detour)return point;
            const float along=authoredPathProgress(actionMotion,phase);
            return legacyActionPoint(std::clamp(along,0.f,1.f))+point-(actionFrom+(actionTo-actionFrom)*along);
        }
        return legacyActionPoint(phase);
    }
    Vec legacyActionPoint(float phase) const {
        if(threepeatHop(actionMotion))return threepeatHopPoint(actionMotion,actionFrom,actionTo,phase);
        if(actionMotion==Motion::backFlipOut)return backFlipExitPoint(actionFrom,actionLandingNormal,phase);
        if(actionMotion==Motion::dropBack) {
            const float travel=phase<=1?phase*phase*phase:1+3*(phase-1);
            return actionFrom+(actionTo-actionFrom)*travel;
        }
        if(detour) {
            if(phase<.25f)return lerp(actionFrom,detourOut,phase/.25f);
            if(phase<.75f)return lerp(detourOut,detourOver,(phase-.25f)/.5f);
            return lerp(detourOver,actionTo,(phase-.75f)/.25f);
        }
        return hopMotion(actionMotion)?hopPoint(phase):lerp(actionFrom,actionTo,phase);
    }
    bool authoredActionClear(World& world,bool roofPathChecked=false) const {
        if(roofPathChecked&&(horizontal(actionStartSurface)-normal).length()<=.00001f&&
            (horizontal(actionTargetSurface)-normal).length()<=.00001f&&(actionLandingNormal-normal).length()<=.00001f)return true;
        AuthoredQueryWorld w(world);
        Vec previous=actionFrom;float phase=0;
        while(phase<1) {
            const float next=authoredMotion(actionMotion).trajectory.nextPhase(phase,std::min(1.f,phase+1.f/32));
            const auto point=actionPoint(next);
            const Vec sourceNormal=roofTransfer||supportRecoveryAction?roofFacing(phase):normal;
            const Vec targetNormal=next>=1&&hopMotion(actionMotion)?actionLandingNormal:roofTransfer||supportRecoveryAction?roofFacing(next):normal;
            if(!clearPath(w,previous,point,false,targetNormal,sourceNormal,roofPathChecked)||(roofTransfer&&!roofPathChecked&&!roofPathClear(w,previous,point))||
                (actionMotion==Motion::backFlipOut&&!w.actionBodyPathClear(actionMotion,previous,point,phase,next,actionLandingNormal,actionPoint((phase+next)*.5f)))||
                (actionMotion>=Motion::kickUp&&actionMotion<=Motion::kickRight&&!kickClearance(w,point))) {
                authoredQueryBudgetExhausted|=w.exhausted;return false;
            }
            previous=point;phase=next;
        }
        authoredQueryBudgetExhausted|=w.exhausted;return !w.exhausted;
    }
    bool checkAuthoredAction(World& w) {
        if(authoredActionChecked)return true;
        authoredActionChecked=authoredActionClear(w);return authoredActionChecked;
    }
    bool checkedBackFlip(World& w) {
        if(authored(Motion::backFlipOut)) {
            AuthoredQueryWorld bounded(w);
            const auto to=backFlipExitPoint(position,normal,1);Vec previous=position;float phase=0;
            auto path=[&](float p){return authoredPath(Motion::backFlipOut)?authoredMotionPoint(Motion::backFlipOut,position,to,normal,p):backFlipExitPoint(position,normal,p);};
            while(phase<1) {
                const float next=authoredMotion(Motion::backFlipOut).trajectory.nextPhase(phase,std::min(1.f,phase+1.f/32));
                const auto target=path(next);
                if(!clearPath(bounded,previous,target)||!bounded.actionBodyPathClear(Motion::backFlipOut,previous,target,phase,next,normal,path((phase+next)*.5f))) {
                    authoredQueryBudgetExhausted|=bounded.exhausted;return false;
                }
                previous=target;phase=next;
            }
            authoredQueryBudgetExhausted|=bounded.exhausted;
            authoredBackFlipPrechecked=!bounded.exhausted;return authoredBackFlipPrechecked;
        }
        Vec previous=position;float previousPhase=0;
        for(int knot=1;knot<=backFlipExitSegments;++knot) {
            const float phase=float(knot)/backFlipExitSegments;const auto target=backFlipExitPoint(position,normal,phase);
            if(!clearPath(w,previous,target)||!w.actionBodyClear(Motion::backFlipOut,previous,target,previousPhase,phase,normal))return false;
            previous=target;previousPhase=phase;
        }
        return true;
    }
    Vec checkedBackFlipAbortVelocity(World& w) const {
        const auto tangent=authored(actionMotion)?(actionPoint(std::min(1.f,actionTime+.001f))-actionPoint(std::max(0.f,actionTime-.001f)))/(actionSeconds*.002f):backFlipExitTangent(actionLandingNormal,actionTime);

        for(float fraction:{1.f,.5f,.25f}) {
            const auto target=position+tangent*(.08f*fraction);
            if(clearPath(w,position,target)&&w.actionBodyClear(Motion::backFlipOut,position,target,
                actionTime,actionTime,actionLandingNormal))return tangent*fraction;
        }
        return {};
    }
    Vec hopPoint(float phase) const {

        const float arc=std::pow(std::sin(3.14159265f*phase),2.f);
        return actionFrom+(actionTo-actionFrom)*hopAlongPhase(phase)+normal*(hopDistance*arc)+Vec{0,0,12*arc};
    }
    float hopAlongPhase(float phase) const {
        float along=phase*phase*(3-2*phase);
        if(actionBeganRunning) {

            const float distance=(actionTo-actionFrom).length();
            const float slope=std::clamp(actionRunSpeed*actionSeconds/std::max(1.f,distance),0.f,2.5f);
            along+=slope*(2*phase*phase*phase-3*phase*phase+phase);
        }
        return along;
    }
    static float ease(float x){x=std::clamp(x,0.f,1.f);return x*x*x*(10+x*(-15+6*x));}
    bool runwayClear(World& w,Vec feet) const {
        const auto plane=surfaceNormal.unit();const Vec side{-normal.y,normal.x,0};
        const auto tangent=(Vec{0,0,1}-plane*plane.z).unit();
        for(float x:{-cfg.radius,0.f,cfg.radius})for(float z:{-cfg.radius,0.f,cfg.radius}) {
            const auto start=feet+side*x+tangent*z;
            if(w.ray(start,start+plane*(cfg.height+15)))return false;
        }
        return true;
    }
    bool kickClearance(World& w,Vec feet) const {

        const auto plane=surfaceNormal.unit();const Vec side{-normal.y,normal.x,0};
        const auto tangent=(Vec{0,0,1}-plane*plane.z).unit();
        const float sideExtent=std::max(cfg.radius,cfg.height*.60f),depth=cfg.height+15.f;
        const float below=-std::max(60.f,cfg.height*.55f),above=cfg.height+15.f;
        auto point=[&](float x,float y,float z){return feet+side*x+plane*y+tangent*z;};
        auto blocked=[&](Vec a,Vec b){return w.ray(a,b).has_value()||w.ray(b,a).has_value();};
        for(float a:{-sideExtent,0.f,sideExtent})for(float b:{below,(below+above)*.5f,above})
            if(blocked(point(a,0,b),point(a,depth,b)))return false;
        for(float depthPhase:{0.f,.5f,1.f}) {
            for(float z:{below,(below+above)*.5f,above})
                if(blocked(point(-sideExtent,depth*depthPhase,z),point(sideExtent,depth*depthPhase,z)))return false;
            for(float x:{-sideExtent,0.f,sideExtent})
                if(blocked(point(x,depth*depthPhase,below),point(x,depth*depthPhase,above)))return false;
        }
        return true;
    }
    bool checkedHop(World& w,Vec from,Vec to,float outward,Motion selected=Motion::none,Vec landing={}) {
        if(landing.length()<.5f)landing=normal;
        if(authoredPath(selected)) {
            Vec previous=from;float phase=0;
            while(phase<1) {
                phase=authoredMotion(selected).trajectory.nextPhase(phase,std::min(1.f,phase+1.f/32));
                const auto point=authoredMotionPoint(selected,from,to,normal,phase);
                if(!clearPath(w,previous,point,false,phase>=1?landing:normal)||(running&&!kickClearance(w,point)))return false;
                previous=point;
            }
            hopDistance=outward;return true;
        }

        const auto oldFrom=actionFrom,oldTo=actionTo;const float oldDistance=hopDistance,oldSeconds=actionSeconds;
        const bool oldRunning=actionBeganRunning;
        actionFrom=from;actionTo=to;hopDistance=outward;

        actionBeganRunning=running||runMotion(stableMotion);actionSeconds=jumpActionSeconds(running);
        Vec previous=from;bool valid=true;
        for(int i=1;i<=32;++i) {
            const float phase=float(i)/32;const auto point=hopPoint(phase);
            if(!clearPath(w,previous,point,false,phase>=1?landing:normal)||(running&&!kickClearance(w,point))){valid=false;break;}previous=point;
        }
        actionFrom=oldFrom;actionTo=oldTo;actionSeconds=oldSeconds;actionBeganRunning=oldRunning;
        if(!valid)hopDistance=oldDistance;
        return valid;
    }
    static Vec horizontal(Vec n) { n.z=0; return n.unit(); }
    static Vec lerp(Vec a,Vec b,float t) { t=t*t*(3-2*t); return a+(b-a)*t; }
    bool entrySideClear(World& w,Vec feet) const {
        const Vec centre=feet+Vec{0,0,6};
        for(Vec direction:{normal*-1,Vec{1,0,0},Vec{-1,0,0},Vec{0,1,0},Vec{0,-1,0}}) {
            const auto to=centre+direction*cfg.radius;
            if(auto hit=w.ray(centre,to);hit&&(!hit->normal.finite()||hit->normal.unit().z<.7f)) {
                blockedHit=hit;blockedFrom=centre;blockedTo=to;return false;
            }
        }
        return true;
    }
    bool entryFootprint(World& w,Vec feet,Vec facing) const {
        const Vec side{-facing.y,facing.x,0};
        const float half=cfg.radius*.55f;
        std::optional<Hit> centre;
        for(Vec offset:{Vec{},side*half,side*-half,facing*half,facing*-half}) {
            const auto p=feet+offset;
            const auto hit=w.ray(p+Vec{0,0,cfg.radius},p-Vec{0,0,cfg.radius+8});
            if(!hit||!hit->climbable||!hit->point.finite()||!hit->normal.finite()||
                hit->normal.length()<.5f||hit->normal.unit().z<.7f)return false;
            if(!centre) {
                if(std::abs(hit->point.z-feet.z)>8)return false;centre=hit;
            } else if(std::abs(hit->point.z-centre->point.z)>half+2||
                hit->normal.unit().dot(centre->normal.unit())<.65f)return false;
        }
        return true;
    }
    static Vec entryPoint(Vec from,Vec to,float phase,bool jump,float lift=0) {
        if(lift>0) {
            const auto point=lerp(from,to,std::clamp((phase-.20f)/.80f,0.f,1.f));
            return {point.x,point.y,lerp(from,to,std::clamp(phase/.70f,0.f,1.f)).z};
        }
        return lerp(from,to,phase)+Vec{0,0,jump?18*std::sin(3.14159265f*phase):0};
    }
    bool entryPathClear(World& w,Vec from,Vec to,bool jump,float begin,float end,Vec previous,float lift=0,bool rounded=false) const {
        const int knots=lift>0?32:16;
        if(jump)for(int knot=1;knot<knots;++knot) {
            const float phase=float(knot)/float(knots);if(phase<=begin||phase>=end)continue;
            const auto point=entryPoint(from,to,phase,true,lift);
            if(!clearPath(w,previous,point,rounded))return false;previous=point;
        }
        return clearPath(w,previous,entryPoint(from,to,end,jump,lift),rounded);
    }
    static Vec rawMantlePoint(float phase,Vec from,Vec apex,Vec to) {
        phase=std::clamp(phase,0.f,1.f);
        if(phase<.60f)return lerp(from,apex,std::clamp((phase-.22f)/.38f,0.f,1.f));
        if(phase<.88f)return lerp(apex,{to.x,to.y,apex.z},(phase-.60f)/.28f);
        return lerp({to.x,to.y,apex.z},to,(phase-.88f)/.12f);
    }
    Vec authoredMantlePoint(float phase,Vec from,Vec to) const {
        phase=std::clamp(phase,0.f,1.f);
        if(phase<=0)return from;if(phase>=1)return to;
        if(mantleRoute.count) {
            const float total=authoredMantleDistance(1);
            return from+mantleRoute.sample(total>.001f?authoredMantleDistance(phase)/total:phase);
        }
        const auto& path=cfg.authoredMantleTrajectory;const float scale=std::clamp(cfg.contextScale,.5f,2.f);
        const Vec right{-normal.y,normal.x,0};
        const Vec source=path.sample(phase)*scale,end=path.sample(1)*scale,delta=to-from;
        const Vec destination{delta.dot(right),delta.dot(normal*-1),delta.z};
        const auto fit=[&](float value,float finish,float target) {
            return std::abs(finish)>=8&&finish*target>=0?value*(target/finish):value+(target-finish)*ease(phase);
        };
        const Vec fitted{fit(source.x,end.x,destination.x),fit(source.y,end.y,destination.y),
            end.z>=8.f*scale?source.z*destination.z/end.z:source.z+(destination.z-end.z)*ease(phase)};
        return from+right*fitted.x-normal*fitted.y+Vec{0,0,fitted.z};
    }
    bool authoredMantleClear(World& w,Vec from,Vec) const {
        if(mantleRoute.count) {
            Vec previous=from;
            for(std::uint32_t i=1;i<mantleRoute.count;++i) {
                const Vec next=from+mantleRoute.knots[i].displacement;
                if(!clearPath(w,previous,next,true))return false;
                previous=next;
            }
            return true;
        }
        Vec previous=from;float phase=0;
        while(phase<1) {
            phase=cfg.authoredMantleTrajectory.nextPhase(phase,std::min(1.f,phase+1.f/32.f));
            const auto next=topPathPoint(phase);
            if(!clearPath(w,previous,next,true))return false;
            previous=next;
        }
        return true;
    }
    float authoredMantleDistance(float phase) const {
        const auto& path=cfg.authoredMantleTrajectory;float distance=0;
        for(std::uint32_t i=1;i<path.count;++i) {
            const auto& a=path.knots[i-1];const auto& b=path.knots[i];
            if(phase<=a.phase)break;
            distance+=(b.displacement-a.displacement).length()*std::clamp((phase-a.phase)/(b.phase-a.phase),0.f,1.f);
        }
        return distance;
    }
    float authoredMantleNextPhase(float phase,float limit) const {
        const auto& path=cfg.authoredMantleTrajectory;limit=path.nextPhase(phase,limit);
        if(!mantleRoute.count)return limit;
        const float total=authoredMantleDistance(1);if(total<=.001f)return limit;
        const float progress=authoredMantleDistance(phase)/total;
        const float next=mantleRoute.nextPhase(progress,1),target=next*total;float distance=0;
        for(std::uint32_t i=1;i<path.count;++i) {
            const auto& a=path.knots[i-1];const auto& b=path.knots[i];
            const float segment=(b.displacement-a.displacement).length();
            if(segment>.00001f&&target<=distance+segment) {
                const float crossing=a.phase+(b.phase-a.phase)*std::clamp((target-distance)/segment,0.f,1.f);
                if(crossing>phase+.000001f)limit=std::min(limit,crossing);
                break;
            }
            distance+=segment;
        }
        return limit;
    }
    bool selectAuthoredMantlePath(World& world) {
        AuthoredQueryWorld w(world,12288);
        mantleRoute={};
        if(authoredMantleClear(w,mantleFrom,mantleTo)&&!w.exhausted)return true;
        if(w.exhausted){authoredQueryBudgetExhausted=true;return false;}
        if(!authoredPath(Motion::contextMantle))return false;
        mantleRoute.count=1;
        for(unsigned i=1;i<=64;++i) {
            const Vec point=roundedMantlePoint(float(i)/64,mantleFrom,mantleApex,mantleTo)-mantleFrom;
            const auto& last=mantleRoute.knots[mantleRoute.count-1];const float distance=(point-last.displacement).length();
            if(distance<.05f&&i<64)continue;
            mantleRoute.knots[mantleRoute.count++]={last.phase+distance,point};
        }
        const float distance=mantleRoute.knots[mantleRoute.count-1].phase;
        if(distance>.001f) {
            for(std::uint32_t i=1;i<mantleRoute.count;++i)mantleRoute.knots[i].phase/=distance;
            const auto end=mantleRoute.knots[mantleRoute.count-1];std::uint32_t kept=1;
            for(std::uint32_t i=1;i+1<mantleRoute.count;++i)
                if(mantleRoute.knots[i].phase-mantleRoute.knots[kept-1].phase>=.0001f&&1-mantleRoute.knots[i].phase>=.0001f)
                    mantleRoute.knots[kept++]=mantleRoute.knots[i];
            mantleRoute.knots[kept++]=end;mantleRoute.count=kept;
            if(mantleRoute.valid()&&authoredMantleClear(w,mantleFrom,mantleTo)&&!w.exhausted)return true;
        }
        authoredQueryBudgetExhausted|=w.exhausted;
        mantleRoute={};return false;
    }
    static Vec roundedMantlePoint(float phase,Vec from,Vec apex,Vec to) {
        const auto raw=rawMantlePoint(phase,from,apex,to);
        const float weight=std::max(ease(1-std::abs(phase-.60f)/.08f),ease(1-std::abs(phase-.88f)/.06f));
        if(weight<=0)return raw;

        const auto filtered=(rawMantlePoint(phase-.025f,from,apex,to)+rawMantlePoint(phase-.0125f,from,apex,to)*2+
            raw*3+rawMantlePoint(phase+.0125f,from,apex,to)*2+rawMantlePoint(phase+.025f,from,apex,to))/9;
        return raw+(filtered-raw)*weight;
    }
    bool roundedMantleClear(World& w,Vec from,Vec apex,Vec to) const {
        Vec previous=from;
        for(int sample=1;sample<=32;++sample) {
            const auto next=roundedMantlePoint(sample/32.f,from,apex,to);
            if(!clearPath(w,previous,next,true))return false;
            previous=next;
        }
        return true;
    }
    Vec offsetFromSurface(Vec feet,const Hit& hit) const {
        const auto n=hit.normal.unit();
        const float horizontalLength=std::sqrt(n.x*n.x+n.y*n.y);

        const float supportHeight=n.z>=0 ? 6.0f : cfg.height;
        const float distance=(feet+Vec{0,0,supportHeight}-hit.point).dot(n)/horizontalLength;
        return feet+horizontal(n)*(cfg.gap-distance);
    }
    bool gripSupport(World& w,Vec p,const Hit& surface,bool narrow) const {
        const auto plane=surface.normal.unit();
        const auto tangent=(Vec{0,0,1}-plane*plane.z).unit();
        const auto facing=horizontal(plane);
        const Vec right{-facing.y,facing.x,0};
        auto contact=[&](Vec anchor)->std::optional<Hit> {
            auto h=w.ray(anchor+plane*16.f,anchor-plane*(cfg.gap+20.f));
            if(h&&h->climbable&&h->point.finite()&&h->normal.finite()&&
                h->normal.length()>.5f&&h->normal.unit().dot(plane)>.35f&&
                (narrow||std::abs((h->point-surface.point).dot(plane))<=8.f))return h;
            return {};
        };

        const std::array<float,2> heights{cfg.chest,cfg.grip};
        const std::array<Vec,9> offsets{Vec{},right*-10.f,right*10.f,tangent*-6.f,tangent*6.f,
            right*-10.f-tangent*6.f,right*-10.f+tangent*6.f,right*10.f-tangent*6.f,right*10.f+tangent*6.f};
        std::array<std::array<std::optional<Hit>,9>,2> samples{};
        std::array<std::array<bool,9>,2> queried{};
        auto sample=[&](std::size_t height,std::size_t index)->const std::optional<Hit>& {
            if(!queried[height][index]) {
                queried[height][index]=true;
                const auto anchor=p+Vec{0,0,6}+tangent*(heights[height]-6);
                samples[height][index]=contact(anchor+offsets[index]);
            }
            return samples[height][index];
        };

        for(std::size_t height=0;height<heights.size();++height) {
            const auto& center=sample(height,0);if(!center)continue;
            for(std::size_t side:{1u,2u}) {
                const auto& neighbour=sample(height,side);
                if(neighbour&&neighbour->normal.unit().dot(center->normal.unit())>.70f&&
                    (neighbour->point-center->point).length()<45)return true;
            }
            if(narrow)for(std::size_t heightOffset:{3u,4u}) {
                const auto& neighbour=sample(height,heightOffset);
                if(neighbour&&neighbour->normal.unit().dot(center->normal.unit())>.9f&&
                    (neighbour->point-center->point).length()<18)return true;
            }
        }

        auto pair=[](const std::optional<Hit>& a,const std::optional<Hit>& b,float normalDot,float distance) {
            if(!a||!b||a->normal.unit().dot(b->normal.unit())<=normalDot)return false;
            const float separation=(a->point-b->point).length();
            return separation>.001f&&separation<distance;
        };
        for(std::size_t height=0;height<heights.size();++height) {
            const auto& left=sample(height,1);const auto& rightContact=sample(height,2);
            if(pair(left,rightContact,.70f,45.f))return true;
            if(narrow) {
                const auto& lower=sample(height,3);const auto& upper=sample(height,4);
                if(pair(lower,upper,.9f,18.f))return true;

                for(std::size_t side:{1u,2u}) {
                    const auto& known=sample(height,side);if(!known)continue;
                    const std::size_t lowerIndex=side==1?5:7;
                    if(pair(known,sample(height,lowerIndex),.9f,18.f))return true;
                    if(pair(known,sample(height,lowerIndex+1),.9f,18.f))return true;
                }
            }
        }
        return false;
    }
    struct GripSupportCache {
        struct Entry {Vec position,plane;bool supported;};
        std::array<Entry,28> entries{};
        std::size_t size{};
    };
    bool trackedGripSupport(World& w,Vec p,const Hit& surface,GripSupportCache* cache) const {
        if(!cache)return gripSupport(w,p,surface,true);
        const auto plane=surface.normal.unit();
        auto equal=[](Vec a,Vec b){return a.x==b.x&&a.y==b.y&&a.z==b.z;};
        for(std::size_t i=0;i<cache->size;++i) {
            const auto& entry=cache->entries[i];
            if(equal(entry.position,p)&&equal(entry.plane,plane))return entry.supported;
        }
        const bool supported=gripSupport(w,p,surface,true);
        if(cache->size<cache->entries.size())cache->entries[cache->size++]={p,plane,supported};
        return supported;
    }
    std::optional<Hit> wall(World& w,Vec p,Vec forward,float reach,AttachFailure* failure=nullptr,bool tracking=false,bool checkedLowTop=false,bool prospective=false,GripSupportCache* grips=nullptr,Vec expectedNormal={}) const {
        const Vec right{forward.y,-forward.x,0};
        std::optional<Hit> best;
        float bestCorrection=1e9f;
        bool bestGrip=false;
        if(failure) *failure=AttachFailure::noWall;
        auto valid=[&](const Hit& hit) {
            if(!hit.climbable||!hit.normal.finite()||!hit.point.finite()||hit.normal.length()<0.5f) return false;
            const auto n=hit.normal.unit();
            if(expectedNormal.length()>.5f&&n.dot(expectedNormal.unit())<.95f)return false;
            return n.z>=(tracking?-.45f:-.15f)&&n.z<=(tracking?std::max(.94f,cfg.maxNormalZ):cfg.maxNormalZ)&&horizontal(n).dot(forward)<(tracking?-.35f:-.65f);
        };

        for(float height : {6.0f,32.0f,cfg.chest,cfg.grip}) {
            const Vec base=p+Vec{0,0,height};
            const Vec from=base-forward*(tracking?16.f:0.f),to=base+forward*reach;
            auto center=w.ray(from,to);
            if(!center) continue;
            if(!valid(*center)) { if(failure) *failure=AttachFailure::surface; continue; }
            bool supported=false;
            for(float side : {-10.0f,10.0f}) {
                auto neighbour=w.ray(from+right*side,to+right*side);
                if(neighbour&&valid(*neighbour)&&center->normal.unit().dot(neighbour->normal.unit())>0.70f&&
                    (neighbour->point-center->point).length()<45) {supported=true;break;}
            }

            if(!supported&&tracking) {
                for(float dz:{-6.f,6.f}) {
                    auto h=w.ray(from+Vec{0,0,dz},to+Vec{0,0,dz});
                    if(h&&valid(*h)&&h->normal.unit().dot(center->normal.unit())>.9f&&
                        (h->point-center->point).length()<18){supported=true;break;}
                }
            }
            if(supported) {
                const float correction=(offsetFromSurface(p,*center)-p).length();
                const bool hasGrip=!tracking&&gripSupport(w,offsetFromSurface(p,*center),*center,false);

                if(((hasGrip&&!bestGrip)||(hasGrip==bestGrip&&correction<bestCorrection))&&(!tracking||
                    (checkedLowTop&&correction<=20.f)||
                    ((!prospective||correction<=cfg.reach)&&
                        trackedGripSupport(w,prospective?offsetFromSurface(p,*center):p,*center,grips)))) {
                    best=center;bestCorrection=correction;bestGrip=hasGrip;
                }
            }
            if(failure) *failure=AttachFailure::support;
        }
        return best;
    }
    std::optional<Hit> support(World& w,Vec p,Vec forward,bool checkedLowTop=false,bool prospective=false,Vec expectedNormal={}) const {
        GripSupportCache grips;
        if(auto h=wall(w,p,forward,cfg.reach,nullptr,true,checkedLowTop,prospective,&grips,expectedNormal))return h;

        static const auto directions=[] {
            std::array<Vec,6> values{};
            constexpr std::array<float,6> angles{-.35f,.35f,-.7f,.7f,-1.05f,1.05f};
            for(std::size_t index=0;index<values.size();++index)
                values[index]={std::cos(angles[index]),std::sin(angles[index]),0};
            return values;
        }();
        for(const auto direction:directions) {
            const Vec f{forward.x*direction.x-forward.y*direction.y,
                forward.x*direction.y+forward.y*direction.x,0};
            if(auto h=wall(w,p,f,cfg.reach,nullptr,true,checkedLowTop,prospective,&grips,expectedNormal))return h;
        }
        return {};
    }
    std::optional<Hit> recoveryLanding(World& w,Vec point,Vec expectedNormal) const {
        const auto surface=expectedNormal.unit();
        const auto hit=support(w,point,horizontal(surface)*-1,false,false,surface);
        if(!hit||hit->normal.unit().dot(surface)<.95f||(offsetFromSurface(point,*hit)-point).length()>2)return {};
        return hit;
    }
    struct Anchor {Vec position;Hit hit;};
    std::optional<Anchor> landingAnchor(World& w,Vec point,Hit hit) const {

        for(int iteration=0;iteration<3;++iteration) {
            point=offsetFromSurface(point,hit);
            const auto actual=support(w,point,horizontal(hit.normal)*-1);
            if(!actual)return {};
            const auto corrected=offsetFromSurface(point,*actual);
            if((corrected-point).length()<.5f)return Anchor{corrected,*actual};
            hit=*actual;
        }
        return {};
    }

    bool clearPath(World& w,Vec from,Vec to,bool supportedTop=false,Vec destinationNormal={},Vec sourceNormal={},bool headingOnly=false,bool targetChecked=false) const {
        if(!to.finite()) return false;
        const bool moving=(to-from).length()>.001f;
        if(sourceNormal.length()<.5f)sourceNormal=normal;
        const auto towardWall=horizontal(sourceNormal)*-1;
        Vec travel=to-from;travel.z=0;travel=travel.unit();
        const Vec sideways{-towardWall.y,towardWall.x,0};
        const bool turning=destinationNormal.length()>.5f&&(destinationNormal-sourceNormal).length()>.00001f;
        const auto nextWall=turning?horizontal(destinationNormal)*-1:towardWall;
        const Vec nextSide{-nextWall.y,nextWall.x,0};
        const std::array<Vec,12> offsets={Vec{},Vec{cfg.radius,0,0},Vec{-cfg.radius,0,0},
            Vec{0,cfg.radius,0},Vec{0,-cfg.radius,0},towardWall*cfg.radius,travel*cfg.radius,
            sideways*cfg.radius,sideways*-cfg.radius,nextWall*cfg.radius,nextSide*cfg.radius,nextSide*-cfg.radius};

        const float lowerRadius=cfg.radius<=6?cfg.radius:
            std::sqrt(std::max(0.f,2*cfg.radius*6.f-36.f));
        std::array<bool,12> checkedOffsets{};
        for(std::size_t index=headingOnly?5:0;index<(turning?offsets.size():9);++index) {
            if(headingOnly&&index==6)continue;
            if(index>=5) {
                bool duplicate=false;
                for(std::size_t prior=0;prior<index;++prior)duplicate|=checkedOffsets[prior]&&(offsets[index]-offsets[prior]).length()<.01f;
                if(duplicate)continue;
            }
            checkedOffsets[index]=true;
            const auto o=offsets[index];
            const auto lower=supportedTop?o*(lowerRadius/std::max(.01f,cfg.radius)):o;
            for(float z:{6.0f,cfg.chest,cfg.height}) {
                if(moving) {
                    const auto section=z==6.f?lower:o;
                    const Vec a=from+section+Vec{0,0,z},b=to+section+Vec{0,0,z};
                    if(auto hit=w.ray(a,b)) {blockedHit=hit;blockedFrom=a;blockedTo=b;return false;}
                }
            }
            if(supportedTop) {

                if(moving) {
                    const Vec a=from+o+Vec{0,0,cfg.radius},b=to+o+Vec{0,0,cfg.radius};
                    if(auto hit=w.ray(a,b)) {blockedHit=hit;blockedFrom=a;blockedTo=b;return false;}
                }
                const Vec a=to+lower+Vec{0,0,6},b=to+o+Vec{0,0,cfg.radius};
                if(auto hit=w.ray(a,b)) {blockedHit=hit;blockedFrom=a;blockedTo=b;return false;}
            }
            const Vec a=to+o+Vec{0,0,supportedTop?cfg.radius:6.f},b=to+o+Vec{0,0,cfg.height};
            if(auto hit=w.ray(a,b)) {blockedHit=hit;blockedFrom=a;blockedTo=b;return false;}
            if(std::abs(o.x)>.01f&&std::abs(o.y)>.01f) {
                const auto centre=to+Vec{0,0,std::clamp(cfg.chest,supportedTop?cfg.radius:6.f,cfg.height)};
                const auto edge=centre+o;
                if(auto hit=w.ray(centre,edge)) {blockedHit=hit;blockedFrom=centre;blockedTo=edge;return false;}
            }
        }
        if(headingOnly&&targetChecked)return true;
        const auto centre=to+Vec{0,0,std::clamp(cfg.chest,supportedTop?cfg.radius:6.f,cfg.height)};
        std::array<Vec,8> measured{};std::size_t measuredCount{};
        for(const Vec axis:{Vec{1,0,0},Vec{-1,0,0},Vec{0,1,0},Vec{0,-1,0},
            Vec{.70710678118f,.70710678118f,0},Vec{-.70710678118f,.70710678118f,0},
            Vec{.70710678118f,-.70710678118f,0},Vec{-.70710678118f,-.70710678118f,0}}) {
            const auto end=centre+axis*(cfg.radius*1.41421356237f);
            const auto hit=w.ray(centre,end);if(!hit)continue;
            if((hit->point-centre).length()<=cfg.radius) {
                blockedHit=hit;blockedFrom=centre;blockedTo=end;return false;
            }
            const auto h=horizontal(hit->normal);if(h.length()<.5f)continue;
            const float distance=(hit->point-centre).dot(h);if(std::abs(distance)>cfg.radius)continue;
            const auto direction=h*(distance<0?-1.f:1.f);
            bool duplicate=false;
            for(std::size_t index=0;index<measuredCount;++index)duplicate|=measured[index].dot(direction)>.99999f;
            if(duplicate)continue;
            measured[measuredCount++]=direction;
            const auto edge=centre+direction*cfg.radius;
            if(auto obstruction=w.ray(centre,edge)) {blockedHit=obstruction;blockedFrom=centre;blockedTo=edge;return false;}
        }
        return true;
    }
    struct Ledge { Vec stand,lip;std::array<Vec,2> hands,normals;bool frontEdge{},crest{}; };
    struct CrestSection {Vec point;Hit nearSide,farSide;};
    bool crestFace(const std::optional<Hit>& hit,bool nearSide) const {
        if(!hit||!hit->climbable||!hit->point.finite()||!hit->normal.finite()||hit->normal.length()<.5f)return false;
        const auto n=hit->normal.unit();
        return n.z>=.30f&&n.z<.70f&&horizontal(n).dot(normal)*(nearSide?1.f:-1.f)>(nearSide?.97f:.80f);
    }
    bool crestCap(const std::optional<Hit>& hit) const {
        return hit&&hit->climbable&&hit->point.finite()&&hit->normal.finite()&&
            hit->normal.length()>.5f&&hit->normal.unit().z>=.94f;
    }
    std::optional<CrestSection> crestSection(World& w,Vec a,Vec b,float high,float low) const {
        auto down=[&](Vec p){return w.ray(Vec{p.x,p.y,high},Vec{p.x,p.y,low});};
        auto nearSide=down(a),farSide=down(b);
        if(!crestFace(nearSide,true)||!crestFace(farSide,false))return {};
        const Hit nearFoot=*nearSide,farFoot=*farSide;
        const Vec axis=nearFoot.normal.unit().cross(farFoot.normal.unit()).unit();
        if(axis.length()<.9f||std::sqrt(std::max(0.f,1-axis.z*axis.z))<.70f)return {};

        for(int i=0;i<10;++i) {
            const auto middle=(a+b)*.5f;const auto hit=down(middle);
            if(crestFace(hit,true)){a=middle;nearSide=hit;}
            else if(crestFace(hit,false)){b=middle;farSide=hit;}
            else if(crestCap(hit)) {

                Vec nearQuery=a,nearTop=middle,farQuery=b,farTop=middle;
                Hit nearSlope=*nearSide,farSlope=*farSide,nearCap=*hit,farCap=*hit;
                for(unsigned refine=0;refine<10;++refine) {
                    const Vec query=(nearQuery+nearTop)*.5f;const auto sample=down(query);
                    if(crestFace(sample,true)){nearQuery=query;nearSlope=*sample;}
                    else if(crestCap(sample)){nearTop=query;nearCap=*sample;}
                    else return {};
                    const Vec opposite=(farQuery+farTop)*.5f;const auto other=down(opposite);
                    if(crestFace(other,false)){farQuery=opposite;farSlope=*other;}
                    else if(crestCap(other)){farTop=opposite;farCap=*other;}
                    else return {};
                }
                const float width=(farCap.point-nearCap.point).length();
                if(width<.1f||width>cfg.radius*1.1f||
                    (nearCap.point-nearSlope.point).length()>.20f||
                    (farCap.point-farSlope.point).length()>.20f)return {};
                const Vec centre=(nearCap.point+farCap.point)*.5f;
                const unsigned samples=std::max(2u,unsigned(std::ceil(width)));
                std::optional<Hit> previous;
                for(unsigned sample=0;sample<=samples;++sample) {
                    const Vec query=nearTop+(farTop-nearTop)*(float(sample)/float(samples));
                    const auto current=down(query);
                    if(!crestCap(current)||std::abs(current->point.z-centre.z)>2.f)return {};
                    if(previous) {
                        const Vec delta=current->point-previous->point;

                        if(std::abs(delta.dot(previous->normal.unit()))>.08f||
                            std::abs(delta.dot(current->normal.unit()))>.08f)return {};
                    }
                    previous=current;
                }
                const auto crown=down(centre);
                if(!crestCap(crown)||crown->point.z+.1f<nearFoot.point.z||crown->point.z+.1f<farFoot.point.z)return {};
                return CrestSection{crown->point,nearFoot,farFoot};
            }
            else return {};
        }
        if((nearSide->point-farSide->point).length()>.20f)return {};
        const auto peak=(nearSide->point+farSide->point)*.5f;
        if(peak.z+.1f<nearFoot.point.z||peak.z+.1f<farFoot.point.z)return {};
        return CrestSection{peak,nearFoot,farFoot};
    }
    std::optional<Vec> crestStanding(World& w,Vec peak) const {
        const float half=std::max(12.f,cfg.radius*.55f);
        auto section=[&](Vec centre)->std::optional<CrestSection> {
            for(float span:{6.f,12.f,18.f,24.f}) {
                auto row=crestSection(w,centre+normal*span,centre-normal*span,centre.z+16,centre.z-48);
                if(row&&(row->point-centre).length()<=.5f&&
                    row->point.z>=row->nearSide.point.z+2&&row->point.z>=row->farSide.point.z+2)return row;
            }
            return {};
        };
        const auto middle=section(peak);
        if(!middle)return {};
        const Vec axis=middle->nearSide.normal.unit().cross(middle->farSide.normal.unit()).unit();
        const float flat=std::sqrt(std::max(0.f,1-axis.z*axis.z));
        if(flat<.70f||(cfg.radius+7.f)*flat<cfg.radius+.1f)return {};
        for(float side:{-half,-half*.5f,half*.5f,half}) {
            const Vec centre=peak+axis*(side/flat);
            const auto row=section(centre);
            if(!row||row->nearSide.normal.unit().dot(middle->nearSide.normal.unit())<.98f||
                row->farSide.normal.unit().dot(middle->farSide.normal.unit())<.98f)return {};
        }
        const Vec target=peak+Vec{0,0,7};
        return clearPath(w,target,target,true)?std::optional<Vec>(target):std::nullopt;
    }
    bool crestOpportunity(World& w) const {
        const float high=position.z+cfg.grip+28,low=position.z-16;
        bool nearSlope=false;
        for(float distance:{cfg.gap,cfg.gap+16,cfg.gap+32,cfg.gap+48,cfg.gap+64}) {
            if(distance>cfg.reach)break;
            const auto query=position-normal*distance;
            const auto hit=w.ray({query.x,query.y,high},{query.x,query.y,low});
            if(crestFace(hit,true)){nearSlope=true;continue;}
            if(nearSlope&&crestCap(hit))continue;
            if(nearSlope&&crestFace(hit,false))return true;
            nearSlope=false;
        }
        return false;
    }
    std::optional<Ledge> findCrestLedge(World& w) const {
        const float high=position.z+cfg.grip+28,low=position.z-16;
        std::optional<Hit> previous;Vec previousQuery{};
        const Vec right{-normal.y,normal.x,0};
        const auto plane=surfaceNormal.unit(),tangent=(Vec{0,0,1}-plane*plane.z).unit();
        for(float distance:{cfg.gap,cfg.gap+16,cfg.gap+32,cfg.gap+48,cfg.gap+64}) {
            if(distance>cfg.reach)break;
            const auto query=position-normal*distance;
            const auto h=w.ray({query.x,query.y,high},{query.x,query.y,low});
            if(crestFace(h,true)){previous=h;previousQuery=query;continue;}
            if(previous&&crestCap(h))continue;
            if(!previous||!crestFace(h,false)){previous.reset();continue;}
            const auto cross=crestSection(w,previousQuery,query,high,low);
            if(!cross)return {};
            const auto stand=crestStanding(w,cross->point);
            if(!stand||stand->z-position.z>cfg.grip+28)return {};
            Ledge out{*stand,cross->point};out.crest=true;
            const float half=std::max(12.f,cfg.radius*.55f);
            Vec ridge=cross->nearSide.normal.unit().cross(cross->farSide.normal.unit()).unit();
            if(ridge.dot(right)<0)ridge=ridge*-1;
            const float flat=std::sqrt(std::max(0.f,1-ridge.z*ridge.z));
            for(int hand=0;hand<2;++hand) {
                const auto nearSide=cross->point+ridge*((hand==0?-half:half)/flat)+normal*6;
                const auto grip=w.ray(nearSide+Vec{0,0,16},nearSide-Vec{0,0,24});
                if(!crestFace(grip,true)&&!crestCap(grip))return {};
                bool reachable=false;
                for(float height:{32.f,cfg.chest,cfg.grip}) {
                    const auto anchor=position+right*(hand==0?-18.f:18.f)+Vec{0,0,6}+tangent*(height-6);
                    reachable|=(grip->point-anchor).length()<=cfg.gap+24;
                }
                if(!reachable)return {};
                out.hands[hand]=grip->point;out.normals[hand]=grip->normal.unit();
            }
            const Vec apex{position.x,position.y,std::max(position.z,out.stand.z+4)};
            const Vec above{out.stand.x,out.stand.y,apex.z};
            if(!clearPath(w,position,apex)||!clearPath(w,apex,above,true)||!clearPath(w,above,out.stand,true))return {};
            ledgeReason="ready at supported roof crest";return out;
        }
        return {};
    }
    std::optional<Ledge> checkedLedge(World& w,Vec stand,Vec lip) const {
        const Vec right{-normal.y,normal.x,0};
        const auto plane=surfaceNormal.unit();
        const auto tangent=(Vec{0,0,1}-plane*plane.z).unit();

        for(float halfSpacing:{18.f,14.f,10.f}) {
            Ledge out{stand,lip};bool pair=true;
            for(int hand=0;hand<2;++hand) {
                bool found=false;const float side=hand==0?-halfSpacing:halfSpacing;
                for(float inset:{3.f,8.f,15.f,23.f}) {
                    const Vec point=lip+right*side-normal*inset;
                    auto hit=w.ray(point+Vec{0,0,22},point-Vec{0,0,24});
                    if(hit&&hit->climbable&&hit->normal.z>=.70f) {
                        bool reachable=false;
                        for(float height:{32.f,cfg.chest,cfg.grip}) {
                            const auto anchor=position+right*(hand==0?-18.f:18.f)+Vec{0,0,6}+tangent*(height-6);
                            if((hit->point-anchor).length()<=cfg.gap+24.f) {reachable=true;break;}
                        }
                        if(!reachable)continue;
                        out.hands[hand]=hit->point;out.normals[hand]=hit->normal.unit();found=true;break;
                    }
                }
                if(!found){pair=false;break;}
            }
            if(pair){ledgeReason="ready";return out;}
        }
        ledgeReason="no solid top under both palms";return {};
    }
    std::optional<Vec> standingSupport(World& w,Vec feet,float down,float up) const {
        const auto floor=w.ray(feet+Vec{0,0,up},feet-Vec{0,0,down});
        if(!floor||!floor->climbable||floor->normal.z<.70f)return {};

        const float footprint=std::max(12.f,cfg.radius*.55f),span=footprint+16;
        Vec target=floor->point;
        for(Vec offset:{Vec{footprint,0,0},Vec{-footprint,0,0},Vec{0,footprint,0},Vec{0,-footprint,0}}) {
            const auto foot=w.ray(floor->point+offset+Vec{0,0,span},floor->point+offset-Vec{0,0,span});
            if(!foot||!foot->climbable||foot->normal.z<.70f)return {};
            target.z=std::max(target.z,foot->point.z);
        }
        target.z+=7;

        return target;
    }
    std::optional<Vec> standingFloor(World& w,Vec feet,float down,float up) const {
        const auto target=standingSupport(w,feet,down,up);
        return target&&clearPath(w,*target,*target,true)?target:std::nullopt;
    }
    std::optional<Ledge> reachableTopEdge(World& w,Vec stand,Hit floor) const {

        const Vec outerProbe=position-normal*(cfg.gap+4);
        const float span=std::min(cfg.reach,(outerProbe-floor.point).dot(normal));
        if(span<=0)return {};
        auto contact=[&](Vec point) {
            point.z=position.z;
            return w.ray(point+Vec{0,0,cfg.grip+28},point-Vec{0,0,16});
        };
        auto walkable=[](const std::optional<Hit>& hit) {
            return hit&&hit->climbable&&hit->point.finite()&&hit->normal.finite()&&hit->normal.z>=.70f;
        };
        Vec previousQuery=floor.point;Hit previous=floor;
        constexpr float maxWalkableGrade=1.020205f;
        for(int sample=1;sample<=12;++sample) {
            const Vec query=floor.point+normal*(span*sample/12.f);
            auto hit=contact(query);
            const float distance=(query-previousQuery).length();
            const bool drop=!walkable(hit)||previous.point.z-hit->point.z>distance*maxWalkableGrade+2.f;
            if(drop) {
                Vec inside=previousQuery,outside=query;Hit lip=previous;

                for(int refine=0;refine<6;++refine) {
                    const auto middle=(inside+outside)*.5f;auto candidate=contact(middle);
                    const float along=(middle-previousQuery).length();
                    if(walkable(candidate)&&previous.point.z-candidate->point.z<=along*maxWalkableGrade+2.f) {
                        inside=middle;lip=*candidate;
                    } else outside=middle;
                }

                return checkedLedge(w,stand,lip.point);
            }
            if(!walkable(hit))break;
            previousQuery=query;previous=*hit;
        }
        return {};
    }
    std::optional<Ledge> faceEdgeLedge(World& w,Vec stand) const {

        if(std::abs(surfaceNormal.z)>.25f)return {};
        const Vec right{-normal.y,normal.x,0};
        auto contact=[&](float height,float side) {
            const auto from=position+right*side+Vec{0,0,height};
            return w.ray(from,from-normal*(cfg.gap+24));
        };
        auto front=[&](const std::optional<Hit>& hit) {
            return hit&&hit->climbable&&hit->point.finite()&&hit->normal.finite()&&
                std::abs(hit->normal.z)<.25f&&horizontal(hit->normal).dot(normal)>.95f;
        };
        float low=0,high=cfg.grip+28;std::optional<Hit> lower;
        if(contact(high,0))return {};
        for(float height:{cfg.grip,cfg.chest,32.f})if(auto h=contact(height,0);front(h)){low=height;lower=h;break;}
        if(!lower)return {};
        for(int i=0;i<8;++i) {
            const float middle=(low+high)*.5f;const auto h=contact(middle,0);
            if(front(h)&&(h->point-lower->point).dot(normal)>-2.f){low=middle;lower=h;}
            else if(h)return {};else high=middle;
        }
        const float edgeZ=position.z+low;
        if(edgeZ-stand.z<4||edgeZ-stand.z>64)return {};
        for(float half:{18.f,14.f,10.f}) {
            Ledge out{stand,{}};out.frontEdge=true;bool pair=true;
            for(int hand=0;hand<2;++hand) {
                const float side=hand==0?-half:half;
                const auto h=contact(low-2,side);
                if(!front(h)||std::abs((h->point-lower->point).dot(normal))>2||
                    contact(high+3,side)||contact(high+16,side)){pair=false;break;}
                bool reachable=false;
                for(float height:{32.f,cfg.chest,cfg.grip}) {
                    const auto anchor=position+right*(hand==0?-18.f:18.f)+Vec{0,0,height};
                    reachable|=(h->point-anchor).length()<=cfg.gap+24;
                }
                if(!reachable){pair=false;break;}
                out.hands[hand]=h->point;out.normals[hand]=h->normal.unit();
            }
            if(!pair)continue;
            out.lip=(out.hands[0]+out.hands[1])*.5f+Vec{0,0,2};
            const Vec apex{position.x,position.y,std::max({position.z,stand.z+4,out.lip.z+8})};
            const Vec above{stand.x,stand.y,apex.z};
            if(clearPath(w,position,apex)&&clearPath(w,apex,above,true)&&clearPath(w,above,stand,true)) {
                ledgeReason="ready at verified front edge";return out;
            }
        }
        return {};
    }
    std::optional<Ledge> findLedge(World& source,bool* reachableStandingPath=nullptr) const {
        TopCandidateBudgetWorld<World,Vec,Hit> budget(source);
        ProbeRayCache w(budget);
        const float highZ=cfg.grip+28;
        if(reachableStandingPath)*reachableStandingPath=false;
        ledgeReason="no walkable top";
        auto candidate=[&](const Hit& floor,float slopeStandAllowance=0.f)->std::optional<Ledge> {
            const auto stand=standingFloor(w,floor.point+Vec{0,0,1},8,8);
            if(!stand){ledgeReason="top footprint or body blocked";return {};}
            const Vec target=*stand;
            if(target.z-position.z>highZ+slopeStandAllowance){ledgeReason="top beyond reach";return {};}
            const Vec apex{position.x,position.y,std::max(position.z,target.z+4)};
            const Vec forwardTop{target.x,target.y,apex.z};
            if(!clearPath(w,position,apex)||!clearPath(w,apex,forwardTop,true)||!clearPath(w,forwardTop,target,true)) {
                ledgeReason="top path blocked";return faceEdgeLedge(w,target);
            }
            if(reachableStandingPath)*reachableStandingPath=true;
            const auto edge=position-normal*(cfg.gap+4);
            const auto lip=w.ray(edge+Vec{0,0,highZ},edge-Vec{0,0,16});
            if(lip&&lip->climbable&&lip->normal.z>=.70f)
                if(auto result=checkedLedge(w,target,lip->point))return result;
            if((floor.point-position).length()<cfg.grip+cfg.gap)
                if(auto result=checkedLedge(w,target,floor.point))return result;
            return reachableTopEdge(w,target,floor);
        };
        const Vec right{-normal.y,normal.x,0};
        auto lane=[&](float side)->std::optional<Ledge> {
        std::array<Hit,5> floors{};int count=0;
        for(float extra : {0.f,12.f,28.f,50.f,80.f}) {
            const Vec over=position+right*side-normal*(cfg.gap+std::max(12.f,cfg.radius*.55f)+4+extra);
            const Vec high=over+Vec{0,0,highZ};
            if(const auto approach=w.ray(position+Vec{0,0,highZ},high)) {
                ledgeReason="top approach blocked";

                if(approach->climbable&&approach->point.finite()&&approach->normal.finite()) {
                    const auto plane=approach->normal.unit();
                    if(plane.z>=.70f) {
                        const float footprint=std::max(12.f,cfg.radius*.55f);
                        const float rise=-(high-approach->point).dot(plane)/plane.z;
                        if(rise>=0&&rise<=footprint+16) {
                            const auto top=w.ray(high+Vec{0,0,rise+8},over-Vec{0,0,16});
                            if(top&&top->climbable&&top->point.finite()&&top->normal.finite()&&top->normal.z>=.70f) {

                                constexpr float maxGrade=1.020205f;
                                const float allowance=footprint*maxGrade+7;
                                if(auto result=candidate(*top,allowance))return result;
                            }
                        }
                    }
                }
                continue;
            }
            auto floor=w.ray(high,over-Vec{0,0,16});
            if(!floor||!floor->climbable||floor->normal.z<0.70f) continue;
            floors[count++]=*floor;
            if(auto result=candidate(*floor))return result;
        }

        for(int i=1;i<count;++i) {
            if(std::abs(floors[i].point.z-floors[i-1].point.z)>2||
                floors[i].normal.dot(floors[i-1].normal)<.98f||
                (floors[i].point-floors[i-1].point).length()<12)continue;
            const auto middle=(floors[i].point+floors[i-1].point)*.5f;
            const Vec over{middle.x,middle.y,position.z};
            const Vec high=over+Vec{0,0,highZ};
            if(w.ray(position+Vec{0,0,highZ},high))continue;
            const auto floor=w.ray(high,over-Vec{0,0,16});
            if(floor&&floor->climbable&&floor->normal.z>=.70f)
                if(auto result=candidate(*floor))return result;
        }
        return {};
        };
        const auto result=searchTopCandidateLanes(cfg.radius,lane);
        if(budget.exhausted){if(reachableStandingPath)*reachableStandingPath=false;ledgeReason="top query budget exhausted";return {};}
        return result;
    }
};
}


