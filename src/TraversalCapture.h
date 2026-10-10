#pragma once
#include "Core.h"
#include <cstring>
#include <iomanip>
#include <limits>
#include <vector>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>

namespace fc {

class TraversalCapture {
public:
    static constexpr std::size_t capacity=8192;
    static constexpr std::size_t maxTextBytes=4*1024*1024;
    static constexpr std::string_view coreVersion="active31-6";
    enum class Kind {ray,body,bodyPath};
    struct Call {
        Kind kind=Kind::ray;
        Vec from{},to{},outward{},midpoint{};
        Hit hit{};
        Motion motion=Motion::none;
        float fromPhase{},toPhase{};
        bool hasHit{},answer{};
    };
    struct ReplayReport {
        bool matched{};
        std::size_t callsConsumed{};
        std::string error;
    };

    class SessionGate {
        unsigned captures{};
        float previousTime{},heldSeconds{},pendingTime{};
        std::array<Vec,4> positions{},directions{};
        Vec pendingPosition{},pendingDirection{};
        bool pending{};
    public:
        void reset(){captures=0;previousTime=heldSeconds=pendingTime=0;positions={};directions={};pendingPosition=pendingDirection={};pending=false;}
        unsigned used()const{return captures;}
        bool arm(const Traversal& before,Input input,float elapsed,float dt) {
            pending=false;
            if(!std::isfinite(dt)||dt<=1e-6f)return false;
            dt=std::min(dt,.05f);
            const bool held=before.geometryHolding();
            heldSeconds=held&&!before.resting()?std::min(1.5f,heldSeconds+dt):0;
            if(captures>=positions.size()||!before.active()||input.release||input.backDrop||
                before.resting()||!std::isfinite(before.stalledSeconds())||
                (held?!before.geometryRetryDue(dt):before.stalledSeconds()<=.35f)||
                !std::isfinite(elapsed)||elapsed<0||!before.position.finite()||
                !std::isfinite(input.x)||!std::isfinite(input.y)||(!held&&std::abs(input.x)+std::abs(input.y)<=.1f))return false;
            if(captures&&elapsed-previousTime<1.f)return false;
            if(captures==positions.size()-1&&(held?heldSeconds:before.stalledSeconds())<1.5f)return false;
            const Vec direction=std::abs(input.x)+std::abs(input.y)<=.1f?Vec{}:
                Vec{std::clamp(input.x,-1.f,1.f),std::clamp(input.y,-1.f,1.f),0}.unit();
            for(unsigned index=0;index<captures;++index)
                if((before.position-positions[index]).length()<=24.f&&
                    (direction.dot(directions[index])>=.95f||(direction.length()==0&&directions[index].length()==0)))return false;
            pendingPosition=before.position;pendingDirection=direction;pendingTime=elapsed;pending=true;return true;
        }
        bool commit(bool queried) {
            const bool accepted=pending&&queried;pending=false;
            if(!accepted)return false;
            positions[captures]=pendingPosition;directions[captures]=pendingDirection;
            ++captures;previousTime=pendingTime;return true;
        }
    };
private:
    struct Snapshot {
        Traversal value;
        std::array<char,96> blocked{},ledge{};
        void save(const Traversal& source) {
            value=source;copyText(blocked,source.blockedReason);copyText(ledge,source.ledgeReason);bindReasons();
        }
        void bindReasons(){value.blockedReason=blocked.data();value.ledgeReason=ledge.data();}
    };
    Snapshot before_,after_;
    Input input_{};
    float dt_{},stamina_{};
    Result result_{};
    std::array<char,96> resultReason_{};

    std::vector<Call> calls_=std::vector<Call>(capacity);
    std::size_t count_{},observed_{};
    bool begun_{},finished_{};

    template<std::size_t N> static void copyText(std::array<char,N>& out,const char* text) {
        const auto size=std::min(text?std::strlen(text):std::size_t{},N-1);
        if(size)std::memcpy(out.data(),text,size);out[size]=0;
    }
    static bool same(Vec a,Vec b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
    void record(Call call) {
        if(!begun_||finished_)return;
        if(count_<capacity)calls_[count_++]=call;
        if(observed_<std::numeric_limits<std::size_t>::max())++observed_;
    }
    struct Writer {
        std::ostream& stream;
        template<class T> void operator()(T& value) {
            if constexpr(std::is_enum_v<T>)stream<<' '<<static_cast<int>(value);
            else stream<<' '<<value;
        }
        void operator()(Vec& value){(*this)(value.x);(*this)(value.y);(*this)(value.z);}
    };
    struct Reader {
        std::istream& stream;
        template<class T> void operator()(T& value) {
            if constexpr(std::is_same_v<T,bool>) {
                int number{};if(!(stream>>number)||(number!=0&&number!=1))throw std::runtime_error("invalid bool");value=number!=0;
            } else if constexpr(std::is_enum_v<T>) {
                int number{};if(!(stream>>number))throw std::runtime_error("missing enum");
                constexpr int high=std::is_same_v<T,State>?int(State::action):std::is_same_v<T,Motion>?motionCount:
                    std::is_same_v<T,AttachFailure>?int(AttachFailure::clearance):1;
                if(number<0||number>high)throw std::runtime_error("enum out of range");
                if constexpr(std::is_same_v<T,Motion>)
                    if(number!=0&&!isActiveMotion(static_cast<Motion>(number)))throw std::runtime_error("retired motion is not replayable");
                value=static_cast<T>(number);
            } else {
                if(!(stream>>value))throw std::runtime_error("missing number");
                if constexpr(std::is_floating_point_v<T>)if(!std::isfinite(value))throw std::runtime_error("nonfinite number");
            }
        }
        void operator()(Vec& value){(*this)(value.x);(*this)(value.y);(*this)(value.z);}
    };
    template<class Visitor> static void authoredFields(Visitor& v,AuthoredMotion& motion) {
        v(motion.enabled);v(motion.seconds);v(motion.stride);v(motion.trajectory.count);
        for(auto& knot:motion.trajectory.knots){v(knot.phase);v(knot.displacement);}
        for(auto& sample:motion.contacts)for(auto& value:sample)v(value);
    }
    template<class Visitor> static void fields(Visitor& v,Traversal& t,bool wallRunSetting=true,bool legacyArc=false,bool wallRunSequences=true,bool mantleRoutes=true,bool contextSequences=true,bool holds=true,bool recovery=true) {
        auto& s=t.cfg;
        v(s.reach);v(s.gap);v(s.radius);v(s.height);v(s.chest);v(s.grip);
        v(s.climbSpeed);v(s.sideSpeed);v(s.downSpeed);v(s.maxNormalZ);
        v(s.drain);v(s.hangDrain);v(s.startStamina);v(s.mantleCost);v(s.mantleSeconds);
        v(s.approachSeconds);v(s.runSpeed);v(s.diagonalRunMultiplier);v(s.hopOut);v(s.kickOut);v(s.fancyJumps);
        v(t.state);v(t.position);v(t.normal);v(t.surfaceNormal);v(t.cooldown);v(t.lastAttachDistance);v(t.lastFailure);
        bool hit=t.blockedHit.has_value();v(hit);
        if(hit){if(!t.blockedHit)t.blockedHit=Hit{};v(t.blockedHit->point);v(t.blockedHit->normal);v(t.blockedHit->climbable);}
        else t.blockedHit.reset();
        v(t.blockedFrom);v(t.blockedTo);
        v(t.mantleFrom);v(t.mantleApex);v(t.mantleTo);v(t.mantleLip);v(t.approachFrom);v(t.approachTo);
        for(auto& value:t.mantleHands)v(value);for(auto& value:t.mantleHandNormals)v(value);
        v(t.mantleTime);v(t.approachTime);v(t.ledgePull);v(t.roundedMantle);
        v(t.actionFrom);v(t.actionTo);v(t.actionTime);v(t.actionSeconds);v(t.actionCooldown);v(t.stalled);
        v(t.clearanceMargin);v(t.runBlend);v(t.diagonalRunBlend);v(t.hopDistance);v(t.actionMotion);v(t.entryMotion);v(t.stableMotion);
        v(t.running);v(t.jumpEntry);v(t.detour);v(t.actionBeganRunning);
        v(t.authoredActionChecked);v(t.authoredActionRejected);v(t.authoredBackFlipPrechecked);v(t.entryPose);
        v(t.actionDirection);v(t.actionLandingNormal);v(t.missingSurface);v(t.runClearanceCooldown);
        v(t.moveDirection);v(t.detourOut);v(t.detourOver);v(t.detourProbe);
        if(legacyArc) {
            bool retiredArc{};v(retiredArc);
            if(retiredArc)throw std::runtime_error("retired motion arc is not replayable");
        }
        v(t.roofTransfer);v(t.actionStartSurface);v(t.actionTargetSurface);v(t.roofProbeCooldown);
        v(t.mantleCrest);
        v(s.contextActions);v(s.contextScale);v(t.edgeAction);v(t.edgeSettled);v(t.edgeProbeCooldown);
        for(auto* edge:{&t.actionSourceEdge,&t.actionTargetEdge}) {
            v(edge->center);v(edge->normal);for(auto& hand:edge->hands)v(hand);v(edge->wallPatch);
        }
        auto& ep=t.edgePreparation;
        v(ep.active);v(ep.motion);v(ep.from);v(ep.to);v(ep.destination);v(ep.direction);v(ep.settle);v(ep.status);
        for(auto* edge:{&ep.source,&ep.target}) {
            v(edge->center);v(edge->normal);for(auto& hand:edge->hands)v(hand);v(edge->wallPatch);
        }
        v(t.cornerActive);v(t.cornerProbeCooldown);auto& route=t.cornerRoute;
        v(route.join);v(route.sourceNormal);v(route.targetNormal);v(route.count);v(route.distance);
        v(route.length);v(route.sideSign);v(route.convex);
        for(auto& point:route.points)v(point);for(auto& normal:route.normals)v(normal);
        v(s.threepeatAnimations);v(s.threepeatHangHeight);v(s.threepeatHandHalfWidth);v(s.threepeatHangForward);
        for(auto& value:s.threepeatHopDistance)v(value);for(auto& value:s.threepeatHopSeconds)v(value);
        v(s.threepeatMantleHeight);v(s.threepeatMantleSeconds);v(s.threepeatMantlePalmHeight);v(s.threepeatMantleHalfWidth);v(s.threepeatMantleForward);
        for(auto& value:s.threepeatMantleReplant)v(value);
        v(t.threepeatMantle);
        v(t.threepeatPlanStatus);
        for(auto* retry:{&t.topSearchRetry,&t.hopSearchRetry}) {
            v(retry->remaining);v(retry->position);v(retry->normal);v(retry->input);v(retry->mode);v(retry->valid);v(retry->standingPath);
        }
        v(s.automaticClimbActions);v(s.autoActionMinSeconds);v(s.autoActionMaxSeconds);
        v(t.automaticRandomState);v(t.automaticActions);v(t.automaticElapsed);v(t.automaticInterval);
        v(t.automaticAttempts);v(t.automaticBlocked);v(t.automaticRetry);
        v(s.surfaceActionVariants);v(t.automaticOpportunityRetry);v(t.automaticOpportunities);v(t.surfaceActions);v(t.contextIdles);
        v(s.groundJumpHeight);v(t.approachLift);v(t.entrySeconds);v(t.approachRounded);v(t.entryUsedTopFallback);v(t.entryTopRise);
        v(t.automaticDirection);v(t.automaticPreparation);
        v(s.wallRunObstacleJumps);v(t.obstacleJump);v(t.obstacleJumps);v(t.obstacleProbeCooldown);v(t.actionRunSpeed);
        for(auto& toe:s.threepeatHangToes)v(toe);
        v(s.legacyAutomaticHops);v(s.staminaEnabled);v(s.contextualMantleEnabled);
        for(auto& weight:s.automaticSideWeights)v(weight);
        auto& profile=s.threepeatProfile;
        for(auto& count:profile.pathCounts)v(count);
        for(auto& side:profile.paths)for(auto& knot:side){v(knot.phase);v(knot.travel);v(knot.lift);v(knot.out);}
        for(auto* weights:{&profile.source,&profile.target})for(auto& side:*weights)for(auto& hand:side)for(auto& edge:hand)v(edge);
        for(auto& hand:profile.mantleRelease)for(auto& edge:hand)v(edge);
        for(auto& edge:profile.mantleUnplant)v(edge);for(auto& edge:profile.mantleReplant)v(edge);
        v(profile.replantSamplePhase);for(auto& side:profile.rise)for(auto& edge:side)v(edge);
        v(s.authoredMantle);v(s.authoredMantleTrajectory.count);
        for(auto& knot:s.authoredMantleTrajectory.knots){v(knot.phase);v(knot.displacement);}
        for(auto& sample:s.authoredMantleContacts)for(auto& value:sample)v(value);
        bool authored=bool(s.authoredMotions);v(authored);
        if(authored) {
            auto data=std::make_shared<std::array<AuthoredMotion,42>>();
            if constexpr(std::is_same_v<Visitor,Writer>)*data=*s.authoredMotions;
            for(auto& motion:*data)authoredFields(v,motion);
            if constexpr(std::is_same_v<Visitor,Reader>)s.authoredMotions=std::move(data);
        } else if constexpr(std::is_same_v<Visitor,Reader>)s.authoredMotions.reset();
        if(wallRunSetting)v(s.wallRunEnabled);
        else if constexpr(std::is_same_v<Visitor,Reader>)s.wallRunEnabled=true;
        if(wallRunSequences) {
            v(t.actionWallRunDirection);bool present=bool(s.authoredWallRunSequences);v(present);
            if(present) {
                auto data=std::make_shared<AuthoredWallRunSequences>();
                if constexpr(std::is_same_v<Visitor,Writer>)*data=*s.authoredWallRunSequences;
                for(auto& valid:data->valid)v(valid);
                for(auto& motion:data->launches)authoredFields(v,motion);
                for(auto& motion:data->catches)authoredFields(v,motion);
                if constexpr(std::is_same_v<Visitor,Reader>)s.authoredWallRunSequences=std::move(data);
            } else if constexpr(std::is_same_v<Visitor,Reader>)s.authoredWallRunSequences.reset();
        } else if constexpr(std::is_same_v<Visitor,Reader>) {
            t.actionWallRunDirection=Motion::none;s.authoredWallRunSequences.reset();
        }
        if(mantleRoutes) {
            v(t.mantleRoute.count);
            for(auto& knot:t.mantleRoute.knots){v(knot.phase);v(knot.displacement);}
        } else if constexpr(std::is_same_v<Visitor,Reader>)t.mantleRoute={};
        if(contextSequences) {
            bool present=bool(s.contextHopReferences);v(present);
            if(present) {
                auto data=std::make_shared<std::array<ContextHopReference,2>>();
                if constexpr(std::is_same_v<Visitor,Writer>)*data=*s.contextHopReferences;
                for(auto& reference:*data) {
                    v(reference.valid);v(reference.height);v(reference.halfWidth);v(reference.forward);
                    for(auto& toe:reference.toes)v(toe);
                    authoredFields(v,reference.preparation);authoredFields(v,reference.recovery);
                }
                if constexpr(std::is_same_v<Visitor,Reader>)s.contextHopReferences=std::move(data);
            } else if constexpr(std::is_same_v<Visitor,Reader>)s.contextHopReferences.reset();
        } else if constexpr(std::is_same_v<Visitor,Reader>)s.contextHopReferences.reset();
        if(holds) {v(t.geometryHeld);v(t.geometryRetry);v(t.staminaResting);}
        else if constexpr(std::is_same_v<Visitor,Reader>) {t.geometryHeld=t.staminaResting=false;t.geometryRetry=0;}
        if(contextSequences&&s.authoredWallRunSequences) {
            auto data=std::make_shared<AuthoredWallRunSequences>(*s.authoredWallRunSequences);
            for(auto& motion:data->braces)authoredFields(v,motion);
            if constexpr(std::is_same_v<Visitor,Reader>)s.authoredWallRunSequences=std::move(data);
        }
        if(recovery) {
            v(t.supportRecoveryRetry);v(t.supportRecoveryOrigin);v(t.supportRecoveryInput);
            v(t.supportRecoveryCursor);v(t.supportRecoveryAction);
        } else if constexpr(std::is_same_v<Visitor,Reader>) {
            t.supportRecoveryRetry=0;t.supportRecoveryOrigin=t.supportRecoveryInput={};
            t.supportRecoveryCursor=0;t.supportRecoveryAction=false;
        }
    }
    template<class Visitor> static void inputFields(Visitor& v,Input& input) {
        v(input.x);v(input.y);v(input.release);v(input.mantle);v(input.hop);v(input.backDrop);v(input.run);v(input.modeBlend);
    }
    template<class Visitor> static void resultFields(Visitor& v,Result& result) {
        v(result.released);v(result.motion);v(result.staminaCost);v(result.completed);v(result.releaseVelocity);
    }
    static void configure(std::ostream& out){out.imbue(std::locale::classic());out<<std::setprecision(std::numeric_limits<float>::max_digits10);}
    static void token(std::istream& in,std::string_view expected) {
        std::string found;if(!(in>>found)||found!=expected)throw std::runtime_error("expected "+std::string(expected));
    }
    template<std::size_t N> static void text(std::istream& in,std::array<char,N>& destination) {
        std::string value;if(!(in>>std::quoted(value))||value.size()>=N||value.find_first_of("\r\n")!=std::string::npos)
            throw std::runtime_error("invalid reason string");
        copyText(destination,value.c_str());
    }
    static void writeSnapshot(std::ostream& out,std::string_view label,const Snapshot& snapshot) {
        out<<label;auto value=snapshot.value;Writer writer{out};fields(writer,value);
        out<<' '<<std::quoted(snapshot.blocked.data())<<' '<<std::quoted(snapshot.ledge.data())<<'\n';
    }
    static void readSnapshot(std::istream& in,std::string_view label,Snapshot& snapshot,bool wallRunSetting,bool legacyArc,bool wallRunSequences,bool mantleRoutes,bool contextSequences,bool holds,bool recovery) {
        token(in,label);Reader reader{in};fields(reader,snapshot.value,wallRunSetting,legacyArc,wallRunSequences,mantleRoutes,contextSequences,holds,recovery);
        if(snapshot.value.supportRecoveryRetry<0||snapshot.value.supportRecoveryRetry>.401f||snapshot.value.supportRecoveryCursor>11||
            snapshot.value.supportRecoveryInput.z!=0||snapshot.value.supportRecoveryInput.length()>1.001f)
            throw std::runtime_error("invalid support recovery state");
        if(!validThreepeatProfile(snapshot.value.cfg.threepeatProfile))throw std::runtime_error("invalid animation profile");
        const auto& settings=snapshot.value.cfg;
        if(!settings.authoredMantleTrajectory.valid()||!snapshot.value.mantleRoute.valid())
            throw std::runtime_error("invalid authored trajectory");
        for(const auto& sample:settings.authoredMantleContacts)for(float value:sample)
            if(!std::isfinite(value)||value<0||value>1)throw std::runtime_error("invalid authored support weights");
        if(settings.authoredMotions)for(std::size_t index=0;index<settings.authoredMotions->size();++index) {
            const auto& motion=(*settings.authoredMotions)[index];
            if(motion.enabled&&!isActiveMotion(Motion(index+1)))throw std::runtime_error("retired authored motion is not replayable");
            if(!motion.trajectory.valid()||(motion.enabled&&(motion.seconds<=0||motion.stride<0||motion.stride>500)))throw std::runtime_error("invalid authored motion");
            for(const auto& sample:motion.contacts)for(float value:sample)
                if(!std::isfinite(value)||value<0||value>1)throw std::runtime_error("invalid authored motion contacts");
        }
        if(snapshot.value.actionWallRunDirection!=Motion::none&&!runMotion(snapshot.value.actionWallRunDirection))
            throw std::runtime_error("invalid wall-run action direction");
        if(settings.contextHopReferences)for(const auto& reference:*settings.contextHopReferences) {
            if(reference.valid&&(reference.height<=90||reference.height>=175||reference.halfWidth<=8||reference.halfWidth>=40||
                std::abs(reference.forward)>500||!reference.toes[0].finite()||!reference.toes[1].finite()))
                throw std::runtime_error("invalid side-hop reference geometry");
            for(const auto* motion:{&reference.preparation,&reference.recovery}) {
                if(!motion->trajectory.valid()||(reference.valid&&motion->seconds<=0))throw std::runtime_error("invalid side-hop reference motion");
                for(const auto& sample:motion->contacts)for(float value:sample)if(value<0||value>1)throw std::runtime_error("invalid side-hop reference contacts");
            }
        }
        if(settings.authoredWallRunSequences)for(const auto* group:{&settings.authoredWallRunSequences->launches,&settings.authoredWallRunSequences->catches,&settings.authoredWallRunSequences->braces})
            for(const auto& motion:*group) {
                if(!motion.trajectory.valid()||(motion.enabled&&(motion.seconds<=0||motion.stride<0||motion.stride>500)))
                    throw std::runtime_error("invalid wall-run authored motion");
                for(const auto& sample:motion.contacts)for(float value:sample)
                    if(value<0||value>1)throw std::runtime_error("invalid wall-run authored contacts");
            }
        text(in,snapshot.blocked);text(in,snapshot.ledge);snapshot.bindReasons();
    }
    static std::string snapshotText(const Traversal& traversal) {
        Snapshot snapshot;snapshot.save(traversal);std::ostringstream out;configure(out);writeSnapshot(out,"STATE",snapshot);return out.str();
    }
public:
    TraversalCapture()=default;
    TraversalCapture(const TraversalCapture&)=delete;
    TraversalCapture& operator=(const TraversalCapture&)=delete;
    void begin(const Traversal& before,Input input,float dt,float stamina) {
        before_.save(before);input_=input;dt_=dt;stamina_=stamina;
        count_=observed_=0;begun_=true;finished_=false;
    }
    void finish(const Traversal& after,const Result& result) {
        if(!begun_)return;
        after_.save(after);result_=result;copyText(resultReason_,result.reason);result_.reason=resultReason_.data();finished_=true;
    }
    bool complete()const{return begun_&&finished_&&observed_==count_;}
    std::size_t count()const{return count_;}
    std::size_t observed()const{return observed_;}
    const Call& call(std::size_t index)const{return calls_.at(index);}
    const Traversal& before()const{return before_.value;}
    const Traversal& after()const{return after_.value;}
    const Result& result()const{return result_;}
    class RecordingWorld final:public World {
        World& source;TraversalCapture& capture;
    public:
        RecordingWorld(World& source_,TraversalCapture& capture_):source(source_),capture(capture_){}
        std::optional<Hit> ray(Vec from,Vec to)override {
            const auto result=source.ray(from,to);Call call;call.from=from;call.to=to;call.hasHit=result.has_value();
            if(result)call.hit=*result;capture.record(call);return result;
        }
        bool actionBodyClear(Motion motion,Vec from,Vec to,float a,float b,Vec outward)override {
            const bool result=source.actionBodyClear(motion,from,to,a,b,outward);Call call;
            call.kind=Kind::body;call.motion=motion;call.from=from;call.to=to;call.fromPhase=a;call.toPhase=b;call.outward=outward;call.answer=result;
            capture.record(call);return result;
        }
        bool actionBodyPathClear(Motion motion,Vec from,Vec to,float a,float b,Vec outward,Vec midpoint)override {
            const bool result=source.actionBodyPathClear(motion,from,to,a,b,outward,midpoint);Call call;
            call.kind=Kind::bodyPath;call.motion=motion;call.from=from;call.to=to;call.fromPhase=a;call.toPhase=b;call.outward=outward;call.midpoint=midpoint;call.answer=result;
            capture.record(call);return result;
        }
    };
    std::string serialize()const {
        if(!begun_||!finished_)return {};
        std::ostringstream out;configure(out);
        out<<"FCGEO_BEGIN 1 "<<coreVersion<<'\n'<<"META "<<count_<<' '<<observed_<<' '<<complete()<<'\n';
        out<<"INPUT";Writer writer{out};auto input=input_;inputFields(writer,input);out<<' '<<dt_<<' '<<stamina_<<'\n';
        writeSnapshot(out,"BEFORE",before_);writeSnapshot(out,"AFTER",after_);
        out<<"RESULT";auto result=result_;resultFields(writer,result);out<<' '<<std::quoted(resultReason_.data())<<'\n';
        for(std::size_t i=0;i<count_;++i) {
            auto call=calls_[i];out<<(call.kind==Kind::ray?"R":call.kind==Kind::body?"B":"P");writer(call.from);writer(call.to);
            if(call.kind==Kind::ray){writer(call.hasHit);if(call.hasHit){writer(call.hit.point);writer(call.hit.normal);writer(call.hit.climbable);}}
            else {writer(call.motion);writer(call.fromPhase);writer(call.toPhase);writer(call.outward);if(call.kind==Kind::bodyPath)writer(call.midpoint);writer(call.answer);}
            out<<'\n';
        }
        out<<"FCGEO_END\n";return out.str();
    }

    bool deserialize(std::string_view data,std::string& error) {
        begun_=finished_=false;count_=observed_=0;error.clear();
        try {
            if(data.size()>maxTextBytes)throw std::runtime_error("capture text exceeds bounded size");
            const auto begin=data.find("FCGEO_BEGIN ");if(begin==data.npos)throw std::runtime_error("capture marker missing");
            const auto end=data.find("FCGEO_END",begin);if(end==data.npos)throw std::runtime_error("capture end missing");
            if(end+9<data.size()&&data[end+9]!='\r'&&data[end+9]!='\n')throw std::runtime_error("invalid capture end marker");
            std::istringstream in(std::string(data.substr(begin,end-begin+9)));in.imbue(std::locale::classic());
            token(in,"FCGEO_BEGIN");int schema{};std::string version;
            if(!(in>>schema>>version)||schema!=1||(version!=coreVersion&&version!="active31-5"&&version!="active31-4"&&version!="active31-3"&&version!="active31-2"&&version!="active31-1"&&version!="active35-9"&&version!="active35-10"))throw std::runtime_error("unsupported capture/Core version");
            token(in,"META");bool markedComplete{};Reader reader{in};reader(count_);reader(observed_);reader(markedComplete);
            if(count_>capacity||observed_<count_||markedComplete!=(count_==observed_))throw std::runtime_error("invalid call count");
            token(in,"INPUT");inputFields(reader,input_);reader(dt_);reader(stamina_);
            const bool legacyArc=version=="active35-9"||version=="active35-10";
            const bool recovery=version==coreVersion,holds=recovery||version=="active31-5";
            const bool context=holds||version=="active31-4",routes=context||version=="active31-3";
            const bool sequences=routes||version=="active31-2";
            readSnapshot(in,"BEFORE",before_,version!="active35-9",legacyArc,sequences,routes,context,holds,recovery);
            readSnapshot(in,"AFTER",after_,version!="active35-9",legacyArc,sequences,routes,context,holds,recovery);
            token(in,"RESULT");resultFields(reader,result_);text(in,resultReason_);result_.reason=resultReason_.data();
            for(std::size_t i=0;i<count_;++i) {
                auto& call=calls_[i];call=Call{};std::string type;if(!(in>>type)||(type!="R"&&type!="B"&&type!="P"))throw std::runtime_error("unknown World call");
                reader(call.from);reader(call.to);
                if(type=="R"){reader(call.hasHit);if(call.hasHit){reader(call.hit.point);reader(call.hit.normal);reader(call.hit.climbable);}}
                else {call.kind=type=="P"?Kind::bodyPath:Kind::body;reader(call.motion);reader(call.fromPhase);reader(call.toPhase);reader(call.outward);if(call.kind==Kind::bodyPath)reader(call.midpoint);reader(call.answer);}
            }
            token(in,"FCGEO_END");begun_=finished_=true;return true;
        }catch(const std::exception& e){error=e.what();begun_=finished_=false;return false;}
    }
    class ReplayWorld final:public World {
        const TraversalCapture& capture;
        std::size_t index{};
        const Call& next(Kind kind,Vec from,Vec to) {
            if(!capture.complete()||index>=capture.count_)throw std::runtime_error("World tape exhausted/incomplete at call "+std::to_string(index));
            const auto& call=capture.calls_[index];
            if(call.kind!=kind||!same(call.from,from)||!same(call.to,to))throw std::runtime_error("World query mismatch at call "+std::to_string(index));
            ++index;return call;
        }
    public:
        explicit ReplayWorld(const TraversalCapture& capture_):capture(capture_){}
        std::size_t consumed()const{return index;}
        std::optional<Hit> ray(Vec from,Vec to)override {
            const auto& call=next(Kind::ray,from,to);return call.hasHit?std::optional<Hit>{call.hit}:std::nullopt;
        }
        bool actionBodyClear(Motion motion,Vec from,Vec to,float a,float b,Vec outward)override {
            const auto& call=next(Kind::body,from,to);
            if(call.motion!=motion||call.fromPhase!=a||call.toPhase!=b||!same(call.outward,outward))
                throw std::runtime_error("body query mismatch at call "+std::to_string(index-1));
            return call.answer;
        }
        bool actionBodyPathClear(Motion motion,Vec from,Vec to,float a,float b,Vec outward,Vec midpoint)override {
            const auto& call=next(Kind::bodyPath,from,to);
            if(call.motion!=motion||call.fromPhase!=a||call.toPhase!=b||!same(call.outward,outward)||!same(call.midpoint,midpoint))
                throw std::runtime_error("body path query mismatch at call "+std::to_string(index-1));
            return call.answer;
        }
    };
    ReplayReport replay()const {
        ReplayReport report;ReplayWorld world(*this);
        try {
            if(!complete())throw std::runtime_error("capture incomplete; exact replay refused");
            auto traversal=before_.value;const auto result=traversal.update(world,input_,dt_,stamina_);
            if(world.consumed()!=count_)throw std::runtime_error("unused recorded World calls");
            if(snapshotText(traversal)!=snapshotText(after_.value))throw std::runtime_error("final Traversal state differs");
            if(result.released!=result_.released||result.motion!=result_.motion||result.staminaCost!=result_.staminaCost||
                result.completed!=result_.completed||!same(result.releaseVelocity,result_.releaseVelocity)||
                std::string_view(result.reason)!=resultReason_.data())throw std::runtime_error("final Result differs");
            report.matched=true;
        }catch(const std::exception& e){report.error=e.what();}
        report.callsConsumed=world.consumed();return report;
    }
};
}
