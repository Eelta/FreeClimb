#pragma once

    bool capturedHangFeetSupported(World& w,Vec origin,const GripEdge& edge) const {
        const float scale=std::clamp(cfg.contextScale,.5f,2.f);
        const Vec right{-normal.y,normal.x,0};
        for(const auto calibrated:contextHangToes()) {
            if(!calibrated.finite()||calibrated.z<10||calibrated.z>100)return false;
            const Vec toe=origin+(right*calibrated.x-normal*(calibrated.y+
                cfg.gap/scale+(edge.wallPatch?-capturedWallPalmOffset:gripEdgeDetail::palmInset)-contextHangForward())+Vec{0,0,calibrated.z})*scale;
            for(const auto offset:{Vec{},right*(2*scale),right*(-2*scale),Vec{0,0,2*scale},Vec{0,0,-2*scale}}) {
                const Vec from=toe+offset;
                const auto hit=w.ray(from+normal*(14*scale),from-normal*(20*scale));
                if(!hit||!hit->climbable||!hit->normal.finite()||hit->normal.unit().dot(normal)<.95f||
                    std::abs(3*scale-(from-hit->point).dot(hit->normal.unit()))>=10*scale)return false;
            }
        }
        return true;
    }
    bool idle39FeetSupported(World& w,Vec origin,const GripEdge& edge) const {

        const auto a=contextHangToes()[0],b=contextHangToes()[1];
        const bool geometryOnly=a.x==0&&a.y==0&&a.z==0&&b.x==0&&b.y==0&&b.z==0;
        return geometryOnly?edgeFeetSupported(w,origin,edge.normal):
            capturedHangFeetSupported(w,origin,edge);
    }
    Motion stationaryMotion() const {
        return edgeSettled&&threepeatHop(actionMotion)?Motion::contextHang:Motion::hang;
    }
    bool prepareWallAction(World& world,Motion motion,Vec direction) {
        if(!cfg.surfaceActionVariants||!cfg.contextActions||running||std::abs(surfaceNormal.z)>.10f)return false;
        if(!cfg.threepeatAnimations||!capturedSideIntent(direction))return false;
        motion=direction.x<0?Motion::contextHopLeft:Motion::contextHopRight;

        struct Budget final:World {
            World& source;unsigned calls{};bool exhausted{};
            explicit Budget(World& value):source(value){}
            std::optional<Hit> ray(Vec a,Vec b) override {
                if(calls++>=3072){exhausted=true;return Hit{a,{0,0,1},false};}
                return source.ray(a,b);
            }
        } w(world);
        const float scale=std::clamp(cfg.contextScale,.5f,2.f);
        const float height=contextHangHeight(motion)*scale;
        const float halfSpan=contextHangHalfWidth(motion)*scale;
        const Vec right{-normal.y,normal.x,0};
        const float distance=cfg.threepeatHopDistance[direction.x<0?0:1]*scale;
        const Vec destination=position+right*(direction.x<0?-distance:distance)+Vec{0,0,capturedSideRise(direction,distance,scale)};
        if(!capturedSideRouteMatches(direction,destination-position,normal,scale))return false;
        edgePreparation.status=1;
        const auto source=findWallGrip(w,position,normal,cfg,height,halfSpan);
        if(!source)return false;
        edgePreparation.status=2;
        const auto target=findWallGrip(w,destination,normal,cfg,height,halfSpan);
        if(!target)return false;
        edgePreparation.status=4;
        const auto destinationSupport=support(w,destination,normal*-1);
        if(!destinationSupport||horizontal(destinationSupport->normal).dot(normal)<.985f||
            (offsetFromSurface(destination,*destinationSupport)-destination).length()>1.25f)return false;
        edgePreparation.status=5;
        if(!edgeFeetSupported(w,position,normal)||!edgeFeetSupported(w,destination,normal))return false;
        edgePreparation.status=6;
        if(!edgeRouteClear(w,motion,position,destination)||w.exhausted)return false;
        EdgePreparation plan;plan.active=true;plan.motion=motion;plan.from=plan.to=position;
        plan.destination=destination;plan.direction=direction;plan.source=*source;plan.target=*target;plan.status=7;
        edgePreparation=plan;edgeAction=edgeSettled=false;
        threepeatPlanStatus=7;
        return true;
    }
    std::optional<Result> tryAutomaticCaptured(World& world,Input input,bool wallFallback) {
        const bool lateral=capturedSideIntent({input.x,input.y,0});
        if(!cfg.surfaceActionVariants||!lateral||!cfg.threepeatAnimations)return {};
        struct Budget final:World {
            World& source;unsigned calls{};bool exhausted{};
            explicit Budget(World& value):source(value){}
            std::optional<Hit> ray(Vec a,Vec b) override {
                if(calls++>=4096){exhausted=true;return Hit{a,{0,0,1},false};}
                return source.ray(a,b);
            }
        } w(world);
        const auto saved=*this;
        const Motion motion=input.x<0?Motion::hopLeft:Motion::hopRight;
        automaticPreparation=true;
        bool prepared=prepareEdgeAction(w,motion,{input.x,input.y,0},true);
        threepeatPlanStatus=edgePreparation.status;
        if(!prepared&&wallFallback&&!w.exhausted)prepared=prepareWallAction(w,motion,{input.x,input.y,0});
        if(!prepared||w.exhausted) {
            const auto edgeStatus=edgePreparation.status,threepeatStatus=threepeatPlanStatus;
            *this=saved;edgePreparation.status=edgeStatus;threepeatPlanStatus=threepeatStatus;
            return {};
        }
        cornerActive=false;
        Result result;result.motion=state==State::action?actionMotion:stableMotion;
        result.staminaCost=state==State::action?15.f:0;
        result.reason=edgePreparation.active&&edgePreparation.source.wallPatch?"automatic captured wall contact":"automatic captured edge opportunity";
        return result;
    }
    Vec threepeatHopPoint(Motion motion,Vec from,Vec to,float phase) const {
        const bool left=motion==Motion::contextHopLeft;
        const float scale=std::clamp(cfg.contextScale,.5f,2.f);
        const Vec displacement=to-from;

        const float rise=threepeatWindow(phase,cfg.threepeatProfile.rise[left?0:1]);
        return from+Vec{displacement.x,displacement.y,0}*threepeatHopTravel(left,phase,cfg.threepeatProfile)+
            normal*(threepeatHopOut(left,phase,cfg.threepeatProfile)*scale)+
            Vec{0,0,threepeatHopLift(left,phase,cfg.threepeatProfile)*scale+displacement.z*rise};
    }
    bool selectThreepeatMantle(World& w) {
        mantleSelectionStatus=1;
        if(!cfg.contextualMantleEnabled||!cfg.threepeatAnimations||!cfg.contextActions||cfg.threepeatMantleSeconds<=0)return false;
        mantleSelectionStatus=2;
        if(mantleCrest||std::abs(surfaceNormal.z)>.10f)return false;
        mantleSelectionStatus=3;
        if(!edgeFeetSupported(w,position,normal))return false;
        const float scale=std::clamp(cfg.contextScale,.5f,2.f),height=mantleLip.z-position.z;
        mantleSelectionStatus=4;
        if(std::abs(height-cfg.threepeatMantlePalmHeight*scale)>24*scale)return false;
        const Vec displacement=mantleTo-mantleFrom,right{-normal.y,normal.x,0};
        mantleSelectionStatus=5;
        if(std::abs(displacement.dot(normal*-1)-cfg.threepeatMantleForward*scale)>24*scale||
            std::abs(displacement.dot(right))>8*scale)return false;
        mantleSelectionStatus=6;
        const auto edge=findGripEdge(w,position,normal,cfg,height-2,height+2,cfg.threepeatMantleHalfWidth*scale);
        if(!edge||edge->normal.dot(normal)<.985f||(edge->center-mantleLip).length()>6)return false;
        mantleSelectionStatus=7;
        for(int hand=0;hand<2;++hand)
            if(!threepeatReplantContact(w,edge->hands[hand],normal,cfg.threepeatMantleReplant[hand],scale))return false;
        mantleHands=edge->hands;mantleHandNormals={Vec{0,0,1},Vec{0,0,1}};
        mantleSelectionStatus=8;
        return true;
    }
    bool threepeatTopStillValid(World& w,float phase) const {
        const float scale=std::clamp(cfg.contextScale,.5f,2.f);
        const auto floor=standingFloor(w,mantleTo,12,8);
        if(!floor||(*floor-mantleTo).length()>1.25f)return false;
        const bool replanted=topReplanted(phase);
        for(int hand=0;hand<2;++hand)if(topHandWeight(hand,phase)>.05f) {
            if(replanted) {
                if(!threepeatReplantContact(w,mantleHands[hand],normal,topReplantOffset(hand),scale))return false;
            } else {
                const auto hit=w.ray(mantleHands[hand]+Vec{0,0,3*scale},mantleHands[hand]-Vec{0,0,3*scale});
                if(!hit||!hit->climbable||hit->normal.z<.95f||(hit->point-mantleHands[hand]).length()>1.25f*scale)return false;
            }
        }
        return true;
    }
    bool adaptiveTopStillValid(World& w,float phase) const {
        const float scale=std::clamp(cfg.contextScale,.5f,2.f);
        if(!mantleCrest) {
            const float depth=std::max(12.f,9.f+std::max(12.f,cfg.radius*.55f)*1.020205f);
            const auto floor=standingSupport(w,mantleTo,depth,8);
            if(!floor||(*floor-mantleTo).length()>1.25f)return false;
        }
        const float sampled=topSamplePhase(phase);
        for(int hand=0;hand<2;++hand)if(topHandWeight(hand,sampled)>.05f) {
            const Vec normal=mantleHandNormals[hand].unit(),point=mantleHands[hand];
            const auto hit=w.ray(point+normal*(3*scale),point-normal*(3*scale));
            if(!hit||!hit->climbable||!hit->normal.finite()||hit->normal.unit().dot(normal)<.95f||
                !hit->point.finite()||(hit->point-point).length()>1.25f*scale)return false;
        }
        return true;
    }
    bool edgeFeetSupported(World& w,Vec base,Vec facing) const {
        const auto start=base+Vec{0,0,25};
        const auto foot=w.ray(start,start-facing*(cfg.gap+12));
        return foot&&foot->climbable&&foot->normal.unit().dot(facing)>.95f;
    }
    bool edgeRouteClear(World& w,Motion motion,Vec from,Vec to) {
        if(authoredPath(motion))return checkedHop(w,from,to,cfg.hopOut,motion);
        if(threepeatHop(motion)) {
            Vec previous=from;
            for(int sample=1;sample<=32;++sample) {
                const Vec next=threepeatHopPoint(motion,from,to,sample/32.f);
                if(!clearPath(w,previous,next))return false;
                previous=next;
            }
            return true;
        }
        return checkedHop(w,from,to,cfg.hopOut,motion);
    }
    void commitEdgeAction(const EdgePreparation& plan) {

        const EdgePreparation saved=plan;
        const bool automatic=automaticPreparation;
        beginAction(saved.motion,position,saved.destination,
            threepeatHop(saved.motion)?
                cfg.threepeatHopSeconds[saved.motion==Motion::contextHopLeft?0:1]:jumpActionSeconds(false));
        if(automatic)++automaticActions;
        if(saved.source.wallPatch)++surfaceActions;
        actionSourceEdge=saved.source;actionTargetEdge=saved.target;edgeAction=true;
        actionLandingNormal=saved.target.normal;edgePreparation.active=false;edgePreparation.status=9;
        if(threepeatHop(saved.motion))threepeatPlanStatus=9;
    }
    bool prepareEdgeAction(World& w,Motion motion,Vec direction,bool candidateThreepeat=false) {
        if(!hopMotion(motion))return false;
        const bool legacySide=motion==Motion::hopLeft||motion==Motion::hopRight;
        if(!candidateThreepeat&&cfg.threepeatAnimations&&legacySide&&capturedSideIntent(direction)) {
            const bool prepared=prepareEdgeAction(w,motion,direction,true);
            threepeatPlanStatus=edgePreparation.status;
            if(prepared)return true;
        }
        if(candidateThreepeat) {
            if(!capturedSideIntent(direction))return false;
            motion=motion==Motion::hopLeft?Motion::contextHopLeft:Motion::contextHopRight;
        }
        const float scale=std::clamp(cfg.contextScale,.5f,2.f);
        const float handHeight=(candidateThreepeat?contextHangHeight(motion):138.12f)*scale;
        const float halfWidth=(candidateThreepeat?contextHangHalfWidth(motion):25.14f)*scale;
        const bool sideways=legacySide||threepeatHop(motion);

        if(direction.length()<.1f)direction=motion==Motion::hopLeft?Vec{-1,0,0}:motion==Motion::hopRight?Vec{1,0,0}:Vec{0,1,0};
        const float minShift=-18.f*scale,maxShift=18.f*scale;
        edgePreparation.status=1;
        const auto upper=findGripEdge(w,position,normal,cfg,handHeight+minShift,handHeight+maxShift,halfWidth);
        if(!upper||upper->normal.dot(normal)<.985f)return false;
        const Vec right{-normal.y,normal.x,0};

        const Vec aligned=upper->center+upper->normal*cfg.gap-Vec{0,0,handHeight};
        const Vec alignment=aligned-position;
        edgePreparation.status=3;
        if(alignment.z<minShift-.05f||alignment.z>maxShift+.05f||
            std::abs(alignment.dot(normal))>14.f||std::abs(alignment.dot(right))>2.f||
            !support(w,aligned,normal*-1)||!clearPath(w,position,aligned))return false;
        edgePreparation.status=5;
        if(!edgeFeetSupported(w,aligned,upper->normal))return false;
        const float authoredSide=candidateThreepeat?cfg.threepeatHopDistance[direction.x<0?0:1]:direction.x<0?105.35f:93.74f;
        const std::array<float,4> distances=candidateThreepeat?
            std::array<float,4>{authoredSide,authoredSide*.94f,authoredSide*1.06f,authoredSide*.88f}:
            std::array<float,4>{authoredSide,90.f,78.f,66.f};
        unsigned routeChecks=0;
        edgePreparation.status=2;
        for(unsigned lane=0;lane<(sideways?distances.size():1u);++lane) {
            const Vec reference=sideways?aligned+right*((direction.x<0?-1.f:1.f)*distances[lane]*scale):aligned;
            const bool diagonalCapture=candidateThreepeat&&direction.y>0;
            const float rise=diagonalCapture?capturedSideRise(direction,distances[lane]*scale,scale):0;
            const float lo=diagonalCapture?handHeight+std::max(0.f,rise-8*scale):
                handHeight+(sideways?(direction.y>0?30.f:-12.f):42.f)*scale;
            const float hi=diagonalCapture?handHeight+rise:
                handHeight+(sideways?(direction.y>0?65.f:12.f):74.f)*scale;
            const auto target=findGripEdge(w,reference,normal,cfg,lo,hi,halfWidth);
            if(!target||target->normal.dot(upper->normal)<.985f)continue;
            const Vec delta=target->center-upper->center;
            if(std::abs(delta.dot(normal))>6||(!sideways&&std::abs(delta.dot(right))>2))continue;
            if(candidateThreepeat&&!capturedSideRouteMatches(direction,delta,normal,scale))continue;
            const Vec wanted=aligned+delta;
            edgePreparation.status=4;
            const auto supportHit=support(w,wanted,normal*-1);
            if(!supportHit)continue;
            const auto anchor=landingAnchor(w,wanted,*supportHit);
            if(!anchor||(anchor->position-wanted).length()>3||std::abs(anchor->hit.normal.unit().z)>.12f||
                horizontal(anchor->hit.normal).dot(target->normal)<.985f)continue;
            edgePreparation.status=5;
            if(!edgeFeetSupported(w,anchor->position,target->normal))continue;
            edgePreparation.status=6;

            if(routeChecks++>=2)return false;
            if(!edgeRouteClear(w,motion,aligned,anchor->position))continue;
            EdgePreparation plan;
            plan.active=true;plan.motion=motion;plan.from=position;plan.to=aligned;plan.destination=anchor->position;
            plan.direction=direction;plan.source=*upper;plan.target=*target;plan.status=7;

            if(alignment.length()<.10f&&stableMotion==(candidateThreepeat?Motion::contextHang:Motion::hang)) {
                commitEdgeAction(plan);return true;
            }
            edgePreparation=plan;edgeAction=edgeSettled=false;
            actionMotion=alignment.z<-.1f?Motion::down:alignment.z>.1f?Motion::up:Motion::hang;
            stableMotion=actionMotion;return true;
        }
        return false;
    }
    std::optional<Result> updateEdgePreparation(World& w,Input input,float dt,float stamina) {
        if(!edgePreparation.active)return {};
        auto cancel=[&](unsigned status) {
            if(threepeatMotion(edgePreparation.motion))threepeatPlanStatus=status;
            edgePreparation.active=false;edgePreparation.status=status;edgeProbeCooldown=.16f;
            automaticPreparation=false;resetAutomaticClock();
        };
        const bool changedDirection=(std::abs(input.x)+std::abs(input.y)>.1f)&&
            (Vec{input.x,input.y,0}.dot(edgePreparation.direction)<-.05f);

        const bool automaticCancelled=automaticPreparation&&
            (!cfg.automaticClimbActions||input.hop||std::hypot(input.x,input.y)<.1f||
                Vec{input.x,input.y,0}.unit().dot(edgePreparation.direction.unit())<.85f||
                (input.mantle&&input.y>.1f&&findLedge(w).has_value()));
        if(input.release||input.run||!cfg.contextActions||stamina<15||changedDirection||automaticCancelled) {
            cancel(10);return {};
        }
        if(!gripEdgeStillValid(w,edgePreparation.source)||!gripEdgeStillValid(w,edgePreparation.target)||
            normal.dot(edgePreparation.source.normal)<.985f) {
            cancel(11);return {};
        }
        const Vec remaining=edgePreparation.to-position;
        if(remaining.length()>.03f) {
            const float speed=std::max(1.f,remaining.z<0?cfg.downSpeed:cfg.climbSpeed);
            Vec next=remaining.length()<=speed*dt?edgePreparation.to:position+remaining.unit()*(speed*dt);
            if(!support(w,next,normal*-1)||!clearPath(w,position,next)) {cancel(3);return {};}
            const auto moved=next-position;position=next;missingSurface=0;stalled=0;
            moveDirection={0,moved.z<0?-1.f:moved.z>0?1.f:0.f,0};
            stableMotion=std::abs(moved.z)>.01f?(moved.z<0?Motion::down:Motion::up):Motion::hang;
            edgePreparation.status=7;
            if(threepeatHop(edgePreparation.motion))threepeatPlanStatus=7;
            Result result;result.motion=stableMotion;result.staminaCost=cfg.drain*dt;return result;
        }
        if(!edgeFeetSupported(w,position,edgePreparation.source.normal)) {cancel(5);return {};}
        moveDirection={};stableMotion=threepeatHop(edgePreparation.motion)&&!authored(edgePreparation.motion)?Motion::contextHang:Motion::hang;edgePreparation.status=8;
        if(threepeatHop(edgePreparation.motion))threepeatPlanStatus=8;
        edgePreparation.settle+=dt;
        if(edgePreparation.settle<(authored(edgePreparation.motion)?0.f:threepeatHop(edgePreparation.motion)?.26f:.20f)) {
            Result result;result.motion=stableMotion;return result;
        }

        if(!support(w,edgePreparation.destination,normal*-1)||
            !edgeFeetSupported(w,edgePreparation.destination,edgePreparation.target.normal)||
            !edgeRouteClear(w,edgePreparation.motion,position,edgePreparation.destination)) {
            cancel(6);return {};
        }
        commitEdgeAction(edgePreparation);
        Result result;result.motion=actionMotion;result.staminaCost=15.f;return result;
    }
