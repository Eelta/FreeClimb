#pragma once

class SurfacePose {
    struct ContactPose {Vec nominal{},start{},pole{},normal{};Quat upper{},middle{};float scale{};};
    struct Contact {Vec point{},rejectedPoint{};ContactPose rejectedPose{};float weight{};bool valid{},retiring{},rejected{};};
    std::array<Contact,4> contacts{};
    Pose previous,penultimate,transitionFrom,previousAuthored;
    PoseContinuation continuation;
    std::array<bool,2> transitionArmGuard{};
    float previousDt{};
    Motion lastMotion=Motion::none;
    Motion lastWallRunDirection=Motion::none,runSequenceDirection=Motion::none;
    bool lastContextRecovery{};
    float phase{},lastSamplePhase{},transition=1,transitionSeconds=.18f;
    Motion bridge=Motion::none;
    Motion bridgeDirection=Motion::none;
    Motion stopSource=Motion::none;
    float stopPhase{};
    Quat runOrientation,actionOrientation;
    Vec actionHeading{};
    float sideGrip{},surfaceOffset{},surfaceMissSeconds{};
    Vec lastPosition{},lastNormal{};
    std::array<Vec,2> topInitialPalms{};
    std::array<Vec,2> topReplant{};
    std::array<bool,2> topReplantValid{};
    bool topReplantProbed{},topPrepared{};
    bool obstacleKick{};
    Motion obstacleReturnMotion=Motion::none;
    Motion entryReturnMotion=Motion::none;
    float entryElapsed{};
    float obstacleAlong{};
    bool started{};
    bool sourceReturn{};
    bool lastWallTargets{};
    static bool wallRun(Motion m) {return m>=Motion::runLeft&&m<=Motion::runDiagonalRight;}
    static bool catchEntry(Motion m) {return m==Motion::reach||m==Motion::jumpCatch||m==Motion::ledgeCatch||m==Motion::runLaunch;}
    static bool launch(Motion m) {return m==Motion::runLaunch||m==Motion::runLaunchLeft||m==Motion::runLaunchRight;}
    static bool climbCycle(Motion m) {return m>=Motion::up&&m<=Motion::right;}

public:
    float maxReachError{};
    int contactCount{},retiredContacts{};
    float topPalmError{};
    float edgePalmError{};
    bool sidePalmContact{};
    float sidePalmError{};
    int runFootContacts{};
    float measuredSurfaceGap{},appliedSurfaceGap{};
    unsigned surfaceSamples{};
    bool surfaceGapValid{},surfaceHeld{};
    std::array<float,2> surfacePalmGaps{};

    float movementScale() const {return 1.f;}
    float gaitPhase() const {return phase;}
    float sampledPhase() const {return lastSamplePhase;}
    float blendProgress() const {return transition;}
    Motion bridgeMotion() const {return bridge;}
    Motion bridgeWallRunDirection() const {return bridgeDirection;}
    unsigned rejectedIdleContacts() const {return unsigned(std::count_if(contacts.begin(),contacts.end(),[](const auto& value){return value.rejected;}));}
    float surfaceHoldSeconds() const {return surfaceHeld?surfaceMissSeconds:0.f;}
    void reset() {*this={};}
    Pose update(const Library& lib,World& world,const Traversal& traversal,Motion motion,float dt,float scale) {
        dt=std::clamp(dt,0.f,.05f);scale=std::clamp(scale,.5f,2.f);
        if(started&&dt<=1e-6f)return previous;
        if(!isActiveMotion(motion))motion=Motion::hang;
        const auto wallRunDirection=traversal.poseDirection(motion);
        const bool contextRecovery=motion==Motion::contextHang&&traversal.holdsDestinationEdge(motion);
        const bool contextSequence=motion==Motion::contextHang&&lib.hasContextHopSequence(wallRunDirection);
        const bool motionChanged=motion!=lastMotion||wallRunDirection!=lastWallRunDirection||contextRecovery!=lastContextRecovery;
        if(runMotion(wallRunDirection))runSequenceDirection=wallRunDirection;
        const auto& clip=lib.clip(motion,wallRunDirection,contextRecovery);
        const bool authoredPlayback=clip.authoredPlayback;
        const bool sourceTransition=authoredPlayback||(started&&lib.clip(lastMotion,lastWallRunDirection,lastContextRecovery).authoredPlayback);
        if(authoredPlayback)sourceReturn=false;
        else if(started&&lib.clip(lastMotion,lastWallRunDirection,lastContextRecovery).authoredPlayback)sourceReturn=true;
        auto solveIK=[&](Pose& value,int a,int b,int c,Vec target,Vec pole){return lib.ik(value,a,b,c,target,pole,!authoredPlayback);};
        const Vec pos=traversal.position,n=traversal.normal;
        const Vec right{-n.y,n.x,0},forward=n*-1;
        auto toWorld=[&](Vec v){return pos+(right*v.x+forward*v.y+Vec{0,0,v.z})*scale;};
        auto toLocal=[&](Vec v){v=(v-pos)/scale;return Vec{v.dot(right),v.dot(forward),v.z};};
        auto shiftCom=[&](Pose& value,Vec delta) {
            if(traversal.crestTop()) {
                const auto& parent=value[0];const Vec local=parent.q.inverse().rotate(delta);
                delta={local.x/parent.s.x,local.y/parent.s.y,local.z/parent.s.z};
            }
            value[4].t=value[4].t+delta;
        };
        const bool running=runMotion(motion);
        const bool runEntry=motion==Motion::runLaunch;
        const bool entryReturn=started&&lastMotion==Motion::runLaunch&&motion==entryReturnMotion;
        if(runEntry&&(!started||motion!=lastMotion)){entryElapsed=0;entryReturnMotion=Motion::none;}
        const bool sidewaysRun=running&&std::abs(traversal.direction().x)>.1f;
        const bool kick=motion>=Motion::kickUp&&motion<=Motion::kickRight;
        const bool obstacleReturn=started&&motion!=lastMotion&&obstacleKick&&motion==obstacleReturnMotion;
        if(!started||motion!=lastMotion) {
            obstacleKick=kick&&traversal.obstacleJumpActive();
            if(obstacleKick)obstacleAlong=traversal.actionRouteDistance();
            if(!obstacleKick&&!obstacleReturn)obstacleReturnMotion=Motion::none;
        }
        const bool parkour=kick;
        const bool backKick=motion==Motion::dropBack;
        const bool backFlip=motion==Motion::backFlipOut;
        const bool departing=backKick||backFlip;
        const bool edgeAction=!authoredPlayback&&traversal.usesEdgeTargets(motion);
        const bool wallTargets=!authoredPlayback&&traversal.usesWallTargets(motion);
        const bool newHang=motion==Motion::contextHang,newHop=threepeatHop(motion),newMantle=motion==Motion::contextMantle;
        const float wallPalmOffset=newHang||newHop?capturedWallPalmOffset:6.f;
        const bool loop=(motion>=Motion::hang&&motion<=Motion::right)||newHang||running;
        const bool mantle=newMantle;
        const bool entry=catchEntry(motion);
        const bool supportedPlacement=loop&&!edgeAction&&(traversal.state==State::wall||traversal.state==State::ledge);
        const bool stationaryIdle=started&&motion==Motion::hang&&!authoredPlayback&&motion==lastMotion&&supportedPlacement&&
            (pos-lastPosition).length()<=.025f*scale&&(n-lastNormal).length()<.001f;
        surfaceSamples=0;surfaceGapValid=false;surfaceHeld=false;surfacePalmGaps={};measuredSurfaceGap=traversal.cfg.gap;
        Vec measuredPoint{},measuredNormal{};
        float surfaceGoal=0;
        if(supportedPlacement) {
            const auto plane=traversal.surfaceNormal.unit();
            const float horizontal=std::hypot(plane.x,plane.y);
            const auto tangent=(Vec{0,0,1}-plane*plane.z).unit();
            std::optional<float> measured;
            if(horizontal>.3f)for(float height:{traversal.cfg.grip,traversal.cfg.chest}) {
                std::array<std::optional<Hit>,3> hits{};
                std::array<float,3> gaps{};
                for(unsigned index=0;index<3;++index) {
                    const float side=index==0?0:index==1?-10.f:10.f;
                    const Vec anchor=pos+Vec{0,0,6}+tangent*(height-6)+right*side;
                    const auto hit=world.ray(anchor+plane*16.f,anchor-plane*(traversal.cfg.gap+20.f));
                    if(!hit||!hit->climbable||!hit->point.finite()||!hit->normal.finite()||
                        hit->normal.length()<.5f||hit->normal.unit().dot(plane)<.95f)continue;
                    const float gap=(pos+Vec{0,0,6}-hit->point).dot(plane)/horizontal;
                    if(!std::isfinite(gap)||gap<traversal.cfg.gap-2.f||gap>traversal.cfg.gap+20.f)continue;
                    hits[index]=hit;gaps[index]=gap;++surfaceSamples;
                }
                for(unsigned a=0;a<3&&!measured;++a)for(unsigned b=a+1;b<3;++b) {
                    if(hits[a]&&hits[b]&&hits[a]->normal.unit().dot(hits[b]->normal.unit())>.95f&&
                        std::abs(gaps[a]-gaps[b])<2.5f&&(hits[a]->point-hits[b]->point).length()>1.f) {
                        measured=(gaps[a]+gaps[b])*.5f;
                        measuredPoint=(hits[a]->point+hits[b]->point)*.5f;
                        measuredNormal=(hits[a]->normal+hits[b]->normal).unit();break;
                    }
                }
                if(measured)break;
            }
            if(measured) {
                surfaceMissSeconds=0;
                measuredSurfaceGap=*measured;surfaceGapValid=true;
                surfaceGoal=std::abs(*measured-traversal.cfg.gap)<.1f?0.f:
                    std::clamp(*measured-traversal.cfg.gap,0.f,20.f);
            }
        }
        if(!surfaceGapValid) {
            surfaceMissSeconds=std::min(.2f,surfaceMissSeconds+dt);
            if(stationaryIdle&&surfaceMissSeconds<=.1f&&surfaceOffset>0) {surfaceGoal=surfaceOffset;surfaceHeld=true;}
        }
        surfaceOffset=started?surfaceOffset+std::clamp(surfaceGoal-surfaceOffset,-90.f*dt,90.f*dt):surfaceGoal;
        const float poseGap=traversal.cfg.gap+surfaceOffset;
        appliedSurfaceGap=poseGap;
        auto placeWall=[&](Pose& pose,bool feet) {
            const float slope=std::clamp(traversal.surfaceNormal.z,0.f,.68f);
            const auto tilt=Quat::axis({1,0,0},-std::asin(slope));
            pose[0].q=tilt*pose[0].q;pose[0].t=tilt.rotate(pose[0].t);
            if(feet) {
                const float h=std::sqrt(1-slope*slope);
                pose[0].t=pose[0].t+Vec{0,h,-slope}*(poseGap*h/scale-37);
            } else pose[4].t.y+=(poseGap*std::sqrt(1-slope*slope)-6*slope)/scale-38;
        };
        auto wallRotation=[&](Vec direction) {
            if(direction.length()<.1f)direction={0,1,0};
            return Quat::axis({1,0,0},std::acos(std::clamp(traversal.surfaceNormal.z,-.15f,.75f)))*
                Quat::axis({0,0,1},std::atan2(-direction.x,direction.y));
        };
        if(running) {
            const auto desired=sidewaysRun?sideRunFrame(traversal.surfaceNormal.z,traversal.direction()).rotation:
                wallRotation(traversal.direction());
            runOrientation=started&&motion==lastMotion?boundedRotation(runOrientation,desired,4.1887902f*dt):desired;
        }
        if(parkour&&(!started||motion!=lastMotion)) {
            actionHeading=traversal.actionHeading();
            actionOrientation=std::abs(actionHeading.x)>.1f?sideRunFrame(traversal.surfaceNormal.z,actionHeading).rotation:
                wallRotation(actionHeading);
        }
        auto placeGround=[&](Pose& pose,Quat orientation) {
            pose[0].q=orientation*pose[0].q;pose[0].t=orientation.rotate(pose[0].t);
            const float z=std::clamp(traversal.surfaceNormal.z,-.15f,.75f),h=std::sqrt(1-z*z);
            pose[0].t=pose[0].t+Vec{0,h,-z}*(poseGap*h/scale-3);
        };
        auto placeOriented=[&](Pose& pose,Quat orientation,Vec direction) {
            if(std::abs(direction.x)>.1f) {
                auto frame=sideRunFrame(traversal.surfaceNormal.z,direction);frame.rotation=orientation;
                const float z=std::clamp(traversal.surfaceNormal.z,-.15f,.75f);
                placeSideRun(lib,pose,frame,-poseGap*std::sqrt(1-z*z)/scale);
            } else placeGround(pose,orientation);
        };
        auto placeRun=[&](Pose& pose,float gait) {
            placeOriented(pose,runOrientation,traversal.direction());
            if(sidewaysRun) {
                auto frame=sideRunFrame(traversal.surfaceNormal.z,traversal.direction());frame.rotation=runOrientation;
                applySideRunBrace(lib,pose,frame,gait,1.f,wallRunDirection);
            }
        };

        const bool stopTargetChanged=stopSource!=Motion::none&&(authoredPlayback||motion!=Motion::hang||edgeAction||traversal.preparingEdge());
        if(stopTargetChanged)stopSource=Motion::none;
        if(!sourceTransition&&started&&motion==Motion::hang&&motion!=lastMotion&&climbCycle(lastMotion)&&!edgeAction&&!traversal.preparingEdge()) {
            const auto before=lib.world(previous);
            auto cost=[&](const Pose& sample) {
                const auto body=lib.world(sample);float value=0;
                for(int bone:{6,7,9,10,24,28,29,31,32}) {
                    const float angle=angleBetween(previous[bone].q,sample[bone].q);
                    value+=angle*angle*(bone<12?2.f:1.f);
                }
                for(int bone:{8,11,38,39}){const auto delta=body[bone].t-before[bone].t;value+=delta.dot(delta)*.002f;}
                return value;
            };
            auto neutral=lib.sample(Motion::hang,0);placeWall(neutral,false);
            const float neutralCost=cost(neutral);
            auto armCost=[&](const Pose& value) {
                float sum=0;for(int bone:{28,29,31,32}){const float angle=angleBetween(previous[bone].q,value[bone].q);sum+=angle*angle;}return sum;
            };
            const float neutralArms=armCost(neutral);
            PoseContinuation forecast;forecast.begin(previous,penultimate,previousDt);
            const auto advancing=forecast.sample(.09f);
            auto advancingArmCost=[&](const Pose& value) {
                float sum=0;for(int bone:{28,29,31,32}){const float angle=angleBetween(advancing[bone].q,value[bone].q);sum+=angle*angle;}return sum;
            };
            const float neutralAdvancingArms=advancingArmCost(neutral);
            struct Candidate {float phase,cost;};std::vector<Candidate> choices;
            for(int frame=0;frame<48;++frame) {
                const float at=frame/48.f;const auto support=lib.contactWeights(lastMotion,at);
                if(std::min(support[0],support[1])<.90f)continue;
                auto value=lib.sample(lastMotion,at);placeWall(value,false);const float score=cost(value);

                if(score<neutralCost*.8f&&armCost(value)<neutralArms*.55f&&
                    advancingArmCost(value)<neutralAdvancingArms*.55f)choices.push_back({at,score});
            }
            std::sort(choices.begin(),choices.end(),[](auto a,auto b){return a.cost<b.cost;});

            for(std::size_t attempt=0;attempt<std::min<std::size_t>(3,choices.size());++attempt) {
                auto value=lib.sample(lastMotion,choices[attempt].phase);placeWall(value,false);
                const auto body=lib.world(value);bool supported=true;
                for(int hand=0;hand<2;++hand) {
                    const Vec nominal=toWorld(body[hand?39:38].t);
                    const auto hit=world.ray(nominal+n*14*scale,nominal-n*20*scale);
                    if(!hit||!hit->climbable||!hit->normal.finite()||hit->normal.unit().dot(n)<.95f){supported=false;break;}
                    const Vec normal=hit->normal.unit();
                    const Vec correction=normal*(5*scale-(nominal-hit->point).dot(normal));
                    if(correction.length()>=10*scale){supported=false;break;}

                    const int upper=hand?31:28,elbow=hand?32:29,wrist=hand?39:38;
                    const Vec goal=body[wrist].t+Vec{correction.dot(right),correction.dot(forward),correction.z}/scale;
                    auto candidate=value;solveIK(candidate,upper,elbow,wrist,goal,body[elbow].t);
                    const auto checked=lib.world(candidate);
                    const float bend=std::acos(std::clamp((checked[elbow].t-checked[upper].t).unit().dot((checked[wrist].t-checked[elbow].t).unit()),-1.f,1.f));
                    if(bend<.17453293f){supported=false;break;}
                }
                if(supported){stopSource=lastMotion;stopPhase=choices[attempt].phase;break;}
            }
        }
        if(started&&(motionChanged||wallTargets!=lastWallTargets||stopTargetChanged)) {

            const bool continueBridge=!sourceTransition&&!edgeAction&&!stopTargetChanged&&bridge!=Motion::none&&transition<1&&wallTargets==lastWallTargets&&
                ((running&&runMotion(lastMotion))||(!running&&!runMotion(lastMotion)&&
                    (climbCycle(motion)||motion==Motion::hang)&&(climbCycle(lastMotion)||lastMotion==Motion::hang)));
            if(!continueBridge) {
                transitionFrom=previous;transition=0;sideGrip=0;for(auto& c:contacts)c={};
                continuation.begin(previous,penultimate,previousDt);
                for(int hand=0;hand<2;++hand)transitionArmGuard[hand]=lib.armBendValid(previous,hand);
                bridge=Motion::none;bridgeDirection=Motion::none;
            }
            if(!continueBridge) {
            if(!sourceTransition&&running&&!runMotion(lastMotion)&&!hopMotion(lastMotion)&&!catchEntry(lastMotion))
                bridge=wallRun(motion)?(traversal.direction().x<0?Motion::runLaunchLeft:Motion::runLaunchRight):Motion::runLaunch;
            else if(!sourceTransition&&!running&&runMotion(lastMotion)&&!mantle&&!hopMotion(motion)&&!departing&&motion!=Motion::drop)bridge=Motion::runCatch;
            if(!sourceTransition&&newMantle&&traversal.topPreparation()<1.f&&(runMotion(lastMotion)||launch(lastMotion)||
                (lastMotion>=Motion::kickUp&&lastMotion<=Motion::kickRight)||traversal.obstacleJumpActive()))bridge=Motion::runCatch;
            if(bridge!=Motion::none) {
                bridgeDirection=launch(bridge)?motion:runMotion(lastMotion)?lastMotion:runSequenceDirection;
                if(lib.clip(bridge,bridgeDirection).authoredPlayback){bridge=Motion::none;bridgeDirection=Motion::none;}
            }
            transitionSeconds=(newMantle&&traversal.topPreparation()<1.f)?.24f:(newHang&&traversal.holdsPreparedEdge(motion))?.06f:(newHop&&lastMotion==Motion::contextHang)?.06f:(newHang&&threepeatHop(lastMotion))?.18f:
                parkour?.07f:backFlip?.10f:backKick?.09f:launch(bridge)?.22f:bridge==Motion::runCatch?.20f:
                obstacleReturn||entryReturn?.08f:running?(hopMotion(lastMotion)?.12f:.20f):hopMotion(lastMotion)?.14f:.18f;
            if(sourceTransition)transitionSeconds=std::clamp(clip.seconds*.12f,.01f,.22f);
            }
            if((newHang||authoredPlayback)&&motionChanged)phase=0.f;
            if(!authoredPlayback&&loop&&!obstacleReturn&&!entryReturn&&motion!=Motion::hang&&!newHang) {

                float best=1e9f;const auto from=lib.world(previous);
                for(int frame=0;frame<48;++frame) {
                    const float candidate=frame/48.f;auto sample=lib.sample(motion,candidate);float cost=0;
                    if(running)placeRun(sample,candidate);else placeWall(sample,false);
                    const auto target=lib.world(sample);
                    for(int bone:{6,7,9,10,24,28,29,31,32}) {
                        const float angle=angleBetween(previous[bone].q,sample[bone].q);
                        cost+=angle*angle*(bone<12?2.f:1.f);
                    }
                    for(int bone:{8,11,38,39}) {const auto delta=target[bone].t-from[bone].t;cost+=delta.dot(delta)*.002f;}
                    cost+=angleBetween(from[4].q,target[4].q)*2;
                    if(cost<best){best=cost;phase=candidate;}
                }
            }
        }
        if(mantle&&(!started||motion!=lastMotion)) {
            topReplantValid={};topReplantProbed=false;topPrepared=newMantle&&traversal.topPreparation()<1.f;
            const auto initial=lib.world(started?previous:lib.sample(motion,0));
            const Vec oldRight{-lastNormal.y,lastNormal.x,0};
            for(int hand=0;hand<2;++hand) {
                const auto point=lib.palm(initial,hand);
                topInitialPalms[hand]=started?lastPosition+(oldRight*point.x-lastNormal*point.y+Vec{0,0,point.z})*scale:toWorld(point);
            }
        }
        const bool replanted=!authoredPlayback&&newMantle&&traversal.topReplanted(traversal.topSamplePhase(traversal.progress()));
        if(replanted&&!topReplantProbed) {
            topReplantProbed=true;
            for(int hand=0;hand<2;++hand) {

                const auto contact=threepeatReplantContact(world,traversal.topHand(hand),n,
                    traversal.topReplantOffset(hand),scale);
                if(contact) {
                    topReplant[hand]=*contact;topReplantValid[hand]=true;
                }
            }
        }
        if(replanted)for(int hand=0;hand<2;++hand)if(topReplantValid[hand]) {
            const auto hit=world.ray(topReplant[hand]+Vec{0,0,3*scale},topReplant[hand]-Vec{0,0,3*scale});
            if(!hit||!hit->climbable||hit->normal.z<.95f||
                (hit->point+hit->normal*(.8f*scale)-topReplant[hand]).length()>1.25f*scale)topReplantValid[hand]=false;
        }
        auto topAnchor=[&](int hand) {
            if(replanted&&topReplantValid[hand])return topReplant[hand];
            const auto target=traversal.topHand(hand)+traversal.topHandNormal(hand)*(.8f*scale);
            return topInitialPalms[hand]+(target-topInitialPalms[hand])*smooth(topPrepared?traversal.topPreparation()/.40f:traversal.progress()/.20f);
        };
        if(authoredPlayback&&loop&&(climbCycle(motion)||running)) {
            if(started&&motion==lastMotion&&clip.stride>1)phase=std::fmod(phase+(pos-lastPosition).length()/(clip.stride*scale),1.f);
        } else if(loop&&clip.stride>1&&started&&!(authoredPlayback&&(motion==Motion::hang||newHang)))phase=std::fmod(phase+std::min((pos-lastPosition).length()/(clip.stride*scale),dt*2.8f/std::max(.1f,clip.seconds)),1.f);
        else if(loop)phase=std::fmod(phase+dt/std::max(authoredPlayback?.001f:.1f,clip.seconds),1.f);
        float samplePhase=loop?phase:authoredPlayback?traversal.authoredPhase(motion):mantle?traversal.progress():
            entry?traversal.reachProgress():traversal.actionProgress();
        if(newMantle)samplePhase=traversal.topSamplePhase(samplePhase);
        if(contextSequence)samplePhase=traversal.contextHangPhase();
        if(stopSource!=Motion::none)samplePhase=stopPhase;
        if(!authoredPlayback&&motion==Motion::reach)samplePhase=.55f+.45f*samplePhase;
        lastSamplePhase=samplePhase;
        auto edgeWeights=[&](int hand) {
            if(traversal.holdsDestinationEdge(motion))return std::array<float,2>{0,1};
            if(traversal.holdsPreparedEdge(motion))return std::array<float,2>{traversal.preparedEdgeWeight(motion),0};
            if(newHop)return std::array<float,2>{threepeatSourceWeight(motion==Motion::contextHopLeft,hand,samplePhase,traversal.cfg.threepeatProfile),
                threepeatTargetWeight(motion==Motion::contextHopLeft,hand,samplePhase,traversal.cfg.threepeatProfile)};

            return std::array<float,2>{1-smooth(samplePhase/.18f),smooth((samplePhase-.78f)/.20f)};
        };
        auto edgeAnchor=[&](int hand) {
            const auto weights=edgeWeights(hand);
            const bool destination=weights[1]>weights[0];
            return traversal.edgeHand(hand,destination)+traversal.edgeContactNormal(hand,destination)*((wallTargets?wallPalmOffset:.8f)*scale);
        };
        auto topWeight=[&](int hand) {
            if(authoredPlayback)return 0.f;
            return newMantle&&(!replanted||topReplantValid[hand])?traversal.topHandWeight(hand,samplePhase):0.f;
        };
        auto topSupport=[&](const Pose& body) {
            Vec palms{},target{};float total=0;
            for(int hand=0;hand<2;++hand) {
                const float weight=topWeight(hand);
                palms=palms+lib.palm(body,hand)*weight;target=target+toLocal(topAnchor(hand))*weight;total+=weight;
            }
            return total>0?(target-palms)/total:Vec{};
        };

        Pose p=lib.sample(stopSource!=Motion::none?stopSource:motion,authoredPlayback?samplePhase:kick?std::clamp(samplePhase/.60f,0.f,1.f):samplePhase,wallRunDirection,contextRecovery);
        if(authoredPlayback) {
            p[0].t=p[0].t-traversal.authoredRoot(motion,samplePhase);
            if(!mantle&&!departing&&motion!=Motion::drop)placeWall(p,false);
        } else if(mantle) {
            if(traversal.crestTop()) {
                const float lean=std::asin(std::clamp(traversal.surfaceNormal.z,0.f,.7f))*(1-smooth((samplePhase-.56f)/.30f));
                const auto before=lib.world(p);const Vec pivot=(lib.palm(before,0)+lib.palm(before,1))*.5f;
                p[4].q=(Quat::axis({1,0,0},-lean)*p[4].q).unit();
                const auto after=lib.world(p);shiftCom(p,pivot-(lib.palm(after,0)+lib.palm(after,1))*.5f);
            }
            const Vec root=(pos-traversal.topStart())/scale;
            const Vec destination=(traversal.topTarget()-traversal.topStart())/scale;
            const float heightDelta=destination.z-clip.height;
            const float forwardDelta=destination.dot(forward)-clip.travel.y;
            p[4].t=p[4].t+Vec{-root.dot(right),forwardDelta*traversal.topForwardProgress(samplePhase)-root.dot(forward),heightDelta*traversal.topRiseProgress(samplePhase)-root.z};
            if(newMantle)p[4].t.x-=clip.travel.x*smooth(samplePhase);
            if(newMantle||samplePhase<.72f) {
                const auto current=lib.world(p);
                Vec delta=topSupport(current);float placement=std::max(topWeight(0),topWeight(1));
                if(traversal.crestTop()) {
                    const auto& release=traversal.cfg.threepeatProfile.mantleRelease;
                    const float begin=std::min(release[0][0],release[1][0]),end=std::max(release[0][1],release[1][1]);
                    placement=1-smooth((samplePhase-begin)/std::max(.01f,end-begin));
                    delta=(toLocal(topAnchor(0))+toLocal(topAnchor(1))-lib.palm(current,0)-lib.palm(current,1))*.5f;
                }
                if(delta.length()>80)delta=delta.unit()*80;
                shiftCom(p,delta*placement);
            }
        } else if(runEntry) {

            entryElapsed+=dt;
            if(!lib.reference(motion,PlaybackReference::launchApproach).authoredPlayback) {
            const float groundPhase=std::fmod(entryElapsed/std::max(.1f,lib.reference(motion,PlaybackReference::launchApproach).seconds),1.f);
            auto incoming=lib.sampleReference(motion,PlaybackReference::launchApproach,groundPhase);
            placeWall(p,true);
            const float planted=smooth(samplePhase/.24f);
            for(std::size_t i=0;i<p.size();++i)p[i]=blend(incoming[i],p[i],planted);
            } else placeWall(p,true);
            if(samplePhase>=.16f) {
                const Vec direction=traversal.direction();
                const bool moving=std::abs(direction.x)+std::abs(direction.y)>.1f;
                const bool resumeRun=traversal.wallRunning()&&moving;
                const Motion target=!moving?Motion::hang:resumeRun?
                    (std::abs(direction.x)<.1f?Motion::runUp:std::abs(direction.y)<.1f?
                        (direction.x<0?Motion::runLeft:Motion::runRight):
                        (direction.x<0?Motion::runDiagonalLeft:Motion::runDiagonalRight)):
                    (std::abs(direction.y)>std::abs(direction.x)?(direction.y<0?Motion::down:Motion::up):
                        (direction.x<0?Motion::left:Motion::right));
                if(!lib.clip(target).authoredPlayback) {
                auto placeDestination=[&](Pose& value,float gait) {
                    if(resumeRun) {
                        const auto orientation=std::abs(direction.x)>.1f?sideRunFrame(traversal.surfaceNormal.z,direction).rotation:wallRotation(direction);
                        placeOriented(value,orientation,direction);
                        if(std::abs(direction.x)>.1f){applySideRunBrace(lib,value,sideRunFrame(traversal.surfaceNormal.z,direction),gait,1.f,target);limitSideRunUpperRoll(lib,value,direction.x>0?0:1);}
                    } else placeWall(value,false);
                };
                if(entryReturnMotion!=target) {
                    const auto before=lib.world(p);float best=1e9f;
                    const auto& destinationClip=lib.clip(target);
                    const float cadence=(resumeRun?traversal.cfg.runSpeed:
                        std::abs(direction.y)>std::abs(direction.x)?traversal.cfg.climbSpeed:traversal.cfg.sideSpeed)/std::max(1.f,destinationClip.stride);
                    const float remaining=traversal.entryDuration()*(1-std::max(.20f,samplePhase));
                    for(int frame=0;frame<48;++frame) {
                        const float candidate=frame/48.f;
                        if(resumeRun) {

                            const auto support=lib.contactWeights(target,std::fmod(candidate+remaining*cadence,1.f));
                            if(std::max(support[2],support[3])<.80f)continue;
                        }
                        auto value=lib.sample(target,candidate);placeDestination(value,candidate);
                        const auto after=lib.world(value);float cost=0;
                        for(int bone:{6,7,9,10,28,29,31,32}){const float angle=angleBetween(p[bone].q,value[bone].q);cost+=angle*angle;}
                        for(int bone:{8,11,38,39}){const Vec delta=after[bone].t-before[bone].t;cost+=delta.dot(delta)*.002f;}
                        if(cost<best){best=cost;phase=candidate;}
                    }
                    entryReturnMotion=target;
                }
                const auto& targetClip=lib.clip(target);
                if(moving&&samplePhase>.20f)phase=std::fmod(phase+dt*(resumeRun?traversal.cfg.runSpeed:
                    std::abs(direction.y)>std::abs(direction.x)?traversal.cfg.climbSpeed:traversal.cfg.sideSpeed)/std::max(1.f,targetClip.stride),1.f);
                auto destination=lib.sample(target,phase);placeDestination(destination,phase);
                const float settle=smooth((samplePhase-.20f)/.70f);
                for(std::size_t i=0;i<p.size();++i)p[i]=blend(p[i],destination[i],settle);
                }
            }
        } else if(running) {
            placeRun(p,phase);
        } else if(parkour) {
            if(motion==Motion::kickUp)placeOriented(p,actionOrientation,actionHeading);
            else placeWall(p,true);
            if(!obstacleKick&&motion!=Motion::kickUp&&!lib.reference(motion,PlaybackReference::kickTakeoff).authoredPlayback) {
                auto push=lib.sampleReference(motion,PlaybackReference::kickTakeoff,actionPushPhase(samplePhase));
                placeOriented(push,actionOrientation,actionHeading);
                const float flight=smooth((samplePhase-.12f)/.14f);
                for(std::size_t i=0;i<p.size();++i)p[i]=blend(push[i],p[i],flight);
            }
            const float returnAt=obstacleKick?.48f:.60f;
            if(samplePhase>returnAt) {
                const auto direction=traversal.direction();
                const bool resumeRun=traversal.wallRunning();
                const bool moving=std::abs(direction.x)+std::abs(direction.y)>.1f;
                const Motion returnMotion=obstacleKick?(!moving?Motion::hang:resumeRun?
                    (std::abs(direction.x)<.1f?Motion::runUp:std::abs(direction.y)<.1f?
                        (direction.x<0?Motion::runLeft:Motion::runRight):
                        (direction.x<0?Motion::runDiagonalLeft:Motion::runDiagonalRight)):
                    (std::abs(direction.y)>std::abs(direction.x)?(direction.y<0?Motion::down:Motion::up):
                        (direction.x<0?Motion::left:Motion::right))):resumeRun?Motion::runUp:
                    motion==Motion::kickLeft?Motion::hopLeft:
                    motion==Motion::kickRight?Motion::hopRight:Motion::hopUp;
                if(!(obstacleKick?lib.clip(returnMotion):lib.reference(motion,resumeRun?PlaybackReference::kickRunLanding:PlaybackReference::kickLanding)).authoredPlayback) {
                const float returnPhase=obstacleKick?std::clamp((samplePhase-returnAt)/(1-returnAt),0.f,1.f):actionReturnPhase(samplePhase);
                Pose destination;
                if(obstacleKick) {

                    const Motion target=returnMotion;
                    auto placeDestination=[&](Pose& value,float gait) {
                        if(resumeRun&&moving) {
                            const auto orientation=std::abs(direction.x)>.1f?sideRunFrame(traversal.surfaceNormal.z,direction).rotation:wallRotation(direction);
                            placeOriented(value,orientation,direction);
                            if(std::abs(direction.x)>.1f) {
                                applySideRunBrace(lib,value,sideRunFrame(traversal.surfaceNormal.z,direction),gait,1.f,target);
                                limitSideRunUpperRoll(lib,value,direction.x>0?0:1);
                            }
                        } else placeWall(value,false);
                    };
                    if(obstacleReturnMotion!=target) {
                        const auto before=lib.world(previous);float best=1e9f;
                        for(int frame=0;frame<48;++frame) {
                            const float candidate=frame/48.f;auto value=lib.sample(target,candidate);placeDestination(value,candidate);
                            const auto body=lib.world(value);float cost=0;
                            for(int bone:{6,7,9,10,24,28,29,31,32}) {
                                const float angle=angleBetween(previous[bone].q,value[bone].q);cost+=angle*angle*(bone<12?2.f:1.f);
                            }
                            for(int bone:{8,11,38,39}) {const auto delta=body[bone].t-before[bone].t;cost+=delta.dot(delta)*.002f;}
                            if(cost<best){best=cost;phase=candidate;}
                        }
                        obstacleReturnMotion=target;
                    }
                    const auto& returnClip=lib.clip(target);

                    const float distance=std::max(0.f,traversal.actionRouteDistance()-obstacleAlong);
                    const float advance=returnClip.stride>1?distance/(returnClip.stride*scale):dt/std::max(.1f,returnClip.seconds);
                    phase=std::fmod(phase+std::min(advance,dt*2.8f/std::max(.1f,returnClip.seconds)),1.f);
                    destination=lib.sample(target,phase);placeDestination(destination,phase);
                } else if(traversal.wallRunning()) {

                    destination=lib.sampleReference(motion,PlaybackReference::kickRunLanding,std::fmod(phase+returnPhase*.30f,1.f));
                    const auto orientation=std::abs(direction.x)>.1f?sideRunFrame(traversal.surfaceNormal.z,direction).rotation:wallRotation(direction);
                    placeOriented(destination,orientation,direction);
                    if(std::abs(direction.x)>.1f) {
                        const float gait=std::fmod(phase+returnPhase*.30f,1.f);
                        const auto reference=lib.reference(motion,PlaybackReference::kickRunBrace).authoredPlayback?destination:
                            lib.sampleReference(motion,PlaybackReference::kickRunBrace,.5f+.10f*std::sin(gait*6.283185307f));
                        applySideRunBrace(lib,destination,sideRunFrame(traversal.surfaceNormal.z,direction),gait,1.f,Motion::none,&reference);
                    }
                } else {
                    destination=lib.sampleReference(motion,PlaybackReference::kickLanding,.65f+.35f*returnPhase);placeWall(destination,false);
                }
                for(std::size_t i=0;i<p.size();++i)p[i]=blend(p[i],destination[i],smooth(returnPhase));
                }
            }
        } else if(backFlip) {
            placeBackFlipOut(lib,p,traversal.surfaceNormal.z,traversal.cfg.gap,scale);
        } else if(backKick) {
            placeWall(p,true);
        } else if(newHang||newHop) {

            p[4].t.y+=traversal.cfg.gap/scale+(wallTargets?-capturedWallPalmOffset:gripEdgeDetail::palmInset)-traversal.contextHangForward();
        } else {

            const float slope=std::clamp(traversal.surfaceNormal.z,0.f,.68f)*(entry?smooth(samplePhase):1.f);
            const auto tilt=Quat::axis({1,0,0},-std::asin(slope));
            p[0].q=authoredPlayback?(tilt*p[0].q).unit():tilt;
            if(authoredPlayback)p[0].t=tilt.rotate(p[0].t);

            p[4].t.y+=(poseGap*std::sqrt(1-slope*slope)-6*slope)/scale-38;
        }

        if(obstacleKick)obstacleAlong=traversal.actionRouteDistance();
        if(edgeAction) {

            const auto body=lib.world(p);Vec correction{};float total=0;
            for(int hand=0;hand<2;++hand) {
                const auto weights=edgeWeights(hand);const float weight=std::max(weights[0],weights[1]);
                correction=correction+(toLocal(edgeAnchor(hand))-lib.palm(body,hand))*weight;total+=weight;
            }
            if(total>0) {
                correction=correction/std::max(1.f,total);
                if(correction.length()>8)correction=correction.unit()*8;
                shiftCom(p,correction);
            }
        }
        transition=std::min(1.f,transition+dt/transitionSeconds);
        if(transitionFrom.size()==p.size()&&transition<1) {
            auto outgoing=continuation.sample(transition*transitionSeconds);
            if(sourceTransition) {
                for(int hand=0;hand<2;++hand)if(transitionArmGuard[hand])lib.guardArmBend(outgoing,hand);
            } else lib.guardArmBends(outgoing);
            if(bridge!=Motion::none) {
                auto transfer=lib.sample(bridge,transition,bridgeDirection);

                placeWall(transfer,launch(bridge));
                for(std::size_t i=0;i<p.size();++i) {
                    const auto target=blend(transfer[i],p[i],smooth((transition-.20f)/.80f));
                    p[i]=blend(outgoing[i],target,smooth(transition/.65f));
                }
            } else for(std::size_t i=0;i<p.size();++i)p[i]=blend(outgoing[i],p[i],smooth(transition));
        }
        if(!authoredPlayback)lib.guardArmBends(p);
        sidePalmContact=false;sidePalmError=0;runFootContacts=0;
        if(!authoredPlayback&&sidewaysRun) {
            auto frame=sideRunFrame(traversal.surfaceNormal.z,traversal.direction());
            const Vec probe=toWorld(sideRunPalmProbe(lib,p,frame));
            const auto hit=world.ray(probe+n*32*scale,probe-n*64*scale);
            const bool realWall=hit&&hit->climbable&&hit->normal.z<.8f&&hit->normal.unit().dot(traversal.surfaceNormal)>.5f;
            sideGrip+=std::clamp((realWall?1.f:0.f)-sideGrip,-dt/.12f,dt/.18f);
            if(realWall&&sideGrip>0) {
                const Vec anchor=toLocal(hit->point+hit->normal.unit()*(1.2f*scale));
                sidePalmError=applySideRunPalm(lib,p,frame,anchor,sideGrip*smooth(transition))*scale;
                const auto solved=lib.world(p);
                const int elbow=frame.innerHand==0?29:32;
                sidePalmContact=sideGrip>.9f&&transition>.95f&&sidePalmError<5*scale&&
                    sideRunPalmNormal(solved,frame.innerHand).dot(frame.normal*-1)>.70f&&
                    solved[elbow].t.dot(frame.normal)>anchor.dot(frame.normal);
            }
        } else sideGrip=0;
        if(newHang&&wallTargets&&traversal.holdsPreparedEdge(motion)) {
            const float clearance=traversal.preparedEdgeClearance(motion);
            for(int hand=0;hand<2&&clearance>0;++hand) {
                const int upper=hand?31:28,elbow=hand?32:29,wrist=hand?39:38;
                const auto body=lib.world(p);const auto local=p[wrist].q,orientation=body[wrist].q;
                solveIK(p,upper,elbow,wrist,body[wrist].t+Vec{0,-clearance,0},body[elbow].t);
                lib.contactOrientation(p,wrist,local,orientation);
            }
        }
        if(newHop&&edgeAction) {

            for(int hand=0;hand<2;++hand) {
                const auto contact=edgeWeights(hand);
                const float clearance=3.f*(1-std::max(contact[0],contact[1]));
                if(clearance<=0)continue;
                const int upper=hand?31:28,elbow=hand?32:29,wrist=hand?39:38;
                const auto body=lib.world(p);const auto local=p[wrist].q,orientation=body[wrist].q;
                solveIK(p,upper,elbow,wrist,body[wrist].t+Vec{0,-clearance,wallTargets?0.f:clearance*(2.f/3.f)},body[elbow].t);
                lib.contactOrientation(p,wrist,local,orientation);
            }
        }
        if(!authoredPlayback)lib.guardArmBends(p);

        auto limitSideArm=[&](Pose& candidate) {
            return !authoredPlayback&&sidewaysRun&&limitSideRunUpperRoll(lib,candidate,
                traversal.direction().x>0?0:1,smooth(transition));
        };
        if(limitSideArm(p))sidePalmContact=false;
        const Pose authored=p;
        auto weights=lib.contactWeights(stopSource!=Motion::none?stopSource:motion,samplePhase,wallRunDirection,contextRecovery);
        if(!authoredPlayback&&(parkour||departing||runEntry))weights={0,0,0,0};
        if(!authoredPlayback&&running)weights[0]=weights[1]=0;
        constexpr int starts[]={28,31,6,9},mids[]={29,32,7,10},ends[]={38,39,8,11};
        std::array<ContactPose,4> contactPoses{};
        auto sameContactPose=[&](const ContactPose& a,const ContactPose& b) {
            const float distance=.25f*scale;
            return (a.nominal-b.nominal).length()<distance&&(a.start-b.start).length()<distance&&
                (a.pole-b.pole).length()<distance&&(a.normal-b.normal).length()<.001f&&std::abs(a.scale-b.scale)<.001f&&
                angleBetween(a.upper,b.upper)<.01f&&angleBetween(a.middle,b.middle)<.01f;
        };
        auto rejectIdleContact=[&](int index) {
            if(!stationaryIdle)return;
            auto& contact=contacts[index];contact.rejected=true;
            contact.rejectedPoint=contact.point;contact.rejectedPose=contactPoses[index];
        };
        maxReachError=0;contactCount=sidePalmContact?1:0;
        for(int i=0;i<4;++i) {
            auto w=lib.world(p);auto& c=contacts[i];
            const Vec nominal=toWorld(w[ends[i]].t);
            contactPoses[i]={nominal,toWorld(w[starts[i]].t),toWorld(w[mids[i]].t),traversal.surfaceNormal,p[starts[i]].q,p[mids[i]].q,scale};
            if(!stationaryIdle||weights[i]<.01f||!sameContactPose(c.rejectedPose,contactPoses[i]))c.rejected=false;
            if(!authoredPlayback&&running&&i>=2) {

                auto& footContact=contacts[i];footContact={};
                if(sidewaysRun) {
                    if(weights[i]>.15f) {
                        const int toe=i==2?50:51;
                        const Vec sole=toWorld(w[toe].t);
                        auto hit=world.ray(sole+n*32*scale,sole-n*40*scale);
                        if(hit&&hit->climbable&&hit->normal.z<.8f) {
                            const Vec delta=hit->point+hit->normal.unit()*(3*scale)-sole;
                            if(delta.length()<28*scale) {
                                const auto ankleOrientation=w[ends[i]].q;
                                const auto target=w[ends[i]].t+Vec{delta.dot(right),delta.dot(forward),delta.z}/scale*(weights[i]*smooth(transition));
                                solveIK(p,starts[i],mids[i],ends[i],target,w[mids[i]].t);
                                lib.contactOrientation(p,ends[i],authored[ends[i]].q,ankleOrientation);
                                if(weights[i]>.9f){++contactCount;++runFootContacts;}
                            }
                        }
                    }
                    continue;
                }
                if(weights[i]>.15f) {
                    const int toe=i==2?50:51;
                    const Vec sole=toWorld(w[toe].t);
                    auto hit=world.ray(sole+n*10*scale,sole-n*10*scale);
                    if(hit&&hit->climbable&&hit->normal.z<.8f) {
                        const Vec normal=hit->normal.unit();
                        const float error=3*scale+(hit->point-sole).dot(normal);
                        if(std::abs(error)<6*scale) {
                            const Vec delta=normal*error*weights[i]*smooth(transition);
                            p[0].t=p[0].t+Vec{delta.dot(right),delta.dot(forward),delta.z}/scale;
                            ++contactCount;++runFootContacts;
                        }
                    }
                }
                continue;
            }
            bool lipTarget=false;
            if(wallTargets&&i>=2) {

                const float source=1-smooth(samplePhase/.08f);
                const float landed=smooth((samplePhase-.80f)/.20f);
                weights[i]=newHang||traversal.holdsDestinationEdge(motion)||traversal.holdsPreparedEdge(motion)?1.f:std::max(source,landed);
                const Vec toe=toWorld(w[i==2?50:51].t);
                const auto hit=world.ray(toe+n*14*scale,toe-n*20*scale);
                if(hit&&hit->climbable&&hit->normal.finite()&&hit->normal.unit().dot(n)>.95f) {
                    const Vec surface=hit->normal.unit();
                    const Vec delta=surface*(3*scale-(toe-hit->point).dot(surface));
                    if(delta.length()<10*scale) {
                        c.point=nominal+delta;c.valid=true;c.retiring=false;lipTarget=true;
                    } else {c={};continue;}
                } else {c={};continue;}
            }
            if(edgeAction&&i<2) {
                const auto targets=edgeWeights(i);const float weight=std::max(targets[0],targets[1]);
                const Vec anchor=edgeAnchor(i);
                const Vec palm=toWorld(lib.palm(w,i)),delta=anchor-palm;
                weights[i]=0;
                if(weight>0&&delta.length()<14*scale) {
                    c.point=nominal+delta;c.valid=true;c.retiring=false;lipTarget=true;weights[i]=weight;
                } else {c={};continue;}
            }

            if(mantle&&i<2&&topWeight(i)>0) {
                const Vec contact=topAnchor(i);
                Vec goal=contact-(toWorld(lib.palm(w,i))-nominal);
                const float reach=(w[mids[i]].t-w[starts[i]].t).length()+(w[ends[i]].t-w[mids[i]].t).length();
                const bool reachable=(toLocal(goal)-w[starts[i]].t).length()<reach-.1f;
                if(reachable) {
                    c.point=goal;c.valid=true;c.retiring=false;
                    lipTarget=true;
                    weights[i]=topWeight(i);
                }
            }
            if(!lipTarget&&c.valid&&!c.retiring&&(nominal-c.point).length()>10*scale){c.retiring=true;++retiredContacts;}
            const float desired=c.retiring?0:weights[i]*smooth(transition);
            const float rate=dt*10;c.weight+=std::clamp(desired-c.weight,-rate,rate);
            if(c.weight<.01f){c.valid=c.retiring=false;continue;}

            if(!c.valid) {
                std::optional<Hit> hit;
                if(mantle)hit=world.ray(nominal+Vec{0,0,12*scale},nominal-Vec{0,0,16*scale});
                if(!hit)hit=world.ray(nominal+n*14*scale,nominal-n*20*scale);
                if(hit&&hit->climbable&&hit->normal.finite()) {
                    const float offset=i<2?5.f:8.f;
                    const Vec surface=hit->normal.unit();
                    const Vec target=nominal+surface*(offset*scale-(nominal-hit->point).dot(surface));
                    if((target-nominal).length()<10*scale) {
                        if(c.rejected&&(target-c.rejectedPoint).length()<.25f*scale){c.weight=0;continue;}
                        if(stationaryIdle) {
                            const auto goal=toLocal(target),start=w[starts[i]].t;
                            const float reach=(w[mids[i]].t-start).length()+(w[ends[i]].t-w[mids[i]].t).length();
                            bool supported=(goal-start).length()<=reach+.5f;
                            if(supported&&i<2) {
                                const Vec reachable=(goal-start).length()>reach-.04f?start+(goal-start).unit()*(reach-.04f):goal;
                                const Vec weighted=w[ends[i]].t+(reachable-w[ends[i]].t)*weights[i];
                                auto checked=p;solveIK(checked,starts[i],mids[i],ends[i],weighted,w[mids[i]].t);
                                supported=angleBetween(authored[starts[i]].q,checked[starts[i]].q)<=.65f&&
                                    angleBetween(authored[mids[i]].q,checked[mids[i]].q)<=.75f&&lib.armBendValid(checked,i);
                            }
                            if(!supported) {
                                c.point=target;rejectIdleContact(i);c.weight=0;continue;
                            }
                        }
                        c.rejected=false;c.point=target;c.valid=true;c.retiring=false;c.weight=std::min(c.weight,dt*10);
                    }
                }
            }
            if(!c.valid)continue;
            Vec target=toLocal(c.point),start=w[starts[i]].t;
            const float reach=(w[mids[i]].t-start).length()+(w[ends[i]].t-w[mids[i]].t).length();
            if((target-start).length()>reach+.5f){
                if(!c.retiring)++retiredContacts;
                c.retiring=true;rejectIdleContact(i);
            }
            if((target-start).length()>reach-.04f)target=start+(target-start).unit()*(reach-.04f);

            const Quat orientation=w[ends[i]].q;
            const Quat localOrientation=p[ends[i]].q;
            const Vec pole=w[mids[i]].t;
            const Vec original=w[ends[i]].t;
            target=original+(target-original)*c.weight;
            maxReachError=std::max(maxReachError,solveIK(p,starts[i],mids[i],ends[i],target,pole)*scale);

            if(i<2&&(angleBetween(authored[starts[i]].q,p[starts[i]].q)>.65f||
                angleBetween(authored[mids[i]].q,p[mids[i]].q)>.75f)) {
                p[starts[i]]=authored[starts[i]];p[mids[i]]=authored[mids[i]];
                c.retiring=true;rejectIdleContact(i);++retiredContacts;continue;
            }
            lib.contactOrientation(p,ends[i],localOrientation,orientation);++contactCount;
        }
        if(newHop&&edgeAction&&!wallTargets) {

            for(int hand=0;hand<2;++hand) {
                const auto body=lib.world(p);const auto load=edgeWeights(hand);
                const Vec anchor=traversal.edgeHand(hand,load[1]>load[0]);
                std::vector<Vec> candidates;
                auto candidate=[&](Vec local) {
                    const Vec point=toWorld(local);
                    if(point.finite()&&(point-anchor).dot(n)<gripEdgeDetail::palmInset*scale&&point.z<anchor.z+.5f*scale)
                        candidates.push_back(point);
                };
                for(int bone=hand?82:67;bone<(hand?97:82);++bone)candidate(body[bone].t);
                for(int digit=0;digit<5;++digit){const int end=(hand?82:67)+digit*3+2;
                    candidate(body[end].t+body[end].q.rotate({0,0,lib.rest[end].t.length()*.75f}));}
                std::sort(candidates.begin(),candidates.end(),[](Vec a,Vec b){return a.z<b.z;});
                float lift=0;
                for(const auto point:candidates) {
                    const auto top=world.ray({point.x,point.y,anchor.z+4*scale},{point.x,point.y,anchor.z-8*scale});
                    if(!top||!top->climbable||!top->point.finite()||!top->normal.finite()||top->normal.z<.95f||
                        std::abs(top->point.z-anchor.z)>1.25f*scale)continue;
                    lift=std::clamp((top->point.z+.5f*scale-point.z)/scale,0.f,6.f);break;
                }
                if(lift<=0)continue;
                const int upper=hand?31:28,elbow=hand?32:29,wrist=hand?39:38;
                const auto orientation=body[wrist].q,local=p[wrist].q;
                solveIK(p,upper,elbow,wrist,body[wrist].t+Vec{0,0,lift},body[elbow].t);
                lib.contactOrientation(p,wrist,local,orientation);
            }
        }

        for(int i=2;i<4&&!hopMotion(motion)&&!running&&!departing&&!runEntry;++i) {
            auto w=lib.world(p);const Vec foot=toWorld(w[ends[i]].t);
            auto hit=world.ray(foot+n*24*scale,foot-n*12*scale);
            if(hit&&hit->climbable&&hit->normal.z<.71f) {
                const Vec surface=hit->normal.unit();const float distance=(foot-hit->point).dot(surface);
                if(distance<10*scale) {
                    Vec target=toLocal(foot+surface*std::min(10*scale,10*scale-distance));
                    const auto orientation=w[ends[i]].q;
                    const auto localOrientation=p[ends[i]].q;
                    solveIK(p,starts[i],mids[i],ends[i],target,w[mids[i]].t);
                    lib.contactOrientation(p,ends[i],localOrientation,orientation);
                }
            }
        }
        for(int bone:ends)p[bone].q=boundedRotation(authored[bone].q,p[bone].q,.2617994f);
        if(!authoredPlayback&&newHop&&edgeAction&&started&&previous.size()==p.size()) {

            const float limit=12.566371f*dt;
            for(int hand=0;hand<2;++hand) {
                const auto load=edgeWeights(hand);
                if(std::max(load[0],load[1])>=.15f)continue;
                const int upper=hand?31:28;
                std::vector<int> chain;
                float largest=0;
                for(int bone=upper;bone<int(p.size());++bone) {
                    int parent=bone;
                    while(parent>upper)parent=lib.parents[parent];
                    if(parent==upper){chain.push_back(bone);largest=std::max(largest,angleBetween(previous[bone].q,p[bone].q));}
                }
                if(largest<=limit)continue;
                const Pose target=p;Pose accepted=p;
                for(int bone:chain)accepted[bone].q=previous[bone].q;
                float low=0,high=1;
                for(int pass=0;pass<12;++pass) {
                    const float amount=(low+high)*.5f;
                    for(int bone:chain)p[bone].q=blend(previous[bone].q,target[bone].q,amount);
                    lib.guardArmBend(p,hand);float angle=0;
                    for(int bone:chain)angle=std::max(angle,angleBetween(previous[bone].q,p[bone].q));
                    if(angle<=limit){low=amount;for(int bone:chain)accepted[bone].q=p[bone].q;}else high=amount;
                }
                for(int bone:chain)p[bone].q=accepted[bone].q;
            }
        }

        if(!authoredPlayback&&started&&previous.size()==p.size()&&dt>0&&!(newMantle&&topPrepared&&traversal.topPreparation()<1.f)) {
            const Pose target=p;float amount=1;
            const float angularLimit=(parkour||running||backFlip?18.849556f:12.566371f)*dt;
            for(std::size_t i=0;i<p.size();++i) {
                const float angle=angleBetween(previous[i].q,target[i].q);
                if(angle>angularLimit)amount=std::min(amount,angularLimit/angle);
            }
            const auto before=lib.world(previous);
            const Vec oldRight{-lastNormal.y,lastNormal.x,0};
            const float speed=backFlip?1500.f:running||kick||runMotion(lastMotion)?1100.f:mantle?750.f:600.f;
            const float limit=(motion!=lastMotion?std::min(speed,runMotion(lastMotion)?1100.f:600.f):speed)*dt;
            auto evaluate=[&](float candidate) {
                for(std::size_t i=0;i<p.size();++i)p[i]=blend(previous[i],target[i],candidate);

                if(!authoredPlayback)lib.guardArmBends(p);
                limitSideArm(p);

                const auto after=lib.world(p);float largest=0;
                for(int bone:ends) {
                    const Vec old=lastPosition+(oldRight*before[bone].t.x-lastNormal*before[bone].t.y+Vec{0,0,before[bone].t.z})*scale;
                    largest=std::max(largest,(toWorld(after[bone].t)-old).length()/scale);
                }
                float ratio=largest/limit;
                for(std::size_t i=0;i<p.size();++i)ratio=std::max(ratio,angleBetween(previous[i].q,p[i].q)/angularLimit);
                return ratio;
            };
            const float initialRatio=evaluate(amount);
            if(initialRatio>1.00001f) {
                Pose accepted=p;float bestRatio=initialRatio;
                const float zeroRatio=evaluate(0);
                if(zeroRatio<=1.00001f) {

                    accepted=p;float low=0,high=amount;
                    for(int pass=0;pass<12;++pass) {
                        const float middle=(low+high)*.5f;
                        if(evaluate(middle)<=1.00001f){low=middle;accepted=p;}
                        else high=middle;
                    }
                } else {

                    if(zeroRatio<bestRatio){bestRatio=zeroRatio;accepted=p;}
                    float ratio=initialRatio;
                    for(int pass=0;pass<12&&ratio>1.00001f;++pass) {
                        amount/=ratio;ratio=evaluate(amount);
                        if(ratio<bestRatio){bestRatio=ratio;accepted=p;}
                    }
                }
                p=std::move(accepted);
            }
        }

        topPalmError=0;edgePalmError=0;
        if(edgeAction) {
            const auto solved=lib.world(p);
            for(int hand=0;hand<2;++hand) {
                const auto handWeights=edgeWeights(hand);
                if(std::max(handWeights[0],handWeights[1])>.95f)
                    edgePalmError=std::max(edgePalmError,(toWorld(lib.palm(solved,hand))-edgeAnchor(hand)).length());
            }
        }
        if(mantle&&std::max(topWeight(0),topWeight(1))>0) {

            const auto supportBody=lib.world(p);
            const Vec supportDelta=topSupport(supportBody);
            shiftCom(p,supportDelta*smooth(topPrepared?traversal.topPreparation()/.40f:samplePhase/.20f)*std::max(topWeight(0),topWeight(1)));
            for(int hand=0;hand<2;++hand) {
                const float load=smooth(topPrepared?traversal.topPreparation()/.40f:samplePhase/.10f)*topWeight(hand);
                if(load<=0)continue;
                const int upper=hand==0?28:31,elbow=hand==0?29:32,wrist=hand==0?38:39;
                const Vec anchor=toLocal(topAnchor(hand));
                for(int iteration=0;iteration<3;++iteration) {
                    const auto w=lib.world(p);const auto palm=lib.palm(w,hand);
                    const Vec target=w[wrist].t+(anchor-palm)*load;

                    const auto orientation=w[wrist].q,localOrientation=authored[wrist].q;

                    auto preferred=p;preferred[upper]=authored[upper];preferred[elbow]=authored[elbow];
                    const auto preferredWorld=lib.world(preferred);
                    solveIK(p,upper,elbow,wrist,target,preferredWorld[elbow].t);
                    lib.contactOrientation(p,wrist,localOrientation,orientation);
                }
                if(load>.98f)topPalmError=std::max(topPalmError,(anchor-lib.palm(lib.world(p),hand)).length()*scale);
            }
        }
        if(newMantle&&traversal.preciseTopContacts())for(int hand=0;hand<2;++hand) {
            const Vec press=topAnchor(hand);
            const auto realTop=world.ray(press+Vec{0,0,4*scale},press-Vec{0,0,8*scale});
            if(!realTop||!realTop->climbable||!realTop->point.finite()||!realTop->normal.finite()||realTop->normal.z<.95f||std::abs(realTop->point.z-(press.z-.8f*scale))>1.25f*scale)continue;

            auto body=lib.world(p);float lift=0;
            const float adapt=topWeight(hand);
            for(int digit=0;digit<5;++digit)for(int joint=0;joint<3;++joint) {
                const int bone=(hand?82:67)+digit*3+joint;
                body=lib.world(p);
                const Vec from=body[bone].t;
                const Vec delta=joint<2?body[bone+1].t-from:body[bone].q.rotate({0,0,lib.rest[bone].t.length()*.75f});
                if(delta.z>=0)continue;
                const Vec flat=Vec{delta.x,delta.y,0}.unit()*delta.length();
                if(flat.length()<.001f)continue;
                const Quat worldWanted=(Quat::between(delta,flat)*body[bone].q).unit();
                const Quat localWanted=(body[lib.parents[bone]].q.inverse()*worldWanted).unit();
                p[bone].q=blend(p[bone].q,boundedRotation(authored[bone].q,localWanted,.7853982f),adapt);
            }
            body=lib.world(p);

            auto sample=[&](Vec point){const Vec q=toWorld(point);
                if(q.z>=realTop->point.z+1.2f*scale)return;
                const Vec projected{q.x,q.y,realTop->point.z};
                const auto solid=world.ray(projected+Vec{0,0,4*scale},projected-Vec{0,0,8*scale});
                if(solid&&solid->climbable&&solid->point.finite()&&solid->normal.finite()&&solid->normal.dot(realTop->normal)>.95f&&std::abs(solid->point.z-realTop->point.z)<.25f*scale)
                    lift=std::max(lift,(solid->point.z+1.2f*scale-q.z)/scale);};
            for(int bone=hand?82:67;bone<(hand?97:82);++bone)sample(body[bone].t);
            for(int digit=0;digit<5;++digit){const int end=(hand?82:67)+digit*3+2;
                sample(body[end].t+body[end].q.rotate({0,0,lib.rest[end].t.length()*.75f}));}
            lift=std::clamp(lift,0.f,12.f);
            if(lift<=0)continue;
            const int upper=hand?31:28,elbow=hand?32:29,wrist=hand?39:38;
            const auto orientation=body[wrist].q,local=p[wrist].q;
            solveIK(p,upper,elbow,wrist,body[wrist].t+Vec{0,0,lift},body[elbow].t);
            lib.contactOrientation(p,wrist,local,orientation);
        }
        if(newMantle&&!traversal.preciseTopContacts())for(int hand=0;hand<2;++hand) {
            const Vec press=topAnchor(hand),normal=traversal.topHandNormal(hand).unit();
            const Vec localNormal{normal.dot(right),normal.dot(forward),normal.z};
            const auto realTop=world.ray(press+normal*(4*scale),press-normal*(8*scale));
            if(!realTop||!realTop->climbable||!realTop->point.finite()||!realTop->normal.finite()||realTop->normal.dot(normal)<.95f||(realTop->point-(press-normal*(.8f*scale))).length()>1.25f*scale)continue;

            auto body=lib.world(p);float lift=0;
            const float adapt=topWeight(hand);
            for(int digit=0;digit<5;++digit)for(int joint=0;joint<3;++joint) {
                const int bone=(hand?82:67)+digit*3+joint;
                body=lib.world(p);
                const Vec from=body[bone].t;
                const Vec delta=joint<2?body[bone+1].t-from:body[bone].q.rotate({0,0,lib.rest[bone].t.length()*.75f});
                if(delta.dot(localNormal)>=0)continue;
                const Vec flat=(delta-localNormal*delta.dot(localNormal)).unit()*delta.length();
                if(flat.length()<.001f)continue;
                const Quat worldWanted=(Quat::between(delta,flat)*body[bone].q).unit();
                const Quat localWanted=(body[lib.parents[bone]].q.inverse()*worldWanted).unit();
                p[bone].q=blend(p[bone].q,boundedRotation(authored[bone].q,localWanted,.7853982f),adapt);
            }
            body=lib.world(p);

            auto sample=[&](Vec point){const Vec q=toWorld(point);
                const float distance=(q-realTop->point).dot(normal);if(distance>=.4f*scale)return;
                const Vec projected=q-normal*distance;
                const auto solid=world.ray(projected+normal*(4*scale),projected-normal*(8*scale));
                if(solid&&solid->climbable&&solid->point.finite()&&solid->normal.finite()&&solid->normal.dot(realTop->normal)>.95f&&std::abs((solid->point-realTop->point).dot(normal))<.25f*scale)
                    lift=std::max(lift,(.4f*scale-(q-solid->point).dot(normal))/scale);};
            for(int bone=hand?82:67;bone<(hand?97:82);++bone)sample(body[bone].t);
            for(int digit=0;digit<5;++digit){const int end=(hand?82:67)+digit*3+2;
                sample(body[end].t+body[end].q.rotate({0,0,lib.rest[end].t.length()*.75f}));}
            lift=std::clamp(lift,0.f,12.f);
            if(lift<=0)continue;
            const int upper=hand?31:28,elbow=hand?32:29,wrist=hand?39:38;
            const auto orientation=body[wrist].q,local=p[wrist].q;
            solveIK(p,upper,elbow,wrist,body[wrist].t+localNormal*lift,body[elbow].t);
            lib.contactOrientation(p,wrist,local,orientation);
        }

        if(mantle&&traversal.topClearanceProgress(samplePhase)>0)for(int leg=0;leg<2;++leg) {
            const int hip=leg==0?6:9,knee=leg==0?7:10,ankle=leg==0?8:11;
            const auto w=lib.world(p);const auto foot=toWorld(w[ankle].t);

            auto probe=foot-n*(42*scale);probe.z=std::max(foot.z+55*scale,traversal.topLip().z+24*scale);
            auto hit=world.ray(probe,foot-n*(42*scale)-Vec{0,0,16*scale});
            if(hit&&hit->climbable&&hit->normal.z>=.7f&&foot.z<hit->point.z+7*scale) {
                const float inside=(foot-traversal.topLip()).dot(forward);
                const float lift=smooth((inside+42*scale)/(38*scale))*traversal.topClearanceProgress(samplePhase);
                Vec target=foot;target.z+=(hit->point.z+7*scale-foot.z)*lift;
                if(started&&previous.size()==p.size()) {
                    const auto old=lib.world(previous)[ankle].t;
                    const Vec oldRight{-lastNormal.y,lastNormal.x,0};
                    const Vec before=lastPosition+(oldRight*old.x-lastNormal*old.y+Vec{0,0,old.z})*scale;
                    const auto delta=target-before;const float limit=420*dt*scale;
                    if(delta.length()>limit)target=before+delta.unit()*limit;
                }
                solveIK(p,hip,knee,ankle,toLocal(target),w[knee].t);
                lib.contactOrientation(p,ankle,authored[ankle].q,w[ankle].q);
            }
        }
        if(!authoredPlayback&&mantle&&started&&previous.size()==p.size()) {

            const auto oldWorld=lib.world(previous);const Vec oldRight{-lastNormal.y,lastNormal.x,0};
            for(int leg=0;leg<2;++leg) {
                const int hip=leg==0?6:9,knee=leg==0?7:10,ankle=leg==0?8:11;
                const auto w=lib.world(p);
                const Vec old=lastPosition+(oldRight*oldWorld[ankle].t.x-lastNormal*oldWorld[ankle].t.y+Vec{0,0,oldWorld[ankle].t.z})*scale;
                const Vec delta=toWorld(w[ankle].t)-old;const float limit=750*dt*scale;
                if(delta.length()>limit) {
                    solveIK(p,hip,knee,ankle,toLocal(old+delta.unit()*limit),w[knee].t);
                    lib.contactOrientation(p,ankle,authored[ankle].q,w[ankle].q);
                }
            }
        }
        const auto correctedArms=authoredPlayback?0:lib.guardArmBends(p);
        for(int hand=0;hand<2;++hand)if(correctedArms&(1u<<hand)) {
            contacts[hand].retiring=true;rejectIdleContact(hand);
            if(sidewaysRun)sidePalmContact=false;
        }
        if(!authoredPlayback)lib.forearmTwist(p);
        if(!authoredPlayback&&(wallTargets||threepeatMotion(motion)||(threepeatMotion(lastMotion)&&transition<1))) {

            auto legal=[&](Pose& value) {
                lib.guardArmBends(value);
                for(int hand=0;hand<2;++hand)lib.guardWristFlexion(value,hand);
                lib.forearmTwist(value);
            };
            legal(p);
            if(started&&previous.size()==p.size()&&!(newMantle&&topPrepared&&traversal.topPreparation()<1.f)) {
                const Pose desired=p;const float limit=12.566371f*dt;
                float ratio=1;
                for(std::size_t bone=0;bone<p.size();++bone)ratio=std::max(ratio,angleBetween(previous[bone].q,p[bone].q)/limit);
                if(ratio>1.00001f) {
                    float low=0,high=1;Pose accepted=previous;
                    for(int pass=0;pass<14;++pass) {
                        const float amount=(low+high)*.5f;Pose candidate=desired;
                        for(std::size_t bone=0;bone<p.size();++bone)candidate[bone].q=blend(previous[bone].q,desired[bone].q,amount);
                        legal(candidate);float worst=0;
                        for(std::size_t bone=0;bone<p.size();++bone)worst=std::max(worst,angleBetween(previous[bone].q,candidate[bone].q));
                        if(worst<=limit+.00001f){low=amount;accepted=std::move(candidate);}else high=amount;
                    }

                    for(std::size_t bone=0;bone<p.size();++bone)accepted[bone].t=desired[bone].t;
                    p=std::move(accepted);
                }
            }
        }
        if(newMantle&&topPrepared&&traversal.topPreparation()>=1.f&&samplePhase<.20f&&
            topWeight(0)>.98f&&topWeight(1)>.98f) {
            Vec correction=topSupport(lib.world(p));correction.z=std::max(0.f,correction.z);
            shiftCom(p,correction);
        }
        auto clearAdaptiveFeet=[&](Pose& value) {
            if(!newMantle||traversal.preciseTopContacts())return;
            for(int leg=0;leg<2;++leg)for(int pass=0;pass<(traversal.lowTopStep()?4:2);++pass) {
                const int hip=leg?9:6,knee=leg?10:7,ankle=leg?11:8,toe=leg?51:50;
                const auto body=lib.world(value);Vec shift{};
                for(int bone:{ankle,toe}) {
                    const Vec actual=toWorld(body[bone].t);
                    const bool steep=traversal.crestTop();
                    const Vec faceNormal=steep?traversal.topHandNormal(0).unit():n;
                    const auto face=world.ray(actual+faceNormal*((steep?72.f:30.f)*scale),actual-faceNormal*(8*scale));
                    const bool validFace=face&&face->climbable&&face->normal.finite()&&face->normal.dot(faceNormal)>.95f;
                    if(validFace&&(steep||actual.z<traversal.topLip().z-12*scale||traversal.lowTopStep())) {
                        const float gap=(actual-face->point).dot(face->normal);
                        if(gap<.5f*scale) {
                            const Vec candidate=face->normal*(.5f*scale-gap);
                            if(candidate.length()>shift.length())shift=candidate;
                        }
                        continue;
                    }
                    Vec start=actual;start.z=std::max(actual.z+30*scale,traversal.topLip().z+90*scale);
                    const auto hit=world.ray(start,actual-Vec{0,0,20*scale});
                    if(!hit||!hit->climbable||hit->normal.z<.7f)continue;
                    const Vec normal=hit->normal.unit();const float distance=(actual-hit->point).dot(normal);
                    if(distance<.5f*scale) {
                        const Vec candidate=normal*(.5f*scale-distance);
                        if(candidate.length()>shift.length())shift=candidate;
                    }
                }
                if(shift.length()<.0001f)break;
                if(traversal.crestTop()&&traversal.topPreparation()<.75f) {
                    shiftCom(value,Vec{shift.dot(right),shift.dot(forward),shift.z}/scale);continue;
                }
                const Vec desired=body[ankle].t+Vec{shift.dot(right),shift.dot(forward),shift.z}/scale;
                const auto orientation=body[ankle].q,local=value[ankle].q;
                solveIK(value,hip,knee,ankle,desired,body[knee].t);lib.contactOrientation(value,ankle,local,orientation);
            }
        };
        clearAdaptiveFeet(p);
        if(!authoredPlayback&&newMantle&&started&&previous.size()==p.size()) {
            const Pose desired=p;const auto before=lib.world(previous);const Vec oldRight{-lastNormal.y,lastNormal.x,0};
            const float angularLimit=12.566371f*dt;
            const float targetSpeed=750.f;
            const float speed=motion!=lastMotion?std::min(targetSpeed,runMotion(lastMotion)?1100.f:600.f):targetSpeed;
            const float limit=speed*dt*scale;
            auto candidate=[&](float amount) {
                for(std::size_t bone=0;bone<p.size();++bone)p[bone]=blend(previous[bone],desired[bone],amount);
                clearAdaptiveFeet(p);
                if(!authoredPlayback)lib.guardArmBends(p);
                for(int hand=0;hand<2;++hand)lib.guardWristFlexion(p,hand);
                if(!authoredPlayback)lib.forearmTwist(p);
                if(newMantle&&traversal.topPreparation()>=.75f&&samplePhase<.20f) {
                    const auto body=lib.world(p);float lift=0;
                    for(int hand=0;hand<2;++hand)for(int digit=0;digit<5;++digit)for(int joint=0;joint<4;++joint) {
                        const int bone=(hand?82:67)+digit*3+std::min(2,joint);
                        Vec point=body[bone].t;
                        if(joint==3)point=point+body[bone].q.rotate({0,0,lib.rest[bone].t.length()*.75f});
                        const Vec actual=toWorld(point);const float top=traversal.topHand(hand).z;
                        if(actual.z>=top+.2f*scale)continue;
                        const auto hit=world.ray({actual.x,actual.y,top+3*scale},{actual.x,actual.y,top-3*scale});
                        if(hit&&hit->climbable&&hit->normal.z>.95f&&std::abs(hit->point.z-top)<.15f*scale)
                            lift=std::max(lift,(hit->point.z+.2f*scale-actual.z)/scale);
                    }
                    p[4].t.z+=lift;
                }

                if(newMantle&&!traversal.preciseTopContacts()&&traversal.topPreparation()>=.75f) {
                    if((topWeight(0)>.98f&&topWeight(1)>.98f)||(traversal.crestTop()&&std::max(topWeight(0),topWeight(1))>.95f)) {
                        const Vec support=topSupport(lib.world(p));
                        shiftCom(p,support*(traversal.crestTop()?amount:1.f));
                        clearAdaptiveFeet(p);
                    }
                    const auto body=lib.world(p);Vec correction{};
                    for(int hand=0;hand<2;++hand)if(topWeight(hand)>.95f) {
                        const Vec normal=traversal.topHandNormal(hand).unit(),anchor=traversal.topHand(hand);
                        if(normal.z<(traversal.crestTop()?.25f:.7f))continue;
                        for(int digit=0;digit<5;++digit)for(int joint=0;joint<4;++joint) {
                            const int bone=(hand?82:67)+digit*3+std::min(2,joint);
                            Vec point=body[bone].t;
                            if(joint==3)point=point+body[bone].q.rotate({0,0,lib.rest[bone].t.length()*.75f});
                            const Vec actual=toWorld(point);const float distance=(actual-anchor).dot(normal);
                            if(distance>=.2f*scale)continue;
                            const Vec projected=actual-normal*distance;
                            const auto hit=world.ray(projected+normal*(3*scale),projected-normal*(3*scale));
                            if(hit&&hit->climbable&&hit->normal.dot(normal)>.95f&&(hit->point-projected).length()<.25f*scale) {
                                const Vec needed=normal*((.2f*scale-(actual-hit->point).dot(normal))/scale);
                                if(needed.length()>correction.length())correction=needed;
                            }
                        }
                    }
                    shiftCom(p,{correction.dot(right),correction.dot(forward),correction.z});
                }
                if(traversal.crestTop()) {
                    for(int hand=0;hand<2;++hand) {
                        const Vec normal=traversal.topHandNormal(hand).unit(),anchor=traversal.topHand(hand);
                        const Vec localNormal{normal.dot(right),normal.dot(forward),normal.z};
                        const auto face=world.ray(anchor+normal*(4*scale),anchor-normal*(4*scale));
                        if(!face||!face->climbable||!face->normal.finite()||face->normal.dot(normal)<.95f||(face->point-anchor).length()>.25f*scale)continue;
                        for(int digit=0;digit<5;++digit)for(int joint=0;joint<3;++joint) {
                            const int bone=(hand?82:67)+digit*3+joint;const auto body=lib.world(p);
                            const Vec delta=joint<2?body[bone+1].t-body[bone].t:body[bone].q.rotate({0,0,lib.rest[bone].t.length()*.75f});
                            if(delta.dot(localNormal)>=0)continue;
                            const Vec flat=(delta-localNormal*delta.dot(localNormal)).unit()*delta.length();
                            if(flat.length()<.001f)continue;
                            const Quat target=(body[lib.parents[bone]].q.inverse()*Quat::between(delta,flat)*body[bone].q).unit();
                            p[bone].q=boundedRotation(previous[bone].q,boundedRotation(authored[bone].q,target,.7853982f),angularLimit);
                        }
                    }
                    const auto body=lib.world(p);Vec correction{};
                    for(int hand=0;hand<2;++hand) {
                        const Vec normal=traversal.topHandNormal(hand).unit();
                        for(int digit=0;digit<5;++digit)for(int joint=0;joint<4;++joint) {
                            const int bone=(hand?82:67)+digit*3+std::min(2,joint);
                            Vec local=body[bone].t;
                            if(joint==3)local=local+body[bone].q.rotate({0,0,lib.rest[bone].t.length()*.75f});
                            const Vec actual=toWorld(local);
                            const auto hit=world.ray(actual+normal*(32*scale),actual-normal*(8*scale));
                            if(!hit||!hit->climbable||!hit->normal.finite()||hit->normal.dot(normal)<.95f)continue;
                            const float distance=(actual-hit->point).dot(normal);
                            if(distance>=.2f*scale)continue;
                            const Vec needed=normal*((.2f*scale-distance)/scale);
                            if(needed.length()>correction.length())correction=needed;
                        }
                    }
                    shiftCom(p,{correction.dot(right),correction.dot(forward),correction.z});
                    clearAdaptiveFeet(p);
                }
                const auto after=lib.world(p);float ratio=0;
                for(int bone:{8,11,38,39}) {
                    const Vec old=lastPosition+(oldRight*before[bone].t.x-lastNormal*before[bone].t.y+Vec{0,0,before[bone].t.z})*scale;
                    ratio=std::max(ratio,(toWorld(after[bone].t)-old).length()/limit);
                }
                for(std::size_t bone=0;bone<p.size();++bone)ratio=std::max(ratio,angleBetween(previous[bone].q,p[bone].q)/angularLimit);
                return ratio;
            };
            if(candidate(1)>1.00001f) {
                float low=0,high=1;const float zero=candidate(0);Pose accepted=zero<=1.00001f?p:previous;
                for(int pass=0;pass<16;++pass) {
                    const float middle=(low+high)*.5f;
                    if(candidate(middle)<=1.00001f){low=middle;accepted=p;}else high=middle;
                }
                p=std::move(accepted);
            }
        }
        if(newMantle) {

            const auto displayed=lib.world(p);topPalmError=0;
            for(int hand=0;hand<2;++hand)if(topWeight(hand)>.98f&&samplePhase>.04f)
                topPalmError=std::max(topPalmError,(toWorld(lib.palm(displayed,hand))-topAnchor(hand)).length());
        }
        if(wallTargets) {

            const auto displayed=lib.world(p);edgePalmError=0;
            for(int hand=0;hand<2;++hand) {
                const auto handWeights=edgeWeights(hand);
                if(std::max(handWeights[0],handWeights[1])>.95f)
                    edgePalmError=std::max(edgePalmError,(toWorld(lib.palm(displayed,hand))-edgeAnchor(hand)).length());
            }
        }
        if(surfaceGapValid) {
            const auto displayed=lib.world(p);
            for(int hand=0;hand<2;++hand)surfacePalmGaps[hand]=(toWorld(lib.palm(displayed,hand))-measuredPoint).dot(measuredNormal);
        }
        if(authoredPlayback) {
            const bool retained=!motionChanged&&previousAuthored.size()==p.size()&&previous.size()==p.size();
            for(std::size_t bone=0;bone<p.size();++bone) {
                const bool extremity=bone==8||bone==11||bone==38||bone==39;
                auto correction=boundedRotation({},(authored[bone].q.inverse()*p[bone].q).unit(),extremity?.2617994f:.7853982f);
                const auto before=retained?(previousAuthored[bone].q.inverse()*previous[bone].q).unit():Quat{};
                correction=boundedRotation(before,correction,12.566371f*dt);
                p[bone].q=(authored[bone].q*correction).unit();
                Vec delta=p[bone].t-authored[bone].t;
                if(delta.length()>20)delta=delta.unit()*20;
                const Vec prior=retained?previous[bone].t-previousAuthored[bone].t:Vec{};
                const Vec change=delta-prior;const float step=240.f*dt;
                if(change.length()>step)delta=prior+change.unit()*step;
                p[bone].t=authored[bone].t+delta;
            }
            for(int hand=0;hand<2;++hand)if(lib.armBendValid(authored,hand)&&!lib.armBendValid(p,hand)) {
                const int elbow=hand?32:29;const auto desired=p[elbow].q;
                float low=0,high=1;p[elbow].q=authored[elbow].q;
                auto accepted=p[elbow].q;
                for(int pass=0;pass<14;++pass) {
                    const float amount=(low+high)*.5f;p[elbow].q=blend(authored[elbow].q,desired,amount);
                    if(lib.armBendValid(p,hand)){low=amount;accepted=p[elbow].q;}else high=amount;
                }
                p[elbow].q=accepted;contacts[hand].retiring=true;
            }
            previousAuthored=authored;
        } else {
            previousAuthored.clear();
            if(sourceReturn&&previous.size()==p.size()) {
                const float angularLimit=12.566371f*dt,linearLimit=240.f*dt;
                bool settled=transition>=1;
                for(std::size_t bone=0;bone<p.size();++bone) {
                    const float angle=angleBetween(previous[bone].q,p[bone].q);
                    settled=settled&&angle<=angularLimit+.00001f;
                    p[bone].q=boundedRotation(previous[bone].q,p[bone].q,angularLimit);
                    if(bone==0||bone==4) {
                        const auto delta=p[bone].t-previous[bone].t;
                        if(delta.length()>linearLimit){p[bone].t=previous[bone].t+delta.unit()*linearLimit;settled=false;}
                    }
                }
                for(int hand=0;hand<2;++hand)if(lib.armBendValid(previous,hand)&&!lib.armBendValid(p,hand)) {
                    const int elbow=hand?32:29;const auto desired=p[elbow].q;
                    float low=0,high=1;auto accepted=previous[elbow].q;
                    for(int pass=0;pass<14;++pass) {
                        const float amount=(low+high)*.5f;p[elbow].q=blend(previous[elbow].q,desired,amount);
                        if(lib.armBendValid(p,hand)){low=amount;accepted=p[elbow].q;}else high=amount;
                    }
                    p[elbow].q=accepted;settled=false;
                }
                sourceReturn=!settled;
            }
        }
        penultimate=previous;previous=p;previousDt=dt;
        lastPosition=pos;lastNormal=n;lastMotion=motion;lastWallRunDirection=wallRunDirection;lastContextRecovery=contextRecovery;lastWallTargets=wallTargets;started=true;
        return p;
    }
};





