#include "AnimationOverrides.h"
#define main surfacePoseThreepeatFixturesMain
#include "ThreepeatMotionTests.cpp"
#undef main

static void wallChain(const Library& lib,int fps,int side,bool far) {
    ThreepeatWorld world;world.boxes={{{-10000,0,-5000},{10000,1000,20000}}};
    if(far){world.origin={75000,-68000,23000};world.yaw=.73f;}
    auto t=attached(world,lib);t.cfg.surfaceActionVariants=true;t.cfg.automaticClimbActions=true;
    t.cfg.autoActionMinSeconds=1;t.cfg.autoActionMaxSeconds=1.6f;
    SurfacePose animator;Pose previous;Motion last=Motion::none;float age=0;
    bool hang=false,leap=false;unsigned downFrames=0;float minFinger=100,maxPalm=0,minFacing=1,maxToeGap=0,maxAngle=0;
    Motion worstMotion=Motion::none;float worstPhase=0,worstAge=0;int worstHand=-1;Vec worstDelta{};
    bool worstPrepared=false,worstDestination=false;
    Motion fingerMotion=Motion::none;float fingerAge=0,fingerPhase=0;int fingerBone=-1;
    float previousFingerClearance=100;bool previouslyWall=false;float incomingFinger=100,preparationFinger=100;
    bool previousPrepared39=false;float incomingPreparationFinger=100,previousPreparationPalm=0,previousPreparationFinger=100;
    float preparationPenetrationIncrease=0,preparationWorstAge=0;int preparationWorstBone=-1;bool preparationWorstTip=false;
    unsigned preparationBoundaries=0;
    unsigned planted=0,restFeet=0;
    auto frame=[&](Input input){
        const auto result=t.update(world,input,1.f/fps,1000);
        check(!result.released,"surface variants retain supported checked wall");
        check((result.motion==Motion::none||isActiveMotion(result.motion)),"enabled side variants cannot restore the retired descending catch");
        if(result.motion==Motion::down)++downFrames;
        const auto p=animator.update(lib,world,t,result.motion,1.f/fps,1);const auto body=lib.world(p);
        const Vec n=t.normal,right{-n.y,n.x,0},forward=n*-1;
        auto point=[&](Vec v){return t.position+right*v.x+forward*v.y+Vec{0,0,v.z};};
        auto vector=[&](Vec v){return right*v.x+forward*v.y+Vec{0,0,v.z};};
        age=result.motion==last?age+1.f/fps:0;last=result.motion;
        float frameFinger=100;int frameFingerBone=-1;bool frameFingerTip=false;
        for(int bone=67;bone<97;++bone) {
            const auto local=world.rotate(point(body[bone].t)-world.origin,-world.yaw);
            if(-local.y<frameFinger){frameFinger=-local.y;frameFingerBone=bone;frameFingerTip=false;}
        }
        for(int hand=0;hand<2;++hand)for(int digit=0;digit<5;++digit) {
            const int end=(hand?82:67)+digit*3+2;

            const Vec tip=body[end].t+body[end].q.rotate({0,0,lib.rest[end].t.length()*.75f});
            const auto local=world.rotate(point(tip)-world.origin,-world.yaw);
            if(-local.y<frameFinger){frameFinger=-local.y;frameFingerBone=end;frameFingerTip=true;}
        }
        const bool wallNow=t.usesWallTargets(result.motion);
        const bool prepared39=wallNow&&result.motion==Motion::contextHang&&t.holdsPreparedEdge(result.motion);
        if(prepared39&&!previousPrepared39)incomingPreparationFinger=previousFingerClearance;
        if(prepared39) {
            const float added=std::max(0.f,-frameFinger)-std::max(0.f,-incomingPreparationFinger);
            if(added>preparationPenetrationIncrease){preparationPenetrationIncrease=added;preparationWorstAge=age;
                preparationWorstBone=frameFingerBone;preparationWorstTip=frameFingerTip;}
        }
        if(wallNow&&!previouslyWall)incomingFinger=std::min(incomingFinger,previousFingerClearance);
        if(wallNow&&result.motion==Motion::hang&&t.holdsPreparedEdge(result.motion))preparationFinger=std::min(preparationFinger,frameFinger);
        previousFingerClearance=frameFinger;previouslyWall=wallNow;
        for(int hand=0;hand<2;++hand)check(lib.armBendValid(p,hand),"wall variants preserve anatomical elbow branch");
        if(!previous.empty())for(std::size_t bone=0;bone<p.size();++bone) {
            const float angle=angleBetween(previous[bone].q,p[bone].q);maxAngle=std::max(maxAngle,angle);
            check(angle<=12.566371f/fps+.015f,"all99 wall-variant bones retain existing angular budget");
        }
        if(t.usesWallTargets(result.motion)) {
            hang|=result.motion==Motion::contextHang;leap|=threepeatHop(result.motion);
            for(int hand=0;hand<2;++hand) {
                float a=0,b=0;
                if(t.holdsDestinationEdge(result.motion))b=1;
                else if(t.holdsPreparedEdge(result.motion))a=t.preparedEdgeWeight(result.motion);
                else if(threepeatHop(result.motion)){a=threepeatSourceWeight(result.motion==Motion::contextHopLeft,hand,t.actionProgress());b=threepeatTargetWeight(result.motion==Motion::contextHopLeft,hand,t.actionProgress());}
                const int wrist=hand?39:38,elbow=hand?32:29,middle=hand?88:73;
                check(std::acos(std::clamp((body[wrist].t-body[elbow].t).unit().dot((body[middle].t-body[wrist].t).unit()),-1.f,1.f))<=1.658064f,"wall captures retain95degree wrist flexion guard");
                const bool settledPose=(result.motion==Motion::hang&&t.holdsPreparedEdge(result.motion))?age>.18f:true;
                if(std::max(a,b)>.95f&&settledPose) {
                    const Vec hit=t.edgeHand(hand,b>a),normal=t.edgeContactNormal(hand,b>a);
                    const float palmOffset=result.motion==Motion::contextHang||threepeatHop(result.motion)?capturedWallPalmOffset:6.f;
                    const Vec target=hit+normal*palmOffset;
                    const float error=(point(lib.palm(body,hand))-target).length();
                    if(error>maxPalm){maxPalm=error;worstMotion=result.motion;worstPhase=t.actionProgress();worstAge=age;
                        worstPrepared=t.holdsPreparedEdge(result.motion);worstDestination=t.holdsDestinationEdge(result.motion);
                        worstHand=hand;worstDelta=point(lib.palm(body,hand))-target;}
                    minFacing=std::min(minFacing,vector(sideRunPalmNormal(body,hand)).dot(normal*-1));
                    auto measureFinger=[&](Vec p,int bone) {
                        const float distance=(point(p)-hit).dot(normal);
                        if(distance<minFinger){minFinger=distance;fingerMotion=result.motion;fingerAge=age;
                            fingerPhase=t.actionProgress();fingerBone=bone;}
                    };
                    for(int finger=hand?82:67;finger<(hand?97:82);++finger)measureFinger(body[finger].t,finger);

                    for(int digit=0;digit<5;++digit) {
                        const int end=(hand?82:67)+digit*3+2;
                        const Vec tip=body[end].t+body[end].q.rotate({0,0,lib.rest[end].t.length()*.75f});
                        measureFinger(tip,end);
                    }
                    ++planted;
                }
            }
            if(result.motion==Motion::contextHang&&
                (t.holdsDestinationEdge(result.motion)||(t.holdsPreparedEdge(result.motion)&&t.preparedEdgeWeight(result.motion)>.95f)))for(int toe:{50,51}) {
                const auto local=world.rotate(point(body[toe].t)-world.origin,-world.yaw);
                maxToeGap=std::max(maxToeGap,-local.y);++restFeet;
            }
        }
        if(prepared39||(threepeatHop(result.motion)&&previousPrepared39)) {
            float palm=0;
            for(int hand=0;hand<2;++hand) {
                const Vec target=t.edgeHand(hand,false)+t.edgeContactNormal(hand,false)*capturedWallPalmOffset;
                palm=std::max(palm,(point(lib.palm(body,hand))-target).length());
            }
            if(prepared39){previousPreparationPalm=palm;previousPreparationFinger=frameFinger;}
            else {
                ++preparationBoundaries;
                std::cout<<"prepare boundary fps="<<fps<<" side="<<side<<" far="<<far<<" lastPalm="<<previousPreparationPalm
                    <<" firstPalm="<<palm<<" lastFinger="<<previousPreparationFinger<<" firstFinger="<<frameFinger<<'\n';
                check(previousPreparationPalm<5.01f&&palm<5.01f,"last39 preparation and first40/41 retain the original loaded palm envelope");
                check(previousPreparationFinger>=-.15f&&frameFinger>=-.15f,"last39 preparation and first40/41 retain original finger clearance");
            }
        }
        previousPrepared39=prepared39;
        previous=p;return result;
    };
    for(int i=0;i<fps;++i)check(frame({}).motion==Motion::hang,"plain-wall idle does not independently select39");
    for(int i=0;i<fps*5&&!leap;++i)frame({float(side),0});
    for(int i=0;i<fps*3;++i)frame({});
    for(int i=0;i<fps*4;++i)frame({0,-1});
    for(int i=0;i<fps*2;++i)frame({});
    std::cout<<"surface pose fps="<<fps<<" side="<<side<<" far="<<far<<" hang/hop/downFrames="<<hang<<'/'<<leap<<'/'<<downFrames
        <<" minFinger="<<minFinger<<" finger="<<int(fingerMotion)<<'/'<<fingerAge<<'/'<<fingerPhase<<'/'<<fingerBone<<" incomingFinger="<<incomingFinger<<" preparationFinger="<<preparationFinger<<" prepAddedDepth="<<preparationPenetrationIncrease<<" prepAge/bone/tip="<<preparationWorstAge<<'/'<<preparationWorstBone<<'/'<<preparationWorstTip<<" maxPalm="<<maxPalm<<" worst="<<int(worstMotion)<<'/'<<worstPhase<<'/'<<worstHand<<" age="<<worstAge<<" prepared/destination="<<worstPrepared<<'/'<<worstDestination<<" delta="<<worstDelta.x<<','<<worstDelta.y<<','<<worstDelta.z<<" facing="<<minFacing<<" toeGap="<<maxToeGap<<" angle="<<maxAngle<<'\n';
    check(hang&&leap&&downFrames>=unsigned(fps*3)&&planted>fps&&restFeet>=2,
        "plain wall retains supported side-hop preparation/catch and ordinary continuous descent");
    check(minFinger>=-.15f,"final displayed finger segment endpoints do not enter the wall");
    check(maxPalm<5.01f,"final displayed wall grip retains existing five-unit contact envelope");
    check(minFacing>.70f,"wall-grip hand presents its actual anatomical palm toward surface");
    check(maxToeGap<6.f,"settled wall brace toes remain close to their actual support");
    check(preparationBoundaries>0&&preparationPenetrationIncrease<=.15f,
        "unloaded hand-spacing transition cannot deepen the actual incoming finger-segment wall penetration");
}
static void contactKindBlend(const Library& lib,int fps) {
    Settings cfg;check(lib.configureThreepeat(cfg),"kind blend calibrated captures");
    ThreepeatWorld wall,lip;wall.boxes={{{-1000,0,-500},{1000,300,20000}}};
    lip.boxes={{{-1000,0,-500},{1000,300,cfg.threepeatHangHeight}}};
    auto wallState=attached(wall,lib),lipState=attached(lip,lib);wallState.cfg.surfaceActionVariants=true;
    auto prepare=[&](ThreepeatWorld& world,Traversal& state,bool automatic) {

        state.cfg.automaticClimbActions=automatic;
        state.cfg.autoActionMinSeconds=state.cfg.autoActionMaxSeconds=.8f;
        Input request{1,0};request.hop=!automatic;
        auto result=state.update(world,request,1.f/fps,1000);
        for(int frame=0;frame<3000&&!(result.motion==Motion::contextHang&&state.holdsPreparedEdge(result.motion)&&state.preparedEdgeWeight(result.motion)>.99f);++frame)
            result=state.update(world,automatic?Input{1,0}:Input{},.001f,1000);
        check(result.motion==Motion::contextHang&&state.holdsPreparedEdge(result.motion)&&state.preparedEdgeWeight(result.motion)>.99f,
            "contact-kind fixture must reach loaded39 only through a real requested41 preparation");
    };
    prepare(wall,wallState,true);prepare(lip,lipState,false);
    check(wallState.usesWallTargets(Motion::contextHang)&&lipState.usesEdgeTargets(Motion::contextHang)&&!lipState.usesWallTargets(Motion::contextHang),"two real side-hop preparation contact types established");

    SurfacePose pose;Pose previous;
    for(int segment=0;segment<3;++segment)for(int frame=0;frame<fps;++frame) {
        const bool edge=segment==1;auto& world=edge?lip:wall;auto& state=edge?lipState:wallState;
        const auto result=pose.update(lib,world,state,Motion::contextHang,1.f/fps,1);
        if(!previous.empty()) {
            for(std::size_t bone=0;bone<result.size();++bone)
                check(angleBetween(previous[bone].q,result[bone].q)<=12.566371f/fps+.015f,"same-motion contact-kind transition retains angular budget");
            const auto a=lib.world(previous),b=lib.world(result);
            for(int limb:{8,11,38,39})check((a[limb].t-b[limb].t).length()<=600.f/fps+1.f,"same-motion contact-kind transition cannot snap hands or feet");
        }
        previous=result;
    }
}
int main(int argc,char**argv){try{
    check(argc>=2&&argc<=3,"supply motion library and optional HKX directory");Library lib;check(lib.load(argv[1]),"load unchanged source captures");
    if(argc==3) {
        const auto overrides=fc::loadHkxOverrides(lib,argv[2]);
        check(overrides.loaded==activeMotionCount&&overrides.rejected==0&&overrides.missing==0,
            "load every active HKX slot without missing or rejected clips");
        for(Motion motion:activeMotions)check(lib.hasAnimationOverride(motion),"every active slot installs its HKX override");
    }
    for(int fps:{30,60,120})for(int side:{-1,1})for(bool far:{false,true})wallChain(lib,fps,side,far);
    for(int fps:{30,60,120})contactKindBlend(lib,fps);
    std::cout<<"PASS final wall-contact poses, fingers, elbows, wrists and supported feet\n";return 0;
}catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
