#define main threepeatSourceSuiteEntry
#include "ThreepeatMotionTests.cpp"
#undef main
#include <set>
#include "CornerTestWorld.h"

struct MovingResult {std::vector<float> starts;std::set<int> motions;};
static ThreepeatWorld broadWall(const Library& lib,bool lip,bool far=false) {
    Settings c;check(lib.configureThreepeat(c),"calibrated optional source family");
    ThreepeatWorld w;w.boxes={{{-20000,0,-5000},{20000,1000,lip?c.threepeatHangHeight:20000}}};
    if(far){w.origin={131146.67f,38997.66f,-11793.61f};w.yaw=.633f;}
    return w;
}
static MovingResult movingAutomatic(const Library& lib,int fps,Input input,bool lip,bool far) {
    auto w=broadWall(lib,lip,far);auto t=attached(w,lib);t.cfg.automaticClimbActions=true;
    t.cfg.legacyAutomaticHops=!lip;t.cfg.surfaceActionVariants=lip;
    SurfacePose poses;Pose previous;MovingResult result;unsigned counted=0;float maxAngle=0,maxPalm=0;
    for(int frame=0;frame<fps*12;++frame) {
        const auto before=t.position;const auto out=t.update(w,input,1.f/fps,1000);
        check(!out.released,"automatic movement remains on its checked supported route");
        const auto pose=poses.update(lib,w,t,out.motion,1.f/fps,1);
        check(pose.size()==99,"automatic actions evaluate full target skeleton through SurfacePose");
        for(int hand=0;hand<2;++hand)check(lib.armBendValid(pose,hand),"automatic action chain never reverses an elbow");
        for(std::size_t bone=0;bone<pose.size();++bone) {
            check(pose[bone].t.finite()&&std::isfinite(pose[bone].q.dot(pose[bone].q)),"automatic source/body output is finite");
            if(!previous.empty())maxAngle=std::max(maxAngle,angleBetween(previous[bone].q,pose[bone].q));
        }
        if(threepeatHop(out.motion)) {
            const auto body=lib.world(pose);
            for(int hand=0;hand<2;++hand) {
                const float source=threepeatSourceWeight(input.x<0,hand,t.actionProgress());
                const float target=threepeatTargetWeight(input.x<0,hand,t.actionProgress());
                if(std::max(source,target)>.95f)maxPalm=std::max(maxPalm,
                    (posePoint(lib.palm(body,hand),t)-(t.edgeHand(hand,target>source)+Vec{0,0,.8f})).length());
            }
        }
        previous=pose;result.motions.insert(int(out.motion));
        if(t.automaticActionCount()!=counted) {
            check(t.automaticActionCount()==counted+1,"one automated start counts exactly once");
            counted=t.automaticActionCount();result.starts.push_back(float(frame+1)/fps);
            check(out.staminaCost>=15&&out.staminaCost<16,"automatic jump pays existing fifteen-point action cost once");
            check((threepeatHop(out.motion)||out.motion==Motion::hopUp||out.motion==Motion::hopLeft||out.motion==Motion::hopRight)&&!t.runningAction(),"automatic selection only makes same-mode ordinary or contextual hops");
            if(lip)check(out.motion==(input.x<0?Motion::contextHopLeft:Motion::contextHopRight),
                "matching real source and destination lips always prefer the new Manny action");
            else check(out.motion==(input.x<0?Motion::hopLeft:input.x>0?Motion::hopRight:Motion::hopUp),
                "uninterrupted wall uses legal same-direction ordinary hop, not invented Manny lips");
        }
    }
    check(result.starts.size()>=2,"holding movement visibly intersperses multiple complete actions");
    check(lip?(result.starts.front()>=.60f&&result.starts.front()<1.f):
        (result.starts.front()>=2&&result.starts.front()<4.1f),"new real-edge opportunities retain move and prepare time; old hops retain their random wait");
    for(std::size_t i=1;i<result.starts.size();++i)check(result.starts[i]-result.starts[i-1]>=(lip?2.f:2.5f),
        "each family completes its action, cooldown and required movement before another action");
    check(maxPalm<5,"actual new-hop palm solve stays within unchanged five-unit contact tolerance");
    check(maxAngle<=12.566371f/fps+.015f,"all automatic transitions preserve existing final per-frame angle budget");
    std::cout<<"automatic fps="<<fps<<" direction="<<input.x<<','<<input.y<<" lip="<<lip<<" far="<<far
        <<" count="<<result.starts.size()<<" angle="<<maxAngle<<" palm="<<maxPalm<<" starts=";
    for(auto start:result.starts)std::cout<<start<<',';std::cout<<'\n';return result;
}
static void controlsAndReplays(const Library& lib,int fps) {
    const float dt=1.f/fps;auto w=broadWall(lib,false);auto t=attached(w,lib);t.cfg.automaticClimbActions=true;t.cfg.legacyAutomaticHops=true;
    while(t.automaticActionPendingSeconds()>.10f)t.update(w,{0,1},dt,1000);
    const auto fresh=t;
    for(Input cancellation:std::array<Input,4>{Input{},Input{0,-1},Input{-1,0},Input{0,1,false,false,false,false,true}}) {
        t=fresh;
        for(int frame=0;frame<fps;++frame) {
            const auto out=t.update(w,cancellation,dt,1000);
            check(t.automaticActionCount()==0&&!hopMotion(out.motion),"stop, descending, reversal and Shift cancel almost-due automatic action");
            if(cancellation.run)check(std::abs(out.staminaCost-2*t.cfg.drain*dt)<.0001f,"wall-run still costs twice ordinary climbing");
        }
        for(int frame=0;frame<int(fps*1.9f);++frame)t.update(w,{0,1},dt,1000);
        check(t.automaticActionCount()==0,"returning to climb never inherits a stale nearly-due jump");
    }
    {
        t=fresh;const auto solids=w.boxes;w.boxes.clear();
        const auto lost=t.update(w,{0,1},dt,1000);
        check(!lost.released&&t.automaticActionCount()==0,"one missed support frame retains attachment without a jump");
        w.boxes=solids;
        for(int frame=0;frame<int(fps*1.9f);++frame)t.update(w,{0,1},dt,1000);
        check(t.automaticActionCount()==0,"recovered support cannot release a stale nearly-due random action");
    }
    for(int kind=0;kind<5;++kind) {
        auto geometry=w;t=attached(geometry,lib);t.cfg.automaticClimbActions=kind!=0;t.cfg.legacyAutomaticHops=true;
        Input input{0,1};float stamina=1000;
        if(kind==1)stamina=14;
        if(kind==2)input={};
        if(kind==3)input.run=true;

        if(kind==4)geometry.boxes.push_back({{-20000,-105,-5000},{20000,-95,20000}});
        for(int frame=0;frame<fps*7;++frame) {
            const auto out=t.update(geometry,input,dt,stamina);
            check(!out.released,"negative automatic fixtures retain normal movement/support");
            check(t.automaticActionCount()==0&&!hopMotion(out.motion),"disabled, low stamina, stationary, running or obstructed route cannot auto-hop");
        }
    }

    t=attached(w,lib);t.cfg.automaticClimbActions=true;t.cfg.legacyAutomaticHops=true;
    for(int frame=0;frame<fps*9;++frame) {
        auto tape=std::make_unique<TraversalCapture>();TraversalCapture::RecordingWorld recorded(w,*tape);
        tape->begin(t,{0,1},dt,1000);const auto out=t.update(recorded,{0,1},dt,1000);tape->finish(t,out);
        if(frame==0||out.staminaCost>=15) {
            check(tape->complete(),"automatic action query count fits bounded diagnostic capture");
            auto decoded=std::make_unique<TraversalCapture>();std::string error;
            check(decoded->deserialize(tape->serialize(),error)&&decoded->replay().matched,
                "automatic timers, PRNG advancement and actual action start round-trip exactly");
        }
    }
}
static void pendingCancellation(const Library& lib,int fps) {
    auto w=broadWall(lib,true);auto t=attached(w,lib);t.cfg.automaticClimbActions=true;t.cfg.legacyAutomaticHops=false;
    t.cfg.surfaceActionVariants=true;const float dt=1.f/fps;
    for(int frame=0;frame<fps*5&&!t.preparingEdge();++frame)t.update(w,{1,0},dt,1000);
    check(t.preparingEdge()&&t.automaticActionCount()==0,"automatic Manny candidate waits for genuine source transition before release");
    const auto prepared=t;
    for(Input input:std::array<Input,5>{Input{},Input{-1,0},Input{0,-1},Input{1,0,false,false,false,false,true},Input{0,-1,true,false,false,true}}) {
        t=prepared;const auto result=t.update(w,input,dt,1000);
        check(!t.preparingEdge()&&t.automaticActionCount()==0&&!threepeatHop(result.motion),
            "automatic pending grip cancels immediately on neutral, reversal, descent, Shift or departure");
    }
    t=prepared;w.boxes[0].high.x=t.position.x+20;
    const auto changed=t.update(w,{1,0},dt,1000);
    check(!t.preparingEdge()&&t.automaticActionCount()==0,"removed actual source/target cancels uncommitted automatic jump");
}
static void topPriority(const Library& lib,int fps) {
    auto w=broadWall(lib,false);w.boxes[0].high.z=230;
    auto t=attached(w,lib);t.cfg.automaticClimbActions=true;t.cfg.legacyAutomaticHops=true;t.cfg.autoActionMinSeconds=t.cfg.autoActionMaxSeconds=2;
    Input input{0,1};input.mantle=true;bool top=false,complete=false;
    for(int frame=0;frame<fps*8&&!complete;++frame) {
        const auto out=t.update(w,input,1.f/fps,1000);top|=t.state==State::mantle;complete=out.completed;
        check(t.automaticActionCount()==0,"safe automatic top-out retains precedence over random variety");
    }
    check(top&&complete,"normal top-out still completes without being consumed by random scheduler");
}
static void cornerPriority(const Library& lib,int fps) {
    for(bool convex:{false,true}) {
        fc_test::CornerWorld w;w.corner(convex);Traversal t;t.cfg=fc_test::settings();
        t.cfg.automaticClimbActions=true;t.cfg.legacyAutomaticHops=true;check(t.attach(w,{-200,-37,0},{0,1,0},1000),"random-enabled corner attaches on real wall");
        bool entered=false,finished=false;SurfacePose pose;
        for(int frame=0;frame<fps*5&&!finished;++frame) {
            const auto out=t.update(w,{1,0},1.f/fps,1000);pose.update(lib,w,t,out.motion,1.f/fps,1);
            check(!out.released&&!hopMotion(out.motion)&&t.automaticActionCount()==0,"corner route owns movement and never becomes an unsolicited airborne action");
            check(w.clearance(t.position,t.cfg)>=t.cfg.radius-.05f,"random scheduler preserves independent full capsule distance at corners");
            if(t.turningCorner())entered=true;else if(entered)finished=true;
        }
        check(entered&&finished,"enabled random variety leaves both convex and concave turns traversable");
    }
}
int main(int argc,char** argv) {
    try {
        check(argc==2,"supply runtime motion library");Library lib;check(lib.load(argv[1]),"load production animation library");
        std::array<std::vector<float>,5> reference;
        for(int fps:{30,60,120}) {
            unsigned scenario=0;
            for(auto input:std::array<Input,3>{Input{0,1},Input{-1,0},Input{1,0}}) {
                auto result=movingAutomatic(lib,fps,input,false,false);
                if(fps==30)reference[scenario]=result.starts;
                else {check(result.starts.size()==reference[scenario].size(),"automatic action counts do not depend on frame rate");
                    for(std::size_t i=0;i<result.starts.size();++i)check(std::abs(result.starts[i]-reference[scenario][i])<.20f,"automatic starts agree across frame rates");}
                ++scenario;
            }
            for(int direction:{-1,1}) {
                auto result=movingAutomatic(lib,fps,{float(direction),0},true,false);
                if(fps==30)reference[scenario]=result.starts;
                else {check(result.starts.size()==reference[scenario].size(),"new-family action counts do not depend on frame rate");
                    for(std::size_t i=0;i<result.starts.size();++i)check(std::abs(result.starts[i]-reference[scenario][i])<.25f,"new-family source preparation remains frame-rate independent");}
                ++scenario;
            }
            controlsAndReplays(lib,fps);pendingCancellation(lib,fps);topPriority(lib,fps);cornerPriority(lib,fps);
        }
        movingAutomatic(lib,60,{1,0},true,true);
        std::cout<<"automatic climbing action checks passed\n";return 0;
    }catch(const std::exception& error){std::cerr<<"automatic climbing actions: "<<error.what()<<'\n';return 1;}
}
