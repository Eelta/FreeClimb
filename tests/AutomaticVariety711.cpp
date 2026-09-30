#define main automaticVariety710FixtureMain
#include "AutomaticVarietyTests.cpp"
#undef main

static void diagonalOrdinaryWall711(const Library& lib,int fps,int side,bool far) {
    auto w=flat(far);auto t=attached(w,lib);const float dt=1.f/fps;
    const Input input{float(side),1};const Motion expected=side<0?Motion::contextHopLeft:Motion::contextHopRight;
    unsigned commits=0,frames=0,legacy=0,peakRays=0;float first=-1;bool inFlight=false;
    Vec previous=t.position;
    for(int frame=0;frame<fps*8;++frame) {
        const auto before=w.geometry.rays;const auto result=t.update(w,input,dt,1000);
        peakRays=std::max(peakRays,w.geometry.rays-before);
        require(!result.released&&t.active(),"diagonal variation retains a fully supported wall route");
        require(w.geometry.clearance(t.position,t.cfg)>=t.cfg.radius-.06f,"diagonal captured path preserves the whole cylinder radius");
        require((t.position-previous).length()<900*dt+1.f,"diagonal source path remains continuous across preparation/commit/landing");previous=t.position;
        if(result.motion==Motion::hopLeft||result.motion==Motion::hopRight)++legacy;
        if(threepeatHop(result.motion)) {
            ++frames;inFlight=true;actualWallAnchors(w,t,result.motion);
            require(result.motion==expected,"new variation keeps the commanded side");
            const Vec delta=w.local(t.edgeTarget())-w.local(t.edgeStart());
            require(delta.x*side>70&&delta.z>23.9f&&delta.z<=32.06f,
                "diagonal capture uses its authored lateral span and a bounded positive rise");
            require(Vec{delta.x,delta.z,0}.unit().dot(Vec{float(side),1,0}.unit())>=.85f,
                "captured destination remains in the explicit input cone");
            const float phase=t.actionProgress(),at=phase*threepeatHopSeconds;
            const float extra=t.position.z-t.edgeStart().z-threepeatHopLift(side<0,phase)*t.cfg.contextScale;
            if(at<=(side<0?.23f:.24f))require(std::abs(extra)<.06f,
                "extra diagonal height cannot move a fully loaded source hand before release");
            if(at>=(side<0?.50f:.53f))require(std::abs(extra-delta.z)<.06f,
                "extra diagonal height finishes before target loading and cannot follow lateral overshoot");
        }
        if(t.automaticActionCount()!=commits) {
            require(t.automaticActionCount()==commits+1&&result.motion==expected&&t.state==State::action,
                "only the actual new captured commit increments the automatic counter");
            ++commits;require(t.surfaceActionCount()==commits,"wall-patch counter tracks each new commit once");
            if(first<0)first=float(frame+1)/fps;
        }
    }
    require(commits>=2&&frames>unsigned(fps)&&legacy==0&&inFlight,
        "diagonal wall motion visibly receives complete new clips instead of old side-hop starvation");
    require(t.automaticOpportunityCount()>0&&first>=.99f&&first<2.5f,
        "diagonal intent enters opportunity scans and retains the ordinary fallback deadline");
    require(peakRays<8192,"diagonal acquisition remains inside bounded existing query workloads");
    std::cout<<"711 diagonal fps="<<fps<<" side="<<side<<" far="<<far<<" commits="<<commits
        <<" newFrames="<<frames<<" first="<<first<<" scans="<<t.automaticOpportunityCount()<<" peakRays="<<peakRays<<'\n';
}

static void finiteDiagonalOpportunity711(const Library& lib,int fps,int side,bool far) {
    Settings calibration;require(lib.configureThreepeat(calibration),"use actual authorized 42-clip calibration");
    const float h=300+calibration.threepeatHangHeight+55;
    const float span=calibration.threepeatHopDistance[side<0?0:1];
    VarietyWorld w;w.transform(far);
    auto append=[&](float a,float b,float top){if(side<0){const float old=a;a=-b;b=-old;}w.geometry.boxes.push_back({{a,0,-10000},{b,8,top}});};
    w.geometry.boxes.push_back({{-20000,8,-10000},{20000,1000,20000}});
    append(20,100,h);append(20+span,100+span,h+30);
    auto t=attached(w,lib);bool pending=false,committed=false;float first=-1;
    for(int frame=0;frame<fps*3;++frame) {
        const auto result=t.update(w,{float(side),1},1.f/fps,1000);
        require(!result.released,"bounded diagonal real-edge source stays supported");
        if(t.preparingEdge()&&!pending){pending=true;first=float(frame+1)/fps;}
        if(threepeatHop(result.motion)) {
            require(!t.usesWallTargets(result.motion),"true slanted pair of lips wins before wall-patch fallback");
            require(t.edgeContactNormal(0,false).z>.95f&&t.edgeContactNormal(0,true).z>.95f,
                "real elevated edge contacts retain their actual top normals");
            const auto delta=w.local(t.edgeTarget())-w.local(t.edgeStart());
            require(delta.x*side>80&&std::abs(delta.z-30)<.06f,"actual elevated target sets the diagonal action endpoint");
            committed=true;break;
        }
    }
    require(pending&&committed&&first>=.35f-.04f&&first<1.f,
        "diagonal opportunity scan catches a finite real lip before the random deadline");
    require(t.automaticActionCount()==1&&t.surfaceActionCount()==0,
        "one genuine diagonal opportunity counts once without being labeled a wall patch");
}

static void diagonalCancellationAndReplay711(const Library& lib,int fps) {
    auto w=flat();auto t=attached(w,lib);const float dt=1.f/fps;
    for(int frame=0;frame<fps*4&&!t.preparingEdge();++frame)t.update(w,{1,1},dt,1000);
    require(t.preparingEdge()&&t.automaticActionCount()==0,"diagonal new action has cancellable preparation");
    const auto prepared=t;
    for(Input cancel:std::array<Input,6>{Input{},Input{-1,1},Input{0,1},Input{0,-1},Input{1,1,false,false,false,false,true},Input{0,-1,true,false,false,true}}) {
        t=prepared;const auto result=t.update(w,cancel,dt,1000);
        require(!t.preparingEdge()&&t.automaticActionCount()==0&&!threepeatHop(result.motion),
            "stop/reverse/pure-up/down/Shift/S+Space cancel the uncommitted diagonal variation");
    }
    t=prepared;w.geometry.boxes.clear();t.update(w,{1,1},dt,1000);
    require(!t.preparingEdge()&&t.automaticActionCount()==0,"lost wall cancels a pending diagonal route");
    w=flat();t=prepared;

    w.geometry.boxes.push_back({{-10000,-200,t.position.z+t.cfg.height+12},{10000,30,t.position.z+t.cfg.height+22}});
    for(int frame=0;frame<fps;++frame) {
        const auto result=t.update(w,{1,1},dt,1000);
        require(!threepeatHop(result.motion)&&t.surfaceActionCount()==0,
            "commit rechecks the full source arc and refuses an intervening solid ceiling");
    }
    w=flat(true);t=attached(w,lib);bool pending=false,commit=false,flight=false;
    for(int frame=0;frame<fps*5&&!flight;++frame) {
        auto tape=std::make_unique<TraversalCapture>();tape->begin(t,{1,1},dt,1000);
        TraversalCapture::RecordingWorld recording(w,*tape);const auto before=t.automaticActionCount();
        const auto result=t.update(recording,{1,1},dt,1000);tape->finish(t,result);
        const bool nowPending=t.preparingEdge(),nowCommit=t.automaticActionCount()!=before;
        const bool nowFlight=threepeatHop(result.motion)&&t.actionProgress()>.4f;
        if((nowPending&&!pending)||nowCommit||nowFlight) {
            require(tape->complete(),"diagonal geometry planning and playback fit the tape budget");
            auto decoded=std::make_unique<TraversalCapture>();std::string error;
            require(decoded->deserialize(tape->serialize(),error)&&decoded->replay().matched,
                "diagonal context source, target, scheduler and contact kinds replay exactly");
            pending|=nowPending;commit|=nowCommit;flight|=nowFlight;
        }
    }
    require(pending&&commit&&flight,"capture exercises diagonal preparation, commit, and live flight");
}

static void intentAndModes711(const Library& lib,int fps) {
    for(const Input input:{Input{0,1},Input{.3f,1},Input{-.3f,1},Input{1,-1}}) {
        auto w=flat();auto t=attached(w,lib);const auto before=w.local(t.position);
        for(int frame=0;frame<fps*4;++frame) {
            const auto result=t.update(w,input,1.f/fps,1000);
            require(!threepeatHop(result.motion),"pure/predominantly up and down-side do not receive unsolicited lateral new captures");
            if(input.x==0)require(std::abs(w.local(t.position).x-before.x)<.05f,"pure W never acquires an involuntary sideways route");
        }
    }
    auto w=flat();auto t=attached(w,lib);Input running{1,1};running.run=true;running.hop=true;
    for(int frame=0;frame<fps*4;++frame) {
        const auto result=t.update(w,running,1.f/fps,1000);
        require(!hopMotion(result.motion)&&t.automaticActionCount()==0,"wall-run manual Space remains disabled with diagonal input");
    }

    idleAndExclusions(lib,fps);
}

static void productionCadence711(const Library& lib,int fps,int side) {
    auto w=flat();auto t=attached(w,lib);t.cfg.autoActionMinSeconds=.8f;t.cfg.autoActionMaxSeconds=1.25f;
    float prepare=-1,commit=-1;
    for(int frame=0;frame<fps*3&&commit<0;++frame) {
        const auto result=t.update(w,{float(side),1},1.f/fps,1000);const float now=float(frame+1)/fps;
        require(!result.released,"production cadence retains checked ordinary wall travel");
        if(t.preparingEdge()&&prepare<0)prepare=now;
        if(t.automaticActionCount()){commit=now;require(threepeatHop(result.motion),"production cadence still prioritizes the new captured family");}
    }
    require(prepare>=.8f-.04f&&prepare<1.f&&commit>prepare&&commit<1.4f,
        "production .8..1.25-second deadline is respected rather than silently clamped back to one second");
    std::cout<<"711 production cadence fps="<<fps<<" side="<<side<<" prepare="<<prepare<<" commit="<<commit<<'\n';
}

int main(int argc,char**argv){try {
    require(argc==2,"supply existing licensed motion library");Library lib;require(lib.load(argv[1])&&lib.hasThreepeat(),"load actual 42 authorized clips");
    for(int fps:{30,60,120}) {
        for(int side:{-1,1})for(bool far:{false,true}) {
            diagonalOrdinaryWall711(lib,fps,side,far);
            finiteDiagonalOpportunity711(lib,fps,side,far);
        }
        diagonalCancellationAndReplay711(lib,fps);intentAndModes711(lib,fps);
        for(int side:{-1,1})productionCadence711(lib,fps,side);
    }
    std::cout<<"PASS 0.7.11 diagonal captured selection, actual geometry, intent, revalidation and replay\n";
}catch(const std::exception&e){std::cerr<<"FAIL 0.7.11 automatic variety: "<<e.what()<<'\n';return 1;}}
