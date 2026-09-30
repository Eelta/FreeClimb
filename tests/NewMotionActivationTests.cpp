#define main threepeatExistingSuiteEntry
#include "ThreepeatMotionTests.cpp"
#undef main

static void resting(const Library& lib,int fps,float scale,float shift,float gap,bool far) {
    Settings c;check(lib.configureThreepeat(c),"configure source capture");
    ThreepeatWorld w;w.boxes={{{-1000,0,-500},{1000,300,c.threepeatHangHeight*scale+shift}}};
    if(far){w.origin={131146.67f,38997.66f,-11793.61f};w.yaw=.633f;}
    auto t=attached(w,lib);t.cfg.contextScale=scale;t.cfg.gap=gap;

    t.position=w.point({0,-gap,0});
    auto ordinary=t;ordinary.cfg.threepeatAnimations=false;
    SurfacePose poses,ordinaryPoses;Pose previous;float maxAngle=0,maxBodyStep=0;unsigned seen=0;
    const float dt=1.f/fps;
    for(int frame=0;frame<fps/3;++frame) {
        const auto moving=t.update(w,{1,0},dt,100);
        check(moving.motion==Motion::right&&!t.preparingEdge(),"ordinary movement remains on its existing source family");
        previous=poses.update(lib,w,t,moving.motion,dt,scale);
        const auto reference=ordinary.update(w,{1,0},dt,100);
        ordinaryPoses.update(lib,w,ordinary,reference.motion,dt,scale);
    }
    const auto from=t.position;Vec old=t.position;
    for(int frame=0;frame<fps*2;++frame) {
        const auto result=t.update(w,{},dt,100);check(!result.released,"resting edge never loses physical support");
        const auto p=poses.update(lib,w,t,result.motion,dt,scale);
        const auto reference=ordinary.update(w,{},dt,100);
        const auto baseline=ordinaryPoses.update(lib,w,ordinary,reference.motion,dt,scale);
        check(p.size()==99,"resting animation uses the complete target skeleton");
        for(std::size_t bone=0;bone<p.size();++bone)
            check(angleBetween(p[bone].q,baseline[bone].q)<.001f&&(p[bone].t-baseline[bone].t).length()<.001f,
                "removing independent39 retains the actual ordinary-stop pose even below its natural palm height");
        for(int hand=0;hand<2;++hand)check(lib.armBendValid(p,hand),"ordinary stopped arms remain in their anatomical elbow branch");
        if(!previous.empty())for(std::size_t bone=0;bone<p.size();++bone)maxAngle=std::max(maxAngle,angleBetween(previous[bone].q,p[bone].q));
        maxBodyStep=std::max(maxBodyStep,(t.position-old).length());old=t.position;previous=p;
        if(result.motion==Motion::contextHang)++seen;
        check(result.motion==Motion::hang&&!t.preparingEdge()&&!t.holdsDestinationEdge(result.motion),
            "ordinary stopped climbing retains hang1 without acquiring an independent39 descriptor");
    }
    std::cout<<"idle fps="<<fps<<" scale="<<scale<<" shift="<<shift<<" gap="<<gap<<" far="<<far
        <<" frames="<<seen<<" angular="<<maxAngle<<" bodyStep="<<maxBodyStep<<'\n';

    check(seen==0,"neutral input never performs the removed independent39 acquisition");
    check((t.position-from).length()<.001f,"stopping never performs the removed automatic39 alignment");
    check(maxBodyStep<=60*dt+.03f&&maxAngle<=12.566371f*dt+.015f,"ordinary stopping blends continuously from the previous moving pose");
    auto departing=t;auto exitPose=poses;auto beforeExit=previous;bool fell=false;
    for(int frame=0;frame<fps;++frame) {
        Input input;if(frame==0){input.y=-1;input.release=input.backDrop=true;}
        const auto out=departing.update(w,input,dt,100);const auto p=exitPose.update(lib,w,departing,out.motion,dt,scale);
        for(std::size_t bone=0;bone<p.size();++bone)
            check(angleBetween(beforeExit[bone].q,p[bone].q)<=12.566371f*dt+.015f,
                "leaving ordinary idle keeps the existing whole-body angular envelope");
        beforeExit=p;if(out.released){fell=true;check(!out.completed,"wall departure remains physical fall");break;}
    }
    check(fell&&!departing.active(),"S plus Space still leaves ordinary idle");

    bool hopSeen=false,landing=false;unsigned ordinaryFrames=0,preparedFrames=0,loadedSamples=0;float loadedPalm=0;Motion last=Motion::none;
    for(int frame=0;frame<fps*4&&ordinaryFrames<unsigned(fps/2);++frame) {
        Input input;if(frame==0){input.x=1;input.hop=true;}
        const auto r=t.update(w,input,dt,1000);check(!r.released,"new manual hop keeps real source/target support");
        last=r.motion;
        if(t.preparingEdge())++preparedFrames;
        const auto p=poses.update(lib,w,t,r.motion,dt,scale),body=lib.world(p);
        for(std::size_t bone=0;bone<p.size();++bone)
            check(angleBetween(previous[bone].q,p[bone].q)<=12.566371f*dt+.015f,"full prepared-hop-to-ordinary chain keeps original bone speed");
        for(int hand=0;hand<2;++hand)check(lib.armBendValid(p,hand),"new hop and ordinary return keep elbow anatomy");
        if(threepeatMotion(r.motion))for(int hand=0;hand<2;++hand) {
            const int elbow=hand?32:29,wrist=hand?39:38,middle=hand?88:73;
            check(std::acos(std::clamp((body[wrist].t-body[elbow].t).unit().dot((body[middle].t-body[wrist].t).unit()),-1.f,1.f))<=1.658064f,
                "new-family handoff preserves the original95-degree wrist guard");
        }
        if(threepeatHop(r.motion)) {
            check(r.motion==Motion::contextHopRight,"explicit right input keeps the correct captured source");hopSeen=true;
            for(int hand=0;hand<2;++hand) {
                const float source=threepeatSourceWeight(false,hand,t.actionProgress()),target=threepeatTargetWeight(false,hand,t.actionProgress());
                if(std::max(source,target)>.95f) {
                    ++loadedSamples;
                    const Vec palm=t.position+(Vec{-t.normal.y,t.normal.x,0}*lib.palm(body,hand).x-t.normal*lib.palm(body,hand).y+Vec{0,0,lib.palm(body,hand).z})*scale;
                    loadedPalm=std::max(loadedPalm,(palm-t.edgeHand(hand,target>source)-Vec{0,0,.8f*scale}).length());
                }
            }
        }else if(hopSeen&&r.motion==Motion::contextHang)landing=true;
        if(hopSeen&&r.motion==Motion::hang&&!t.preparingEdge()&&(t.state==State::wall||t.state==State::ledge)) {
            check(!t.holdsDestinationEdge(r.motion),"ordinary return releases the short39 destination ownership");
            ++ordinaryFrames;
        }else ordinaryFrames=0;
        previous=p;
    }
    std::cout<<"manual side fps="<<fps<<" scale="<<scale<<" shift="<<shift<<" gap="<<gap<<" far="<<far<<" prepared="<<preparedFrames<<" hop="<<hopSeen<<" short39="<<landing<<" ordinary="<<ordinaryFrames<<" palm="<<loadedPalm<<" status="<<t.threepeatStatus()<<" state="<<unsigned(t.state)<<" motion="<<unsigned(last)<<'\n';
    check(hopSeen&&landing&&ordinaryFrames>=unsigned(fps/2),"new side hop prepares, catches briefly with39 then completes its return to ordinary hang1");
    check(loadedSamples>=unsigned(fps/4)&&loadedPalm<5,"the retained new hop still uses the original five-unit loaded-palm budget");
}
static void cancellationAndAbsence(const Library& lib) {
    Settings c;lib.configureThreepeat(c);ThreepeatWorld w;
    w.boxes={{{-1000,0,-500},{1000,300,c.threepeatHangHeight+10}}};
    for(Input input:std::array<Input,3>{Input{-1,0},Input{1,0,false,false,false,false,true},Input{0,-1,true,false,false,true}}) {
        auto t=attached(w,lib);
        auto capture=std::make_unique<TraversalCapture>();TraversalCapture::RecordingWorld recording(w,*capture);
        Input hop{1,0};hop.hop=true;
        capture->begin(t,hop,1.f/60,100);const auto prepared=t.update(recording,hop,1.f/60,100);capture->finish(t,prepared);
        auto decoded=std::make_unique<TraversalCapture>();std::string error;
        check(decoded->deserialize(capture->serialize(),error)&&decoded->replay().matched,
            "genuine explicit-hop preparation retains same-version capture replay");
        check(t.preparingEdge(),"cancellation fixture has a real manual captured-hop preparation");
        const auto result=t.update(w,input,1.f/60,100);
        check(!t.holdsPreparedEdge(Motion::contextHang),"reverse, Shift or departure cancels a pending captured-hop grip");
        check(!threepeatHop(result.motion),"a cancelled prepared hop cannot consume the new traversal input");
    }
    for(int kind=0;kind<5;++kind) {
        auto geometry=w;
        if(kind==0)geometry.boxes[0].high.z=1000;
        if(kind==1)geometry.boxes[0].high.z=c.threepeatHangHeight+25;
        if(kind==2)geometry.boxes[0].high.x=8;
        if(kind==3)geometry.boxes[0].low.z=110;
        if(kind==4)geometry.boxes[0].high.y=4;
        Traversal t;t.cfg.gap=37;t.cfg.radius=31;t.cfg.height=138;lib.configureThreepeat(t.cfg);t.cfg.threepeatAnimations=true;
        t.position={0,-37,0};t.normal=t.surfaceNormal={0,-1,0};t.state=State::wall;
        for(int frame=0;frame<30;++frame) {
            const auto r=t.update(geometry,{},1.f/60,100);
            check(r.motion!=Motion::contextHang&&!t.holdsPreparedEdge(Motion::contextHang),
                "absent lip, unreachable height, missing palm/feet or thin unsupported strip cannot synthesize a new grip");
        }
    }
    auto t=attached(w,lib);Input explicitHop{1,0};explicitHop.hop=true;t.update(w,explicitHop,1.f/60,100);
    check(t.preparingEdge(),"real manual source is prepared before removing a palm");w.boxes[0].high.x=8;
    const auto result=t.update(w,{},1.f/60,100);
    check(!t.preparingEdge()&&result.motion!=Motion::contextHang,
        "a moved or missing palm cancels real prepared-hop acquisition before ownership is committed");

    w.boxes={{{-45,0,-500},{45,300,c.threepeatHangHeight-17}}};
    t=attached(w,lib);Input side{1,0};side.hop=true;t.update(w,side,1.f/60,100);
    check(t.threepeatStatus()==2&&t.contextStatus()==1,
        "a legacy no-source fallback cannot overwrite the new family's no-destination diagnostic");
}
int main(int argc,char** argv) {
    try {
        check(argc==2,"supply runtime motion library");Library lib;check(lib.load(argv[1]),"load runtime source clips");
        cancellationAndAbsence(lib);
        for(int fps:{30,60,120})for(float scale:{1.f,1.03f})for(float shift:{-10.f,0.f,10.f})
            resting(lib,fps,scale,shift,37,false);
        for(int fps:{30,60,120})resting(lib,fps,1.03f,8,45,true);
        std::cout<<"ordinary idle, retained new-hop transitions and geometry/input gates passed\n";return 0;
    } catch(const std::exception& error){std::cerr<<"new motion activation: "<<error.what()<<'\n';return 1;}
}
