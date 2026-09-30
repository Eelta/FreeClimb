#define main fixtureMain
#include "AttachSurfaceTests.cpp"
#undef main
#ifdef FC_GROUND_BASELINE
#include "NativeWalkableApproach.baseline.h"
#else
#include "NativeWalkableApproach.h"
#endif
#include "Controls.h"
#include "TraversalCapture.h"
struct SlopedFootprintWorld:AttachWorld {
    float grade=.69f,planeOffset{},patchBegin=18,patchEnd=40,patchHalfWidth=40;
    bool plane=true;
    std::optional<Hit> ray(Vec from,Vec to)override {
        auto result=AttachWorld::ray(from,to);if(!plane)return result;
        const auto a=local(from),d=local(to)-a;
        const float speed=d.z-grade*d.y;
        if(std::abs(speed)<1e-8)return result;
        const float phase=(grade*(a.y+50)+planeOffset-a.z)/speed;
        if(phase<=1e-6f||phase>1)return result;
        const auto p=a+d*phase;
        if(p.y+50<patchBegin||p.y+50>patchEnd||std::abs(p.x)>patchHalfWidth)return result;
        if(result&&(result->point-from).length()<=(to-from).length()*phase)return result;
        return Hit{global(p),direction(Vec{0,-grade,1}.unit()),true};
    }
};
struct SelectedLowWorld:SlopedFootprintWorld {
    float tilt=.06f,top=20.27f,depth=100,treadGrade{};bool object=true;
    std::optional<Hit> ray(Vec from,Vec to)override {
        auto best=SlopedFootprintWorld::ray(from,to);if(!object)return best;
        const auto a=local(from),d=local(to)-a;
        const float slope=tilt/std::sqrt(1-tilt*tilt);
        const std::array<Vec,6> normals{{{1,0,0},{-1,0,0},{0,-1,slope},{0,1,-slope},{0,-treadGrade,1},{0,0,-1}}};
        const std::array<float,6> limits{160,160,0,depth,top,1000};
        double entry=0,leave=1;Vec normal{};bool valid=true;
        for(unsigned i=0;i<normals.size();++i) {
            const double remainder=limits[i]-normals[i].dot(a),speed=normals[i].dot(d);
            if(std::abs(speed)<1e-8){if(remainder<0)valid=false;continue;}
            const double phase=remainder/speed;
            if(speed<0){if(phase>entry){entry=phase;normal=normals[i].unit();}}else leave=std::min(leave,phase);
        }
        const float prior=best?(best->point-from).length()/std::max(.001f,(to-from).length()):2;
        if(valid&&entry<=leave&&entry>1e-6&&entry<=1&&entry<prior)
            return Hit{global(a+d*float(entry)),direction(normal),true};
        return best;
    }
};
static Settings geometryBody(){Settings cfg;cfg.radius=31;cfg.gap=37;cfg.height=138;return cfg;}
int main(){try {
    unsigned checks=0,failures=0,groundCases=0,fallbackCases=0,peak=0,actualCandidates=0,actualLow=0,raisedCandidates=0;
    for(float yaw:{0.f,.73f,1.57f})for(Vec origin:{Vec{},Vec{134767.25f,39721.29f,-11971.05f}}) {
        auto ground=[&](SlopedFootprintWorld w,bool expected,const char* label) {
            w.rotation=yaw;w.origin=origin;unsigned casts=0;
            const auto support=entryGroundSupport([&](Vec a,Vec b){++casts;return w.ray(a,b);},w.global({0,-50,0}),geometryBody(),.707107f);
            peak=std::max(peak,casts);++checks;++groundCases;
            if(bool(support.floor)!=expected){++failures;std::cout<<"FAIL "<<label<<" yaw="<<yaw<<" far="<<(origin.x!=0)<<" samples="<<support.samples<<" casts="<<casts<<'\n';}
            if(support.floor) {
                check(support.samples>=3,"outer native footprint has three adjacent actual coplanar support points");
                check(std::abs(support.height-w.global({0,-50,w.planeOffset}).z)<.03f,"sloped support uses plane height at actor root");
                check(support.height-w.global({0,-50,0}).z<=6.02f,"expanded ray coverage does not expand root-height support tolerance");
            }
        };
        for(float grade:{.35f,.69f,.95f})for(float offset:{0.f,2.62f}) {
            SlopedFootprintWorld w;w.grade=grade;w.planeOffset=offset;
            ground(w,true,"uphill part of real native footprint survives center and inner-ring terrain gaps");
        }
        SlopedFootprintWorld high;high.planeOffset=6.1f;ground(high,false,"projected floor above original root support limit stays rejected");
        auto low=high;low.planeOffset=-16.1f;ground(low,false,"projected floor below original root support limit stays rejected");
        auto missing=high;missing.plane=false;ground(missing,false,"native grounded state cannot manufacture a floor");
        auto isolated=high;isolated.planeOffset=0;isolated.patchHalfWidth=1;ground(isolated,false,"single forward footprint witness cannot manufacture support");
        auto steep=high;steep.planeOffset=0;steep.grade=1.05f;ground(steep,false,"steep surface is not native floor");
#ifndef FC_GROUND_BASELINE
        for(float tilt:{-.044f,.06f,.114f,.2f})for(float top:{18.49f,20.27f,38.f,70.f,1000.f})for(bool peripheral:{false,true}) {
            SelectedLowWorld w;w.rotation=yaw;w.origin=origin;w.tilt=tilt;w.top=top;
            w.plane=peripheral;
            if(!peripheral)w.boxes={{{-1000,-1000,-1000},{1000,1000,0}}};
            const auto feet=w.global({0,-50,0}),facing=w.direction({0,1,0});
            Traversal candidate;candidate.cfg=geometryBody();
            const bool accepted=candidate.attach(w,feet,facing,1000,60,false,true);
            if(!accepted&&peripheral) {
                check(!candidate.active()&&!candidate.entryFallbackTopRise(),"an actual uphill body obstruction retains native ownership before attachment");
                check(candidate.lastFailure==AttachFailure::clearance||candidate.lastFailure==AttachFailure::support,
                    "peripheral uphill platform can only reject its actual body route or absent upper support");
                ++checks;continue;
            }
            if(!accepted)std::cerr<<"actual attach miss tilt="<<tilt<<" top="<<top<<" yaw="<<yaw<<" far="<<(origin.x!=0)<<" reason="<<name(candidate.lastFailure)<<'\n';
            check(accepted,"real tilted low face, higher ledge and full wall candidates remain geometrically discoverable");
            ++actualCandidates;actualLow+=top<=48;const auto selected=candidate.entryFallbackTopRise();
            const auto result=groundedLowTopFallback(candidate.cfg,true,selected);
            const bool expected=selected&&*selected<=48.02f;
            if(result.excludes()!=expected)std::cerr<<"actual selected tilt="<<tilt<<" top="<<top<<" peripheral="<<peripheral<<" yaw="<<yaw<<" far="<<(origin.x!=0)<<" fallback="<<(selected?*selected:-1000.f)<<" end="<<w.local(candidate.entryTarget()).z<<'\n';
            check(result.excludes()==expected,"only the actual selected short platform is intercepted before native ownership");
            if(top<=48)check(expected,"a discovered short obstacle cannot retain traversal ownership from its local platform or adjacent same-height ground");
            check(groundedLowTopFallback(candidate.cfg,false,selected).casts==0&&!groundedLowTopFallback(candidate.cfg,false,selected).excludes(),
                "identical falling low-platform regrab stays eligible");
            if(top<=48) {
                check(selected&&*selected<=48.02f,"Core records a bounded actual selected standing lip relative to original root");
                if(tilt>=0)check(std::abs(*selected-top)<.03f,"forward-leaning low face records its exact selected top lip");
                const auto support=entryGroundSupport([&](Vec a,Vec b){return w.ray(a,b);},feet,candidate.cfg,.707107f);
                if(peripheral)check(support.floor&&support.samples>=3,"actual Core-selected slope low object has measured peripheral native support");
                candidate.stop();candidate.cooldown=0;w.object=false;w.boxes.clear();w.plane=false;
                check(!candidate.attach(w,feet,facing,1000,60,false,true)&&!candidate.entryFallbackTopRise(),
                    "a later failed attach clears previous low-platform evidence");
                w.object=true;w.top=1000;
                check(candidate.attach(w,feet,facing,1000,60,false,true)&&!candidate.entryFallbackTopRise(),
                    "a later legitimate full wall cannot inherit old rejected low-platform evidence");
            } else if(top==1000)check(!selected,"complete real wall upper grips never depend on low-top fallback");
            ++checks;
        }
        for(float rootHeight:{6.f,12.f})for(float top:{18.49f,20.27f}) {
            SelectedLowWorld w;w.rotation=yaw;w.origin=origin;w.tilt=-.044f;w.top=top;w.plane=false;
            w.boxes={{{-1000,-1000,-1000},{1000,1000,0}}};
            Traversal candidate;candidate.cfg=geometryBody();const auto feet=w.global({0,-50,rootHeight});
            check(candidate.attach(w,feet,w.direction({0,1,0}),1000,60,false,true),"actual root-offset small obstacle candidate is discoverable");
            const auto selected=candidate.entryFallbackTopRise();
            check(selected&&std::abs(*selected+rootHeight)<.03f,"actual verified same-ground fallback records negative height relative to supported root");
            check(groundedLowTopFallback(candidate.cfg,true,selected).excludes(),"actual below-root fallback cannot acquire climbing ownership");
            check(!groundedLowTopFallback(candidate.cfg,false,selected).excludes(),"falling onto the identical low edge remains eligible");
            TraversalCapture capture,decoded;capture.begin(candidate,{},1.f/60,1000);
            TraversalCapture::RecordingWorld recorder(w,capture);
            const auto frame=candidate.update(recorder,{},1.f/60,1000);capture.finish(candidate,frame);
            const auto encoded=capture.serialize();std::string error;
            check(decoded.deserialize(encoded,error)&&decoded.serialize()==encoded,
                "selected negative low-top metadata retains exact state through current capture layout");
            check(decoded.replay().matched,"actual low-face frame replays every unchanged query with selected fallback evidence");
            ++checks;++actualCandidates;++actualLow;
        }
        for(float treadGrade:{.1f,.3f}) {
            SelectedLowWorld w;w.rotation=yaw;w.origin=origin;w.tilt=.06f;w.top=36;w.treadGrade=treadGrade;w.plane=false;
            w.boxes={{{-1000,-1000,-1000},{1000,1000,0}}};
            const auto feet=w.global({0,-50,0});Traversal candidate;candidate.cfg=geometryBody();
            check(candidate.attach(w,feet,w.direction({0,1,0}),1000,60,false,true),"selected low face with real ascending top remains discoverable");
            const auto selected=candidate.entryFallbackTopRise();
            check(selected&&*selected>=36&&*selected<48,"short selected lip is measured separately from rising surface behind it");
            for(unsigned frame=0;candidate.state==State::approach&&frame<60;++frame)
                check(!candidate.update(w,{},1.f/60,1000).released,"control fixture follows unchanged actual entry body path");
            check(candidate.state==State::wall,"unfiltered reference enters its actual wall state");
            check(!candidate.update(w,{0,0,false,true},1.f/60,1000).released&&candidate.state==State::mantle,
                "unfiltered reference verifies the same actual low-top mantle");
            check(std::abs(candidate.topLip().z-feet.z-*selected)<.03f,
                "entry evidence comes from actual lip rather than a deeper elevated standing point");
            check(groundedLowTopFallback(candidate.cfg,true,selected).excludes(),"ascending top does not change the short selected front lip into a high-wall permission");
            ++checks;++actualCandidates;++actualLow;
        }
        AttachWorld raised;raised.rotation=yaw;raised.origin=origin;
        raised.boxes={{{-1000,-1000,-1000},{1000,1000,0}},{{-1000,94.5f,-1000},{1000,500,1000}},{{22.5f,30,7},{60,85,8}}};
        Traversal lifted;lifted.cfg=geometryBody();
        check(lifted.attach(raised,raised.global({0,0,0}),raised.direction({0,1,0}),1000,60,false,true)&&lifted.entryLiftHeight()==24,
            "actual rounded candidate requiring 24-unit lift still accepts a supported full wall");
        check(!lifted.entryFallbackTopRise()&&!groundedLowTopFallback(lifted.cfg,true,lifted.entryFallbackTopRise()).excludes(),
            "raised upper-only candidate never borrows or inherits a low-platform exclusion");
        ++checks;++actualCandidates;++raisedCandidates;
#endif
        for(bool grounded:{false,true})for(float rise:{-20.f,0.f,2.f,18.49f,20.27f,24.f,48.f,48.1f,70.f,1000.f}) {
#ifdef FC_GROUND_BASELINE
            GroundEntryExclusionResult result;
#else
            const auto result=groundedLowTopFallback(geometryBody(),grounded,rise);
#endif
            const bool expected=grounded&&rise<=48.f;
            ++checks;++fallbackCases;
            if(result.excludes()!=expected){++failures;std::cout<<"FAIL verified low-top entry grounded="<<grounded<<" rise="<<rise<<" far="<<(origin.x!=0)<<'\n';}
            check(result.casts==0,"low-top fallback rejection uses the actual selected Core platform without extra casts");
            check(!result.groundFound,"rejecting a verified short platform never claims an invented ground plane");
            if(result.excludes())check(std::abs(result.selectedRise-rise)<.001f,"selected low-top geometry preserves measured rise");
        }
#ifndef FC_GROUND_BASELINE
        for(auto rise:{std::optional<float>{},std::optional<float>{NAN},std::optional<float>{INFINITY}})
            check(!groundedLowTopFallback(geometryBody(),true,rise).excludes(),"absent and invalid platform evidence does not suppress a real wall");
        auto shortBody=geometryBody();shortBody.height=100;
        check(groundedLowTopFallback(shortBody,true,35.f).excludes(),"short body retains original .35-height limit");
        check(!groundedLowTopFallback(shortBody,true,35.1f).excludes(),"short body high ledge remains eligible");
#endif
    }
    for(unsigned mask=0;mask<32;++mask) {
        const bool grounded=groundEntryGeometryAllowed(mask&1,mask&2,mask&4,mask&8,mask&16);
#ifndef FC_GROUND_BASELINE
        const auto result=groundedLowTopFallback(geometryBody(),grounded,20.f);
        check(result.excludes()==(mask==1),"actual jump and airborne catches cannot acquire the grounded low-top rejection");
#endif
        ++checks;
    }
    for(float speed:{0.f,-14.14f,-41.54f,-69.f}) {
        const auto flight=grabFlight(false,false,false,false,speed);
        check(!flight.airborne&&!flight.confirmedAirborne,"native grounded negative vertical velocity remains grounded");
#ifndef FC_GROUND_BASELINE
        check(groundedLowTopFallback(geometryBody(),groundEntryGeometryAllowed(true,false,false,flight.airborne,flight.confirmedAirborne),20.f).excludes(),
            "native on-ground entry still rejects the selected short platform while moving down a stair");
#endif
        ++checks;
    }
    std::cout<<"Ground-entry geometry checks="<<checks<<" slopeFootprints="<<groundCases<<" selectedLowTop="<<fallbackCases<<" actualCandidates="<<actualCandidates<<" interceptedLow="<<actualLow<<" raised24="<<raisedCandidates<<" failures="<<failures<<" peakQueries="<<peak<<'\n';return failures?1:0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
