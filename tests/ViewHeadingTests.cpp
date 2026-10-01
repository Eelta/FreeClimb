#include "ViewHeading.h"
#include "CameraHeading.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
static void check(bool ok,const char* text){if(!ok)throw std::runtime_error(text);}
static void stableViewAndExactBody() {
    for(int fps:{30,48,60,120}) {
        ViewHeading view;view.begin(6.20f);
        float previous=view.value();
        for(int frame=0;frame<fps*8;++frame) {
            const float time=float(frame+1)/fps;

            const float target=normalizeYaw(6.20f+4.f*time+(time>3.f?.8f:0.f));
            const float current=view.advance(target,1.f/fps);
            check(std::abs(yawDifference(current,previous))<=ViewHeading::maxSpeed/fps+.00001f,
                "camera reference must respect angular speed at every supported frame rate");
            check(std::abs(yawDifference(target,current))<1.9f,"rapid cylinder motion cannot accumulate a half-turn lag");
            const Quat parent=Quat::axis({0,0,1},-previous);
            const Transform source{{8,14,7},Quat::axis({1,0,0},frame%2?1.1f:0.f),{1,1,1}};
            const auto local=WallYawFrame(parent,target).toParent(source);
            const auto actual=(parent*local.q).unit();
            const auto expected=(Quat::axis({0,0,1},-target)*source.q).unit();
            check(angleBetween(actual,expected)<.001f,
                "camera smoothing cannot delay the actual body facing during climb/run switches");
            previous=current;
        }
        const float endpoint=view.value();
        view.advance(NAN,.016f);view.advance(1.f,NAN);view.advance(1.f,0);view.advance(1.f,-1);
        check(view.value()==endpoint,"invalid or zero time preserves view state");
    }
}
static void entryReleaseAndNoise() {
    for(int fps:{30,48,60,120}) {
        ViewHeading view;view.begin(.4f);
        const float first=view.advance(1.9f,1.f/fps);
        check(std::abs(yawDifference(first,.4f))<.13f,"entry begins from the real camera reference without a snap");
        for(int i=0;i<fps;++i)view.advance(1.9f,1.f/fps);
        check(std::abs(yawDifference(1.9f,view.value()))<.001f,"a stable wall heading settles promptly");
        for(int i=0;i<fps;++i) {
            const float old=view.value();view.advance(1.9f+(i%2?.04f:-.04f),1.f/fps);
            check(std::abs(yawDifference(view.value(),old))<.01f,"alternating triangle normals cannot shake the view by their full amplitude");
        }
        const float released=view.value();view.reset();
        check(view.value()==released&&!view.ready(),"release discards follow ownership without snapping to the unsmoothed wall");
        view.begin(5.8f);check(view.value()==5.8f&&view.speed()==0,"a new session starts at the current native heading");
    }
    ViewHeading view;view.begin(6.27f);view.advance(.01f,.05f);
    check(std::abs(yawDifference(view.value(),6.27f))<.0232f,"zero crossing uses the short turn");
    view.begin(0);view.advance(3.f,2.f);
    check(view.value()<=ViewHeading::maxSpeed*.05f,"a long stall cannot cause a single-frame heading jump");
}
static void independentFreeCamera() {
    for(int fps:{30,48,60,120})for(float direction:{-1.f,1.f}) {
        ViewHeading body;body.begin(6.2f);
        float actor=6.2f,relative=.7f,expected=normalizeYaw(actor+relative);
        float oldWorld=expected,travel=0;
        for(int frame=0;frame<fps*12;++frame) {
            const float look=frame<fps*3?0.f:direction*.6f/fps;
            relative+=look;expected=normalizeYaw(expected+look);
            const float target=normalizeYaw(6.2f+direction*4.f*(frame+1)/fps);
            const auto view=CameraHeading::capture(actor,relative);
            check(view.has_value(),"a finite active camera creates a per-write heading snapshot");
            const float previous=actor;
            actor=body.advance(target,1.f/fps);
            travel+=yawDifference(actor,previous);
            relative=*view->relativeTo(actor);
            const float currentWorld=normalizeYaw(actor+relative);
            check(std::abs(yawDifference(currentWorld,expected))<.0006f,
                "continuous cylinder turns cannot add rotation to the player's world-space view");
            check(std::abs(yawDifference(currentWorld,oldWorld)-look)<.00001f,
                "mouse and right-stick look remain live during each body rotation");
            oldWorld=currentWorld;
        }
        check(direction*travel>40.f,"preserving camera direction cannot freeze repeated body circuits around a tree");
    }
}
static void cameraWriteBoundaries() {
    for(float before:{0.f,.1f,3.1f,6.27f})for(float after:{0.f,.02f,3.2f,6.28f})for(float offset:{-2.9f,0.f,2.9f}) {
        const auto view=CameraHeading::capture(before,offset);
        check(view.has_value(),"finite yaw snapshot is valid");
        const float corrected=*view->relativeTo(after);
        check(std::abs(yawDifference(normalizeYaw(after+corrected),normalizeYaw(before+offset)))<.000001f,
            "observed actor yaw preserves camera heading through wrap and partial native turns");
        const float alreadyCompensated=yawDifference(offset-yawDifference(after,before),0);
        check(std::abs(yawDifference(corrected,alreadyCompensated))<.000001f,
            "an existing camera compensation must not be applied a second time");
        check(std::abs(yawDifference(*view->relativeTo(before),offset))<.000001f,
            "a rejected native body turn cannot alter the user's camera direction");
    }
    for(float invalid:{NAN,INFINITY,-INFINITY}) {
        check(!CameraHeading::capture(invalid,0)&&!CameraHeading::capture(0,invalid),
            "invalid camera or actor angles cannot authorize an offset write");
        check(!CameraHeading::capture(0,0)->relativeTo(invalid),
            "an invalid native result cannot propagate into the camera");
    }
    const auto huge=CameraHeading::capture(3e38f,3e38f);
    check(huge&&huge->relativeTo(-3e38f)&&std::isfinite(*huge->relativeTo(-3e38f)),
        "finite large yaw inputs normalize before addition and subtraction");
}
int main(){try {
    stableViewAndExactBody();entryReleaseAndNoise();independentFreeCamera();cameraWriteBoundaries();
    std::cout<<"PASS: camera reference smoothing, exact independent body frame, entry/release, facet noise, yaw wrap, free camera continuity and compensated native turns\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
