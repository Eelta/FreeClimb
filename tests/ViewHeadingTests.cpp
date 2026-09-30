#include "ViewHeading.h"
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
int main(){try {
    stableViewAndExactBody();entryReleaseAndNoise();
    std::cout<<"PASS: camera reference smoothing, exact independent body frame, entry/release, facet noise and yaw wrap\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
