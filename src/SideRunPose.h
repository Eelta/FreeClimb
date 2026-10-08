#pragma once

struct SideRunFrame {
    Quat rotation{};
    Vec forward{1,0,0},travel{1,0,0},up{0,0,1},right{0,-1,0},normal{0,-1,0};
    int innerHand{};
};

inline SideRunFrame sideRunFrame(float surfaceZ,Vec direction,float leanRadians=.48869219f) {
    const float z=std::isfinite(surfaceZ)?std::clamp(surfaceZ,-.15f,.75f):0.f;
    const float h=std::sqrt(1-z*z);
    if(!direction.finite()||direction.x*direction.x+direction.y*direction.y<.0001f)direction={1,0,0};
    SideRunFrame frame;
    frame.normal={0,-h,z};
    frame.travel=Vec{direction.x,direction.y*z,direction.y*h}.unit();

    const float pitch=std::clamp(std::atan2(direction.y,std::max(.001f,std::abs(direction.x))),-.2443461f,.2443461f);
    frame.forward=Vec{direction.x>=0?std::cos(pitch):-std::cos(pitch),std::sin(pitch)*z,std::sin(pitch)*h};
    Vec upright=Vec{0,0,1}-frame.forward*frame.forward.z-frame.normal*frame.normal.z;
    if(upright.length()<.01f)upright=frame.normal;

    const float lean=std::isfinite(leanRadians)?std::clamp(leanRadians,0.f,.61086524f):.48869219f;
    frame.up=(upright.unit()*std::cos(lean)+frame.normal*std::sin(lean)).unit();
    frame.right=frame.forward.cross(frame.up).unit();
    frame.up=frame.right.cross(frame.forward).unit();
    frame.rotation=Quat::boneFrame(frame.up,frame.right);
    frame.innerHand=direction.x>=0?0:1;
    return frame;
}

inline void sideRunWorldRotation(const Library& lib,Pose& pose,int bone,Quat change) {
    const auto world=lib.world(pose);const int parent=lib.parents[bone];
    pose[bone].q=((parent>=0?world[parent].q.inverse():Quat{})*change*world[bone].q).unit();
}

inline Quat boundedSideRunJoint(Quat authored,Quat desired,Vec axis,float swingLimit,float rollLimit);

inline float sideRunUpperRoll(const Library& lib,const Pose& pose,int hand) {
    const int upper=hand==0?28:31,elbow=hand==0?29:32;
    if(hand<0||hand>1||pose.size()<=std::size_t(upper)||lib.rest.size()<=std::size_t(elbow))return 0;
    auto delta=(lib.rest[upper].q.inverse()*pose[upper].q).unit();
    if(delta.w<0)delta={-delta.x,-delta.y,-delta.z,-delta.w};
    return 2*std::atan2(Vec{delta.x,delta.y,delta.z}.dot(lib.rest[elbow].t.unit()),delta.w);
}

inline bool limitSideRunUpperRoll(const Library& lib,Pose& pose,int hand,float weight=1.f) {
    const int upper=hand==0?28:31,elbow=hand==0?29:32;
    if(hand<0||hand>1||pose.size()<=std::size_t(upper)||lib.rest.size()<=std::size_t(elbow))return false;
    if(!std::isfinite(weight))return false;
    weight=std::clamp(weight,0.f,1.f);if(weight<=0)return false;
    const auto before=pose[upper].q;
    const auto safe=boundedSideRunJoint(lib.rest[upper].q,before,lib.rest[elbow].t,3.14159265f,.785398163f);
    pose[upper].q=blend(before,safe,weight);
    return angleBetween(before,pose[upper].q)>.00001f;
}

inline void applySideRunBrace(const Library& lib,Pose& pose,const SideRunFrame& frame,
        float gaitPhase,float weight=1.f,Motion direction=Motion::none,const Pose* privateReference=nullptr) {
    if(pose.size()<99||!std::isfinite(gaitPhase)||!std::isfinite(weight))return;
    weight=std::clamp(weight,0.f,1.f);if(weight<=0)return;
    const Pose before=pose;
    const float wave=std::sin(gaitPhase*6.283185307f);
    const float side=frame.innerHand==0?1.f:-1.f;

    for(int spine:{24,25,26}) {
        sideRunWorldRotation(lib,pose,spine,
            Quat::axis(frame.up,side*.05817764f)*Quat::axis(frame.right,-.03490659f));
    }
    const int hand=frame.innerHand,clavicle=hand==0?27:30,upper=hand==0?28:31;
    const int elbow=hand==0?29:32,wrist=hand==0?38:39,base=hand==0?67:82;
    sideRunWorldRotation(lib,pose,clavicle,Quat::axis(frame.forward,side*(.06981317f+.01047198f*wave)));

    const auto reference=privateReference?*privateReference:lib.clip(Motion::sideBrace,direction).authoredPlayback?before:lib.sample(Motion::sideBrace,.5f+.10f*wave,direction);
    const auto world=lib.world(pose);
    const Vec wallUp=(Vec{0,0,1}-frame.normal*frame.normal.z).unit();

    const auto neutral=(world[lib.parents[upper]].q*lib.rest[upper].q).unit();
    const Vec elbowDirection=(frame.normal*-1-frame.forward*.65f-wallUp*.58f).unit();
    lib.rotateWorld(pose,upper,
        Quat::between(neutral.rotate(lib.rest[elbow].t),elbowDirection)*neutral*
        Quat::axis(lib.rest[elbow].t,-side*.61086524f));

    const Vec upperAxis=lib.rest[elbow].t.unit();
    const Vec capturedForearm=reference[elbow].q.rotate(lib.rest[wrist].t).unit();
    const Vec hinge=upperAxis.cross(capturedForearm).unit();
    const float capturedFlex=std::acos(std::clamp(upperAxis.dot(capturedForearm),-1.f,1.f));
    const float flexion=(110.f+2.f*wave)*.017453293f;
    pose[elbow].q=(Quat::axis(hinge,flexion-capturedFlex)*reference[elbow].q).unit();
    pose[wrist].q=reference[wrist].q;
    for(int bone=base;bone<base+15;++bone)pose[bone].q=reference[bone].q;
    for(int bone:hand==0?std::array<int,4>{52,53,54,55}:std::array<int,4>{56,57,58,59})
        pose[bone].q=reference[bone].q;

    const auto brace=lib.world(pose);
    const Vec palm=lib.palm(brace,hand);
    const float wallPlane=brace[4].t.dot(frame.normal)-24.f;
    const Vec delta=frame.normal*(wallPlane+6.f-palm.dot(frame.normal));
    const auto authoredUpper=pose[upper].q,authoredElbow=pose[elbow].q;
    lib.ik(pose,upper,elbow,wrist,brace[wrist].t+delta,brace[elbow].t);
    pose[upper].q=boundedSideRunJoint(authoredUpper,pose[upper].q,lib.rest[elbow].t,.61086524f,.20943951f);
    pose[elbow].q=boundedSideRunJoint(authoredElbow,pose[elbow].q,lib.rest[wrist].t,.61086524f,.20943951f);
    limitSideRunUpperRoll(lib,pose,hand);
    lib.guardArmBend(pose,hand);

    const auto legTurn=Quat::between(frame.forward,frame.travel);
    for(int hip:{6,9})sideRunWorldRotation(lib,pose,hip,legTurn);
    for(std::size_t bone=0;bone<pose.size();++bone)pose[bone]=blend(before[bone],pose[bone],weight);
}

inline void placeSideRun(const Library& lib,Pose& pose,const SideRunFrame& frame,
        float wallPlane,float bodyClearance=24.f) {
    if(pose.size()<40)return;
    pose[0].q=(frame.rotation*pose[0].q).unit();
    pose[0].t=frame.rotation.rotate(pose[0].t);
    const auto world=lib.world(pose);
    const float distance=std::clamp(bodyClearance,18.f,32.f);
    pose[0].t=pose[0].t+frame.normal*(wallPlane+distance-world[4].t.dot(frame.normal));
}

inline Vec sideRunPalmProbe(const Library& lib,const Pose& pose,const SideRunFrame& frame) {
    return lib.palm(lib.world(pose),frame.innerHand);
}

inline Quat boundedSideRunJoint(Quat authored,Quat desired,Vec axis,float swingLimit,float rollLimit) {
    Quat delta=(authored.inverse()*desired).unit();
    if(delta.w<0)delta={-delta.x,-delta.y,-delta.z,-delta.w};
    axis=axis.unit();
    const Vec projection=axis*Vec{delta.x,delta.y,delta.z}.dot(axis);
    const Quat roll=Quat{projection.x,projection.y,projection.z,delta.w}.unit();
    const Quat swing=(delta*roll.inverse()).unit();
    return (authored*boundedRotation({},swing,swingLimit)*boundedRotation({},roll,rollLimit)).unit();
}

inline Vec sideRunPalmNormal(const Pose& world,int hand) {
    const int wrist=hand==0?38:39,base=hand==0?67:82;

    const Vec normal=(world[base+12].t-world[base+3].t).cross(world[base+6].t-world[wrist].t).unit();
    return normal*(hand==0?1.f:-1.f);
}

inline float applySideRunPalm(const Library& lib,Pose& pose,const SideRunFrame& frame,
        Vec actualPalmTarget,float weight) {
    if(pose.size()<99||!actualPalmTarget.finite()||!std::isfinite(weight))return 0;
    weight=std::clamp(weight,0.f,1.f);
    const int hand=frame.innerHand,upper=hand==0?28:31,elbow=hand==0?29:32,wrist=hand==0?38:39;
    const Pose before=pose;
    const auto initial=lib.world(pose);
    const Vec initialPalm=lib.palm(initial,hand);
    const float error=(actualPalmTarget-initialPalm).length();
    if(weight<=0)return error;
    const float facing=sideRunPalmNormal(initial,hand).dot(frame.normal*-1);

    const float fit=(1-smooth((error-5.f)/13.f))*smooth((facing-.15f)/.55f);
    weight*=fit;
    if(weight<=.0001f)return error;
    const Vec shoulder=initial[upper].t;
    const float reach=(initial[elbow].t-shoulder).length()+(initial[wrist].t-initial[elbow].t).length();
    const Vec target=initialPalm+(actualPalmTarget-initialPalm)*weight;
    const Vec wristTarget=initial[wrist].t+target-initialPalm;
    if((wristTarget-shoulder).length()>reach*.94f)return error;
    for(int iteration=0;iteration<2;++iteration) {
        const auto world=lib.world(pose);
        lib.ik(pose,upper,elbow,wrist,world[wrist].t+target-lib.palm(world,hand),initial[elbow].t);
        pose[upper].q=boundedSideRunJoint(before[upper].q,pose[upper].q,lib.rest[elbow].t,.61086524f,.20943951f);
        pose[elbow].q=boundedSideRunJoint(before[elbow].q,pose[elbow].q,lib.rest[wrist].t,.61086524f,.20943951f);
        pose[wrist].q=before[wrist].q;
        limitSideRunUpperRoll(lib,pose,hand,weight);
    }
    const auto solved=lib.world(pose);

    const float result=(actualPalmTarget-lib.palm(solved,hand)).length();
    const float plane=actualPalmTarget.dot(frame.normal);
    if(!solved[wrist].t.finite()||!std::isfinite(result)||result>error+.01f||
        solved[elbow].t.dot(frame.normal)<plane+1.f||lib.palm(solved,hand).dot(frame.normal)<plane-.5f) {
        pose=before;return error;
    }
    return result;
}
