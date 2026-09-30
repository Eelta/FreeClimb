#pragma once
#include "Core.h"
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace fc {
inline float smooth(float t) { t=std::clamp(t,0.f,1.f); return t*t*t*(10+t*(-15+6*t)); }
struct Quat {
    float x{},y{},z{},w{1};
    float dot(Quat b) const { return x*b.x+y*b.y+z*b.z+w*b.w; }
    Quat unit() const { float n=std::sqrt(dot(*this)); return n>1e-6f?Quat{x/n,y/n,z/n,w/n}:Quat{}; }
    Quat inverse() const { return {-x,-y,-z,w}; }
    Quat operator*(Quat b) const {
        return {w*b.x+x*b.w+y*b.z-z*b.y,w*b.y-x*b.z+y*b.w+z*b.x,
            w*b.z+x*b.y-y*b.x+z*b.w,w*b.w-x*b.x-y*b.y-z*b.z};
    }
    Vec rotate(Vec v) const { Vec a{x,y,z}; auto t=a.cross(v)*2; return v+t*w+a.cross(t); }
    static Quat axis(Vec v,float radians) { v=v.unit()*std::sin(radians/2); return {v.x,v.y,v.z,std::cos(radians/2)}; }
    static Quat between(Vec a,Vec b) {
        a=a.unit(); b=b.unit(); float d=a.dot(b);
        if(d<-.9999f) { auto axis=a.cross({1,0,0}); if(axis.length()<.01f)axis=a.cross({0,1,0}); return Quat::axis(axis,3.141592654f); }
        auto c=a.cross(b); return Quat{c.x,c.y,c.z,1+d}.unit();
    }
    static Quat boneFrame(Vec direction,Vec hinge) {
        Vec z=direction.unit(),y=z.cross(hinge).unit(),x=y.cross(z).unit();
        float trace=x.x+y.y+z.z;Quat q;
        if(trace>0) {float s=std::sqrt(trace+1)*2;q={(y.z-z.y)/s,(z.x-x.z)/s,(x.y-y.x)/s,s/4};}
        else if(x.x>y.y&&x.x>z.z) {float s=std::sqrt(1+x.x-y.y-z.z)*2;q={s/4,(y.x+x.y)/s,(z.x+x.z)/s,(y.z-z.y)/s};}
        else if(y.y>z.z) {float s=std::sqrt(1+y.y-x.x-z.z)*2;q={(y.x+x.y)/s,s/4,(z.y+y.z)/s,(z.x-x.z)/s};}
        else {float s=std::sqrt(1+z.z-x.x-y.y)*2;q={(z.x+x.z)/s,(z.y+y.z)/s,s/4,(x.y-y.x)/s};}
        return q.unit();
    }
};
inline Quat blend(Quat a,Quat b,float t) {
    float d=a.dot(b); if(d<0) { b={-b.x,-b.y,-b.z,-b.w}; d=-d; }
    float u=1-t,v=t;
    if(d<.9995f) { float angle=std::acos(std::clamp(d,-1.f,1.f)),s=std::sin(angle); u=std::sin((1-t)*angle)/s;v=std::sin(t*angle)/s; }
    return Quat{a.x*u+b.x*v,a.y*u+b.y*v,a.z*u+b.z*v,a.w*u+b.w*v}.unit();
}
inline float angleBetween(Quat a,Quat b) {

    const double dot=double(a.x)*b.x+double(a.y)*b.y+double(a.z)*b.z+double(a.w)*b.w;
    const double aa=double(a.x)*a.x+double(a.y)*a.y+double(a.z)*a.z+double(a.w)*a.w;
    const double bb=double(b.x)*b.x+double(b.y)*b.y+double(b.z)*b.z+double(b.w)*b.w;
    return aa*bb>1e-20?float(2*std::acos(std::clamp(std::abs(dot)/std::sqrt(aa*bb),0.,1.))):3.14159265f;
}
inline Quat boundedRotation(Quat authored,Quat desired,float limit) {
    const float angle=angleBetween(authored,desired);
    return blend(authored,desired,angle>limit?limit/angle:1.f);
}
struct Transform { Vec t; Quat q; Vec s{1,1,1}; };
inline Transform blend(Transform a,Transform b,float t) { return {a.t+(b.t-a.t)*t,blend(a.q,b.q,t),a.s+(b.s-a.s)*t}; }
inline Vec multiply(Vec a,Vec b) { return {a.x*b.x,a.y*b.y,a.z*b.z}; }
inline Transform compose(Transform a,Transform b) { return {a.t+a.q.rotate(multiply(a.s,b.t)),(a.q*b.q).unit(),multiply(a.s,b.s)}; }
using Pose=std::vector<Transform>;
struct Clip { float seconds{},stride{},height{};Vec travel{}; std::vector<Pose> frames; std::vector<std::array<float,4>> contacts; };
struct RotationOverride { std::vector<std::array<Quat,99>> frames;std::array<bool,99> bones{}; };
struct Library;
bool loadAnimationPackFile(Library& library,const std::string& path,std::string& error);
struct Library {
    ThreepeatProfile threepeatProfile=defaultThreepeatProfile();
    bool animationPack{};
    std::string lastLoadError;
    std::vector<std::string> names;
    std::vector<int> parents;
    Pose rest;
    std::array<Clip,motionCount> clips;
    std::array<RotationOverride,motionCount> rotationOverrides;
    bool hasAnimationOverride(Motion motion) const {
        const int index=int(motion)-1;
        return isActiveMotion(motion)&&rotationOverrides[index].frames.size()>=2;
    }
    void clearAnimationOverrides() {for(auto& replacement:rotationOverrides)replacement={};}
    std::uint64_t sourceFingerprint{};
    std::size_t sourceBytes{};
    bool sourceValidated{};
    bool hasLegacyThreepeatFingerBasis() const {

        return sourceValidated&&sourceBytes==9975522&&sourceFingerprint==0x3ee1dcacbb182677ull;
    }
    bool hasThreepeat() const {
        for(int i=legacyMotionCount;i<42;++i)if(clips[i].frames.size()<2)return false;
        return true;
    }
    bool configureThreepeat(Settings& cfg) const {
        if(!hasThreepeat()){cfg.threepeatAnimations=false;return false;}
        cfg.threepeatProfile=threepeatProfile;
        const auto hang=world(clip(Motion::contextHang).frames.front());
        const auto left=palm(hang,0),right=palm(hang,1),middle=(left+right)*.5f;
        cfg.threepeatHangHeight=middle.z;
        cfg.threepeatHandHalfWidth=std::abs(right.x-left.x)*.5f;
        cfg.threepeatHangForward=middle.y;
        cfg.threepeatHangToes={hang[50].t,hang[51].t};
        for(int side=0;side<2;++side) {
            const auto& hop=clip(side==0?Motion::contextHopLeft:Motion::contextHopRight);
            cfg.threepeatHopDistance[side]=std::abs(hop.travel.x);
            cfg.threepeatHopSeconds[side]=hop.seconds;
        }
        const auto& mantle=clip(Motion::contextMantle);
        cfg.threepeatMantleHeight=mantle.height;cfg.threepeatMantleSeconds=mantle.seconds;cfg.threepeatMantleForward=mantle.travel.y;
        const auto mantleStart=world(mantle.frames.front());
        const auto mantleLeft=palm(mantleStart,0),mantleRight=palm(mantleStart,1);
        cfg.threepeatMantlePalmHeight=(mantleLeft.z+mantleRight.z)*.5f;
        cfg.threepeatMantleHalfWidth=std::abs(mantleRight.x-mantleLeft.x)*.5f;
        const auto replanted=world(sampleBase(Motion::contextMantle,threepeatProfile.replantSamplePhase));
        for(int hand=0;hand<2;++hand)cfg.threepeatMantleReplant[hand]=palm(replanted,hand)-palm(mantleStart,hand);
        const bool valid=cfg.threepeatHangHeight>90&&cfg.threepeatHangHeight<175&&
            cfg.threepeatHandHalfWidth>8&&cfg.threepeatHandHalfWidth<40&&
            cfg.threepeatHopDistance[0]>40&&cfg.threepeatHopDistance[1]>40;
        if(!valid)cfg.threepeatAnimations=false;
        return valid;
    }
    struct ArmBendReference {Vec axis{},normal{},direction{};bool valid{};};
    std::array<ArmBendReference,2> armBends{};
    void calibrateArmBends() {
        armBends={};
        const auto& hang=clip(Motion::hang).frames;
        if(hang.empty()||hang.front().size()<40||rest.size()<40||parents.size()<40)return;
        for(int hand=0;hand<2;++hand) {
            const int upper=hand?31:28,elbow=hand?32:29,wrist=hand?39:38;
            if(parents[elbow]!=upper||parents[wrist]!=elbow)continue;
            const auto& capture=hang.front();
            const Vec axis=rest[elbow].t.unit();
            const Vec forearm=capture[elbow].q.rotate(capture[wrist].t).unit();
            const Vec normal=axis.cross(forearm);
            if(axis.length()<.9f||normal.length()<.02f)continue;
            armBends[hand]={axis,normal.unit(),normal.unit().cross(axis).unit(),true};
        }
    }
    float signedArmBend(const Pose& p,int hand) const {
        if(hand<0||hand>1||!armBends[hand].valid||p.size()<40)return 0;
        const int elbow=hand?32:29,wrist=hand?39:38;
        const auto& reference=armBends[hand];
        const Vec direction=p[elbow].q.rotate(p[wrist].t).unit();
        return std::atan2(reference.axis.cross(direction).dot(reference.normal),reference.axis.dot(direction));
    }
    bool armBendValid(const Pose& p,int hand) const {
        if(hand<0||hand>1||!armBends[hand].valid||p.size()<40)return true;
        const int elbow=hand?32:29,wrist=hand?39:38;
        const Vec forearm=p[elbow].q.rotate(p[wrist].t).unit();
        return forearm.finite()&&forearm.dot(armBends[hand].direction)>=-1e-5f;
    }
    bool guardArmBend(Pose& p,int hand) const {
        if(armBendValid(p,hand))return false;
        const int elbow=hand?32:29,wrist=hand?39:38;
        const auto& reference=armBends[hand];
        const Vec forearm=p[elbow].q.rotate(p[wrist].t).unit();
        if(!forearm.finite())return false;

        const Vec projected=forearm-reference.direction*forearm.dot(reference.direction);
        const float margin=std::max(.0001f,std::abs(projected.dot(reference.axis))*.105104235f);
        const Vec legal=(projected+reference.direction*margin).unit();
        p[elbow].q=(Quat::between(forearm,legal)*p[elbow].q).unit();
        return true;
    }

    std::uint8_t guardArmBends(Pose& p) const {
        std::uint8_t result=0;
        for(int hand=0;hand<2;++hand)if(guardArmBend(p,hand))result|=std::uint8_t(1u<<hand);
        return result;
    }
    bool load(const std::string& path) {
        if(path.size()>=5&&path.substr(path.size()-5)==".json")return loadAnimationPackFile(*this,path,lastLoadError);
        return loadLegacy(path);
    }
    bool loadLegacy(const std::string& path) {
        *this={}; std::ifstream in(path,std::ios::binary);
        sourceFingerprint=14695981039346656037ull;
        auto readExact=[&](void* data,std::size_t size) {
            if(!in.read(static_cast<char*>(data),size))return false;
            const auto* bytes=static_cast<const unsigned char*>(data);
            for(std::size_t i=0;i<size;++i)sourceFingerprint=(sourceFingerprint^bytes[i])*1099511628211ull;
            sourceBytes+=size;return true;
        };
        auto read=[&](auto& v){ return readExact(&v,sizeof(v)); };
        std::uint32_t magic{},version{},bones{},count{};
        if(!read(magic)||magic!=0x344D4346||!read(version)||version!=2||!read(bones)||bones!=99||!read(count)||
            (count!=legacyMotionCount&&count!=motionCount)) return false;
        auto transform=[&](Transform& t) {
            if(!read(t.t)||!read(t.q)||!read(t.s)||!t.t.finite()||!t.s.finite()) return false;
            return std::isfinite(t.q.dot(t.q))&&std::abs(t.q.dot(t.q)-1)<.01f&&t.t.length()<1000&&
                t.s.x>.1f&&t.s.y>.1f&&t.s.z>.1f&&t.s.x<5&&t.s.y<5&&t.s.z<5;
        };
        for(std::uint32_t i=0;i<bones;++i) {
            std::int32_t parent{}; std::uint32_t len{};
            if(!read(parent)||parent>=int(i)||parent< -1||!read(len)||len<1||len>100) return false;
            std::string name(len,'\0'); if(!readExact(name.data(),len)) return false;
            Transform t; if(!transform(t)) return false;
            names.push_back(name);parents.push_back(parent);rest.push_back(t);
        }
        for(std::uint32_t clipIndex=0;clipIndex<count;++clipIndex) {
            const bool active=isActiveMotion(Motion(clipIndex+1));
            Clip discarded;auto& c=active?clips[clipIndex]:discarded;
            std::uint32_t frames{};
            if(!read(c.seconds)||!std::isfinite(c.seconds)||c.seconds<=0||c.seconds>10||!read(frames)||frames<2||frames>1201) return false;
            if(!read(c.stride)||!read(c.height)||!read(c.travel)||!std::isfinite(c.stride)||c.stride<0||c.stride>500||!std::isfinite(c.height)||std::abs(c.height)>500||!c.travel.finite())return false;
            if(active){c.frames.assign(frames,Pose(bones));c.contacts.resize(frames);}
            for(std::size_t i=0;i<frames;++i) {
                for(std::size_t bone=0;bone<bones;++bone) {
                    Transform ignored;auto& t=active?c.frames[i][bone]:ignored;if(!transform(t))return false;
                }
                std::array<float,4> ignored;auto& contacts=active?c.contacts[i]:ignored;
                if(!read(contacts))return false;
                for(float weight:contacts)if(!std::isfinite(weight)||weight<0||weight>1)return false;
            }
        }
        if(in.peek()!=std::char_traits<char>::eof())return false;
        calibrateArmBends();
        sourceValidated=true;
        return true;
    }
    Pose sampleBase(Motion motion,float phase) const {
        if(!isActiveMotion(motion))return rest;
        const int index=int(motion)-1;const auto& frames=clips[index].frames;
        if(frames.empty()) return rest;
        float f=std::clamp(phase,0.f,1.f)*float(frames.size()-1);
        auto a=std::min(std::size_t(f),frames.size()-2); Pose out=frames[a];
        for(std::size_t i=0;i<out.size();++i) out[i]=blend(out[i],frames[a+1][i],f-float(a));
        if(index>=legacyMotionCount&&hasLegacyThreepeatFingerBasis()) {

            struct Basis {int bone;Quat right;};
            static constexpr std::array<Basis,10> fixes{{
                {69,{.7019841075f,.08495571464f,-.2038385123f,-.6770899296f}},
                {72,{.2568211257f,.658818841f,-.6836140156f,-.1807557344f}},
                {75,{.2272611558f,.6695911288f,-.7048909068f,-.05593606457f}},
                {78,{-.09134239703f,.7011820078f,-.7069627643f,.01428010501f}},
                {81,{-.2106214762f,.6750078797f,-.6957176328f,.1264115125f}},
                {84,{.6595822573f,.254858911f,.1382157356f,.6934657097f}},
                {87,{.7000772357f,.09946484119f,.01974205114f,.7068301439f}},
                {90,{-.6815471649f,-.1884010732f,-.01532627642f,-.7069396377f}},
                {93,{.7026153207f,-.07958163321f,-.002436876297f,.7071014643f}},
                {96,{.7005730867f,-.09591475874f,-.009442031384f,.707042098f}}
            }};
            for(const auto& fix:fixes)out[fix.bone].q=(out[fix.bone].q*fix.right).unit();
        }
        return out;
    }
    Pose sample(Motion motion,float phase) const {
        Pose result=sampleBase(motion,phase);
        if(!hasAnimationOverride(motion)||result.size()!=99)return result;
        const auto& replacement=rotationOverrides[int(motion)-1];
        const auto& frames=replacement.frames;
        const float frame=std::clamp(phase,0.f,1.f)*float(frames.size()-1);
        const auto first=std::min(std::size_t(frame),frames.size()-2);
        for(std::size_t bone=5;bone<97;++bone)if(replacement.bones[bone])
            result[bone].q=blend(frames[first][bone],frames[first+1][bone],frame-float(first));
        return result;
    }
    const Clip& clip(Motion motion) const {
        static const Clip unavailable;
        return isActiveMotion(motion)?clips[int(motion)-1]:unavailable;
    }
    std::array<float,4> contactWeights(Motion motion,float phase) const {
        const auto& weights=clip(motion).contacts;if(weights.size()<2)return {};
        float f=std::clamp(phase,0.f,1.f)*float(weights.size()-1);
        auto a=std::min(std::size_t(f),weights.size()-2);std::array<float,4> result{};
        for(int i=0;i<4;++i)result[i]=weights[a][i]+(weights[a+1][i]-weights[a][i])*(f-float(a));
        return result;
    }
    Pose world(const Pose& local) const {
        Pose result(local.size());
        for(std::size_t i=0;i<local.size();++i) result[i]=parents[i]<0?local[i]:compose(result[parents[i]],local[i]);
        return result;
    }
    Vec palm(const Pose& worldPose,int hand) const {
        const int wrist=hand==0?38:39,base=hand==0?67:82;
        return worldPose[wrist].t*.5f+(worldPose[base+3].t+worldPose[base+6].t+
            worldPose[base+9].t+worldPose[base+12].t)*.125f;
    }
    bool guardWristFlexion(Pose& pose,int hand,float limit=1.65806279f) const {
        const int elbow=hand?32:29,wrist=hand?39:38,middle=hand?88:73;
        const auto body=world(pose);
        const Vec forearm=(body[wrist].t-body[elbow].t).unit(),finger=(body[middle].t-body[wrist].t).unit();
        const float angle=std::acos(std::clamp(forearm.dot(finger),-1.f,1.f));
        if(angle<=limit||!std::isfinite(angle))return false;
        const auto swing=Quat::between(finger,forearm);
        const auto correction=blend(Quat{},swing,(angle-limit)/angle);
        rotateWorld(pose,wrist,(correction*body[wrist].q).unit());return true;
    }
    void rotateWorld(Pose& p,int index,Quat desired) const {
        auto w=world(p); p[index].q=(parents[index]<0?desired:w[parents[index]].q.inverse()*desired).unit();
    }
    void contactOrientation(Pose& p,int index,Quat authored,Quat desiredWorld) const {
        const auto w=world(p);
        const Quat desired=w[parents[index]].q.inverse()*desiredWorld;

        p[index].q=boundedRotation(authored,desired,.2617994f);
    }
    void forearmTwist(Pose& p) const {
        for(int hand:{38,39}) {
            const Vec axis=rest[hand].t.unit();
            Quat delta=p[hand].q*rest[hand].q.inverse();
            if(delta.w<0)delta={-delta.x,-delta.y,-delta.z,-delta.w};
            const Vec along=axis*Vec{delta.x,delta.y,delta.z}.dot(axis);
            const Quat twist=Quat{along.x,along.y,along.z,delta.w}.unit();
            const int first=hand==38?52:56;
            for(int bone:{first,first+1}) {
                const float amount=std::clamp(rest[bone].t.length()/rest[hand].t.length(),0.f,1.f);
                p[bone].q=blend(Quat{},twist,amount)*rest[bone].q;
            }
        }
    }

    float ik(Pose& p,int a,int b,int c,Vec target,Vec pole) const {
        const int hand=a==28&&b==29&&c==38?0:a==31&&b==32&&c==39?1:-1;
        if(hand>=0)guardArmBend(p,hand);
        auto w=world(p); Vec start=w[a].t,mid=w[b].t,end=w[c].t;
        float upper=(mid-start).length(),lower=(end-mid).length();
        Vec axis=(target-start).unit();
        if(upper<.001f||lower<.001f||axis.length()<.9f)return (target-end).length();
        float distance=std::clamp((target-start).length(),std::abs(upper-lower)+.02f,upper+lower-.04f);
        Vec plane=pole-start;plane=plane-axis*plane.dot(axis);
        if(plane.length()<.01f) {plane=axis.cross({0,1,0}); if(plane.length()<.01f)plane=axis.cross({1,0,0});}
        float along=(upper*upper-lower*lower+distance*distance)/(2*distance);
        const float radial=std::sqrt(std::max(0.f,upper*upper-along*along));
        const auto originalA=p[a].q,originalB=p[b].q,upperWorld=w[a].q;
        auto solve=[&](float side) {
            p[a].q=originalA;p[b].q=originalB;
            const Vec joint=start+axis*along+plane.unit()*(radial*side);
            rotateWorld(p,a,Quat::between(mid-start,joint-start)*upperWorld);
            const auto aimed=world(p);
            rotateWorld(p,b,Quat::between(aimed[c].t-aimed[b].t,start+axis*distance-aimed[b].t)*aimed[b].q);
        };
        solve(1);
        if(hand>=0&&!armBendValid(p,hand)) {

            solve(-1);
            if(!armBendValid(p,hand))guardArmBend(p,hand);
        }
        return (target-world(p)[c].t).length();
    }
};

#include "SideRunPose.h"
#include "PoseContinuation.h"
#include "BackFlipPose.h"
#include "SurfacePose.h"
}
