#pragma once

struct BackFlipBodyPoint {Vec point{};float radius{};};
using BackFlipBody=std::array<BackFlipBodyPoint,13>;

inline BackFlipBody backFlipBody(const Library& lib,const Pose& pose) {
    const auto w=lib.world(pose);
    BackFlipBody body{};
    const std::array<int,13> bones={4,26,36,28,29,38,31,32,39,7,50,10,51};
    const std::array<float,13> padding={12,14,10,6,6,8,6,6,8,7,5,7,5};
    for(std::size_t i=0;i<body.size();++i)body[i]={w[bones[i]].t,padding[i]+2.f};
    body[5].point=lib.palm(w,0);body[8].point=lib.palm(w,1);
    return body;
}

inline void tiltBackFlipAboutCOM(const Library& lib,Pose& pose,float slope) {
    const Vec center=lib.world(pose)[4].t;
    const auto tilt=Quat::axis({1,0,0},-std::asin(slope));
    pose[0].q=(tilt*pose[0].q).unit();
    pose[0].t=tilt.rotate(pose[0].t)+center-tilt.rotate(center);
}

inline void placeBackFlipOut(const Library& lib,Pose& pose,float slope,float gap,float scale) {
    if(pose.size()<99||lib.clip(Motion::backFlipOut).frames.empty())return;
    slope=std::clamp(slope,0.f,.68f);scale=std::clamp(scale,.5f,2.f);
    const float horizontal=std::sqrt(1-slope*slope);
    const Vec outward{0,-horizontal,slope};
    auto first=lib.sample(Motion::backFlipOut,0);
    tiltBackFlipAboutCOM(lib,first,slope);
    float nearest=1e9f;
    for(const auto& marker:backFlipBody(lib,first))nearest=std::min(nearest,marker.point.dot(outward)-marker.radius);
    const float wallPlane=-gap*horizontal/scale;
    const float shift=std::max(0.f,wallPlane+1.f/scale-nearest);
    tiltBackFlipAboutCOM(lib,pose,slope);
    pose[0].t=pose[0].t+outward*shift;
}

inline Pose sampleBackFlipOut(const Library& lib,float phase,float slope,float gap,float scale) {
    auto pose=lib.sample(Motion::backFlipOut,std::clamp(phase,0.f,1.f));
    placeBackFlipOut(lib,pose,slope,gap,scale);
    return pose;
}

struct BackFlipSweepStats {unsigned casts{};};

inline bool backFlipBodyClear(World& world,const Library& lib,Vec from,Vec to,
        float fromPhase,float toPhase,Vec outward,float slope,float gap,float scale,
        BackFlipSweepStats* stats=nullptr) {
    if(lib.rest.size()<99||lib.parents.size()<99||lib.clip(Motion::backFlipOut).frames.size()<2||
        !from.finite()||!to.finite()||!outward.finite()||!std::isfinite(fromPhase)||!std::isfinite(toPhase)||
        !std::isfinite(slope)||!std::isfinite(gap)||!std::isfinite(scale)||scale<.5f||scale>2.f||gap<=0||
        fromPhase<0||toPhase>1||toPhase<fromPhase||toPhase-fromPhase>1.f/backFlipExitSegments+.001f)return false;
    outward=Vec{outward.x,outward.y,0}.unit();if(outward.length()<.9f)return false;
    const Vec side{-outward.y,outward.x,0},up{0,0,1};
    const float middlePhase=(fromPhase+toPhase)*.5f;

    const Vec middle=(from+to)*.5f+backFlipExitPoint({},outward,middlePhase)-
        (backFlipExitPoint({},outward,fromPhase)+backFlipExitPoint({},outward,toPhase))*.5f;
    std::array<BackFlipBody,3> bodies;
    const std::array<float,3> phases={fromPhase,middlePhase,toPhase};
    const std::array<Vec,3> origins={from,middle,to};
    for(int sample=0;sample<3;++sample) {
        bodies[sample]=backFlipBody(lib,sampleBackFlipOut(lib,phases[sample],slope,gap,scale));
        for(auto& marker:bodies[sample]) {
            const auto p=marker.point;
            marker.point=origins[sample]+(side*p.x-outward*p.y+up*p.z)*scale;
            marker.radius*=scale;
            if(!marker.point.finite()||!std::isfinite(marker.radius))return false;
        }
    }
    auto clear=[&](Vec a,Vec b) {
        if((b-a).length()<.001f)return true;
        if(stats)++stats->casts;
        return !world.ray(a,b).has_value();
    };

    for(int half=0;half<2;++half)for(std::size_t i=0;i<bodies[0].size();++i) {
        const auto& a=bodies[half][i];const auto& b=bodies[half+1][i];
        if(!clear(a.point,b.point))return false;
        if(i<2)continue;
        const Vec rail=(i==2||i==10||i==12?up:side)*std::max(a.radius,b.radius);
        if(!clear(a.point+rail,b.point+rail)||!clear(a.point-rail,b.point-rail))return false;
    }
    constexpr std::array<std::array<int,2>,12> links={{{0,1},{1,2},{1,3},{3,4},{4,5},
        {1,6},{6,7},{7,8},{0,9},{9,10},{0,11},{11,12}}};
    for(int sample:{1,2})for(const auto& link:links)
        if(!clear(bodies[sample][link[0]].point,bodies[sample][link[1]].point))return false;
    for(const auto& marker:bodies[2]) {
        const Vec a=marker.point-outward*marker.radius,b=marker.point+outward*marker.radius;
        if(!clear(a,b)||!clear(b,a))return false;
    }
    return true;
}
