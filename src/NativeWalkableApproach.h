#pragma once
#include "Core.h"

namespace fc {
inline bool groundEntryGeometryAllowed(bool onGround,bool wantsJump,bool nativeJump,bool airborne,bool confirmedAirborne) {
    return onGround&&!wantsJump&&!nativeJump&&!airborne&&!confirmedAirborne;
}
struct EntryGroundSupport {std::optional<Hit> floor;unsigned samples{};float height{};};
template<class Ray>
inline EntryGroundSupport entryGroundSupport(Ray&& ray,Vec feet,const Settings& cfg,float minimumNormalZ) {
    EntryGroundSupport result;
    auto valid=[&](const std::optional<Hit>& h) {
        return h&&h->climbable&&h->point.finite()&&h->normal.finite()&&h->normal.length()>.5f&&
            h->normal.unit().z>=minimumNormalZ&&h->normal.unit().z>0;
    };
    const auto middle=ray(feet+Vec{0,0,6},feet-Vec{0,0,18});
    if(valid(middle)&&feet.z-middle->point.z<=16&&middle->point.z-feet.z<=6){result.floor=middle;result.samples=1;result.height=middle->point.z;return result;}
    auto ringSupport=[&](float radius,unsigned count) {
        std::array<std::optional<Hit>,16> ring{};
        for(unsigned i=0;i<count;++i) {
            const float angle=i*(6.28318530718f/count);
            const Vec p=feet+Vec{std::cos(angle),std::sin(angle),0}*radius;
            auto h=ray(p+Vec{0,0,radius+6.f},p-Vec{0,0,radius+18.f});
            if(valid(h))ring[i]=h;
        }
        for(unsigned i=0;i<count;++i) {
            if(!ring[i])continue;
            const auto n=ring[i]->normal.unit();unsigned matches=0;
            auto consistent=[&](const std::optional<Hit>& h) {
                return h&&n.dot(h->normal.unit())>.98f&&std::abs((h->point-ring[i]->point).dot(n))<=1.f;
            };
            for(unsigned j=0;j<count;++j)if(consistent(ring[j]))++matches;
            if(matches<3||!consistent(ring[(i+1)%count])||!consistent(ring[(i+count-1)%count]))continue;
            const float height=ring[i]->point.z-((feet.x-ring[i]->point.x)*n.x+(feet.y-ring[i]->point.y)*n.y)/n.z;
            if(feet.z-height>16||height-feet.z>6)continue;
            result.floor=ring[i];result.samples=matches;result.height=height;return true;
        }
        return false;
    };
    if(ringSupport(cfg.radius*.6f,8))return result;
    ringSupport(cfg.radius*.9f,16);
    return result;
}
struct NativeWalkableResult {bool walkable{};unsigned casts{};};

inline NativeWalkableResult nativeWalkableApproach(World& world,Vec feet,Vec facing,
    const Settings& cfg,bool grounded,float maxStep=16.f,float minimumNormalZ=.707107f) {
    NativeWalkableResult result;
    facing.z=0;facing=facing.unit();
    if(!grounded||!feet.finite()||!facing.finite()||facing.length()<.99f||
        !std::isfinite(maxStep)||maxStep<=0||maxStep>16||
        !std::isfinite(minimumNormalZ)||minimumNormalZ<.7f||minimumNormalZ>1||
        !std::isfinite(cfg.reach)||cfg.reach<=0||
        !std::isfinite(cfg.radius)||cfg.radius<10||cfg.radius>45||
        !std::isfinite(cfg.height)||cfg.height<2*cfg.radius||cfg.height>200)return result;
    constexpr unsigned budget=3072;
    bool exhausted=false;
    auto ray=[&](Vec from,Vec to)->std::optional<Hit> {
        if(result.casts>=budget){exhausted=true;return Hit{from,{0,0,0},false};}
        ++result.casts;return world.ray(from,to);
    };
    auto floor=[&](Vec location,float reference,float up,float down)->std::optional<float> {
        const auto hit=ray({location.x,location.y,reference+up},{location.x,location.y,reference-down});
        if(!hit||!hit->climbable||!hit->point.finite()||!hit->normal.finite()||
            hit->normal.length()<.5f||hit->normal.unit().z<minimumNormalZ)return {};
        return hit->point.z;
    };
    const auto support=entryGroundSupport(ray,feet,cfg,minimumNormalZ);
    if(!support.floor)return result;
    const float initialFloor=support.height;
    const Vec side{-facing.y,facing.x,0};
    const float distance=std::clamp(cfg.reach+cfg.radius,80.f,150.f);
    const int steps=static_cast<int>(std::ceil(distance/8.f));
    std::array<Vec,21> route{};
    float previousFloor=initialFloor;
    for(int step=0;step<=steps;++step) {
        const auto p=feet+facing*(distance*float(step)/float(steps));
        const auto middle=floor(p,previousFloor,maxStep+.25f,maxStep+.25f);
        if(!middle||std::abs(*middle-previousFloor)>maxStep+.02f)return result;

        for(float lateral:{-.65f,.65f}) {
            const auto outer=floor(p+side*(cfg.radius*lateral),*middle,maxStep+.25f,maxStep+.25f);
            if(!outer||std::abs(*outer-*middle)>maxStep+.02f)return result;
        }

        route[static_cast<std::size_t>(step)]={p.x,p.y,*middle+maxStep};
        previousFloor=*middle;
    }
    auto bodyClear=[&](Vec from,Vec to) {
        for(float z:{.25f,cfg.radius,cfg.height*.5f,cfg.height-cfg.radius,cfg.height-.25f}) {
            const float cap=std::min(z,cfg.height-z);
            const float radius=cap>=cfg.radius?cfg.radius:std::sqrt(std::max(0.f,2*cfg.radius*cap-cap*cap));
            const auto a=from+Vec{0,0,z},b=to+Vec{0,0,z};
            if(ray(a,b))return false;
            for(int index=0;index<8;++index) {
                const float angle=float(index)*.7853981634f;
                const Vec offset=(facing*std::cos(angle)+side*std::sin(angle))*radius;
                if(ray(a+offset,b+offset)||ray(b,b+offset))return false;
            }
        }

        for(int index=0;index<8;++index) {
            const float angle=float(index)*.7853981634f;
            const Vec offset=(facing*std::cos(angle)+side*std::sin(angle))*cfg.radius;
            if(ray(to+offset+Vec{0,0,cfg.radius},to+offset+Vec{0,0,cfg.height-cfg.radius}))return false;
        }
        return !exhausted;
    };
    if(!bodyClear(feet,route[0]))return result;
    for(int step=1;step<=steps;++step)
        if(!bodyClear(route[static_cast<std::size_t>(step-1)],route[static_cast<std::size_t>(step)]))return result;
    result.walkable=!exhausted;return result;
}

enum class GroundEntryExclusion { none, lowObstacle, staircase };
inline const char* name(GroundEntryExclusion reason) {
    switch(reason) {
    case GroundEntryExclusion::lowObstacle:return "verified low obstacle";
    case GroundEntryExclusion::staircase:return "verified staircase";
    default:return "none";
    }
}
struct GroundEntryExclusionResult {
    GroundEntryExclusion reason=GroundEntryExclusion::none;
    unsigned casts{};
    bool groundFound{};float baseHeight{};unsigned supportSamples{};float selectedRise{};
    bool excludes() const {return reason!=GroundEntryExclusion::none;}
};

inline GroundEntryExclusionResult groundedLowTopFallback(const Settings& cfg,bool grounded,std::optional<float> selectedTopRise) {
    GroundEntryExclusionResult result;
    if(!grounded||!selectedTopRise||!std::isfinite(*selectedTopRise)||
        !std::isfinite(cfg.height)||!std::isfinite(cfg.radius)||cfg.radius<10||cfg.radius>45||
        cfg.height<2*cfg.radius||cfg.height>200)return result;
    const float limit=std::min(48.f,.35f*cfg.height);
    if(*selectedTopRise>limit+.02f)return result;
    result.selectedRise=*selectedTopRise;result.reason=GroundEntryExclusion::lowObstacle;return result;
}

inline GroundEntryExclusionResult groundedLowFace(World& world,Vec feet,const Settings& cfg,
    bool grounded,Vec candidateFeet,Vec candidateSurfaceNormal,float minimumNormalZ=.707107f) {
    GroundEntryExclusionResult result;
    if(!grounded||!feet.finite()||!candidateFeet.finite()||!candidateSurfaceNormal.finite()||
        !std::isfinite(cfg.radius)||cfg.radius<10||cfg.radius>45||!std::isfinite(cfg.height)||
        cfg.height<2*cfg.radius||cfg.height>200||!std::isfinite(cfg.gap)||cfg.gap<cfg.radius||cfg.gap>70||
        !std::isfinite(cfg.chest)||!std::isfinite(cfg.grip)||cfg.chest<=0||cfg.grip<=0||
        !std::isfinite(minimumNormalZ)||minimumNormalZ<.7f||minimumNormalZ>1)return result;
    const Vec surface=candidateSurfaceNormal.unit();Vec normal=surface;normal.z=0;normal=normal.unit();
    if(normal.length()<.99f||std::abs(surface.z)>.25f)return result;
    auto ray=[&](Vec a,Vec b){++result.casts;return world.ray(a,b);};
    const auto support=entryGroundSupport(ray,feet,cfg,minimumNormalZ);
    if(!support.floor)return result;
    result.groundFound=true;result.baseHeight=support.height;result.supportSamples=support.samples;
    const float base=result.baseHeight,limit=std::min(48.f,.35f*cfg.height);
    const Vec face=candidateFeet-normal*cfg.gap+Vec{0,0,surface.z>=0?6.f:cfg.height};
    if((Vec{face.x,face.y,feet.z}-feet).length()>180)return result;
    const float horizontalLength=std::sqrt(std::max(0.f,1-surface.z*surface.z));
    auto faceAt=[&](float height) {
        const float dz=height-face.z;
        return face-normal*(surface.z*dz/horizontalLength)+Vec{0,0,dz};
    };
    const auto top=ray(faceAt(base+limit+.5f)-normal*.25f,faceAt(base+2.f)-normal*.25f);
    if(!top||!top->climbable||!top->point.finite()||!top->normal.finite()||top->normal.length()<.5f||
        top->normal.unit().z<minimumNormalZ||top->point.z-base<=2||top->point.z-base>limit+.02f)return result;
    for(float height:{base+1.f,base+(top->point.z-base)*.5f}) {
        const auto level=faceAt(height);
        const auto h=ray(level+normal*12.f,level-normal*.25f);
        if(!h||!h->climbable||!h->point.finite()||!h->normal.finite()||h->normal.length()<.5f||
            h->normal.unit().dot(surface)<.9f||std::abs((h->point-level).dot(normal))>2)return result;
    }
    for(float height:{top->point.z+4.f,base+cfg.chest,base+cfg.grip}) {
        const auto level=faceAt(height);
        if(ray(level+normal*12.f,level-normal*.25f))return result;
    }
    result.selectedRise=top->point.z-base;result.reason=GroundEntryExclusion::lowObstacle;return result;
}

inline GroundEntryExclusionResult groundedEntryExclusion(World& world,Vec feet,Vec facing,
    const Settings& cfg,bool grounded,Vec candidateFeet,Vec candidateSurfaceNormal,
    float minimumNormalZ=.707107f) {
    GroundEntryExclusionResult result;
    if(!grounded||!feet.finite()||!facing.finite()||!candidateFeet.finite()||!candidateSurfaceNormal.finite()||
        !std::isfinite(cfg.radius)||cfg.radius<10||cfg.radius>45||!std::isfinite(cfg.height)||
        cfg.height<2*cfg.radius||cfg.height>200||!std::isfinite(cfg.gap)||cfg.gap<cfg.radius||cfg.gap>70||
        !std::isfinite(cfg.reach)||cfg.reach<=0||!std::isfinite(minimumNormalZ)||minimumNormalZ<.70f||minimumNormalZ>1)return result;
    facing.z=0;facing=facing.unit();Vec normal=candidateSurfaceNormal;normal.z=0;normal=normal.unit();
    if(facing.length()<.99f||normal.length()<.99f||normal.dot(facing)>-.35f)return result;
    const Vec side{-facing.y,facing.x,0};
    const float lowHeight=std::min(48.f,.35f*cfg.height),stairHeight=24.f;
    const float footprint=std::max(12.f,cfg.radius*.55f);
    const Vec face=candidateFeet-normal*cfg.gap;
    const float faceDistance=(face-feet).dot(normal)/facing.dot(normal);

    const float distance=faceDistance+footprint+2;
    if(!std::isfinite(distance)||faceDistance<2||distance>180)return result;

    const Vec forwardFace=feet+facing*faceDistance;
    if(std::abs((face-forwardFace).dot(side))>footprint)return result;
    constexpr unsigned budget=3072;bool exhausted=false;
    auto ray=[&](Vec a,Vec b)->std::optional<Hit> {
        if(result.casts>=budget){exhausted=true;return Hit{a,{},false};}
        ++result.casts;return world.ray(a,b);
    };
    auto floor=[&](Vec p,float reference,float up,float down)->std::optional<Hit> {
        auto h=ray({p.x,p.y,reference+up},{p.x,p.y,reference-down});
        if(!h||!h->climbable||!h->point.finite()||!h->normal.finite()||h->normal.unit().z<minimumNormalZ)return {};
        return h;
    };
    const auto support=entryGroundSupport(ray,feet,cfg,minimumNormalZ);
    if(!support.floor)return result;
    auto first=support.floor;
    result.groundFound=true;result.baseHeight=support.height;result.supportSamples=support.samples;
    const float base=support.height;
    const int steps=static_cast<int>(std::ceil(distance/4.f));
    std::array<Vec,47> route{};
    float previous=base,maxRise=0,maxGround=base;Vec previousPoint=feet;
    unsigned risers=0;bool selectedRiser=false,allStairSteps=true;
    for(int i=0;i<=steps;++i) {
        const Vec p=feet+facing*(distance*float(i)/float(steps));
        const auto mid=floor(p,previous,lowHeight+.25f,16.25f);
        if(!mid)return result;
        const float height=mid->point.z,rise=height-previous;
        if(rise>lowHeight+.02f||rise< -16.02f)return result;

        for(float lateral:{-1.f,1.f}) {
            auto foot=floor(p+side*(footprint*lateral),height,3,3);
            if(!foot||std::abs(foot->point.z-height)>2)return result;
        }
        if(rise>2) {
            if(mid->normal.unit().z<.96f||first->normal.unit().z<.96f)return result;

            std::optional<Hit> lower;
            for(float level:{previous+std::min(2.f,rise*.2f),previous+rise*.5f}) {
                const auto h=ray({previousPoint.x,previousPoint.y,level},{p.x,p.y,level});
                if(!h||!h->climbable||!h->point.finite()||!h->normal.finite()||
                    std::abs(h->normal.unit().z)>.25f||h->normal.unit().dot(facing)>-.5f)return result;
                if(!lower)lower=h;
            }
            ++risers;allStairSteps&=rise<=stairHeight+.02f;
            maxRise=std::max(maxRise,rise);
            selectedRiser|=lower->normal.unit().dot(normal)>.95f&&std::abs((lower->point-face).dot(normal))<1.f;
        }
        route[static_cast<std::size_t>(i)]={p.x,p.y,height};
        maxGround=std::max(maxGround,height);previous=height;previousPoint=p;first=mid;
    }
    if(!risers||!selectedRiser)return result;
    GroundEntryExclusion kind=GroundEntryExclusion::none;
    if(risers>=2&&allStairSteps)kind=GroundEntryExclusion::staircase;
    else if(maxGround-base<=lowHeight+.02f)kind=GroundEntryExclusion::lowObstacle;
    if(kind==GroundEntryExclusion::none)return result;

    const auto final=route[static_cast<std::size_t>(steps)];
    for(float sign:{-1.f,1.f}) {
        const auto h=floor(final+facing*(footprint*sign),final.z,3,3);
        if(!h||std::abs(h->point.z-final.z)>2)return result;
    }

    const float lift=std::max(16.f,maxRise);
    auto bodyClear=[&](Vec from,Vec to) {
        for(float z:{.25f,cfg.radius,cfg.height*.5f,cfg.height-cfg.radius,cfg.height-.25f}) {
            const float cap=std::min(z,cfg.height-z);
            const float radius=cap>=cfg.radius?cfg.radius:std::sqrt(std::max(0.f,2*cfg.radius*cap-cap*cap));
            const Vec a=from+Vec{0,0,z},b=to+Vec{0,0,z};
            if(ray(a,b))return false;
            for(int ring=0;ring<8;++ring) {
                const float angle=ring*.7853981634f;
                const Vec offset=(facing*std::cos(angle)+side*std::sin(angle))*radius;
                if(ray(a+offset,b+offset)||ray(b,b+offset))return false;
            }
        }
        for(int ring=0;ring<8;++ring) {
            const float angle=ring*.7853981634f;
            const Vec offset=(facing*std::cos(angle)+side*std::sin(angle))*cfg.radius;
            if(ray(to+offset+Vec{0,0,cfg.radius},to+offset+Vec{0,0,cfg.height-cfg.radius}))return false;
        }
        return !exhausted;
    };
    Vec previousBody=feet;

    for(int i=0;i<=steps;++i) {
        if(i>0&&i<steps&&i%2&&std::abs(route[i].z-route[i-1].z)<.01f&&std::abs(route[i+1].z-route[i].z)<.01f)continue;
        const auto next=route[static_cast<std::size_t>(i)]+Vec{0,0,lift};
        if(!bodyClear(previousBody,next))return result;
        previousBody=next;
    }
    if(!exhausted)result.reason=kind;
    return result;
}
inline GroundEntryExclusionResult groundedStepFace(World& world,Vec feet,Vec facing,const Settings& cfg,
    bool grounded,Vec candidateFeet,Vec candidateSurfaceNormal,float minimumNormalZ=.707107f) {
    GroundEntryExclusionResult result;
    if(!grounded||!feet.finite()||!facing.finite()||!candidateFeet.finite()||!candidateSurfaceNormal.finite()||
        !std::isfinite(cfg.radius)||cfg.radius<10||cfg.radius>45||!std::isfinite(cfg.height)||
        cfg.height<2*cfg.radius||cfg.height>200||!std::isfinite(cfg.gap)||cfg.gap<cfg.radius||cfg.gap>70||
        !std::isfinite(cfg.chest)||!std::isfinite(cfg.grip)||cfg.chest<=0||cfg.grip<=0||
        !std::isfinite(minimumNormalZ)||minimumNormalZ<.7f||minimumNormalZ>1)return result;
    facing.z=0;facing=facing.unit();const auto surface=candidateSurfaceNormal.unit();
    Vec normal=surface;normal.z=0;const float horizontalLength=normal.length();normal=normal.unit();
    if(facing.length()<.99f||normal.length()<.99f||surface.z<-.15f||surface.z>=minimumNormalZ||normal.dot(facing)>-.35f)return result;
    const Vec face=candidateFeet-normal*cfg.gap+Vec{0,0,surface.z>=0?6.f:cfg.height};
    Vec target=face;target.z=feet.z;const auto direction=(target-feet).unit();
    if(direction.length()<.99f||direction.dot(facing)<.7f)return result;
    const float distance=(target-feet).length()+2.f+24.f*std::max(0.f,surface.z)/horizontalLength;
    if(distance<2||distance>180)return result;
    constexpr unsigned budget=768;bool exhausted=false;
    auto ray=[&](Vec a,Vec b)->std::optional<Hit> {
        if(result.casts>=budget){exhausted=true;return Hit{a,{},false};}
        ++result.casts;return world.ray(a,b);
    };
    auto valid=[&](const std::optional<Hit>& hit) {
        return hit&&hit->climbable&&hit->point.finite()&&hit->normal.finite()&&hit->normal.length()>.5f;
    };
    auto selected=[&](const Hit& h) {
        return h.normal.unit().dot(surface)>.92f&&std::abs((h.point-face).dot(surface))<1.f;
    };
    const auto support=entryGroundSupport(ray,feet,cfg,minimumNormalZ);
    if(!support.floor)return result;
    result.groundFound=true;result.baseHeight=support.height;result.supportSamples=support.samples;
    float searchDistance=distance;
    float previous=support.height,base=support.height;Vec previousPoint=feet,start=feet;
    bool onRiser=false,matched=false;unsigned risers=0;
    for(unsigned i=1;i<=181;++i) {
        const float travel=std::min(float(i)-.5f,searchDistance);
        const auto p=feet+direction*travel;
        const auto h=ray({p.x,p.y,previous+24.25f},{p.x,p.y,previous-16.25f});
        if(!valid(h))return result;
        const auto n=h->normal.unit();const float rise=h->point.z-previous;
        if(rise>24.02f||rise< -2.f)return result;
        if(n.z<minimumNormalZ) {
            if(n.z<-.15f||n.dot(direction)>-.35f)return result;
            if(!onRiser){onRiser=true;base=previous;start=previousPoint;matched=false;}
            if(h->point.z-base>24.02f)return result;
            matched|=selected(*h);
            if(matched) {
                const float remaining=(24.02f-(h->point.z-base))*std::max(0.f,n.z)/std::max(.35f,-n.dot(direction));
                searchDistance=std::min(180.f,std::max(searchDistance,travel+remaining+2.f));
            }
        } else if(onRiser||rise>2) {
            if(!onRiser){base=previous;start=previousPoint;matched=false;}
            const float totalRise=h->point.z-base;
            if(totalRise<=2||totalRise>24.02f)return result;
            for(float z:{base+std::min(1.f,totalRise*.2f),base+totalRise*.5f}) {
                const auto r=ray({start.x,start.y,z},{p.x,p.y,z});
                if(!valid(r)||r->normal.unit().z<-.15f||r->normal.unit().z>=minimumNormalZ||r->normal.unit().dot(direction)>-.35f)return result;
                matched|=selected(*r);
            }
            ++risers;onRiser=false;
            if(matched) {
                for(float forward:{.25f,.75f}) {
                    const auto q=p+direction*forward;
                    const auto tread=ray({q.x,q.y,h->point.z+4.f},{q.x,q.y,h->point.z-4.f});
                    if(!valid(tread)||tread->normal.unit().z<minimumNormalZ||
                        std::abs((tread->point-h->point).dot(n))>.5f)return result;
                }
                for(float height:{h->point.z+2.f,std::max(h->point.z+2.f,candidateFeet.z+cfg.chest),
                    std::max(h->point.z+2.f,candidateFeet.z+cfg.grip)}) {
                    const float dz=height-face.z;
                    const auto level=face-normal*(surface.z*dz/horizontalLength)+Vec{0,0,dz};
                    if(ray(level+normal*4.f,level-normal*.25f))return result;
                }
                if(!exhausted){result.selectedRise=totalRise;result.reason=risers>=2?GroundEntryExclusion::staircase:GroundEntryExclusion::lowObstacle;}
                return result;
            }
        }
        previous=h->point.z;previousPoint=p;
        if(travel>=searchDistance)break;
    }
    return result;
}

}
