#pragma once

struct CornerRoute {
    static constexpr unsigned capacity=24;
    std::array<Vec,capacity> points{},normals{};
    Vec join{},sourceNormal{},targetNormal{};
    unsigned count{};
    float distance{},length{},sideSign{};
    bool convex{};
};
struct CornerStep {Vec position{},normal{};bool complete{},reversed{};};
inline Vec cornerHorizontal(Vec value){value.z=0;return value.unit();}
inline Vec cornerRotate(Vec value,float angle) {
    return {value.x*std::cos(angle)-value.y*std::sin(angle),value.x*std::sin(angle)+value.y*std::cos(angle),value.z};
}
inline bool cornerFace(const std::optional<Hit>& hit,Vec normal) {
    return hit&&hit->climbable&&hit->point.finite()&&hit->normal.finite()&&hit->normal.length()>.5f&&
        hit->normal.unit().z>=-.45f&&hit->normal.unit().z<=.70f&&hit->normal.unit().dot(normal)>.985f;
}

inline Vec cornerSeamRise(const CornerRoute& route,float height) {
    const Vec a=route.sourceNormal,b=route.targetNormal;
    const float determinant=a.x*b.y-a.y*b.x;
    if(std::abs(determinant)<.05f)return {};
    return {(-a.z*b.y+a.y*b.z)*height/determinant,
            (-a.x*b.z+a.z*b.x)*height/determinant,height};
}
inline Vec cornerAnchor(Vec feet,Vec normal,float height) {
    const Vec tangent=(Vec{0,0,1}-normal*normal.z).unit();
    return feet+Vec{0,0,6}+tangent*(height-6);
}
inline bool cornerJoinValid(World& world,const Settings& cfg,const CornerRoute& route) {
    const Vec sourceTravel=Vec{-route.sourceNormal.y,route.sourceNormal.x,0}.unit()*route.sideSign;
    const Vec targetTravel=Vec{-route.targetNormal.y,route.targetNormal.x,0}.unit()*route.sideSign;
    for(bool destination:{false,true}) {
        const Vec normal=destination?route.targetNormal:route.sourceNormal;
        const Vec away=destination?targetTravel:sourceTravel*-1;
        for(float height:{cfg.chest,cfg.grip}) {
            auto patch=[&](float side) {
                const Vec point=route.join+cornerSeamRise(route,height)+away*side;
                const auto contact=world.ray(point+normal*12.f,point-normal*3.f);
                return cornerFace(contact,normal)&&(contact->point-point).length()<=.35f;
            };

            if(!patch(.75f))return false;
            if(!patch(8.f)&&(!patch(1.25f)||!patch(2.f)))return false;
        }
    }
    return true;
}
inline bool cornerGrip(World& world,const Settings& cfg,const CornerRoute& route,Vec feet,Vec facing) {
    for(Vec normal:{facing,route.sourceNormal,route.targetNormal}) {
        const Vec side=Vec{-normal.y,normal.x,0}.unit();

        for(float heightCenter:{cfg.grip,cfg.chest}) {

          if(heightCenter==cfg.chest&&std::abs(route.sourceNormal.z)<.12f&&std::abs(route.targetNormal.z)<.12f)continue;
          for(float offset:{0.f,-8.f,8.f,-18.f,18.f}) {
            std::optional<Hit> previous;
            bool pair=true;
            for(float height:{heightCenter-5.f,heightCenter+5.f}) {
                const Vec point=cornerAnchor(feet,normal,height)+side*offset;
                const auto hit=world.ray(point+normal*12.f,point-normal*(cfg.gap+20.f));
                if(!hit||!hit->climbable||!hit->point.finite()||!hit->normal.finite()||hit->normal.length()<.5f||
                    (hit->normal.unit().z<-.45f||hit->normal.unit().z>cfg.maxNormalZ)){pair=false;break;}
                const Vec measured=hit->normal.unit();
                const bool source=measured.dot(route.sourceNormal)>.985f&&std::abs((hit->point-route.join).dot(route.sourceNormal))<.5f;
                const bool target=measured.dot(route.targetNormal)>.985f&&std::abs((hit->point-route.join).dot(route.targetNormal))<.5f;
                if((!source&&!target)||(previous&&measured.dot(previous->normal.unit())<.985f)){pair=false;break;}
                previous=hit;
            }
            if(pair)return true;
          }
        }
    }
    return false;
}

inline bool cornerLandingGrip(World& world,const Settings& cfg,const CornerRoute& route,Vec feet) {
    const Vec normal=route.targetNormal;
    for(float center:{cfg.grip,cfg.chest}) {
        if(center==cfg.chest&&std::abs(route.sourceNormal.z)<.12f&&std::abs(route.targetNormal.z)<.12f)continue;
        bool pair=true;
        for(float height:{center-5.f,center+5.f}) {
            const Vec point=cornerAnchor(feet,normal,height);
            const auto hit=world.ray(point+normal*12.f,point-normal*(cfg.gap+20.f));
            if(!cornerFace(hit,normal)||std::abs((hit->point-route.join).dot(normal))>=.5f){pair=false;break;}
        }
        if(pair)return true;
    }
    return false;
}

inline bool cornerBodyClear(World& world,const Settings& cfg,Vec from,Vec to,bool inclined=false) {
    const float radius=cfg.radius+std::max(4.f,cfg.radius*.085f);
    if(inclined) {

        const std::array<float,7> heights{6.f,(6.f+cfg.radius)*.5f,cfg.radius,cfg.chest,
            cfg.height-cfg.radius,cfg.height-(6.f+cfg.radius)*.5f,cfg.height-6.f};
        for(unsigned index=0;index<8;++index) {
            Vec previous{};bool first=true;
            for(float height:heights) {
                const float axial=std::max({cfg.radius-height,0.f,height-(cfg.height-cfg.radius)});
                const float width=std::sqrt(std::max(0.f,radius*radius-axial*axial));
                const Vec offset=cornerRotate({width,0,height},float(index)*.785398163397f);
                if((to-from).length()>.001f&&world.ray(from+offset,to+offset))return false;
                if(!first&&world.ray(to+previous,to+offset))return false;
                previous=offset;first=false;
            }
        }
        return true;
    }
    for(unsigned index=0;index<8;++index) {
        const Vec offset=cornerRotate({radius,0,0},float(index)*.785398163397f);
        if((to-from).length()>.001f)for(float height:{6.f,cfg.chest,cfg.height})
            if(world.ray(from+offset+Vec{0,0,height},to+offset+Vec{0,0,height}))return false;
        if(world.ray(to+offset+Vec{0,0,6},to+offset+Vec{0,0,cfg.height}))return false;
    }
    return true;
}
inline CornerStep cornerSample(const CornerRoute& route,float distance) {
    if(route.count<2||route.count>CornerRoute::capacity)return {};
    float remaining=std::clamp(distance,0.f,route.length);
    for(unsigned index=1;index<route.count;++index) {
        const float segment=(route.points[index]-route.points[index-1]).length();
        if(remaining<=segment||index+1==route.count) {
            const float phase=segment>.0001f?std::clamp(remaining/segment,0.f,1.f):1.f;
            return {route.points[index-1]+(route.points[index]-route.points[index-1])*phase,
                (route.normals[index-1]*(1-phase)+route.normals[index]*phase).unit(),
                distance>=route.length-.001f,distance<=.001f};
        }
        remaining-=segment;
    }
    return {};
}
inline float cornerSpeedLimit(const CornerRoute& route,float requestedSpeed) {
    if(route.count<2||route.count>CornerRoute::capacity)return 0;
    float curvature=0;
    for(unsigned index=1;index<route.count;++index) {
        const float distance=(route.points[index]-route.points[index-1]).length();
        const float angle=std::acos(std::clamp(route.normals[index-1].dot(route.normals[index]),-1.f,1.f));
        if(distance>.01f)curvature=std::max(curvature,angle/distance);
    }

    return std::min(std::max(0.f,requestedSpeed),curvature>.0001f?8.f/curvature:requestedSpeed);
}
template<class ClearPath>
inline std::optional<CornerStep> advanceCornerRoute(World& world,const Settings& cfg,CornerRoute& route,
    float travel,ClearPath&& clearPath) {
    if(route.count<2||route.count>CornerRoute::capacity||!std::isfinite(route.distance)||!std::isfinite(travel)||
        !cornerJoinValid(world,cfg,route))return {};
    const float target=std::clamp(route.distance+travel,0.f,route.length);
    auto previous=cornerSample(route,route.distance);
    const unsigned steps=std::max(1u,unsigned(std::ceil(std::abs(target-route.distance)/3.f)));
    for(unsigned index=1;index<=steps;++index) {
        const auto next=cornerSample(route,route.distance+(target-route.distance)*(float(index)/float(steps)));
        if(!cornerGrip(world,cfg,route,next.position,next.normal)||!clearPath(previous.position,next.position)||
            !cornerBodyClear(world,cfg,previous.position,next.position,std::abs(route.sourceNormal.z)>.12f||std::abs(route.targetNormal.z)>.12f))return {};
        previous=next;
    }
    if(target>=route.length-.001f&&!cornerLandingGrip(world,cfg,route,previous.position))return {};
    route.distance=target;return previous;
}
template<class ClearPath>
inline std::optional<CornerStep> shiftCornerRoute(World& world,const Settings& cfg,CornerRoute& route,
    float rise,ClearPath&& clearPath) {
    if(route.count<2||route.count>CornerRoute::capacity||!std::isfinite(rise))return {};
    CornerRoute shifted=route;
    const Vec displacement=cornerSeamRise(route,rise);
    if(!displacement.finite()||displacement.length()>std::abs(rise)*2.25f)return {};
    for(unsigned index=0;index<shifted.count;++index)shifted.points[index]=shifted.points[index]+displacement;
    shifted.join=shifted.join+displacement;
    const auto before=cornerSample(route,route.distance),after=cornerSample(shifted,shifted.distance);

    if(!cornerJoinValid(world,cfg,shifted)||!cornerGrip(world,cfg,shifted,after.position,after.normal)||
        !clearPath(before.position,after.position)||!cornerBodyClear(world,cfg,before.position,after.position,std::abs(route.sourceNormal.z)>.12f||std::abs(route.targetNormal.z)>.12f))return {};
    route=shifted;return after;
}
template<class ClearPath>
inline std::optional<CornerRoute> findCornerRoute(World& world,const Settings& cfg,Vec feet,Vec outward,
    float sideSign,ClearPath&& clearPath) {
    if(!feet.finite()||!outward.finite()||(outward.unit().z<-.45f||outward.unit().z>cfg.maxNormalZ)||std::abs(sideSign)<.5f)return {};
    Vec sourceNormal=outward.unit();
    outward=cornerHorizontal(sourceNormal);sideSign=sideSign<0?-1.f:1.f;
    Vec travel=Vec{-outward.y,outward.x,0}*sideSign;
    const float ahead=std::min(cfg.reach+cfg.gap,cfg.gap+64.f);
    Vec upper=cornerAnchor(feet,sourceNormal,cfg.grip);
    auto front=[&](float offset) {const Vec point=upper+travel*offset;return world.ray(point+sourceNormal*12.f,point-sourceNormal*(cfg.gap+20.f));};

    auto destination=world.ray(upper,upper+travel*ahead);
    bool convex=false;
    auto nextFace=front(ahead);
    if(!destination&&cornerFace(nextFace,sourceNormal))return {};
    auto source=front(0);float sourceOffset=0;
    auto sourceCandidate=[&](const std::optional<Hit>& hit) {
        return hit&&hit->climbable&&hit->point.finite()&&hit->normal.finite()&&hit->normal.length()>.5f&&
            hit->normal.unit().z>=-.45f&&hit->normal.unit().z<=cfg.maxNormalZ&&hit->normal.unit().dot(sourceNormal)>.70f;
    };
    if(!sourceCandidate(source))for(float back:{-8.f,-16.f,-24.f}) {
        source=front(back);sourceOffset=back;if(sourceCandidate(source))break;
    }
    if(!sourceCandidate(source))for(float back:{0.f,-8.f,-16.f,-24.f}) {

        const Vec point=feet+Vec{0,0,cfg.grip}+travel*back;
        source=world.ray(point+outward*12.f,point-outward*(cfg.gap+20.f));sourceOffset=back;
        if(sourceCandidate(source))break;
    }
    if(!sourceCandidate(source))return {};

    const Vec sampledSource=source->normal.unit();
    if((sampledSource-sourceNormal).length()>.0001f) {
        sourceNormal=sampledSource;outward=cornerHorizontal(sourceNormal);
        travel=Vec{-outward.y,outward.x,0}*sideSign;upper=cornerAnchor(feet,sourceNormal,cfg.grip);
        source=front(sourceOffset);if(!cornerFace(source,sourceNormal))return {};
        destination=world.ray(upper,upper+travel*ahead);nextFace=front(ahead);
        if(!destination&&cornerFace(nextFace,sourceNormal))return {};
    }
    auto crossFace=[&](const std::optional<Hit>& hit,bool exterior) {
        if(!hit||!hit->climbable||!hit->normal.finite()||!hit->point.finite()||hit->normal.length()<.5f||
            (hit->normal.unit().z<-.45f||hit->normal.unit().z>cfg.maxNormalZ))return false;
        const Vec n=cornerHorizontal(hit->normal);
        return std::abs(n.dot(outward))<.94f&&n.dot(travel)*(exterior?1.f:-1.f)>.33f;
    };
    if(!crossFace(destination,false)) {
        if(cornerFace(nextFace,sourceNormal))return {};
        float low=sourceOffset,high=ahead;
        for(unsigned step=0;step<10;++step) {
            const float middle=(low+high)*.5f;
            if(cornerFace(front(middle),sourceNormal))low=middle;else high=middle;
        }
        const Vec edgeProbe=upper+travel*((low+high)*.5f);
        const Vec edge=edgeProbe-sourceNormal*(edgeProbe-source->point).dot(sourceNormal);
        destination.reset();

        for(float inset:{.75f,2.f,6.f,12.f,24.f}) {
            const Vec start=edge+travel*32.f-outward*inset;
            const auto candidate=world.ray(start,start-travel*(cfg.gap+64.f));
            if(crossFace(candidate,true)){destination=candidate;break;}
        }
        if(!destination)return {};
        convex=true;
    }

    sourceNormal=source->normal.unit();
    const Vec targetNormal=destination->normal.unit();
    const float preliminaryDet=sourceNormal.x*targetNormal.y-sourceNormal.y*targetNormal.x;
    if(std::abs(preliminaryDet)<.15f)return {};
    const float sourceD=(source->point-feet).dot(sourceNormal),targetD=(destination->point-feet).dot(targetNormal);
    CornerRoute measured;measured.sourceNormal=sourceNormal;measured.targetNormal=targetNormal;
    measured.join=feet+Vec{(sourceD*targetNormal.y-sourceNormal.y*targetD)/preliminaryDet,
        (sourceNormal.x*targetD-sourceD*targetNormal.x)/preliminaryDet,0};
    const Vec sourceTravel=Vec{-sourceNormal.y,sourceNormal.x,0}.unit()*sideSign;
    std::optional<Hit> seamSource;
    for(float back:{8.f,4.f,2.f}) {
        const Vec patch=measured.join+cornerSeamRise(measured,cfg.grip)-sourceTravel*back;
        const auto hit=world.ray(patch+sourceNormal*12.f,patch-sourceNormal*3.f);
        if(cornerFace(hit,sourceNormal)){seamSource=hit;break;}
    }
    if(!seamSource)return {};
    source=seamSource;sourceNormal=source->normal.unit();
    outward=cornerHorizontal(sourceNormal);travel=Vec{-outward.y,outward.x,0}*sideSign;
    CornerRoute route;route.sourceNormal=sourceNormal;route.targetNormal=targetNormal;
    const Vec targetHorizontal=cornerHorizontal(route.targetNormal);
    route.convex=convex;route.sideSign=sideSign;

    const float determinant=sourceNormal.x*route.targetNormal.y-sourceNormal.y*route.targetNormal.x;
    if(std::abs(determinant)<.15f)return {};
    const float sourceDistance=(source->point-feet).dot(sourceNormal),targetDistance=(destination->point-feet).dot(route.targetNormal);
    route.join=feet+Vec{(sourceDistance*route.targetNormal.y-sourceNormal.y*targetDistance)/determinant,
        (sourceNormal.x*targetDistance-sourceDistance*route.targetNormal.x)/determinant,0};

    const float sourceHeight=sourceNormal.z>=0?6.f:cfg.height;
    const float targetHeight=route.targetNormal.z>=0?6.f:cfg.height;
    const float a=-sourceNormal.z*sourceHeight,b=-route.targetNormal.z*targetHeight;
    const Vec bodyJoin=route.join+Vec{(a*route.targetNormal.y-sourceNormal.y*b)/determinant,
        (sourceNormal.x*b-a*route.targetNormal.x)/determinant,0};
    if((route.join-feet).length()>cfg.reach+cfg.gap||!cornerJoinValid(world,cfg,route))return {};
    const Vec targetTravel=Vec{-route.targetNormal.y,route.targetNormal.x,0}.unit()*sideSign;
    const float angle=std::atan2(outward.cross(targetHorizontal).z,outward.dot(targetHorizontal));
    if(angle*sideSign*(convex?1.f:-1.f)<=0)return {};
    auto append=[&](Vec point,Vec normal) {
        if(route.count&&((point-route.points[route.count-1]).length()<.01f)) {route.normals[route.count-1]=normal;return true;}
        if(route.count>=CornerRoute::capacity)return false;
        if(route.count)route.length+=(point-route.points[route.count-1]).length();
        route.points[route.count]=point;route.normals[route.count]=normal;++route.count;return true;
    };
    append(feet,sourceNormal);
    Vec center{},radial{};float arcAngle=angle;bool lateInside=false;
    if(convex) {
        center=bodyJoin;
        const Vec fromJoin=feet-center;
        float radius=(feet-bodyJoin).dot(outward);
        if(radius<cfg.radius+4.25f||radius>cfg.gap+16)return {};
        if(fromJoin.dot(travel)>0) {
            radius=fromJoin.length();radial=fromJoin;
            const float already=std::atan2(outward.cross(fromJoin.unit()).z,outward.dot(fromJoin.unit()));
            if(already*sideSign<0||std::abs(already)>std::abs(angle)*.55f||radius>cfg.gap+16)return {};
            radius=std::min(radius,std::max(cfg.gap,cfg.radius+4.25f));
            radial=fromJoin.unit()*radius;append(center+radial,sourceNormal);
            arcAngle=angle-already;
        } else {
            radius=std::min(radius,std::max(cfg.gap,cfg.radius+4.25f));
            radial=outward*radius;append(center+radial,sourceNormal);
        }
    } else {

        const float maintainedGap=(feet-bodyJoin).dot(outward);
        if(maintainedGap<cfg.radius+4.25f||maintainedGap>cfg.gap+16)return {};
        const float radius=std::clamp(cfg.gap*.35f,8.f,16.f);
        center=bodyJoin+(outward+targetHorizontal)*((maintainedGap+radius)/(1+outward.dot(targetHorizontal)));
        radial=outward*-radius;
        const Vec entry=center+radial;
        if(std::abs((entry-feet).dot(outward))>2)return {};
        if((entry-feet).dot(travel)<-.5f) {

            const float targetGap=(feet-bodyJoin).dot(targetHorizontal);
            const float capAxial=route.targetNormal.z>=0?cfg.radius-6.f:cfg.radius;
            const float capsulePlaneDistance=targetGap*std::hypot(route.targetNormal.x,route.targetNormal.y)+
                std::abs(route.targetNormal.z)*capAxial;
            if(capsulePlaneDistance<cfg.radius+4.25f||targetGap>cfg.gap+16.f||
                (entry-feet).length()>cfg.gap+16.f)return {};
            lateInside=true;
        } else append(entry,sourceNormal);
    }
    float approachAdvance=0;
    if(convex&&(std::abs(route.sourceNormal.z)>.12f||std::abs(route.targetNormal.z)>.12f)) {
        const Vec nominalEnd=center+cornerRotate(radial,arcAngle);
        for(float distance:{18.f,32.f,48.f,64.f}) {
            if(cornerLandingGrip(world,cfg,route,nominalEnd+targetTravel*distance)) {
                approachAdvance=distance>18.f?distance:0.f;break;
            }
        }
    }
    constexpr unsigned arcSteps=12;
    for(unsigned index=1;index<=arcSteps;++index) {
        const float phase=float(index)/float(arcSteps);
        const float normalPhase=approachAdvance>0?phase*phase:phase;
        Vec n=cornerRotate(outward,angle*normalPhase);
        const float slope=sourceNormal.z/std::hypot(sourceNormal.x,sourceNormal.y)*(1-normalPhase)+
            route.targetNormal.z/std::hypot(route.targetNormal.x,route.targetNormal.y)*normalPhase;
        n.z=slope;
        const float advancePhase=std::clamp((phase-.5f)*2.f,0.f,1.f);
        const float advanceWeight=advancePhase*advancePhase*(3.f-2.f*advancePhase);
        append((lateInside?feet+targetTravel*(24.f*phase):center+cornerRotate(radial,arcAngle*phase))+
            targetTravel*(approachAdvance*advanceWeight),n.unit());
    }
    if(route.count<2||route.length>cfg.reach+cfg.gap+64.f)return {};
    if(!cornerGrip(world,cfg,route,feet,sourceNormal))return {};
    auto segmentValid=[&](Vec start,Vec finish,Vec beforeNormal,Vec afterNormal) {

        const unsigned gripSteps=std::max(1u,unsigned(std::ceil((finish-start).length()/3.f)));
        for(unsigned step=1;step<gripSteps;++step) {
            const float phase=float(step)/float(gripSteps);
            const Vec point=start+(finish-start)*phase;
            const Vec facing=(beforeNormal*(1-phase)+afterNormal*phase).unit();
            if(!cornerGrip(world,cfg,route,point,facing))return false;
        }
        const unsigned segments=std::max(1u,unsigned(std::ceil((finish-start).length()/8.f)));
        Vec previous=start;
        for(unsigned step=1;step<=segments;++step) {
            const float phase=float(step)/float(segments);
            const Vec point=start+(finish-start)*phase;
            const Vec facing=(beforeNormal*(1-phase)+afterNormal*phase).unit();
            if(!cornerGrip(world,cfg,route,point,facing)||!clearPath(previous,point)||!cornerBodyClear(world,cfg,previous,point,std::abs(route.sourceNormal.z)>.12f||std::abs(route.targetNormal.z)>.12f))return false;
            previous=point;
        }
        return true;
    };
    for(unsigned index=1;index<route.count;++index)
        if(!segmentValid(route.points[index-1],route.points[index],route.normals[index-1],route.normals[index]))return {};
    const Vec end=route.points[route.count-1];
    bool tail=false;
    for(float distance:{18.f,8.f,4.f,2.f}) {
        const Vec target=end+targetTravel*distance;
        if(route.length+distance<=cfg.reach+cfg.gap+64.f&&cornerLandingGrip(world,cfg,route,target)&&segmentValid(end,target,route.targetNormal,route.targetNormal)) {
            append(target,route.targetNormal);tail=true;break;
        }
    }
    if(!tail)return {};
    return route;
}
