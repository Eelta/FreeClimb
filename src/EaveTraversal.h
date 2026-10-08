bool eaveObstacleAhead(World& w,Input intent) const {
    if(intent.y>.1f||std::abs(intent.x)<.6f||intent.y<0)return false;
    const Vec heading=Vec{-normal.y,normal.x,0}*(intent.x<0?-1.f:1.f);
    for(float height:{6.f,cfg.chest,cfg.height}) {
        const Vec from=position+Vec{0,0,height};
        const auto hit=w.ray(from,from+heading*(cfg.radius+28));
        if(hit&&hit->point.finite()&&hit->normal.finite()&&hit->normal.unit().dot(heading)<-.1f)return true;
    }
    return false;
}
bool tryEaveTransfer(World& originalWorld,Input intent={0,1},bool wallRun=false,float speed=0,float stamina=1000) {
    const Motion selected=std::abs(intent.x)>.6f&&intent.y<.5f?(intent.x<0?Motion::hopLeft:Motion::hopRight):Motion::hopUp;
    const bool sourceRoute=authored(selected);
    struct BoundedWorld final:World {
        World& source;unsigned calls{},limit{};bool exhausted{};
        BoundedWorld(World& world,unsigned maximum):source(world),limit(maximum){}
        std::optional<Hit> ray(Vec a,Vec b)override {
            if(calls>=limit){exhausted=true;return Hit{a,(a-b).unit(),false};}
            ++calls;return source.ray(a,b);
        }
    } bounded(originalWorld,sourceRoute?12288:1800);
    World& w=bounded;
    if(!position.finite()||!surfaceNormal.finite()||std::abs(surfaceNormal.z)>=cfg.maxNormalZ||
        intent.y<0||std::hypot(intent.x,intent.y)<.1f||(wallRun&&!cfg.wallRunObstacleJumps))return false;
    const Vec right{-normal.y,normal.x,0};
    const Vec uphill=(Vec{0,0,1}-surfaceNormal*surfaceNormal.z).unit();
    const bool sideways=std::abs(intent.x)>.6f&&intent.y<.5f;
    const Vec heading=sideways?right*(intent.x<0?-1.f:1.f):uphill;
    if(heading.length()<.9f||clearPath(w,position,position+heading*28)||!blockedHit)return false;
    const Hit obstruction=*blockedHit;
    if(!obstruction.point.finite()||!obstruction.normal.finite()||
        obstruction.normal.unit().dot(heading)>-.1f)return false;
    const auto source=support(w,position,normal*-1);
    if(!source||!gripSupport(w,position,*source,true)||bounded.exhausted)return false;
    for(float height:{6.f,cfg.chest,cfg.height}) {
        const Vec center=position+Vec{0,0,height};
        for(const auto direction:bodyRingDirections()) {
            const Vec outer=center+direction*(cfg.radius+4);
            if(w.ray(center,outer)||w.ray(outer,center)||bounded.exhausted)return false;
        }
    }
    const float trailing=sideways?cfg.radius:std::max(0.f,-6.f*heading.z);
    float measuredAdvance=0,measuredOutward=0;
    const Vec inside=obstruction.point+heading*2;
    if(const auto farFace=w.ray(inside+heading*176,obstruction.point-heading*4);
        farFace&&farFace->point.finite()&&farFace->normal.finite()&&farFace->normal.unit().dot(heading)>.15f) {
        measuredAdvance=(farFace->point-position).dot(heading)+trailing+6;
    }
    if(const auto edge=w.ray(inside+normal*(cfg.reach+cfg.radius+8),inside-normal*4);
        edge&&edge->point.finite()&&edge->normal.finite()&&edge->normal.unit().dot(normal)>.3f) {
        measuredOutward=(edge->point-position).dot(normal)+cfg.radius+6;
    }
    std::array<float,9> advances{measuredAdvance,measuredAdvance+12,measuredAdvance+28,52,80,104,128,152,176};
    std::array<float,8> outwardSteps{measuredOutward,measuredOutward+8,18,48,64,80,96,110};
    std::array<float,5> sideSteps=sideways?std::array<float,5>{0,0,0,0,0}:
        std::abs(intent.x)>.1f?std::array<float,5>{0,intent.x<0?-24.f:24.f,intent.x<0?-40.f:40.f,0,0}:
        std::array<float,5>{0,-24,24,-40,40};
    for(std::size_t si=0;si<sideSteps.size();++si) {
        const float side=sideSteps[si];
        if(si>0&&side==0)continue;
        for(std::size_t oi=0;oi<outwardSteps.size();++oi) {
            const float outward=outwardSteps[oi];
            if(bounded.exhausted)return false;
            if(outward<12||outward>cfg.reach)continue;
            bool duplicate=false;
            for(std::size_t prior=0;prior<oi;++prior)duplicate|=std::abs(outwardSteps[prior]-outward)<1;
            if(duplicate)continue;
            const Vec shift=normal*outward+right*side;
            if(shift.length()>cfg.reach+.01f)continue;
            const Vec outside=position+shift;
            if(!sourceRoute&&(!clearPath(w,position,outside)||!roofPathClear(w,position,outside)))continue;
            bool prefixChecked=false,prefixClear=false;
            for(std::size_t ai=0;ai<advances.size();++ai) {
                const float advance=advances[ai];
                if(bounded.exhausted)return false;
                if(advance<48||advance>176)continue;
                duplicate=false;
                for(std::size_t prior=0;prior<ai;++prior)duplicate|=std::abs(advances[prior]-advance)<1;
                if(duplicate)continue;
                const Vec over=outside+heading*advance;
                if(!sourceRoute&&(!clearPath(w,outside,over)||!roofPathClear(w,outside,over))) {
                    if(!prefixChecked) {
                        const Vec prefix=outside+heading*48;
                        prefixClear=clearPath(w,outside,prefix)&&roofPathClear(w,outside,prefix);
                        prefixChecked=true;
                    }
                    if(!prefixClear)break;
                    continue;
                }
                for(Vec search:{over,position+heading*advance+right*side}) {
                    bool nearby=false;
                    for(float height:{cfg.chest,32.f,cfg.grip}) {
                        const Vec probe=search+Vec{0,0,height};
                        const auto hit=w.ray(probe+normal*16,probe-normal*cfg.reach);
                        if(hit&&hit->climbable&&hit->normal.finite()&&hit->normal.length()>.5f&&
                            std::abs(hit->normal.unit().z)<cfg.maxNormalZ){nearby=true;break;}
                    }
                    if(!nearby)continue;
                    const auto face=support(w,search,normal*-1,true,true);
                    if(!face||!face->climbable||!face->normal.finite())continue;
                    const Vec plane=face->normal.unit();
                    if(std::abs(plane.z)>=cfg.maxNormalZ||horizontal(plane).dot(normal)<.8f)continue;
                    const auto anchor=landingAnchor(w,search,*face);
                    if(!anchor||anchor->hit.normal.unit().dot(plane)<.95f||
                        !gripSupport(w,anchor->position,anchor->hit,true))continue;
                    const Vec target=anchor->position,delta=target-position;
                    if(delta.dot(heading)<48||delta.dot(heading)>176||delta.z<-.5f||
                        delta.length()>cfg.reach+80||(target-over).length()>cfg.reach||
                        (!sourceRoute&&(!clearPath(w,over,target)||!roofPathClear(w,over,target))))continue;
                    const float route=shift.length()+advance+(target-over).length();
                    const float cadence=wallRun?std::clamp(route/(std::max(150.f,speed)*1.35f),.58f,.90f):
                        std::clamp(route/340.f,.72f,1.04f);
                    const float peakDistance=1.5f*std::max({shift.length()*4,advance*2,(target-over).length()*4});
                    const float seconds=std::max(cadence,peakDistance/420.f);
                    if(stamina<(wallRun?30.f+2*cfg.drain*(motionDuration(selected,seconds)+.05f):15.f)||bounded.exhausted)return false;
                    if(sourceRoute) {
                        if(!commitAuthoredRoute(w,selected,target,seconds,horizontal(anchor->hit.normal),true,outside,over,true,anchor->hit.normal.unit(),0,wallRun?speed:0))continue;
                    } else {
                        const Vec startSurface=surfaceNormal;
                        beginAction(selected,position,target,seconds);
                        detour=true;detourOut=outside;detourOver=over;roofTransfer=true;
                        actionStartSurface=startSurface;actionTargetSurface=anchor->hit.normal.unit();
                        actionLandingNormal=horizontal(actionTargetSurface);
                    }
                    if(wallRun) {
                        actionBeganRunning=true;actionRunSpeed=speed;obstacleJump=true;++obstacleJumps;
                        obstacleProbeCooldown=.5f;runClearanceCooldown=0;
                    }
                    return true;
                }
            }
        }
    }
    return false;
}