#pragma once

bool tryRecessedWallTransfer(World& originalWorld,Input intent,bool wallRun,float speed,float stamina) {
    if(intent.y<=.5f||std::abs(intent.x)>=.5f||intent.release||intent.backDrop||
        (wallRun&&!cfg.wallRunObstacleJumps)||!position.finite()||std::abs(surfaceNormal.z)>.12f)return false;
    struct BudgetWorld final:World {
        World& source;unsigned calls{},limit{};bool exhausted{};
        BudgetWorld(World& world,unsigned budget):source(world),limit(budget){}
        std::optional<Hit> ray(Vec a,Vec b)override {
            if(calls>=limit){exhausted=true;return Hit{a,(a-b).unit(),false};}
            ++calls;return source.ray(a,b);
        }
    } w(originalWorld,wallRun?4096:1800);
    const auto source=support(w,position,normal*-1);
    if(!source||!gripSupport(w,position,*source,true)||w.exhausted)return false;
    const auto edge=findGripEdge(w,position,normal,cfg,50,130);
    if(!edge||edge->normal.dot(normal)<.985f||w.exhausted)return false;
    for(float height:{6.f,cfg.chest,cfg.height})for(unsigned ring=0;ring<16;++ring) {
        const float angle=ring*(6.28318530718f/16);const Vec center=position+Vec{0,0,height};
        const Vec outer=center+Vec{std::cos(angle),std::sin(angle),0}*(cfg.radius+4);
        if(w.ray(center,outer)||w.ray(outer,center)||w.exhausted)return false;
    }
    const Vec probe{position.x,position.y,edge->center.z+cfg.chest};
    const auto upper=w.ray(probe+normal*16,probe-normal*cfg.reach);
    if(!upper||!upper->climbable||!upper->point.finite()||!upper->normal.finite()||
        std::abs(upper->normal.unit().z)>.12f||upper->normal.unit().dot(normal)<.985f)return false;
    const float recession=(upper->point-edge->center).dot(normal*-1);
    if(recession<6||recession>cfg.reach||w.exhausted)return false;
    for(float lift:{4.f,16.f,32.f}) {
        const Vec wanted{position.x,position.y,edge->center.z+lift};
        const auto anchor=landingAnchor(w,wanted,*upper);
        if(!anchor||anchor->hit.normal.unit().dot(upper->normal.unit())<.985f||
            !gripSupport(w,anchor->position,anchor->hit,true))continue;
        const Vec target=anchor->position,delta=target-position;
        if(delta.z<48||delta.z>176||delta.length()>cfg.reach+80||
            std::abs(delta.dot(Vec{-normal.y,normal.x,0}))>2||
            !findWallGrip(w,target,normal,cfg,cfg.grip,gripEdgeDetail::handHalfSpacing))continue;
        for(float outward:{18.f,32.f,48.f,64.f,96.f}) {
            if(w.exhausted)return false;
            if(outward>cfg.reach)continue;
            const Vec outside=position+normal*outward,over=outside+Vec{0,0,delta.z};
            if((target-over).length()>cfg.reach||!clearPath(w,position,outside)||!roofPathClear(w,position,outside)||
                !clearPath(w,outside,over)||!roofPathClear(w,outside,over)||
                !clearPath(w,over,target)||!roofPathClear(w,over,target))continue;
            auto point=[&](float phase) {
                if(phase<.25f)return lerp(position,outside,phase/.25f);
                if(phase<.75f)return lerp(outside,over,(phase-.25f)/.5f);
                return lerp(over,target,(phase-.75f)/.25f);
            };
            bool kick=wallRun&&kickClearance(w,position);
            if(kick)for(unsigned knot=1;knot<=32;++knot)
                if(!kickClearance(w,point(knot/32.f))){kick=false;break;}
            if(w.exhausted)return false;
            const float peak=1.5f*std::max({(outside-position).length()*4,(over-outside).length()*2,(target-over).length()*4});
            const float seconds=std::max(wallRun?.58f:.72f,peak/(kick?680.f:420.f));
            if(stamina<(wallRun?30.f+2*cfg.drain*(seconds+.05f):15.f))return false;
            const auto startSurface=surfaceNormal;
            beginAction(kick?Motion::kickUp:Motion::hopUp,position,target,seconds);
            detour=true;detourOut=outside;detourOver=over;roofTransfer=true;
            actionStartSurface=startSurface;actionTargetSurface=anchor->hit.normal.unit();actionLandingNormal=normal;
            if(wallRun) {
                actionBeganRunning=true;actionRunSpeed=speed;obstacleJump=true;++obstacleJumps;
                obstacleProbeCooldown=.5f;runClearanceCooldown=0;
            }
            blockedReason=wallRun?"checked recessed wall-run catch":"checked recessed climb catch";
            return true;
        }
    }
    return false;
}
