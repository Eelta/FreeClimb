#define main irregularFixtureMain
#include "IrregularCornerTests.cpp"
#undef main

namespace {
TriangleWorld bevelWorld(Vec shear,float side,bool distant,float width=4) {
    TriangleWorld world;world.shear=shear;world.mirror=side;world.yaw=.47f;
    if(distant)world.origin={131065.84f,41791.12f,-11204.17f};
    world.prism({{-400,0,0},{-width,0,0},{0,width,0},{0,400,0},{-400,400,0}});
    return world;
}
Vec bevelStart(const TriangleWorld& world) {
    return world.point({-80,-37,0})+cornerRotate({world.shear.x*6*world.mirror,world.shear.y*6,0},world.yaw);
}
Vec unshear(const TriangleWorld& world,Vec value) {
    value=cornerRotate(value-world.origin,-world.yaw);value.x*=world.mirror;
    value.x-=world.shear.x*value.z;value.y-=world.shear.y*value.z;return value;
}
bool measuredUpperPair(TriangleWorld& world,const Settings& cfg,Vec feet,Vec facing) {

    const Vec side=Vec{-facing.y,facing.x,0}.unit();
    for(float height:{cfg.grip,cfg.chest})for(float offset:{0.f,-8.f,8.f,-18.f,18.f}) {
        if(height==cfg.chest&&std::abs(facing.z)<.12f)continue;
        std::optional<Hit> previous;bool valid=true;
        for(float sample:{height-5,height+5}) {
            const Vec at=cornerAnchor(feet,facing,sample)+side*offset;
            const auto hit=world.ray(at+facing*12,at-facing*(cfg.gap+20));
            if(!hit||!hit->climbable||hit->normal.z<-.45f||hit->normal.z>cfg.maxNormalZ||
                (previous&&previous->normal.dot(hit->normal)<.985f)){valid=false;break;}
            previous=hit;
        }
        if(valid)return true;
    }
    return false;
}
void facetedCore(const Library& library,Vec shear,int fps,float side,bool distant,bool run,int intent) {
    auto world=bevelWorld(shear,side,distant);Traversal traversal;traversal.cfg=config();
    const Vec start=bevelStart(world);
    require(traversal.attach(world,start,world.normal({0,-1,0})*-1,1000),"real Core entry attaches before a short beveled seam");
    SurfacePose surface;Pose oldPose;Vec old=traversal.position,oldNormal=traversal.normal;
    float clearance=1e9f,maxYaw=0,maxBone=0;unsigned entries=0,exits=0,peakCasts=0;bool inCorner=false,completed=false,reversed=false,failed=false,grips=true,poseValid=true;
    int reverseAt=-1;const float dt=1.f/fps;
    for(int frame=0;frame<fps*7&&traversal.active();++frame) {
        Input input;input.x=side*(reverseAt>=0?-1.f:1.f);input.run=run;
        if(intent==2&&traversal.turningCorner())input.y=.7f;
        world.casts=0;const auto result=traversal.update(world,input,dt,1000);peakCasts=std::max(peakCasts,world.casts);
        const bool active=traversal.turningCorner();entries+=active&&!inCorner;exits+=!active&&inCorner;inCorner=active;
        failed|=result.released;
        const unsigned samples=std::max(1u,unsigned(std::ceil((traversal.position-old).length()/.5f)));
        for(unsigned sample=0;sample<=samples;++sample)
            clearance=std::min(clearance,world.solid.capsuleDistance(old+(traversal.position-old)*(float(sample)/samples)));
        if(active)grips&=measuredUpperPair(world,traversal.cfg,traversal.position,traversal.surfaceNormal);
        maxYaw=std::max(maxYaw,std::acos(std::clamp(oldNormal.dot(traversal.normal),-1.f,1.f)));
        const auto pose=surface.update(library,world,traversal,result.motion,dt,1);
        poseValid&=pose.size()==99;
        for(unsigned bone=0;bone<pose.size();++bone) {
            poseValid&=pose[bone].t.finite()&&std::isfinite(pose[bone].q.dot(pose[bone].q));
            if(oldPose.size()==pose.size()) {
                const float step=angleBetween(oldPose[bone].q,pose[bone].q);maxBone=std::max(maxBone,step);
                poseValid&=step<=(runMotion(result.motion)?18.849556f:12.566371f)*dt+.016f;
            }
        }
        oldPose=pose;old=traversal.position;oldNormal=traversal.normal;
        const Vec local=unshear(world,traversal.position);
        const bool atTarget=traversal.surfaceNormal.dot(world.normal({1,0,0}))>.995f&&local.y>24&&!active;
        if(atTarget&&reverseAt<0) {
            completed=true;if(intent!=1)break;reverseAt=frame;
        }
        if(reverseAt>=0&&frame>reverseAt&&traversal.surfaceNormal.dot(world.normal({0,-1,0}))>.995f&&local.x<-24&&!active){reversed=true;break;}
    }
    require(completed&&!failed,"held lateral intent completes both short seams without hanging or releasing");
    if(intent==1)require(reversed,"reversed actual input returns across the two adjoining facets");
    if(intent==2)require(traversal.position.z-start.z>5,"diagonal input actually advances height while turning");
    require(entries>=2&&exits>=2,"actual Core finishes both measured seams instead of snapping directly from the short facet onto the third face");
    if(intent==1)require(entries>=4&&exits>=4,"reversed actual input measures and completes both seams in the return direction");
    require(clearance>=30.96f,"all actual movement chords retain full radius31 height138 capsule outside the closed solid");
    require(grips,"actual turn samples retain a real upper same-face contact pair");
    require(maxYaw<=8.3f/fps+.025f,"short-facet handoff retains the established continuous heading bound");
    require(poseValid,"all99 final bones remain finite and within existing ordinary/run angular budgets");
    ++cases;std::cout<<"faceted fps="<<fps<<" side="<<side<<" far="<<distant<<" run="<<run<<" intent="<<intent<<" shear="<<shear.x<<','<<shear.y
        <<" completed="<<completed<<" reversed="<<reversed<<" entries="<<entries<<" exits="<<exits<<" clear="<<clearance<<" grips="<<grips<<" yaw="<<maxYaw<<" bone="<<maxBone<<" peakCasts="<<peakCasts<<'\n';
}
void facetNegatives(float side,bool distant) {
    const auto cfg=config();auto world=bevelWorld({},side,distant);const Vec start=bevelStart(world),normal=world.normal({0,-1,0});
    auto route=findCornerRoute(world,cfg,start,normal,side,[](Vec,Vec){return true;});
    require(route.has_value(),"negative fixtures begin from an otherwise measured legal faceted route");if(!route)return;
    require(!findCornerRoute(world,cfg,start,normal,side,[](Vec,Vec){return false;}),"new short face certificates never bypass the caller's full body path rejection");
    struct Limited final:World {
        TriangleWorld& actual;unsigned limit,count{};bool exhausted{};
        Limited(TriangleWorld& source,unsigned maximum):actual(source),limit(maximum){}
        std::optional<Hit> ray(Vec from,Vec to)override {
            if(count++>=limit){exhausted=true;return Hit{from,(from-to).unit(),false};}
            return actual.ray(from,to);
        }
    };
    for(unsigned limit:{0u,8u,32u,64u,128u}) {
        Limited limited(world,limit);
        require(!findCornerRoute(limited,cfg,start,normal,side,[](Vec,Vec){return true;})&&limited.exhausted,
            "an exhausted query budget remains unknown/blocked and never certifies a short seam");
    }
    {
        auto shortSolid=bevelWorld({},side,distant);shortSolid.solid.triangles.clear();
        shortSolid.prism({{-400,0,0},{-4,0,0},{0,4,0},{0,400,0},{-400,400,0}},-400,95);
        require(!findCornerRoute(shortSolid,cfg,start,normal,side,[](Vec,Vec){return true;}),"a closed low solid without actual upper hands cannot become a corner grip");
    }
    {
        auto gap=bevelWorld({},side,distant);gap.solid.triangles.clear();
        gap.prism({{-400,0,0},{-6,0,0},{-6,400,0},{-400,400,0}});
        gap.prism({{0,6,0},{400,6,0},{400,400,0},{0,400,0}});
        require(!findCornerRoute(gap,cfg,start,normal,side,[](Vec,Vec){return true;}),"separate closed solids with an unjoined gap are not joined by the short-face fallback");
    }
    {
        auto missing=world;missing.solid.triangles.clear();auto live=*route;const float distance=live.distance;
        require(!advanceCornerRoute(missing,cfg,live,2,[](Vec,Vec){return true;})&&live.distance==distance,
            "removed live support cannot move or consume a stored corner route");
    }
    {

        auto blocked=world;Traversal traversal;traversal.cfg=cfg;
        require(traversal.attach(blocked,start,normal*-1,1000),"dynamic-blocker Core attaches before support changes");
        for(unsigned i=0;i<120&&!traversal.turningCorner();++i)traversal.update(blocked,{side,0},1.f/60,1000);
        require(traversal.turningCorner(),"blocker is added only after actual Core has selected a route");
        const Vec future=unshear(blocked,cornerSample(*route,route->length*.72f).position);
        blocked.prism({{future.x-3,future.y-3,0},{future.x+3,future.y-3,0},{future.x+3,future.y+3,0},{future.x-3,future.y+3,0}},future.z+50,future.z+90);
        require(blocked.solid.capsuleDistance(traversal.position)>=30.96f,"new closed blocker starts ahead rather than being spawned inside the actor");
        bool stopped=false;float clearance=1e9f;Vec old=traversal.position;
        for(unsigned i=0;i<180&&traversal.active();++i) {
            traversal.update(blocked,{side,0},1.f/60,1000);
            for(unsigned j=0;j<=8;++j)clearance=std::min(clearance,blocked.solid.capsuleDistance(old+(traversal.position-old)*(j/8.f)));
            stopped|=(traversal.position-old).length()<.01f;
            old=traversal.position;
        }
        require(stopped&&clearance>=30.96f,"a newly inserted finite body blocker stops live travel before any full capsule intersection");
        std::cout<<"dynamic-blocker side="<<side<<" far="<<distant<<" stopped="<<stopped<<" clearance="<<clearance<<'\n';
    }
    ++cases;
}
}
int main(int argc,char** argv) {
    Library library;if(argc!=2||!library.load(argv[1])){std::cerr<<"motion library required\n";return 1;}
    for(int fps:{30,60,120})for(float side:{-1.f,1.f})for(bool distant:{false,true}) {
        for(Vec shear:{Vec{},Vec{-.03f,.025f,0},Vec{-.15f,.10f,0}})
            facetedCore(library,shear,fps,side,distant,false,0);
        facetedCore(library,{},fps,side,distant,true,0);
        facetedCore(library,{},fps,side,distant,false,1);
        facetedCore(library,{-.03f,.025f,0},fps,side,distant,false,2);
    }
    for(float side:{-1.f,1.f})for(bool distant:{false,true})facetNegatives(side,distant);
    std::cout<<"FacetedCornerSafetyTests cases="<<cases<<" failures="<<failures<<'\n';return failures?1:0;
}
