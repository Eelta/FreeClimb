#include "Pose.h"
#include <bit>
#include <cstdlib>
#include <iostream>
#include <new>
#include <stdexcept>
using namespace fc;
static thread_local bool countAllocations{};
static thread_local std::size_t allocations{};
void* operator new(std::size_t size){if(countAllocations)++allocations;if(auto* result=std::malloc(size))return result;throw std::bad_alloc();}
void operator delete(void* value)noexcept{std::free(value);}
void operator delete(void* value,std::size_t)noexcept{std::free(value);}
void* operator new[](std::size_t size){return ::operator new(size);}
void operator delete[](void* value)noexcept{::operator delete(value);}
void operator delete[](void* value,std::size_t)noexcept{::operator delete(value);}
static void check(bool good,const char* message){if(!good)throw std::runtime_error(message);}
static bool exact(float a,float b){return std::bit_cast<std::uint32_t>(a)==std::bit_cast<std::uint32_t>(b);}
static bool exact(Vec a,Vec b){return exact(a.x,b.x)&&exact(a.y,b.y)&&exact(a.z,b.z);}
static bool exact(Quat a,Quat b){return exact(a.x,b.x)&&exact(a.y,b.y)&&exact(a.z,b.z)&&exact(a.w,b.w);}
static bool exact(const Transform& a,const Transform& b){return exact(a.t,b.t)&&exact(a.q,b.q)&&exact(a.s,b.s);}
static void equivalent(const Library& library,const Pose& pose) {
    const auto complete=library.world(pose);
    for(std::size_t bone=0;bone<pose.size();++bone) {
        allocations=0;countAllocations=true;const auto actual=library.worldBone(pose,int(bone));countAllocations=false;
        check(actual&&exact(*actual,complete[bone]),"single-bone transform exactly matches full forward kinematics");
        check(allocations==0,"bounded ancestor transforms perform no heap allocations");
    }
}
static void boundaries() {
    Library lib;lib.parents={-1,0,1};Pose pose(3);pose[0].t={1,2,3};pose[1].t={4,5,6};pose[2].t={7,8,9};equivalent(lib,pose);
    check(!lib.worldBone(pose,-1)&&!lib.worldBone(pose,3)&&!lib.worldBone({},0),"invalid or absent requested tracks are rejected");
    lib.parents.pop_back();check(!lib.worldBone(pose,2),"missing hierarchy entries are rejected");
    lib.parents={-1,9,1};check(!lib.worldBone(pose,2),"out-of-range ancestors are rejected");
    lib.parents={-1,2,1};check(!lib.worldBone(pose,2),"cycles are bounded and rejected");
    lib.parents.resize(99);pose.resize(99);for(int i=0;i<99;++i)lib.parents[i]=i-1;equivalent(lib,pose);
    lib.parents.push_back(98);pose.emplace_back();check(!lib.worldBone(pose,99),"chains exceeding the supported skeleton bound are rejected");
    lib.parents.back()=-1;check(lib.worldBone(pose,99).has_value(),"independent roots are valid without assuming canonical bone numbers");
    const auto before=pose;lib.rotateWorld(pose,-1,{});lib.rotateWorld(pose,100,{});lib.contactOrientation(pose,100,{},{});
    for(std::size_t bone=0;bone<pose.size();++bone)check(exact(before[bone],pose[bone]),"out-of-range rotation requests do not modify any pose track");
}
static void variedHierarchies() {
    std::uint32_t state=0x5eeda11u;auto next=[&]{state=state*1664525u+1013904223u;return state;};
    for(int tree=0;tree<64;++tree) {
        Library lib;Pose pose(99);lib.parents.resize(99);
        for(int bone=0;bone<99;++bone) {
            lib.parents[bone]=bone==0||next()%11==0?-1:int(next()%bone);
            pose[bone].t={float(int(next()%401)-200)*.1f,float(int(next()%401)-200)*.1f,float(int(next()%401)-200)*.1f};
            pose[bone].q=Quat::axis(Vec{float(next()%31+1),float(next()%37+1),float(next()%41+1)},float(next()%629)*.01f);
            pose[bone].s={.8f+float(next()%41)*.01f,.8f+float(next()%41)*.01f,.8f+float(next()%41)*.01f};
        }
        equivalent(lib,pose);
    }
}
int main(int argc,char** argv)try {
    check(argc==2,"current animation pack path required");Library lib;check(lib.load(argv[1]),"load current complete animation pack");
    unsigned poses=0;for(int value=1;value<=motionCount;++value)if(isActiveMotion(Motion(value)))
        for(float phase:{0.f,.13f,.37f,.73f,1.f}){equivalent(lib,lib.sample(Motion(value),phase));++poses;}
    boundaries();variedHierarchies();
    std::cout<<"PASS exact allocation-free bone transforms: "<<poses<<" real clip samples, 64 varied hierarchies, roots, nonuniform scales and malformed boundaries\n";return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
