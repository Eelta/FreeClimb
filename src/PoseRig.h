#pragma once
#include <array>
#include <cmath>
#include <cstddef>
#include <utility>

namespace fc {
template<class PoseType> class PoseRig {
public:
    static constexpr std::size_t count=99;
    using Local=typename PoseType::value_type;
private:
    PoseType original,native;
    std::array<Local,count> bases{};
    std::array<bool,count> mapped{};
    bool enabled{};
    template<class Vector> static bool same(Vector a,Vector b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
    static bool identity(const Local& value) {
        return same(value.t,decltype(value.t){})&&same(value.s,decltype(value.s){1,1,1})&&
            value.q.x==0&&value.q.y==0&&value.q.z==0&&std::abs(value.q.w)==1;
    }
public:
    template<class Vector> static bool validAxes(Vector x,Vector y,Vector z) {
        return x.finite()&&y.finite()&&z.finite()&&std::abs(x.dot(x)-1)<.02f&&std::abs(y.dot(y)-1)<.02f&&
            std::abs(z.dot(z)-1)<.02f&&std::abs(x.dot(y))<.02f&&std::abs(x.dot(z))<.02f&&
            std::abs(y.dot(z))<.02f&&x.cross(y).dot(z)>.98f;
    }
    static bool validLocal(const Local& value) {
        const auto norm=value.q.dot(value.q);
        return value.t.finite()&&value.t.length()<1000&&std::isfinite(norm)&&std::abs(norm-1)<.01f&&
            value.s.finite()&&value.s.x>=.1f&&value.s.y>=.1f&&value.s.z>=.1f&&value.s.x<=5&&value.s.y<=5&&value.s.z<=5;
    }
    static bool currentStructure(const Local& live,const Local& reference,const Local& captured) {
        return validLocal(live)&&validLocal(reference)&&same(reference.t,captured.t)&&same(reference.s,captured.s)&&
            reference.q.x==captured.q.x&&reference.q.y==captured.q.y&&reference.q.z==captured.q.z&&reference.q.w==captured.q.w;
    }
    static Local relative(const Local& parent,const Local& child) {
        const auto delta=parent.q.inverse().rotate(child.t-parent.t);
        return {{delta.x/parent.s.x,delta.y/parent.s.y,delta.z/parent.s.z},
            (parent.q.inverse()*child.q).unit(),{child.s.x/parent.s.x,child.s.y/parent.s.y,child.s.z/parent.s.z}};
    }
    static Local structuralReference(std::size_t bone,Local live,const Local& reference,bool translationLocked) {
        if(bone==0||bone==4||!translationLocked)live.t=reference.t;
        return live;
    }
    bool configure(const PoseType& source,const PoseType& actual,const std::array<Local,count>& basis,const std::array<bool,count>& selected) {
        if(source.size()!=count||actual.size()!=count)return false;
        bool changed=false;
        for(std::size_t i=0;i<count;++i) {
            if(!validLocal(source[i]))return false;
            if(!selected[i])continue;
            if(!validLocal(actual[i])||!validLocal(basis[i]))return false;
            changed|=(i!=0&&i!=4&&!same(actual[i].t,source[i].t))||!same(actual[i].s,source[i].s)||!identity(basis[i]);
        }
        original=source;native=actual;bases=basis;mapped=selected;enabled=changed;
        return true;
    }
    bool active()const{return enabled;}
    const PoseType& source()const{return original;}
    PoseType reference()const {auto result=original;adapt(result);return result;}
    Local toEffective(std::size_t bone,const Local& local)const {
        return enabled&&bone<count&&mapped[bone]&&!identity(bases[bone])?compose(bases[bone],local):local;
    }
    Local toLocal(std::size_t bone,const Local& effective)const {
        return enabled&&bone<count&&mapped[bone]&&!identity(bases[bone])?relative(bases[bone],effective):effective;
    }
    template<class Rotation> Rotation rotation(std::size_t bone,Rotation value)const {
        return enabled&&bone<count&&mapped[bone]&&!identity(bases[bone])?(bases[bone].q*value).unit():value;
    }
    void adapt(PoseType& pose)const {
        if(!enabled||pose.size()!=count)return;
        for(std::size_t i=0;i<count;++i)if(mapped[i]) {
            if(i!=0&&i!=4)pose[i].t=native[i].t+(pose[i].t-original[i].t);
            pose[i].s={native[i].s.x*(pose[i].s.x/original[i].s.x),native[i].s.y*(pose[i].s.y/original[i].s.y),native[i].s.z*(pose[i].s.z/original[i].s.z)};
            pose[i]=toEffective(i,pose[i]);
        }
    }
    void clear(){*this={};}
};
}
