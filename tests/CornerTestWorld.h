#pragma once
#include "Core.h"
#include <vector>
namespace fc_test {
using namespace fc;
struct Box {Vec low,high;bool climbable=true;};
struct CornerWorld final:World {
    std::vector<Box> boxes;
    std::vector<std::vector<Vec>> footprints;
    Vec origin{};float yaw{},mirror=1;unsigned rays{};
    Vec rotate(Vec value,float angle)const{return {value.x*std::cos(angle)-value.y*std::sin(angle),value.x*std::sin(angle)+value.y*std::cos(angle),value.z};}
    Vec direction(Vec value)const {value.x*=mirror;return rotate(value,yaw);}
    Vec global(Vec value)const {return origin+direction(value);}
    Vec local(Vec value)const {value=rotate(value-origin,-yaw);value.x*=mirror;return value;}
    void corner(bool convex) {
        boxes.clear();
        boxes.push_back({{-400,0,-400},{convex?0.f:400.f,400,400}});
        if(!convex)boxes.push_back({{0,-400,-400},{400,400,400}});
    }
    std::optional<Hit> ray(Vec from,Vec to)override {
        ++rays;const Vec start=local(from),delta=local(to)-start;
        std::optional<Hit> hit;double best=2;
        for(const auto& box:boxes) {
            double enter=0,leave=1;Vec normal{};bool rejected=false;
            for(unsigned axis=0;axis<3;++axis) {
                const float p=axis==0?start.x:axis==1?start.y:start.z;
                const float d=axis==0?delta.x:axis==1?delta.y:delta.z;
                const float lo=axis==0?box.low.x:axis==1?box.low.y:box.low.z;
                const float hi=axis==0?box.high.x:axis==1?box.high.y:box.high.z;
                if(std::abs(d)<1.e-7f){if(p<lo||p>hi){rejected=true;break;}continue;}
                const double a=(lo-double(p))/d,b=(hi-double(p))/d;
                const double first=std::min(a,b),last=std::max(a,b);
                if(first>enter) {enter=first;normal={};const float sign=a<b?-1.f:1.f;if(axis==0)normal.x=sign;else if(axis==1)normal.y=sign;else normal.z=sign;}
                leave=std::min(leave,last);if(leave<enter){rejected=true;break;}
            }
            if(!rejected&&enter>1.e-6&&enter<=1&&enter<best&&normal.length()>.9f) {
                best=enter;hit=Hit{global(start+delta*float(enter)),direction(normal),box.climbable};
            }
        }
        for(const auto& polygon:footprints) {
            double enter=0,leave=1;Vec normal{};bool rejected=false;
            auto plane=[&](Vec n,float distance) {
                const double d=start.dot(n)-distance,rate=delta.dot(n);
                if(std::abs(rate)<1.e-8){if(d>0)rejected=true;return;}
                const double phase=-d/rate;
                if(rate<0){if(phase>enter){enter=phase;normal=n;}}else leave=std::min(leave,phase);
                if(leave<enter)rejected=true;
            };
            for(unsigned index=0;index<polygon.size()&&!rejected;++index) {
                const Vec edge=polygon[(index+1)%polygon.size()]-polygon[index],outward=Vec{edge.y,-edge.x,0}.unit();
                plane(outward,polygon[index].dot(outward));
            }
            plane({0,0,1},400);plane({0,0,-1},400);
            if(!rejected&&enter>1.e-6&&enter<=1&&enter<best&&normal.length()>.9f) {
                best=enter;hit=Hit{global(start+delta*float(enter)),direction(normal),true};
            }
        }
        return hit;
    }

    float clearance(Vec feet,const Settings& cfg)const {
        const Vec point=local(feet);float distance=10000;
        for(const auto& box:boxes) {
            if(point.z+cfg.height<box.low.z||point.z+6>box.high.z)continue;
            const float x=std::max({box.low.x-point.x,0.f,point.x-box.high.x});
            const float y=std::max({box.low.y-point.y,0.f,point.y-box.high.y});
            distance=std::min(distance,std::hypot(x,y));
        }
        if(point.z+cfg.height>=-400&&point.z+6<=400)for(const auto& polygon:footprints) {
            bool interior=true;float perimeter=10000;
            for(unsigned index=0;index<polygon.size();++index) {
                const Vec start=polygon[index],edge=polygon[(index+1)%polygon.size()]-start;
                const Vec flat{point.x,point.y,0};
                if(edge.cross(flat-start).z<0)interior=false;
                const float along=std::clamp((flat-start).dot(edge)/edge.dot(edge),0.f,1.f);
                perimeter=std::min(perimeter,(flat-(start+edge*along)).length());
            }
            distance=std::min(distance,interior?0.f:perimeter);
        }
        return distance;
    }
};
Settings settings(){Settings cfg;cfg.gap=37;cfg.radius=31;cfg.height=138;cfg.approachSeconds=0;return cfg;}
}
