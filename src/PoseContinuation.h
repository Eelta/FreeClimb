#pragma once

class PoseContinuation {
    Pose source;
    std::vector<Vec> linear,angular;
public:
    void begin(const Pose& current,const Pose& older,float seconds) {
        source=current;linear.assign(source.size(),{});angular.assign(source.size(),{});
        if(older.size()!=current.size()||seconds<=1e-5f)return;
        for(std::size_t i=0;i<source.size();++i) {
            linear[i]=(current[i].t-older[i].t)/seconds;
            if(linear[i].length()>400)linear[i]=linear[i].unit()*400;
            auto delta=(current[i].q*older[i].q.inverse()).unit();
            if(delta.w<0)delta={-delta.x,-delta.y,-delta.z,-delta.w};
            const Vec axis{delta.x,delta.y,delta.z};
            const float radians=2*std::atan2(axis.length(),std::clamp(delta.w,0.f,1.f));
            angular[i]=axis.unit()*std::min(12.f,radians/seconds);
        }
    }
    Pose sample(float elapsed) const {
        Pose result=source;

        const float time=.065f*(1-std::exp(-std::max(0.f,elapsed)/.065f));
        for(std::size_t i=0;i<result.size();++i) {
            result[i].t=result[i].t+linear[i]*time;
            const float speed=angular[i].length();
            if(speed>1e-5f)result[i].q=(Quat::axis(angular[i]/speed,speed*time)*result[i].q).unit();
        }
        return result;
    }
};
