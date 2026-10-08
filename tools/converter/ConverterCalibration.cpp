#include "ConverterCalibration.h"
#include "MotionSlots.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace fc {
namespace {
using Json=nlohmann::json;
constexpr std::size_t count=65;
constexpr std::array<int,10> joints{29,32,50,51,7,10,36,5,28,31};
constexpr std::array<int,12> rotations{5,6,7,9,10,24,26,28,29,31,32,36};
struct Feature {std::array<Vec,12> points;std::array<Quat,12> angles;};
Pose sample(const std::vector<Pose>& frames,float phase) {
    const float frame=std::clamp(phase,0.f,1.f)*float(frames.size()-1);
    const auto a=std::min(std::size_t(frame),frames.size()-2);Pose result=frames[a];
    for(std::size_t bone=0;bone<result.size();++bone)result[bone]=blend(result[bone],frames[a+1][bone],frame-float(a));
    return result;
}
Feature feature(const Library& base,const Pose& pose) {
    const auto world=base.world(pose);const auto origin=world[4].t;
    const auto inverse=world[0].q.inverse();Feature result;
    for(int hand=0;hand<2;++hand)result.points[hand]=inverse.rotate(base.palm(world,hand)-origin)/65.f;
    for(std::size_t i=0;i<joints.size();++i)result.points[i+2]=inverse.rotate(world[joints[i]].t-origin)/65.f;
    for(std::size_t i=0;i<rotations.size();++i)result.angles[i]=pose[rotations[i]].q.unit();
    return result;
}
float difference(const Feature& a,const Feature& b) {
    float distance=0,weights=0;
    for(std::size_t i=0;i<a.points.size();++i) {
        const float weight=i<2?3.f:i<6?1.5f:.5f;const auto delta=a.points[i]-b.points[i];
        distance+=weight*delta.dot(delta);weights+=weight;
    }
    distance/=weights;
    for(std::size_t i=0;i<a.angles.size();++i) {
        const float dot=std::clamp(std::abs(a.angles[i].dot(b.angles[i])),0.f,1.f);
        distance+=(1-dot*dot)*.006f;
    }
    return distance;
}
float at(const std::vector<std::array<float,2>>& map,float phase) {
    const float position=std::clamp(phase,0.f,1.f)*float(map.size()-1);
    const auto index=std::min(std::size_t(position),map.size()-2);
    return map[index][1]+(map[index+1][1]-map[index][1])*(position-float(index));
}
float inverseAt(const std::vector<std::array<float,2>>& map,float phase) {
    phase=std::clamp(phase,0.f,1.f);
    const auto next=std::upper_bound(map.begin(),map.end(),phase,[](float value,const auto& item){return value<item[1];});
    if(next==map.begin())return 0;if(next==map.end())return 1;
    const auto& a=*(next-1);const auto& b=*next;
    return a[0]+(b[0]-a[0])*std::clamp((phase-a[1])/(b[1]-a[1]),0.f,1.f);
}
bool validWindow(const Json& value) {
    return value.is_array()&&value.size()==2&&value[0].is_number()&&value[1].is_number()&&
        std::isfinite(value[0].get<float>())&&std::isfinite(value[1].get<float>())&&
        value[0].get<float>()>=0&&value[1].get<float>()<=1&&value[1].get<float>()-value[0].get<float>()>=.001f;
}
void remapConfiguration(Json& config,const std::vector<std::array<float,2>>& map,std::size_t frames) {
    const auto contacts=config.at("contacts");
    if(!contacts.is_array()||contacts.size()<2||contacts.size()>1201)throw std::runtime_error("Invalid template contact samples");
    for(const auto& row:contacts) {
        if(!row.is_array()||row.size()!=4)throw std::runtime_error("Template contacts require four limb weights");
        for(const auto& value:row)if(!value.is_number()||!std::isfinite(value.get<float>())||value.get<float>()<0||value.get<float>()>1)
            throw std::runtime_error("Template contact weight is outside 0..1");
    }
    config["contacts"]=Json::array();
    for(std::size_t frame=0;frame<frames;++frame) {
        const float position=at(map,float(frame)/float(frames-1))*float(contacts.size()-1);
        const auto a=std::min(std::size_t(position),contacts.size()-2);Json row=Json::array();
        for(int limb=0;limb<4;++limb) {
            const float low=contacts[a][limb].get<float>(),high=contacts[a+1][limb].get<float>();
            row.push_back(std::clamp(low+(high-low)*(position-float(a)),0.f,1.f));
        }
        config["contacts"].push_back(std::move(row));
    }
    const auto window=[&](Json& value) {
        value=Json::array({inverseAt(map,value.at(0).get<float>()),inverseAt(map,value.at(1).get<float>())});
        if(!validWindow(value))throw std::runtime_error("Automatic timing would collapse a support transition");
    };
    for(const auto key:{"sourceHands","targetHands","releaseHands"})if(config.contains(key))
        for(auto& value:config[key])window(value);
    for(const auto key:{"verticalBlend","unplant","replant"})if(config.contains(key))window(config[key]);
    if(config.contains("replantSamplePhase"))config["replantSamplePhase"]=inverseAt(map,config["replantSamplePhase"].get<float>());
    if(config.contains("path")) {
        float previous=-1;
        for(auto& row:config["path"]) {
            const float phase=inverseAt(map,row.at(0).get<float>());
            if(previous>=0&&phase-previous<.001f)throw std::runtime_error("Automatic timing would collapse route samples");
            row[0]=phase;previous=phase;
        }
    }
}
void geometryWarnings(const Library& base,std::string_view slot,const ConverterInputClip& clip,std::vector<std::string>& warnings) {
    const auto first=base.world(clip.frames.front()),last=base.world(clip.frames.back());
    const auto displacement=last[4].t-first[4].t;
    if(displacement.length()>4&&slot!="contextMantle")
        warnings.push_back("This clip has net body movement. Check the In-place option before export; FreeClimb also moves the character along its own route.");
    if(slot=="contextHang"||slot=="contextMantle") {
        const auto left=base.palm(first,0),right=base.palm(first,1),middle=(left+right)*.5f;
        const float width=std::abs(right.x-left.x)*.5f;
        if(slot=="contextHang"&&(middle.z<=90||middle.z>=175||width<=8||width>=40))
            warnings.push_back("The first hanging pose does not fit the supported hand height or spacing. Adjust the pose before using it as a hanging reference.");
        if(slot=="contextMantle"&&(middle.z<65||std::abs(middle.x)>24||std::abs(left.z-right.z)>28))
            warnings.push_back("The first mantle pose has unusual hand placement. Review the wall/ledge reference before export.");
    }
}
}
ConverterCalibration calibrateConverterClip(const Library& base,std::string_view slot,const ConverterInputClip& clip,const Json& templateConfig) {
    const auto found=std::find(motionSlotNames.begin(),motionSlotNames.end(),slot);
    if(found==motionSlotNames.end()||clip.frames.size()<2||clip.frames.size()>1201||base.parents.size()!=99)
        throw std::runtime_error("Cannot calibrate an invalid animation or action slot");
    for(std::size_t i=0;i<base.parents.size();++i)if(base.parents[i]<-1||base.parents[i]>=int(i))
        throw std::runtime_error("Calibration reference hierarchy is invalid");
    const auto validate=[](const std::vector<Pose>& frames) {
        if(frames.size()<2||frames.size()>1201)throw std::runtime_error("Calibration requires valid reference frames");
        for(const auto& pose:frames) {
            if(pose.size()!=99)throw std::runtime_error("Calibration requires the converted 99-track pose");
            for(const auto& bone:pose) {
                const float norm=bone.q.dot(bone.q);
                if(!bone.t.finite()||!bone.s.finite()||!std::isfinite(norm)||std::abs(norm-1)>.01f)
                    throw std::runtime_error("Calibration pose contains invalid values");
            }
        }
    };
    validate(clip.frames);
    const auto& reference=base.clips[std::size_t(found-motionSlotNames.begin())];
    if(reference.frames.size()<2)throw std::runtime_error("Calibration reference is missing");
    validate(reference.frames);
    ConverterCalibration result;result.config=templateConfig;
    std::array<Feature,count> authored{},target{};
    float identityError=0,activity=0;
    for(std::size_t i=0;i<count;++i) {
        const float phase=float(i)/float(count-1);
        authored[i]=feature(base,sample(clip.frames,phase));target[i]=feature(base,sample(reference.frames,phase));
        identityError+=difference(authored[i],target[i]);
        if(i)activity+=difference(authored[i],authored[i-1]);
        result.phaseMap.push_back({phase,phase});
    }
    identityError/=count;
    if(identityError<.000001f)result.confidence=1;
    else {
        std::array<std::array<float,count>,count> cost{};
        std::array<std::array<unsigned char,count>,count> steps{};
        for(auto& row:cost)row.fill(std::numeric_limits<float>::infinity());
        cost[0][0]=difference(authored[0],target[0]);
        for(std::size_t i=0;i<count;++i)for(std::size_t j=0;j<count;++j) {
            if((!i&&!j)||std::abs(int(i)-int(j))>20)continue;
            float previous=std::numeric_limits<float>::infinity();unsigned char step=0;
            if(i&&j){previous=cost[i-1][j-1];step=1;}
            if(i&&cost[i-1][j]+.003f<previous){previous=cost[i-1][j]+.003f;step=2;}
            if(j&&cost[i][j-1]+.003f<previous){previous=cost[i][j-1]+.003f;step=3;}
            cost[i][j]=previous+difference(authored[i],target[j]);steps[i][j]=step;
        }
        std::array<float,count> phaseSum{},phaseCount{};
        std::size_t i=count-1,j=count-1,visits=0;float mismatch=0;
        for(;;) {
            phaseSum[i]+=float(j)/float(count-1);phaseCount[i]+=1;mismatch+=difference(authored[i],target[j]);++visits;
            if(!i&&!j)break;
            const auto step=steps[i][j];if(step==1){--i;--j;}else if(step==2)--i;else if(step==3)--j;
            else throw std::runtime_error("Animation timing alignment could not be completed");
        }
        mismatch/=float(visits);result.confidence=std::clamp(std::exp(-6.f*mismatch),0.f,1.f);
        const float ends=.5f*(difference(authored.front(),target.front())+difference(authored.back(),target.back()));
        result.confidence*=std::clamp(std::exp(-2.f*ends),0.f,1.f);
        if(activity<.0001f&&identityError>.002f)result.confidence=std::min(result.confidence,.2f);
        if(result.confidence>=.55f) {
            std::array<float,count> mapped{};
            for(std::size_t index=0;index<count;++index)mapped[index]=.2f*float(index)/float(count-1)+.8f*phaseSum[index]/std::max(phaseCount[index],1.f);
            mapped.front()=0;mapped.back()=1;
            for(std::size_t index=1;index+1<count;++index)result.phaseMap[index][1]=(mapped[index-1]+2*mapped[index]+mapped[index+1])*.25f;
        } else result.warnings.push_back("This animation differs substantially from the selected action. Default support timing was kept; check the preview or choose a closer action.");
    }
    try {remapConfiguration(result.config,result.phaseMap,clip.frames.size());}
    catch(const std::exception&) {
        result.config=templateConfig;result.confidence=std::min(result.confidence,.4f);
        for(auto& point:result.phaseMap)point[1]=point[0];
        remapConfiguration(result.config,result.phaseMap,clip.frames.size());
        result.warnings.push_back("The suggested timing was too compressed. Default timing was retained to keep valid transitions.");
    }
    if(result.confidence<.9f&&result.confidence>=.55f)
        result.warnings.push_back("Support timing was matched to the reference action. Review the colored hand/foot timeline; this is an estimate, not detected wall contact.");
    geometryWarnings(base,slot,clip,result.warnings);
    return result;
}
}
