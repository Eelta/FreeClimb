#include "ConverterTimeline.h"
#include "ConverterSourceMotion.h"
#include "../../src/AnimationConfig.h"
#include <set>

namespace fc {
namespace {
void requireTimeline(bool accepted,const char* message){if(!accepted)throw std::runtime_error(message);}
Pose sampleTimeline(const ConverterInputClip& clip,float phase){
    const float at=std::clamp(phase,0.f,1.f)*float(clip.frames.size()-1);const auto first=std::min(std::size_t(at),clip.frames.size()-2);
    Pose result(99);for(std::size_t bone=0;bone<99;++bone)result[bone]=blend(clip.frames[first][bone],clip.frames[first+1][bone],at-float(first));return result;
}
bool samePose(const Pose& a,const Pose& b){
    for(std::size_t bone=0;bone<99;++bone)if((a[bone].t-b[bone].t).length()>.0001f||(a[bone].s-b[bone].s).length()>.00001f||angleBetween(a[bone].q,b[bone].q)>.0001f)return false;return true;
}
void rotations(ConverterInputClip& clip){clip.rotations.resize(clip.frames.size());for(std::size_t frame=0;frame<clip.frames.size();++frame){auto& row=clip.rotations[frame];row.resize(99);for(int bone=0;bone<99;++bone)row[bone]=clip.frames[frame][bone].q;}}
void dimensions(const ConverterInputClip& clip){
    requireTimeline(std::isfinite(clip.duration)&&clip.duration>0&&clip.duration<=10&&clip.frames.size()>=2&&clip.frames.size()<=1201,"Timeline source duration or frame count is invalid");
    requireTimeline(clip.boneIndices.size()==99&&clip.trackNames.size()==99&&std::all_of(clip.frames.begin(),clip.frames.end(),[](const auto& row){return row.size()==99;}),"Timeline source must use canonical 99-bone frames");
    for(int bone=0;bone<99;++bone)requireTimeline(clip.boneIndices[bone]==bone,"Timeline source has a reordered binding");
    const auto& reference=clip.referenceFrame;if(!reference.samples.empty()){
        requireTimeline(reference.samples.size()>=2&&reference.samples.size()<=1201&&std::isfinite(reference.duration)&&std::abs(reference.duration-clip.duration)<.0001f,"Timeline extracted movement has invalid samples or duration");
        for(const auto& row:reference.samples)for(const auto value:row)requireTimeline(std::isfinite(value),"Timeline extracted movement contains a nonfinite value");
    }
}
}
static ConverterTimeline composeTimeline(const std::vector<ConverterTimelinePart>& parts,float transitionSeconds,double forcedRate){
    requireTimeline(!parts.empty()&&parts.size()<=3&&std::isfinite(transitionSeconds)&&transitionSeconds>=0&&transitionSeconds<=.25f,"Timeline requires 1..3 stages and a bounded transition");
    float duration=0;std::set<std::string> names;for(const auto& part:parts){dimensions(part.clip);requireTimeline(!part.name.empty()&&names.insert(part.name).second,"Timeline stage names must be unique");duration+=part.clip.duration;}
    requireTimeline(duration<=10,"Complete timeline exceeds ten seconds");
    const double baseRate=double(parts.front().clip.frames.size()-1)/parts.front().clip.duration,budgetDuration=duration+double(parts.size()-1)*transitionSeconds;double rate=forcedRate;bool resampled=forcedRate>0,quantized=false;float maximumPositionError=0,maximumAngleError=0,maximumClockError=0;
    for(unsigned multiple=1;rate==0&&multiple<=1200;++multiple){const double candidate=baseRate*multiple;if(candidate*budgetDuration+double(parts.size()-1)>1200.01)break;bool compatible=true;
        for(const auto& part:parts){const double sourceRate=double(part.clip.frames.size()-1)/part.clip.duration,ratio=candidate/sourceRate;compatible=compatible&&ratio>=.99999&&std::abs(ratio-std::round(ratio))<.0001;}
        if(compatible){rate=candidate;break;}}
    if(rate==0){for(unsigned multiple=1;multiple<=1200;++multiple){const double candidate=baseRate*multiple;if(candidate*budgetDuration+double(parts.size()-1)>1200.01)break;bool compatible=true;std::size_t frames=1;
            for(const auto& part:parts){const double sourceRate=double(part.clip.frames.size()-1)/part.clip.duration;const auto scale=std::max(std::size_t(1),std::size_t(std::llround(candidate/sourceRate)));const auto intervals=(part.clip.frames.size()-1)*scale;const double changed=double(intervals)/candidate;compatible=compatible&&std::abs(changed-part.clip.duration)<=std::min({.5/candidate,double(part.clip.duration)*.01,1.0/120});frames+=intervals;}
            frames+=std::size_t(std::ceil(transitionSeconds*candidate))*(parts.size()-1);if(compatible&&frames<=1201){rate=candidate;quantized=true;break;}}
    }
    if(rate==0){resampled=true;rate=std::floor((1200-double(parts.size()-1)*2)/budgetDuration);requireTimeline(rate>=30,"Complete timeline cannot retain a useful bounded sample rate");}
    ConverterTimeline result;result.clip=parts.front().clip;auto& output=result.clip;output.frames.clear();output.rotations.clear();output.annotations.clear();output.referenceFrame={};
    bool baked=false;for(const auto& part:parts)baked=baked||converterSourceMotionBaked(part.clip);
    for(const auto& part:parts){
        Vec shift{};if(!output.frames.empty())shift=output.frames.back()[0].t-part.clip.frames.front()[0].t;
        auto first=part.clip.frames.front();first[0].t=first[0].t+shift;std::size_t start=0;
        if(!output.frames.empty()){
            const auto previous=output.frames.back();if(samePose(previous,first))start=output.frames.size()-1;
            else{const auto intervals=std::max(std::size_t(1),std::size_t(std::ceil(double(transitionSeconds)*rate)));requireTimeline(output.frames.size()+intervals<=1201,"Complete timeline transition exceeds the frame budget");
                for(std::size_t step=1;step<intervals;++step){Pose pose(99);const float phase=smooth(float(step)/float(intervals));for(int bone=0;bone<99;++bone)pose[bone]=blend(previous[bone],first[bone],phase);output.frames.push_back(std::move(pose));}start=output.frames.size();output.frames.push_back(std::move(first));}
        }else output.frames.push_back(std::move(first));
        const auto stride=std::max(std::size_t(1),std::size_t(std::llround(rate*part.clip.duration/double(part.clip.frames.size()-1))));
        const auto intervals=quantized?(part.clip.frames.size()-1)*stride:std::size_t(std::llround(double(part.clip.duration)*rate));requireTimeline(intervals>=1&&start+intervals+1<=1201,"Complete timeline exceeds the frame budget");
        for(std::size_t step=1;step<=intervals;++step){auto pose=!resampled&&stride&&step%stride==0?part.clip.frames[std::min(step/stride,part.clip.frames.size()-1)]:sampleTimeline(part.clip,float(step)/float(intervals));pose[0].t=pose[0].t+shift;output.frames.push_back(std::move(pose));}
        result.ranges.push_back({part.name,{start,start+intervals},shift});const float offset=float(double(start)/rate),stageDuration=float(double(intervals)/rate);
        maximumClockError=std::max(maximumClockError,std::abs(stageDuration-part.clip.duration));
        if(resampled){for(std::size_t probe=0;probe<part.clip.frames.size()*2-1;++probe){const float phase=float(probe)/float((part.clip.frames.size()-1)*2),at=phase*float(intervals);const auto index=std::min(std::size_t(at),intervals-1);const auto expected=sampleTimeline(part.clip,phase);
            for(unsigned bone=0;bone<99;++bone){const auto actual=blend(output.frames[start+index][bone],output.frames[start+index+1][bone],at-float(index));const auto position=expected[bone].t+(bone==0?shift:Vec{});maximumPositionError=std::max(maximumPositionError,(actual.t-position).length());maximumAngleError=std::max(maximumAngleError,angleBetween(actual.q,expected[bone].q));requireTimeline((actual.s-expected[bone].s).length()<=.00001f,"Timeline resampling would change bone scales");}}
            if(maximumPositionError>.02f||maximumAngleError>.00174533f)throw std::runtime_error("Timeline resampling exceeds fidelity bounds: stage="+part.name+" sourceHz="+std::to_string(float(part.clip.frames.size()-1)/part.clip.duration)+" outputHz="+std::to_string(rate)+" positionError="+std::to_string(maximumPositionError)+" angleDegrees="+std::to_string(maximumAngleError*180.f/3.14159265358979323846f));}

        for(auto event:part.clip.annotations){if(event.text==converterSourceMotionMarker)continue;requireTimeline(std::isfinite(event.time)&&event.time>=0&&event.time<=part.clip.duration+.0001f&&event.track<99,"Timeline annotation lies outside its source");event.time=offset+std::clamp(event.time,0.f,part.clip.duration)*(stageDuration/part.clip.duration);output.annotations.push_back(std::move(event));}
    }
    output.duration=float(double(output.frames.size()-1)/rate);requireTimeline(output.duration<=10,"Complete timeline with transitions exceeds ten seconds");
    if(baked)output.annotations.push_back({0,0,std::string(converterSourceMotionMarker)});
    std::stable_sort(output.annotations.begin(),output.annotations.end(),[](const auto& a,const auto& b){return a.time<b.time;});
    output.annotations.erase(std::unique(output.annotations.begin(),output.annotations.end(),[](const auto& a,const auto& b){return a.track==b.track&&a.text==b.text&&std::abs(a.time-b.time)<.00001f;}),output.annotations.end());
    const auto reference=std::find_if(parts.begin(),parts.end(),[](const auto& part){return !part.clip.referenceFrame.samples.empty();});
    if(reference!=parts.end()){
        auto& merged=output.referenceFrame;merged.duration=output.duration;merged.up=reference->clip.referenceFrame.up;merged.forward=reference->clip.referenceFrame.forward;merged.samples.resize(output.frames.size());std::array<float,4> end{};std::size_t written=0;
        for(std::size_t stage=0;stage<parts.size();++stage){const auto& part=parts[stage].clip;const auto& range=result.ranges[stage].frames;const auto& data=part.referenceFrame;
            requireTimeline(data.samples.empty()||((data.up-merged.up).length()<.0001f&&(data.forward-merged.forward).length()<.0001f),"Extracted movement bases differ between timeline stages");
            for(;written<range[0];++written)merged.samples[written]=end;
            const auto origin=data.samples.empty()?std::array<float,4>{}:converter_source_motion::reference(data,0);const auto offset=end;
            for(std::size_t frame=range[0];frame<=range[1];++frame){const auto value=data.samples.empty()?std::array<float,4>{}:converter_source_motion::reference(data,part.duration*float(frame-range[0])/float(range[1]-range[0]));for(int axis=0;axis<4;++axis)merged.samples[frame][axis]=value[axis]-origin[axis]+offset[axis];}
            end=merged.samples[range[1]];written=range[1]+1;
        }
    }
    if(quantized)result.warnings.push_back("Stage timing was aligned to a common "+std::to_string(rate)+" Hz grid while preserving every source key pose. Maximum section timing change: "+std::to_string(maximumClockError)+" seconds (within half a sample interval, 1% and 1/120 second).");
    if(resampled)result.warnings.push_back("Stage frame grids were resampled at "+std::to_string(rate)+" Hz: maximum source-key/midpoint error "+std::to_string(maximumPositionError)+" units / "+std::to_string(maximumAngleError*180.f/3.14159265358979323846f)+" degrees; maximum section timing change "+std::to_string(maximumClockError)+" seconds (within half a sample interval).");
    rotations(output);return result;
}
ConverterTimeline composeConverterTimeline(const std::vector<ConverterTimelinePart>& parts,float transitionSeconds){
    try{return composeTimeline(parts,transitionSeconds,0);}catch(const std::runtime_error& first){
        if(std::string_view(first.what()).find("Timeline resampling exceeds fidelity bounds")!=0)throw;
        double duration=double(parts.size()-1)*transitionSeconds;for(const auto& part:parts)duration+=part.clip.duration;const double maximum=std::floor((1200-double(parts.size()-1)*2)/duration);std::set<double,std::greater<double>> rates;
        for(const auto& part:parts){const double source=double(part.clip.frames.size()-1)/part.clip.duration;for(unsigned multiple=1;multiple<=1200&&source*multiple<=maximum;++multiple)if(source*multiple>=30)rates.insert(source*multiple);}
        for(const auto rate:rates)try{return composeTimeline(parts,transitionSeconds,rate);}catch(const std::runtime_error&){}
        throw;
    }
}
ConverterInputClip sliceConverterTimeline(const ConverterInputClip& source,const nlohmann::json& config){
    if(!config.contains("frameRange")){requireTimeline(!config.contains("rootShift"),"Root shift requires a timeline range");return source;}
    dimensions(source);const auto range=animationFrameRange(config,source.frames.size());const auto first=range[0],last=range[1];Vec shift{};
    if(config.contains("rootShift")){const auto& value=config.at("rootShift");shift={value[0].get<float>(),value[1].get<float>(),value[2].get<float>()};}
    auto result=source;result.frames.assign(source.frames.begin()+first,source.frames.begin()+last+1);for(auto& pose:result.frames)pose[0].t=pose[0].t-shift;
    const float interval=source.duration/float(source.frames.size()-1),begin=interval*float(first),end=interval*float(last);result.duration=interval*float(last-first);result.annotations.clear();
    for(auto event:source.annotations)if(event.text==converterSourceMotionMarker||(event.time>=begin-.00005f&&event.time<=end+.00005f)){event.time=event.text==converterSourceMotionMarker?0:std::clamp(event.time-begin,0.f,result.duration);result.annotations.push_back(std::move(event));}
    if(!source.referenceFrame.samples.empty()){
        auto& reference=result.referenceFrame;reference.duration=result.duration;reference.samples.resize(result.frames.size());const auto origin=converter_source_motion::reference(source.referenceFrame,begin);
        for(std::size_t frame=0;frame<result.frames.size();++frame){const auto value=converter_source_motion::reference(source.referenceFrame,begin+interval*float(frame));for(int axis=0;axis<4;++axis)reference.samples[frame][axis]=value[axis]-origin[axis];}
    }
    rotations(result);return result;
}
}
