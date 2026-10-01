#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>

namespace fc {
inline constexpr float threepeatHopSeconds=1.3f;
inline constexpr float threepeatMantleSeconds=1.8666666667f;
inline float threepeatEase(float t){t=std::clamp(t,0.f,1.f);return t*t*(3-2*t);}
struct ThreepeatPathKnot { float phase,travel,lift,out; };
inline constexpr std::array<ThreepeatPathKnot,27> threepeatLeftPath{{
    {0.0000000000f,-0.0000000000f,0.0000000000f,0.0000000000f},
    {0.0384615385f,0.0567648158f,6.3850443144f,0.0000000000f},
    {0.0769230769f,0.1148505173f,17.3130421251f,0.0000000000f},
    {0.1153846154f,0.1886311934f,25.2482726617f,0.0000000000f},
    {0.1538461538f,0.2848022855f,33.7968837158f,0.0000000000f},
    {0.1923076923f,0.3991321070f,41.3231674161f,0.0000000000f},
    {0.2307692308f,0.4990913252f,45.7759494747f,0.0000000000f},
    {0.2692307692f,0.5912101048f,47.2138236664f,0.0000000000f},
    {0.3076923077f,0.6772435367f,46.0672785845f,0.0000000000f},
    {0.3461538462f,0.7552451683f,41.2463102541f,0.0000000000f},
    {0.3846153846f,0.8301000437f,34.0188935474f,0.0000000000f},
    {0.4230769231f,0.9138180939f,23.7257610848f,0.0000000000f},
    {0.4615384615f,1.0053833699f,10.7447471824f,0.0000000000f},
    {0.5000000000f,1.0682925909f,9.7650110943f,0.0000000000f},
    {0.5384615385f,1.1121551545f,11.6659435736f,0.0000000000f},
    {0.5769230769f,1.1315783720f,13.8268132461f,0.0000000000f},
    {0.6153846154f,1.1325224135f,15.0058026601f,0.0000000000f},
    {0.6538461538f,1.1322004444f,14.1449475563f,0.0000000000f},
    {0.6923076923f,1.0910365364f,11.9736484186f,0.0000000000f},
    {0.7307692308f,1.0528788717f,9.2579233279f,0.0000000000f},
    {0.7692307692f,1.0420885785f,6.9420053180f,0.0000000000f},
    {0.8076923077f,1.0298639527f,3.1287461466f,0.0000000000f},
    {0.8461538462f,1.0179535915f,-1.1907996446f,0.0000000000f},
    {0.8846153846f,1.0095578149f,-4.4792461031f,0.0000000000f},
    {0.9230769231f,1.0046686250f,-6.2023162664f,0.0000000000f},
    {0.9615384615f,1.0000000000f,0.0000000000f,0.0000000000f},
    {1.0000000000f,1.0000000000f,0.0000000000f,0.0000000000f},
}};
inline constexpr std::array<ThreepeatPathKnot,27> threepeatRightPath{{
    {0.0000000000f,0.0000000000f,0.0000000000f,0.0000000000f},
    {0.0384615385f,0.0282321707f,5.0090989181f,0.0000000000f},
    {0.0769230769f,0.0952727103f,15.6790433577f,0.0000000000f},
    {0.1153846154f,0.1886280794f,25.2730003074f,0.0000000000f},
    {0.1538461538f,0.2847992378f,33.8342186379f,0.0000000000f},
    {0.1923076923f,0.3991290693f,41.3754900726f,0.0000000000f},
    {0.2307692308f,0.4990882802f,45.8413759912f,0.0000000000f},
    {0.2692307692f,0.5912070530f,47.2913338531f,0.0000000000f},
    {0.3076923077f,0.6772405520f,46.1560670807f,0.0000000000f},
    {0.3461538462f,0.7552421779f,41.3453165157f,0.0000000000f},
    {0.3846153846f,0.8300971946f,34.1277127082f,0.0000000000f},
    {0.4230769231f,0.9138150919f,23.8455511835f,0.0000000000f},
    {0.4615384615f,1.0053803612f,10.8765407619f,0.0000000000f},
    {0.5000000000f,1.0682895776f,9.9050553880f,0.0000000000f},
    {0.5384615385f,1.1121521379f,11.8117340865f,0.0000000000f},
    {0.5769230769f,1.1315753539f,13.9751538033f,0.0000000000f},
    {0.6153846154f,1.1325193220f,15.1542669640f,0.0000000000f},
    {0.6538461538f,1.1321973529f,14.2933696527f,0.5933487989f},
    {0.6923076923f,1.0910335948f,12.1166742726f,1.4433012843f},
    {0.7307692308f,1.0528758594f,9.3959431907f,1.8954084595f},
    {0.7692307692f,1.0420855670f,7.0786106590f,2.0016562432f},
    {0.8076923077f,1.0298609421f,3.2637451215f,1.9017441577f},
    {0.8461538462f,1.0190555345f,1.1109308720f,1.5521664429f},
    {0.8846153846f,1.0087040973f,0.2227482796f,0.9918477372f},
    {0.9230769231f,1.0009693558f,-0.3592441067f,0.5293257675f},
    {0.9615384615f,1.0000000000f,0.0000000000f,0.0000000000f},
    {1.0000000000f,1.0000000000f,0.0000000000f,0.0000000000f},
}};
inline float threepeatPathValue(bool left,float phase,int slot) {
    const auto& rows=left?threepeatLeftPath:threepeatRightPath;
    phase=std::clamp(phase,0.f,1.f);
    const int i=std::min(int(phase*float(rows.size()-1)),int(rows.size())-2);
    const auto a=rows[i],b=rows[i+1],before=rows[std::max(0,i-1)],after=rows[std::min(int(rows.size())-1,i+2)];
    const auto value=[&](ThreepeatPathKnot k){return slot==0?k.travel:slot==1?k.lift:k.out;};
    const float width=b.phase-a.phase,t=(phase-a.phase)/width;
    float da=(value(b)-value(before))/(b.phase-before.phase),db=(value(after)-value(a))/(after.phase-a.phase);
    if(slot==2){const float slope=(value(b)-value(a))/width;
        if(slope>-1e-8f&&slope<1e-8f)da=db=0;
        else {const float bound=slope>0?3*slope:-3*slope;
            da=da*slope<=0?0:std::clamp(da,-bound,bound);db=db*slope<=0?0:std::clamp(db,-bound,bound);}}
    return (2*t*t*t-3*t*t+1)*value(a)+(t*t*t-2*t*t+t)*width*da+(-2*t*t*t+3*t*t)*value(b)+(t*t*t-t*t)*width*db;
}
inline float threepeatHopTravel(bool left,float phase){return threepeatPathValue(left,phase,0);}
inline float threepeatHopLift(bool left,float phase){return threepeatPathValue(left,phase,1);}
inline float threepeatHopOut(bool left,float phase){return threepeatPathValue(left,phase,2);}
inline float threepeatSourceWeight(bool left,int hand,float phase){
    const float at=phase*threepeatHopSeconds;
    const float begin=left?(hand==0?.03f:.13f):(hand==0?.12f:.025f);
    const float end=left?(hand==0?.10f:.23f):(hand==0?.24f:.11f);
    return 1-threepeatEase((at-begin)/(end-begin));
}
inline float threepeatTargetWeight(bool left,int hand,float phase){
    const float at=phase*threepeatHopSeconds;
    const float begin=left?(hand==0?.50f:.61f):(hand==0?.64f:.53f);
    const float end=left?(hand==0?.66f:.71f):(hand==0?.73f:.69f);
    return threepeatEase((at-begin)/(end-begin));
}
inline float threepeatMantleWeight(int hand,float phase){
    const float begin=hand==0?.84f:.93f,end=hand==0?.97f:1.05f;
    const float at=phase*threepeatMantleSeconds;
    const float plant=1-threepeatEase((at-.30f)/.09f)+threepeatEase((at-.62f)/.08f);
    return plant*(1-threepeatEase((at-begin)/(end-begin)));
}
struct ThreepeatProfile {
    std::array<std::array<ThreepeatPathKnot,64>,2> paths{};
    std::array<std::uint32_t,2> pathCounts{};
    std::array<std::array<std::array<float,2>,2>,2> source{},target{};
    std::array<std::array<float,2>,2> mantleRelease{};
    std::array<float,2> mantleUnplant{},mantleReplant{};
    float replantSamplePhase{};
    std::array<std::array<float,2>,2> rise{};
};
inline ThreepeatProfile defaultThreepeatProfile() {
    ThreepeatProfile p;
    std::copy(threepeatLeftPath.begin(),threepeatLeftPath.end(),p.paths[0].begin());
    std::copy(threepeatRightPath.begin(),threepeatRightPath.end(),p.paths[1].begin());
    p.pathCounts={27,27};
    p.source={{{{{.03f/threepeatHopSeconds,.10f/threepeatHopSeconds},{.13f/threepeatHopSeconds,.23f/threepeatHopSeconds}}},
        {{{.12f/threepeatHopSeconds,.24f/threepeatHopSeconds},{.025f/threepeatHopSeconds,.11f/threepeatHopSeconds}}}}};
    p.target={{{{{.50f/threepeatHopSeconds,.66f/threepeatHopSeconds},{.61f/threepeatHopSeconds,.71f/threepeatHopSeconds}}},
        {{{.64f/threepeatHopSeconds,.73f/threepeatHopSeconds},{.53f/threepeatHopSeconds,.69f/threepeatHopSeconds}}}}};
    p.mantleRelease={{{.84f/threepeatMantleSeconds,.97f/threepeatMantleSeconds},{.93f/threepeatMantleSeconds,1.05f/threepeatMantleSeconds}}};
    p.mantleUnplant={.30f/threepeatMantleSeconds,.39f/threepeatMantleSeconds};
    p.mantleReplant={.62f/threepeatMantleSeconds,.70f/threepeatMantleSeconds};
    p.replantSamplePhase=.70f/threepeatMantleSeconds;
    p.rise={{{.23f/threepeatHopSeconds,.50f/threepeatHopSeconds},{.24f/threepeatHopSeconds,.53f/threepeatHopSeconds}}};
    return p;
}
inline bool validThreepeatProfile(const ThreepeatProfile& p) {
    const auto bounded=[](float value,float low,float high){return std::isfinite(value)&&value>=low&&value<=high;};
    const auto window=[&](const std::array<float,2>& values){return bounded(values[0],0,1)&&bounded(values[1],0,1)&&values[1]-values[0]>=.001f;};
    for(int side=0;side<2;++side) {
        const auto count=p.pathCounts[side];if(count<2||count>64)return false;
        for(std::size_t i=0;i<count;++i) {
            const auto& k=p.paths[side][i];
            if(!bounded(k.phase,0,1)||!bounded(k.travel,-.25f,1.5f)||!bounded(k.lift,-32,96)||!bounded(k.out,0,48)||
                (i&&k.phase-p.paths[side][i-1].phase<.001f))return false;
        }
        const auto a=p.paths[side][0],b=p.paths[side][count-1];
        if(a.phase!=0||b.phase!=1||std::abs(a.travel)>.0001f||std::abs(b.travel-1)>.0001f||
            std::abs(a.lift)>.0001f||std::abs(b.lift)>.0001f||std::abs(a.out)>.0001f||std::abs(b.out)>.0001f)return false;
        for(int hand=0;hand<2;++hand)if(!window(p.source[side][hand])||!window(p.target[side][hand])||
            p.source[side][hand][1]>p.target[side][hand][0])return false;
        if(!window(p.rise[side])||p.rise[side][0]<std::max(p.source[side][0][1],p.source[side][1][1])||
            p.rise[side][1]>std::min(p.target[side][0][0],p.target[side][1][0]))return false;
    }
    if(!window(p.mantleUnplant)||!window(p.mantleReplant)||p.mantleUnplant[1]>p.mantleReplant[0]||
        !bounded(p.replantSamplePhase,p.mantleReplant[0],p.mantleReplant[1]))return false;
    for(int hand=0;hand<2;++hand)if(!window(p.mantleRelease[hand])||p.mantleReplant[1]>p.mantleRelease[hand][0])return false;
    return true;
}
inline float threepeatPathValue(bool left,float phase,int slot,const ThreepeatProfile& profile) {
    const auto side=left?0:1,count=std::clamp(int(profile.pathCounts[side]),2,64);
    const auto& rows=profile.paths[side];phase=std::clamp(phase,0.f,1.f);
    int i=0;while(i<count-2&&phase>=rows[i+1].phase)++i;
    const auto a=rows[i],b=rows[i+1],before=rows[std::max(0,i-1)],after=rows[std::min(count-1,i+2)];
    const auto value=[&](ThreepeatPathKnot k){return slot==0?k.travel:slot==1?k.lift:k.out;};
    const float width=b.phase-a.phase,t=(phase-a.phase)/width;
    float da=(value(b)-value(before))/(b.phase-before.phase),db=(value(after)-value(a))/(after.phase-a.phase);
    if(slot==2){const float slope=(value(b)-value(a))/width;
        if(slope>-1e-8f&&slope<1e-8f)da=db=0;
        else {const float bound=slope>0?3*slope:-3*slope;
            da=da*slope<=0?0:std::clamp(da,-bound,bound);db=db*slope<=0?0:std::clamp(db,-bound,bound);}}
    return (2*t*t*t-3*t*t+1)*value(a)+(t*t*t-2*t*t+t)*width*da+(-2*t*t*t+3*t*t)*value(b)+(t*t*t-t*t)*width*db;
}
inline float threepeatHopTravel(bool left,float phase,const ThreepeatProfile& p){return threepeatPathValue(left,phase,0,p);}
inline float threepeatHopLift(bool left,float phase,const ThreepeatProfile& p){return threepeatPathValue(left,phase,1,p);}
inline float threepeatHopOut(bool left,float phase,const ThreepeatProfile& p){return threepeatPathValue(left,phase,2,p);}
inline float threepeatWindow(float phase,const std::array<float,2>& window) {
    return threepeatEase((phase-window[0])/(window[1]-window[0]));
}
inline float threepeatSourceWeight(bool left,int hand,float phase,const ThreepeatProfile& p) {
    return 1-threepeatWindow(phase,p.source[left?0:1][hand]);
}
inline float threepeatTargetWeight(bool left,int hand,float phase,const ThreepeatProfile& p) {
    return threepeatWindow(phase,p.target[left?0:1][hand]);
}
inline float threepeatMantleWeight(int hand,float phase,const ThreepeatProfile& p) {
    return (1-threepeatWindow(phase,p.mantleUnplant)+threepeatWindow(phase,p.mantleReplant))*
        (1-threepeatWindow(phase,p.mantleRelease[hand]));
}
}
