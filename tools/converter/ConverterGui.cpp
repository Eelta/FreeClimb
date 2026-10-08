#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <windowsx.h>
#include <commctrl.h>
#include <commdlg.h>
#include <shellapi.h>
#include "ConverterGuiContract.h"
#include "ConverterEditor.h"
#include "ConverterSourceMotion.h"
#include "ConverterPreview.h"
#include "ConverterActionPreview.h"
#include "ConverterWallRunGroup.h"
#include "ConverterActionAuthoring.h"
#include "ConverterTheme.h"
#include <functional>
#include <map>
#include <memory>
#include <thread>

namespace {
using Json=nlohmann::json;
namespace view=fc::converter::visual;
namespace theme=fc::converter::theme;
constexpr UINT completed=WM_APP+1;
enum Id {inputId=101,slotId,packId,languageId,tabId,openId=201,baseId,packBrowseId,exportId,playId,previousId,nextId,
    scrubId,resetId,inHereId,outHereId,boneId,clearBoneId,contactId,contactEnableId,autoId,wallId,ledgeId,jointsId,fitId,previewId,timelineId,inPlaceXId,inPlaceYId,inPlaceZId,geometryId,knotId,simplifyId,stageId,advancedId,loopBeginId,loopEndId,loopBeginHereId,loopEndHereId,stageEditingId,sourceComparisonId};
constexpr std::array<std::wstring_view,25> visibleActions{
    L"hang",L"up",L"down",L"left",L"right",L"reach",L"hopLeft",L"hopRight",L"hopUp",L"drop",
    L"jumpCatch",L"dropBack",L"ledgeCatch",L"runUp",L"runLeft",L"runRight",L"runDiagonalLeft",L"runDiagonalRight",
    L"kickUp",L"kickLeft",L"kickRight",L"backFlipOut",L"contextHopLeft",L"contextHopRight",L"contextMantle"
};
struct SliderDefinition {int page;std::wstring_view en,zh;float minimum,maximum,initial;};
constexpr std::array<SliderDefinition,33> definitions{{
    {0,L"Trim start",L"裁剪起点",0,1,0},{0,L"Trim end",L"裁剪终点",0,1,1},{0,L"Clip speed",L"片段速度",.5f,2,1},
    {0,L"Facing",L"朝向角度",-180,180,0},{0,L"Root X",L"根位移 X",-100,100,0},{0,L"Root Y",L"根位移 Y",-100,100,0},
    {0,L"Root Z",L"根位移 Z",-100,100,0},{0,L"Body X",L"身体偏移 X",-30,30,0},{0,L"Body Y",L"身体偏移 Y",-30,30,0},{0,L"Body Z",L"身体偏移 Z",-30,30,0},
    {1,L"Local rotation X",L"局部旋转 X",-45,45,0},{1,L"Local rotation Y",L"局部旋转 Y",-45,45,0},{1,L"Local rotation Z",L"局部旋转 Z",-45,45,0},
    {1,L"Window start",L"窗口起点",0,1,0},{1,L"Window end",L"窗口终点",0,1,1},{1,L"Blend width",L"混合宽度",.01f,.3f,.1f},
    {2,L"Window start",L"窗口起点",0,1,0},{2,L"Window end",L"窗口终点",0,1,1},{2,L"Contact blend",L"接触混合",.01f,.3f,.05f},{2,L"Contact weight",L"接触权重",0,1,1},
    {3,L"Wall position",L"墙面位置",-100,200,32},{3,L"Ledge height",L"边沿高度",0,300,128},{3,L"View yaw",L"观察角度",-180,180,-37},{3,L"View pitch",L"观察俯仰",-65,65,9},
    {4,L"Stride",L"步幅",0,500,0},{4,L"Height",L"高度",-500,500,0},{4,L"Travel X",L"轨迹位移 X",-500,500,0},{4,L"Travel Y",L"轨迹位移 Y",-500,500,0},{4,L"Travel Z",L"轨迹位移 Z",-500,500,0},
    {4,L"Knot phase",L"节点阶段",0,1,0},{4,L"Path progress",L"路径进度",-.25f,1.5f,0},{4,L"Lift",L"抬升",-32,96,0},{4,L"Out from wall",L"离墙偏移",0,48,0}
}};
constexpr std::array<std::wstring_view,fc::converter::slots.size()> slotEnglish{L"Wall idle",L"Climb up",L"Climb down",L"Climb left",L"Climb right",L"Reach",L"Hop left",L"Hop right",L"Hop up",
    L"Release in place",L"Catch after jump",L"Push away",L"Catch a ledge",L"Wall run up",L"Wall run left",L"Wall run right",L"Diagonal run left",L"Diagonal run right",
    L"Wall run launch",L"Wall run catch",L"Kick up",L"Kick left",L"Kick right",L"Run launch left",L"Run launch right",L"Side brace",L"Backflip away",
    L"Contextual preparation",L"Contextual hop left",L"Contextual hop right",L"Top-out"};
constexpr std::array<std::wstring_view,fc::converter::slots.size()> slotChinese{L"墙面待机",L"向上攀爬",L"向下攀爬",L"向左攀爬",L"向右攀爬",L"伸手",L"向左跃抓",L"向右跃抓",L"向上跃抓",
    L"原地松手",L"跳起抓墙",L"向外蹬离",L"抓住边沿",L"向上墙跑",L"向左墙跑",L"向右墙跑",L"左斜向墙跑",L"右斜向墙跑",L"墙跑起跳",L"墙跑落抓",
    L"向上蹬跳",L"向左蹬跳",L"向右蹬跳",L"墙跑左向起跳",L"墙跑右向起跳",L"侧向支撑",L"后空翻离墙",L"情境抓握准备",L"情境左跃抓",L"情境右跃抓",L"登顶"};
std::wstring fromUtf8(std::string_view text) {
    if(text.empty())return {};const int count=MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,text.data(),int(text.size()),nullptr,0);
    if(!count)return L"Invalid UTF-8 text.";std::wstring result(count,L'\0');MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,text.data(),int(text.size()),result.data(),count);return result;
}
std::string toUtf8(std::wstring_view text) {
    if(text.empty())return {};const int count=WideCharToMultiByte(CP_UTF8,WC_ERR_INVALID_CHARS,text.data(),int(text.size()),nullptr,0,nullptr,nullptr);
    if(!count)throw std::runtime_error("Invalid Unicode text");std::string result(count,'\0');WideCharToMultiByte(CP_UTF8,WC_ERR_INVALID_CHARS,text.data(),int(text.size()),result.data(),count,nullptr,nullptr);return result;
}
std::wstring fieldText(HWND item){const int length=GetWindowTextLengthW(item);std::wstring value(std::size_t(length)+1,L'\0');GetWindowTextW(item,value.data(),length+1);value.resize(length);return value;}
bool regularFile(const std::filesystem::path& path){std::error_code error;return std::filesystem::is_regular_file(path,error);}
bool samePath(const std::filesystem::path& left,const std::filesystem::path& right){
    std::error_code error;auto a=std::filesystem::weakly_canonical(left,error);if(error)return false;auto b=std::filesystem::weakly_canonical(right,error);if(error)return false;
    a.make_preferred();b.make_preferred();return CompareStringOrdinal(a.c_str(),-1,b.c_str(),-1,TRUE)==CSTR_EQUAL;
}
std::wstring number(float value,int decimals=2){wchar_t text[64]{};swprintf_s(text,L"%.*f",decimals,double(value));return text;}
std::vector<std::array<float,4>> contactFrames(const fc::ConverterEditedAnimation& animation) {
    std::vector<std::array<float,4>> result;const auto count=animation.clip.frames.size();result.reserve(count);
    for(std::size_t i=0;i<count;++i)result.push_back(fc::sampleConverterContacts(animation,animation.clip.duration*float(i)/float(std::max<std::size_t>(1,count-1))));return result;
}
struct WindowRole {std::string_view role,key;int hand,channel;std::wstring_view en,zh;};
constexpr std::array<WindowRole,5> mantleRoles{{
    {"releaseLeft","releaseHands",0,0,L"Left hand release",L"左手放开"},{"releaseRight","releaseHands",1,1,L"Right hand release",L"右手放开"},
    {"unplant","unplant",-1,0,L"Lift from wall",L"离开原抓点"},{"replant","replant",-1,0,L"Plant on ledge",L"撑住顶部"},{"replantSample","replantSamplePhase",-1,0,L"Ledge pose sample",L"顶部姿态采样"}
}};
constexpr std::array<WindowRole,5> hopRoles{{
    {"sourceLeft","sourceHands",0,0,L"Left hand takeoff",L"左手松开起跳"},{"sourceRight","sourceHands",1,1,L"Right hand takeoff",L"右手松开起跳"},
    {"targetLeft","targetHands",0,0,L"Left hand catch",L"左手落抓"},{"targetRight","targetHands",1,1,L"Right hand catch",L"右手落抓"},{"vertical","verticalBlend",-1,0,L"Vertical path blend",L"竖直轨迹混合"}
}};
enum class Work {load,edit,exportZip};
struct Completion {Work work{};std::uint64_t revision{};std::shared_ptr<const fc::ConverterEditorDocument> document;std::shared_ptr<const fc::ConverterEditedAnimation> animation;std::shared_ptr<const fc::ConverterActionPreview> runtimePreview,fullPreview;Json report;std::string failure,previewWarning;bool imported{};std::string direction;std::optional<fc::ConverterEditOptions> initialOptions;std::shared_ptr<fc::ConverterWallRunSequence> authoredSequence;std::shared_ptr<const fc::ConverterWallRunDocument> wholeDocument;bool wholeAction{};};
struct Slider {HWND label{},track{},value{};};
struct StageEdit {
    std::shared_ptr<const fc::ConverterEditorDocument> document;
    std::shared_ptr<const fc::ConverterEditedAnimation> animation;
    fc::ConverterEditOptions options;
    std::string direction;
    std::array<float,definitions.size()> values{};
    int bone{},contact{},knot{};
    float time{};
    bool dirty{},ready{};
    view::Camera camera;
    view::Reference reference;
    std::shared_ptr<const fc::ConverterWallRunDocument> wholeDocument;
    std::shared_ptr<const fc::ConverterWallRunSequence> sequence;
    std::wstring loopBegin,loopEnd;
};
std::shared_ptr<const fc::ConverterActionPreview> buildPreview(const fc::ConverterEditorDocument& document,const fc::ConverterEditedAnimation& animation,const std::optional<std::vector<StageEdit>>& stages,std::string direction={},std::string* warning=nullptr) {
    if(document.authored)return {};
    try {if(!stages)return std::make_shared<fc::ConverterActionPreview>();std::vector<fc::ConverterEditorExport> overlays;
    for(const auto& stage:*stages)overlays.push_back({stage.document.get(),stage.animation.get(),stage.direction});fc::ConverterActionPreviewOptions options;if(!direction.empty()){options.kind=fc::ConverterActionPreviewKind::wallRun;options.direction=std::move(direction);options.cycles=1;}return std::make_shared<fc::ConverterActionPreview>(fc::buildConverterActionPreview(document,animation,options,overlays));
    }catch(const std::exception& failure){if(warning){if(!warning->empty())*warning+="; ";*warning+=failure.what();}return std::make_shared<fc::ConverterActionPreview>();}
}
void wholePreview(Completion& result) {
    if(!result.authoredSequence)return;const auto& sequence=*result.authoredSequence;std::vector<StageEdit> stages;
    for(const auto& item:sequence.stages){StageEdit saved{std::make_shared<fc::ConverterEditorDocument>(item.document),std::make_shared<fc::ConverterEditedAnimation>(item.animation),item.options};saved.direction=sequence.primary;stages.push_back(std::move(saved));}
    const auto selected=std::find_if(stages.begin(),stages.end(),[&](const auto& item){return item.document->slot==sequence.primary;});
    if(selected==stages.end())throw std::runtime_error("Complete action has no loop section");
    result.fullPreview=buildPreview(*selected->document,*selected->animation,stages,sequence.primary,&result.previewWarning);
}
LRESULT CALLBACK panelProcedure(HWND,UINT,WPARAM,LPARAM);
LRESULT CALLBACK panelChildProcedure(HWND,UINT,WPARAM,LPARAM,UINT_PTR,DWORD_PTR);
LRESULT CALLBACK themedProcedure(HWND,UINT,WPARAM,LPARAM,UINT_PTR,DWORD_PTR);
struct App {
    HWND window{},input{},slot{},pack{},language{},open{},base{},packBrowse{},exportZip{},tabs{},panel{},preview{},timeline{},play{},previous{},next{},scrub{},clock{},status{},help{},
        reset{},inHere{},outHere{},bone{},clearBone{},contact{},contactEnable{},autoCalibration{},wall{},ledge{},joints{},fit{},special{},geometryEnable{},knot{},simplify{},stage{},stageLabel{},brand{},tagline{},tooltips{},advanced{},loopBegin{},loopEnd{},loopBeginLabel{},loopEndLabel{},loopBeginHere{},loopEndHere{},splitHelp{},stageEditingToggle{},sourceComparisonToggle{};
    std::array<HWND,3> inPlace{};std::array<Slider,definitions.size()> sliders;
    HFONT font{},brandFont{},smallFont{};HBRUSH canvasBrush=CreateSolidBrush(theme::canvas),panelBrush=CreateSolidBrush(theme::panel),fieldBrush=CreateSolidBrush(theme::field);int dpi{96},page{},selectedBone{},selectedContact{},selectedKnot{};bool chinese{},busy{},setting{},playing{},dragging{},editedAfterLoad{},readyBeforeLoad{};Work currentWork{Work::load};std::thread worker;
    std::array<int,5> panelScroll{};int panelContentHeight{},panelViewportHeight{},wheelRemainder{};
    bool advancedMotion{},stageEditing{},sourceComparison{},previewGroupReady{true},pendingEdit{};std::unique_ptr<view::Canvas> previewCanvas,timelineCanvas;
    std::filesystem::path executable,lastOutput,lastOutputInput;std::string lastOutputTarget;std::shared_ptr<const fc::ConverterEditorDocument> document;std::shared_ptr<const fc::ConverterEditedAnimation> animation;fc::ConverterEditOptions options,displayOptions;
    std::shared_ptr<const fc::ConverterActionPreview> runtimePreview,fullPreview;bool preferAdapted{true},preferSequence{};
    std::shared_ptr<const fc::ConverterActionPreview> boundedPreview;view::WallBounds previewWallBounds;
    std::map<std::string,StageEdit> stages;std::string documentDirection="runUp";std::shared_ptr<const fc::ConverterWallRunDocument> wholeDocument;std::shared_ptr<const fc::ConverterWallRunSequence> wholeSequence;
    view::Camera camera;view::Reference reference;float time{};ULONGLONG lastTick{};std::uint64_t revision{},displayRevision{};std::vector<std::array<float,4>> contacts;
    std::wstring statusEnglish,statusChinese,tooltipText;POINT dragStart{};int timelineDrag{};
    ~App(){if(worker.joinable())worker.join();for(const auto handle:{font,brandFont,smallFont})if(handle)DeleteObject(handle);for(const auto handle:{canvasBrush,panelBrush,fieldBrush})if(handle)DeleteObject(handle);}
    std::wstring_view text(std::wstring_view en,std::wstring_view zh)const{return chinese?zh:en;}
    void set(HWND item,std::wstring_view value){if(fieldText(item)!=value){const std::wstring owned(value);SetWindowTextW(item,owned.c_str());}}
    HWND control(const wchar_t* type,DWORD style,int id=0,HWND parent=nullptr) {
        if(std::wstring_view(type)==L"COMBOBOX")style|=CBS_OWNERDRAWFIXED|CBS_HASSTRINGS;
        if(std::wstring_view(type)==WC_TABCONTROLW)style|=TCS_OWNERDRAWFIXED;
        if(std::wstring_view(type)==L"BUTTON"&&((style&BS_TYPEMASK)==BS_PUSHBUTTON||(style&BS_TYPEMASK)==BS_DEFPUSHBUTTON))style=(style&~BS_TYPEMASK)|BS_OWNERDRAW;
        const auto item=CreateWindowExW(0,type,L"",WS_CHILD|WS_VISIBLE|WS_CLIPSIBLINGS|style,0,0,1,1,parent?parent:window,reinterpret_cast<HMENU>(INT_PTR(id)),GetModuleHandleW(nullptr),nullptr);
        if(std::wstring_view(type)==L"BUTTON"&&(style&BS_TYPEMASK)==BS_OWNERDRAW)SetPropW(item,L"FreeClimbOwnerDraw",reinterpret_cast<HANDLE>(1));
        SetWindowSubclass(item,themedProcedure,2,reinterpret_cast<DWORD_PTR>(this));
        if(parent==panel&&parent)SetWindowSubclass(item,panelChildProcedure,1,reinterpret_cast<DWORD_PTR>(this));
        SendMessageW(item,WM_SETFONT,reinterpret_cast<WPARAM>(font),TRUE);if(std::wstring_view(type)==L"EDIT")SendMessageW(item,EM_SETLIMITTEXT,32760,0);return item;
    }
    void tooltip(HWND item) {
        TOOLINFOW info{sizeof(info)};info.uFlags=TTF_IDISHWND|TTF_SUBCLASS;info.hwnd=window;info.uId=reinterpret_cast<UINT_PTR>(item);info.lpszText=LPSTR_TEXTCALLBACKW;SendMessageW(tooltips,TTM_ADDTOOLW,0,reinterpret_cast<LPARAM>(&info));
    }
    HWND panelControl(const wchar_t* type,DWORD style,int id=0){return control(type,style,id,panel);}
    void message(std::wstring_view en,std::wstring_view zh){statusEnglish=en;statusChinese=zh;set(status,text(en,zh));}
    void error(std::string_view failure){const auto value=fromUtf8(failure);message(L"Unable to apply: "+value,L"无法应用："+value);}
    float get(unsigned index)const{const auto& d=definitions[index];return d.minimum+(d.maximum-d.minimum)*float(SendMessageW(sliders[index].track,TBM_GETPOS,0,0))/1000.f;}
    void sliderValue(unsigned index) {
        const float value=get(index);std::wstring output;
        if(index<2||(index>=13&&index<=19)||index==29||index==30)output=number(value*100,1)+L"%";else if(index==2)output=number(value)+L"×";
        else if(index==3||(index>=10&&index<=12)||(index>=22&&index<=23))output=number(value,1)+L"°";else output=number(value,1);set(sliders[index].value,output);
    }
    void put(unsigned index,float value){const auto& d=definitions[index];const auto position=std::lround(std::clamp((value-d.minimum)/(d.maximum-d.minimum),0.f,1.f)*1000);if(SendMessageW(sliders[index].track,TBM_GETPOS,0,0)!=position)SendMessageW(sliders[index].track,TBM_SETPOS,TRUE,position);sliderValue(index);}
    std::span<const WindowRole> windowRoles()const {
        if(!document||document->authored||!document->contextGroup.empty())return {};if(document->slot=="contextMantle")return mantleRoles;if(document->slot=="contextHopLeft"||document->slot=="contextHopRight")return hopRoles;return {};
    }
    std::array<float,2> currentWindow()const {
        const auto roles=windowRoles();if(roles.empty()||selectedContact<0||std::size_t(selectedContact)>=roles.size()||!animation)return {0,1};
        const auto& role=roles[std::size_t(selectedContact)];const auto key=std::string(role.key);if(!animation->config.contains(key))return {0,1};const auto& value=animation->config.at(key);
        if(value.is_number())return {value.get<float>(),value.get<float>()};const auto& phaseWindow=role.hand>=0?value.at(std::size_t(role.hand)):value;return {phaseWindow.at(0).get<float>(),phaseWindow.at(1).get<float>()};
    }
    void markerSummary() {
        if(!animation){set(special,L"");return;}std::wstring result;
        for(const auto& role:windowRoles()) {
            const auto key=std::string(role.key);if(!animation->config.contains(key))continue;const auto& value=animation->config.at(key);
            const auto& phase=value.is_number()?value:(role.hand>=0?value.at(std::size_t(role.hand)):value);const float begin=value.is_number()?phase.get<float>():phase.at(0).get<float>();
            const float end=value.is_number()?begin:phase.at(1).get<float>();result+=std::wstring(chinese?role.zh:role.en)+L"\r\n"+number(begin*animation->clip.duration)+L" – "+number(end*animation->clip.duration)+L" s\r\n\r\n";
        }set(special,result);
    }
    float previewDuration()const {const auto shown=activePreview();return shown?shown->seconds:animation?animation->clip.duration:0;}
    const fc::ConverterActionPreview* activePreview()const {
        if(document&&document->authored)return nullptr;
        const auto& selected=preferSequence?fullPreview:runtimePreview;return preferAdapted&&selected&&selected->available()&&previewGroupReady?selected.get():nullptr;
    }
    bool sequencePreview()const{return preferSequence&&activePreview();}
    std::optional<fc::ConverterWallRunCuts> readCuts()const {
        try{auto parse=[](HWND item){const auto text=fieldText(item);std::size_t used{};const auto value=std::stof(text,&used);if(used!=text.size()||!std::isfinite(value))throw std::runtime_error("Invalid loop boundary");return value;};fc::ConverterWallRunCuts cuts{parse(loopBegin),parse(loopEnd)};if(wholeDocument&&!wholeDocument->stages.empty()){if(fieldText(loopBegin)==number(wholeDocument->cuts.loopBegin,4))cuts.loopBegin=wholeDocument->cuts.loopBegin;if(fieldText(loopEnd)==number(wholeDocument->cuts.loopEnd,4))cuts.loopEnd=wholeDocument->cuts.loopEnd;}return cuts;}catch(...){return {};}
    }
    float sourceSeconds(const fc::ConverterActionPreviewFrame& sample)const {
        if(sample.motion==fc::Motion::hang){const auto& segments=activePreview()->segments;const auto ending=std::find_if(segments.begin(),segments.end(),[](const auto& value){return value.stage==fc::ConverterActionPreviewStage::catching;});return ending!=segments.end()&&sample.seconds>=ending->begin?wholeSequence->sourceRanges[2][1]:wholeSequence->sourceRanges[0][0];}
        unsigned section=0;if(sample.motion==fc::Motion::runCatch)section=2;
        else if(sample.motion==fc::Motion::runUp||sample.motion==fc::Motion::runLeft||sample.motion==fc::Motion::runRight||sample.motion==fc::Motion::runDiagonalLeft||sample.motion==fc::Motion::runDiagonalRight)section=1;
        const auto range=wholeSequence->sourceRanges[section];return std::lerp(range[0],range[1],std::clamp(sample.phase,0.f,1.f));
    }
    float sourceTime(float seconds)const {
        if(wholeDocument&&wholeSequence&&sequencePreview())return sourceSeconds(fc::sampleConverterActionPreview(*activePreview(),seconds));
        if(mantlePreview())return displayOptions.trimIn+fc::sampleConverterActionPreview(*activePreview(),seconds).phase*animation->clip.duration*displayOptions.speed;
        return displayOptions.trimIn+seconds*displayOptions.speed;
    }
    bool mantlePreview()const {const auto shown=activePreview();return shown&&shown->ledgeHeight.has_value();}
    float sourceTime()const{return sourceTime(time);}
    float sourcePhase(float previewPhase)const {
        if(!(wholeDocument&&wholeSequence&&sequencePreview())&&!mantlePreview())return previewPhase;
        const float end=displayOptions.trimOut<0?document->clip.duration:displayOptions.trimOut;return std::clamp((sourceTime(previewPhase*previewDuration())-displayOptions.trimIn)/std::max(.001f,end-displayOptions.trimIn),0.f,1.f);
    }
    float previewPhase(float sourcePhase)const {
        if(!(wholeDocument&&wholeSequence&&sequencePreview())&&!mantlePreview())return sourcePhase;
        if(mantlePreview()) {
            const auto& shown=*activePreview();const float target=std::clamp(sourcePhase,0.f,1.f);
            for(std::size_t i=1;i<shown.frames.size();++i){const auto& a=shown.frames[i-1];const auto& b=shown.frames[i];if(b.phase>=target){const float mix=b.phase>a.phase?std::clamp((target-a.phase)/(b.phase-a.phase),0.f,1.f):0;return std::lerp(a.seconds,b.seconds,mix)/shown.seconds;}}
            return 1;
        }
        const float end=displayOptions.trimOut<0?document->clip.duration:displayOptions.trimOut,target=std::lerp(displayOptions.trimIn,end,sourcePhase);float closest=std::numeric_limits<float>::max(),seconds=0;
        for(const auto& sample:activePreview()->frames){const float error=std::abs(sourceSeconds(sample)-target);if(error<closest){closest=error;seconds=sample.seconds;}}
        return seconds/std::max(.001f,previewDuration());
    }
    std::string chosenDirection()const {const auto action=selectedName(slot);return fc::converter::isWallRunPrimary(action)?action:documentDirection;}
    void previewChoice() {preferSequence=chosenPrimary()=="wallRun"&&!stageEditing;preferAdapted=!sourceComparison;SendMessageW(sourceComparisonToggle,BM_SETCHECK,sourceComparison?BST_CHECKED:BST_UNCHECKED,0);SendMessageW(stageEditingToggle,BM_SETCHECK,stageEditing?BST_CHECKED:BST_UNCHECKED,0);}
    std::wstring stageCaption(const fc::ConverterActionPreviewSegment& segment)const {
        switch(segment.stage){case fc::ConverterActionPreviewStage::launch:return std::wstring(text(L"Start",L"起步"));case fc::ConverterActionPreviewStage::loop:return std::wstring(text(L"Loop",L"循环"));case fc::ConverterActionPreviewStage::catching:return std::wstring(text(L"Finish",L"收尾"));default:return fromUtf8(segment.slot);}
    }
    static std::array<float,definitions.size()> editValues(const fc::ConverterEditorDocument& doc,const fc::ConverterEditOptions& edits) {
        std::array<float,definitions.size()> values;for(unsigned i=0;i<definitions.size();++i)values[i]=definitions[i].initial;values[0]=edits.trimIn/doc.clip.duration;values[1]=edits.trimOut<0?1:edits.trimOut/doc.clip.duration;values[2]=edits.speed;values[3]=edits.yawDegrees;values[4]=edits.rootOffset.x;values[5]=edits.rootOffset.y;values[6]=edits.rootOffset.z;values[7]=edits.comOffset.x;values[8]=edits.comOffset.y;values[9]=edits.comOffset.z;return values;
    }
    void updateClock() {
        const float duration=previewDuration();const auto frames=activePreview()?activePreview()->frames.size():animation?animation->clip.frames.size():0;
        const auto frame=frames>1?std::min(frames-1,std::size_t(std::lround(time/std::max(duration,.001f)*float(frames-1)))):0;
        std::wstring stageName;if(sequencePreview())for(const auto& segment:activePreview()->segments)if(time>=segment.begin&&time<=segment.end){stageName=stageCaption(segment)+L" · ";break;}
        set(clock,stageName+number(time)+L" / "+number(duration)+L"s"+(sequencePreview()?L"":L" · "+std::to_wstring(frame+unsigned(frames>0))+L"/"+std::to_wstring(frames)));const auto position=duration>0?std::lround(std::clamp(time/duration,0.f,1.f)*1000):0;if(SendMessageW(scrub,TBM_GETPOS,0,0)!=position)SendMessageW(scrub,TBM_SETPOS,TRUE,position);
    }
    void invalidate(){InvalidateRect(preview,nullptr,FALSE);InvalidateRect(timeline,nullptr,FALSE);updateClock();}
    bool adaptedPreview()const{return activePreview()!=nullptr;}
    bool sourceTravel()const {if(!animation||animation->clip.frames.empty()||animation->clip.frames.front().empty())return false;const auto first=animation->clip.frames.front()[0].t;return std::any_of(animation->clip.frames.begin(),animation->clip.frames.end(),[&](const auto& pose){return !pose.empty()&&(pose[0].t-first).length()>.01f;});}
    fc::Pose previewPose(float seconds)const{return activePreview()?fc::placedConverterActionPreviewPose(fc::sampleConverterActionPreview(*activePreview(),seconds)):animation?fc::sampleConverterEditor(*animation,seconds):fc::Pose{};}
    void updateReferenceBounds() {
        const auto& shown=preferSequence?fullPreview:runtimePreview;if(shown==boundedPreview)return;boundedPreview=shown;previewWallBounds={};if(!document||!shown||!shown->available())return;
        fc::Vec low{10000,10000,10000},high{-10000,-10000,-10000};
        for(const auto& frame:shown->frames)if(const auto world=view::worldPose(fc::placedConverterActionPreviewPose(frame),document->base.parents))
            for(std::size_t i=0;i<world->size();++i)if(view::bodyBone(document->base.names[i])){const auto p=world->at(i).t;low={std::min(low.x,p.x),std::min(low.y,p.y),std::min(low.z,p.z)};high={std::max(high.x,p.x),std::max(high.y,p.y),std::max(high.z,p.z)};}
        previewWallBounds=view::wallBounds(low,high);
    }
    void resetView() {
        camera={};if(document&&animation) {
            fc::Vec low{10000,10000,10000},high{-10000,-10000,-10000};
            std::vector<fc::Pose> frames;if(const auto shown=activePreview()){frames.reserve(shown->frames.size());for(const auto& frame:shown->frames)frames.push_back(fc::placedConverterActionPreviewPose(frame));}else frames=animation->clip.frames;
            for(const auto& pose:frames)if(const auto world=view::worldPose(pose,document->base.parents))
                for(std::size_t i=0;i<world->size();++i)if(view::bodyBone(document->base.names[i])){const auto p=world->at(i).t;low={std::min(low.x,p.x),std::min(low.y,p.y),std::min(low.z,p.z)};high={std::max(high.x,p.x),std::max(high.y,p.y),std::max(high.z,p.z)};}
            if(high.z>low.z){camera.center=(low+high)*.5f;camera.zoom=std::clamp(170.f/std::max({40.f,high.z-low.z,(high.x-low.x)*.8f,(high.y-low.y)*.8f}),.25f,3.f);}
        }put(22,camera.yaw*57.29578f);put(23,camera.pitch*57.29578f);invalidate();
    }
    bool boneEditable()const{return document&&selectedBone>=0&&fc::converterEditableBone(document->base,unsigned(selectedBone));}
    std::array<bool,5> geometryFields()const {
        if(!document)return {};if(document->authored)return {animation&&fc::converterEditableStride(*document,*animation),false,false,false,false};
        if(document->slot=="contextMantle")return {false,true,true,true,false};
        if(document->slot=="contextHopLeft"||document->slot=="contextHopRight")return {false,false,true,false,false};
        constexpr std::array<std::string_view,11> loops{"hang","up","down","left","right","contextHang","runUp","runLeft","runRight","runDiagonalLeft","runDiagonalRight"};
        return {std::find(loops.begin(),loops.end(),document->slot)!=loops.end(),false,false,false,false};
    }
    void routeHelp() {
        if(page!=4)return;std::wstring value;
        if(!document)value=text(L"Import an HKX or load a base slot to see its available route settings.",L"先导入 HKX 或载入基础槽位，再查看该动作可用的轨迹设置。");
        else {
            const auto fields=geometryFields();const auto route=routeKnots();
            if(document->authored){const bool sourcePath=sourceTravel();value=fields[0]?text(L"The preview retains source movement. In game, this cycle uses an editable base stride because it has no net Root travel.",L"预览保留源动作位移。此循环没有 Root 净行程，游戏中使用可调整的基础步幅。"):sourcePath?text(L"The preview shows source Root movement. In game, travel is adapted to a collision-checked destination.",L"预览显示源 Root 位移。游戏中会将行程适配到通过碰撞检查的落点。"):text(L"The preview stays in place. In game, this action uses the selected slot's collision-checked route.",L"预览保留原地动作。游戏中使用所选动作槽通过碰撞检查的路线。");}
            else if(std::none_of(fields.begin(),fields.end(),[](bool enabled){return enabled;}))value=text(L"This action has no editable movement metadata. Adjust the clip on Motion, Bones or Contacts instead.",L"此动作没有可调的移动参数。请在“动作”“骨骼”或“接触”页调整。");
            else if(route.empty())value=text(L"This action has no path knots. The four knot controls apply only to contextual left/right hops. Enable the override to edit this action's other available fields.",L"此动作没有路径节点。下面四项仅适用于情境左右跃抓，登顶和待机不使用。勾选覆盖后可调整此动作的其他可用参数。");
            else if(!options.geometry)value=text(L"Enable Override route geometry, then choose a middle knot from the list. The start and end anchors are fixed.",L"先勾选“覆盖路线几何参数”，再从下拉列表选择中间节点。起点和终点固定。");
            else if(route.size()>64)value=text(L"This route has more than 64 knots. Use Simplify route for editing to resample it to 33 knots before editing.",L"当前路线超过 64 个节点。点击“简化路线以便编辑”，重采样为 33 个节点后再调整。");
            else if(selectedKnot==0||selectedKnot==int(route.size())-1)value=text(L"The selected start/end anchor is fixed: all four values are locked. Choose a middle knot to edit the jump arc.",L"当前选中起点或终点，四项数值固定。请在下拉列表选择中间节点，调整跃抓弧线。");
            else value=text(L"Knot phase is animation time; path progress is travel toward the target. Lift and outward offset use Skyrim units. These settings change game movement, not the skeleton preview.",L"节点阶段是动作时间，路径进度是移向目标的进度。抬升和离墙偏移使用 Skyrim 单位。这些设置改变游戏路线，不移动骨架预览。");
        }
        if(fieldText(help)!=value){set(help,value);layout();}
    }
    void availability() {
        previewGroupReady=groupReady();
        updateReferenceBounds();
        EnableWindow(sourceComparisonToggle,animation&&!busy&&!pendingEdit&&revision==displayRevision);EnableWindow(stageEditingToggle,!busy&&!pendingEdit);
        const bool editable=document&&(currentWork==Work::edit||!busy);for(const auto item:{open,base,packBrowse,input,pack,slot,stage})EnableWindow(item,!busy&&!pendingEdit);
        for(const auto item:{loopBegin,loopEnd})EnableWindow(item,document&&animation&&(!busy||currentWork==Work::edit)&&selectionMatchesDocument());
        for(const auto item:{loopBeginHere,loopEndHere})EnableWindow(item,document&&animation&&!busy&&(!adaptedPreview()||wholeDocument));
        EnableWindow(exportZip,animation&&!busy&&revision==displayRevision&&selectionMatchesDocument()&&groupReady()&&(!wholeDocument||wholeSequence));
        for(unsigned i=0;i<sliders.size();++i)EnableWindow(sliders[i].track,definitions[i].page==3||editable);
        for(unsigned i=20;i<22;++i)EnableWindow(sliders[i].track,!adaptedPreview());EnableWindow(ledge,!adaptedPreview());
        for(const auto item:{reset,inHere,outHere,bone,clearBone,contact,contactEnable,autoCalibration})EnableWindow(item,editable);EnableWindow(inHere,editable&&(!sequencePreview()||wholeDocument));EnableWindow(outHere,editable&&(!sequencePreview()||wholeDocument));
        for(const auto item:inPlace)EnableWindow(item,editable);for(const auto item:{play,previous,next,scrub})EnableWindow(item,animation!=nullptr);
        const bool correctable=editable&&boneEditable();for(unsigned i=10;i<16;++i)EnableWindow(sliders[i].track,correctable);EnableWindow(clearBone,correctable);
        if(page==1&&document)set(help,boneEditable()?text(L"Local bone corrections blend across the selected phase window. Combined correction is limited to 60°.",L"骨骼局部修正在所选阶段内混合，合成旋转幅度限制为 60°。"):text(L"This helper bone is controlled by the game. You can inspect it, but cannot correct it here.",L"此辅助骨骼由游戏引擎管理。可以查看，但不能在这里修正。"));
        for(unsigned i=16;i<20;++i)EnableWindow(sliders[i].track,editable&&SendMessageW(contactEnable,BM_GETCHECK,0,0)==BST_CHECKED);
        if(!windowRoles().empty()){EnableWindow(sliders[18].track,FALSE);EnableWindow(sliders[19].track,FALSE);if(windowRoles()[std::size_t(std::max(selectedContact,0))].role=="replantSample")EnableWindow(sliders[17].track,FALSE);}
        const auto fields=geometryFields();EnableWindow(geometryEnable,editable&&std::any_of(fields.begin(),fields.end(),[](bool enabled){return enabled;}));
        for(unsigned i=24;i<29;++i)EnableWindow(sliders[i].track,editable&&options.geometry.has_value()&&fields[i-24]);
        const auto route=routeKnots();const bool pathEditable=editable&&options.geometry&&route.size()>=2&&route.size()<=64;
        EnableWindow(knot,editable&&options.geometry&&route.size()>=2);EnableWindow(simplify,editable&&options.geometry&&route.size()>64);
        for(unsigned i=29;i<33;++i)EnableWindow(sliders[i].track,pathEditable);
        if(selectedKnot==0||selectedKnot==int(route.size())-1)for(unsigned i=29;i<33;++i)EnableWindow(sliders[i].track,FALSE);
        for(unsigned i=24;i<33;++i)if((i<29&&!fields[i-24])||(i>=29&&route.empty()))set(sliders[i].value,text(L"N/A",L"不适用"));else sliderValue(i);
        if(adaptedPreview()){set(sliders[20].value,number(activePreview()->wallDistance,1));set(sliders[21].value,mantlePreview()?number(*activePreview()->ledgeHeight,1):text(L"N/A",L"不适用"));}else{sliderValue(20);sliderValue(21);}
        for(const auto& slider:sliders){const auto enabled=IsWindowEnabled(slider.track);EnableWindow(slider.label,enabled);EnableWindow(slider.value,enabled);}routeHelp();
    }
    void translate() {
        set(window,text(L"FreeClimb Animation Author",L"FreeClimb 动作制作工具"));set(brand,L"FreeClimb");set(tagline,text(L"ANIMATION AUTHOR",L"动作制作工具"));set(open,text(L"Import HKX…",L"导入 HKX…"));set(base,text(L"Base action",L"基础动作"));
        set(packBrowse,text(L"Base pack…",L"基础包…"));set(exportZip,text(L"Export MO2 ZIP…",L"导出 MO2 ZIP…"));set(stageLabel,text(L"Stage",L"阶段"));set(play,playing?text(L"Pause",L"暂停"):text(L"Play",L"播放"));set(previous,L"◀");set(next,L"▶");
        set(loopBeginLabel,text(L"Run begins (source seconds)",L"跑动开始（源动作秒数）"));set(loopEndLabel,text(L"Run ends (source seconds)",L"跑动结束（源动作秒数）"));
        set(loopBeginHere,text(L"Mark loop start here",L"当前帧设循环开始"));set(loopEndHere,text(L"Mark loop end here",L"当前帧设循环结束"));
        set(splitHelp,text(L"Mark where the repeating run begins and ends. The start and ending remain in this same file. Changes apply automatically.",L"标记持续跑动的开始和结束。起步、跑动和收尾保留在同一文件中，修改后自动更新。"));
        set(stageEditingToggle,chosenPrimary()=="wallRun"?text(L"Edit this action’s wall brace",L"编辑当前动作的扶墙参考"):text(L"Edit individual stages",L"编辑内部阶段"));set(sourceComparisonToggle,text(L"Compare original pose",L"对照原始姿态"));set(reset,text(L"Reset edits",L"还原编辑"));set(inHere,text(L"Start here",L"当前帧设起点"));set(outHere,text(L"End here",L"当前帧设终点"));set(clearBone,text(L"Clear bone correction",L"清除该骨修正"));set(advanced,advancedMotion?text(L"− More adjustments",L"− 更多调整"):text(L"+ More adjustments",L"+ 更多调整"));
        set(contactEnable,text(L"Override selected window",L"覆盖选定窗口"));set(autoCalibration,text(L"Match reference timing automatically",L"自动匹配参考时序"));
        set(wall,text(L"Wall reference",L"显示墙面参照"));set(ledge,text(L"Ledge reference",L"显示边沿参照"));set(joints,text(L"Show all bone links",L"显示所有骨连线"));set(fit,text(L"Fit / reset view",L"适配 / 重置视图"));
        set(geometryEnable,text(L"Override route geometry",L"覆盖路线几何参数"));set(simplify,text(L"Simplify route for editing",L"简化路线以便编辑"));
        constexpr std::array<std::wstring_view,3> ien{L"In-place X",L"In-place Y",L"In-place Z"},izh{L"原地 X",L"原地 Y",L"原地 Z"};
        for(unsigned i=0;i<3;++i)set(inPlace[i],chinese?izh[i]:ien[i]);
        TCITEMW tab{};tab.mask=TCIF_TEXT;constexpr std::array<std::wstring_view,5> en{L"Motion",L"Bones",L"Contacts",L"Scene",L"Route"},zh{L"动作",L"骨骼",L"接触",L"场景",L"轨迹"};
        for(unsigned i=0;i<5;++i){const std::wstring value(chinese?zh[i]:en[i]);tab.pszText=const_cast<wchar_t*>(value.c_str());TabCtrl_SetItem(tabs,int(i),&tab);}
        const auto chosen=selectedName(slot),actual=chosenSlot();SendMessageW(slot,CB_RESETCONTENT,0,0);
        for(unsigned i=0;i<visibleActions.size();++i){const auto found=std::find(fc::converter::slots.begin(),fc::converter::slots.end(),visibleActions[i]);const auto index=std::size_t(found-fc::converter::slots.begin());const auto label=chinese?slotChinese[index]:slotEnglish[index];const auto at=SendMessageW(slot,CB_ADDSTRING,0,reinterpret_cast<LPARAM>(label.data()));SendMessageW(slot,CB_SETITEMDATA,at,i);}
        selectPrimary(chosen.empty()?"hang":chosen);populateStages(actual);SendMessageW(contact,CB_RESETCONTENT,0,0);
        const auto roles=windowRoles();if(roles.empty()) {
            constexpr std::array<std::wstring_view,4> cen{L"Left hand",L"Right hand",L"Left foot",L"Right foot"},czh{L"左手",L"右手",L"左脚",L"右脚"};
            for(unsigned i=0;i<4;++i){const std::wstring value(chinese?czh[i]:cen[i]);SendMessageW(contact,CB_ADDSTRING,0,reinterpret_cast<LPARAM>(value.c_str()));}
        }else for(const auto& role:roles){const std::wstring value(chinese?role.zh:role.en);SendMessageW(contact,CB_ADDSTRING,0,reinterpret_cast<LPARAM>(value.c_str()));}
        SendMessageW(contact,CB_SETCURSEL,selectedContact,0);for(unsigned i=0;i<definitions.size();++i)set(sliders[i].label,chinese?definitions[i].zh:definitions[i].en);
        constexpr std::array<std::wstring_view,5> hen{
            L"Clip speed changes timing, not game speed. Timing match is not grip reliability. Positions use Skyrim units.",
            L"Local bone corrections blend across the selected phase window. Combined correction is limited to 60°.",
            L"Drag selected window edges on the timeline. Special action windows control their actual grip phases.",
            L"Base actions preview their controller route; imported actions show only their edited source motion. Game collisions, skin and physics are not tested.",
            L"Optional geometry changes affect exported metadata. Routes over 64 knots are retained unless you explicitly simplify them."};
        constexpr std::array<std::wstring_view,5> hzh{
            L"片段速度修改时序，不是游戏移动速度。时序匹配度不等于抓点可靠性。位移使用 Skyrim 单位。",
            L"骨骼局部修正在所选阶段内混合，合成旋转幅度限制为 60°。",
            L"拖动时间线中选定窗口的边沿。特殊动作窗口控制真实抓放阶段。",
            L"基础动作预览控制器路线；导入动作仅显示编辑后的源动作及位移。不测试游戏碰撞、蒙皮或物理。",
            L"可选几何修改写入动作配置。超过 64 节点的原路线会保留，除非主动选择简化。"};
        std::wstring helpText(page==2&&document&&document->authored?text(L"Each limb has its own support curve. Support requires an actual nearby surface and bounded IK; source limb motion is retained.",L"四肢可分别编辑支撑曲线。支撑需要实际邻近表面并受 IK 限制，保留源手脚运动。"):(chinese?hzh[std::size_t(page)]:hen[std::size_t(page)]));
        if(page==0&&wholeDocument)helpText+=text(L"\r\nEdits apply to the complete selected action.",L"\r\n编辑会应用于当前完整动作。");
        set(help,helpText);set(status,text(statusEnglish,statusChinese));markerSummary();layout();invalidate();availability();
    }
    void changeDpi(int value) {
        dpi=value;const auto replacement=CreateFontW(-MulDiv(14,dpi,96),0,0,0,FW_NORMAL,FALSE,FALSE,FALSE,DEFAULT_CHARSET,OUT_DEFAULT_PRECIS,CLIP_DEFAULT_PRECIS,CLEARTYPE_QUALITY,DEFAULT_PITCH,L"Segoe UI");
        if(replacement){EnumChildWindows(window,[](HWND item,LPARAM data){SendMessageW(item,WM_SETFONT,WPARAM(data),TRUE);return TRUE;},reinterpret_cast<LPARAM>(replacement));if(font)DeleteObject(font);font=replacement;}
        if(brandFont)DeleteObject(brandFont);if(smallFont)DeleteObject(smallFont);
        brandFont=CreateFontW(-MulDiv(22,dpi,96),0,0,0,FW_SEMIBOLD,FALSE,FALSE,FALSE,DEFAULT_CHARSET,OUT_DEFAULT_PRECIS,CLIP_DEFAULT_PRECIS,CLEARTYPE_QUALITY,DEFAULT_PITCH,L"Segoe UI");
        smallFont=CreateFontW(-MulDiv(10,dpi,96),0,0,0,FW_NORMAL,FALSE,FALSE,FALSE,DEFAULT_CHARSET,OUT_DEFAULT_PRECIS,CLIP_DEFAULT_PRECIS,CLEARTYPE_QUALITY,DEFAULT_PITCH,L"Segoe UI");
        SendMessageW(brand,WM_SETFONT,reinterpret_cast<WPARAM>(brandFont),TRUE);SendMessageW(tagline,WM_SETFONT,reinterpret_cast<WPARAM>(smallFont),TRUE);
        for(const auto item:{slot,stage,language,bone,contact,knot}){SendMessageW(item,CB_SETITEMHEIGHT,WPARAM(-1),MulDiv(25,dpi,96));SendMessageW(item,CB_SETITEMHEIGHT,0,MulDiv(27,dpi,96));}layout();
    }
    void scrollPanel(int position) {
        const int target=std::clamp(position,0,std::max(0,panelContentHeight-panelViewportHeight));if(target==panelScroll[std::size_t(page)])return;
        panelScroll[std::size_t(page)]=target;layout();
    }
    void revealControl(HWND item) {
        RECT area{};GetWindowRect(item,&area);for(const auto& slider:sliders)if(slider.track==item){RECT label{};GetWindowRect(slider.label,&label);UnionRect(&area,&area,&label);break;}MapWindowPoints(nullptr,panel,reinterpret_cast<POINT*>(&area),2);
        const int top=MulDiv(area.top,96,dpi),bottom=MulDiv(area.bottom,96,dpi),position=panelScroll[std::size_t(page)];
        if(top<4)scrollPanel(position+top-4);else if(bottom>panelViewportHeight-4)scrollPanel(position+bottom-panelViewportHeight+4);
    }
    void layout() {
        RECT area{};GetClientRect(window,&area);const int width=MulDiv(area.right,96,dpi),height=MulDiv(area.bottom,96,dpi),side=312,content=width-side-42,x=width-side-16;
        auto place=[&](HWND item,int px,int py,int w,int h){RECT before{};GetWindowRect(item,&before);MapWindowPoints(nullptr,GetParent(item),reinterpret_cast<POINT*>(&before),2);const int left=MulDiv(px,dpi,96),top=MulDiv(py,dpi,96),width=MulDiv(w,dpi,96),height=MulDiv(h,dpi,96);wchar_t name[32]{};GetClassNameW(item,name,32);if(before.left!=left||before.top!=top||before.right-before.left!=width||(std::wstring_view(name)!=L"ComboBox"&&before.bottom-before.top!=height))SetWindowPos(item,nullptr,left,top,width,height,SWP_NOZORDER|SWP_NOACTIVATE);};
        place(brand,16,10,124,33);place(tagline,145,23,std::max(80,content-380),17);place(open,content-208,12,108,32);place(base,content-90,12,106,32);place(exportZip,x,12,side,34);
        place(slot,16,57,content,380);place(language,x,57,side,200);
        place(input,16,96,std::max(100,content-145),25);place(packBrowse,content-114,93,130,29);ShowWindow(pack,SW_HIDE);
        ShowWindow(stage,SW_HIDE);ShowWindow(stageLabel,SW_HIDE);
        const int stageHeight=sequencePreview()?30:0;place(preview,16,131,content,std::max(100,height-342-stageHeight));place(timeline,16,height-204-stageHeight,content,105+stageHeight);
        place(play,16,height-88,72,29);place(previous,96,height-88,32,29);place(next,134,height-88,32,29);
        place(scrub,178,height-90,std::max(80,content-440),35);place(clock,content-241,height-88,257,29);place(status,16,height-43,width-32,27);
        place(tabs,x,104,side,std::max(120,height-156));if(!panel)return;
        RECT tabArea{};GetClientRect(tabs,&tabArea);TabCtrl_AdjustRect(tabs,FALSE,&tabArea);MapWindowPoints(tabs,window,reinterpret_cast<POINT*>(&tabArea),2);
        const int inset=MulDiv(4,dpi,96);RECT oldPanel{};GetWindowRect(panel,&oldPanel);MapWindowPoints(nullptr,window,reinterpret_cast<POINT*>(&oldPanel),2);const RECT newPanel{tabArea.left+inset,tabArea.top+inset,tabArea.right-inset,tabArea.bottom-inset};if(!EqualRect(&oldPanel,&newPanel))SetWindowPos(panel,nullptr,newPanel.left,newPanel.top,std::max(1,int(newPanel.right-newPanel.left)),std::max(1,int(newPanel.bottom-newPanel.top)),SWP_NOZORDER|SWP_NOACTIVATE);
        RECT panelArea{};GetClientRect(panel,&panelArea);const int panelWidth=int(panelArea.right)*96/dpi,inner=4,available=std::max(80,panelWidth-8);
        panelViewportHeight=std::max(1,int(panelArea.bottom)*96/dpi);int y=4;
        const auto dc=GetDC(panel);const auto oldFont=SelectObject(dc,font);
        auto textHeight=[&](HWND item,int w){RECT bounds{0,0,MulDiv(w,dpi,96),0};const auto value=fieldText(item);DrawTextW(dc,value.c_str(),int(value.size()),&bounds,DT_CALCRECT|DT_WORDBREAK|DT_NOPREFIX);return std::max(19,(int(bounds.bottom)*96+dpi-1)/dpi+2);};
        std::vector<HWND> shown;
        auto show=[&](HWND item,int px,int py,int w,int h){shown.push_back(item);place(item,px,py-panelScroll[std::size_t(page)],w,h);if(!(GetWindowLongPtrW(item,GWL_STYLE)&WS_VISIBLE)){ShowWindow(item,SW_SHOW);wchar_t type[32]{};GetClassNameW(item,type,32);if(std::wstring_view(type)==TRACKBAR_CLASSW)SendMessageW(item,TBM_SETPOS,TRUE,SendMessageW(item,TBM_GETPOS,0,0));}};
        auto button=[&](HWND item,bool checkbox){const int h=std::max(27,textHeight(item,available-(checkbox?24:12))+8);show(item,inner,y,available,h);y+=h+6;};
        auto combo=[&](HWND item){show(item,inner,y,available,240);RECT bounds{};GetWindowRect(item,&bounds);y+=(int(bounds.bottom-bounds.top)*96+dpi-1)/dpi+10;};
        if(page==0&&wholeDocument&&(document->authored||advancedMotion)) {
            const int helpHeight=textHeight(splitHelp,available);show(splitHelp,inner,y,available,helpHeight);y+=helpHeight+8;
            for(const auto row:{std::pair{loopBeginLabel,loopBegin},std::pair{loopEndLabel,loopEnd}}){const int h=textHeight(row.first,available);show(row.first,inner,y,available,h);y+=h+3;show(row.second,inner,y,available,27);y+=34;}
            button(loopBeginHere,false);button(loopEndHere,false);y+=8;
        }
        if(page==0&&(stageEditing||advancedMotion)&&document&&!document->authored)button(sourceComparisonToggle,true);
        if(page==0){button(autoCalibration,true);const int half=(available-6)/2;show(inHere,inner,y,half,29);show(outHere,inner+half+6,y,available-half-6,29);y+=37;}
        if(page==1)combo(bone);
        if(page==2){combo(contact);button(contactEnable,true);}
        if(page==3){button(wall,true);button(ledge,true);button(joints,true);}
        if(page==4){button(geometryEnable,true);const int h=textHeight(help,available);show(help,inner,y,available,h);y+=h+10;combo(knot);button(simplify,false);}
        for(unsigned i=0;i<definitions.size();++i)if(definitions[i].page==page){if(page==0&&i==4)button(advanced,false);if(page==0&&i>=4&&!advancedMotion)continue;const int h=std::max(textHeight(sliders[i].label,available-76),textHeight(sliders[i].value,68));show(sliders[i].label,inner,y,available-76,h);show(sliders[i].value,inner+available-68,y,68,h);show(sliders[i].track,inner,y+h+2,available,24);if(SendMessageW(sliders[i].track,TBM_GETTHUMBLENGTH,0,0)!=MulDiv(18,dpi,96))SendMessageW(sliders[i].track,TBM_SETTHUMBLENGTH,MulDiv(18,dpi,96),0);y+=h+32;}
        if(page==0){if(advancedMotion){const int third=available/3;for(unsigned i=0;i<3;++i)show(inPlace[i],inner+int(i)*third,y+3,third-2,29);y+=38;}button(reset,false);}
        if(page==1)button(clearBone,false);if(page==3)button(fit,false);
        if(page!=4){const int helpHeight=textHeight(help,available);show(help,inner,y+6,available,helpHeight);y+=helpHeight+18;}
        if(page==2&&!windowRoles().empty()){show(special,inner,y,available,120);y+=132;}
        for(HWND item=GetWindow(panel,GW_CHILD);item;item=GetWindow(item,GW_HWNDNEXT))if(std::find(shown.begin(),shown.end(),item)==shown.end())ShowWindow(item,SW_HIDE);
        SelectObject(dc,oldFont);ReleaseDC(panel,dc);panelContentHeight=y;
        const int position=std::clamp(panelScroll[std::size_t(page)],0,std::max(0,panelContentHeight-panelViewportHeight));
        SCROLLINFO scroll{sizeof(scroll),SIF_RANGE|SIF_PAGE|SIF_POS|SIF_DISABLENOSCROLL,0,std::max(0,panelContentHeight-1),UINT(panelViewportHeight),position};SetScrollInfo(panel,SB_VERT,&scroll,TRUE);
        if(position!=panelScroll[std::size_t(page)]){panelScroll[std::size_t(page)]=position;layout();}
    }
    void initialize() {
        dpi=int(GetDpiForWindow(window));font=CreateFontW(-MulDiv(14,dpi,96),0,0,0,FW_NORMAL,FALSE,FALSE,FALSE,DEFAULT_CHARSET,OUT_DEFAULT_PRECIS,CLIP_DEFAULT_PRECIS,CLEARTYPE_QUALITY,DEFAULT_PITCH,L"Segoe UI");
        brand=control(L"STATIC",0);tagline=control(L"STATIC",0);input=control(L"EDIT",ES_AUTOHSCROLL|ES_READONLY,inputId);pack=control(L"EDIT",ES_AUTOHSCROLL|ES_READONLY,packId);slot=control(L"COMBOBOX",CBS_DROPDOWNLIST|WS_TABSTOP|WS_VSCROLL,slotId);
        stage=control(L"COMBOBOX",CBS_DROPDOWNLIST|WS_TABSTOP,stageId);stageLabel=control(L"STATIC",0);
        language=control(L"COMBOBOX",CBS_DROPDOWNLIST|WS_TABSTOP,languageId);for(const auto name:{L"English",L"简体中文"})SendMessageW(language,CB_ADDSTRING,0,reinterpret_cast<LPARAM>(name));SendMessageW(language,CB_SETCURSEL,0,0);
        open=control(L"BUTTON",BS_PUSHBUTTON|WS_TABSTOP,openId);base=control(L"BUTTON",BS_PUSHBUTTON|WS_TABSTOP,baseId);packBrowse=control(L"BUTTON",BS_PUSHBUTTON|WS_TABSTOP,packBrowseId);exportZip=control(L"BUTTON",BS_PUSHBUTTON|WS_TABSTOP,exportId);
        tabs=control(WC_TABCONTROLW,WS_TABSTOP|WS_CLIPSIBLINGS,tabId);TCITEMW tab{};tab.mask=TCIF_TEXT;tab.pszText=const_cast<wchar_t*>(L"");for(int i=0;i<5;++i)TabCtrl_InsertItem(tabs,i,&tab);
        panel=CreateWindowExW(WS_EX_CONTROLPARENT,L"FreeClimbEditorPanel",L"",WS_CHILD|WS_VISIBLE|WS_VSCROLL|WS_CLIPCHILDREN|WS_CLIPSIBLINGS,0,0,1,1,window,nullptr,GetModuleHandleW(nullptr),this);
        SetWindowPos(tabs,panel,0,0,0,0,SWP_NOMOVE|SWP_NOSIZE|SWP_NOACTIVATE);
        preview=control(L"FreeClimbPreview",WS_CLIPSIBLINGS,previewId);timeline=control(L"FreeClimbPreview",WS_CLIPSIBLINGS,timelineId);SetWindowLongPtrW(preview,GWLP_USERDATA,reinterpret_cast<LONG_PTR>(this));SetWindowLongPtrW(timeline,GWLP_USERDATA,reinterpret_cast<LONG_PTR>(this));
        play=control(L"BUTTON",BS_PUSHBUTTON|WS_TABSTOP,playId);previous=control(L"BUTTON",BS_PUSHBUTTON|WS_TABSTOP,previousId);next=control(L"BUTTON",BS_PUSHBUTTON|WS_TABSTOP,nextId);
        scrub=control(TRACKBAR_CLASSW,TBS_NOTICKS|WS_TABSTOP,scrubId);SendMessageW(scrub,TBM_SETRANGE,TRUE,MAKELPARAM(0,1000));clock=control(L"STATIC",0);status=control(L"STATIC",SS_ENDELLIPSIS);
        help=panelControl(L"STATIC",0);special=panelControl(L"EDIT",ES_MULTILINE|ES_READONLY|WS_VSCROLL);reset=panelControl(L"BUTTON",BS_PUSHBUTTON|WS_TABSTOP,resetId);
        stageEditingToggle=panelControl(L"BUTTON",BS_AUTOCHECKBOX|BS_MULTILINE|WS_TABSTOP,stageEditingId);sourceComparisonToggle=panelControl(L"BUTTON",BS_AUTOCHECKBOX|BS_MULTILINE|WS_TABSTOP,sourceComparisonId);
        inHere=panelControl(L"BUTTON",BS_PUSHBUTTON|WS_TABSTOP,inHereId);outHere=panelControl(L"BUTTON",BS_PUSHBUTTON|WS_TABSTOP,outHereId);bone=panelControl(L"COMBOBOX",CBS_DROPDOWNLIST|WS_TABSTOP|WS_VSCROLL,boneId);advanced=panelControl(L"BUTTON",BS_PUSHBUTTON|WS_TABSTOP,advancedId);
        loopBegin=panelControl(L"EDIT",ES_AUTOHSCROLL|WS_TABSTOP|WS_BORDER,loopBeginId);loopEnd=panelControl(L"EDIT",ES_AUTOHSCROLL|WS_TABSTOP|WS_BORDER,loopEndId);
        loopBeginLabel=panelControl(L"STATIC",0);loopEndLabel=panelControl(L"STATIC",0);splitHelp=panelControl(L"STATIC",0);
        loopBeginHere=panelControl(L"BUTTON",BS_PUSHBUTTON|WS_TABSTOP,loopBeginHereId);loopEndHere=panelControl(L"BUTTON",BS_PUSHBUTTON|WS_TABSTOP,loopEndHereId);
        clearBone=panelControl(L"BUTTON",BS_PUSHBUTTON|WS_TABSTOP,clearBoneId);contact=panelControl(L"COMBOBOX",CBS_DROPDOWNLIST|WS_TABSTOP,contactId);contactEnable=panelControl(L"BUTTON",BS_AUTOCHECKBOX|BS_MULTILINE|WS_TABSTOP,contactEnableId);
        autoCalibration=panelControl(L"BUTTON",BS_AUTOCHECKBOX|BS_MULTILINE|WS_TABSTOP,autoId);wall=panelControl(L"BUTTON",BS_AUTOCHECKBOX|BS_MULTILINE|WS_TABSTOP,wallId);ledge=panelControl(L"BUTTON",BS_AUTOCHECKBOX|BS_MULTILINE|WS_TABSTOP,ledgeId);
        joints=panelControl(L"BUTTON",BS_AUTOCHECKBOX|BS_MULTILINE|WS_TABSTOP,jointsId);fit=panelControl(L"BUTTON",BS_PUSHBUTTON|WS_TABSTOP,fitId);for(unsigned i=0;i<3;++i)inPlace[i]=panelControl(L"BUTTON",BS_AUTOCHECKBOX|BS_MULTILINE|WS_TABSTOP,inPlaceXId+int(i));
        geometryEnable=panelControl(L"BUTTON",BS_AUTOCHECKBOX|BS_MULTILINE|WS_TABSTOP,geometryId);knot=panelControl(L"COMBOBOX",CBS_DROPDOWNLIST|WS_TABSTOP|WS_VSCROLL,knotId);simplify=panelControl(L"BUTTON",BS_PUSHBUTTON|WS_TABSTOP,simplifyId);
        for(unsigned i=0;i<definitions.size();++i){sliders[i].label=panelControl(L"STATIC",0);sliders[i].value=panelControl(L"STATIC",SS_RIGHT);sliders[i].track=panelControl(TRACKBAR_CLASSW,TBS_NOTICKS|TBS_FIXEDLENGTH|WS_TABSTOP,300+int(i));SendMessageW(sliders[i].track,TBM_SETRANGE,TRUE,MAKELPARAM(0,1000));put(i,definitions[i].initial);}
        for(const auto item:{autoCalibration,wall,ledge})SendMessageW(item,BM_SETCHECK,BST_CHECKED,0);if(const auto found=fc::converter::discoverPack(executable,regularFile))set(pack,found->wstring());
        tooltips=CreateWindowExW(WS_EX_TOPMOST,TOOLTIPS_CLASSW,nullptr,WS_POPUP|TTS_ALWAYSTIP|TTS_NOPREFIX,0,0,0,0,window,nullptr,GetModuleHandleW(nullptr),nullptr);SendMessageW(tooltips,TTM_SETMAXTIPWIDTH,0,MulDiv(600,dpi,96));SendMessageW(tooltips,TTM_SETTIPBKCOLOR,theme::field,0);SendMessageW(tooltips,TTM_SETTIPTEXTCOLOR,theme::text,0);tooltip(input);tooltip(packBrowse);tooltip(status);
        if(const auto module=LoadLibraryW(L"dwmapi.dll")){using SetAttribute=HRESULT(WINAPI*)(HWND,DWORD,LPCVOID,DWORD);if(const auto apply=reinterpret_cast<SetAttribute>(GetProcAddress(module,"DwmSetWindowAttribute"))){const BOOL dark=TRUE;apply(window,20,&dark,sizeof(dark));}FreeLibrary(module);}
        changeDpi(dpi);translate();availability();message(L"Choose an action, import an HKX or load its base, then preview and export.",L"选择动作，导入 HKX 或载入基础动作，再预览并导出。");
        DragAcceptFiles(window,TRUE);lastTick=GetTickCount64();SetTimer(window,1,16,nullptr);
    }
    std::optional<std::filesystem::path> browse(bool save,const wchar_t* filter,const wchar_t* extension,std::filesystem::path initial={}) {
        std::vector<wchar_t> buffer(32768);const auto value=initial.wstring();if(value.size()<buffer.size())std::copy(value.begin(),value.end(),buffer.begin());
        OPENFILENAMEW dialog{sizeof(dialog)};dialog.hwndOwner=window;dialog.lpstrFilter=filter;dialog.lpstrFile=buffer.data();dialog.nMaxFile=DWORD(buffer.size());dialog.lpstrDefExt=extension;
        dialog.Flags=OFN_EXPLORER|OFN_NOCHANGEDIR|OFN_PATHMUSTEXIST|(save?0:OFN_FILEMUSTEXIST);
        if(save?GetSaveFileNameW(&dialog):GetOpenFileNameW(&dialog))return std::filesystem::path(buffer.data());return {};
    }
    void startWorker(Work type,std::function<void(Completion&)> operation) {
        if(busy)return;if(worker.joinable())worker.join();busy=true;currentWork=type;availability();
        try{worker=std::thread([target=window,type,tag=revision,operation=std::move(operation)] {
            auto result=std::make_unique<Completion>();result->work=type;result->revision=tag;
            try{operation(*result);}catch(const std::exception& failure){result->failure=failure.what();}catch(...){result->failure="Unexpected author tool failure";}
            if(PostMessageW(target,completed,0,reinterpret_cast<LPARAM>(result.get())))result.release();
        });}catch(const std::exception& failure){busy=false;if(type==Work::load)restoreLoadedDocument();availability();error(failure.what());}
    }
    std::string selectedName(HWND item)const {
        const auto selected=SendMessageW(item,CB_GETCURSEL,0,0);if(selected<0)return {};
        const auto index=SendMessageW(item,CB_GETITEMDATA,WPARAM(selected),0);if(item==slot)return index>=0&&index<int(visibleActions.size())?toUtf8(visibleActions[std::size_t(index)]):std::string{};return index>=0&&index<int(fc::converter::slots.size())?toUtf8(fc::converter::slots[std::size_t(index)]):std::string{};
    }
    std::string chosenPrimary()const{return std::string(fc::converter::actionPrimaryForSlot(selectedName(slot)));}
    std::string chosenSlot()const{return fc::converter::isActionGroupPrimary(chosenPrimary())?selectedName(stage):chosenPrimary();}
    void selectDirection(std::string_view name) {if(fc::converter::isWallRunPrimary(name))selectPrimary(name);}
    static std::string stageKey(std::string_view name,std::string_view direction){return fc::converter::isWallRunHelper(name)?std::string(direction)+"/"+std::string(name):std::string(name);}
    void selectPrimary(std::string_view name) {
        if(name=="wallRun")name=fc::converter::isWallRunPrimary(documentDirection)?documentDirection:"runUp";
        else if(name=="contextHop")name=selectedName(slot)=="contextHopRight"?"contextHopRight":"contextHopLeft";
        for(int i=0;i<SendMessageW(slot,CB_GETCOUNT,0,0);++i){const auto at=SendMessageW(slot,CB_GETITEMDATA,i,0);if(at>=0&&at<int(visibleActions.size())&&toUtf8(visibleActions[std::size_t(at)])==name){SendMessageW(slot,CB_SETCURSEL,i,0);return;}}
    }
    void populateStages(std::string_view wanted={}) {
        const auto selectedAction=selectedName(slot);if(wanted.empty())wanted=selectedAction;SendMessageW(stage,CB_RESETCONTENT,0,0);const auto primary=chosenPrimary();const auto names=primary=="wallRun"?fc::converter::wallRunStages(chosenDirection()):fc::converter::actionStages(primary);int selected=-1;
        for(const auto name:names){std::wstring label;
            if(name=="runCatch")label=text(L"Finish / return to climbing",L"收尾／回到攀岩");
            else if(name=="sideBrace")label=text(L"Private wall brace reference",L"当前动作的扶墙参考");
            else if(name=="contextHang")label=text(L"Private preparation / catch reference",L"当前动作的准备／落抓参考");
            else if(name=="runLaunchLeft")label=text(L"Start",L"起步");
            else if(name=="runLaunchRight")label=text(L"Start",L"起步");
            else if(name=="runLaunch")label=text(L"Start upward",L"向上起步");
            else {const auto found=std::find_if(fc::converter::slots.begin(),fc::converter::slots.end(),[&](auto slotName){return toUtf8(slotName)==name;});const auto at=std::size_t(found-fc::converter::slots.begin());label=chinese?slotChinese[at]:slotEnglish[at];}
            const auto index=SendMessageW(stage,CB_ADDSTRING,0,reinterpret_cast<LPARAM>(label.c_str()));
            for(unsigned i=0;i<fc::converter::slots.size();++i)if(toUtf8(fc::converter::slots[i])==name)SendMessageW(stage,CB_SETITEMDATA,WPARAM(index),i);
            if(name==wanted)selected=int(index);
        }SendMessageW(stage,CB_SETCURSEL,selected>=0?selected:(!names.empty()?0:-1),0);
    }
    void selectDocumentIdentity(std::string_view actual) {
        selectPrimary(fc::converter::isWallRunHelper(actual)?documentDirection:actual);populateStages(actual);
    }
    void saveStage() {
        if(!document||!animation||!fc::converter::isActionGroupMember(document->slot))return;
        StageEdit saved{document,animation,options};for(unsigned i=0;i<definitions.size();++i)saved.values[i]=get(i);
        saved.wholeDocument=wholeDocument;saved.sequence=wholeSequence;saved.loopBegin=fieldText(loopBegin);saved.loopEnd=fieldText(loopEnd);saved.bone=selectedBone;saved.contact=selectedContact;saved.knot=selectedKnot;saved.time=time;saved.direction=documentDirection;saved.dirty=editedAfterLoad;saved.ready=revision==displayRevision;saved.camera=camera;saved.reference=reference;
        stages[stageKey(document->slot,documentDirection)]=std::move(saved);
    }
    bool groupReady()const {
        const auto selectedPack=std::filesystem::path(fieldText(pack));const auto group=chosenPrimary();if(group=="wallRun")return true;
        for(const auto& [key,saved]:stages)if(saved.dirty&&saved.document&&fc::converter::actionPrimaryForSlot(saved.document->slot)==group&&samePath(saved.document->pack,selectedPack)&&key!=stageKey(document?document->slot:std::string{},documentDirection))if(!saved.ready)return false;
        return true;
    }
    std::optional<std::vector<StageEdit>> previewStages(std::string_view name,const std::filesystem::path& packFile)const {
        std::vector<StageEdit> result;const auto primary=fc::converter::actionPrimaryForSlot(name,chosenPrimary());if(primary=="wallRun")return result;
        for(const auto& [key,saved]:stages)if(saved.document&&saved.dirty&&fc::converter::actionPrimaryForSlot(saved.document->slot)==primary&&samePath(saved.document->pack,packFile)&&key!=stageKey(name,chosenDirection())) {
            if(!saved.ready)return {};result.push_back(saved);
        }return result;
    }
    bool restoreStage(std::string_view name) {
        const auto at=stages.find(stageKey(name,chosenDirection()));if(at==stages.end()||!at->second.document||!samePath(at->second.document->pack,std::filesystem::path(fieldText(pack))))return false;
        const auto saved=at->second;++revision;Completion loaded;loaded.work=Work::load;loaded.revision=revision;loaded.document=saved.document;loaded.animation=saved.animation;loaded.direction=name=="sideBrace"?chosenDirection():saved.direction;loaded.wholeDocument=saved.wholeDocument;loaded.wholeAction=saved.wholeDocument!=nullptr;if(saved.sequence)loaded.authoredSequence=std::make_shared<fc::ConverterWallRunSequence>(*saved.sequence);finished(loaded);setting=true;set(loopBegin,saved.loopBegin);set(loopEnd,saved.loopEnd);setting=false;
        options=saved.options;editedAfterLoad=saved.dirty;selectedBone=saved.bone;selectedContact=saved.contact;selectedKnot=saved.knot;time=saved.time;camera=saved.camera;reference=saved.reference;
        setting=true;for(unsigned i=0;i<definitions.size();++i)put(i,saved.values[i]);setting=false;
        SendMessageW(bone,CB_SETCURSEL,selectedBone,0);SendMessageW(contact,CB_SETCURSEL,selectedContact,0);
        SendMessageW(autoCalibration,BM_SETCHECK,options.autoCalibration?BST_CHECKED:BST_UNCHECKED,0);
        const std::array<float,3> axes{options.makeInPlaceAxes.x,options.makeInPlaceAxes.y,options.makeInPlaceAxes.z};for(unsigned i=0;i<3;++i)SendMessageW(inPlace[i],BM_SETCHECK,axes[i]!=0?BST_CHECKED:BST_UNCHECKED,0);
        for(const auto pair:{std::pair{wall,reference.wall},std::pair{ledge,reference.ledge},std::pair{joints,reference.joints}})SendMessageW(pair.first,BM_SETCHECK,pair.second?BST_CHECKED:BST_UNCHECKED,0);
        SendMessageW(geometryEnable,BM_SETCHECK,options.geometry?BST_CHECKED:BST_UNCHECKED,0);selectBone();selectContact();populateRoute();
        displayRevision=saved.ready?revision:revision-1;availability();invalidate();
        if(!saved.ready)message(L"This stage has unapplied or invalid edits. Correct them before exporting the group.",L"此阶段有未应用或无效的修改，请修正后再导出动作组。");else apply();return true;
    }
    std::vector<StageEdit> exportStages() {
        saveStage();std::vector<StageEdit> result;
        if(wholeDocument){if(!wholeSequence||revision!=displayRevision)throw std::runtime_error("Complete action has invalid or unapplied edits");for(const auto& item:wholeSequence->stages){StageEdit saved{std::make_shared<fc::ConverterEditorDocument>(item.document),std::make_shared<fc::ConverterEditedAnimation>(item.animation),item.options};saved.direction=chosenDirection();saved.ready=saved.dirty=true;result.push_back(std::move(saved));}return result;}
        for(const auto& [key,saved]:stages)if(saved.document&&fc::converter::actionPrimaryForSlot(saved.document->slot)==chosenPrimary()&&samePath(saved.document->pack,document->pack)&&(saved.dirty||key==stageKey(document->slot,documentDirection))){
            if(!saved.ready)throw std::runtime_error("An action direction has unapplied or invalid edits");result.push_back(saved);
        }return result;
    }
    bool selectionMatchesDocument()const {
        return document&&chosenSlot()==document->slot&&(chosenPrimary()!="wallRun"||chosenDirection()==documentDirection)&&samePath(std::filesystem::path(fieldText(pack)),document->pack);
    }
    void restoreLoadedDocument() {
        if(!document)return;set(input,document->input.wstring());set(pack,document->pack.wstring());
        selectDirection(documentDirection);selectDocumentIdentity(document->slot);layout();
        if(readyBeforeLoad&&animation)displayRevision=revision;
    }
    void selectPack(const std::filesystem::path& source) {
        if(busy)return;set(pack,source.wstring());availability();
        if(document&&!selectionMatchesDocument())message(L"Base pack changed. Import or load a base slot before exporting; selecting the previous pack keeps your current edits.",L"基础包已更改。请重新导入或载入基础槽位后导出；重新选择原基础包可保留当前编辑。");
        else if(document)message(L"The loaded base pack is selected. Your current edits are retained.",L"已选择当前文档的基础包，保留现有编辑。");
    }
    void load(std::filesystem::path source) {
        if(busy)return;saveStage();readyBeforeLoad=animation&&revision==displayRevision;const auto packFile=std::filesystem::path(fieldText(pack));const auto name=chosenSlot();if(!regularFile(packFile)){restoreLoadedDocument();availability();message(L"Select the complete FreeClimb pack.json with Base pack.",L"请用“基础包”选择完整的 FreeClimb pack.json。");return;}
        if(!regularFile(source)){restoreLoadedDocument();availability();message(L"The input HKX does not exist.",L"输入 HKX 文件不存在。");return;}playing=false;set(play,text(L"Play",L"播放"));++revision;message(L"Decoding HKX and matching reference timing…",L"正在解码 HKX 并匹配参考时序…");
        const bool whole=fc::converter::isWallRunPrimary(name)&&!stageEditing;
        auto overlays=previewStages(name,packFile);startWorker(Work::load,[source=std::move(source),packFile,name,whole,direction=chosenPrimary()=="wallRun"?chosenDirection():std::string{},overlays=std::move(overlays)](Completion& result){
            result.direction=direction;result.imported=true;result.wholeAction=whole;
            if(whole){auto current=std::make_shared<fc::ConverterWallRunDocument>(fc::loadConverterWallRunEditor(packFile,name));if(samePath(source,current->document.input)){result.imported=false;result.document=std::make_shared<fc::ConverterEditorDocument>(current->document);result.authoredSequence=std::make_shared<fc::ConverterWallRunSequence>();std::string error;if(!fc::applyConverterWallRunEdits(*current,{},current->cuts,*result.authoredSequence,error))throw std::runtime_error(error);result.wholeDocument=std::move(current);result.animation=std::make_shared<fc::ConverterEditedAnimation>(result.authoredSequence->whole);wholePreview(result);return;}
                auto value=std::make_shared<fc::ConverterWallRunDocument>();value->document=fc::loadConverterEditor(source,packFile,name,direction);result.document=std::make_shared<fc::ConverterEditorDocument>(value->document);result.wholeDocument=std::move(value);result.animation=std::make_shared<fc::ConverterEditedAnimation>(fc::applyConverterEdits(*result.document,{}));return;}

            result.document=std::make_shared<fc::ConverterEditorDocument>(fc::loadConverterEditor(source,packFile,name,direction));result.animation=std::make_shared<fc::ConverterEditedAnimation>(fc::applyConverterEdits(*result.document,{}));result.runtimePreview=buildPreview(*result.document,*result.animation,overlays,{},&result.previewWarning);if(fc::converter::actionPrimaryForSlot(name)=="wallRun")result.fullPreview=buildPreview(*result.document,*result.animation,overlays,direction,&result.previewWarning);
        });
    }
    void markLoopBoundary(bool end) {
        if(!document||!animation||(adaptedPreview()&&!wholeDocument))return;
        set(end?loopEnd:loopBegin,number(sourceTime(),4));
    }
    void loadBase() {
        if(busy)return;previewChoice();saveStage();readyBeforeLoad=animation&&revision==displayRevision;const auto packFile=std::filesystem::path(fieldText(pack));const auto name=chosenSlot();playing=false;set(play,text(L"Play",L"播放"));++revision;
        message(L"Loading and validating the base action…",L"正在载入并验证基础动作…");
        const bool whole=fc::converter::isWallRunPrimary(name)&&!stageEditing;
        auto overlays=previewStages(name,packFile);startWorker(Work::load,[packFile,name,whole,direction=chosenPrimary()=="wallRun"?chosenDirection():std::string{},overlays=std::move(overlays)](Completion& result){
            result.direction=direction;result.wholeAction=whole;
            if(whole){result.wholeDocument=std::make_shared<fc::ConverterWallRunDocument>(fc::loadConverterWallRunEditor(packFile,name));result.document=std::make_shared<fc::ConverterEditorDocument>(result.wholeDocument->document);result.authoredSequence=std::make_shared<fc::ConverterWallRunSequence>();std::string error;if(!fc::applyConverterWallRunEdits(*result.wholeDocument,{},result.wholeDocument->cuts,*result.authoredSequence,error))throw std::runtime_error(error);result.animation=std::make_shared<fc::ConverterEditedAnimation>(result.authoredSequence->whole);wholePreview(result);return;}
            result.document=std::make_shared<fc::ConverterEditorDocument>(fc::loadConverterBaseEditor(packFile,name,direction));result.animation=std::make_shared<fc::ConverterEditedAnimation>(fc::applyConverterEdits(*result.document,{}));result.runtimePreview=buildPreview(*result.document,*result.animation,overlays,{},&result.previewWarning);if(fc::converter::actionPrimaryForSlot(name)=="wallRun")result.fullPreview=buildPreview(*result.document,*result.animation,overlays,direction,&result.previewWarning);
        });
    }
    std::vector<std::array<float,4>> routeKnots()const {
        if(options.geometry&&!options.geometry->path.empty())return options.geometry->path;
        if(animation&&animation->config.contains("path"))return animation->config.at("path").get<std::vector<std::array<float,4>>>();return {};
    }
    void selectKnot() {
        const auto route=routeKnots();selectedKnot=int(SendMessageW(knot,CB_GETCURSEL,0,0));if(selectedKnot<0||std::size_t(selectedKnot)>=route.size())return;
        setting=true;for(unsigned i=0;i<4;++i)put(29+i,route[std::size_t(selectedKnot)][i]);setting=false;availability();invalidate();
    }
    void populateRoute() {
        SendMessageW(knot,CB_RESETCONTENT,0,0);const auto route=routeKnots();
        for(std::size_t i=0;i<route.size();++i){const auto label=std::to_wstring(i+1)+L" · "+number(route[i][0]*100,1)+L"%";SendMessageW(knot,CB_ADDSTRING,0,reinterpret_cast<LPARAM>(label.c_str()));}
        if(route.empty()){const std::wstring label(text(L"No path knots for this action",L"此动作不使用路径节点"));SendMessageW(knot,CB_ADDSTRING,0,reinterpret_cast<LPARAM>(label.c_str()));}
        selectedKnot=std::clamp(selectedKnot,0,int(std::max<std::size_t>(1,route.size())-1));SendMessageW(knot,CB_SETCURSEL,selectedKnot,0);if(!route.empty())selectKnot();
        if(animation) {
            const auto& config=animation->config;const auto travel=config.at("travel");const auto geometry=options.geometry.value_or(fc::ConverterGeometryEdit{config.value("stride",0.f),config.value("height",0.f),{travel.at(0).get<float>(),travel.at(1).get<float>(),travel.at(2).get<float>()}});
            setting=true;put(24,geometry.stride);put(25,geometry.height);put(26,geometry.travel.x);put(27,geometry.travel.y);put(28,geometry.travel.z);setting=false;
        }
    }
    void toggleGeometry() {
        if(!animation)return;
        if(SendMessageW(geometryEnable,BM_GETCHECK,0,0)==BST_CHECKED){const auto& config=animation->config;const auto travel=config.at("travel");options.geometry=fc::ConverterGeometryEdit{config.value("stride",0.f),config.value("height",0.f),{travel.at(0).get<float>(),travel.at(1).get<float>(),travel.at(2).get<float>()}};}
        else options.geometry.reset();populateRoute();changed();
    }
    void updateGeometry(unsigned index) {
        if(!document||setting||!options.geometry||index<24||index>32)return;const auto fields=geometryFields();if(index<29&&!fields[index-24])return;auto& geometry=*options.geometry;
        if(index==24)geometry.stride=get(index);else if(index==25)geometry.height=get(index);else if(index==26)geometry.travel.x=get(index);else if(index==27)geometry.travel.y=get(index);else if(index==28)geometry.travel.z=get(index);
        else {
            auto route=routeKnots();if(route.size()<2||route.size()>64||selectedKnot<0||std::size_t(selectedKnot)>=route.size())return;const auto at=std::size_t(selectedKnot);
            if(at==0||at+1==route.size()){selectKnot();return;}
            float value=get(index);if(index==29){if(at==0)value=0;else if(at+1==route.size())value=1;else value=std::clamp(value,route[at-1][0]+.001f,route[at+1][0]-.001f);put(index,value);}
            if(index==30&&(at==0||at+1==route.size()))value=at==0?0.f:1.f;route[at][index-29]=value;geometry.path=std::move(route);
        }changed();
    }
    void simplifyRoute() {
        if(!options.geometry)return;const auto source=routeKnots();if(source.size()<=64)return;std::vector<std::array<float,4>> reduced;
        for(unsigned i=0;i<33;++i){const float phase=float(i)/32;auto at=std::upper_bound(source.begin(),source.end(),phase,[](float value,const auto& key){return value<key[0];});
            const auto b=std::clamp<std::size_t>(std::size_t(at-source.begin()),1,source.size()-1),a=b-1;const float alpha=std::clamp((phase-source[a][0])/(source[b][0]-source[a][0]),0.f,1.f);
            std::array<float,4> key{phase};for(unsigned channel=1;channel<4;++channel)key[channel]=source[a][channel]+(source[b][channel]-source[a][channel])*alpha;reduced.push_back(key);
        }reduced.front()=source.front();reduced.back()=source.back();options.geometry->path=std::move(reduced);populateRoute();changed();
    }
    void changed(bool deferred=false) {
        if(setting||!document)return;++revision;editedAfterLoad=true;playing=false;set(play,text(L"Play",L"播放"));availability();
        if(deferred){pendingEdit=true;SetTimer(window,2,120,nullptr);availability();message(L"Updating the edited preview…",L"正在更新编辑后的预览…");}else {pendingEdit=false;KillTimer(window,2);if(!busy)apply();else if(currentWork==Work::edit)message(L"Updating the edited preview…",L"正在更新编辑后的预览…");}
    }
    void apply() {
        if(busy||!document)return;pendingEdit=false;KillTimer(window,2);const auto source=document;const auto edits=options;message(L"Updating the edited preview…",L"正在更新编辑后的预览…");
        if(wholeDocument){const auto whole=wholeDocument;const auto cuts=readCuts();startWorker(Work::edit,[source,whole,cuts,edits](Completion& result){result.direction=source->slot;result.wholeAction=true;result.authoredSequence=std::make_shared<fc::ConverterWallRunSequence>();std::string error;
            if(cuts&&fc::applyConverterWallRunEdits(*whole,edits,*cuts,*result.authoredSequence,error)){result.animation=std::make_shared<fc::ConverterEditedAnimation>(result.authoredSequence->whole);wholePreview(result);}
            else {result.authoredSequence.reset();result.animation=std::make_shared<fc::ConverterEditedAnimation>(fc::applyConverterEdits(*source,edits));result.previewWarning=cuts?error:"Mark the loop start and end before exporting this complete action";}
        });return;}
        auto overlays=previewStages(source->slot,source->pack);startWorker(Work::edit,[source,edits,direction=chosenPrimary()=="wallRun"?chosenDirection():std::string{},overlays=std::move(overlays)](Completion& result){result.direction=direction;result.animation=std::make_shared<fc::ConverterEditedAnimation>(fc::applyConverterEdits(*source,edits));result.runtimePreview=buildPreview(*source,*result.animation,overlays,{},&result.previewWarning);if(fc::converter::actionPrimaryForSlot(source->slot)=="wallRun")result.fullPreview=buildPreview(*source,*result.animation,overlays,direction,&result.previewWarning);});
    }
    void updateBone() {
        if(!boneEditable())return;
        std::erase_if(options.bones,[&](const auto& edit){return edit.bone==unsigned(selectedBone);});const fc::Vec rotation{get(10),get(11),get(12)};
        if(rotation.length()>.01f)options.bones.push_back({unsigned(selectedBone),rotation,get(13),std::max(get(13),get(14)),get(15)});
    }
    void selectBone() {
        selectedBone=int(SendMessageW(bone,CB_GETCURSEL,0,0));setting=true;fc::ConverterBoneEdit selected{unsigned(std::max(selectedBone,0))};
        for(const auto& edit:options.bones)if(edit.bone==selected.bone)selected=edit;
        put(10,selected.eulerDegrees.x);put(11,selected.eulerDegrees.y);put(12,selected.eulerDegrees.z);put(13,selected.beginPhase);put(14,selected.endPhase);put(15,selected.fadePhase);setting=false;availability();invalidate();
    }
    void updateContact(int changedIndex=-1) {
        const auto roles=windowRoles();const bool enabled=SendMessageW(contactEnable,BM_GETCHECK,0,0)==BST_CHECKED;
        if(roles.empty()) {
            std::erase_if(options.contacts,[&](const auto& edit){return edit.contact==unsigned(selectedContact);});
            if(enabled)options.contacts.push_back({unsigned(selectedContact),get(16),std::max(get(16),get(17)),get(18),get(19)});
        }else {
            const auto role=roles[std::size_t(selectedContact)].role;auto bounds=currentWindow();
            for(const auto& edit:options.windows)if(edit.role==role)bounds={edit.beginPhase,edit.endPhase};
            if(changedIndex==16)bounds[0]=get(16);else if(changedIndex==17)bounds[1]=get(17);
            std::erase_if(options.windows,[&](const auto& edit){return edit.role==role;});
            if(enabled)options.windows.push_back({std::string(role),bounds[0],bounds[1]});
        }
    }
    void selectContact() {
        selectedContact=int(SendMessageW(contact,CB_GETCURSEL,0,0));setting=true;fc::ConverterContactEdit selected{unsigned(std::max(selectedContact,0))};bool enabled=false;
        if(windowRoles().empty()){for(const auto& edit:options.contacts)if(edit.contact==selected.contact){selected=edit;enabled=true;}}
        else {
            const auto phaseWindow=currentWindow();selected.beginPhase=phaseWindow[0];selected.endPhase=phaseWindow[1];const auto role=windowRoles()[std::size_t(selectedContact)].role;
            for(const auto& edit:options.windows)if(edit.role==role){selected.beginPhase=edit.beginPhase;selected.endPhase=edit.endPhase;enabled=true;}
        }
        SendMessageW(contactEnable,BM_SETCHECK,enabled?BST_CHECKED:BST_UNCHECKED,0);put(16,selected.beginPhase);put(17,selected.endPhase);put(18,selected.fadePhase);put(19,selected.weight);setting=false;availability();invalidate();
    }
    void optionsFromSliders(unsigned index) {
        if(index>=24){updateGeometry(index);return;}
        if(index>=20){reference.wallY=get(20);reference.ledgeZ=get(21);camera.yaw=get(22)*.01745329252f;camera.pitch=get(23)*.01745329252f;InvalidateRect(preview,nullptr,FALSE);return;}
        if(!document||setting||(index>=10&&index<16&&!boneEditable()))return;
        if(index<10) {
            const float minimum=1.f/float(std::max<std::size_t>(2,document->clip.frames.size()-1));float in=get(0),out=get(1);
            if(out-in<minimum){if(index==0)in=std::max(0.f,out-minimum);else out=std::min(1.f,in+minimum);put(0,in);put(1,out);}
            options.trimIn=in*document->clip.duration;options.trimOut=out*document->clip.duration;options.speed=get(2);options.yawDegrees=get(3);
            options.rootOffset={get(4),get(5),get(6)};options.comOffset={get(7),get(8),get(9)};
        }else if(index<16)updateBone();else updateContact(int(index));changed();
    }
    void resetEdits() {
        if(!document)return;options={};setting=true;for(unsigned i=0;i<20;++i)put(i,definitions[i].initial);
        SendMessageW(autoCalibration,BM_SETCHECK,BST_CHECKED,0);SendMessageW(contactEnable,BM_SETCHECK,BST_UNCHECKED,0);SendMessageW(geometryEnable,BM_SETCHECK,BST_UNCHECKED,0);for(const auto item:inPlace)SendMessageW(item,BM_SETCHECK,BST_UNCHECKED,0);setting=false;populateRoute();changed();
    }
    void finished(Completion& result) {
        if(worker.joinable())worker.join();busy=false;
        if(!result.failure.empty()){if(result.work==Work::load)restoreLoadedDocument();availability();if(result.revision==revision||result.work!=Work::edit)error(result.failure);if(result.work==Work::edit&&result.revision!=revision&&!pendingEdit)apply();return;}
        if(result.work==Work::exportZip) {
            const auto value=result.report.value("output",std::string{});message(L"MO2 override created: "+fromUtf8(value)+L"\r\nInstall it after FreeClimb and test the action in game.",L"已生成 MO2 覆盖包："+fromUtf8(value)+L"\r\n安装在 FreeClimb 之后，并在游戏中测试动作。");availability();return;
        }
        if(result.work==Work::load) {
            wholeDocument=result.wholeDocument;wholeSequence=result.wholeAction?result.authoredSequence:nullptr;
            document=std::move(result.document);documentDirection=result.direction;if(documentDirection.empty()&&fc::converter::isWallRunPrimary(document->slot))documentDirection=document->slot;selectDirection(documentDirection);options=result.initialOptions.value_or(fc::ConverterEditOptions{});time=0;editedAfterLoad=result.imported;set(input,document->input.wstring());set(pack,document->pack.wstring());SendMessageW(bone,CB_RESETCONTENT,0,0);
            selectDocumentIdentity(document->slot);if(wholeDocument){setting=true;set(loopBegin,wholeSequence?number(wholeSequence->actualCuts.loopBegin,4):L"");set(loopEnd,wholeSequence?number(wholeSequence->actualCuts.loopEnd,4):L"");setting=false;stageEditing=false;previewChoice();}
            for(const auto& name:document->base.names){const auto label=fromUtf8(name);SendMessageW(bone,CB_ADDSTRING,0,reinterpret_cast<LPARAM>(label.c_str()));}
            selectedBone=selectedContact=selectedKnot=0;SendMessageW(bone,CB_SETCURSEL,0,0);setting=true;const auto values=editValues(*document,options);for(unsigned i=0;i<20;++i)put(i,values[i]);SendMessageW(geometryEnable,BM_SETCHECK,BST_UNCHECKED,0);
            SendMessageW(autoCalibration,BM_SETCHECK,options.autoCalibration?BST_CHECKED:BST_UNCHECKED,0);SendMessageW(contactEnable,BM_SETCHECK,BST_UNCHECKED,0);const std::array<float,3> axes{options.makeInPlaceAxes.x,options.makeInPlaceAxes.y,options.makeInPlaceAxes.z};for(unsigned i=0;i<3;++i)SendMessageW(inPlace[i],BM_SETCHECK,axes[i]!=0?BST_CHECKED:BST_UNCHECKED,0);setting=false;
        }
        if(result.revision!=revision){if(!pendingEdit)apply();return;}
        if(result.wholeAction)wholeSequence=result.authoredSequence;
        const float phase=time/std::max(previewDuration(),.001f);animation=std::move(result.animation);runtimePreview=std::move(result.runtimePreview);fullPreview=std::move(result.fullPreview);displayRevision=result.revision;displayOptions=options;
        time=result.work==Work::load?0:std::clamp(phase,0.f,1.f)*previewDuration();contacts=contactFrames(*animation);
        markerSummary();
        std::wstring warnings;for(const auto& warning:animation->warnings)warnings+=L" · "+fromUtf8(warning);if(result.authoredSequence)for(const auto& warning:result.authoredSequence->warnings)if(std::find(animation->warnings.begin(),animation->warnings.end(),warning)==animation->warnings.end())warnings+=L" · "+fromUtf8(warning);
        const bool external=fc::converterSourceMotionBaked(animation->clip);
        const std::wstring en=external?L"Source movement baked once. · ":L"";
        const std::wstring zh=external?L"源位移已烘焙一次。· ":L"";
        const bool sourcePath=document->authored&&sourceTravel();
        const auto timingEn=document->authored?std::wstring(sourcePath?L"Source timing / Root movement":L"Source timing / in-place"):L"Timing match "+number(animation->confidence*100,0)+L"%";
        const auto timingZh=document->authored?std::wstring(sourcePath?L"原动作时序 / Root 位移":L"原动作时序 / 原地动作"):L"时序匹配 "+number(animation->confidence*100,0)+L"%";
        message(en+L"Ready · "+std::to_wstring(animation->clip.frames.size())+L" frames · "+number(animation->clip.duration)+L"s · "+timingEn+warnings,
            zh+L"就绪 · "+std::to_wstring(animation->clip.frames.size())+L" 帧 · "+number(animation->clip.duration)+L"秒 · "+timingZh+warnings);
        if(result.work==Work::load) {
            resetView();if(const auto world=view::worldPose(fc::sampleConverterEditor(*animation,0),document->base.parents)){
                reference.wallY=(document->base.palm(*world,0).y+document->base.palm(*world,1).y)*.5f;reference.ledgeZ=(document->base.palm(*world,0).z+document->base.palm(*world,1).z)*.5f;
                put(20,reference.wallY);put(21,reference.ledgeZ);
            }reference.wall=reference.ledge=!document->authored;SendMessageW(wall,BM_SETCHECK,reference.wall?BST_CHECKED:BST_UNCHECKED,0);SendMessageW(ledge,BM_SETCHECK,reference.ledge?BST_CHECKED:BST_UNCHECKED,0);translate();selectContact();
        }populateRoute();availability();invalidate();
        if(!selectionMatchesDocument())message(L"Preview updated for the loaded document. The selected base pack differs; reload it before exporting.",L"已更新当前文档预览。所选基础包不同，请重新载入后导出。");
        else if(!result.previewWarning.empty())message(L"Source animation is editable. Route preview unavailable: "+fromUtf8(result.previewWarning),L"源动作仍可编辑，路线预览不可用："+fromUtf8(result.previewWarning));
        else if(!groupReady())message(L"An action stage has invalid edits. Correct it to restore route previews and group export.",L"某个动作阶段有无效修改，请修正后再使用路线预览及动作组导出。");
    }
    std::string outputTarget()const{return chosenPrimary()=="wallRun"&&document->slot!="sideBrace"?chosenDirection():chosenSlot();}
    void rememberOutput(const std::filesystem::path& output){lastOutput=output;lastOutputInput=document->input;lastOutputTarget=outputTarget();}
    std::filesystem::path suggestedOutput()const {
        const auto target=outputTarget();if(!lastOutput.empty()&&samePath(lastOutputInput,document->input)&&lastOutputTarget==target)return lastOutput;
        auto directory=lastOutput.empty()?document->input.parent_path():lastOutput.parent_path();const auto root=document->pack.parent_path();
        for(auto parent=directory;!parent.empty();parent=parent.parent_path()){if(samePath(parent,root)){directory=root.parent_path();break;}if(parent==parent.parent_path())break;}
        return directory/(document->input.stem().wstring()+L"-FreeClimb-"+fromUtf8(target)+L".zip");
    }
    void exportCurrent() {
        if(busy||!animation||revision!=displayRevision||!selectionMatchesDocument()||!groupReady()||(wholeDocument&&!wholeSequence))return;
        const auto output=browse(true,chinese?L"MO2 覆盖包 (*.zip)\0*.zip\0\0":L"MO2 override (*.zip)\0*.zip\0\0",L"zip",suggestedOutput());if(!output)return;
        if(_wcsicmp(output->extension().c_str(),L".zip")){message(L"The output must be a ZIP file.",L"输出必须是 ZIP 文件。");return;}
        const bool overwrite=regularFile(*output);
        if(overwrite&&MessageBoxW(window,(std::wstring(text(L"Replace this existing ZIP?\r\n",L"替换现有 ZIP 吗？\r\n"))+output->wstring()).c_str(),L"FreeClimb",MB_YESNO|MB_ICONQUESTION|MB_DEFBUTTON2)!=IDYES)return;
        rememberOutput(*output);const auto doc=document;const auto edited=animation;playing=false;set(play,text(L"Play",L"播放"));
        message(L"Exporting the selected action…",L"正在导出当前动作…");
        if(fc::converter::isActionGroupPrimary(chosenPrimary())&&document->slot!="sideBrace"){
            auto selected=exportStages();startWorker(Work::exportZip,[selected=std::move(selected),destination=*output,overwrite](Completion& result){std::vector<fc::ConverterEditorExport> exports;for(const auto& item:selected)exports.push_back({item.document.get(),item.animation.get(),item.direction});result.report=fc::exportConverterEditorGroup(exports,destination,overwrite);});
        }else startWorker(Work::exportZip,[doc,edited,destination=*output,overwrite](Completion& result){result.report=fc::exportConverterEditor(*doc,*edited,destination,overwrite);});
    }
    void seek(float seconds){if(animation){time=std::clamp(seconds,0.f,previewDuration());invalidate();}}
    void step(int change){playing=false;set(play,text(L"Play",L"播放"));if(animation)seek(time+float(change)*(activePreview()?1.f/60:animation->clip.duration/float(std::max<std::size_t>(1,animation->clip.frames.size()-1))));}
    void tick(){const auto now=GetTickCount64();const float elapsed=float(std::min<ULONGLONG>(now-lastTick,100))*.001f;lastTick=now;if(playing&&animation){time+=elapsed;if(time>=previewDuration()){time=previewDuration();playing=false;set(play,text(L"Play",L"播放"));}invalidate();}}
    void paint(HWND target,HDC destination) {
        RECT area{};GetClientRect(target,&area);if(area.right<1||area.bottom<1)return;auto& buffer=target==preview?previewCanvas:timelineCanvas;if(!buffer||buffer->width()!=area.right||buffer->height()!=area.bottom)buffer=std::make_unique<view::Canvas>(area.right,area.bottom);auto& canvas=*buffer;const auto old=SelectObject(canvas.dc(),font);
        const float phase=animation?time/std::max(previewDuration(),.001f):0;
        if(target==preview) {
            const auto pose=previewPose(time);const auto weights=activePreview()?fc::sampleConverterActionPreview(*activePreview(),time).contacts:animation?fc::sampleConverterContacts(*animation,time):std::array<float,4>{};auto shownReference=reference;
            if(adaptedPreview()){shownReference.wallY=activePreview()->wallDistance;shownReference.ledge=mantlePreview();shownReference.ledgeZ=activePreview()->ledgeHeight.value_or(reference.ledgeZ);shownReference.bounds=previewWallBounds;}
            view::drawSkeleton(canvas.dc(),area,pose,document?document->base.parents:std::vector<int>{},document?document->base.names:std::vector<std::string>{},camera,shownReference,weights,page==1?selectedBone:-1,chinese,adaptedPreview(),sequencePreview());
        }else {
            RECT contactArea=area;std::vector<std::array<float,4>> shownContacts;const auto shown=activePreview();
            if(shown){shownContacts.reserve(shown->frames.size());for(const auto& frame:shown->frames)shownContacts.push_back(frame.contacts);}
            if(sequencePreview()) {
                contactArea.top+=MulDiv(30,dpi,96);view::background(canvas.dc(),area,theme::panel);
                const auto layout=view::timelineLayout(canvas.dc(),contactArea,chinese);const int width=layout.right-layout.left;
                for(const auto& segment:shown->segments){const int left=layout.left+int(segment.begin/shown->seconds*width),right=layout.left+int(segment.end/shown->seconds*width);const RECT box{left,area.top+3,right,contactArea.top-2};view::background(canvas.dc(),box,time>=segment.begin&&time<=segment.end?theme::hover:theme::field);view::label(canvas.dc(),box,stageCaption(segment),theme::text,DT_CENTER|DT_SINGLELINE|DT_VCENTER|DT_END_ELLIPSIS);view::stroke(canvas.dc(),{left,box.top},{left,box.bottom},theme::border);}
            }
            view::drawTimeline(canvas.dc(),contactArea,shown?shownContacts:contacts,phase,0,1,chinese);
            if(page==2&&(!sequencePreview()||wholeDocument)) {
                const auto roles=windowRoles();const int channel=roles.empty()?selectedContact:roles[std::size_t(selectedContact)].channel;
                const auto phaseWindow=SendMessageW(contactEnable,BM_GETCHECK,0,0)==BST_CHECKED?std::array<float,2>{get(16),get(17)}:currentWindow();
                const auto layout=view::timelineLayout(canvas.dc(),contactArea,chinese);const int y=layout.top+channel*layout.row;
                for(const float marker:phaseWindow){const int x=layout.left+int(previewPhase(marker)*float(layout.right-layout.left));view::stroke(canvas.dc(),{x,y-3},{x,y+layout.row},RGB(255,249,205),3);view::circle(canvas.dc(),{x,y+layout.row/2},5,RGB(255,249,205));}
            }
        }BitBlt(destination,0,0,area.right,area.bottom,canvas.dc(),0,0,SRCCOPY);SelectObject(canvas.dc(),old);
    }
    void timelineMove(POINT point,bool begin) {
        if(!animation)return;RECT area{};GetClientRect(timeline,&area);if(sequencePreview())area.top+=MulDiv(30,dpi,96);const auto dc=GetDC(timeline);const auto old=SelectObject(dc,font);const auto layout=view::timelineLayout(dc,area,chinese);SelectObject(dc,old);ReleaseDC(timeline,dc);
        const float phase=std::clamp(float(point.x-layout.left)/float(std::max(1,layout.right-layout.left)),0.f,1.f);
        if(begin) {
            playing=false;set(play,text(L"Play",L"播放"));const int channel=std::clamp<int>((point.y-layout.top)/layout.row,0,3);const auto roles=windowRoles();const int selectedChannel=roles.empty()?selectedContact:roles[std::size_t(selectedContact)].channel;
            if(page==2&&(!sequencePreview()||wholeDocument)&&channel==selectedChannel&&(!busy||currentWork==Work::edit)) {
                if(SendMessageW(contactEnable,BM_GETCHECK,0,0)!=BST_CHECKED&&!roles.empty()){const auto phaseWindow=currentWindow();put(16,phaseWindow[0]);put(17,phaseWindow[1]);}
                const float a=std::abs(phase-previewPhase(get(16))),b=std::abs(phase-previewPhase(get(17)));timelineDrag=std::min(a,b)<.06f?(a<=b?1:2):0;
                if(timelineDrag)SendMessageW(contactEnable,BM_SETCHECK,BST_CHECKED,0);
            }else timelineDrag=0;
        }
        if(timelineDrag) {
            const auto contactPhase=sourcePhase(phase);if(timelineDrag==1)put(16,std::min(contactPhase,get(17)));else put(17,std::max(contactPhase,get(16)));updateContact(timelineDrag==1?16:17);changed();
        }else seek(phase*previewDuration());
    }
    void drop(HDROP handle) {
        const UINT count=DragQueryFileW(handle,0xFFFFFFFF,nullptr,0);std::vector<wchar_t> value(32768);const UINT length=count==1?DragQueryFileW(handle,0,value.data(),UINT(value.size())):0;DragFinish(handle);
        if(busy)return;if(count!=1||!length||length>=value.size()||_wcsicmp(std::filesystem::path(value.data()).extension().c_str(),L".hkx")){message(L"Drop one HKX animation.",L"请拖入一个 HKX 动作。");return;}load(value.data());
    }
};
LRESULT CALLBACK themedProcedure(HWND window,UINT message,WPARAM wParam,LPARAM lParam,UINT_PTR,DWORD_PTR data) {
    auto* app=reinterpret_cast<App*>(data);wchar_t name[64]{};GetClassNameW(window,name,64);const std::wstring_view type(name);
    const bool styled=type==L"Button"||type==L"ComboBox"||type==L"Static"||type==TRACKBAR_CLASSW||type==WC_TABCONTROLW||window==app->input||window==app->pack;
    if(message==WM_MOUSEMOVE){if(!GetPropW(window,L"FreeClimbThemeHover")){SetPropW(window,L"FreeClimbThemeHover",reinterpret_cast<HANDLE>(1));TRACKMOUSEEVENT tracking{sizeof(tracking),TME_LEAVE,window,0};TrackMouseEvent(&tracking);InvalidateRect(window,nullptr,FALSE);}
        if(type==WC_TABCONTROLW){TCHITTESTINFO hit{{GET_X_LPARAM(lParam),GET_Y_LPARAM(lParam)},0};const int selected=TabCtrl_HitTest(window,&hit)+1;if(INT_PTR(GetPropW(window,L"FreeClimbHoverTab"))!=selected){SetPropW(window,L"FreeClimbHoverTab",reinterpret_cast<HANDLE>(INT_PTR(selected)));InvalidateRect(window,nullptr,FALSE);}}}
    if(message==WM_MOUSELEAVE){RemovePropW(window,L"FreeClimbThemeHover");RemovePropW(window,L"FreeClimbHoverTab");InvalidateRect(window,nullptr,FALSE);}
    if(message==WM_ERASEBKGND&&styled)return 1;
    if((message==WM_PRINT||message==WM_PRINTCLIENT)&&styled&&type!=L"Button"&&type!=TRACKBAR_CLASSW){theme::control(window,reinterpret_cast<HDC>(wParam),app->dpi,window==app->exportZip,window==app->input||window==app->pack);return 0;}
    if(message==WM_PAINT&&styled&&type!=L"Button"&&type!=TRACKBAR_CLASSW){PAINTSTRUCT paint{};const auto dc=BeginPaint(window,&paint);theme::control(window,dc,app->dpi,window==app->exportZip,window==app->input||window==app->pack);EndPaint(window,&paint);return 0;}
    if(message==WM_NCDESTROY){RemovePropW(window,L"FreeClimbOwnerDraw");RemovePropW(window,L"FreeClimbThemeHover");RemovePropW(window,L"FreeClimbHoverTab");RemoveWindowSubclass(window,themedProcedure,2);}
    if(message==BM_SETSTYLE&&GetPropW(window,L"FreeClimbOwnerDraw"))wParam=(wParam&~WPARAM(BS_TYPEMASK))|BS_OWNERDRAW;
    if(message==TBM_SETPOS&&type==TRACKBAR_CLASSW&&!(GetWindowLongPtrW(window,GWL_STYLE)&WS_VISIBLE))wParam=FALSE;
    const auto result=DefSubclassProc(window,message,wParam,lParam);
    if(message==WM_NCPAINT&&GetWindowLongPtrW(window,GWL_STYLE)&WS_VSCROLL){const auto dc=GetWindowDC(window);theme::scrollbar(window,dc);ReleaseDC(window,dc);}
    if((GetWindowLongPtrW(window,GWL_STYLE)&WS_VISIBLE)&&(message==WM_ENABLE||message==WM_SETFOCUS||message==WM_KILLFOCUS||message==WM_LBUTTONDOWN||message==WM_LBUTTONUP||message==WM_CAPTURECHANGED||message==WM_CANCELMODE||message==WM_KEYDOWN||message==WM_KEYUP||message==WM_THEMECHANGED||message==WM_SYSCOLORCHANGE||message==BM_SETSTYLE||message==BM_SETSTATE||message==BM_SETCHECK||message==CB_SETCURSEL||message==TBM_SETPOS))InvalidateRect(window,nullptr,FALSE);
    return result;
}
LRESULT CALLBACK panelChildProcedure(HWND window,UINT message,WPARAM wParam,LPARAM lParam,UINT_PTR,DWORD_PTR data) {
    if(message==WM_SETFOCUS)reinterpret_cast<App*>(data)->revealControl(window);
    if(message==WM_NCDESTROY)RemoveWindowSubclass(window,panelChildProcedure,1);
    return DefSubclassProc(window,message,wParam,lParam);
}
LRESULT CALLBACK panelProcedure(HWND window,UINT message,WPARAM wParam,LPARAM lParam) {
    auto* app=reinterpret_cast<App*>(GetWindowLongPtrW(window,GWLP_USERDATA));
    if(message==WM_NCCREATE){app=static_cast<App*>(reinterpret_cast<CREATESTRUCTW*>(lParam)->lpCreateParams);SetWindowLongPtrW(window,GWLP_USERDATA,reinterpret_cast<LONG_PTR>(app));}
    if(!app)return DefWindowProcW(window,message,wParam,lParam);
    if(message==WM_COMMAND||message==WM_HSCROLL||message==WM_NOTIFY||message==WM_DRAWITEM||message==WM_MEASUREITEM||(message>=WM_CTLCOLORMSGBOX&&message<=WM_CTLCOLORSTATIC))return SendMessageW(app->window,message,wParam,lParam);
    if(message==WM_VSCROLL){SCROLLINFO info{sizeof(info),SIF_ALL};GetScrollInfo(window,SB_VERT,&info);int position=info.nPos;
        switch(LOWORD(wParam)){case SB_TOP:position=0;break;case SB_BOTTOM:position=info.nMax;break;case SB_LINEUP:position-=24;break;case SB_LINEDOWN:position+=24;break;case SB_PAGEUP:position-=int(info.nPage);break;case SB_PAGEDOWN:position+=int(info.nPage);break;case SB_THUMBPOSITION:case SB_THUMBTRACK:position=info.nTrackPos;break;default:return 0;}
        app->scrollPanel(position);return 0;}
    if(message==WM_MOUSEWHEEL){app->wheelRemainder+=GET_WHEEL_DELTA_WPARAM(wParam);const int steps=app->wheelRemainder/WHEEL_DELTA;app->wheelRemainder%=WHEEL_DELTA;UINT lines=3;SystemParametersInfoW(SPI_GETWHEELSCROLLLINES,0,&lines,0);app->scrollPanel(app->panelScroll[std::size_t(app->page)]-steps*(lines==WHEEL_PAGESCROLL?app->panelViewportHeight:24*int(lines)));return 0;}
    if(message==WM_PRINTCLIENT||message==WM_ERASEBKGND){RECT area{};GetClientRect(window,&area);FillRect(reinterpret_cast<HDC>(wParam),&area,app->panelBrush);return message==WM_ERASEBKGND?1:0;}
    const auto result=DefWindowProcW(window,message,wParam,lParam);if(message==WM_NCPAINT){const auto dc=GetWindowDC(window);theme::scrollbar(window,dc);ReleaseDC(window,dc);}return result;
}
LRESULT CALLBACK previewProcedure(HWND window,UINT message,WPARAM wParam,LPARAM lParam) {
    auto* app=reinterpret_cast<App*>(GetWindowLongPtrW(window,GWLP_USERDATA));if(!app)return DefWindowProcW(window,message,wParam,lParam);
    const bool preview=window==app->preview;const POINT point{GET_X_LPARAM(lParam),GET_Y_LPARAM(lParam)};
    if(message==WM_ERASEBKGND)return 1;
    if(message==WM_PRINTCLIENT){try{app->paint(window,reinterpret_cast<HDC>(wParam));}catch(...){}return 0;}
    if(message==WM_PAINT){PAINTSTRUCT paint{};const auto dc=BeginPaint(window,&paint);try{app->paint(window,dc);}catch(...){FillRect(dc,&paint.rcPaint,app->canvasBrush);}EndPaint(window,&paint);return 0;}
    if(message==WM_LBUTTONDOWN){SetCapture(window);app->dragging=true;app->dragStart=point;if(!preview)app->timelineMove(point,true);return 0;}
    if(message==WM_LBUTTONUP||message==WM_CAPTURECHANGED){app->dragging=false;app->timelineDrag=0;if(message==WM_LBUTTONUP)ReleaseCapture();return 0;}
    if(message==WM_MOUSEMOVE&&app->dragging) {
        if(preview){const float scale=96.f/app->dpi;app->camera.orbit(float(point.x-app->dragStart.x)*scale,float(point.y-app->dragStart.y)*scale);app->dragStart=point;app->put(22,app->camera.yaw*57.29578f);app->put(23,app->camera.pitch*57.29578f);InvalidateRect(app->preview,nullptr,FALSE);}
        else app->timelineMove(point,false);return 0;
    }
    if(message==WM_MOUSEWHEEL&&preview){app->camera.scale(float(GET_WHEEL_DELTA_WPARAM(wParam))/120);InvalidateRect(app->preview,nullptr,FALSE);return 0;}
    if(message==WM_LBUTTONDBLCLK&&preview){app->resetView();return 0;}return DefWindowProcW(window,message,wParam,lParam);
}
LRESULT CALLBACK procedure(HWND window,UINT message,WPARAM wParam,LPARAM lParam) {
    auto* app=reinterpret_cast<App*>(GetWindowLongPtrW(window,GWLP_USERDATA));
    if(message==WM_CREATE){app=static_cast<App*>(reinterpret_cast<CREATESTRUCTW*>(lParam)->lpCreateParams);app->window=window;SetWindowLongPtrW(window,GWLP_USERDATA,reinterpret_cast<LONG_PTR>(app));app->initialize();return 0;}
    if(!app)return DefWindowProcW(window,message,wParam,lParam);
    if(message==WM_ERASEBKGND){RECT area{};GetClientRect(window,&area);FillRect(reinterpret_cast<HDC>(wParam),&area,app->canvasBrush);return 1;}
    if(message>=WM_CTLCOLORMSGBOX&&message<=WM_CTLCOLORSTATIC){const auto dc=reinterpret_cast<HDC>(wParam);const auto item=reinterpret_cast<HWND>(lParam);SetTextColor(dc,IsWindowEnabled(item)?theme::text:theme::disabled);SetBkColor(dc,theme::panel);return reinterpret_cast<LRESULT>(app->panelBrush);}
    if(message==WM_MEASUREITEM){auto* item=reinterpret_cast<MEASUREITEMSTRUCT*>(lParam);if(item->CtlType==ODT_COMBOBOX){item->itemHeight=MulDiv(27,app->dpi,96);return TRUE;}}
    if(message==WM_DRAWITEM&&!(GetWindowLongPtrW(reinterpret_cast<DRAWITEMSTRUCT*>(lParam)->hwndItem,GWL_STYLE)&WS_VISIBLE))return TRUE;
    if(message==WM_DRAWITEM&&reinterpret_cast<DRAWITEMSTRUCT*>(lParam)->CtlType==ODT_TAB){const auto* item=reinterpret_cast<DRAWITEMSTRUCT*>(lParam);const int saved=SaveDC(item->hDC);IntersectClipRect(item->hDC,item->rcItem.left,item->rcItem.top,item->rcItem.right,item->rcItem.bottom);theme::control(item->hwndItem,item->hDC,app->dpi);RestoreDC(item->hDC,saved);return TRUE;}
    if(message==WM_DRAWITEM){const auto* item=reinterpret_cast<DRAWITEMSTRUCT*>(lParam);if(item->CtlType==ODT_BUTTON){theme::control(item->hwndItem,item->hDC,app->dpi,item->hwndItem==app->exportZip,false,item->itemState);return TRUE;}if(item->CtlType==ODT_COMBOBOX){const int saved=SaveDC(item->hDC);SelectObject(item->hDC,app->font);const bool selected=item->itemState&ODS_SELECTED;view::background(item->hDC,item->rcItem,selected?theme::hover:theme::field);auto area=item->rcItem;area.left+=MulDiv(10,app->dpi,96);view::label(item->hDC,area,theme::comboValue(item->hwndItem,int(item->itemID)),item->itemState&ODS_DISABLED?theme::disabled:theme::text,DT_LEFT|DT_VCENTER|DT_SINGLELINE|DT_END_ELLIPSIS|DT_NOPREFIX);RestoreDC(item->hDC,saved);return TRUE;}}
    if(message==WM_NOTIFY&&reinterpret_cast<NMHDR*>(lParam)->code==NM_CUSTOMDRAW) {
        const auto* draw=reinterpret_cast<NMCUSTOMDRAW*>(lParam);wchar_t type[32]{};GetClassNameW(draw->hdr.hwndFrom,type,32);
        if(std::wstring_view(type)==L"Button"||std::wstring_view(type)==TRACKBAR_CLASSW) {
            if(!(GetWindowLongPtrW(draw->hdr.hwndFrom,GWL_STYLE)&WS_VISIBLE))return CDRF_SKIPDEFAULT;
            if(draw->dwDrawStage==CDDS_PREERASE||draw->dwDrawStage==CDDS_PREPAINT){const UINT state=(draw->uItemState&CDIS_SELECTED?ODS_SELECTED:0)|(draw->uItemState&CDIS_FOCUS?ODS_FOCUS:0)|(draw->uItemState&CDIS_DISABLED?ODS_DISABLED:0)|(draw->uItemState&CDIS_HOT?ODS_HOTLIGHT:0);theme::control(draw->hdr.hwndFrom,draw->hdc,app->dpi,draw->hdr.hwndFrom==app->exportZip,false,state);return CDRF_SKIPDEFAULT;}
        }
    }
    if(message==WM_NOTIFY&&reinterpret_cast<NMHDR*>(lParam)->code==TTN_GETDISPINFOW){auto* info=reinterpret_cast<NMTTDISPINFOW*>(lParam);const auto target=reinterpret_cast<HWND>(info->hdr.idFrom);app->tooltipText=fieldText(target==app->packBrowse?app->pack:target);info->lpszText=app->tooltipText.data();return 0;}
    if(message==completed){std::unique_ptr<Completion> result(reinterpret_cast<Completion*>(lParam));app->finished(*result);return 0;}
    if(message==WM_SIZE){app->layout();return 0;}
    if(message==WM_MOUSEWHEEL){RECT area{};GetWindowRect(app->panel,&area);if(PtInRect(&area,{GET_X_LPARAM(lParam),GET_Y_LPARAM(lParam)}))return SendMessageW(app->panel,message,wParam,lParam);}
    if(message==WM_DPICHANGED){app->changeDpi(HIWORD(wParam));const auto* area=reinterpret_cast<RECT*>(lParam);SetWindowPos(window,nullptr,area->left,area->top,area->right-area->left,area->bottom-area->top,SWP_NOZORDER|SWP_NOACTIVATE);return 0;}
    if(message==WM_GETMINMAXINFO){auto* info=reinterpret_cast<MINMAXINFO*>(lParam);MONITORINFO monitor{sizeof(monitor)};GetMonitorInfoW(MonitorFromWindow(window,MONITOR_DEFAULTTONEAREST),&monitor);const auto available=theme::initialSize(app->dpi,monitor.rcWork.right-monitor.rcWork.left,monitor.rcWork.bottom-monitor.rcWork.top);info->ptMinTrackSize={std::min<int>(MulDiv(1030,app->dpi,96),available.cx),std::min<int>(MulDiv(800,app->dpi,96),available.cy)};return 0;}
    if(message==WM_DROPFILES){app->drop(reinterpret_cast<HDROP>(wParam));return 0;}if(message==WM_TIMER){if(wParam==2){if(!app->busy)app->apply();}else app->tick();return 0;}
    if(message==WM_CLOSE&&app->busy){app->message(L"An operation is still running. The preview remains available while it finishes.",L"操作正在进行，完成前仍可查看预览。");return 0;}
    if(message==WM_DESTROY){KillTimer(window,1);KillTimer(window,2);PostQuitMessage(0);return 0;}
    if(message==WM_NOTIFY&&reinterpret_cast<NMHDR*>(lParam)->idFrom==tabId&&reinterpret_cast<NMHDR*>(lParam)->code==TCN_SELCHANGE){app->page=TabCtrl_GetCurSel(app->tabs);app->translate();return 0;}
    if(message==WM_HSCROLL) {
        const auto item=reinterpret_cast<HWND>(lParam);
        if(item==app->scrub){app->playing=false;app->set(app->play,app->text(L"Play",L"播放"));if(app->animation)app->seek(float(SendMessageW(item,TBM_GETPOS,0,0))*.001f*app->previewDuration());return 0;}
        for(unsigned i=0;i<app->sliders.size();++i)if(app->sliders[i].track==item){app->sliderValue(i);app->optionsFromSliders(i);return 0;}
    }
    if(message==WM_COMMAND) {
        const int id=LOWORD(wParam),event=HIWORD(wParam);
        if(id==languageId&&event==CBN_SELCHANGE){app->chinese=SendMessageW(app->language,CB_GETCURSEL,0,0)==1;app->translate();return 0;}
        if(id==slotId&&event==CBN_SELCHANGE&&!app->busy) {
            const bool fromGroup=app->document&&fc::converter::isActionGroupMember(app->document->slot);
            if(app->document&&!fromGroup&&app->editedAfterLoad&&MessageBoxW(window,std::wstring(app->text(L"Changing the target action resets the current edits. Continue?",L"切换目标动作会还原当前编辑，继续吗？")).c_str(),L"FreeClimb",MB_YESNO|MB_ICONQUESTION|MB_DEFBUTTON2)!=IDYES){app->selectDocumentIdentity(app->document->slot);return 0;}
            app->saveStage();app->stageEditing=false;app->sourceComparison=false;app->previewChoice();app->populateStages();app->layout();
            if(app->document){if(fromGroup||fc::converter::isActionGroupPrimary(app->chosenPrimary())){if(!app->restoreStage(app->chosenSlot()))app->loadBase();}else app->load(app->document->input);}else app->translate();return 0;
        }
        if(id==stageEditingId&&!app->busy){app->saveStage();app->stageEditing=SendMessageW(app->stageEditingToggle,BM_GETCHECK,0,0)==BST_CHECKED;app->sourceComparison=false;app->previewChoice();app->time=0;app->playing=false;if(app->chosenPrimary()=="wallRun"){app->populateStages(app->stageEditing?"sideBrace":app->chosenDirection());if(!app->restoreStage(app->chosenSlot()))app->loadBase();}else {app->translate();app->resetView();}return 0;}
        if(id==sourceComparisonId&&!app->busy){app->sourceComparison=SendMessageW(app->sourceComparisonToggle,BM_GETCHECK,0,0)==BST_CHECKED;app->previewChoice();app->time=0;app->playing=false;app->availability();app->layout();app->resetView();return 0;}
        if(id==stageId&&event==CBN_SELCHANGE&&!app->busy) {
            app->saveStage();app->stageEditing=true;app->sourceComparison=false;app->previewChoice();if(!app->restoreStage(app->chosenSlot()))app->loadBase();return 0;
        }
        if((id==loopBeginId||id==loopEndId)&&event==EN_CHANGE&&!app->setting&&app->wholeDocument){app->changed(true);return 0;}
        if(id==loopBeginHereId||id==loopEndHereId){app->markLoopBoundary(id==loopEndHereId);return 0;}
        if(id==openId){if(const auto source=app->browse(false,L"Skyrim HKX (*.hkx)\0*.hkx\0\0",L"hkx"))app->load(*source);return 0;}
        if(id==baseId){app->loadBase();return 0;}
        if(id==packBrowseId){if(!app->busy)if(const auto source=app->browse(false,L"FreeClimb pack.json\0*.json\0\0",L"json",fieldText(app->pack)))app->selectPack(*source);return 0;}
        if(id==exportId){app->exportCurrent();return 0;}
        if(id==playId){if(app->animation){if(app->time>=app->previewDuration())app->time=0;app->playing=!app->playing;app->lastTick=GetTickCount64();app->set(app->play,app->playing?app->text(L"Pause",L"暂停"):app->text(L"Play",L"播放"));}return 0;}
        if(id==previousId||id==nextId){app->step(id==previousId?-1:1);return 0;}if(id==resetId){app->resetEdits();return 0;}
        if(id==inHereId||id==outHereId){if(app->document&&app->animation){const float sourceTime=app->sourceTime();app->put(id==inHereId?0:1,sourceTime/app->document->clip.duration);app->optionsFromSliders(id==inHereId?0:1);}return 0;}
        if(id==boneId&&event==CBN_SELCHANGE){app->selectBone();return 0;}
        if(id==clearBoneId){if(!app->boneEditable())return 0;for(unsigned i=10;i<13;++i)app->put(i,0);app->updateBone();app->changed();return 0;}
        if(id==contactId&&event==CBN_SELCHANGE){app->selectContact();return 0;}if(id==contactEnableId){app->updateContact();app->changed();return 0;}
        if(id==autoId){app->options.autoCalibration=SendMessageW(app->autoCalibration,BM_GETCHECK,0,0)==BST_CHECKED;app->changed();return 0;}
        if(id>=inPlaceXId&&id<=inPlaceZId){app->options.makeInPlaceAxes={SendMessageW(app->inPlace[0],BM_GETCHECK,0,0)==BST_CHECKED?1.f:0.f,SendMessageW(app->inPlace[1],BM_GETCHECK,0,0)==BST_CHECKED?1.f:0.f,SendMessageW(app->inPlace[2],BM_GETCHECK,0,0)==BST_CHECKED?1.f:0.f};app->changed();return 0;}
        if(id==wallId||id==ledgeId||id==jointsId){app->reference.wall=SendMessageW(app->wall,BM_GETCHECK,0,0)==BST_CHECKED;app->reference.ledge=SendMessageW(app->ledge,BM_GETCHECK,0,0)==BST_CHECKED;app->reference.joints=SendMessageW(app->joints,BM_GETCHECK,0,0)==BST_CHECKED;app->invalidate();return 0;}
        if(id==fitId){app->resetView();return 0;}
        if(id==advancedId){app->advancedMotion=!app->advancedMotion;app->set(app->advanced,app->advancedMotion?app->text(L"− More adjustments",L"− 更多调整"):app->text(L"+ More adjustments",L"+ 更多调整"));app->layout();return 0;}
        if(id==geometryId){app->toggleGeometry();return 0;}if(id==knotId&&event==CBN_SELCHANGE){app->selectKnot();return 0;}if(id==simplifyId){app->simplifyRoute();return 0;}
    }return DefWindowProcW(window,message,wParam,lParam);
}
bool registerClasses(HINSTANCE instance) {
    WNDCLASSEXW preview{sizeof(preview)};preview.lpfnWndProc=previewProcedure;preview.hInstance=instance;preview.hCursor=LoadCursorW(nullptr,IDC_HAND);preview.lpszClassName=L"FreeClimbPreview";preview.style=CS_DBLCLKS;
    if(!RegisterClassExW(&preview))return false;WNDCLASSEXW panel{sizeof(panel)};panel.lpfnWndProc=panelProcedure;panel.hInstance=instance;panel.hCursor=LoadCursorW(nullptr,IDC_ARROW);panel.hbrBackground=reinterpret_cast<HBRUSH>(GetStockObject(BLACK_BRUSH));panel.lpszClassName=L"FreeClimbEditorPanel";
    if(!RegisterClassExW(&panel))return false;WNDCLASSEXW main{sizeof(main)};main.lpfnWndProc=procedure;main.hInstance=instance;main.hCursor=LoadCursorW(nullptr,IDC_ARROW);
    main.hbrBackground=reinterpret_cast<HBRUSH>(GetStockObject(BLACK_BRUSH));main.lpszClassName=L"FreeClimbConverterGui";return RegisterClassExW(&main)!=0;
}
HWND bottomChild(HWND parent){const auto first=GetWindow(parent,GW_CHILD);return first?GetWindow(first,GW_HWNDLAST):nullptr;}
void printControl(HWND item,HWND parent,HDC dc) {
    if(!(GetWindowLongPtrW(item,GWL_STYLE)&WS_VISIBLE))return;RECT area{};GetWindowRect(item,&area);MapWindowPoints(nullptr,parent,reinterpret_cast<POINT*>(&area),2);
    const int saved=SaveDC(dc);IntersectClipRect(dc,area.left,area.top,area.right,area.bottom);SetViewportOrgEx(dc,area.left,area.top,nullptr);
    SendMessageW(item,WM_PRINT,reinterpret_cast<WPARAM>(dc),PRF_CLIENT|PRF_NONCLIENT|PRF_ERASEBKGND);
    wchar_t type[48]{};GetClassNameW(item,type,48);
    theme::scrollbar(item,dc);RestoreDC(dc,saved);
    if(std::wstring_view(type)==L"FreeClimbEditorPanel") {
        RECT client{};GetClientRect(item,&client);MapWindowPoints(item,parent,reinterpret_cast<POINT*>(&client),2);const int clipped=SaveDC(dc);IntersectClipRect(dc,client.left,client.top,client.right,client.bottom);
        for(HWND child=bottomChild(item);child;child=GetWindow(child,GW_HWNDPREV))printControl(child,parent,dc);RestoreDC(dc,clipped);
    }
}
Json inspectPanelVisibility(const App& app) {
    bool inFront=false;for(HWND item=GetWindow(app.window,GW_CHILD);item;item=GetWindow(item,GW_HWNDNEXT)){if(item==app.panel){inFront=true;break;}if(item==app.tabs)break;}
    RECT client{};GetClientRect(app.panel,&client);bool hitPanel=client.right>12&&client.bottom>12;
    for(const int x:{6,int(client.right/2),int(client.right-6)})for(const int y:{6,int(client.bottom/2),int(client.bottom-6)}) {
        POINT point{x,y};MapWindowPoints(app.panel,app.window,&point,1);hitPanel=hitPanel&&ChildWindowFromPointEx(app.window,point,CWP_ALL)==app.panel;
    }
    RECT header{};const bool itemFound=TabCtrl_GetItemRect(app.tabs,app.page,&header)!=FALSE;POINT point{(header.left+header.right)/2,(header.top+header.bottom)/2};MapWindowPoints(app.tabs,app.window,&point,1);
    return {{"panelInFrontOfTabs",inFront},{"panelHitTests",hitPanel},{"tabHeaderHitTest",itemFound&&ChildWindowFromPointEx(app.window,point,CWP_ALL)==app.tabs},
        {"siblingClipping",bool((GetWindowLongPtrW(app.tabs,GWL_STYLE)&GetWindowLongPtrW(app.panel,GWL_STYLE))&WS_CLIPSIBLINGS)},
        {"panelClipsChildren",bool(GetWindowLongPtrW(app.panel,GWL_STYLE)&WS_CLIPCHILDREN)}};
}
Json inspectControls(HWND parent,HWND root,const App& app) {
    Json controls=Json::array();
    for(HWND item=GetWindow(parent,GW_CHILD);item;item=GetWindow(item,GW_HWNDNEXT))if(GetWindowLongPtrW(item,GWL_STYLE)&WS_VISIBLE) {
        RECT area{},client{};GetWindowRect(item,&area);MapWindowPoints(nullptr,root,reinterpret_cast<POINT*>(&area),2);GetClientRect(item,&client);
        wchar_t type[48]{};GetClassNameW(item,type,48);const std::wstring_view name(type);const auto value=fieldText(item);const auto style=GetWindowLongPtrW(item,GWL_STYLE);
        Json entry{{"id",GetDlgCtrlID(item)},{"class",toUtf8(name)},{"text",toUtf8(value)},{"parent",parent==app.panel?"panel":"window"},{"rect",{area.left,area.top,area.right,area.bottom}},{"clientSize",{client.right,client.bottom}}};
        if(item==app.slot||item==app.stage){const auto selected=SendMessageW(item,CB_GETCURSEL,0,0);std::array<wchar_t,160> label{};SendMessageW(item,CB_GETLBTEXT,WPARAM(selected),reinterpret_cast<LPARAM>(label.data()));const auto dc=GetDC(item);const auto old=SelectObject(dc,reinterpret_cast<HFONT>(SendMessageW(item,WM_GETFONT,0,0)));SIZE size{};GetTextExtentPoint32W(dc,label.data(),int(wcslen(label.data())),&size);entry["selectedText"]=toUtf8(label.data());entry["textExtent"]={size.cx,size.cy};entry["textFits"]=size.cx<=client.right-MulDiv(28,app.dpi,96)&&size.cy<=client.bottom;SelectObject(dc,old);ReleaseDC(item,dc);}
        if(name==L"Static"||name==L"Button") {
            const auto dc=GetDC(item);const auto font=reinterpret_cast<HFONT>(SendMessageW(item,WM_GETFONT,0,0));const auto old=SelectObject(dc,font);TEXTMETRICW metrics{};GetTextMetricsW(dc,&metrics);
            const bool checkbox=name==L"Button"&&((style&BS_TYPEMASK)==BS_AUTOCHECKBOX);const bool ellipsis=name==L"Static"&&(style&SS_ENDELLIPSIS)==SS_ENDELLIPSIS;const bool wrapping=(name==L"Static"&&!ellipsis)||(style&BS_MULTILINE);
            const int textWidth=std::max(1,int(client.right)-(name==L"Button"?MulDiv(checkbox?24:12,app.dpi,96):0));RECT textArea{0,0,textWidth,0};
            DrawTextW(dc,value.c_str(),int(value.size()),&textArea,DT_CALCRECT|DT_NOPREFIX|(wrapping?DT_WORDBREAK:DT_SINGLELINE));
            entry["textExtent"]={textArea.right,textArea.bottom};entry["textWidth"]=textWidth;entry["fontHeight"]=metrics.tmHeight;entry["ellipsized"]=ellipsis;entry["textFits"]=(ellipsis||textArea.right<=textWidth)&&textArea.bottom<=client.bottom;
            SelectObject(dc,old);ReleaseDC(item,dc);
        }
        if(name==TRACKBAR_CLASSW){RECT thumb{},channel{};SendMessageW(item,TBM_GETTHUMBRECT,0,reinterpret_cast<LPARAM>(&thumb));SendMessageW(item,TBM_GETCHANNELRECT,0,reinterpret_cast<LPARAM>(&channel));entry["thumbRect"]={thumb.left,thumb.top,thumb.right,thumb.bottom};entry["channelRect"]={channel.left,channel.top,channel.right,channel.bottom};}
        if(item==app.timeline){const auto dc=GetDC(item);const auto old=SelectObject(dc,app.font);auto content=client;if(app.sequencePreview())content.top+=MulDiv(30,app.dpi,96);const auto layout=view::timelineLayout(dc,content,app.chinese);Json labels=Json::array();const auto names=view::timelineLabels(app.chinese);
            for(unsigned i=0;i<names.size();++i){SIZE size{};GetTextExtentPoint32W(dc,names[i].data(),int(names[i].size()),&size);const int top=layout.top+int(i)*layout.row;labels.push_back({{"text",toUtf8(names[i])},{"rect",{layout.labelLeft,top,layout.labelRight,top+layout.row}},{"textExtent",{size.cx,size.cy}},{"textFits",size.cx<=layout.labelRight-layout.labelLeft&&size.cy<=layout.row&&top+layout.row<=client.bottom}});}
            entry["timeline"]={{"left",layout.left},{"right",layout.right},{"top",layout.top},{"row",layout.row},{"labels",labels},{"seconds",app.previewDuration()},{"complete",app.sequencePreview()}};if(app.sequencePreview()){entry["timeline"]["stages"]=Json::array();for(const auto& segment:app.activePreview()->segments)entry["timeline"]["stages"].push_back({{"begin",segment.begin},{"end",segment.end},{"slot",segment.slot},{"label",toUtf8(app.stageCaption(segment))}});}SelectObject(dc,old);ReleaseDC(item,dc);}
        if(item==app.preview){const auto dc=GetDC(item);const auto old=SelectObject(dc,app.font);Json captions=Json::array();const auto values=view::previewCaptions(app.chinese,app.adaptedPreview(),app.sequencePreview());
            for(unsigned i=0;i<values.size();++i){const auto bounds=view::previewCaptionRect(dc,client,values[i],i==1);RECT required{0,0,bounds.right-bounds.left,0};DrawTextW(dc,values[i].data(),int(values[i].size()),&required,DT_CALCRECT|DT_WORDBREAK|DT_NOPREFIX);captions.push_back({{"text",toUtf8(values[i])},{"rect",{bounds.left,bounds.top,bounds.right,bounds.bottom}},{"textExtent",{required.right,required.bottom}},{"textFits",required.right<=bounds.right-bounds.left&&required.bottom<=bounds.bottom-bounds.top}});}
            entry["previewCaptions"]=captions;SelectObject(dc,old);ReleaseDC(item,dc);}
        if(parent==app.panel){RECT bounds{};GetClientRect(parent,&bounds);MapWindowPoints(parent,root,reinterpret_cast<POINT*>(&bounds),2);entry["viewport"]={bounds.left,bounds.top,bounds.right,bounds.bottom};entry["contentRect"]={area.left-bounds.left,area.top-bounds.top+MulDiv(app.panelScroll[std::size_t(app.page)],app.dpi,96),area.right-bounds.left,area.bottom-bounds.top+MulDiv(app.panelScroll[std::size_t(app.page)],app.dpi,96)};}
        if(item==app.panel)entry["children"]=inspectControls(item,root,app);controls.push_back(std::move(entry));
    }return controls;
}
int verifyState(int count,wchar_t** args,HINSTANCE instance,const std::filesystem::path& executable) {
    if(count!=4)return 2;App app;HWND window{};unsigned checks{};
    try {
        auto check=[&](bool valid,const char* label){++checks;if(!valid)throw std::runtime_error(label);};
        app.executable=executable;window=CreateWindowExW(0,L"FreeClimbConverterGui",L"FreeClimb state verification",WS_OVERLAPPEDWINDOW|WS_CLIPCHILDREN,0,0,1280,800,nullptr,nullptr,instance,&app);
        check(window!=nullptr,"hidden editor created");KillTimer(window,1);app.selectPack(args[2]);
        auto wait=[&]{const auto started=GetTickCount64();while(app.busy||app.pendingEdit){MSG message{};while(PeekMessageW(&message,nullptr,0,0,PM_REMOVE)){if(!IsDialogMessageW(window,&message)){TranslateMessage(&message);DispatchMessageW(&message);}}check(GetTickCount64()-started<30000,"editor operation completed within limit");if(app.busy||app.pendingEdit)MsgWaitForMultipleObjects(0,nullptr,FALSE,10,QS_ALLINPUT);}};
        auto click=[&](HWND item,int id){SendMessageW(window,WM_COMMAND,MAKEWPARAM(id,BN_CLICKED),reinterpret_cast<LPARAM>(item));wait();};
        auto slide=[&](unsigned index,float value){app.put(index,value);SendMessageW(app.panel,WM_HSCROLL,TB_THUMBPOSITION,reinterpret_cast<LPARAM>(app.sliders[index].track));};
        click(app.base,baseId);check(app.document&&app.animation&&app.selectionMatchesDocument()&&IsWindowEnabled(app.exportZip),"initial base load is exportable");
        NMTTDISPINFOW tooltip{};tooltip.hdr.idFrom=reinterpret_cast<UINT_PTR>(app.input);tooltip.hdr.code=TTN_GETDISPINFOW;SendMessageW(window,WM_NOTIFY,0,reinterpret_cast<LPARAM>(&tooltip));check(tooltip.lpszText&&std::wstring(tooltip.lpszText)==app.document->input.wstring(),"compact source display retains complete path in tooltip");
        tooltip.hdr.idFrom=reinterpret_cast<UINT_PTR>(app.packBrowse);SendMessageW(window,WM_NOTIFY,0,reinterpret_cast<LPARAM>(&tooltip));check(std::wstring(tooltip.lpszText)==app.document->pack.wstring(),"base-pack tooltip retains exact identity while path field is hidden");
        {
            struct ThemeProbe {
                HWND parent{},button{},checkbox{},track{};unsigned ownerPaints{},customPaints{},trackPaints{},tabPaints{},buttonClicks{},checkClicks{};bool accepted{true};
                static LRESULT CALLBACK route(HWND window,UINT message,WPARAM wParam,LPARAM lParam,UINT_PTR,DWORD_PTR data) {
                    auto& probe=*reinterpret_cast<ThemeProbe*>(data);const auto result=DefSubclassProc(window,message,wParam,lParam);
                    if(message==WM_DRAWITEM&&reinterpret_cast<DRAWITEMSTRUCT*>(lParam)->CtlType==ODT_BUTTON){++probe.ownerPaints;probe.accepted=probe.accepted&&result==TRUE;}
                    if(message==WM_DRAWITEM&&reinterpret_cast<DRAWITEMSTRUCT*>(lParam)->CtlType==ODT_TAB){++probe.tabPaints;probe.accepted=probe.accepted&&result==TRUE;}
                    if(message==WM_NOTIFY&&reinterpret_cast<NMHDR*>(lParam)->code==NM_CUSTOMDRAW&&reinterpret_cast<NMHDR*>(lParam)->hwndFrom==probe.checkbox&&(reinterpret_cast<NMCUSTOMDRAW*>(lParam)->dwDrawStage==CDDS_PREPAINT||reinterpret_cast<NMCUSTOMDRAW*>(lParam)->dwDrawStage==CDDS_PREERASE)){++probe.customPaints;probe.accepted=probe.accepted&&result==CDRF_SKIPDEFAULT;}
                    if(message==WM_NOTIFY&&reinterpret_cast<NMHDR*>(lParam)->code==NM_CUSTOMDRAW&&reinterpret_cast<NMHDR*>(lParam)->hwndFrom==probe.track&&reinterpret_cast<NMCUSTOMDRAW*>(lParam)->dwDrawStage==CDDS_PREPAINT){++probe.trackPaints;probe.accepted=probe.accepted&&result==CDRF_SKIPDEFAULT;}
                    if(message==WM_COMMAND&&HIWORD(wParam)==BN_CLICKED){if(reinterpret_cast<HWND>(lParam)==probe.button)++probe.buttonClicks;if(reinterpret_cast<HWND>(lParam)==probe.checkbox)++probe.checkClicks;}
                    return result;
                }
                ~ThemeProbe(){if(button)DestroyWindow(button);if(checkbox)DestroyWindow(checkbox);if(track)DestroyWindow(track);RemoveWindowSubclass(parent,route,93);}
            } probe;probe.parent=window;check(SetWindowSubclass(window,ThemeProbe::route,93,reinterpret_cast<DWORD_PTR>(&probe))!=FALSE,"native draw observer attached");
            probe.button=app.control(L"BUTTON",BS_PUSHBUTTON|WS_TABSTOP,9501);probe.checkbox=app.control(L"BUTTON",BS_AUTOCHECKBOX|WS_TABSTOP,9502,app.panel);
            probe.track=app.control(TRACKBAR_CLASSW,TBS_NOTICKS|WS_TABSTOP,9503,app.panel);MoveWindow(probe.track,0,0,180,36,FALSE);SendMessageW(probe.track,TBM_SETRANGE,FALSE,MAKELPARAM(0,1000));
            MoveWindow(probe.button,0,0,180,36,FALSE);MoveWindow(probe.checkbox,0,0,180,36,FALSE);SetWindowTextW(probe.button,L"Theme probe");SetWindowTextW(probe.checkbox,L"Theme probe");
            check((GetWindowLongPtrW(probe.button,GWL_STYLE)&BS_TYPEMASK)==BS_OWNERDRAW&&(GetWindowLongPtrW(probe.checkbox,GWL_STYLE)&BS_TYPEMASK)==BS_AUTOCHECKBOX,"push buttons own their painting while checkboxes retain native automatic state");
            auto color=[&](HWND item,int x=-1,int y=6){RECT bounds{};GetClientRect(item,&bounds);view::Canvas painted(bounds.right,bounds.bottom);view::background(painted.dc(),bounds,RGB(255,0,255));SendMessageW(item,WM_PRINTCLIENT,reinterpret_cast<WPARAM>(painted.dc()),PRF_CLIENT);return GetPixel(painted.dc(),x<0?bounds.right/2:x,y);};
            const auto resting=color(probe.button);SendMessageW(probe.button,WM_MOUSEMOVE,0,MAKELPARAM(4,4));const auto hovered=color(probe.button);SendMessageW(probe.button,BM_SETSTATE,TRUE,0);const auto pressed=color(probe.button);SendMessageW(probe.button,BM_SETSTATE,FALSE,0);SendMessageW(probe.button,WM_MOUSELEAVE,0,0);
            check(probe.ownerPaints>=3&&probe.accepted&&resting==theme::field&&hovered==theme::hover&&pressed==theme::border,"native owner-draw callbacks paint all button interaction states");
            const auto primaryRest=color(app.exportZip);SendMessageW(app.exportZip,WM_MOUSEMOVE,0,MAKELPARAM(4,4));const auto primaryHot=color(app.exportZip);SendMessageW(app.exportZip,BM_SETSTATE,TRUE,0);const auto primaryDown=color(app.exportZip);SendMessageW(app.exportZip,BM_SETSTATE,FALSE,0);SendMessageW(app.exportZip,WM_MOUSELEAVE,0,0);
            check(primaryRest==theme::primaryFill&&primaryHot==theme::primaryHover&&primaryDown==theme::primaryPressed,"export button has distinct neutral gray rest hover and press states");
            RemovePropW(probe.button,L"FreeClimbOwnerDraw");SendMessageW(probe.button,BM_SETSTYLE,BS_DEFPUSHBUTTON,TRUE);check((GetWindowLongPtrW(probe.button,GWL_STYLE)&BS_TYPEMASK)==BS_DEFPUSHBUTTON,"negative control confirms native default-style messages replace owner drawing without the guard");SetPropW(probe.button,L"FreeClimbOwnerDraw",reinterpret_cast<HANDLE>(1));SendMessageW(probe.button,BM_SETSTYLE,BS_PUSHBUTTON,TRUE);
            for(const auto style:{BS_PUSHBUTTON,BS_DEFPUSHBUTTON,BS_PUSHBUTTON}){SendMessageW(probe.button,BM_SETSTYLE,WPARAM(style),TRUE);check((GetWindowLongPtrW(probe.button,GWL_STYLE)&BS_TYPEMASK)==BS_OWNERDRAW,"dialog default-button style changes retain owner drawing");check(color(probe.button)==theme::field,"default-style messages cannot expose a native white button face");}
            SetFocus(app.open);for(unsigned i=0;i<12;++i){MSG key{};key.hwnd=window;key.message=WM_KEYDOWN;key.wParam=VK_TAB;key.lParam=1;check(IsDialogMessageW(window,&key)!=FALSE,"real dialog Tab navigation is handled");for(const auto button:{app.open,app.base,app.packBrowse,app.exportZip,app.play,app.previous,app.next,app.advanced})check((GetWindowLongPtrW(button,GWL_STYLE)&BS_TYPEMASK)==BS_OWNERDRAW,"Tab focus changes preserve every custom button style");}
            auto press=[&](HWND item){SendMessageW(item,WM_LBUTTONDOWN,MK_LBUTTON,MAKELPARAM(8,18));};
            auto release=[&](HWND item){SendMessageW(item,WM_LBUTTONUP,0,MAKELPARAM(8,18));};
            press(probe.button);check(SendMessageW(probe.button,BM_GETSTATE,0,0)&BST_PUSHED,"native mouse press sets the owner-drawn button state");
            SendMessageW(probe.button,WM_MOUSEMOVE,MK_LBUTTON,MAKELPARAM(220,18));SendMessageW(probe.button,WM_LBUTTONUP,0,MAKELPARAM(220,18));check(probe.buttonClicks==0&&!(SendMessageW(probe.button,BM_GETSTATE,0,0)&BST_PUSHED),"dragging out cancels a button click without retaining the pressed state");
            press(probe.button);release(probe.button);check(probe.buttonClicks==1,"native mouse release emits exactly one click");
            SetFocus(probe.button);SendMessageW(probe.button,WM_KEYDOWN,VK_SPACE,1);check(SendMessageW(probe.button,BM_GETSTATE,0,0)&BST_PUSHED,"space presses the native owner-drawn button");SendMessageW(probe.button,WM_KEYUP,VK_SPACE,LPARAM(0xC0000001));check(probe.buttonClicks==2&&!(SendMessageW(probe.button,BM_GETSTATE,0,0)&BST_PUSHED),"space release emits exactly one native click");
            press(probe.button);SendMessageW(probe.button,WM_CANCELMODE,0,0);check(probe.buttonClicks==2&&!(SendMessageW(probe.button,BM_GETSTATE,0,0)&BST_PUSHED),"cancel mode releases button capture without activating it");
            EnableWindow(probe.button,FALSE);check(color(probe.button)==theme::field&&!(SendMessageW(probe.button,BM_GETSTATE,0,0)&BST_PUSHED),"disabled button remains dark and released");EnableWindow(probe.button,TRUE);
            const int tickY=(36-MulDiv(14,app.dpi,96))/2+3;const auto unchecked=color(probe.checkbox,4,tickY);SendMessageW(probe.checkbox,BM_SETCHECK,BST_CHECKED,0);const auto checked=color(probe.checkbox,4,tickY);
            check(probe.customPaints>=2&&probe.accepted&&unchecked==theme::field&&checked==theme::accent,"native v6 checkbox custom-draw callbacks skip the system face and paint the current check state");
            SendMessageW(probe.checkbox,BM_SETCHECK,BST_UNCHECKED,0);SetFocus(probe.checkbox);SendMessageW(probe.checkbox,WM_KEYDOWN,VK_SPACE,1);SendMessageW(probe.checkbox,WM_KEYUP,VK_SPACE,LPARAM(0xC0000001));check(SendMessageW(probe.checkbox,BM_GETCHECK,0,0)==BST_CHECKED&&probe.checkClicks==1,"native checkbox space toggles once and forwards its panel notification");
            press(probe.checkbox);SendMessageW(probe.checkbox,WM_MOUSEMOVE,MK_LBUTTON,MAKELPARAM(220,18));SendMessageW(probe.checkbox,WM_LBUTTONUP,0,MAKELPARAM(220,18));check(SendMessageW(probe.checkbox,BM_GETCHECK,0,0)==BST_CHECKED&&probe.checkClicks==1,"canceled checkbox press retains its prior value");
            press(probe.checkbox);release(probe.checkbox);check(SendMessageW(probe.checkbox,BM_GETCHECK,0,0)==BST_UNCHECKED&&probe.checkClicks==2,"native checkbox mouse click toggles exactly once");
            EnableWindow(probe.checkbox,FALSE);check(color(probe.checkbox,4,tickY)==theme::field,"disabled checkbox keeps its dark face");EnableWindow(probe.checkbox,TRUE);
            for(const auto item:{probe.button,probe.checkbox}){SendMessageW(item,WM_THEMECHANGED,0,0);SendMessageW(item,WM_SYSCOLORCHANGE,0,0);SendMessageW(item,WM_MOUSELEAVE,0,0);}
            check(color(probe.button)==theme::field&&color(probe.checkbox,4,tickY)==theme::field&&probe.accepted,"theme and system-color restart preserves native dark drawing callbacks");
            SendMessageW(probe.track,TBM_SETPOS,TRUE,250);check(color(probe.track)==theme::panel&&probe.trackPaints>0&&probe.accepted,"native trackbar custom draw suppresses the default track and thumb");
            SetFocus(probe.track);SendMessageW(probe.track,WM_KEYDOWN,VK_RIGHT,1);SendMessageW(probe.track,WM_KEYUP,VK_RIGHT,LPARAM(0xC0000001));check(SendMessageW(probe.track,TBM_GETPOS,0,0)==251,"custom-drawn trackbar retains native keyboard increments");
            SendMessageW(probe.track,WM_THEMECHANGED,0,0);EnableWindow(probe.track,FALSE);check(color(probe.track)==theme::panel&&probe.accepted,"disabled trackbar survives theme reload without a light native face");
            {
                view::Canvas target(200,60);const COLORREF untouched=RGB(255,0,255);const RECT region{0,0,200,60};NMCUSTOMDRAW delayed{};delayed.hdr.code=NM_CUSTOMDRAW;delayed.dwDrawStage=CDDS_PREPAINT;delayed.hdc=target.dc();
                for(const auto item:{probe.track,probe.checkbox}){ShowWindow(item,SW_HIDE);view::background(target.dc(),region,untouched);theme::control(item,target.dc(),app.dpi);check(GetPixel(target.dc(),90,18)!=untouched,"negative control reproduces obsolete hidden controls painting over another page");view::background(target.dc(),region,untouched);delayed.hdr.hwndFrom=item;check(SendMessageW(window,WM_NOTIFY,0,reinterpret_cast<LPARAM>(&delayed))==CDRF_SKIPDEFAULT&&GetPixel(target.dc(),90,18)==untouched,"delayed native custom draw from a hidden control cannot overwrite current help text");ShowWindow(item,SW_SHOW);}
                ShowWindow(probe.track,SW_HIDE);ValidateRect(probe.track,nullptr);SendMessageW(probe.track,TBM_SETPOS,TRUE,700);check(SendMessageW(probe.track,TBM_GETPOS,0,0)==700&&!GetUpdateRect(probe.track,nullptr,FALSE),"hidden slider values update without requesting paint over the current page");ShowWindow(probe.track,SW_SHOW);
                EnableWindow(probe.track,TRUE);int previousCenter=-1;for(const int position:{0,500,1000}){SendMessageW(probe.track,TBM_SETPOS,TRUE,position);RECT thumb{};SendMessageW(probe.track,TBM_GETTHUMBRECT,0,reinterpret_cast<LPARAM>(&thumb));const int center=(thumb.left+thumb.right)/2;check(center>previousCenter,"visible slider native thumb geometry follows beginning middle and end");view::background(target.dc(),region,untouched);SendMessageW(probe.track,WM_PRINTCLIENT,reinterpret_cast<WPARAM>(target.dc()),PRF_CLIENT);check(GetPixel(target.dc(),center,18)==theme::accent,"visible slider paints its thumb at the current native position");previousCenter=center;}
                ShowWindow(probe.button,SW_HIDE);view::background(target.dc(),region,untouched);DRAWITEMSTRUCT delayedButton{};delayedButton.CtlType=ODT_BUTTON;delayedButton.hwndItem=probe.button;delayedButton.hDC=target.dc();check(SendMessageW(window,WM_DRAWITEM,0,reinterpret_cast<LPARAM>(&delayedButton))==TRUE&&GetPixel(target.dc(),90,18)==untouched,"hidden owner-drawn buttons cannot overwrite another page during state changes");ShowWindow(probe.button,SW_SHOW);
                for(const auto item:{probe.track,probe.checkbox,probe.button})check(GetWindowLongPtrW(item,GWL_STYLE)&WS_CLIPSIBLINGS,"control drawing clips against current siblings");
            }
            check(GetWindowLongPtrW(app.tabs,GWL_STYLE)&TCS_OWNERDRAWFIXED,"tabs retain native interaction with owned item painting");RECT tabClient{};GetClientRect(app.tabs,&tabClient);view::Canvas nativeTabs(tabClient.right,tabClient.bottom);RemoveWindowSubclass(app.tabs,themedProcedure,2);SendMessageW(app.tabs,WM_PRINTCLIENT,reinterpret_cast<WPARAM>(nativeTabs.dc()),PRF_CLIENT);SetWindowSubclass(app.tabs,themedProcedure,2,reinterpret_cast<DWORD_PTR>(&app));
            RECT selectedTab{};TabCtrl_GetItemRect(app.tabs,TabCtrl_GetCurSel(app.tabs),&selectedTab);check(probe.tabPaints>0&&probe.accepted&&GetPixel(nativeTabs.dc(),(selectedTab.left+selectedTab.right)/2,selectedTab.top+4)==theme::field,"native tab item callbacks use the selected dark face");
            const auto gdiBefore=GetGuiResources(GetCurrentProcess(),GR_GDIOBJECTS);for(unsigned i=0;i<80;++i){color(probe.button);color(probe.checkbox);}check(GetGuiResources(GetCurrentProcess(),GR_GDIOBJECTS)<=gdiBefore+1,"repeated native themed painting releases temporary GDI resources");SetFocus(app.tabs);
        }
        view::Canvas popup(500,40);DRAWITEMSTRUCT popupItem{};popupItem.CtlType=ODT_COMBOBOX;popupItem.itemID=0;popupItem.itemState=ODS_SELECTED;popupItem.hwndItem=app.slot;popupItem.hDC=popup.dc();popupItem.rcItem={0,0,500,40};SendMessageW(window,WM_DRAWITEM,slotId,reinterpret_cast<LPARAM>(&popupItem));
        check(GetPixel(popup.dc(),490,20)==theme::hover&&!theme::comboValue(app.slot,0).empty(),"native combo owner-draw callback paints its selected row with dark colors");
        RECT tabBounds{};TabCtrl_GetItemRect(app.tabs,1,&tabBounds);SendMessageW(app.tabs,WM_MOUSEMOVE,0,MAKELPARAM((tabBounds.left+tabBounds.right)/2,(tabBounds.top+tabBounds.bottom)/2));check(INT_PTR(GetPropW(app.tabs,L"FreeClimbHoverTab"))==2,"tab hover tracks the actual native tab hit target");SendMessageW(app.tabs,WM_MOUSELEAVE,0,0);
        const auto savedCamera=app.camera;const auto savedOptions=app.options;
        ValidateRect(app.timeline,nullptr);
        auto orbit=[&](int dx,int dy){SendMessageW(app.preview,WM_LBUTTONDOWN,MK_LBUTTON,MAKELPARAM(100,100));SendMessageW(app.preview,WM_MOUSEMOVE,MK_LBUTTON,MAKELPARAM(100+dx,100+dy));SendMessageW(app.preview,WM_LBUTTONUP,0,MAKELPARAM(100+dx,100+dy));};
        orbit(20,0);check(std::abs(app.camera.yaw-savedCamera.yaw+.18f*96.f/app.dpi)<.00001f&&app.camera.pitch==savedCamera.pitch,"rightward preview drag follows the requested yaw direction without changing pitch");
        orbit(-20,0);check(std::abs(app.camera.yaw-savedCamera.yaw)<.00001f,"leftward preview drag exactly reverses rightward drag");
        orbit(0,20);check(std::abs(app.camera.pitch-savedCamera.pitch+.12f*96.f/app.dpi)<.00001f,"downward preview drag follows the requested pitch direction");
        orbit(0,-20);check(std::abs(app.camera.pitch-savedCamera.pitch)<.00001f,"upward preview drag reverses downward drag");
        check(app.time==0&&!GetUpdateRect(app.timeline,nullptr,FALSE),"camera-only dragging leaves the animation clock and timeline unchanged");
        const auto previousZoom=app.camera.zoom;SendMessageW(app.preview,WM_MOUSEWHEEL,MAKEWPARAM(0,WHEEL_DELTA),0);check(std::abs(app.camera.zoom-previousZoom*std::exp(.12f))<.00001f,"preview wheel retains its previous zoom direction and sensitivity");
        app.camera.yaw=-3.13f;orbit(5,0);check(app.camera.yaw>3&&std::abs(std::remainder(app.camera.yaw+3.13f,6.28318530718f)+.045f*96.f/app.dpi)<.00001f,"preview drag wraps through the negative yaw boundary without a visual jump");
        app.camera.yaw=3.13f;orbit(-5,0);check(app.camera.yaw<-3&&std::abs(std::remainder(app.camera.yaw-3.13f,6.28318530718f)-.045f*96.f/app.dpi)<.00001f,"preview drag wraps through the positive yaw boundary without a visual jump");
        check(std::abs(app.get(22)-app.camera.yaw*57.29578f)<.5f&&app.options.yawDegrees==savedOptions.yawDegrees,"wrapped camera yaw updates its scene slider without changing clip facing");
        app.camera=savedCamera;app.put(22,savedCamera.yaw*57.29578f);app.put(23,savedCamera.pitch*57.29578f);
        check(!app.advancedMotion&&!(GetWindowLongPtrW(app.sliders[4].track,GWL_STYLE)&WS_VISIBLE),"position adjustments start collapsed");const auto beforeExpand=app.revision;click(app.advanced,advancedId);check(app.advancedMotion&&(GetWindowLongPtrW(app.sliders[9].track,GWL_STYLE)&WS_VISIBLE)&&app.revision==beforeExpand,"position controls expand without editing the clip");click(app.advanced,advancedId);check(!app.advancedMotion&&app.revision==beforeExpand,"collapsing position controls preserves edits");
        RECT renderArea{};GetClientRect(app.preview,&renderArea);view::Canvas target(renderArea.right,renderArea.bottom);app.paint(app.preview,target.dc());const auto buffer=app.previewCanvas.get();for(unsigned i=0;i<20;++i){app.camera.orbit(1,1);app.paint(app.preview,target.dc());}check(app.previewCanvas.get()==buffer&&app.revision==beforeExpand,"orbit frames reuse the preview buffer without rebuilding animation data");app.camera=savedCamera;
        const auto document=app.document;const auto pack=document->pack;const auto invalid=pack.parent_path()/std::filesystem::path(document->templateConfig.at("file").get<std::string>());
        check(app.suggestedOutput().parent_path()==pack.parent_path().parent_path(),"base export defaults outside protected pack");
        const auto namedOutput=pack.parent_path().parent_path()/L"custom-action.zip";app.rememberOutput(namedOutput);
        check(app.suggestedOutput()==namedOutput,"repeat export of the same input and target retains the author's chosen filename");
        app.lastOutputInput=pack.parent_path()/L"previous-input.hkx";
        check(app.suggestedOutput().filename()==document->input.stem().wstring()+L"-FreeClimb-"+fromUtf8(app.chosenSlot())+L".zip"&&app.suggestedOutput().parent_path()==namedOutput.parent_path(),"another source gets its own filename while retaining the export directory");app.rememberOutput(namedOutput);
        auto equivalent=pack.generic_wstring();for(auto& character:equivalent)if(character>=L'a'&&character<=L'z')character-=L'a'-L'A';app.selectPack(equivalent);
        check(app.selectionMatchesDocument()&&IsWindowEnabled(app.exportZip),"case and slash equivalent pack stays exportable");
        app.selectPack(invalid);check(!IsWindowEnabled(app.exportZip)&&app.document==document,"pending pack change retains document and blocks export");
        SendMessageW(app.slot,CB_SETCURSEL,1,0);SendMessageW(window,WM_COMMAND,MAKEWPARAM(slotId,CBN_SELCHANGE),reinterpret_cast<LPARAM>(app.slot));wait();
        check(app.document==document&&app.chosenSlot()==document->slot&&samePath(fieldText(app.pack),pack)&&IsWindowEnabled(app.exportZip),"failed slot reload restores document identity and ready preview");
        slide(4,3);wait();const auto edited=app.animation;const auto offset=app.options.rootOffset.x;
        check(offset>0&&edited&&IsWindowEnabled(app.exportZip),"actual motion edit completed");
        app.selectPack(invalid);slide(4,4);wait();check(!IsWindowEnabled(app.exportZip)&&app.options.rootOffset.x>offset,"editing cannot re-enable export for mismatched pack");
        app.selectPack(pack);check(IsWindowEnabled(app.exportZip),"returning to loaded pack retains exportable edits");const auto retained=app.animation;const auto retainedOffset=app.options.rootOffset.x;
        app.selectPack(invalid);SendMessageW(app.slot,CB_SETCURSEL,1,0);click(app.base,baseId);
        check(app.document==document&&app.animation==retained&&app.options.rootOffset.x==retainedOffset&&app.selectionMatchesDocument()&&IsWindowEnabled(app.exportZip),"failed base load preserves edits and reconciles selectors");
        for(unsigned i=10;i<13;++i)slide(i,45);wait();check(app.animation==retained&&!IsWindowEnabled(app.exportZip)&&app.revision!=app.displayRevision,"invalid combined correction blocks stale preview export");
        app.selectPack(invalid);SendMessageW(app.slot,CB_SETCURSEL,1,0);click(app.base,baseId);
        check(app.selectionMatchesDocument()&&!IsWindowEnabled(app.exportZip)&&app.revision!=app.displayRevision,"failed load cannot bless an invalid edit preview");
        for(unsigned i=10;i<13;++i)slide(i,0);wait();check(app.selectionMatchesDocument()&&IsWindowEnabled(app.exportZip),"valid correction recovers export readiness");
        const auto output=std::filesystem::path(args[3]).parent_path()/L"state-export.zip";const auto report=fc::exportConverterEditor(*app.document,*app.animation,output,true);
        check(report.at("ok")==true&&report.at("slot")==app.chosenSlot()&&report.at("loaded")==fc::activeMotionCount,"recovered export matches visible slot and validates complete pack");
        check(SendMessageW(app.slot,CB_GETCOUNT,0,0)==25,"one action selector exposes complete directions without internal helpers");
        auto visible=[](HWND item){return (GetWindowLongPtrW(item,GWL_STYLE)&WS_VISIBLE)!=0;};
        auto choose=[&](std::string_view name){app.selectPrimary(name);SendMessageW(window,WM_COMMAND,MAKEWPARAM(slotId,CBN_SELCHANGE),reinterpret_cast<LPARAM>(app.slot));wait();};
        app.selectPrimary("contextMantle");app.populateStages("contextMantle");click(app.base,baseId);check(app.adaptedPreview()&&app.activePreview()->frames.back().phase==1,"mantle preview reaches the source terminal pose before handing off");
        check(app.suggestedOutput().filename()==L"contextMantle-FreeClimb-contextMantle.zip"&&app.suggestedOutput()!=namedOutput,"changing action no longer suggests the previous hang archive");
        app.rememberOutput(namedOutput);app.lastOutputInput=app.document->input;app.lastOutputTarget="hang";
        check(app.suggestedOutput().filename()==L"contextMantle-FreeClimb-contextMantle.zip","the same source retargeted to another action receives the current target name");
        for(const float phase:{0.f,.2f,.5f,.9f,1.f}){const float previewPhase=app.previewPhase(phase);check(std::abs(app.sourcePhase(previewPhase)-phase)<.002f,"mantle contact markers map bidirectionally between source and controller clocks");app.seek(previewPhase*app.previewDuration());check(std::abs(app.sourceTime()/app.document->clip.duration-phase)<.002f,"current mantle frame selects the corresponding source trim time");}
        app.seek(app.previewPhase(.2f)*app.previewDuration());click(app.inHere,inHereId);wait();check(std::abs(app.options.trimIn/app.document->clip.duration-.2f)<.002f,"start-here trim uses source mantle time rather than accelerated controller time");click(app.reset,resetId);wait();
        for(const int testDpi:{96,120,144})for(const bool translated:{false,true})for(const bool expanded:{false,true})for(const int testPage:{3,0,4,2,1,0}){
            app.changeDpi(testDpi);app.chinese=translated;app.page=testPage;app.advancedMotion=expanded;app.translate();RECT helpBounds{};GetWindowRect(app.help,&helpBounds);
            for(HWND item=GetWindow(app.panel,GW_CHILD);item;item=GetWindow(item,GW_HWNDNEXT))if(item!=app.help&&visible(item)){RECT bounds{},intersection{};GetWindowRect(item,&bounds);check(!IntersectRect(&intersection,&helpBounds,&bounds),"current panel help has no overlapping visible controls after language DPI and page changes");}
        }
        app.page=0;app.chinese=false;app.advancedMotion=false;app.changeDpi(96);app.translate();
        app.selectPrimary("runLeft");app.populateStages("runLeft");click(app.base,baseId);
        check(app.wholeDocument&&app.wholeSequence&&app.document->slot=="runLeft"&&app.sequencePreview(),"direction loads one complete editable source and previews all stages");
        check(!visible(app.stage)&&!visible(app.stageEditingToggle)&&!visible(app.loopBegin),"base direction hides internal stages and unnecessary source-boundary controls");
        check(app.animation->clip.duration>app.wholeSequence->stages[1].animation.clip.duration,"whole document includes start and ending outside its loop");
        app.seek(0);check(std::abs(app.sourceTime()-app.wholeSequence->sourceRanges[0][0])<.00001f,"first hold maps to the real source start");app.seek(app.previewDuration());check(std::abs(app.sourceTime()-app.wholeSequence->sourceRanges[2][1])<.00001f&&std::abs(app.sourcePhase(1)-1)<.00001f,"final hold maps to the source end for trim markers and contacts");app.seek(0);
        check(IsWindowEnabled(app.sliders[2].track)&&IsWindowEnabled(app.inHere)&&IsWindowEnabled(app.exportZip),"whole action edits and export are available without a second stage selector");
        const auto ownSeconds=app.animation->clip.duration;app.load(app.document->input);wait();check(app.wholeSequence&&app.animation->clip.duration==ownSeconds&&!app.document->authored,"importing the current base HKX retains the complete timeline and existing markers");
        const auto unchanged=app.animation;const auto revision=app.revision;click(app.advanced,advancedId);
        check(visible(app.sourceComparisonToggle)&&visible(app.loopBegin),"optional comparison and existing loop markers expand in more adjustments");
        SendMessageW(app.sourceComparisonToggle,BM_SETCHECK,BST_CHECKED,0);click(app.sourceComparisonToggle,sourceComparisonId);
        check(!app.adaptedPreview()&&app.animation==unchanged&&app.revision==revision,"optional original-pose comparison never modifies the action");
        SendMessageW(app.sourceComparisonToggle,BM_SETCHECK,BST_UNCHECKED,0);click(app.sourceComparisonToggle,sourceComparisonId);check(app.sequencePreview(),"returning from comparison restores complete preview automatically");
        check(!visible(app.stageEditingToggle),"private reference data never exposes a shared-reference editor");
        const auto before=app.wholeSequence;const float clockBefore=app.previewDuration();const auto phaseBefore=app.sourcePhase(.6f);slide(2,1.2f);check(app.sequencePreview()&&app.previewDuration()==clockBefore&&app.sourcePhase(.6f)==phaseBefore&&!IsWindowEnabled(app.exportZip),"pending edits retain the displayed preview clock and mapping without permitting stale export");check(!IsWindowEnabled(app.stageEditingToggle),"mode checkbox cannot visually toggle while its edit handler is busy");wait();check(app.wholeSequence&&app.animation->clip.duration<unchanged->clip.duration,"speed adjustment changes the whole continuous action");
        for(unsigned i=0;i<3;++i)check(app.wholeSequence->stages[i].animation.clip.duration<before->stages[i].animation.clip.duration,"speed applies to start loop and ending rather than only loop");
        const float speed=app.options.speed;const auto editedLeft=app.animation;choose("runRight");check(app.wholeDocument&&app.options.speed==1&&app.sequencePreview(),"another direction has its own complete unchanged source");
        choose("runLeft");check(app.options.speed==speed&&app.animation->clip.frames.size()==editedLeft->clip.frames.size(),"returning to a direction retains complete edits");
        auto selected=app.exportStages();check(selected.size()==4,"direction export includes only its own start loop ending and private reference");
        std::vector<fc::ConverterEditorExport> exports;for(const auto& item:selected){check(item.direction=="runLeft","direction export never includes another direction");exports.push_back({item.document.get(),item.animation.get(),item.direction});}
        const auto groupOutput=fc::exportConverterEditorGroup(exports,std::filesystem::path(args[3]).parent_path()/L"state-group-export.zip",true);check(groupOutput.at("ok")==true&&groupOutput.at("loaded")==fc::activeMotionCount,"whole direction export validates through production loader");
        for(unsigned i=10;i<13;++i)slide(i,45);wait();check(!IsWindowEnabled(app.exportZip),"invalid whole-pose edits cannot export an earlier valid result");
        for(unsigned i=10;i<13;++i)slide(i,0);wait();check(IsWindowEnabled(app.exportZip)&&app.wholeSequence,"correcting a whole-pose edit rebuilds its complete export");
        choose("runUp");app.load(invalid);wait();check(app.wholeDocument&&app.document->authored&&!app.wholeSequence&&!IsWindowEnabled(app.exportZip),"complete import keeps source editable while requiring two real loop boundaries");
        check(visible(app.loopBegin)&&visible(app.loopEnd)&&!visible(app.stage),"complete import exposes only two source markers without Generate or stage selectors");
        const float inputSeconds=app.document->clip.duration;app.seek(inputSeconds*.25f);click(app.loopBeginHere,loopBeginHereId);check(!IsWindowEnabled(app.exportZip),"one marker cannot generate an incomplete export");
        app.seek(inputSeconds*.75f);click(app.loopEndHere,loopEndHereId);wait();check(app.wholeSequence&&!app.adaptedPreview()&&IsWindowEnabled(app.exportZip),"second marker prepares the complete export without replacing imported source playback with a base route");
        check(!visible(app.sourceComparisonToggle)&&!app.reference.wall&&!app.reference.ledge,"imported complete action has no redundant source toggle or assumed wall reference");
        for(const float phase:{0.f,.2f,.5f,.9f,1.f}){app.seek(phase*app.previewDuration());check(std::abs(app.sourceTime()-phase*inputSeconds)<.00001f&&app.sourcePhase(phase)==phase&&app.previewPhase(phase)==phase,"complete source playback retains linear source and contact clocks after splitting");const auto pose=app.previewPose(app.time);const auto original=fc::sampleConverterEditor(*app.animation,app.time);check((pose[0].t-original[0].t).length()<.00001f,"complete source preview retains its Root without controller displacement");}
        check(std::abs(app.wholeSequence->actualCuts.loopBegin-inputSeconds*.25f)<=inputSeconds/float(app.document->clip.frames.size()-1)&&std::abs(app.wholeSequence->actualCuts.loopEnd-inputSeconds*.75f)<=inputSeconds/float(app.document->clip.frames.size()-1),"source markers map to actual source frames");
        const auto cutBegin=fieldText(app.loopBegin),cutEnd=fieldText(app.loopEnd);app.set(app.loopEnd,L"bad");check(app.pendingEdit&&IsWindowEnabled(app.loopEnd)&&!IsWindowEnabled(app.exportZip),"typing a loop marker remains enabled and debounces validation");wait();check(!app.wholeSequence&&!IsWindowEnabled(app.exportZip)&&app.animation,"invalid marker retains editable source and blocks stale export");app.set(app.loopEnd,cutEnd);wait();check(app.wholeSequence&&IsWindowEnabled(app.exportZip),"corrected marker automatically restores validated full export");
        const auto sourceBefore=app.wholeSequence;slide(4,7);wait();check(app.wholeSequence&&app.options.rootOffset.x>0,"full imported source transforms update automatically");
        for(unsigned i=0;i<3;++i)check(app.wholeSequence->stages[i].animation.clip.frames.size()==sourceBefore->stages[i].animation.clip.frames.size(),"whole-source editing preserves all three timing regions");
        selected=app.exportStages();exports.clear();for(const auto& item:selected)exports.push_back({item.document.get(),item.animation.get(),item.direction});
        const auto complete=fc::exportConverterEditorGroup(exports,std::filesystem::path(args[3]).parent_path()/L"complete-direction.zip",true);check(complete.at("ok")==true,"automatically assembled whole import exports without a Generate step");
        const auto importedWhole=app.wholeDocument;choose("runLeft");choose("runUp");check(app.wholeDocument==importedWhole&&fieldText(app.loopBegin)==cutBegin&&fieldText(app.loopEnd)==cutEnd&&app.options.rootOffset.x>0,"switching actions preserves whole imported source markers and edits");
        choose("contextHopLeft");check(app.document->slot=="contextHopLeft"&&!visible(app.stage)&&!visible(app.stageEditingToggle),"left contextual leap is edited as one independent action without shared stage controls");
        choose("contextHopRight");check(app.document->slot=="contextHopRight"&&!visible(app.stage)&&!visible(app.stageEditingToggle),"right contextual leap is edited as one independent action without shared stage controls");
        const auto sourceBase=fc::loadConverterBaseEditor(pack,"hang");
        for(const float amplitude:{0.f,.5f,1.f,2.f}) {
            auto clip=sourceBase.clip;const fc::Vec travel=fc::Vec{3,2,4}*amplitude;
            for(std::size_t i=0;i<clip.frames.size();++i)clip.frames[i][0].t=travel*(float(i)/float(clip.frames.size()-1));
            const auto fileName=std::filesystem::path(args[3]).parent_path()/(L"source-travel-"+number(amplitude,1)+L".hkx");
            const auto bytes=fc::writeCanonicalConverterHkx(clip,sourceBase.base);{std::ofstream file(fileName,std::ios::binary);file.write(reinterpret_cast<const char*>(bytes.data()),std::streamsize(bytes.size()));check(bool(file),"independent imported Root-motion fixture written");}
            for(const auto name:visibleActions) {
                app.stageEditing=false;app.selectPrimary(toUtf8(name));app.populateStages(toUtf8(name));app.previewChoice();app.load(fileName);wait();
                check(app.document->authored&&!app.adaptedPreview()&&!app.runtimePreview&&!app.fullPreview,"every imported action bypasses the base controller route");
                check(!visible(app.sourceComparisonToggle)&&!app.reference.wall&&!app.reference.ledge,"every source preview hides comparison and starts without a fictitious wall");
                const auto camera=app.camera;const auto start=app.previewPose(0);const auto end=app.previewPose(app.previewDuration());
                check((end[0].t-start[0].t-travel).length()<.001f,"preview preserves imported travel and leaves in-place sources stationary");
                for(const float phase:{0.f,.25f,.5f,.75f,1.f}) {
                    app.seek(phase*app.previewDuration());const auto pose=app.previewPose(app.time);
                    check((pose[0].t-travel*phase).length()<.001f,"source displacement remains visible at every sampled preview time");
                    check((app.camera.center-camera.center).length()<.00001f&&app.camera.zoom==camera.zoom,"playback does not recenter the camera to hide source movement");
                    check(std::abs(app.sourceTime()-app.document->clip.duration*phase)<.00001f&&app.sourcePhase(phase)==phase,"imported source trim and contact clocks remain linear");
                }
            }
            app.selectPrimary("contextMantle");app.populateStages("contextMantle");app.previewChoice();click(app.base,baseId);
            check(!app.document->authored&&app.mantlePreview()&&app.reference.wall&&app.reference.ledge,"returning to the base restores its calibrated ledge route without retaining source-preview state");
            app.load(fileName);wait();check(app.document->authored&&!app.adaptedPreview()&&!app.runtimePreview&&!app.fullPreview,"reimporting after the base cannot retain its controller movement");
            const auto fixtureOutput=std::filesystem::path(args[3]).parent_path()/(L"source-travel-export-"+number(amplitude,1)+L".zip");const auto converted=fc::exportConverterEditor(*app.document,*app.animation,fixtureOutput,true);
            check(converted.at("ok")==true&&IsWindowEnabled(app.exportZip),"source-preview isolation preserves valid export and production-loader validation");
        }
        DestroyWindow(window);window=nullptr;std::ofstream file(std::filesystem::path(args[3]),std::ios::binary);file<<Json{{"ok",true},{"checks",checks},{"guiLaunched",false},{"hiddenWindow",true},{"slot",report.at("slot")},{"output",report.at("output")}}.dump(2);return file?0:1;
    }catch(const std::exception& failure){if(window)DestroyWindow(window);std::ofstream file(std::filesystem::path(args[3]),std::ios::binary);file<<Json{{"ok",false},{"checks",checks},{"error",failure.what()},{"guiLaunched",false}}.dump(2);return 1;}
}
int renderOffscreen(int count,wchar_t** args,HINSTANCE instance,const std::filesystem::path& executable) {
    if(count<7||count>16)return 2;
    try {
        Completion loaded;loaded.work=Work::load;
        const bool base=std::wstring_view(args[2])==L"--base";const auto action=toUtf8(args[4]);
        if(base&&fc::converter::isWallRunPrimary(action)){loaded.wholeAction=true;loaded.direction=action;loaded.wholeDocument=std::make_shared<fc::ConverterWallRunDocument>(fc::loadConverterWallRunEditor(args[3],action));loaded.document=std::make_shared<fc::ConverterEditorDocument>(loaded.wholeDocument->document);loaded.authoredSequence=std::make_shared<fc::ConverterWallRunSequence>();std::string error;if(!fc::applyConverterWallRunEdits(*loaded.wholeDocument,{},loaded.wholeDocument->cuts,*loaded.authoredSequence,error))throw std::runtime_error(error);loaded.animation=std::make_shared<fc::ConverterEditedAnimation>(loaded.authoredSequence->whole);wholePreview(loaded);}
        else {loaded.document=std::make_shared<fc::ConverterEditorDocument>(base?fc::loadConverterBaseEditor(args[3],action):fc::loadConverterEditor(args[2],args[3],action));loaded.animation=std::make_shared<fc::ConverterEditedAnimation>(fc::applyConverterEdits(*loaded.document,{}));if(fc::converter::isWallRunPrimary(action)){auto whole=std::make_shared<fc::ConverterWallRunDocument>();whole->document=*loaded.document;loaded.wholeDocument=std::move(whole);loaded.wholeAction=true;loaded.imported=true;loaded.direction=action;}else loaded.runtimePreview=buildPreview(*loaded.document,*loaded.animation,std::vector<StageEdit>{});}
        const auto document=loaded.document;const auto animation=loaded.animation;
        const float seconds=count>=8?std::stof(args[7]):animation->clip.duration*.45f;const bool chinese=count>=9&&std::wstring_view(args[8])==L"chinese";const int page=count>=10?std::stoi(args[9]):0;
        const int width=count>=11?std::stoi(args[10]):1280,height=count>=12?std::stoi(args[11]):800,dpi=count>=13?std::stoi(args[12]):96,scroll=count>=14?std::stoi(args[13]):0;
        if(width<640||width>8192||height<360||height>4320||page<0||page>4||dpi<96||dpi>384||scroll<0)throw std::runtime_error("Invalid offscreen layout");
        App app;app.executable=executable;RECT extent{0,0,width,height};AdjustWindowRectEx(&extent,WS_OVERLAPPEDWINDOW,FALSE,0);
        const auto window=CreateWindowExW(0,L"FreeClimbConverterGui",L"FreeClimb Animation Author",WS_OVERLAPPEDWINDOW|WS_CLIPCHILDREN,0,0,extent.right-extent.left,extent.bottom-extent.top,nullptr,nullptr,instance,&app);
        if(!window)throw std::runtime_error("Unable to create hidden author tool");app.changeDpi(dpi);app.finished(loaded);app.sourceComparison=count>=15&&std::wstring_view(args[14])==L"source";app.previewChoice();
        app.chinese=chinese;app.page=page;app.advancedMotion=count>=16&&std::wstring_view(args[15])==L"expanded";TabCtrl_SetCurSel(app.tabs,page);SendMessageW(app.language,CB_SETCURSEL,chinese?1:0,0);app.translate();app.seek(seconds);KillTimer(window,1);
        const auto oldWall=app.reference.wall;SendMessageW(app.wall,BM_SETCHECK,oldWall?BST_UNCHECKED:BST_CHECKED,0);SendMessageW(app.panel,WM_COMMAND,MAKEWPARAM(wallId,BN_CLICKED),reinterpret_cast<LPARAM>(app.wall));
        const bool commandForwarded=app.reference.wall!=oldWall;SendMessageW(app.wall,BM_SETCHECK,oldWall?BST_CHECKED:BST_UNCHECKED,0);SendMessageW(app.panel,WM_COMMAND,MAKEWPARAM(wallId,BN_CLICKED),reinterpret_cast<LPARAM>(app.wall));
        const auto oldPosition=SendMessageW(app.sliders[20].track,TBM_GETPOS,0,0);SendMessageW(app.sliders[20].track,TBM_SETPOS,TRUE,oldPosition==0?500:0);SendMessageW(app.panel,WM_HSCROLL,TB_THUMBPOSITION,reinterpret_cast<LPARAM>(app.sliders[20].track));const bool sliderForwarded=std::abs(app.reference.wallY-app.get(20))<.001f;
        SendMessageW(app.sliders[20].track,TBM_SETPOS,TRUE,oldPosition);SendMessageW(app.panel,WM_HSCROLL,TB_THUMBPOSITION,reinterpret_cast<LPARAM>(app.sliders[20].track));
        TabCtrl_SetCurSel(app.tabs,(page+1)%5);NMHDR notification{app.tabs,tabId,TCN_SELCHANGE};SendMessageW(app.panel,WM_NOTIFY,tabId,reinterpret_cast<LPARAM>(&notification));const bool notifyForwarded=app.page==(page+1)%5;
        TabCtrl_SetCurSel(app.tabs,page);SendMessageW(app.panel,WM_NOTIFY,tabId,reinterpret_cast<LPARAM>(&notification));SendMessageW(app.panel,WM_VSCROLL,SB_BOTTOM,0);const bool scrollForwarded=app.panelScroll[std::size_t(page)]==std::max(0,app.panelContentHeight-app.panelViewportHeight);
        HWND focusTarget{};for(unsigned i=0;i<definitions.size();++i)if(definitions[i].page==page&&(GetWindowLongPtrW(app.sliders[i].track,GWL_STYLE)&WS_VISIBLE))focusTarget=app.sliders[i].track;
        const bool enabled=IsWindowEnabled(focusTarget)!=FALSE;EnableWindow(focusTarget,TRUE);app.scrollPanel(0);SetFocus(focusTarget);const bool focusRetained=GetFocus()==focusTarget;
        RECT focused{},viewport{};GetWindowRect(focusTarget,&focused);MapWindowPoints(nullptr,app.panel,reinterpret_cast<POINT*>(&focused),2);GetClientRect(app.panel,&viewport);const bool focusRevealed=focused.top>=0&&focused.bottom<=viewport.bottom;
        const auto tabNext=GetNextDlgTabItem(window,focusTarget,FALSE);const bool tabReachable=tabNext&&tabNext!=focusTarget;SetFocus(app.tabs);EnableWindow(focusTarget,enabled);app.scrollPanel(scroll);
        RECT timelineArea{};GetClientRect(app.timeline,&timelineArea);const auto timelineDc=GetDC(app.timeline);const auto oldTimelineFont=SelectObject(timelineDc,app.font);const auto timelineLayout=view::timelineLayout(timelineDc,timelineArea,chinese);SelectObject(timelineDc,oldTimelineFont);ReleaseDC(app.timeline,timelineDc);bool timelineMapping=true;
        for(const float phase:{0.f,.25f,.5f,.75f,1.f}){const int x=timelineLayout.left+int(std::lround(phase*float(timelineLayout.right-timelineLayout.left)));app.timelineMove({x,timelineLayout.top+timelineLayout.row/2},false);const float expected=float(x-timelineLayout.left)/float(std::max(1,timelineLayout.right-timelineLayout.left));timelineMapping=timelineMapping&&std::abs(app.time/app.previewDuration()-expected)<.0001f;}app.seek(seconds);
        const auto initialVisibility=inspectPanelVisibility(app);SetWindowPos(app.panel,app.tabs,0,0,0,0,SWP_NOMOVE|SWP_NOSIZE|SWP_NOACTIVATE);
        const auto occludedVisibility=inspectPanelVisibility(app);const bool detectsOcclusion=!occludedVisibility["panelInFrontOfTabs"].get<bool>()&&!occludedVisibility["panelHitTests"].get<bool>();
        SetWindowPos(app.tabs,app.panel,0,0,0,0,SWP_NOMOVE|SWP_NOSIZE|SWP_NOACTIVATE);const auto restoredVisibility=inspectPanelVisibility(app);
        bool panelVisible=detectsOcclusion;for(const auto& result:{initialVisibility,restoredVisibility})for(const auto& value:result)panelVisible=panelVisible&&value.get<bool>();
        view::Canvas canvas(width,height);view::background(canvas.dc(),{0,0,width,height},theme::canvas);
        for(HWND item=bottomChild(window);item;item=GetWindow(item,GW_HWNDPREV))printControl(item,window,canvas.dc());
        if(!canvas.save(args[5])){DestroyWindow(window);throw std::runtime_error("Unable to save preview bitmap");}
        const auto controls=inspectControls(window,window,app);
        auto vector=[](fc::Vec value){return Json::array({value.x,value.y,value.z});};const auto startPose=app.previewPose(0),endPose=app.previewPose(app.previewDuration()),currentPose=app.previewPose(app.time);
        const Json root{{"sourceStart",vector(animation->clip.frames.front()[0].t)},{"sourceEnd",vector(animation->clip.frames.back()[0].t)},{"displayStart",vector(startPose[0].t)},{"displayEnd",vector(endPose[0].t)},{"displayCurrent",vector(currentPose[0].t)}};
        const Json report{{"ok",panelVisible},{"previewMode",app.adaptedPreview()?"wall-run":"source"},{"authored",document->authored},{"root",root},{"previewDuration",app.previewDuration()},{"panelVisibility",{{"initial",initialVisibility},{"negativeControlDetected",detectsOcclusion},{"restored",restoredVisibility}}},{"slot",document->slot},{"frames",animation->clip.frames.size()},{"duration",animation->clip.duration},{"sampleTime",app.time},
            {"guiLaunched",false},{"hiddenWindow",true},{"width",width},{"height",height},{"dpi",dpi},{"windowDpi",GetDpiForWindow(window)},{"page",page},{"scroll",app.panelScroll[std::size_t(page)]},{"contentHeight",app.panelContentHeight},{"viewportHeight",app.panelViewportHeight},{"eventRouting",{{"command",commandForwarded},{"slider",sliderForwarded},{"notify",notifyForwarded},{"scroll",scrollForwarded},{"focusRetained",focusRetained},{"focusRevealed",focusRevealed},{"tabReachable",tabReachable},{"timelineMapping",timelineMapping}}},{"controls",controls}};
        DestroyWindow(window);std::ofstream file(std::filesystem::path(args[6]),std::ios::binary);file<<report.dump(2);return file&&panelVisible?0:1;
    }catch(const std::exception& failure){std::ofstream file(std::filesystem::path(args[6]),std::ios::binary);file<<Json{{"ok",false},{"error",failure.what()},{"guiLaunched",false}}.dump(2);return 1;}
}
}
int WINAPI wWinMain(HINSTANCE instance,HINSTANCE,LPWSTR,int show) {
    SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);INITCOMMONCONTROLSEX controls{sizeof(controls),ICC_STANDARD_CLASSES|ICC_BAR_CLASSES|ICC_TAB_CLASSES};InitCommonControlsEx(&controls);
    std::vector<wchar_t> path(32768);const auto length=GetModuleFileNameW(nullptr,path.data(),DWORD(path.size()));if(!length||length>=path.size()||!registerClasses(instance))return 1;
    int count{};auto* arguments=CommandLineToArgvW(GetCommandLineW(),&count);
    if(arguments&&count>1&&std::wstring_view(arguments[1])==L"--verify-state"){const int result=verifyState(count,arguments,instance,path.data());LocalFree(arguments);return result;}
    if(arguments&&count>1&&(std::wstring_view(arguments[1])==L"--render"||std::wstring_view(arguments[1])==L"--render-ui")){const int result=renderOffscreen(count,arguments,instance,path.data());LocalFree(arguments);return result;}
    if(arguments)LocalFree(arguments);App app;app.executable=std::filesystem::path(path.data());const int dpi=int(GetDpiForSystem());
    MONITORINFO monitor{sizeof(monitor)};GetMonitorInfoW(MonitorFromPoint({0,0},MONITOR_DEFAULTTOPRIMARY),&monitor);
    const auto initial=theme::initialSize(dpi,monitor.rcWork.right-monitor.rcWork.left,monitor.rcWork.bottom-monitor.rcWork.top);const int width=initial.cx,height=initial.cy;
    const auto window=CreateWindowExW(WS_EX_ACCEPTFILES,L"FreeClimbConverterGui",L"FreeClimb Animation Author",WS_OVERLAPPEDWINDOW|WS_CLIPCHILDREN,monitor.rcWork.left+(monitor.rcWork.right-monitor.rcWork.left-width)/2,monitor.rcWork.top+(monitor.rcWork.bottom-monitor.rcWork.top-height)/2,width,height,nullptr,nullptr,instance,&app);
    if(!window)return 1;ShowWindow(window,show);UpdateWindow(window);MSG message{};
    while(GetMessageW(&message,nullptr,0,0)>0)if(!IsDialogMessageW(window,&message)){TranslateMessage(&message);DispatchMessageW(&message);}return int(message.wParam);
}
