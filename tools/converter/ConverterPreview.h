#pragma once
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include "../../src/Pose.h"
#include <array>
#include <filesystem>
#include <fstream>
#include <optional>

namespace fc::converter::visual {
struct Camera {
    float yaw{-.65f},pitch{.16f},zoom{1};
    Vec center{0,0,88};
    void orbit(float dx,float dy) {yaw=std::remainder(yaw-dx*.009f,6.28318530718f);pitch=std::clamp(pitch-dy*.006f,-1.134464f,1.134464f);}
    void scale(float amount) {zoom=std::clamp(zoom*std::exp(amount*.12f),.25f,6.f);}
};
struct WallBounds {float left{-160},right{160},bottom{},top{230};};
struct Reference {bool wall{true},ledge{true},joints{};float wallY{32},ledgeZ{128};WallBounds bounds;};
inline WallBounds wallBounds(Vec low,Vec high) {
    if(!low.finite()||!high.finite()||low.x>high.x||low.z>high.z)return {};
    return {std::min(-160.f,low.x-32),std::max(160.f,high.x+32),std::min(0.f,low.z-32),std::max(230.f,high.z+32)};
}
inline unsigned gridDivisions(float span,float spacing) {return unsigned(std::clamp(std::ceil(span/spacing),1.f,24.f));}
struct Projected {float x{},y{},depth{};};
inline std::optional<Projected> project(Vec point,const Camera& camera,int width,int height) {
    if(!point.finite()||width<1||height<1||!camera.center.finite()||!std::isfinite(camera.yaw)||!std::isfinite(camera.pitch)||!std::isfinite(camera.zoom))return {};
    const Vec v=point-camera.center;const float sy=std::sin(camera.yaw),cy=std::cos(camera.yaw),sp=std::sin(camera.pitch),cp=std::cos(camera.pitch);
    const float x=cy*v.x+sy*v.y,y=-sy*v.x+cy*v.y,z=cp*v.z-sp*y,depth=cp*y+sp*v.z;
    const float perspective=500.f/(500.f+depth);
    if(!std::isfinite(perspective)||perspective<=0||perspective>10)return {};
    const float scale=float(std::min(width,height))*.0041f*std::clamp(camera.zoom,.25f,6.f)*perspective;
    return Projected{width*.5f+x*scale,height*.53f-z*scale,depth};
}
inline std::optional<Pose> worldPose(const Pose& local,const std::vector<int>& parents) {
    if(local.empty()||local.size()!=parents.size()||local.size()>256)return {};
    Pose world(local.size());
    for(std::size_t i=0;i<local.size();++i) {
        if(parents[i]<-1||parents[i]>=int(i)||!local[i].t.finite()||!local[i].s.finite()||!std::isfinite(local[i].q.dot(local[i].q)))return {};
        world[i]=parents[i]<0?local[i]:compose(world[std::size_t(parents[i])],local[i]);
        if(!world[i].t.finite())return {};
    }
    return world;
}
inline COLORREF limbColor(std::string_view name) {
    if(name.find("NPC L ")!=std::string_view::npos)return RGB(67,201,224);
    if(name.find("NPC R ")!=std::string_view::npos)return RGB(248,170,89);
    return RGB(228,228,228);
}
inline bool bodyBone(std::string_view name) {
    for(const auto word:{"Pelvis","Spine","Neck","Head [","Clavicle","UpperArm","Forearm","Hand [","Thigh","Calf","Foot [","Toe0"})
        if(name.find(word)!=std::string_view::npos)return true;
    return false;
}
inline void stroke(HDC dc,POINT a,POINT b,COLORREF color,int width=1) {
    const auto pen=CreatePen(PS_SOLID,width,color);const auto previous=SelectObject(dc,pen);MoveToEx(dc,a.x,a.y,nullptr);LineTo(dc,b.x,b.y);SelectObject(dc,previous);DeleteObject(pen);
}
inline void circle(HDC dc,POINT at,int radius,COLORREF color) {
    const auto brush=CreateSolidBrush(color);const auto oldBrush=SelectObject(dc,brush);const auto oldPen=SelectObject(dc,GetStockObject(NULL_PEN));
    Ellipse(dc,at.x-radius,at.y-radius,at.x+radius+1,at.y+radius+1);SelectObject(dc,oldPen);SelectObject(dc,oldBrush);DeleteObject(brush);
}
inline void label(HDC dc,RECT area,std::wstring_view value,COLORREF color,int format=DT_LEFT|DT_SINGLELINE|DT_VCENTER) {
    SetBkMode(dc,TRANSPARENT);SetTextColor(dc,color);DrawTextW(dc,value.data(),int(value.size()),&area,UINT(format));
}
inline void background(HDC dc,RECT area,COLORREF color) {const auto brush=CreateSolidBrush(color);FillRect(dc,&area,brush);DeleteObject(brush);}
inline POINT pixel(Projected p) {return {LONG(std::lround(p.x)),LONG(std::lround(p.y))};}
inline void drawReference(HDC dc,const Camera& camera,const Reference& reference,int width,int height) {
    auto line=[&](Vec a,Vec b,COLORREF color,int thickness=1){const auto from=project(a,camera,width,height),to=project(b,camera,width,height);if(from&&to)stroke(dc,pixel(*from),pixel(*to),color,thickness);};
    for(int i=-4;i<=4;++i){const float n=float(i)*40;line({n,-160,0},{n,160,0},RGB(53,53,53));line({-160,n,0},{160,n,0},RGB(53,53,53));}
    line({0,0,0},{45,0,0},RGB(183,78,81),2);line({0,0,0},{0,45,0},RGB(85,178,125),2);line({0,0,0},{0,0,45},RGB(104,148,223),2);
    if(reference.wall) {
        const auto bounds=reference.ledge?WallBounds{-160,160,0,reference.ledgeZ}:reference.bounds;
        const auto columns=gridDivisions(bounds.right-bounds.left,40),rows=gridDivisions(bounds.top-bounds.bottom,32);
        for(unsigned i=0;i<=columns;++i){const float x=std::lerp(bounds.left,bounds.right,float(i)/float(columns));line({x,reference.wallY,bounds.bottom},{x,reference.wallY,bounds.top},RGB(83,83,83));}
        for(unsigned i=0;i<=rows;++i){const float z=std::lerp(bounds.bottom,bounds.top,float(i)/float(rows));line({bounds.left,reference.wallY,z},{bounds.right,reference.wallY,z},RGB(73,73,73));}
    }
    if(reference.ledge){line({-160,reference.wallY,reference.ledgeZ},{160,reference.wallY,reference.ledgeZ},RGB(165,165,165),3);
        line({-160,reference.wallY,reference.ledgeZ},{-160,reference.wallY+75,reference.ledgeZ},RGB(124,124,124));
        line({160,reference.wallY,reference.ledgeZ},{160,reference.wallY+75,reference.ledgeZ},RGB(124,124,124));
        line({-160,reference.wallY+75,reference.ledgeZ},{160,reference.wallY+75,reference.ledgeZ},RGB(124,124,124));}
}
inline std::array<std::wstring_view,2> previewCaptions(bool chinese,bool adapted=false,bool complete=false) {
    if(complete)return chinese?std::array<std::wstring_view,2>{L"左拖旋转 · 滚轮缩放 · 双击重置视图",L"完整墙跑 · 标准平墙 · 无游戏蒙皮 / 物理"}:
        std::array<std::wstring_view,2>{L"Drag to orbit · Wheel to zoom · Double-click to reset",L"Complete wall run · Standard flat wall · No game skin / physics"};
    if(adapted)return chinese?std::array<std::wstring_view,2>{L"左拖旋转 · 滚轮缩放 · 双击重置视图",L"标准路线示意 · 含控制器位移 · 非地图碰撞测试"}:
        std::array<std::wstring_view,2>{L"Drag to orbit · Wheel to zoom · Double-click to reset",L"Standard route + controller movement · Not a map collision test"};
    return chinese?std::array<std::wstring_view,2>{L"左拖旋转 · 滚轮缩放 · 双击重置视图",L"原始动作与源位移 · 未应用游戏中的贴墙适配"}:
        std::array<std::wstring_view,2>{L"Drag to orbit · Wheel to zoom · Double-click to reset",L"Source animation + motion · Game wall adaptation not applied"};
}
inline RECT previewCaptionRect(HDC dc,RECT area,std::wstring_view value,bool bottom) {
    TEXTMETRICW metrics{};GetTextMetricsW(dc,&metrics);const int padding=std::max(16,int(metrics.tmHeight)/2),margin=std::max(bottom?6:10,int(metrics.tmHeight)/3);
    RECT text{0,0,std::max(1,int(area.right-area.left)-2*padding),0};DrawTextW(dc,value.data(),int(value.size()),&text,DT_CALCRECT|DT_WORDBREAK|DT_NOPREFIX);
    const int height=text.bottom+2,top=bottom?area.bottom-margin-height:area.top+margin;return {area.left+padding,top,area.right-padding,top+height};
}
inline bool drawSkeleton(HDC dc,RECT area,const Pose& local,const std::vector<int>& parents,const std::vector<std::string>& names,
    const Camera& camera,const Reference& reference,std::array<float,4> contacts={},int selected=-1,bool chinese=false,bool adapted=false,bool complete=false) {
    const int width=area.right-area.left,height=area.bottom-area.top;if(width<1||height<1)return false;
    const int saved=SaveDC(dc);IntersectClipRect(dc,area.left,area.top,area.right,area.bottom);SetViewportOrgEx(dc,area.left,area.top,nullptr);
    background(dc,{0,0,width,height},RGB(23,23,23));drawReference(dc,camera,reference,width,height);const auto world=worldPose(local,parents);
    if(!world||names.size()!=local.size()) {label(dc,{24,height/2-30,width-24,height/2+30},chinese?L"导入 HKX 查看真实动作":L"Import an HKX to view its decoded animation",RGB(196,196,196));RestoreDC(dc,saved);return false;}
    struct Segment {std::size_t bone{};Projected a,b;};std::vector<Segment> segments;
    for(std::size_t i=0;i<world->size();++i)if(parents[i]>=0) {
        const auto a=project((*world)[std::size_t(parents[i])].t,camera,width,height),b=project((*world)[i].t,camera,width,height);
        if(a&&b&&(reference.joints||bodyBone(names[i])))segments.push_back({i,*a,*b});
    }
    std::stable_sort(segments.begin(),segments.end(),[](const auto& a,const auto& b){return a.a.depth+a.b.depth>b.a.depth+b.b.depth;});
    for(const auto& segment:segments) {
        const bool core=bodyBone(names[segment.bone]);const auto color=segment.bone==std::size_t(std::max(selected,0))&&selected>=0?RGB(255,234,131):limbColor(names[segment.bone]);
        const int thick=core?std::max(3,int(float(std::min(width,height))*.009f*std::sqrt(camera.zoom))):1;
        if(core)stroke(dc,pixel(segment.a),pixel(segment.b),RGB(15,15,15),thick+3);
        stroke(dc,pixel(segment.a),pixel(segment.b),color,thick);if(core)circle(dc,pixel(segment.b),std::max(2,thick/2),color);
    }
    for(std::size_t i=0;i<names.size();++i)if(names[i].find("Head [")!=std::string::npos)if(const auto p=project((*world)[i].t,camera,width,height)) {
        const int radius=std::max(5,int(float(std::min(width,height))*.024f*std::sqrt(camera.zoom)));circle(dc,pixel(*p),radius+2,RGB(15,15,15));circle(dc,pixel(*p),radius,RGB(228,228,228));
    }
    constexpr std::array<unsigned,4> points{38,39,50,51};
    for(unsigned i=0;i<4;++i)if(points[i]<world->size())if(const auto p=project(i<2&&world->size()==99?Library{}.palm(*world,int(i)):(*world)[points[i]].t,camera,width,height)) {
        const float c=std::clamp(contacts[i],0.f,1.f);const auto color=RGB(int(238-159*c),int(123+88*c),int(104+35*c));circle(dc,pixel(*p),5,color);
    }
    const auto captions=previewCaptions(chinese,adapted,complete);for(unsigned i=0;i<captions.size();++i)label(dc,previewCaptionRect(dc,{0,0,width,height},captions[i],i==1),captions[i],i==0?RGB(180,180,180):RGB(151,151,151),DT_LEFT|DT_WORDBREAK|DT_NOPREFIX);
    RestoreDC(dc,saved);return true;
}
inline std::array<float,4> sampleContacts(const std::vector<std::array<float,4>>& frames,float phase) {
    if(frames.empty())return {};const float f=std::clamp(phase,0.f,1.f)*float(frames.size()-1);const auto a=std::size_t(f),b=std::min(a+1,frames.size()-1);
    std::array<float,4> result{};for(unsigned i=0;i<4;++i)result[i]=frames[a][i]+(frames[b][i]-frames[a][i])*(f-float(a));return result;
}
struct TimelineLayout {int left{},right{},top{},row{},labelLeft{},labelRight{};};
inline std::array<std::wstring_view,4> timelineLabels(bool chinese) {
    constexpr std::array<std::wstring_view,4> en{L"Left hand",L"Right hand",L"Left foot",L"Right foot"},zh{L"左手",L"右手",L"左脚",L"右脚"};return chinese?zh:en;
}
inline TimelineLayout timelineLayout(HDC dc,RECT area,bool chinese=false) {
    TEXTMETRICW metrics{};GetTextMetricsW(dc,&metrics);int textWidth{},textHeight=metrics.tmHeight;
    for(const auto value:timelineLabels(chinese)){SIZE extent{};GetTextExtentPoint32W(dc,value.data(),int(value.size()),&extent);textWidth=std::max(textWidth,int(extent.cx));textHeight=std::max(textHeight,int(extent.cy));}
    const int padding=std::max(8,textHeight/2),gap=std::max(6,textHeight/3),labelLeft=area.left+padding,labelRight=labelLeft+textWidth;
    return {labelRight+gap,area.right-padding,area.top+padding,std::max(textHeight+2,(int(area.bottom-area.top)-2*padding)/4),labelLeft,labelRight};
}
inline void drawTimeline(HDC dc,RECT area,const std::vector<std::array<float,4>>& contacts,float phase,float trimIn,float trimOut,bool chinese=false) {
    const int saved=SaveDC(dc);IntersectClipRect(dc,area.left,area.top,area.right,area.bottom);background(dc,area,RGB(30,30,30));
    const auto layout=timelineLayout(dc,area,chinese);const int left=layout.left,right=layout.right,width=std::max(1,right-left),top=layout.top,row=layout.row;const auto names=timelineLabels(chinese);
    const std::array<COLORREF,4> colors{RGB(67,201,224),RGB(248,170,89),RGB(112,184,147),RGB(187,144,222)};
    for(unsigned track=0;track<4;++track) {
        const int y=top+int(track)*row;label(dc,{layout.labelLeft,y,layout.labelRight,y+row},names[track],RGB(209,209,209));
        stroke(dc,{left,y+row-3},{right,y+row-3},RGB(68,68,68));
        const auto pen=CreatePen(PS_SOLID,1,colors[track]);const auto previous=SelectObject(dc,pen);
        for(int x=0;x<width;++x) {
            const float value=sampleContacts(contacts,float(x)/float(width))[track];
            if(value>.01f){MoveToEx(dc,left+x,y+row-4,nullptr);LineTo(dc,left+x,y+row-4-int(value*float(row-6)));}
        }
        SelectObject(dc,previous);DeleteObject(pen);
    }
    const int current=left+int(std::clamp(phase,0.f,1.f)*float(width));stroke(dc,{current,area.top+4},{current,area.bottom-4},RGB(252,235,177),2);
    for(const float marker:{trimIn,trimOut}) {const int x=left+int(std::clamp(marker,0.f,1.f)*float(width));stroke(dc,{x,area.top+4},{x,area.bottom-4},RGB(214,214,214));}
    RestoreDC(dc,saved);
}
class Canvas {
    HDC dc_{};HBITMAP bitmap_{};HGDIOBJ old_{};void* pixels_{};int width_{},height_{};
public:
    Canvas(int width,int height):width_(width),height_(height) {
        if(width<1||height<1||width>8192||height>8192)throw std::runtime_error("Invalid preview dimensions");
        BITMAPINFO info{};info.bmiHeader.biSize=sizeof(BITMAPINFOHEADER);info.bmiHeader.biWidth=width;info.bmiHeader.biHeight=-height;info.bmiHeader.biPlanes=1;info.bmiHeader.biBitCount=32;info.bmiHeader.biCompression=BI_RGB;
        dc_=CreateCompatibleDC(nullptr);bitmap_=CreateDIBSection(dc_,&info,DIB_RGB_COLORS,&pixels_,nullptr,0);
        if(!dc_||!bitmap_){if(bitmap_)DeleteObject(bitmap_);if(dc_)DeleteDC(dc_);throw std::runtime_error("Unable to create preview canvas");}old_=SelectObject(dc_,bitmap_);
    }
    Canvas(const Canvas&)=delete;Canvas& operator=(const Canvas&)=delete;
    ~Canvas(){SelectObject(dc_,old_);DeleteObject(bitmap_);DeleteDC(dc_);}
    HDC dc()const{return dc_;}int width()const{return width_;}int height()const{return height_;}
    const std::uint32_t* pixels()const{GdiFlush();return static_cast<const std::uint32_t*>(pixels_);}
    bool save(const std::filesystem::path& path)const {
        GdiFlush();BITMAPFILEHEADER file{};BITMAPINFOHEADER info{};file.bfType=0x4d42;file.bfOffBits=sizeof(file)+sizeof(info);file.bfSize=file.bfOffBits+DWORD(width_*height_*4);
        info.biSize=sizeof(info);info.biWidth=width_;info.biHeight=-height_;info.biPlanes=1;info.biBitCount=32;info.biCompression=BI_RGB;info.biSizeImage=DWORD(width_*height_*4);
        std::ofstream out(path,std::ios::binary);out.write(reinterpret_cast<const char*>(&file),sizeof(file));out.write(reinterpret_cast<const char*>(&info),sizeof(info));out.write(static_cast<const char*>(pixels_),std::streamsize(info.biSizeImage));return bool(out);
    }
};
}
