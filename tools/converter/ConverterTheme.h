#pragma once
#include "ConverterPreview.h"
#include <commctrl.h>
#include <optional>

namespace fc::converter::theme {
inline constexpr COLORREF canvas=RGB(18,18,18),panel=RGB(28,28,28),field=RGB(36,36,36),hover=RGB(49,49,49),border=RGB(65,65,65),text=RGB(238,238,238),muted=RGB(164,164,164),disabled=RGB(110,110,110),accent=RGB(212,212,212);
inline constexpr COLORREF primaryFill=RGB(68,68,68),primaryHover=RGB(84,84,84),primaryPressed=RGB(54,54,54);
inline SIZE initialSize(int dpi,int width,int height){const int margin=MulDiv(24,dpi,96);return {std::min(MulDiv(1440,dpi,96),std::max(1,width-margin)),std::min(MulDiv(1000,dpi,96),std::max(1,height-margin))};}
inline void box(HDC dc,RECT r,COLORREF fill,COLORREF edge,int radius=7) {
    const auto brush=CreateSolidBrush(fill);const auto pen=CreatePen(PS_SOLID,1,edge);const auto a=SelectObject(dc,brush),b=SelectObject(dc,pen);
    RoundRect(dc,r.left,r.top,r.right,r.bottom,radius,radius);SelectObject(dc,b);SelectObject(dc,a);DeleteObject(pen);DeleteObject(brush);
}
inline std::wstring value(HWND item) {
    const int size=GetWindowTextLengthW(item);std::wstring result(std::size_t(size)+1,L'\0');GetWindowTextW(item,result.data(),size+1);result.resize(size);return result;
}
inline std::wstring comboValue(HWND item,int index) {
    if(index<0)return {};const auto count=SendMessageW(item,CB_GETLBTEXTLEN,WPARAM(index),0);if(count<0||count>4096)return {};
    std::wstring result(std::size_t(count)+1,L'\0');SendMessageW(item,CB_GETLBTEXT,WPARAM(index),reinterpret_cast<LPARAM>(result.data()));result.resize(std::size_t(count));return result;
}
inline void arrow(HDC dc,RECT r,COLORREF color,int direction=1) {
    const int x=(r.left+r.right)/2,y=(r.top+r.bottom)/2;visual::stroke(dc,{x-4,y-direction*2},{x,y+direction*2},color,2);visual::stroke(dc,{x,y+direction*2},{x+4,y-direction*2},color,2);
}
inline void scrollbar(HWND item,HDC dc) {
    SCROLLBARINFO info{sizeof(info)};if(!GetScrollBarInfo(item,OBJID_VSCROLL,&info)||info.rgstate[0]&STATE_SYSTEM_INVISIBLE)return;
    RECT window{};GetWindowRect(item,&window);auto r=info.rcScrollBar;OffsetRect(&r,-window.left,-window.top);visual::background(dc,r,panel);
    const int center=(r.left+r.right)/2;const int a=r.top+info.xyThumbTop,b=r.top+info.xyThumbBottom;
    if(b>a+2)box(dc,{center-2,a,center+3,b},border,border,5);
}
inline bool control(HWND item,HDC dc,int dpi,bool primary=false,bool compactPath=false,std::optional<UINT> buttonState={}) {
    wchar_t name[64]{};GetClassNameW(item,name,64);const std::wstring_view type(name);RECT r{};GetClientRect(item,&r);if(r.right<=0||r.bottom<=0)return true;
    const auto font=reinterpret_cast<HFONT>(SendMessageW(item,WM_GETFONT,0,0));const int saved=SaveDC(dc);IntersectClipRect(dc,r.left,r.top,r.right,r.bottom);if(font)SelectObject(dc,font);SetBkMode(dc,TRANSPARENT);
    const bool enabled=IsWindowEnabled(item)!=FALSE&&(!buttonState||!(*buttonState&ODS_DISABLED));
    const bool focused=buttonState?bool(*buttonState&ODS_FOCUS):GetFocus()==item;
    const bool hot=GetPropW(item,L"FreeClimbThemeHover")!=nullptr||(buttonState&&(*buttonState&ODS_HOTLIGHT));const auto color=enabled?text:disabled;
    const int pad=MulDiv(10,dpi,96);auto label=value(item);const auto style=GetWindowLongPtrW(item,GWL_STYLE);
    wchar_t parentName[64]{};GetClassNameW(GetParent(item),parentName,64);const auto surface=std::wstring_view(parentName)==L"FreeClimbEditorPanel"?panel:canvas;
    if(type==L"Button") {
        const bool checked=(style&BS_TYPEMASK)==BS_AUTOCHECKBOX;visual::background(dc,r,surface);
        if(checked){const int size=MulDiv(14,dpi,96),y=(r.bottom-size)/2;RECT tick{1,y,size+1,y+size};box(dc,tick,SendMessageW(item,BM_GETCHECK,0,0)==BST_CHECKED&&enabled?accent:field,focused?accent:border,3);
            if(SendMessageW(item,BM_GETCHECK,0,0)==BST_CHECKED){const auto ink=enabled?canvas:muted;visual::stroke(dc,{tick.left+3,tick.top+size/2},{tick.left+size/2-1,tick.bottom-4},ink,2);visual::stroke(dc,{tick.left+size/2-1,tick.bottom-4},{tick.right-3,tick.top+4},ink,2);}
            r.left+=MulDiv(24,dpi,96);if(style&BS_MULTILINE){RECT measured{0,0,r.right-r.left,0};DrawTextW(dc,label.data(),int(label.size()),&measured,DT_CALCRECT|DT_WORDBREAK|DT_NOPREFIX);r.top+=std::max<LONG>(0,(r.bottom-r.top-measured.bottom)/2);visual::label(dc,r,label,color,DT_LEFT|DT_WORDBREAK|DT_NOPREFIX);}else visual::label(dc,r,label,color,DT_LEFT|DT_VCENTER|DT_SINGLELINE|DT_END_ELLIPSIS|DT_NOPREFIX);
        }else{const bool pressed=enabled&&(buttonState?bool(*buttonState&ODS_SELECTED):bool(SendMessageW(item,BM_GETSTATE,0,0)&BST_PUSHED));
            const auto fill=!enabled?field:primary?(pressed?primaryPressed:hot?primaryHover:primaryFill):(pressed?border:hot?hover:field);
            box(dc,r,fill,focused?accent:primary&&enabled?muted:border,MulDiv(7,dpi,96));
            visual::label(dc,r,label,color,DT_CENTER|DT_VCENTER|DT_SINGLELINE|DT_NOPREFIX);}
    }else if(type==L"ComboBox") {
        visual::background(dc,r,surface);box(dc,r,enabled?field:panel,focused?accent:border,MulDiv(6,dpi,96));
        RECT textArea=r;textArea.left+=pad;textArea.right-=MulDiv(28,dpi,96);visual::label(dc,textArea,comboValue(item,int(SendMessageW(item,CB_GETCURSEL,0,0))),color,DT_LEFT|DT_VCENTER|DT_SINGLELINE|DT_END_ELLIPSIS|DT_NOPREFIX);
        RECT mark=r;mark.left=mark.right-MulDiv(26,dpi,96);arrow(dc,mark,enabled?muted:disabled);
    }else if(type==TRACKBAR_CLASSW) {
        visual::background(dc,r,surface);RECT thumb{},channel{};SendMessageW(item,TBM_GETTHUMBRECT,0,reinterpret_cast<LPARAM>(&thumb));SendMessageW(item,TBM_GETCHANNELRECT,0,reinterpret_cast<LPARAM>(&channel));
        const int y=(r.top+r.bottom)/2,x=(thumb.left+thumb.right)/2,thick=std::max(2,MulDiv(2,dpi,96));visual::stroke(dc,{channel.left,y},{channel.right,y},border,thick);
        if(enabled)visual::stroke(dc,{channel.left,y},{x,y},muted,thick);visual::circle(dc,{x,y},MulDiv(focused||hot?5:4,dpi,96),enabled?accent:disabled);
    }else if(type==WC_TABCONTROLW) {
        visual::background(dc,r,panel);const int selected=TabCtrl_GetCurSel(item);visual::stroke(dc,{0,0},{r.right,0},border);
        for(int i=0;i<TabCtrl_GetItemCount(item);++i){RECT tab{};TabCtrl_GetItemRect(item,i,&tab);std::array<wchar_t,96> textBuffer{};TCITEMW entry{};entry.mask=TCIF_TEXT;entry.pszText=textBuffer.data();entry.cchTextMax=int(textBuffer.size());TabCtrl_GetItem(item,i,&entry);
            if(i==selected){visual::background(dc,tab,field);visual::stroke(dc,{tab.left+8,tab.bottom-2},{tab.right-8,tab.bottom-2},accent,2);}else if(INT_PTR(GetPropW(item,L"FreeClimbHoverTab"))==i+1)visual::background(dc,tab,hover);visual::label(dc,tab,textBuffer.data(),i==selected?text:muted,DT_CENTER|DT_VCENTER|DT_SINGLELINE|DT_NOPREFIX);}
    }else if(type==L"Static") {
        visual::background(dc,r,surface);const auto alignment=(style&SS_TYPEMASK)==SS_RIGHT?DT_RIGHT:DT_LEFT;const auto layout=(style&SS_ELLIPSISMASK)==SS_ENDELLIPSIS?DT_SINGLELINE|DT_VCENTER|DT_END_ELLIPSIS:DT_WORDBREAK;visual::label(dc,r,label,color,alignment|layout|DT_NOPREFIX);
    }else if(type==L"Edit"&&compactPath) {
        visual::background(dc,r,canvas);label=label.empty()?L"—":std::filesystem::path(label).filename().wstring();r.left+=2;visual::label(dc,r,label,muted,DT_LEFT|DT_VCENTER|DT_SINGLELINE|DT_END_ELLIPSIS|DT_NOPREFIX);
    }else{RestoreDC(dc,saved);return false;}
    RestoreDC(dc,saved);return true;
}
}
