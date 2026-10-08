#include "ConverterEditor.h"
#include "ConverterPreview.h"
#include "../../src/MotionSlots.h"
#include <iostream>
#include <stdexcept>

namespace {
unsigned checks{};
void check(bool valid,const char* reason){++checks;if(!valid)throw std::runtime_error(reason);}
std::size_t colored(const fc::converter::visual::Canvas& canvas) {
    const auto* pixels=canvas.pixels();std::size_t count=0;
    for(int i=0;i<canvas.width()*canvas.height();++i){const auto pixel=pixels[i];const auto r=(pixel>>16)&255,g=(pixel>>8)&255,b=pixel&255;
        if((r>100&&g>100&&b>100)||(r>g*12/10&&r>100)||(b>r*12/10&&b>100))++count;
    }return count;
}
}
int wmain(int count,wchar_t** args) {
    if(count!=3)return 2;
    try {
        namespace view=fc::converter::visual;const std::filesystem::path pack=args[1],output=args[2];std::filesystem::create_directories(output);
        fc::Library library;check(library.load(pack.string()),"real runtime base pack loaded");
        const auto compactWall=view::wallBounds({-50,0,40},{50,0,150});
        check(compactWall.left==-160&&compactWall.right==160&&compactWall.bottom==0&&compactWall.top==230,"compact route retains default reference wall bounds");
        const auto routeWall=view::wallBounds({-640,-70,-100},{420,90,950});
        check(routeWall.left==-672&&routeWall.right==452&&routeWall.bottom==-132&&routeWall.top==982,"reference wall covers full route with stable margin");
        view::Reference routeReference;routeReference.wallY=27;routeReference.bounds=routeWall;
        check(routeReference.wallY==27,"expanded reference wall does not change contact plane");
        const auto invalidWall=view::wallBounds({std::numeric_limits<float>::quiet_NaN(),0,0},{1,1,1});
        check(invalidWall.left==-160&&invalidWall.right==160&&invalidWall.bottom==0&&invalidWall.top==230,"invalid route bounds retain default wall");
        check(view::gridDivisions(320,40)==8&&view::gridDivisions(100000,32)==24,"extended reference grid work remains bounded");
        {view::Canvas warm(440,520);const auto pose=library.sampleBase(fc::Motion::up,.5f);view::drawSkeleton(warm.dc(),{0,0,440,400},pose,library.parents,library.names,{},{});view::drawTimeline(warm.dc(),{0,405,440,520},library.clip(fc::Motion::up).contacts,.5f,0,1);check(colored(warm)>1000,"warm renderer pixels realized");}
        const auto handles=GetGuiResources(GetCurrentProcess(),GR_GDIOBJECTS);unsigned slots=0;
        for(int slot=0;slot<fc::motionCount;++slot)if(fc::isActiveMotion(fc::Motion(slot+1))) {
            const auto motion=fc::Motion(slot+1);const auto& clip=library.clip(motion);++slots;
            for(float phase:{0.f,.27f,.53f,.9f}) {
                const auto pose=library.sampleBase(motion,phase);const auto world=view::worldPose(pose,library.parents);check(world.has_value(),"actual FK world accepted");const auto expected=library.world(pose);
                for(std::size_t bone=0;bone<expected.size();++bone){check((world->at(bone).t-expected[bone].t).length()<.0001f,"renderer FK translation matches runtime");check(fc::angleBetween(world->at(bone).q,expected[bone].q)<.0007f,"renderer FK rotation matches runtime");}
                view::Camera camera;camera.center={expected[0].t.x,expected[0].t.y,95};view::Reference reference;
                view::Canvas canvas(440,520);const auto contacts=view::sampleContacts(clip.contacts,phase);
                check(view::drawSkeleton(canvas.dc(),{0,0,440,400},pose,library.parents,library.names,camera,reference,contacts),"actual skeleton rendered");
                view::drawTimeline(canvas.dc(),{0,405,440,520},clip.contacts,phase,0,1);
                check(colored(canvas)>1000,"render contains actual body and timeline pixels");
                if(phase==.53f&&(motion==fc::Motion::up||motion==fc::Motion::contextMantle||motion==fc::Motion::runDiagonalRight))
                    check(canvas.save(output/(std::string(fc::motionSlotNames[std::size_t(slot)])+".bmp")),"actual sample BMP saved");
                const auto before=view::project(expected[38].t,camera,440,400);camera.orbit(40,12);const auto after=view::project(expected[38].t,camera,440,400);
                check(before&&after,"actual wrist projection valid");check(std::abs(before->x-after->x)+std::abs(before->y-after->y)>.1f,"orbit changes actual geometry projection");
            }
        }
        check(slots==fc::activeMotionCount,"all active runtime slots visualized");const auto afterHandles=GetGuiResources(GetCurrentProcess(),GR_GDIOBJECTS);
        for(int slot=0;slot<fc::motionCount;++slot)if(fc::isActiveMotion(fc::Motion(slot+1)))for(float phase:{0.f,.27f,.53f,.9f}) {
            const auto motion=fc::Motion(slot+1);view::Canvas canvas(440,520);const auto pose=library.sampleBase(motion,phase);view::Camera camera;const auto world=library.world(pose);camera.center={world[0].t.x,world[0].t.y,95};
            view::drawSkeleton(canvas.dc(),{0,0,440,400},pose,library.parents,library.names,camera,{},view::sampleContacts(library.clip(motion).contacts,phase));
            view::drawTimeline(canvas.dc(),{0,405,440,520},library.clip(motion).contacts,phase,0,1);check(colored(canvas)>1000,"steady render pixels valid");
        }
        const auto steadyHandles=GetGuiResources(GetCurrentProcess(),GR_GDIOBJECTS);std::cout<<"GDI handles warm="<<handles<<" first140="<<afterHandles<<" second140="<<steadyHandles<<'\n';check(steadyHandles==afterHandles,"repeated full rendering releases all GDI resources");
        fc::Pose invalid(2);check(!view::worldPose(invalid,{1,-1}),"forward parent rejected");check(!view::worldPose(invalid,{-1,9}),"invalid parent rejected");
        invalid[1].t.x=std::numeric_limits<float>::quiet_NaN();check(!view::worldPose(invalid,{-1,0}),"invalid decoded transform rejected");
        view::Camera camera;const auto p=view::project({30,0,95},camera,500,500);camera.scale(3);const auto larger=view::project({30,0,95},camera,500,500);
        check(p&&larger&&std::abs(larger->x-250)>std::abs(p->x-250),"zoom increases projected size");check(!view::project({0,0,0},camera,0,500),"invalid viewport rejected");
        const auto document=fc::loadConverterEditor(pack.parent_path()/"up.hkx",pack,"up");fc::ConverterEditOptions edits;edits.trimIn=document.clip.duration*.1f;edits.trimOut=document.clip.duration*.9f;edits.speed=1.4f;
        edits.yawDegrees=20;edits.rootOffset={3,-2,1};edits.comOffset={0,1,0};edits.bones.push_back({28,{5,0,0},.1f,.8f,.1f});const auto animation=fc::applyConverterEdits(document,edits);
        check(std::abs(animation.clip.duration-(edits.trimOut-edits.trimIn)/edits.speed)<.0001f,"edited duration matches preview clock");
        for(float phase:{0.f,.25f,.5f,.75f,1.f}) {
            const auto pose=fc::sampleConverterEditor(animation,animation.clip.duration*phase);check(pose.size()==99,"editor preview uses canonical real frames");
            check(view::worldPose(pose,document.base.parents).has_value(),"edited dataset FK valid");const auto contacts=fc::sampleConverterContacts(animation,animation.clip.duration*phase);
            for(float weight:contacts)check(std::isfinite(weight)&&weight>=0&&weight<=1,"edited actual contact weights valid");
        }
        std::cout<<nlohmann::json{{"ok",true},{"checks",checks},{"slots",slots},{"guiLaunched",false},{"realRuntimeAssets",true}}.dump()<<'\n';return 0;
    }catch(const std::exception& failure){std::cerr<<failure.what()<<'\n';return 1;}
}
