#include "ConverterRuntimePreview.h"
#include <iostream>
#include <stdexcept>

using namespace fc;
static void require(bool value,const char* reason){if(!value)throw std::runtime_error(reason);}
static float difference(const Pose& a,const Pose& b) {
    require(a.size()==99&&b.size()==99,"preview output is a canonical 99-bone pose");float maximum=0;
    for(std::size_t i=0;i<a.size();++i){maximum=std::max(maximum,angleBetween(a[i].q,b[i].q));maximum=std::max(maximum,(a[i].t-b[i].t).length());}
    return maximum;
}
int main(int argc,char** argv)try {
    require(argc==2,"base pack argument required");const std::filesystem::path pack=argv[1];
    for(const auto slot:{"runUp","runLeft","runRight","runDiagonalLeft","runDiagonalRight"}) {
        const auto doc=loadConverterBaseEditor(pack,slot);const auto edited=applyConverterEdits(doc,{});const auto original=edited.clip.frames;
        const auto preview=buildConverterRuntimePreview(doc,edited),repeat=buildConverterRuntimePreview(doc,edited);
        require(preview.available()&&preview.surfaceQueries>0,"base wall-run preview invokes actual runtime pose with wall contacts");
        require(preview.frames.size()==repeat.frames.size(),"repeat runtime preview has stable cached sampling");
        const auto queries=preview.surfaceQueries;float nonRootChange=0;
        for(float phase:{0.f,.1f,.25f,.5f,.9f,1.f,.5f,.1f}) {
            const auto actual=sampleConverterRuntimePreview(preview,phase*preview.seconds),raw=sampleConverterEditor(edited,phase*edited.clip.duration);
            require(difference(actual,sampleConverterRuntimePreview(repeat,phase*repeat.seconds))<.0001f,"seek order cannot change cached runtime pose");
            require(angleBetween(actual[0].q,raw[0].q)>.7f,"base wall-run preview includes the actual wall-facing rotation missing from the raw HKX");
            for(std::size_t bone=1;bone<actual.size();++bone) {
                nonRootChange=std::max(nonRootChange,angleBetween(actual[bone].q,raw[bone].q));
                require(actual[bone].t.finite()&&std::abs(actual[bone].q.dot(actual[bone].q)-1)<.002f,"adapted preview remains finite and normalized");
                if(bone!=4)require((actual[bone].t-raw[bone].t).length()<.001f,"runtime preview retains structural bone lengths");
            }
        }
        if(std::string_view(slot)!="runUp")require(nonRootChange>.3f,"side and diagonal previews include the actual brace and limb adaptation");
        require(preview.surfaceQueries==queries,"scrubbing cached poses never issues a new collision query");
        for(std::size_t frame=0;frame<original.size();++frame)require(difference(original[frame],edited.clip.frames[frame])<.0001f,"adapted preview cannot change exported source curves");
        auto source=doc;source.authored=true;
        source.clip.frames.assign(121,doc.base.clip(Motion::hang).frames.front());source.clip.duration=1;
        for(std::size_t frame=0;frame<source.clip.frames.size();++frame) {
            const float phase=float(frame)/120;source.clip.frames[frame][0].t={2*std::sin(phase*6.2831853f),0,80*phase};
            source.clip.frames[frame][24].q=(Quat::axis({0,0,1},.1f*std::sin(phase*6.2831853f))*source.clip.frames[frame][24].q).unit();
        }
        ConverterEditOptions options;for(unsigned hand=0;hand<4;++hand)options.contacts.push_back({hand,0,1,0,0});
        const auto authored=applyConverterEdits(source,options);const auto adapted=buildConverterRuntimePreview(source,authored);
        require(adapted.available(),"authored wall-run source supports the same runtime preview path");
        float maxRoot=0,maxRotation=0;
        for(float phase:{.1f,.25f,.5f,.75f,.9f}) {
            const auto actual=sampleConverterRuntimePreview(adapted,phase*adapted.seconds),raw=sampleConverterEditor(authored,phase*authored.clip.duration);
            maxRoot=std::max(maxRoot,(actual[0].t-(raw[0].t-Vec{0,0,80*phase})).length());
            for(std::size_t bone=0;bone<actual.size();++bone)maxRotation=std::max(maxRotation,angleBetween(actual[bone].q,raw[bone].q));
        }
        require(maxRoot<.005f,"authored preview removes consumed Root once and retains local source sway");
        require(maxRotation<.002f,"authored preview does not inject legacy sideBrace or sideways run rotations");
        const auto braceDoc=loadConverterBaseEditor(pack,"sideBrace",std::string_view(slot)=="runUp"?"runLeft":slot);ConverterEditOptions braceOptions;
        for(const auto bone:{29u,32u,38u,39u,67u,82u})braceOptions.bones.push_back({bone,{20,0,0},0,1,.01f});
        const auto braceEdited=applyConverterEdits(braceDoc,braceOptions);
        if(std::string_view(slot)!="runUp") {
            const std::vector<ConverterEditorExport> overlays{{&braceDoc,&braceEdited}};
            const auto changed=buildConverterRuntimePreview(doc,edited,overlays);float privateDifference=0;
            for(float phase:{.1f,.25f,.5f,.75f,.9f})privateDifference=std::max(privateDifference,difference(sampleConverterRuntimePreview(changed,phase*changed.seconds),sampleConverterRuntimePreview(preview,phase*preview.seconds)));
            require(privateDifference>.005f,"editing the current private reference changes only its owning wall-run preview");
            const auto isolated=buildConverterRuntimePreview(source,authored,overlays);
            for(float phase:{.1f,.5f,.9f})require(difference(sampleConverterRuntimePreview(isolated,phase*isolated.seconds),sampleConverterRuntimePreview(adapted,phase*adapted.seconds))<.0001f,"private legacy reference cannot deform an authored full-body loop");
        }
        auto invalidBrace=braceDoc;invalidBrace.slot="contextHang";bool rejected=false;
        try{buildConverterRuntimePreview(doc,edited,{{&invalidBrace,&braceEdited}});}catch(const std::exception&){rejected=true;}
        require(rejected,"unrelated action group cannot leak into a preview overlay");
        std::cout<<slot<<" runtimeFrames="<<preview.frames.size()<<" wallQueries="<<queries<<" braceChange="<<nonRootChange<<" sourceRotation="<<maxRotation<<" rootError="<<maxRoot<<'\n';
    }
    const auto doc=loadConverterBaseEditor(pack,"hang");require(!buildConverterRuntimePreview(doc,applyConverterEdits(doc,{})).available(),"unsupported scene actions remain explicitly source-only previews");
    std::cout<<"Runtime wall-run preview, source isolation, Root and deterministic seek checks passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
