#include "Pose.h"
#include "TraversalAudio.h"
#include <iostream>
#include <stdexcept>
using namespace fc;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
int main(int argc,char** argv) {
    try {
        require(argc==2,"motion path required");Library library;require(library.load(argv[1]),"load actual library");
        for(int fps:{30,60,120}) {
            TraversalAudio audio;unsigned sounds=0;const float dt=1.f/fps;
            const auto& clip=library.clip(Motion::runDiagonalRight);
            for(int frame=0;frame<fps*3;++frame) {
                const float seconds=frame*dt,phase=std::fmod(seconds/clip.seconds,1.f);
                auto out=audio.update(Motion::runDiagonalRight,phase,{seconds*330,0,0},library.contactWeights(Motion::runDiagonalRight,phase),dt,true);
                sounds+=out.count;require(out.count<=1,"no contact burst");
                require(!audio.update(Motion::runDiagonalRight,phase,{seconds*330,0,0},{},0,true).count,"duplicate zero-time callback is silent");
            }
            require(sounds>=8&&sounds<=10,"run cadence follows source two support onsets per cycle at every fps");
            unsigned fast=0;audio.reset();
            for(int frame=0;frame<fps*3;++frame) {
                const float seconds=frame*dt,phase=std::fmod(seconds*1.15f/clip.seconds,1.f);
                fast+=audio.update(Motion::runDiagonalRight,phase,{seconds*379.5f,0,0},{},dt,true).count;
            }
            require(fast>sounds,"faster real gait also accelerates sound cadence");
            audio.reset();Vec stuck{10,20,30};
            for(int frame=0;frame<fps*3;++frame) {
                float phase=std::fmod(frame*dt,1.f);
                require(!audio.update(Motion::runDiagonalRight,phase,stuck,{1,1,1,1},dt,true).count,"blocked wall run never loops footsteps");
            }
            for(auto motion:{Motion::hang,Motion::contextHang})for(int i=0;i<fps;++i)
                require(!audio.update(motion,std::fmod(i*dt,1.f),stuck,{1,1,1,1},dt,true).count,"stationary holds remain quiet");
            for(auto motion:{Motion::up,Motion::down,Motion::left,Motion::right}) {
                audio.reset();unsigned grips=0,steps=0;const auto& c=library.clip(motion);
                for(int frame=0;frame<fps*4;++frame) {
                    float phase=std::fmod(frame*dt/c.seconds,1.f);
                    auto out=audio.update(motion,phase,{frame*dt*82,0,0},library.contactWeights(motion,phase),dt,true);
                    for(unsigned i=0;i<out.count;++i){grips+=out.items[i].cue==SoundCue::grip;steps+=out.items[i].cue==SoundCue::step;}
                }
                require(grips+steps>=3&&grips+steps<30,"climb audio uses actual source contacts with bounded rate");
            }
            for(auto motion:{Motion::jumpCatch,Motion::ledgeCatch,Motion::hopLeft,Motion::contextHopLeft,Motion::contextHopRight,Motion::backFlipOut,Motion::contextMantle}) {
                audio.reset();unsigned grips=0,pushes=0,tops=0;
                for(int frame=0;frame<=fps*2;++frame) {
                    const float phase=std::min(1.f,frame*dt/1.5f);
                    const auto out=audio.update(motion,phase,{phase*100,0,0},{},dt,true,motion==Motion::contextMantle&&phase==1);
                    for(unsigned i=0;i<out.count;++i){grips+=out.items[i].cue==SoundCue::grip;pushes+=out.items[i].cue==SoundCue::push;tops+=out.items[i].cue==SoundCue::top;}
                }
                if(motion==Motion::backFlipOut)require(pushes==1&&grips==0,"outward exit has one push and no imaginary catch");
                else if(motion==Motion::contextMantle)require(grips==1&&tops==1,"top-out contacts and completion occur only once");
                else require(grips==1,"retained original entry and side-hop catch occurs once");
            }
            audio.reset();
            for(int id:{12,13,14,33,38})for(int frame=0;frame<=fps*2;++frame) {
                const float phase=std::min(1.f,frame*dt/1.5f);
                require(!audio.update(Motion(id),phase,{0,0,-phase*62},{1,1,1,1},dt,true,true).count,
                    "retired motions cannot produce traversal sound cues or completion sounds");
            }
            audio.reset();unsigned entrySteps=0;
            for(int frame=0;frame<=fps;++frame) {
                const float phase=std::min(1.f,frame*dt/.4f);
                const auto out=audio.update(Motion::runLaunch,phase,{0,phase*50,phase*18},{},dt,true);
                for(unsigned i=0;i<out.count;++i) {
                    require(out.items[i].cue==SoundCue::step&&phase>=.94f,"wall-run entry sounds a late foot plant, never an early hanging hand catch");
                    ++entrySteps;
                }
                require(!audio.update(Motion::runLaunch,phase,{0,phase*50,phase*18},{},0,true).count,"entry zero-time callbacks cannot repeat the foot plant");
            }
            require(entrySteps==1,"wall-run entry has one contact sound at every frame rate");
            for(auto motion:{Motion::kickUp,Motion::kickLeft,Motion::kickRight}) {
                audio.reset();unsigned pushes=0,steps=0,grips=0;
                for(int frame=0;frame<=int(std::ceil(.40f/dt))+2;++frame) {
                    const float phase=std::min(1.f,frame*dt/.40f);
                    const auto out=audio.update(motion,phase,{phase*160,0,0},{},dt,true);
                    for(unsigned i=0;i<out.count;++i) {
                        pushes+=out.items[i].cue==SoundCue::push;
                        steps+=out.items[i].cue==SoundCue::step;
                        grips+=out.items[i].cue==SoundCue::grip;
                        if(out.items[i].cue==SoundCue::step)require(phase>=.95f,"wall kick landing sound waits until the checked return");
                    }
                    require(!audio.update(motion,phase,{phase*160,0,0},{},0,true).count,"fast kick zero-time callbacks stay silent");
                }
                require(pushes==1&&steps==1&&grips==0,"fast wall kick has one push and one returning footstep, not a hanging hand catch");
            }
            require(!audio.update(Motion::runUp,.8f,{100,0,0},{},dt,false).count,"unavailable pose output cannot produce sound");
            require(!audio.update(Motion::runLeft,.6f,{101,0,0},{},dt,true).count,"refresh/new phase cannot synthesize footsteps");
            std::cout<<"audio fps="<<fps<<" run="<<sounds<<" faster="<<fast<<" passed\n";
        }
        return 0;
    } catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
