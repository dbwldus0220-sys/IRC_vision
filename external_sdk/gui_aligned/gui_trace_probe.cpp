#include "gui_playback.hpp"
#include <iostream>
#include <set>
int main(int argc,char**argv) {
    if(argc!=2)return 2;
    const auto library=irc_step::MotionLibrary::loadGuiJson(argv[1]);
    irc_step::JointAngles initial;
    for(int id=0;id<23;++id) initial[id]=id*0.25-3;
    for(const auto& name:library.names()) {
        const auto& p=library.motion(name);
        std::set<std::int64_t> times{p.durationMs()};
        for(std::int64_t t=0;t<p.durationMs();t+=5)times.insert(t);
        for(const auto& f:p.frames()) {times.insert(f.start_ms);times.insert(f.start_ms+f.time_ms);}
        for(auto t:times) {
            bool boundary=false;
            for(const auto& f:p.frames()) if(f.playback_cycle_end && f.start_ms+f.time_ms==t)boundary=true;
            auto target=irc_step::sampleGuiMotion(p.frames(),initial,t,boundary);
            std::cout<<name<<'\t'<<t<<'\t'<<boundary;
            for(int id=0;id<23;++id)std::cout<<'\t'<<(target.commanded[id]?irc_step::guiPositionRaw(target.angles[id]):-1);
            std::cout<<'\n';
        }
    }
}
