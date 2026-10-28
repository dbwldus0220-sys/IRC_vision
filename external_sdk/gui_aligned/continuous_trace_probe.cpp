#include "gui_playback.hpp"
#include <iostream>
#include <iomanip>
#include <set>
int main(int argc,char**argv) {
    if(argc!=2)return 2;
    std::cout<<std::setprecision(17);
    const auto library=irc_step::MotionLibrary::loadGuiJson(argv[1]);
    for(const auto& name:library.names()) {
        irc_step::JointAngles initial;
        for(int id=0;id<23;++id) initial[id]=id*.25-3;
        const auto& p=library.motion(name);
        for(int repeat=1;repeat<=p.repeatCount();++repeat) {
            irc_step::GuiContinuousTrajectory trajectory;
            trajectory.reset(p.frames(),initial,p.playbackSpeed());
            std::set<double> times{double(p.durationMs())};
            for(double real_ms=0;real_ms*p.playbackSpeed()<p.durationMs();real_ms+=5)
                times.insert(real_ms*p.playbackSpeed());
            for(const auto& f:p.frames()) { times.insert(f.start_ms); times.insert(f.start_ms+f.time_ms); }
            for(double t:times) {
                auto sample=trajectory.sample(t);
                std::cout<<name<<'\t'<<repeat<<'\t'<<t;
                for(int id=0;id<23;++id) std::cout<<'\t'<<sample.angles[id]<<':'<<irc_step::guiPositionRaw(sample.angles[id]);
                std::cout<<'\n';
            }
            for(const auto& f:p.frames()) for(const auto& [id,a]:f.angles) initial[id]=a;
        }
    }
}
