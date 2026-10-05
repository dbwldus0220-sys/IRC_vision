#include "robot_motion_player.hpp"
#include "gui_playback.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
using namespace irc_step;
struct Bus : IMotionHardware {
    double ms{1000}, read_delay{0}, write_delay{0};
    JointAngles present;
    bool initialize() noexcept override { return true; }
    bool ready() const noexcept override { return true; }
    bool commandPosition(const JointAngles&, const std::vector<int>&, uint32_t, uint32_t) noexcept override { return false; }
    bool commandImmediatePosition(const JointAngles& goal, const std::vector<int>& ids) noexcept override {
        std::cout << "W\t" << ms;
        for (int id : ids) std::cout << '\t' << id << ':' << goal.at(id) << ':' << guiPositionRaw(goal.at(id));
        std::cout << '\n'; ms += write_delay; return true;
    }
    bool readPresentPositions(JointAngles& value) noexcept override {
        std::cout << "R\t" << ms << '\n'; ms += read_delay; value = present; return true;
    }
    bool holdCurrentPosition(uint32_t) noexcept override { return true; }
    bool setTorqueEnabled(bool) noexcept override { return true; }
    std::string_view lastError() const noexcept override { return {}; }
};
int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--raw") {
        std::cout << std::setprecision(17);
        for (int raw = 0; raw < 4095; ++raw) {
            const double angle = (raw + .5 - 2048) * (360.0 / 4096.0);
            for (double value : {angle, std::nextafter(angle, -INFINITY), std::nextafter(angle, INFINITY)})
                std::cout << value << '\t' << guiPositionRaw(value) << '\n';
        }
        for (double angle : {-1080., -540., -180., 180., 540., 1080.})
            std::cout << angle << '\t' << guiPositionRaw(angle) << '\n';
        return 0;
    }
    if (argc < 6) return 1;
    std::cout << std::setprecision(17);
    Bus bus; bus.read_delay=std::stod(argv[4]); bus.write_delay=std::stod(argv[5]);
    for(int id=0; id<23; ++id) bus.present[id]=id*.25-3;
    RobotMotionPlayer p(argv[1],bus,[&]{return RobotMotionPlayer::Clock::time_point(
        std::chrono::duration_cast<RobotMotionPlayer::Clock::duration>(std::chrono::duration<double,std::milli>(bus.ms)));});
    std::vector<double> periods; std::istringstream in(argv[3]); std::string token;
    while(std::getline(in,token,',')) periods.push_back(std::stod(token));
    if(argc>7) p.configureDiagnostics(5,argv[7],true);
    if(!p.initialize() || p.start(argv[2])!=StartResult::Accepted) { std::cerr<<p.lastError(); return 2; }
    if(argc>6 && std::string(argv[6])!="-" && p.queueNext(argv[6])!=QueueResult::Queued) return 3;
    std::uint64_t completed=0;
    for(int tick=0; tick<1000000; ++tick) {
        bus.ms += periods[tick%periods.size()];
        p.update();
        if(p.completionSequence()!=completed) {
            completed=p.completionSequence();std::cout<<"C\t"<<bus.ms<<'\t'<<completed<<'\n';
        }
        std::cout<<"T\t"<<bus.ms<<'\t'<<p.playbackTimelineMs()<<'\t'<<p.playbackRepeat()<<'\t'<<p.running()<<'\n';
        if(!p.running()) return p.succeeded()?0:4;
    }
    return 5;
}
