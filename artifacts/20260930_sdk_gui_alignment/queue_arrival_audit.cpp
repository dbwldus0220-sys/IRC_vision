#include "robot_motion_player.hpp"
#include "gui_playback.hpp"
#include <cassert>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
using namespace irc_step;
using namespace std::chrono_literals;
struct Hardware : IMotionHardware {
    JointAngles present, goal;
    bool online=false, read_ok=true, write_ok=true, follow=false;
    int writes=0, reads=0, torque_writes=0;
    std::function<void()> after_write=[]{};
    Hardware() { for(int i=0;i<23;++i) present[i]=0; }
    bool initialize() noexcept override { online=true; return true; }
    bool ready() const noexcept override { return online; }
    bool commandPosition(const JointAngles&,const std::vector<int>&,uint32_t,uint32_t) noexcept override {
        assert(false && "time-profile path must never be used"); return false;
    }
    bool commandImmediatePosition(const JointAngles& g,const std::vector<int>& ids) noexcept override {
        ++writes; if(!write_ok)return false;
        for(int id:ids) { goal[id]=g.at(id); if(follow)present[id]=g.at(id); }
        after_write(); return true;
    }
    bool readPresentPositions(JointAngles& p) noexcept override { ++reads; if(!read_ok)return false; p=present;return true; }
    bool holdCurrentPosition(uint32_t duration) noexcept override { assert(duration==0);if(!read_ok)return false;goal=present;return true; }
    bool setTorqueEnabled(bool) noexcept override { ++torque_writes;return true; }
    std::string_view lastError() const noexcept override {return "injected hardware failure";}
};
int main() {
    const auto path=std::filesystem::temp_directory_path()/"step_queue_arrival_audit.json";
    std::ofstream(path)<<R"({"motions":[{"name":"turn","max_seq_ms":100,"repeat_count":1,"playback_speed":1,"start_pose":"ready","end_pose":"ready","frames":[{"name":"normal","start_ms":0,"time_ms":100,"angles":{"1":90}}]}]})";
    for (const bool queued : {false, true}) {
        RobotMotionPlayer::Clock::time_point now=RobotMotionPlayer::Clock::time_point{}+1s;
        Hardware hw;
        RobotMotionPlayer p(path.string(),hw,[&]{return now;});
        assert(p.initialize());
        p.setPositionToleranceEnabled(true);
        assert(p.start("turn")==StartResult::Accepted);
        if(queued) assert(p.queueNext("turn")==QueueResult::Queued);
        now+=100ms;
        const auto status=p.update();
        assert(hw.present.at(1)==0);
        if(queued) {
            assert(status==MotionStatus::Running && p.completionSequence()==1);
            std::cout<<"arrival_check=true queued=true present=0 goal=90 completionSequence=1 next motion RUNNING\n";
        } else {
            assert(status==MotionStatus::Settling && p.completionSequence()==0);
            std::cout<<"arrival_check=true queued=false present=0 goal=90 completionSequence=0 SETTLING\n";
        }
    }
    std::filesystem::remove(path);
}
