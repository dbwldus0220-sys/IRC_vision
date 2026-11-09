// Real playback code with an in-memory bus and explicit, assumed preparation delays.
// No ROS or Dynamixel backend is linked; results are simulated, not motor measurements.
#include "robot_motion_player.hpp"
#include <cassert>
#include <chrono>
#include <iostream>
using namespace irc_step;
struct MemoryBus final : IMotionHardware {
    int& clock;
    int delay, prepares = 0;
    JointAngles present;
    MemoryBus(int& value, int preparation_ms) : clock(value), delay(preparation_ms) {}
    bool initialize() noexcept override {
        for (int id=0; id<23; ++id) present[id]=0;
        return true;
    }
    bool ready() const noexcept override { return true; }
    bool prepareImmediatePlayback() noexcept override { ++prepares; clock+=delay; return true; }
    bool commandPosition(const JointAngles& values, const std::vector<int>& ids,
                         uint32_t, uint32_t) noexcept override {
        for (int id:ids) present[id]=values.at(id);
        return true;
    }
    bool readPresentPositions(JointAngles& values) noexcept override { values=present; return true; }
    bool holdCurrentPosition(uint32_t) noexcept override { return true; }
    bool setTorqueEnabled(bool) noexcept override { return true; }
    std::string_view lastError() const noexcept override { return {}; }
};
int main(int argc, char** argv) {
    assert(argc==2);
    std::cout << "queued,assumed_preparation_ms,transition_activation_gap_ms,prepare_calls\n";
    for (bool queued : {false,true}) for (int delay : {0,20,100}) {
        int ms=0;
        MemoryBus hardware(ms,delay);
        RobotMotionPlayer player(argv[1],hardware,[&] {
            return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(ms));
        });
        player.setContinuousInterpolation(true);
        player.setForwardBoundaryFeedbackAccounting(true);
        player.setPositionToleranceEnabled(false);
        assert(player.initialize());
        assert(player.start("t전진실전")==StartResult::Accepted);
        const int began=ms;
        const int duration=player.motionInfo("t전진실전").expected_duration_ms;
        bool reserved=false;
        while (player.completionSequence()==0 && ms<15000) {
            ms+=5;
            if (queued && !reserved && ms-began >= duration-170) {
                assert(player.queueNext("t라인복귀좌공용")==QueueResult::Queued);
                reserved=true;
            }
            player.update();
        }
        assert(player.completionSequence()==1);
        const int boundary=ms;
        if (!queued) assert(player.start("t라인복귀좌공용")==StartResult::Accepted);
        assert(player.currentMotion()=="t라인복귀좌공용");
        const int activation=ms;
        assert(activation-boundary==(queued ? 0 : delay));
        assert(hardware.prepares==(queued ? 1 : 2));
        while (player.running() && ms<15000) { ms+=5; player.update(); }
        assert(player.succeeded());
        std::cout << queued << ',' << delay << ',' << activation-boundary << ',' << hardware.prepares << '\n';
    }
}
