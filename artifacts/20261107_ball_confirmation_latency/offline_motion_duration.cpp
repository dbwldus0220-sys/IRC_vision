// Offline timing baseline. This executable has no ROS or Dynamixel backend.
#include "robot_motion_player.hpp"
#include <chrono>
#include <iostream>
#include <stdexcept>
using namespace irc_step;
struct MemoryHardware final : IMotionHardware {
    JointAngles present;
    bool initialize() noexcept override {
        for (int id = 0; id < 23; ++id) present[id] = 0;
        return true;
    }
    bool ready() const noexcept override { return true; }
    bool commandPosition(const JointAngles& values, const std::vector<int>& ids,
                         uint32_t, uint32_t) noexcept override {
        for (int id : ids) present[id] = values.at(id);
        return true;
    }
    bool readPresentPositions(JointAngles& values) noexcept override {
        values = present; return true;
    }
    bool holdCurrentPosition(uint32_t) noexcept override { return true; }
    bool setTorqueEnabled(bool) noexcept override { return true; }
    std::string_view lastError() const noexcept override { return {}; }
};
int main(int argc, char** argv) {
    if (argc != 2) return 2;
    MemoryHardware hardware;
    int64_t elapsed_ms = 0;
    RobotMotionPlayer player(argv[1], hardware, [&] {
        return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(elapsed_ms));
    });
    player.setContinuousInterpolation(true);
    player.setPositionToleranceEnabled(false);
    if (!player.initialize() || player.start("t라인복귀좌공용") != StartResult::Accepted)
        throw std::runtime_error("offline motion did not start");
    while (player.running() && elapsed_ms < 10000) {
        elapsed_ms += 5;
        player.update();
    }
    if (!player.succeeded()) throw std::runtime_error("offline motion did not finish");
    std::cout << "motion,tick_ms,communication_delay_ms,simulated_duration_ms\n"
              << "t라인복귀좌공용,5,0," << elapsed_ms << '\n';
}
