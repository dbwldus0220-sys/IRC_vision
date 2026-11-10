// Exercise production trajectories with memory hardware and a simulated clock.
#include "robot_motion_player.hpp"
#include <cassert>
#include <chrono>
#include <iostream>
#include <vector>
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
    bool readPresentPositions(JointAngles& values) noexcept override { values = present; return true; }
    bool holdCurrentPosition(uint32_t) noexcept override { return true; }
    bool setTorqueEnabled(bool) noexcept override { return true; }
    std::string_view lastError() const noexcept override { return {}; }
};
int main(int argc, char** argv) {
    assert(argc == 3);
    const std::vector<std::vector<std::string>> sequences{
        {"p전진45(4회)", "p제자리좌회전45(5회)", "p전진45(4회)"},
        {"p전진45(4회)", "p제자리좌회전45(1회)", "p전진45(4회)"},
        {"p전진45(4회)", "p제자리좌회전45(2회)", "p전진45(4회)"},
        {"p전진45(4회)", "p제자리좌회전45(3회)", "p전진45(4회)"},
        {"p전진45(4회)", "p제자리좌회전45(4회)", "p전진45(4회)"},
        {"p전진45(4회)", "p제자리좌회전45(6회)", "p전진45(4회)"},
        {"p전진실전0도(4회)", "p제자리좌회전0도(1회)", "p전진실전0도(4회)"},
        {"p전진실전0도(4회)", "p제자리좌회전0도(2회)", "p전진실전0도(4회)"},
        {"p전진실전0도(4회)", "p제자리좌회전0도(3회)", "p전진실전0도(4회)"},
        {"p전진실전0도(4회)", "p제자리좌회전0도(4회)", "p전진실전0도(4회)"},
        {"p전진실전0도(4회)", "p제자리좌회전0도(5회)", "p전진실전0도(4회)"},
        {"p전진실전0도(4회)", "p제자리좌회전0도(6회)", "p전진실전0도(4회)"},
        {"p전진90도(4회)", "p제자리좌회전90도(1회)", "p전진90도(4회)"},
        {"p전진90도(4회)", "p제자리좌회전90도(2회)", "p전진90도(4회)"},
        {"p전진90도(4회)", "p제자리좌회전90도(3회)", "p전진90도(4회)"},
        {"p전진90도(4회)", "p제자리좌회전90도(4회)", "p전진90도(4회)"},
        {"p전진90도(4회)", "p제자리좌회전90도(5회)", "p전진90도(4회)"},
        {"p전진90도(4회)", "p제자리좌회전90도(6회)", "p전진90도(4회)"},
        {"p전진45(4회)", "p공후 제자리좌회전", "p전진45(4회)"},
        {"p전진45(4회)", "p골대후제자리좌회전", "p전진45(4회)"},
    };
    std::cout << "tolerance_enabled,motion,expected_ms,simulated_ms\n";
    // Production GUI playback disables arrival waits; enabling them is unsupported.
    for (bool tolerance : {false}) {
        for (const auto& sequence : sequences) {
            MemoryHardware hardware;
            int64_t elapsed = 0;
            RobotMotionPlayer player(argv[1], hardware, [&] {
                return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(elapsed));
            });
            player.setContinuousInterpolation(true);
            player.setPositionToleranceEnabled(tolerance);
            player.configureJointPolicy(argv[2], false, false);
            assert(player.initialize());
            for (const auto& name : sequence) {
                const auto info = player.motionInfo(name);
                assert(info.expected_duration_ms + 3000 < 12000);
                assert(player.start(name) == StartResult::Accepted);
                const auto start = elapsed;
                while (player.running() && elapsed - start < 12000) {
                    elapsed += 5;
                    player.update();
                }
                if (!player.succeeded()) std::cerr << name << ": " << player.lastError() << '\n';
                assert(player.succeeded());
                std::cout << tolerance << ',' << name << ',' << info.expected_duration_ms
                          << ',' << elapsed - start << '\n';
                elapsed += name == "찐공확인자세" ? 3000 : 500;
            }
        }
    }
}
