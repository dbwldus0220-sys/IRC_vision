#include "robot_motion_player.hpp"
#include "mock_motion_hardware.hpp"
#include "irc_step_motion_executor/startup_pose_catalog.hpp"
#include <cassert>
#include <iostream>
int main(int argc, char** argv) {
    assert(argc == 3);
    std::vector<double> pose;
    std::string error;
    assert(irc_step_motion_executor::load_startup_pose_with_policy(
        argv[1], argv[2], "오뒤무게중심앞", true, false, pose, error));
    irc_step::MockMotionHardware bus;
    auto now = irc_step::RobotMotionPlayer::Clock::time_point{};
    irc_step::RobotMotionPlayer player(argv[1], bus, [&] { return now; });
    player.configureJointPolicy(argv[2], true, false);
    assert(player.initialize());
    for (const auto& name : player.motionNames()) {
        irc_step::JointAngles initial;
        for (int id=0; id<23; ++id) initial[id] = pose[id];
        bus.setPresentPositions(initial);
        if (player.start(name) != irc_step::StartResult::Accepted) {
            std::cerr << name << ": " << player.lastError() << '\n'; return 1;
        }
        for (int tick=0; player.running() && tick<100000; ++tick) {
            now += std::chrono::milliseconds(5); player.update();
        }
        assert(player.succeeded());
    }
    std::cout << "PASS startup pose and policy-reference playback: " << player.motionNames().size() << " motions\n";
}
