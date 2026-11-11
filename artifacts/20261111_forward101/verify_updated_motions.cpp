// Replay every changed motion with fake motor I/O and the deployed PC policy.
#include "robot_motion_player.hpp"
#include "mock_motion_hardware.hpp"

#include <cassert>
#include <chrono>
#include <iostream>

using namespace irc_step;

int main(int argc, char** argv) {
    assert(argc == 3);
    const auto catalog = MotionLibrary::loadGuiJson(argv[1]);
    int checked = 0;
    for (const auto& name : catalog.names()) {
        if (!name.starts_with("p전진") && name != "n공에서 전진하고 후진준비자세"
            && !name.starts_with("t골대후제자리우회전(6회)+")
            && !name.starts_with("p골대후제자리좌회전(4회)+")) continue;
        MockMotionHardware hardware;
        JointAngles initial;
        for (int id = 0; id < 23; ++id) initial[id] = 0.;
        hardware.setPresentPositions(initial);
        int ms = 0;
        RobotMotionPlayer player(argv[1], hardware, [&] {
            return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(ms));
        });
        player.configureJointPolicy(argv[2], true, false);
        player.setContinuousInterpolation(true);
        assert(player.initialize());
        assert(player.start(name) == StartResult::Accepted);
        for (; player.running() && ms < 15000; ms += 5) {
            player.update();
            hardware.setPresentPositions(hardware.commandedPositions());
        }
        assert(player.succeeded());
        assert(player.completionSequence() == 1);
        assert(hardware.commandedPositions() == catalog.motion(name).frames().back().angles);
        ++checked;
    }
    assert(checked == 15);
    std::cout << "PASS " << checked << " updated motions: start, complete, final joint targets\n";
}
