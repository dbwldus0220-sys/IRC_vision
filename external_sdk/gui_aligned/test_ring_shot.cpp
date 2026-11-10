#include "robot_motion_player.hpp"
#include "mock_motion_hardware.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <iostream>

using namespace irc_step;

int main(int argc, char** argv) {
    assert(argc == 2);
    const auto library = MotionLibrary::loadGuiJson(argv[1]);
    const std::vector<std::string> shots{
        "R골넣기1번구역", "R골넣기2번구역", "r골넣기8번구역",
        "r골넣기 5번구역", "r골넣기 6번구역", "r골넣기7번구역"};
    for (const auto& shot : shots) for (double angle : {-23.456, 0.0, 17.891}) {
        MockMotionHardware hardware;
        JointAngles initial;
        for (int id = 0; id < 23; ++id) initial[id] = 0.0;
        hardware.setPresentPositions(initial);
        std::int64_t ms = 0;
        RobotMotionPlayer player(argv[1], hardware, [&] {
            return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(ms));
        });
        assert(player.initialize());
        assert(player.start(shot, angle) == StartResult::Accepted);
        ms += 500;
        player.update();
        assert(std::abs(hardware.commandedPositions().at(11) - (0.47265625 + angle)) < 1e-8);
        assert(std::abs(hardware.commandedPositions().at(3) - -96.0) < 1e-8);
        assert(player.start(shot, -angle) == StartResult::RejectedBusy);
        const auto& frames = library.motion(shot).frames();
        // Hold the motor-signed aim through all throwing frames and their gaps.
        const auto return_start_ms = frames.back().start_ms;
        while (ms < return_start_ms) {
            ms = std::min(ms + 10, return_start_ms);
            player.update();
            assert(player.running());
            assert(std::abs(hardware.commandedPositions().at(11) - (0.47265625 + angle)) < 1e-8);
        }
        for (int i = 0; player.running() && i < 400; ++i) { ms += 10; player.update(); }
        assert(player.succeeded());
        assert(std::abs(hardware.commandedPositions().at(11) - -0.615234375) < 1e-8);
        // Neither the loaded catalog nor the next playback inherits the delta.
        assert(player.start(shot) == StartResult::Accepted);
        ms += 500; player.update();
        assert(std::abs(hardware.commandedPositions().at(11) - 0.47265625) < 1e-8);
        assert(player.cancel() == CancelResult::Cancelled);
        assert(player.start(shot, std::numeric_limits<double>::quiet_NaN()) == StartResult::InvalidMotion);
        assert(player.start(shot, 90.0) == StartResult::InvalidMotion);
        assert(player.start(shot, angle) == StartResult::Accepted);
        hardware.setCommandSuccess(false);
        ms += 500; player.update();
        assert(!player.running());
        hardware.setCommandSuccess(true);
        assert(player.start(shot) == StartResult::Accepted);
        ms += 500; player.update();
        assert(std::abs(hardware.commandedPositions().at(11) - 0.47265625) < 1e-8);
    }
    std::cout << "PASS 6 regions: waist delta, original return pose, busy/cancel/failure isolation\n";
}
