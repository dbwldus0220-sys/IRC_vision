// Offline verification against the production-linked SDK; no serial library.
#include "robot_motion_player.hpp"
#include "mock_motion_hardware.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

int main(int argc, char** argv) {
    assert(argc == 3);
    using namespace irc_step;
    const auto reference = MotionLibrary::loadGuiJson(argv[2]);
    MockMotionHardware bus;
    auto now = RobotMotionPlayer::Clock::time_point(std::chrono::seconds(1));
    RobotMotionPlayer player(argv[1], bus, [&] { return now; });
    player.configureJointPolicy(argv[2], true, false);
    player.configureDiagnostics(5, "", true, 65536);
    player.setPositionToleranceEnabled(false);
    assert(player.initialize());
    std::size_t motion_count = 0, endpoint_count = 0;
    for (const auto& name : player.motionNames()) {
        JointAngles initial;
        for (int id = 0; id < 23; ++id) initial[id] = id * .25 - 3;
        bus.setPresentPositions(initial);
        const auto result = player.start(name);
        if (result != StartResult::Accepted) {
            std::cerr << name << ": " << player.lastError() << '\n';
            return 1;
        }
        for (int tick = 0; tick < 100000 && player.running(); ++tick) {
            player.update();
            now += std::chrono::milliseconds(5);
        }
        assert(player.succeeded());
        const auto trace = player.lastDiagnostics();
        assert(trace && trace->dropped_trace == 0 && trace->goal_failures == 0);
        const auto& original = reference.motion(name);
        assert(trace->final_corrections == original.frames().size() * original.repeatCount());
        for (const auto& tick : trace->trace) {
            if (!tick.final_goal) continue;
            const auto& frame = original.frames().at(tick.frame);
            for (int id : {4, 5}) {
                assert(std::abs(tick.goals[id] - frame.angles.at(id)) < 1e-8);
            }
            ++endpoint_count;
        }
        ++motion_count;
    }
    std::cout << "PASS motions=" << motion_count << " original_shoulder_endpoints="
              << endpoint_count << '\n';
}
