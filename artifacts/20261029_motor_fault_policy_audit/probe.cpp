#include "dynamixel_motion_hardware.hpp"
#include "robot_motion_player.hpp"
#include <cassert>
#include <fstream>
#include <iostream>

using namespace irc_step;

int main(int argc, char** argv) {
    assert(argc == 2);
    const std::string path = std::string(argv[1]) + "/probe_motion.json";
    std::ofstream(path) << R"({"motions":[{"name":"test","repeatable":true,
      "start_pose":"same","end_pose":"same","max_seq_ms":100,
      "frames":[{"name":"frame","start_ms":0,"time_ms":100,
      "angles":{"1":20,"5":40}}]}]})";
    for (int scenario = 0; scenario < 6; ++scenario) {
        dynamixel::reset();
        DynamixelMotionHardware hardware(DynamixelMotionHardwareConfig{});
        int ms = 1000;
        RobotMotionPlayer player(path, hardware, [&] {
            return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(ms));
        });
        player.setPositionToleranceEnabled(false);
        player.setContinuousInterpolation(true);
        assert(player.initialize());
        assert(player.start("test") == StartResult::Accepted);
        player.update();
        if (scenario == 5) assert(player.queueNext("test") == QueueResult::Queued);
        if (scenario == 1) dynamixel::registers[5][64] = 0;
        if (scenario == 2) dynamixel::torque_read_failure = 5;
        if (scenario == 3 || scenario == 5) dynamixel::offline = 5;
        if (scenario == 4) dynamixel::write_failure = true;
        for (int tick = 0; player.running() && tick < 100; ++tick) {
            ms += 5;
            player.update();
        }
        const bool completed = player.succeeded();
        assert(completed == (scenario != 4));
        assert(!player.completionConfirmsArrival());
        if (scenario == 5) assert(player.completionSequence() == 2);
        const auto next = player.start("test");
        bool expected_accepted = scenario == 0;
#ifdef OMIT_ADDED_TORQUE_GUARD
        expected_accepted = scenario <= 2;
#endif
        assert((next == StartResult::Accepted) == expected_accepted);
        const char* names[] = {"healthy", "motor5_torque_off", "motor5_torque_read_failure",
                              "motor5_offline", "bus_write_failure", "queued_motor5_offline"};
        std::cout << names[scenario] << ": position_tolerance=false; current="
                  << (completed ? "SUCCEEDED_without_arrival_confirmation" : "FAILED")
                  << "; next=" << (expected_accepted ? "ACCEPTED" : "HardwareNotReady")
                  << "; detail=" << player.lastError() << '\n';
    }
}
