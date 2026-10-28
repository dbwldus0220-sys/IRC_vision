#include "robot_motion_player.hpp"
#include <cassert>
#include <filesystem>
#include <fstream>

using namespace irc_step;

struct TimingHardware : IMotionHardware {
    JointAngles goals, present;
    int prepares = 0;
    bool initialize() noexcept override {
        for (int id = 0; id < 23; ++id) present[id] = 0;
        return true;
    }
    bool ready() const noexcept override { return true; }
    bool prepareImmediatePlayback() noexcept override { ++prepares; return true; }
    bool commandPosition(const JointAngles& values, const std::vector<int>& ids,
                         uint32_t duration, uint32_t acceleration) noexcept override {
        assert(duration == 0 && acceleration == 0);
        for (int id : ids) goals[id] = values.at(id);
        return true;
    }
    bool readPresentPositions(JointAngles& values) noexcept override { values = present; return true; }
    bool holdCurrentPosition(uint32_t) noexcept override { return true; }
    bool setTorqueEnabled(bool) noexcept override { return true; }
    std::string_view lastError() const noexcept override { return {}; }
};

int main(int argc, char** argv) {
    assert(argc == 2);
    const auto path = (std::filesystem::path(argv[1]) / "forward_timing.json").string();
    std::ofstream(path) << R"({"motions":[{
      "name":"forward","max_seq_ms":290,"repeat_count":2,"frames":[
      {"name":"lift","start_ms":90,"time_ms":100,"angles":{"1":10}},
      {"name":"back","start_ms":190,"time_ms":100,"angles":{"1":0}}
    ]}]})";
    for (bool queued : {false, true}) {
        TimingHardware hardware;
        int ms = 0;
        RobotMotionPlayer player(path, hardware, [&] {
            return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(ms));
        });
        assert(player.initialize());
        assert(player.start("forward") == StartResult::Accepted);
        assert(player.motionInfo("forward").expected_duration_ms == 580);
        if (queued) assert(player.queueNext("forward") == QueueResult::Queued);
        for (ms = 5; player.running() && ms <= 1500; ms += 5) {
            player.update();
            const int cycle_time = ms % 290;
            // Saved leading hold must survive both repeat and queue boundaries.
            if (cycle_time <= 90) assert(hardware.goals.at(1) == 0);
            if (cycle_time > 90 && cycle_time < 190) assert(hardware.goals.at(1) > 0);
        }
        assert(player.succeeded());
        assert(ms - 5 == (queued ? 1160 : 580));
        assert(hardware.prepares == 1);
    }
}
