#include "robot_motion_player.hpp"
#include <cassert>
#include <iostream>
#include <vector>

using namespace irc_step;

struct Hardware : IMotionHardware {
    JointAngles goals;
    bool initialize() noexcept override {
        for (int id = 0; id < 23; ++id) goals[id] = 0.;
        return true;
    }
    bool ready() const noexcept override { return true; }
    bool prepareImmediatePlayback() noexcept override { return true; }
    bool commandPosition(const JointAngles& angles, const std::vector<int>& ids,
                         uint32_t, uint32_t) noexcept override {
        for (int id : ids) goals[id] = angles.at(id);
        return true;
    }
    bool readPresentPositions(JointAngles& angles) noexcept override { angles = goals; return true; }
    bool holdCurrentPosition(uint32_t) noexcept override { return true; }
    bool setTorqueEnabled(bool) noexcept override { return true; }
    std::string_view lastError() const noexcept override { return {}; }
};

int main(int argc, char** argv) {
    assert(argc == 2);
    const auto catalog = MotionLibrary::loadGuiJson(argv[1]);
    const std::vector<std::string> shots = {
        "R골넣기1번구역", "R골넣기2번구역", "R골넣기3번구역", "R골넣기4번구역",
        "r골넣기 5번구역", "r골넣기 6번구역", "r골넣기7번구역", "r골넣기8번구역",
    };
    for (const auto& shot : shots)
    for (const std::string exit : {"t골대후제자리우회전", "p골대후제자리좌회전"}) {
        Hardware hardware;
        int ms = 0;
        RobotMotionPlayer player(argv[1], hardware, [&] {
            return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(ms));
        });
        player.setContinuousInterpolation(true);
        assert(player.initialize());
        for (const auto& name : std::vector<std::string>{"p오뒤에서 미세오뒤90도", shot, exit, "p전진45(4회)"}) {
            assert(player.start(name) == StartResult::Accepted);
            const int deadline = ms + 12000;
            for (++ms; player.running() && ms < deadline; ++ms) player.update();
            assert(player.succeeded());
            assert(hardware.goals == catalog.motion(name).frames().back().angles);
            ms += 1000;
        }
        assert(player.completionSequence() == 4);
    }
    std::cout << "All 8 shots x 2 post-shot routes completed with fake hardware.\n";
}
