#include "robot_motion_player.hpp"
#include "mock_motion_hardware.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace irc_step;
struct Rig {
    MockMotionHardware bus;
    int ms = 1000;
    RobotMotionPlayer player;
    Rig(const std::string& path) : player(path, bus, [this] {
        return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(ms));
    }) {
        JointAngles initial;
        for (int id = 0; id < 23; ++id) initial[id] = 0;
        bus.setPresentPositions(initial);
        assert(player.initialize());
    }
    void finish(int limit) {
        const int deadline = ms + limit;
        while (player.running() && ms < deadline) { ms += 5; player.update(); }
        if (!player.succeeded()) std::cerr << player.currentMotion() << ": " << player.lastError() << '\n';
        assert(player.succeeded());
    }
};
void near(double actual, double expected) { assert(std::abs(actual - expected) < 1e-8); }
int main(int argc, char** argv) {
    assert(argc == 3);
    const std::string runtime = argv[1], pc = argv[2];
    const auto production = MotionLibrary::loadGuiJson(runtime);
    const auto reference = MotionLibrary::loadGuiJson(pc);
    int runs = 0;
    for (const auto& name : production.names()) {
        const bool forward = name.starts_with("전45도(") || name.starts_with("전90도(");
        for (int mode = 0; mode < (forward ? 4 : 1); ++mode) {
            const bool head = !(mode & 1), shoulder = !(mode & 2);
            Rig r(runtime);
            r.player.configureJointPolicy(pc, head, shoulder);
            if (forward) r.player.setJointOverride(0, -64);
            assert(r.player.start(name) == StartResult::Accepted);
            const auto& pattern = production.motion(name);
            const int limit = int(pattern.durationMs() / pattern.playbackSpeed() * pattern.repeatCount())
                + 20 * int(pattern.frames().size()) * pattern.repeatCount() + 1000;
            r.finish(limit);
            assert(r.player.playbackRepeat() == pattern.repeatCount());
            if (forward) {
                const auto& expected = reference.motion(name).frames().back().angles;
                near(r.bus.commandedPositions().at(0), head ? -64 : expected.at(0));
                near(r.bus.commandedPositions().at(4), shoulder ? 18 : expected.at(4));
                near(r.bus.commandedPositions().at(5), shoulder ? -18 : expected.at(5));
                assert(pattern.durationMs() == 506);
            }
            ++runs;
        }
    }
    for (const auto& name : {"전45도(2회)", "전90도(2회)"}) {
        Rig r(runtime);
        assert(r.player.start(name) == StartResult::Accepted);
        assert(r.player.queueNext(name) == QueueResult::Queued);
        r.finish(10000);
        assert(r.player.completionSequence() == 2);
    }
    std::cout << "PASS: " << runs << " mock playbacks (82 motions; 8 forward variants x 4 policy modes), 2 queued transitions\n";
}
