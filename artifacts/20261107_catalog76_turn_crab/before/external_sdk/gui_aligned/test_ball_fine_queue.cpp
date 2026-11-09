#include "robot_motion_player.hpp"

#include <cassert>
#include <iostream>
#include <vector>

using namespace irc_step;

struct BallQueueHardware : IMotionHardware {
    JointAngles present, goals;
    int prepares = 0;
    bool initialize() noexcept override {
        for (int id = 0; id < 23; ++id) present[id] = 0.;
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
    const auto catalog = MotionLibrary::loadGuiJson(argv[1]);
    for (const std::string entry : {"R전진45도(4회)", "t라인복귀좌공용", "t라인복귀우공용"}) {
        for (bool right_first : {false, true}) {
            const std::string first_turn = right_first ? "t제자리우회전45도(2회)" : "t제자리좌회전45도(2회)";
            const std::string second_turn = right_first ? "t제자리좌회전45도(3회)" : "t제자리우회전45도(3회)";
            const std::vector<std::string> sequence{
                entry, first_turn, "t오뒤에서 미세오뒤(45도)", "t미세45도(5회)",
                "t미세45도(5회)", "rr미세오뒤에서 오뒤45도", second_turn,
                first_turn, "t오뒤에서 미세오뒤(45도)", "t미세45도(5회)",
            };
            BallQueueHardware hardware;
            int ms = 0;
            RobotMotionPlayer player(argv[1], hardware, [&] {
                return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(ms));
            });
            player.setContinuousInterpolation(true);
            player.setForwardBoundaryFeedbackAccounting(true);
            assert(player.initialize());
            assert(player.start(sequence.front()) == StartResult::Accepted);
            std::size_t queued_after = sequence.size();
            int started_at = 0;
            std::uint64_t previous_completion = 0;
            for (ms = 1; player.running() && ms < 30000; ++ms) {
                const auto index = player.completionSequence();
                const auto info = player.motionInfo(sequence[index]);
                if (index + 1 < sequence.size() && queued_after != index
                    && ms >= started_at + info.expected_duration_ms * .8) {
                    const auto result = player.queueNext(sequence[index + 1]);
                    if (result != QueueResult::Queued)
                        std::cerr << "Rejected: " << sequence[index] << " -> " << sequence[index + 1] << '\n';
                    assert(result == QueueResult::Queued);
                    assert(player.queueNext(sequence[index + 1]) == QueueResult::RejectedQueueFull);
                    queued_after = index;
                }
                player.update();
                if (player.completionSequence() != previous_completion) {
                    previous_completion = player.completionSequence();
                    started_at = ms;
                    if (previous_completion < sequence.size()) {
                        assert(player.running());
                        assert(player.currentMotion() == sequence[previous_completion]);
                        // No posture jump or inserted idle on queue activation.
                        const auto& end = catalog.motion(sequence[previous_completion - 1]).frames().back().angles;
                        assert(hardware.goals == end);
                    }
                }
            }
            assert(player.succeeded());
            assert(player.completionSequence() == sequence.size());
            assert(hardware.prepares == 1);
        }
    }
    // Support changes must never be skipped even when a target is in the same mission.
    for (const auto& pair : std::vector<std::pair<std::string, std::string>>{
             {"t미세45도(5회)", "t제자리좌회전45도(2회)"},
             {"t제자리좌회전45도(2회)", "t미세45도(5회)"},
             {"R전진45도(4회)", "t미세45도(5회)"}}) {
        BallQueueHardware hardware;
        RobotMotionPlayer player(argv[1], hardware);
        assert(player.initialize());
        assert(player.start(pair.first) == StartResult::Accepted);
        assert(player.queueNext(pair.second) == QueueResult::IncompatibleTransition);
    }
    // Source75 repeat-count changes must remain queueable at the same gait.
    for (int camera : {0, 45, 90}) {
        for (int current = 1; current <= 5; ++current) {
            for (int next = 1; next <= 5; ++next) {
                const auto name = [camera](int count) {
                    return "t미세" + std::to_string(camera) + "도("
                        + std::to_string(count) + "회)";
                };
                BallQueueHardware hardware;
                int ms = 0;
                RobotMotionPlayer player(argv[1], hardware, [&] {
                    return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(ms));
                });
                player.setContinuousInterpolation(true);
                player.setForwardBoundaryFeedbackAccounting(true);
                assert(player.initialize());
                assert(player.start(name(current)) == StartResult::Accepted);
                assert(player.queueNext(name(next)) == QueueResult::Queued);
                for (ms = 1; player.running() && ms < 10000; ++ms) {
                    hardware.present = hardware.goals;
                    player.update();
                }
                assert(player.succeeded());
                assert(player.completionSequence() == 2);
                assert(player.lastCompletedMotion() == name(next));
            }
        }
    }
    {
        BallQueueHardware hardware;
        int ms = 0;
        RobotMotionPlayer player(argv[1], hardware, [&] {
            return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(ms));
        });
        player.setContinuousInterpolation(true);
        assert(player.initialize());
        assert(player.start("t허들찐") == StartResult::Accepted);
        for (ms = 1; player.running() && ms < 12000; ++ms) {
            hardware.present = hardware.goals;
            player.update();
        }
        assert(player.succeeded());
        assert(hardware.goals == catalog.motion("t허들찐").frames().back().angles);
    }
    // Include camera45/count6, now imported separately from source74.
    const auto check_pair = [&](const std::string& first, const std::string& second) {
        BallQueueHardware hardware;
        int ms = 0;
        RobotMotionPlayer player(argv[1], hardware, [&] {
            return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(ms));
        });
        player.setContinuousInterpolation(true);
        player.setForwardBoundaryFeedbackAccounting(true);
        assert(player.initialize());
        assert(player.start(first) == StartResult::Accepted);
        const auto result = player.queueNext(second);
        if (result != QueueResult::Queued)
            std::cerr << "Rejected: " << first << " -> " << second << '\n';
        assert(result == QueueResult::Queued);
        for (ms = 1; player.running() && ms < 12000; ++ms) {
            hardware.present = hardware.goals;
            const auto previous = player.completionSequence();
            player.update();
            if (previous == 0 && player.completionSequence() == 1)
                assert(hardware.goals == catalog.motion(first).frames().back().angles);
        }
        assert(player.succeeded());
        assert(player.completionSequence() == 2);
        assert(player.lastCompletedMotion() == second);
    };
    for (int camera : {0, 45, 90}) {
        const auto name = [camera](int count) {
            return "t제자리좌회전" + std::to_string(camera) + "도("
                + std::to_string(count) + "회)";
        };
        for (int first = 1; first <= 6; ++first) {
            for (int second = 1; second <= 6; ++second)
                check_pair(name(first), name(second));
            if (camera == 45) {
                check_pair("R전진45도(4회)", name(first));
                check_pair(name(first), "R전진45도(4회)");
                check_pair(name(first), "t오뒤에서 미세오뒤(45도)");
                check_pair("rr미세오뒤에서 오뒤45도", name(first));
            }
        }
    }
    std::cout << "Six alignment chains, 75 fine pairs, 108 left-turn pairs, "
                 "24 walking/preparation pairs, hurdle and rejected shortcuts passed\n";
}
