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
    for (const std::string entry : {"j전진45(4회)", "m라인복귀좌회전공용", "m라인복귀우회전공용"}) {
        for (bool right_first : {false, true}) {
            const std::string first_turn = right_first ? "t제자리우회전45도(2회)" : "p제자리좌회전45(2회)";
            const std::string second_turn = right_first ? "p제자리좌회전45(3회)" : "t제자리우회전45도(3회)";
            const std::vector<std::string> sequence{
                entry, first_turn, "p오뒤에서 미세오뒤(45도)", "p미세45도(5회)",
                "p미세45도(5회)", "p미세오뒤에서 오뒤45도", second_turn,
                first_turn, "p오뒤에서 미세오뒤(45도)", "p미세45도(5회)",
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
             {"p미세45도(5회)", "p제자리좌회전45(2회)"},
             {"p제자리좌회전45(2회)", "p미세45도(5회)"},
             {"j전진45(4회)", "p미세45도(5회)"}}) {
        BallQueueHardware hardware;
        RobotMotionPlayer player(argv[1], hardware);
        assert(player.initialize());
        assert(player.start(pair.first) == StartResult::Accepted);
        assert(player.queueNext(pair.second) == QueueResult::IncompatibleTransition);
    }
    // Source77 repeat-count changes must remain queueable at the same gait.
    for (int camera : {0, 45, 90}) {
        for (int current = 1; current <= 5; ++current) {
            for (int next = 1; next <= 5; ++next) {
                const auto name = [camera](int count) {
                    return "p미세" + std::to_string(camera) + "도("
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
    // Include all cameras/counts and the source88 forward/recovery speed boundary.
    const auto check_pair = [&](const std::string& first, const std::string& second,
                                bool fixed_head = false) {
        BallQueueHardware hardware;
        int ms = 0;
        RobotMotionPlayer player(argv[1], hardware, [&] {
            return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(ms));
        });
        player.setContinuousInterpolation(true);
        player.setForwardBoundaryFeedbackAccounting(true);
        assert(player.initialize());
        if (fixed_head) player.setJointOverride(0, 1.);
        assert(player.start(first) == StartResult::Accepted);
        const auto result = player.queueNext(second);
        if (result != QueueResult::Queued)
            std::cerr << "Rejected: " << first << " -> " << second << '\n';
        assert(result == QueueResult::Queued);
        for (ms = 1; player.running() && ms < 12000; ++ms) {
            hardware.present = hardware.goals;
            const auto previous = player.completionSequence();
            player.update();
            if (previous == 0 && player.completionSequence() == 1) {
                auto expected = catalog.motion(first).frames().back().angles;
                if (fixed_head) expected[0] = 1.;
                assert(hardware.goals == expected);
            }
        }
        assert(player.succeeded());
        assert(player.completionSequence() == 2);
        assert(player.lastCompletedMotion() == second);
    };
    // Near Ball approach now shares the regular two-cycle forward support.
    for (const std::string turn : {"p제자리좌회전45(2회)", "t제자리우회전45도(3회)"}) {
        check_pair("j전진45(2회)", turn);
        check_pair(turn, "j전진45(2회)");
    }
    check_pair("p미세오뒤에서 오뒤45도", "j전진45(2회)");
    for (int camera : {0, 45, 90}) {
        const auto name = [camera](int count) {
            return "p제자리좌회전" + std::to_string(camera) + (camera == 45 ? "(" : "도(")
                + std::to_string(count) + "회)";
        };
        for (int first = 1; first <= 6; ++first) {
            for (int second = 1; second <= 6; ++second)
                check_pair(name(first), name(second));
            if (camera == 45) {
                check_pair("j전진45(4회)", name(first));
                check_pair(name(first), "j전진45(4회)");
                check_pair(name(first), "p오뒤에서 미세오뒤(45도)");
                check_pair("p미세오뒤에서 오뒤45도", name(first));
            }
        }
    }
    // Source98 names must retain forward/recovery reservations at every boundary.
    const std::vector<std::vector<std::string>> walking_groups{
        {"j전진45(2회)", "j전진45(4회)", "j전진45(6회)", "j전진45(8회)",
         "m라인복귀좌회전45도", "m라인복귀좌회전공용", "m라인복귀우회전45도", "m라인복귀우회전공용"},
        {"j전진90도(2회)", "j전진90도(4회)", "j전진90도(6회)",
         "m라인복귀좌회전90도", "m라인복귀우회전90"},
        {"j전진0도(4회)", "j전진0도(6회)"},
    };
    for (const auto& group : walking_groups)
        for (const auto& first : group)
            for (const auto& second : group) check_pair(first, second);
    // Goal navigation uses camera-45 recovery data with an executor head override.
    for (const std::string forward : {"j전진90도(2회)", "j전진90도(4회)", "j전진90도(6회)"})
        for (const std::string recovery : {"m라인복귀좌회전45도", "m라인복귀우회전45도"}) {
            check_pair(forward, recovery, true);
            check_pair(recovery, forward, true);
            BallQueueHardware hardware;
            RobotMotionPlayer player(argv[1], hardware);
            assert(player.initialize());
            assert(player.start(forward) == StartResult::Accepted);
            assert(player.queueNext(recovery) == QueueResult::IncompatibleTransition);
        }
    // Pickup reservations retain the same gait; count changes share its frames.
    const std::vector<std::vector<std::string>> crab_groups{
        {"tt미세왼옆0도", "tt미세왼옆0도(2회)"},
        {"t미세오옆꽃게(0도)-1회", "t미세오옆꽃게(0도)-2회"},
        {"tt미세왼옆90도", "tt미세왼옆90도(2회)"},
        {"t미세오옆꽃게90도(1회)", "t미세오옆꽃게90도(2회)"},
    };
    for (const auto& group : crab_groups)
        for (const auto& first : group)
            for (const auto& second : group) check_pair(first, second);

    // The bridge dispatches posture changes and direction changes after completion,
    // not through queueNext. Exercise these real SDK start/completion boundaries.
    for (const auto& sequence : std::vector<std::vector<std::string>>{
        {"찐공잡기전후진-2(2회)", "r공잡기 400", "n공확인자세",
         "t공후 제자리우회전", "j전진45(4회)"},
        {"찐공잡기전후진-2(2회)", "r공잡기500", "n공확인자세",
         "p공후 제자리좌회전", "j전진45(4회)"},
        {"p미세0도(10회)", "t허들찐", "j전진45(6회)"},
        {"p미세0도(5회)", "p미세오뒤에서 오뒤0도", "m라인복귀좌회전45도",
         "p오뒤에서 미세오뒤(0도)", "p미세0도(5회)", "p미세0도(10회)",
         "t허들찐", "j전진45(6회)"},
        {"p미세0도(5회)", "p미세오뒤에서 오뒤45도", "m라인복귀우회전45도",
         "p오뒤에서 미세오뒤(45도)", "p미세0도(5회)", "p미세0도(10회)",
         "t허들찐", "j전진45(6회)"},
        {"n공확인자세", "p공후 제자리좌회전", "j전진45(4회)"},
        {"R골넣기1번구역", "p골대후제자리좌회전", "j전진45(4회)"},
        {"R골넣기2번구역", "p골대후제자리좌회전", "j전진45(4회)"},
        {"R골넣기3번구역", "p골대후제자리좌회전", "j전진45(4회)"},
        {"R골넣기4번구역", "p골대후제자리좌회전", "j전진45(4회)"},
        {"p오뒤에서 미세오뒤(0도)", "p미세0도(5회)",
         "p미세오뒤에서 오뒤0도", "p제자리좌회전0도(1회)"},
        {"p오뒤에서 미세오뒤(45도)", "p미세45도(5회)",
         "p미세오뒤에서 오뒤45도", "p제자리좌회전45(1회)"},
        {"p오뒤에서 미세오뒤90도", "p미세90도(5회)",
         "p미세오뒤에서 오뒤90도", "p제자리좌회전90도(1회)"},
        {"r오뒤에서 오옆꽃게0도", "tt미세왼옆0도", "t미세오옆꽃게(0도)-1회",
         "tt미세왼옆0도(2회)", "r왼옆에서 미세오뒤(0도)"},
        {"r미세오뒤에서 오옆0도", "tt미세왼옆0도(2회)", "r왼옆에서 오뒤0도"},
        {"r오뒤에서 오옆꽃게90도", "tt미세왼옆90도", "t미세오옆꽃게90도(1회)",
         "tt미세왼옆90도(2회)", "r왼옆에서 미세오뒤(90도)"},
        {"r미세오뒤에서 오옆 90도", "tt미세왼옆90도(2회)", "r왼옆에서 오뒤90도"},
    }) {
        BallQueueHardware hardware;
        int ms = 0;
        RobotMotionPlayer player(argv[1], hardware, [&] {
            return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(ms));
        });
        player.setContinuousInterpolation(true);
        assert(player.initialize());
        for (const auto& name : sequence) {
            assert(player.start(name) == StartResult::Accepted);
            const int deadline = ms + 10000;
            for (++ms; player.running() && ms < deadline; ++ms) {
                hardware.present = hardware.goals;
                player.update();
            }
            assert(player.succeeded());
            assert(hardware.goals == catalog.motion(name).frames().back().angles);
        }
        assert(player.completionSequence() == sequence.size());
    }
    std::cout << "Six alignment chains, 75 fine pairs, 108 left-turn pairs, "
                 "five near-Ball two-cycle transition pairs, "
                 "24 walking/preparation pairs, 93 forward/recovery pairs, "
                 "16 crab queue pairs, four crab chains, three camera transition chains, "
                 "five source79 exit chains, two source80 pickup chains, hurdle exit, "
                 "two hurdle fine/recovery chains, "
                 "12 goal head-override pairs and rejected shortcuts passed\n";
}
