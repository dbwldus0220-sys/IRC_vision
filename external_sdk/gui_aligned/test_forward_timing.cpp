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

struct BoundaryReadHardware : TimingHardware {
    int* clock = nullptr;
    int boundary_delay_ms = 0;
    bool readPresentPositions(JointAngles& values) noexcept override {
        if (*clock == 290) *clock += boundary_delay_ms;
        values = present;
        return true;
    }
};

int main(int argc, char** argv) {
    assert(argc == 2 || argc == 3);
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

    // A renamed repeat-count variant must behave exactly like internal repeats.
    std::ofstream(path) << R"({"motions":[
      {"name":"forward_2","max_seq_ms":380,"repeat_count":2,"start_pose":"lift","end_pose":"back","frames":[
        {"name":"lift","start_ms":90,"time_ms":100,"angles":{"1":10}},
        {"name":"back","start_ms":280,"time_ms":100,"stop_at_end":true,"angles":{"1":0}}]},
      {"name":"forward_4","max_seq_ms":380,"repeat_count":4,"start_pose":"lift","end_pose":"back","frames":[
        {"name":"lift","start_ms":90,"time_ms":100,"angles":{"1":10}},
        {"name":"back","start_ms":280,"time_ms":100,"stop_at_end":true,"angles":{"1":0}}]},
      {"name":"other_camera","max_seq_ms":190,"start_pose":"other_lift","end_pose":"other_back","frames":[
        {"name":"lift","start_ms":90,"time_ms":100,"angles":{"1":20}}]},
      {"name":"changed_timing","max_seq_ms":380,"start_pose":"lift","end_pose":"back","frames":[
        {"name":"lift","start_ms":0,"time_ms":100,"angles":{"1":10}},
        {"name":"back","start_ms":280,"time_ms":100,"stop_at_end":true,"angles":{"1":0}}]}
    ]})";
    for (bool continuous : {false, true}) {
        TimingHardware hardware;
        int ms = 0;
        RobotMotionPlayer player(path, hardware, [&] {
            return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(ms));
        });
        player.setContinuousInterpolation(continuous);
        assert(player.initialize());
        assert(player.start("forward_2") == StartResult::Accepted);
        assert(player.queueNext("other_camera") == QueueResult::IncompatibleTransition);
        assert(player.queueNext("changed_timing") == QueueResult::IncompatibleTransition);
        assert(player.queueNext("forward_4") == QueueResult::Queued);
        for (ms = 5; player.running() && ms <= 2500; ms += 5) {
            player.update();
            const int cycle_time = ms % 380;
            // Preserve leading and internal holds, and the explicit stop at every end.
            if (cycle_time <= 90) assert(hardware.goals.at(1) == 0);
            if (cycle_time >= 190 && cycle_time <= 280) assert(hardware.goals.at(1) == 10);
        }
        assert(player.succeeded());
        assert(ms - 5 == 380 * 6);
        assert(player.completionSequence() == 2);
        assert(hardware.prepares == 1);
    }

    // Feedback must consume physical hold time, not add another hold before it.
    for (bool queued : {false, true}) for (bool enabled : {false, true})
    for (bool continuous : {false, true}) for (int delay : {0, 16, 140})
    for (const std::string name : {"turn", "R전진45도(6회)", "R전진라인복귀좌", "R라인복귀우회전"})
    for (int explicit_hold : {0, 25}) {
        if (!queued && explicit_hold != 0) continue;
        std::ofstream(path) << "{\"motions\":[{\"name\":\"" << name
            << "\",\"max_seq_ms\":290,\"repeat_count\":"
            << (queued ? 1 : 2) << R"(,"frames":[
              {"name":"lift","start_ms":90,"time_ms":100,"angles":{"1":10}},
              {"name":"back","start_ms":190,"time_ms":100,"stop_at_end":true,"angles":{"1":0}}
            ]}]})";
        int ms = 0;
        BoundaryReadHardware hardware;
        hardware.clock = &ms;
        hardware.boundary_delay_ms = delay;
        RobotMotionPlayer player(path, hardware, [&] {
            return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(ms));
        });
        player.setContinuousInterpolation(continuous);
        player.setForwardBoundaryFeedbackAccounting(enabled);
        player.setQueuedTransitionHoldMs(explicit_hold);
        assert(player.initialize());
        assert(player.start(name) == StartResult::Accepted);
        if (queued) assert(player.queueNext(name) == QueueResult::Queued);
        for (ms = 5; ms < 290; ms += 5) player.update();
        player.update();
        assert(ms == 290 + delay);
        assert(hardware.goals.at(1) == 0);
        if (explicit_hold > 0) {
            ms += explicit_hold;
            player.update();
        }
        const bool accounted = enabled && name != "turn" && explicit_hold == 0;
        const int hold_end = accounted ? std::max(380, ms) : ms + 90;
        ms = hold_end;
        player.update();
        assert(hardware.goals.at(1) == 0);  // Full saved hold; no skipped movement.
        ms += 5;
        player.update();
        assert(hardware.goals.at(1) > 0);
        assert(hardware.goals.at(1) < 1);  // A long read cannot jump into the lift.
        assert(hardware.prepares == 1);
    }

    // Approved line gaits share the entry frame, but their final joint targets differ.
    std::ofstream(path) << R"JSON({"motions":[
      {"name":"R전진45도(6회)","max_seq_ms":380,"repeat_count":2,"start_pose":"lift","end_pose":"back","frames":[
        {"name":"lift","start_ms":90,"time_ms":100,"angles":{"1":10}},
        {"name":"back","start_ms":280,"time_ms":100,"stop_at_end":true,"angles":{"1":0}}]},
      {"name":"R전진라인복귀좌","max_seq_ms":380,"repeat_count":4,"start_pose":"lift","end_pose":"left_back","frames":[
        {"name":"lift","start_ms":90,"time_ms":100,"angles":{"1":10}},
        {"name":"left_back","start_ms":280,"time_ms":100,"stop_at_end":true,"angles":{"1":2}}]},
      {"name":"R라인복귀우회전","max_seq_ms":380,"repeat_count":4,"start_pose":"lift","end_pose":"right_back","frames":[
        {"name":"lift","start_ms":90,"time_ms":100,"angles":{"1":10}},
        {"name":"right_back","start_ms":280,"time_ms":100,"stop_at_end":true,"angles":{"1":-2}}]},
      {"name":"R전진45도(2회)","max_seq_ms":380,"start_pose":"lift","end_pose":"changed_back","frames":[
        {"name":"lift","start_ms":90,"time_ms":100,"angles":{"1":20}},
        {"name":"changed_back","start_ms":280,"time_ms":100,"angles":{"1":0}}]},
      {"name":"R전진90도(6회)","max_seq_ms":380,"start_pose":"lift","end_pose":"camera_back","frames":[
        {"name":"lift","start_ms":90,"time_ms":100,"angles":{"1":10}},
        {"name":"camera_back","start_ms":280,"time_ms":100,"angles":{"1":0}}]}
    ]})JSON";
    const std::vector<std::string> names = {"R전진45도(6회)", "R전진라인복귀좌", "R라인복귀우회전"};
    for (bool continuous : {false, true}) for (const auto& first : names)
    for (const auto& second : names) {
        TimingHardware hardware;
        int ms = 0;
        RobotMotionPlayer player(path, hardware, [&] {
            return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(ms));
        });
        player.setContinuousInterpolation(continuous);
        player.setForwardBoundaryFeedbackAccounting(true);
        assert(player.initialize());
        assert(player.start(first) == StartResult::Accepted);
        assert(player.queueNext("R전진45도(2회)") == QueueResult::IncompatibleTransition);
        assert(player.queueNext("R전진90도(6회)") == QueueResult::IncompatibleTransition);
        assert(player.queueNext(second) == QueueResult::Queued);
        const int boundary = first == names[0] ? 760 : 1520;
        const int total = boundary + (second == names[0] ? 760 : 1520);
        const double endpoint = first == names[0] ? 0. : first == names[1] ? 2. : -2.;
        for (ms = 5; player.running() && ms <= total + 100; ms += 5) {
            player.update();
            if (ms >= boundary && ms <= boundary + 90)
                assert(hardware.goals.at(1) == endpoint);
            if (ms == boundary + 95) {
                // Start from the commanded endpoint, not the stale measured zero.
                assert(hardware.goals.at(1) > endpoint);
                assert(hardware.goals.at(1) < endpoint + 1.);
            }
            const int cycle_time = ms % 380;
            if (cycle_time >= 190 && cycle_time <= 280)
                assert(hardware.goals.at(1) == 10);  // Explicit internal hold remains.
        }
        assert(player.succeeded());
        assert(ms - 5 == total);
        assert(player.completionSequence() == 2);
        assert(hardware.prepares == 1);
    }

    // Camera ownership is required for Goal/Line recovery cross-camera queues.
    // Reject body/timing/torque mismatches even while the head is overridden.
    std::ofstream(path) << R"JSON({"motions":[
      {"name":"R전진90도(4회)","max_seq_ms":380,"repeat_count":2,"start_pose":"head_up_lift","end_pose":"head_up_back","frames":[
        {"name":"head_up_lift","start_ms":90,"time_ms":100,"angles":{"0":1,"1":10},"torques":{"0":true,"1":true}},
        {"name":"head_up_back","start_ms":280,"time_ms":100,"angles":{"0":1,"1":0}}]},
      {"name":"R전진라인복귀좌","max_seq_ms":380,"repeat_count":2,"start_pose":"lift","end_pose":"left_back","frames":[
        {"name":"lift","start_ms":90,"time_ms":100,"angles":{"0":-32,"1":10},"torques":{"0":true,"1":true}},
        {"name":"left_back","start_ms":280,"time_ms":100,"angles":{"0":-32,"1":2}}]},
      {"name":"R라인복귀우회전","max_seq_ms":380,"repeat_count":2,"start_pose":"lift","end_pose":"right_back","frames":[
        {"name":"lift","start_ms":90,"time_ms":100,"angles":{"0":-32,"1":20},"torques":{"0":true,"1":true}},
        {"name":"right_back","start_ms":280,"time_ms":100,"angles":{"0":-32,"1":-2}}]}
    ]})JSON";
    for (bool reverse : {false, true}) {
        TimingHardware hardware;
        int ms = 0;
        RobotMotionPlayer player(path, hardware, [&] {
            return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(ms));
        });
        assert(player.initialize());
        const auto first = reverse ? "R전진라인복귀좌" : "R전진90도(4회)";
        const auto second = reverse ? "R전진90도(4회)" : "R전진라인복귀좌";
        assert(player.start(first) == StartResult::Accepted);
        assert(player.queueNext(second) == QueueResult::IncompatibleTransition);
        player.setJointOverride(0, 1.);
        assert(player.queueNext("R라인복귀우회전") == QueueResult::IncompatibleTransition);
        assert(player.queueNext(second) == QueueResult::Queued);
        for (ms = 5; player.running() && ms < 2000; ms += 5) player.update();
        assert(player.succeeded());
        assert(player.completionSequence() == 2);
        assert(hardware.prepares == 1);
    }
    // Optional production-catalog replay: no motor device is opened by TimingHardware.
    if (argc == 3) {
        const std::vector<std::string> goal_walks = {
            "R전진90도(2회)", "R전진90도(4회)", "R전진90도(6회)",
            "R전진라인복귀좌", "R라인복귀우회전",
        };
        for (const auto& first : goal_walks) for (const auto& second : goal_walks) {
            TimingHardware hardware;
            int ms = 0;
            RobotMotionPlayer player(argv[2], hardware, [&] {
                return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(ms));
            });
            assert(player.initialize());
            player.setJointOverride(0, 1.);
            assert(player.start(first) == StartResult::Accepted);
            assert(player.queueNext(second) == QueueResult::Queued);
            for (ms = 5; player.running() && ms < 20000; ms += 5) player.update();
            assert(player.succeeded());
            assert(player.completionSequence() == 2);
            assert(hardware.prepares == 1);
        }
    }
}
