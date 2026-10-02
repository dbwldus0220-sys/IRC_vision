// No robot-control library is linked: all bus operations below are fake.
#include "robot_motion_player.hpp"
#include "mock_motion_hardware.hpp"
#include "gui_goal_position.hpp"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

using namespace irc_step;
using Clock = RobotMotionPlayer::Clock;

// Inject synchronous bus latency without sleeping or opening a device.
struct DelayedHardware : IMotionHardware {
    MockMotionHardware& bus;
    Clock::time_point& now;
    int write_delay_ms{0};
    DelayedHardware(MockMotionHardware& bus, Clock::time_point& now) : bus(bus), now(now) {}
    bool initialize() noexcept override { return bus.initialize(); }
    bool ready() const noexcept override { return bus.ready(); }
    bool commandPosition(const JointAngles& goals, const std::vector<int>& ids,
                         std::uint32_t duration, std::uint32_t acceleration) noexcept override {
        return bus.commandPosition(goals, ids, duration, acceleration);
    }
    bool commandDirectPosition(const JointAngles& goals, const std::vector<int>& ids) noexcept override {
        const bool success = bus.commandDirectPosition(goals, ids);
        now += std::chrono::milliseconds(write_delay_ms);
        return success;
    }
    bool restoreDirectPlaybackProfile() noexcept override { return bus.restoreDirectPlaybackProfile(); }
    bool readPresentPositions(JointAngles& pose) noexcept override { return bus.readPresentPositions(pose); }
    bool holdCurrentPosition(std::uint32_t duration) noexcept override { return bus.holdCurrentPosition(duration); }
    bool setTorqueEnabled(bool enabled) noexcept override { return bus.setTorqueEnabled(enabled); }
    std::string_view lastError() const noexcept override { return bus.lastError(); }
};

struct Rig {
    MockMotionHardware bus;
    Clock::time_point now{std::chrono::seconds(100)};
    DelayedHardware transport{bus, now};
    RobotMotionPlayer player;
    explicit Rig(const std::string& file) : player(file, transport, [this] { return now; }) {
        JointAngles initial;
        for (int id = 0; id < 23; ++id) initial[id] = 0;
        bus.setPresentPositions(initial);
        assert(player.initialize());
        player.configureDiagnostics(5, "", true, 10000);
    }
    void tick(int ms) {
        now = Clock::time_point(std::chrono::seconds(100) + std::chrono::milliseconds(ms));
        const int before = bus.commandCount();
        player.update();
        assert(bus.commandCount() - before <= 1);
    }
};

void near(double actual, double expected) { assert(std::abs(actual - expected) < 1e-8); }

void checkPcBoundaryPolicy(const std::filesystem::path& directory) {
    const auto file = directory / "pc_boundary_policy.json";
    std::ofstream(file) << R"({"motions":[
      {"name":"ordinary","max_seq_ms":999,"frames":[
        {"name":"first","start_ms":0,"time_ms":100,"angles":{"1":10}},
        {"name":"second","start_ms":100,"time_ms":100,"angles":{"1":20}}]},
      {"name":"repeat","max_seq_ms":999,"repeat_count":2,"frames":[
        {"name":"first","start_ms":0,"time_ms":100,"angles":{"1":10}},
        {"name":"second","start_ms":100,"time_ms":100,"angles":{"1":20},"playback_cycle_end":true}]},
      {"name":"internal","max_seq_ms":999,"repeat_count":2,"frames":[
        {"name":"first","start_ms":0,"time_ms":100,"angles":{"1":10},"playback_cycle_end":true},
        {"name":"second","start_ms":100,"time_ms":100,"angles":{"1":20},"playback_cycle_end":true}]},
      {"name":"fast_internal","max_seq_ms":999,"playback_speed":2,"frames":[
        {"name":"first","start_ms":0,"time_ms":100,"angles":{"1":10},"playback_cycle_end":true},
        {"name":"second","start_ms":100,"time_ms":100,"angles":{"1":20}}]},
      {"name":"partial","max_seq_ms":999,"frames":[
        {"name":"first","start_ms":0,"time_ms":100,"angles":{"1":10}},
        {"name":"second","start_ms":100,"time_ms":100,"angles":{"2":20}}]}
    ]})";

    // Owner's PC reference: restart at 200ms (208ms after an 8ms write),
    // then sample 5ms into 20 -> 10 degrees, not a new 0ms sample.
    for (int delay : {0, 8}) {
        Rig r(file.string()); r.player.start("repeat");
        r.tick(0); r.tick(100);
        r.transport.write_delay_ms = delay;
        r.tick(200); near(r.bus.commandedPositions().at(1), 20);
        assert(r.player.running() && r.player.completionSequence() == 0);
        assert(r.bus.commandCount() == 3);
        r.transport.write_delay_ms = 0;
        r.tick(205 + delay);
        near(r.bus.commandedPositions().at(1), 19.938441702975688);
        assert(guiGoalPosition(r.bus.commandedPositions().at(1)) == 2275);
        assert(r.bus.readCount() == 1);
        r.tick(300 + delay); r.tick(400 + delay);
        assert(r.player.succeeded() && r.player.completionSequence() == 1);
        assert(r.player.lastDiagnostics()->trace.at(3).repeat == 2);
        near(r.player.lastDiagnostics()->trace.at(3).timeline_ms, 5);
    }
    for (bool internal : {false, true}) for (int delay : {0, 8}) {
        Rig r(file.string()); r.player.start(internal ? "internal" : "ordinary"); r.tick(0);
        r.transport.write_delay_ms = delay;
        r.tick(125); near(r.bus.commandedPositions().at(1), 10);
        r.transport.write_delay_ms = 0;
        r.tick(130 + delay);
        const double progress = (internal ? 5.0 : 30.0 + delay) / 100.0;
        near(r.bus.commandedPositions().at(1), 10 + 10 * (0.5 - 0.5 * std::cos(M_PI * progress)));
        assert(r.player.completionSequence() == 0 && r.bus.readCount() == 1);
        if (internal) {
            // Final flag must use the whole-repeat path exactly once.
            r.tick(225 + delay); r.tick(230 + delay);
            near(r.bus.commandedPositions().at(1), 19.938441702975688);
            r.tick(325 + delay); r.tick(425 + delay);
        } else r.tick(200 + delay);
        assert(r.player.succeeded());
        assert(r.player.lastDiagnostics()->final_corrections == (internal ? 4 : 2));
    }
    {
        Rig r(file.string()); r.player.start("fast_internal"); r.tick(0); r.tick(65); r.tick(70);
        near(r.bus.commandedPositions().at(1), 10 + 10 * (0.5 - 0.5 * std::cos(M_PI * 0.1)));
        r.tick(115); assert(r.player.succeeded());
    }
    for (int failed_at : {100, 200}) {
        Rig r(file.string()); r.player.start("internal"); r.tick(0);
        if (failed_at == 200) r.tick(100);
        r.bus.setCommandSuccess(false); r.tick(failed_at);
        assert(r.player.status() == MotionStatus::Failed);
        assert(r.player.result() == MotionError::FrameSendFailed);
        assert(r.player.lastDiagnostics()->trace.back().repeat == 1);
        const auto count = r.bus.commandCount();
        r.bus.setCommandSuccess(true); r.tick(failed_at + 5);
        assert(r.bus.commandCount() == count && r.player.completionSequence() == 0);
    }
    for (bool complete_initial_pose : {false, true}) {
        Rig r(file.string());
        if (!complete_initial_pose) r.bus.setPresentPositions({{1, 0}});
        r.player.start("partial"); r.tick(0); r.tick(125);
        // PC writes ID 1's endpoint, then ID 2's boundary value. If ID 2
        // has no baseline, the GUI uses its stored target as its start.
        near(r.bus.commandedPositions().at(1), 10);
        near(r.bus.commandedPositions().at(2), complete_initial_pose ? 0 : 20);
        assert(r.bus.commandCount() == 2);
        r.tick(130);
        near(r.bus.commandedPositions().at(1), 10);
        near(r.bus.commandedPositions().at(2), complete_initial_pose
            ? 20 * (0.5 - 0.5 * std::cos(M_PI * 0.3)) : 20);
    }
    {
        Rig r(file.string()); r.bus.setAutoReachGoal(false);
        r.player.start("ordinary"); r.tick(0);
        r.bus.setReadSuccess(false); r.tick(100); r.tick(200);
        // Default completion means all planned Goals were sent, not arrival.
        assert(r.player.succeeded() && r.player.result() == MotionError::None);
        assert(r.bus.readCount() == 1);
    }
    for (bool read_success : {false, true}) {
        Rig r(file.string()); r.bus.setAutoReachGoal(false);
        assert(r.player.startPoseTransition(std::vector<double>(23, 10), 100));
        r.bus.setReadSuccess(read_success);
        r.now += std::chrono::milliseconds(100);
        const auto status = r.player.updateStartupPose();
        assert(r.bus.readCount() == 1);
        if (!read_success) {
            assert(status == MotionStatus::Failed);
            assert(r.player.result() == MotionError::PresentPositionReadFailed);
        } else {
            assert(status == MotionStatus::Settling); // Not within tolerance.
            JointAngles arrived;
            for (int id = 0; id < 23; ++id) arrived[id] = 10;
            r.bus.setPresentPositions(arrived);
            r.now += std::chrono::milliseconds(20);
            assert(r.player.updateStartupPose() == MotionStatus::Settling);
            r.now += std::chrono::milliseconds(60);
            assert(r.player.updateStartupPose() == MotionStatus::Settling);
            r.now += std::chrono::milliseconds(20);
            assert(r.player.updateStartupPose() == MotionStatus::Succeeded);
            assert(r.bus.profileRestoreCount() == 1);
        }
    }
    std::cout << "PC_BOUNDARY_POLICY_PASSED repeat/internal clocks, latency, partial joints, startup separation\n";
}

int main(int argc, char** argv) {
    assert(argc == 4);
    const std::string runtime = argv[1], pc = argv[2];
    checkPcBoundaryPolicy(argv[3]);
    const auto fixture = std::filesystem::path(argv[3]) / "common_playback_fixture.json";
    std::ofstream(fixture) << R"({"motions":[
      {"name":"A","max_seq_ms":999,"repeat_count":1,"playback_speed":1,
       "repeatable":true,"start_pose":"pose","end_pose":"pose","frames":[
        {"frame_id":"a","name":"first","start_ms":0,"time_ms":10,"angles":{"1":10}},
        {"frame_id":"b","name":"second","start_ms":10,"time_ms":10,"angles":{"1":20}}]},
      {"name":"gap","max_seq_ms":999,"frames":[
        {"name":"first","start_ms":10,"time_ms":10,"angles":{"1":10}},
        {"name":"second","start_ms":30,"time_ms":10,"angles":{"2":20}}]},
      {"name":"repeat","max_seq_ms":999,"repeat_count":4,"frames":[
        {"name":"first","start_ms":0,"time_ms":10,"angles":{"1":10}}]},
      {"name":"speed","max_seq_ms":999,"playback_speed":1.05,"frames":[
        {"name":"first","start_ms":0,"time_ms":100,"angles":{"1":10}}]}
    ]})";

    // Common interpolation, including the flag that the previous SDK ignored.
    MotionFrame lift;
    lift.name = "오들"; lift.time_ms = 100; lift.lift_early_arrival = true;
    near(RobotMotionPlayer::frameMotionProgress(lift, 80), 1);
    lift.lift_early_arrival = false;
    assert(RobotMotionPlayer::frameMotionProgress(lift, 80) < 1);
    lift.lift_early_arrival = true; lift.name = "ordinary";
    assert(RobotMotionPlayer::frameMotionProgress(lift, 80) < 1);
    lift.name = "오들"; lift.time_ms = 1;
    near(RobotMotionPlayer::frameMotionProgress(lift, 0.5), 0.5);
    near(RobotMotionPlayer::interpolateShortest(170, -170, 0.5), 180);
    assert(guiGoalPosition(0) == 2048);

    {
        Rig r(fixture.string());
        assert(r.player.start("A") == StartResult::Accepted);
        r.tick(0); r.tick(5); near(r.bus.commandedPositions().at(1), 5);
        r.tick(15); near(r.bus.commandedPositions().at(1), 10); // final only
        r.tick(16);
        near(r.bus.commandedPositions().at(1), 10 + 10 * (0.5 - 0.5 * std::cos(M_PI * 0.6)));
        r.tick(25); near(r.bus.commandedPositions().at(1), 20);
        r.tick(30); assert(r.player.succeeded()); // max_seq_ms=999 is irrelevant
        assert(r.player.lastDiagnostics()->final_corrections == 2);
        assert(r.player.lastDiagnostics()->over_7_5 == 2);
    }
    {
        Rig r(fixture.string()); r.player.start("A"); r.tick(0);
        r.tick(100); near(r.bus.commandedPositions().at(1), 10);
        r.tick(105); near(r.bus.commandedPositions().at(1), 20);
        r.tick(110); assert(r.player.succeeded()); // no catch-up burst
    }
    for (int hold : {0, 20, 50, 100}) {
        Rig r(fixture.string()); r.player.setQueuedTransitionHoldMs(hold);
        r.player.start("A"); r.player.queueNext("A"); r.tick(0); r.tick(10); r.tick(20);
        assert(r.player.completionSequence() == 0);
        const int count = r.bus.commandCount(), reads = r.bus.readCount();
        if (hold > 0) { r.tick(20 + hold - 1); assert(r.player.completionSequence() == 0); }
        r.tick(20 + std::max(hold, 1));
        assert(r.player.completionSequence() == 1);
        assert(r.bus.commandCount() == count); // activation does not transmit
        r.tick(25 + std::max(hold, 1));
        assert(r.bus.readCount() == reads); // queue never replaces final pose with feedback
        assert(r.player.cancel() == CancelResult::Cancelled);
    }
    for (int failure_time : {5, 10, 20}) {
        Rig r(fixture.string()); r.player.start("A"); r.player.queueNext("A"); r.tick(0);
        if (failure_time == 20) r.tick(10);
        r.bus.setCommandSuccess(false); r.tick(failure_time);
        assert(r.player.status() == MotionStatus::Failed);
        assert(r.player.result() == MotionError::FrameSendFailed);
        assert(!r.player.hasQueuedMotion()); assert(r.player.completionSequence() == 0);
        const auto count = r.bus.commandCount(); r.tick(100); assert(count == r.bus.commandCount());
        assert(r.player.lastDiagnostics()->goal_failures == 1);
    }
    {
        Rig r(fixture.string()); r.player.start("A"); r.bus.setReadSuccess(false); r.tick(0);
        assert(r.player.result() == MotionError::PresentPositionReadFailed);
        assert(r.bus.commandCount() == 0);
    }
    for (bool read_recovers : {false, true}) {
        Rig r(fixture.string()); r.player.start("A"); r.player.queueNext("A"); r.tick(0);
        r.bus.setCommandSuccess(false); r.tick(5);
        assert(r.player.status() == MotionStatus::Failed);
        assert(!r.player.hasQueuedMotion());
        const int reads = r.bus.readCount(), writes = r.bus.commandCount();
        r.bus.setCommandSuccess(true);
        r.bus.setReadSuccess(read_recovers);
        JointAngles actual;
        for (int id = 0; id < 23; ++id) actual[id] = 0;
        actual[1] = 3;
        r.bus.setPresentPositions(actual);
        // A new standalone start uses the ordinary PP read, with no extra
        // recovery ping, arrival gate, or restoration of the failed queue.
        assert(r.player.start("A") == StartResult::Accepted);
        r.tick(10);
        assert(r.bus.readCount() == reads + 1);
        if (read_recovers) {
            assert(r.player.running());
            near(r.bus.commandedPositions().at(1), 3);
            r.tick(15); near(r.bus.commandedPositions().at(1), 6.5);
            assert(r.bus.readCount() == reads + 1);
        } else {
            assert(r.player.status() == MotionStatus::Failed);
            assert(r.player.result() == MotionError::PresentPositionReadFailed);
            assert(r.bus.commandCount() == writes);
        }
    }
    {
        Rig r(fixture.string()); r.player.start("gap"); r.tick(0); r.tick(5);
        near(r.bus.commandedPositions().at(1), 0);
        r.tick(20); r.tick(25); near(r.bus.commandedPositions().at(1), 10);
        r.tick(35); near(r.bus.commandedPositions().at(1), 10);
        near(r.bus.commandedPositions().at(2), 10);
    }
    {
        Rig r(fixture.string()); r.player.start("repeat");
        for (int ms = 0; r.player.running() && ms < 100; ms += 5) r.tick(ms);
        assert(r.player.succeeded()); assert(r.bus.readCount() == 1);
        assert(r.player.lastDiagnostics()->final_corrections == 4);
    }
    {
        Rig r(fixture.string()); r.player.start("speed"); r.tick(0); r.tick(50);
        near(r.bus.commandedPositions().at(1), 10 * (0.5 - 0.5 * std::cos(M_PI * 0.52)));
        r.tick(96); near(r.bus.commandedPositions().at(1), 10);
        r.tick(101); assert(r.player.succeeded());
    }
    for (bool emergency : {false, true}) {
        Rig r(fixture.string()); r.player.setQueuedTransitionHoldMs(100);
        r.player.start("A"); r.player.queueNext("A"); r.tick(0); r.tick(10); r.tick(20);
        if (emergency) assert(r.player.emergencyStop());
        else assert(r.player.cancel() == CancelResult::Cancelled);
        assert(!r.player.running()); assert(!r.player.hasQueuedMotion());
    }
    {
        Rig r(fixture.string()); r.player.setPositionToleranceEnabled(true);
        r.player.start("A"); r.tick(0); r.tick(10); r.tick(20);
        r.bus.setReadSuccess(false); r.tick(25);
        assert(r.player.status() == MotionStatus::Failed);
        assert(r.player.result() == MotionError::PresentPositionReadFailed);
    }
    {
        Rig r(fixture.string()); r.player.setPositionToleranceEnabled(true);
        r.bus.setAutoReachGoal(false);
        r.player.start("A"); r.tick(0); r.tick(10); r.tick(20); r.tick(25); r.tick(125);
        // Existing arrival timeout remains best effort: no new arrival gate.
        assert(r.player.succeeded());
        assert(r.player.result() == MotionError::PositionTimeout);
    }
    {
        const auto trace = std::filesystem::path(argv[3]) / "trace.csv";
        {
            Rig r(fixture.string());
            r.player.configureDiagnostics(5, trace.string(), true, 2);
            r.player.start("A");
            for (int ms : {0, 5, 10, 15, 20, 25}) r.tick(ms);
            assert(r.player.lastDiagnostics()->trace.size() == 2);
            assert(r.player.lastDiagnostics()->dropped_trace == 3);
        } // Joins the asynchronous writer, making this assertion deterministic.
        std::ifstream in(trace);
        std::string line; int ticks = 0, summaries = 0;
        while (std::getline(in, line)) {
            ticks += line.starts_with("tick,"); summaries += line.starts_with("summary,");
        }
        assert(ticks == 2 && summaries == 1);
    }

    const auto reference = MotionLibrary::loadGuiJson(pc);
    const auto production = MotionLibrary::loadGuiJson(runtime);
    assert(reference.names().size() == 82);
    std::size_t frame_count = 0;
    for (const auto& name : reference.names()) frame_count += reference.motion(name).frames().size();
    assert(frame_count == 286);
    const auto& forward = production.motion("찐찐전진45(4회)");
    assert(forward.durationMs() == 555 && forward.repeatCount() == 4);
    near(forward.playbackSpeed(), 1.05);
    const int starts[]{80, 148, 360, 430}, times[]{68, 132, 70, 125};
    for (int i = 0; i < 4; ++i) {
        assert(forward.frames()[i].start_ms == starts[i]); assert(forward.frames()[i].time_ms == times[i]);
    }

    // Exercise every production motion, including both turns, crab, pickup,
    // hurdle and recovery poses. No physical startup or serial bus is involved.
    for (const auto& name : production.names()) {
        Rig r(runtime); assert(r.player.start(name) == StartResult::Accepted);
        const auto& pattern = production.motion(name);
        const auto limit = int(pattern.durationMs() / pattern.playbackSpeed() * pattern.repeatCount())
            + 20 * int(pattern.frames().size()) * pattern.repeatCount() + 1000;
        for (int ms = 0; r.player.running() && ms < limit; ms += 5) r.tick(ms);
        assert(r.player.succeeded());
        assert(r.player.lastDiagnostics()->final_corrections == pattern.frames().size() * pattern.repeatCount());
        assert(r.bus.readCount() == 1);
    }
    for (bool head : {false, true}) for (bool shoulder : {false, true}) {
        Rig r(runtime); r.player.configureJointPolicy(pc, head, shoulder);
        r.player.setJointOverride(0, -64);
        assert(r.player.start("찐찐전진45(4회)") == StartResult::Accepted);
        r.tick(0); r.tick(142);
        const auto& original = reference.motion("찐찐전진45(4회)").frames().front();
        const auto& current = forward.frames().front();
        for (const auto& [id, value] : r.bus.commandedPositions()) {
            const double expected = id == 0 ? (head ? -64 : original.angles.at(0))
                : id == 4 ? (shoulder ? 18 : original.angles.at(4))
                : id == 5 ? (shoulder ? -18 : original.angles.at(5)) : current.angles.at(id);
            near(value, expected);
        }
    }
    {
        Rig r(runtime); r.player.configureJointPolicy(pc, false, false);
        assert(r.player.start("찐찐라인복귀좌회전45도(6회)") == StartResult::InvalidMotion);
        assert(r.bus.commandCount() == 0); // missing original is never guessed
    }
    std::cout << "COMMON_PLAYBACK_TESTS_PASSED all production motions + deterministic edge cases\n";
}
