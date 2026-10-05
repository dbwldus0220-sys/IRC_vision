#include "robot_motion_player.hpp"
#include "gui_playback.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <utility>

namespace irc_step {
namespace {
constexpr std::uint32_t kCancelHoldDurationMs = 0;
constexpr auto kPositionCheckInterval = std::chrono::milliseconds(20);
double guiSeconds(RobotMotionPlayer::Clock::time_point value) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(value.time_since_epoch()).count() / 1e9;
}
}

RobotMotionPlayer::RobotMotionPlayer(
    const std::string& json_path, IMotionHardware& hardware, Now now)
    : now_(std::move(now)), library_(MotionLibrary::loadGuiJson(json_path)),
      hardware_(&hardware) {
    if (!now_) throw std::invalid_argument("clock callback is required");
}

RobotMotionPlayer::~RobotMotionPlayer() { shutdown(); }

bool RobotMotionPlayer::initialize() noexcept {
    if (hardware_ == nullptr) {
        fail(MotionError::InternalError, "motion hardware is not configured");
        return false;
    }
    if (!hardware_->initialize()) {
        fail(MotionError::HardwareNotReady, std::string(hardware_->lastError()));
        return false;
    }
    initialized_ = true;
    status_ = MotionStatus::Idle;
    error_ = MotionError::None;
    last_error_.clear();
    return true;
}

bool RobotMotionPlayer::hardwareReady() const noexcept {
    return initialized_ && hardware_ != nullptr && hardware_->ready();
}

std::vector<std::string> RobotMotionPlayer::motionNames() const {
    return library_.names();
}

bool RobotMotionPlayer::contains(std::string_view name) const {
    return library_.contains(std::string(name));
}

MotionInfo RobotMotionPlayer::motionInfo(std::string_view name) const {
    const std::string owned_name(name);
    const auto& pattern = library_.motion(owned_name);
    const auto nominal_once_ms = static_cast<std::int64_t>(std::ceil(
        pattern.durationMs() / pattern.playbackSpeed()));
    return MotionInfo{
        owned_name,
        pattern.durationMs(),
        nominal_once_ms * pattern.repeatCount(),
        pattern.startPose(),
        pattern.endPose(),
        pattern.repeatable(),
        pattern.repeatCount(),
        pattern.playbackSpeed(),
    };
}

StartResult RobotMotionPlayer::start(std::string_view motion_name) noexcept {
    if (running()) return StartResult::RejectedBusy;
    if (!hardwareReady()) {
        fail(MotionError::HardwareNotReady, "motion hardware is not ready");
        return StartResult::HardwareNotReady;
    }
    const std::string name(motion_name);
    if (!library_.contains(name)) {
        error_ = MotionError::None;
        last_error_ = "motion not found: " + name;
        return StartResult::MotionNotFound;
    }
    try {
        const auto& pattern = library_.motion(name);
        if (pattern.frames().empty()) {
            last_error_ = "motion has no frames: " + name;
            return StartResult::InvalidMotion;
        }
        current_motion_ = name;
        ++trace_run_;
        trace_tick_ = 0;
        trace_evaluate_at_ = Clock::time_point{};
        trace_timeline_ms_ = 0;
        trace_frame_ = -1;
        repeat_ = 1;
        if (!hardware_->prepareImmediatePlayback()) {
            fail(MotionError::CommunicationError, "cannot set playback profiles to zero");
            return StartResult::HardwareNotReady;
        }
        if (!readStartAngles()) return StartResult::HardwareNotReady;
        startup_active_ = false;
        clearQueuedMotion();
        resetMotionRuntime(pattern, name, now_());
        error_ = MotionError::None;
        last_error_.clear();
        return StartResult::Accepted;
    } catch (const std::exception& error) {
        fail(MotionError::InternalError, error.what());
        return StartResult::InvalidMotion;
    } catch (...) {
        fail(MotionError::InternalError, "unknown motion start error");
        return StartResult::InvalidMotion;
    }
}

QueueResult RobotMotionPlayer::queueNext(std::string_view motion_name) noexcept {
    if (!hardwareReady()) return QueueResult::HardwareNotReady;
    if (!running() || pattern_ == nullptr) return QueueResult::RejectedNotRunning;
    if (queued_pattern_ != nullptr) return QueueResult::RejectedQueueFull;

    const std::string name(motion_name);
    if (!library_.contains(name)) return QueueResult::MotionNotFound;
    try {
        const auto& next = library_.motion(name);
        if (next.frames().empty()) return QueueResult::InvalidMotion;
        if (!canSeamlesslyTransition(*pattern_, current_motion_, next, name))
            return QueueResult::IncompatibleTransition;
        queued_motion_ = name;
        queued_pattern_ = &next;
        traceEvent("queue_accept", now_(), true);
        return QueueResult::Queued;
    } catch (const std::exception& error) {
        last_error_ = error.what();
        return QueueResult::InvalidMotion;
    } catch (...) {
        last_error_ = "unknown queued motion error";
        return QueueResult::InvalidMotion;
    }
}

bool RobotMotionPlayer::hasQueuedMotion() const noexcept {
    return queued_pattern_ != nullptr;
}

std::string_view RobotMotionPlayer::queuedMotion() const noexcept {
    return queued_motion_;
}

void RobotMotionPlayer::clearQueuedMotion() noexcept {
    queued_pattern_ = nullptr;
    queued_motion_.clear();
}

std::uint64_t RobotMotionPlayer::completionSequence() const noexcept {
    return completion_sequence_;
}

std::string_view RobotMotionPlayer::lastCompletedMotion() const noexcept {
    return last_completed_motion_;
}

bool RobotMotionPlayer::canSeamlesslyTransition(
    const MotionPattern& current, std::string_view current_name,
    const MotionPattern& next, std::string_view next_name) const noexcept {
    // 같은 repeatable 모션은 기존 내부 repeat_count 순환과 같은 경계입니다.
    if (current_name == next_name && current.repeatable()) return true;
    return !current.endPose().empty()
        && !next.startPose().empty()
        && current.endPose() == next.startPose();
}

void RobotMotionPlayer::resetMotionRuntime(
    const MotionPattern& pattern, std::string name, Clock::time_point now) {
    pattern_ = &pattern;
    current_motion_ = std::move(name);
    playback_frames_ = pattern.frames();
    for (auto& frame : playback_frames_) frame.angles = correctedFrameAngles(frame);
    command_ids_.reserve(kMotionMotorCount);
    command_angles_ = start_angles_;
    queued_boundary_pending_ = false;
    trace_timeline_ms_ = 0;
    trace_frame_ = -1;
    cycle_start_ms_ = 0;
    feedback_sampled_ = false;
    next_frame_ = 0;
    repeat_ = 1;
    final_goal_deg_.clear();
    final_goal_ids_.clear();
    gated_goal_deg_.clear();
    gated_goal_ids_.clear();
    gated_frame_ = nullptr;
    frame_gate_wait_started_at_ = Clock::time_point{};
    frame_gate_last_check_at_ = Clock::time_point{};
    settling_started_at_ = Clock::time_point{};
    within_tolerance_since_ = Clock::time_point{};
    last_position_check_at_ = Clock::time_point{};
    within_tolerance_ = false;
    started_at_ = now;
    gui_started_sec_ = guiSeconds(now);
    status_ = MotionStatus::Running;
    // 유효 프레임은 정책/보정을 적용한 값이다. 기록은 재생 알고리즘을 바꾸지 않는다.
    if (trace_) {
        traceEvent("run_start", now, true, start_angles_);
        for (std::size_t i = 0; i < playback_frames_.size(); ++i) {
            trace_frame_ = static_cast<int>(i);
            traceEvent("definition", now, true, playback_frames_[i].angles);
        }
        trace_frame_ = -1;
    }
}

bool RobotMotionPlayer::activateQueuedMotion(Clock::time_point now) {
    if (queued_pattern_ == nullptr) return false;
    const MotionPattern* next = queued_pattern_;
    std::string next_name = std::move(queued_motion_);
    queued_pattern_ = nullptr;
    queued_motion_.clear();
    last_completed_motion_ = current_motion_;
    ++completion_sequence_;
    // SDK queue는 조합 연결: 직전 명령 자세를 이어받는다. 새 start()만 PP를 읽는다.
    // 조합 타임라인과 동일하게 완료된 프레임의 논리 목표각을 누적한다.
    // wrapped raw/중간각을 새 원본으로 삼으면 ±360도 표현이 달라질 수 있다.
    for (const auto& frame : playback_frames_)
        for (const auto& [id, angle] : frame.angles) start_angles_[id] = angle;
    ++trace_run_;
    trace_tick_ = 0;
    trace_evaluate_at_ = Clock::time_point{};
    resetMotionRuntime(*next, std::move(next_name), now_());
    traceEvent("queue_activate", now_(), true);
    static_cast<void>(now);
    error_ = MotionError::None;
    last_error_.clear();
    return true;
}

MotionStatus RobotMotionPlayer::update() noexcept {
    try {
        const auto now = now_();
        if (status_ == MotionStatus::Running || status_ == MotionStatus::Settling) {
            ++trace_tick_;
            trace_evaluate_at_ = now;
            traceEvent("tick", now, true);
            if (status_ == MotionStatus::Running) return updateRunning(now);
            return updateSettling(now);
        }
        return status_;
    } catch (const std::exception& error) {
        fail(MotionError::InternalError, error.what());
    } catch (...) {
        fail(MotionError::InternalError, "unknown RobotMotionPlayer update error");
    }
    return status_;
}

bool RobotMotionPlayer::readStartAngles() {
    JointAngles present;
    const auto read_begin = now_();
    if (!hardware_->readPresentPositions(present)) {
        traceEvent("initial", read_begin, false);
        fail(MotionError::PresentPositionReadFailed, "cannot capture actual start pose");
        return false;
    }
    if (present.size() != kMotionMotorCount) {
        traceEvent("initial", read_begin, false);
        fail(MotionError::PresentPositionReadFailed, "incomplete actual start pose");
        return false;
    }
    for (const auto& [id, angle] : present) {
        if (id < 0 || id >= kMotionMotorCount || !std::isfinite(angle)) {
            fail(MotionError::PresentPositionReadFailed, "invalid actual start pose");
            return false;
        }
    }
    traceEvent("initial", read_begin, true, present);
    start_angles_ = present;
    feedback_angles_ = std::move(present);
    return true;
}

void RobotMotionPlayer::markSucceeded() {
    traceEvent(startup_active_ ? "arrived" : "goals_complete", now_(), true);
    status_ = MotionStatus::Succeeded;
    error_ = MotionError::None;
    last_error_.clear();
    if (!startup_active_) {
        last_completed_motion_ = current_motion_;
        ++completion_sequence_;
    }
}

bool RobotMotionPlayer::sendGuiSample(std::int64_t timeline_ms, bool boundary) {
    trace_timeline_ms_ = timeline_ms;
    const auto target = sampleGuiMotion(playback_frames_, start_angles_, timeline_ms, boundary);
    command_ids_.clear();
    for (int id = 0; id < kMotionMotorCount; ++id) {
        if (!target.commanded[id] && !joint_overrides_.contains(id)) continue;
        const double angle = target.commanded[id] ? target.angles[id] : command_angles_.at(id);
        command_angles_[id] = angle;
        command_ids_.push_back(id);
        final_goal_deg_[id] = (guiPositionRaw(angle) - 2048) * (360.0 / 4096.0);
    }
    traceEvent("planned", now_(), true, command_angles_, &command_ids_);
    for (int id : command_ids_) {
        if (const auto override = joint_overrides_.find(id); override != joint_overrides_.end()) {
            command_angles_[id] = override->second;
            final_goal_deg_[id] = (guiPositionRaw(override->second) - 2048) * (360.0 / 4096.0);
        }
    }
    if (!command_ids_.empty() && !writeGoals(command_angles_, command_ids_, "sample")) {
        fail(MotionError::FrameSendFailed, std::string(hardware_->lastError()));
        return false;
    }
    const auto now = now_();
    if (boundary || !feedback_sampled_ || guiSeconds(now) - last_feedback_sec_ >= 0.010) {
        last_feedback_at_ = now;
        last_feedback_sec_ = guiSeconds(now);
        feedback_sampled_ = true;
        // GUI 재생과 같이 피드백은 관측용이다. 시작 자세/정착 검사는 별도다.
        JointAngles present;
        const bool ok = hardware_->readPresentPositions(present);
        traceEvent("feedback", now, ok, present);
        if (ok) feedback_angles_ = std::move(present);
    }
    return true;
}

MotionStatus RobotMotionPlayer::updateRunning(Clock::time_point now) {
    if (pattern_ == nullptr) {
        fail(MotionError::InternalError, "running motion has no pattern");
        return status_;
    }
    if (queued_boundary_pending_) {
        if (now - queued_boundary_at_ >= std::chrono::milliseconds(queued_hold_ms_))
            activateQueuedMotion(now);
        return status_;
    }
    const double real_ms = (guiSeconds(now) - gui_started_sec_) * 1000.0;
    auto timeline_ms = static_cast<std::int64_t>(real_ms * pattern_->playbackSpeed() + 1e-7);
    const auto sequence_end = pattern_->durationMs();
    std::int64_t boundary_ms = sequence_end;
    for (const auto& frame : playback_frames_) {
        const auto end = frame.start_ms + frame.time_ms;
        if (frame.playback_cycle_end && cycle_start_ms_ < end && end < boundary_ms)
            boundary_ms = end;
    }
    // GUI gate는 도착 검사가 아니다. 직전에 활성화했던 프레임의 종점을
    // 전송한 뒤, 같은 tick에서 경계 sample을 보낸다. 늦은 tick도 시계는 유지한다.
    timeline_ms = std::min(timeline_ms, boundary_ms);
    if (gated_frame_ != nullptr) {
        const auto end = gated_frame_->start_ms + gated_frame_->time_ms;
        if (timeline_ms >= end) {
            JointAngles endpoint = gated_frame_->angles;
            for (const auto& [id, value] : joint_overrides_)
                if (endpoint.contains(id)) endpoint[id] = value;
            std::vector<int> ids;
            for (int id = 0; id < kMotionMotorCount; ++id)
                if (endpoint.contains(id)) ids.push_back(id);
            trace_timeline_ms_ = end;
            if (ids.empty() || !writeGoals(endpoint, ids, "endpoint")) {
                fail(MotionError::FrameSendFailed, "frame endpoint write failed");
                return status_;
            }
            for (const auto& [id, angle] : endpoint) command_angles_[id] = angle;
            timeline_ms = end;
            gated_frame_ = nullptr;
        }
    }
    const bool boundary = timeline_ms >= boundary_ms;
    trace_frame_ = -1;
    for (std::size_t i = 0; i < playback_frames_.size(); ++i) {
        const auto& f = playback_frames_[i];
        if ((f.start_ms <= timeline_ms && timeline_ms < f.start_ms + f.time_ms) ||
            (boundary && timeline_ms == f.start_ms + f.time_ms)) {
            trace_frame_ = static_cast<int>(i); break;
        }
    }
    if (!sendGuiSample(timeline_ms, boundary)) return status_;
    for (const auto& frame : playback_frames_) {
        const auto end = frame.start_ms + frame.time_ms;
        if ((frame.start_ms <= timeline_ms && timeline_ms < end) ||
            (boundary && timeline_ms == end)) {
            gated_frame_ = &frame;
            break;
        }
    }
    if (!boundary) return status_;
    if (boundary_ms < sequence_end) {
        traceEvent("cycle", now_(), true);
        gated_frame_ = nullptr;
        cycle_start_ms_ = boundary_ms;
        for (const auto& frame : playback_frames_)
            if (frame.start_ms + frame.time_ms == boundary_ms)
                for (const auto& [id, angle] : frame.angles) start_angles_[id] = angle;
        // 통신 완료 이후 시계를 재시작해 GUI 반복 경계의 지연을 재현한다.
        gui_started_sec_ = guiSeconds(now_()) - boundary_ms / (1000.0 * pattern_->playbackSpeed());
        started_at_ = now_() - std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double, std::milli>(boundary_ms / pattern_->playbackSpeed()));
        feedback_sampled_ = false;
        return status_;
    }
    if (repeat_ < pattern_->repeatCount()) {
        traceEvent("repeat", now_(), true);
        ++repeat_;
        gated_frame_ = nullptr;
        next_frame_ = 0;
        for (const auto& [id, angle] : playback_frames_.back().angles) start_angles_[id] = angle;
        cycle_start_ms_ = 0;
        trace_timeline_ms_ = 0;
        started_at_ = now_();
        gui_started_sec_ = guiSeconds(started_at_);
        feedback_sampled_ = false;
        return status_;
    }
    if (queued_pattern_ != nullptr) {
        traceEvent("goals_complete", now_(), true);
        if (queued_hold_ms_ == 0) activateQueuedMotion(now_());
        else { queued_boundary_pending_ = true; queued_boundary_at_ = now_(); }
        return status_;
    }
    if (startup_active_) {
        final_goal_ids_.clear();
        for (const auto& [id, angle] : final_goal_deg_) final_goal_ids_.push_back(id);
        status_ = MotionStatus::Settling;
        settling_started_at_ = now_();
        last_position_check_at_ = Clock::time_point{};
        within_tolerance_ = false;
        return updateSettling(now_());
    }
    markSucceeded();
    return status_;
}

MotionStatus RobotMotionPlayer::updateSettling(Clock::time_point now) {
    if (pattern_ == nullptr) {
        fail(MotionError::InternalError, "settling motion has no pattern");
        return status_;
    }
    // Executor가 경계 직후 Settling에서 예약했더라도 다음 update에서 즉시
    // 안전한 호환 모션으로 전환합니다.
    if (activateQueuedMotion(now)) return status_;
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - settling_started_at_).count();
    if (elapsed_ms > pattern_->completion().settle_timeout_ms) {
        fail(
            MotionError::PositionTimeout,
            "final Present Position did not reach the goal before timeout");
        return status_;
    }
    if (last_position_check_at_ != Clock::time_point{}
        && now - last_position_check_at_ < kPositionCheckInterval) {
        return status_;
    }
    last_position_check_at_ = now;

    JointAngles present_deg;
    if (!hardware_->readPresentPositions(present_deg)) {
        fail(
            MotionError::PresentPositionReadFailed,
            hardware_->lastError().empty()
                ? "failed to read final Present Position"
                : std::string(hardware_->lastError()));
        return status_;
    }

    bool all_reached = !final_goal_ids_.empty();
    for (const int motor_id : final_goal_ids_) {
        const auto goal = final_goal_deg_.find(motor_id);
        const auto present = present_deg.find(motor_id);
        if (goal == final_goal_deg_.end() || present == present_deg.end()
            || std::abs(goal->second - present->second)
                > pattern_->completion().position_tolerance_deg) {
            all_reached = false;
            break;
        }
    }

    if (!all_reached) {
        within_tolerance_ = false;
        return status_;
    }
    if (!within_tolerance_) {
        within_tolerance_ = true;
        within_tolerance_since_ = now;
        return status_;
    }
    const auto stable_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - within_tolerance_since_).count();
    if (stable_ms < pattern_->completion().settle_duration_ms)
        return status_;

    markSucceeded();
    return status_;
}

bool RobotMotionPlayer::running() const noexcept {
    return status_ == MotionStatus::Running
        || status_ == MotionStatus::Settling;
}

MotionStatus RobotMotionPlayer::status() const noexcept { return status_; }

bool RobotMotionPlayer::succeeded() const noexcept {
    return status_ == MotionStatus::Succeeded;
}

MotionError RobotMotionPlayer::result() const noexcept { return error_; }

std::string_view RobotMotionPlayer::lastError() const noexcept {
    return last_error_;
}

std::string_view RobotMotionPlayer::currentMotion() const noexcept {
    return current_motion_;
}

CancelResult RobotMotionPlayer::cancel() noexcept {
    if (!running()) return CancelResult::NotRunning;
    if (!hardwareReady()) {
        fail(MotionError::HardwareNotReady, "hardware is not ready for cancel");
        return CancelResult::HardwareNotReady;
    }
    if (!hardware_->holdCurrentPosition(kCancelHoldDurationMs)) {
        fail(
            MotionError::CancelFailed,
            hardware_->lastError().empty()
                ? "failed to hold current motor positions"
                : std::string(hardware_->lastError()));
        return CancelResult::HoldFailed;
    }
    status_ = MotionStatus::Cancelled;
    error_ = MotionError::None;
    last_error_.clear();
    pattern_ = nullptr;
    next_frame_ = 0;
    clearQueuedMotion();
    return CancelResult::Cancelled;
}

bool RobotMotionPlayer::emergencyStop() noexcept {
    if (hardware_ == nullptr) return false;
    const bool success = hardware_->setTorqueEnabled(false);
    initialized_ = false;
    status_ = success ? MotionStatus::Cancelled : MotionStatus::Failed;
    if (!success) {
        error_ = MotionError::CommunicationError;
        last_error_ = std::string(hardware_->lastError());
    }
    pattern_ = nullptr;
    next_frame_ = 0;
    clearQueuedMotion();
    return success;
}

void RobotMotionPlayer::stop() noexcept {
    if (running()) static_cast<void>(cancel());
}

bool RobotMotionPlayer::playBlocking(std::string_view motion_name) {
    if (start(motion_name) != StartResult::Accepted) return false;
    while (running()) {
        update();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return succeeded();
}

void RobotMotionPlayer::setJointCorrection(
    int motor_id, double correction_deg) {
    if (motor_id < 0 || motor_id >= kMotionMotorCount)
        throw std::out_of_range("invalid correction motor ID");
    corrections_deg_[motor_id] = correction_deg;
}

void RobotMotionPlayer::setCorrections(const JointAngles& corrections_deg) {
    for (const auto& [motor_id, correction] : corrections_deg)
        setJointCorrection(motor_id, correction);
}

void RobotMotionPlayer::clearJointCorrections() noexcept {
    corrections_deg_.clear();
}

void RobotMotionPlayer::setFrameCorrection(
    const std::string& motion_name, const std::string& frame_name,
    int motor_id, double correction_deg) {
    if (motion_name.empty() || frame_name.empty())
        throw std::invalid_argument("motion and frame names are required");
    if (!library_.contains(motion_name))
        throw std::out_of_range("unknown correction motion: " + motion_name);
    if (motor_id < 0 || motor_id >= kMotionMotorCount)
        throw std::out_of_range("invalid frame correction motor ID");
    const auto& frames = library_.motion(motion_name).frames();
    const bool matching_target = std::any_of(
        frames.begin(), frames.end(),
        [&](const MotionFrame& frame) {
            return frame.name == frame_name && frame.angles.contains(motor_id);
        });
    if (!matching_target)
        throw std::out_of_range(
            "motion/frame does not command motor " + std::to_string(motor_id));
    frame_corrections_deg_[{motion_name, frame_name, motor_id}] = correction_deg;
}

void RobotMotionPlayer::clearFrameCorrection(
    const std::string& motion_name, const std::string& frame_name,
    int motor_id) noexcept {
    frame_corrections_deg_.erase({motion_name, frame_name, motor_id});
}

void RobotMotionPlayer::clearFrameCorrections() noexcept {
    frame_corrections_deg_.clear();
}

void RobotMotionPlayer::clearCorrections() noexcept {
    clearJointCorrections();
    clearFrameCorrections();
}

void RobotMotionPlayer::shutdown() noexcept {
    // GUI 종료와 동일하게 마지막 목표와 토크 상태를 유지하고 통신 소유권만 해제한다.
    initialized_ = false;
    clearQueuedMotion();
    if (running()) status_ = MotionStatus::Cancelled;
}

void RobotMotionPlayer::fail(
    MotionError error, std::string message) noexcept {
    error_ = error;
    last_error_ = std::move(message);
    status_ = MotionStatus::Failed;
    pattern_ = nullptr;
    next_frame_ = 0;
    clearQueuedMotion();
}

JointAngles RobotMotionPlayer::correctedFrameAngles(
    const MotionFrame& frame) const {
    JointAngles corrected = frame.angles;
    if (policy_reference_ && !startup_active_) {
        const auto& frames = policy_reference_->motion(current_motion_).frames();
        const MotionFrame* matched = nullptr;
        for (const auto& candidate : frames) {
            if ((!frame.frame_id.empty() && candidate.frame_id == frame.frame_id &&
                 candidate.start_ms == frame.start_ms) ||
                (candidate.name == frame.name && candidate.start_ms == frame.start_ms)) {
                if (matched) throw std::runtime_error("ambiguous policy reference frame");
                matched = &candidate;
            }
        }
        if (!matched) throw std::runtime_error("missing policy reference frame: " + frame.name);
        for (int id : {0, 4, 5}) {
            if ((id == 0 && head_override_enabled_) || (id != 0 && shoulder_override_enabled_)) continue;
            if (matched->angles.contains(id)) corrected[id] = matched->angles.at(id);
            else corrected.erase(id);
        }
    }
    for (auto& [motor_id, degree] : corrected) {
        if (const auto correction = corrections_deg_.find(motor_id);
            correction != corrections_deg_.end()) {
            degree += correction->second;
        }
        if (const auto correction = frame_corrections_deg_.find(
                {current_motion_, frame.name, motor_id});
            correction != frame_corrections_deg_.end()) {
            degree += correction->second;
        }
    }
    return corrected;
}

void RobotMotionPlayer::setJointOverride(int motor_id, double target_deg) {
    if (motor_id == 0 && !head_override_enabled_) return;
    if (motor_id < 0 || motor_id >= kMotionMotorCount || !std::isfinite(target_deg))
        throw std::invalid_argument("invalid joint override");
    joint_overrides_[motor_id] = target_deg;
    // 대기 중에도 ROS의 카메라 이동 요청은 다음 모션을 기다리지 않는다.
    if (hardwareReady() && !running()) {
        JointAngles goal{{motor_id, target_deg}};
        if (!hardware_->commandImmediatePosition(goal, std::vector<int>{motor_id}))
            throw std::runtime_error("failed to send idle joint override");
    }
}
void RobotMotionPlayer::clearJointOverride(int motor_id) noexcept {
    joint_overrides_.erase(motor_id);
}
bool RobotMotionPlayer::startPoseTransition(
    const std::vector<double>& angles, std::int64_t duration_ms) noexcept {
    if (running() || !hardwareReady()) return false;
    try {
        if (angles.size() != kMotionMotorCount || duration_ms <= 0)
            throw std::invalid_argument("startup pose requires 23 angles and positive duration");
        current_motion_ = "STEP startup";
        ++trace_run_;
        trace_tick_ = 0;
        trace_evaluate_at_ = Clock::time_point{};
        trace_timeline_ms_ = 0;
        trace_frame_ = -1;
        repeat_ = 1;
        MotionFrame frame;
        frame.name = "STEP startup";
        frame.time_ms = duration_ms;
        for (int id = 0; id < kMotionMotorCount; ++id) {
            if (!std::isfinite(angles[id])) throw std::invalid_argument("non-finite startup angle");
            frame.angles[id] = angles[id];
        }
        if (!hardware_->prepareImmediatePlayback()) return false;
        if (!readStartAngles() || start_angles_.size() != kMotionMotorCount) return false;
        MotionCompletion completion;
        completion.position_tolerance_deg = 5.0;
        startup_pattern_ = std::make_unique<MotionPattern>(std::vector<MotionFrame>{frame},
            duration_ms, 1, 1.0, false, "", "", completion);
        startup_active_ = true;
        clearQueuedMotion();
        resetMotionRuntime(*startup_pattern_, "STEP startup", now_());
        error_ = MotionError::None;
        last_error_.clear();
        return true;
    } catch (const std::exception& e) {
        fail(MotionError::InternalError, e.what());
        return false;
    } catch (...) {
        fail(MotionError::InternalError, "unknown startup pose error");
        return false;
    }
}
MotionStatus RobotMotionPlayer::updateStartupPose() noexcept {
    if (!startup_active_) return MotionStatus::Failed;
    return update();
}

void RobotMotionPlayer::configureJointPolicy(const std::string& reference, bool head, bool shoulder) {
    if (running()) throw std::logic_error("cannot change policy during playback");
    if (!head || !shoulder)
        policy_reference_ = std::make_unique<MotionLibrary>(MotionLibrary::loadGuiJson(reference));
    else policy_reference_.reset();
    head_override_enabled_ = head;
    shoulder_override_enabled_ = shoulder;
    if (!head) joint_overrides_.erase(0);
}
void RobotMotionPlayer::setQueuedTransitionHoldMs(std::int64_t ms) {
    if (ms < 0) throw std::invalid_argument("negative queue hold");
    queued_hold_ms_ = ms;
}
void RobotMotionPlayer::configureDiagnostics(double tick_ms, const std::string& path, bool goals) {
    if (running() || tick_ms <= 0) throw std::invalid_argument("invalid trace configuration");
    trace_.reset();
    trace_goals_ = goals;
    if (!path.empty()) trace_ = std::make_unique<PlaybackTrace>(path);
}
void RobotMotionPlayer::traceEvent(const char* event, Clock::time_point begin, bool success,
                                  const JointAngles& angles, const std::vector<int>* ids) {
    if (!trace_) return;
    PlaybackTraceRow row;
    row.run = trace_run_;
    row.event_seq = ++trace_event_seq_;
    row.tick = trace_tick_;
    std::snprintf(row.motion.data(), row.motion.size(), "%s", current_motion_.c_str());
    row.event = event;
    const auto end = now_();
    row.begin_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(begin.time_since_epoch()).count();
    row.end_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end.time_since_epoch()).count();
    row.evaluate_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(trace_evaluate_at_.time_since_epoch()).count();
    row.begin_ms = row.begin_ns / 1e6;
    row.end_ms = row.end_ns / 1e6;
    if (pattern_) {
        row.speed = pattern_->playbackSpeed();
        row.repeat_target = pattern_->repeatCount();
    }
    if (trace_frame_ >= 0 && static_cast<std::size_t>(trace_frame_) < playback_frames_.size()) {
        row.frame_start_ms = playback_frames_[trace_frame_].start_ms;
        row.frame_duration_ms = playback_frames_[trace_frame_].time_ms;
        const auto& frame = playback_frames_[trace_frame_];
        std::snprintf(row.frame_name.data(), row.frame_name.size(), "%s", frame.name.c_str());
        row.lift_early = frame.lift_early_arrival;
        row.cycle_end = frame.playback_cycle_end;
    }
    row.timeline_ms = trace_timeline_ms_; row.repeat = repeat_; row.frame = trace_frame_;
    row.success = success;
    if (trace_goals_) for (const auto& [id, value] : angles) {
        if (id >= 0 && id < 23 && (!ids || std::find(ids->begin(), ids->end(), id) != ids->end())) { row.ids[id] = true; row.angles[id] = value; }
    }
    trace_->push(row);
}
bool RobotMotionPlayer::writeGoals(const JointAngles& goals, const std::vector<int>& ids,
                                  const char* event) {
    const auto begin = now_();
    const bool ok = hardware_->commandImmediatePosition(goals, ids);
    traceEvent(event, begin, ok, goals, &ids);
    return ok;
}

}  // namespace irc_step
