#include "dynamixel_motion_hardware.hpp"

#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <exception>
#include <utility>

namespace irc_step {

DynamixelMotionHardware::DynamixelMotionHardware(DynamixelMotionHardwareConfig config)
    : config_(std::move(config)), desired_deg_(NUMBER_OF_DYNAMIXELS, 0.0), controller_(&dxl_),
      desired_rad_(Eigen::VectorXd::Zero(NUMBER_OF_DYNAMIXELS)) {
    auto ids = config_.motor_ids;
    std::sort(ids.begin(), ids.end());
    if (config_.device_path.empty() || config_.baud_rate <= 0 || ids.size() != 23)
        throw std::invalid_argument("invalid hardware configuration");
    for (int id = 0; id < 23; ++id)
        if (ids[id] != id) throw std::invalid_argument("hardware requires IDs 0..22 exactly once");
}

void DynamixelMotionHardware::setError(std::string message) noexcept {
    last_error_ = std::move(message);
}

bool DynamixelMotionHardware::preflight() noexcept {
    try {
        if (!dxl_.Initialize(config_.device_path, config_.baud_rate)) {
            setError("failed to open DYNAMIXEL port"); return false;
        }
        if (!dxl_.SetTorqueEnabled(false)) {
            setError("failed to disable torque"); return false;
        }
        if (!dxl_.ReadPositionDegrees(desired_deg_)) {
            setError("failed to read all motor positions"); return false;
        }
        initialized_ = false;
        last_error_.clear();
        return true;
    } catch (const std::exception& e) { setError(e.what()); return false; }
      catch (...) { setError("unknown hardware preflight error"); return false; }
}

bool DynamixelMotionHardware::initialize() noexcept {
    try {
        if (!preflight()) return false;
        if (!dxl_.ConfigureTimeBasedProfile()) {
            setError("failed to prepare profile 0"); return false;
        }
        // 토크 ON 직전에 현재 자세를 다시 읽어 동일 Goal을 선등록한다.
        if (!dxl_.ReadPositionDegrees(desired_deg_) ||
            !dxl_.WriteGoalDegrees(desired_deg_, config_.motor_ids)) {
            setError("failed to seed current-position goals before torque ON"); return false;
        }
        if (!dxl_.SetTorqueEnabled(true)) {
            static_cast<void>(dxl_.SetTorqueEnabled(false));
            setError("failed to enable torque"); return false;
        }
        initialized_ = true;
        last_error_.clear();
        return true;
    } catch (const std::exception& e) { setError(e.what()); return false; }
      catch (...) { setError("unknown hardware initialization error"); return false; }
}

bool DynamixelMotionHardware::prepareImmediatePlayback() noexcept {
    if (!ready() || !dxl_.WriteZeroProfiles()) {
        setError("failed to set Profile Acceleration/Velocity to zero"); return false;
    }
    return true;
}

bool DynamixelMotionHardware::commandImmediatePosition(
    const JointAngles& goals, const std::vector<int>& ids) noexcept {
    if (!ready()) { setError("hardware is not ready"); return false; }
    for (const int id : ids) {
        const auto value = goals.find(id);
        if (id < 0 || id >= 23 || value == goals.end() || !std::isfinite(value->second)) {
            setError("invalid immediate motor goal"); return false;
        }
        desired_deg_[id] = value->second;
    }
    if (!dxl_.WriteGoalDegrees(desired_deg_, ids)) {
        setError("Goal Position SyncWrite failed"); return false;
    }
    last_error_.clear();
    return true;
}

bool DynamixelMotionHardware::ready() const noexcept {
    return initialized_ && dxl_.IsReady();
}

bool DynamixelMotionHardware::commandPosition(
    const JointAngles& goal_deg, const std::vector<int>& motor_ids,
    std::uint32_t duration_ms, std::uint32_t acceleration_ms) noexcept {
    if (!ready()) {
        setError("DYNAMIXEL hardware is not initialized");
        return false;
    }
    for (const int motor_id : motor_ids) {
        const auto goal = goal_deg.find(motor_id);
        if (motor_id < 0 || motor_id >= NUMBER_OF_DYNAMIXELS
            || goal == goal_deg.end()) {
            setError("invalid or missing motor goal");
            return false;
        }
        desired_rad_[motor_id] = goal->second * DEG2RAD;
    }
    if (!controller_.SetTimeBasedPosition(
            desired_rad_, motor_ids, duration_ms, acceleration_ms)) {
        setError("DYNAMIXEL time-profile SyncWrite failed");
        return false;
    }
    last_error_.clear();
    return true;
}

bool DynamixelMotionHardware::readPresentPositions(
    JointAngles& positions_deg) noexcept {
    if (!ready()) {
        setError("DYNAMIXEL hardware is not initialized");
        return false;
    }
    try {
        if (!dxl_.ReadPositionDegrees(desired_deg_)) {
            setError("Present Position SyncRead failed"); return false;
        }
        positions_deg.clear();
        for (int id = 0; id < 23; ++id) positions_deg[id] = desired_deg_[id];
        last_error_.clear();
        return true;
    } catch (const std::exception& error) {
        setError(error.what());
        return false;
    } catch (...) {
        setError("unknown Present Position read error");
        return false;
    }
}

bool DynamixelMotionHardware::holdCurrentPosition(
    std::uint32_t stop_duration_ms) noexcept {
    JointAngles current_deg;
    if (!readPresentPositions(current_deg)) return false;
    std::vector<int> motor_ids;
    motor_ids.reserve(current_deg.size());
    for (const auto& [motor_id, unused] : current_deg)
        motor_ids.push_back(motor_id);
    static_cast<void>(stop_duration_ms);
    return commandImmediatePosition(current_deg, motor_ids);
}

bool DynamixelMotionHardware::setTorqueEnabled(bool enabled) noexcept {
    if (!dxl_.IsReady()) {
        setError("DYNAMIXEL port is not ready");
        return false;
    }
    if (!controller_.SetTorqueEnabled(enabled)) {
        setError(enabled ? "failed to enable torque"
                         : "failed to disable torque");
        return false;
    }
    if (!enabled) initialized_ = false;
    last_error_.clear();
    return true;
}

std::string_view DynamixelMotionHardware::lastError() const noexcept {
    return last_error_;
}

}  // namespace irc_step
