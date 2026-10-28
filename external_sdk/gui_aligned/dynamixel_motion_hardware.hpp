#pragma once

#include "dynamixel_controller.hpp"
#include "motion_hardware.hpp"

#include <string>
#include <mutex>

namespace irc_step {

struct DynamixelMotionHardwareConfig {
    std::string device_path{"/dev/ttyUSB0"};
    int baud_rate{4000000};
    bool exclude_motor_5{false};
    std::vector<int> motor_ids{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22};
};

class DynamixelMotionHardware final : public IMotionHardware {
public:
    explicit DynamixelMotionHardware(DynamixelMotionHardwareConfig config = {});
    bool preflight() noexcept;
    bool motorEnabled(int id) const noexcept override {
        return id >= 0 && id < 23 && !(config_.exclude_motor_5 && id == 5);
    }
    bool commandImmediatePosition(const JointAngles& goal_deg,
        const std::vector<int>& motor_ids) noexcept override;

    bool initialize() noexcept override;
    [[nodiscard]] bool ready() const noexcept override;
    bool prepareImmediatePlayback() noexcept override;
    bool commandPosition(
        const JointAngles& goal_deg,
        const std::vector<int>& motor_ids,
        std::uint32_t duration_ms,
        std::uint32_t acceleration_ms) noexcept override;
    bool readPresentPositions(JointAngles& positions_deg) noexcept override;
    bool holdCurrentPosition(
        std::uint32_t stop_duration_ms) noexcept override;
    bool setTorqueEnabled(bool enabled) noexcept override;
    [[nodiscard]] std::string_view lastError() const noexcept override;
    [[nodiscard]] BusDiagnostics busDiagnostics() const noexcept override {
        const std::lock_guard<std::recursive_mutex> lock(bus_mutex_);
        return dxl_.BusDiagnostics();
    }

private:
    void setError(std::string message) noexcept;

    DynamixelMotionHardwareConfig config_;
    std::vector<double> desired_deg_;
    Dxl dxl_;
    Dxl_Controller controller_;
    Eigen::VectorXd desired_rad_;
    std::string last_error_;
    bool initialized_{false};
    // initialize -> preflight 및 hold -> read/write의 중첩 호출도 직렬화한다.
    mutable std::recursive_mutex bus_mutex_;
};

}  // namespace irc_step
