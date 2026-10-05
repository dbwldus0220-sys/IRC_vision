#pragma once

#include "dynamixel_controller.hpp"
#include "motion_hardware.hpp"

#include <string>

namespace irc_step {

struct DynamixelMotionHardwareConfig {
    std::string device_path{"/dev/ttyUSB0"};
    int baud_rate{4000000};
    std::vector<int> motor_ids{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22};
};

class DynamixelMotionHardware final : public IMotionHardware {
public:
    explicit DynamixelMotionHardware(DynamixelMotionHardwareConfig config = {});
    bool preflight() noexcept;
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

private:
    void setError(std::string message) noexcept;

    DynamixelMotionHardwareConfig config_;
    std::vector<double> desired_deg_;
    Dxl dxl_;
    Dxl_Controller controller_;
    Eigen::VectorXd desired_rad_;
    std::string last_error_;
    bool initialized_{false};
};

}  // namespace irc_step
