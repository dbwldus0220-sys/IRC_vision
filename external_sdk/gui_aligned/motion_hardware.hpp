#pragma once

#include "motion_pattern.hpp"

#include <cstdint>
#include <string_view>
#include <vector>

namespace irc_step {

inline constexpr int kMotionMotorCount = 23;
inline constexpr std::uint32_t kMaxTimeProfileMs = 32737;

// RobotMotionPlayer와 실제 Dynamixel 통신을 분리하는 최소 인터페이스입니다.
// ROS/mock 테스트에서는 이 인터페이스의 가짜 구현을 주입할 수 있습니다.
class IMotionHardware {
public:
    virtual ~IMotionHardware() = default;

    virtual bool initialize() noexcept = 0;
    [[nodiscard]] virtual bool ready() const noexcept = 0;

    virtual bool prepareImmediatePlayback() noexcept { return true; }

    virtual bool commandPosition(
        const JointAngles& goal_deg,
        const std::vector<int>& motor_ids,
        std::uint32_t duration_ms,
        std::uint32_t acceleration_ms) noexcept = 0;

    // 프로파일 0 상태의 중간 Goal만 전송한다. 시간 프로파일 전송과 분리한다.
    virtual bool commandImmediatePosition(const JointAngles& goal_deg,
        const std::vector<int>& motor_ids) noexcept {
        return commandPosition(goal_deg, motor_ids, 0, 0);
    }

    virtual bool readPresentPositions(JointAngles& positions_deg) noexcept = 0;
    virtual bool holdCurrentPosition(
        std::uint32_t stop_duration_ms) noexcept = 0;
    virtual bool setTorqueEnabled(bool enabled) noexcept = 0;

    [[nodiscard]] virtual std::string_view lastError() const noexcept = 0;
};

}  // namespace irc_step
