#ifndef IRC_STEP_MOTION_EXECUTOR__ROBOT_MOTION_RUNTIME_CONFIG_HPP_
#define IRC_STEP_MOTION_EXECUTOR__ROBOT_MOTION_RUNTIME_CONFIG_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace irc_step_motion_executor
{

struct RobotMotionRuntimeConfig
{
  std::string motion_json_path;
  // Initialization-policy prerequisites only. The current SDK still uses
  // fixed device, baud-rate, and motor-ID constants and does not receive them.
  bool enable_robot_hardware{false};
  std::string device_path;
  std::int64_t baud_rate{0};
  std::vector<std::int64_t> motor_ids;
  bool explicit_torque_approval{false};
};

struct RobotMotionRuntimeConfigResult
{
  RobotMotionRuntimeConfig config;
  std::string error_code;
  std::string message;

  explicit operator bool() const noexcept
  {
    return error_code.empty();
  }
};

RobotMotionRuntimeConfig make_robot_motion_runtime_config(
  std::string motion_json_path,
  bool enable_robot_hardware,
  std::string device_path,
  std::int64_t baud_rate,
  std::vector<std::int64_t> motor_ids,
  bool explicit_torque_approval);

RobotMotionRuntimeConfigResult parse_robot_motion_runtime_config(
  const std::map<std::string, std::string> & settings);

RobotMotionRuntimeConfigResult validate_robot_motion_runtime_config(
  const RobotMotionRuntimeConfig & config);

RobotMotionRuntimeConfigResult validate_robot_hardware_initialization_policy(
  const RobotMotionRuntimeConfig & config);

}  // namespace irc_step_motion_executor

#endif  // IRC_STEP_MOTION_EXECUTOR__ROBOT_MOTION_RUNTIME_CONFIG_HPP_
