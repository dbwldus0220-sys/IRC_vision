#ifndef IRC_STEP_MOTION_EXECUTOR__ROBOT_MOTION_RUNTIME_CONFIG_HPP_
#define IRC_STEP_MOTION_EXECUTOR__ROBOT_MOTION_RUNTIME_CONFIG_HPP_

#include <map>
#include <string>

namespace irc_step_motion_executor
{

struct RobotMotionRuntimeConfig
{
  std::string motion_json_path;
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

RobotMotionRuntimeConfigResult parse_robot_motion_runtime_config(
  const std::map<std::string, std::string> & settings);

RobotMotionRuntimeConfigResult validate_robot_motion_runtime_config(
  const RobotMotionRuntimeConfig & config);

}  // namespace irc_step_motion_executor

#endif  // IRC_STEP_MOTION_EXECUTOR__ROBOT_MOTION_RUNTIME_CONFIG_HPP_
