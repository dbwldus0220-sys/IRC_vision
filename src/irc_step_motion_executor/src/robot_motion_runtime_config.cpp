#include "irc_step_motion_executor/robot_motion_runtime_config.hpp"

#include <filesystem>
#include <utility>

namespace irc_step_motion_executor
{
namespace
{

RobotMotionRuntimeConfigResult error(
  std::string error_code, std::string message)
{
  return {{}, std::move(error_code), std::move(message)};
}

}  // namespace

RobotMotionRuntimeConfigResult parse_robot_motion_runtime_config(
  const std::map<std::string, std::string> & settings)
{
  for (const auto & [name, unused] : settings) {
    static_cast<void>(unused);
    if (name != "motion_json_path") {
      return error(
        "UNKNOWN_ROBOT_MOTION_RUNTIME_SETTING",
        "unknown RobotMotionPlayer runtime setting '" + name +
        "'; allowed setting is: motion_json_path");
    }
  }

  RobotMotionRuntimeConfig config;
  const auto motion_json = settings.find("motion_json_path");
  if (motion_json != settings.end()) {
    config.motion_json_path = motion_json->second;
  }
  return validate_robot_motion_runtime_config(config);
}

RobotMotionRuntimeConfigResult validate_robot_motion_runtime_config(
  const RobotMotionRuntimeConfig & config)
{
  if (config.motion_json_path.empty()) {
    return error(
      "MOTION_JSON_PATH_REQUIRED",
      "motion_json_path must be explicitly configured");
  }

  const std::filesystem::path path(config.motion_json_path);
  std::error_code filesystem_error;
  if (!std::filesystem::exists(path, filesystem_error) || filesystem_error) {
    return error(
      "MOTION_JSON_FILE_NOT_FOUND",
      "motion JSON file does not exist: " + config.motion_json_path);
  }
  if (!std::filesystem::is_regular_file(path, filesystem_error) ||
    filesystem_error)
  {
    return error(
      "MOTION_JSON_PATH_NOT_FILE",
      "motion_json_path is not a regular file: " + config.motion_json_path);
  }

  return {config, "", ""};
}

}  // namespace irc_step_motion_executor
