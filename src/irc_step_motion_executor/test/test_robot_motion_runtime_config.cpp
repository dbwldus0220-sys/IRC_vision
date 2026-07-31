#include "irc_step_motion_executor/robot_motion_runtime_config.hpp"

#include <gtest/gtest.h>

#include <map>
#include <string>

namespace
{

TEST(RobotMotionRuntimeConfig, RejectsMissingPath)
{
  const auto result =
    irc_step_motion_executor::validate_robot_motion_runtime_config({});

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "MOTION_JSON_PATH_REQUIRED");
}

TEST(RobotMotionRuntimeConfig, RejectsMissingFile)
{
  irc_step_motion_executor::RobotMotionRuntimeConfig config;
  config.motion_json_path =
    "/definitely/not/a/robot_motion_runtime_config.json";

  const auto result =
    irc_step_motion_executor::validate_robot_motion_runtime_config(config);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "MOTION_JSON_FILE_NOT_FOUND");
}

TEST(RobotMotionRuntimeConfig, RejectsUnknownSetting)
{
  const auto result =
    irc_step_motion_executor::parse_robot_motion_runtime_config(
    {{"device_path", "/dev/ttyUSB0"}});

  EXPECT_FALSE(result);
  EXPECT_EQ(
    result.error_code, "UNKNOWN_ROBOT_MOTION_RUNTIME_SETTING");
}

}  // namespace
