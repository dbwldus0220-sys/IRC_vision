#include "irc_step_motion_executor/robot_motion_runtime_factory.hpp"

#include <gtest/gtest.h>

#include <string>

namespace
{

TEST(ProductionRobotMotionRuntimeFactory, ValidConfigIsBlockedWithoutRuntime)
{
  irc_step_motion_executor::ProductionRobotMotionRuntimeFactory factory;
  irc_step_motion_executor::RobotMotionRuntimeConfig config;
  config.motion_json_path = TEST_EXISTING_RUNTIME_FILE;

  const auto result = factory.create(config);

  EXPECT_FALSE(result);
  EXPECT_EQ(
    result.error_code, "ROBOT_MOTION_RUNTIME_NOT_SAFE_TO_INSTANTIATE");
  EXPECT_EQ(result.runtime.backend, nullptr);
  EXPECT_EQ(result.runtime.runtime_owner, nullptr);
  EXPECT_NE(result.message.find("accesses /dev/ttyUSB0"), std::string::npos);
  EXPECT_NE(
    result.message.find("changes Dynamixel torque/operating mode"),
    std::string::npos);
  EXPECT_NE(
    result.message.find("explicit hardware approval and SDK refactor are required"),
    std::string::npos);
}

TEST(ProductionRobotMotionRuntimeFactory, InvalidConfigErrorTakesPrecedence)
{
  irc_step_motion_executor::ProductionRobotMotionRuntimeFactory factory;

  const auto result = factory.create({});

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "MOTION_JSON_PATH_REQUIRED");
  EXPECT_EQ(result.runtime.backend, nullptr);
  EXPECT_EQ(result.runtime.runtime_owner, nullptr);
}

}  // namespace
