#include "irc_step_motion_executor/robot_motion_runtime_factory.hpp"

#include "fake_robot_motion_sdk_test_support.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace
{

TEST(ProductionRobotMotionRuntimeFactory, CreatesUninitializedOwnedRuntime)
{
  irc_step::fake_sdk::reset_tracking();
  irc_step_motion_executor::ProductionRobotMotionRuntimeFactory factory;
  irc_step_motion_executor::RobotMotionRuntimeConfig config;
  config.motion_json_path = TEST_EXISTING_RUNTIME_FILE;

  auto result = factory.create(config);

  ASSERT_TRUE(result);
  ASSERT_NE(result.runtime.backend, nullptr);
  ASSERT_NE(result.runtime.runtime_owner, nullptr);
  EXPECT_EQ(irc_step::fake_sdk::hardware_construction_count(), 1);
  EXPECT_EQ(irc_step::fake_sdk::player_construction_count(), 1);
  EXPECT_EQ(irc_step::fake_sdk::hardware_initialize_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::player_initialize_count(), 0);

  const auto start_result = result.runtime.backend->start_motion("test_motion");
  EXPECT_FALSE(start_result.accepted);
  EXPECT_EQ(start_result.error_code, "SDK_HARDWARE_NOT_READY");
  EXPECT_EQ(irc_step::fake_sdk::hardware_initialize_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::player_initialize_count(), 0);

  result.runtime.backend.reset();
  EXPECT_TRUE(irc_step::fake_sdk::destruction_order().empty());
  result.runtime.runtime_owner.reset();
  EXPECT_EQ(
    irc_step::fake_sdk::destruction_order(),
    (std::vector<std::string>{"player", "hardware"}));
}

TEST(ProductionRobotMotionRuntimeFactory, InvalidConfigErrorTakesPrecedence)
{
  irc_step::fake_sdk::reset_tracking();
  irc_step_motion_executor::ProductionRobotMotionRuntimeFactory factory;

  const auto result = factory.create({});

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "MOTION_JSON_PATH_REQUIRED");
  EXPECT_EQ(result.runtime.backend, nullptr);
  EXPECT_EQ(result.runtime.runtime_owner, nullptr);
  EXPECT_EQ(irc_step::fake_sdk::hardware_construction_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::player_construction_count(), 0);
}

TEST(ProductionRobotMotionRuntimeFactory, ConvertsConstructionException)
{
  irc_step::fake_sdk::reset_tracking();
  irc_step::fake_sdk::set_player_constructor_throws(true);
  irc_step_motion_executor::ProductionRobotMotionRuntimeFactory factory;
  irc_step_motion_executor::RobotMotionRuntimeConfig config;
  config.motion_json_path = TEST_EXISTING_RUNTIME_FILE;

  const auto result = factory.create(config);

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error_code, "ROBOT_MOTION_RUNTIME_CREATION_FAILED");
  EXPECT_NE(result.message.find("fake player construction failed"), std::string::npos);
  EXPECT_EQ(result.runtime.backend, nullptr);
  EXPECT_EQ(result.runtime.runtime_owner, nullptr);
  EXPECT_EQ(irc_step::fake_sdk::hardware_initialize_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::player_initialize_count(), 0);
  EXPECT_EQ(
    irc_step::fake_sdk::destruction_order(),
    (std::vector<std::string>{"hardware"}));
}

TEST(ProductionRobotMotionRuntimeFactory, PolicyValidationCreatesNoSdkObjects)
{
  irc_step::fake_sdk::reset_tracking();
  irc_step_motion_executor::RobotMotionRuntimeConfig config;
  config.motion_json_path = TEST_EXISTING_RUNTIME_FILE;
  config.enable_robot_hardware = true;
  config.device_path = "/dev/ttyUSB0";
  config.baud_rate = 4000000;
  config.motor_ids = {0, 1, 22};
  config.explicit_torque_approval = true;

  const auto result =
    irc_step_motion_executor::validate_robot_hardware_initialization_policy(config);

  EXPECT_TRUE(result);
  EXPECT_EQ(irc_step::fake_sdk::hardware_construction_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::player_construction_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::hardware_initialize_count(), 0);
  EXPECT_EQ(irc_step::fake_sdk::player_initialize_count(), 0);
}

}  // namespace
