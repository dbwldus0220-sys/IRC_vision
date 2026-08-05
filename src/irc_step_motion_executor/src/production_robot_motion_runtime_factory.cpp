#include "irc_step_motion_executor/robot_motion_runtime_factory.hpp"

#include "irc_step_motion_executor/robot_motion_player_backend.hpp"

#include "dynamixel_motion_hardware.hpp"

#include <exception>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

namespace irc_step_motion_executor
{
namespace
{

irc_step::DynamixelMotionHardwareConfig to_sdk_hardware_config(
  const RobotMotionRuntimeConfig & config)
{
  irc_step::DynamixelMotionHardwareConfig sdk_config;
  sdk_config.device_path = config.device_path;
  sdk_config.baud_rate = config.baud_rate;
  sdk_config.motor_ids.reserve(config.motor_ids.size());
  for (const std::int64_t motor_id : config.motor_ids) {
    if (motor_id < std::numeric_limits<int>::min() ||
      motor_id > std::numeric_limits<int>::max())
    {
      throw std::out_of_range(
              "RobotMotionRuntimeConfig motor ID cannot be represented as int");
    }
    sdk_config.motor_ids.push_back(static_cast<int>(motor_id));
  }
  return sdk_config;
}

class ProductionRobotMotionRuntimeOwner
{
public:
  explicit ProductionRobotMotionRuntimeOwner(
    const RobotMotionRuntimeConfig & config)
  : hardware_(to_sdk_hardware_config(config)),
    player_(config.motion_json_path, hardware_),
    player_api_(player_)
  {
  }

  BorrowedRobotMotionPlayerApi & player_api() noexcept
  {
    return player_api_;
  }

  bool initialize()
  {
    return player_.initialize();
  }

  bool preflight()
  {
    return hardware_.preflight();
  }

  std::string last_error() const
  {
    return std::string(player_.lastError());
  }

  std::string preflight_error() const
  {
    return std::string(hardware_.lastError());
  }

private:
  // Members are destroyed in reverse declaration order: API, player, hardware.
  irc_step::DynamixelMotionHardware hardware_;
  irc_step::RobotMotionPlayer player_;
  BorrowedRobotMotionPlayerApi player_api_;
};

RobotMotionRuntimeFactoryResult creation_error(std::string message)
{
  return {
    {}, "ROBOT_MOTION_RUNTIME_CREATION_FAILED", std::move(message)};
}

RobotMotionRuntimeFactoryResult initialization_error(std::string message)
{
  return {
    {}, "ROBOT_MOTION_RUNTIME_INITIALIZATION_FAILED", std::move(message)};
}

RobotMotionPreflightResult preflight_error(std::string message)
{
  return {
    {}, "ROBOT_MOTION_RUNTIME_PREFLIGHT_FAILED", std::move(message)};
}

}  // namespace

RobotMotionPreflightResult ProductionRobotMotionRuntimeFactory::preflight(
  const RobotMotionRuntimeConfig & config)
{
  const auto config_result = validate_robot_hardware_preflight_policy(config);
  if (!config_result) {
    return {{}, config_result.error_code, config_result.message};
  }

  try {
    auto owner = std::make_shared<ProductionRobotMotionRuntimeOwner>(
      config_result.config);
    if (!owner->preflight()) {
      const auto sdk_message = owner->preflight_error();
      return preflight_error(
        sdk_message.empty() ?
        "RobotMotionPlayer hardware preflight failed" : sdk_message);
    }
    return {std::move(owner), "", ""};
  } catch (const std::exception & exception) {
    return {
      {}, "ROBOT_MOTION_RUNTIME_CREATION_FAILED",
      "failed to create RobotMotionPlayer preflight objects: " +
      std::string(exception.what())};
  } catch (...) {
    return {
      {}, "ROBOT_MOTION_RUNTIME_CREATION_FAILED",
      "failed to create RobotMotionPlayer preflight objects: unknown exception"};
  }
}

RobotMotionRuntimeFactoryResult ProductionRobotMotionRuntimeFactory::create(
  const RobotMotionRuntimeConfig & config)
{
  const auto config_result =
    validate_robot_hardware_initialization_policy(config);
  if (!config_result) {
    return {{}, config_result.error_code, config_result.message};
  }

  try {
    auto owner = std::make_shared<ProductionRobotMotionRuntimeOwner>(
      config_result.config);
    if (!owner->initialize()) {
      const auto sdk_message = owner->last_error();
      return initialization_error(
        sdk_message.empty() ?
        "RobotMotionPlayer hardware initialization failed" : sdk_message);
    }
    RobotMotionRuntime runtime;
    runtime.runtime_owner = owner;
    runtime.backend =
      std::make_unique<RobotMotionPlayerBackend>(owner->player_api());
    return {std::move(runtime), "", ""};
  } catch (const std::exception & exception) {
    return creation_error(
      "failed to create RobotMotionPlayer runtime objects: " +
      std::string(exception.what()));
  } catch (...) {
    return creation_error(
      "failed to create RobotMotionPlayer runtime objects: unknown exception");
  }
}

}  // namespace irc_step_motion_executor
