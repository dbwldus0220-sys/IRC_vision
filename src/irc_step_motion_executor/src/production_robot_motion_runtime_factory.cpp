#include "irc_step_motion_executor/robot_motion_runtime_factory.hpp"

#include "irc_step_motion_executor/robot_motion_player_backend.hpp"

#include "dynamixel_motion_hardware.hpp"

#include <exception>
#include <memory>
#include <utility>

namespace irc_step_motion_executor
{
namespace
{

class ProductionRobotMotionRuntimeOwner
{
public:
  explicit ProductionRobotMotionRuntimeOwner(const std::string & motion_json_path)
  : hardware_(),
    player_(motion_json_path, hardware_),
    player_api_(player_)
  {
  }

  BorrowedRobotMotionPlayerApi & player_api() noexcept
  {
    return player_api_;
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

}  // namespace

RobotMotionRuntimeFactoryResult ProductionRobotMotionRuntimeFactory::create(
  const RobotMotionRuntimeConfig & config)
{
  const auto config_result = validate_robot_motion_runtime_config(config);
  if (!config_result) {
    return {{}, config_result.error_code, config_result.message};
  }

  try {
    auto owner = std::make_shared<ProductionRobotMotionRuntimeOwner>(
      config_result.config.motion_json_path);
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
