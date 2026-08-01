#include "irc_step_motion_executor/robot_motion_runtime_factory.hpp"

namespace irc_step_motion_executor
{

RobotMotionRuntimeFactoryResult ProductionRobotMotionRuntimeFactory::create(
  const RobotMotionRuntimeConfig & config)
{
  const auto config_result = validate_robot_motion_runtime_config(config);
  if (!config_result) {
    return {{}, config_result.error_code, config_result.message};
  }

  return {
    {},
    "ROBOT_MOTION_RUNTIME_NOT_SAFE_TO_INSTANTIATE",
    "SDK hardware constructor accesses /dev/ttyUSB0; constructor changes "
    "Dynamixel torque/operating mode; explicit hardware approval and SDK "
    "refactor are required"};
}

}  // namespace irc_step_motion_executor
