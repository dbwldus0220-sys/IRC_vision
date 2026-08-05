#ifndef IRC_STEP_MOTION_EXECUTOR__ROBOT_MOTION_RUNTIME_FACTORY_HPP_
#define IRC_STEP_MOTION_EXECUTOR__ROBOT_MOTION_RUNTIME_FACTORY_HPP_

#include "irc_step_motion_executor/motion_backend.hpp"
#include "irc_step_motion_executor/robot_motion_runtime_config.hpp"

#include <memory>
#include <string>

namespace irc_step_motion_executor
{

struct RobotMotionRuntime
{
  // Declared first so it is destroyed after the backend that may borrow it.
  std::shared_ptr<void> runtime_owner;
  std::unique_ptr<MotionBackend> backend;
};

struct RobotMotionRuntimeFactoryResult
{
  RobotMotionRuntime runtime;
  std::string error_code;
  std::string message;

  explicit operator bool() const noexcept
  {
    return runtime.backend != nullptr;
  }
};

struct RobotMotionPreflightResult
{
  std::shared_ptr<void> runtime_owner;
  std::string error_code;
  std::string message;

  explicit operator bool() const noexcept
  {
    return runtime_owner != nullptr && error_code.empty();
  }
};

class RobotMotionRuntimeFactory
{
public:
  virtual ~RobotMotionRuntimeFactory() = default;

  virtual RobotMotionRuntimeFactoryResult create(
    const RobotMotionRuntimeConfig & config) = 0;
};

// The SDK-backed implementation is available only in SDK-enabled builds.
class ProductionRobotMotionRuntimeFactory final
  : public RobotMotionRuntimeFactory
{
public:
  RobotMotionPreflightResult preflight(
    const RobotMotionRuntimeConfig & config);

  RobotMotionRuntimeFactoryResult create(
    const RobotMotionRuntimeConfig & config) override;
};

}  // namespace irc_step_motion_executor

#endif  // IRC_STEP_MOTION_EXECUTOR__ROBOT_MOTION_RUNTIME_FACTORY_HPP_
