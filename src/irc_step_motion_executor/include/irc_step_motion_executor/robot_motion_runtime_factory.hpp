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

class RobotMotionRuntimeFactory
{
public:
  virtual ~RobotMotionRuntimeFactory() = default;

  virtual RobotMotionRuntimeFactoryResult create(
    const RobotMotionRuntimeConfig & config) = 0;
};

// This factory deliberately contains no SDK types.  It remains blocked until
// the SDK constructors are made free of hardware side effects.
class ProductionRobotMotionRuntimeFactory final
  : public RobotMotionRuntimeFactory
{
public:
  RobotMotionRuntimeFactoryResult create(
    const RobotMotionRuntimeConfig & config) override;
};

}  // namespace irc_step_motion_executor

#endif  // IRC_STEP_MOTION_EXECUTOR__ROBOT_MOTION_RUNTIME_FACTORY_HPP_
