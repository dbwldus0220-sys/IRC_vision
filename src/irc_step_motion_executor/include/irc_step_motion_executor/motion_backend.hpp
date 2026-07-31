#ifndef IRC_STEP_MOTION_EXECUTOR__MOTION_BACKEND_HPP_
#define IRC_STEP_MOTION_EXECUTOR__MOTION_BACKEND_HPP_

#include <string>

namespace irc_step_motion_executor
{

enum class BackendState
{
  IDLE,
  RUNNING,
  SETTLING,
  SUCCEEDED,
  CANCELLED,
  FAILED,
};

struct BackendStartResult
{
  bool accepted{false};
  std::string error_code;
  std::string message;
};

struct BackendCancelResult
{
  bool accepted{false};
  std::string error_code;
  std::string message;
};

struct BackendStatus
{
  BackendState state{BackendState::IDLE};
  std::string error_code;
  std::string message;
};

class MotionBackend
{
public:
  virtual ~MotionBackend() = default;

  virtual BackendStartResult start_motion(
    const std::string & resolved_motion_name) = 0;
  virtual BackendCancelResult cancel_motion() = 0;
  virtual BackendStatus poll_status() = 0;
};

}  // namespace irc_step_motion_executor

#endif  // IRC_STEP_MOTION_EXECUTOR__MOTION_BACKEND_HPP_
