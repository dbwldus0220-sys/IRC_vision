#ifndef FAKE_ROBOT_MOTION_PLAYER_HPP_
#define FAKE_ROBOT_MOTION_PLAYER_HPP_

#include <cstdint>
#include <string_view>

namespace irc_step
{

enum class MotionStatus : std::uint8_t
{
  Idle,
  Running,
  Settling,
  Succeeded,
  Cancelled,
  Failed,
};

enum class StartResult : std::uint8_t
{
  Accepted,
  RejectedBusy,
  MotionNotFound,
  HardwareNotReady,
  InvalidMotion,
};

enum class CancelResult : std::uint8_t
{
  Cancelled,
  NotRunning,
  HardwareNotReady,
  HoldFailed,
};

enum class MotionError : std::uint8_t
{
  None,
  JsonError,
  HardwareNotReady,
  CommunicationError,
  FrameSendFailed,
  PresentPositionReadFailed,
  PositionTimeout,
  CancelFailed,
  InternalError,
};

class RobotMotionPlayer
{
public:
  StartResult start(std::string_view motion_name) noexcept;
  CancelResult cancel() noexcept;
  MotionStatus update() noexcept;
  MotionError result() const noexcept;
  std::string_view lastError() const noexcept;
};

}  // namespace irc_step

#endif  // FAKE_ROBOT_MOTION_PLAYER_HPP_
