#ifndef FAKE_ROBOT_MOTION_PLAYER_HPP_
#define FAKE_ROBOT_MOTION_PLAYER_HPP_

#include "motion_hardware.hpp"

#include <cstdint>
#include <string>
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
  RobotMotionPlayer() = default;
  RobotMotionPlayer(const std::string & json_path, IMotionHardware & hardware);
  ~RobotMotionPlayer();

  bool initialize() noexcept;
  StartResult start(std::string_view motion_name) noexcept;
  CancelResult cancel() noexcept;
  MotionStatus update() noexcept;
  MotionError result() const noexcept;
  std::string_view lastError() const noexcept;

private:
  IMotionHardware * hardware_{nullptr};
  bool initialized_{false};
  std::string last_error_;
};

}  // namespace irc_step

#endif  // FAKE_ROBOT_MOTION_PLAYER_HPP_
