#include "robot_motion_player.hpp"

namespace irc_step
{

StartResult RobotMotionPlayer::start(std::string_view) noexcept
{
  return StartResult::Accepted;
}

CancelResult RobotMotionPlayer::cancel() noexcept
{
  return CancelResult::Cancelled;
}

MotionStatus RobotMotionPlayer::update() noexcept
{
  return MotionStatus::Idle;
}

MotionError RobotMotionPlayer::result() const noexcept
{
  return MotionError::None;
}

std::string_view RobotMotionPlayer::lastError() const noexcept
{
  return {};
}

}  // namespace irc_step
