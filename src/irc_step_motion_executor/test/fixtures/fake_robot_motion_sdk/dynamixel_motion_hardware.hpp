#ifndef FAKE_DYNAMIXEL_MOTION_HARDWARE_HPP_
#define FAKE_DYNAMIXEL_MOTION_HARDWARE_HPP_

#include "motion_hardware.hpp"

namespace irc_step
{

class DynamixelMotionHardware final : public IMotionHardware
{
public:
  DynamixelMotionHardware();
  ~DynamixelMotionHardware() override;

  bool initialize() noexcept override;
  bool ready() const noexcept override;
};

}  // namespace irc_step

#endif  // FAKE_DYNAMIXEL_MOTION_HARDWARE_HPP_
