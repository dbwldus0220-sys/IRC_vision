#include "robot_motion_player.hpp"

#include "dynamixel_motion_hardware.hpp"
#include "fake_robot_motion_sdk_test_support.hpp"

#include <string>
#include <stdexcept>
#include <utility>
#include <vector>

namespace irc_step
{
namespace
{

int hardware_constructions = 0;
int hardware_initializations = 0;
int player_constructions = 0;
int player_initializations = 0;
bool player_constructor_throws = false;
bool player_initialize_success = true;
std::string player_initialize_error;
bool hardware_initialized = false;
DynamixelMotionHardwareConfig received_hardware_config;
std::vector<std::string> destructions;

}  // namespace

DynamixelMotionHardware::DynamixelMotionHardware()
  : DynamixelMotionHardware(DynamixelMotionHardwareConfig{})
{
}

DynamixelMotionHardware::DynamixelMotionHardware(
  DynamixelMotionHardwareConfig config)
{
  received_hardware_config = std::move(config);
  ++hardware_constructions;
}

DynamixelMotionHardware::~DynamixelMotionHardware()
{
  destructions.emplace_back("hardware");
}

bool DynamixelMotionHardware::initialize() noexcept
{
  ++hardware_initializations;
  hardware_initialized = true;
  return true;
}

bool DynamixelMotionHardware::ready() const noexcept
{
  return hardware_initialized;
}

RobotMotionPlayer::RobotMotionPlayer(
  const std::string &, IMotionHardware & hardware)
: hardware_(&hardware)
{
  if (player_constructor_throws) {
    throw std::runtime_error("fake player construction failed");
  }
  ++player_constructions;
}

RobotMotionPlayer::~RobotMotionPlayer()
{
  destructions.emplace_back("player");
}

bool RobotMotionPlayer::initialize() noexcept
{
  ++player_initializations;
  const bool hardware_success =
    hardware_ != nullptr && hardware_->initialize();
  initialized_ = hardware_success && player_initialize_success;
  if (!initialized_) {
    last_error_ = player_initialize_error.empty() ?
      "fake player initialization failed" : player_initialize_error;
  } else {
    last_error_.clear();
  }
  return initialized_;
}

StartResult RobotMotionPlayer::start(std::string_view) noexcept
{
  if (!initialized_ || hardware_ == nullptr || !hardware_->ready()) {
    last_error_ = "motion hardware is not ready";
    return StartResult::HardwareNotReady;
  }
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
  return last_error_;
}

namespace fake_sdk
{

void reset_tracking()
{
  hardware_constructions = 0;
  hardware_initializations = 0;
  player_constructions = 0;
  player_initializations = 0;
  player_constructor_throws = false;
  player_initialize_success = true;
  player_initialize_error.clear();
  hardware_initialized = false;
  received_hardware_config = {};
  destructions.clear();
}

void set_player_constructor_throws(bool value)
{
  player_constructor_throws = value;
}

void set_player_initialize_result(bool success, std::string error_message)
{
  player_initialize_success = success;
  player_initialize_error = std::move(error_message);
}

int hardware_construction_count() {return hardware_constructions;}
int hardware_initialize_count() {return hardware_initializations;}
int player_construction_count() {return player_constructions;}
int player_initialize_count() {return player_initializations;}
const std::string & hardware_device_path()
{
  return received_hardware_config.device_path;
}
std::int64_t hardware_baud_rate()
{
  return received_hardware_config.baud_rate;
}
const std::vector<int> & hardware_motor_ids()
{
  return received_hardware_config.motor_ids;
}
const std::vector<std::string> & destruction_order() {return destructions;}

}  // namespace fake_sdk

}  // namespace irc_step
