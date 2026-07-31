#ifndef IRC_STEP_MOTION_EXECUTOR__SDK_EXECUTOR_DRIVER_HPP_
#define IRC_STEP_MOTION_EXECUTOR__SDK_EXECUTOR_DRIVER_HPP_

#include "irc_step_motion_executor/sdk_executor_core.hpp"

#include <cstdint>
#include <functional>
#include <string>

namespace irc_step_motion_executor
{

class SdkExecutorDriver
{
public:
  using NowProvider = std::function<std::uint64_t()>;
  using StatusPublisher = std::function<void(const std::string &)>;

  SdkExecutorDriver(
    SdkExecutorCore & core, NowProvider now_provider,
    StatusPublisher status_publisher);

  void handle_request(const std::string & payload);
  void handle_cancel(const std::string & payload);
  void poll();

private:
  void publish(const MotionStatus & status);

  SdkExecutorCore & core_;
  NowProvider now_provider_;
  StatusPublisher status_publisher_;
};

}  // namespace irc_step_motion_executor

#endif  // IRC_STEP_MOTION_EXECUTOR__SDK_EXECUTOR_DRIVER_HPP_
