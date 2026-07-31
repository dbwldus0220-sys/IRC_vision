#ifndef IRC_STEP_MOTION_EXECUTOR__SDK_EXECUTOR_CORE_HPP_
#define IRC_STEP_MOTION_EXECUTOR__SDK_EXECUTOR_CORE_HPP_

#include "irc_step_motion_executor/catalog_only_core.hpp"
#include "irc_step_motion_executor/motion_backend.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace irc_step_motion_executor
{

class SdkExecutorCore
{
public:
  SdkExecutorCore(MotionAliasCatalog catalog, MotionBackend & backend);

  MotionStatus handle_request(
    const std::string & payload, std::uint64_t now_ms);
  MotionStatus handle_cancel(const std::string & payload);
  std::optional<MotionStatus> poll(std::uint64_t now_ms);
  bool has_active_request() const;

private:
  struct ActiveRequest
  {
    MotionRequest request;
    std::uint64_t started_at_ms{0};
    std::uint64_t timeout_ms{0};
  };

  MotionStatus status_for_active(
    const std::string & status, const std::string & error_code,
    const std::string & message) const;
  void clear_active();

  MotionAliasCatalog catalog_;
  MotionBackend & backend_;
  std::optional<ActiveRequest> active_;
};

}  // namespace irc_step_motion_executor

#endif  // IRC_STEP_MOTION_EXECUTOR__SDK_EXECUTOR_CORE_HPP_
