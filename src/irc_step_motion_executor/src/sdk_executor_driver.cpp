#include "irc_step_motion_executor/sdk_executor_driver.hpp"

#include "irc_step_motion_executor/catalog_only_core.hpp"

#include <utility>

namespace irc_step_motion_executor
{

SdkExecutorDriver::SdkExecutorDriver(
  SdkExecutorCore & core, NowProvider now_provider,
  StatusPublisher status_publisher)
: core_(core),
  now_provider_(std::move(now_provider)),
  status_publisher_(std::move(status_publisher))
{
}

void SdkExecutorDriver::handle_request(const std::string & payload)
{
  publish(core_.handle_request(payload, now_provider_()));
}

void SdkExecutorDriver::handle_cancel(const std::string & payload)
{
  publish(core_.handle_cancel(payload));
}

void SdkExecutorDriver::poll()
{
  const auto status = core_.poll(now_provider_());
  if (status) {
    publish(*status);
  }
}

void SdkExecutorDriver::publish(const MotionStatus & status)
{
  status_publisher_(CatalogOnlyCore::to_json(status));
}

}  // namespace irc_step_motion_executor
