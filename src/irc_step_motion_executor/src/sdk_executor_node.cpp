#include "irc_step_motion_executor/sdk_executor_core.hpp"
#include "irc_step_motion_executor/sdk_executor_driver.hpp"
#include "irc_step_motion_executor/simulated_motion_backend.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace irc_step_motion_executor
{
namespace
{

constexpr std::int64_t kDefaultPollPeriodMs = 20;
constexpr std::int64_t kDefaultRunningPolls = 2;
constexpr std::int64_t kDefaultSettlingPolls = 1;

std::uint64_t steady_now_ms()
{
  return static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

}  // namespace

class SdkMotionExecutorNode : public rclcpp::Node
{
public:
  SdkMotionExecutorNode()
  : Node("sdk_motion_executor")
  {
    const std::string default_alias_path =
      ament_index_cpp::get_package_share_directory(
      "irc_step_motion_executor") + "/config/motion_aliases.yaml";
    const std::string alias_path = declare_parameter<std::string>(
      "motion_aliases_file", default_alias_path);
    const std::int64_t poll_period_ms = positive_parameter_or_default(
      "poll_period_ms", kDefaultPollPeriodMs);
    const std::int64_t running_polls = nonnegative_parameter_or_default(
      "running_polls", kDefaultRunningPolls);
    const std::int64_t settling_polls = nonnegative_parameter_or_default(
      "settling_polls", kDefaultSettlingPolls);

    MotionAliasCatalog catalog;
    std::string error_message;
    if (!catalog.load(alias_path, error_message)) {
      throw std::runtime_error(
              "failed to load motion alias catalog: " + error_message);
    }

    SimulatedMotionBackendConfig backend_config;
    backend_config.running_polls =
      static_cast<std::size_t>(running_polls);
    backend_config.settling_polls =
      static_cast<std::size_t>(settling_polls);
    backend_config.force_start_failure = declare_parameter<bool>(
      "force_start_failure", false);
    backend_config.force_backend_failure = declare_parameter<bool>(
      "force_backend_failure", false);

    backend_ = std::make_unique<SimulatedMotionBackend>(backend_config);
    core_ = std::make_unique<SdkExecutorCore>(
      std::move(catalog), *backend_);
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/motion/executor/status", 10);
    driver_ = std::make_unique<SdkExecutorDriver>(
      *core_, steady_now_ms,
      [this](const std::string & payload) {
        std_msgs::msg::String message;
        message.data = payload;
        status_publisher_->publish(message);
      });

    request_subscription_ = create_subscription<std_msgs::msg::String>(
      "/motion/executor/request", 10,
      [this](const std_msgs::msg::String::SharedPtr message) {
        driver_->handle_request(message->data);
      });
    cancel_subscription_ = create_subscription<std_msgs::msg::String>(
      "/motion/executor/cancel", 10,
      [this](const std_msgs::msg::String::SharedPtr message) {
        driver_->handle_cancel(message->data);
      });
    poll_timer_ = create_wall_timer(
      std::chrono::milliseconds(poll_period_ms),
      [this]() {driver_->poll();});

    RCLCPP_WARN(
      get_logger(),
      "Simulated backend only: no SDK or hardware access is available");
  }

private:
  std::int64_t positive_parameter_or_default(
    const std::string & name, std::int64_t default_value)
  {
    const std::int64_t value =
      declare_parameter<std::int64_t>(name, default_value);
    if (value >= 1) {
      return value;
    }
    RCLCPP_WARN(
      get_logger(), "%s must be at least 1; using %ld",
      name.c_str(), default_value);
    return default_value;
  }

  std::int64_t nonnegative_parameter_or_default(
    const std::string & name, std::int64_t default_value)
  {
    const std::int64_t value =
      declare_parameter<std::int64_t>(name, default_value);
    if (value >= 0) {
      return value;
    }
    RCLCPP_WARN(
      get_logger(), "%s must be nonnegative; using %ld",
      name.c_str(), default_value);
    return default_value;
  }

  std::unique_ptr<SimulatedMotionBackend> backend_;
  std::unique_ptr<SdkExecutorCore> core_;
  std::unique_ptr<SdkExecutorDriver> driver_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr request_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cancel_subscription_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
};

}  // namespace irc_step_motion_executor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<irc_step_motion_executor::SdkMotionExecutorNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("sdk_motion_executor"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
