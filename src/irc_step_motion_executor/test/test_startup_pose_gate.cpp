#include "fake_motion_backend.hpp"
#include "irc_step_motion_executor/sdk_executor_driver.hpp"
#include "irc_step_motion_executor/startup_pose_gate.hpp"

#include <gtest/gtest.h>

#include <deque>
#include <string>
#include <vector>

#ifndef TEST_ALIAS_CONFIG
#define TEST_ALIAS_CONFIG ""
#endif

namespace
{

class FakeStartupPoseController : public irc_step_motion_executor::StartupPoseController
{
public:
  bool start_result{true};
  int start_calls{0};
  std::string received_name;
  std::int64_t received_duration_ms{0};
  std::deque<irc_step_motion_executor::StartupPoseUpdate> updates;

  bool start(const std::vector<double> & angles, std::int64_t duration, std::string & error) override
  {
    ++start_calls;
    received_name = angles.size() == 23U ? "angles-0-22" : "invalid";
    received_duration_ms = duration;
    if (!start_result) {error = "write failed";}
    return start_result;
  }

  irc_step_motion_executor::StartupPoseUpdate update() override
  {
    if (updates.empty()) {
      return {irc_step_motion_executor::StartupPoseState::MOVING, "", ""};
    }
    auto value = updates.front();
    updates.pop_front();
    return value;
  }
};

irc_step_motion_executor::MotionAliasCatalog catalog()
{
  irc_step_motion_executor::MotionAliasCatalog value;
  std::string error;
  EXPECT_TRUE(value.load(TEST_ALIAS_CONFIG, error)) << error;
  return value;
}

std::string request()
{
  return R"({"action":"STRAIGHT","command_id":1,"event_id":2,"request_id":3,"motion_id":"forward","timeout_ms":5000})";
}

struct Fixture
{
  FakeMotionBackend backend;
  irc_step_motion_executor::SdkExecutorCore core{catalog(), backend};
  FakeStartupPoseController startup;
  irc_step_motion_executor::StartupPoseGate gate{
    true, "오뒤307", std::vector<double>(23, 0.0), 1800, &startup};
  std::vector<std::string> statuses;
  irc_step_motion_executor::SdkExecutorDriver driver{
    core, []() {return 100U;},
    [this](const std::string & value) {statuses.push_back(value);}, &gate};
};

TEST(StartupPoseGate, DoesNotStartBeforeFirstReadyPollAndStartsExactlyOnce)
{
  Fixture fixture;
  EXPECT_EQ(fixture.startup.start_calls, 0);
  fixture.driver.poll();
  fixture.driver.poll();
  EXPECT_EQ(fixture.startup.start_calls, 1);
  EXPECT_EQ(fixture.startup.received_name, "angles-0-22");
  EXPECT_EQ(fixture.startup.received_duration_ms, 1800);
}

TEST(StartupPoseGate, BlocksNavigationUntilSuccessThenAllowsIt)
{
  Fixture fixture;
  fixture.driver.handle_request(request());
  EXPECT_TRUE(fixture.backend.started_motion_names.empty());
  fixture.startup.updates.push_back(
    {irc_step_motion_executor::StartupPoseState::SETTLING, "", ""});
  fixture.startup.updates.push_back(
    {irc_step_motion_executor::StartupPoseState::SUCCEEDED, "", ""});
  fixture.driver.poll();
  fixture.driver.poll();
  fixture.driver.poll();
  fixture.driver.handle_request(request());
  ASSERT_EQ(fixture.backend.started_motion_names.size(), 1U);
}

TEST(StartupPoseGate, FailurePermanentlyKeepsNavigationBlocked)
{
  Fixture fixture;
  fixture.startup.start_result = false;
  fixture.driver.poll();
  fixture.driver.poll();
  fixture.driver.handle_request(request());
  EXPECT_EQ(fixture.startup.start_calls, 1);
  EXPECT_TRUE(fixture.backend.started_motion_names.empty());
  EXPECT_EQ(fixture.gate.state(), irc_step_motion_executor::StartupPoseGate::State::ERROR);
}

TEST(StartupPoseGate, DisabledGateNeverRunsAndAllowsSimulatedNavigation)
{
  FakeStartupPoseController startup;
  irc_step_motion_executor::StartupPoseGate gate{
    false, "오뒤307", {}, 1800, &startup};
  gate.poll();
  EXPECT_EQ(startup.start_calls, 0);
  EXPECT_TRUE(gate.navigation_allowed());
}

}  // namespace
