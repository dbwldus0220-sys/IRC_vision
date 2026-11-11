#include "fake_motion_backend.hpp"
#include "irc_step_motion_executor/sdk_executor_core.hpp"
#include <gtest/gtest.h>

namespace {
class RingBackend : public FakeMotionBackend {
public:
  double received_angle{0};
  int ring_starts{0};
  irc_step_motion_executor::BackendStartResult start_ring_shot(
    const std::string & name, double angle) override
  {
    received_angle = angle;
    ++ring_starts;
    return start_motion(name);
  }
};

irc_step_motion_executor::MotionAliasCatalog catalog() {
  irc_step_motion_executor::MotionAliasCatalog result;
  std::string error;
  EXPECT_TRUE(result.load(TEST_ALIAS_CONFIG, error)) << error;
  return result;
}

std::string request(const std::string & motion, const std::string & angle = "") {
  return "{\"action\":\"SHOT\",\"command_id\":1,\"event_id\":2,\"request_id\":3,"
         "\"timeout_ms\":5000,\"motion_id\":\"" + motion + "\"" +
         (angle.empty() ? "" : ",\"waist_ring_aim_deg\":" + angle) + "}";
}

TEST(RingShotCore, FiveRegionsReceiveExactSignedAngle) {
  for (int region : {1, 2, 8, 5, 6}) {
    RingBackend backend;
    irc_step_motion_executor::SdkExecutorCore core(catalog(), backend);
    ASSERT_EQ(core.handle_request(request("goal_shot_region_" + std::to_string(region), "-12.345"), 0).status, "RUNNING");
    EXPECT_DOUBLE_EQ(backend.received_angle, -12.345);
    EXPECT_EQ(backend.ring_starts, 1);
    EXPECT_EQ(core.handle_request(request("goal_shot_region_6", "20"), 1).status, "REJECTED");
    EXPECT_EQ(backend.ring_starts, 1);
  }
}

TEST(RingShotCore, RejectsDisabledMissingInvalidAndForeignAngle) {
  for (const auto & payload : {
      request("goal_shot_region_3", "0"), request("goal_shot_region_4", "0"),
      request("goal_shot_region_7", "0"), request("goal_shot_region_7"),
      request("goal_shot"), request("goal_shot_region_6"),
      request("goal_shot_region_6", "true"), request("goal_shot_region_6", "null"),
      request("goal_shot_region_6", "NaN"), request("goal_shot_region_6", "90"),
      request("goal_shot_region_6", "-90"), request("forward", "20")}) {
    RingBackend backend;
    irc_step_motion_executor::SdkExecutorCore core(catalog(), backend);
    EXPECT_EQ(core.handle_request(payload, 0).status, "REJECTED");
    EXPECT_TRUE(backend.started_motion_names.empty());
  }
}

TEST(RingShotCore, CannotQueueShotAndTerminalStatusAllowsFreshShot) {
  RingBackend backend;
  backend.queue_result = {true, "", "accepted"};
  irc_step_motion_executor::SdkExecutorCore core(catalog(), backend);
  ASSERT_EQ(core.handle_request(request("forward"), 0).status, "RUNNING");
  EXPECT_EQ(core.handle_request(request("goal_shot_region_6", "10"), 1).status, "REJECTED");
  EXPECT_TRUE(backend.queued_motion_names.empty());
  backend.statuses.push_back({irc_step_motion_executor::BackendState::SUCCEEDED, "", ""});
  ASSERT_EQ(core.poll(2)->status, "SUCCEEDED");
  ASSERT_EQ(core.handle_request(request("goal_shot_region_6", "10"), 3).status, "RUNNING");
  EXPECT_EQ(core.handle_request(request("forward"), 4).status, "REJECTED");
  backend.statuses.push_back({irc_step_motion_executor::BackendState::FAILED, "FAIL", ""});
  ASSERT_EQ(core.poll(5)->status, "FAILED");
  ASSERT_EQ(core.handle_request(request("goal_shot_region_1", "-5"), 6).status, "RUNNING");
  EXPECT_DOUBLE_EQ(backend.received_angle, -5);
}

TEST(RingShotCore, UnsupportedBackendDoesNotSilentlyDropAngle) {
  FakeMotionBackend backend;
  irc_step_motion_executor::SdkExecutorCore core(catalog(), backend);
  EXPECT_EQ(core.handle_request(request("goal_shot_region_6", "10"), 0).error_code, "RING_AIM_UNSUPPORTED");
  EXPECT_TRUE(backend.started_motion_names.empty());
}
}  // namespace
