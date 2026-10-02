#include "fake_motion_backend.hpp"
#include "irc_step_motion_executor/sdk_executor_core.hpp"

#include <gtest/gtest.h>

namespace
{
using namespace irc_step_motion_executor;

MotionAliasCatalog catalog()
{
  MotionAliasCatalog result;
  std::string error;
  EXPECT_TRUE(result.load(TEST_ALIAS_CONFIG, error)) << error;
  return result;
}

std::string request(int id)
{
  return "{\"action\":\"STRAIGHT\",\"command_id\":17,\"event_id\":null,"
         "\"request_id\":" + std::to_string(id) +
         ",\"motion_id\":\"line_forward_4\",\"timeout_ms\":5000}";
}

TEST(QueueControlTick, ActivationStatusDoesNotSkipBackendUpdate)
{
  FakeMotionBackend backend;
  backend.queue_result = {true, "", "queued"};
  SdkExecutorCore core(catalog(), backend);
  ASSERT_EQ(core.handle_request(request(1), 0).status, "RUNNING");
  ASSERT_EQ(core.handle_request(request(2), 1).status, "QUEUED");
  backend.sequence = 1;
  ASSERT_EQ(core.poll(5)->status, "SUCCEEDED");
  EXPECT_EQ(backend.poll_calls, 1);
  const auto activated = core.poll(10);
  ASSERT_TRUE(activated);
  EXPECT_EQ(activated->status, "RUNNING");
  EXPECT_EQ(activated->request_id, 2);
  EXPECT_EQ(backend.poll_calls, 2);
}

TEST(QueueControlTick, FailureOnActivationTickIsReportedImmediately)
{
  FakeMotionBackend backend;
  backend.queue_result = {true, "", "queued"};
  SdkExecutorCore core(catalog(), backend);
  ASSERT_EQ(core.handle_request(request(1), 0).status, "RUNNING");
  ASSERT_EQ(core.handle_request(request(2), 1).status, "QUEUED");
  backend.sequence = 1;
  ASSERT_EQ(core.poll(5)->status, "SUCCEEDED");
  backend.statuses.push_back({BackendState::FAILED, "FRAME_SEND_FAILED", "write failed"});
  const auto failed = core.poll(10);
  ASSERT_TRUE(failed);
  EXPECT_EQ(failed->status, "FAILED");
  EXPECT_EQ(failed->request_id, 2);
  EXPECT_FALSE(core.has_active_request());
}
}  // namespace
