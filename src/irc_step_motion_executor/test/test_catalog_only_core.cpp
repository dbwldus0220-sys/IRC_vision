#include "irc_step_motion_executor/catalog_only_core.hpp"

#include <gtest/gtest.h>
#include <json-c/json.h>

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#ifndef TEST_ALIAS_CONFIG
#define TEST_ALIAS_CONFIG ""
#endif

namespace
{

irc_step_motion_executor::CatalogOnlyCore make_core()
{
  irc_step_motion_executor::MotionAliasCatalog catalog;
  std::string error;
  EXPECT_TRUE(catalog.load(TEST_ALIAS_CONFIG, error)) << error;
  return irc_step_motion_executor::CatalogOnlyCore(std::move(catalog));
}

TEST(MotionAliasCatalog, LoadsOnlyCandidateAliases)
{
  irc_step_motion_executor::MotionAliasCatalog catalog;
  std::string error;
  ASSERT_TRUE(catalog.load(TEST_ALIAS_CONFIG, error)) << error;
  EXPECT_EQ(catalog.size(), 2U);
  EXPECT_EQ(catalog.resolve("forward"), std::optional<std::string>("전진"));
  EXPECT_EQ(
    catalog.resolve("forward_short"), std::optional<std::string>("첫발"));
  EXPECT_FALSE(catalog.resolve("turn_left").has_value());
  EXPECT_FALSE(catalog.resolve("shoot").has_value());
}

TEST(CatalogOnlyCore, PreservesCorrelationFieldsAndRejectsExecution)
{
  const auto core = make_core();
  const auto status = core.handle_request(
    R"({"action":"STRAIGHT","command_id":17,"event_id":29,)"
    R"("request_id":41,"motion_id":"forward","timeout_ms":5000})");

  EXPECT_EQ(status.status, "REJECTED");
  EXPECT_EQ(status.action, std::optional<std::string>("STRAIGHT"));
  EXPECT_EQ(status.command_id, std::optional<std::int64_t>(17));
  EXPECT_EQ(status.event_id, std::optional<std::int64_t>(29));
  EXPECT_EQ(status.request_id, 41);
  EXPECT_EQ(status.motion_id, "forward");
  EXPECT_EQ(status.error_code, "HARDWARE_NOT_READY");
  EXPECT_NE(status.message.find("catalog-only mode"), std::string::npos);
}

TEST(CatalogOnlyCore, RejectsInvalidJsonSafely)
{
  const auto status = make_core().handle_request("{not-json");
  EXPECT_EQ(status.status, "REJECTED");
  EXPECT_EQ(status.error_code, "INVALID_REQUEST");
}

TEST(CatalogOnlyCore, RejectsUnknownAliasWithoutFallback)
{
  const auto status = make_core().handle_request(
    R"({"action":"TURN_LEFT","command_id":1,"event_id":2,)"
    R"("request_id":3,"motion_id":"turn_left"})");
  EXPECT_EQ(status.status, "REJECTED");
  EXPECT_EQ(status.motion_id, "turn_left");
  EXPECT_EQ(status.error_code, "INVALID_MOTION");
  EXPECT_NE(status.message.find("no fallback"), std::string::npos);
}

TEST(CatalogOnlyCore, SerializedStatusContainsContractFields)
{
  const auto status = make_core().handle_request(
    R"({"action":"STEP","command_id":4,"event_id":5,)"
    R"("request_id":6,"motion_id":"forward_short"})");
  const std::string payload =
    irc_step_motion_executor::CatalogOnlyCore::to_json(status);
  json_object * object = json_tokener_parse(payload.c_str());
  ASSERT_NE(object, nullptr);
  for (const char * field : {
      "status", "action", "command_id", "event_id", "request_id",
      "motion_id", "error_code", "message"})
  {
    json_object * value = nullptr;
    EXPECT_TRUE(json_object_object_get_ex(object, field, &value)) << field;
  }
  json_object_put(object);
}

TEST(CatalogOnlyCore, CancelNeverTouchesHardware)
{
  const auto status = make_core().handle_cancel(R"({"request_id":88})");
  EXPECT_EQ(status.status, "REJECTED");
  EXPECT_EQ(status.request_id, 88);
  EXPECT_EQ(status.error_code, "NOT_RUNNING");
  EXPECT_NE(status.message.find("catalog-only mode"), std::string::npos);
}

}  // namespace
