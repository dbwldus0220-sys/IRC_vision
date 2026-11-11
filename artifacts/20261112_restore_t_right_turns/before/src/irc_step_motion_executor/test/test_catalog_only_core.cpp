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

#ifndef TEST_CONTRACT_VECTORS
#define TEST_CONTRACT_VECTORS ""
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

json_object * load_contract_vectors()
{
  json_object * vectors = json_object_from_file(TEST_CONTRACT_VECTORS);
  EXPECT_NE(vectors, nullptr);
  if (vectors != nullptr) {
    EXPECT_EQ(json_object_get_type(vectors), json_type_array);
  }
  return vectors;
}

TEST(MotionAliasCatalog, LoadsLatestSdkAndCanonicalAliases)
{
  irc_step_motion_executor::MotionAliasCatalog catalog;
  std::string error;
  ASSERT_TRUE(catalog.load(TEST_ALIAS_CONFIG, error)) << error;
  EXPECT_EQ(catalog.size(), 175U);
  for (const auto & [alias, motion] : {
      std::pair{"post_shot_exit_right_9_recovery_right_4", "m골대후제자리우회전+라인복귀우회전45도(4회)"},
      std::pair{"post_shot_exit_left_4_recovery_left_4", "p골대후제자리좌회전+라인복귀좌회전45도(4회)"},
      std::pair{"post_shot_exit_right_5_forward_4", "m골대후제자리우회전(6회)+라인복귀좌회전45도(4회)+라인복귀좌회전45도(4회)"},
      std::pair{"post_shot_exit_left_3_forward_4", "p골대후제자리좌회전(4회)+라인복귀우회전45도(4회)+라인복귀우회전45도(4회)"},
      std::pair{"forward", "m전진45(8회)"},
      std::pair{"line_forward_8", "m전진45(8회)"},
      std::pair{"goal_camera_90_forward_2", "m전진90도(2회)"},
      std::pair{"goal_shot_region_5", "r골넣기 5번구역"},
      std::pair{"goal_shot_region_6", "r골넣기 6번구역"},
      std::pair{"goal_shot_region_8", "r골넣기8번구역"},
      std::pair{"line_turn_right_large", "t제자리우회전45도(9회)"},
      std::pair{"line_recovery_left_4", "m라인복귀좌회전45도"},
      std::pair{"line_recovery_left_6", "m라인복귀좌회전45도"},
      std::pair{"line_recovery_right_4", "m라인복귀우회전45도"},
      std::pair{"pickup_crab_right_0", "t미세오옆꽃게(0도)-1회"},
      std::pair{"pickup_crab_left_0", "tt미세왼옆0도"},
      std::pair{"pickup_fine_prepare", "p오뒤에서 미세오뒤(45도)"},
      std::pair{"pickup_fine_prepare_0", "p오뒤에서 미세오뒤(0도)"},
      std::pair{"goal_fine_prepare_90", "p오뒤에서 미세오뒤90도"},
      std::pair{"goal_crab_right_to_fine_90", "r오옆에서 미세오뒤(90도)"},
      std::pair{"goal_crab_left_to_fine_90", "r왼옆에서 미세오뒤(90도)"},
      std::pair{"hurdle_fine_prepare_0", "p오뒤에서 미세오뒤(0도)"},
      std::pair{"pickup_fine_to_crab_left_0", "r미세오뒤에서 오옆0도"},
      std::pair{"goal_crab_right_to_turn_90", "r오옆에서 오뒤90도"},
      std::pair{"fine_to_turn_ready_45", "p미세오뒤에서 오뒤45도"},
      std::pair{"goal_camera_90_crab_right", "t미세오옆꽃게90도(1회)"},
      std::pair{"goal_camera_90_crab_left", "tt미세왼옆90도"},
      std::pair{"stationary_turn_left", "p제자리좌회전45(6회)"},
      std::pair{"stationary_turn_right", "t제자리우회전45도(9회)"},
      std::pair{"pickup_camera_down_turn_left_6", "p제자리좌회전0도(6회)"},
      std::pair{"pickup_camera_down_turn_right_9", "t제자리우회전0도(9회)"},
      std::pair{"pickup", "r공잡기 400"},
      std::pair{"pickup_400", "r공잡기 400"},
      std::pair{"pickup_500", "r공잡기500"}})
  {
    EXPECT_EQ(catalog.resolve(alias), std::optional<std::string>(motion));
  }
  EXPECT_FALSE(catalog.resolve("line_forward_10").has_value());
  EXPECT_FALSE(catalog.resolve("forward_short").has_value());
  EXPECT_FALSE(catalog.resolve("turn_left").has_value());
  EXPECT_FALSE(catalog.resolve("fine_left").has_value());
  EXPECT_FALSE(catalog.resolve("align_left").has_value());
  EXPECT_FALSE(catalog.resolve("shoot").has_value());
}

TEST(MotionAliasCatalog, LoadsAllSource77LeftTurnCamerasAndCounts)
{
  irc_step_motion_executor::MotionAliasCatalog catalog;
  std::string error;
  ASSERT_TRUE(catalog.load(TEST_ALIAS_CONFIG, error)) << error;
  for (const auto & [prefix, camera] : {
      std::pair{"pickup_camera_down", "0"},
      std::pair{"post_ball_line", "45"},
      std::pair{"ball_align", "45"},
      std::pair{"goal_camera_90", "90"}})
  {
    for (int count = 1; count <= 6; ++count) {
      const auto suffix = std::to_string(count);
      EXPECT_EQ(catalog.resolve(std::string(prefix) + "_turn_left_" + suffix),
        std::optional<std::string>(std::string("p제자리좌회전") + camera + (std::string(camera) == "45" ? "(" : "도(") + suffix + "회)"));
    }
  }
  EXPECT_EQ(catalog.resolve("stationary_turn_left"),
    std::optional<std::string>("p제자리좌회전45(6회)"));
  EXPECT_EQ(catalog.resolve("line_search_left_2"),
    std::optional<std::string>("p제자리좌회전45(2회)"));
}

TEST(MotionAliasCatalog, Source76CrabsShareEntryAndRemoveDirectionTransitions)
{
  irc_step_motion_executor::MotionAliasCatalog catalog;
  std::string error;
  ASSERT_TRUE(catalog.load(TEST_ALIAS_CONFIG, error)) << error;
  for (const auto & [alias, motion] : {
      std::pair{"pickup_crab_left_0", "tt미세왼옆0도"},
      std::pair{"pickup_crab_left_0_2", "tt미세왼옆0도(2회)"},
      std::pair{"goal_camera_90_crab_left", "tt미세왼옆90도"},
      std::pair{"pickup_fine_to_crab_left_0", "r미세오뒤에서 오옆0도"},
      std::pair{"pickup_forward_to_crab_left_0", "r오뒤에서 오옆꽃게0도"},
      std::pair{"goal_fine_to_crab_left_90", "r미세오뒤에서 오옆 90도"},
      std::pair{"goal_forward_to_crab_left_90", "r오뒤에서 오옆꽃게90도"}})
  {
    EXPECT_EQ(catalog.resolve(alias), std::optional<std::string>(motion));
  }
  for (const auto * alias : {"pickup_crab_right_to_left_0", "pickup_crab_left_to_right_0",
      "goal_crab_right_to_left_90", "goal_crab_left_to_right_90"})
  {
    EXPECT_FALSE(catalog.resolve(alias).has_value());
  }
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
  EXPECT_FALSE(status.action.has_value());
  EXPECT_FALSE(status.command_id.has_value());
  EXPECT_FALSE(status.event_id.has_value());
  EXPECT_EQ(status.request_id, 0);
  EXPECT_TRUE(status.motion_id.empty());
}

TEST(CatalogOnlyCore, DistinguishesRequiredNullEventIdFromMissingEventId)
{
  const auto core = make_core();
  const auto null_status = core.handle_request(
    R"({"action":"STRAIGHT","command_id":17,"event_id":null,)"
    R"("request_id":41,"motion_id":"forward"})");
  EXPECT_EQ(null_status.error_code, "HARDWARE_NOT_READY");
  EXPECT_FALSE(null_status.event_id.has_value());

  const auto missing_status = core.handle_request(
    R"({"action":"STRAIGHT","command_id":17,)"
    R"("request_id":41,"motion_id":"forward"})");
  EXPECT_EQ(missing_status.status, "REJECTED");
  EXPECT_EQ(missing_status.error_code, "INVALID_REQUEST");
}

TEST(CatalogOnlyCore, RequiresEveryRequestContractField)
{
  const auto core = make_core();
  for (const std::string payload : {
      R"({"command_id":1,"event_id":2,"request_id":3,"motion_id":"forward"})",
      R"({"action":"STRAIGHT","event_id":2,"request_id":3,"motion_id":"forward"})",
      R"({"action":"STRAIGHT","command_id":1,"request_id":3,"motion_id":"forward"})",
      R"({"action":"STRAIGHT","command_id":1,"event_id":2,"motion_id":"forward"})",
      R"({"action":"STRAIGHT","command_id":1,"event_id":2,"request_id":3})"})
  {
    EXPECT_EQ(core.handle_request(payload).error_code, "INVALID_REQUEST");
  }
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
    R"("request_id":6,"motion_id":"pickup"})");
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
  json_object * value = nullptr;
  ASSERT_TRUE(json_object_object_get_ex(object, "status", &value));
  EXPECT_EQ(json_object_get_type(value), json_type_string);
  ASSERT_TRUE(json_object_object_get_ex(object, "action", &value));
  EXPECT_EQ(json_object_get_type(value), json_type_string);
  ASSERT_TRUE(json_object_object_get_ex(object, "command_id", &value));
  EXPECT_EQ(json_object_get_type(value), json_type_int);
  ASSERT_TRUE(json_object_object_get_ex(object, "event_id", &value));
  EXPECT_EQ(json_object_get_type(value), json_type_int);
  ASSERT_TRUE(json_object_object_get_ex(object, "request_id", &value));
  EXPECT_EQ(json_object_get_type(value), json_type_int);
  ASSERT_TRUE(json_object_object_get_ex(object, "motion_id", &value));
  EXPECT_EQ(json_object_get_type(value), json_type_string);
  ASSERT_TRUE(json_object_object_get_ex(object, "error_code", &value));
  EXPECT_EQ(json_object_get_type(value), json_type_string);
  ASSERT_TRUE(json_object_object_get_ex(object, "message", &value));
  EXPECT_EQ(json_object_get_type(value), json_type_string);
  json_object_put(object);
}

TEST(CatalogOnlyCore, SerializedStatusPreservesNullCorrelationTypes)
{
  const auto status = make_core().handle_request(
    R"({"action":"PICKUP_NOW","command_id":null,"event_id":null,)"
    R"("request_id":7,"motion_id":"pickup"})");
  const std::string payload =
    irc_step_motion_executor::CatalogOnlyCore::to_json(status);
  json_object * object = json_tokener_parse(payload.c_str());
  ASSERT_NE(object, nullptr);
  json_object * value = nullptr;
  ASSERT_TRUE(json_object_object_get_ex(object, "command_id", &value));
  EXPECT_EQ(json_object_get_type(value), json_type_null);
  ASSERT_TRUE(json_object_object_get_ex(object, "event_id", &value));
  EXPECT_EQ(json_object_get_type(value), json_type_null);
  json_object_put(object);
}

TEST(CatalogOnlyContract, SharedVectorsPreserveFieldsWithoutFallback)
{
  json_object * vectors = load_contract_vectors();
  ASSERT_NE(vectors, nullptr);
  const auto core = make_core();

  const std::size_t count = json_object_array_length(vectors);
  for (std::size_t index = 0; index < count; ++index) {
    json_object * vector = json_object_array_get_idx(vectors, index);
    json_object * request = nullptr;
    json_object * expected_error = nullptr;
    ASSERT_TRUE(json_object_object_get_ex(vector, "request", &request));
    ASSERT_TRUE(
      json_object_object_get_ex(
        vector, "expected_error_code", &expected_error));

    const auto status = core.handle_request(
      json_object_to_json_string_ext(request, JSON_C_TO_STRING_PLAIN));
    EXPECT_EQ(status.status, "REJECTED");
    EXPECT_EQ(status.error_code, json_object_get_string(expected_error));

    json_object * value = nullptr;
    ASSERT_TRUE(json_object_object_get_ex(request, "action", &value));
    ASSERT_TRUE(status.action.has_value());
    EXPECT_EQ(*status.action, json_object_get_string(value));
    ASSERT_TRUE(json_object_object_get_ex(request, "request_id", &value));
    EXPECT_EQ(status.request_id, json_object_get_int64(value));
    ASSERT_TRUE(json_object_object_get_ex(request, "motion_id", &value));
    EXPECT_EQ(status.motion_id, json_object_get_string(value));

    ASSERT_TRUE(json_object_object_get_ex(request, "command_id", &value));
    if (json_object_get_type(value) == json_type_null) {
      EXPECT_FALSE(status.command_id.has_value());
    } else {
      ASSERT_TRUE(status.command_id.has_value());
      EXPECT_EQ(*status.command_id, json_object_get_int64(value));
    }
    ASSERT_TRUE(json_object_object_get_ex(request, "event_id", &value));
    if (json_object_get_type(value) == json_type_null) {
      EXPECT_FALSE(status.event_id.has_value());
    } else {
      ASSERT_TRUE(status.event_id.has_value());
      EXPECT_EQ(*status.event_id, json_object_get_int64(value));
    }
  }
  json_object_put(vectors);
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
