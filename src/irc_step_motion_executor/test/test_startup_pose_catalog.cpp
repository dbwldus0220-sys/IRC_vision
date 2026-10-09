#include "irc_step_motion_executor/startup_pose_catalog.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace
{

std::string angles_json(int missing_id = -1, int changed_id = -1)
{
  std::string output = "{";
  bool first = true;
  for (int id = 0; id <= 22; ++id) {
    if (id == missing_id) {continue;}
    if (!first) {output += ",";}
    first = false;
    output += "\"" + std::to_string(id) + "\":" +
      std::to_string(id == changed_id ? 999.0 : static_cast<double>(id));
  }
  return output + "}";
}

std::filesystem::path write_catalog(
  const std::string & first_angles, const std::string & second_angles)
{
  const auto path = std::filesystem::temp_directory_path() /
    "irc_step_startup_pose_catalog_test.json";
  std::ofstream stream(path);
  stream << "{\"motions\":[{\"frames\":["
    << "{\"name\":\"오뒤307\",\"frame_id\":\"same\",\"angles\":"
    << first_angles << "},"
    << "{\"name\":\"오뒤307\",\"frame_id\":\"same\",\"angles\":"
    << second_angles << "}]}]}";
  return path;
}

TEST(StartupPoseCatalog, AcceptsRepeatedNameWithIdenticalAngles)
{
  const auto path = write_catalog(angles_json(), angles_json());
  std::vector<double> angles;
  std::string error;
  EXPECT_TRUE(irc_step_motion_executor::load_startup_pose_angles(
      path.string(), "오뒤307", angles, error)) << error;
  ASSERT_EQ(angles.size(), 23U);
  EXPECT_DOUBLE_EQ(angles[22], 22.0);
  std::filesystem::remove(path);
}

TEST(StartupPoseCatalog, RejectsRepeatedNameWithDifferentAngles)
{
  const auto path = write_catalog(angles_json(), angles_json(-1, 7));
  std::vector<double> angles;
  std::string error;
  EXPECT_FALSE(irc_step_motion_executor::load_startup_pose_angles(
      path.string(), "오뒤307", angles, error));
  EXPECT_EQ(error, "startup pose name is ambiguous: 오뒤307");
  std::filesystem::remove(path);
}

TEST(StartupPoseCatalog, RejectsRepeatedNameWithMissingMotor)
{
  const auto path = write_catalog(angles_json(), angles_json(12));
  std::vector<double> angles;
  std::string error;
  EXPECT_FALSE(irc_step_motion_executor::load_startup_pose_angles(
      path.string(), "오뒤307", angles, error));
  EXPECT_EQ(error, "startup pose name is ambiguous: 오뒤307");
  std::filesystem::remove(path);
}

TEST(StartupPoseCatalog, LoadsGeonStartupPoseWithDefaultPolicy)
{
  std::vector<double> angles;
  std::string error;
  ASSERT_TRUE(irc_step_motion_executor::load_startup_pose_with_policy(
      TEST_RUNTIME_CATALOG, TEST_PC_CATALOG, "김오뒤3",
      true, true, angles, error)) << error;
  ASSERT_EQ(angles.size(), 23U);
  EXPECT_DOUBLE_EQ(angles[4], 18.0);
  EXPECT_DOUBLE_EQ(angles[5], -18.0);
  EXPECT_DOUBLE_EQ(angles[13], -41.220703125);
  EXPECT_DOUBLE_EQ(angles[14], 55.3046875);
}

TEST(StartupPoseCatalog, PolicyOffChangesOnlySelectedAxesToVerifiedPcPose)
{
  std::vector<double> production, pc, actual;
  std::string error;
  ASSERT_TRUE(irc_step_motion_executor::load_startup_pose_angles(
      TEST_RUNTIME_CATALOG, "김오뒤3", production, error)) << error;
  ASSERT_TRUE(irc_step_motion_executor::load_startup_pose_angles(
      TEST_PC_CATALOG, "김오뒤3", pc, error)) << error;
  for (bool head : {false, true}) {
    for (bool shoulder : {false, true}) {
      ASSERT_TRUE(irc_step_motion_executor::load_startup_pose_with_policy(
          TEST_RUNTIME_CATALOG, TEST_PC_CATALOG, "김오뒤3",
          head, shoulder, actual, error)) << error;
      for (int id = 0; id < 23; ++id) {
        const bool use_pc = (id == 0 && !head) || ((id == 4 || id == 5) && !shoulder);
        EXPECT_DOUBLE_EQ(actual[id], use_pc ? pc[id] : production[id]);
      }
    }
  }
  actual.clear();
  EXPECT_FALSE(irc_step_motion_executor::load_startup_pose_with_policy(
      TEST_RUNTIME_CATALOG, "/missing/pc.json", "김오뒤3", false, true, actual, error));
  EXPECT_TRUE(actual.empty());
}

TEST(StartupPoseCatalog, LoadsCatalog49ForwardStartWithEveryPolicyCombination)
{
  const std::vector<double> expected{
    -32.34375, -0.3515625, -8.4375, 7.55859375, 18.0, -18.0,
    74.35546875, -76.2890625, -10.546875, 2.4609375, 0.693359375,
    -3.0, 4.0, -89.82421875, 57.0, -1.0, 6.0, -67.236328125,
    32.0, 61.0, -68.0, 2.0, -12.0};
  for (bool head : {false, true}) {
    for (bool shoulder : {false, true}) {
      std::vector<double> actual;
      std::string error;
      ASSERT_TRUE(irc_step_motion_executor::load_startup_pose_with_policy(
          TEST_RUNTIME_CATALOG, TEST_PC_CATALOG, "김오들(앞먼저닿음)",
          head, shoulder, actual, error)) << error;
      EXPECT_EQ(actual, expected);
    }
  }
}

TEST(StartupPoseCatalog, LoadsCatalog51StartupWithEveryPolicyCombination)
{
  const std::vector<double> source{
    -33.57421875, 0.52734375, -9.84375, 9.931640625, 16.5234375, -17.40234375,
    67.060546875, -68.37890625, -10.986328125, 2.63671875, 1.845703125,
    -2.724609375, 2.548828125, -45.220703125, 59.3046875, 3.603515625,
    -1.0546875, -26.015625, 26.279296875, 67.587890625, -57.568359375,
    2.658203125, -5.009765625};
  for (bool head : {false, true}) {
    for (bool shoulder : {false, true}) {
      auto expected = source;
      if (shoulder) {expected[4] = 18.0; expected[5] = -18.0;}
      std::vector<double> actual;
      std::string error;
      ASSERT_TRUE(irc_step_motion_executor::load_startup_pose_with_policy(
          TEST_RUNTIME_CATALOG, TEST_PC_CATALOG, "오뒤무게중심앞",
          head, shoulder, actual, error)) << error;
      EXPECT_EQ(actual, expected);
    }
  }
}

}  // namespace
