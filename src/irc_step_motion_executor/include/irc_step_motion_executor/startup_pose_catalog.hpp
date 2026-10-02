#ifndef IRC_STEP_MOTION_EXECUTOR__STARTUP_POSE_CATALOG_HPP_
#define IRC_STEP_MOTION_EXECUTOR__STARTUP_POSE_CATALOG_HPP_

#include <string>
#include <vector>

namespace irc_step_motion_executor
{

bool load_startup_pose_angles(
  const std::string & json_path, const std::string & pose_name,
  std::vector<double> & angles_deg, std::string & error_message);

bool load_startup_pose_with_policy(
  const std::string & runtime_path, const std::string & reference_path,
  const std::string & pose_name, bool enable_head_override,
  bool enable_shoulder_override, std::vector<double> & angles_deg,
  std::string & error_message);

}  // namespace irc_step_motion_executor

#endif
