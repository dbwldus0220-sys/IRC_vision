#ifndef FAKE_ROBOT_MOTION_SDK_TEST_SUPPORT_HPP_
#define FAKE_ROBOT_MOTION_SDK_TEST_SUPPORT_HPP_

#include <string>
#include <vector>

namespace irc_step::fake_sdk
{

void reset_tracking();
void set_player_constructor_throws(bool value);
int hardware_construction_count();
int hardware_initialize_count();
int player_construction_count();
int player_initialize_count();
const std::vector<std::string> & destruction_order();

}  // namespace irc_step::fake_sdk

#endif  // FAKE_ROBOT_MOTION_SDK_TEST_SUPPORT_HPP_
