#include "robot_motion_player.hpp"
#include "mock_motion_hardware.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

// Uses only the mock bus; tests the actual SDK queue acceptance rules.
int main(int argc, char** argv) {
    if (argc != 3) return 2;
    std::ifstream input(argv[2]);
    std::vector<std::pair<std::string, std::string>> aliases;
    std::string line;
    while (std::getline(input, line)) {
        auto tab = line.find('\t');
        aliases.emplace_back(line.substr(0, tab), line.substr(tab + 1));
    }
    irc_step::MockMotionHardware bus;
    irc_step::JointAngles initial;
    for (int id = 0; id < 23; ++id) initial[id] = 0.;
    bus.setPresentPositions(initial);
    irc_step::RobotMotionPlayer player(argv[1], bus);
    if (!player.initialize()) return 3;
    for (bool fixed_head : {false, true}) {
        if (fixed_head) player.setJointOverride(0, -64.);
        for (const auto& [from_id, from] : aliases) {
            for (const auto& [to_id, to] : aliases) {
                player.cancel();
                auto start = player.start(from);
                if (start != irc_step::StartResult::Accepted) return 4;
                auto result = player.queueNext(to);
                std::cout << fixed_head << '\t' << from_id << '\t' << to_id
                          << '\t' << static_cast<int>(result) << '\n';
            }
        }
    }
}
