#include "robot_motion_player.hpp"
#include "mock_motion_hardware.hpp"
#include <chrono>
#include <iostream>

// Offline only: link the same player library used by the installed executor.
int main(int argc, char** argv) {
    if (argc != 4) return 2;
    using namespace irc_step;
    auto now = RobotMotionPlayer::Clock::time_point(std::chrono::seconds(1));
    MockMotionHardware hardware;
    JointAngles initial;
    for (int i = 0; i < 23; ++i) initial[i] = i * .25 - 3;
    hardware.setPresentPositions(initial);
    RobotMotionPlayer player(argv[1], hardware, [&] { return now; });
    player.setPositionToleranceEnabled(false);
    player.configureDiagnostics(5, "", true);
    if (!player.initialize() || player.start(argv[2]) != StartResult::Accepted) return 3;
    const bool queue = std::stoi(argv[3]);
    if (queue && player.queueNext(argv[2]) != QueueResult::Queued) return 4;
    int previous_commands = 0;
    std::uint64_t previous_completed = 0;
    for (int ms = 0; ms < 20000; ms += 5) {
        now = RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(1000 + ms));
        player.update();
        if (hardware.commandCount() != previous_commands) {
            std::cout << "WRITE " << ms << ' ' << player.completionSequence();
            for (int id : {13, 14, 17, 18, 19, 20})
                std::cout << ' ' << hardware.commandedPositions().at(id);
            std::cout << '\n';
            previous_commands = hardware.commandCount();
        }
        if (player.completionSequence() != previous_completed) {
            std::cout << "COMPLETE " << ms << ' ' << player.completionSequence()
                      << ' ' << hardware.readCount() << '\n';
            previous_completed = player.completionSequence();
        }
        if (!player.running()) {
            const auto d = player.lastDiagnostics();
            std::cout << "RESULT " << ms << ' ' << hardware.commandCount()
                      << ' ' << hardware.readCount() << ' '
                      << (d ? d->final_corrections : 0) << '\n';
            return player.succeeded() ? 0 : 5;
        }
    }
    return 6;
}
