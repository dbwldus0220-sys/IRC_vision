// 실측 GUI 시각으로 실제 RobotMotionPlayer를 실행하는 무하드웨어 검증기.
// Dynamixel/ROS와 링크하지 않으며 로봇에는 명령을 보내지 않는다.
#include "robot_motion_player.hpp"
#include "gui_playback.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>

using namespace irc_step;
struct RecordedIo {
    char kind{};
    double begin{}, end{};
    bool success{};
    JointAngles angles;
    std::map<int, int> raw;
};
struct RecordedTick { double at{}; std::vector<RecordedIo> io; };

class RecordedBus : public IMotionHardware {
public:
    double ms{1000};
    JointAngles initial;
    const std::vector<RecordedIo>* events{nullptr};
    std::size_t index{0}, goal_mismatches{0}, unmatched_io{0}, late_events{0};
    bool initialize() noexcept override { return true; }
    bool ready() const noexcept override { return true; }
    bool commandPosition(const JointAngles&, const std::vector<int>&, uint32_t, uint32_t) noexcept override { return false; }
    const RecordedIo* consume(char kind) noexcept {
        if (!events || index >= events->size() || (*events)[index].kind != kind) {
            ++unmatched_io; return nullptr;
        }
        const auto& event = (*events)[index++];
        if (ms > 1000 + event.begin + 1e-5) ++late_events;
        // GUI의 화면 처리가 쓰기 전에 사용한 시간도 기록대로 반영한다.
        ms = std::max(ms, 1000 + event.end);
        return &event;
    }
    bool commandImmediatePosition(const JointAngles& goal, const std::vector<int>& ids) noexcept override {
        const auto* event = consume('W');
        if (!event) return false;
        bool same = ids.size() == event->raw.size();
        for (int id : ids) {
            const auto expected = event->raw.find(id);
            if (expected == event->raw.end() || guiPositionRaw(goal.at(id)) != expected->second) same = false;
        }
        if (!same) ++goal_mismatches;
        return event->success;
    }
    bool readPresentPositions(JointAngles& value) noexcept override {
        if (!events) { value = initial; return true; }
        const auto* event = consume('R');
        if (!event) return false;
        value = event->angles;
        return event->success;
    }
    bool holdCurrentPosition(uint32_t) noexcept override { return false; }
    bool setTorqueEnabled(bool) noexcept override { return true; }
    std::string_view lastError() const noexcept override { return "recorded I/O order differs"; }
};

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "usage: recorded_execution_probe catalog.json schedule.txt\n"; return 2;
    }
    try {
        RecordedBus bus;
        std::ifstream input(argv[2]);
        if (!input) throw std::runtime_error("cannot open schedule");
        std::vector<RecordedTick> ticks;
        std::string line;
        bool have_initial = false;
        while (std::getline(input, line)) {
            std::istringstream row(line);
            char kind; row >> kind;
            if (kind == 'I') {
                if (have_initial || !ticks.empty()) throw std::runtime_error("misplaced initial angles");
                for (int id=0; id<23; ++id) {
                    double value; if (!(row >> value) || !std::isfinite(value)) throw std::runtime_error("invalid initial angle");
                    bus.initial[id] = value;
                }
                have_initial = true;
            } else if (kind == 'T') {
                double at;
                if (!have_initial || !(row >> at) || !std::isfinite(at) || at < 0 || (!ticks.empty() && at < ticks.back().at))
                    throw std::runtime_error("invalid tick clock");
                ticks.push_back({at, {}});
            } else if (kind == 'W' || kind == 'R') {
                RecordedIo event; event.kind = kind; int count;
                if (ticks.empty() || !(row >> event.begin >> event.end >> event.success >> count) ||
                    !std::isfinite(event.begin) || !std::isfinite(event.end) ||
                    event.begin < ticks.back().at || event.end < event.begin || count < 0 || count > 23)
                    throw std::runtime_error("invalid I/O interval");
                if (!ticks.back().io.empty() && event.begin < ticks.back().io.back().end)
                    throw std::runtime_error("overlapping I/O intervals");
                for (int i=0; i<count; ++i) {
                    int id, raw; double angle;
                    if (!(row >> id >> angle >> raw) || id < 0 || id >= 23 || !std::isfinite(angle) || event.angles.contains(id))
                        throw std::runtime_error("invalid recorded joint");
                    if (kind == 'W' && (raw < 0 || raw > 4095 || guiPositionRaw(angle) != raw))
                        throw std::runtime_error("recorded angle/raw disagree");
                    event.angles[id] = angle; event.raw[id] = raw;
                }
                ticks.back().io.push_back(std::move(event));
            } else throw std::runtime_error("unknown schedule event");
            std::string extra; if (row >> extra) throw std::runtime_error("trailing schedule data");
        }
        if (ticks.empty()) throw std::runtime_error("empty schedule");
        RobotMotionPlayer player(argv[1], bus, [&] {
            return RobotMotionPlayer::Clock::time_point(std::chrono::duration_cast<RobotMotionPlayer::Clock::duration>(
                std::chrono::duration<double, std::milli>(bus.ms)));
        });
        if (!player.initialize() || player.start("recorded_gui") != StartResult::Accepted)
            throw std::runtime_error("cannot start recorded motion");
        for (const auto& tick : ticks) {
            if (1000 + tick.at + 1e-5 < bus.ms) throw std::runtime_error("tick overlaps previous I/O");
            bus.ms = 1000 + tick.at;
            bus.events = &tick.io; bus.index = 0;
            player.update();
            bus.unmatched_io += tick.io.size() - bus.index;
        }
        std::cout << "{\"hardware_accessed\":false,\"goal_mismatches\":" << bus.goal_mismatches
                  << ",\"unmatched_io\":" << bus.unmatched_io << ",\"late_events\":" << bus.late_events
                  << ",\"completed\":" << (player.succeeded() ? "true" : "false") << "}\n";
        return player.succeeded() && !bus.goal_mismatches && !bus.unmatched_io && !bus.late_events ? 0 : 1;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 2; }
}
