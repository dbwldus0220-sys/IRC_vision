#pragma once
#include "gui_playback.hpp"
#include <array>
#include <atomic>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <thread>

namespace irc_step {
struct PlaybackTraceRow {
    const char* event{""};
    std::array<char, 256> motion{};
    std::array<char, 256> frame_name{};
    bool lift_early{false}, cycle_end{false};
    std::uint64_t run{0};
    std::uint64_t event_seq{0}, tick{0};
    std::int64_t begin_ns{0}, end_ns{0}, evaluate_ns{0};
    std::int64_t frame_start_ms{0}, frame_duration_ms{0};
    double speed{1.0};
    int repeat_target{1};
    double begin_ms{0}, end_ms{0};
    std::int64_t timeline_ms{0};
    int repeat{0}, frame{-1};
    bool success{true};
    std::array<double, 23> angles{};
    std::array<bool, 23> ids{};
};
// 단일 제어 스레드 -> 파일 기록 스레드. 루프에서 파일 I/O/lock/할당을 하지 않는다.
class PlaybackTrace {
public:
    static constexpr std::size_t capacity = 8192;
    explicit PlaybackTrace(const std::string& path) : rows_(new PlaybackTraceRow[capacity]), out_(path) {
        if (!out_) throw std::runtime_error("cannot open playback trace: " + path);
        out_ << "run,motion,event,begin_ms,end_ms,duration_ms,timeline_ms,repeat,frame,success";
        for (int id = 0; id < 23; ++id) out_ << ",deg_" << id;
        for (int id = 0; id < 23; ++id) out_ << ",raw_" << id;
        out_ << ",event_seq,tick,begin_ns,end_ns,evaluate_ns,speed,repeat_target,frame_start_ms,frame_duration_ms,frame_name,lift_early,cycle_end";
        out_ << '\n' << std::setprecision(17);
        worker_ = std::thread([this] { consume(); });
    }
    ~PlaybackTrace() {
        stop_.store(true);
        worker_.join();
        out_ << "# dropped_rows=" << dropped_.load() << '\n';
    }
    void push(const PlaybackTraceRow& row) noexcept {
        const auto head = head_.load(std::memory_order_relaxed);
        const auto next = (head + 1) % capacity;
        if (next == tail_.load(std::memory_order_acquire)) { ++dropped_; return; }
        rows_[head] = row;
        head_.store(next, std::memory_order_release);
    }
private:
    void consume() {
        while (!stop_.load() || tail_.load() != head_.load()) {
            auto tail = tail_.load(std::memory_order_relaxed);
            if (tail == head_.load(std::memory_order_acquire)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            const auto& r = rows_[tail];
            // CSV의 따옴표는 backslash가 아니라 두 번 써서 escape한다.
            out_ << r.run << ",\"";
            for (const char* c = r.motion.data(); *c; ++c) {
                if (*c == '"') out_ << '"';
                out_ << *c;
            }
            out_ << "\"," << r.event << ','
                 << r.begin_ms << ',' << r.end_ms << ',' << r.end_ms-r.begin_ms << ','
                 << r.timeline_ms << ',' << r.repeat << ',' << r.frame << ',' << r.success;
            for (int id = 0; id < 23; ++id) {
                out_ << ',';
                if (r.ids[id]) out_ << r.angles[id];
            }
            for (int id = 0; id < 23; ++id) {
                out_ << ',';
                if (r.ids[id]) out_ << guiPositionRaw(r.angles[id]);
            }
            out_ << ',' << r.event_seq << ',' << r.tick << ',' << r.begin_ns << ','
                 << r.end_ns << ',' << r.evaluate_ns << ',' << r.speed << ','
                 << r.repeat_target << ',' << r.frame_start_ms << ',' << r.frame_duration_ms << ",\"";
            for (const char* c = r.frame_name.data(); *c; ++c) {
                if (*c == '"') out_ << '"';
                out_ << *c;
            }
            out_ << "\"," << r.lift_early << ',' << r.cycle_end << '\n';
            tail_.store((tail + 1) % capacity, std::memory_order_release);
        }
    }
    std::unique_ptr<PlaybackTraceRow[]> rows_;
    std::ofstream out_;
    std::thread worker_;
    std::atomic<std::size_t> head_{0}, tail_{0}, dropped_{0};
    std::atomic<bool> stop_{false};
};
}
