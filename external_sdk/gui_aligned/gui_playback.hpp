#pragma once
#include "motion_pattern.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace irc_step {
inline double guiShortestDelta(double start, double end) {
    double delta = std::fmod(end - start + 180.0, 360.0);
    if (delta < 0.0) delta += 360.0;
    return delta - 180.0;
}
inline double guiFrameProgress(const MotionFrame& frame, std::int64_t time_ms) {
    const bool lift = frame.lift_early_arrival &&
        (frame.name.find("발들") != std::string::npos ||
         frame.name.find("들기") != std::string::npos ||
         frame.name.find("오들") != std::string::npos ||
         frame.name.find("왼들") != std::string::npos);
    const double duration = std::max(1.0, frame.time_ms * (lift ? 0.8 : 1.0));
    const double fraction = std::clamp((time_ms - frame.start_ms) / duration, 0.0, 1.0);
    return 0.5 - 0.5 * std::cos(std::acos(-1.0) * fraction);
}
// Python round()와 같은 ties-to-even. 전송 각도는 GUI와 같은 4096 step/rev.
inline int guiPositionRaw(double degree) {
    if (!std::isfinite(degree)) throw std::invalid_argument("non-finite motor angle");
    if (degree > 180.0 || degree < -180.0) {
        degree = std::fmod(degree, 360.0);
        if (degree > 180.0) degree -= 360.0;
        if (degree < -180.0) degree += 360.0;
    }
    degree = std::clamp(degree, -180.0, 179.912109375);
    const double raw = 2048.0 + degree * (4096.0 / 360.0);
    const double lower = std::floor(raw);
    const double fraction = raw - lower;
    return static_cast<int>(lower + (fraction > 0.5 ||
        (fraction == 0.5 && static_cast<int>(lower) % 2 != 0)));
}
struct GuiSample {
    std::array<double, 23> angles{};
    std::array<bool, 23> commanded{};
};
// 구간 생성은 시작/반복 경계에서만 한다. 5ms 샘플링에서는 할당하지 않는다.
class GuiContinuousTrajectory {
    struct Segment {
        double start_ms{}, end_ms{}, duration_ms{};
        GuiSample start, goal;
        std::array<bool, 23> active{};
        std::array<double, 23> delta{}, v0{}, v1{};
    };
    std::vector<Segment> segments_;
public:
    void reset(const std::vector<MotionFrame>& frames, const JointAngles& initial,
               double /* speed */) {
        segments_.clear();
        segments_.reserve(frames.size());
        GuiSample pose;
        for (const auto& [id, angle] : initial) {
            pose.angles.at(id) = angle;
            pose.commanded.at(id) = true;
        }
        for (const auto& frame : frames) {
            Segment s;
            s.start_ms = frame.start_ms;
            s.end_ms = frame.start_ms + frame.time_ms;
            // 저장된 프레임 시각에는 원본 배속이 반영되어 있다. 전체 배속은
            // 플레이어 시계에서만 적용하고 경계속도는 재생 ms 기준으로 계산한다.
            s.duration_ms = static_cast<double>(frame.time_ms);
            s.start = pose;
            for (const auto& [id, goal] : frame.angles) {
                if (!pose.commanded.at(id)) s.start.angles.at(id) = goal;
                s.active.at(id) = true;
                s.delta.at(id) = guiShortestDelta(s.start.angles.at(id), goal);
                pose.angles.at(id) = goal;
                pose.commanded.at(id) = true;
            }
            s.goal = pose;
            segments_.push_back(s);
        }
        for (std::size_t i = 1; i < segments_.size(); ++i) {
            auto& a = segments_[i-1]; auto& b = segments_[i];
            const auto& fa = frames[i-1]; const auto& fb = frames[i];
            if (a.end_ms != b.start_ms || fa.playback_cycle_end || fa.stop_at_end ||
                fa.name.find("[착지]") != std::string::npos ||
                fb.name.find("[착지]") != std::string::npos) continue;
            for (int id = 0; id < 23; ++id) {
                if (!a.active[id] || !b.active[id] || a.delta[id] * b.delta[id] <= 0) continue;
                // 최단각과 저장된 각도 차이가 다르면 인코더 래핑 구간이다.
                if (std::abs(a.goal.angles[id] - a.start.angles[id] - a.delta[id]) > 1e-9 ||
                    std::abs(b.goal.angles[id] - b.start.angles[id] - b.delta[id]) > 1e-9 ||
                    a.start.angles[id] < -180 || a.start.angles[id] > 179.912109375 ||
                    a.goal.angles[id] < -180 || a.goal.angles[id] > 179.912109375 ||
                    b.goal.angles[id] < -180 || b.goal.angles[id] > 179.912109375) continue;
                a.v1[id] = b.v0[id] = std::copysign(std::min(
                    std::abs(a.delta[id] / a.duration_ms),
                    std::abs(b.delta[id] / b.duration_ms)), a.delta[id]);
            }
        }
    }
    GuiSample sample(double time_ms, bool boundary = false) const {
        if (segments_.empty()) return {};
        GuiSample pose = segments_.front().start;
        for (const auto& s : segments_) {
            if (time_ms < s.start_ms) return pose;
            if (time_ms == s.end_ms && (boundary || time_ms == segments_.back().end_ms)) {
                pose = s.goal;
                pose.commanded = s.active;
                return pose;
            }
            if (time_ms >= s.end_ms) { pose = s.goal; continue; }
            pose = s.start;
            pose.commanded = s.active;
            const double u = std::clamp((time_ms - s.start_ms) / (s.end_ms - s.start_ms), 0.0, 1.0);
            for (int id = 0; id < 23; ++id) {
                if (!s.active[id]) continue;
                const double m0 = s.v0[id] * s.duration_ms, m1 = s.v1[id] * s.duration_ms;
                const double d = s.delta[id];
                const double c3 = 10*d - 6*m0 - 4*m1;
                const double c4 = -15*d + 8*m0 + 7*m1;
                const double c5 = 6*d - 3*m0 - 3*m1;
                pose.angles[id] = s.start.angles[id] + m0*u + u*u*u*(c3 + u*(c4 + u*c5));
                pose.commanded[id] = true;
            }
            return pose;
        }
        return pose;
    }
};
inline GuiSample sampleGuiMotion(const std::vector<MotionFrame>& frames,
    const JointAngles& initial, std::int64_t time_ms, bool cycle_end = false) {
    GuiSample base;
    auto merge = [](GuiSample& sample, const JointAngles& angles) {
        for (const auto& [id, value] : angles) {
            if (id < 0 || id >= 23 || !std::isfinite(value))
                throw std::invalid_argument("invalid GUI joint target");
            sample.angles[id] = value;
            sample.commanded[id] = true;
        }
    };
    merge(base, initial);
    const auto end = frames.back().start_ms + frames.back().time_ms;
    for (const auto& frame : frames) {
        const auto frame_end = frame.start_ms + frame.time_ms;
        const bool active = (frame.start_ms <= time_ms && time_ms < frame_end) ||
            (time_ms == frame_end && (cycle_end || time_ms == end));
        if (active) {
            GuiSample target;
            const double progress = guiFrameProgress(frame, time_ms);
            for (const auto& [id, goal] : frame.angles) {
                const double start = base.commanded[id] ? base.angles[id] : goal;
                target.angles[id] = start + guiShortestDelta(start, goal) * progress;
                target.commanded[id] = true;
            }
            return target;
        }
        if (frame_end <= time_ms) merge(base, frame.angles);
    }
    return base;
}
}  // namespace irc_step
