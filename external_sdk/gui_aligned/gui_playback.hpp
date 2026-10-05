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
