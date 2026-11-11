"""Confirm one or two visible line markers before a bounded viewing turn."""

from collections import deque
import math

from .motion_decision_planner import MotionDecisionPlanner


class VisibleSparseLineObserver:
    WAIT_SEC = 0.5
    REQUIRED_FRAMES = 3
    MIN_CONFIDENCE = 0.6
    MAX_POINT_DRIFT_NORM = 0.025
    CENTER_DEADBAND_NORM = 0.02

    def __init__(self):
        self.samples = deque(maxlen=self.REQUIRED_FRAMES)
        self.last_stamp = None
        self.wait_since = None
        self.direction = None
        self.bearing = None
        self.ready = False
        self.reason = "line_sparse_waiting_for_confirmation"

    def reset(self):
        self.samples.clear()
        self.wait_since = self.direction = self.bearing = None
        self.ready = False

    def observe(self, info, *, stamp, ros_now, now, minimum_stamp, max_age, stationary):
        self.ready = False
        self.reason = "line_sparse_waiting_for_confirmation"
        if (not stationary or type(stamp) is not int or type(ros_now) is not int
                or stamp <= minimum_stamp or not 0 <= ros_now - stamp <= int(max_age * 1e9)
                or self.last_stamp is not None and stamp < self.last_stamp):
            self.reset()
            self.reason = "line_sparse_waiting_for_fresh_frame"
            return
        number = MotionDecisionPlanner._number
        width, height, center = [number(info, k) for k in (
            "image_width", "image_height", "robot_center_x_px",
        )]
        points, confidences = info.get("visible_line_points_px"), info.get("visible_line_confidences")
        try:
            if (width is None or height is None or center is None or width <= 0 or height <= 1
                    or not 0 <= center < width or len(points) not in (1, 2)
                    or len(confidences) != len(points)):
                raise ValueError
            normalized = []
            for point, confidence in zip(points, confidences):
                if len(point) != 2:
                    raise ValueError
                x, y = [number({"v": v}, "v") for v in point]
                confidence = number({"v": confidence}, "v")
                if (x is None or y is None or not 0 <= x < width or not 0 <= y < height - 1
                        or confidence is None or not self.MIN_CONFIDENCE <= confidence <= 1):
                    raise ValueError
                normalized.append((x / width, y / height))
        except (TypeError, ValueError):
            self.reset()
            self.reason = "line_sparse_invalid_observation"
            return
        x = sum(p[0] for p in points) / len(points)
        y = sum(p[1] for p in points) / len(points)
        if abs(x - center) <= width * self.CENTER_DEADBAND_NORM:
            self.reset()
            self.reason = "line_sparse_direction_uncertain"
            return
        direction = "RIGHT" if x > center else "LEFT"
        if (self.samples and (
                direction != self.direction or len(normalized) != len(self.samples[0][1])
                or ros_now - self.samples[-1][0] > int(max_age * 1e9)
                or any(math.dist(p, ref) > self.MAX_POINT_DRIFT_NORM
                       for p, ref in zip(normalized, self.samples[0][1])))):
            self.reset()
        if stamp == self.last_stamp:
            return
        self.last_stamp = stamp
        if self.wait_since is None:
            self.wait_since = now
        self.direction = direction
        self.bearing = math.degrees(math.atan2(x - center, height - 1 - y))
        self.samples.append((stamp, normalized))
        while self.samples and ros_now - self.samples[0][0] > int(max_age * 1e9):
            self.samples.popleft()
        self.ready = len(self.samples) >= self.REQUIRED_FRAMES and now - self.wait_since >= self.WAIT_SEC
        if self.ready:
            self.reason = "line_sparse_view_turn"

    def diagnostics(self):
        return {"confirmed_frames": len(self.samples), "wait_sec": self.WAIT_SEC,
                "direction": self.direction, "ready": self.ready, "reason": self.reason}
