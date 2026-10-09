"""Confirm two-point corner headings and bound low-confidence recovery motions."""

from collections import deque
from dataclasses import replace
import math

from .motion_decision_planner import MotionDecisionPlanner


def usable_two_point_line(info, min_quality=.35):
    """Recognize visible short geometry without claiming a reliable heading."""
    if not isinstance(info, dict):
        return False
    return (
        info.get("detected") is True
        and info.get("ground_projection_enabled") is True
        and info.get("ground_projection_valid") is False
        and info.get("ground_fit_reason") == "too_few_segment_points"
        and info.get("ground_fit_segment") in {"PRE_CORNER", "FULL_PATH"}
        and info.get("ground_fit_input_point_count") == 2
        and all(
            (MotionDecisionPlanner._number(info, key) or 0.) >= min_quality
            for key in ("geometry_quality", "detection_quality")
        )
    )


class SparseLineRecovery:
    REQUIRED_FRAMES = 3
    MAX_HEADING_SPREAD_DEG = 5.0
    MAX_POINT_DRIFT_M = 0.015
    MAX_MOTIONS = 2
    MIN_PROGRESS_M = 2 * MAX_POINT_DRIFT_M
    MIN_PROGRESS_DEG = MAX_HEADING_SPREAD_DEG

    def __init__(self):
        self.samples = deque(maxlen=self.REQUIRED_FRAMES)
        self.last_stamp = None
        self.last_seen_at = None
        self.minimum_stamp = 0
        self.direction = None
        self.wait_since = None
        self.stable = False
        self.eligible = False
        self.normal_hits = 0
        self.motions_used = 0
        self.failed = False
        self.pending_action = None
        self.progress_reference = None
        self.current_geometry = None

    def reset_observations(self):
        """Motion, phase changes and invalid frames discard evidence, not budget."""
        self.samples.clear()
        self.direction = None
        self.wait_since = None
        self.stable = self.eligible = False
        self.normal_hits = 0

    def diagnostics(self):
        return {"eligible": self.eligible, "confirmed_frames": len(self.samples),
                "heading_stable": self.stable, "motions_used": self.motions_used,
                "max_motions": self.MAX_MOTIONS, "motion_failed": self.failed}

    @staticmethod
    def _number(info, key):
        return MotionDecisionPlanner._number(info, key)

    def prepare(self, info, *, stamp, ros_now, now, max_age, stationary, min_quality=.35):
        """Accept only fresh, ordered camera captures while the executor is idle."""
        self.eligible = False
        fresh = (
            stationary and isinstance(info, dict)
            and type(stamp) is int and type(ros_now) is int
            and stamp > self.minimum_stamp
            and 0 <= ros_now - stamp <= int(max_age * 1e9)
        )
        if not fresh or (self.last_stamp is not None and stamp < self.last_stamp):
            self.reset_observations()
            return info
        if self.last_seen_at is not None and now - self.last_seen_at > max_age:
            normal_hits = self.normal_hits
            self.reset_observations()
            # Full-fit recovery may be observed between separate normal motions.
            if info.get("ground_projection_valid") is True:
                self.normal_hits = normal_hits
        new_frame = stamp != self.last_stamp
        if new_frame:
            self.last_stamp, self.last_seen_at = stamp, now
        qualities = [self._number(info, key) for key in (
            "heading_quality", "geometry_quality", "detection_quality")]
        quality_ok = all(q is not None and q >= min_quality for q in qualities)
        if info.get("ground_projection_valid") is True:
            if (new_frame and quality_ok and info.get("detected") is True
                    and (self._number(info, "ground_fit_point_count") or 0) >= 3
                    and self._number(info, "ground_heading_error_deg") is not None):
                self.normal_hits += 1
                if self.normal_hits >= self.REQUIRED_FRAMES:
                    self.motions_used = 0
                    self.failed = False
                    self.progress_reference = None
            elif new_frame:
                self.normal_hits = 0
            self.samples.clear()
            self.direction = self.wait_since = None
            self.stable = False
            return info
        self.normal_hits = 0
        candidate = info.get("ground_two_point_candidate")
        distance = self._number(info, "corner_start_distance_m")
        eligible = (
            quality_ok and info.get("detected") is True
            and info.get("ground_projection_enabled") is True
            and info.get("ground_projection_valid") is False
            and info.get("ground_fit_reason") == "too_few_segment_points"
            and info.get("ground_fit_segment") == "PRE_CORNER"
            and info.get("ground_fit_input_point_count") == 2
            and info.get("corner_preview_raw_detected") is True
            and info.get("corner_preview_confirmed") is True
            and info.get("corner_preview_held") is False
            and info.get("corner_start_depth_valid") is True
            and info.get("corner_direction") in {"LEFT", "RIGHT"}
            and distance is not None and distance > 0.0
            and isinstance(candidate, dict)
        )
        if not eligible:
            self.reset_observations()
            return info
        heading = self._number(candidate, "heading_deg")
        points = candidate.get("points_m")
        try:
            if (heading is None or abs(heading) >= 90. or len(points) != 2
                    or any(len(p) != 2 for p in points)
                    or any(type(v) not in (float, int) or not math.isfinite(v)
                           for p in points for v in p)
                    or any(p[1] <= 0. for p in points)):
                raise ValueError("invalid two-point geometry")
            dx, dz = points[1][0] - points[0][0], points[1][1] - points[0][1]
            if (math.hypot(dx, dz) < .04 or dz < .025
                    or abs(math.degrees(math.atan2(dx, dz)) - heading) > .01):
                raise ValueError("invalid two-point baseline")
        except (TypeError, ValueError):
            self.reset_observations()
            return info
        direction = info["corner_direction"]
        if direction != self.direction:
            self.reset_observations()
            self.direction, self.wait_since = direction, now
        self.eligible = True
        if new_frame:
            self.samples.append((heading, points))
        headings = [sample[0] for sample in self.samples]
        self.stable = (
            len(self.samples) == self.REQUIRED_FRAMES
            and max(headings) - min(headings) <= self.MAX_HEADING_SPREAD_DEG
            and all(math.dist(p, ref) <= self.MAX_POINT_DRIFT_M
                    for _, pair in self.samples for p, ref in zip(pair, self.samples[0][1]))
        )
        if not self.stable or self.failed:
            return info
        slope = math.tan(math.radians(heading))
        intercept = points[0][0] - slope * points[0][1]
        lateral_distance = intercept / math.hypot(1., slope)
        self.current_geometry = (distance, abs(lateral_distance), abs(heading))
        if self.progress_reference is not None and self.pending_action is None:
            previous = self.progress_reference
            # Refund only completed movement with a newly confirmed improvement;
            # merely seeing the same corner cannot restart the motion allowance.
            progress = (
                previous[0] - distance > self.MIN_PROGRESS_M
                or (distance <= previous[0] + self.MIN_PROGRESS_M and (
                    previous[1] - abs(lateral_distance) > self.MIN_PROGRESS_M
                    or previous[2] - abs(heading) > self.MIN_PROGRESS_DEG
                ))
            )
            if progress:
                self.motions_used = 0
            self.progress_reference = None
        if self.motions_used >= self.MAX_MOTIONS:
            return info
        return {
            **info, "ground_projection_valid": True,
            "ground_heading_error_deg": heading, "ground_line_points_m": points,
            "ground_fit_point_count": 2, "ground_fit_mode": "TWO_POINT_CONFIRMED",
            "ground_lateral_offset_m": lateral_distance,
            "ground_lookahead_distance_m": points[-1][1],
        }

    @staticmethod
    def _wait(decision, reason):
        return replace(
            decision, action="WAIT", valid=False, reason=reason,
            sdk_motion_requested=False, requires_ack=False,
            source_command={
                **decision.source_command, "valid": False, "motion": "STOP",
                "reason": reason, "linear_speed_mps": 0.,
                "lateral_speed_mps": 0., "angular_speed_rad_s": 0.,
            },
        )

    def constrain(self, decision, info, now):
        """Keep confirmed local recovery/corners, and shorten forward/heading steps."""
        if not self.eligible or decision.source != "line":
            return decision
        if self.failed or self.motions_used >= self.MAX_MOTIONS:
            return self._wait(decision, "sparse_line_motion_failed" if self.failed
                              else "sparse_line_motion_limit_reached")
        if decision.action == "POST_BALL_LINE_ALIGNED":
            return decision
        if self.stable and decision.valid:
            if decision.action.startswith("STRAIGHT"):
                turn_distance = decision.source_command.get("corner_turn_distance_m", .15)
                if self._number(info, "corner_start_distance_m") > turn_distance:
                    return self._motion(decision, "LINE_SPARSE_FORWARD", "two_point_short_forward")
            elif (decision.action.startswith("RECOVER_")
                  or (decision.action in {"LEFT", "RIGHT"}
                      and decision.source_command.get("corner_turn_authorized") is True)):
                return self._motion(decision, decision.action, decision.reason)
            elif "TURN_RIGHT" in decision.action:
                return self._turn(decision, "RIGHT", "two_point_short_turn")
            elif "TURN_LEFT" in decision.action:
                return self._turn(decision, "LEFT", "two_point_short_turn")
            else:
                return decision
        allowed_waits = {
            "invalid_ground_line_geometry", "line_corner_waiting_for_usable_line",
            "straight_heading_not_aligned", "line_corner_approach_heading_not_aligned",
        }
        if not decision.valid and decision.reason not in allowed_waits:
            return decision
        return self._wait(decision, "sparse_line_waiting_for_confirmation")

    def _motion(self, decision, action, reason):
        return replace(
            decision, action=action, valid=True, reason=reason,
            sdk_motion_requested=False, requires_ack=False,
            source_command={
                **decision.source_command, "valid": True,
                "motion": action, "reason": reason, "sparse_line_motion": True,
                "sparse_line_motions_used": self.motions_used,
                "sparse_line_max_motions": self.MAX_MOTIONS,
                "ground_fit_mode": "TWO_POINT_CONFIRMED" if self.stable else "TWO_POINT_UNSTABLE",
                "two_point_heading_deg": self.samples[-1][0],
                "two_point_corner_direction": self.direction,
            },
        )

    def _turn(self, decision, direction, reason):
        count = 1 if direction == "LEFT" else 2
        result = self._motion(decision, f"LINE_SPARSE_TURN_{direction}_{count}", reason)
        return replace(result, source_command={
            **result.source_command,
            "turn_direction": direction, "turn_count": count, "turn_angle_deg": 15.,
            "target_heading_change_deg": 15. if direction == "RIGHT" else -15.,
        })

    def published(self, decision, stamp):
        sparse_motion = decision.source_command.get("sparse_line_motion") is True
        if decision.valid and sparse_motion:
            self.motions_used += 1
            self.pending_action = decision.action
            self.progress_reference = self.current_geometry
        if decision.valid:
            if type(stamp) is int:
                self.minimum_stamp = max(self.minimum_stamp, stamp)
            normal_hits = self.normal_hits if decision.source == "line" and not sparse_motion else 0
            self.reset_observations()
            self.normal_hits = normal_hits

    def completed(self, action, status, stamp=None):
        if self.pending_action == action:
            self.failed = status != "SUCCEEDED"
            self.pending_action = None
            if type(stamp) is int:
                self.minimum_stamp = max(self.minimum_stamp, stamp)
            self.reset_observations()
