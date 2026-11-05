#!/usr/bin/env python3
"""Convert goal geometry into SDK-oriented scoring action candidates."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

from .approach_distance import approach_level_from_motion
from .stationary_turns import stationary_turn_repeat_count
from .goal_scoring import (
    GOAL_SCORE_MIN_DEPTH_M, GOAL_SCORE_TARGET_DEPTH_M, GOAL_SCORE_MAX_DEPTH_M,
    goal_shot_pixel_limit, goal_shot_region,
)


GOAL_FINE_APPROACH_MAX_DEPTH_M = 0.850
GOAL_FINE_APPROACH_TURN_MIN_DEG = 30.0
GOAL_FINE_STEP_DISTANCE_M = 0.030
GOAL_FINE_MAX_REPEATS = 4


@dataclass(frozen=True)
class GoalNavigationConfig:
    """Provisional goal alignment and scoring thresholds."""

    min_confidence: float = 0.55
    control_start_depth_m: float = 2.0
    score_target_depth_m: float = GOAL_SCORE_TARGET_DEPTH_M
    score_crab_max_offset_px: float = 140.0


@dataclass(frozen=True)
class GoalActionCommand:
    """One abstract goal action; it does not drive robot hardware."""

    valid: bool
    action: str
    reason: str
    sdk_motion_requested: bool
    confidence: float
    depth_m: float | None
    distance_m: float | None
    depth_error_m: float | None
    bearing_error_deg: float | None
    offset_x_norm: float | None
    is_centered: bool
    depth_in_score_range: bool
    score_now: bool
    shot_region: int | None = None
    target_steering_angle_deg: float | None = None
    target_lateral_error_m: float | None = None

    def to_dict(self) -> dict[str, Any]:
        """Return a rounded JSON-compatible representation."""
        approach_level = approach_level_from_motion(self.action)
        return {
            "valid": self.valid,
            "action": self.action,
            "reason": self.reason,
            "sdk_motion_requested": self.sdk_motion_requested,
            "confidence": round(self.confidence, 4),
            "depth_m": _round_optional(self.depth_m, 3),
            "distance_m": _round_optional(self.distance_m, 3),
            "depth_error_m": _round_optional(self.depth_error_m, 3),
            "bearing_error_deg": _round_optional(
                self.bearing_error_deg,
                3,
            ),
            "offset_x_norm": _round_optional(self.offset_x_norm, 6),
            "is_centered": self.is_centered,
            "depth_in_score_range": self.depth_in_score_range,
            "score_now": self.score_now,
            "shot_region": self.shot_region,
            "target_steering_angle_deg": _round_optional(self.target_steering_angle_deg, 3),
            "target_lateral_error_m": _round_optional(self.target_lateral_error_m, 6),
            "target_forward_error_m": _round_optional(self.depth_error_m, 3),
            "approach_motion": (
                self.action
                if self.action == "STRAIGHT" or approach_level is not None
                else None
            ),
            "approach_level": approach_level,
            "approach_target_distance_m": _round_optional(
                self.depth_m,
                3,
            ),
        }


def _round_optional(value: float | None, digits: int) -> float | None:
    if value is None:
        return None
    return round(value, digits)


def _number(data: dict[str, Any], key: str) -> float | None:
    value = data.get(key)
    if value is None or isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


class GoalNavigationPlanner:
    """Choose a scoring SDK action from one fresh ``goal_info`` sample."""

    def __init__(
        self,
        config: GoalNavigationConfig | None = None,
    ) -> None:
        self.config = config or GoalNavigationConfig()

    def wait(self, reason: str) -> GoalActionCommand:
        """Return a non-action command for missing or unsafe input."""
        return GoalActionCommand(
            valid=False,
            action="WAIT",
            reason=reason,
            sdk_motion_requested=False,
            confidence=0.0,
            depth_m=None,
            distance_m=None,
            depth_error_m=None,
            bearing_error_deg=None,
            offset_x_norm=None,
            is_centered=False,
            depth_in_score_range=False,
            score_now=False,
        )

    def plan(self, goal_info: dict[str, Any]) -> GoalActionCommand:
        """Create one alignment, distance-adjustment, or scoring action."""
        if not bool(goal_info.get("detected", False)):
            return self.wait("goal_not_detected")
        confidence = _number(goal_info, "confidence")
        if confidence is None or confidence < self.config.min_confidence:
            return self.wait("low_goal_confidence")
        depth = _number(goal_info, "depth_m")
        if not bool(goal_info.get("depth_valid", False)) or depth is None or depth <= 0.0:
            return self.wait("missing_valid_goal_depth")
        if depth > self.config.control_start_depth_m:
            return self.wait("goal_outside_control_range")

        distance = _number(goal_info, "distance_m")
        bearing = _number(goal_info, "bearing_deg")
        offset = _number(goal_info, "offset_x_norm")
        offset_px = _number(goal_info, "offset_x_px")
        if offset_px is None:
            return self.wait("invalid_goal_alignment")
        region = goal_shot_region(depth, offset_px)
        centered = abs(offset_px) <= goal_shot_pixel_limit(depth)
        depth_error = depth - self.config.score_target_depth_m
        depth_in_range = (
            GOAL_SCORE_MIN_DEPTH_M <= depth < GOAL_SCORE_MAX_DEPTH_M
        )
        ready_geometry = region is not None
        analyzer_score_now = goal_info.get("score_now")
        score_now = bool(
            ready_geometry
            and analyzer_score_now is True
        )
        target_steering = None
        lateral_error = None

        if score_now:
            action = "SHOT"
            reason = "goal_centered_at_scoring_depth"
        elif depth < GOAL_SCORE_MIN_DEPTH_M:
            action = "GOAL_CAMERA90_BACKWARD_1"
            reason = "retreat_goal_to_scoring_depth"
        elif depth_in_range and not centered:
            direction = "RIGHT" if offset_px > 0.0 else "LEFT"
            if abs(offset_px) > self.config.score_crab_max_offset_px:
                # Re-evaluate pixels after each completed turn and stationary pause.
                if bearing is None:
                    return self.wait("missing_goal_bearing_for_camera90_alignment")
                if bearing * offset_px < 0.0:
                    return self.wait("goal_bearing_pixel_direction_mismatch")
                count = stationary_turn_repeat_count(bearing, direction)
                if count == 0:
                    # Pixels still require correction even below the yaw bin.
                    action = f"GOAL_CAMERA90_CRAB_{direction}"
                    reason = "align_goal_pixels_below_turn_minimum"
                else:
                    action = f"GOAL_CAMERA90_TURN_{direction}_{count}"
                    reason = "align_goal_large_pixel_offset_camera90"
            else:
                action = f"GOAL_CAMERA90_CRAB_{direction}"
                reason = "align_goal_lateral_camera90"
        elif ready_geometry:
            action = "WAIT_SCORE_CONFIRMATION"
            reason = "waiting_for_stable_score_condition"
        elif GOAL_SCORE_MAX_DEPTH_M <= depth <= GOAL_FINE_APPROACH_MAX_DEPTH_M:
            if bearing is None:
                return self.wait("missing_goal_bearing_for_camera90_alignment")
            if abs(bearing) >= 90.0:
                return self.wait("invalid_goal_bearing_for_camera90_alignment")
            if bearing * offset_px < 0.0:
                return self.wait("goal_bearing_pixel_direction_mismatch")
            direction = "RIGHT" if bearing > 0.0 else "LEFT"
            if abs(bearing) >= GOAL_FINE_APPROACH_TURN_MIN_DEG:
                count = stationary_turn_repeat_count(bearing, direction)
                action = f"GOAL_CAMERA90_TURN_{direction}_{count}"
                reason = "align_goal_yaw_before_fine_approach"
            else:
                # Enter the scoring band strictly below its upper bound.
                # Cap each request at four steps, then reobserve the actual depth.
                remaining_steps = round(
                    (depth - GOAL_SCORE_MAX_DEPTH_M) / GOAL_FINE_STEP_DISTANCE_M, 9,
                )
                count = min(GOAL_FINE_MAX_REPEATS, max(1, math.floor(remaining_steps) + 1))
                action = f"GOAL_CAMERA90_FINE_FORWARD_{count}"
                reason = "approach_goal_by_depth"
        elif depth > GOAL_FINE_APPROACH_MAX_DEPTH_M:
            # The board is elevated: use its measured X/Z, not a floor-ray
            # intersection. Steer toward the stopping point 510 mm before it.
            lateral_error = _number(goal_info, "lateral_offset_m")
            if ("lateral_offset_m" not in goal_info
                    and bearing is not None and abs(bearing) < 90.0):
                lateral_error = depth * math.tan(math.radians(bearing))
            if lateral_error is None or depth_error <= 0.0:
                return self.wait("missing_valid_goal_target_geometry")
            if lateral_error * offset_px < 0.0:
                return self.wait("goal_target_pixel_direction_mismatch")
            # Suppress floating-point roundoff at inclusive angular boundaries.
            target_steering = round(math.degrees(math.atan2(lateral_error, depth_error)), 9)
            direction = "RIGHT" if target_steering > 0.0 else "LEFT"
            if abs(target_steering) <= 15.0:
                action = "GOAL_CAMERA_90_FORWARD_2"
                reason = "approach_goal_target_forward_4"
            elif abs(target_steering) < 45.0:
                action = f"GOAL_CAMERA90_RECOVER_{direction}_4"
                reason = "approach_goal_target_recovery"
            else:
                count = stationary_turn_repeat_count(target_steering, direction)
                action = f"GOAL_CAMERA90_TURN_{direction}_{count}"
                reason = "approach_goal_target_stationary_turn"
        else:
            return self.wait("goal_too_close_hold")

        return GoalActionCommand(
            valid=True,
            action=action,
            reason=reason,
            sdk_motion_requested=score_now,
            confidence=confidence,
            depth_m=depth,
            distance_m=distance,
            depth_error_m=depth_error,
            bearing_error_deg=bearing,
            offset_x_norm=offset,
            is_centered=centered,
            depth_in_score_range=depth_in_range,
            score_now=score_now,
            shot_region=region,
            target_steering_angle_deg=target_steering,
            target_lateral_error_m=lateral_error,
        )
