#!/usr/bin/env python3
"""Convert goal geometry into SDK-oriented scoring action candidates."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

from .approach_distance import approach_level_from_motion


@dataclass(frozen=True)
class GoalNavigationConfig:
    """Provisional goal alignment and scoring thresholds."""

    min_confidence: float = 0.55
    control_start_depth_m: float = 2.0
    score_target_depth_m: float = 0.795
    score_depth_tolerance_m: float = 0.025
    score_left_bound_px: float = -40.0
    score_right_bound_px: float = 100.0


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
        centered = self.config.score_left_bound_px <= offset_px <= self.config.score_right_bound_px
        depth_error = depth - self.config.score_target_depth_m
        depth_in_range = (
            self.config.score_target_depth_m - self.config.score_depth_tolerance_m
            <= depth
            <= self.config.score_target_depth_m + self.config.score_depth_tolerance_m
        )
        ready_geometry = centered and depth_in_range
        analyzer_score_now = goal_info.get("score_now")
        score_now = bool(
            ready_geometry
            and analyzer_score_now is True
        )

        if score_now:
            action = "SHOT"
            reason = "goal_centered_at_scoring_depth"
        elif depth < self.config.score_target_depth_m - self.config.score_depth_tolerance_m:
            action = "GOAL_CAMERA90_BACKWARD_1"
            reason = "retreat_goal_to_scoring_depth"
        elif depth_in_range and not centered:
            # During approach, the mission planner corrects yaw before advancing.
            # Reserve lateral steps for final alignment at scoring depth.
            direction = "RIGHT" if offset_px > self.config.score_right_bound_px else "LEFT"
            action = f"GOAL_CAMERA90_CRAB_{direction}"
            reason = "align_goal_lateral_camera90"
        elif ready_geometry:
            action = "WAIT_SCORE_CONFIRMATION"
            reason = "waiting_for_stable_score_condition"
        elif depth_error > self.config.score_depth_tolerance_m:
            # Use camera Depth Z directly; ground distance is a different metric.
            if depth > 1.280:
                action = "GOAL_CAMERA_90_FORWARD"
            elif depth > 1.025:
                action = "GOAL_CAMERA_90_FORWARD_2"
            elif depth > 0.985:
                action = "GOAL_CAMERA90_FINE_FORWARD_4"
            elif depth > 0.950:
                action = "GOAL_CAMERA90_FINE_FORWARD_3"
            elif depth > 0.875:
                action = "GOAL_CAMERA90_FINE_FORWARD_2"
            else:
                action = "GOAL_CAMERA90_FINE_FORWARD_1"
            reason = "approach_goal_by_depth"
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
        )
