#!/usr/bin/env python3
"""Convert hurdle geometry into SDK-oriented jump action candidates."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Any

from .approach_distance import approach_level_from_motion
from .approach_distance import HURDLE_FINE_DISTANCE_M
from .stationary_turns import stationary_turn_repeat_count
from .line_navigation_planner import LINE_RECOVERY_MIN_ANGLE_DEG, LINE_RECOVERY_MAX_ANGLE_DEG


HURDLE_HEAD_DOWN_BOTTOM_DISTANCE_PX = 120
HURDLE_ALIGNMENT_MAX_DEPTH_AGE_SEC = 0.7
# Screen-edge evidence for interpreting the first missing near observation.
HURDLE_LOSS_EDGE_DISTANCE_PX = 100.0


@dataclass(frozen=True)
class HurdleNavigationConfig:
    """Provisional hurdle alignment and jump thresholds."""

    min_confidence: float = 0.60
    control_start_depth_m: float = 1.0
    go_target_depth_m: float = 0.10
    go_depth_tolerance_m: float = 0.10
    final_sequence_start_depth_m: float = HURDLE_FINE_DISTANCE_M
    normal_turn_stop_depth_m: float = 0.450
    go_angle_tolerance_deg: float = 10.0
    path_center_tolerance_norm: float = 0.10
    close_turn_stop_bottom_distance_px: float = 100.0
    positioning_turn_min_angle_deg: float = 70.0
    center_turn_min_angle_deg: float = 15.0
    lost_side_deadband_norm: float = 0.10


@dataclass(frozen=True)
class HurdleActionCommand:
    """One abstract hurdle action; it does not drive robot hardware."""

    valid: bool
    action: str
    reason: str
    sdk_motion_requested: bool
    confidence: float
    depth_m: float | None
    distance_m: float | None
    ground_gap_m: float | None
    camera_bottom_gap_m: float | None
    ground_gap_error_m: float | None
    hurdle_angle_deg: float | None
    is_parallel: bool
    ground_gap_in_go_range: bool
    go_now: bool
    bottom_distance_px: float | None = None
    close_rotation_blocked: bool = False
    depth_fallback_requested: bool = False
    center_steering_deg: float | None = None
    fine_sequence_requested: bool = False
    recovery_active: bool = False
    last_seen_side: str | None = None
    turn_count: int | None = None
    alignment_reference: str = "none"
    near_control: bool = False

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
            # Legacy distance keys carry raw Depth Z for compatibility.
            "distance_m": _round_optional(self.depth_m, 3),
            "ground_gap_m": _round_optional(self.depth_m, 3),
            "camera_bottom_gap_m": _round_optional(
                self.camera_bottom_gap_m,
                3,
            ),
            "ground_gap_error_m": _round_optional(
                self.ground_gap_error_m,
                3,
            ),
            "hurdle_angle_deg": _round_optional(
                self.hurdle_angle_deg,
                3,
            ),
            "center_steering_deg": _round_optional(self.center_steering_deg, 3),
            "alignment_reference": self.alignment_reference,
            "hurdle_near_control": self.near_control,
            "hurdle_search_until_visible": self.reason.startswith("hurdle_top_search"),
            "is_parallel": self.is_parallel,
            "ground_gap_in_go_range": self.ground_gap_in_go_range,
            "go_now": self.go_now,
            "bottom_distance_px": self.bottom_distance_px,
            "close_rotation_blocked": self.close_rotation_blocked,
            "depth_fallback_requested": self.depth_fallback_requested,
            "fine_sequence_requested": self.fine_sequence_requested,
            "recovery_active": self.recovery_active,
            "last_seen_side": self.last_seen_side,
            **({"turn_count": self.turn_count} if self.turn_count is not None else {}),
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


class HurdleNavigationPlanner:
    """Choose a jump SDK action from one fresh ``hurdle_info`` sample."""

    def __init__(
        self,
        config: HurdleNavigationConfig | None = None,
    ) -> None:
        self.config = config or HurdleNavigationConfig()
        self.reset()

    def reset(self) -> None:
        """Release the near-entry latch only when the hurdle mission is reset."""
        self.close_rotation_blocked = False
        self.fine_approach_active = False
        self.last_seen_side: str | None = None
        self.recovery_active = False
        self.first_near_decision = True
        self.entry_line_side: str | None = None
        self.search_direction: str | None = None

    def begin_near(self, line_side: str | None = None) -> None:
        """Keep the 580 mm crossing despite camera movement or depth rebound."""
        if not self.fine_approach_active:
            self.fine_approach_active = True
            self.first_near_decision = True
            self.entry_line_side = line_side

    def observe(self, info: dict[str, Any]) -> None:
        """Remember a confirmed screen-edge observation while a motion runs."""
        confidence = _number(info, "confidence")
        if (info.get("detected") is not True
                or info.get("confirmation_confirmed") is not True
                or confidence is None or confidence < self.config.min_confidence):
            return
        edges = info.get("hurdle_bar_clipped_edges") or []
        bottom = _number(info, "bottom_distance_px")
        top = _number(info, "center_y")
        if "bottom" in edges or bottom is not None and 0 <= bottom <= HURDLE_LOSS_EDGE_DISTANCE_PX:
            self.last_seen_side = "BOTTOM"
        elif "top" in edges or top is not None and 0 <= top <= HURDLE_LOSS_EDGE_DISTANCE_PX:
            self.last_seen_side = "TOP"
        else:
            self.last_seen_side = None
        self.search_direction = None
        self.recovery_active = False

    def record_published(self, action: str, reason: str) -> None:
        """Consume first-decision recovery only after a command is dispatched."""
        if not self.fine_approach_active:
            return
        if reason.startswith("hurdle_top_search"):
            self.search_direction = "RIGHT" if action == "ALIGN_RIGHT" else "LEFT"
            self.recovery_active = True
        self.first_near_decision = False

    @staticmethod
    def _calibrated_angle(info: dict[str, Any]) -> float | None:
        """Select the robot-frame bar angle for the settled camera posture."""
        pose = info.get("hurdle_camera_pose")
        if info.get("hurdle_camera_ready") is not True or pose not in ("45", "0"):
            return None
        angle = _number(info, f"hurdle_angle_{pose}_deg")
        age = _number(info, "depth_age_sec")
        samples = _number(info, "hurdle_angle_fit_point_count")
        if (info.get("confirmation_confirmed") is not True
                or angle is None or abs(angle) >= 90.0
                or age is None or not 0.0 <= age <= HURDLE_ALIGNMENT_MAX_DEPTH_AGE_SEC
                or samples is None or samples < 2
                or info.get("hurdle_angle_sample_source") != "yellow_bar_centerline"):
            return None
        return angle

    def wait(self, reason: str) -> HurdleActionCommand:
        """Return a non-action command for missing or unsafe input."""
        return HurdleActionCommand(
            valid=False,
            action="WAIT",
            reason=reason,
            sdk_motion_requested=False,
            confidence=0.0,
            depth_m=None,
            distance_m=None,
            ground_gap_m=None,
            camera_bottom_gap_m=None,
            ground_gap_error_m=None,
            hurdle_angle_deg=None,
            is_parallel=False,
            ground_gap_in_go_range=False,
            go_now=False,
            close_rotation_blocked=self.close_rotation_blocked,
            recovery_active=self.recovery_active,
            last_seen_side=self.last_seen_side,
        )

    def plan(
        self, hurdle_info: dict[str, Any], *, positioning: bool = False,
    ) -> HurdleActionCommand:
        """Use hurdle yaw for far walking and camera-zero alignment after entry."""
        self.observe(hurdle_info)
        depth = _number(hurdle_info, "depth_m")
        confidence = _number(hurdle_info, "confidence")
        detected = hurdle_info.get("detected") is True
        confirmed = hurdle_info.get("confirmation_confirmed") is True
        valid_depth = hurdle_info.get("depth_valid") is True and depth is not None and depth > 0
        if (detected and confirmed and valid_depth
                and confidence is not None and confidence >= self.config.min_confidence
                and depth <= HURDLE_FINE_DISTANCE_M):
            self.begin_near(hurdle_info.get("line_robot_side"))
        near = self.fine_approach_active
        if near and (hurdle_info.get("hurdle_camera_ready") is not True
                     or hurdle_info.get("hurdle_camera_pose") != "0"):
            return self.wait("waiting_for_hurdle_camera_zero_frame")
        if not detected:
            if hurdle_info.get("raw_detected") is True:
                return self.wait("hurdle_confirmation_pending")
            if hurdle_info.get("detected") is not False or not near:
                return self.wait("hurdle_not_detected")
            if self.first_near_decision and self.last_seen_side == "BOTTOM":
                return replace(self.wait("hurdle_first_near_lost_below_image"),
                               valid=True, action="HURDLE_LOST_BACKWARD_2", near_control=True)
            direction = self.search_direction
            initial_search = direction is None
            if self.first_near_decision and self.last_seen_side == "TOP":
                direction = hurdle_info.get("line_robot_side") or self.entry_line_side
            if direction in {"LEFT", "RIGHT"}:
                count = (3 if direction == "RIGHT" else 2) if initial_search else 1
                return replace(self.wait("hurdle_top_search_initial" if initial_search
                                         else "hurdle_top_search_continue"),
                               valid=True, action=f"ALIGN_{direction}", turn_count=count,
                               recovery_active=True, near_control=True)
            return self.wait("hurdle_loss_direction_unavailable")
        if not confirmed:
            return self.wait("hurdle_confirmation_pending")
        if confidence is None or confidence < self.config.min_confidence:
            return self.wait("low_hurdle_confidence")
        if not valid_depth:
            return self.wait("missing_valid_hurdle_depth")
        if not near and depth > self.config.control_start_depth_m:
            return self.wait("hurdle_outside_control_range")
        angle = self._calibrated_angle(hurdle_info)
        if angle is None:
            return self.wait("waiting_for_fresh_hurdle_calibrated_angle")
        heading = -angle
        magnitude = abs(heading)
        direction = "RIGHT" if heading > 0 else "LEFT"
        count = None
        go = False
        if near:
            if magnitude > self.config.go_angle_tolerance_deg:
                action = f"ALIGN_{direction}"
                # The new >10-degree trigger also covers the old right-turn gap (10, 15).
                count = max(1, stationary_turn_repeat_count(heading, direction))
                reason = "hurdle_near_camera_zero_alignment"
            else:
                action, reason, go = "GO", "hurdle_near_aligned_fixed_sequence", True
        elif magnitude < LINE_RECOVERY_MIN_ANGLE_DEG:
            action, reason = "STRAIGHT_3", "hurdle_far_forward_six"
        elif magnitude <= LINE_RECOVERY_MAX_ANGLE_DEG:
            action = f"RECOVER_{direction}_TURN_{direction}_4"
            reason = "hurdle_far_recovery_by_yaw"
        else:
            action = f"ALIGN_{direction}"
            count = stationary_turn_repeat_count(heading, direction)
            reason = "hurdle_far_stationary_alignment"
        return replace(
            self.wait(reason), valid=True, action=action, sdk_motion_requested=go,
            confidence=confidence, depth_m=depth, distance_m=depth, ground_gap_m=depth,
            ground_gap_error_m=depth - HURDLE_FINE_DISTANCE_M,
            hurdle_angle_deg=angle, center_steering_deg=heading,
            is_parallel=magnitude <= self.config.go_angle_tolerance_deg,
            ground_gap_in_go_range=near, go_now=go, fine_sequence_requested=go,
            bottom_distance_px=_number(hurdle_info, "bottom_distance_px"),
            turn_count=count, near_control=near,
            alignment_reference=f"hurdle_angle_{hurdle_info['hurdle_camera_pose']}",
        )
