#!/usr/bin/env python3
"""Convert hurdle geometry into SDK-oriented jump action candidates."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any


@dataclass(frozen=True)
class HurdleNavigationConfig:
    """Provisional hurdle alignment and jump thresholds."""

    min_confidence: float = 0.35
    go_target_depth_m: float = 0.80
    go_depth_tolerance_m: float = 0.10
    go_center_tolerance_norm: float = 0.12


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
    depth_error_m: float | None
    bearing_error_deg: float | None
    offset_x_norm: float | None
    hurdle_angle_deg: float | None
    is_centered: bool
    depth_in_go_range: bool
    go_now: bool

    def to_dict(self) -> dict[str, Any]:
        """Return a rounded JSON-compatible representation."""
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
            "hurdle_angle_deg": _round_optional(
                self.hurdle_angle_deg,
                3,
            ),
            "is_centered": self.is_centered,
            "depth_in_go_range": self.depth_in_go_range,
            "go_now": self.go_now,
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
            depth_error_m=None,
            bearing_error_deg=None,
            offset_x_norm=None,
            hurdle_angle_deg=None,
            is_centered=False,
            depth_in_go_range=False,
            go_now=False,
        )

    def plan(self, hurdle_info: dict[str, Any]) -> HurdleActionCommand:
        """Create one alignment, distance-adjustment, or GO action."""
        if not bool(hurdle_info.get("detected", False)):
            return self.wait("hurdle_not_detected")
        confidence = _number(hurdle_info, "confidence")
        if confidence is None or confidence < self.config.min_confidence:
            return self.wait("low_hurdle_confidence")
        depth = _number(hurdle_info, "depth_m")
        if not bool(hurdle_info.get("depth_valid", False)) or depth is None:
            return self.wait("missing_valid_hurdle_depth")
        offset = _number(hurdle_info, "offset_x_norm")
        if offset is None:
            return self.wait("invalid_hurdle_alignment")

        distance = _number(hurdle_info, "distance_m")
        bearing = _number(hurdle_info, "bearing_deg")
        hurdle_angle = _number(hurdle_info, "hurdle_angle_deg")
        centered = abs(offset) <= self.config.go_center_tolerance_norm
        depth_error = depth - self.config.go_target_depth_m
        depth_in_range = (
            abs(depth_error)
            <= self.config.go_depth_tolerance_m + 1e-9
        )
        go_now = centered and depth_in_range

        if go_now:
            action = "GO"
            reason = "hurdle_centered_at_jump_depth"
        elif not centered:
            action = "ALIGN_RIGHT" if offset > 0.0 else "ALIGN_LEFT"
            reason = "align_hurdle_horizontally"
        elif depth_error > self.config.go_depth_tolerance_m:
            action = "APPROACH_HURDLE"
            reason = "hurdle_too_far"
        else:
            action = "RETREAT_HURDLE"
            reason = "hurdle_too_close"

        return HurdleActionCommand(
            valid=True,
            action=action,
            reason=reason,
            sdk_motion_requested=go_now,
            confidence=confidence,
            depth_m=depth,
            distance_m=distance,
            depth_error_m=depth_error,
            bearing_error_deg=bearing,
            offset_x_norm=offset,
            hurdle_angle_deg=hurdle_angle,
            is_centered=centered,
            depth_in_go_range=depth_in_range,
            go_now=go_now,
        )
