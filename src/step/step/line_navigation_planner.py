#!/usr/bin/env python3
"""Convert line geometry into bounded, hardware-independent motion targets."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any


@dataclass(frozen=True)
class NavigationConfig:
    """Tunable limits and gains for line-following command generation."""

    min_line_quality: float = 0.35
    line_center_offset_tolerance: float = 0.12
    line_large_offset_threshold: float = 0.28
    line_heading_tolerance_deg: float = 4.0
    line_large_heading_threshold_deg: float = 14.0
    line_lost_frame_threshold: int = 2
    line_max_recovery_attempts: int = 3
    fine_turn_supported: bool = False
    max_linear_speed_mps: float = 0.05
    min_linear_speed_mps: float = 0.015
    max_angular_speed_rad_s: float = 0.60
    max_angular_accel_rad_s2: float = 1.20
    heading_gain: float = 1.0
    offset_gain_deg: float = 24.0
    preview_gain: float = 0.15
    preview_min_turn_deg: float = 8.0
    preview_min_consistency: float = 0.55
    steering_response_sec: float = 0.70
    turn_enter_deg: float = 7.0
    turn_exit_deg: float = 4.0
    command_duration_sec: float = 0.40


@dataclass(frozen=True)
class NavigationCommand:
    """One abstract command for the behavior or walking algorithm."""

    valid: bool
    motion: str
    reason: str
    linear_speed_mps: float
    lateral_speed_mps: float
    angular_speed_rad_s: float
    angular_accel_rad_s2: float
    command_duration_sec: float
    travel_distance_m: float
    lateral_travel_distance_m: float
    target_heading_change_deg: float
    steering_error_deg: float
    heading_component_deg: float
    offset_component_deg: float
    preview_component_deg: float
    heading_error_deg: float | None
    lateral_offset_norm: float | None
    preview_turn_deg: float | None
    line_quality: float

    def to_dict(self) -> dict[str, Any]:
        """Return a rounded JSON-compatible representation."""
        return {
            "valid": self.valid,
            "motion": self.motion,
            "reason": self.reason,
            "linear_speed_mps": round(self.linear_speed_mps, 4),
            "lateral_speed_mps": round(self.lateral_speed_mps, 4),
            "angular_speed_rad_s": round(self.angular_speed_rad_s, 4),
            "angular_accel_rad_s2": round(self.angular_accel_rad_s2, 4),
            "command_duration_sec": round(self.command_duration_sec, 3),
            "travel_distance_m": round(self.travel_distance_m, 4),
            "lateral_travel_distance_m": round(
                self.lateral_travel_distance_m,
                4,
            ),
            "target_heading_change_deg": round(
                self.target_heading_change_deg, 3
            ),
            "steering_error_deg": round(self.steering_error_deg, 3),
            "heading_component_deg": round(self.heading_component_deg, 3),
            "offset_component_deg": round(self.offset_component_deg, 3),
            "preview_component_deg": round(self.preview_component_deg, 3),
            "heading_error_deg": _round_optional(self.heading_error_deg, 3),
            "lateral_offset_norm": _round_optional(
                self.lateral_offset_norm, 6
            ),
            "preview_turn_deg": _round_optional(self.preview_turn_deg, 3),
            "line_quality": round(self.line_quality, 4),
        }


def _round_optional(value: float | None, digits: int) -> float | None:
    if value is None:
        return None
    return round(value, digits)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _number(data: dict[str, Any], key: str) -> float | None:
    value = data.get(key)
    if value is None or isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


class LineNavigationPlanner:
    """Create smooth line-following setpoints from ``/vision/line_info``."""

    def __init__(self, config: NavigationConfig | None = None) -> None:
        self.config = config or NavigationConfig()
        self.previous_motion = "STOP"
        self.previous_angular_speed_rad_s = 0.0
        self.last_valid_line_offset: float | None = None
        self.last_valid_line_heading: float | None = None
        self.line_lost_frames = 0
        self.line_recovery_attempts = 0
        self.recovering_line = False

    def stop(self, reason: str) -> NavigationCommand:
        """Create an immediate stop and reset steering state."""
        self.previous_motion = "STOP"
        self.previous_angular_speed_rad_s = 0.0
        return NavigationCommand(
            valid=False,
            motion="STOP",
            reason=reason,
            linear_speed_mps=0.0,
            lateral_speed_mps=0.0,
            angular_speed_rad_s=0.0,
            angular_accel_rad_s2=0.0,
            command_duration_sec=self.config.command_duration_sec,
            travel_distance_m=0.0,
            lateral_travel_distance_m=0.0,
            target_heading_change_deg=0.0,
            steering_error_deg=0.0,
            heading_component_deg=0.0,
            offset_component_deg=0.0,
            preview_component_deg=0.0,
            heading_error_deg=None,
            lateral_offset_norm=None,
            preview_turn_deg=None,
            line_quality=0.0,
        )

    def plan(
        self,
        line_info: dict[str, Any] | None,
        dt_sec: float,
    ) -> NavigationCommand:
        """Generate one bounded command from a fresh line analysis sample."""
        if line_info is None or not bool(line_info.get("detected", False)):
            return self._handle_missing_line()

        heading = _number(line_info, "filtered_heading_error_deg")
        if heading is None:
            heading = _number(line_info, "heading_error_deg")

        offset = _number(line_info, "filtered_lateral_offset_norm")
        if offset is None:
            offset = _number(line_info, "lateral_offset_norm")

        if heading is None or offset is None:
            return self._handle_missing_line("invalid_line_geometry")

        qualities = [
            _number(line_info, "heading_quality"),
            _number(line_info, "geometry_quality"),
            _number(line_info, "detection_quality"),
        ]
        valid_qualities = [value for value in qualities if value is not None]
        if not valid_qualities:
            return self.stop("invalid_line_quality")
        quality = min(valid_qualities)
        quality = _clamp(quality, 0.0, 1.0)
        if quality < self.config.min_line_quality:
            return self._handle_missing_line("low_line_quality")

        self.last_valid_line_offset = offset
        self.last_valid_line_heading = heading
        self.line_lost_frames = 0
        if (
            abs(offset) <= self.config.line_center_offset_tolerance
            and abs(heading) <= self.config.line_heading_tolerance_deg
        ):
            self.recovering_line = False
            self.line_recovery_attempts = 0

        recovery_motion = self._classify_recovery(offset)
        if recovery_motion is not None:
            return self._recovery_command(
                recovery_motion,
                heading,
                offset,
                quality,
            )

        preview_turn = _number(line_info, "turn_angle_deg")
        turn_consistency = _number(line_info, "turn_consistency")
        preview_is_reliable = (
            preview_turn is not None
            and abs(preview_turn) >= self.config.preview_min_turn_deg
            and turn_consistency is not None
            and turn_consistency >= self.config.preview_min_consistency
        )
        preview_component = preview_turn if preview_is_reliable else 0.0
        heading_component = self.config.heading_gain * heading
        offset_component = self.config.offset_gain_deg * offset
        preview_component = self.config.preview_gain * preview_component
        steering_error = (
            heading_component
            + offset_component
            + preview_component
        )

        max_steering_deg = math.degrees(
            self.config.max_angular_speed_rad_s
            * self.config.steering_response_sec
        )
        steering_error = _clamp(
            steering_error,
            -max_steering_deg,
            max_steering_deg,
        )

        desired_angular_speed = math.radians(steering_error) / max(
            self.config.steering_response_sec,
            1e-3,
        )
        desired_angular_speed = _clamp(
            desired_angular_speed,
            -self.config.max_angular_speed_rad_s,
            self.config.max_angular_speed_rad_s,
        )

        dt_sec = _clamp(dt_sec, 1e-3, 1.0)
        max_delta = self.config.max_angular_accel_rad_s2 * dt_sec
        angular_delta = _clamp(
            desired_angular_speed - self.previous_angular_speed_rad_s,
            -max_delta,
            max_delta,
        )
        angular_speed = self.previous_angular_speed_rad_s + angular_delta
        angular_accel = angular_delta / dt_sec

        motion = self._classify_motion(steering_error, heading, offset)
        speed = self._calculate_linear_speed(steering_error, quality)
        duration = self.config.command_duration_sec

        fine_turn_unmapped = (
            motion in {"FINE_LEFT", "FINE_RIGHT"}
            and not self.config.fine_turn_supported
        )
        self.previous_motion = motion
        self.previous_angular_speed_rad_s = angular_speed

        return NavigationCommand(
            valid=not fine_turn_unmapped,
            motion=motion,
            reason=(
                "fine_turn_motion_unmapped"
                if fine_turn_unmapped
                else "line_tracking"
            ),
            linear_speed_mps=speed,
            lateral_speed_mps=0.0,
            angular_speed_rad_s=angular_speed,
            angular_accel_rad_s2=angular_accel,
            command_duration_sec=duration,
            travel_distance_m=speed * duration,
            lateral_travel_distance_m=0.0,
            target_heading_change_deg=math.degrees(angular_speed * duration),
            steering_error_deg=steering_error,
            heading_component_deg=heading_component,
            offset_component_deg=offset_component,
            preview_component_deg=preview_component,
            heading_error_deg=heading,
            lateral_offset_norm=offset,
            preview_turn_deg=preview_turn,
            line_quality=quality,
        )

    def _classify_recovery(self, lateral_offset_norm: float) -> str | None:
        """Return a general turn only for excessive lateral offset."""
        is_recovering = self.previous_motion in {
            "LEFT",
            "RIGHT",
        }
        threshold = (
            self.config.line_center_offset_tolerance
            if is_recovering
            else self.config.line_large_offset_threshold
        )
        if lateral_offset_norm > threshold:
            return "RIGHT"
        if lateral_offset_norm < -threshold:
            return "LEFT"
        return None

    def _recovery_command(
        self,
        motion: str,
        heading: float,
        offset: float,
        quality: float,
        reason: str = "line_large_deviation_turn",
    ) -> NavigationCommand:
        """Create a bounded general turn toward the remembered line."""
        direction = 1.0 if motion == "RIGHT" else -1.0
        angular_speed = direction * self.config.max_angular_speed_rad_s
        duration = self.config.command_duration_sec
        self.previous_motion = motion
        self.previous_angular_speed_rad_s = angular_speed
        return NavigationCommand(
            valid=True,
            motion=motion,
            reason=reason,
            linear_speed_mps=0.0,
            lateral_speed_mps=0.0,
            angular_speed_rad_s=angular_speed,
            angular_accel_rad_s2=0.0,
            command_duration_sec=duration,
            travel_distance_m=0.0,
            lateral_travel_distance_m=0.0,
            target_heading_change_deg=math.degrees(
                angular_speed * duration
            ),
            steering_error_deg=0.0,
            heading_component_deg=heading,
            offset_component_deg=self.config.offset_gain_deg * offset,
            preview_component_deg=0.0,
            heading_error_deg=heading,
            lateral_offset_norm=offset,
            preview_turn_deg=None,
            line_quality=quality,
        )

    def _classify_motion(
        self,
        steering_error_deg: float,
        heading_error_deg: float,
        lateral_offset_norm: float,
    ) -> str:
        """Keep straight inside tolerance and correct moderate deviations."""
        if (
            abs(lateral_offset_norm)
            <= self.config.line_center_offset_tolerance
            and abs(heading_error_deg)
            <= self.config.line_heading_tolerance_deg
            and abs(steering_error_deg) <= self.config.turn_enter_deg
        ):
            return "STRAIGHT"

        threshold = (
            self.config.turn_exit_deg
            if self.previous_motion in {"FINE_LEFT", "FINE_RIGHT"}
            else self.config.turn_enter_deg
        )
        if abs(lateral_offset_norm) > self.config.line_center_offset_tolerance:
            if steering_error_deg > 0.0:
                return "FINE_RIGHT"
            if steering_error_deg < 0.0:
                return "FINE_LEFT"
            return (
                "FINE_RIGHT"
                if lateral_offset_norm > 0.0
                else "FINE_LEFT"
            )
        if (
            abs(heading_error_deg)
            >= self.config.line_large_heading_threshold_deg
        ):
            return "RIGHT" if heading_error_deg > 0.0 else "LEFT"
        if abs(heading_error_deg) > self.config.line_heading_tolerance_deg:
            return (
                "FINE_RIGHT"
                if heading_error_deg > 0.0
                else "FINE_LEFT"
            )
        if steering_error_deg > threshold:
            return "FINE_RIGHT"
        if steering_error_deg < -threshold:
            return "FINE_LEFT"
        return "STRAIGHT"

    def _handle_missing_line(
        self,
        initial_reason: str = "line_not_detected",
    ) -> NavigationCommand:
        """Wait through a short dropout, then recover from remembered geometry."""
        self.line_lost_frames += 1
        threshold = max(1, self.config.line_lost_frame_threshold)
        if self.line_lost_frames < threshold:
            return self.stop(initial_reason)

        recovery_motion = self._remembered_recovery_motion()
        if recovery_motion is None:
            self.recovering_line = False
            return self.stop("line_lost_without_history")

        self.recovering_line = True
        if (
            self.line_recovery_attempts
            >= max(0, self.config.line_max_recovery_attempts)
        ):
            return self.stop("line_recovery_attempts_exhausted")

        self.line_recovery_attempts += 1
        return self._recovery_command(
            recovery_motion,
            self.last_valid_line_heading or 0.0,
            self.last_valid_line_offset or 0.0,
            0.0,
            reason="line_lost_recovery",
        )

    def _remembered_recovery_motion(self) -> str | None:
        """Choose recovery direction from offset first, then heading."""
        offset = self.last_valid_line_offset
        if (
            offset is not None
            and abs(offset) > self.config.line_center_offset_tolerance
        ):
            return "RIGHT" if offset > 0.0 else "LEFT"

        heading = self.last_valid_line_heading
        if (
            heading is not None
            and abs(heading) > self.config.line_heading_tolerance_deg
        ):
            return "RIGHT" if heading > 0.0 else "LEFT"
        return None

    def _calculate_linear_speed(
        self,
        steering_error_deg: float,
        quality: float,
    ) -> float:
        """Slow down for large turns and uncertain line geometry."""
        max_steering_deg = max(
            math.degrees(
                self.config.max_angular_speed_rad_s
                * self.config.steering_response_sec
            ),
            1e-3,
        )
        turn_scale = 1.0 - 0.70 * min(
            abs(steering_error_deg) / max_steering_deg,
            1.0,
        )
        quality_scale = 0.50 + 0.50 * quality
        speed = self.config.max_linear_speed_mps * turn_scale * quality_scale
        return _clamp(
            speed,
            self.config.min_linear_speed_mps,
            self.config.max_linear_speed_mps,
        )
