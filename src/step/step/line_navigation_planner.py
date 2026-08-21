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
    line_center_offset_tolerance: float = 0.20
    line_large_offset_threshold: float = 0.28
    line_heading_tolerance_deg: float = 7.0
    line_large_heading_threshold_deg: float = 18.0
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
    turn_enter_deg: float = 12.0
    turn_exit_deg: float = 7.0
    direction_confirmation_frames: int = 5
    ambiguity_min_angle_deg: float = 25.0
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


def _normalized_severity(
    value: float,
    tolerance: float,
    threshold: float,
) -> float:
    span = threshold - tolerance
    if span <= 0.0:
        return 1.0 if abs(value) > tolerance else 0.0
    return _clamp((abs(value) - tolerance) / span, 0.0, 1.0)


def _fine_turn_severity(
    config: NavigationConfig,
    lateral_offset_norm: float,
    heading_error_deg: float,
    steering_error_deg: float,
) -> float:
    """Return the strongest normalized fine-turn correction demand."""
    return max(
        _normalized_severity(
            lateral_offset_norm,
            config.line_center_offset_tolerance,
            config.line_large_offset_threshold,
        ),
        _normalized_severity(
            heading_error_deg,
            config.line_heading_tolerance_deg,
            config.line_large_heading_threshold_deg,
        ),
        _normalized_severity(
            steering_error_deg,
            config.turn_exit_deg,
            config.line_large_heading_threshold_deg,
        ),
    )


def _fine_turn_repeat_count(severity: float) -> int:
    """Map normalized fine-turn severity to an SDK sequence level."""
    severity = round(_clamp(severity, 0.0, 1.0), 12)
    if severity < 0.2:
        return 2
    if severity < 0.4:
        return 4
    if severity < 0.6:
        return 6
    if severity < 0.8:
        return 8
    return 10


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
        self.turn_candidate: str | None = None
        self.turn_candidate_hits = 0
        self.active_correction_direction: str | None = None
        self.last_valid_line_offset: float | None = None
        self.last_valid_line_heading: float | None = None
        self.line_lost_frames = 0
        self.line_recovery_attempts = 0
        self.recovering_line = False

    def stop(self, reason: str) -> NavigationCommand:
        """Create an immediate stop and reset steering state."""
        self.previous_motion = "STOP"
        self.previous_angular_speed_rad_s = 0.0
        self._reset_turn_state()
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

        preview_turn = _number(line_info, "turn_angle_deg")
        turn_consistency = _number(line_info, "turn_consistency")
        preview_is_reliable = (
            preview_turn is not None
            and abs(preview_turn) >= self.config.preview_min_turn_deg
            and turn_consistency is not None
            and turn_consistency >= self.config.preview_min_consistency
        )
        preview_component = preview_turn if preview_is_reliable else 0.0
        direction_is_ambiguous = bool(
            preview_is_reliable
            and abs(heading) >= self.config.ambiguity_min_angle_deg
            and abs(preview_turn) >= self.config.ambiguity_min_angle_deg
            and heading * preview_turn < 0.0
        )
        heading_component = self.config.heading_gain * heading
        offset_component = self.config.offset_gain_deg * offset
        preview_component = self.config.preview_gain * preview_component
        steering_error = (
            heading_component
            + offset_component
            + preview_component
        )
        if direction_is_ambiguous:
            steering_error = 0.0

        max_steering_deg = math.degrees(
            self.config.max_angular_speed_rad_s
            * self.config.steering_response_sec
        )
        steering_error = _clamp(
            steering_error,
            -max_steering_deg,
            max_steering_deg,
        )

        recovery_motion = self._classify_recovery(offset)
        requested_motion = recovery_motion or self._classify_motion(
            steering_error,
            heading,
            offset,
        )
        if direction_is_ambiguous:
            self._reset_turn_state()
            requested_motion = "STRAIGHT"
        base_motion = self._confirm_motion(requested_motion)
        turn_confirmation_pending = bool(
            base_motion == "STRAIGHT"
            and requested_motion
            in {"FINE_LEFT", "FINE_RIGHT", "LEFT", "RIGHT"}
        )
        if (
            recovery_motion is not None
            and not direction_is_ambiguous
            and not turn_confirmation_pending
        ):
            return self._recovery_command(
                base_motion,
                heading,
                offset,
                quality,
            )
        control_steering_error = (
            0.0
            if direction_is_ambiguous or turn_confirmation_pending
            else steering_error
        )

        desired_angular_speed = math.radians(control_steering_error) / max(
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

        motion = base_motion
        speed = self._calculate_linear_speed(steering_error, quality)
        duration = self.config.command_duration_sec

        fine_turn_fallback = (
            base_motion in {"FINE_LEFT", "FINE_RIGHT"}
            and not self.config.fine_turn_supported
        )
        if direction_is_ambiguous:
            motion = "STRAIGHT"
            angular_speed = 0.0
            angular_accel = 0.0
            speed = self.config.min_linear_speed_mps
            reason = "conflicting_heading_and_preview"
        elif turn_confirmation_pending:
            angular_speed = 0.0
            angular_accel = 0.0
            speed = self.config.min_linear_speed_mps
            reason = "turn_confirmation_pending"
        elif fine_turn_fallback:
            motion = "STRAIGHT"
            angular_speed = 0.0
            angular_accel = 0.0
            reason = "fine_turn_unavailable_straight_fallback"
        else:
            if base_motion in {"FINE_LEFT", "FINE_RIGHT"}:
                severity = _fine_turn_severity(
                    self.config,
                    offset,
                    heading,
                    steering_error,
                )
                repeat_count = _fine_turn_repeat_count(severity)
                motion = f"{base_motion}_{repeat_count}"
            reason = "line_tracking"

        if not turn_confirmation_pending:
            self.previous_motion = (
                base_motion if not fine_turn_fallback else motion
            )
        self.previous_angular_speed_rad_s = angular_speed

        return NavigationCommand(
            valid=True,
            motion=motion,
            reason=reason,
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
        threshold = (
            self.config.turn_exit_deg
            if self.previous_motion
            in {"FINE_LEFT", "FINE_RIGHT", "LEFT", "RIGHT"}
            else self.config.turn_enter_deg
        )
        if (
            abs(heading_error_deg)
            >= self.config.line_large_heading_threshold_deg
        ):
            return "RIGHT" if heading_error_deg > 0.0 else "LEFT"
        if steering_error_deg >= threshold:
            return "FINE_RIGHT"
        if steering_error_deg <= -threshold:
            return "FINE_LEFT"
        return "STRAIGHT"

    @staticmethod
    def _motion_direction(motion: str) -> str | None:
        if motion in {"FINE_LEFT", "LEFT"}:
            return "LEFT"
        if motion in {"FINE_RIGHT", "RIGHT"}:
            return "RIGHT"
        return None

    def _confirm_motion(self, requested_motion: str) -> str:
        """Confirm only a new or reversed correction direction."""
        direction = self._motion_direction(requested_motion)
        if direction is None:
            self._reset_turn_state()
            return requested_motion
        if direction == self.active_correction_direction:
            self.turn_candidate = None
            self.turn_candidate_hits = 0
            return requested_motion

        if direction == self.turn_candidate:
            self.turn_candidate_hits += 1
        else:
            self.turn_candidate = direction
            self.turn_candidate_hits = 1

        if self.turn_candidate_hits >= max(
            1, self.config.direction_confirmation_frames
        ):
            self.active_correction_direction = direction
            self.turn_candidate = None
            self.turn_candidate_hits = 0
            return requested_motion
        return "STRAIGHT"

    def _reset_turn_state(self) -> None:
        self.turn_candidate = None
        self.turn_candidate_hits = 0
        self.active_correction_direction = None

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
