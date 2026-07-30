"""Unit tests for hardware-independent line navigation decisions."""

import pytest

from step.line_navigation_planner import LineNavigationPlanner
from step.line_navigation_planner import NavigationConfig


def line_info(**overrides):
    """Create a valid line analysis sample with optional changes."""
    sample = {
        "detected": True,
        "filtered_heading_error_deg": 0.0,
        "filtered_lateral_offset_norm": 0.0,
        "turn_angle_deg": 0.0,
        "turn_consistency": 0.9,
        "heading_quality": 0.9,
        "geometry_quality": 0.8,
        "detection_quality": 0.85,
    }
    sample.update(overrides)
    return sample


def test_straight_command_contains_speed_and_distance():
    planner = LineNavigationPlanner()

    command = planner.plan(line_info(), 0.1)

    assert command.valid is True
    assert command.motion == "STRAIGHT"
    assert command.linear_speed_mps > 0.0
    assert command.travel_distance_m == pytest.approx(
        command.linear_speed_mps * command.command_duration_sec
    )


@pytest.mark.parametrize(
    ("heading", "expected"),
    [(14.0, "RIGHT"), (-14.0, "LEFT")],
)
def test_heading_sign_selects_turn_direction(heading, expected):
    planner = LineNavigationPlanner()

    command = planner.plan(
        line_info(filtered_heading_error_deg=heading),
        0.1,
    )

    assert command.motion == expected


def test_offset_and_preview_are_used_for_steering():
    planner = LineNavigationPlanner()

    command = planner.plan(
        line_info(
            filtered_lateral_offset_norm=0.1,
            turn_angle_deg=40.0,
        ),
        0.1,
    )

    assert command.motion == "FINE_RIGHT"
    assert command.valid is False
    assert command.reason == "fine_turn_motion_unmapped"
    assert command.steering_error_deg > 7.0


@pytest.mark.parametrize(
    ("offset", "expected", "lateral_sign"),
    [
        (0.35, "RIGHT", 1),
        (-0.35, "LEFT", -1),
    ],
)
def test_large_offset_creates_separate_recovery_command(
    offset,
    expected,
    lateral_sign,
):
    planner = LineNavigationPlanner()

    command = planner.plan(
        line_info(filtered_lateral_offset_norm=offset),
        0.1,
    )

    assert command.motion == expected
    assert command.reason == "line_large_deviation_turn"
    assert command.linear_speed_mps == 0.0
    assert command.lateral_speed_mps == 0.0
    assert command.angular_speed_rad_s * lateral_sign > 0.0
    assert command.target_heading_change_deg * lateral_sign > 0.0


def test_recovery_uses_exit_threshold_before_returning_to_tracking():
    planner = LineNavigationPlanner()

    first = planner.plan(
        line_info(filtered_lateral_offset_norm=0.35),
        0.1,
    )
    held = planner.plan(
        line_info(filtered_lateral_offset_norm=0.20),
        0.1,
    )
    released = planner.plan(
        line_info(filtered_lateral_offset_norm=0.10),
        0.1,
    )

    assert first.motion == "RIGHT"
    assert held.motion == "RIGHT"
    assert released.motion == "STRAIGHT"


def test_small_or_inconsistent_far_curve_is_not_used_early():
    planner = LineNavigationPlanner()

    small_curve = planner.plan(
        line_info(turn_angle_deg=7.9, turn_consistency=1.0),
        0.1,
    )
    inconsistent_curve = LineNavigationPlanner().plan(
        line_info(turn_angle_deg=30.0, turn_consistency=0.2),
        0.1,
    )

    assert small_curve.preview_component_deg == 0.0
    assert inconsistent_curve.preview_component_deg == 0.0


def test_angular_acceleration_is_limited():
    config = NavigationConfig(max_angular_accel_rad_s2=0.5)
    planner = LineNavigationPlanner(config)

    command = planner.plan(
        line_info(filtered_heading_error_deg=40.0),
        0.1,
    )

    assert command.angular_accel_rad_s2 == pytest.approx(0.5)
    assert command.angular_speed_rad_s == pytest.approx(0.05)


@pytest.mark.parametrize(
    ("sample", "reason"),
    [
        ({"detected": False}, "line_not_detected"),
        (
            line_info(geometry_quality=0.1),
            "low_line_quality",
        ),
    ],
)
def test_unsafe_input_produces_stop(sample, reason):
    planner = LineNavigationPlanner()

    command = planner.plan(sample, 0.1)

    assert command.valid is False
    assert command.motion == "STOP"
    assert command.reason == reason
    assert command.linear_speed_mps == 0.0
