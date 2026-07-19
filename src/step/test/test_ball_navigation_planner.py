"""Unit tests for hardware-independent ball navigation decisions."""

import pytest

from step.ball_navigation_planner import BallNavigationConfig
from step.ball_navigation_planner import BallNavigationPlanner


def ball_info(**overrides):
    """Create one valid ball analysis sample with optional changes."""
    sample = {
        "detected": True,
        "confidence": 0.9,
        "bearing_deg": 0.0,
        "offset_x_norm": 0.0,
        "depth_m": 0.88,
        "distance_m": 0.89,
        "depth_valid": True,
        "pickup_ready": False,
        "pickup_now": False,
    }
    sample.update(overrides)
    return sample


@pytest.mark.parametrize(
    ("bearing", "expected"),
    [(12.0, "TURN_RIGHT"), (-12.0, "TURN_LEFT")],
)
def test_bearing_sign_selects_turn_direction(bearing, expected):
    planner = BallNavigationPlanner()

    command = planner.plan(ball_info(bearing_deg=bearing), 0.1)

    assert command.valid is True
    assert command.motion == expected
    assert command.linear_speed_mps == 0.0


def test_centered_ball_inside_90cm_uses_fine_step():
    planner = BallNavigationPlanner()

    command = planner.plan(ball_info(), 0.1)

    assert command.motion == "FINE_FORWARD_STEP"
    assert command.linear_speed_mps > 0.0
    assert command.travel_distance_m == pytest.approx(
        command.linear_speed_mps * command.command_duration_sec
    )


def test_close_ball_uses_fine_forward_step():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(depth_m=0.82, distance_m=0.83, pickup_ready=True),
        0.1,
    )

    assert command.motion == "FINE_FORWARD_STEP"
    assert command.linear_speed_mps == pytest.approx(
        planner.config.min_linear_speed_mps
    )


def test_aligned_pickup_distance_stops_forward_motion():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(depth_m=0.80, distance_m=0.81, pickup_now=True),
        0.1,
    )

    assert command.valid is True
    assert command.motion == "PICKUP_NOW"
    assert command.linear_speed_mps == 0.0
    assert command.pickup_now is True


def test_offset_is_used_when_camera_bearing_is_missing():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(bearing_deg=None, offset_x_norm=0.3),
        0.1,
    )

    assert command.motion == "TURN_RIGHT"


def test_ball_outside_90cm_control_range_does_not_move_robot():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(depth_m=0.91, distance_m=0.92),
        0.1,
    )

    assert command.valid is False
    assert command.motion == "STOP"
    assert command.reason == "ball_outside_control_range"


def test_turn_hysteresis_holds_until_exit_threshold():
    planner = BallNavigationPlanner()

    first = planner.plan(ball_info(bearing_deg=8.0), 0.1)
    held = planner.plan(ball_info(bearing_deg=3.0), 0.1)
    released = planner.plan(ball_info(bearing_deg=2.0), 0.1)

    assert first.motion == "TURN_RIGHT"
    assert held.motion == "TURN_RIGHT"
    assert released.motion == "FINE_FORWARD_STEP"


def test_angular_acceleration_is_limited():
    config = BallNavigationConfig(max_angular_accel_rad_s2=0.5)
    planner = BallNavigationPlanner(config)

    command = planner.plan(ball_info(bearing_deg=30.0), 0.1)

    assert command.angular_accel_rad_s2 == pytest.approx(0.5)
    assert command.angular_speed_rad_s == pytest.approx(0.05)


@pytest.mark.parametrize(
    ("sample", "reason"),
    [
        ({"detected": False}, "ball_not_detected"),
        (ball_info(confidence=0.1), "low_ball_confidence"),
        (
            ball_info(depth_m=None, depth_valid=False),
            "missing_valid_ball_depth",
        ),
    ],
)
def test_unsafe_input_produces_stop(sample, reason):
    planner = BallNavigationPlanner()

    command = planner.plan(sample, 0.1)

    assert command.valid is False
    assert command.motion == "STOP"
    assert command.reason == reason
