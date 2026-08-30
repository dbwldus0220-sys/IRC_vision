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
    [(12.0, "TURN_RIGHT_4"), (-12.0, "TURN_LEFT_2")],
)
def test_bearing_sign_selects_turn_direction(bearing, expected):
    planner = BallNavigationPlanner()

    command = planner.plan(ball_info(bearing_deg=bearing), 0.1)

    assert command.valid is True
    assert command.motion == expected
    assert command.linear_speed_mps == 0.0


def test_centered_ball_between_78_and_90cm_uses_generic_straight():
    planner = BallNavigationPlanner()

    command = planner.plan(ball_info(), 0.1)

    assert command.motion == "STRAIGHT"
    assert command.linear_speed_mps > 0.0
    assert command.travel_distance_m == pytest.approx(
        command.linear_speed_mps * command.command_duration_sec
    )


def test_close_ball_uses_measured_straight_level():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(depth_m=0.68, distance_m=0.69, pickup_ready=False),
        0.1,
    )

    assert command.motion == "STRAIGHT_4"
    assert command.to_dict()["approach_level"] == 4


def test_aligned_pickup_distance_stops_forward_motion():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(depth_m=0.07, distance_m=0.07, pickup_now=True),
        0.1,
    )

    assert command.valid is True
    assert command.motion == "PICKUP_NOW"
    assert command.linear_speed_mps == 0.0
    assert command.pickup_now is True


def test_stale_80cm_pickup_flag_cannot_trigger_pickup():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(depth_m=0.80, distance_m=0.81, pickup_now=True),
        0.1,
    )

    assert command.motion == "STRAIGHT"
    assert command.pickup_now is False


def test_offset_is_used_when_camera_bearing_is_missing():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(bearing_deg=None, offset_x_norm=0.3),
        0.1,
    )

    assert command.motion == "TURN_RIGHT_4"


def test_bottom_center_path_angle_has_priority_over_camera_bearing():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(steering_angle_deg=-30.0, bearing_deg=12.0),
        0.1,
    )

    assert command.motion == "TURN_LEFT_4"
    assert command.target_heading_change_deg == -30.0


def test_ball_inside_1_5m_control_range_moves_robot():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(depth_m=0.91, distance_m=0.92),
        0.1,
    )

    assert command.valid is True
    assert command.motion == "STRAIGHT"
    assert command.reason == "ball_aligned_discrete_approach"


def test_ball_beyond_1_5m_control_range_does_not_move_robot():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(depth_m=1.501, distance_m=1.51),
        0.1,
    )

    assert command.valid is False
    assert command.motion == "STOP"
    assert command.reason == "ball_outside_control_range"


def test_missing_depth_still_allows_visual_centering_turn():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            bearing_deg=12.0,
            depth_m=None,
            distance_m=None,
            depth_valid=False,
        ),
        0.1,
    )

    assert command.valid is True
    assert command.motion == "TURN_RIGHT_4"
    assert command.reason == "align_ball_center"
    assert command.depth_valid is False


def test_turn_hysteresis_holds_until_exit_threshold():
    planner = BallNavigationPlanner()

    first = planner.plan(ball_info(bearing_deg=8.0), 0.1)
    held = planner.plan(ball_info(bearing_deg=3.0), 0.1)
    released = planner.plan(ball_info(bearing_deg=2.0), 0.1)

    assert first.motion == "TURN_RIGHT_4"
    assert held.motion == "TURN_RIGHT_4"
    assert released.motion == "STRAIGHT"


@pytest.mark.parametrize(
    ("bearing", "expected_motion", "expected_angle"),
    [
        (12.0, "TURN_RIGHT_4", 15.0),
        (30.0, "TURN_RIGHT_6", 30.0),
        (-45.0, "TURN_LEFT_6", -45.0),
    ],
)
def test_ball_turn_reuses_line_recovery_numbered_angles(
    bearing,
    expected_motion,
    expected_angle,
):
    planner = BallNavigationPlanner()

    command = planner.plan(ball_info(bearing_deg=bearing), 0.1)
    payload = command.to_dict()

    assert command.motion == expected_motion
    assert command.target_heading_change_deg == expected_angle
    assert payload["turn_motion"] == expected_motion
    assert payload["turn_angle_deg"] == expected_angle


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
