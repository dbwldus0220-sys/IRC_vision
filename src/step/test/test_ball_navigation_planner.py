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
        "ground_distance_m": 0.88,
        "distance_m": 0.89,
        "depth_valid": True,
        "depth_age_sec": 0.05,
        "pickup_ready": False,
        "pickup_now": False,
    }
    sample.update(overrides)
    if "ground_distance_m" not in overrides and "depth_m" in overrides:
        sample["ground_distance_m"] = overrides["depth_m"]
    return sample


@pytest.mark.parametrize(
    "bearing",
    [-12.0, 0.0, 12.0],
)
def test_first_ball_curve_direction_does_not_follow_bearing(bearing):
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            bearing_deg=bearing,
            depth_m=0.70,
            ground_distance_m=0.70,
        ),
        0.1,
    )

    assert command.valid is True
    assert command.motion == "RECOVER_LEFT_TURN_LEFT_8"
    assert command.linear_speed_mps == 0.0


def test_centered_ball_above_78cm_uses_straight_3():
    planner = BallNavigationPlanner()

    command = planner.plan(ball_info(), 0.1)

    assert command.motion == "STRAIGHT_3"
    assert command.linear_speed_mps > 0.0
    assert command.travel_distance_m == pytest.approx(
        command.linear_speed_mps * command.command_duration_sec
    )


def test_first_ball_at_68cm_uses_left_6_curve():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(depth_m=0.68, distance_m=0.69, pickup_ready=False),
        0.1,
    )

    assert command.motion == "RECOVER_LEFT_TURN_LEFT_6"


@pytest.mark.parametrize(
    ("ground_distance", "expected_motion"),
    [
        (0.80, "STRAIGHT_3"),
        (1.50, "STRAIGHT_3"),
        (0.70, "RECOVER_LEFT_TURN_LEFT_8"),
        (0.60, "RECOVER_LEFT_TURN_LEFT_6"),
        (0.50, "STRAIGHT_3"),
        (0.30, "STRAIGHT_2"),
        (0.20, "STRAIGHT_1"),
        (0.13, "STRAIGHT_0"),
    ],
)
def test_aligned_general_ball_approach_distance_policy(
    ground_distance,
    expected_motion,
):
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            depth_m=max(ground_distance, 0.50),
            ground_distance_m=ground_distance,
            bearing_deg=0.0,
            offset_x_norm=0.0,
        ),
        0.1,
    )

    assert command.motion == expected_motion


@pytest.mark.parametrize(
    ("ground_distance", "bearing"),
    [(0.70, 30.0), (0.70, -30.0), (0.60, 30.0), (0.60, -30.0)],
)
def test_valid_general_approach_does_not_emit_raw_numbered_turn(
    ground_distance,
    bearing,
):
    command = BallNavigationPlanner().plan(
        ball_info(
            depth_m=ground_distance,
            ground_distance_m=ground_distance,
            bearing_deg=bearing,
        ),
        0.1,
    )

    assert not command.motion.startswith(("TURN_LEFT_", "TURN_RIGHT_"))


def test_raw_depth_starts_pickup_sequence_at_400mm_without_vision_gate():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            depth_m=0.40,
            ground_distance_m=0.15,
            distance_m=0.62,
            pickup_ready=False,
            pickup_now=False,
        ),
        0.1,
    )

    assert command.valid is True
    assert command.motion == "PICKUP_NOW"
    assert command.reason == "ball_raw_depth_entered_pickup_sequence"
    assert command.linear_speed_mps == 0.0
    assert command.pickup_now is True
    assert command.pickup_approach_motion == "STRAIGHT_2"
    assert command.ground_distance_m == 0.15


@pytest.mark.parametrize("depth_age_sec", [None, 0.701])
def test_invalid_or_stale_depth_does_not_start_pickup_sequence(
    depth_age_sec,
):
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            depth_m=0.40,
            ground_distance_m=0.15,
            pickup_ready=True,
            pickup_now=True,
            depth_age_sec=depth_age_sec,
        ),
        0.1,
    )

    assert command.valid is False
    assert command.motion == "STOP"
    assert command.reason == "stale_ball_depth_for_pickup_sequence"


def test_pickup_now_false_still_starts_sequence_at_close_raw_depth():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(depth_m=0.40, ground_distance_m=0.13),
        0.1,
    )

    assert command.motion == "PICKUP_NOW"
    assert command.pickup_now is True
    assert command.pickup_ready is False
    assert command.pickup_approach_motion == "STRAIGHT_2"


def test_pickup_sequence_does_not_start_above_400mm():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            depth_m=0.401,
            ground_distance_m=0.15,
            pickup_ready=True,
            pickup_now=True,
        ),
        0.1,
    )

    assert command.valid is True
    assert command.motion != "PICKUP_NOW"
    assert command.pickup_now is False


def test_pickup_sequence_uses_fine_motion_at_130mm():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            depth_m=0.13,
            ground_distance_m=0.05,
            pickup_ready=True,
            pickup_now=True,
        ),
        0.1,
    )

    assert command.motion == "PICKUP_NOW"
    assert command.pickup_approach_motion == "STRAIGHT_0"


def test_stale_80cm_pickup_flag_cannot_trigger_pickup():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(depth_m=0.80, distance_m=0.81, pickup_now=True),
        0.1,
    )

    assert command.motion == "STRAIGHT_3"
    assert command.pickup_now is False


def test_offset_is_used_when_camera_bearing_is_missing():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            bearing_deg=None,
            offset_x_norm=0.3,
            depth_m=0.70,
            ground_distance_m=0.70,
        ),
        0.1,
    )

    assert command.motion == "RECOVER_LEFT_TURN_LEFT_8"


def test_bottom_center_path_angle_has_priority_over_camera_bearing():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            steering_angle_deg=-30.0,
            bearing_deg=12.0,
            offset_x_norm=-0.3,
            depth_m=0.60,
            ground_distance_m=0.60,
        ),
        0.1,
    )

    assert command.motion == "RECOVER_LEFT_TURN_LEFT_6"
    assert command.linear_speed_mps == 0.0


def test_near_center_deadband_suppresses_path_angle_turn():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            steering_angle_deg=12.0,
            offset_x_norm=0.04,
            depth_m=0.60,
            ground_distance_m=0.55,
        ),
        0.1,
    )

    assert not command.motion.startswith("TURN_")
    assert command.target_heading_change_deg == 0.0


@pytest.mark.parametrize(
    "ground_distance",
    [1.0, 1.5, 1.51, 2.0, 10.0],
)
def test_valid_far_ground_distance_uses_straight_3(ground_distance):
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            depth_m=ground_distance,
            ground_distance_m=ground_distance,
            distance_m=ground_distance,
        ),
        0.1,
    )

    assert command.valid is True
    assert command.motion == "STRAIGHT_3"
    assert command.reason == "ball_aligned_discrete_approach"


@pytest.mark.parametrize(
    "ground_distance",
    [
        None,
        "invalid",
        float("nan"),
        float("inf"),
        float("-inf"),
        0.0,
        -0.1,
    ],
)
def test_invalid_ground_distance_stops(ground_distance):
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            depth_m=1.0,
            ground_distance_m=ground_distance,
            distance_m=1.0,
        ),
        0.1,
    )

    assert command.valid is False
    assert command.motion == "STOP"
    assert command.reason == "missing_valid_ball_ground_distance"


def test_missing_depth_does_not_emit_unsupported_raw_turn():
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            bearing_deg=12.0,
            depth_m=None,
            ground_distance_m=None,
            distance_m=None,
            depth_valid=False,
        ),
        0.1,
    )

    assert command.valid is False
    assert command.motion == "STOP"
    assert command.reason == "missing_valid_ball_ground_distance"
    assert command.depth_valid is False


@pytest.mark.parametrize(
    ("ball_occurrence", "ground_distance", "bearing", "expected_motion"),
    [
        (1, 0.70, -30.0, "RECOVER_LEFT_TURN_LEFT_8"),
        (1, 0.70, 30.0, "RECOVER_LEFT_TURN_LEFT_8"),
        (1, 0.70, 0.0, "RECOVER_LEFT_TURN_LEFT_8"),
        (1, 0.60, -30.0, "RECOVER_LEFT_TURN_LEFT_6"),
        (1, 0.60, 30.0, "RECOVER_LEFT_TURN_LEFT_6"),
        (1, 0.60, 0.0, "RECOVER_LEFT_TURN_LEFT_6"),
        (2, 0.70, -30.0, "RECOVER_RIGHT_TURN_RIGHT_8"),
        (2, 0.70, 30.0, "RECOVER_RIGHT_TURN_RIGHT_8"),
        (2, 0.70, 0.0, "RECOVER_RIGHT_TURN_RIGHT_8"),
        (2, 0.60, -30.0, "RECOVER_RIGHT_TURN_RIGHT_6"),
        (2, 0.60, 30.0, "RECOVER_RIGHT_TURN_RIGHT_6"),
        (2, 0.60, 0.0, "RECOVER_RIGHT_TURN_RIGHT_6"),
    ],
)
def test_ball_occurrence_selects_fixed_curve_independent_of_steering(
    ball_occurrence,
    ground_distance,
    bearing,
    expected_motion,
):
    planner = BallNavigationPlanner()

    command = planner.plan(
        ball_info(
            bearing_deg=bearing,
            depth_m=ground_distance,
            ground_distance_m=ground_distance,
            ball_occurrence=ball_occurrence,
        ),
        0.1,
    )

    assert command.motion == expected_motion
    assert command.linear_speed_mps == 0.0
    assert command.reason == "ball_directional_recovery_approach"


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
            "missing_valid_ball_ground_distance",
        ),
    ],
)
def test_unsafe_input_produces_stop(sample, reason):
    planner = BallNavigationPlanner()

    command = planner.plan(sample, 0.1)

    assert command.valid is False
    assert command.motion == "STOP"
    assert command.reason == reason
