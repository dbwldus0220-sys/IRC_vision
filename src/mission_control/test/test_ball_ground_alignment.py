"""Use ground steering only at the normal-head-pose BALL checkpoint."""

import pytest

from mission_control.motion_decision_planner import MotionDecisionPlanner


def ball_sample(angle):
    return {
        "detected": True, "confidence": 0.95,
        "ground_projection_enabled": True, "ground_projection_valid": True,
        "ground_projection_scope": "ball_approach_only",
        "ground_coordinate_frame": "robot_x_right_z_forward",
        "ground_steering_angle_deg": angle,
        "steering_angle_deg": -45.0, "bearing_deg": -30.0,
        "offset_x_norm": 0.0, "offset_x_px": 0,
        "depth_valid": True, "depth_age_sec": 0.01,
        "depth_m": 0.9, "distance_m": 0.9,
        "bottom_distance_px": 400, "pickup_x_tolerance_norm": 0.08,
    }


@pytest.mark.parametrize("angle,action", [
    (14.999, "BALL_APPROACH_ALIGNED"),
    (15.0, "BALL_APPROACH_TURN_RIGHT_2"),
    (30.0, "BALL_APPROACH_TURN_RIGHT_3"),
    (-29.999, "BALL_APPROACH_ALIGNED"),
    (-30.0, "BALL_APPROACH_TURN_LEFT_1"),
    (-45.0, "BALL_APPROACH_TURN_LEFT_2"),
])
def test_general_turn_uses_ground_sign_and_existing_motion_thresholds(angle, action):
    decision = MotionDecisionPlanner().plan_ball_approach_alignment(ball_sample(angle))
    assert decision.action == action
    assert decision.source_command["steering_error_deg"] == angle
    assert decision.source_command["steering_source"] == "ground_steering_angle_deg"


def test_invalid_ground_projection_waits_without_pixel_fallback():
    sample = ball_sample(30.0)
    sample["ground_projection_valid"] = False
    decision = MotionDecisionPlanner().plan_ball_approach_alignment(sample)
    assert decision.action == "WAIT"
    assert decision.valid is False


@pytest.mark.parametrize("method,action", [
    ("plan_ball_pickup_initial_alignment", "BALL_PICKUP_CAMERA_DOWN_TURN_LEFT_2"),
    ("plan_ball_pickup_post_backward_alignment", "BALL_PICKUP_POST_BACKWARD_TURN_LEFT_2"),
])
@pytest.mark.parametrize("ground_valid", [False, True])
def test_head_down_checkpoints_keep_image_geometry(method, action, ground_valid):
    sample = ball_sample(30.0)
    sample["ground_projection_valid"] = ground_valid
    decision = getattr(MotionDecisionPlanner(), method)(sample)
    assert decision.action == action
    assert decision.source_command["steering_error_deg"] == -45.0
    assert decision.source_command["steering_source"] == "head_down_image_angle"
