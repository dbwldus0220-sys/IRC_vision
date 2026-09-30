"""Keep moving BALL approach separate from close stationary alignment."""

import pytest

from mission_control.motion_decision_planner import MotionDecisionPlanner
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
from mission_control.motion_command_gate import normalize_general_action


def ball_sample(angle, **changes):
    sample = {
        "detected": True, "confidence": 0.95,
        "ground_projection_enabled": True, "ground_projection_valid": True,
        "ground_projection_scope": "ball_approach_only",
        "ground_coordinate_frame": "robot_x_right_z_forward",
        "ground_steering_angle_deg": angle,
        "ground_forward_distance_m": 0.9,
        "steering_angle_deg": -45.0, "bearing_deg": -30.0,
        "offset_x_norm": 0.06, "offset_x_px": 0,
        "depth_valid": True, "depth_age_sec": 0.01,
        "depth_m": 0.9, "distance_m": 0.9,
        "bottom_distance_px": 400, "pickup_x_tolerance_norm": 0.08,
    }
    sample.update(changes)
    return sample


@pytest.mark.parametrize("angle,action", [
    (-89.999, "BALL_APPROACH_RECOVER_LEFT_4"),
    (-20.0, "BALL_APPROACH_RECOVER_LEFT_4"),
    (-19.999, "STRAIGHT"), (0.0, "STRAIGHT"), (19.999, "STRAIGHT"),
    (20.0, "BALL_APPROACH_RECOVER_RIGHT_4"),
    (89.999, "BALL_APPROACH_RECOVER_RIGHT_4"),
])
@pytest.mark.parametrize("distance", [0.551, 0.9, 1.5])
def test_general_approach_uses_ground_angle_and_twenty_degree_boundary(angle, action, distance):
    planner = MotionDecisionPlanner()
    sample = ball_sample(angle, depth_m=distance, distance_m=distance)
    checkpoint = planner.plan_ball_approach_alignment(sample)
    assert checkpoint.action == "BALL_APPROACH_ALIGNED"
    assert checkpoint.source_command["steering_error_deg"] == angle
    decision = planner.plan("BALL_APPROACH", {"ball": sample}, 0.1)
    assert decision.valid
    assert decision.source == "ball"
    assert decision.action == action
    assert decision.source_command["steering_error_deg"] == angle
    assert decision.source_command["steering_source"] == "ground_steering_angle_deg"
    if "RECOVER" in action:
        direction = "left" if angle < 0 else "right"
        assert MotionCommandBridgeNode.motion_id_for_action(action) == f"line_recovery_{direction}_4"
        assert normalize_general_action(action) == action
        assert decision.source_command["turn_count"] == 4
        assert decision.source_command["turn_angle_deg"] == 30.0
        assert decision.source_command["target_heading_change_deg"] == (
            -30.0 if angle < 0 else 30.0
        )


@pytest.mark.parametrize("angle", [-45.0, 45.0])
@pytest.mark.parametrize("distance", [0.4, 0.55])
def test_close_distance_selects_pickup_instead_of_recover(angle, distance):
    decision = MotionDecisionPlanner().plan(
        "BALL_APPROACH",
        {"ball": ball_sample(angle, depth_m=distance, distance_m=distance)}, 0.1,
    )
    assert decision.action == "PICKUP_NOW"


@pytest.mark.parametrize("change", [
    {"ground_projection_valid": False}, {"ground_steering_angle_deg": float("nan")},
    {"ground_steering_angle_deg": None}, {"depth_valid": False},
    {"ground_steering_angle_deg": 90.0}, {"ground_steering_angle_deg": -90.0},
    {"distance_m": None}, {"confidence": 0.34}, {"detected": False},
])
def test_invalid_input_never_starts_recover_or_stationary_turn(change):
    sample = ball_sample(30.0, **change)
    decision = MotionDecisionPlanner().plan("BALL_APPROACH", {"ball": sample}, 0.1)
    assert not decision.valid
    assert "TURN" not in decision.action
    assert "RECOVER" not in decision.action


def test_invalid_ground_projection_waits_without_pixel_fallback():
    sample = ball_sample(30.0, ground_projection_valid=False)
    decision = MotionDecisionPlanner().plan_ball_approach_alignment(sample)
    assert decision.action == "WAIT"
    assert decision.valid is False


@pytest.mark.parametrize("method,prefix", [
    ("plan_ball_pickup_initial_alignment", "BALL_PICKUP_CAMERA_DOWN"),
    ("plan_ball_pickup_post_backward_alignment", "BALL_PICKUP_POST_BACKWARD"),
])
@pytest.mark.parametrize("forward,angle,source,turn", [
    (0.349999, -45.0, "near_image_angle", "LEFT_3"),
    (0.35, -45.0, "near_image_angle", "LEFT_3"),
    (0.350001, 30.0, "ground_steering_angle_deg", "RIGHT_3"),
    (1.5, 30.0, "ground_steering_angle_deg", "RIGHT_3"),
])
def test_pickup_checkpoints_share_distance_based_steering(method, prefix, forward, angle, source, turn):
    sample = ball_sample(
        30.0, ground_forward_distance_m=forward, depth_m=0.5, distance_m=0.5,
        offset_x_px=-200,
    )
    decision = getattr(MotionDecisionPlanner(), method)(sample)
    assert decision.action == f"{prefix}_TURN_{turn}"
    assert decision.source_command["steering_error_deg"] == angle
    assert decision.source_command["steering_source"] == source
    checkpoint = MotionDecisionPlanner().plan_ball_approach_alignment(sample)
    assert checkpoint.source_command["steering_error_deg"] == angle
    assert checkpoint.source_command["steering_source"] == source


@pytest.mark.parametrize("method", [
    "plan_ball_pickup_initial_alignment", "plan_ball_pickup_post_backward_alignment",
])
@pytest.mark.parametrize("changes", [
    {"ground_projection_valid": False}, {"ground_forward_distance_m": None},
    {"ground_forward_distance_m": float("nan")}, {"ground_steering_angle_deg": None},
])
def test_pickup_waits_when_selected_geometry_is_invalid(method, changes):
    sample = ball_sample(30.0, depth_m=0.5, distance_m=0.5, **changes)
    decision = getattr(MotionDecisionPlanner(), method)(sample)
    assert decision.action == "WAIT"
    assert not decision.valid


def test_close_crab_still_uses_pixel_side_when_ground_heading_points_opposite():
    sample = ball_sample(
        30.0, ground_forward_distance_m=0.4, depth_m=0.5, distance_m=0.5,
        offset_x_px=-200, bottom_distance_px=100,
    )
    decision = MotionDecisionPlanner().plan_ball_pickup_initial_alignment(sample)
    assert decision.action == "BALL_PICKUP_INITIAL_CRAB_LEFT"
    assert decision.source_command["steering_error_deg"] == 30.0


@pytest.mark.parametrize("method", [
    "plan_ball_pickup_initial_alignment",
    "plan_ball_pickup_post_backward_alignment",
])
@pytest.mark.parametrize("depth,bottom,allowed", [
    (0.55, 120.001, True), (0.55, 121, True), (0.4, 300, True),
    (0.551, 300, False), (1.5, 300, False),
    (0.55, 120, False), (0.4, 100, False),
    (None, 300, False), (0.0, 300, False), (-0.1, 300, False),
    (float("nan"), 300, False), (0.5, None, False),
])
def test_stationary_alignment_requires_close_depth_and_bottom_clearance(
    method, depth, bottom, allowed,
):
    sample = ball_sample(30.0, depth_m=depth, distance_m=0.5, bottom_distance_px=bottom)
    decision = getattr(MotionDecisionPlanner(), method)(sample)
    assert ("_TURN_" in decision.action) is allowed


@pytest.mark.parametrize("method", [
    "plan_ball_pickup_initial_alignment",
    "plan_ball_pickup_post_backward_alignment",
])
@pytest.mark.parametrize("change", [
    {"depth_valid": False}, {"depth_age_sec": None},
    {"depth_age_sec": -0.1}, {"depth_age_sec": 0.701},
])
def test_stationary_alignment_rejects_invalid_or_stale_depth(method, change):
    sample = ball_sample(30.0, depth_m=0.5, distance_m=0.5, **change)
    decision = getattr(MotionDecisionPlanner(), method)(sample)
    assert "_TURN_" not in decision.action


@pytest.mark.parametrize("line_angle", [-80.0, 80.0])
def test_recover_targets_ball_even_when_line_points_the_other_way(line_angle):
    decision = MotionDecisionPlanner().plan("BALL_APPROACH", {
        "ball": ball_sample(-30.0),
        "line": {"detected": True, "ground_heading_error_deg": line_angle},
    }, 0.1)
    assert decision.source == "ball"
    assert decision.action == "BALL_APPROACH_RECOVER_LEFT_4"


@pytest.mark.parametrize("method", [
    "plan_lost_ball_approach_alignment", "plan_ball_pickup_initial_alignment",
    "plan_ball_pickup_fine_alignment", "plan_ball_pickup_post_backward_alignment",
])
def test_lost_ball_never_uses_remembered_geometry_to_authorize_a_turn(method):
    planner = MotionDecisionPlanner()
    planner._update_ball_tracking(ball_sample(
        30.0, depth_m=0.5, distance_m=0.5, camera_center_offset_x_px=200,
    ), 0.0)
    decision = getattr(planner, method)({"detected": False, "raw_detected": False})
    assert decision.action == "WAIT"
    assert decision.valid is False
