"""Reproduce turn/crab selection and the persistent near-ball turn block."""

import pytest

from mission_control.motion_decision_planner import MotionDecisionPlanner
from test_motion_decision_planner import ball_info


def ground_ball(angle, bottom=414, offset=194):
    return ball_info(
        depth_m=.44, bottom_distance_px=bottom, offset_x_px=offset,
        ground_projection_enabled=True, ground_projection_valid=True,
        ground_projection_scope='ball_approach_only',
        ground_coordinate_frame='robot_x_right_z_forward',
        ground_forward_distance_m=.640, ground_steering_angle_deg=angle,
    )


@pytest.mark.parametrize('angle,prefix', [
    (17.79, 'BALL_PICKUP_CAMERA_DOWN_TURN_RIGHT_'),
    (14.99, 'BALL_PICKUP_INITIAL_ALIGN_CONTINUE'),
    (-15., 'BALL_PICKUP_CAMERA_DOWN_TURN_LEFT_'),
])
def test_pickup_uses_ground_heading_before_lateral_pixels(angle, prefix):
    decision = MotionDecisionPlanner().plan_ball_pickup_initial_alignment(ground_ball(angle))
    assert decision.valid and decision.action.startswith(prefix)


def test_previous_close_capture_does_not_suppress_turn_above_180px():
    planner = MotionDecisionPlanner()
    planner._remember_pickup_close_ball(ground_ball(0., bottom=100))
    decision = planner.plan_ball_pickup_initial_alignment(ground_ball(17.79))
    assert decision.action.startswith('BALL_PICKUP_CAMERA_DOWN_TURN_RIGHT_')
    assert decision.source_command['pickup_close_alignment_active'] is False
    assert decision.source_command['bottom_distance_px'] == 414
    # The same visible geometry without a previous close capture selects yaw.
    fresh = MotionDecisionPlanner().plan_ball_pickup_initial_alignment(ground_ball(17.79))
    assert fresh.action.startswith('BALL_PICKUP_CAMERA_DOWN_TURN_RIGHT_')


@pytest.mark.parametrize('bottom,prefix', [(180, 'BALL_PICKUP_INITIAL_CRAB_RIGHT'),
                                          (180.001, 'BALL_PICKUP_CAMERA_DOWN_TURN_RIGHT_')])
def test_near_foot_pixels_prevent_stationary_turn_even_without_close_latch(bottom, prefix):
    decision = MotionDecisionPlanner().plan_ball_pickup_initial_alignment(ground_ball(25., bottom=bottom))
    assert decision.action.startswith(prefix)
