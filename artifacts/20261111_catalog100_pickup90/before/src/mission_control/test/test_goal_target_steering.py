"""Replay target-point steering through the Goal planner, gate and SDK bridge."""

import math

import pytest

from mission_control.motion_decision_planner import MotionDecisionPlanner
from mission_control.motion_command_gate import GENERAL_ACTIONS
from step.goal_navigation_planner import GoalNavigationPlanner
from test_motion_command_bridge import FakeBridge, navigation_message, decoded_messages
from test_shot_start_posture import finish_sequence


def target_sample(angle, depth=1.0):
    lateral = (depth - .630) * math.tan(math.radians(angle)) - .630 * 40 / 900
    return dict(detected=True, confidence=.99, depth_valid=True, depth_m=depth,
                camera_fx_px=900.0, lateral_offset_m=lateral, bearing_deg=math.degrees(math.atan2(lateral, depth)),
                offset_x_px=lateral / depth * 900, offset_x_norm=lateral / depth,
                score_now=False)


@pytest.mark.parametrize('angle,action,motion', [
    (0, 'GOAL_CAMERA_90_FORWARD_2', 'goal_camera_90_forward_4'),
    (15, 'GOAL_CAMERA90_RECOVER_RIGHT_4', 'line_recovery_right_4'),
    (-15, 'GOAL_CAMERA90_RECOVER_LEFT_4', 'line_recovery_left_4'),
    (15.001, 'GOAL_CAMERA90_RECOVER_RIGHT_4', 'line_recovery_right_4'),
    (-15.001, 'GOAL_CAMERA90_RECOVER_LEFT_4', 'line_recovery_left_4'),
    (44.999, 'GOAL_CAMERA90_RECOVER_RIGHT_4', 'line_recovery_right_4'),
    (-44.999, 'GOAL_CAMERA90_RECOVER_LEFT_4', 'line_recovery_left_4'),
    (45, 'GOAL_CAMERA90_RECOVER_RIGHT_4', 'line_recovery_right_4'),
    (-45, 'GOAL_CAMERA90_RECOVER_LEFT_4', 'line_recovery_left_4'),
    (50, 'GOAL_CAMERA90_RECOVER_RIGHT_4', 'line_recovery_right_4'),
    (-50, 'GOAL_CAMERA90_RECOVER_LEFT_4', 'line_recovery_left_4'),
    (50.001, 'GOAL_CAMERA90_TURN_RIGHT_5', 'goal_camera_90_turn_right_5'),
    (-50.001, 'GOAL_CAMERA90_TURN_LEFT_3', 'goal_camera_90_turn_left_3'),
    (60, 'GOAL_CAMERA90_TURN_RIGHT_6', 'goal_camera_90_turn_right_6'),
])
def test_target_angle_boundaries_reach_sdk(angle, action, motion):
    info = target_sample(angle)
    standalone = GoalNavigationPlanner().plan(info)
    decision = MotionDecisionPlanner().plan('GOAL_APPROACH', {'goal': info}, .1)
    assert standalone.action == decision.action == action
    assert decision.source_command['target_steering_angle_deg'] == pytest.approx(angle)
    assert decision.source_command['target_forward_error_m'] == .370
    assert action in GENERAL_ACTIONS
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        action=action, source='goal', source_command=decision.source_command))
    finish_sequence(bridge)
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == motion


def test_remaining_distance_changes_steering_even_with_same_board_bearing():
    # Raw board bearing is 8 degrees in both cases. The stopping-point bearing
    # grows near the goal and must change the selected motion.
    planner = GoalNavigationPlanner()
    actions = []
    for depth in (2.0, .900):
        sample = target_sample(0, depth)
        sample.update(lateral_offset_m=depth * math.tan(math.radians(8)),
                      bearing_deg=8, offset_x_px=900 * math.tan(math.radians(8)), offset_x_norm=.248)
        actions.append(planner.plan(sample).action)
    assert actions == ['GOAL_CAMERA_90_FORWARD_2', 'GOAL_CAMERA90_RECOVER_RIGHT_4']


@pytest.mark.parametrize('overrides', [
    {'depth_valid': False}, {'confidence': .1}, {'detected': False},
    {'depth_m': float('nan')}, {'offset_x_px': None},
    {'lateral_offset_m': None, 'bearing_deg': None},
    {'lateral_offset_m': -.2},
])
def test_invalid_target_geometry_never_walks(overrides):
    info = {**target_sample(30), **overrides}
    command = GoalNavigationPlanner().plan(info)
    assert command.action == 'WAIT'
    assert command.valid is False


@pytest.mark.parametrize('side', ['left', 'right'])
def test_recovery_to_close_crab_keeps_forward_posture_transition(side):
    bridge = FakeBridge()
    bridge.last_physical_motion_id = f'line_recovery_{side}_4'
    bridge.navigation_command_callback(navigation_message(
        source='goal', action=f'GOAL_CAMERA90_CRAB_{side.upper()}'))
    finish_sequence(bridge)
    motions = [m['motion_id'] for m in decoded_messages(bridge.executor_request_publisher)]
    assert motions == [f'goal_forward_to_crab_{side}_90',
                       f'goal_camera_90_crab_{side}']
