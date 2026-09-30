"""Verify intersection recovery reaches the gait bridge only before fine entry."""

import pytest

from mission_control.motion_decision_planner import MotionDecisionPlanner
from step.hurdle_navigation_planner import HurdleNavigationPlanner
from test_hurdle_positioning import hurdle, line_info
from test_motion_command_bridge import FakeBridge, decoded_messages, navigation_message


@pytest.mark.parametrize('x,direction', [(500, 'LEFT'), (900, 'RIGHT')])
@pytest.mark.parametrize('depth', [1.0, 0.8, 0.700001])
def test_intersection_side_selects_actual_line_recovery_motion(x, direction, depth):
    decision = MotionDecisionPlanner().plan('AUTO', {
        'hurdle': hurdle(depth, camera_center_offset_x_px=600 if x < 710 else -600,
                          hurdle_angle_deg=None),
        'line': line_info(x=x),
    }, .1)
    assert decision.action == f'RECOVER_{direction}_TURN_{direction}_4'
    assert decision.source == 'hurdle'
    assert decision.source_command['path_reference_point_px'] == [x, 500]
    assert decision.source_command['alignment_reference'] == 'line_hurdle_intersection'
    assert not decision.requires_ack
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        source=decision.source, action=decision.action, source_command=decision.source_command,
    ))
    assert [r['motion_id'] for r in decoded_messages(bridge.executor_request_publisher)] == [
        f'line_recovery_{direction.lower()}_4',
    ]


@pytest.mark.parametrize('x,action', [
    (645, 'RECOVER_LEFT_TURN_LEFT_4'), (646, 'STRAIGHT'),
    (710, 'STRAIGHT'), (774, 'STRAIGHT'), (775, 'RECOVER_RIGHT_TURN_RIGHT_4'),
])
def test_intersection_tolerance_uses_calibrated_robot_center(x, action):
    decision = MotionDecisionPlanner().plan('AUTO', {
        'hurdle': hurdle(.8), 'line': line_info(x=x),
    }, .1)
    assert decision.action == action


@pytest.mark.parametrize('depth,action', [(.700001, 'RECOVER_LEFT_TURN_LEFT_4'),
                                          (.7, 'STRAIGHT_0'), (.54, 'GO')])
def test_exact_fine_entry_boundary(depth, action):
    decision = MotionDecisionPlanner().plan('AUTO', {
        'hurdle': hurdle(depth), 'line': line_info(x=500),
    }, .1)
    assert decision.action == action


def test_fine_entry_latches_through_depth_rebound_and_resets_after_hurdle():
    planner = MotionDecisionPlanner()
    planner.plan('AUTO', {'hurdle': hurdle(.7)}, .1)
    rebound = planner.plan('AUTO', {'hurdle': hurdle(.85), 'line': line_info(x=900)}, .1)
    assert rebound.action == 'STRAIGHT_0'
    assert rebound.source_command['close_rotation_blocked']
    planner.plan('HURDLE_DONE', {'hurdle': {'detected': False}}, .1)
    planner.plan('AUTO', {'hurdle': {'detected': False}}, .1)
    new_hurdle = planner.plan('AUTO', {'hurdle': hurdle(.8), 'line': line_info(x=900)}, .1)
    assert new_hurdle.action == 'RECOVER_RIGHT_TURN_RIGHT_4'


@pytest.mark.parametrize('line', [None, {'detected': False}, line_info(x=1100)])
def test_invalid_intersection_waits_without_center_fallback(line):
    planner = MotionDecisionPlanner()
    planner.plan('AUTO', {'hurdle': hurdle(.8), 'line': line_info(x=500)}, .1)
    decision = planner.plan('AUTO', {'hurdle': hurdle(.8), 'line': line}, .1)
    assert decision.action == 'WAIT' and not decision.valid


@pytest.mark.parametrize('bottom', [100, None, -1])
def test_coarse_recovery_cannot_bypass_bottom_guard(bottom):
    decision = MotionDecisionPlanner().plan('AUTO', {
        'hurdle': hurdle(.8, bottom_distance_px=bottom), 'line': line_info(x=500),
    }, .1)
    assert decision.action == 'WAIT' and not decision.valid


def test_standalone_positioning_latches_fine_and_rejects_held_coarse_reference():
    planner = HurdleNavigationPlanner()
    held = {**hurdle(.8), 'path_reference_valid': True,
            'path_offset_x_norm': -.3, 'path_reference_source': 'held'}
    assert not planner.plan(held, positioning=True).valid
    assert planner.plan(hurdle(.7), positioning=True).action == 'STRAIGHT_0'
    assert planner.plan(held, positioning=True).action == 'STRAIGHT_0'
    planner.reset()
    assert not planner.fine_approach_active and not planner.close_rotation_blocked
