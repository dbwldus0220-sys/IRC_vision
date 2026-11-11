"""Keep fine entry at 700mm and calibrated hurdle correction until 450mm."""

import pytest

from mission_control.motion_decision_planner import MotionDecisionPlanner
from step.hurdle_navigation_planner import HurdleNavigationPlanner
from test_hurdle_positioning import hurdle, line_info
from test_motion_command_bridge import FakeBridge, decoded_messages, navigation_message


@pytest.mark.parametrize('x,direction', [(500, 'LEFT'), (900, 'RIGHT')])
@pytest.mark.parametrize('depth', [1.0, 0.8, 0.700001, .7, .6, .450001])
def test_hurdle_yaw_selects_actual_stationary_motion(x, direction, depth):
    decision = MotionDecisionPlanner().plan('AUTO', {
        'hurdle': hurdle(depth, camera_center_offset_x_px=600 if x < 710 else -600,
                          hurdle_angle_deg=None, hurdle_angle_45_deg=40 if direction == "LEFT" else -40),
        'line': line_info(x=x, heading=-40 if direction == "LEFT" else 40),
    }, .1)
    assert decision.action == f'ALIGN_{direction}'
    assert decision.source == 'hurdle'
    assert decision.source_command['path_reference_point_px'] == [x, 500]
    assert decision.source_command['alignment_reference'] == 'hurdle_angle_45'
    assert not decision.requires_ack
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        source=decision.source, action=decision.action, source_command=decision.source_command,
    ))
    assert [r['motion_id'] for r in decoded_messages(bridge.executor_request_publisher)] == [
        f'post_ball_line_turn_{direction.lower()}_{2 if direction == "LEFT" else 3}',
    ]


@pytest.mark.parametrize('angle,action', [
    (-15, 'ALIGN_LEFT'), (-14.999, 'STRAIGHT_1'), (0, 'STRAIGHT_1'),
    (14.999, 'STRAIGHT_1'), (15, 'ALIGN_RIGHT'),
])
def test_hurdle_yaw_threshold_ignores_intersection_angle(angle, action):
    decision = MotionDecisionPlanner().plan('AUTO', {
        'hurdle': hurdle(.8, hurdle_angle_45_deg=-angle), 'line': line_info(x=900, heading=angle),
    }, .1)
    assert decision.action == action


@pytest.mark.parametrize('depth,action', [(.700001, 'ALIGN_LEFT'),
                                        (.7, 'ALIGN_LEFT'),
                                        (.450001, 'ALIGN_LEFT'),
                                        (.45, 'GO'), (.449999, 'GO')])
def test_exact_fine_entry_boundary(depth, action):
    decision = MotionDecisionPlanner().plan('AUTO', {
        'hurdle': hurdle(depth, hurdle_angle_45_deg=40), 'line': line_info(x=500, heading=-40),
    }, .1)
    assert decision.action == action


def test_fine_entry_latches_through_depth_rebound_and_resets_after_hurdle():
    planner = MotionDecisionPlanner()
    planner.plan('AUTO', {'hurdle': hurdle(.7)}, .1)
    rebound = planner.plan('AUTO', {'hurdle': hurdle(.85, hurdle_angle_45_deg=-40), 'line': line_info(x=900, heading=40)}, .1)
    assert rebound.action == 'ALIGN_RIGHT'
    assert not rebound.source_command['close_rotation_blocked']
    assert planner.hurdle_planner.fine_approach_active
    assert rebound.source_command['hurdle_stage'] == 'FINE_APPROACH'
    centered = planner.plan('AUTO', {'hurdle': hurdle(.85), 'line': line_info()}, .1)
    assert centered.action == 'STRAIGHT_1'
    planner.plan('HURDLE_DONE', {'hurdle': {'detected': False}}, .1)
    planner.plan('AUTO', {'hurdle': {'detected': False}}, .1)
    new_hurdle = planner.plan('AUTO', {'hurdle': hurdle(.8, hurdle_angle_45_deg=-40), 'line': line_info(x=900, heading=40)}, .1)
    assert new_hurdle.action == 'ALIGN_RIGHT'


@pytest.mark.parametrize('line', [None, {'detected': False},
                                       {**line_info(), 'filtered_heading_error_deg': None}])
def test_missing_line_does_not_block_calibrated_hurdle_yaw(line):
    planner = MotionDecisionPlanner()
    planner.plan('AUTO', {'hurdle': hurdle(.8), 'line': line_info(x=500, heading=-40)}, .1)
    decision = planner.plan('AUTO', {'hurdle': hurdle(.8), 'line': line}, .1)
    assert decision.action == 'STRAIGHT_1' and decision.valid


@pytest.mark.parametrize('bottom', [100, None, -1])
def test_coarse_recovery_cannot_bypass_bottom_guard(bottom):
    decision = MotionDecisionPlanner().plan('AUTO', {
        'hurdle': hurdle(.8, bottom_distance_px=bottom, hurdle_angle_45_deg=40), 'line': line_info(x=500, heading=-40),
    }, .1)
    assert decision.action == 'WAIT' and not decision.valid


def test_standalone_positioning_latches_fine_without_using_held_intersection():
    planner = HurdleNavigationPlanner()
    held = {**hurdle(.8), 'path_reference_valid': True,
            'path_offset_x_norm': -.3, 'path_reference_source': 'held'}
    assert planner.plan(held, positioning=True).action == 'STRAIGHT_1'
    assert planner.plan(hurdle(.7), positioning=True).action == 'STRAIGHT_1'
    assert planner.plan(held, positioning=True).action == 'STRAIGHT_1'
    planner.reset()
    assert not planner.fine_approach_active and not planner.close_rotation_blocked
