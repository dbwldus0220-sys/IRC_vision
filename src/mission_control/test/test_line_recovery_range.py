"""Shared recovery boundaries and fresh-image, single-command fallback contracts."""

import math

import pytest

from mission_control.ball_entry_line_alignment import BallEntryLineAlignment
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.motion_decision_planner import MotionDecisionPlanner
from step.line_navigation_planner import LineNavigationPlanner, NavigationConfig
from step.hurdle_navigation_planner import HurdleNavigationPlanner
from test_line_ground_fit_recovery import clock, node_for, frame, unconfirmed_sparse_info
from test_mission_phase_flow import line_info


ANGLES = [-50.001, -50., -45., -15., -14.999, -8., 0., 8., 14.999, 15., 45., 50., 50.001]


@pytest.mark.parametrize('angle', ANGLES)
@pytest.mark.parametrize('source', ['ground', 'image'])
def test_line_and_ball_entry_use_same_recovery_boundaries(angle, source):
    direction = 'RIGHT' if angle > 0 else 'LEFT'
    info = line_info(heading=angle, offset=math.copysign(.3, angle))
    planner = LineNavigationPlanner(NavigationConfig(heading_source=source))
    result = planner.plan(info, .1)
    ball = BallEntryLineAlignment._steering_command(
        MotionDecisionPlanner(), angle, math.copysign(.3, angle), .9, 1.)
    if abs(angle) < 15:
        assert result.motion == 'STRAIGHT'
        assert ball['motion'] == 'STRAIGHT_2'
    elif abs(angle) <= 50:
        assert result.motion == ball['motion'] == f'RECOVER_{direction}_TURN_{direction}_4'
    else:
        assert result.motion == f'LINE_HEADING_TURN_{direction}'
        assert ball['motion'].startswith(f'LINE_HEADING_TURN_{direction}_')


@pytest.mark.parametrize('angle', ANGLES)
def test_hurdle_line_heading_uses_stationary_turns_from_15_degrees(angle):
    info = dict(detected=True, confidence=.9, depth_valid=True, depth_m=.8,
                hurdle_angle_deg=60., bottom_distance_px=200.,
                path_reference_valid=True, path_reference_source='fresh',
                path_offset_x_norm=math.copysign(.3, angle), path_target_bearing_deg=-89.,
                line_heading_valid=True, line_heading_deg=angle, line_heading_reference_source="fresh")
    command = HurdleNavigationPlanner().plan(info, positioning=True)
    mission = MotionDecisionPlanner()._plan_source('hurdle', info, .1)
    assert command.valid and mission['valid']
    assert mission['action'] == command.action
    direction = 'RIGHT' if angle > 0 else 'LEFT'
    if abs(angle) < 15:
        assert command.action == 'STRAIGHT'
    else:
        assert command.action == f'ALIGN_{direction}'
        assert mission['center_steering_deg'] == angle
        assert mission['turn_count'] > 0


def image_info(angle):
    target_x = 710. + 269. * math.tan(math.radians(angle))
    return {**unconfirmed_sparse_info(), 'ground_two_point_candidate': None,
            'center_points_px': [[target_x, 650], [target_x, 450]]}


def publish_image_command(node, clock, info):
    for _ in range(12):
        commands = frame(node, clock, info)
        valid = [command for command in commands if command['valid']]
        if valid:
            return valid[-1]
        clock[0] += .15
    pytest.fail(f'No command: {node.last_selected_decision}')


@pytest.mark.parametrize('angle', ANGLES)
@pytest.mark.parametrize('phase', [
    'AUTO', 'LINE_TRACK', 'LINE_TRACK_AFTER_PICKUP', 'POST_BALL_LINE_ALIGN',
    'BALL_SEARCH', 'GOAL_SEARCH', 'HURDLE_APPROACH',
])
def test_image_fallback_boundaries_reach_dispatch(clock, angle, phase):
    node = node_for(phase)
    command = publish_image_command(node, clock, image_info(angle))
    assert command['source_command']['invalid_ground_recovery']
    direction = 'RIGHT' if angle > 0 else 'LEFT'
    expected = ('STRAIGHT_1' if abs(angle) < 15 else
                f'RECOVER_{direction}_TURN_{direction}_4' if abs(angle) <= 50 else
                f'LINE_OFFSET_TURN_{direction}_1')
    assert command['action'] == expected
    assert MotionCommandBridgeNode.motion_id_for_action(expected)


@pytest.mark.parametrize('phase', ['LINE_TRACK', 'LINE_TRACK_AFTER_PICKUP', 'POST_BALL_LINE_ALIGN'])
@pytest.mark.parametrize('angle', [-50., -20., -8., 0., 8., 20., 50.])
def test_image_motion_cannot_prequeue_even_if_ground_returns(clock, angle, phase):
    node = node_for(phase)
    command = publish_image_command(node, clock, image_info(angle))
    node.send_status(command['action'], command['command_id'], 'RUNNING')
    before = len(node.publisher.messages)
    for _ in range(18):
        clock[0] += .12
        frame(node, clock, {**line_info(), 'ground_fit_point_count': 3})
    assert not any(command['valid'] for command in node.publisher.messages[before:])
    assert getattr(node, 'queued_general_command_id', None) is None
    node.send_status(command['action'], command['command_id'], 'SUCCEEDED')
    assert node.latest_info['line'] is None
    assert not node.general_motion_gate.locked
    assert node.line_motion_min_rgb_stamp_ns == int(clock[0] * 1e9)
    before = len(node.publisher.messages)
    clock[0] += .1
    Node._publish_decision(node)
    assert not any(command['valid'] for command in node.publisher.messages[before:])
    next_command = publish_image_command(node, clock, image_info(-angle))
    assert next_command['command_id'] != command['command_id']


@pytest.mark.parametrize('angle', [-20., 0., 20.])
@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'REJECTED'])
def test_image_walk_failure_stops_retry(clock, angle, status):
    node = node_for()
    command = publish_image_command(node, clock, image_info(angle))
    if status != 'REJECTED':
        node.send_status(command['action'], command['command_id'], 'RUNNING')
    node.send_status(command['action'], command['command_id'], status)
    assert node.line_ground_recovery_failed
    for _ in range(8):
        clock[0] += .15
        assert not any(command['valid'] for command in frame(node, clock, image_info(-angle)))


@pytest.mark.parametrize('angle', [-20., 0., 20.])
@pytest.mark.parametrize('updates', [
    {'rgb_stamp_ns': 1}, {'rgb_stamp_ns': 100000000000},
    {'heading_quality': .1}, {'center_points_px': []}, {'detected': False},
])
def test_image_walk_requires_current_quality_geometry(clock, angle, updates):
    node = node_for()
    for _ in range(8):
        assert not any(command['valid'] for command in frame(node, clock, image_info(angle), **updates))
        clock[0] += .1


@pytest.mark.parametrize('angle', [-20., 0., 20.])
def test_disabled_ground_projection_can_use_valid_images(clock, angle):
    node = node_for()
    command = publish_image_command(node, clock, {
        **image_info(angle), 'ground_projection_enabled': False, 'ground_fit_reason': 'disabled',
    })
    assert command['source_command']['invalid_ground_recovery']


@pytest.mark.parametrize('angle', [-50., -15., -14.999, 0., 14.999, 15., 50.])
def test_two_point_corner_cannot_bypass_recovery_minimum(clock, angle):
    from test_sparse_line_recovery import sparse_info
    node = node_for()
    info = sparse_info(heading=angle, direction='RIGHT' if angle >= 0 else 'LEFT', distance=.14)
    info.update(turn_angle_deg=math.copysign(78.2, angle),
                filtered_lateral_offset_norm=math.copysign(.29, angle))
    command = publish_image_command(node, clock, info)
    if abs(angle) < 15:
        assert command['action'] == 'LINE_SPARSE_FORWARD'
    else:
        assert command['action'] in {'LEFT', 'RIGHT'} or command['action'].startswith('RECOVER_')


@pytest.mark.parametrize('depth,allowed', [(1.001, True), (1.0, False)])
def test_hurdle_reservation_check_requires_an_actual_running_walk(clock, depth, allowed):
    from test_hurdle_positioning import HurdleHarness, hurdle
    node = HurdleHarness()
    assert not Node._line_only_prequeue_allowed(node, {'hurdle': hurdle(depth)})
    node.general_motion_gate.active_action = 'STRAIGHT'
    node.general_motion_gate.active_command_id = 1
    assert Node._line_only_prequeue_allowed(node, {'hurdle': hurdle(depth)}) is allowed


@pytest.mark.parametrize('angle', [-20., 0., 20.])
def test_three_post_pickup_image_steps_release_to_goal(clock, angle):
    node = node_for('LINE_TRACK_AFTER_PICKUP')
    node.phase_manager.pickups_completed = 1
    node.phase_manager.ball_grasp_results[1] = 'GRABBED'
    for ordinal in (1, 2, 3):
        command = publish_image_command(node, clock, image_info(angle))
        assert command['source_command']['post_pickup_line_index'] == ordinal
        node.send_status(command['action'], command['command_id'], 'RUNNING')
        clock[0] += 2.
        node.send_status(command['action'], command['command_id'], 'SUCCEEDED')
        assert not node.general_motion_gate.locked
        clock[0] += .01
    assert node.mission_phase == 'GOAL_APPROACH'
    assert node.line_ground_recovery_command_id is None
