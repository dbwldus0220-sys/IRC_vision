"""Replay the 800/550 mm handoff and ground-only steering without motors."""

import json

import pytest
from std_msgs.msg import String

from mission_control.ball_entry_line_alignment import BallEntryLineAlignment
from mission_control.motion_decision_planner import MotionDecisionPlanner
from test_ball_entry_line_alignment import (
    Node, FakeBridge, clock, fresh, ground_ball_info, line_info,
    node_with_entry, send, status, late_line,
)


@pytest.mark.parametrize('distance,reference', [
    (1.5, 'ground'), (.800001, 'ground'), (.8, 'ball_ground'), (.7, 'ball_ground'), (.550001, 'ball_ground'),
])
def test_range_selects_line_or_ball_reference(distance, reference):
    decision = BallEntryLineAlignment().select(
        MotionDecisionPlanner(), line_info(-30., -.3), .1,
        ground_ball_info(depth_m=distance, ground_steering_angle_deg=30., offset_x_norm=-.9))
    assert decision.valid
    assert decision.source_command['heading_source'] == reference
    assert decision.action == ('RECOVER_LEFT_TURN_LEFT_4' if reference == 'ground'
                               else 'RECOVER_RIGHT_TURN_RIGHT_4')


@pytest.mark.parametrize('angle,action', [
    (-89., 'LINE_HEADING_TURN_LEFT_5'), (-75., 'LINE_HEADING_TURN_LEFT_5'),
    (-60., 'LINE_HEADING_TURN_LEFT_4'), (-45., 'LINE_HEADING_TURN_LEFT_3'),
    (-44.999, 'RECOVER_LEFT_TURN_LEFT_4'), (-15.001, 'RECOVER_LEFT_TURN_LEFT_4'),
    (-15., 'STRAIGHT_2'), (0., 'STRAIGHT_2'), (15., 'STRAIGHT_2'),
    (15.001, 'RECOVER_RIGHT_TURN_RIGHT_4'), (44.999, 'RECOVER_RIGHT_TURN_RIGHT_4'),
    (45., 'LINE_HEADING_TURN_RIGHT_5'), (60., 'LINE_HEADING_TURN_RIGHT_6'),
    (75., 'LINE_HEADING_TURN_RIGHT_7'), (89., 'LINE_HEADING_TURN_RIGHT_7'),
])
@pytest.mark.parametrize('line', [None, line_info(-60., -.9)])
def test_ground_bins_ignore_line_and_image_steering(angle, action, line):
    decision = BallEntryLineAlignment().select(MotionDecisionPlanner(), line, .1,
        ground_ball_info(depth_m=.6, ground_forward_distance_m=.25,
                         ground_steering_angle_deg=angle, steering_angle_deg=-80.,
                         bearing_deg=-80., offset_x_norm=-.8))
    assert decision.valid and decision.action == action
    assert decision.source_command['steering_error_deg'] == angle
    assert decision.source_command['alignment_reference'] == 'ball_ground_steering'


@pytest.mark.parametrize('change', [
    {'ground_projection_enabled': False}, {'ground_projection_valid': False},
    {'ground_projection_scope': 'wrong'}, {'ground_coordinate_frame': 'camera'},
    {'ground_steering_angle_deg': None}, {'ground_steering_angle_deg': float('nan')},
    {'ground_steering_angle_deg': float('inf')}, {'ground_steering_angle_deg': True},
    {'ground_forward_distance_m': None}, {'ground_forward_distance_m': 0.},
    {'ground_forward_distance_m': -1.}, {'depth_age_sec': 5.},
    {'confirmation_confirmed': False}, {'confidence': .1},
])
def test_invalid_ground_ball_never_falls_back_to_line_or_image(change):
    decision = BallEntryLineAlignment().select(MotionDecisionPlanner(), line_info(), .1,
        ground_ball_info(depth_m=.6, **change))
    assert not decision.valid and decision.action == 'STOP'


def test_ground_forward_needs_new_ball_not_new_line_and_enters_pickup(clock):
    node, bridge = node_with_entry(), FakeBridge()
    fresh(node, clock, 'ball', ground_ball_info(depth_m=.66))
    first = node.publisher.messages[-1]
    assert first['source'] == 'ball' and first['action'] == 'STRAIGHT_2'
    request = send(bridge, first)
    assert request['motion_id'] == 'line_forward_4'
    status(node, bridge, request, 'RUNNING')
    status(node, bridge, request, 'SUCCEEDED')
    count = len([m for m in node.publisher.messages if m['valid']])
    fresh(node, clock, 'line', line_info())
    assert len([m for m in node.publisher.messages if m['valid']]) == count
    fresh(node, clock, 'ball', ground_ball_info(depth_m=.6))
    second = node.publisher.messages[-1]
    assert second['valid'] and second['command_id'] != first['command_id']
    request = send(bridge, second)
    status(node, bridge, request, 'RUNNING')
    fresh(node, clock, 'ball', ground_ball_info(depth_m=.55), plan=False)
    assert node.ball_pickup_entry_pending
    assert node.queued_general_command_id is None
    status(node, bridge, request, 'SUCCEEDED')
    pickup = node.publisher.messages[-1]
    assert pickup['action'] == 'PICKUP_NOW'
    assert node.ball_entry_line_alignment.state == 'DONE'
    bridge.navigation_command_callback(String(data=json.dumps(pickup)))
    assert bridge.pickup_initial_align_waiting


def test_stale_line_candidate_waits_for_ball_capture_when_crossing_800(clock):
    node, bridge = node_with_entry(), FakeBridge()
    fresh(node, clock, 'line', line_info(), plan=False)
    fresh(node, clock, 'ball', ground_ball_info(depth_m=.81))
    request = send(bridge, node.publisher.messages[-1])
    status(node, bridge, request, 'RUNNING')
    old_decision = node.ball_entry_line_alignment.select(
        node.planner, line_info(), .1, ground_ball_info(depth_m=.81))
    fresh(node, clock, 'ball', ground_ball_info(depth_m=.66), plan=False)
    late_line(node, clock, line_info())
    assert node.queued_general_command_id is None
    assert not Node._walking_prequeue_allowed(node, old_decision, {
        'line': line_info(), 'ball': ground_ball_info(depth_m=.66)}, clock[0])


def test_ground_turn_refreshes_ball_after_dwell_without_waiting_for_line(clock):
    node, bridge = node_with_entry(), FakeBridge()
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = Node.LINE_TURN_PRE_MOTION_SETTLE_SEC
    ball = ground_ball_info(depth_m=.6, ground_steering_angle_deg=65.)
    fresh(node, clock, 'ball', ball)
    clock[0] += 1.1
    fresh(node, clock, 'ball', ball)
    command = node.publisher.messages[-1]
    assert command['valid'] and command['action'] == 'LINE_HEADING_TURN_RIGHT_6', json.dumps(command)
    assert node.lost_search_turn_limiter.counts['line'] == 0
    assert node.lost_search_turn_limiter.counts['ball'] == 0
    request = send(bridge, command)
    assert request['motion_id'] == 'post_ball_line_turn_right_6'
    status(node, bridge, request, 'RUNNING')
    status(node, bridge, request, 'SUCCEEDED')
    assert node.correction_post_motion_source == 'ball'
    clock[0] = node.correction_post_motion_dwell_until + .01
    Node._publish_decision(node)
    assert node.latest_info['ball'] is None
    fresh(node, clock, 'ball', ground_ball_info(depth_m=.6))
    assert node.publisher.messages[-1]['action'] == 'STRAIGHT_2'
    assert node.publisher.messages[-1]['valid']


@pytest.mark.parametrize('angle,direction', [(-25., 'left'), (25., 'right')])
def test_ground_recovery_dispatches_existing_four_repeat_gait_without_line(clock, angle, direction):
    node, bridge = node_with_entry(), FakeBridge()
    fresh(node, clock, 'ball', ground_ball_info(depth_m=.6, ground_steering_angle_deg=angle))
    command = node.publisher.messages[-1]
    assert command['valid'] and command['source'] == 'ball'
    request = send(bridge, command)
    assert request['motion_id'] == f'line_recovery_{direction}_4'
    status(node, bridge, request, 'RUNNING')
    fresh(node, clock, 'ball', ground_ball_info(depth_m=.56), plan=False)
    assert node.queued_general_command_id is None
