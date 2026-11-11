"""Audit reservations and release conditions around pickup and hurdle alignment."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.lost_search_turn_limiter import LostSearchTurnLimiter
from test_continuous_approach import clock
from test_hurdle_angle_fallback import sample
from test_hurdle_positioning import HurdleHarness
from test_mission_phase_flow import release_general, line_info
from test_motion_command_bridge import FakeBridge, decoded_messages
from test_pickup_continuous_walk import forward_status
from test_post_ball_goal_handoff import goal, receive
from test_three_line_goal_handoff import start, reserve_line, finish_rest


def accept_reservation(node, bridge, command):
    before = len(bridge.motion_status_publisher.messages)
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    for message in bridge.motion_status_publisher.messages[before:]:
        Node._motion_status_callback(node, message)
    assert bridge.queued_request_id == node.queued_general_command_id == command['command_id']
    if not bridge.queued_request_deferred:
        forward_status(node, bridge, decoded_messages(bridge.executor_request_publisher)[-1], 'QUEUED')


def start_reserved(node, bridge, current, clock):
    forward_status(node, bridge, current, 'SUCCEEDED')
    finish_rest(bridge, clock)
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, request, 'RUNNING')
    return request


@pytest.mark.parametrize('index', [1, 2])
@pytest.mark.parametrize('angle', [0, -60, 60])
def test_yaw_walks_and_goal_reservation_complete_with_current_vision_schema(clock, index, angle):
    node, bridge, current = start(clock, index)
    for heading in (60., 0., -60., 0.):
        command = reserve_line(node, clock, heading)
        accept_reservation(node, bridge, command)
        current = start_reserved(node, bridge, current, clock)
    assert node.post_ball_line_started_count == 3
    assert len(node.post_ball_line_command_ids) == 3
    assert node.camera_requests
    clock[0] += .4
    Node._executor_heartbeat_callback(node, String(data=json.dumps({
        'sequence': 100, 'goal_head_prepare_id': current['goal_head_prepare_id'],
        'goal_head_transition_complete': True,
    })))
    # Current Goal planning requires measured ring depth and waist aim.
    for _ in range(3):
        clock[0] += .1
        receive(node, clock, 'goal', goal(angle, ring_center_depth_m=1., waist_ring_aim_deg=0.))
    following = node.publisher.messages[-1]
    assert following['source'] == 'goal'
    assert following['source_command']['post_ball_goal_handoff']
    accept_reservation(node, bridge, following)
    request = start_reserved(node, bridge, current, clock)
    assert request['request_id'] == following['command_id']
    assert node.general_motion_gate.active_command_id == following['command_id']
    assert node.mission_phase == 'GOAL_APPROACH'
    forward_status(node, bridge, request, 'SUCCEEDED')
    assert not node.general_motion_gate.locked
    assert node.queued_general_command_id is None


@pytest.mark.parametrize('index', [1, 2])
@pytest.mark.parametrize('heading', [0., 60.])
def test_rejected_walk_or_yaw_retries_without_consuming_a_slot(clock, index, heading):
    node, bridge, first = start(clock, index)
    second = reserve_line(node, clock)
    accept_reservation(node, bridge, second)
    current = start_reserved(node, bridge, first, clock)
    rejected = reserve_line(node, clock, heading)
    node.send_status(rejected['action'], rejected['command_id'], 'REJECTED')
    assert len(node.post_ball_line_command_ids) == 2
    assert node.queued_general_command_id is None
    assert getattr(node, 'post_ball_goal_camera_request_id', None) is None
    assert not node.camera_requests
    for _ in range(3):
        clock[0] += .03
        receive(node, clock, 'line', line_info(heading=heading))
    retry = node.publisher.messages[-1]
    assert retry['command_id'] != rejected['command_id']
    accept_reservation(node, bridge, retry)
    request = start_reserved(node, bridge, current, clock)
    assert request['request_id'] == retry['command_id']
    assert node.post_ball_line_started_count == (3 if heading == 0. else 2)


@pytest.mark.parametrize('index', [1, 2])
def test_delayed_camera_ack_releases_goal_wait_without_fourth_line_walk(clock, index):
    node, bridge, current = start(clock, index)
    for _ in range(2):
        command = reserve_line(node, clock)
        accept_reservation(node, bridge, command)
        current = start_reserved(node, bridge, current, clock)
    forward_status(node, bridge, current, 'SUCCEEDED')
    assert node.mission_phase == 'GOAL_APPROACH'
    assert node.publisher.messages[-1]['action'] == 'WAIT'
    assert not node.general_motion_gate.locked
    clock[0] += .4
    Node._executor_heartbeat_callback(node, String(data=json.dumps({
        'sequence': 100, 'goal_head_prepare_id': current['goal_head_prepare_id'],
        'goal_head_transition_complete': True,
    })))
    # A head ACK alone cannot release motion with missing ring geometry.
    clock[0] += .1
    receive(node, clock, 'goal', goal())
    Node._publish_decision(node)
    assert not node.general_motion_gate.locked
    for _ in range(3):
        clock[0] += .1
        receive(node, clock, 'goal', goal(ring_center_depth_m=1., waist_ring_aim_deg=0.))
        Node._publish_decision(node)
    following = node.publisher.messages[-1]
    assert following['source'] == 'goal' and following['valid']
    assert node.general_motion_gate.locked
    assert len([m for m in node.publisher.messages
                if m.get('source_command', {}).get('post_pickup_line_index')]) == 3


@pytest.mark.parametrize('status', ['REJECTED', 'FAILED', 'TIMEOUT', 'CANCELED', 'CANCELLED', 'UNSUPPORTED'])
def test_third_cancellation_after_head_prepare_holds_instead_of_blind_retry(clock, status):
    node, bridge, first = start(clock)
    second = reserve_line(node, clock)
    accept_reservation(node, bridge, second)
    current = start_reserved(node, bridge, first, clock)
    third = reserve_line(node, clock)
    accept_reservation(node, bridge, third)
    assert node.camera_requests
    node.send_status(third['action'], third['command_id'], status)
    assert node.post_ball_line_run_failed
    assert node.queued_general_command_id is None
    assert node.post_ball_goal_camera_request_id is None
    # Complete the active gait directly; the canceled reservation must stay blocked.
    node.send_status(second['action'], current['command_id'], 'SUCCEEDED', current['motion_id'])
    before = len(node.publisher.messages)
    reserve_line_info = node.latest_info['line']
    clock[0] += .1
    receive(node, clock, 'line', reserve_line_info)
    Node._publish_decision(node)
    assert not any(message['valid'] for message in node.publisher.messages[before:])
    assert not node.general_motion_gate.locked


def test_hurdle_fallback_reobserves_after_turn_and_releases_to_forward(clock):
    node = HurdleHarness(phase='HURDLE_POSITIONING')
    node.lost_search_turn_limiter = LostSearchTurnLimiter()
    node.planner.hurdle_planner.recovery_active = True
    receive(node, clock, 'hurdle', sample())
    node.publish()
    clock[0] += 1.1
    receive(node, clock, 'hurdle', sample())
    turn = node.publish()[-1]
    assert turn['action'] == 'ALIGN_RIGHT', turn
    assert turn['source_command']['alignment_reference'] == 'hurdle_depth_angle'
    bridge = FakeBridge()
    bridge.navigation_command_callback(String(data=json.dumps(turn)))
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == 'post_ball_line_turn_right_1'
    receive(node, clock, 'hurdle', sample(angle=0.))
    assert node.publish() == []
    assert getattr(node, 'queued_general_command_id', None) is None
    assert node.lost_search_turn_limiter.counts['hurdle'] == 0
    release_general(node, turn)
    clock[0] += .5
    receive(node, clock, 'hurdle', sample(angle=0.))
    assert node.publish() == []
    clock[0] += .6
    node.publish()
    node.publish()
    receive(node, clock, 'hurdle', sample(angle=0.))
    following = node.publish()[-1]
    assert following['action'] == 'STRAIGHT_1' and following['valid']
    assert not node.planner.hurdle_planner.recovery_active
