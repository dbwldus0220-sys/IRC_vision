"""Replay accepted final-Line reservations before a short Goal handoff."""
import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_continuous_approach import clock
from test_line_turn_prequeue import late, offset
from test_motion_command_bridge import decoded_messages
from test_pickup_continuous_walk import forward_status
from test_post_ball_goal_handoff import goal, receive
from test_three_line_goal_handoff import start, reserve_line


def last_line_reserved(clock, *, turn=True):
    node, bridge, first = start(clock)
    second = reserve_line(node, clock)
    bridge.navigation_command_callback(String(data=json.dumps(second)))
    forward_status(node, bridge, first, 'SUCCEEDED')
    second_request = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, second_request, 'RUNNING')
    second_end = node.active_line_motion_started_at + node.active_line_motion_duration_sec
    if turn:
        late(node, clock, offset())
        third = node.publisher.messages[-1]
        assert third['action'] == 'LINE_OFFSET_TURN_LEFT_2'
    else:
        third = reserve_line(node, clock)
    assert third['source_command']['post_pickup_line_index'] == 3
    assert not node.camera_requests
    return node, bridge, second_request, third, second_end


def accept_last_line(node, bridge, third):
    before = len(bridge.motion_status_publisher.messages)
    bridge.navigation_command_callback(String(data=json.dumps(third)))
    statuses = bridge.motion_status_publisher.messages[before:]
    if bridge.queued_request_deferred:
        assert len(statuses) == 1
        assert json.loads(statuses[0].data)['status'] == 'QUEUED'
        Node._motion_status_callback(node, statuses[0])
    else:
        request = decoded_messages(bridge.executor_request_publisher)[-1]
        forward_status(node, bridge, request, 'QUEUED')


def head_ready(node, clock, command_id):
    Node._executor_heartbeat_callback(node, String(data=json.dumps({
        'sequence': 100, 'goal_head_prepare_id': command_id,
        'goal_head_transition_complete': True,
    })))


@pytest.mark.parametrize('turn', [True, False])
def test_only_accepted_last_line_can_prepare_camera_early(clock, turn):
    node, bridge, _, third, _ = last_line_reserved(clock, turn=turn)
    Node._prepare_post_ball_goal_camera(node, clock[0])
    assert not node.camera_requests
    accept_last_line(node, bridge, third)
    assert node.post_ball_line_started_count == 2
    assert node.camera_requests[-1]['request_id'] == third['command_id']
    assert getattr(node, 'post_ball_goal_camera_ready_at', None) is None
    count = len(node.camera_requests)
    node.send_status(third['action'], third['command_id'], 'QUEUED')
    assert len(node.camera_requests) == count  # Duplicate ack cannot restart the transition.


def test_short_third_turn_reserves_goal_before_its_completion(clock):
    node, bridge, second, third, second_end = last_line_reserved(clock)
    accept_last_line(node, bridge, third)
    clock[0] += .4
    assert clock[0] < second_end
    head_ready(node, clock, third['command_id'])
    receive(node, clock, 'goal', goal())
    assert node.queued_general_command_id == third['command_id']
    assert getattr(node, 'post_ball_goal_capture_count', 0) == 0  # Second Line gait still running.

    clock[0] = second_end
    forward_status(node, bridge, second, 'SUCCEEDED')
    third_request = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, third_request, 'RUNNING')
    third_start = clock[0]
    for _ in range(3):
        clock[0] += .1
        receive(node, clock, 'goal', goal())
    next_command = node.publisher.messages[-1]
    assert next_command['source_command']['post_ball_goal_handoff']
    assert clock[0] < third_start + .8
    bridge.navigation_command_callback(String(data=json.dumps(next_command)))
    assert bridge.queued_request_deferred
    clock[0] = third_start + .8
    forward_status(node, bridge, third_request, 'SUCCEEDED')
    goal_request = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, goal_request, 'RUNNING')
    assert node.mission_phase == 'GOAL_APPROACH'
    assert goal_request['command_id'] == next_command['command_id']
    assert len([m for m in node.publisher.messages
                if m.get('source_command', {}).get('post_pickup_line_index')]) == 3


def test_accepted_prepare_retries_when_subscriber_becomes_available(clock):
    node, bridge, _, third, _ = last_line_reserved(clock)
    node.goal_camera_publisher.get_subscription_count = lambda: 0
    accept_last_line(node, bridge, third)
    assert not node.camera_requests
    node.goal_camera_publisher.get_subscription_count = lambda: 1
    Node._prepare_post_ball_goal_camera(node, clock[0])
    assert node.camera_requests[-1]['request_id'] == third['command_id']


def test_second_motion_failure_prevents_further_camera_requests_and_handoff(clock):
    node, bridge, second, third, _ = last_line_reserved(clock)
    accept_last_line(node, bridge, third)
    forward_status(node, bridge, second, 'FAILED')
    assert node.post_ball_line_run_failed
    assert bridge.queued_request_id is None
    count = len(node.camera_requests)
    clock[0] += 1
    Node._prepare_post_ball_goal_camera(node, clock[0])
    receive(node, clock, 'goal', goal())
    assert len(node.camera_requests) == count
    assert not any(m['valid'] and m['command_id'] > third['command_id']
                   for m in node.publisher.messages)


@pytest.mark.parametrize('bad_ack', ['second_id', 'wrong_action', 'rejected'])
def test_unaccepted_reservation_cannot_raise_head(clock, bad_ack):
    node, _, second, third, _ = last_line_reserved(clock)
    command_id = second['command_id'] if bad_ack == 'second_id' else third['command_id']
    action = 'STRAIGHT' if bad_ack == 'wrong_action' else third['action']
    node.send_status(action, command_id, 'REJECTED' if bad_ack == 'rejected' else 'QUEUED')
    Node._prepare_post_ball_goal_camera(node, clock[0])
    assert not node.camera_requests


@pytest.mark.parametrize('status', ['REJECTED', 'FAILED', 'TIMEOUT', 'CANCELLED'])
def test_failed_accepted_reservation_blocks_line_replanning_with_raised_head(clock, status):
    node, bridge, _, third, _ = last_line_reserved(clock)
    accept_last_line(node, bridge, third)
    node.send_status(third['action'], third['command_id'], status)
    assert node.post_ball_line_run_failed
    assert node.queued_general_command_id is None
    assert node.post_ball_goal_camera_queued_id is None
    count = len(node.camera_requests)
    Node._prepare_post_ball_goal_camera(node, clock[0] + 1)
    Node._publish_decision(node)
    assert len(node.camera_requests) == count
    assert not any(m['valid'] and m['command_id'] > third['command_id']
                   for m in node.publisher.messages)


@pytest.mark.parametrize('blocker', ['no_ack', 'old_ack', 'unconfirmed', 'before_head'])
def test_early_camera_does_not_relax_goal_evidence(clock, blocker):
    node, bridge, second, third, second_end = last_line_reserved(clock)
    accept_last_line(node, bridge, third)
    clock[0] += .4
    if blocker != 'no_ack':
        head_ready(node, clock, third['command_id'] - 1 if blocker == 'old_ack' else third['command_id'])
    before_head = clock[0] - .05
    clock[0] = second_end
    forward_status(node, bridge, second, 'SUCCEEDED')
    third_request = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, third_request, 'RUNNING')
    for _ in range(3):
        clock[0] += .05
        sample = goal(confirmation_confirmed=blocker != 'unconfirmed')
        stamp = before_head if blocker == 'before_head' else clock[0]
        Node._info_callback(node, 'goal')(String(data=json.dumps({
            **sample, 'rgb_stamp_ns': round(stamp * 1e9),
        })))
    assert node.queued_general_command_id is None
