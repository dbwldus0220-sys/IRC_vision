"""Three actual Line commands and a head override starting with the third."""
import json
from types import SimpleNamespace

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_continuous_approach import clock
from test_post_ball_goal_handoff import receive, goal
from test_mission_phase_flow import line_info
from test_wait_refresh_and_fine_settle import LiveInputHarness
from test_motion_command_bridge import FakeBridge, decoded_messages
from test_pickup_continuous_walk import forward_status


def start(clock, index=1, heading=0.):
    node = LiveInputHarness(phase='POST_BALL_LINE_ALIGN')
    node.phase_manager.pickups_completed = index
    node.phase_manager.ball_sections_processed = index - 1
    node.phase_manager.ball_grasp_results[index] = 'GRABBED'
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = .5
    node.camera_requests = []
    node.goal_camera_publisher = SimpleNamespace(
        publish=lambda msg: node.camera_requests.append(json.loads(msg.data)),
        get_subscription_count=lambda: 1)
    bridge = FakeBridge(advance_transition_dwells=False)
    receive(node, clock, 'line', line_info(heading=heading))
    Node._publish_decision(node)
    first = node.publisher.messages[-1]
    assert first['source_command']['post_pickup_line_index'] == 1
    bridge.navigation_command_callback(String(data=json.dumps(first)))
    finish_rest(bridge, clock)
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, request, 'RUNNING')
    return node, bridge, request


def finish_rest(bridge, clock):
    deadline = bridge.transition_dwell_until
    if deadline is None:
        return
    before = len(bridge.executor_request_publisher.messages)
    assert deadline == pytest.approx(clock[0] + .5)
    clock[0] = deadline - .001
    bridge._check_atomic_dwell()
    assert len(bridge.executor_request_publisher.messages) == before
    clock[0] = deadline
    bridge._check_atomic_dwell()
    assert len(bridge.executor_request_publisher.messages) == before + 1


def reserve_line(node, clock, heading=0.):
    started, duration = node.active_line_motion_started_at, node.active_line_motion_duration_sec
    clock[0] = max(started + duration * .75, started + duration - Node.LINE_RECOVERY_PREQUEUE_LEAD_SEC)
    for _ in range(3):
        clock[0] += .03
        receive(node, clock, 'line', line_info(heading=heading))
    return node.publisher.messages[-1]


@pytest.mark.parametrize('index', [1, 2])
@pytest.mark.parametrize('goal_angle', [0, -50, 50])
@pytest.mark.parametrize('headings', [(0., 0., 0.), (30., -30., 0.), (60., 0., -60.)])
def test_three_commands_preserve_turn_rests_and_goal_uses_third_motion_images(clock, index, headings, goal_angle):
    node, bridge, current = start(clock, index, headings[0])
    for ordinal, heading in enumerate(headings[1:], 2):
        command = reserve_line(node, clock, heading)
        assert command['source_command']['post_pickup_line_index'] == ordinal
        assert node.post_ball_line_started_count == ordinal - 1
        assert not node.camera_requests
        bridge.navigation_command_callback(String(data=json.dumps(command)))
        queued = bridge.queued_request_id
        assert queued == command['command_id']
        forward_status(node, bridge, current, 'SUCCEEDED')
        turn_boundary = bridge._is_stationary_turn(current['motion_id']) or bridge._is_stationary_turn(
            bridge.motion_id_for_action(command['action']))
        assert (bridge.transition_dwell_until is not None) is turn_boundary
        finish_rest(bridge, clock)
        current = decoded_messages(bridge.executor_request_publisher)[-1]
        assert current['request_id'] == queued
        assert ('goal_head_prepare_id' in current) is (ordinal == 3)
        forward_status(node, bridge, current, 'RUNNING')
        forward_status(node, bridge, current, 'RUNNING')
        assert node.post_ball_line_started_count == ordinal
        assert getattr(node, 'correction_post_motion_dwell_until', None) is None
        assert bridge.active_dwell_until is None and bridge.transition_dwell_until is None
    assert len(node.post_ball_line_command_ids) == 3
    camera_id = current['goal_head_prepare_id']
    clock[0] += .4
    Node._executor_heartbeat_callback(node, String(data=json.dumps({
        'sequence': 100, 'goal_head_prepare_id': camera_id,
        'goal_head_transition_complete': True})))
    for _ in range(3):
        clock[0] += .1
        receive(node, clock, 'goal', goal(goal_angle))
    command = node.publisher.messages[-1]
    assert command['source'] == 'goal' and command['source_command']['post_ball_goal_handoff']
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    forward_status(node, bridge, current, 'SUCCEEDED')
    finish_rest(bridge, clock)
    next_request = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, next_request, 'RUNNING')
    assert node.mission_phase == 'GOAL_APPROACH'
    assert node.general_motion_gate.active_command_id == command['command_id']
    assert len([m for m in node.publisher.messages if m.get('source_command', {}).get('post_pickup_line_index')]) == 3


def test_time_and_waits_do_not_consume_a_decision(clock):
    node, _, _ = start(clock)
    clock[0] += 20
    for _ in range(5):
        Node._publish_decision(node)
    assert len(node.post_ball_line_command_ids) == node.post_ball_line_started_count == 1
    assert not node.camera_requests


@pytest.mark.parametrize('blocker', ['no_ack', 'old_ack', 'before_head', 'stale', 'future', 'duplicate', 'lost', 'unconfirmed'])
def test_goal_reservation_requires_fresh_confirmed_head_up_images(clock, blocker):
    node, bridge, current = start(clock, 2)
    for _ in range(2):
        following = reserve_line(node, clock)
        bridge.navigation_command_callback(String(data=json.dumps(following)))
        forward_status(node, bridge, current, 'SUCCEEDED')
        current = decoded_messages(bridge.executor_request_publisher)[-1]
        forward_status(node, bridge, current, 'RUNNING')
    camera_id = current['goal_head_prepare_id']
    clock[0] += .4
    if blocker != 'no_ack':
        Node._executor_heartbeat_callback(node, String(data=json.dumps({
            'sequence': 111, 'goal_head_prepare_id': camera_id - 1 if blocker == 'old_ack' else camera_id,
            'goal_head_transition_complete': True})))
    ready_at = clock[0]
    for _ in range(3):
        clock[0] += .1
        sample = goal()
        if blocker == 'lost':
            sample['detected'] = False
        if blocker == 'unconfirmed':
            sample['confirmation_confirmed'] = False
        stamp = clock[0]
        if blocker == 'before_head':
            stamp = ready_at - .1
        elif blocker == 'stale':
            stamp -= 1.
        elif blocker == 'future':
            stamp += 1.
        elif blocker == 'duplicate':
            stamp = ready_at + .1
        Node._info_callback(node, 'goal')(String(data=json.dumps({**sample, 'rgb_stamp_ns': round(stamp * 1e9)})))
    assert getattr(node, 'queued_general_command_id', None) is None
    assert len(node.post_ball_line_command_ids) == 3


def test_third_finishing_before_head_ready_never_runs_a_fourth_line_or_blind_goal(clock):
    node, bridge, current = start(clock, 1)
    for _ in range(2):
        following = reserve_line(node, clock)
        bridge.navigation_command_callback(String(data=json.dumps(following)))
        forward_status(node, bridge, current, 'SUCCEEDED')
        current = decoded_messages(bridge.executor_request_publisher)[-1]
        forward_status(node, bridge, current, 'RUNNING')
    before = len(bridge.executor_request_publisher.messages)
    forward_status(node, bridge, current, 'SUCCEEDED')
    assert node.mission_phase == 'GOAL_APPROACH'
    assert node.publisher.messages[-1]['action'] == 'WAIT'
    assert len(bridge.executor_request_publisher.messages) == before


def test_third_reservation_rejection_reuses_ordinal_without_raising_head(clock):
    node, bridge, first = start(clock)
    second = reserve_line(node, clock)
    bridge.navigation_command_callback(String(data=json.dumps(second)))
    forward_status(node, bridge, first, 'SUCCEEDED')
    second_request = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, second_request, 'RUNNING')
    third = reserve_line(node, clock)
    node.send_status(third['action'], third['command_id'], 'REJECTED')
    assert len(node.post_ball_line_command_ids) == 2
    assert node.post_ball_goal_camera_request_id is None
    assert not node.camera_requests


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED'])
def test_failure_aborts_counted_stage(clock, status):
    node, bridge, current = start(clock)
    second = reserve_line(node, clock)
    bridge.navigation_command_callback(String(data=json.dumps(second)))
    forward_status(node, bridge, current, status)
    assert node.post_ball_line_run_failed
    assert bridge.queued_request_id is None
    assert not node.camera_requests


@pytest.mark.parametrize('motion_id', ['__TRANSITION_PRE_DWELL__', 'goal_fine_to_default', 'pickup_fine_prepare'])
def test_preparation_running_does_not_count_as_third_start_or_raise_camera(clock, motion_id):
    node, bridge, first = start(clock)
    second = reserve_line(node, clock)
    bridge.navigation_command_callback(String(data=json.dumps(second)))
    forward_status(node, bridge, first, 'SUCCEEDED')
    second_request = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, second_request, 'RUNNING')
    third = reserve_line(node, clock)
    node.send_status(second['action'], second['command_id'], 'SUCCEEDED')
    node.send_status(third['action'], third['command_id'], 'RUNNING', motion_id=motion_id)
    Node._publish_decision(node)
    assert node.post_ball_line_started_count == 2
    assert node.active_line_motion_started_at is None
    assert not node.camera_requests
    node.send_status(third['action'], third['command_id'], 'RUNNING', motion_id='line_forward_6')
    assert node.post_ball_line_started_count == 3
