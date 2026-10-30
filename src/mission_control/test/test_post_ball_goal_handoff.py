"""Preview Goal during timed Line travel and dispatch at its completed boundary."""

import json
from types import SimpleNamespace

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_wait_refresh_and_fine_settle import LiveInputHarness
from test_mission_phase_flow import line_info
from test_goal_target_steering import target_sample
from test_motion_command_bridge import FakeBridge, navigation_message, decoded_messages


@pytest.fixture
def clock(monkeypatch):
    now = [10.0]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: now[0])
    monkeypatch.setattr(Node, '_current_ros_time_ns', lambda _: round(now[0] * 1e9))
    return now


def receive(node, clock, source, info):
    Node._info_callback(node, source)(String(data=json.dumps({
        **info, 'rgb_stamp_ns': round(clock[0] * 1e9),
    })))


def start_line(clock):
    node = LiveInputHarness(phase='LINE_TRACK_AFTER_PICKUP')
    node.camera_requests = []
    node.goal_camera_publisher = SimpleNamespace(
        publish=lambda msg: node.camera_requests.append(json.loads(msg.data)),
        get_subscription_count=lambda: 1,
    )
    node.phase_manager.pickups_completed = 1
    node.phase_manager.ball_grasp_results[1] = 'GRABBED'
    receive(node, clock, 'line', line_info())
    Node._publish_decision(node)
    first = node.publisher.messages[-1]
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    assert node.post_ball_line_run_until == 17.0
    return node, first


def raise_camera(node, clock):
    clock[0] = 15.0
    Node._publish_decision(node)
    assert len(node.camera_requests) == 1
    clock[0] = 15.4
    Node._executor_heartbeat_callback(node, String(data=json.dumps({
        'sequence': 99, 'goal_head_prepare_id': node.camera_requests[-1]['request_id'],
        'goal_head_transition_complete': True,
    })))


def goal(angle=0, **updates):
    return {**target_sample(angle), 'confirmation_confirmed': True,
            'confirmation_hits': 20, **updates}


def test_camera_starts_at_five_while_locked_and_only_new_head_up_frames_are_used(clock):
    node, first = start_line(clock)
    clock[0] = 14.999
    Node._publish_decision(node)
    receive(node, clock, 'goal', goal())
    assert not node.camera_requests
    assert node.post_ball_goal_preview is None
    raise_camera(node, clock)
    # A delayed pre-raise image must not become the preview.
    Node._info_callback(node, 'goal')(String(data=json.dumps({
        **goal(), 'rgb_stamp_ns': round(15.1 * 1e9),
    })))
    assert node.post_ball_goal_preview is None
    clock[0] = 15.5
    receive(node, clock, 'goal', goal(0))
    assert node.post_ball_goal_preview['action'] == 'GOAL_CAMERA_90_FORWARD_2'
    clock[0] = 16.0
    receive(node, clock, 'goal', goal(25))
    assert node.post_ball_goal_preview['action'] == 'GOAL_CAMERA90_RECOVER_RIGHT_4'
    old_line = node.latest_info['line']
    receive(node, clock, 'line', line_info(heading=-60.))
    assert node.latest_info['line'] == old_line
    Node._publish_decision(node)
    assert len(node.publisher.messages) == 1
    assert len(node.camera_requests) == 1
    assert node.general_motion_gate.active_command_id == first['command_id']
    assert node.mission_phase == 'LINE_TRACK_AFTER_PICKUP'


@pytest.mark.parametrize('angle,action', [
    (0, 'GOAL_CAMERA_90_FORWARD_2'),
    (25, 'GOAL_CAMERA90_RECOVER_RIGHT_4'),
    (-25, 'GOAL_CAMERA90_RECOVER_LEFT_4'),
    (50, 'GOAL_CAMERA90_TURN_RIGHT_5'),
    (-50, 'GOAL_CAMERA90_TURN_LEFT_3'),
])
def test_fresh_target_dispatches_in_line_completion_callback(clock, angle, action):
    node, first = start_line(clock)
    raise_camera(node, clock)
    clock[0] = 15.5
    receive(node, clock, 'goal', goal(-angle))
    # The actual boundary uses current steering, not the first head-up snapshot.
    clock[0] = 16.4
    receive(node, clock, 'goal', goal(angle))
    clock[0] = 16.5
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    assert node.mission_phase == 'GOAL_APPROACH'
    assert node.publisher.messages[-1]['action'] == action
    assert len(node.publisher.messages) == 2
    assert node.goal_post_motion_dwell_until is None
    assert node.pre_motion_settle_started_at is None
    assert node.active_special_command_id is None
    assert not any(m['action'] == 'POST_BALL_GOAL_TRANSITION' for m in node.publisher.messages)


@pytest.mark.parametrize('bad', ['stale', 'candidate', 'lost', 'future', 'before_preview'])
def test_boundary_never_uses_stale_or_unconfirmed_goal(clock, bad):
    node, first = start_line(clock)
    raise_camera(node, clock)
    clock[0] = 15.5
    receive(node, clock, 'goal', goal())
    if bad != 'stale':
        clock[0] = 16.4
        info = goal()
        if bad == 'candidate':
            info.update(detected=False, raw_detected=True, confirmation_confirmed=False)
        elif bad == 'lost':
            info.update(detected=False, raw_detected=False)
        if bad in {'future', 'before_preview'}:
            stamp = round((20.0 if bad == 'future' else 12.0) * 1e9)
            Node._info_callback(node, 'goal')(String(data=json.dumps({**info, 'rgb_stamp_ns': stamp})))
        else:
            receive(node, clock, 'goal', info)
    clock[0] = 16.5
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    assert node.mission_phase == 'GOAL_APPROACH'
    assert not any(m['valid'] for m in node.publisher.messages[1:])
    assert not any('TURN_LEFT_6' in m['action'] for m in node.publisher.messages)
    clock[0] = 16.6
    receive(node, clock, 'goal', goal())
    Node._publish_decision(node)
    assert node.publisher.messages[-1]['action'] == 'GOAL_CAMERA_90_FORWARD_2'


def test_failed_line_never_starts_goal_from_preview(clock):
    node, first = start_line(clock)
    clock[0] = 16.4
    receive(node, clock, 'goal', goal())
    clock[0] = 16.5
    node.send_status(first['action'], first['command_id'], 'FAILED')
    Node._publish_decision(node)
    assert node.mission_phase == 'LINE_TRACK_AFTER_PICKUP'
    assert len(node.publisher.messages) == 1


def test_line_to_goal_fine_preparation_has_no_fixed_dwell():
    bridge = FakeBridge()
    bridge.last_physical_motion_id = 'line_forward_6'
    bridge.navigation_command_callback(navigation_message(action='GOAL_CAMERA90_FINE_FORWARD_3', command_id=1))
    requests = decoded_messages(bridge.executor_request_publisher)
    assert requests[0]['motion_id'] == 'pickup_fine_prepare'
    assert bridge.transition_pending_request is None
    assert not any('__TRANSITION_PRE_DWELL__' in str(m) for m in bridge.motion_status_publisher.messages)


@pytest.mark.parametrize('previous', ['line_forward_6', 'post_ball_line_turn_right_3'])
@pytest.mark.parametrize('action', [
    'GOAL_CAMERA_90_FORWARD_2', 'GOAL_CAMERA90_FINE_FORWARD_3',
    'GOAL_CAMERA90_RECOVER_RIGHT_4', 'GOAL_CAMERA90_TURN_LEFT_3',
    'GOAL_CAMERA90_CRAB_RIGHT', 'GOAL_CAMERA90_CRAB_LEFT', 'GOAL_CAMERA90_BACKWARD_1',
])
def test_first_goal_navigation_has_no_transition_dwell(previous, action):
    from test_motion_command_bridge import complete_active_motion

    bridge = FakeBridge()
    bridge.last_physical_motion_id = previous
    bridge.navigation_command_callback(navigation_message(
        action=action, command_id=1, phase='GOAL_APPROACH',
        source_command={'post_ball_goal_handoff': True},
    ))
    assert bridge.executor_request_publisher.messages
    for _ in range(6):
        assert bridge.active_dwell_until is None
        assert bridge.transition_dwell_until is None
        if not bridge.motion_in_progress:
            break
        complete_active_motion(bridge)
    assert not bridge.motion_in_progress
    statuses = [json.loads(m.data) for m in bridge.motion_status_publisher.messages]
    assert not any('DWELL' in str(s.get('motion_id')) for s in statuses)
    # The exemption belongs only to the first transition command.
    bridge.navigation_command_callback(navigation_message(
        action='GOAL_CAMERA90_BACKWARD_1', command_id=2, phase='GOAL_APPROACH',
    ))
    assert bridge.active_dwell_until is not None


def test_goal_preview_requires_matching_camera_acknowledgement(clock):
    node, first = start_line(clock)
    clock[0] = 15.0
    Node._publish_decision(node)
    request_id = node.camera_requests[-1]['request_id']
    clock[0] = 15.4
    for ack_id, complete in [(request_id + 1, True), (request_id, False)]:
        Node._executor_heartbeat_callback(node, String(data=json.dumps({
            'sequence': 99, 'goal_head_prepare_id': ack_id,
            'goal_head_transition_complete': complete,
        })))
        receive(node, clock, 'goal', goal())
        assert node.post_ball_goal_preview is None
    Node._publish_decision(node)
    assert len(node.camera_requests) == 2
    assert node.camera_requests[-1]['request_id'] == request_id
    assert len(node.publisher.messages) == 1


def test_camera_connection_can_recover_after_last_line_finishes(clock):
    node, first = start_line(clock)
    node.goal_camera_publisher.get_subscription_count = lambda: 0
    clock[0] = 15.0
    Node._publish_decision(node)
    assert not node.camera_requests
    clock[0] = 15.1
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    assert node.mission_phase == 'GOAL_APPROACH'
    assert not node.publisher.messages[-1]['valid']
    node.goal_camera_publisher.get_subscription_count = lambda: 1
    clock[0] = 15.2
    Node._publish_decision(node)
    assert len(node.camera_requests) == 1
    clock[0] = 15.7
    Node._executor_heartbeat_callback(node, String(data=json.dumps({
        'sequence': 99, 'goal_head_prepare_id': node.camera_requests[-1]['request_id'],
        'goal_head_transition_complete': True,
    })))
    clock[0] = 15.8
    receive(node, clock, 'goal', goal())
    Node._publish_decision(node)
    assert node.publisher.messages[-1]['action'] == 'GOAL_CAMERA_90_FORWARD_2'


@pytest.mark.parametrize('blocked', ['failed', 'not_grabbed', 'hold'])
def test_camera_preparation_respects_motion_and_grasp_gates(clock, blocked):
    node, first = start_line(clock)
    if blocked == 'failed':
        node.post_ball_line_run_failed = True
    elif blocked == 'not_grabbed':
        node.phase_manager.ball_grasp_results[1] = 'NOT_GRABBED'
    else:
        node.executor_auto_ready = False
    clock[0] = 15.0
    Node._publish_decision(node)
    assert not node.camera_requests
