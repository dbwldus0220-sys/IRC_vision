"""A later pickup must not inherit the previous Goal camera handoff."""

import json
from types import SimpleNamespace

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_mission_phase_flow import line_info
from test_post_ball_goal_handoff import goal, receive
from test_wait_refresh_and_fine_settle import LiveInputHarness


@pytest.fixture
def clock(monkeypatch):
    now = [10.0]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: now[0])
    monkeypatch.setattr(Node, '_current_ros_time_ns', lambda _: round(now[0] * 1e9))
    return now


def finish_pickup(ball_index, *, status='SUCCEEDED', grasp='GRABBED'):
    node = LiveInputHarness(phase='BALL_APPROACH')
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = Node.LINE_TURN_PRE_MOTION_SETTLE_SEC
    node.phase_manager.pickups_completed = ball_index - 1
    node.phase_manager.pickups_executed = ball_index - 1
    node.phase_manager.ball_sections_processed = ball_index - 1
    node.phase_manager.shots_completed = ball_index - 1
    node.phase_manager.shots_executed = ball_index - 1
    if ball_index == 2:
        node.post_ball_goal_camera_request_id = 9_000_000_000
        node.post_ball_goal_camera_ready_at = 9.4
        node.post_ball_goal_camera_last_sent = 9.0
        node.post_ball_goal_preview = {'action': 'GOAL_CAMERA90_TURN_LEFT_3'}
        node.post_ball_goal_handoff_pending = True
    node.command_id = 10
    assert node.phase_manager.start_special_action('PICKUP_NOW', node.command_id)
    node.send_status('PICKUP_NOW', node.command_id, 'RUNNING', motion_id='pickup_400')
    assert node.phase_manager.record_active_pickup_grasp_result(grasp)
    node.send_status('PICKUP_NOW', node.command_id, status, motion_id=(
        'pickup_first_backward_turn_right' if ball_index == 1
        else 'pickup_second_backward_turn_left'
    ))
    return node


@pytest.mark.parametrize('ball_index', [1, 2])
@pytest.mark.parametrize('heading,action', [
    (0., 'STRAIGHT'),
    (-56., 'LINE_HEADING_TURN_LEFT_3'),
    (56., 'LINE_HEADING_TURN_RIGHT_5'),
])
def test_both_pickups_resume_line_then_request_their_own_goal_camera(clock, ball_index, heading, action):
    node = finish_pickup(ball_index)
    assert node.mission_phase == 'POST_BALL_LINE_ALIGN'
    assert node.planner.post_ball_line_search_direction == ('RIGHT' if ball_index == 1 else 'LEFT')
    if ball_index == 2:
        assert node.post_ball_goal_camera_request_id is None
        assert getattr(node, 'post_ball_goal_camera_ready_at', None) is None
        assert node.post_ball_goal_camera_last_sent is None
        assert node.post_ball_goal_preview is None
        assert node.post_ball_goal_handoff_pending is False
    camera_requests = []
    node.goal_camera_publisher = SimpleNamespace(
        publish=lambda message: camera_requests.append(json.loads(message.data)),
        get_subscription_count=lambda: 1,
    )
    # Production settling and expiry are both 0.5 s. Input arrives before each tick.
    for tick in range(8):
        clock[0] = 10.0 + tick * .1
        receive(node, clock, 'line', line_info(heading=heading))
        assert node.vision_transport['line']['status'] == 'accepted'
        clock[0] += .04
        Node._publish_decision(node)
        assert node.last_selected_decision.action == action
        if node.publisher.messages:
            break
    command = node.publisher.messages[-1]
    assert command['valid'] and command['action'] == action
    assert node.mission_phase == 'LINE_TRACK_AFTER_PICKUP'
    node.send_status(action, command['command_id'], 'RUNNING')
    assert node.post_ball_line_started_count == 1
    from test_three_line_goal_handoff import reserve_line
    for ordinal in (2, 3):
        following = reserve_line(node, clock)
        assert following['source_command']['post_pickup_line_index'] == ordinal
        assert not camera_requests
        node.send_status(action, command['command_id'], 'SUCCEEDED')
        command, action = following, following['action']
        node.send_status(action, command['command_id'], 'RUNNING')
        assert node.post_ball_line_started_count == ordinal
        assert getattr(node, 'correction_post_motion_dwell_until', None) is None
    request_id = command['source_command']['goal_head_prepare_id']
    assert request_id != 9_000_000_000
    assert getattr(node, 'post_ball_goal_camera_ready_at', None) is None
    # A delayed acknowledgement from the first ball must not unlock the second.
    Node._executor_heartbeat_callback(node, String(data=json.dumps({
        'sequence': 99, 'goal_head_prepare_id': 9_000_000_000,
        'goal_head_transition_complete': True,
    })))
    assert getattr(node, 'post_ball_goal_camera_ready_at', None) is None
    clock[0] += .4
    Node._executor_heartbeat_callback(node, String(data=json.dumps({
        'sequence': 100, 'goal_head_prepare_id': request_id,
        'goal_head_transition_complete': True,
    })))
    for _ in range(3):
        clock[0] += .1
        receive(node, clock, 'goal', goal())
    receive(node, clock, 'line', line_info())
    assert node.vision_transport['line']['status'] == 'goal_camera_raised_line_frozen'
    node.send_status(action, command['command_id'], 'SUCCEEDED')
    goal_command = node.publisher.messages[-1]
    node.send_status(goal_command['action'], goal_command['command_id'], 'RUNNING')
    assert node.mission_phase == 'GOAL_APPROACH'
    assert node.publisher.messages[-1]['action'] == 'GOAL_CAMERA_90_FORWARD_2'
    assert node.publisher.messages[-1]['valid']


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED'])
def test_second_pickup_failure_does_not_enter_line_or_reset_camera(clock, status):
    node = finish_pickup(2, status=status)
    assert node.mission_phase == 'BALL_APPROACH'
    assert node.phase_manager.ball_mode_entry_blocked()
    assert node.post_ball_goal_camera_request_id == 9_000_000_000
    assert not node.publisher.messages


def test_empty_second_pickup_clears_previous_camera_and_accepts_line(clock):
    node = finish_pickup(2, grasp='NOT_GRABBED')
    assert node.post_ball_goal_camera_request_id is None
    assert getattr(node, 'post_ball_goal_camera_ready_at', None) is None
    assert node.post_ball_goal_camera_last_sent is None
    receive(node, clock, 'line', line_info())
    Node._publish_decision(node)
    assert node.vision_transport['line']['status'] == 'accepted'
    assert node.phase_manager.ball_sections_processed == 2
    assert node.mission_phase != 'GOAL_APPROACH'
