"""Goal retreat waits one second without changing other retreat routes."""

import pytest

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
from test_motion_command_bridge import (
    FakeBridge, complete_active_motion, decoded_messages, executor_status, navigation_message,
)


@pytest.fixture
def clock(monkeypatch):
    now = [10.0]
    monkeypatch.setattr('mission_control.motion_command_bridge_node.time.monotonic', lambda: now[0])
    return now


def request_backward(bridge, command_id=1):
    bridge.navigation_command_callback(navigation_message(
        source='goal', action='GOAL_CAMERA90_BACKWARD_1', command_id=command_id,
    ))


def test_each_goal_retreat_waits_full_second_and_sends_only_once(clock):
    bridge = FakeBridge()
    for command_id in range(1, 4):
        clock[0] += 2.0
        request_backward(bridge, command_id)
        deadline = clock[0] + 1.0
        assert bridge.active_dwell_until == deadline
        assert bridge.active_motion_id == MotionCommandBridgeNode.GOAL_BACKWARD_PRE_DWELL_MARKER
        assert bridge.motion_in_progress
        assert len(bridge.executor_request_publisher.messages) == command_id - 1
        bridge._check_atomic_dwell(deadline - .001)
        assert len(bridge.executor_request_publisher.messages) == command_id - 1
        # An unsent motion cannot complete the pause through a stale status.
        bridge.executor_status_callback(executor_status(
            status='SUCCEEDED', command_id=command_id, request_id=command_id,
            motion_id='goal_camera_90_backward_1',
        ))
        assert bridge.active_dwell_until == deadline
        clock[0] = deadline
        bridge._check_atomic_dwell()
        request = decoded_messages(bridge.executor_request_publisher)[-1]
        assert request['motion_id'] == 'goal_camera_90_backward_1'
        assert request['command_id'] == command_id
        bridge._check_atomic_dwell(deadline + 1.0)
        assert len(bridge.executor_request_publisher.messages) == command_id
        complete_active_motion(bridge)
        assert not bridge.motion_in_progress
        assert decoded_messages(bridge.motion_status_publisher)[-1]['status'] == 'SUCCEEDED'


@pytest.mark.parametrize('other', ['GOAL_CAMERA_90_FORWARD', 'GOAL_CAMERA90_TURN_LEFT_1', 'SHOT', 'GOAL_CAMERA90_BACKWARD_1'])
def test_goal_retreat_pause_locks_out_interleaved_commands(clock, other):
    bridge = FakeBridge()
    request_backward(bridge)
    bridge.navigation_command_callback(navigation_message(action=other, command_id=2))
    assert decoded_messages(bridge.motion_status_publisher)[-1]['error_code'] == 'ATOMIC_SEQUENCE_LOCKED'
    assert bridge.active_dwell_until == 11.0
    assert bridge.queued_request_id is None
    assert not bridge.executor_request_publisher.messages


def test_retreat_pause_cannot_overlap_the_previous_motion(clock):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action='GOAL_CAMERA_90_FORWARD', command_id=1))
    request_backward(bridge, 2)
    assert decoded_messages(bridge.motion_status_publisher)[-1]['error_code'] == 'REJECTED_BUSY'
    assert bridge.active_dwell_until is None
    clock[0] = 20.0
    complete_active_motion(bridge)
    request_backward(bridge, 3)
    assert bridge.active_dwell_until == 21.0
    assert len(bridge.executor_request_publisher.messages) == 1


def test_clearing_pending_retreat_prevents_delayed_execution(clock):
    bridge = FakeBridge()
    request_backward(bridge)
    bridge._clear_active_request()
    bridge._check_atomic_dwell(100.0)
    assert not bridge.executor_request_publisher.messages


@pytest.mark.parametrize('motion_id', [
    'pickup_pre_backward_camera_down', 'pickup_lost_ball_backward_1', 'pickup_retreat_2',
    'pickup_first_backward_turn_right', 'pickup_second_backward_turn_left',
])
def test_pickup_retreat_requests_keep_existing_timing(clock, motion_id):
    bridge = FakeBridge()
    bridge._publish_executor_request(
        action='PICKUP_NOW', command_id=1, event_id=1, request_id=1,
        motion_id=motion_id, timeout_ms=12000,
    )
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == motion_id
    assert bridge.active_dwell_until is None


def test_hurdle_retreat_has_no_new_pause(clock):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action='HURDLE_LOST_BACKWARD_1', command_id=1))
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == 'pickup_lost_ball_backward_1'
    assert bridge.active_dwell_until is None
