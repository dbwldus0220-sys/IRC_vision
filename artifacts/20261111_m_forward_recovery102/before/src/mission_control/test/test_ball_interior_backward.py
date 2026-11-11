"""Replay interior Ball loss, retreat completion, and fresh reacquisition without motors."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_wait_refresh_and_fine_settle import LiveInputHarness
from test_motion_decision_planner import ball_info
from test_motion_command_bridge import FakeBridge, decoded_messages, navigation_message


@pytest.fixture
def clock(monkeypatch):
    now = [10.0]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: now[0])
    monkeypatch.setattr(Node, '_current_ros_time_ns', lambda _: round(now[0] * 1e9))
    return now


def observe(node, clock, **updates):
    clock[0] += .13
    Node._info_callback(node, 'ball')(String(data=json.dumps({
        'detected': False, 'raw_detected': False,
        'rgb_stamp_ns': round(clock[0] * 1e9), **updates,
    })))
    Node._publish_decision(node)


def visible(**updates):
    return ball_info(image_width=1280, image_height=720,
                     bbox=[500, 300, 600, 400], depth_m=1.0, **updates)


def start_retreat(clock):
    node = LiveInputHarness(phase='BALL_APPROACH')
    node.planner._update_ball_tracking(visible(), 0.0)
    node.ball_approach_alignment_pending = True
    for _ in range(4):
        observe(node, clock)
        assert not any(m['valid'] for m in node.publisher.messages)
    observe(node, clock)
    command = node.publisher.messages[-1]
    assert command['action'] == 'BALL_LOST_BACKWARD_2'
    assert node.planner.ball_interior_loss_backward_state == 'PENDING'
    node.send_status(command['action'], command['command_id'], 'RUNNING')
    return node, command


def test_retreat_then_forward_needs_success_and_new_confirmed_loss(clock):
    node, command = start_retreat(clock)
    for _ in range(6):
        observe(node, clock)
    assert not any(m['action'] == 'BALL_LOST_FORWARD_4' for m in node.publisher.messages)
    node.send_status(command['action'], command['command_id'], 'SUCCEEDED')
    assert node.planner.ball_interior_loss_backward_state == 'COMPLETE'
    assert getattr(node, "correction_post_motion_dwell_until", None) is None
    assert node.ball_loss_confirmation.confirmed is False
    for _ in range(4):
        observe(node, clock)
        assert not any(m['action'] == 'BALL_LOST_FORWARD_4' for m in node.publisher.messages)
    observe(node, clock)
    assert node.publisher.messages[-1]['action'] == 'BALL_LOST_FORWARD_4'


@pytest.mark.parametrize('status', ['FAILED', 'CANCELLED', 'TIMEOUT', 'REJECTED'])
def test_unsuccessful_retreat_cannot_authorize_forward(clock, status):
    node, command = start_retreat(clock)
    node.send_status(command['action'], command['command_id'], status)
    for _ in range(8):
        observe(node, clock)
    assert node.planner.ball_interior_loss_backward_state != 'COMPLETE'
    assert not any(m['action'] == 'BALL_LOST_FORWARD_4' for m in node.publisher.messages)


@pytest.mark.parametrize('confirmed', [False, True])
def test_ball_or_candidate_during_retreat_cancels_forward_fallback(clock, confirmed):
    node, command = start_retreat(clock)
    observe(node, clock, **visible(detected=confirmed, raw_detected=True))
    assert node.planner.ball_interior_loss_backward_state == 'IDLE'
    node.send_status(command['action'], command['command_id'], 'SUCCEEDED')
    observe(node, clock, **visible(detected=confirmed, raw_detected=True))
    assert not any(m['action'] == 'BALL_LOST_FORWARD_4' for m in node.publisher.messages)


def test_retreat_action_uses_existing_backward_motion():
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        action='BALL_LOST_BACKWARD_2', source='ball', command_id=9100,
    ))
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request['motion_id'] == 'ball_lost_camera_down_backward_2'


@pytest.mark.parametrize('stage', ['INITIAL', 'FINE'])
def test_pickup_checkpoint_returns_to_new_loss_check_after_backward(clock, stage):
    node = LiveInputHarness(phase='BALL_APPROACH')
    assert node.phase_manager.start_special_action('PICKUP_NOW', 1)
    node.command_id = 1
    pickup = {'command_id': 1}
    marker = Node.PICKUP_INITIAL_ALIGN_MARKER if stage == 'INITIAL' else Node.PICKUP_FINE_ALIGN_MARKER
    node.send_status('PICKUP_NOW', pickup['command_id'], 'RUNNING', motion_id=marker)
    node.planner._update_ball_tracking(visible(), 0.0)
    for _ in range(5):
        observe(node, clock)
    assert node.publisher.messages[-1]['action'] == f'BALL_PICKUP_{stage}_SEARCH_BACKWARD_2'
    assert node.planner.ball_interior_loss_backward_state == 'PENDING'
    clock[0] += 1.0
    node.send_status('PICKUP_NOW', pickup['command_id'], 'RUNNING', motion_id=marker)
    assert node.planner.ball_interior_loss_backward_state == 'COMPLETE'
    for _ in range(4):
        observe(node, clock)
        assert node.publisher.messages[-1]['action'] != f'BALL_PICKUP_{stage}_SEARCH_FORWARD_4'
    observe(node, clock)
    assert node.publisher.messages[-1]['action'] == f'BALL_PICKUP_{stage}_SEARCH_FORWARD_4'
    assert node.active_special_command_id == pickup['command_id']
    assert not node.pickup_fixed_sequence_started


def test_retreat_alias_reuses_two_repeat_runtime_motion():
    from pathlib import Path
    import yaml

    root = Path(__file__).resolve().parents[3]
    aliases = yaml.safe_load((root / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    motions = json.loads((root / 'artifacts/robot_motions_runtime.json').read_text())['motions']
    target = aliases['ball_lost_camera_down_backward_2']
    assert target == '찐후진실전-2(2회)'
    motion = next(m for m in motions if m['name'] == target)
    assert motion['repeat_count'] == 2


@pytest.mark.parametrize('stage', ['INITIAL', 'FINE'])
def test_pickup_motor_failure_in_retreat_cannot_return_a_success_checkpoint(stage):
    from test_motion_command_bridge import (
        enter_pickup_fine_alignment, fine_alignment_message, complete_active_motion,
    )
    from mission_control.safety_interlock import RECOVERABLE_MOTOR_ERROR_CODES

    bridge = FakeBridge()
    if stage == 'FINE':
        enter_pickup_fine_alignment(bridge)
    else:
        bridge.navigation_command_callback(navigation_message(action='PICKUP_NOW'))
    bridge.navigation_command_callback(fine_alignment_message(
        f'BALL_PICKUP_{stage}_SEARCH_BACKWARD_2', command_id=9100,
    ))
    complete_active_motion(bridge, status='FAILED', error_code=next(iter(RECOVERABLE_MOTOR_ERROR_CODES)))
    assert not bridge.pickup_initial_align_waiting
    assert not bridge.pickup_fine_align_waiting
    assert not bridge.motion_in_progress
    assert decoded_messages(bridge.motion_status_publisher)[-1]['status'] == 'FAILED'
