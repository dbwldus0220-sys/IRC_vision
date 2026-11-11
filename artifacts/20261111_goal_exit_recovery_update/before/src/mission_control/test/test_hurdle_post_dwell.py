"""Verify the physical hurdle's speed and post-success mission lock."""

import json
from pathlib import Path

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode
from test_hurdle_positioning import HurdleHarness, clock, enter_positioning
from test_motion_command_bridge import (
    FakeBridge, complete_active_motion, decoded_messages, executor_status,
    hurdle_fine_sequence_message, navigation_message,
)


def start_hurdle(bridge, clock):
    bridge.navigation_command_callback(hurdle_fine_sequence_message())
    clock[0] = 15.0
    complete_active_motion(bridge)
    clock[0] = bridge.active_dwell_until
    bridge._check_atomic_dwell()
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request['motion_id'] == 'hurdle'
    return request


def test_runtime_hurdle_speed_is_original_times_point_95():
    root = Path(__file__).resolve().parents[3]
    def motion(relative):
        return next(m for m in json.loads((root / relative).read_text())['motions']
                    if m['name'] == 't허들찐')
    source = motion('artifacts/20261109_catalog81_hurdle/source_robot_motions.json')
    pc = motion('artifacts/robot_motions_pc.json')
    runtime = motion('artifacts/robot_motions_runtime.json')
    assert pc['playback_speed'] == source['playback_speed'] == 1.0
    assert runtime['playback_speed'] == pytest.approx(source['playback_speed'] * .95)


@pytest.mark.parametrize('duplicate', ['RUNNING', 'SUCCEEDED', 'FAILED', 'TIMEOUT', 'CANCELLED'])
def test_post_hurdle_pause_blocks_commands_and_ignores_late_status(clock, duplicate):
    bridge = FakeBridge()
    request = start_hurdle(bridge, clock)
    clock[0] = 25.0
    complete_active_motion(bridge)
    assert bridge.active_motion_id == bridge.HURDLE_POST_GO_DWELL_MARKER
    assert bridge.active_dwell_until == 26.0
    assert bridge.motion_in_progress
    assert not any(s['status'] == 'SUCCEEDED' for s in decoded_messages(bridge.motion_status_publisher))
    sent = len(bridge.executor_request_publisher.messages)
    clock[0] = 25.4
    bridge.executor_status_callback(executor_status(**request, status=duplicate))
    assert bridge.active_dwell_until == 26.0
    for command_id, action in enumerate(('STRAIGHT', 'GO'), 8200):
        bridge.navigation_command_callback(navigation_message(action=action, command_id=command_id))
        assert decoded_messages(bridge.motion_status_publisher)[-1]['error_code'] == 'ATOMIC_SEQUENCE_LOCKED'
    assert bridge.queued_request_id is None
    clock[0] = 25.999
    bridge._check_atomic_dwell()
    assert bridge.motion_in_progress
    assert len(bridge.executor_request_publisher.messages) == sent
    clock[0] = 26.0
    bridge._check_atomic_dwell()
    assert not bridge.motion_in_progress
    assert bridge.active_dwell_until is None
    terminals = [s for s in decoded_messages(bridge.motion_status_publisher) if s['status'] == 'SUCCEEDED']
    assert len(terminals) == 1 and terminals[0]['action'] == 'GO'
    bridge._check_atomic_dwell()
    bridge.executor_status_callback(executor_status(**request, status='SUCCEEDED'))
    assert len([s for s in decoded_messages(bridge.motion_status_publisher) if s['status'] == 'SUCCEEDED']) == 1
    bridge.navigation_command_callback(navigation_message(action='STRAIGHT', command_id=8300))
    assert len(bridge.executor_request_publisher.messages) == sent + 1
    assert bridge.active_action == 'STRAIGHT'


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED'])
def test_unsuccessful_hurdle_does_not_wait_or_report_success(clock, status):
    bridge = FakeBridge()
    start_hurdle(bridge, clock)
    complete_active_motion(bridge, status, 'SDK_COMMUNICATION_ERROR' if status == 'FAILED' else '')
    assert not bridge.motion_in_progress
    assert bridge.active_dwell_until is None
    assert bridge.hurdle_sequence_fine_completed == 1
    assert decoded_messages(bridge.motion_status_publisher)[-1]['status'] == status


def test_node_completes_hurdle_once_after_pause_and_resumes_with_fresh_line(clock):
    node, bridge = HurdleHarness(), FakeBridge()
    command = enter_positioning(node, clock)
    forwarded = 0

    def sync_status():
        nonlocal forwarded
        for message in bridge.motion_status_publisher.messages[forwarded:]:
            MotionDecisionNode._motion_status_callback(node, message)
        forwarded = len(bridge.motion_status_publisher.messages)

    bridge.navigation_command_callback(String(data=json.dumps(command)))
    sync_status()
    clock[0] += 5.0
    complete_active_motion(bridge)
    sync_status()
    clock[0] = bridge.active_dwell_until
    bridge._check_atomic_dwell()
    sync_status()
    clock[0] += 8.0
    complete_active_motion(bridge)
    sync_status()
    node.receive({'detected': False})
    node.receive_line()
    assert node.phase_manager.hurdles_completed == 0
    assert node.active_special_action == 'GO'
    assert not any(m['valid'] for m in node.publish())
    clock[0] = bridge.active_dwell_until - .001
    bridge._check_atomic_dwell()
    sync_status()
    assert node.active_special_action == 'GO'
    clock[0] += .001
    bridge._check_atomic_dwell()
    sync_status()
    assert node.phase_manager.hurdles_completed == 1
    assert node.mission_phase == 'LINE_TRACK'
    assert node.active_special_command_id is None
    assert not node.general_motion_gate.locked
    node.receive_line()
    following = node.publish()[-1]
    assert following['valid'] and following['action'] == 'STRAIGHT'
    bridge.navigation_command_callback(String(data=json.dumps(following)))
    sync_status()
    assert bridge.active_action == 'STRAIGHT'
