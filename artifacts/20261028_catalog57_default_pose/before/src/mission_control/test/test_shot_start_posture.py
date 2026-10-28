"""Scoring preparation follows the last completed posture, not historical gait flags."""

import json
from pathlib import Path

import pytest
import yaml

from test_motion_command_bridge import (
    FakeBridge, navigation_message, decoded_messages, executor_status, complete_active_motion,
)


def finish_sequence(bridge):
    for _ in range(12):
        if not bridge.motion_in_progress:
            return
        if bridge.active_dwell_until is not None:
            bridge._check_atomic_dwell(bridge.active_dwell_until)
        else:
            complete_active_motion(bridge)
    raise AssertionError('motion sequence did not finish')


@pytest.mark.parametrize('actions,prepare', [
    (['GOAL_CAMERA90_FINE_FORWARD_1'], 'goal_fine_to_default'),
    (['GOAL_CAMERA90_CRAB_LEFT', 'GOAL_CAMERA90_FINE_FORWARD_2'], 'goal_fine_to_default'),
    (['GOAL_CAMERA90_TURN_LEFT_1'], 'goal_forward_to_default'),
    (['GOAL_CAMERA90_TURN_RIGHT_9'], 'goal_forward_to_default'),
    (['GOAL_CAMERA_90_FORWARD'], 'goal_forward_to_default'),
    (['POST_BALL_GOAL_TRANSITION'], 'goal_forward_to_default'),
    (['GOAL_CAMERA90_FINE_FORWARD_1', 'GOAL_CAMERA90_TURN_LEFT_1'], 'goal_forward_to_default'),
    (['GOAL_CAMERA90_CRAB_RIGHT', 'GOAL_CAMERA90_FINE_FORWARD_1',
      'GOAL_CAMERA90_TURN_LEFT_1'], 'goal_forward_to_default'),
])
def test_last_motion_selects_preparation_and_one_second_pause(monkeypatch, actions, prepare):
    clock = [10.]
    monkeypatch.setattr('mission_control.motion_command_bridge_node.time.monotonic', lambda: clock[0])
    bridge = FakeBridge()
    for command_id, action in enumerate(actions, 1):
        bridge.navigation_command_callback(navigation_message(action=action, command_id=command_id))
        finish_sequence(bridge)
    bridge.navigation_command_callback(navigation_message(action='SHOT', command_id=50))
    assert bridge.active_motion_id == prepare
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    count = len(bridge.executor_request_publisher.messages)
    clock[0] = 20.
    complete_active_motion(bridge)
    assert bridge.active_dwell_until == 21.
    assert bridge.motion_in_progress
    clock[0] = 20.5
    bridge.executor_status_callback(executor_status(**request, status='SUCCEEDED'))
    assert bridge.active_dwell_until == 21.
    bridge._check_atomic_dwell(20.999)
    assert len(bridge.executor_request_publisher.messages) == count
    bridge._check_atomic_dwell(21.)
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == 'goal_shot'
    assert len(bridge.executor_request_publisher.messages) == count + 1
    finish_sequence(bridge)
    assert not bridge.motion_in_progress


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED'])
def test_forward_posture_preparation_failure_never_sends_shot(status):
    bridge = FakeBridge()
    bridge.last_physical_motion_id = 'goal_camera_90_turn_left_1'
    bridge.navigation_command_callback(navigation_message(action='SHOT', command_id=50))
    complete_active_motion(bridge, status)
    bridge._check_atomic_dwell(float('inf'))
    assert not bridge.motion_in_progress
    assert [m['motion_id'] for m in decoded_messages(bridge.executor_request_publisher)] == [
        'goal_forward_to_default']


def test_shot_aliases_resolve_to_existing_default_posture_motions():
    root = Path(__file__).resolve().parents[3]
    aliases = yaml.safe_load((root / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    motions = {m['name']: m for m in json.loads((root / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
    assert aliases['goal_forward_to_default'] == '건오뒤에서 기본자세(카메라45도)'
    assert aliases['goal_fine_to_default'] == '건미세오뒤에서 기본자세45도'
    for alias in ('goal_forward_to_default', 'goal_fine_to_default'):
        assert '기본자세' in motions[aliases[alias]]['end_pose']


@pytest.mark.parametrize('previous,prepare', [
    ('goal_camera_90_turn_left_1', 'goal_forward_to_default'),
    ('goal_camera_90_fine_forward_1', 'goal_fine_to_default'),
])
def test_retry_after_failed_preparation_cannot_skip_it(previous, prepare):
    bridge = FakeBridge()
    bridge.last_physical_motion_id = previous
    bridge.navigation_command_callback(navigation_message(action='SHOT', command_id=1))
    complete_active_motion(bridge, 'FAILED', 'SDK_POSITION_TIMEOUT')
    assert bridge.last_physical_motion_id is None
    bridge.navigation_command_callback(navigation_message(action='SHOT', command_id=2))
    assert bridge.active_motion_id == prepare
    assert 'goal_shot' not in [m['motion_id'] for m in decoded_messages(bridge.executor_request_publisher)]
