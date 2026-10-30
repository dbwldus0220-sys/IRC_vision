"""Verify Goal fine-to-turn support changes, dwell boundaries and source fidelity."""

import copy
import json
from pathlib import Path

import pytest
import yaml

from test_gait_transitions import finish_request, requests
from test_motion_command_bridge import FakeBridge, decoded_messages, navigation_message
from test_transition_dwells import check_both_pauses, clock


ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261031_goal_fine_turn'
ALIAS_PATH = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')


@pytest.mark.parametrize('count', range(1, 5))
@pytest.mark.parametrize('side', ['LEFT', 'RIGHT'])
@pytest.mark.parametrize('queued', [False, True])
def test_goal_fine_then_turn_waits_before_and_after_preparation(clock, count, side, queued):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(navigation_message(
        source='goal', action=f'GOAL_CAMERA90_FINE_FORWARD_{count}', command_id=1,
    ))
    turn = navigation_message(source='goal', action=f'GOAL_CAMERA90_TURN_{side}_1', command_id=2)
    if queued:
        bridge.navigation_command_callback(turn)
        assert len(requests(bridge)) == 1
        assert bridge.transition_dwell_until is None
        clock[0] += 2.0
        finish_request(bridge)
    else:
        finish_request(bridge)
        bridge.navigation_command_callback(turn)
    target = f'goal_camera_90_turn_{side.lower()}_1'
    check_both_pauses(bridge, clock, 'fine_to_turn_ready_90', target)
    finish_request(bridge)
    assert not bridge.motion_in_progress


@pytest.mark.parametrize('action', ['GOAL_CAMERA90_TURN_LEFT_1', 'GOAL_CAMERA90_TURN_RIGHT_1',
                                  'TURN_LEFT', 'TURN_RIGHT'])
def test_goal_fine_turn_cannot_use_handoff_pause_bypass(clock, action):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = 'goal_camera_90_fine_forward_2'
    bridge.goal_handoff_command_id = 2
    # No heartbeat: the completed Goal gait still determines the 90-degree pose.
    bridge.navigation_command_callback(navigation_message(source='goal', action=action, command_id=2))
    check_both_pauses(bridge, clock, 'fine_to_turn_ready_90', bridge.motion_id_for_action(action))


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED'])
def test_goal_preparation_failure_never_starts_turn(clock, status):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = 'goal_camera_90_fine_forward_4'
    bridge.navigation_command_callback(navigation_message(
        source='goal', action='GOAL_CAMERA90_TURN_RIGHT_1', command_id=2,
    ))
    clock[0] += 1.0
    bridge._check_atomic_dwell()
    finish_request(bridge, status, 'TEST_FAILURE')
    clock[0] += 10.0
    bridge._check_atomic_dwell()
    assert [r['motion_id'] for r in requests(bridge)] == ['fine_to_turn_ready_90']
    assert not bridge.motion_in_progress
    assert decoded_messages(bridge.motion_status_publisher)[-1]['error_code'] == 'TURN_PREPARATION_FAILED'


@pytest.mark.parametrize('filename', ['robot_motions_pc.json', 'robot_motions_runtime.json'])
def test_only_requested_source_motion_added_with_correct_runtime_speed(filename):
    source = json.loads((UPDATE / 'source62.json').read_text())['motions']
    expected = copy.deepcopy(next(m for m in source if m['name'] == 'r미세오뒤에서 오뒤90도'))
    if 'runtime' in filename:
        expected['playback_speed'] = .9975
        expected['completion']['position_tolerance_deg'] = 5.0
    before = json.loads((UPDATE / 'before/artifacts' / filename).read_text())
    actual = json.loads((ROOT / 'artifacts' / filename).read_text())
    assert actual == {**before, 'motions': [*before['motions'], expected]}
    assert max(f['start_ms'] + f['time_ms'] for f in expected['frames']) == 160
    aliases = yaml.safe_load((ROOT / ALIAS_PATH).read_text())['motion_aliases']
    old_aliases = yaml.safe_load((UPDATE / 'before' / ALIAS_PATH).read_text())['motion_aliases']
    assert aliases == {**old_aliases, 'fine_to_turn_ready_90': expected['name']}
