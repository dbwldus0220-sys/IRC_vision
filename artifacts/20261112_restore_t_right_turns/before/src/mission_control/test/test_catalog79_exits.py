"""Check source79 data, unchanged exit directions and recovery reservation timing."""

import copy
from decimal import Decimal
import json
from pathlib import Path

import pytest
import yaml

from mission_control.mission_phase_manager import MissionPhaseManager
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_motion_command_bridge import (
    FakeBridge, decoded_messages, executor_status, navigation_message,
)

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261109_catalog79_exits'
SNAPSHOT = ROOT / 'artifacts/20261109_catalog80_pickup_hurdle_forward/before'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
ALIAS_PATH = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
ALIASES = yaml.safe_load((ROOT / ALIAS_PATH).read_text())['motion_aliases']


def catalog(path):
    motions = json.loads(path.read_text())['motions']
    result = {m['name']: m for m in motions}
    assert len(result) == len(motions)
    return result


SOURCE = catalog(UPDATE / 'source_robot_motions.json')
RUNTIME = catalog(ROOT / 'artifacts/robot_motions_runtime.json')


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_only_requested_motions_change_and_source_frames_are_preserved(kind):
    before = catalog(UPDATE / f'before/artifacts/robot_motions_{kind}.json')
    after = catalog(SNAPSHOT / f'artifacts/robot_motions_{kind}.json')
    renames = MANIFEST['renames']
    assert set(after) == (set(before) - set(renames)) | set(renames.values())
    for name, original in before.items():
        if name not in renames:
            assert after[name] == original, name
            continue
        expected = copy.deepcopy(SOURCE[renames[name]])
        for frame in expected['frames']:
            frame['name'] = MANIFEST['pose_renames'].get(frame['name'], frame['name'])
        for key in ('start_pose', 'end_pose'):
            expected[key] = MANIFEST['pose_renames'].get(expected[key], expected[key])
        if kind == 'runtime':
            expected['playback_speed'] = float(Decimal(str(expected['playback_speed'])) * Decimal('.95'))
            expected['completion']['position_tolerance_deg'] = 5.0
        assert after[renames[name]] == expected


def test_alias_ids_stay_stable_and_all_targets_resolve():
    before = yaml.safe_load((UPDATE / 'before' / ALIAS_PATH).read_text())['motion_aliases']
    after = yaml.safe_load((SNAPSHOT / ALIAS_PATH).read_text())['motion_aliases']
    assert after == {a: MANIFEST['renames'].get(n, n) for a, n in before.items()}
    assert set(MANIFEST['changed_aliases']) == {
        'pickup_second_backward_turn_left', 'post_shot_default_turn_left',
    }
    assert set(ALIASES.values()) <= RUNTIME.keys()
    assert set(Bridge.ACTION_TO_MOTION_ID.values()) <= ALIASES.keys()


@pytest.mark.parametrize('index,name', [
    (1, 'm골대후제자리우회전'), (2, 'p골대후제자리좌회전'),
])
def test_goal_exit_keeps_first_right_second_left_and_completes(index, name):
    manager = MissionPhaseManager(initial_phase='GOAL_APPROACH')
    manager.ball_sections_processed = index
    manager.current_phase = 'POST_SHOT_TURN'
    action = manager.post_shot_turn_action()
    assert action == ('POST_SHOT_TURN_RIGHT_9' if index == 1 else 'POST_SHOT_TURN_LEFT_4')
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        phase='POST_SHOT_TURN', source='goal', action=action))
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert ALIASES[request['motion_id']] == name
    assert RUNTIME[name]['repeat_count'] == 1
    bridge.executor_status_callback(executor_status(**request, status='SUCCEEDED'))
    assert not bridge.motion_in_progress
    assert manager.handle_post_shot_motion(action, 'SUCCEEDED')
    assert manager.current_phase == 'POST_SHOT_LINE_ALIGN'


@pytest.mark.parametrize('index,name', [
    (1, 'm공후제자리우회전'), (2, 'p공후 제자리좌회전'),
])
def test_pickup_exit_keeps_first_right_second_left(index, name):
    sequence = Bridge._pickup_motion_sequence({
        'source_command': {}, 'mission_progress': {'pickups_completed': index - 1},
    })
    assert ALIASES[sequence[-2]] == name
    assert RUNTIME[name]['repeat_count'] == 1


def test_recovery_capture_duration_matches_new_speed():
    alias = 'line_recovery_ball_first_left_4'
    motion = RUNTIME[ALIASES[alias]]
    duration = max(f['start_ms'] + f['time_ms'] for f in motion['frames'])
    seconds = duration * motion['repeat_count'] / motion['playback_speed'] / 1000
    assert Node.BALL_RECOVERY_DURATION_SEC[alias] == pytest.approx(seconds)
    assert seconds == pytest.approx(2.06)


@pytest.mark.parametrize('name', MANIFEST['renames'].values())
def test_new_motions_fit_timeout_and_end_at_standard_walking_support(name):
    motion = RUNTIME[name]
    duration = max(f['start_ms'] + f['time_ms'] for f in motion['frames'])
    assert duration * motion['repeat_count'] / motion['playback_speed'] + motion['completion']['settle_timeout_ms'] < Bridge.DEFAULT_TIMEOUT_MS
    end = motion['frames'][-1]
    standard = RUNTIME['p전진45(4회)']['frames'][-1]
    assert end['angles'] == standard['angles']
    assert end['torques'] == standard['torques']


def test_imported_pose_names_are_unambiguous():
    imported_names = {f['name'] for name in MANIFEST['renames'].values()
                      for f in RUNTIME[name]['frames']} | {'오뒤무게중심앞'}
    for name in imported_names:
        frames = [f for m in RUNTIME.values() for f in m['frames'] if f['name'] == name]
        assert frames
        assert all((f['angles'], f['torques']) == (frames[0]['angles'], frames[0]['torques']) for f in frames), name
