"""Validate source76 data and actual bridge routes without motor access."""

import copy
import json
from pathlib import Path

import pytest
import yaml

from mission_control.ball_fine_approach import duration_sec
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from mission_control.pickup_walk_reservation import PICKUP_CRAB_DURATIONS
from test_catalog63_transitions import start_case
from test_motion_command_bridge import (
    FakeBridge, decoded_messages, executor_status, fine_alignment_message, navigation_message,
)
from test_transition_dwells import clock, check_both_pauses

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261107_catalog76_turn_crab'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
SNAPSHOT = ROOT / 'artifacts/20261108_catalog77_p_motions/before'
ALIAS_PATH = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')


def catalog(path):
    motions = json.loads(path.read_text())['motions']
    result = {m['name']: m for m in motions}
    assert len(result) == len(motions)
    return result


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_exact_source_frames_and_unrelated_motion_preservation(kind):
    source = catalog(UPDATE / 'source_robot_motions.json')
    before = catalog(UPDATE / f'before/artifacts/robot_motions_{kind}.json')
    after = catalog(SNAPSHOT / f'artifacts/robot_motions_{kind}.json')
    renames = MANIFEST['renames']
    assert set(after) == (set(before) - set(renames) - set(MANIFEST['removed_motions'])) | set(renames.values())
    for old, motion in before.items():
        if old in MANIFEST['removed_motions']:
            continue
        if old not in renames:
            assert after[old] == motion, old
            continue
        name = renames[old]
        expected = copy.deepcopy(source['tt미세왼옆0도' if name == 'tt미세왼옆0도(2회)' else name])
        expected['name'] = name
        if name == 'tt미세왼옆0도(2회)':
            expected['repeat_count'] = 2
        if name == 't제자리좌회전45도(6회)':
            expected['playback_speed'] = .9
        poses = MANIFEST['pose_renames']
        for frame in expected['frames']:
            frame['name'] = poses.get(frame['name'], frame['name'])
        for key in ('start_pose', 'end_pose'):
            expected[key] = poses.get(expected[key], expected[key])
        if kind == 'runtime':
            expected['playback_speed'] = .855 if name.startswith('t제자리좌회전') else .95
            expected['completion']['position_tolerance_deg'] = 5.
        assert after[name] == expected, name


def test_aliases_resolve_and_removed_transitions_cannot_be_requested():
    before = yaml.safe_load((UPDATE / 'before' / ALIAS_PATH).read_text())['motion_aliases']
    after = yaml.safe_load((SNAPSHOT / ALIAS_PATH).read_text())['motion_aliases']
    assert after == {alias: MANIFEST['changed_aliases'].get(alias, name)
                     for alias, name in before.items() if alias not in MANIFEST['removed_aliases']}
    motions = catalog(SNAPSHOT / 'artifacts/robot_motions_runtime.json')
    assert set(after.values()) <= motions.keys()
    assert set(MANIFEST['removed_aliases']).isdisjoint(Bridge.TRANSITION_MOTION_IDS)
    for pose in MANIFEST['pose_renames'].values():
        frames = [f for m in motions.values() for f in m['frames'] if f['name'] == pose]
        assert frames and all(f['angles'] == frames[0]['angles'] for f in frames)


@pytest.mark.parametrize('camera', [0, 90])
def test_shared_support_matches_body_and_torque(camera):
    motions = catalog(ROOT / 'artifacts/robot_motions_runtime.json')
    left = motions[f'tt미세왼옆{camera}도']['frames'][-1]
    name = 't미세오옆꽃게(0도)-1회' if camera == 0 else 't미세오옆꽃게90도(1회)'
    right = motions[name]['frames'][-1]
    assert left['torques'] == right['torques']
    assert {k: v for k, v in left['angles'].items() if k != '0'} == {
        k: v for k, v in right['angles'].items() if k != '0'}
    assert abs(left['angles']['0'] - right['angles']['0']) < .11


def test_observation_times_match_runtime_data():
    motions = catalog(ROOT / 'artifacts/robot_motions_runtime.json')
    aliases = yaml.safe_load((ROOT / ALIAS_PATH).read_text())['motion_aliases']
    estimates = {**PICKUP_CRAB_DURATIONS,
                 **{f'ball_align_turn_left_{n}': duration_sec(f'ball_align_turn_left_{n}') for n in range(1, 7)}}
    for alias, seconds in estimates.items():
        m = motions[aliases[alias]]
        cycle_ms = max(f['start_ms'] + f['time_ms'] for f in m['frames'])
        assert seconds == pytest.approx(cycle_ms / 1000 * m['repeat_count'] / m['playback_speed'])


@pytest.mark.parametrize('side,opposite', [('left', 'right'), ('right', 'left')])
@pytest.mark.parametrize('previous_count', ['', '_2'])
@pytest.mark.parametrize('next_count', ['', '_2'])
def test_pickup_direction_switch_waits_half_second_without_pose_transition(clock, side, opposite, previous_count, next_count):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(navigation_message(action='PICKUP_NOW'))
    bridge.pickup_initial_align_waiting = False
    bridge.pickup_fine_align_waiting = True
    bridge.active_motion_id = bridge.FINE_ALIGN_MARKER
    bridge.last_physical_motion_id = f'pickup_crab_{side}_0{previous_count}'
    command = fine_alignment_message(f'BALL_PICKUP_CRAB_{opposite.upper()}')
    payload = json.loads(command.data)
    payload['source_command'] = {'offset_x_px': 250 if next_count else 100}
    command.data = json.dumps(payload)
    bridge.navigation_command_callback(command)
    assert not decoded_messages(bridge.executor_request_publisher)
    assert bridge.transition_motion_id is None
    started = clock[0]
    clock[0] = started + .499
    bridge._check_atomic_dwell()
    assert not decoded_messages(bridge.executor_request_publisher)
    clock[0] = started + .5
    bridge._check_atomic_dwell()
    requests = decoded_messages(bridge.executor_request_publisher)
    assert requests[-1]['motion_id'] == f'pickup_crab_{opposite}_0{next_count}'
    assert bridge.transition_dwell_until is None
    assert bridge.pending_pickup_crab_motion_id is None
    bridge.executor_status_callback(executor_status(**requests[-1], status='SUCCEEDED'))
    assert bridge.last_physical_motion_id == f'pickup_crab_{opposite}_0{next_count}'


@pytest.mark.parametrize('side,opposite', [('left', 'right'), ('right', 'left')])
def test_goal_direction_switch_keeps_only_existing_observation_pause(clock, side, opposite):
    bridge = start_case(f'goal_camera_90_crab_{side}', f'GOAL_CAMERA90_CRAB_{opposite.upper()}')
    assert bridge._transition_prepare_motion(f'goal_camera_90_crab_{opposite}') is None
    assert bridge.transition_dwell_until is None
    clock[0] += .5
    bridge._check_atomic_dwell()
    requests = decoded_messages(bridge.executor_request_publisher)
    assert [r['motion_id'] for r in requests] == [f'goal_camera_90_crab_{opposite}']
    bridge.executor_status_callback(executor_status(**requests[-1], status='SUCCEEDED'))
    assert not bridge.motion_in_progress


ENTRIES = [
    ('pickup_fine_forward_0', 'BALL_PICKUP_CRAB_LEFT', 'pickup_fine_to_crab_left_0', 'pickup_crab_left_0'),
    ('line_forward_4', 'BALL_PICKUP_CRAB_LEFT', 'pickup_forward_to_crab_left_0', 'pickup_crab_left_0'),
    ('goal_camera_90_fine_forward_1', 'GOAL_CAMERA90_CRAB_LEFT', 'goal_fine_to_crab_left_90', 'goal_camera_90_crab_left'),
    ('goal_camera_90_forward_4', 'GOAL_CAMERA90_CRAB_LEFT', 'goal_forward_to_crab_left_90', 'goal_camera_90_crab_left'),
]


@pytest.mark.parametrize('previous,action,prepare,target', ENTRIES)
def test_shared_entry_still_completes_preparation_before_crab(clock, previous, action, prepare, target):
    bridge = start_case(previous, action)
    check_both_pauses(bridge, clock, prepare, target)


@pytest.mark.parametrize('previous,action,prepare,target', ENTRIES)
@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED'])
def test_failed_entry_never_starts_crab(clock, previous, action, prepare, target, status):
    bridge = start_case(previous, action)
    clock[0] += .5
    bridge._check_atomic_dwell()
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request['motion_id'] == prepare
    bridge.executor_status_callback(executor_status(**request, status=status, error_code='SDK_COMMUNICATION_ERROR'))
    clock[0] += 10
    bridge._check_atomic_dwell()
    assert [r['motion_id'] for r in decoded_messages(bridge.executor_request_publisher)] == [prepare]
    assert not bridge.motion_in_progress
