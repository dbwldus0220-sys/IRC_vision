"""Check the seven source82 replacements and their motion handoffs offline."""

import copy
import json
from pathlib import Path
import sys

import pytest
import yaml

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from test_continuous_approach import clock
from test_motion_command_bridge import (
    FakeBridge, complete_active_motion, decoded_messages, navigation_message,
)
from test_shot_start_posture import finish_sequence

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261110_catalog82_pickup_shot'
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source

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
def test_only_seven_requested_motions_changed_with_original_speeds(kind):
    relative = Path(f'artifacts/robot_motions_{kind}.json')
    before = catalog(UPDATE / 'before' / relative)
    # Source83 later replaces the left-turn families; inspect this import's result.
    after = catalog(ROOT / 'artifacts/20261110_catalog83_left_turns/before' / relative)
    assert after.keys() == before.keys()
    assert {n for n in after if after[n] != before[n]} == set(MANIFEST['imports'])
    for name in after:
        assert after[name]['playback_speed'] == before[name]['playback_speed']
        if name not in MANIFEST['imports']:
            continue
        expected = copy.deepcopy(SOURCE[name])
        for frame in expected['frames']:
            frame['name'] = MANIFEST['pose_renames'].get(frame['name'], frame['name'])
        if kind == 'runtime':
            expected = runtime_motion_from_source(expected)
        assert after[name] == expected
        assert after[name]['playback_speed'] == 1.0


def test_dispatch_and_pose_lookup_remain_unambiguous():
    snapshot = ROOT / 'artifacts/20261110_catalog83_left_turns/before'
    assert (snapshot / ALIAS_PATH).read_bytes() == (UPDATE / 'before' / ALIAS_PATH).read_bytes()
    assert set(ALIASES.values()) <= RUNTIME.keys()
    assert set(Bridge.ACTION_TO_MOTION_ID.values()) <= ALIASES.keys()
    poses = {f['name'] for n in MANIFEST['imports']
             for f in RUNTIME[ALIASES['pickup_grasp_check_pose'] if n == '찐공확인자세' else n]['frames']}
    for pose in poses:
        frames = [f for m in RUNTIME.values() for f in m['frames'] if f['name'] == pose]
        assert all((f['angles'], f['torques']) ==
                   (frames[0]['angles'], frames[0]['torques']) for f in frames), pose


@pytest.mark.parametrize('name', MANIFEST['imports'])
def test_actual_motion_duration_fits_executor_timeout(name):
    motion = RUNTIME[ALIASES['pickup_grasp_check_pose'] if name == '찐공확인자세' else name]
    duration = max(f['start_ms'] + f['time_ms'] for f in motion['frames'])
    duration *= motion['repeat_count'] / motion['playback_speed']
    assert duration == MANIFEST['durations_ms'][name]['after']
    assert duration + motion['completion']['settle_timeout_ms'] < Bridge.DEFAULT_TIMEOUT_MS
    assert motion['completion']['position_tolerance_deg'] == 5.0


@pytest.mark.parametrize('pickup', ['pickup_400', 'pickup_500'])
@pytest.mark.parametrize('completed,exit_motion', [
    (0, 'pickup_first_backward_turn_right'),
    (1, 'pickup_second_backward_turn_left'),
])
def test_new_pickup_end_pose_releases_check_and_exit_after_existing_dwells(clock, pickup, completed, exit_motion):
    bridge = FakeBridge(advance_transition_dwells=False)
    sequence = Bridge._with_pickup_motion_dwells(Bridge._pickup_motion_sequence({
        'source_command': {}, 'mission_progress': {'pickups_completed': completed},
    }))
    sequence = tuple(pickup if stage == 'pickup' else stage for stage in sequence)
    bridge.motion_in_progress = True
    bridge.active_action = 'PICKUP_NOW'
    bridge.active_command_id = bridge.active_request_id = 10
    bridge.active_motion_id = pickup
    bridge.active_pickup_sequence = sequence
    bridge.active_sequence_index = sequence.index(pickup)
    complete_active_motion(bridge)
    assert bridge.active_dwell_until == clock[0] + .5
    clock[0] = bridge.active_dwell_until
    bridge._check_atomic_dwell()
    assert bridge.active_motion_id == 'pickup_grasp_check_pose'
    complete_active_motion(bridge)
    assert bridge.active_dwell_until == clock[0] + 3.
    deadline = bridge.active_dwell_until
    clock[0] = deadline - .001
    bridge._check_atomic_dwell()
    assert bridge.active_motion_id != exit_motion
    clock[0] = deadline
    bridge._check_atomic_dwell()
    assert bridge.active_motion_id == exit_motion
    finish_sequence(bridge)
    assert not bridge.motion_in_progress
    assert decoded_messages(bridge.motion_status_publisher)[-1]['status'] == 'SUCCEEDED'


@pytest.mark.parametrize('region', [1, 2, 3, 4])
def test_each_changed_shot_finishes_and_releases_the_motion_lock(region):
    bridge = FakeBridge()
    bridge.last_physical_motion_id = 'goal_camera_90_fine_forward_3'
    bridge.navigation_command_callback(navigation_message(
        action='SHOT', source='goal', source_command={'shot_region': region}))
    assert bridge.active_motion_id == f'goal_shot_region_{region}'
    assert ALIASES[bridge.active_motion_id] == f'R골넣기{region}번구역'
    finish_sequence(bridge)
    assert not bridge.motion_in_progress
    assert decoded_messages(bridge.motion_status_publisher)[-1]['status'] == 'SUCCEEDED'


@pytest.mark.parametrize('region,depth,offset', [
    (1, .48, 40), (2, .55, 40), (3, .55, -40), (4, .48, -40),
])
@pytest.mark.parametrize('shot_index', [1, 2])
def test_committed_walking_reservation_reaches_each_changed_shot(clock, monkeypatch, region, depth, offset, shot_index):
    import test_goal_region_queue_audit as audit
    from test_goal_extended_regions import sample

    # Reuse the complete reservation/prepare/observe/shot/exit replay for regions 1..4.
    monkeypatch.setattr(audit, 'sample', lambda distance, **kw: sample(distance, offset, **kw))
    audit.test_committed_queue_finishes_once_then_reaches_correct_shot(
        clock, region, depth, shot_index)
