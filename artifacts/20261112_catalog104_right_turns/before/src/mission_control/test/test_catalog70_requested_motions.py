"""Check source70 data, camera routing, repeat counts and capture timing offline."""

import copy
import hashlib
import json
from pathlib import Path

import pytest
import yaml

from mission_control.pickup_walk_reservation import PICKUP_WALK_DURATIONS


ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261105_catalog70_requested'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
SOURCE = {m['name']: m for m in json.loads(
    (UPDATE / 'source_requested.json').read_text())['motions']}
# Catalog71/72 supersede only the right-crab family; retain other source70 checks.
LATEST_CRAB = {m['name']: m for m in json.loads((ROOT /
    'artifacts/20261105_catalog71_crab/source_requested.json').read_text())['motions']}
LATEST_CRAB.update({m['name']: m for m in json.loads((ROOT /
    'artifacts/20261105_catalog72_crab90/source_requested.json').read_text())['motions']})
ALIASES = yaml.safe_load((ROOT /
    'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']


def catalog(kind):
    motions = json.loads((ROOT / f'artifacts/robot_motions_{kind}.json').read_text())['motions']
    result = {m['name']: m for m in motions}
    assert len(result) == len(motions)
    return result


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
@pytest.mark.parametrize('name', list(SOURCE) + list(MANIFEST['derived']))
def test_source_frames_timing_torque_and_speed_are_preserved(kind, name):
    if name in SOURCE:
        expected = copy.deepcopy(LATEST_CRAB.get(name, SOURCE[name]))
    else:
        spec = MANIFEST['derived'][name]
        expected = copy.deepcopy(SOURCE[spec['source']])
        expected.update(name=name, repeat_count=spec['repeat_count'])
    # Source70 reuses the startup pose name for a different set of joint targets.
    for old, new in MANIFEST.get('pose_renames', {}).items():
        for frame in expected['frames']:
            if frame['name'] == old:
                frame['name'] = new
        for key in ('start_pose', 'end_pose'):
            if expected[key] == old:
                expected[key] = new
    if kind == 'runtime':
        expected['completion']['position_tolerance_deg'] = 5.0
    assert catalog(kind)[name] == expected


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_unrelated_data_unchanged_and_every_alias_resolves(kind):
    actual = catalog(kind)
    record = MANIFEST['catalogs'][f'robot_motions_{kind}.json']
    assert len(actual) == record['after_count']
    for name, digest in record['unchanged_sha256'].items():
        encoded = json.dumps(actual[name], ensure_ascii=False, sort_keys=True).encode()
        assert hashlib.sha256(encoded).hexdigest() == digest, name
    assert set(ALIASES.values()) <= actual.keys()
    assert not set(MANIFEST['renames']) & actual.keys()
    assert ALIASES == MANIFEST['aliases_after']


@pytest.mark.parametrize('camera,prefix', [
    (0, 'pickup_camera_down'), (45, 'line'),
    (45, 'post_ball_line'), (90, 'goal_camera_90'),
])
@pytest.mark.parametrize('count', [1, 2, 3, 5, 6, 7, 9])
def test_right_turn_aliases_keep_camera_and_count(camera, prefix, count):
    name = ALIASES[f'{prefix}_turn_right_{count}']
    assert name == f't제자리우회전{camera}도({count}회)'
    assert catalog('runtime')[name]['repeat_count'] == count


@pytest.mark.parametrize('alias,name', [
    ('pickup_first_backward_turn_right', 't공후 제자리우회전'),
    ('pickup_second_backward_turn_left', 't공후 제자리좌회전'),
    ('post_shot_default_turn_right', 't골대후제자리우회전'),
    ('post_shot_default_turn_left', 't골대후 제자리좌회전'),
    ('pickup_crab_right_0', 't미세오옆꽃게(0도)-1회'),
    ('pickup_crab_right_0_2', 't미세오옆꽃게(0도)-2회'),
    ('goal_camera_90_crab_right', 't미세오옆꽃게90도(1회)'),
    ('goal_camera_90_crab_right_2', 't미세오옆꽃게90도(2회)'),
    ('pickup_forward_to_crab_0', 'r오뒤에서 오옆꽃게0도'),
    ('pickup_fine_to_crab_right_0', 'r미세오뒤에서 오옆0도'),
    ('goal_forward_to_crab_right_90', 'r오뒤에서 오옆꽃게90도'),
    ('goal_fine_to_crab_right_90', 'r미세오뒤에서 오옆 90도'),
])
def test_exit_and_crab_routes_use_requested_source(alias, name):
    assert ALIASES[alias] == name
    assert catalog('runtime')[name]['repeat_count'] == SOURCE[name]['repeat_count']


def test_pickup_capture_estimate_matches_new_crab_duration():
    motion = SOURCE['t미세오옆꽃게(0도)-1회']
    duration = max(f['start_ms'] + f['time_ms'] for f in motion['frames'])
    duration *= motion['repeat_count'] / motion['playback_speed'] / 1000
    assert PICKUP_WALK_DURATIONS['pickup_crab_right_0'] == pytest.approx(duration)


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_startup_pose_is_unambiguous_and_preserves_existing_targets(kind):
    motions = catalog(kind)
    poses = [frame['angles'] for motion in motions.values() for frame in motion['frames']
             if frame['name'] == '오뒤무게중심앞']
    assert poses and all(pose == poses[0] for pose in poses)
    assert poses[0] == motions['R오뒤45도']['frames'][0]['angles']
    updated = [frame['angles'] for motion in motions.values() for frame in motion['frames']
               if frame['name'] == '오뒤무게중심앞(catalog70)']
    assert updated and all(pose == updated[0] for pose in updated)
    assert updated[0] != poses[0]
