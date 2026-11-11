"""Check source95 fidelity and the shortened turn/forward playback boundary."""

import copy
import hashlib
import json
from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261111_catalog95_requested_motions'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source


def catalog(path):
    values = json.loads(path.read_text())['motions']
    result = {m['name']: m for m in values}
    assert len(result) == len(values)
    return result


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_requested_imports_preserve_source_and_unrelated_motions(kind):
    relative = Path(f'artifacts/robot_motions_{kind}.json')
    before = catalog(UPDATE / 'before' / relative)
    actual = catalog(ROOT / relative)
    source = catalog(UPDATE / 'source_robot_motions.json')
    assert actual.keys() == before.keys()
    assert {n for n in actual if actual[n] != before[n]} == {
        't공후 제자리우회전', 't골대후제자리우회전',
        't골대후제자리우회전(5회)', 't골대후제자리우회전(5회)+전진4회',
        'p골대후제자리좌회전(3회)+전진4회',
    }
    for name in MANIFEST['imports']:
        expected = copy.deepcopy(source[name])
        renames = MANIFEST['pose_renames'][name]
        for frame in expected['frames']:
            frame['name'] = renames.get(frame['name'], frame['name'])
        for field in ('start_pose', 'end_pose'):
            expected[field] = renames.get(expected[field], expected[field])
        if kind == 'runtime':
            expected = runtime_motion_from_source(expected)
        assert actual[name] == expected
    assert hashlib.sha256((UPDATE / 'source_robot_motions.json').read_bytes()).hexdigest() == MANIFEST['source_sha256']


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_shortened_turn_preserves_source_with_half_second_hold_before_forward(kind):
    relative = Path(f'artifacts/robot_motions_{kind}.json')
    actual = catalog(ROOT / relative)
    before = catalog(UPDATE / 'before' / relative)
    base = actual['t골대후제자리우회전']
    short = actual['t골대후제자리우회전(5회)']
    combined = actual['t골대후제자리우회전(5회)+전진4회']
    assert short['frames'] == base['frames'][:12]
    assert sum(f['name'] == '제우오들25' for f in short['frames']) == 5
    assert combined['frames'][:12] == short['frames']
    assert short['max_seq_ms'] == 2287
    assert combined['frames'][12]['start_ms'] == short['max_seq_ms'] + 500
    for new, old in zip(combined['frames'][12:], before[combined['name']]['frames'][12:]):
        assert new == {**old, 'start_ms': old['start_ms'] - 15 + 500}
    for field in ('angles', 'torques'):
        assert short['frames'][-1][field] == base['frames'][-1][field]
        assert combined['frames'][-1][field] == actual['p전진45(4회)']['frames'][-1][field]
    for name, milliseconds in [('t공후 제자리우회전', 2953), (base['name'], 2953),
                               (short['name'], 2287), (combined['name'], 4763)]:
        motion = actual[name]
        assert motion['playback_speed'] == 1.0 and motion['repeat_count'] == 1
        assert max(f['start_ms'] + f['time_ms'] for f in motion['frames']) == milliseconds
        assert milliseconds + motion['completion']['settle_timeout_ms'] < 12000
        assert all(a['start_ms'] + a['time_ms'] <= b['start_ms']
                   for a, b in zip(motion['frames'], motion['frames'][1:]))


def test_deployed_catalog_and_aliases_resolve_all_six_requests():
    relative = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
    assert (ROOT / relative).read_bytes() == (UPDATE / 'before' / relative).read_bytes()
    aliases = yaml.safe_load((ROOT / relative).read_text())['motion_aliases']
    runtime = catalog(ROOT / 'artifacts/robot_motions_runtime.json')
    assert set(aliases.values()) <= runtime.keys()
    assert [aliases[key] for key in (
        'pickup_grasp_check_pose', 'pickup_first_backward_turn_right',
        'post_shot_default_turn_right', 'goal_shot_region_5',
        'goal_shot_region_6', 'goal_shot_region_7',
    )] == MANIFEST['imports']
    for name in MANIFEST['imports'] + MANIFEST['derived']:
        assert runtime[name] == runtime_motion_from_source(runtime[name])
