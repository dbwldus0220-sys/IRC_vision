"""Source92 fidelity, unchanged routing and safe derived goal-exit boundaries."""

import copy
import hashlib
import json
from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261110_catalog92_requested_motions'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
SNAPSHOT = ROOT / 'artifacts/20261110_catalog93_goal_shots/before'
ALIASES = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source


def motions(path):
    values = json.loads(path.read_text())['motions']
    result = {m['name']: m for m in values}
    assert len(result) == len(values)
    return result


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_exact_source_import_and_untouched_unrelated_motions(kind):
    relative = Path(f'artifacts/robot_motions_{kind}.json')
    before = motions(UPDATE / 'before' / relative)
    actual = motions(SNAPSHOT / relative)
    source = motions(UPDATE / 'source_robot_motions.json')
    assert len(actual) == len(before) == 160
    changed = set(MANIFEST['imports'] + MANIFEST['derived'])
    assert {n for n in before if before[n] != actual[n]} == changed
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
def test_turn_variant_has_five_complete_cycles_and_composite_keeps_forward_tail(kind):
    relative = Path(f'artifacts/robot_motions_{kind}.json')
    actual = motions(SNAPSHOT / relative)
    before = motions(UPDATE / 'before' / relative)
    base = actual['t골대후제자리우회전']
    short = actual['t골대후제자리우회전(5회)']
    combined = actual['t골대후제자리우회전(5회)+전진4회']
    assert short['frames'] == base['frames'][:12]
    assert sum(f['name'] == '제우오들25' for f in short['frames']) == 5
    assert combined['frames'][:12] == short['frames']
    assert combined['frames'][12:] == before[combined['name']]['frames'][12:]
    assert combined['frames'][12]['start_ms'] == short['max_seq_ms'] == 2302
    for field in ('angles', 'torques'):
        assert short['frames'][-1][field] == base['frames'][-1][field]
        assert combined['frames'][-1][field] == actual['p전진45(4회)']['frames'][-1][field]


def test_check_pose_changed_only_requested_source_joint_values_and_routes_stay_valid():
    runtime = motions(SNAPSHOT / 'artifacts/robot_motions_runtime.json')
    before = motions(UPDATE / 'before/artifacts/robot_motions_runtime.json')
    new = runtime['n공확인자세']['frames'][0]
    old = before['n공확인자세']['frames'][0]
    assert {key for key in old['angles'] if old['angles'][key] != new['angles'][key]} == {'19', '20'}
    assert new['angles']['19'] == 70.400390625
    assert new['angles']['20'] == -64.599609375
    assert new['torques'] == old['torques']
    assert (ROOT / ALIASES).read_bytes() == (UPDATE / 'before' / ALIASES).read_bytes()
    aliases = yaml.safe_load((ROOT / ALIASES).read_text())['motion_aliases']
    assert set(aliases.values()) <= runtime.keys()
    assert aliases['pickup_grasp_check_pose'] == 'n공확인자세'
    assert aliases['pickup_first_backward_turn_right'] == 't공후 제자리우회전'
    assert aliases['post_shot_default_turn_right'] == 't골대후제자리우회전'
    for name in MANIFEST['imports'] + MANIFEST['derived']:
        motion = runtime[name]
        duration = max(f['start_ms'] + f['time_ms'] for f in motion['frames'])
        duration *= motion['repeat_count'] / motion['playback_speed']
        assert duration == MANIFEST['catalogs']['runtime']['durations'][name]['before_ms']
        assert motion['playback_speed'] == 1.0
        assert motion == runtime_motion_from_source(motion)
