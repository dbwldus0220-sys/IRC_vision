"""Verify source101 forward/recovery data, count corrections, and unchanged motions."""

import copy
import hashlib
import json
from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261112_pt_forward_recovery101'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source
from build_post_shot_forward_exits import build_exit, GOAL_EXIT_RECOVERIES, FORWARD_EXIT_NAMES


def catalog(path):
    data = json.loads(path.read_text())['motions']
    result = {m['name']: m for m in data}
    assert len(result) == len(data)
    return result


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_source_fidelity_counts_cameras_and_unchanged_motions(kind):
    source_path = UPDATE / 'source_robot_motions.json'
    assert hashlib.sha256(source_path.read_bytes()).hexdigest() == MANIFEST['source_sha256']
    source = catalog(source_path)
    relative = f'artifacts/robot_motions_{kind}.json'
    before, after = catalog(UPDATE / 'before' / relative), catalog(ROOT / relative)
    renames = MANIFEST['renames']
    assert len(renames) == 18
    assert after.keys() == {renames.get(name, name) for name in before}
    reverse = {new: old for old, new in renames.items()}
    for name, source_name in MANIFEST['source_mapping'].items():
        old = before[reverse.get(name, name)]
        expected = copy.deepcopy(source[source_name])
        expected.update(name=name, repeat_count=old['repeat_count'])
        for frame in expected['frames']:
            if frame['name'] == '오뒤무게중심앞':
                frame['name'] += '(catalog80)'
        for key in ('start_pose', 'end_pose'):
            if expected[key] == '오뒤무게중심앞':
                expected[key] += '(catalog80)'
        if kind == 'runtime':
            expected = runtime_motion_from_source(expected)
        assert after[name] == expected
        assert after[name]['playback_speed'] == 1.0
        assert [f['angles']['0'] for f in after[name]['frames']] == [f['angles']['0'] for f in old['frames']]
    assert after['p전진0도(2회)']['repeat_count'] == 2
    assert after['p전진45(8회)']['repeat_count'] == 8
    assert after['p전진45(8회)']['frames'] == after['p전진45(6회)']['frames']
    for name in before.keys() - renames.keys() - set(MANIFEST['replaced']):
        assert after[name] == before[name], name


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_embedded_forward_and_full_catalog_durations(kind):
    pc = catalog(ROOT / 'artifacts/robot_motions_pc.json')
    after = catalog(ROOT / f'artifacts/robot_motions_{kind}.json')
    assert after['n공에서 전진하고 후진준비자세']['frames'][:4] == after['p전진0도(2회)']['frames']
    for turn, recovery, blocks in GOAL_EXIT_RECOVERIES:
        expected = build_exit(pc[turn], pc[recovery], blocks,
                              forward=pc['p전진45도(2회)'] if turn in FORWARD_EXIT_NAMES else None)
        if kind == 'runtime':
            expected = runtime_motion_from_source(expected)
        assert after[expected['name']] == expected
    for name, seconds in MANIFEST['duration_seconds'].items():
        motion = after[name]
        assert max(f['start_ms'] + f['time_ms'] for f in motion['frames']) * motion['repeat_count'] / motion['playback_speed'] / 1000 == pytest.approx(seconds)


def test_only_requested_alias_targets_change():
    relative = 'src/irc_step_motion_executor/config/motion_aliases.yaml'
    before = yaml.safe_load((UPDATE / 'before' / relative).read_text())['motion_aliases']
    after = yaml.safe_load((ROOT / relative).read_text())['motion_aliases']
    assert after == {key: MANIFEST['renames'].get(name, name) for key, name in before.items()}
    assert set(after.values()) <= catalog(ROOT / 'artifacts/robot_motions_runtime.json').keys()
