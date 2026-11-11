"""Verify recovery source fidelity, stable public IDs, and composite gait timing."""

import copy
import hashlib
import json
from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261111_recovery101'
DEPLOYED = ROOT / 'artifacts/20261111_forward101/before'
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
def test_six_recoveries_match_source_and_unrelated_motions_are_unchanged(kind):
    source_path = UPDATE / 'source_robot_motions.json'
    assert hashlib.sha256(source_path.read_bytes()).hexdigest() == MANIFEST['source_sha256']
    source = catalog(source_path)
    relative = f'artifacts/robot_motions_{kind}.json'
    before, after = catalog(UPDATE / 'before' / relative), catalog(DEPLOYED / relative)
    renames = MANIFEST['renames']
    assert len(renames) == 6
    assert after.keys() == {renames.get(name, name) for name in before}
    for old, name in renames.items():
        expected = copy.deepcopy(source[name])
        for frame in expected['frames']:
            if frame['name'] == '오뒤무게중심앞':
                frame['name'] += '(catalog80)'
        for key in ('start_pose', 'end_pose'):
            if expected[key] == '오뒤무게중심앞':
                expected[key] += '(catalog80)'
        if kind == 'runtime':
            expected = runtime_motion_from_source(expected)
        assert after[name] == expected
        assert after[name]['repeat_count'] == before[old]['repeat_count'] == 4
        assert after[name]['playback_speed'] == 1.0
        for key in ('angles', 'torques'):
            assert after[name]['frames'][-1][key] == before[old]['frames'][-1][key]
    for name in before.keys() - renames.keys() - set(MANIFEST['replaced']):
        assert after[name] == before[name], name


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_all_four_composites_use_current_forward_and_recovery(kind):
    pc = catalog(ROOT / 'artifacts/robot_motions_pc.json')
    after = catalog(ROOT / f'artifacts/robot_motions_{kind}.json')
    for turn, recovery, blocks in GOAL_EXIT_RECOVERIES:
        expected = build_exit(pc[turn], pc[recovery], blocks,
                              forward=pc['p전진45도(2회)'] if turn in FORWARD_EXIT_NAMES else None)
        if kind == 'runtime':
            expected = runtime_motion_from_source(expected)
        actual = after[expected['name']]
        assert actual == expected
        assert actual['repeat_count'] == 1
        assert actual['frames'][:len(pc[turn]['frames'])] == pc[turn]['frames']
        duration = max(f['start_ms'] + f['time_ms'] for f in actual['frames']) / actual['playback_speed'] / 1000
        assert duration == pytest.approx(MANIFEST['duration_seconds'][actual['name']])


def test_only_recovery_aliases_change_and_all_targets_exist():
    relative = 'src/irc_step_motion_executor/config/motion_aliases.yaml'
    before = yaml.safe_load((UPDATE / 'before' / relative).read_text())['motion_aliases']
    after = yaml.safe_load((DEPLOYED / relative).read_text())['motion_aliases']
    assert after == {key: MANIFEST['renames'].get(name, name) for key, name in before.items()}
    assert set(after.values()) <= catalog(DEPLOYED / 'artifacts/robot_motions_runtime.json').keys()
