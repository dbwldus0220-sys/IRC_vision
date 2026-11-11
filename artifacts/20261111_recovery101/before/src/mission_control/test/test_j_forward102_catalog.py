"""Verify source102 body data, preserved camera/count semantics, and embedded gaits."""

import hashlib
import json
from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261111_j_forward102'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
sys.path.insert(0, str(ROOT / 'tools'))
from build_post_shot_forward_exits import build_exit, GOAL_EXIT_RECOVERIES, FORWARD_EXIT_NAMES
from upsert_motion_catalog import runtime_motion_from_source


def catalog(path):
    data = json.loads(path.read_text())['motions']
    result = {m['name']: m for m in data}
    assert len(result) == len(data)
    return result


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_all_forward_variants_use_source_body_and_preserve_command_semantics(kind):
    source_path = UPDATE / 'source_robot_motions.json'
    assert hashlib.sha256(source_path.read_bytes()).hexdigest() == MANIFEST['source_sha256']
    source = catalog(source_path)['j전진45(6회)']
    relative = f'artifacts/robot_motions_{kind}.json'
    before, after = catalog(UPDATE / 'before' / relative), catalog(ROOT / relative)
    renames = MANIFEST['renames']
    assert len(renames) == 12
    assert after.keys() == {renames.get(name, name) for name in before}
    for old_name, new_name in renames.items():
        old, new = before[old_name], after[new_name]
        assert new['repeat_count'] == old['repeat_count']
        assert new['playback_speed'] == source['playback_speed'] == 1.0
        assert new['repeatable'] == source['repeatable']
        assert len(new['frames']) == len(source['frames']) == 4
        for actual, original, previous in zip(new['frames'], source['frames'], old['frames']):
            assert actual['angles']['0'] == previous['angles']['0']
            assert {k: v for k, v in actual['angles'].items() if k != '0'} == {
                k: v for k, v in original['angles'].items() if k != '0'}
            assert {k: v for k, v in actual.items() if k not in ('angles', 'name')} == {
                k: v for k, v in original.items() if k not in ('angles', 'name')}
        assert new['frames'][-1]['angles'] == old['frames'][-1]['angles']
        assert new['completion'] == (runtime_motion_from_source(source) if kind == 'runtime' else source)['completion']
    for name in before.keys() - renames.keys() - set(MANIFEST['replaced']):
        assert after[name] == before[name], name


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_embedded_forward_and_remaining_preparation_frames(kind):
    relative = f'artifacts/robot_motions_{kind}.json'
    before, after = catalog(UPDATE / 'before' / relative), catalog(ROOT / relative)
    name = 'n공에서 전진하고 후진준비자세'
    assert after[name]['frames'][:4] == after['j전진0도(2회)']['frames']
    assert after[name]['frames'][4:] == before[name]['frames'][4:]
    assert after[name]['repeat_count'] == before[name]['repeat_count'] == 1
    assert after[name]['end_pose'] == before[name]['end_pose']
    pc = catalog(ROOT / 'artifacts/robot_motions_pc.json')
    for turn, recovery, blocks in GOAL_EXIT_RECOVERIES:
        expected = build_exit(pc[turn], pc[recovery], blocks,
                              forward=pc['j전진45(2회)'] if turn in FORWARD_EXIT_NAMES else None)
        if kind == 'runtime':
            expected = runtime_motion_from_source(expected)
        assert after[expected['name']] == expected


def test_aliases_change_only_forward_targets_and_all_resolve():
    relative = 'src/irc_step_motion_executor/config/motion_aliases.yaml'
    before = yaml.safe_load((UPDATE / 'before' / relative).read_text())['motion_aliases']
    after = yaml.safe_load((ROOT / relative).read_text())['motion_aliases']
    assert after == {key: MANIFEST['renames'].get(name, name) for key, name in before.items()}
    assert set(after.values()) <= catalog(ROOT / 'artifacts/robot_motions_runtime.json').keys()
