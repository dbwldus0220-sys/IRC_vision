"""Source102 m import, camera correction, mixed-speed timing, and stable routes."""

import copy
import hashlib
import json
from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261111_m_forward_recovery102'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source
from build_post_shot_forward_exits import build_exit, GOAL_EXIT_RECOVERIES, FORWARD_EXIT_NAMES


def catalog(path):
    motions = json.loads(path.read_text())['motions']
    result = {m['name']: m for m in motions}
    assert len(result) == len(motions)
    return result


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_source_data_counts_camera_and_unchanged_motions(kind):
    source_path = UPDATE / 'source_robot_motions.json'
    assert hashlib.sha256(source_path.read_bytes()).hexdigest() == MANIFEST['source_sha256']
    source = catalog(source_path)
    relative = f'artifacts/robot_motions_{kind}.json'
    before, after = catalog(UPDATE / 'before' / relative), catalog(ROOT / relative)
    renames = MANIFEST['renames']
    assert len(renames) == 18
    assert len(MANIFEST['replaced']) == 23
    assert after.keys() == {renames.get(name, name) for name in before}
    for old_name, name in renames.items():
        old = before[old_name]
        expected = copy.deepcopy(source[MANIFEST['source_mapping'][name]])
        expected.update(name=name, repeat_count=old['repeat_count'])
        for frame in expected['frames']:
            if frame['name'] == '오뒤무게중심앞':
                frame['name'] += '(catalog80)'
            if name == 'm라인복귀우회전90':
                frame['angles']['0'] = 1.0
        for key in ('start_pose', 'end_pose'):
            if expected[key] == '오뒤무게중심앞':
                expected[key] += '(catalog80)'
        if kind == 'runtime':
            expected = runtime_motion_from_source(expected)
        assert after[name] == expected
        assert after[name]['playback_speed'] == 1.05
        assert [f['angles']['0'] for f in after[name]['frames']] == [f['angles']['0'] for f in old['frames']]
        for field in ('angles', 'torques'):
            assert after[name]['frames'][-1][field] == old['frames'][-1][field]
    assert after['m전진45(8회)']['repeat_count'] == 8
    assert after['m전진45(8회)']['frames'] == after['m전진45(6회)']['frames']
    for name in before.keys() - renames.keys() - set(MANIFEST['replaced']):
        assert after[name] == before[name], name


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_embedded_gaits_preserve_turns_preparation_and_effective_speed(kind):
    pc = catalog(ROOT / 'artifacts/robot_motions_pc.json')
    after = catalog(ROOT / f'artifacts/robot_motions_{kind}.json')
    before = catalog(UPDATE / 'before' / f'artifacts/robot_motions_{kind}.json')
    for turn, recovery, blocks in GOAL_EXIT_RECOVERIES:
        expected = build_exit(pc[turn], pc[recovery], blocks,
                              forward=pc['m전진45(2회)'] if turn in FORWARD_EXIT_NAMES else None)
        if kind == 'runtime':
            expected = runtime_motion_from_source(expected)
        assert after[expected['name']] == expected
        assert expected['frames'][:len(pc[turn]['frames'])] == pc[turn]['frames']

    name = 'n공에서 전진하고 후진준비자세'
    prepare, old, gait = after[name], before[name], after['m전진0도(2회)']
    assert prepare['repeat_count'] == old['repeat_count'] == 1
    assert prepare['playback_speed'] == old['playback_speed'] == 1.0
    assert prepare['start_pose'] == gait['start_pose'] == prepare['frames'][0]['name']
    assert prepare['end_pose'] == old['end_pose'] == prepare['frames'][-1]['name']
    for actual, source in zip(prepare['frames'][:4], gait['frames'], strict=True):
        for field in ('angles', 'torques', 'name', 'stop_at_end'):
            assert actual.get(field) == source.get(field)
        for time in ('start_ms', 'time_ms'):
            assert abs(actual[time] - source[time] / 1.05) <= 1.0
    shift = prepare['frames'][3]['start_ms'] + prepare['frames'][3]['time_ms'] - 514
    for actual, original in zip(prepare['frames'][4:], old['frames'][4:], strict=True):
        expected = copy.deepcopy(original)
        expected['start_ms'] += shift
        assert actual == expected
    assert prepare['max_seq_ms'] == 730
    for name, seconds in MANIFEST['duration_seconds'].items():
        motion = after[name]
        assert max(f['start_ms'] + f['time_ms'] for f in motion['frames']) * motion['repeat_count'] / motion['playback_speed'] / 1000 == pytest.approx(seconds)


def test_aliases_keep_public_commands_and_resolve():
    relative = 'src/irc_step_motion_executor/config/motion_aliases.yaml'
    before = yaml.safe_load((UPDATE / 'before' / relative).read_text())['motion_aliases']
    after = yaml.safe_load((ROOT / relative).read_text())['motion_aliases']
    assert after == {key: MANIFEST['renames'].get(name, name) for key, name in before.items()}
    assert set(after.values()) <= catalog(ROOT / 'artifacts/robot_motions_runtime.json').keys()
