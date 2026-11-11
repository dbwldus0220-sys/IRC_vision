"""Source103 gait and hurdle import with camera correction and stable command routes."""

import copy
import hashlib
import json
from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261112_catalog103'
DEPLOYED = ROOT / 'artifacts/20261112_catalog104_right_turns/before'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source
from build_post_shot_forward_exits import build_exit, FORWARD_EXIT_NAMES

# Source103 deployment, before source104 replaced the right turns.
GOAL_EXIT_RECOVERIES = (
    ('t골대후제자리우회전', 'm라인복귀우회전45도', 1),
    ('p골대후제자리좌회전', 'm라인복귀우회전45도', 1),
    ('t골대후제자리우회전(6회)', 'm라인복귀우회전45도', 1),
    ('p골대후제자리좌회전(4회)', 'm라인복귀좌회전45도', 1),
)


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
    before, after = catalog(UPDATE / 'before' / relative), catalog(DEPLOYED / relative)
    renames = MANIFEST['renames']
    assert len(renames) == 18
    assert len(MANIFEST['replaced']) == 24
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
        assert after[name]['playback_speed'] == 1.0
        assert [f['angles']['0'] for f in after[name]['frames']] == [f['angles']['0'] for f in old['frames']]
        for field in ('angles', 'torques'):
            assert after[name]['frames'][-1][field] == old['frames'][-1][field]
    assert after['m전진45(8회)']['repeat_count'] == 8
    assert after['m전진45(8회)']['frames'] == after['m전진45(6회)']['frames']
    for name in before.keys() - renames.keys() - set(MANIFEST['replaced']):
        assert after[name] == before[name], name


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_embedded_gaits_preserve_turns_preparation_and_effective_speed(kind):
    pc = catalog(DEPLOYED / 'artifacts/robot_motions_pc.json')
    after = catalog(DEPLOYED / f'artifacts/robot_motions_{kind}.json')
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
            assert abs(actual[time] - source[time] / 1.0) <= 1.0
    shift = prepare['frames'][3]['start_ms'] + prepare['frames'][3]['time_ms'] - 514
    for actual, original in zip(prepare['frames'][4:], old['frames'][4:], strict=True):
        expected = copy.deepcopy(original)
        expected['start_ms'] += shift
        assert actual == expected
    assert prepare['max_seq_ms'] == 754
    for name, seconds in MANIFEST['duration_seconds'].items():
        motion = after[name]
        if kind == 'pc' and name == 't허들찐':
            seconds *= .95  # PC keeps source speed; only runtime slows the hurdle.
        assert max(f['start_ms'] + f['time_ms'] for f in motion['frames']) * motion['repeat_count'] / motion['playback_speed'] / 1000 == pytest.approx(seconds)


def test_aliases_keep_public_commands_and_resolve():
    relative = 'src/irc_step_motion_executor/config/motion_aliases.yaml'
    before = yaml.safe_load((UPDATE / 'before' / relative).read_text())['motion_aliases']
    after = yaml.safe_load((DEPLOYED / relative).read_text())['motion_aliases']
    assert after == {key: MANIFEST['renames'].get(name, name) for key, name in before.items()}
    assert set(after.values()) <= catalog(DEPLOYED / 'artifacts/robot_motions_runtime.json').keys()


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_hurdle_matches_source_once_scaled_and_preserves_exit(kind):
    source = catalog(UPDATE / 'source_robot_motions.json')['t허들찐']
    expected = copy.deepcopy(source)
    for frame in expected['frames']:
        if frame['name'] in ('s왼들', '오뒤무게중심앞'):
            frame['name'] += '(catalog80)'
    for key in ('start_pose', 'end_pose'):
        if expected[key] in ('s왼들', '오뒤무게중심앞'):
            expected[key] += '(catalog80)'
    if kind == 'runtime':
        expected = runtime_motion_from_source(expected)
    after = catalog(DEPLOYED / f'artifacts/robot_motions_{kind}.json')['t허들찐']
    before = catalog(UPDATE / 'before' / f'artifacts/robot_motions_{kind}.json')['t허들찐']
    assert after == expected
    assert after['repeat_count'] == 1
    assert after['playback_speed'] == (0.95 if kind == 'runtime' else 1.0)
    assert max(f['start_ms'] + f['time_ms'] for f in after['frames']) == 7312
    assert after['frames'][9]['angles']['20'] == -68.412109375
    for field in ('angles', 'torques'):
        assert after['frames'][-1][field] == before['frames'][-1][field]
