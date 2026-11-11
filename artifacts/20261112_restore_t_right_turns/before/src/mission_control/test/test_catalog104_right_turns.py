"""Source104 right turns retain complete cycles, source targets, and command routes."""

import copy
import hashlib
import json
from pathlib import Path
import sys

import pytest
import yaml

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from mission_control.motion_decision_node import MotionDecisionNode as Node

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261112_catalog104_right_turns'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
sys.path.insert(0, str(ROOT / 'tools'))
from build_post_shot_forward_exits import build_exit
from upsert_motion_catalog import runtime_motion_from_source


def catalog(path):
    motions = json.loads(path.read_text())['motions']
    result = {m['name']: m for m in motions}
    assert len(result) == len(motions)
    return result


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_source_frames_and_unrelated_motions_are_preserved(kind):
    path = UPDATE / 'source_robot_motions.json'
    assert hashlib.sha256(path.read_bytes()).hexdigest() == MANIFEST['source_sha256']
    source = catalog(path)
    relative = f'artifacts/robot_motions_{kind}.json'
    before, after = catalog(UPDATE / 'before' / relative), catalog(ROOT / relative)
    renames = MANIFEST['renames']
    assert after.keys() == {renames.get(name, name) for name in before}
    for name in before.keys() - renames.keys():
        assert after[name] == before[name], name
    for name in ('m공후제자리우회전', 'm골대후제자리우회전'):
        expected = copy.deepcopy(source[name])
        for frame in expected['frames']:
            if frame['name'] == '오뒤무게중심앞':
                frame['name'] += '(catalog104)'
        if kind == 'runtime':
            expected = runtime_motion_from_source(expected)
        assert after[name] == expected
        assert after[name]['playback_speed'] == 1.0
        assert after[name]['frames'][-1]['angles']['0'] == -64.0


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_short_turns_use_complete_four_frame_cycles(kind):
    motions = catalog(ROOT / f'artifacts/robot_motions_{kind}.json')
    base = motions['m골대후제자리우회전']
    assert len(base['frames']) == 2 + 4 * 7
    for count in (5, 6):
        variant = motions[f'm골대후제자리우회전({count}회)']
        expected = copy.deepcopy(base)
        expected['name'] = variant['name']
        expected['frames'] = base['frames'][:2 + 4 * count]
        expected['max_seq_ms'] = max(f['start_ms'] + f['time_ms'] for f in expected['frames'])
        assert variant == expected
        for field in ('angles', 'torques'):
            assert variant['frames'][-1][field] == base['frames'][-1][field]


def test_all_aliases_resolve_and_updated_durations_fit_timeout():
    relative = 'src/irc_step_motion_executor/config/motion_aliases.yaml'
    before = yaml.safe_load((UPDATE / 'before' / relative).read_text())['motion_aliases']
    aliases = yaml.safe_load((ROOT / relative).read_text())['motion_aliases']
    assert aliases == {key: MANIFEST['renames'].get(name, name) for key, name in before.items()}
    motions = catalog(ROOT / 'artifacts/robot_motions_runtime.json')
    assert set(aliases.values()) <= motions.keys()
    for name, seconds in MANIFEST['duration_seconds'].items():
        motion = motions[name]
        assert max(f['start_ms'] + f['time_ms'] for f in motion['frames']) / motion['playback_speed'] / 1000 == pytest.approx(seconds)
        assert seconds * 1000 + motion['completion']['settle_timeout_ms'] < Bridge.DEFAULT_TIMEOUT_MS
    for action, (seconds, _) in Node.POST_SHOT_EXIT_CAPTURE_CONFIG.items():
        motion = motions[aliases[Bridge.ACTION_TO_MOTION_ID[action]]]
        assert seconds == pytest.approx(motion['max_seq_ms'] / motion['playback_speed'] / 1000)


def test_camera_join_keeps_body_and_torque_checks():
    pc = catalog(ROOT / 'artifacts/robot_motions_pc.json')
    turn, recovery = pc['m골대후제자리우회전'], pc['m라인복귀우회전45도']
    assert turn['frames'][-1]['angles']['0'] != recovery['frames'][-1]['angles']['0']
    assert build_exit(turn, recovery) == pc[turn['name'] + '+라인복귀우회전45도(4회)']
    bad_body = copy.deepcopy(turn)
    bad_body['frames'][-1]['angles']['4'] += 1.0
    with pytest.raises(AssertionError):
        build_exit(bad_body, recovery)
    bad_torque = copy.deepcopy(turn)
    bad_torque['frames'][-1]['torques']['0'] = False
    with pytest.raises(AssertionError):
        build_exit(bad_torque, recovery)
