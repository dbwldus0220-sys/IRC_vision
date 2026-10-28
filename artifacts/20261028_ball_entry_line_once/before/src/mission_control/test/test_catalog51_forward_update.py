"""Validate the supplied forward gaits, alias coverage and walking preview timing."""

import copy
import hashlib
import json
from pathlib import Path

import pytest
import yaml

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from mission_control.motion_decision_node import MotionDecisionNode as Node


ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261009_catalog51_forward_update'
NEXT_UPDATE = ROOT / 'artifacts/20261027_catalog40_forward_pickup_update'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
SOURCE_PATH = ROOT / MANIFEST['source_file']
SOURCE = {m['name']: m for m in json.loads(SOURCE_PATH.read_text())['motions']}
ALIASES = yaml.safe_load((ROOT / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']


@pytest.mark.parametrize('filename', ['robot_motions_runtime.json', 'robot_motions_pc.json'])
def test_forward_import_matches_source_and_preserves_other_motions(filename):
    before = {m['name']: m for m in json.loads((UPDATE / filename.replace('.json', '.before.json')).read_text())['motions']}
    # The later 전. import supersedes these gaits; verify the saved boundary.
    motions = json.loads((NEXT_UPDATE / filename.replace('.json', '.before.json')).read_text())['motions']
    current = {m['name']: m for m in motions}
    assert len(current) == len(motions) == len(before) == 83
    assert current.keys() == before.keys() - MANIFEST['renames'].keys() | set(MANIFEST['renames'].values())
    for old, name in MANIFEST['renames'].items():
        expected = copy.deepcopy(SOURCE[name])
        expected['repeat_count'] = before[old]['repeat_count']
        assert expected['repeat_count'] == int(name.split('(')[1][0])
        for frame in expected['frames']:
            frame['name'] = MANIFEST['pose_renames'].get(frame['name'], frame['name'])
            if 'runtime' in filename:
                frame['angles'].update({'4': 18., '5': -18.})
        for key in ('start_pose', 'end_pose'):
            expected[key] = MANIFEST['pose_renames'].get(expected[key], expected[key])
        if 'runtime' in filename:
            expected['completion']['position_tolerance_deg'] = 5.
        assert current[name] == expected
    for name in before.keys() - MANIFEST['renames'].keys():
        assert current[name] == before[name]
    # The retained legacy posture remains unique for stationary turns/startup lookup.
    poses = [f['angles'] for m in motions for f in m['frames'] if f['name'] == '김오뒤3']
    assert poses and all(pose == poses[0] for pose in poses)


def test_all_normal_forward_aliases_resolve_and_fine_gaits_are_preserved():
    expected = {key: MANIFEST['renames'].get(name, name)
                for key, name in MANIFEST['aliases_before'].items()}
    expected['goal_forward_to_default'] = '건오뒤에서 기본자세(카메라45도)'
    aliases = yaml.safe_load((NEXT_UPDATE / 'motion_aliases.before.yaml').read_text())['motion_aliases']
    assert aliases == expected
    assert sum(name in MANIFEST['renames'].values() for name in aliases.values()) == 14
    runtime = {m['name'] for m in json.loads((NEXT_UPDATE / 'robot_motions_runtime.before.json').read_text())['motions']}
    assert set(aliases.values()) <= runtime
    assert not runtime & MANIFEST['renames'].keys()
    assert hashlib.sha256(SOURCE_PATH.read_bytes()).hexdigest() == MANIFEST['source_sha256']


@pytest.mark.parametrize('action', [
    'STRAIGHT', 'STRAIGHT_1', 'STRAIGHT_2', 'STRAIGHT_3', 'STRAIGHT_4',
    'GOAL_CAMERA_90_FORWARD', 'GOAL_CAMERA_90_FORWARD_1', 'GOAL_CAMERA_90_FORWARD_2',
])
def test_next_motion_capture_uses_the_new_catalog_duration(action):
    runtime = {m['name']: m for m in json.loads((ROOT / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
    motion = runtime[ALIASES[Bridge.motion_id_for_action(action)]]
    expected = max(f['start_ms'] + f['time_ms'] for f in motion['frames']) * motion['repeat_count'] / motion['playback_speed'] / 1000
    actual = (Node.LINE_MOTION_CAPTURE_CONFIG[action][0] if action.startswith('STRAIGHT')
              else Node.GOAL_FORWARD_DURATION_SEC[action])
    assert actual == pytest.approx(expected)


@pytest.mark.parametrize('action', ['LEFT', 'RIGHT', 'RECOVER_LEFT_TURN_LEFT_4', 'RECOVER_RIGHT_TURN_RIGHT_4'])
def test_recovery_preview_uses_four_cycles_at_approved_speed(action):
    runtime = {m['name']: m for m in json.loads((ROOT / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
    motion = runtime[ALIASES[Bridge.motion_id_for_action(action)]]
    assert motion['repeat_count'] == 4
    assert motion['playback_speed'] == 0.90
    duration = max(f['start_ms'] + f['time_ms'] for f in motion['frames']) * 4 / motion['playback_speed'] / 1000
    assert Node.LINE_MOTION_CAPTURE_CONFIG[action][0] == pytest.approx(duration)
