"""Verify source78 transition imports, speed-only turns and timing contracts."""

import copy
import json
from pathlib import Path

import pytest
import yaml

from mission_control.ball_fine_approach import duration_sec
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from mission_control.motion_decision_node import MotionDecisionNode as Node

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261109_catalog78_transitions'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
ALIAS_PATH = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')


def catalog(path):
    motions = json.loads(path.read_text())['motions']
    result = {m['name']: m for m in motions}
    assert len(result) == len(motions)
    return result


SOURCE = catalog(UPDATE / 'source_robot_motions.json')
RUNTIME = catalog(ROOT / 'artifacts/robot_motions_runtime.json')
ALIASES = yaml.safe_load((ROOT / ALIAS_PATH).read_text())['motion_aliases']


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_exact_source_transitions_speed_only_turns_and_unrelated_preservation(kind):
    before = catalog(UPDATE / f'before/artifacts/robot_motions_{kind}.json')
    after = catalog(ROOT / f'artifacts/robot_motions_{kind}.json')
    renames = MANIFEST['renames']
    assert set(after) == (set(before) - set(renames)) | set(renames.values())
    for name, original in before.items():
        if name in renames:
            expected = copy.deepcopy(SOURCE[renames[name]])
            for frame in expected['frames']:
                frame['name'] = MANIFEST['pose_renames'].get(frame['name'], frame['name'])
            for key in ('start_pose', 'end_pose'):
                expected[key] = MANIFEST['pose_renames'].get(expected[key], expected[key])
            if kind == 'runtime':
                expected['completion']['position_tolerance_deg'] = 5.0
        else:
            expected = copy.deepcopy(original)
            if name in MANIFEST['speed_only_motions']:
                expected['playback_speed'] = SOURCE[name]['playback_speed']
        assert after[renames.get(name, name)] == expected, name


def test_all_aliases_keep_ids_and_only_requested_targets_change():
    before = yaml.safe_load((UPDATE / 'before' / ALIAS_PATH).read_text())['motion_aliases']
    assert ALIASES == {a: MANIFEST['renames'].get(n, n) for a, n in before.items()}
    assert len(MANIFEST['changed_aliases']) == 9
    assert set(ALIASES.values()) <= RUNTIME.keys()
    for alias in Bridge.ACTION_TO_MOTION_ID.values():
        assert alias in ALIASES


@pytest.mark.parametrize('camera', [0, 45, 90])
def test_transition_endpoints_match_fine_and_standard_support(camera):
    to_fine = f'p오뒤에서 미세오뒤{camera}도' if camera == 90 else f'p오뒤에서 미세오뒤({camera}도)'
    to_turn = f'p미세오뒤에서 오뒤{camera}도'
    standard = {0: 'p전진실전0도(4회)', 45: 'p전진45(4회)', 90: 'p전진90도(4회)'}[camera]
    for transition, gait in [(to_fine, f'p미세{camera}도(5회)'), (to_turn, standard)]:
        a, b = RUNTIME[transition]['frames'][-1], RUNTIME[gait]['frames'][-1]
        assert a['angles'] == b['angles']
        assert a['torques'] == b['torques']


@pytest.mark.parametrize('name', MANIFEST['speed_only_motions'] + list(MANIFEST['renames'].values()))
def test_requested_speed_is_source_verbatim(name):
    assert RUNTIME[name]['playback_speed'] == SOURCE[name]['playback_speed']


def seconds(alias):
    motion = RUNTIME[ALIASES[alias]]
    return max(f['start_ms'] + f['time_ms'] for f in motion['frames']) / 1000 * motion['repeat_count'] / motion['playback_speed']


@pytest.mark.parametrize('count', range(1, 7))
def test_faster_turn_capture_and_reservation_windows_match_runtime(count):
    actual = seconds(f'ball_align_turn_left_{count}')
    assert duration_sec(f'ball_align_turn_left_{count}') == pytest.approx(actual)
    assert Node.LINE_TURN_CAPTURE_CONFIG[f'LINE_OFFSET_TURN_LEFT_{count}'][0] == pytest.approx(actual)
    assert Node.POST_SHOT_ALIGN_CAPTURE_CONFIG[f'POST_SHOT_LINE_TURN_LEFT_{count}'][0] == pytest.approx(actual)


@pytest.mark.parametrize('alias', ['ball_align_prepare_fine_45', 'ball_align_prepare_turn_45'])
def test_preparation_reservation_window_matches_source_speed(alias):
    assert duration_sec(alias) == pytest.approx(seconds(alias))


def test_imported_pose_names_are_unambiguous():
    names = set(MANIFEST['pose_renames'].values()) | {'오뒤무게중심앞'}
    for name in names:
        angles = [f['angles'] for m in RUNTIME.values() for f in m['frames'] if f['name'] == name]
        assert angles and all(a == angles[0] for a in angles), name
