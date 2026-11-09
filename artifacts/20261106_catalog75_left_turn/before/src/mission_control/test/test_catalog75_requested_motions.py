"""Validate the selective import, speed-only turns, and reservation timing."""

import copy
from decimal import Decimal
import json
from pathlib import Path

import pytest
import yaml

from mission_control.ball_fine_approach import duration_sec
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.pickup_walk_reservation import PICKUP_WALK_DURATIONS

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261106_catalog75_fine_hurdle'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
ALIASES = yaml.safe_load((ROOT / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']


def catalog(path):
    motions = json.loads(path.read_text())['motions']
    result = {m['name']: m for m in motions}
    assert len(result) == len(motions)
    return result


SOURCE = catalog(UPDATE / 'source_robot_motions.json')


def scaled(value):
    return float(Decimal(str(value)) * Decimal('.95'))


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_only_requested_data_and_required_preparation_endpoint_change(kind):
    before = catalog(UPDATE / f'before/artifacts/robot_motions_{kind}.json')
    after = catalog(ROOT / f'artifacts/robot_motions_{kind}.json')
    imported = set(MANIFEST['imported_motions'])
    assert set(after) == (set(before) - {'찐허들'}) | imported
    for name, old in before.items():
        if name in imported or name == '찐허들':
            continue
        expected = copy.deepcopy(old)
        if name in MANIFEST['speed_only_sources']:
            speed = SOURCE[MANIFEST['speed_only_sources'][name]]['playback_speed']
            expected['playback_speed'] = scaled(speed) if kind == 'runtime' else speed
        if name == MANIFEST['preparation_endpoint_update']:
            expected['frames'][-1]['angles'] = copy.deepcopy(SOURCE[name]['frames'][-1]['angles'])
            rename_poses(expected)
        assert after[name] == expected, name
    for name in imported:
        expected = copy.deepcopy(SOURCE[name])
        rename_poses(expected)
        if kind == 'runtime':
            expected['playback_speed'] = scaled(expected['playback_speed'])
            expected['completion']['position_tolerance_deg'] = 5.
        assert after[name] == expected, name


def rename_poses(motion):
    renames = MANIFEST['pose_renames']
    for frame in motion['frames']:
        frame['name'] = renames.get(frame['name'], frame['name'])
    for key in ('start_pose', 'end_pose'):
        motion[key] = renames.get(motion[key], motion[key])


def runtime_duration(alias):
    motion = catalog(ROOT / 'artifacts/robot_motions_runtime.json')[ALIASES[alias]]
    return max(f['start_ms'] + f['time_ms'] for f in motion['frames']) / 1000 * motion['repeat_count'] / motion['playback_speed']


def test_alias_ids_and_only_requested_routes_change():
    before = yaml.safe_load((UPDATE / 'before/src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    assert set(before) == set(ALIASES)
    assert {k for k in ALIASES if ALIASES[k] != before[k]} == {
        'hurdle', *(f'goal_camera_90_fine_forward_{i}' for i in range(1, 5))}
    assert ALIASES['hurdle'] == 't허들찐'
    for count in range(1, 5):
        assert ALIASES[f'goal_camera_90_fine_forward_{count}'] == f't미세90도({count}회)'
    assert set(ALIASES.values()) <= catalog(ROOT / 'artifacts/robot_motions_runtime.json').keys()


@pytest.mark.parametrize('side,counts', [('LEFT', range(1, 7)), ('RIGHT', (1, 2, 3, 5, 6, 7, 9))])
def test_turn_capture_and_prequeue_times_match_actual_retained_frames(side, counts):
    for count in counts:
        action = f'LINE_OFFSET_TURN_{side}_{count}'
        actual = runtime_duration(Bridge.motion_id_for_action(action))
        assert Node.LINE_TURN_CAPTURE_CONFIG[action][0] == pytest.approx(actual)
        assert Node.POST_SHOT_ALIGN_CAPTURE_CONFIG[f'POST_SHOT_LINE_TURN_{side}_{count}'][0] == pytest.approx(actual)
        assert duration_sec(f'ball_align_turn_{side.lower()}_{count}') == pytest.approx(actual)


def test_fine_capture_estimates_and_hurdle_timeout():
    estimates = {f'goal_camera_90_fine_forward_{i}': Node.GOAL_FORWARD_DURATION_SEC[f'GOAL_CAMERA90_FINE_FORWARD_{i}'] for i in range(1, 5)}
    estimates.update({k: v for k, v in PICKUP_WALK_DURATIONS.items() if 'fine_forward' in k})
    estimates['ball_approach_fine_forward_45_5'] = Node.APPROACH_WALK_DURATION_SEC['BALL_APPROACH_FINE_FORWARD_5']
    estimates['ball_align_fine_forward_45_5'] = duration_sec('ball_align_fine_forward_45_5')
    for alias, estimate in estimates.items():
        assert estimate == pytest.approx(runtime_duration(alias))
    hurdle = SOURCE['t허들찐']
    assert runtime_duration('hurdle') * 1000 + hurdle['completion']['settle_timeout_ms'] < Bridge.DEFAULT_TIMEOUT_MS


def test_preparation_and_new_pose_names_do_not_resolve_to_old_targets():
    motions = catalog(ROOT / 'artifacts/robot_motions_runtime.json')
    assert motions['t오뒤에서 미세오뒤(45도)']['frames'][-1]['angles'] == motions['t미세45도(5회)']['frames'][-1]['angles']
    for pose in set(MANIFEST['pose_renames'].values()) | {'오뒤무게중심앞'}:
        angles = [f['angles'] for m in motions.values() for f in m['frames'] if f['name'] == pose]
        assert angles and all(a == angles[0] for a in angles), pose
