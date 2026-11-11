"""Preserve source75 history; verify current reservation timing separately."""

import copy
import json
from pathlib import Path

import pytest
import yaml

from mission_control.ball_fine_approach import duration_sec
from mission_control.motion_decision_node import MotionDecisionNode as Node

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261106_catalog75_left_turn'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
LEFT6_UPDATE = ROOT / 'artifacts/20261106_catalog74_left6'
LEFT6_MANIFEST = json.loads((LEFT6_UPDATE / 'manifest.json').read_text())
SNAPSHOT = ROOT / 'artifacts/20261107_catalog76_turn_crab/before'
ALIAS_PATH = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')


def catalog(path):
    motions = json.loads(path.read_text())['motions']
    result = {m['name']: m for m in motions}
    assert len(result) == len(motions)
    return result


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_all_eighteen_turns_match_source_and_every_other_motion_is_unchanged(kind):
    source = catalog(UPDATE / 'source_robot_motions.json')
    before = catalog(UPDATE / f'before/artifacts/robot_motions_{kind}.json')
    after = catalog(SNAPSHOT / f'artifacts/robot_motions_{kind}.json')
    renames = MANIFEST['renames']
    assert len(renames) == 18
    assert set(after) == (set(before) - set(renames)) | set(renames.values())
    for old, motion in before.items():
        if old not in renames:
            assert after[old] == motion, old
            continue
        name = renames[old]
        expected = copy.deepcopy(source[name])
        poses = MANIFEST['pose_renames']
        if name == LEFT6_MANIFEST['motion']:
            expected = copy.deepcopy(catalog(LEFT6_UPDATE / 'source_robot_motions.json')[name])
            poses = LEFT6_MANIFEST['pose_renames']
        for frame in expected['frames']:
            frame['name'] = poses.get(frame['name'], frame['name'])
        for key in ('start_pose', 'end_pose'):
            expected[key] = poses.get(expected[key], expected[key])
        if kind == 'runtime':
            expected['playback_speed'] = .855
            expected['completion']['position_tolerance_deg'] = 5.
        assert after[name] == expected, name


def test_every_alias_keeps_its_id_camera_and_repeat_count():
    before = yaml.safe_load((UPDATE / 'before' / ALIAS_PATH).read_text())['motion_aliases']
    after = yaml.safe_load((SNAPSHOT / ALIAS_PATH).read_text())['motion_aliases']
    assert after == {alias: MANIFEST['renames'].get(name, name) for alias, name in before.items()}
    motions = catalog(SNAPSHOT / 'artifacts/robot_motions_runtime.json')
    assert set(after.values()) <= motions.keys()
    for camera, prefix in [(0, 'pickup_camera_down'), (45, 'post_ball_line'), (90, 'goal_camera_90')]:
        for count in range(1, 7):
            name = after[f'{prefix}_turn_left_{count}']
            assert name == f't제자리좌회전{camera}도({count}회)'
            assert motions[name]['repeat_count'] == count


@pytest.mark.parametrize('count', range(1, 7))
def test_reservation_duration_uses_new_frames_and_common_speed(count):
    motion = catalog(ROOT / 'artifacts/robot_motions_runtime.json')[f'p제자리좌회전45({count}회)']
    cycle_ms = max(f['start_ms'] + f['time_ms'] for f in motion['frames'])
    assert cycle_ms == 355
    seconds = cycle_ms / 1000 * motion['repeat_count'] / motion['playback_speed']
    assert duration_sec(f'ball_align_turn_left_{count}') == pytest.approx(seconds)
    assert Node.LINE_TURN_CAPTURE_CONFIG[f'LINE_OFFSET_TURN_LEFT_{count}'][0] == pytest.approx(seconds)
    assert Node.POST_SHOT_ALIGN_CAPTURE_CONFIG[f'POST_SHOT_LINE_TURN_LEFT_{count}'][0] == pytest.approx(seconds)


def test_imported_pose_names_do_not_override_startup_targets():
    motions = catalog(SNAPSHOT / 'artifacts/robot_motions_runtime.json')
    for pose in set(MANIFEST['pose_renames'].values()) | set(LEFT6_MANIFEST['pose_renames'].values()) | {'오뒤무게중심앞'}:
        angles = [f['angles'] for m in motions.values() for f in m['frames'] if f['name'] == pose]
        assert angles and all(a == angles[0] for a in angles), pose


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_source74_replacement_changes_only_the_requested_motion(kind):
    before = catalog(LEFT6_UPDATE / f'before/artifacts/robot_motions_{kind}.json')
    after = catalog(SNAPSHOT / f'artifacts/robot_motions_{kind}.json')
    assert set(after) == set(before)
    assert {name for name in before if before[name] != after[name]} == {LEFT6_MANIFEST['motion']}
    assert (SNAPSHOT / ALIAS_PATH).read_bytes() == (LEFT6_UPDATE / 'before' / ALIAS_PATH).read_bytes()
