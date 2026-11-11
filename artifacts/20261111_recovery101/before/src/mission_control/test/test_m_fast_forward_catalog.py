"""Check source fidelity, command routes, and reservation timing without motors."""

import copy
import json
from pathlib import Path
import sys

import pytest
import yaml

from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.ball_fine_approach import duration_sec
from mission_control.pickup_walk_reservation import PICKUP_WALK_DURATIONS

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261111_m_fast_forward'
DEPLOYED = ROOT / 'artifacts/20261111_j_forward102/before'
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
def test_requested_data_and_unrelated_motions(kind):
    source = catalog(UPDATE / 'source_robot_motions.json')
    relative = f'artifacts/robot_motions_{kind}.json'
    before, after = catalog(UPDATE / 'before' / relative), catalog(DEPLOYED / relative)
    renames = MANIFEST['renames']
    assert after.keys() == {renames.get(name, name) for name in before}
    for name, old in before.items():
        target = renames.get(name, name)
        if target not in MANIFEST['replaced']:
            assert after[target] == old
    for name in [*renames.values(), 'r골넣기8번구역']:
        expected = copy.deepcopy(source['m전진45(6회)' if name == 'm전진45(8회)' else name])
        expected['name'] = name
        if name == 'm전진45(8회)':
            expected['repeat_count'] = 8
        for frame in expected['frames']:
            if frame['name'] == '오뒤무게중심앞':
                frame['name'] += '(catalog80)'
        for key in ('start_pose', 'end_pose'):
            if expected[key] == '오뒤무게중심앞':
                expected[key] += '(catalog80)'
        if kind == 'runtime':
            expected = runtime_motion_from_source(expected)
        assert after[name] == expected
    pc = catalog(DEPLOYED / 'artifacts/robot_motions_pc.json')
    for turn, recovery, blocks in GOAL_EXIT_RECOVERIES:
        expected = build_exit(pc[turn], pc[recovery], blocks,
                              forward=pc['m전진45(2회)'] if turn in FORWARD_EXIT_NAMES else None)
        if kind == 'runtime':
            expected = runtime_motion_from_source(expected)
        assert after[expected['name']] == expected


def test_aliases_keep_public_ids_and_resolve_new_names():
    relative = 'src/irc_step_motion_executor/config/motion_aliases.yaml'
    before = yaml.safe_load((UPDATE / 'before' / relative).read_text())['motion_aliases']
    after = yaml.safe_load((DEPLOYED / relative).read_text())['motion_aliases']
    assert after == {key: MANIFEST['renames'].get(value, value) for key, value in before.items()}
    assert set(after.values()) <= catalog(DEPLOYED / 'artifacts/robot_motions_runtime.json').keys()


def test_reservation_durations_match_actual_catalog():
    runtime = catalog(ROOT / 'artifacts/robot_motions_runtime.json')
    aliases = yaml.safe_load((ROOT / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']

    def duration(alias):
        motion = runtime[aliases[alias]]
        return max(f['start_ms'] + f['time_ms'] for f in motion['frames']) * motion['repeat_count'] / motion['playback_speed'] / 1000

    for action, count in [('STRAIGHT', 6), ('STRAIGHT_1', 2), ('STRAIGHT_2', 4), ('STRAIGHT_3', 6), ('STRAIGHT_4', 8)]:
        assert Node.LINE_MOTION_CAPTURE_CONFIG[action][0] == pytest.approx(duration(f'line_forward_{count}'))
    for action, (seconds, _) in Node.LINE_MOTION_CAPTURE_CONFIG.items():
        if 'LEFT' == action or 'TURN_LEFT' in action:
            assert seconds == pytest.approx(duration('line_recovery_left_4'))
        elif 'RIGHT' == action or 'TURN_RIGHT' in action:
            assert seconds == pytest.approx(duration('line_recovery_right_4'))
    for alias, seconds in Node.BALL_RECOVERY_DURATION_SEC.items():
        assert seconds == pytest.approx(duration(alias))
    for action, (seconds, _) in Node.POST_SHOT_EXIT_CAPTURE_CONFIG.items():
        assert seconds == pytest.approx(duration(action.lower()))
    for action, alias in {
        'GOAL_CAMERA_90_FORWARD': 'goal_camera_90_forward_6',
        'GOAL_CAMERA_90_FORWARD_1': 'goal_camera_90_forward_2',
        'GOAL_CAMERA_90_FORWARD_2': 'goal_camera_90_forward_4',
        'GOAL_CAMERA90_RECOVER_LEFT_4': 'line_recovery_left_4',
        'GOAL_CAMERA90_RECOVER_RIGHT_4': 'line_recovery_right_4',
    }.items():
        assert Node.GOAL_FORWARD_DURATION_SEC[action] == pytest.approx(duration(alias))
    for action, alias in {
        'BALL_APPROACH_FORWARD_2': 'line_forward_2', 'STRAIGHT': 'line_forward_4',
        'STRAIGHT_1': 'line_forward_2', 'RECOVER_LEFT_TURN_LEFT_4': 'line_recovery_left_4',
        'RECOVER_RIGHT_TURN_RIGHT_4': 'line_recovery_right_4',
    }.items():
        assert Node.APPROACH_WALK_DURATION_SEC[action] == pytest.approx(duration(alias))
    assert duration_sec('ball_align_forward_2') == pytest.approx(duration('ball_align_forward_2'))
    assert PICKUP_WALK_DURATIONS['ball_camera_down_forward_2'] == pytest.approx(duration('ball_camera_down_forward_2'))
