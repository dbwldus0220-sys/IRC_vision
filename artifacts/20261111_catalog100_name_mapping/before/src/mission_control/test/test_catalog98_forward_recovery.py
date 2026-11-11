"""Source98 fidelity and reservation budgets for every deployed walking route."""

import copy
import hashlib
import json
from pathlib import Path
import sys

import pytest
import yaml

from mission_control import ball_fine_approach
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.pickup_walk_reservation import PICKUP_WALK_DURATIONS

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261111_catalog98_forward_recovery'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source


def catalog(path):
    motions = json.loads(path.read_text())['motions']
    result = {m['name']: m for m in motions}
    assert len(result) == len(motions)
    return result


SOURCE = catalog(UPDATE / 'source_robot_motions.json')
RUNTIME = catalog(ROOT / 'artifacts/robot_motions_runtime.json')
ALIASES = yaml.safe_load((ROOT / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']


def seconds(alias):
    m = RUNTIME[ALIASES[alias]]
    return max(f['start_ms'] + f['time_ms'] for f in m['frames']) * m['repeat_count'] / m['playback_speed'] / 1000


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_supplied_joints_torques_timing_and_unrelated_motions_are_preserved(kind):
    # Source98 is superseded; check its archived deployment, not source99 data.
    actual = catalog(ROOT / f'artifacts/20261111_catalog99_forward_recovery/before/artifacts/robot_motions_{kind}.json')
    before = catalog(UPDATE / f'before/artifacts/robot_motions_{kind}.json')
    for name in MANIFEST['imports']:
        source_name, count = MANIFEST['derived'].get(name, (name, SOURCE.get(name, {}).get('repeat_count')))
        expected = copy.deepcopy(SOURCE[source_name])
        expected.update(name=name, repeat_count=count)
        renames = MANIFEST['pose_renames']
        for frame in expected['frames']:
            frame['name'] = renames.get(frame['name'], frame['name'])
        for field in ('start_pose', 'end_pose'):
            expected[field] = renames.get(expected[field], expected[field])
        if kind == 'runtime':
            expected = runtime_motion_from_source(expected)
        assert actual[name] == expected, name
    for name, motion in before.items():
        if name not in MANIFEST['renames'] and not name.endswith('+전진4회'):
            assert actual[name] == motion, name
    assert not set(MANIFEST['renames']) & actual.keys()
    assert len(actual) == len(before) + 3
    assert hashlib.sha256((UPDATE / 'source_robot_motions.json').read_bytes()).hexdigest() == MANIFEST['source_sha256']


def test_every_route_resolves_and_startup_pose_remains_unambiguous():
    assert set(Bridge.ACTION_TO_MOTION_ID.values()) <= ALIASES.keys()
    assert set(ALIASES.values()) <= RUNTIME.keys()
    before = yaml.safe_load((UPDATE / 'before/src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    expected = {alias: MANIFEST['renames'].get(name, name) for alias, name in before.items()}
    # Later goal-exit policy replaces only the two composite tails.
    expected.update({
        'pickup_forward_to_backward_prepare': 'n공에서 전진하고 후진준비자세',
        'post_shot_exit_right_5_forward_4': 't골대후제자리우회전(5회)+라인복귀좌회전45도(4회)',
        'post_shot_exit_left_3_forward_4': 'p골대후제자리좌회전(3회)+라인복귀우회전45도(4회)',
    })
    from test_goal_exit_recovery_catalog import EXPECTED_ALIASES
    expected.update(EXPECTED_ALIASES)
    assert ALIASES == expected
    for pose in {'오뒤무게중심앞'} | {f['name'] for n in MANIFEST['imports'] for f in RUNTIME[n]['frames']}:
        frames = [f for m in RUNTIME.values() for f in m['frames'] if f['name'] == pose]
        assert frames and all((f['angles'], f['torques']) == (frames[0]['angles'], frames[0]['torques']) for f in frames), pose


@pytest.mark.parametrize('name', MANIFEST['imports'])
def test_repeat_counts_speed_and_timeout_budget(name):
    m = RUNTIME[name]
    if name.startswith('n전진'):
        assert m['repeat_count'] == int(name.split('(')[1].split('회')[0])
    assert m['playback_speed'] == 1.0
    assert m['completion']['position_tolerance_deg'] == 5.0
    duration = max(f['start_ms'] + f['time_ms'] for f in m['frames']) * m['repeat_count']
    assert duration + m['completion']['settle_timeout_ms'] < Bridge.DEFAULT_TIMEOUT_MS


@pytest.mark.parametrize('action', [action for action in Node.LINE_MOTION_CAPTURE_CONFIG
                                   if action in Bridge.ACTION_TO_MOTION_ID])
def test_line_reservation_time_matches_catalog(action):
    assert Node.LINE_MOTION_CAPTURE_CONFIG[action][0] == pytest.approx(seconds(Bridge.motion_id_for_action(action)))


@pytest.mark.parametrize('action', Node.GOAL_FORWARD_DURATION_SEC)
def test_goal_reservation_time_matches_catalog(action):
    assert Node.GOAL_FORWARD_DURATION_SEC[action] == pytest.approx(seconds(Bridge.motion_id_for_action(action)))


@pytest.mark.parametrize('action,alias', [
    ('STRAIGHT', 'line_forward_4'), ('STRAIGHT_1', 'line_forward_2'),
    ('BALL_APPROACH_FORWARD_2', 'ball_align_forward_2'),
    ('RECOVER_LEFT_TURN_LEFT_4', 'line_recovery_left_4'),
    ('RECOVER_RIGHT_TURN_RIGHT_4', 'line_recovery_right_4'),
])
def test_ball_hurdle_reservation_time_matches_catalog(action, alias):
    assert Node.APPROACH_WALK_DURATION_SEC[action] == pytest.approx(seconds(alias))


def test_pickup_recovery_and_composite_budgets_match_catalog():
    assert ball_fine_approach.duration_sec('ball_align_forward_2') == pytest.approx(seconds('ball_align_forward_2'))
    assert PICKUP_WALK_DURATIONS['ball_camera_down_forward_2'] == pytest.approx(seconds('ball_camera_down_forward_2'))
    for alias, duration in Node.BALL_RECOVERY_DURATION_SEC.items():
        assert duration == pytest.approx(seconds(alias))
    for action, (duration, _) in Node.POST_SHOT_EXIT_CAPTURE_CONFIG.items():
        assert duration == pytest.approx(seconds(Bridge.motion_id_for_action(action)))
