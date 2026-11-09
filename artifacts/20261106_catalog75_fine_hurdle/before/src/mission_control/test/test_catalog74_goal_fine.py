"""Verify calibrated Goal steps, isolated imports, and scaled capture timing."""

import copy
from decimal import Decimal
import json
from pathlib import Path

import pytest
import yaml

from mission_control.ball_fine_approach import duration_sec
from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.pickup_walk_reservation import PICKUP_WALK_DURATIONS
from step.goal_navigation_planner import GoalNavigationPlanner
from test_continuous_approach import clock, fresh, start_goal
from test_goal_target_steering import target_sample
from test_motion_command_bridge import FakeBridge, decoded_messages, navigation_message
from test_wait_refresh_and_fine_settle import LiveInputHarness

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261106_catalog74_goal_fine'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
ALIASES = yaml.safe_load((ROOT / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']


def catalog(path):
    motions = json.loads(path.read_text())['motions']
    result = {m['name']: m for m in motions}
    assert len(result) == len(motions)
    return result


@pytest.mark.parametrize('depth,count', [
    (.590, 1), (.600, 1), (.619999, 1), (.620, 2), (.649999, 2),
    (.650, 3), (.679999, 3), (.680, 4), (.700, 4), (.710, 4), (.850, 4),
])
def test_depth_selects_smallest_available_step_into_scoring_band(depth, count):
    command = GoalNavigationPlanner().plan(target_sample(0, depth))
    assert command.action == f'GOAL_CAMERA90_FINE_FORWARD_{count}'
    if depth < .710:
        assert depth - count * .030 < .590
        assert round(depth - (count - 1) * .030, 9) >= .590
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        source='goal', action=command.action, source_command=command.to_dict()))
    motion_id = decoded_messages(bridge.executor_request_publisher)[-1]['motion_id']
    assert ALIASES[motion_id] == f't미세0도({count}회)(골대74)'


@pytest.mark.parametrize('depth,updates,action', [
    (.589999, {}, 'WAIT_SCORE_CONFIRMATION'),
    (.589999, {'score_now': True}, 'SHOT'),
    (.449999, {}, 'GOAL_CAMERA90_BACKWARD_1'),
    (.850001, {}, 'GOAL_CAMERA_90_FORWARD_2'),
    (.650, {'bearing_deg': 30, 'offset_x_px': 200}, 'GOAL_CAMERA90_TURN_RIGHT_3'),
    (.650, {'depth_valid': False}, 'WAIT'),
])
def test_other_goal_decisions_keep_their_priority(depth, updates, action):
    assert GoalNavigationPlanner().plan({**target_sample(0, depth), **updates}).action == action


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_import_preserves_source_and_unrelated_motions(kind):
    after = catalog(ROOT / f'artifacts/robot_motions_{kind}.json')
    before = catalog(UPDATE / f'before/artifacts/robot_motions_{kind}.json')
    source = catalog(UPDATE / 'source_robot_motions.json')
    assert set(after) - set(before) == set(MANIFEST['imported_motions'])
    for name, old in before.items():
        expected = copy.deepcopy(old)
        if kind == 'runtime' and name in MANIFEST['speed_only_motions']:
            original = catalog(UPDATE / 'before/artifacts/robot_motions_pc.json')[name]
            expected['playback_speed'] = float(Decimal(str(original['playback_speed'])) * Decimal('.95'))
        assert after[name] == expected, name
    for name, original_name in MANIFEST['imported_motions'].items():
        expected = copy.deepcopy(source[original_name])
        expected['name'] = name
        renames = MANIFEST['imported_pose_renames']
        for frame in expected['frames']:
            frame['name'] = renames[frame['name']]
        for key in ('start_pose', 'end_pose'):
            expected[key] = renames[expected[key]]
        if kind == 'runtime':
            expected['playback_speed'] = float(Decimal(str(expected['playback_speed'])) * Decimal('.95'))
            expected['completion']['position_tolerance_deg'] = 5.
        assert after[name] == expected


def test_only_goal_aliases_change_and_all_aliases_resolve():
    before = yaml.safe_load((UPDATE / 'before/src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    changed = {k for k in ALIASES if ALIASES[k] != before[k]}
    assert changed == {f'goal_camera_90_fine_forward_{i}' for i in range(1, 5)}
    assert set(ALIASES) == set(before)
    assert set(ALIASES.values()) <= catalog(ROOT / 'artifacts/robot_motions_runtime.json').keys()


def test_scaled_capture_times_match_runtime_catalog():
    motions = catalog(ROOT / 'artifacts/robot_motions_runtime.json')
    estimates = {f'goal_camera_90_fine_forward_{i}': Node.GOAL_FORWARD_DURATION_SEC[f'GOAL_CAMERA90_FINE_FORWARD_{i}'] for i in range(1, 5)}
    estimates.update({key: value for key, value in PICKUP_WALK_DURATIONS.items() if 'fine_forward' in key})
    estimates['ball_approach_fine_forward_45_5'] = Node.APPROACH_WALK_DURATION_SEC['BALL_APPROACH_FINE_FORWARD_5']
    estimates['ball_align_fine_forward_45_5'] = duration_sec('ball_align_fine_forward_45_5')
    for alias, estimate in estimates.items():
        motion = motions[ALIASES[alias]]
        end_ms = max(f['start_ms'] + f['time_ms'] for f in motion['frames'])
        assert estimate == pytest.approx(end_ms / 1000 * motion['repeat_count'] / motion['playback_speed'])


def test_goal_reduces_reserved_repeat_count_with_latest_depth(clock):
    node = LiveInputHarness(phase='GOAL_APPROACH')
    first = start_goal(node, clock, .700)
    assert first['action'] == 'GOAL_CAMERA90_FINE_FORWARD_4'
    clock[0] = node.goal_forward_started_at + Node.GOAL_FORWARD_DURATION_SEC[first['action']] * .75
    for _ in range(3):
        fresh(node, clock, 'goal', target_sample(0, .600), plan=False)
    second = node.publisher.messages[-1]
    assert second['action'] == 'GOAL_CAMERA90_FINE_FORWARD_1'
    assert node.queued_general_command_id == second['command_id']


def test_goal_in_scoring_band_never_reserves_another_fine_step(clock):
    node = LiveInputHarness(phase='GOAL_APPROACH')
    first = start_goal(node, clock, .600)
    clock[0] = node.goal_forward_started_at + Node.GOAL_FORWARD_DURATION_SEC[first['action']] * .75
    for _ in range(3):
        fresh(node, clock, 'goal', target_sample(0, .570), plan=False)
    assert getattr(node, 'queued_general_command_id', None) is None
