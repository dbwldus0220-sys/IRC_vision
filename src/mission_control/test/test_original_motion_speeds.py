"""Restore source speeds without changing motion data or reservation timing."""

import json
from decimal import Decimal
from pathlib import Path

import pytest
import yaml

from mission_control.ball_fine_approach import duration_sec
from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.pickup_walk_reservation import PICKUP_WALK_DURATIONS


ROOT = Path(__file__).resolve().parents[3]
BEFORE = ROOT / 'artifacts/20261110_restore_original_speeds/before/artifacts'
EXCEPTIONS = {'t허들찐', 'p공후 제자리좌회전', 'p골대후제자리좌회전'}
CURRENT_EXCEPTIONS = {'t허들찐'}


def catalog(path):
    return json.loads(path.read_text())


SOURCE = catalog(ROOT / 'artifacts/robot_motions_pc.json')
RUNTIME = catalog(ROOT / 'artifacts/robot_motions_runtime.json')
ALIASES = yaml.safe_load((ROOT / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
MOTIONS = {m['name']: m for m in RUNTIME['motions']}


def seconds(alias):
    m = MOTIONS[ALIASES[alias]]
    return max(f['start_ms'] + f['time_ms'] for f in m['frames']) / 1000 * m['repeat_count'] / m['playback_speed']


@pytest.mark.parametrize('original', SOURCE['motions'], ids=lambda m: m['name'])
def test_current_runtime_speed_policy(original):
    factor = Decimal('.95') if original['name'] in CURRENT_EXCEPTIONS else Decimal('1')
    expected = float(Decimal(str(original['playback_speed'])) * factor)
    assert MOTIONS[original['name']]['playback_speed'] == expected


def test_only_speeds_changed_and_original_catalog_is_untouched():
    before = catalog(BEFORE / 'robot_motions_runtime.json')
    assert len(RUNTIME['motions']) == len(before['motions']) == 155
    expected = catalog(BEFORE / 'robot_motions_runtime.json')
    changed = []
    for old, new, patched in zip(before['motions'], RUNTIME['motions'], expected['motions']):
        assert old['name'] == new['name']
        if old['playback_speed'] != new['playback_speed']:
            changed.append(new['name'])
        if old['name'] in EXCEPTIONS:
            assert old == new
        patched['playback_speed'] = new['playback_speed']
    assert RUNTIME == expected
    assert len(changed) == 63
    assert (ROOT / 'artifacts/robot_motions_pc.json').read_bytes() == (BEFORE / 'robot_motions_pc.json').read_bytes()


@pytest.mark.parametrize('action,count', [('STRAIGHT', 6), ('STRAIGHT_1', 2),
                                        ('STRAIGHT_2', 4), ('STRAIGHT_3', 6), ('STRAIGHT_4', 8)])
def test_line_duration_tracks_restored_forward_speed(action, count):
    assert Node.LINE_MOTION_CAPTURE_CONFIG[action][0] == pytest.approx(seconds(f'line_forward_{count}'))


@pytest.mark.parametrize('alias', [a for a in ALIASES if a.startswith('ball_align_')])
def test_ball_alignment_duration_matches_catalog(alias):
    assert duration_sec(alias) == pytest.approx(seconds(alias))


@pytest.mark.parametrize('alias,estimate', list(PICKUP_WALK_DURATIONS.items())
                         + list(Node.BALL_RECOVERY_DURATION_SEC.items()))
def test_pickup_and_recovery_reservations_use_actual_speed(alias, estimate):
    assert estimate == pytest.approx(seconds(alias))


@pytest.mark.parametrize('action,alias', [
    ('GOAL_CAMERA_90_FORWARD', 'goal_camera_90_forward_6'),
    ('GOAL_CAMERA_90_FORWARD_1', 'goal_camera_90_forward_2'),
    ('GOAL_CAMERA_90_FORWARD_2', 'goal_camera_90_forward_4'),
    ('GOAL_CAMERA90_RECOVER_LEFT_4', 'line_recovery_left_4'),
    ('GOAL_CAMERA90_RECOVER_RIGHT_4', 'line_recovery_right_4'),
    *[(f'GOAL_CAMERA90_FINE_FORWARD_{n}', f'goal_camera_90_fine_forward_{n}') for n in range(1, 6)],
])
def test_goal_reservations_use_actual_speed(action, alias):
    assert Node.GOAL_FORWARD_DURATION_SEC[action] == pytest.approx(seconds(alias))


def test_approach_reservations_use_actual_speed():
    assert Node.APPROACH_WALK_DURATION_SEC['STRAIGHT'] == pytest.approx(seconds('line_forward_4'))
    assert Node.APPROACH_WALK_DURATION_SEC['BALL_APPROACH_FINE_FORWARD_5'] == pytest.approx(seconds('ball_align_fine_forward_45_5'))
    assert Node.APPROACH_WALK_DURATION_SEC['RECOVER_LEFT_TURN_LEFT_4'] == pytest.approx(seconds('line_recovery_left_4'))
    assert Node.APPROACH_WALK_DURATION_SEC['RECOVER_RIGHT_TURN_RIGHT_4'] == pytest.approx(seconds('line_recovery_right_4'))
