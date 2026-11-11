"""Replay signed repeat thresholds through planners, reservations and the bridge."""

import copy
import hashlib
import json
from pathlib import Path

import pytest
import yaml

from mission_control.crab_motion_selection import (
    crab_repeat_motion, goal_crab_motion_id, GOAL_CRAB_DURATIONS,
)
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from mission_control.motion_decision_planner import MotionDecisionPlanner
from mission_control.pickup_walk_reservation import pickup_walk_motion, PICKUP_CRAB_DURATIONS
from test_continuous_approach import clock, fresh
from test_motion_command_bridge import FakeBridge, decoded_messages, navigation_message
from test_motion_decision_planner import goal_info
from test_pickup_continuous_walk import ball, forward_status
from test_shot_start_posture import finish_sequence

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261110_catalog84_crab_thresholds'
MANIFEST = json.loads((UPDATE / 'manifest.json').read_text())
ALIAS_PATH = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
ALIASES = yaml.safe_load((ROOT / ALIAS_PATH).read_text())['motion_aliases']


def catalog(path):
    motions = json.loads(path.read_text())['motions']
    result = {m['name']: m for m in motions}
    assert len(result) == len(motions)
    return result


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_source84_left_turns_and_new_goal_left_two_keep_original_data(kind):
    source = catalog(UPDATE / 'source_robot_motions.json')
    before = catalog(UPDATE / f'before/artifacts/robot_motions_{kind}.json')
    after = catalog(ROOT / f'artifacts/20261110_catalog85_check_speed/before/artifacts/robot_motions_{kind}.json')
    assert set(after) - set(before) == set(MANIFEST['added_motions'])
    assert all(after[name] == original for name, original in before.items())
    for name in MANIFEST['left_turn_imports'] + MANIFEST['added_motions']:
        expected = copy.deepcopy(source[name])
        for f in expected['frames']:
            f['name'] = MANIFEST['pose_renames'].get(f['name'], f['name'])
        for key in ('start_pose', 'end_pose'):
            expected[key] = MANIFEST['pose_renames'].get(expected[key], expected[key])
        if kind == 'runtime':
            expected['completion']['position_tolerance_deg'] = 5.0
        assert after[name] == expected
        assert after[name]['playback_speed'] == source[name]['playback_speed']


def test_new_alias_and_crab_duration_estimates_match_real_catalog():
    assert hashlib.sha256((UPDATE / 'source_robot_motions.json').read_bytes()).hexdigest() == MANIFEST['source_sha256']
    before = yaml.safe_load((UPDATE / 'before' / ALIAS_PATH).read_text())['motion_aliases']
    after = yaml.safe_load((ROOT / 'artifacts/20261110_catalog85_check_speed/before' / ALIAS_PATH).read_text())['motion_aliases']
    assert after == {**before, 'goal_camera_90_crab_left_2': 'tt미세왼옆90도(2회)'}
    motions = catalog(ROOT / 'artifacts/robot_motions_runtime.json')
    assert set(ALIASES.values()) <= motions.keys()
    for alias, duration in {**PICKUP_CRAB_DURATIONS, **GOAL_CRAB_DURATIONS}.items():
        m = motions[ALIASES[alias]]
        actual = max(f['start_ms'] + f['time_ms'] for f in m['frames']) * m['repeat_count'] / m['playback_speed'] / 1000
        assert duration == pytest.approx(actual)
        assert actual * 1000 + m['completion']['settle_timeout_ms'] < Bridge.DEFAULT_TIMEOUT_MS
        base = motions[ALIASES[alias.removesuffix('_2')]]
        assert m['frames'] == base['frames']


@pytest.mark.parametrize('offset,side,repeats', [
    (-200, 'LEFT', 2), (-100.001, 'LEFT', 2), (-100, 'LEFT', 1), (-99.999, 'LEFT', 1),
    (199.999, 'RIGHT', 1), (200, 'RIGHT', 1), (200.001, 'RIGHT', 2),
])
@pytest.mark.parametrize('stage', ['initial', 'fine'])
def test_ball_signed_boundaries_agree_for_dispatch_and_reservation(offset, side, repeats, stage):
    planner = MotionDecisionPlanner()
    info = ball(offset_x_px=offset, bottom_distance_px=250)
    decision = (planner.plan_ball_pickup_initial_alignment(info) if stage == 'initial'
                else planner.plan_ball_pickup_fine_alignment(info))
    assert decision.valid and decision.action.endswith(f'CRAB_{side}')
    base = f'pickup_crab_{side.lower()}_0'
    expected = base + ('_2' if repeats == 2 else '')
    assert Bridge._pickup_crab_repeat_motion(base, {'source_command': decision.source_command}) == expected
    assert pickup_walk_motion(decision.action, base, decision.source_command) == expected


def goal_sample(offset, **updates):
    return goal_info(**{'depth_m': .55, 'offset_x_px': offset, 'offset_x_norm': offset / 640,
                        'bearing_deg': -7.9 if offset < 0 else 14.2, 'score_now': False,
                        **updates})


@pytest.mark.parametrize('offset,side,repeats', [
    (-120.001, 'left', 2), (-120, 'left', 1), (-119.999, 'left', 1),
    (149.999, 'right', 1), (150, 'right', 1), (150.001, 'right', 2),
])
def test_goal_signed_boundaries_reach_real_bridge(offset, side, repeats):
    decision = MotionDecisionPlanner().plan('GOAL_APPROACH', {'goal': goal_sample(offset)}, .1)
    assert decision.action == f'GOAL_CAMERA90_CRAB_{side.upper()}'
    assert decision.source_command['offset_x_px'] == offset
    bridge = FakeBridge()
    bridge.last_physical_motion_id = f'goal_camera_90_crab_{side}'
    bridge.navigation_command_callback(navigation_message(
        action=decision.action, source='goal', source_command=decision.source_command))
    finish_sequence(bridge)
    expected = f'goal_camera_90_crab_{side}' + ('_2' if repeats == 2 else '')
    assert [r['motion_id'] for r in decoded_messages(bridge.executor_request_publisher)] == [expected]
    assert bridge.goal_crab_completed and not bridge.motion_in_progress


@pytest.mark.parametrize('offset,expected', [(140, 'right'), (-110, 'left')])
def test_goal_repeat_threshold_uses_robot_dx_instead_of_shifted_shot_error(offset, expected):
    info = goal_sample(offset, depth_m=.63, camera_fx_px=4000.)
    decision = MotionDecisionPlanner().plan('GOAL_APPROACH', {'goal': info}, .1)
    assert decision.action == f'GOAL_CAMERA90_CRAB_{expected.upper()}'
    assert goal_crab_motion_id(decision.action, decision.source_command) == f'goal_camera_90_crab_{expected}'


@pytest.mark.parametrize('base,offset', [
    ('pickup_crab_left_0', 250), ('pickup_crab_right_0', -250),
    ('goal_camera_90_crab_left', 200), ('goal_camera_90_crab_right', -200),
    *[('goal_camera_90_crab_right', value) for value in (None, True, '200', float('nan'), float('inf'))],
])
def test_opposite_sign_or_invalid_pixel_does_not_authorize_two_repeats(base, offset):
    assert crab_repeat_motion(base, offset) == base


@pytest.mark.parametrize('side,offset', [('left', -100.001), ('right', 200.001)])
def test_ball_two_cycle_reservation_releases_to_fresh_checkpoint(clock, side, offset):
    from test_pickup_continuous_walk import test_same_gait_reserves_one_step_and_keeps_parent_pickup_locked
    test_same_gait_reserves_one_step_and_keeps_parent_pickup_locked(
        clock, f'pickup_crab_{side}_0_2', offset)


@pytest.mark.parametrize('side,offset', [('left', -120), ('left', -120.001), ('right', 150), ('right', 150.001)])
def test_goal_one_and_two_cycle_reservations_use_actual_running_duration(clock, monkeypatch, side, offset):
    import test_goal_crab_queue as replay
    monkeypatch.setattr(replay, 'sample', lambda side='left', **changes: goal_sample(offset, **changes))
    replay.test_same_crab_reservation_preserves_half_second_rest(clock, side)


@pytest.mark.parametrize('first_offset,next_offset', [(-120, -121), (-121, -120), (150, 151), (151, 150)])
def test_goal_repeat_change_waits_for_current_completion(clock, monkeypatch, first_offset, next_offset):
    import test_goal_crab_queue as replay
    from test_reservation_expansion import deliver
    from test_pickup_crab_observation import advance_dwell

    side = 'left' if first_offset < 0 else 'right'
    monkeypatch.setattr(replay, 'sample', lambda side='left', **changes: goal_sample(first_offset, **changes))
    node, bridge, first = replay.start(clock, side)
    replay.images(node, clock, side, info=goal_sample(next_offset))
    assert node.queued_general_command_id is None
    assert bridge.queued_request_id is None
    forward_status(node, bridge, first, 'SUCCEEDED')
    clock[0] += .6
    # Reacquire fresh images through the existing post-completion input gates.
    fresh(node, clock, 'goal', goal_sample(next_offset))
    fresh(node, clock, 'goal', goal_sample(next_offset))
    command = node.publisher.messages[-1]
    assert command['valid'] and command['action'] == first['action']
    assert command['command_id'] != first['command_id']
    deliver(node, bridge, command)
    while bridge.transition_dwell_until is not None or bridge.active_dwell_until is not None:
        advance_dwell(node, bridge, clock)
    following = decoded_messages(bridge.executor_request_publisher)[-1]
    assert following['motion_id'] == goal_crab_motion_id(first['action'], {'offset_x_px': next_offset})
    assert following['motion_id'] != first['motion_id']
    forward_status(node, bridge, following, 'RUNNING')
    assert node.goal_forward_duration_sec == GOAL_CRAB_DURATIONS[following['motion_id']]


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('next_action,prepare', [
    ('GOAL_CAMERA90_FINE_FORWARD_3', 'fine'), ('GOAL_CAMERA90_TURN_LEFT_3', 'turn'), ('SHOT', 'fine'),
])
def test_goal_two_cycle_support_reaches_next_motion_without_locking(side, next_action, prepare):
    bridge = FakeBridge()
    bridge.last_physical_motion_id = f'goal_camera_90_crab_{side}_2'
    bridge.navigation_command_callback(navigation_message(action=next_action, source='goal'))
    finish_sequence(bridge)
    requests = decoded_messages(bridge.executor_request_publisher)
    assert requests[0]['motion_id'] == f'goal_crab_{side}_to_{prepare}_90'
    assert requests[-1]['motion_id'] == Bridge.motion_id_for_action(next_action)
    assert not bridge.motion_in_progress
