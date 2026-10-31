"""Exercise directional support changes, timing and failures without hardware."""

import copy
from decimal import Decimal
import hashlib
import json
from pathlib import Path

import pytest
import yaml

from test_motion_command_bridge import (
    FakeBridge, decoded_messages, executor_status, fine_alignment_message,
    navigation_message,
)
from test_transition_dwells import clock, check_both_pauses


ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261031_catalog63_transitions'
ALIAS_PATH = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')

# Expected physical routes from the requested table, independent of implementation.
CASES = [
    *[(previous, 'BALL_APPROACH_FINE_FORWARD_5', 'pickup_fine_prepare',
       'ball_approach_fine_forward_45_5', 'rr오뒤에서 미세오뒤(45도)')
      for previous in ('line_forward_4', 'stationary_turn_left', 'stationary_turn_right',
                       'line_recovery_left_4', 'line_recovery_right_4')],
    *[(previous, action, f'fine_to_turn_ready_{camera}', target, f'rr미세오뒤에서 오뒤{camera}도')
      for previous, turn_camera, forward_camera in (('pickup_fine_forward_0', 0, 0),
                                                   ('ball_approach_fine_forward_45_5', 0, 45),
                                                   ('goal_camera_90_fine_forward_1', 90, 90))
      for action, target, camera in (('STRAIGHT', 'line_forward_6', forward_camera),
                                     ('TURN_LEFT', 'stationary_turn_left', turn_camera),
                                     ('TURN_RIGHT', 'stationary_turn_right', turn_camera))],
    ('goal_camera_90_fine_forward_1', 'GOAL_CAMERA_90_FORWARD', 'fine_to_turn_ready_90',
     'goal_camera_90_forward_6', 'rr미세오뒤에서 오뒤90도'),
    ('goal_camera_90_fine_forward_1', 'GOAL_CAMERA90_RECOVER_LEFT_4', 'fine_to_turn_ready_90',
     'line_recovery_left_4', 'rr미세오뒤에서 오뒤90도'),
]

for side, opposite, korean in (('right', 'left', '오'), ('left', 'right', '왼')):
    pickup = f'pickup_crab_{side}_0'
    goal = f'goal_camera_90_crab_{side}'
    CASES.extend([
        *[(previous, f'BALL_PICKUP_CRAB_{side.upper()}',
           f'pickup_fine_to_crab_{side}_0', pickup, f'r미세오뒤에서 {korean}옆0도')
          for previous in ('pickup_fine_forward_0', 'ball_approach_fine_forward_45_5',
                           'hurdle_fine_forward_5', 'hurdle_fine_forward_10')],
        *[(previous, f'BALL_PICKUP_CRAB_{side.upper()}',
           'pickup_forward_to_crab_0' if side == 'right' else 'pickup_forward_to_crab_left_0',
           pickup, 'r오뒤에서 오옆꽃게0도' if side == 'right' else 'r오뒤에서 왼옆0도')
          for previous in ('pickup_lost_ball_backward_1', 'ball_lost_camera_down_backward_2',
                           'line_forward_4')],
        ('goal_camera_90_fine_forward_1', f'GOAL_CAMERA90_CRAB_{side.upper()}',
         f'goal_fine_to_crab_{side}_90', goal,
         'r미세오뒤에서 오옆 90도' if side == 'right' else 'r미세오뒤에서왼옆90도'),
        *[(previous, f'GOAL_CAMERA90_CRAB_{side.upper()}', f'goal_forward_to_crab_{side}_90', goal,
           'r오뒤에서 오옆꽃게90도' if side == 'right' else 'r오뒤에서 왼옆으로90도')
          for previous in ('goal_camera_90_forward_4', 'line_recovery_left_4',
                           'line_recovery_right_4', 'goal_camera_90_backward_1',
                           'goal_camera_90_turn_left_1', 'goal_camera_90_turn_right_1')],
        (goal, 'SHOT', f'goal_crab_{side}_to_turn_90', 'goal_shot', f'r{korean}옆에서 오뒤90도'),
        (goal, 'GOAL_CAMERA90_TURN_LEFT_1', f'goal_crab_{side}_to_turn_90',
         'goal_camera_90_turn_left_1', f'r{korean}옆에서 오뒤90도'),
        (pickup, 'BALL_PICKUP_FINE_FORWARD', f'pickup_crab_{side}_to_fine_0',
         'pickup_fine_forward_0', f'r{korean}옆에서 미세오뒤(0도)'),
        (pickup, f'BALL_PICKUP_CRAB_{opposite.upper()}', f'pickup_crab_{side}_to_{opposite}_0',
         f'pickup_crab_{opposite}_0', f'r{korean}옆에서 {"왼" if opposite == "left" else "오"}옆0도'),
        (goal, f'GOAL_CAMERA90_CRAB_{opposite.upper()}', f'goal_crab_{side}_to_{opposite}_90',
         f'goal_camera_90_crab_{opposite}', f'r{korean}옆에서 {"왼" if opposite == "left" else "오"}옆90도'),
        (pickup, 'BALL_PICKUP_FINE_TURN_LEFT_1', f'pickup_crab_{side}_to_turn_0',
         'pickup_camera_down_turn_left_1', f'r{korean}옆에서 오뒤0도'),
    ])


def start_case(previous, action, **kwargs):
    bridge = FakeBridge(advance_transition_dwells=False)
    if action.startswith('BALL_PICKUP_'):
        bridge.navigation_command_callback(navigation_message(action='PICKUP_NOW'))
        bridge.pickup_initial_align_waiting = False
        bridge.pickup_fine_align_waiting = True
        bridge.active_motion_id = bridge.FINE_ALIGN_MARKER
        bridge.last_physical_motion_id = previous
        bridge.navigation_command_callback(fine_alignment_message(action))
    else:
        bridge.last_physical_motion_id = previous
        bridge.navigation_command_callback(navigation_message(action=action, **kwargs))
    return bridge


@pytest.mark.parametrize('previous,action,prepare,target,name', CASES)
def test_requested_route_and_exact_half_second_holds(clock, previous, action, prepare, target, name):
    aliases = yaml.safe_load((ROOT / ALIAS_PATH).read_text())['motion_aliases']
    assert aliases[prepare] == name
    bridge = start_case(previous, action)
    check_both_pauses(bridge, clock, prepare, target)


@pytest.mark.parametrize('previous,action,prepare,target,name', CASES)
@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED'])
def test_failed_transition_never_sends_target(clock, previous, action, prepare, target, name, status):
    bridge = start_case(previous, action)
    clock[0] += .5
    bridge._check_atomic_dwell()
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request['motion_id'] == prepare
    bridge.executor_status_callback(executor_status(**request, status=status,
                                                  error_code='SDK_COMMUNICATION_ERROR'))
    clock[0] += 10
    bridge._check_atomic_dwell()
    assert [r['motion_id'] for r in decoded_messages(bridge.executor_request_publisher)] == [prepare]
    assert not bridge.motion_in_progress


@pytest.mark.parametrize('filename', ['robot_motions_pc.json', 'robot_motions_runtime.json'])
def test_source_fidelity_speed_and_unrelated_motion_preservation(filename):
    manifest = json.loads((UPDATE / 'manifest.json').read_text())
    raw = (UPDATE / 'source63.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest() == manifest['source_sha256']
    source = {m['name']: m for m in json.loads(raw)['motions']}
    before = json.loads((UPDATE / 'before/artifacts' / filename).read_text())
    # The subsequent Hurdle camera-state update adds a separate zero-degree entry.
    snapshot = ROOT / 'artifacts/20261031_hurdle_camera_transitions/before'
    actual = json.loads((snapshot / 'artifacts' / filename).read_text())
    assert actual.keys() == before.keys()
    expected = copy.deepcopy(before)
    names = {case[4] for case in CASES}
    assert names == set(manifest['motion_names'])
    for name in manifest['motion_names']:
        motion = copy.deepcopy(source[name])
        for old, new in manifest.get('pose_renames', {}).get(name, {}).items():
            for frame in motion['frames']:
                if frame['name'] == old:
                    frame['name'] = new
            for field in ('start_pose', 'end_pose'):
                if motion[field] == old:
                    motion[field] = new
        if 'runtime' in filename:
            motion['playback_speed'] = float(Decimal(str(motion['playback_speed'])) * Decimal('.95'))
            motion['completion']['position_tolerance_deg'] = 5.0
        expected['motions'].append(motion)
    assert actual == expected
    aliases = yaml.safe_load((snapshot / ALIAS_PATH).read_text())['motion_aliases']
    old_aliases = yaml.safe_load((UPDATE / 'before' / ALIAS_PATH).read_text())['motion_aliases']
    assert aliases == {**old_aliases, **manifest['aliases']}
    assert set(aliases.values()) <= {m['name'] for m in actual['motions']}


@pytest.mark.parametrize('side', ['left', 'right'])
def test_goal_crab_to_fine_uses_new_directional_transition(side):
    bridge = FakeBridge()
    bridge.last_physical_motion_id = f'goal_camera_90_crab_{side}'
    assert bridge._transition_prepare_motion('goal_camera_90_fine_forward_1') == f'goal_crab_{side}_to_fine_90'


def test_goal_handoff_keeps_both_transition_holds(clock):
    bridge = start_case('line_forward_4', 'GOAL_CAMERA90_FINE_FORWARD_1',
                        phase='GOAL_APPROACH', source_command={'post_ball_goal_handoff': True})
    check_both_pauses(bridge, clock, 'goal_fine_prepare_90', 'goal_camera_90_fine_forward_1')


@pytest.mark.parametrize('previous,prepare', [
    ('ball_approach_fine_forward_45_5', 'fine_to_turn_ready_0'),
    ('pickup_fine_forward_0', 'fine_to_turn_ready_0'),
    ('goal_camera_90_fine_forward_1', 'fine_to_turn_ready_90'),
    ('hurdle_fine_forward_5', 'fine_to_turn_ready_0'),
    ('hurdle_fine_forward_10', 'fine_to_turn_ready_0'),
])
@pytest.mark.parametrize('head_state', [
    {}, {'goal_head_override_active': True}, {'ball_head_override_active': True},
])
def test_revised_ball_goal_turn_routes_follow_completed_mission(clock, previous, prepare, head_state):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = previous
    bridge.head_override_state = head_state
    bridge.head_override_received_at = clock[0]
    bridge.navigation_command_callback(navigation_message(action='TURN_RIGHT'))
    check_both_pauses(bridge, clock, prepare, 'stationary_turn_right')


def test_fine_to_crab_routes_are_scoped_to_the_requested_missions():
    bridge = FakeBridge()
    bridge.last_physical_motion_id = 'goal_camera_90_fine_forward_1'
    assert bridge._transition_prepare_motion('pickup_crab_left_0') is None
    for previous in ('pickup_fine_forward_0', 'hurdle_fine_forward_5'):
        bridge.last_physical_motion_id = previous
        assert bridge._transition_prepare_motion('goal_camera_90_crab_right') is None
