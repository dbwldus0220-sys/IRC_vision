"""Revised Ball/Goal entry routes, real pause boundaries and source fidelity."""
import copy
from decimal import Decimal
import json
from pathlib import Path

import pytest
import yaml

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from mission_control.motion_decision_node import MotionDecisionNode as Decision
from test_catalog63_transitions import start_case
from test_motion_command_bridge import decoded_messages, executor_status
from test_transition_dwells import clock, check_both_pauses

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261031_transition_table_revision'
CASES = [
    (previous, f'GOAL_CAMERA90_FINE_FORWARD_{count}', 'goal_fine_prepare_90',
     f'goal_camera_90_fine_forward_{count}')
    for previous in ('goal_camera_90_forward_6', 'line_recovery_left_4', 'line_recovery_right_4',
                     'goal_camera_90_backward_1', 'goal_camera_90_turn_left_2',
                     'goal_camera_90_turn_right_3', 'line_forward_4', 'stationary_turn_left')
    for count in range(1, 5)
] + [
    (f'goal_camera_90_crab_{side}', f'GOAL_CAMERA90_FINE_FORWARD_{count}',
     f'goal_crab_{side}_to_fine_90', f'goal_camera_90_fine_forward_{count}')
    for side in ('left', 'right') for count in range(1, 5)
] + [
    (previous, action, 'pickup_fine_prepare_0', target)
    for previous in ('pickup_camera_down_turn_left_1', 'pickup_camera_down_turn_right_3',
                     'post_ball_line_turn_left_2', 'post_ball_line_turn_right_3',
                     'pickup_lost_ball_backward_1', 'ball_lost_camera_down_backward_2')
    for action, target in (('BALL_APPROACH_FINE_FORWARD_5', 'ball_approach_fine_forward_45_5'),
                           ('BALL_FINE_FORWARD_8', 'ball_general_fine_forward_8'),
                           ('BALL_PICKUP_FINE_FORWARD', 'pickup_fine_forward_0'))
]


@pytest.mark.parametrize('previous,action,prepare,target', CASES)
def test_new_routes_hold_exactly_half_second_before_and_after(clock, previous, action, prepare, target):
    bridge = start_case(previous, action)
    check_both_pauses(bridge, clock, prepare, target)


@pytest.mark.parametrize('previous,action,prepare,target', CASES)
@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED'])
def test_new_transition_failure_blocks_target(clock, previous, action, prepare, target, status):
    bridge = start_case(previous, action)
    clock[0] += .5
    bridge._check_atomic_dwell()
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request['motion_id'] == prepare
    bridge.executor_status_callback(executor_status(**request, status=status,
                                                  error_code='SDK_COMMUNICATION_ERROR'))
    clock[0] += 10
    bridge._check_atomic_dwell()
    assert decoded_messages(bridge.executor_request_publisher) == [request]
    assert not bridge.motion_in_progress


@pytest.mark.parametrize('filename', ['robot_motions_pc.json', 'robot_motions_runtime.json'])
def test_only_requested_data_added_and_speed_scaled_once(filename):
    before = json.loads((UPDATE / 'before/artifacts' / filename).read_text())
    snapshot = ROOT / 'artifacts/20261031_backward_dwell_goal_entry/before'
    actual = json.loads((snapshot / 'artifacts' / filename).read_text())
    expected = copy.deepcopy(before)
    source = json.loads((UPDATE / 'source_requested_motions.json').read_text())
    for motion in source:
        if 'runtime' in filename:
            motion['playback_speed'] = float(Decimal(str(motion['playback_speed'])) * Decimal('.95'))
            motion['completion']['position_tolerance_deg'] = 5.0
        if motion['name'] not in {m['name'] for m in before['motions']}:
            expected['motions'].append(motion)
        assert next(m for m in actual['motions'] if m['name'] == motion['name']) == motion
    assert actual == expected
    path = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
    aliases = yaml.safe_load((snapshot / path).read_text())['motion_aliases']
    old_aliases = yaml.safe_load((UPDATE / 'before' / path).read_text())['motion_aliases']
    manifest = json.loads((UPDATE / 'manifest.json').read_text())
    assert aliases == {**old_aliases, **manifest['aliases']}
    assert set(aliases.values()) <= {m['name'] for m in actual['motions']}


def test_all_intentional_one_second_holds_are_now_half_second():
    manifest = json.loads((UPDATE / 'manifest.json').read_text())
    for path, names in manifest['waits_changed_to_half_second'].items():
        cls = Bridge if 'bridge' in path else Decision
        for name in names:
            assert getattr(cls, "BACKWARD_DWELL_SEC" if name == "GOAL_BACKWARD_PRE_DWELL_SEC" else name) == .5
    assert Bridge.PICKUP_GRASP_DWELL_SEC == 3.0
