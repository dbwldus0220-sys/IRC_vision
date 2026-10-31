"""Backward commands own one half-second hold on each side of execution."""
import copy
from decimal import Decimal
import json
from pathlib import Path

import pytest
import yaml

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from mission_control.motion_decision_node import MotionDecisionNode as Decision
from test_motion_command_bridge import FakeBridge, decoded_messages, executor_status, navigation_message
from test_transition_dwells import clock

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261031_backward_dwell_goal_entry'
BACKWARDS = (
    'pickup_pre_backward_camera_down', 'pickup_lost_ball_backward_1',
    'ball_lost_camera_down_backward_2', 'pickup_retreat_2', 'goal_camera_90_backward_1',
)


def direct_backward(motion_id):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.motion_in_progress = True
    bridge.active_action = 'PICKUP_NOW'
    bridge.active_command_id = bridge.active_request_id = 10
    bridge.active_event_id = 8
    bridge.active_motion_id = motion_id
    bridge._publish_executor_request(action='PICKUP_NOW', command_id=10, event_id=8,
                                    request_id=10, motion_id=motion_id, timeout_ms=12000)
    return bridge


def finish(bridge, status='SUCCEEDED'):
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    bridge.executor_status_callback(executor_status(**request, status=status,
                                                    error_code='' if status == 'SUCCEEDED'
                                                    else 'TEST_FAILURE'))
    return request


@pytest.mark.parametrize('motion_id', BACKWARDS)
def test_all_backward_aliases_hold_exactly_half_second_on_each_side(clock, motion_id):
    bridge = direct_backward(motion_id)
    assert not bridge.executor_request_publisher.messages
    assert bridge.transition_dwell_until == 10.5
    # Even a matching terminal status cannot finish an unsent backward command.
    bridge.executor_status_callback(executor_status(command_id=10, request_id=10,
                                                    motion_id=motion_id, status='SUCCEEDED'))
    clock[0] = 10.499
    bridge._check_atomic_dwell()
    assert not bridge.executor_request_publisher.messages
    clock[0] = 10.5
    bridge._check_atomic_dwell()
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == motion_id
    clock[0] = 12.0
    request = finish(bridge)
    assert bridge.motion_in_progress
    assert bridge.transition_dwell_until == 12.5
    assert decoded_messages(bridge.motion_status_publisher)[-1]['motion_id'] == '__BACKWARD_POST_DWELL__'
    clock[0] = 12.499
    bridge.executor_status_callback(executor_status(**request, status='SUCCEEDED'))
    bridge._check_atomic_dwell()
    assert bridge.motion_in_progress
    assert bridge.transition_dwell_until == 12.5
    clock[0] = 12.5
    bridge._check_atomic_dwell()
    assert not bridge.motion_in_progress
    assert decoded_messages(bridge.motion_status_publisher)[-1]['status'] == 'SUCCEEDED'
    assert len(bridge.executor_request_publisher.messages) == 1


@pytest.mark.parametrize('motion_id', BACKWARDS)
@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'REJECTED', 'CANCELLED'])
def test_failed_backward_does_not_advance_pickup_tail(clock, motion_id, status):
    bridge = direct_backward(motion_id)
    bridge.active_pickup_sequence = (motion_id, bridge.DWELL_MARKER, 'pickup')
    clock[0] = 10.5
    bridge._check_atomic_dwell()
    finish(bridge, status)
    clock[0] = 100.0
    bridge._check_atomic_dwell()
    assert not bridge.motion_in_progress
    assert [m['motion_id'] for m in decoded_messages(bridge.executor_request_publisher)] == [motion_id]


@pytest.mark.parametrize('action', ['BALL_LOST_BACKWARD_2', 'HURDLE_LOST_BACKWARD_1'])
def test_following_forward_is_deferred_until_backward_post_hold_finishes(clock, action):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(navigation_message(action=action, command_id=1))
    bridge.navigation_command_callback(navigation_message(action='STRAIGHT', command_id=2))
    assert bridge.queued_request_deferred
    assert not bridge.executor_request_publisher.messages
    clock[0] = 10.5
    bridge._check_atomic_dwell()
    clock[0] = 12.
    finish(bridge)
    clock[0] = 12.499
    bridge._check_atomic_dwell()
    assert len(bridge.executor_request_publisher.messages) == 1
    clock[0] = 12.5
    bridge._check_atomic_dwell()
    assert [m['motion_id'] for m in decoded_messages(bridge.executor_request_publisher)] == [
        Bridge.ACTION_TO_MOTION_ID[action], 'line_forward_6']


@pytest.mark.parametrize('action', ['BALL_LOST_BACKWARD_2', 'HURDLE_LOST_BACKWARD_1'])
def test_backward_pre_hold_starts_only_after_previous_forward_finishes(clock, action):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(navigation_message(action='STRAIGHT', command_id=1))
    bridge.navigation_command_callback(navigation_message(action=action, command_id=2))
    assert bridge.queued_request_deferred
    assert bridge.transition_dwell_until is None
    clock[0] = 12.
    finish(bridge)
    assert bridge.transition_dwell_until == 12.5
    assert len(bridge.executor_request_publisher.messages) == 1
    clock[0] = 12.5
    bridge._check_atomic_dwell()
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == Bridge.ACTION_TO_MOTION_ID[action]


def test_adjacent_pickup_markers_do_not_double_backward_pauses(clock):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.motion_in_progress = True
    bridge.active_action = 'PICKUP_NOW'
    bridge.active_command_id = bridge.active_request_id = 10
    bridge.active_motion_id = bridge.last_physical_motion_id = 'pickup_fine_forward_0'
    bridge.active_pickup_sequence = (
        'pickup_fine_forward_0', bridge.DWELL_MARKER, 'pickup_pre_backward_camera_down',
        bridge.DWELL_MARKER, bridge.POST_BACKWARD_ALIGN_MARKER,
    )
    assert bridge._start_next_pickup_motion()
    assert bridge.transition_dwell_until == 11.0
    assert bridge.active_dwell_until is None
    clock[0] = 11.0
    bridge._check_atomic_dwell()
    clock[0] = 12.
    finish(bridge)
    clock[0] = 12.5
    bridge._check_atomic_dwell()
    assert bridge.pickup_post_backward_align_waiting
    assert bridge.active_dwell_until is None
    assert bridge.transition_dwell_until is None


@pytest.mark.parametrize('action', ['BALL_LOST_BACKWARD_2', 'HURDLE_LOST_BACKWARD_1', 'GOAL_CAMERA90_BACKWARD_1'])
def test_decision_node_does_not_add_another_post_backward_hold(action):
    assert not Decision._needs_correction_dwell(action)


@pytest.mark.parametrize('filename', ['robot_motions_pc.json', 'robot_motions_runtime.json'])
def test_goal_entry_replaces_only_requested_source_motion(filename):
    before = json.loads((UPDATE / 'before/artifacts' / filename).read_text())
    expected = copy.deepcopy(before)
    motion = json.loads((UPDATE / 'source_requested_motion.json').read_text())
    if 'runtime' in filename:
        motion['playback_speed'] = float(Decimal(str(motion['playback_speed'])) * Decimal('.95'))
        motion['completion']['position_tolerance_deg'] = 5.0
    expected['motions'] = [motion if m['name'] == 'rr오뒤에서 미세오뒤(골대용90도)' else m
                           for m in expected['motions']]
    assert json.loads((ROOT / 'artifacts' / filename).read_text()) == expected
    path = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
    aliases = yaml.safe_load((ROOT / path).read_text())['motion_aliases']
    old = yaml.safe_load((UPDATE / 'before' / path).read_text())['motion_aliases']
    assert aliases == {**old, 'goal_fine_prepare_90': 'rr오뒤에서 미세오뒤90도'}


def test_recoverable_pickup_fault_keeps_existing_policy_after_half_second_hold(clock):
    bridge = direct_backward('pickup_pre_backward_camera_down')
    bridge.active_pickup_sequence = (
        'pickup_pre_backward_camera_down', bridge.DWELL_MARKER, 'pickup_400',
    )
    clock[0] = 10.5
    bridge._check_atomic_dwell()
    clock[0] = 12.
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    failure = executor_status(**request, status='FAILED', error_code='SDK_COMMUNICATION_ERROR')
    bridge.executor_status_callback(failure)
    assert bridge.transition_dwell_until == 12.5
    clock[0] = 12.499
    bridge.executor_status_callback(failure)
    bridge._check_atomic_dwell()
    assert len(bridge.executor_request_publisher.messages) == 1
    clock[0] = 12.5
    bridge._check_atomic_dwell()
    assert bridge.pickup_consecutive_motor_failures == 1
    assert bridge.last_physical_motion_id is None
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == 'pickup_400'
    assert bridge.active_dwell_until is None


@pytest.mark.parametrize('source,phase,action', [
    ('ball', 'BALL_APPROACH', 'BALL_LOST_BACKWARD_2'),
    ('hurdle', 'HURDLE_POSITIONING', 'HURDLE_LOST_BACKWARD_1'),
    ('goal', 'GOAL_APPROACH', 'GOAL_CAMERA90_BACKWARD_1'),
])
def test_backward_success_reobserves_without_second_decision_hold(clock, source, phase, action):
    from test_mission_phase_flow import MissionFlowHarness

    node = MissionFlowHarness(phase=phase)
    node.active_general_source = source
    node.latest_info[source] = {'detected': True}
    node.latest_time[source] = clock[0]
    node.general_motion_gate.on_new_vision_input()
    node.general_motion_gate.on_command_published(action, command_id=1)
    node.send_status(action, 1, 'RUNNING')
    # The bridge forwards terminal success only after its physical post hold.
    clock[0] = 12.5
    node.send_status(action, 1, 'SUCCEEDED')
    assert getattr(node, 'correction_post_motion_dwell_until', None) is None
    assert getattr(node, f'{source}_post_motion_dwell_until', None) is None
    assert node.latest_info[source] is None
    assert not node.general_motion_gate.has_required_fresh_vision()


def test_grasp_observation_still_lasts_three_seconds_before_backward_pre_hold(clock):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.motion_in_progress = True
    bridge.active_action = 'PICKUP_NOW'
    bridge.active_command_id = bridge.active_request_id = 10
    bridge.active_motion_id = bridge.PICKUP_GRASP_CHECK_MOTION_ID
    bridge.active_pickup_sequence = (
        bridge.PICKUP_GRASP_CHECK_MOTION_ID, bridge.DWELL_MARKER, 'pickup_retreat_2',
    )
    assert bridge._start_next_pickup_motion()
    assert bridge.active_dwell_until == 13.
    assert bridge.transition_dwell_until is None
