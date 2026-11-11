"""Adjacent motions share a continuous rest, never time spent moving."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_motion_command_bridge import FakeBridge, decoded_messages, executor_status, navigation_message
from test_transition_dwells import clock
from test_line_turn_settle_aliases import harness, decision
from test_mission_phase_flow import MissionFlowHarness, line_info


def requests(bridge):
    return decoded_messages(bridge.executor_request_publisher)


def complete(bridge, **changes):
    status = executor_status(**{**requests(bridge)[-1], 'status': 'SUCCEEDED', **changes})
    bridge.executor_status_callback(status)
    return status


@pytest.mark.parametrize('already_waited', [0., .2, .5, .8])
def test_turn_to_backward_uses_only_remaining_rest(clock, already_waited):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(navigation_message(action='TURN_LEFT', command_id=1))
    clock[0] = 11.
    complete(bridge)
    clock[0] += already_waited
    bridge.navigation_command_callback(navigation_message(action='BALL_LOST_BACKWARD_2', command_id=2))
    if already_waited < .5:
        assert len(requests(bridge)) == 1
        assert bridge.transition_dwell_until == 11.5
        clock[0] = 11.499
        bridge._check_atomic_dwell()
        assert len(requests(bridge)) == 1
        clock[0] = 11.5
        bridge._check_atomic_dwell()
    assert len(requests(bridge)) == 2
    assert requests(bridge)[-1]['motion_id'] == 'ball_lost_camera_down_backward_2'
    assert bridge.stationary_since is None


def test_backward_post_hold_covers_next_backward_pre_hold(clock):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(navigation_message(action='BALL_LOST_BACKWARD_2', command_id=1))
    clock[0] = 10.5
    bridge._check_atomic_dwell()
    clock[0] = 12.
    terminal = complete(bridge)
    clock[0] = 12.3
    bridge.executor_status_callback(terminal)
    assert bridge.stationary_since == 12.
    clock[0] = 12.5
    bridge._check_atomic_dwell()
    assert decoded_messages(bridge.motion_status_publisher)[-1]['stationary_elapsed_sec'] == .5
    bridge.navigation_command_callback(navigation_message(action='BALL_LOST_BACKWARD_2', command_id=2))
    assert len(requests(bridge)) == 2
    assert bridge.transition_dwell_until is None


def test_crab_to_backward_keeps_rest_after_actual_posture_transition(clock):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = 'pickup_crab_left_0'
    bridge.stationary_since = 9.
    bridge.motion_in_progress = True
    bridge.active_action = 'PICKUP_NOW'
    bridge.active_command_id = bridge.active_request_id = 10
    bridge.active_event_id = 8
    bridge.active_motion_id = 'pickup_pre_backward_camera_down'
    bridge._publish_executor_request(action='PICKUP_NOW', command_id=10, event_id=8,
                                    request_id=10, motion_id=bridge.active_motion_id, timeout_ms=12000)
    assert requests(bridge)[-1]['motion_id'] == 'pickup_crab_left_to_fine_0'
    assert bridge.stationary_since is None
    clock[0] = 12.
    complete(bridge)
    assert bridge.transition_dwell_until == 12.5
    clock[0] = 12.499
    bridge._check_atomic_dwell()
    assert len(requests(bridge)) == 1
    clock[0] = 12.5
    bridge._check_atomic_dwell()
    assert requests(bridge)[-1]['motion_id'] == 'pickup_pre_backward_camera_down'
    assert len(requests(bridge)) == 2
    assert bridge.transition_dwell_until is None


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED'])
def test_failed_motion_does_not_grant_rest_credit(clock, status):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(navigation_message(action='TURN_LEFT', command_id=1))
    complete(bridge, status=status, error_code='TEST_FAILURE')
    clock[0] = 20.
    bridge.navigation_command_callback(navigation_message(action='BALL_LOST_BACKWARD_2', command_id=2))
    assert len(requests(bridge)) == 1
    assert bridge.transition_dwell_until == 20.5


def test_intervening_motion_and_wrong_completion_cannot_reuse_old_rest(clock):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(navigation_message(action='TURN_LEFT', command_id=1))
    stale = complete(bridge)
    clock[0] = 12.
    bridge.navigation_command_callback(navigation_message(action='STRAIGHT', command_id=2))
    bridge.executor_status_callback(stale)
    assert bridge.stationary_since is None
    clock[0] = 15.
    complete(bridge)
    bridge.navigation_command_callback(navigation_message(action='BALL_LOST_BACKWARD_2', command_id=3))
    assert bridge.transition_dwell_until == 15.5


def owned_status(node, *, status, action='POST_SHOT_LINE_TURN_RIGHT_1', command_id=1, **extra):
    Node._motion_status_callback(node, String(data=json.dumps(dict(
        status=status, action=action, command_id=command_id, **extra))))


@pytest.mark.parametrize('elapsed', [0., .25, .5])
def test_line_post_hold_counts_toward_next_turn_pre_hold(clock, elapsed):
    node = harness()
    node.phase_manager.set_phase('POST_SHOT_LINE_ALIGN')
    node.general_motion_gate.on_command_published('POST_SHOT_LINE_TURN_RIGHT_1', 1)
    node.active_general_source = 'line'
    owned_status(node, status='RUNNING', motion_id='post_ball_line_turn_right_1')
    owned_status(node, status='SUCCEEDED', stationary_elapsed_sec=elapsed)
    next_turn = decision('POST_SHOT_LINE_TURN_LEFT_1')
    if elapsed < .5:
        assert not node._pre_motion_settle_ready(next_turn, 10.)
        assert node.pre_motion_settle_deadline == 10.5 - elapsed
    assert node._pre_motion_settle_ready(next_turn, 10.5 - elapsed)


def test_unowned_status_cannot_grant_node_rest_credit(clock):
    node = harness()
    node.general_motion_gate.on_command_published('POST_SHOT_LINE_TURN_RIGHT_1', 1)
    owned_status(node, status='SUCCEEDED', command_id=999, stationary_elapsed_sec=50.)
    assert getattr(node, 'motion_stationary_since', None) is None


def test_running_next_motion_discards_node_rest_credit(clock):
    node = harness()
    node.motion_stationary_since = 1.
    node.general_motion_gate.on_command_published('POST_SHOT_LINE_TURN_RIGHT_1', 1)
    node.active_general_source = 'line'
    owned_status(node, status='RUNNING', motion_id='post_ball_line_turn_right_1')
    assert node.motion_stationary_since is None


def test_post_shot_turn_to_turn_publishes_after_one_half_second(clock):
    node = MissionFlowHarness(phase='POST_SHOT_TURN')
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = .5
    node.phase_manager.ball_sections_processed = 1
    turn = node.publish_vision(line=line_info(heading=30.))[-1]
    node.send_status(turn['action'], turn['command_id'], 'RUNNING')
    clock[0] = 12.
    node.send_status(turn['action'], turn['command_id'], 'SUCCEEDED')
    assert node.post_shot_dwell_until == 12.5
    clock[0] = 12.499
    assert not node.publish_vision(line=line_info(heading=30.))
    clock[0] = 12.5
    node.publish_vision(line=line_info(heading=30.))
    clock[0] = 12.51
    sent = node.publish_vision(line=line_info(heading=30.))
    assert sent[-1]['action'].startswith('POST_SHOT_LINE_TURN_')
    assert sent[-1]['valid']
