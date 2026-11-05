"""Shared Goal samples and posture-transition checks for the counted Line handoff."""

import json
from types import SimpleNamespace

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_wait_refresh_and_fine_settle import LiveInputHarness
from test_mission_phase_flow import line_info
from test_goal_target_steering import target_sample
from test_motion_command_bridge import FakeBridge, navigation_message, decoded_messages


@pytest.fixture
def clock(monkeypatch):
    now = [10.0]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: now[0])
    monkeypatch.setattr(Node, '_current_ros_time_ns', lambda _: round(now[0] * 1e9))
    return now


def receive(node, clock, source, info):
    Node._info_callback(node, source)(String(data=json.dumps({
        **info, 'rgb_stamp_ns': round(clock[0] * 1e9),
    })))


def goal(angle=0, **updates):
    return {**target_sample(angle), 'confirmation_confirmed': True,
            'confirmation_hits': 20, **updates}



def test_line_to_goal_fine_preparation_has_half_second_dwells(clock):
    from test_transition_dwells import check_both_pauses
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = 'line_forward_6'
    bridge.navigation_command_callback(navigation_message(action='GOAL_CAMERA90_FINE_FORWARD_3', command_id=1))
    check_both_pauses(bridge, clock, 'goal_fine_prepare_90', 'goal_camera_90_fine_forward_3')


@pytest.mark.parametrize('previous', ['line_forward_6', 'post_ball_line_turn_right_3'])
@pytest.mark.parametrize('action', [
    'GOAL_CAMERA_90_FORWARD_2', 'GOAL_CAMERA90_FINE_FORWARD_3',
    'GOAL_CAMERA90_RECOVER_RIGHT_4', 'GOAL_CAMERA90_TURN_LEFT_3',
    'GOAL_CAMERA90_CRAB_RIGHT', 'GOAL_CAMERA90_CRAB_LEFT', 'GOAL_CAMERA90_BACKWARD_1',
])
def test_first_goal_navigation_preserves_support_transition_holds(clock, previous, action):
    from test_motion_command_bridge import complete_active_motion
    from test_transition_dwells import check_both_pauses

    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = previous
    target = bridge.motion_id_for_action(action)
    prepare = bridge._transition_prepare_motion(target)
    bridge.navigation_command_callback(navigation_message(
        action=action, command_id=1, phase='GOAL_APPROACH',
        source_command={'post_ball_goal_handoff': True},
    ))
    if prepare is not None:
        check_both_pauses(bridge, clock, prepare, target)
    backward = action == 'GOAL_CAMERA90_BACKWARD_1'
    if backward:
        assert not bridge.executor_request_publisher.messages
        assert bridge.transition_dwell_until == clock[0] + .5
        clock[0] += .5
        bridge._check_atomic_dwell()
    assert bridge.executor_request_publisher.messages
    for _ in range(6):
        assert bridge.active_dwell_until is None
        assert bridge.transition_dwell_until is None
        if not bridge.motion_in_progress:
            break
        complete_active_motion(bridge)
        if backward and bridge.transition_dwell_until is not None:
            assert bridge.transition_dwell_until == clock[0] + .5
            clock[0] += .5
            bridge._check_atomic_dwell()
    assert not bridge.motion_in_progress
    statuses = [json.loads(m.data) for m in bridge.motion_status_publisher.messages]
    assert any('DWELL' in str(s.get('motion_id')) for s in statuses) == (prepare is not None or backward)
    # Every backward command, including the handoff, owns its pre-motion hold.
    bridge.navigation_command_callback(navigation_message(
        action='GOAL_CAMERA90_BACKWARD_1', command_id=2, phase='GOAL_APPROACH',
    ))
    expected_hold = 1.0 if action.startswith('GOAL_CAMERA90_FINE_FORWARD_') else .5
    assert bridge.transition_dwell_until == clock[0] + expected_hold
