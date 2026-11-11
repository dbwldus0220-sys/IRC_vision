"""Right-turn rest boundaries and locally reserved successors, without motors."""

import pytest

from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.stationary_turn_timing import right_turn_dwell_sec
from test_continuous_approach import clock
from test_mission_phase_flow import MissionFlowHarness
from test_motion_command_bridge import FakeBridge, decoded_messages, executor_status, navigation_message


@pytest.mark.parametrize('identifier,default', [
    ('RECOVER_LEFT_TURN_RIGHT_4', .5), ('line_recovery_right_4', .5),
    ('POST_SHOT_EXIT_RIGHT_5_FORWARD_4', .5), ('post_shot_exit_right_5_forward_4', .5),
    ('pickup_first_backward_turn_right', .5), ('pickup_crab_right_to_fine_0', .5),
    ('BALL_PICKUP_CRAB_RIGHT', .5), ('pickup_pre_backward_camera_down', 1.),
    ('GO', 1.), ('LINE_OFFSET_TURN_LEFT_1', .5), ('ball_align_turn_left_1', .5),
    ('LINE_OFFSET_TURN_RIGHT_1', 0.),
])
def test_other_motions_and_disabled_rest_are_preserved(identifier, default):
    assert right_turn_dwell_sec(identifier, default) == default


@pytest.mark.parametrize('action,source,phase,field', [
    ('LINE_OFFSET_TURN_RIGHT_1', 'line', 'LINE_TRACK', 'correction_post_motion_dwell_until'),
    ('ALIGN_RIGHT', 'hurdle', 'HURDLE_POSITIONING', 'hurdle_post_motion_dwell_until'),
    ('GOAL_CAMERA90_TURN_RIGHT_1', 'goal', 'GOAL_APPROACH', 'goal_post_motion_dwell_until'),
    ('BALL_APPROACH_TURN_RIGHT_1', 'ball', 'BALL_APPROACH', 'ball_post_motion_dwell_until'),
])
def test_node_right_turn_post_rest_is_point_three_without_restarting_on_duplicate(clock, action, source, phase, field):
    node = MissionFlowHarness(phase=phase)
    node.BALL_POST_MOTION_DWELL_SEC = .5
    node.general_motion_gate.on_command_published(action, 1)
    node.active_general_source = source
    node.send_status(action, 1, 'RUNNING')
    clock[0] = 12.
    node.send_status(action, 1, 'SUCCEEDED')
    assert getattr(node, field) == pytest.approx(12.3)
    clock[0] = 12.1
    node.send_status(action, 1, 'SUCCEEDED')
    assert getattr(node, field) == pytest.approx(12.3)


@pytest.mark.parametrize('action,source', [
    ('LINE_OFFSET_TURN_RIGHT_1', 'line'), ('ALIGN_RIGHT', 'hurdle'),
    ('GOAL_CAMERA90_TURN_RIGHT_1', 'goal'),
])
@pytest.mark.parametrize('next_action,required', [('STRAIGHT_1', .3), ('LINE_OFFSET_TURN_RIGHT_1', .3), ('LINE_OFFSET_TURN_LEFT_1', .5)])
def test_reserved_successor_waits_for_shared_rest_then_starts_once(clock, action, source, next_action, required):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(navigation_message(action=action, source=source, command_id=1, event_id=None,
                                                          source_command={'turn_count': 1}))
    assert not bridge.executor_request_publisher.messages
    clock[0] = 10.299
    bridge._check_atomic_dwell()
    assert not bridge.executor_request_publisher.messages
    clock[0] = 10.3
    bridge._check_atomic_dwell()
    first = decoded_messages(bridge.executor_request_publisher)[-1]
    bridge.executor_status_callback(executor_status(**first, status='RUNNING'))
    bridge.navigation_command_callback(navigation_message(action=next_action, command_id=2, event_id=None))
    assert bridge.queued_request_deferred
    assert len(bridge.executor_request_publisher.messages) == 1
    clock[0] = 12.
    terminal = executor_status(**first, status='SUCCEEDED')
    bridge.executor_status_callback(terminal)
    assert bridge.transition_dwell_until == pytest.approx(12. + required)
    clock[0] = 12.1
    bridge.executor_status_callback(terminal)
    assert bridge.transition_dwell_until == pytest.approx(12. + required)
    clock[0] = 12. + required - .001
    bridge._check_atomic_dwell()
    assert len(bridge.executor_request_publisher.messages) == 1
    clock[0] = 12. + required
    bridge._check_atomic_dwell()
    assert len(bridge.executor_request_publisher.messages) == 2
    assert decoded_messages(bridge.executor_request_publisher)[-1]['command_id'] == 2
    assert bridge.transition_dwell_until is None
    bridge._check_atomic_dwell()
    assert len(bridge.executor_request_publisher.messages) == 2


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED'])
def test_failed_right_turn_drops_reserved_successor(clock, status):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(navigation_message(action='LINE_OFFSET_TURN_RIGHT_1', command_id=1, event_id=None))
    clock[0] = 10.3
    bridge._check_atomic_dwell()
    first = decoded_messages(bridge.executor_request_publisher)[-1]
    bridge.navigation_command_callback(navigation_message(action='STRAIGHT_1', command_id=2, event_id=None))
    bridge.executor_status_callback(executor_status(**first, status=status))
    clock[0] += 10.
    bridge._check_atomic_dwell()
    assert len(bridge.executor_request_publisher.messages) == 1
    assert bridge.queued_request_id is None
