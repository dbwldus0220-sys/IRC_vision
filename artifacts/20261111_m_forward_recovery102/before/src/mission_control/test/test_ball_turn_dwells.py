"""Verify physical rests around Ball turns without moving robot motors."""

import pytest

from test_motion_command_bridge import (
    FakeBridge, decoded_messages, executor_status, navigation_message,
)
from test_wait_refresh_and_fine_settle import clock


def fine_command(action, command_id):
    return navigation_message(
        action=action, command_id=command_id, event_id=None,
        source='ball', phase='BALL_APPROACH',
        source_command={'ball_fine_alignment': True},
    )


@pytest.mark.parametrize('direction,count', [('LEFT', 2), ('RIGHT', 3)])
@pytest.mark.parametrize('rest', [0., .2, .5, .8])
def test_turn_pre_dwell_counts_only_completed_stationary_time(clock, direction, count, rest):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(navigation_message(command_id=1, event_id=None))
    walk = decoded_messages(bridge.executor_request_publisher)[-1]
    bridge.executor_status_callback(executor_status(**walk, status='SUCCEEDED'))
    clock[0] += rest
    bridge.navigation_command_callback(fine_command(f'BALL_APPROACH_TURN_{direction}_{count}', 2))
    dwell = .3 if direction == 'RIGHT' else .5
    deadline = max(10. + dwell, clock[0])
    if rest < dwell:
        assert bridge.transition_dwell_until == pytest.approx(deadline)
        clock[0] = deadline - .001
        bridge._check_atomic_dwell()
        assert len(decoded_messages(bridge.executor_request_publisher)) == 1
    clock[0] = deadline
    bridge._check_atomic_dwell()
    assert len(decoded_messages(bridge.executor_request_publisher)) == 2
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == (
        f'ball_align_turn_{direction.lower()}_{count}')


@pytest.mark.parametrize('following', ['BALL_APPROACH_PREPARE_FINE', 'BALL_APPROACH_TURN_RIGHT_3'])
@pytest.mark.parametrize('direction,dwell', [('LEFT', .5), ('RIGHT', .3)])
def test_reserved_successor_waits_after_turn_without_double_rest(clock, following, direction, dwell):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(fine_command(f'BALL_APPROACH_TURN_{direction}_2', 1))
    assert not bridge.executor_request_publisher.messages
    clock[0] = 10. + dwell
    bridge._check_atomic_dwell()
    turn = decoded_messages(bridge.executor_request_publisher)[-1]
    bridge.executor_status_callback(executor_status(**turn, status='RUNNING'))
    clock[0] = 11.
    bridge.navigation_command_callback(fine_command(following, 2))
    assert bridge.queued_request_deferred
    assert len(bridge.executor_request_publisher.messages) == 1
    clock[0] = 11.2
    bridge.executor_status_callback(executor_status(**turn, status='SUCCEEDED'))
    assert bridge.transition_dwell_until == pytest.approx(11.2 + dwell)
    # Duplicate completion must neither shorten nor restart the rest.
    clock[0] = 11.4
    bridge.executor_status_callback(executor_status(**turn, status='SUCCEEDED'))
    assert bridge.transition_dwell_until == pytest.approx(11.2 + dwell)
    clock[0] = 11.2 + dwell - .001
    bridge._check_atomic_dwell()
    assert len(bridge.executor_request_publisher.messages) == 1
    clock[0] = 11.2 + dwell
    bridge._check_atomic_dwell()
    assert len(bridge.executor_request_publisher.messages) == 2
    assert bridge.transition_dwell_until is None
    assert bridge.active_command_id == 2


@pytest.mark.parametrize('result', ['FAILED', 'TIMEOUT', 'CANCELLED'])
def test_failed_predecessor_discards_reserved_turn(clock, result):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(navigation_message(command_id=1, event_id=None))
    walk = decoded_messages(bridge.executor_request_publisher)[-1]
    bridge.navigation_command_callback(fine_command('BALL_APPROACH_TURN_LEFT_2', 2))
    assert bridge.queued_request_deferred
    bridge.executor_status_callback(executor_status(**walk, status=result))
    clock[0] += 10.
    bridge._check_atomic_dwell()
    assert len(bridge.executor_request_publisher.messages) == 1
    assert bridge.queued_request_id is None
    assert bridge.transition_dwell_until is None


@pytest.mark.parametrize('result', ['FAILED', 'TIMEOUT', 'CANCELLED'])
def test_failed_turn_never_runs_its_reserved_successor(clock, result):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(fine_command('BALL_APPROACH_TURN_LEFT_2', 1))
    clock[0] = 10.5
    bridge._check_atomic_dwell()
    turn = decoded_messages(bridge.executor_request_publisher)[-1]
    bridge.navigation_command_callback(fine_command('BALL_APPROACH_PREPARE_FINE', 2))
    bridge.executor_status_callback(executor_status(**turn, status=result))
    clock[0] += 10.
    bridge._check_atomic_dwell()
    assert len(bridge.executor_request_publisher.messages) == 1
    assert not bridge.motion_in_progress
    assert bridge.queued_request_id is None


@pytest.mark.parametrize('direction', ['LEFT', 'RIGHT'])
def test_pickup_turn_adds_pre_rest_and_keeps_one_checkpoint_post_rest(clock, direction):
    dwell = .3 if direction == 'RIGHT' else .5
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(navigation_message(action='PICKUP_NOW'))
    bridge.navigation_command_callback(navigation_message(
        action=f'BALL_PICKUP_CAMERA_DOWN_TURN_{direction}_2', command_id=8001,
        event_id=None, active_special_command_id=8000, active_special_event_id=8,
    ))
    assert not bridge.executor_request_publisher.messages
    assert bridge.transition_dwell_until == pytest.approx(10. + dwell)
    clock[0] = 10. + dwell - .001
    bridge._check_atomic_dwell()
    assert not bridge.executor_request_publisher.messages
    clock[0] = 10. + dwell
    bridge._check_atomic_dwell()
    turn = decoded_messages(bridge.executor_request_publisher)[-1]
    clock[0] = 11.2
    bridge.executor_status_callback(executor_status(**turn, status='SUCCEEDED'))
    assert bridge.transition_dwell_until is None
    assert bridge.active_dwell_until == pytest.approx(11.2 + dwell)
    clock[0] = 11.2 + dwell - .001
    bridge._check_atomic_dwell()
    assert not bridge.pickup_initial_align_waiting
    clock[0] = 11.2 + dwell
    bridge._check_atomic_dwell()
    assert bridge.pickup_initial_align_waiting


@pytest.mark.parametrize('action', ['BALL_APPROACH_PREPARE_FINE', 'BALL_APPROACH_PREPARE_TURN'])
def test_explicit_ball_preparation_keeps_immediate_dispatch(clock, action):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(fine_command(action, 1))
    assert len(bridge.executor_request_publisher.messages) == 1
    assert bridge.transition_dwell_until is None


def test_ball_fine_alignment_alias_preserves_half_second_rest(clock):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(fine_command('BALL_APPROACH_FINE_FORWARD_5', 1))
    assert not bridge.executor_request_publisher.messages
    clock[0] += .499
    bridge._check_atomic_dwell()
    assert not bridge.executor_request_publisher.messages
    clock[0] += .001
    bridge._check_atomic_dwell()
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == 'ball_align_fine_forward_45_5'


@pytest.mark.parametrize('direction', ['LEFT', 'RIGHT'])
def test_normal_approach_turn_adds_pre_rest_and_leaves_post_rest_to_node(clock, direction):
    dwell = .3 if direction == 'RIGHT' else .5
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(navigation_message(
        action=f'BALL_APPROACH_TURN_{direction}_2', source='ball', phase='BALL_APPROACH',
    ))
    assert not bridge.executor_request_publisher.messages
    assert bridge.transition_dwell_until == pytest.approx(10. + dwell)
    clock[0] = 10. + dwell
    bridge._check_atomic_dwell()
    turn = decoded_messages(bridge.executor_request_publisher)[-1]
    assert turn['motion_id'] == f'post_ball_line_turn_{direction.lower()}_2'
    clock[0] = 11.2
    bridge.executor_status_callback(executor_status(**turn, status='SUCCEEDED'))
    assert bridge.transition_dwell_until is None
    assert decoded_messages(bridge.motion_status_publisher)[-1]['status'] == 'SUCCEEDED'
