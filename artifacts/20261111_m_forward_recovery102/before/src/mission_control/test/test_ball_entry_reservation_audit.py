"""Audit reservation retries and chained Ball entry motions through the bridge."""

import pytest

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_ball_entry_line_alignment import send, status
from test_ball_entry_recent_reservation import begin, emit, prepared, tick
from test_continuous_approach import clock
from test_motion_command_bridge import decoded_messages


@pytest.mark.parametrize('running_action', [
    'STRAIGHT', 'RECOVER_RIGHT_TURN_LEFT_4', 'RECOVER_LEFT_TURN_RIGHT_4',
])
@pytest.mark.parametrize('angle,expected', [
    (0., 'STRAIGHT_2'), (-25., 'RECOVER_LEFT_TURN_LEFT_4'),
    (25., 'RECOVER_RIGHT_TURN_RIGHT_4'),
])
@pytest.mark.parametrize('missing', [False, True])
def test_late_candidate_reserves_and_promotes_for_each_direction(
        clock, running_action, angle, expected, missing):
    node, bridge, running = begin(clock, running_action)
    duration = node.active_line_motion_duration_sec
    # Recovery chains retain their existing final-0.3s reservation window.
    for remaining in (.43, .39, .35):
        emit(node, clock, 1 - remaining / duration, angle=angle)
    if missing:
        emit(node, clock, 1 - .32 / duration, missing=True)
    tick(node, clock, 1 - .28 / duration)
    assert node.queued_general_action == expected
    queued = node.publisher.messages[-1]
    following = send(bridge, queued)
    assert not bridge.queued_request_deferred
    before = clock[0]
    status(node, bridge, running, 'SUCCEEDED')
    status(node, bridge, following, 'RUNNING')
    assert clock[0] == before
    assert node.general_motion_gate.active_command_id == queued['command_id']
    assert node.queued_general_command_id is None
    assert bridge.transition_dwell_until is None
    assert len(decoded_messages(bridge.executor_request_publisher)) == 2


@pytest.mark.parametrize('guard', ['executor', 'subscriber'])
@pytest.mark.parametrize('missing', [False, True])
def test_transient_guard_release_retries_without_new_image(clock, guard, missing):
    node, _, _ = prepared(clock, ratios=(.60, .63, .66))
    if missing:
        emit(node, clock, .69, missing=True)
    if guard == 'executor':
        node.executor_auto_ready = False
    else:
        node.publisher.get_subscription_count = lambda: 0
    tick(node, clock)
    assert node.queued_general_command_id is None
    if guard == 'executor':
        node.executor_auto_ready = True
    else:
        node.publisher.get_subscription_count = lambda: 1
    tick(node, clock, .71)
    assert node.queued_general_action == 'STRAIGHT_2'


def test_expired_history_does_not_prevent_fresh_reacquisition(clock):
    node, _, _ = prepared(clock)
    emit(node, clock, .82, missing=True)
    tick(node, clock, .83)
    assert node.queued_general_command_id is None
    for ratio in (.84, .86, .88):
        emit(node, clock, ratio)
    assert node.queued_general_action == 'STRAIGHT_2'


def test_new_motion_accepts_fresh_history_after_cached_continuation(clock):
    node, bridge, running = prepared(clock)
    emit(node, clock, .69, missing=True)
    tick(node, clock)
    following = send(bridge, node.publisher.messages[-1])
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec
    status(node, bridge, running, 'SUCCEEDED')
    status(node, bridge, following, 'RUNNING')
    for ratio in (.61, .64, .67):
        emit(node, clock, ratio)
    tick(node, clock)
    assert node.queued_general_action == 'STRAIGHT_2'
    assert node.queued_general_command_id != following['command_id']


def test_recovery_chain_keeps_existing_late_window_then_retries(clock):
    node, _, _ = begin(clock, 'RECOVER_RIGHT_TURN_LEFT_4')
    for ratio in (.61, .64, .67):
        emit(node, clock, ratio, angle=25.)
    tick(node, clock)
    assert node.queued_general_command_id is None
    assert node.ball_entry_prequeue_diagnostic['reason'] == 'recovery_chain_too_early'
    duration = node.active_line_motion_duration_sec
    for remaining in (.43, .39, .35):
        emit(node, clock, 1 - remaining / duration, angle=25.)
    emit(node, clock, 1 - .32 / duration, missing=True)
    tick(node, clock, 1 - .28 / duration)
    assert node.queued_general_action == 'RECOVER_RIGHT_TURN_RIGHT_4'


@pytest.mark.parametrize('confirmation_path', ['initial', 'reobservation'])
def test_verified_candidate_history_can_reserve_during_a_short_miss(clock, confirmation_path):
    node, _, _ = begin(clock)
    if confirmation_path == 'initial':
        emit(node, clock, .55, confirmed=False)
        emit(node, clock, .60, confirmed=False)
        emit(node, clock, .65)
    else:
        emit(node, clock, .53)
        for ratio in (.61, .63, .65):
            emit(node, clock, ratio, confirmed=False)
    assert node.approach_walk_capture_count == 3
    assert node.queued_general_command_id is None
    emit(node, clock, .69, missing=True)
    tick(node, clock)
    assert node.queued_general_action == 'STRAIGHT_2'
    assert node.ball_entry_prequeue_diagnostic['reason'] == 'reserved_from_recent_history'
