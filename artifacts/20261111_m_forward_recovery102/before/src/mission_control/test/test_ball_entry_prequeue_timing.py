"""Ball reservation deadlines, freshness diagnostics and the 0.6m handoff."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_ball_entry_line_alignment import send, status
from test_ball_entry_recent_reservation import begin, emit, prepared, tick
from test_boundary_decision_reservation import Timer
from test_continuous_approach import clock


def enable_timers(node):
    node.create_timer = lambda period, callback, **kwargs: Timer(period, callback)

    def destroy(timer):
        assert timer.cancelled

    node.destroy_timer = destroy


def fire(node, clock):
    timer = node.ball_entry_prequeue_timer
    assert timer is not None
    clock[0] = node.ball_entry_prequeue_deadline
    timer.callback()
    assert timer.cancelled
    return timer


@pytest.mark.parametrize('distance', [.600001, .61, .65, .70, 1.2])
def test_cached_reservation_accepts_distances_above_point_six(clock, distance):
    node, _, _ = prepared(clock, distance=distance)
    emit(node, clock, .69, missing=True)
    tick(node, clock)
    assert node.queued_general_action == 'STRAIGHT_2'
    assert node.publisher.messages[-1]['source_command']['ball_entry_cached_reservation']


@pytest.mark.parametrize('distance', [.60, .59])
def test_near_observation_clears_old_far_candidate_before_missing(clock, distance):
    node, _, _ = prepared(clock, distance=.65)
    enable_timers(node)
    Node._schedule_ball_entry_prequeue(node)
    timer = node.ball_entry_prequeue_timer
    emit(node, clock, .68, distance=distance)
    assert node.queued_general_command_id is None
    emit(node, clock, .69, missing=True)
    tick(node, clock)
    assert node.queued_general_command_id is None
    assert not node.ball_entry_walk_frames
    assert timer.cancelled


@pytest.mark.parametrize('missing', [False, True])
def test_deadline_reserves_without_periodic_tick_or_another_image(clock, missing):
    node, _, _ = begin(clock)
    enable_timers(node)
    for ratio in (.55, .60, .65):
        emit(node, clock, ratio)
    deadline = node.active_line_motion_started_at + node.active_line_motion_duration_sec * .70
    assert node.ball_entry_prequeue_deadline == pytest.approx(deadline)
    timer = node.ball_entry_prequeue_timer
    if missing:
        emit(node, clock, .69, missing=True)
        assert node.ball_entry_prequeue_timer is timer
    raw = dict(node.latest_info['ball'])
    fire(node, clock)
    assert node.queued_general_action == 'STRAIGHT_2'
    assert node.latest_info['ball'] == raw
    assert node.ball_entry_prequeue_timer is None
    count = len(node.publisher.messages)
    timer.callback()
    assert len(node.publisher.messages) == count


def test_verified_reobservation_reserves_on_deadline_with_raw_distance_missing(clock):
    node, _, _ = begin(clock)
    enable_timers(node)
    emit(node, clock, .53)
    for ratio in (.61, .63, .65):
        emit(node, clock, ratio, confirmed=False)
    assert node.latest_info['ball']['distance_m'] is None
    fire(node, clock)
    assert node.queued_general_action == 'STRAIGHT_2'
    assert node.latest_info['ball']['detected'] is False


@pytest.mark.parametrize('use_deadline', [False, True])
def test_deadline_uses_capture_before_a_later_tick_would_expire_it(clock, use_deadline):
    node, _, _ = begin(clock)
    enable_timers(node)
    duration = node.active_line_motion_duration_sec
    for age_at_deadline in (.43, .37, .31):
        emit(node, clock, .70 - age_at_deadline / duration,
             received_ratio=.70 - (age_at_deadline - .23) / duration)
    if use_deadline:
        fire(node, clock)
        assert node.queued_general_action == 'STRAIGHT_2'
        diagnostic = node.ball_entry_prequeue_diagnostic
        assert diagnostic['capture_to_receive_ms'] == pytest.approx(230., abs=.001)
        assert diagnostic['receive_to_evaluation_ms'] == pytest.approx(80., abs=.001)
        assert diagnostic['capture_age_ms'] == pytest.approx(310., abs=.001)
        assert diagnostic['capture_interval_ms'] == pytest.approx(60., abs=.001)
    else:
        tick(node, clock, .70 + .06 / duration)
        assert node.queued_general_command_id is None


@pytest.mark.parametrize('angle', [-25., 25.])
@pytest.mark.parametrize('missing', [False, True])
def test_recovery_deadline_uses_final_point_three_seconds(clock, angle, missing):
    node, _, _ = begin(clock, 'RECOVER_RIGHT_TURN_LEFT_4')
    enable_timers(node)
    duration = node.active_line_motion_duration_sec
    for remaining in (.43, .39, .35):
        emit(node, clock, 1 - remaining / duration, angle=angle)
    assert node.ball_entry_prequeue_deadline == pytest.approx(
        node.active_line_motion_started_at + duration - .30)
    if missing:
        emit(node, clock, 1 - .32 / duration, missing=True)
    fire(node, clock)
    assert node.queued_general_action == ('RECOVER_LEFT_TURN_LEFT_4' if angle < 0 else 'RECOVER_RIGHT_TURN_RIGHT_4')


def test_changed_action_replaces_timer_and_ignores_cancelled_callback(clock):
    node, _, _ = begin(clock, 'RECOVER_RIGHT_TURN_LEFT_4')
    enable_timers(node)
    for ratio in (.61, .64, .67):
        emit(node, clock, ratio)
    old = node.ball_entry_prequeue_timer
    for ratio in (.675, .68, .685):
        emit(node, clock, ratio, angle=25.)
    new = node.ball_entry_prequeue_timer
    assert old.cancelled and new is not old
    old.callback()
    assert node.ball_entry_prequeue_timer is new
    assert not new.cancelled
    assert node.queued_general_command_id is None


@pytest.mark.parametrize('event', ['clock', 'failure', 'completion', 'special', 'safety', 'near'])
def test_obsolete_deadline_never_dispatches(clock, event):
    node, bridge, running = prepared(clock)
    enable_timers(node)
    Node._schedule_ball_entry_prequeue(node)
    timer, deadline = node.ball_entry_prequeue_timer, node.ball_entry_prequeue_deadline
    if event == 'clock':
        Node._invalidate_clock_observations(node, round(clock[0] * 1e9))
    elif event in {'failure', 'completion'}:
        status(node, bridge, running, 'FAILED' if event == 'failure' else 'SUCCEEDED')
    elif event == 'special':
        node.phase_manager.start_special_action('PICKUP_NOW', 99)
    elif event == 'near':
        emit(node, clock, .68, distance=.6)
    else:
        node.safety_interlock.observe_executor_status(error_code='SDK_INTERNAL_ERROR',
            message='test', action='STRAIGHT', command_id=1, event_id=None)
    count = len(node.publisher.messages)
    clock[0] = deadline
    timer.callback()
    assert node.queued_general_command_id is None
    assert len(node.publisher.messages) == count


def test_queue_ack_latency_is_correlated_and_does_not_block_promotion(clock):
    node, bridge, running = prepared(clock, distance=.65)
    enable_timers(node)
    emit(node, clock, .69, missing=True)
    fire(node, clock)
    queued = node.publisher.messages[-1]
    following = send(bridge, queued)
    clock[0] += .02
    wrong = {'command_id': queued['command_id'] + 1, 'action': queued['action'], 'status': 'QUEUED'}
    Node._motion_status_callback(node, String(data=json.dumps(wrong)))
    assert 'publish_to_queued_ms' not in node.ball_entry_reservation_timing
    status(node, bridge, following, 'QUEUED')
    assert node.ball_entry_reservation_timing['publish_to_queued_ms'] == pytest.approx(20.)
    clock[0] += .01
    status(node, bridge, following, 'QUEUED')
    assert node.ball_entry_reservation_timing['publish_to_queued_ms'] == pytest.approx(20.)
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec
    status(node, bridge, running, 'SUCCEEDED')
    status(node, bridge, following, 'RUNNING')
    assert node.general_motion_gate.active_command_id == queued['command_id']
    assert 'publish_to_running_ms' in node.ball_entry_reservation_timing


def test_near_handoff_does_not_block_new_far_observations(clock):
    node, _, _ = prepared(clock, distance=.65)
    enable_timers(node)
    emit(node, clock, .68, distance=.6)
    for ratio in (.69, .695, .70):
        emit(node, clock, ratio, distance=.65)
    assert node.queued_general_action == 'STRAIGHT_2'


@pytest.mark.parametrize('distance', [.6, .59])
def test_far_candidate_cleanup_keeps_existing_fine_reservation(clock, distance):
    node, _, _ = prepared(clock, distance=.65)
    enable_timers(node)
    Node._schedule_ball_entry_prequeue(node)
    for ratio in (.68, .71, .73):
        emit(node, clock, ratio, distance=distance)
    assert node.queued_general_action == 'BALL_APPROACH_PREPARE_FINE'
    assert node.publisher.messages[-1]['source_command']['ball_fine_alignment']
    assert node.ball_entry_prequeue_timer is None


@pytest.mark.parametrize('missing', [False, True])
def test_late_timer_rejects_expired_evidence_and_fresh_frames_can_retry(clock, missing):
    node, _, _ = prepared(clock, ratios=(.63, .64, .65))
    enable_timers(node)
    if missing:
        emit(node, clock, .69, missing=True)
    Node._schedule_ball_entry_prequeue(node)
    timer = node.ball_entry_prequeue_timer
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * .78
    timer.callback()
    assert timer.cancelled and node.ball_entry_prequeue_timer is None
    assert node.queued_general_command_id is None
    for ratio in (.80, .82, .84):
        emit(node, clock, ratio)
    assert node.queued_general_action == 'STRAIGHT_2'


@pytest.mark.parametrize('guard', ['executor', 'subscriber'])
@pytest.mark.parametrize('missing', [False, True])
def test_blocked_deadline_can_retry_after_guard_release(clock, guard, missing):
    node, _, _ = prepared(clock, ratios=(.60, .63, .66))
    enable_timers(node)
    if missing:
        emit(node, clock, .69, missing=True)
    Node._schedule_ball_entry_prequeue(node)
    if guard == 'executor':
        node.executor_auto_ready = False
    else:
        node.publisher.get_subscription_count = lambda: 0
    fire(node, clock)
    assert node.queued_general_command_id is None
    node.executor_auto_ready = True
    node.publisher.get_subscription_count = lambda: 1
    tick(node, clock, .71)
    assert node.queued_general_action == 'STRAIGHT_2'


def test_image_can_reserve_before_pending_timer_callback_runs(clock):
    node, _, _ = prepared(clock)
    enable_timers(node)
    Node._schedule_ball_entry_prequeue(node)
    timer = node.ball_entry_prequeue_timer
    emit(node, clock, .71)
    assert node.queued_general_action == 'STRAIGHT_2'
    assert timer.cancelled
    count = len(node.publisher.messages)
    timer.callback()
    assert len(node.publisher.messages) == count


def test_new_miss_does_not_cancel_already_published_reservation(clock):
    node, bridge, running = prepared(clock, distance=.65)
    enable_timers(node)
    Node._schedule_ball_entry_prequeue(node)
    fire(node, clock)
    queued = node.publisher.messages[-1]
    following = send(bridge, queued)
    for ratio in (.73, .76, .80):
        emit(node, clock, ratio, missing=True)
    assert node.queued_general_command_id == queued['command_id']
    assert node.ball_entry_prequeue_timer is None
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec
    status(node, bridge, running, 'SUCCEEDED')
    status(node, bridge, following, 'RUNNING')
    assert node.general_motion_gate.active_command_id == queued['command_id']


def test_missing_interval_and_capture_age_rejections_are_distinguishable(clock):
    node, _, _ = prepared(clock, ratios=(.63, .64, .65))
    emit(node, clock, .72, missing=True)
    assert node.ball_entry_prequeue_diagnostic['reason'] == 'missing_gap_exceeded'
    diagnostic = node.ball_entry_prequeue_diagnostic
    assert diagnostic['missing_gap_ms'] > diagnostic['missing_gap_limit_ms']
    assert diagnostic['capture_age_ms'] < diagnostic['capture_age_limit_ms']
    tick(node, clock, .77)
    diagnostic = node.ball_entry_prequeue_diagnostic
    assert diagnostic['reason'] == 'last_ball_capture_expired'
    assert diagnostic['capture_age_ms'] > diagnostic['capture_age_limit_ms']
