"""Bounded Ball history must authorize only one far-approach continuation."""

import json

import pytest
from std_msgs.msg import String

from mission_control.ball_entry_line_alignment import BallEntryLineAlignment
from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_ball_entry_line_alignment import send, status
from test_ball_entry_reobservation import observation
from test_continuous_approach import clock
from test_line_turn_prequeue import start
from test_motion_command_bridge import decoded_messages


def begin(clock, action='STRAIGHT'):
    node, bridge, running = start(clock, action, 'AUTO')
    node.ball_entry_line_alignment = BallEntryLineAlignment(enabled=True)
    return node, bridge, running


def emit(node, clock, ratio, *, missing=False, angle=0., distance=1.2, track=17,
         received_ratio=None, confirmed=True, **changes):
    started, duration = node.active_line_motion_started_at, node.active_line_motion_duration_sec
    captured = started + duration * ratio
    clock[0] = started + duration * (ratio if received_ratio is None else received_ratio)
    info = ({'detected': False, 'raw_detected': False, 'rgb_stamp_ns': round(captured * 1e9)}
            if missing else observation([captured], confirmed=confirmed, angle=angle, distance=distance))
    info.update(confirmation_track_id=track, **changes)
    Node._info_callback(node, 'ball')(String(data=json.dumps(info)))
    return info


def prepared(clock, *, action='STRAIGHT', distance=1.2, ratios=(.55, .60, .65)):
    node, bridge, running = begin(clock, action)
    for ratio in ratios:
        emit(node, clock, ratio, distance=distance)
    assert node.approach_walk_capture_count == 3
    assert node.queued_general_command_id is None
    return node, bridge, running


def tick(node, clock, ratio=.70):
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * ratio
    Node._publish_decision(node)


@pytest.mark.parametrize('action', ['STRAIGHT', 'RECOVER_RIGHT_TURN_LEFT_4'])
def test_recent_pre_60_history_commits_on_timer_without_another_capture(clock, action):
    node, _, _ = prepared(clock, action=action)
    tick(node, clock)
    assert node.queued_general_action == 'STRAIGHT_2'
    assert node.ball_entry_prequeue_diagnostic['reason'] == 'reserved_from_visible_ball'


@pytest.mark.parametrize('action', ['STRAIGHT', 'RECOVER_RIGHT_TURN_LEFT_4'])
def test_short_miss_reserves_once_and_keeps_raw_input_unchanged(clock, action):
    node, bridge, running = prepared(clock, action=action)
    emit(node, clock, .69, missing=True)
    assert node.queued_general_command_id is None
    raw = dict(node.latest_info['ball'])
    tick(node, clock)
    queued = node.publisher.messages[-1]
    assert queued['valid'] and queued['action'] == 'STRAIGHT_2'
    assert queued['source_command']['ball_entry_cached_reservation']
    assert node.latest_info['ball'] == raw
    assert not node.ball_pickup_entry_pending
    assert node.ball_entry_prequeue_diagnostic['reason'] == 'reserved_from_recent_history'
    following = send(bridge, queued)
    assert not bridge.queued_request_deferred
    count = len(node.publisher.messages)
    tick(node, clock, .71)
    assert len(node.publisher.messages) == count
    assert len(decoded_messages(bridge.executor_request_publisher)) == 2
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec
    status(node, bridge, running, 'SUCCEEDED')
    status(node, bridge, following, 'RUNNING')
    assert node.general_motion_gate.active_command_id == queued['command_id']
    assert not node.ball_entry_walk_frames
    # Old geometry must never fund a second blind continuation.
    emit(node, clock, .71, missing=True)
    tick(node, clock, .72)
    assert node.queued_general_command_id is None
    assert len(decoded_messages(bridge.executor_request_publisher)) == 2
    assert bridge.transition_dwell_until is None


@pytest.mark.parametrize('case,reason', [
    ('long_missing', 'missing_gap_exceeded'),
    ('old_capture', 'last_ball_capture_expired'),
    ('changed_track', 'ball_track_changed'),
    ('unverified', 'last_ball_not_verified'),
])
def test_missing_fallback_is_bounded(clock, case, reason):
    node, _, _ = prepared(clock, ratios=(.63, .64, .65))
    if case == 'unverified':
        emit(node, clock, .68, confirmed=False)
    ratio = .72 if case == 'long_missing' else .69
    received = .77 if case == 'old_capture' else ratio
    emit(node, clock, ratio, missing=True, track=18 if case == 'changed_track' else 17,
         received_ratio=received)
    tick(node, clock, max(.70, received))
    assert node.queued_general_command_id is None
    assert node.ball_entry_prequeue_diagnostic['reason'] == reason


@pytest.mark.parametrize('guard', ['safety', 'executor', 'subscriber', 'pickup', 'special', 'clock'])
def test_cached_candidate_obeys_publication_guards(clock, guard):
    node, _, _ = prepared(clock)
    emit(node, clock, .69, missing=True)
    if guard == 'safety':
        node.safety_interlock.observe_executor_status(error_code='SDK_INTERNAL_ERROR',
            message='test', action='STRAIGHT', command_id=1, event_id=None)
    elif guard == 'executor':
        node.executor_auto_ready = False
    elif guard == 'subscriber':
        node.publisher.get_subscription_count = lambda: 0
    elif guard == 'pickup':
        node.ball_pickup_entry_pending = True
    elif guard == 'special':
        node.phase_manager.start_special_action('PICKUP_NOW', 99)
    else:
        Node._invalidate_clock_observations(node, round(clock[0] * 1e9))
    if guard == 'clock':
        clock[0] += .05
        Node._publish_decision(node)
    else:
        tick(node, clock)
    assert node.queued_general_command_id is None


def test_duplicate_frames_do_not_build_a_candidate(clock):
    node, _, _ = begin(clock)
    emit(node, clock, .55)
    for received in (.60, .65, .71):
        emit(node, clock, .55, received_ratio=received)
    emit(node, clock, .72, missing=True)
    assert node.queued_general_command_id is None


def test_new_direction_cannot_use_old_direction_votes(clock):
    node, _, _ = prepared(clock)
    emit(node, clock, .68, angle=25.)
    emit(node, clock, .69, missing=True)
    tick(node, clock)
    assert node.queued_general_command_id is None
    assert node.approach_walk_capture_count == 1


def test_failed_motion_cannot_dispatch_cached_candidate(clock):
    node, bridge, running = prepared(clock)
    emit(node, clock, .69, missing=True)
    status(node, bridge, running, 'FAILED')
    Node._publish_decision(node)
    assert node.queued_general_command_id is None
    assert len(decoded_messages(bridge.executor_request_publisher)) == 1
