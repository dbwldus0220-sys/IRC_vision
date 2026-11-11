"""Initial confirmation may commit prepared geometry, never a raw candidate."""

import pytest

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_ball_entry_line_alignment import node_with_entry, start_line, send, status
from test_ball_entry_reobservation import observation
from test_continuous_approach import clock
from test_motion_command_bridge import FakeBridge, decoded_messages
from test_wait_refresh_and_fine_settle import observe


def start(clock):
    node, bridge = node_with_entry(), FakeBridge(advance_transition_dwells=False)
    _, running = start_line(node, bridge, clock)
    return node, bridge, running


def emit(node, clock, ratio, *, confirmed=False, track=7, angle=-25., changes=None):
    stamp = node.active_line_motion_started_at + node.active_line_motion_duration_sec * ratio
    info = observation([stamp], confirmed=confirmed, angle=angle, distance=.86)
    info.update(confirmation_track_id=track, confirmation_spatial_pending=False)
    info.update(changes or {})
    clock[0] = stamp - .01
    observe(node, clock, 'ball', info, plan=False)


@pytest.mark.parametrize('angle,action', [(0., 'STRAIGHT_2'), (-25., 'RECOVER_LEFT_TURN_LEFT_4'),
                                       (25., 'RECOVER_RIGHT_TURN_RIGHT_4')])
def test_confirmation_uses_two_prepared_candidates_and_latest_confirmed_frame(clock, angle, action):
    node, bridge, running = start(clock)
    for ratio in (.61, .64):
        emit(node, clock, ratio, angle=angle)
        assert node.queued_general_command_id is None
        assert not node.ball_approach_entry_pending
    emit(node, clock, .71, confirmed=True, angle=angle)
    queued = node.publisher.messages[-1]
    assert node.queued_general_command_id == queued['command_id']
    assert queued['action'] == action
    assert queued['source_command']['heading_source'] == 'ball_ground'
    following = send(bridge, queued)
    assert not bridge.queued_request_deferred
    status(node, bridge, running, 'SUCCEEDED')
    status(node, bridge, following, 'RUNNING')
    assert node.general_motion_gate.active_command_id == queued['command_id']
    assert len(decoded_messages(bridge.executor_request_publisher)) == 2


def test_prepared_history_never_bypasses_confirmation_or_70_percent(clock):
    node, _, _ = start(clock)
    for ratio in (.61, .64, .67):
        emit(node, clock, ratio)
    assert node.queued_general_command_id is None
    emit(node, clock, .68, confirmed=True)
    assert node.approach_walk_capture_count == 3
    assert node.queued_general_command_id is None
    Node._publish_decision(node)
    assert node.queued_general_command_id is None
    emit(node, clock, .71, confirmed=True)
    assert node.queued_general_command_id is not None


@pytest.mark.parametrize('changes', [
    {'confirmation_track_id': 8}, {'confirmation_track_id': None},
    {'bbox': [950, 240, 1000, 290]},
])
def test_changed_or_unknown_candidate_cannot_reuse_history(clock, changes):
    node, _, _ = start(clock)
    for ratio in (.61, .64):
        emit(node, clock, ratio)
    emit(node, clock, .71, confirmed=True, changes=changes)
    assert node.queued_general_command_id is None
    assert node.approach_walk_capture_count == 1


def test_changed_direction_requires_new_matching_frames(clock):
    node, _, _ = start(clock)
    for ratio in (.61, .64):
        emit(node, clock, ratio, angle=-25.)
    for ratio in (.71, .73):
        emit(node, clock, ratio, confirmed=True, angle=25.)
        assert node.queued_general_command_id is None
    emit(node, clock, .75, confirmed=True, angle=25.)
    assert node.queued_general_action == 'RECOVER_RIGHT_TURN_RIGHT_4'


@pytest.mark.parametrize('changes', [
    {'confirmation_spatial_pending': True}, {'raw_detected': False},
    {'approach_candidate': None},
])
def test_uncertain_or_missing_geometry_discards_prepared_history(clock, changes):
    node, _, _ = start(clock)
    emit(node, clock, .61)
    emit(node, clock, .64, changes=changes)
    emit(node, clock, .71, confirmed=True)
    assert node.queued_general_command_id is None


def test_pre_60_candidates_are_retained_but_expired_candidates_are_not_reused(clock):
    node, _, _ = start(clock)
    for ratio in (.55, .58):
        emit(node, clock, ratio)
    assert len(node.ball_entry_candidate_history.frames) == 2
    for ratio in (.61, .64):
        emit(node, clock, ratio)
    emit(node, clock, .84, confirmed=True)
    assert node.queued_general_command_id is None
    assert node.approach_walk_capture_count == 1


def test_prepared_geometry_does_not_bypass_safety_interlock(clock):
    node, _, _ = start(clock)
    for ratio in (.61, .64):
        emit(node, clock, ratio)
    node.safety_interlock.observe_executor_status(error_code='SDK_INTERNAL_ERROR',
        message='test', action='STRAIGHT', command_id=1, event_id=None)
    emit(node, clock, .71, confirmed=True)
    assert node.queued_general_command_id is None
