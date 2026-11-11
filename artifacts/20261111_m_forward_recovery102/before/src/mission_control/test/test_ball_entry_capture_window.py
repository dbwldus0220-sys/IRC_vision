"""Check recent Ball history and late commitment without a ROS graph."""

import pytest

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_ball_entry_line_alignment import node_with_entry, start_line, send, status
from test_ball_entry_reobservation import observation
from test_continuous_approach import clock
from test_motion_command_bridge import FakeBridge, decoded_messages
from test_wait_refresh_and_fine_settle import observe


def prepare(clock):
    node, bridge = node_with_entry(), FakeBridge(advance_transition_dwells=False)
    _, running = start_line(node, bridge, clock)
    return node, bridge, running


def capture(node, clock, ratio, *, received_ratio=None, angle=0., changes=None):
    started = node.active_line_motion_started_at
    duration = node.active_line_motion_duration_sec
    captured_at = started + duration * ratio
    info = observation([captured_at], confirmed=True, angle=angle, distance=1.2)
    info.update(changes or {})
    received_at = started + duration * (ratio if received_ratio is None else received_ratio)
    clock[0] = received_at - .01
    observe(node, clock, 'ball', info, plan=False)
    return info


@pytest.mark.parametrize('angle,action', [
    (0., 'STRAIGHT_2'), (-25., 'RECOVER_LEFT_TURN_LEFT_4'),
    (25., 'RECOVER_RIGHT_TURN_RIGHT_4'),
])
def test_collects_before_70_and_first_late_capture_reserves(clock, angle, action):
    node, bridge, running = prepare(clock)
    for ratio in (.61, .64, .67):
        capture(node, clock, ratio, angle=angle)
    assert node.approach_walk_capture_count == 3
    assert node.queued_general_command_id is None
    # The normal decision timer must also respect the commit boundary.
    Node._publish_decision(node)
    assert node.queued_general_command_id is None
    capture(node, clock, .71, angle=angle)
    queued = node.publisher.messages[-1]
    assert queued['action'] == action
    assert queued['source_command']['heading_source'] == 'ball_ground'
    assert node.queued_general_command_id == queued['command_id']
    following = send(bridge, queued)
    assert not bridge.queued_request_deferred
    boundary = clock[0]
    status(node, bridge, running, 'SUCCEEDED')
    status(node, bridge, following, 'RUNNING')
    assert clock[0] == boundary
    assert node.general_motion_gate.active_command_id == queued['command_id']
    assert len(decoded_messages(bridge.executor_request_publisher)) == 2


def test_recent_pre_60_captures_count_without_early_commit(clock):
    node, _, _ = prepare(clock)
    for ratio in (.57, .58, .59):
        capture(node, clock, ratio, received_ratio=ratio + .04)
    assert node.approach_walk_capture_count == 3
    assert node.queued_general_command_id is None
    capture(node, clock, .65)
    assert node.approach_walk_capture_count == 3
    assert node.queued_general_command_id is None


def test_recent_capture_before_70_can_commit_after_70(clock):
    node, _, _ = prepare(clock)
    for ratio in (.61, .64, .67):
        capture(node, clock, ratio)
    capture(node, clock, .69, received_ratio=.72)
    assert node.approach_walk_capture_count == 3
    assert node.queued_general_command_id is not None


def test_new_direction_after_70_cannot_use_old_direction_count(clock):
    node, _, _ = prepare(clock)
    for ratio in (.61, .64, .67):
        capture(node, clock, ratio, angle=-25.)
    for ratio in (.71, .73):
        capture(node, clock, ratio, angle=25.)
        assert node.queued_general_command_id is None
    capture(node, clock, .75, angle=25.)
    assert node.queued_general_action == 'RECOVER_RIGHT_TURN_RIGHT_4'


@pytest.mark.parametrize('changes', [
    {'depth_valid': False}, {'confirmation_confirmed': False},
    {'ground_projection_valid': False},
])
def test_invalid_late_capture_cannot_commit_early_history(clock, changes):
    node, _, _ = prepare(clock)
    for ratio in (.61, .64, .67):
        capture(node, clock, ratio)
    capture(node, clock, .71, changes=changes)
    assert node.queued_general_command_id is None


def test_duplicate_capture_cannot_complete_three_frames(clock):
    node, _, _ = prepare(clock)
    capture(node, clock, .61)
    capture(node, clock, .64)
    for received in (.66, .68):
        capture(node, clock, .64, received_ratio=received)
    assert node.approach_walk_capture_count == 2
    assert node.queued_general_command_id is None
    capture(node, clock, .71)
    assert node.queued_general_command_id is not None
