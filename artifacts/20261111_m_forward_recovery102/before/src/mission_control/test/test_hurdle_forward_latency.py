"""Exercise both hurdle approaches at the recording's low frame rate, without motors."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node

from test_continuous_approach import clock
from test_hurdle_forward2 import start, observe, duration
from test_pickup_continuous_walk import forward_status
from test_motion_command_bridge import decoded_messages


@pytest.mark.parametrize('completed_hurdles', [0, 1])
def test_five_fps_with_latency_misses_queue_but_reuses_valid_boundary_input(clock, completed_hurdles):
    node, bridge, request, started = start(clock)
    node.phase_manager.hurdles_completed = completed_hurdles
    # A dropped frame still leaves too little evidence for a reservation.
    for elapsed in (.79, .99):
        clock[0] = started + elapsed
        observe(node, clock, .53, delay=.35)
    assert node.approach_walk_capture_count == 2
    assert getattr(node, 'queued_general_command_id', None) is None
    before = len(node.publisher.messages)
    clock[0] = started + duration('line_forward_2')
    forward_status(node, bridge, request, 'SUCCEEDED')
    commands = [c for c in node.publisher.messages[before:] if c['valid']]
    assert len(commands) == 1
    assert commands[0]['action'] == 'STRAIGHT_1'
    assert getattr(node, 'correction_post_motion_dwell_until', None) is None
    assert node.hurdle_post_motion_dwell_until is None


@pytest.mark.parametrize('completed_hurdles', [0, 1])
@pytest.mark.parametrize('invalid', ['unconfirmed', 'camera', 'expired'])
def test_missing_confirmation_camera_or_freshness_blocks_queue_and_boundary_reuse(clock, completed_hurdles, invalid):
    node, bridge, request, started = start(clock)
    node.phase_manager.hurdles_completed = completed_hurdles
    changes = {'confirmation_confirmed': False} if invalid == 'unconfirmed' else (
        {'hurdle_camera_ready': False} if invalid == 'camera' else {})
    for elapsed in (.79, .99):
        clock[0] = started + elapsed
        observe(node, clock, .53, delay=.53 if invalid == 'expired' else .35, **changes)
    assert getattr(node, 'queued_general_command_id', None) is None
    before = len(node.publisher.messages)
    clock[0] = started + duration('line_forward_2')
    forward_status(node, bridge, request, 'SUCCEEDED')
    assert not any(c['valid'] for c in node.publisher.messages[before:])
    assert not node.general_motion_gate.locked
    assert getattr(node, 'correction_post_motion_dwell_until', None) is None


def reserve_slow_frames(node, clock, started, depth=.53):
    for elapsed in (.39, .59, .79, .99):
        clock[0] = started + elapsed
        observe(node, clock, depth, delay=.35)
    return node.publisher.messages[-1]


@pytest.mark.parametrize('completed_hurdles', [0, 1])
@pytest.mark.parametrize('receipts,delay', [
    ((.39, .59, .79, .99), .35),  # 5 FPS, old 50% window missed the queue.
    ((.40, .65, .90), .35),      # 4 FPS.
    ((.45, .65, .85), .25),
    ((.50, .70, .90), .35),
])
def test_low_rate_delayed_frames_reserve_one_forward2_and_chain(clock, completed_hurdles, receipts, delay):
    node, bridge, request, started = start(clock)
    node.phase_manager.hurdles_completed = completed_hurdles
    for elapsed in receipts:
        clock[0] = started + elapsed
        observe(node, clock, .53, delay=delay)
    command = node.publisher.messages[-1]
    assert node.queued_general_command_id == command['command_id']
    assert command['source_command']['hurdle_forward_prequeue']
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    second = decoded_messages(bridge.executor_request_publisher)[-1]
    assert second['motion_id'] == 'line_forward_2'
    assert not bridge.queued_request_deferred
    forward_status(node, bridge, second, 'QUEUED')
    assert node.hurdle_prequeue_block_reason == 'accepted'
    count = len(node.publisher.messages)
    node.publish()
    observe(node, clock, .53, delay=delay)
    assert len(node.publisher.messages) == count
    clock[0] = started + duration('line_forward_2')
    forward_status(node, bridge, request, 'SUCCEEDED')
    forward_status(node, bridge, second, 'RUNNING')
    assert node.general_motion_gate.active_command_id == command['command_id']
    assert node.approach_walk_capture_count == 0
    assert node.hurdle_forward_frames == []
    assert node.hurdle_post_motion_dwell_until is None
    assert bridge.transition_dwell_until is None


def test_tick_can_commit_early_evidence_without_counting_duplicate_frames(clock):
    node, _, _, started = start(clock)
    for elapsed in (.10, .35, .60):
        clock[0] = started + elapsed
        observe(node, clock, .53)
    assert node.approach_walk_capture_count == 3
    assert getattr(node, 'queued_general_command_id', None) is None
    clock[0] = started + .72
    node.publish()
    assert node.queued_general_command_id is not None
    assert node.approach_walk_capture_count == 3


def test_input_must_remain_fresh_at_expected_completion(clock):
    node, _, _, started = start(clock)
    for elapsed in (.39, .59, .79):
        clock[0] = started + elapsed
        observe(node, clock, .53, delay=.35)
    assert node.approach_walk_capture_count == 3
    assert getattr(node, 'queued_general_command_id', None) is None
    assert node.hurdle_prequeue_block_reason == 'frame_expires_before_completion'
    clock[0] = started + .99
    observe(node, clock, .53, delay=.35)
    assert node.queued_general_command_id is not None


@pytest.mark.parametrize('depths', [(.65, .59, .53), (.66, .50, .54)])
def test_closing_trend_or_rebound_cannot_queue_across_final_checkpoint(clock, depths):
    node, _, _, started = start(clock)
    for elapsed, depth in zip((.50, .70, .90), depths):
        clock[0] = started + elapsed
        observe(node, clock, depth, delay=.35)
    assert node.pending_hurdle_final_sequence is None  # Prediction never authorizes GO.
    assert getattr(node, 'queued_general_command_id', None) is None
    assert node.hurdle_prequeue_block_reason == 'final_checkpoint_before_completion'
    clock[0] = started + .96
    observe(node, clock, .44, delay=.10)
    assert node.pending_hurdle_final_sequence is not None
    assert getattr(node, 'queued_general_command_id', None) is None


@pytest.mark.parametrize('completed_hurdles', [0, 1])
def test_rejected_reservation_revalidates_same_frame_at_completion_once(clock, completed_hurdles):
    node, bridge, first, started = start(clock)
    node.phase_manager.hurdles_completed = completed_hurdles
    command = reserve_slow_frames(node, clock, started)
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    second = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, second, 'REJECTED', error_code='INCOMPATIBLE_TRANSITION')
    assert getattr(node, 'queued_general_command_id', None) is None
    assert node.general_motion_gate.locked
    before = len(node.publisher.messages)
    clock[0] = started + duration('line_forward_2')
    forward_status(node, bridge, first, 'SUCCEEDED')
    commands = [c for c in node.publisher.messages[before:] if c['valid']]
    assert len(commands) == 1
    assert commands[0]['action'] == 'STRAIGHT_1'
    assert commands[0]['command_id'] > command['command_id']
    bridge.navigation_command_callback(String(data=json.dumps(commands[0])))
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == 'line_forward_2'
    forward_status(node, bridge, first, 'SUCCEEDED')
    node.publish()
    assert len([c for c in node.publisher.messages[before:] if c['valid']]) == 1


@pytest.mark.parametrize('changes', [
    {'confirmation_confirmed': False}, {'hurdle_camera_ready': False},
    {'depth_valid': False}, {'detected': False, 'raw_detected': False},
    {'hurdle_angle_45_deg': 30.},
])
def test_early_evidence_does_not_override_invalid_latest_frame(clock, changes):
    node, _, _, started = start(clock)
    for elapsed in (.39, .59):
        clock[0] = started + elapsed
        observe(node, clock, .53, delay=.35)
    clock[0] = started + .79
    observe(node, clock, .53, delay=.35, **changes)
    clock[0] = started + .99
    observe(node, clock, .53, delay=.35)
    assert getattr(node, 'queued_general_command_id', None) is None
    assert node.approach_walk_capture_count <= 1


def test_pose_change_requires_three_frames_from_new_pose(clock):
    node, _, _, started = start(clock)
    for elapsed, pose in ((.39, '45'), (.59, '45'), (.79, '0'), (.99, '0')):
        clock[0] = started + elapsed
        observe(node, clock, .53, delay=.35, hurdle_camera_pose=pose)
    assert node.approach_walk_capture_count == 2
    assert getattr(node, 'queued_general_command_id', None) is None


def test_late_running_status_cannot_reserve_after_expected_end(clock):
    node, bridge, first, started = start(clock)
    for elapsed in (.39, .59, 1.05):
        clock[0] = started + elapsed
        observe(node, clock, .53, delay=.35)
    assert getattr(node, 'queued_general_command_id', None) is None
    assert node.hurdle_prequeue_block_reason == 'waiting_for_completion_status'
    before = len(node.publisher.messages)
    forward_status(node, bridge, first, 'SUCCEEDED')
    assert len([c for c in node.publisher.messages[before:] if c['valid']]) == 1


@pytest.mark.parametrize('rejected_after_completion', [False, True])
def test_rejected_queue_resumes_on_next_valid_frame_without_periodic_tick(clock, rejected_after_completion):
    from test_boundary_decision_reservation import BoundaryHarness

    node, bridge, first, started = start(clock)
    node.boundary_decision_enabled = True
    node.create_timer = BoundaryHarness.create_timer.__get__(node)
    node.destroy_timer = BoundaryHarness.destroy_timer.__get__(node)
    command = reserve_slow_frames(node, clock, started)
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    second = decoded_messages(bridge.executor_request_publisher)[-1]
    clock[0] = started + 1.01
    observe(node, clock, .53, hurdle_camera_ready=False)
    if not rejected_after_completion:
        forward_status(node, bridge, second, 'REJECTED', error_code='INCOMPATIBLE_TRANSITION')
    clock[0] = started + duration('line_forward_2')
    forward_status(node, bridge, first, 'SUCCEEDED')
    if rejected_after_completion:
        forward_status(node, bridge, second, 'REJECTED', error_code='INCOMPATIBLE_TRANSITION')
    before = len(node.publisher.messages)
    assert not node.general_motion_gate.locked
    clock[0] += .02
    observe(node, clock, .53)
    assert len([c for c in node.publisher.messages[before:] if c['valid']]) == 1
    assert node.publisher.messages[-1]['action'] == 'STRAIGHT_1'


def test_expired_frame_breaks_evidence_then_fresh_frames_recover(clock):
    node, _, _, started = start(clock)
    for elapsed in (.25, .45):
        clock[0] = started + elapsed
        observe(node, clock, .53, delay=.15)
    clock[0] = started + .6
    observe(node, clock, .53, delay=.51)
    assert node.approach_walk_capture_count == 0
    assert node.hurdle_forward_frames == []
    for elapsed in (.75, .85, .95):
        clock[0] = started + elapsed
        observe(node, clock, .53, delay=.15)
    assert node.queued_general_command_id is not None


def test_boundary_fallback_cannot_bypass_projected_distance_veto(clock):
    from test_boundary_decision_reservation import BoundaryHarness

    node, bridge, first, started = start(clock)
    node.boundary_decision_enabled = True
    node.create_timer = BoundaryHarness.create_timer.__get__(node)
    node.destroy_timer = BoundaryHarness.destroy_timer.__get__(node)
    for elapsed, depth in zip((.50, .70, .90), (.65, .59, .53)):
        clock[0] = started + elapsed
        observe(node, clock, depth, delay=.35)
    assert node.hurdle_prequeue_block_reason == 'final_checkpoint_before_completion'
    before = len(node.publisher.messages)
    clock[0] = started + duration('line_forward_2')
    forward_status(node, bridge, first, 'SUCCEEDED')
    node.publish()
    assert not any(c['valid'] for c in node.publisher.messages[before:])
    assert node.publisher.messages[-1]['reason'] == 'waiting_for_hurdle_distance_recheck'
    clock[0] += .02
    observe(node, clock, .52)
    assert len([c for c in node.publisher.messages[before:] if c['valid']]) == 1
    assert node.publisher.messages[-1]['action'] == 'STRAIGHT_1'
