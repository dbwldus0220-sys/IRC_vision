"""Replay accumulated fine approach through planner, reservation and bridge."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.motion_decision_planner import MotionDecisionPlanner
from mission_control.pickup_fine_progress import PickupFineProgress
from mission_control.pickup_walk_reservation import PICKUP_WALK_DURATIONS
from test_continuous_approach import clock, fresh
from test_motion_command_bridge import (
    FakeBridge, decoded_messages, complete_active_motion, continue_pickup_after_backward,
)
from test_pickup_continuous_walk import ball, start, forward_status, late_frames
from test_wait_refresh_and_fine_settle import LiveInputHarness


DURATION = PICKUP_WALK_DURATIONS['ball_general_fine_forward_8']


def observe(progress, t, bottom=220, **changes):
    stamp = 10_000_000_000 + round(t * 1e9)
    progress.observe(MotionDecisionPlanner(), ball(bottom_distance_px=bottom, **changes),
                     stamp, stamp, 10_000_000_000, DURATION, .5)


def qualify(progress):
    for t, bottom in [(0.2, 216), (.44, 216), (.8, 244), (1.1, 244), (1.3, 222)]:
        observe(progress, t, bottom)


def test_spread_hits_latch_across_final_sway():
    progress = PickupFineProgress()
    qualify(progress)
    assert progress.complete and progress.hit_count == 3
    observe(progress, 1.4, 252)
    assert progress.complete


@pytest.mark.parametrize('times', [(1.3, 1.3, 1.3), (1.3, 1.34, 1.4), (.1, .3, .5)])
def test_duplicate_short_cluster_and_early_only_do_not_complete(times):
    progress = PickupFineProgress()
    for t in times:
        observe(progress, t)
    assert not progress.complete


def test_low_fraction_of_hits_does_not_complete():
    progress = PickupFineProgress()
    for i in range(30):
        observe(progress, i * .05, 220 if i in {1, 10, 26} else 300)
    assert progress.hit_count == 3 and not progress.complete


@pytest.mark.parametrize('change', [
    {'head_down_requested': False}, {'detected': False}, {'confirmation_confirmed': False},
    {'depth_valid': False}, {'depth_age_sec': .8}, {'confidence': .01},
    {'offset_x_norm': .8}, {'distance_m': .2},
])
def test_invalid_or_changed_target_discards_credit(change):
    progress = PickupFineProgress()
    qualify(progress)
    observe(progress, 1.4, 252, **change)
    assert not progress.complete


def test_gap_discards_credit():
    progress = PickupFineProgress()
    qualify(progress)
    observe(progress, 1.81, 252)
    assert not progress.complete


@pytest.mark.parametrize('bottom,complete', [(230, True), (230.001, False)])
def test_inclusive_configured_threshold(bottom, complete):
    progress = PickupFineProgress()
    for t in (.2, .6, 1., 1.3):
        observe(progress, t, bottom)
    assert progress.complete is complete


def running_pickup(clock, index):
    node = LiveInputHarness(phase='BALL_APPROACH')
    node.phase_manager.active_ball_mode_index = index
    node.phase_manager.ball_modes_confirmed = index
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = 'pickup_fine_forward_0'
    parent, request = start(node, bridge, clock)
    assert request['motion_id'] == 'ball_general_fine_forward_8'
    return node, bridge, parent, request


def moving_observations(node, clock, *, last_bottom=252, offset=0):
    started = node.pickup_checkpoint_observation.started_at
    # The second low reading is a different capture, never a duplicated timestamp.
    for elapsed, bottom in [(.05, 216), (.283, 216), (.488, 244), (.712, 244),
                            (.786, 244), (.974, 246), (1.039, 246), (1.061, 274),
                            (1.278, 222), (1.333, 256), (1.429, 256), (1.8, last_bottom)]:
        clock[0] = started + elapsed
        fresh(node, clock, 'ball', ball(bottom_distance_px=bottom, offset_x_px=offset), plan=False)
    return started


@pytest.mark.parametrize('index', [1, 2])
def test_first_fixed_fine_counts_and_immediately_selects_retreat_then_grasp(clock, index):
    node, bridge, parent, request = running_pickup(clock, index)
    started = moving_observations(node, clock)
    assert node.pickup_checkpoint_observation.fine_progress.complete
    assert not node.planner.pickup_fine_progress_complete  # Requires physical success.
    assert not node.pickup_walk_reservation.pending
    assert bridge.pickup_queued_request is None
    before = len(node.publisher.messages)
    clock[0] = started + DURATION
    forward_status(node, bridge, request, 'SUCCEEDED')
    assert len(node.publisher.messages) == before + 1  # No new image/timer required.
    command = node.publisher.messages[-1]
    assert command['action'] == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'
    assert command['source_command']['pickup_fine_progress_complete']
    assert command['source_command']['bottom_distance_px'] == 252
    assert bridge.active_dwell_until is None
    assert node.active_special_command_id == parent['command_id']
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    # Existing one-second support settling remains; no observation pause is added.
    assert bridge.transition_dwell_until == pytest.approx(clock[0] + 1.)
    clock[0] = bridge.transition_dwell_until
    bridge._check_transition_dwell(clock[0])
    requests = decoded_messages(bridge.executor_request_publisher)
    assert [r['motion_id'] for r in requests] == [request['motion_id'], 'pickup_pre_backward_camera_down']
    assert not bridge.pickup_fine_align_waiting and bridge.pickup_fixed_sequence_started
    complete_active_motion(bridge)
    bridge._check_atomic_dwell(float('inf'))
    assert bridge.pickup_post_backward_align_waiting
    continue_pickup_after_backward(bridge, bottom_distance_px=400)
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == 'pickup_400'


@pytest.mark.parametrize('offset,bottom,action', [
    (100, 200, 'BALL_PICKUP_CRAB_RIGHT'), (-100, 200, 'BALL_PICKUP_CRAB_LEFT'),
    (100, 252, 'BALL_PICKUP_FINE_FORWARD'), (-100, 252, 'BALL_PICKUP_FINE_FORWARD'),
    (0, 80, 'WAIT'), (0, 380, 'BALL_PICKUP_FINE_FORWARD'),
])
def test_credit_preserves_alignment_and_distance_guards(clock, offset, bottom, action):
    node, bridge, _, request = running_pickup(clock, 1)
    started = moving_observations(node, clock, last_bottom=bottom, offset=offset)
    clock[0] = started + DURATION
    forward_status(node, bridge, request, 'SUCCEEDED')
    if action == 'WAIT':
        Node._publish_decision(node)
        assert node.last_selected_decision.action == 'WAIT'
        assert not any(m['action'] == 'BALL_PICKUP_FINE_BACKWARD' for m in node.publisher.messages)
        return
    assert node.publisher.messages[-1]['action'] == action


@pytest.mark.parametrize('status', ['FAILED', 'CANCELLED', 'TIMEOUT'])
def test_failure_never_commits_completion(clock, status):
    node, bridge, _, request = running_pickup(clock, 1)
    moving_observations(node, clock)
    forward_status(node, bridge, request, status, error_code='SDK_COMMUNICATION_ERROR')
    assert not node.planner.pickup_fine_progress_complete
    assert not bridge.pickup_fixed_sequence_started
    assert not node.pickup_walk_reservation.pending


@pytest.mark.parametrize('field', ['command_id', 'event_id', 'request_id'])
def test_wrong_owner_cannot_commit_credit(clock, field):
    node, bridge, _, request = running_pickup(clock, 1)
    moving_observations(node, clock)
    before = len(node.publisher.messages)
    forward_status(node, bridge, {**request, field: -99}, 'SUCCEEDED')
    assert not node.planner.pickup_fine_progress_complete
    assert len(node.publisher.messages) == before


@pytest.mark.parametrize('index', [1, 2])
def test_accepted_queue_retains_ownership_and_can_finish_next_step(clock, index):
    node, bridge, parent, request = running_pickup(clock, index)
    late_frames(node, clock)
    queued_command = node.publisher.messages[-1]
    assert queued_command['source_command']['pickup_walk_prequeue']
    bridge.navigation_command_callback(String(data=json.dumps(queued_command)))
    queued_request = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, queued_request, 'QUEUED')
    for _ in range(4):
        fresh(node, clock, 'ball', ball(bottom_distance_px=220), plan=False)
    assert node.pickup_walk_reservation.pending
    assert not node.planner.pickup_fine_progress_complete
    forward_status(node, bridge, request, 'SUCCEEDED')
    assert bridge.active_request_id == queued_request['request_id']
    forward_status(node, bridge, queued_request, 'RUNNING')
    assert not node.pickup_walk_reservation.pending
    started = moving_observations(node, clock)
    clock[0] = started + DURATION
    forward_status(node, bridge, queued_request, 'SUCCEEDED')
    command = node.publisher.messages[-1]
    assert command['action'] == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'
    assert node.active_special_command_id == parent['command_id']


def test_clock_jump_discards_credit_but_keeps_running_motion(clock):
    node, bridge, parent, request = running_pickup(clock, 1)
    moving_observations(node, clock)
    Node._invalidate_clock_observations(node, round(clock[0] * 1e9))
    assert node.pickup_checkpoint_observation.key is None
    assert not node.planner.pickup_fine_progress_complete
    assert node.active_special_command_id == parent['command_id']
    forward_status(node, bridge, request, 'SUCCEEDED')
    assert node.pickup_fine_align_waiting


@pytest.mark.parametrize('change', [
    {'detected': False, 'raw_detected': False}, {'head_down_requested': False},
    {'confirmation_confirmed': False}, {'offset_x_norm': .8},
])
def test_lost_or_changed_ball_before_success_cannot_reuse_progress(clock, change):
    node, bridge, _, request = running_pickup(clock, 1)
    moving_observations(node, clock)
    fresh(node, clock, 'ball', ball(bottom_distance_px=252, **change), plan=False)
    forward_status(node, bridge, request, 'SUCCEEDED')
    assert not node.planner.pickup_fine_progress_complete


def test_screenshot_values_are_not_counted_twice_when_capture_stamp_repeats():
    progress = PickupFineProgress()
    observe(progress, .05, 216)
    observe(progress, .05, 216)
    for t in (.4, .8, 1.1):
        observe(progress, t, 244)
    observe(progress, 1.3, 222)
    assert progress.hit_count == 2 and not progress.complete
