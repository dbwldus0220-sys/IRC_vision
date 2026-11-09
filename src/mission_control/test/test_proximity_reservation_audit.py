"""Exercise live proximity gating across queue and physical completion boundaries."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.pickup_proximity import PickupProximity
from mission_control.pickup_walk_reservation import PICKUP_WALK_DURATIONS
from test_continuous_approach import clock, fresh, start_goal
from test_goal_target_steering import target_sample
from test_motion_command_bridge import FakeBridge, decoded_messages
from test_pickup_continuous_walk import ball, start, forward_status
from test_pickup_proximity import heartbeat, ready, frame, fine
from test_wait_refresh_and_fine_settle import LiveInputHarness


def head(node, clock, *, complete=True, head_id=1, stamp=9_900_000_000):
    Node._executor_heartbeat_callback(node, String(data=json.dumps(
        heartbeat(complete, head_id, stamp))))


def pickup(clock, motion, offset=0):
    node = LiveInputHarness(phase='BALL_APPROACH')
    node.pickup_proximity = PickupProximity()
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = motion
    head(node, clock)
    parent, first = start(node, bridge, clock, ball(offset_x_px=offset))
    return node, bridge, parent, first


@pytest.mark.parametrize('motion,offset', [
    ('ball_general_fine_forward_8', 0),
    ('pickup_crab_left_0', -100), ('pickup_crab_right_0', 100),
])
def test_live_head_guard_keeps_same_gait_reservation_and_promotion(clock, motion, offset):
    node, bridge, parent, first = pickup(clock, motion, offset)
    clock[0] = node.pickup_walk_reservation.started_at + PICKUP_WALK_DURATIONS[motion] * .75
    for _ in range(3):
        head(node, clock)
        fresh(node, clock, 'ball', ball(offset_x_px=offset), plan=False)
    queued = node.publisher.messages[-1]
    assert queued['source_command']['pickup_walk_prequeue']
    bridge.navigation_command_callback(String(data=json.dumps(queued)))
    second = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, second, 'QUEUED')
    forward_status(node, bridge, first, 'SUCCEEDED')
    forward_status(node, bridge, second, 'RUNNING')
    assert bridge.active_request_id == second['request_id']
    assert node.active_special_command_id == parent['command_id']
    assert not node.pickup_walk_reservation.pending
    assert bridge.transition_dwell_until is None


def test_head_transition_blocks_reservation_then_new_frames_release_it(clock):
    node, bridge, _, _ = pickup(clock, 'pickup_crab_right_0', 100)
    reservation = node.pickup_walk_reservation
    clock[0] = reservation.started_at + PICKUP_WALK_DURATIONS[reservation.motion_id] * .75
    head(node, clock, complete=False, head_id=2)
    for _ in range(3):
        fresh(node, clock, 'ball', ball(offset_x_px=100), plan=False)
    assert not reservation.pending and reservation.ready is None
    head(node, clock, head_id=2, stamp=round(clock[0] * 1e9))
    for _ in range(3):
        fresh(node, clock, 'ball', ball(offset_x_px=100), plan=False)
    assert reservation.pending
    assert node.publisher.messages[-1]['source_command']['pickup_walk_prequeue']


def test_moving_observations_survive_checkpoint_with_head_guard(clock):
    node, bridge, _, first = pickup(clock, 'ball_general_fine_forward_8')
    started = node.pickup_walk_reservation.started_at
    duration = PICKUP_WALK_DURATIONS[first['motion_id']]
    for remaining in (.15, .10, .05):
        clock[0] = started + duration - remaining
        head(node, clock)
        fresh(node, clock, 'ball', ball(bottom_distance_px=220), plan=False)
    assert node.pickup_checkpoint_observation.sample is not None
    before = len(node.publisher.messages)
    stamp = node.latest_info['ball']['rgb_stamp_ns']
    clock[0] = started + duration
    head(node, clock)
    forward_status(node, bridge, first, 'SUCCEEDED')
    assert len(node.publisher.messages) == before + 1
    assert node.publisher.messages[-1]['action'] == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'
    assert node.latest_info['ball']['rgb_stamp_ns'] == stamp


@pytest.mark.parametrize('index', [1, 2])
def test_rejected_pickup_queue_recovers_at_fresh_checkpoint(clock, index):
    node, bridge, parent, first = pickup(clock, 'ball_general_fine_forward_8')
    node.phase_manager.active_ball_mode_index = index
    node.phase_manager.ball_modes_confirmed = index
    reservation = node.pickup_walk_reservation
    clock[0] = reservation.started_at + PICKUP_WALK_DURATIONS[first['motion_id']] * .75
    for _ in range(3):
        head(node, clock)
        fresh(node, clock, 'ball', ball(), plan=False)
    bridge.navigation_command_callback(String(data=json.dumps(node.publisher.messages[-1])))
    rejected = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, rejected, 'REJECTED', error_code='INCOMPATIBLE_TRANSITION')
    assert bridge.active_request_id == first['request_id']
    forward_status(node, bridge, first, 'SUCCEEDED')
    assert bridge.pickup_fine_align_waiting
    head(node, clock)
    fresh(node, clock, 'ball', ball())
    retry = node.publisher.messages[-1]
    assert retry['action'] == 'BALL_PICKUP_FINE_FORWARD'
    bridge.navigation_command_callback(String(data=json.dumps(retry)))
    assert decoded_messages(bridge.executor_request_publisher)[-1]['request_id'] != rejected['request_id']
    assert node.active_special_command_id == parent['command_id']


@pytest.mark.parametrize('bottom', [100, 110, 119, 120])
def test_retreat_rearm_hysteresis_does_not_block_valid_alignment(bottom):
    guard = ready()
    guard.mark_retreat_sent()
    for t in (10.01, 10.02, 10.03):
        sample = frame(guard, t, bottom, offset_x_px=100)
        assert fine(sample).action == 'BALL_PICKUP_CRAB_RIGHT'
    if bottom < 120:
        assert guard.retreat_sent  # Do not permit another retreat on pixel jitter.
        assert fine(frame(guard, 10.04, 99)).action == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'


DEPTHS = {1: .770, 2: .780, 3: .800, 4: .840, 5: .850}


@pytest.mark.parametrize('first_count', range(1, 6))
@pytest.mark.parametrize('next_count', range(1, 6))
def test_every_calibrated_goal_count_queues_and_unlocks(clock, first_count, next_count):
    node = LiveInputHarness(phase='GOAL_APPROACH')
    bridge = FakeBridge(advance_transition_dwells=False)
    first = start_goal(node, clock, DEPTHS[first_count])
    bridge.navigation_command_callback(String(data=json.dumps(first)))
    first_request = decoded_messages(bridge.executor_request_publisher)[-1]
    clock[0] += Node.GOAL_FORWARD_DURATION_SEC[first['action']] * .75
    for _ in range(3):
        fresh(node, clock, 'goal', target_sample(0, DEPTHS[next_count]), plan=False)
    second = node.publisher.messages[-1]
    assert second['action'] == f'GOAL_CAMERA90_FINE_FORWARD_{next_count}'
    assert node.queued_general_command_id == second['command_id']
    bridge.navigation_command_callback(String(data=json.dumps(second)))
    requests = decoded_messages(bridge.executor_request_publisher)
    assert len(requests) == 2
    assert not bridge.queued_request_deferred
    second_request = requests[-1]
    forward_status(node, bridge, second_request, 'QUEUED')
    forward_status(node, bridge, first_request, 'SUCCEEDED')
    forward_status(node, bridge, second_request, 'RUNNING')
    assert node.general_motion_gate.active_command_id == second['command_id']
    assert node.queued_general_command_id is None
    forward_status(node, bridge, second_request, 'SUCCEEDED')
    assert not node.general_motion_gate.locked
    assert not bridge.motion_in_progress
    assert bridge.transition_dwell_until is None


def test_rejected_five_step_goal_queue_does_not_leave_a_lock(clock):
    node = LiveInputHarness(phase='GOAL_APPROACH')
    bridge = FakeBridge(advance_transition_dwells=False)
    first = start_goal(node, clock, DEPTHS[1])
    bridge.navigation_command_callback(String(data=json.dumps(first)))
    first_request = decoded_messages(bridge.executor_request_publisher)[-1]
    clock[0] += Node.GOAL_FORWARD_DURATION_SEC[first['action']] * .75
    for _ in range(3):
        fresh(node, clock, 'goal', target_sample(0, DEPTHS[5]), plan=False)
    bridge.navigation_command_callback(String(data=json.dumps(node.publisher.messages[-1])))
    rejected = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, rejected, 'REJECTED', error_code='INCOMPATIBLE_TRANSITION')
    assert node.queued_general_command_id is None
    assert node.general_motion_gate.active_command_id == first['command_id']
    forward_status(node, bridge, first_request, 'SUCCEEDED')
    fresh(node, clock, 'goal', target_sample(0, DEPTHS[5]))
    retry = node.publisher.messages[-1]
    assert retry['action'] == 'GOAL_CAMERA90_FINE_FORWARD_5'
    assert retry['command_id'] != rejected['command_id']
    bridge.navigation_command_callback(String(data=json.dumps(retry)))
    assert bridge.active_command_id == retry['command_id']
