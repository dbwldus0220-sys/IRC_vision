"""Replay catalog70 crab reservations at real frame intervals and turn deferral."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.pickup_walk_reservation import PICKUP_WALK_DURATIONS
from test_continuous_approach import clock, fresh
from test_gait_transitions import finish_request, requests
from test_motion_command_bridge import (
    FakeBridge, navigation_message,
)
from test_pickup_continuous_walk import ball, forward_status, start
from test_wait_refresh_and_fine_settle import LiveInputHarness


def running_right_crab(clock):
    node = LiveInputHarness(phase='BALL_APPROACH')
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = 'pickup_crab_right_0'
    parent, request = start(node, bridge, clock, ball(offset_x_px=100))
    assert request['motion_id'] == 'pickup_crab_right_0'
    return node, bridge, parent, request


def deliver_periodic_frames(node, clock, fps, phase, delay):
    reservation = node.pickup_walk_reservation
    started = reservation.started_at
    end = started + PICKUP_WALK_DURATIONS[reservation.motion_id]
    captured = started + phase
    while captured + delay < end:
        clock[0] = captured + delay
        info = ball(offset_x_px=100, rgb_stamp_ns=round(captured * 1e9))
        Node._info_callback(node, 'ball')(String(data=json.dumps(info)))
        if reservation.pending:
            return True
        captured += 1.0 / fps
    clock[0] = end
    return False


@pytest.mark.parametrize('phase', [0, .008, .016, .024, .032])
@pytest.mark.parametrize('delay', [0, .030, .050])
def test_right_crab_reserves_at_30hz_with_processing_delay(clock, phase, delay):
    node, bridge, parent, first = running_right_crab(clock)
    assert deliver_periodic_frames(node, clock, 30, phase, delay)
    command = node.publisher.messages[-1]
    assert command['source_command']['pickup_walk_prequeue']
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    second = requests(bridge)[-1]
    assert second['motion_id'] == 'pickup_crab_right_0'
    assert second['request_id'] != first['request_id']
    assert second['command_id'] == parent['command_id']
    forward_status(node, bridge, second, 'QUEUED')
    forward_status(node, bridge, first, 'SUCCEEDED')
    forward_status(node, bridge, second, 'RUNNING')
    assert bridge.active_request_id == second['request_id']
    assert bridge.transition_dwell_until is None
    assert bridge.active_dwell_until is None
    forward_status(node, bridge, second, 'SUCCEEDED')
    assert bridge.pickup_initial_align_waiting
    assert bridge.pickup_queued_request is None


@pytest.mark.parametrize('phase', [0, .025, .050, .075])
def test_right_crab_at_10hz_misses_prequeue_but_resumes_after_completion(clock, phase):
    node, bridge, _, first = running_right_crab(clock)
    assert not deliver_periodic_frames(node, clock, 10, phase, 0)
    assert bridge.pickup_queued_request is None
    forward_status(node, bridge, first, 'SUCCEEDED')
    assert bridge.pickup_initial_align_waiting
    assert node.pickup_walk_reservation.key is None
    fresh(node, clock, 'ball', ball(offset_x_px=100))
    bridge.navigation_command_callback(String(data=json.dumps(node.publisher.messages[-1])))
    assert len(requests(bridge)) == 2
    second = requests(bridge)[-1]
    # Sequential pickup steps retain the parent request ID; SDK queues use a new ID.
    assert second['request_id'] == first['request_id']
    assert second['motion_id'] == 'pickup_crab_right_0'
    assert not bridge.pickup_initial_align_waiting
    assert bridge.transition_dwell_until is None and bridge.active_dwell_until is None
    forward_status(node, bridge, second, 'RUNNING')
    forward_status(node, bridge, second, 'SUCCEEDED')
    assert bridge.pickup_initial_align_waiting


def test_right_crab_rejected_prequeue_releases_reservation_and_resumes(clock):
    node, bridge, _, first = running_right_crab(clock)
    assert deliver_periodic_frames(node, clock, 30, 0, 0)
    bridge.navigation_command_callback(String(data=json.dumps(node.publisher.messages[-1])))
    queued = requests(bridge)[-1]
    forward_status(node, bridge, queued, 'REJECTED', error_code='INCOMPATIBLE_TRANSITION')
    assert bridge.pickup_queued_request is None
    assert bridge.active_request_id == first['request_id']
    forward_status(node, bridge, first, 'SUCCEEDED')
    assert bridge.pickup_initial_align_waiting
    fresh(node, clock, 'ball', ball(offset_x_px=100))
    bridge.navigation_command_callback(String(data=json.dumps(node.publisher.messages[-1])))
    assert len(requests(bridge)) == 3
    resumed = requests(bridge)[-1]
    assert resumed['request_id'] == first['request_id']
    assert resumed['motion_id'] == 'pickup_crab_right_0'
    assert not bridge.pickup_initial_align_waiting
    forward_status(node, bridge, resumed, 'RUNNING')
    forward_status(node, bridge, resumed, 'SUCCEEDED')
    assert bridge.pickup_initial_align_waiting


@pytest.mark.parametrize('first_count,next_count', [(2, 5), (5, 2), (5, 9), (9, 5)])
def test_camera45_turn_count_change_starts_after_completion(first_count, next_count):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        action=f'POST_BALL_LINE_TURN_RIGHT_{first_count}', command_id=1))
    first = requests(bridge)[-1]
    bridge.navigation_command_callback(navigation_message(
        action=f'POST_BALL_LINE_TURN_RIGHT_{next_count}', command_id=2))
    assert len(requests(bridge)) == 1
    assert bridge.queued_request_deferred
    finish_request(bridge)
    assert len(requests(bridge)) == 2
    assert requests(bridge)[-1]['motion_id'] == f'post_ball_line_turn_right_{next_count}'
    assert requests(bridge)[-1]['request_id'] != first['request_id']
    finish_request(bridge)
    assert not bridge.motion_in_progress
