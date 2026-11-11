"""Both ball modes share fine-step observations and SDK prequeue policy."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from test_continuous_approach import clock, fresh
from test_wait_refresh_and_fine_settle import LiveInputHarness
from test_motion_command_bridge import FakeBridge, decoded_messages
from test_pickup_continuous_walk import ball, start, late_frames, forward_status


def second_pickup(clock):
    node = LiveInputHarness(phase='BALL_APPROACH')
    # Use the confirmed occurrence even if an earlier pickup did not succeed.
    node.phase_manager.active_ball_mode_index = 2
    node.phase_manager.ball_modes_confirmed = 2
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = 'pickup_fine_forward_0'
    parent, request = start(node, bridge, clock)
    assert 'pickup_second_backward_turn_left' in bridge.active_pickup_sequence
    return node, bridge, parent, request


@pytest.mark.parametrize('bottom,expected', [
    (269, 'BALL_PICKUP_FINE_ALIGN_CONTINUE'),
    (270, 'BALL_PICKUP_FINE_ALIGN_CONTINUE'),
    (270.001, 'BALL_PICKUP_FINE_FORWARD'),
    (344, 'BALL_PICKUP_FINE_FORWARD'),
])
def test_second_fine_without_usable_cache_uses_fresh_pixels_without_timed_pause(clock, bottom, expected):
    node, bridge, parent, request = second_pickup(clock)
    assert request['motion_id'] == 'ball_general_fine_forward_8'
    assert node.pickup_walk_reservation.key[-1] == bridge.active_motion_id
    assert node.pickup_checkpoint_observation.key[-1] == request['motion_id']
    clock[0] += 2.
    forward_status(node, bridge, request, 'SUCCEEDED')
    assert bridge.active_dwell_until is None
    assert bridge.pickup_fine_align_waiting and node.latest_info['ball'] is None
    fresh(node, clock, 'ball', ball(bottom_distance_px=bottom))
    command = node.publisher.messages[-1]
    assert command['action'] == expected
    assert node.active_special_command_id == parent['command_id']
    if expected == 'BALL_PICKUP_FINE_FORWARD':
        bridge.navigation_command_callback(String(data=json.dumps(command)))
        repeated = decoded_messages(bridge.executor_request_publisher)[-1]
        assert repeated['motion_id'] == 'pickup_fine_forward_0'
        forward_status(node, bridge, repeated, 'RUNNING')
        assert node.pickup_walk_reservation.key[-1] == bridge.active_motion_id
        forward_status(node, bridge, repeated, 'SUCCEEDED')
        assert bridge.active_dwell_until is None
        assert bridge.pickup_fine_align_waiting


@pytest.mark.parametrize('index,offset', [(1, 0), (2, 0), (1, -100), (2, -100), (2, 100)])
def test_both_ball_modes_reserve_same_fine_and_crab_gaits(clock, index, offset):
    node = LiveInputHarness(phase='BALL_APPROACH')
    node.phase_manager.active_ball_mode_index = index
    node.phase_manager.ball_modes_confirmed = index
    bridge = FakeBridge(advance_transition_dwells=False)
    motion = ('ball_general_fine_forward_8' if offset == 0
              else 'pickup_crab_left_0' if offset < 0 else 'pickup_crab_right_0')
    bridge.last_physical_motion_id = motion
    _, request = start(node, bridge, clock, ball(offset_x_px=offset))
    assert request['motion_id'] == motion
    late_frames(node, clock, ball(offset_x_px=offset))
    queued = node.publisher.messages[-1]
    assert queued['source_command']['pickup_walk_prequeue']
    bridge.navigation_command_callback(String(data=json.dumps(queued)))
    assert bridge.pickup_queued_request is not None
    forward_status(node, bridge, request, 'SUCCEEDED')
    assert bridge.active_dwell_until is None


def test_second_fine_prequeue_promotes_without_checkpoint_or_extra_pause(clock):
    node, bridge, parent, request = second_pickup(clock)
    late_frames(node, clock)
    command = node.publisher.messages[-1]
    assert command['source_command']['pickup_walk_prequeue']
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    queued = decoded_messages(bridge.executor_request_publisher)[-1]
    assert queued['request_id'] != request['request_id']
    assert queued['motion_id'] == request['motion_id']
    assert node.pickup_walk_reservation.pending
    forward_status(node, bridge, queued, 'QUEUED')
    forward_status(node, bridge, request, 'SUCCEEDED')
    forward_status(node, bridge, queued, 'RUNNING')
    assert bridge.active_request_id == queued['request_id']
    assert not bridge.pickup_fine_align_waiting
    assert bridge.active_dwell_until is None and bridge.transition_dwell_until is None
    assert not node.pickup_walk_reservation.pending
    assert node.active_special_command_id == parent['command_id']
    assert 'pickup_second_backward_turn_left' in bridge.active_pickup_sequence
