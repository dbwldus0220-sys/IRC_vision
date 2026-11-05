"""Second-ball fine steps settle and reobserve without changing first-ball queues."""

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


def finish_observation_pause(node, bridge, clock):
    clock[0] = bridge.active_dwell_until
    before = len(bridge.motion_status_publisher.messages)
    bridge._check_atomic_dwell(clock[0])
    for message in bridge.motion_status_publisher.messages[before:]:
        Node._motion_status_callback(node, message)
    assert bridge.pickup_fine_align_waiting
    assert node.latest_info['ball'] is None


@pytest.mark.parametrize('bottom,expected', [
    (219, 'BALL_PICKUP_FINE_ALIGN_CONTINUE'),
    (220, 'BALL_PICKUP_FINE_FORWARD'),
    (221, 'BALL_PICKUP_FINE_FORWARD'),
    (344, 'BALL_PICKUP_FINE_FORWARD'),
])
def test_second_fine_waits_then_uses_only_post_pause_pixels(clock, bottom, expected):
    node, bridge, parent, request = second_pickup(clock)
    assert request['motion_id'] == 'ball_general_fine_forward_8'
    assert node.pickup_walk_reservation.key is None
    clock[0] += 1.5
    for _ in range(4):
        fresh(node, clock, 'ball', ball(bottom_distance_px=344))
    assert not any(m.get('source_command', {}).get('pickup_walk_prequeue')
                   for m in node.publisher.messages)
    before = len(bridge.executor_request_publisher.messages)
    forward_status(node, bridge, request, 'SUCCEEDED')
    deadline = bridge.active_dwell_until
    assert deadline == pytest.approx(clock[0] + .5)
    assert not bridge.pickup_fine_align_waiting
    fresh(node, clock, 'ball', ball(bottom_distance_px=220))
    clock[0] = deadline - .001
    bridge._check_atomic_dwell(clock[0])
    assert not bridge.pickup_fine_align_waiting
    assert len(bridge.executor_request_publisher.messages) == before
    finish_observation_pause(node, bridge, clock)
    Node._publish_decision(node)
    assert node.publisher.messages[-1]['action'] != 'BALL_PICKUP_FINE_ALIGN_CONTINUE'
    fresh(node, clock, 'ball', ball(bottom_distance_px=bottom))
    command = node.publisher.messages[-1]
    assert command['action'] == expected
    assert node.active_special_command_id == parent['command_id']
    if expected == 'BALL_PICKUP_FINE_FORWARD':
        bridge.navigation_command_callback(String(data=json.dumps(command)))
        repeated = decoded_messages(bridge.executor_request_publisher)[-1]
        assert repeated['motion_id'] == 'pickup_fine_forward_0'
        forward_status(node, bridge, repeated, 'RUNNING')
        assert node.pickup_walk_reservation.key is None
        forward_status(node, bridge, repeated, 'SUCCEEDED')
        assert bridge.active_dwell_until == pytest.approx(clock[0] + .5)


@pytest.mark.parametrize('index,offset', [(1, 0), (1, -100), (2, -100), (2, 100)])
def test_first_ball_and_second_ball_crab_keep_existing_reservations(clock, index, offset):
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


def test_second_fine_rejects_direct_prequeue(clock):
    node, bridge, parent, request = second_pickup(clock)
    before = len(bridge.executor_request_publisher.messages)
    bridge.navigation_command_callback(String(data=json.dumps({
        'command_id': 9999, 'source': 'ball', 'action': 'BALL_PICKUP_FINE_FORWARD',
        'valid': True, 'active_special_command_id': parent['command_id'],
        'active_special_event_id': parent['event_id'],
        'source_command': {'pickup_walk_prequeue': True,
                           'pickup_running_request_id': request['request_id'],
                           'pickup_queue_motion_id': request['motion_id'],
                           'pickup_queue_frame_count': 3},
    })))
    assert len(bridge.executor_request_publisher.messages) == before
    assert bridge.pickup_queued_request is None
    assert decoded_messages(bridge.motion_status_publisher)[-1]['error_code'] == 'PICKUP_QUEUE_NOT_AVAILABLE'
