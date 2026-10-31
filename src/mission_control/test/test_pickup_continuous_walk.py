"""Replay pickup prequeue, real checkpoint ownership, and failure boundaries."""

import json
from pathlib import Path

import pytest
import yaml
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from mission_control.motion_decision_planner import MotionDecisionPlanner
from mission_control.pickup_walk_reservation import (
    PickupWalkReservation, PICKUP_WALK_DURATIONS,
)
from test_continuous_approach import clock, fresh
from test_wait_refresh_and_fine_settle import LiveInputHarness
from test_motion_command_bridge import FakeBridge, decoded_messages, executor_status, navigation_message
from test_motion_decision_planner import ball_info


def ball(**changes):
    return ball_info(**{
        'depth_m': .49, 'distance_m': .49, 'depth_age_sec': .05,
        'confirmation_confirmed': True, 'bottom_distance_px': 340,
        'offset_x_px': 0, 'steering_angle_deg': 0., 'head_down_requested': True,
        **changes,
    })


def forward_status(node, bridge, request, status, **changes):
    before = len(bridge.motion_status_publisher.messages)
    bridge.executor_status_callback(executor_status(**request, status=status, **changes))
    for message in bridge.motion_status_publisher.messages[before:]:
        Node._motion_status_callback(node, message)


def start(node, bridge, clock, sample=None):
    sample = sample or ball()
    fresh(node, clock, 'ball', sample)
    parent = node.publisher.messages[-1]
    assert parent['action'] == 'PICKUP_NOW'
    bridge.navigation_command_callback(String(data=json.dumps(parent)))
    Node._motion_status_callback(node, bridge.motion_status_publisher.messages[-1])
    fresh(node, clock, 'ball', sample)
    command = node.publisher.messages[-1]
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, request, 'RUNNING')
    return parent, request


def late_frames(node, clock, sample=None, count=3):
    reservation = node.pickup_walk_reservation
    clock[0] = reservation.started_at + PICKUP_WALK_DURATIONS[reservation.motion_id] * .75
    for _ in range(count):
        fresh(node, clock, 'ball', sample or ball(), plan=False)


@pytest.mark.parametrize('motion,offset', [
    ('ball_general_fine_forward_8', 0), ('pickup_crab_left_0', -100), ('pickup_crab_right_0', 100),
])
def test_same_gait_reserves_one_step_and_keeps_parent_pickup_locked(clock, motion, offset):
    node, bridge = LiveInputHarness(phase='BALL_APPROACH'), FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = motion
    parent, first = start(node, bridge, clock, ball(offset_x_px=offset))
    assert first['motion_id'] == motion
    assert bridge.transition_dwell_until is None and bridge.active_dwell_until is None
    for _ in range(3):
        fresh(node, clock, 'ball', ball(offset_x_px=offset), plan=False)
    assert not node.pickup_walk_reservation.pending
    late_frames(node, clock, ball(offset_x_px=offset))
    queued = node.publisher.messages[-1]
    assert queued['source_command']['pickup_walk_prequeue']
    assert node.pickup_walk_reservation.pending
    bridge.navigation_command_callback(String(data=json.dumps(queued)))
    second = decoded_messages(bridge.executor_request_publisher)[-1]
    assert second['motion_id'] == motion
    assert second['command_id'] == parent['command_id']
    assert second['request_id'] != first['request_id']
    assert bridge.transition_dwell_until is None and bridge.active_dwell_until is None
    forward_status(node, bridge, second, 'QUEUED')
    count = len(node.publisher.messages)
    for _ in range(4):
        fresh(node, clock, 'ball', ball(offset_x_px=offset), plan=False)
    assert len(node.publisher.messages) == count
    forward_status(node, bridge, first, 'SUCCEEDED')
    assert bridge.active_request_id == second['request_id']
    assert node.active_special_command_id == parent['command_id']
    assert not bridge.pickup_initial_align_waiting and not bridge.pickup_fine_align_waiting
    forward_status(node, bridge, first, 'SUCCEEDED')  # Stale duplicate cannot advance.
    forward_status(node, bridge, second, 'RUNNING')
    assert node.pickup_walk_reservation.key[1] == second['request_id']
    assert not node.pickup_walk_reservation.pending
    # Only one chunk was reserved. With no new frames it finishes at a checkpoint.
    forward_status(node, bridge, second, 'SUCCEEDED')
    assert bridge.pickup_initial_align_waiting or bridge.pickup_fine_align_waiting
    assert node.latest_info['ball'] is None
    assert node.active_special_command_id == parent['command_id']


@pytest.mark.parametrize('change', [
    {'bottom_distance_px': 270}, {'offset_x_px': 80}, {'distance_m': .501},
    {'depth_valid': False}, {'depth_age_sec': .701}, {'confidence': .1},
    {'confirmation_confirmed': False}, {'detected': False, 'raw_detected': False},
])
def test_late_invalid_or_changed_target_does_not_reserve(clock, change):
    node, bridge = LiveInputHarness(phase='BALL_APPROACH'), FakeBridge()
    bridge.last_physical_motion_id = 'pickup_fine_forward_0'
    start(node, bridge, clock)
    late_frames(node, clock, ball(**change))
    assert not node.pickup_walk_reservation.pending
    assert not any(m.get('source_command', {}).get('pickup_walk_prequeue') for m in node.publisher.messages)


def test_repeated_image_and_camera_preparation_cannot_reserve(clock):
    planner, reservation = MotionDecisionPlanner(), PickupWalkReservation()
    payload = dict(status='RUNNING', command_id=1, request_id=1, motion_id='pickup_fine_forward_0',
                   pickup_alignment_stage='fine')
    reservation.on_status(payload, 10., 10_000_000_000)
    sample = ball(rgb_stamp_ns=11_500_000_000)
    for _ in range(5):
        reservation.observe(planner, sample, 11.5, 11_500_000_000, .5)
    assert reservation.count == 1 and reservation.ready is None
    reservation.on_status({**payload, 'motion_id': 'pickup_fine_prepare'}, 11.6, 11_600_000_000)
    assert reservation.key is None


@pytest.mark.parametrize('status,error', [('FAILED', 'SDK_COMMUNICATION_ERROR'), ('TIMEOUT', 'TIMEOUT'), ('CANCELLED', '')])
def test_active_failure_discards_reservation_and_does_not_skip_failed_motion(clock, status, error):
    node, bridge = LiveInputHarness(phase='BALL_APPROACH'), FakeBridge()
    bridge.last_physical_motion_id = 'pickup_fine_forward_0'
    _, first = start(node, bridge, clock)
    late_frames(node, clock)
    bridge.navigation_command_callback(String(data=json.dumps(node.publisher.messages[-1])))
    second = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, first, status, error_code=error)
    assert bridge.pickup_queued_request is None and not bridge.motion_in_progress
    assert not node.pickup_walk_reservation.pending
    count = len(bridge.executor_request_publisher.messages)
    forward_status(node, bridge, second, 'RUNNING')
    assert len(bridge.executor_request_publisher.messages) == count
    assert not bridge.motion_in_progress


def test_sdk_queue_rejection_falls_back_to_fresh_checkpoint(clock):
    node, bridge = LiveInputHarness(phase='BALL_APPROACH'), FakeBridge()
    bridge.last_physical_motion_id = 'pickup_fine_forward_0'
    _, first = start(node, bridge, clock)
    late_frames(node, clock)
    bridge.navigation_command_callback(String(data=json.dumps(node.publisher.messages[-1])))
    second = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, second, 'REJECTED', error_code='INCOMPATIBLE_TRANSITION')
    assert bridge.motion_in_progress and bridge.active_request_id == first['request_id']
    forward_status(node, bridge, first, 'SUCCEEDED')
    assert bridge.pickup_fine_align_waiting
    assert node.latest_info['ball'] is None


def test_repeated_fine_at_checkpoint_skips_redundant_prepare(clock):
    node, bridge = LiveInputHarness(phase='BALL_APPROACH'), FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = 'pickup_fine_forward_0'
    _, first = start(node, bridge, clock)
    forward_status(node, bridge, first, 'SUCCEEDED')
    fresh(node, clock, 'ball', ball())
    bridge.navigation_command_callback(String(data=json.dumps(node.publisher.messages[-1])))
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == 'pickup_fine_forward_0'
    assert bridge.transition_dwell_until is None and bridge.active_dwell_until is None
    second = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, second, 'RUNNING')
    late_frames(node, clock)
    assert node.publisher.messages[-1]['source_command']['pickup_queue_motion_id'] == 'pickup_fine_forward_0'


@pytest.mark.parametrize('change', [
    {'active_special_command_id': -1}, {'active_special_event_id': -1},
    {'action': 'BALL_PICKUP_CRAB_LEFT'},
])
def test_wrong_parent_or_gait_cannot_queue(clock, change):
    node, bridge = LiveInputHarness(phase='BALL_APPROACH'), FakeBridge()
    bridge.last_physical_motion_id = 'pickup_fine_forward_0'
    _, first = start(node, bridge, clock)
    late_frames(node, clock)
    command = {**node.publisher.messages[-1], **change}
    count = len(bridge.executor_request_publisher.messages)
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    assert len(bridge.executor_request_publisher.messages) == count
    assert bridge.pickup_queued_request is None
    assert bridge.active_request_id == first['request_id']


def test_only_one_queue_and_three_successive_chunks_keep_one_pickup(clock):
    node, bridge = LiveInputHarness(phase='BALL_APPROACH'), FakeBridge()
    bridge.last_physical_motion_id = 'pickup_fine_forward_0'
    parent, current = start(node, bridge, clock)
    ids = {current['request_id']}
    for _ in range(3):
        late_frames(node, clock)
        command = node.publisher.messages[-1]
        bridge.navigation_command_callback(String(data=json.dumps(command)))
        following = decoded_messages(bridge.executor_request_publisher)[-1]
        assert following['request_id'] not in ids
        ids.add(following['request_id'])
        before = len(bridge.executor_request_publisher.messages)
        bridge.navigation_command_callback(String(data=json.dumps(command)))
        assert len(bridge.executor_request_publisher.messages) == before
        forward_status(node, bridge, following, 'QUEUED')
        forward_status(node, bridge, current, 'SUCCEEDED')
        forward_status(node, bridge, following, 'RUNNING')
        assert node.active_special_command_id == parent['command_id']
        assert node.pickups_completed == 0
        current = following
    late_frames(node, clock, ball(bottom_distance_px=270))
    assert not node.pickup_walk_reservation.pending
    forward_status(node, bridge, current, 'SUCCEEDED')
    fresh(node, clock, 'ball', ball(bottom_distance_px=270))
    command = node.publisher.messages[-1]
    assert command['action'] == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    assert bridge.active_motion_id == 'pickup_pre_backward_camera_down'


def test_stationary_turn_checkpoint_keeps_one_second_and_fresh_frame(clock):
    node, bridge = LiveInputHarness(phase='BALL_APPROACH'), FakeBridge()
    bridge.last_physical_motion_id = 'pickup_camera_down_turn_right_3'
    _, request = start(node, bridge, clock, ball(bottom_distance_px=400, steering_angle_deg=30.,
                                               offset_x_norm=.3, offset_x_px=120))
    assert request['motion_id'] == 'pickup_camera_down_turn_right_3'
    assert node.pickup_walk_reservation.key is None
    forward_status(node, bridge, request, 'SUCCEEDED')
    assert bridge.active_dwell_until == clock[0] + Bridge.PICKUP_TURN_DWELL_SEC
    assert not bridge.pickup_initial_align_waiting


def test_all_existing_support_transition_ids_keep_their_pause(clock):
    for motion in Bridge.TRANSITION_MOTION_IDS:
        bridge = FakeBridge(advance_transition_dwells=False)
        bridge.motion_in_progress = True
        bridge.active_action = 'PICKUP_NOW'
        bridge.active_command_id = bridge.active_event_id = bridge.active_request_id = 1
        bridge.active_motion_id = motion
        bridge._publish_executor_request(action='PICKUP_NOW', command_id=1, event_id=1,
                                        request_id=1, motion_id=motion, timeout_ms=20000,
                                        prepare_pickup_fine=False)
        assert not bridge.executor_request_publisher.messages, motion
        assert bridge.transition_dwell_until == clock[0] + Bridge.TRANSITION_DWELL_SEC
        clock[0] += Bridge.TRANSITION_DWELL_SEC
        bridge._check_transition_dwell()
        request = decoded_messages(bridge.executor_request_publisher)[-1]
        bridge.executor_status_callback(executor_status(**request, status='SUCCEEDED'))
        assert (bridge.transition_dwell_until or bridge.active_dwell_until) == clock[0] + .5, motion


def test_reservation_durations_match_current_runtime():
    root = Path(__file__).resolve().parents[3]
    aliases = yaml.safe_load((root / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    assert aliases['ball_general_fine_forward_8'] == aliases['pickup_fine_forward_0']
    motions = {m['name']: m for m in json.loads((root / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
    for motion_id, duration in PICKUP_WALK_DURATIONS.items():
        motion = motions[aliases[motion_id]]
        assert motion['repeatable']
        assert duration == pytest.approx(max(f['start_ms'] + f['time_ms'] for f in motion['frames'])
                                         / 1000 * motion['repeat_count'] / motion['playback_speed'])
