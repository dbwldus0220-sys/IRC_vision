"""Confirmed exits override a straight tangent without adding walking pauses."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_continuous_approach import clock, fresh
from test_line_corner_memory import corner_info
from test_line_turn_prequeue import start, late
from test_motion_command_bridge import FakeBridge, decoded_messages, navigation_message, executor_status
from test_catalog86_crab_entry import crab_message
from test_pickup_continuous_walk import forward_status
from test_wait_refresh_and_fine_settle import LiveInputHarness


def exit_info(angle=30., distance=.14, **changes):
    return corner_info(
        'RIGHT' if angle > 0 else 'LEFT', ground_heading_error_deg=0.,
        corner_start_distance_m=distance, corner_preview_raw_detected=True,
        corner_exit_ground_valid=True, corner_exit_ground_heading_deg=angle,
        corner_exit_image_heading_deg=angle, **changes,
    )


@pytest.mark.parametrize('angle', [-90., -50., -15., 15., 50., 90.])
def test_straight_incoming_tangent_does_not_hide_near_exit(clock, angle):
    node = LiveInputHarness(phase='LINE_TRACK')
    fresh(node, clock, 'line', exit_info(angle))
    decision = node.last_candidate_decision
    direction = 'RIGHT' if angle > 0 else 'LEFT'
    assert decision.valid
    assert decision.reason == 'line_corner_exit_alignment'
    if abs(angle) <= 50:
        assert decision.action == f'RECOVER_{direction}_TURN_{direction}_4'
    else:
        assert decision.action.startswith(f'LINE_HEADING_TURN_{direction}_')


@pytest.mark.parametrize('distance,action', [(1.65, 'STRAIGHT'), (.6, 'STRAIGHT_3'), (.4, 'STRAIGHT_1')])
def test_corner_distance_reduces_forward_chunk_without_wait(clock, distance, action):
    node = LiveInputHarness(phase='LINE_TRACK')
    fresh(node, clock, 'line', exit_info(distance=distance))
    assert node.publisher.messages[-1]['action'] == action
    assert node.publisher.messages[-1]['valid']


@pytest.mark.parametrize('angle', [-30., 30.])
def test_recovery_exit_is_prequeued_and_promoted_without_artificial_pause(clock, angle):
    node, bridge, first = start(clock)
    late(node, clock, exit_info(angle), count=12)
    command = node.publisher.messages[-1]
    assert command['reason'] == 'line_corner_exit_alignment'
    assert command['command_id'] == node.queued_general_command_id
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    assert not bridge.queued_request_deferred
    assert len(bridge.executor_request_publisher.messages) == 2
    forward_status(node, bridge, first, 'SUCCEEDED')
    assert bridge.transition_dwell_until is None
    assert node.correction_post_motion_dwell_until is None
    assert bridge.active_command_id == command['command_id']


@pytest.mark.parametrize('change', [
    {'corner_preview_held': True}, {'corner_preview_raw_detected': False},
    {'corner_start_depth_valid': False}, {'heading_quality': .01},
    {'corner_exit_ground_heading_deg': -30., 'corner_exit_image_heading_deg': -30.},
])
def test_unverified_or_conflicting_exit_does_not_force_turn(clock, change):
    node = LiveInputHarness(phase='LINE_TRACK')
    info = {**exit_info(), **change}
    fresh(node, clock, 'line', info)
    assert node.last_candidate_decision.reason != 'line_corner_exit_alignment'


@pytest.mark.parametrize('angle', [-90., 90.])
def test_large_ground_exit_reserves_turn_but_keeps_existing_half_second_hold(clock, angle):
    node, bridge, first = start(clock)
    late(node, clock, exit_info(angle), count=3)
    command = node.publisher.messages[-1]
    assert command['reason'] == 'line_corner_exit_alignment'
    assert command['source_command']['line_turn_prequeue']
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    assert bridge.queued_request_deferred
    forward_status(node, bridge, first, 'SUCCEEDED')
    assert bridge.transition_dwell_until == pytest.approx(clock[0] + .5)
    clock[0] = bridge.transition_dwell_until
    bridge._check_atomic_dwell()
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == bridge.motion_id_for_action(command['action'])


@pytest.mark.parametrize('angle', [-90., 90.])
def test_missing_ground_uses_only_one_small_image_turn_and_requires_new_capture(clock, angle):
    node = LiveInputHarness(phase='LINE_TRACK')
    info = {
        **exit_info(angle), 'ground_projection_valid': False,
        'ground_projection_enabled': True, 'ground_fit_reason': 'too_few_segment_points',
        'ground_heading_error_deg': None, 'corner_exit_ground_valid': False,
        'corner_exit_ground_heading_deg': None,
    }
    fresh(node, clock, 'line', info)
    command = node.publisher.messages[-1]
    direction = 'RIGHT' if angle > 0 else 'LEFT'
    assert command['action'] == f'LINE_HEADING_TURN_{direction}_1'
    assert command['source_command']['invalid_ground_recovery']
    node.send_status(command['action'], command['command_id'], 'RUNNING')
    late(node, clock, info, count=12)
    assert getattr(node, 'queued_general_command_id', None) is None
    node.send_status(command['action'], command['command_id'], 'SUCCEEDED')
    assert node.latest_info['line'] is None
    assert node.line_offset_min_rgb_stamp_ns == round(clock[0] * 1e9)


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED'])
def test_failed_walk_discards_reserved_corner(clock, status):
    node, bridge, first = start(clock)
    late(node, clock, exit_info(90.), count=3)
    bridge.navigation_command_callback(String(data=json.dumps(node.publisher.messages[-1])))
    assert bridge.queued_request_id is not None
    forward_status(node, bridge, first, status)
    assert node.queued_general_command_id is None
    assert bridge.queued_request_id is None
    assert len(bridge.executor_request_publisher.messages) == 1


@pytest.mark.parametrize('distance', [-.11, .11, None])
def test_off_center_corner_keeps_ordinary_alignment(clock, distance):
    node = LiveInputHarness(phase='LINE_TRACK')
    fresh(node, clock, 'line', exit_info(corner_start_lateral_offset_m=distance))
    assert node.last_candidate_decision.reason != 'line_corner_exit_alignment'


def test_short_corner_approach_stays_seamlessly_queueable(clock):
    node, bridge, first = start(clock, action='STRAIGHT_1')
    late(node, clock, exit_info(distance=.35), count=5)
    command = node.publisher.messages[-1]
    assert command['action'] == 'STRAIGHT_1'
    assert command['reason'] == 'line_corner_continuous_approach'
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    assert not bridge.queued_request_deferred
    forward_status(node, bridge, first, 'SUCCEEDED')
    assert bridge.transition_dwell_until is None
    assert node.correction_post_motion_dwell_until is None


def test_two_point_policy_does_not_expand_image_corner_turn(clock):
    from mission_control.sparse_line_recovery import SparseLineRecovery
    from test_sparse_line_recovery import prepare, sparse_info

    node = LiveInputHarness(phase='LINE_TRACK')
    info = {**exit_info(90), 'corner_exit_ground_valid': False, 'corner_exit_ground_heading_deg': None}
    fresh(node, clock, 'line', info)
    decision = node.last_candidate_decision
    assert decision.action == 'LINE_HEADING_TURN_RIGHT_1'
    policy = SparseLineRecovery()
    sparse = sparse_info(heading=20., distance=.14)
    for moment in (10., 10.1, 10.2):
        prepare(policy, sparse, moment)
    result = policy.constrain(decision, sparse, 10.2)
    assert result.action == 'LINE_HEADING_TURN_RIGHT_1'
    assert result.source_command['turn_count'] == 1


@pytest.mark.parametrize('age', [-1., 1.])
def test_stale_or_future_exit_cannot_authorize_corner(clock, age):
    from test_wait_refresh_and_fine_settle import observe

    node = LiveInputHarness(phase='LINE_TRACK')
    observe(node, clock, 'line', {**exit_info(), 'rgb_stamp_ns': round((clock[0] - age) * 1e9)})
    assert not any(command['valid'] for command in node.publisher.messages)


@pytest.mark.parametrize('previous', ['line_recovery_ball_first_left_4', 'line_recovery_ball_second_right_4'])
@pytest.mark.parametrize('direction', ['LEFT', 'RIGHT'])
def test_crab_preparation_has_half_second_before_and_after(clock, previous, direction):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = previous
    bridge.navigation_command_callback(navigation_message(action='PICKUP_NOW'))
    bridge.navigation_command_callback(crab_message(direction, 1))
    assert not bridge.executor_request_publisher.messages
    deadline = bridge.transition_dwell_until
    assert deadline == pytest.approx(clock[0] + .5)
    clock[0] = deadline - .001
    bridge._check_atomic_dwell()
    assert not bridge.executor_request_publisher.messages
    clock[0] = deadline
    bridge._check_atomic_dwell()
    preparation = decoded_messages(bridge.executor_request_publisher)[-1]
    clock[0] += .165
    bridge.executor_status_callback(executor_status(**preparation, status='SUCCEEDED'))
    deadline = bridge.transition_dwell_until
    assert deadline == pytest.approx(clock[0] + .5)
    clock[0] = deadline - .001
    bridge._check_atomic_dwell()
    assert len(bridge.executor_request_publisher.messages) == 1
    clock[0] = deadline
    bridge._check_atomic_dwell()
    assert [request['motion_id'] for request in decoded_messages(bridge.executor_request_publisher)] == [
        preparation['motion_id'], f'pickup_crab_{direction.lower()}_0',
    ]
    assert bridge.transition_dwell_until is None
