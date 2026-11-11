"""Verify uninterrupted forward/recovery queues across both Ball ranges."""

import pytest

from test_ball_entry_line_alignment import (
    Node, FakeBridge, clock, fresh, ground_ball_info, line_info,
    node_with_entry, send, status,
)


def late_observations(node, clock, distance, angle, *, missing_line=False):
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec - .25
    fresh(node, clock, 'ball', ground_ball_info(
        depth_m=distance, ground_steering_angle_deg=angle), plan=False)
    if distance > .66 and not missing_line:
        for _ in range(16):
            fresh(node, clock, 'line', line_info(angle, angle / 90), plan=False)
    for _ in range(3):
        fresh(node, clock, 'ball', ground_ball_info(
            depth_m=distance, ground_steering_angle_deg=angle), plan=False)


def start_walk(node, bridge, clock, distance, angle, *, missing_line=False):
    if distance > .66 and not missing_line:
        fresh(node, clock, 'line', line_info(angle, angle / 90), plan=False)
    fresh(node, clock, 'ball', ground_ball_info(depth_m=distance, ground_steering_angle_deg=angle))
    first = node.publisher.messages[-1]
    assert first['valid']
    request = send(bridge, first)
    status(node, bridge, request, 'RUNNING')
    return first, request


@pytest.mark.parametrize('distances', [(.9, .8), (.65, .6), (.7, .66), (.6, .7)])
@pytest.mark.parametrize('first_angle,next_angle', [
    (0., 0.), (0., -30.), (0., 30.), (-30., 0.), (30., 0.),
    (-30., -30.), (30., 30.), (-30., 30.), (30., -30.),
])
def test_walk_pairs_queue_before_completion_and_promote_without_dwell(
        clock, distances, first_angle, next_angle):
    node, bridge = node_with_entry(), FakeBridge()
    first, request = start_walk(node, bridge, clock, distances[0], first_angle)
    late_observations(node, clock, distances[1], next_angle)
    queued = node.publisher.messages[-1]
    assert queued['valid'] and queued['command_id'] != first['command_id']
    assert node.queued_general_command_id == queued['command_id']
    assert node.general_motion_gate.active_command_id == first['command_id']
    expected = ('line_forward_4' if next_angle == 0. else
                'line_recovery_left_4' if next_angle < 0. else 'line_recovery_right_4')
    next_request = send(bridge, queued)
    assert next_request['motion_id'] == expected
    assert not bridge.queued_request_deferred
    assert bridge.active_command_id == first['command_id']
    status(node, bridge, request, 'SUCCEEDED')
    status(node, bridge, next_request, 'RUNNING')
    assert node.general_motion_gate.active_command_id == queued['command_id']
    assert Node._ball_entry_line_running(node)
    assert getattr(node, 'correction_post_motion_dwell_until', None) is None
    assert getattr(bridge, 'transition_dwell_until', None) is None
    assert node.pre_motion_settle_started_at is None


def test_line_missing_fallback_also_queues_forward_without_dwell(clock):
    node, bridge = node_with_entry(), FakeBridge()
    first, request = start_walk(node, bridge, clock, .9, 0., missing_line=True)
    late_observations(node, clock, .8, 0., missing_line=True)
    queued = node.publisher.messages[-1]
    assert queued['command_id'] != first['command_id']
    assert queued['source_command']['ball_entry_line_fallback']
    next_request = send(bridge, queued)
    assert next_request['motion_id'] == 'line_forward_4'
    assert not bridge.queued_request_deferred


@pytest.mark.parametrize('change', [
    {'depth_m': .55}, {'ground_steering_angle_deg': 60.},
    {'ground_projection_valid': False}, {'depth_age_sec': 10.},
    {'confidence': .1}, {'detected': False, 'raw_detected': False},
])
def test_no_walking_queue_for_pickup_turn_loss_or_invalid_ball(clock, change):
    node, bridge = node_with_entry(), FakeBridge()
    start_walk(node, bridge, clock, .65, 0.)
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec - .25
    for _ in range(4):
        fresh(node, clock, 'ball', ground_ball_info(**{'depth_m': .6, **change}), plan=False)
    assert node.queued_general_command_id is None


def test_ground_queue_requires_distinct_late_ball_frames(clock):
    node, bridge = node_with_entry(), FakeBridge()
    start_walk(node, bridge, clock, .65, 0.)
    fresh(node, clock, 'ball', ground_ball_info(depth_m=.6), plan=False)
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec - .25
    for _ in range(4):
        Node._prepare_pending_approach_walk(node, clock[0])
    assert node.queued_general_command_id is None
    for _ in range(2):
        fresh(node, clock, 'ball', ground_ball_info(depth_m=.6), plan=False)
    for _ in range(4):
        Node._prepare_pending_approach_walk(node, clock[0])
    assert node.queued_general_command_id is None
    fresh(node, clock, 'ball', ground_ball_info(depth_m=.6), plan=False)
    assert node.queued_general_command_id is not None


def test_pickup_latch_blocks_third_gait_after_already_queued_walk(clock):
    node, bridge = node_with_entry(), FakeBridge()
    _, request = start_walk(node, bridge, clock, .65, 0.)
    late_observations(node, clock, .6, 0.)
    queued_request = send(bridge, node.publisher.messages[-1])
    fresh(node, clock, 'ball', ground_ball_info(depth_m=.55), plan=False)
    assert node.ball_pickup_entry_pending
    status(node, bridge, request, 'SUCCEEDED')
    status(node, bridge, queued_request, 'RUNNING')
    late_observations(node, clock, .57, 0.)
    assert node.queued_general_command_id is None
    status(node, bridge, queued_request, 'SUCCEEDED')
    assert node.publisher.messages[-1]['action'] == 'PICKUP_NOW'
