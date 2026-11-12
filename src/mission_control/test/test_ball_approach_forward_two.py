"""Exercise near Ball walking through the node, bridge and completion gates."""

import pytest

from test_ball_fine_alignment_queue import ball
from test_ball_entry_line_alignment import node_with_entry, send, status
from test_continuous_approach import clock, fresh
from test_motion_command_bridge import FakeBridge, decoded_messages, executor_status
from test_wait_refresh_and_fine_settle import observe


def start(node, bridge, clock):
    fresh(node, clock, 'ball', ball())
    command = node.publisher.messages[-1]
    assert command['action'] == 'BALL_APPROACH_FORWARD_2'
    request = send(bridge, command)
    assert request['motion_id'] == 'ball_align_forward_2'
    status(node, bridge, request, 'RUNNING')
    assert node.ball_fine_duration_sec == pytest.approx(1.028 / 1.05)
    return request, node.ball_fine_started_at


@pytest.mark.parametrize('index', [1, 2])
@pytest.mark.parametrize('latency', [0., .15, .35])
def test_ten_hz_captures_queue_or_resume_after_completion(clock, index, latency):
    node, bridge = node_with_entry(), FakeBridge(advance_transition_dwells=False)
    node.phase_manager.active_ball_mode_index = index
    node.phase_manager.ball_modes_confirmed = index
    request, started = start(node, bridge, clock)
    end = started + node.ball_fine_duration_sec
    for capture_delta in (.60, .70, .80):
        received = started + capture_delta + latency
        if received >= end:
            break
        clock[0] = received - .01
        observe(node, clock, 'ball', {**ball(), 'rgb_stamp_ns': round((started + capture_delta) * 1e9)}, plan=False)
    queued_id = node.queued_general_command_id
    if latency <= .15:
        assert queued_id is not None
    else:
        assert queued_id is None
    if queued_id is not None:
        following = send(bridge, node.publisher.messages[-1])
        assert following['motion_id'] == 'ball_align_forward_2'
        assert not bridge.queued_request_deferred
        status(node, bridge, following, 'QUEUED')
    clock[0] = end
    status(node, bridge, request, 'SUCCEEDED')
    if queued_id is not None:
        status(node, bridge, following, 'RUNNING')
        assert node.general_motion_gate.active_command_id == queued_id
    else:
        for _ in range(3):
            fresh(node, clock, 'ball', ball())
        command = node.publisher.messages[-1]
        assert command['valid'] and command['action'] == 'BALL_APPROACH_FORWARD_2'
        following = send(bridge, command)
        assert following['motion_id'] == 'ball_align_forward_2'
    assert bridge.transition_dwell_until is None


@pytest.mark.parametrize('bad', ['lost', 'stale_depth', 'invalid_ground', 'pickup_distance'])
def test_running_forward_cannot_reserve_from_invalid_or_pickup_observation(clock, bad):
    node, bridge = node_with_entry(), FakeBridge()
    request, started = start(node, bridge, clock)
    changes = {
        'lost': {'detected': False, 'raw_detected': False},
        'stale_depth': {'depth_age_sec': .701},
        'invalid_ground': {'ground_projection_valid': False},
        'pickup_distance': {'distance_m': .5, 'depth_m': .5},
    }[bad]
    for delta in (.75, .85, .95):
        clock[0] = started + delta
        fresh(node, clock, 'ball', {**ball(), **changes}, plan=False)
    assert node.queued_general_command_id is None
    assert len(bridge.executor_request_publisher.messages) == 1


@pytest.mark.parametrize('failure', ['FAILED', 'CANCELLED', 'TIMEOUT', 'REJECTED'])
def test_failed_forward_discards_queued_forward_and_late_success(clock, failure):
    node, bridge = node_with_entry(), FakeBridge()
    request, started = start(node, bridge, clock)
    for delta in (.75, .85, .95):
        clock[0] = started + delta
        fresh(node, clock, 'ball', ball(), plan=False)
    assert node.queued_general_command_id is not None
    following = send(bridge, node.publisher.messages[-1])
    status(node, bridge, following, 'QUEUED')
    status(node, bridge, request, failure)
    assert node.ball_fine_faulted
    assert node.queued_general_command_id is None
    assert bridge.queued_request_id is None
    before = len(bridge.executor_request_publisher.messages)
    bridge.executor_status_callback(executor_status(**request, status='SUCCEEDED'))
    for _ in range(3):
        fresh(node, clock, 'ball', ball())
    assert not node.publisher.messages[-1]['valid']
    assert len(bridge.executor_request_publisher.messages) == before


def test_rejected_reservation_can_retry_after_successful_current_walk(clock):
    node, bridge = node_with_entry(), FakeBridge()
    request, started = start(node, bridge, clock)
    for delta in (.75, .85, .95):
        clock[0] = started + delta
        fresh(node, clock, 'ball', ball(), plan=False)
    following = send(bridge, node.publisher.messages[-1])
    status(node, bridge, following, 'REJECTED')
    assert node.queued_general_command_id is None
    assert not getattr(node, 'ball_fine_faulted', False)
    clock[0] = started + node.ball_fine_duration_sec
    status(node, bridge, request, 'SUCCEEDED')
    for _ in range(3):
        fresh(node, clock, 'ball', ball())
    command = node.publisher.messages[-1]
    assert command['valid'] and command['action'] == 'BALL_APPROACH_FORWARD_2'
    assert send(bridge, command)['motion_id'] == 'ball_align_forward_2'
