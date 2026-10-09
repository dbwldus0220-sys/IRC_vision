"""Replay walking boundaries with live Vision and correlated executor statuses."""

import pytest

from mission_control.motion_decision_node import MotionDecisionNode
from test_line_offset_alignment import sample
from test_motion_decision_planner import goal_info
from test_mission_phase_flow import line_info
from test_line_corner_memory import corner_info
from test_wait_refresh_and_fine_settle import LiveInputHarness, observe
from test_motion_command_bridge import (
    FakeBridge, decoded_messages, executor_status, navigation_message,
)


@pytest.fixture
def clock(monkeypatch):
    now = [10.]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: now[0])
    monkeypatch.setattr(MotionDecisionNode, '_current_ros_time_ns', lambda _: round(now[0] * 1e9))
    return now


def fresh(node, clock, source, info, *, plan=True):
    observe(node, clock, source, {**info, 'rgb_stamp_ns': round((clock[0] + .01) * 1e9)}, plan=plan)


def start_goal(node, clock, depth):
    fresh(node, clock, 'goal', goal_info(depth_m=depth, score_now=False))
    command = node.publisher.messages[-1]
    motion = FakeBridge.ACTION_TO_MOTION_ID[command['action']]
    node.send_status(command['action'], command['command_id'], 'RUNNING', motion)
    return command


@pytest.mark.parametrize('phase', ['LINE_TRACK', 'LINE_TRACK_AFTER_PICKUP'])
def test_offset_forward_queues_and_promotes_without_clearing_vision(clock, phase):
    node = LiveInputHarness(phase=phase)
    aligned = sample(target_angle=0., ground_heading_error_deg=-30.)
    fresh(node, clock, 'line', aligned)
    first = node.publisher.messages[-1]
    assert first['action'] == 'STRAIGHT_1'
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    clock[0] += .7
    for _ in range(6):
        fresh(node, clock, 'line', aligned, plan=False)
    second = node.publisher.messages[-1]
    assert second['command_id'] != first['command_id']
    assert node.queued_general_command_id == second['command_id']
    assert second['action'] == 'STRAIGHT_1'
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    assert node.latest_info['line'] is not None
    node.send_status(second['action'], second['command_id'], 'RUNNING')
    assert node.general_motion_gate.active_command_id == second['command_id']
    assert node.line_offset_motion_command_id == second['command_id']
    node.send_status(second['action'], second['command_id'], 'FAILED')
    assert node.planner.line_offset_alignment_failed


@pytest.mark.parametrize('depth', [1.5, 1., .7, .49])
def test_goal_forward_queues_only_one_then_promotes_without_pause(clock, depth):
    node = LiveInputHarness(phase='GOAL_APPROACH')
    first = start_goal(node, clock, depth)
    # Early images do not reserve a second chunk.
    for _ in range(3):
        fresh(node, clock, 'goal', goal_info(depth_m=depth, score_now=False), plan=False)
    assert getattr(node, 'queued_general_command_id', None) is None
    clock[0] = node.goal_forward_started_at + MotionDecisionNode.GOAL_FORWARD_DURATION_SEC[first['action']] * .75
    for _ in range(6):
        fresh(node, clock, 'goal', goal_info(depth_m=depth, score_now=False), plan=False)
    assert len([m for m in node.publisher.messages if m['valid']]) == 2
    second = node.publisher.messages[-1]
    assert second['action'] == first['action']
    assert node.queued_general_command_id == second['command_id']
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    assert node.latest_info['goal'] is not None
    assert node.goal_post_motion_dwell_until is None
    assert getattr(node, 'correction_post_motion_dwell_until', None) is None
    node.send_status(second['action'], second['command_id'], 'RUNNING', FakeBridge.ACTION_TO_MOTION_ID[second['action']])
    assert node.general_motion_gate.active_command_id == second['command_id']
    assert node.goal_forward_started_at == clock[0]


@pytest.mark.parametrize('changes', [
    {'detected': False}, {'depth_valid': False}, {'confidence': .1},
    {'confirmation_confirmed': False}, {'bearing_deg': 30.},
    {'depth_m': .43, 'score_now': True}, {'depth_m': .35},
])
def test_goal_does_not_reserve_forward_for_loss_turn_shot_or_retreat(clock, changes):
    node = LiveInputHarness(phase='GOAL_APPROACH')
    start_goal(node, clock, 1.5)
    clock[0] += 2.4
    for _ in range(4):
        fresh(node, clock, 'goal', goal_info(**{
            'depth_m': 1.5, 'score_now': False, **changes,
        }), plan=False)
    assert getattr(node, 'queued_general_command_id', None) is None
    assert not node.planner.goal_terminal_requested


def test_goal_queue_rejects_frames_captured_before_late_window(clock):
    node = LiveInputHarness(phase='GOAL_APPROACH')
    start_goal(node, clock, .49)
    clock[0] += .3
    for i in range(4):
        observe(node, clock, 'goal', goal_info(depth_m=.49, score_now=False,
                rgb_stamp_ns=10_050_000_000 + i * 10_000_000), plan=False)
    assert getattr(node, 'queued_general_command_id', None) is None


def test_goal_failure_clears_reserved_motion_and_requires_new_vision(clock):
    node = LiveInputHarness(phase='GOAL_APPROACH')
    first = start_goal(node, clock, 1.5)
    clock[0] += 2.4
    for _ in range(3):
        fresh(node, clock, 'goal', goal_info(depth_m=1.5, score_now=False), plan=False)
    assert node.queued_general_command_id is not None
    node.send_status(first['action'], first['command_id'], 'FAILED')
    assert node.queued_general_command_id is None
    assert node.latest_info['goal'] is None
    assert not node.general_motion_gate.has_required_fresh_vision()


def test_goal_forward_without_reservation_uses_recent_frame_immediately(clock):
    node = LiveInputHarness(phase='GOAL_APPROACH')
    first = start_goal(node, clock, 1.5)
    clock[0] += .1
    fresh(node, clock, 'goal', goal_info(depth_m=1., score_now=False), plan=False)
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    MotionDecisionNode._publish_decision(node)
    assert node.publisher.messages[-1]['action'] == 'GOAL_CAMERA_90_FORWARD_2'


def test_goal_forward_does_not_reuse_expired_vision(clock):
    node = LiveInputHarness(phase='GOAL_APPROACH')
    first = start_goal(node, clock, 1.5)
    clock[0] += 4.
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    MotionDecisionNode._publish_decision(node)
    assert not node.publisher.messages[-1]['valid']


@pytest.mark.parametrize('first,second', [
    ('GOAL_CAMERA_90_FORWARD', 'GOAL_CAMERA_90_FORWARD_2'),
    ('GOAL_CAMERA90_FINE_FORWARD_3', 'GOAL_CAMERA90_FINE_FORWARD_1'),
])
def test_bridge_reserves_different_goal_lengths_without_post_motion_dwell(first, second):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action=first, source='goal'))
    bridge.navigation_command_callback(navigation_message(action=second, source='goal', command_id=8001))
    assert bridge.queued_request_deferred
    assert len(bridge.executor_request_publisher.messages) == 1
    bridge.executor_status_callback(executor_status(status='SUCCEEDED', motion_id=bridge.ACTION_TO_MOTION_ID[first]))
    requests = decoded_messages(bridge.executor_request_publisher)
    assert requests[-1]['motion_id'] == bridge.ACTION_TO_MOTION_ID[second]
    assert bridge.active_command_id == 8001
    assert bridge.transition_dwell_until is None


def test_goal_preparation_status_does_not_start_forward_capture(clock):
    node = LiveInputHarness(phase='GOAL_APPROACH')
    fresh(node, clock, 'goal', goal_info(depth_m=.7, score_now=False))
    command = node.publisher.messages[-1]
    node.send_status(command['action'], command['command_id'], 'RUNNING', 'pickup_fine_prepare')
    clock[0] += 2.
    for _ in range(4):
        fresh(node, clock, 'goal', goal_info(depth_m=.7, score_now=False), plan=False)
    assert getattr(node, 'goal_forward_started_at', None) is None
    assert getattr(node, 'queued_general_command_id', None) is None
    node.send_status(command['action'], command['command_id'], 'RUNNING', 'goal_camera_90_fine_forward_3')
    assert node.goal_forward_started_at == clock[0]


def test_bridge_reports_actual_goal_preparation_before_walking():
    bridge = FakeBridge()
    bridge.last_physical_motion_id = 'goal_camera_90_forward_4'
    bridge.navigation_command_callback(navigation_message(
        action='GOAL_CAMERA90_FINE_FORWARD_3', source='goal'))
    bridge.executor_status_callback(executor_status(motion_id='pickup_fine_prepare'))
    status = decoded_messages(bridge.motion_status_publisher)[-1]
    assert status['action'] == 'GOAL_CAMERA90_FINE_FORWARD_3'
    assert status['status'] == 'RUNNING'
    assert status['motion_id'] == 'pickup_fine_prepare'


@pytest.mark.parametrize('blocker', ['turn', 'invalid', 'deadline'])
def test_offset_forward_reservation_respects_new_geometry_and_timed_exit(clock, blocker):
    node = LiveInputHarness(phase='LINE_TRACK_AFTER_PICKUP')
    aligned = sample(target_angle=0., ground_heading_error_deg=-30.)
    fresh(node, clock, 'line', aligned)
    first = node.publisher.messages[-1]
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    clock[0] += .7
    if blocker == 'deadline':
        node.post_ball_line_run_until = clock[0] + .4
        next_line = aligned
    elif blocker == 'turn':
        next_line = sample(target_angle=25., ground_heading_error_deg=-10.)
    else:
        next_line = {**aligned, 'ground_projection_valid': False}
    for _ in range(6):
        fresh(node, clock, 'line', next_line, plan=False)
    assert getattr(node, 'queued_general_command_id', None) is None


@pytest.mark.parametrize('phase', ['AUTO', 'LINE_TRACK', 'LINE_TRACK_AFTER_PICKUP'])
@pytest.mark.parametrize('recovery', [False, True])
def test_walking_boundary_keeps_recent_corner_frame_without_another_capture(clock, phase, recovery):
    from dataclasses import replace
    node = LiveInputHarness(phase=phase)
    node.planner.line_planner.config = replace(
        node.planner.line_planner.config, direction_confirmation_frames=1)
    node.post_ball_line_run_until = clock[0] + 30.
    initial = line_info(heading=-20., offset=-.3) if recovery else line_info()
    fresh(node, clock, 'line', initial)
    first = node.publisher.messages[-1]
    assert first['action'] == ('RECOVER_LEFT_TURN_LEFT_4' if recovery else 'STRAIGHT')
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    clock[0] += 3.
    recent = corner_info(corner_start_distance_m=.5)
    fresh(node, clock, 'line', recent, plan=False)
    assert getattr(node, 'queued_general_command_id', None) is None
    clock[0] += .2
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    assert node.latest_info['line'] is not None
    assert node.pending_line_corner['minimum_rgb_stamp_ns'] is None
    MotionDecisionNode._publish_decision(node)
    command = node.publisher.messages[-1]
    assert command['command_id'] != first['command_id']
    assert command['valid'] and command['action'] == 'STRAIGHT_1'
    assert command['reason'] == 'line_corner_turn_too_far'


@pytest.mark.parametrize('age', [.6, -1.])
def test_walking_boundary_still_rejects_expired_or_future_corner_capture(clock, age):
    node = LiveInputHarness(phase='LINE_TRACK')
    fresh(node, clock, 'line', line_info())
    first = node.publisher.messages[-1]
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    clock[0] += 3.
    observe(node, clock, 'line', corner_info(
        corner_start_distance_m=.5,
        rgb_stamp_ns=round((clock[0] - age) * 1e9),
    ), plan=False)
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    MotionDecisionNode._publish_decision(node)
    assert not node.publisher.messages[-1]['valid']


@pytest.mark.parametrize('start_recovery', [False, True])
@pytest.mark.parametrize('deadline_fits', [False, True])
def test_post_pickup_prequeue_includes_walking_recovery_in_both_directions(clock, start_recovery, deadline_fits):
    node = LiveInputHarness(phase='LINE_TRACK_AFTER_PICKUP')
    recovery = line_info(heading=-20., offset=-.3)
    fresh(node, clock, 'line', recovery if start_recovery else line_info())
    first = node.publisher.messages[-1]
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * .75
    node.post_ball_line_run_until = clock[0] + (30. if deadline_fits else .2)
    for _ in range(16):
        fresh(node, clock, 'line', line_info() if start_recovery else recovery, plan=False)
    queued = getattr(node, 'queued_general_command_id', None)
    assert (queued is not None) is deadline_fits
    if deadline_fits:
        assert node.publisher.messages[-1]['action'] == (
            'STRAIGHT' if start_recovery else 'RECOVER_LEFT_TURN_LEFT_4'
        )
