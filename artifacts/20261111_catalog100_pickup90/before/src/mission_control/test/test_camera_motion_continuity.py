"""Replay camera supervision together with continuous walking and Goal handoff."""

import json

import pytest
from std_msgs.msg import String

from mission_control.camera_time_health import CameraRecoveryGate, CameraTimeHealth
from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_camera_recovery import EPOCH, camera_status, frames
from test_camera_recovery import supervised_node, advance
from test_continuous_approach import clock
from test_mission_phase_flow import line_info
from test_motion_command_bridge import decoded_messages
from test_pickup_continuous_walk import forward_status
from test_post_ball_goal_handoff import goal, receive
from test_three_line_goal_handoff import start
from test_vision_time_guard import clocks, deliver


def test_asynchronous_latest_frames_reuse_fresh_matched_pair_only():
    health = CameraTimeHealth(0.)
    ros, now = frames(health)
    assert health.status(ros, now)[0] == 'READY'
    now += .15
    ros += 150_000_000
    health.observe('rgb', ros - 20_000_000, ros, now)
    assert abs(health.frames['rgb'][0] - health.frames['depth'][0]) > 100_000_000
    assert health.status(ros, now)[0] == 'READY'
    # New RGB traffic cannot prolong an old Depth frame or matching pair.
    now += .4
    ros += 400_000_000
    health.observe('rgb', ros - 20_000_000, ros, now)
    assert health.status(ros, now)[0] == 'WAIT_FRAMES'
    assert health.frame_failure == 'depth_expired'


@pytest.mark.parametrize('bad_age', [-.001, 1.])
def test_bad_packet_does_not_erase_or_renew_accepted_camera_frame(bad_age):
    health = CameraTimeHealth(0.)
    ros, now = frames(health)
    previous = health.frames['rgb']
    health.observe('rgb', ros - round(bad_age * 1e9), ros, now)
    assert health.frames['rgb'] == previous
    assert health.status(ros, now)[0] == 'READY'
    assert health.status(ros + 510_000_000, now + .51)[0] == 'WAIT_FRAMES'


def test_pair_history_is_cleared_on_clock_jump_and_restart():
    health = CameraTimeHealth(0.)
    ros, now = frames(health)
    assert health.status(ros, now)[0] == 'READY'
    health.check_clock(ros + 10_000_000_000, now + .1)
    assert all(not history for history in health.history.values())
    assert not health.frames
    ros, now = frames(health, base=EPOCH + 10_000_000_000, start=2.5)
    health.restarted(ros, now)
    assert all(not history for history in health.history.values())
    assert health.status(ros, now)[0] != 'READY'


def test_fresh_unpaired_streams_cannot_reuse_expired_pair():
    health = CameraTimeHealth(0.)
    ros, now = frames(health)
    for delta in (.6, .61, .62):
        current = now + delta
        clock_ns = ros + round(delta * 1e9)
        health.observe('rgb', clock_ns - 20_000_000, clock_ns, current)
        health.observe('depth', clock_ns - 350_000_000, clock_ns, current)
    assert health.status(clock_ns, current)[0] == 'WAIT_FRAMES'
    assert health.frame_failure == 'rgb_depth_unpaired'


def test_transient_raw_failure_preserves_input_and_resumes_without_readmission(clocks):
    node = supervised_node(clocks)
    camera_status(node)
    for _ in range(3):
        advance(node, clocks)
    assert Node._camera_recovery_ready(node, clocks[0])
    boundary = node.vision_time_guard.minimum_stamp_ns
    info = node.latest_info['line']
    camera_status(node, 'WAIT_FRAMES')
    assert not Node._camera_recovery_ready(node, clocks[0])
    assert node.latest_info['line'] is info
    advance(node, clocks)
    assert node.latest_info['line'] is not info
    assert node.vision_time_guard.minimum_stamp_ns == boundary
    camera_status(node)
    assert Node._camera_recovery_ready(node, clocks[0])
    assert not node.camera_recovery.reobserve
    assert node.camera_recovery.reason == 'READY'


@pytest.mark.parametrize('invalid', ['head_pending', 'before_head', 'unconfirmed', 'missing'])
def test_third_line_recovery_requires_usable_head_up_goal_frames(clocks, invalid):
    node = supervised_node(clocks, phase='LINE_TRACK_AFTER_PICKUP')
    node.phase_manager.pickups_completed = 1
    node.phase_manager.ball_grasp_results[1] = 'GRABBED'
    node.post_ball_line_started_count = 3
    node.post_ball_goal_camera_ready_at = None if invalid == 'head_pending' else clocks[0]
    if invalid == 'before_head':
        node.post_ball_goal_camera_ready_at = clocks[0] + .2
    node.executor_active = True
    camera_status(node)
    for _ in range(3):
        clocks[0] += .1
        clocks[1] += .1
        node.executor_heartbeat_watchdog.observe(sequence=2, observed_at=clocks[0])
        deliver(node, clocks, 'goal',
                age=max(0., clocks[0] - 10.1) if invalid == 'before_head' else 0.,
                info=goal(detected=invalid != 'missing',
                          confirmation_confirmed=invalid != 'unconfirmed'))
    assert not Node._camera_recovery_ready(node, clocks[0])
    assert node.camera_recovery.reason == 'WAIT_PHASE_GOAL'
    assert not node.publisher.messages


def supervise_running_node(node, clock):
    node.camera_recovery = CameraRecoveryGate()
    # The first gait has already passed startup admission.
    node.camera_recovery.status = dict(state='READY', session='test-camera', generation=1)
    node.camera_recovery.received_at = clock[0]
    node.camera_recovery.reobserve = False
    node.camera_recovery.reason = 'READY'
    node.camera_restart_future = None
    node.camera_restart_failure = None
    node.executor_active = True


def refresh(node, clock, state='READY', generation=1):
    camera_status(node, state, generation=generation)
    node.executor_heartbeat_watchdog.observe(sequence=100, observed_at=clock[0])


def reserve_line(node, clock):
    started, duration = node.active_line_motion_started_at, node.active_line_motion_duration_sec
    clock[0] = max(started + duration * .75, started + duration - Node.LINE_RECOVERY_PREQUEUE_LEAD_SEC)
    for _ in range(3):
        clock[0] += .03
        refresh(node, clock)
        receive(node, clock, 'line', {**line_info(), 'ground_fit_point_count': 5})
    return node.publisher.messages[-1]


@pytest.mark.parametrize('fault', ['transient', 'new_generation'])
def test_supervised_three_line_gaits_queue_goal_without_line_reobservation(clock, fault):
    node, bridge, current = start(clock)
    supervise_running_node(node, clock)
    for ordinal in (2, 3):
        # A short raw-stream failure must not move the capture boundary or
        # erase planning state while the current gait finishes.
        before = node.latest_info['line']
        boundary = getattr(getattr(node, 'vision_time_guard', None), 'minimum_stamp_ns', None)
        refresh(node, clock, 'WAIT_FRAMES')
        assert node.latest_info['line'] is before
        assert not node.camera_recovery.reobserve
        assert getattr(node.vision_time_guard, 'minimum_stamp_ns', None) == boundary
        command = reserve_line(node, clock)
        assert command['source_command']['post_pickup_line_index'] == ordinal
        bridge.navigation_command_callback(String(data=json.dumps(command)))
        clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec
        refresh(node, clock)
        forward_status(node, bridge, current, 'SUCCEEDED')
        current = decoded_messages(bridge.executor_request_publisher)[-1]
        forward_status(node, bridge, current, 'RUNNING')

    third_end = node.active_line_motion_started_at + node.active_line_motion_duration_sec
    clock[0] += .4
    Node._executor_heartbeat_callback(node, String(data=json.dumps({
        'sequence': 101, 'active': True,
        'goal_head_prepare_id': current['goal_head_prepare_id'],
        'goal_head_transition_complete': True,
    })))
    generation = 2 if fault == 'new_generation' else 1
    refresh(node, clock, 'READY' if generation == 2 else 'WAIT_FRAMES', generation)
    for _ in range(3):
        clock[0] += .05
        refresh(node, clock, generation=generation)
        receive(node, clock, 'line', {'detected': False, 'ground_fit_point_count': 0})
        receive(node, clock, 'goal', goal())
    queued = node.publisher.messages[-1]
    assert queued['source'] == 'goal'
    assert queued['source_command']['post_ball_goal_handoff']
    assert clock[0] < third_end
    assert not node.camera_recovery.reobserve
    bridge.navigation_command_callback(String(data=json.dumps(queued)))
    clock[0] = third_end
    refresh(node, clock, generation=generation)
    forward_status(node, bridge, current, 'SUCCEEDED')
    successor = decoded_messages(bridge.executor_request_publisher)[-1]
    assert successor['command_id'] == queued['command_id']
    forward_status(node, bridge, successor, 'RUNNING')
    assert node.mission_phase == 'GOAL_APPROACH'
    assert bridge.transition_dwell_until is None
    assert bridge.active_dwell_until is None
    assert len([m for m in node.publisher.messages
                if m.get('source_command', {}).get('post_pickup_line_index')]) == 3
