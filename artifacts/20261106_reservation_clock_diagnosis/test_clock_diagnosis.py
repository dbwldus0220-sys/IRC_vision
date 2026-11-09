"""Offline diagnosis only: no ROS graph or motor access."""
import json
import pytest
from std_msgs.msg import String
from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_continuous_approach import clock, fresh
from test_ball_entry_line_alignment import node_with_entry, start_line
from test_mission_phase_flow import line_info
from test_motion_decision_planner import ball_info
from test_motion_command_bridge import FakeBridge
from test_line_corner_memory import corner_info


@pytest.mark.parametrize('age', [0.1, 0.6, 371.409, -0.1])
def test_line_reservation_clock_gate(clock, age):
    clock[0] = 1000.
    node = node_with_entry()
    start_line(node, FakeBridge(), clock)
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * .75
    for _ in range(12):
        clock[0] += .01
        stamp = round((clock[0] - age) * 1e9)
        payload = {**line_info(), 'stamp': {'sec': stamp // 10**9, 'nanosec': stamp % 10**9}}
        Node._info_callback(node, 'line')(String(data=json.dumps(payload)))
    if age == .1:
        assert node.queued_general_command_id is not None
    else:
        assert node.queued_general_command_id is None
        assert len(node.active_line_motion_frames) == 10
        if hasattr(node, 'line_prequeue_block_reason'):
            assert node.line_prequeue_block_reason == 'capture_not_fresh'


@pytest.mark.parametrize('age', [0.1, 0.6, 371.409, -0.1])
def test_confirmed_ball_entry_clock_gate(clock, age):
    clock[0] = 1000.
    node = node_with_entry()
    start_line(node, FakeBridge(), clock)
    info = ball_info(confidence=.91, depth_m=1.01, raw_detected=True,
                     confirmation_confirmed=True, rgb_stamp_ns=round((clock[0] - age) * 1e9))
    Node._info_callback(node, 'ball')(String(data=json.dumps(info)))
    if age == .1:
        assert node.mission_phase == 'BALL_APPROACH'
        assert node.vision_transport['ball']['status'] == 'accepted'
    else:
        assert node.mission_phase == 'AUTO'
        assert node.vision_transport['ball']['status'] == 'ball_capture_time_rejected'
        fresh(node, clock, 'ball', info | {'rgb_stamp_ns': round(clock[0] * 1e9)}, plan=False)
        assert node.mission_phase == 'BALL_APPROACH'


@pytest.mark.parametrize('age', [0.1, 0.6, 371.409, -0.1])
def test_visible_corner_clock_gate(clock, age):
    clock[0] = 1000.
    node = node_with_entry()
    info = corner_info(corner_start_distance_m=.46,
                       rgb_stamp_ns=round((clock[0] - age) * 1e9))
    node.latest_info['line'] = info
    node.latest_time['line'] = clock[0]
    decision = node.planner.plan('AUTO', {'line': info}, .1)
    result = Node._apply_pending_line_corner(node, decision, info)
    if age == .1:
        assert node.pending_line_corner is not None
        assert result.reason != 'line_corner_waiting_for_fresh_distance'
    else:
        assert getattr(node, 'pending_line_corner', None) is None
        assert result.action == 'WAIT'
        assert result.reason == 'line_corner_waiting_for_fresh_distance'
        assert not result.source_command


@pytest.mark.parametrize('age', [0.6, 371.409, -0.1])
@pytest.mark.parametrize('boundary_enabled', [False, True])
def test_rejected_reservation_still_allows_ordinary_straight(clock, age, boundary_enabled):
    """Document the existing inconsistent gates, not the desired safe behavior."""
    clock[0] = 1000.
    node = node_with_entry()
    node.boundary_decision_enabled = boundary_enabled
    first, _ = start_line(node, FakeBridge(), clock)
    started, duration = node.active_line_motion_started_at, node.active_line_motion_duration_sec
    clock[0] = started + duration * .75
    for _ in range(12):
        clock[0] += .01
        info = {**line_info(), 'rgb_stamp_ns': round((clock[0] - age) * 1e9)}
        Node._info_callback(node, 'line')(String(data=json.dumps(info)))
    assert node.queued_general_command_id is None
    assert not Node._recent_walking_input(node, 'line', clock[0])
    assert Node._fresh_observations(node, clock[0])[0]['line'] is not None
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    Node._publish_decision(node)
    second = node.publisher.messages[-1]
    assert second['valid'] and second['action'] == 'STRAIGHT'
    assert second['command_id'] != first['command_id']
