"""Replay the 0.40 m corner and distinguish rejected input from missing packets."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_decision_planner import MotionDecisionPlanner
from test_line_ground_fit_recovery import frame
from test_motion_decision_planner import ball_info
from test_sparse_line_recovery import sparse_info
from test_wait_refresh_and_fine_settle import LiveInputHarness


@pytest.fixture
def clock(monkeypatch):
    now = [10.0]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: now[0])
    monkeypatch.setattr(MotionDecisionNode, '_current_ros_time_ns', lambda self: int(now[0] * 1e9))
    return now


@pytest.mark.parametrize('direction', ['LEFT', 'RIGHT'])
@pytest.mark.parametrize('ground_valid', [False, True])
def test_confirmed_entry_precedes_fit_recovery_and_large_heading(clock, direction, ground_valid):
    node = LiveInputHarness(phase='LINE_TRACK')
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = 1.0
    info = sparse_info(direction=direction, distance=.4)
    if ground_valid:
        info.update(ground_projection_valid=True, ground_heading_error_deg=45.5,
                    ground_fit_point_count=3, ground_fit_reason='ok')
    # The local offset or heading may point away from the confirmed corner.
    info['filtered_lateral_offset_norm'] = -.3
    assert frame(node, clock, info) == []
    clock[0] += .5
    assert frame(node, clock, info) == []
    clock[0] += .51
    command = frame(node, clock, info)[-1]
    assert command['valid'] and command['action'] == direction
    assert command['reason'] == 'line_corner_ready'
    assert command['source_command']['corner_without_ground_fit'] is not ground_valid
    assert command['source_command']['corner_turn_distance_m'] == .4
    assert 'invalid_ground_recovery' not in command['source_command']
    node.send_status(direction, command['command_id'], 'RUNNING')
    clock[0] += .1
    assert frame(node, clock, info) == []


@pytest.mark.parametrize('updates', [
    {'corner_start_distance_m': .4001}, {'corner_preview_held': True},
    {'corner_preview_raw_detected': False}, {'corner_preview_confirmed': False},
    {'corner_start_depth_valid': False}, {'corner_start_distance_m': None},
    {'heading_quality': .1}, {'geometry_quality': None},
    {'ground_fit_reason': 'invalid_parameters'}, {'ground_fit_segment': 'FULL'},
    {'ground_fit_input_point_count': 1}, {'filtered_lateral_offset_norm': None},
    {'detected': False}, {'rgb_stamp_ns': 1}, {'rgb_stamp_ns': 11_000_000_000},
])
def test_two_point_corner_exception_is_narrow_and_requires_fresh_input(clock, updates):
    node = LiveInputHarness(phase='LINE_TRACK')
    info = {**sparse_info(), 'rgb_stamp_ns': 10_000_000_000, **updates}
    node.latest_time['line'] = clock[0]
    decision = node.planner.plan('LINE_TRACK', {'line': info}, .1)
    result = MotionDecisionNode._apply_pending_line_corner(node, decision, info)
    assert result.action not in {'LEFT', 'RIGHT'}


def test_motion_capture_boundary_still_blocks_confirmed_two_point_corner(clock):
    node = LiveInputHarness(phase='LINE_TRACK')
    info = {**sparse_info(), 'rgb_stamp_ns': 10_000_000_000}
    node.latest_time['line'] = clock[0]
    MotionDecisionNode._remember_line_corner(node, info)
    node.pending_line_corner['minimum_rgb_stamp_ns'] = 10_000_000_000
    decision = node.planner.plan('LINE_TRACK', {'line': info}, .1)
    result = MotionDecisionNode._apply_pending_line_corner(node, decision, info)
    assert not result.valid
    assert 'captured_after_motion' in result.source_command['line_input_checks']['failed_checks']


def test_transport_records_packet_rejected_before_motion_boundary(clock):
    node = LiveInputHarness(phase='LINE_TRACK')
    node.line_offset_min_rgb_stamp_ns = 10_000_000_000
    callback = MotionDecisionNode._info_callback(node, 'line')
    callback(String(data=json.dumps({**sparse_info(), 'rgb_stamp_ns': 9_900_000_000})))
    packet = node.vision_transport['line']
    assert node.latest_info['line'] is None
    assert packet['status'] == 'before_line_motion_boundary'
    assert packet['capture_age_at_receive_sec'] == .1
    assert packet['received_count'] == 1
    clock[0] += .1
    callback(String(data=json.dumps({**sparse_info(), 'rgb_stamp_ns': 10_100_000_000})))
    assert node.vision_transport['line']['status'] == 'accepted'
    assert node.vision_transport['line']['receive_gap_sec'] == .1
    assert node.vision_transport['line']['received_count'] == 2


def test_recorded_centered_ball_finishes_fine_alignment_at_270_pixels():
    planner = MotionDecisionPlanner()
    decision = planner.plan_ball_pickup_fine_alignment(ball_info(
        confidence=.99, offset_x_px=-21, bottom_distance_px=258, depth_m=.42,
    ))
    assert decision.action == 'BALL_PICKUP_FINE_ALIGN_CONTINUE'
    assert decision.source_command['pickup_fine_align_bottom_distance_px'] == 270


@pytest.mark.parametrize('robot', [False, True])
@pytest.mark.parametrize('distance', ['0.40', '0.15'])
def test_launch_shares_one_corner_entry_distance(monkeypatch, tmp_path, robot, distance):
    from launch_ros.actions import Node
    from test_full_system_launch import node_parameters
    if robot:
        from test_full_system_robot_launch import launch_description, default_context
    else:
        from test_full_system_launch import launch_description, launch_context as default_context
    description = launch_description(monkeypatch, tmp_path)
    context = default_context(description)
    assert context.launch_configurations['corner_turn_margin_m'] == '0.40'
    assert context.launch_configurations['line_corner_memory_timeout_sec'] == '15.0'
    context.launch_configurations['corner_turn_margin_m'] = distance
    context.launch_configurations['line_corner_memory_timeout_sec'] = '7.5'
    nodes = [e for e in description.entities if isinstance(e, Node)]
    decision = next(n for n in nodes if n.node_executable == 'motion_decision_node')
    analyzer = next(n for n in nodes if n.node_executable == 'unified_vision_node')
    assert node_parameters(decision, context)['line_corner_turn_distance_m'] == float(distance)
    assert node_parameters(decision, context)['line_corner_memory_timeout_sec'] == 7.5
    assert node_parameters(analyzer, context)['corner_turn_margin_m'] == float(distance)
