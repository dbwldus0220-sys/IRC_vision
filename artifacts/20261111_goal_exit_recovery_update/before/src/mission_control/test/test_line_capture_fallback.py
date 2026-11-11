"""Replay a bad latest line after usable in-motion captures, without hardware."""

import pytest

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_ball_entry_line_alignment import node_with_entry, start_line
from test_continuous_approach import clock, fresh
from test_mission_phase_flow import line_info
from test_motion_command_bridge import FakeBridge
from test_motion_decision_planner import ball_info


def capture_before_queue_window(node, clock, heading):
    start = node.active_line_motion_started_at
    duration = node.active_line_motion_duration_sec
    clock[0] = start + duration * .65 - .1
    for _ in range(10):
        fresh(node, clock, 'line', line_info(heading, .3), plan=False)
    assert node.queued_general_command_id is None
    clock[0] = start + duration * .71


@pytest.mark.parametrize('ball_mode', [False, True])
@pytest.mark.parametrize('bad_line', [
    {'detected': False}, {'ground_projection_valid': False}, {'heading_quality': .1},
])
@pytest.mark.parametrize('heading', [0., 30.])
def test_recent_capture_reserves_walk_despite_bad_latest_line(clock, ball_mode, bad_line, heading):
    node = node_with_entry(enabled=ball_mode)
    first, _ = start_line(node, FakeBridge(), clock)
    capture_before_queue_window(node, clock, heading)
    # Publish the bad input first; the Ball observation then allows its reservation.
    if ball_mode:
        node.ball_entry_line_alignment.arm()
        node.phase_manager.phase = 'BALL_APPROACH'
    fresh(node, clock, 'line', {**line_info(), **bad_line}, plan=False)
    if ball_mode:
        fresh(node, clock, 'ball', ball_info(depth_m=.9), plan=False)
    queued = node.publisher.messages[-1]
    assert node.queued_general_command_id == queued['command_id']
    assert queued['command_id'] != first['command_id']
    assert queued['action'] == ('RECOVER_RIGHT_TURN_RIGHT_4' if heading else
                                'STRAIGHT_2' if ball_mode else 'STRAIGHT')
    assert node.latest_info['line'].items() >= bad_line.items()
    assert not queued['source_command'].get('ball_entry_line_fallback')
    # No second reservation can be added while the first is pending.
    fresh(node, clock, 'line', {**line_info(), **bad_line}, plan=False)
    assert node.queued_general_command_id == queued['command_id']


@pytest.mark.parametrize('bad_capture', ['expired', 'future', 'no_stamp', 'before_motion', 'too_few'])
def test_bad_latest_line_cannot_revive_invalid_capture(clock, bad_capture):
    node = node_with_entry()
    start_line(node, FakeBridge(), clock)
    capture_before_queue_window(node, clock, 30.)
    frames = node.active_line_motion_frames
    if bad_capture == 'expired':
        clock[0] += node.timeouts['line'] + .1
    elif bad_capture == 'future':
        frames[-1]['rgb_stamp_ns'] = round((clock[0] + 1.) * 1e9)
    elif bad_capture == 'no_stamp':
        frames[-1].pop('rgb_stamp_ns')
    elif bad_capture == 'before_motion':
        frames[-1]['rgb_stamp_ns'] = round((node.active_line_motion_started_at - .1) * 1e9)
    else:
        del frames[:-1]
    fresh(node, clock, 'line', {**line_info(), 'detected': False}, plan=False)
    assert node.queued_general_command_id is None


def test_stationary_turn_is_decided_from_capture_but_not_queued_mid_stride(clock):
    node = node_with_entry()
    start_line(node, FakeBridge(), clock)
    capture_before_queue_window(node, clock, 60.)
    node.ball_entry_line_alignment.arm()
    node.phase_manager.phase = 'BALL_APPROACH'
    fresh(node, clock, 'line', {**line_info(), 'detected': False}, plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=.9), plan=False)
    assert node.pending_line_decision.action == 'LINE_HEADING_TURN_RIGHT_6'
    assert node.queued_general_command_id is None


def test_close_ball_still_blocks_reservation_from_line_capture(clock):
    node = node_with_entry()
    start_line(node, FakeBridge(), clock)
    capture_before_queue_window(node, clock, 30.)
    node.ball_entry_line_alignment.arm()
    node.phase_manager.phase = 'BALL_APPROACH'
    fresh(node, clock, 'line', {**line_info(), 'detected': False}, plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=.55), plan=False)
    assert node.queued_general_command_id is None
