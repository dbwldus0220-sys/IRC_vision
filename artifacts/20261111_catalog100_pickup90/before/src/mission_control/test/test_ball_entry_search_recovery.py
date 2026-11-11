"""Replay Ball-owned Line motion, confirmed loss and bounded reacquisition."""

import pytest

from mission_control.ball_entry_line_alignment import BallEntryLineAlignment
from mission_control.motion_decision_planner import MotionDecisionPlanner
from test_ball_entry_line_alignment import (
    Node, FakeBridge, clock, fresh, node_with_entry, send, status, ground_ball_info,
)
from test_mission_phase_flow import line_info
from test_motion_decision_planner import ball_info
from test_staged_lost_search import sequence


MISSING = {'detected': False, 'raw_detected': False}


@pytest.mark.parametrize('side', ['LEFT', 'RIGHT'])
@pytest.mark.parametrize('distance', [.9, .6])
def test_entry_runs_existing_two_direction_search_after_walk_then_stops(clock, side, distance):
    node, bridge = node_with_entry(), FakeBridge()
    fresh(node, clock, 'line', line_info(), plan=False)
    fresh(node, clock, 'ball', ground_ball_info(
        depth_m=distance, camera_center_offset_x_px=-500 if side == 'LEFT' else 500))
    first = node.publisher.messages[-1]
    request = send(bridge, first)
    status(node, bridge, request, 'RUNNING')
    for _ in range(8):
        clock[0] += .1
        fresh(node, clock, 'ball', MISSING)
    assert node.general_motion_gate.active_command_id == first['command_id']
    assert not any(m['valid'] and m['command_id'] != first['command_id']
                   for m in node.publisher.messages)
    status(node, bridge, request, 'SUCCEEDED')
    previous_id = first['command_id']
    for direction, count, stage in sequence(side):
        command = None
        for _ in range(40):
            clock[0] += .1
            fresh(node, clock, 'ball', MISSING)
            valid = [m for m in node.publisher.messages
                     if m['valid'] and m['command_id'] > previous_id]
            if valid:
                command = valid[0]
                break
        assert command is not None, node.last_selected_decision
        assert command['action'] == f'BALL_APPROACH_TURN_{direction}_{count}'
        assert command['source_command']['lost_search_stage'] == stage
        assert not command['source_command'].get('ball_entry_line_alignment')
        request = send(bridge, command)
        status(node, bridge, request, 'RUNNING')
        previous_id = command['command_id']
        status(node, bridge, request, 'SUCCEEDED')
    for _ in range(40):
        clock[0] += .1
        fresh(node, clock, 'ball', MISSING)
    assert not any(m['valid'] and m['command_id'] > previous_id
                   for m in node.publisher.messages)
    assert node.lost_search_turn_limiter.counts['ball'] == 10
    assert node.ball_entry_line_alignment.pending
    # A new verified target resumes Line approach and rearms the Ball budget.
    fresh(node, clock, 'line', line_info(), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=.8))
    assert node.publisher.messages[-1]['action'] == 'STRAIGHT_2'
    assert node.publisher.messages[-1]['valid']
    assert node.lost_search_turn_limiter.counts['ball'] == 0


@pytest.mark.parametrize('info', [None, MISSING,
    {**MISSING, 'ball_loss_confirmed': False},
    {**MISSING, 'ball_loss_confirmed': True, 'raw_detected': True}])
def test_entry_does_not_search_without_fresh_confirmed_loss(info):
    entry, planner = BallEntryLineAlignment(), MotionDecisionPlanner()
    entry.arm()
    planner._update_ball_tracking(ball_info(camera_center_offset_x_px=-500), 0.)
    result = entry.select(planner, line_info(), .1, info)
    assert not result.valid


@pytest.mark.parametrize('side,counts', [('LEFT', (1, 2, 3, 4, 5, 6)),
                                       ('RIGHT', (2, 3, 5, 6, 7, 9))])
def test_shared_bins_and_entry_stationary_turns_follow_requested_table(side, counts):
    planner, entry = MotionDecisionPlanner(), BallEntryLineAlignment()
    sign = -1 if side == 'LEFT' else 1
    assert planner._turn_repeat_count(14.999, side) == 0
    for lower, count in zip((15, 30, 45, 60, 75, 90), counts):
        for angle in (lower, lower + 14.999):
            assert planner._turn_repeat_count(sign * angle, side) == count
            if lower >= 45:
                decision = entry.select(planner, line_info(sign * angle), .1, ball_info())
                assert decision.action == f'LINE_HEADING_TURN_{side}_{count}'
                assert decision.source_command['heading_error_deg'] == sign * angle


def test_reserved_entry_walk_does_not_queue_a_search():
    planner, entry = MotionDecisionPlanner(), BallEntryLineAlignment()
    planner._update_ball_tracking(ball_info(camera_center_offset_x_px=500), 0.)
    entry.arm()
    entry.published(1)
    entry.on_status(1, 'RUNNING')
    entry.published(2)
    lost = {**MISSING, 'ball_loss_confirmed': True}
    assert not entry.select(planner, line_info(), .1, lost).valid
    entry.on_status(1, 'SUCCEEDED')
    assert not entry.select(planner, line_info(), .1, lost).valid
    entry.on_status(2, 'RUNNING')
    entry.on_status(2, 'SUCCEEDED')
    assert entry.select(planner, line_info(), .1, lost).source == 'ball'
