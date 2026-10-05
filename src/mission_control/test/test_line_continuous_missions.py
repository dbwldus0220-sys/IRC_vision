"""Exercise forward queueing and the timed post-pickup motion boundary."""

from dataclasses import replace

import pytest

from mission_control.motion_decision_node import MotionDecisionNode
from test_mission_phase_flow import MissionFlowHarness, line_info


@pytest.fixture
def clock(monkeypatch):
    now = [10.]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: now[0])
    return now


def start_forward(phase):
    node = MissionFlowHarness(phase=phase)
    node.phase_manager.pickups_completed = 1
    node.phase_manager.ball_grasp_results[1] = 'GRABBED'
    first = node.publish_vision(line=line_info())[-1]
    assert first['action'] == 'STRAIGHT'
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    return node, first


def capture(node, clock):
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * .8
    node.active_line_motion_frames = [line_info() for _ in range(10)]
    MotionDecisionNode._prepare_pending_line_decision(node, clock[0])


@pytest.mark.parametrize('phase', ['AUTO', 'LINE_TRACK', 'LINE_TRACK_AFTER_PICKUP'])
def test_normal_line_queues_next_forward_before_current_finishes(clock, phase):
    node, first = start_forward(phase)
    capture(node, clock)
    queued = node.publisher.messages[-1]
    assert queued['command_id'] != first['command_id']
    assert queued['action'] == 'STRAIGHT'
    assert getattr(node, 'queued_general_command_id', None) == queued['command_id']
    assert node.general_motion_gate.active_command_id == first['command_id']
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    node.send_status(queued['action'], queued['command_id'], 'RUNNING')
    assert node.general_motion_gate.active_command_id == queued['command_id']
    assert getattr(node, 'queued_general_command_id', None) is None


@pytest.mark.parametrize('source,allowed', [('ball', True), ('goal', True), ('hurdle', False), ('finish', False)])
def test_timed_line_masks_only_the_same_targets_as_mission_selection(clock, source, allowed):
    node, _ = start_forward('LINE_TRACK_AFTER_PICKUP')
    observations = {source: {'detected': True, 'raw_detected': True}}
    assert MotionDecisionNode._line_only_prequeue_allowed(node, observations) is allowed


@pytest.mark.parametrize('remaining,allowed', [(5., True), (2.467, False), (.1, False), (-.1, False)])
def test_no_extra_queue_when_next_full_forward_will_not_fit(clock, remaining, allowed):
    node, _ = start_forward('LINE_TRACK_AFTER_PICKUP')
    current_end = node.active_line_motion_started_at + node.active_line_motion_duration_sec
    node.post_ball_line_run_until = current_end + remaining
    capture(node, clock)
    assert (getattr(node, 'queued_general_command_id', None) is not None) is allowed


def test_timed_stage_waits_for_already_queued_forward_before_transition(clock):
    node, first = start_forward('LINE_TRACK_AFTER_PICKUP')
    capture(node, clock)
    second = node.publisher.messages[-1]
    # A slow executor may finish after the nominal duration. Drain the single
    # accepted command; never change camera pose while that command is pending.
    clock[0] = 21.
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    MotionDecisionNode._publish_decision(node)
    assert node.mission_phase == 'LINE_TRACK_AFTER_PICKUP'
    node.send_status(second['action'], second['command_id'], 'RUNNING')
    capture(node, clock)
    assert len(node.publisher.messages) == 2
    node.send_status(second['action'], second['command_id'], 'SUCCEEDED')
    MotionDecisionNode._publish_decision(node)
    assert node.publisher.messages[-1]['action'] == 'POST_BALL_GOAL_TRANSITION'


@pytest.mark.parametrize('action', ['RIGHT', 'LINE_HEADING_TURN_RIGHT_2', 'RECOVER_LEFT_TURN_LEFT_4'])
def test_timed_queue_does_not_bypass_turn_boundary(clock, action):
    node, _ = start_forward('LINE_TRACK_AFTER_PICKUP')
    decision = replace(node.last_selected_decision, action=action)
    MotionDecisionNode._publish_decision(node, decision, queue_while_locked=True)
    assert len(node.publisher.messages) == 1
    assert getattr(node, 'queued_general_command_id', None) is None
