"""Separate physical rest from current shot permission without robot hardware."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode
from test_mission_phase_flow import (
    MissionFlowHarness, mark_next_ball_grabbed, score_ready_goal,
)


@pytest.fixture
def resting_goal(monkeypatch):
    clock = [10.0]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: clock[0])
    node = MissionFlowHarness(phase='GOAL_APPROACH')
    node.SHOT_PRE_MOTION_SETTLE_SEC = MotionDecisionNode.SHOT_PRE_MOTION_SETTLE_SEC
    node.FINE_FORWARD_PRE_MOTION_SETTLE_SEC = 1.0
    mark_next_ball_grabbed(node)
    return node, clock


def test_one_mm_boundary_noise_does_not_restart_shot_rest(resting_goal):
    node, clock = resting_goal
    ready = {**score_ready_goal(), 'depth_m': 0.470}
    outside = {**ready, 'depth_m': 0.471, 'score_now': False}
    assert node.publish_vision(goal=ready) == []
    for stamp in (10.4, 10.8):
        clock[0] = stamp
        assert node.publish_vision(goal=outside) == []
        assert node.last_candidate_decision.action == 'GOAL_CAMERA90_FINE_FORWARD_1'
        assert node.goal_stationary_since == 10.0
    clock[0] = 10.999
    assert node.publish_vision(goal=ready) == []
    clock[0] = 11.0
    assert node.publish_vision(goal=ready)[-1]['action'] == 'SHOT'


@pytest.mark.parametrize('invalid', [
    None,
    {**score_ready_goal(), 'score_now': False},
    {**score_ready_goal(), 'depth_valid': False},
    {**score_ready_goal(), 'confidence': 0.1},
])
def test_elapsed_rest_never_authorizes_invalid_or_missing_goal(resting_goal, invalid):
    node, clock = resting_goal
    assert node.publish_vision(goal=score_ready_goal()) == []
    clock[0] = 12.0
    commands = node.publish_vision(goal=invalid)
    assert all(command['action'] != 'SHOT' for command in commands)
    assert node.active_special_command_id is None
    assert node.goal_stationary_since == 10.0
    clock[0] = 12.1
    assert node.publish_vision(goal=score_ready_goal())[-1]['action'] == 'SHOT'


def test_executed_fine_step_restarts_rest_and_existing_dwell_counts(resting_goal):
    node, clock = resting_goal
    outside = {**score_ready_goal(), 'depth_m': 0.49, 'score_now': False}
    assert node.publish_vision(goal=outside) == []
    clock[0] = 11.0
    motion = node.publish_vision(goal=outside)[-1]
    assert motion['action'] == 'GOAL_CAMERA90_FINE_FORWARD_1'
    assert node.goal_stationary_since is None
    node.send_status(motion['action'], motion['command_id'], 'RUNNING')
    clock[0] = 14.0
    assert node.publish_vision(goal=score_ready_goal()) == []
    assert node.goal_stationary_since is None
    node.send_status(motion['action'], motion['command_id'], 'SUCCEEDED')
    assert node.goal_stationary_since == 14.0
    clock[0] = 14.999
    assert node.publish_vision(goal=score_ready_goal()) == []
    clock[0] = 15.0
    # The existing post-motion dwell discards its old image before re-observation.
    assert node.publish_vision(goal=score_ready_goal()) == []
    assert node.latest_info['goal'] is None
    assert node.publish_vision(goal=score_ready_goal())[-1]['action'] == 'SHOT'


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'REJECTED'])
def test_failed_motion_does_not_reuse_old_rest(resting_goal, status):
    node, clock = resting_goal
    outside = {**score_ready_goal(), 'depth_m': 0.49, 'score_now': False}
    node.publish_vision(goal=outside)
    clock[0] = 11.0
    motion = node.publish_vision(goal=outside)[-1]
    if status != 'REJECTED':
        node.send_status(motion['action'], motion['command_id'], 'RUNNING')
    clock[0] = 14.0
    node.send_status(motion['action'], motion['command_id'], status)
    assert node.goal_stationary_since is None
    assert node.publish_vision(goal=score_ready_goal()) == []
    assert node.goal_stationary_since == 14.0
    clock[0] = 14.999
    assert node.publish_vision(goal=score_ready_goal()) == []
    clock[0] = 15.0
    assert node.publish_vision(goal=score_ready_goal())[-1]['action'] == 'SHOT'


@pytest.mark.parametrize('active,auto_ready', [(True, True), (False, False)])
def test_executor_motion_or_startup_hold_invalidates_rest(resting_goal, active, auto_ready):
    node, clock = resting_goal
    node.publish_vision(goal=score_ready_goal())
    clock[0] = 10.8
    MotionDecisionNode._executor_heartbeat_callback(node, String(data=json.dumps({
        'sequence': 1, 'active': active, 'auto_ready': auto_ready,
    })))
    assert node.goal_stationary_since is None
    clock[0] = 12.0
    assert node.publish_vision(goal=score_ready_goal()) == []
    MotionDecisionNode._executor_heartbeat_callback(node, String(data=json.dumps({
        'sequence': 2, 'active': False, 'auto_ready': True,
    })))
    # A fresh observation also releases the startup gate in the real callback.
    node.executor_ready_requires_fresh_vision = False
    assert node.publish_vision(goal=score_ready_goal()) == []
    assert node.goal_stationary_since == 12.0
    clock[0] = 12.999
    assert node.publish_vision(goal=score_ready_goal()) == []
    clock[0] = 13.0
    assert node.publish_vision(goal=score_ready_goal())[-1]['action'] == 'SHOT'


def test_unknown_executor_state_cannot_authorize_shot(resting_goal):
    node, clock = resting_goal
    node.executor_active = None
    assert node.publish_vision(goal=score_ready_goal()) == []
    clock[0] = 20.0
    assert node.publish_vision(goal=score_ready_goal()) == []
    assert node.goal_stationary_since is None


@pytest.mark.parametrize('grasp', ['UNKNOWN', 'NOT_GRABBED'])
def test_rest_does_not_bypass_grasp_permission(resting_goal, grasp):
    node, clock = resting_goal
    node.phase_manager.ball_grasp_results[1] = grasp
    node.publish_vision(goal=score_ready_goal())
    clock[0] = 12.0
    commands = node.publish_vision(goal=score_ready_goal())
    assert all(command['action'] != 'SHOT' for command in commands)
    assert node.active_special_command_id is None
