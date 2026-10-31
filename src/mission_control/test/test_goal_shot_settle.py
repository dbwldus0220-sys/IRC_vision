"""Require physical preparation and fresh goal permission after a one-second hold."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode
from test_mission_phase_flow import MissionFlowHarness, mark_next_ball_grabbed, score_ready_goal


@pytest.fixture
def resting_goal(monkeypatch):
    clock = [10.0]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: clock[0])
    node = MissionFlowHarness(phase='GOAL_APPROACH')
    node.SHOT_PRE_MOTION_SETTLE_SEC = MotionDecisionNode.SHOT_PRE_MOTION_SETTLE_SEC
    mark_next_ball_grabbed(node)
    return node, clock


def finish_preparation(node):
    command = node.publish_vision(goal=score_ready_goal())[-1]
    assert command['action'] == 'GOAL_SHOT_PREPARE'
    node.send_status(command['action'], command['command_id'], 'RUNNING')
    node.send_status(command['action'], command['command_id'], 'SUCCEEDED')
    return command


def test_boundary_noise_during_observation_does_not_trigger_motion(resting_goal):
    node, clock = resting_goal
    finish_preparation(node)
    for stamp in (10.2, 10.4, 10.999):
        clock[0] = stamp
        commands = node.publish_vision(goal={**score_ready_goal(), 'depth_m': .570, 'score_now': False})
        assert all(c['action'] == 'WAIT' for c in commands)
        assert node.goal_shot_observe_until == 11.0
    clock[0] = 11.0
    assert node.publish_vision(goal=score_ready_goal())[-1]['action'] == 'SHOT'


@pytest.mark.parametrize('invalid', [None,
    {**score_ready_goal(), 'score_now': False},
    {**score_ready_goal(), 'depth_valid': False},
    {**score_ready_goal(), 'confidence': .1},
])
def test_elapsed_observation_never_authorizes_invalid_or_missing_goal(resting_goal, invalid):
    node, clock = resting_goal
    finish_preparation(node)
    clock[0] = 12.0
    assert all(c['action'] != 'SHOT' for c in node.publish_vision(goal=invalid))
    assert node.active_special_command_id is None
    clock[0] = 12.1
    assert node.publish_vision(goal=score_ready_goal())[-1]['action'] == 'SHOT'


def test_correction_after_window_requires_preparation_and_new_window(resting_goal):
    node, clock = resting_goal
    finish_preparation(node)
    clock[0] = 11.0
    correction = node.publish_vision(goal={**score_ready_goal(), 'depth_m': .60, 'score_now': False})[-1]
    assert correction['action'] == 'GOAL_CAMERA90_FINE_FORWARD_3'
    assert node.goal_shot_observe_until is None
    node.send_status(correction['action'], correction['command_id'], 'RUNNING')
    clock[0] = 14.0
    node.send_status(correction['action'], correction['command_id'], 'SUCCEEDED')
    finish_preparation(node)
    assert node.goal_shot_observe_until == 15.0
    clock[0] = 14.999
    assert all(c['action'] == 'WAIT' for c in node.publish_vision(goal=score_ready_goal()))
    clock[0] = 15.0
    assert node.publish_vision(goal=score_ready_goal())[-1]['action'] == 'SHOT'


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'REJECTED'])
def test_failed_preparation_does_not_start_observation(resting_goal, status):
    node, clock = resting_goal
    command = node.publish_vision(goal=score_ready_goal())[-1]
    node.send_status(command['action'], command['command_id'], 'RUNNING')
    node.send_status(command['action'], command['command_id'], status)
    clock[0] = 14.0
    assert node.goal_shot_observe_until is None
    assert all(c['action'] != 'SHOT' for c in node.publish_vision(goal=score_ready_goal()))


@pytest.mark.parametrize('active,auto_ready', [(True, True), (False, False)])
def test_executor_motion_or_startup_hold_cannot_authorize_shot(resting_goal, active, auto_ready):
    node, clock = resting_goal
    finish_preparation(node)
    clock[0] = 12.0
    MotionDecisionNode._executor_heartbeat_callback(node, String(data=json.dumps({
        'sequence': 1, 'active': active, 'auto_ready': auto_ready,
    })))
    assert all(c['action'] != 'SHOT' for c in node.publish_vision(goal=score_ready_goal()))


def test_unknown_executor_state_cannot_authorize_preparation(resting_goal):
    node, clock = resting_goal
    node.executor_active = None
    assert all(c['action'] == 'WAIT' for c in node.publish_vision(goal=score_ready_goal()))
    clock[0] = 20.0
    assert all(c['action'] == 'WAIT' for c in node.publish_vision(goal=score_ready_goal()))
    assert node.goal_stationary_since is None


@pytest.mark.parametrize('grasp', ['UNKNOWN', 'NOT_GRABBED'])
def test_observation_does_not_bypass_grasp_permission(resting_goal, grasp):
    node, clock = resting_goal
    node.phase_manager.ball_grasp_results[1] = grasp
    clock[0] = 12.0
    commands = node.publish_vision(goal=score_ready_goal())
    assert all(c['action'] not in {'SHOT', 'GOAL_SHOT_PREPARE'} for c in commands)
    assert node.active_special_command_id is None
