"""Count actual mission execution and prevent a third object-mode entry."""

import pytest

from mission_control.mission_phase_manager import MissionPhaseManager
from mission_control.motion_decision_node import MotionDecisionNode
from test_mission_phase_flow import (
    MissionFlowHarness,
    approaching_ball,
    approaching_goal,
    approaching_hurdle,
    line_info,
)


MISSIONS = [
    ("PICKUP_NOW", "pickup", "pickups_executed", "BALL_APPROACH", "ball"),
    ("SHOT", "goal_shot", "shots_executed", "GOAL_APPROACH", "goal"),
    ("GO", "hurdle", "hurdles_executed", "HURDLE_APPROACH", "hurdle"),
]


@pytest.mark.parametrize("action,motion,counter,phase,source", MISSIONS)
@pytest.mark.parametrize("status", [
    "SUCCEEDED", "FAILED", "TIMEOUT", "CANCELLED", "REJECTED", "UNSUPPORTED",
])
def test_execution_counts_once_regardless_of_result(action, motion, counter, phase, source, status):
    manager = MissionPhaseManager(initial_phase=phase)
    for command_id in (1, 2):
        assert manager.start_special_action(action, command_id)
        assert getattr(manager, counter) == command_id - 1
        stale = manager.handle_motion_status(action, command_id + 99, "RUNNING", motion_id=motion)
        assert not stale.handled
        for _ in range(3):
            assert manager.handle_motion_status(action, command_id, "RUNNING", motion_id=motion).handled
        assert getattr(manager, counter) == command_id
        assert manager.execution_limit_reached(action) is (command_id == 2)
        # The limit applies to new commands, never to the currently active one.
        assert manager.start_special_action(action, command_id)
        assert manager.active_special_command_id == command_id
        assert manager.handle_motion_status(action, command_id, status, motion_id=motion).handled
        assert manager.handle_motion_status(action, command_id, status, motion_id=motion).duplicate
        manager.handle_motion_status(action, command_id, "RUNNING", motion_id=motion)
        assert getattr(manager, counter) == command_id
    assert not manager.start_special_action(action, 3)
    assert manager.active_special_command_id is None
    for other_action, _, other_counter, _, _ in MISSIONS:
        if other_action != action:
            assert getattr(manager, other_counter) == 0
            assert not manager.execution_limit_reached(other_action)


@pytest.mark.parametrize("action,motion,counter,phase,source", MISSIONS)
@pytest.mark.parametrize("preparation", [
    "__NON_BLOCKING_DWELL__", "pickup_fine_forward_0", "goal_fine_to_default",
])
def test_preparation_does_not_consume_execution_budget(action, motion, counter, phase, source, preparation):
    manager = MissionPhaseManager(initial_phase=phase)
    for command_id in (1, 2, 3):
        assert manager.start_special_action(action, command_id)
        manager.handle_motion_status(action, command_id, "RUNNING", motion_id=preparation)
        assert getattr(manager, counter) == 0
        manager.handle_motion_status(action, command_id, "FAILED", motion_id=preparation)
    assert not manager.execution_limit_reached(action)


@pytest.mark.parametrize("action,motion,counter,phase,source", MISSIONS)
@pytest.mark.parametrize("status", ["REJECTED", "UNSUPPORTED"])
def test_rejection_before_execution_does_not_count(action, motion, counter, phase, source, status):
    manager = MissionPhaseManager()
    assert manager.start_special_action(action, 1)
    assert manager.handle_motion_status(action, 1, status, motion_id=motion).handled
    assert getattr(manager, counter) == 0


@pytest.mark.parametrize("action,motion,counter,phase,source", MISSIONS)
def test_success_proves_execution_when_actual_running_was_missed(action, motion, counter, phase, source):
    manager = MissionPhaseManager()
    assert manager.start_special_action(action, 1)
    manager.handle_motion_status(action, 1, "RUNNING", motion_id="__NON_BLOCKING_DWELL__")
    manager.handle_motion_status(action, 1, "SUCCEEDED", motion_id="exit_pose")
    assert getattr(manager, counter) == 1


@pytest.mark.parametrize("action,motion,counter,phase,source", MISSIONS)
@pytest.mark.parametrize("other_source", ["ball", "goal", "hurdle"])
def test_two_failed_executions_block_only_the_exhausted_mode(
    action, motion, counter, phase, source, other_source,
):
    harness = MissionFlowHarness(phase=phase)
    for command_id in (1, 2):
        assert harness.phase_manager.start_special_action(action, command_id)
        harness.send_status(action, command_id, "RUNNING", motion_id=motion)
        assert getattr(harness.phase_manager, counter) == command_id
        assert harness.active_special_command_id == command_id
        harness.send_status(action, command_id, "FAILED", motion_id=motion)
    assert harness.active_special_command_id is None
    assert getattr(harness.phase_manager, counter) == 2
    assert MotionDecisionNode._mission_progress(harness)[counter] == 2
    assert harness.phase_manager.pickups_completed == 0
    assert harness.phase_manager.shots_completed == 0
    assert harness.phase_manager.hurdles_completed == 0

    # Even a forced approach phase and a fresh false detection cannot re-enter.
    harness.phase_manager.set_phase(phase)
    samples = {"ball": approaching_ball(), "goal": approaching_goal(), "hurdle": approaching_hurdle()}
    decision = MotionDecisionNode._select_mission_decision(
        harness, {"line": line_info(), source: samples[source]}, 0.1,
    )
    assert decision.source == "line"
    assert harness.mission_phase not in {"BALL_APPROACH", "GOAL_APPROACH", "HURDLE_APPROACH"}
    decision = MotionDecisionNode._select_mission_decision(
        harness,
        {"line": line_info(), source: samples[source], other_source: samples[other_source]},
        0.1,
    )
    assert decision.source == ("line" if other_source == source else other_source)
    assert not harness.phase_manager.start_special_action(action, 3)


@pytest.mark.parametrize("action,motion,counter,phase,source", MISSIONS)
def test_second_execution_keeps_motion_lock_until_terminal(action, motion, counter, phase, source):
    harness = MissionFlowHarness(phase=phase)
    setattr(harness.phase_manager, counter, 1)
    assert harness.phase_manager.start_special_action(action, 2)
    harness.send_status(action, 2, "RUNNING", motion_id=motion)
    decision = MotionDecisionNode._select_mission_decision(
        harness, {"line": line_info()}, 0.1,
    )
    assert decision.action == "WAIT"
    assert harness.active_special_command_id == 2
    assert harness.mission_phase == phase
    harness.send_status(action, 2, "SUCCEEDED", motion_id=motion)
    assert harness.active_special_command_id is None
    assert getattr(harness.phase_manager, counter) == 2
    assert harness.mission_phase == {
        "PICKUP_NOW": "POST_BALL_LINE_ALIGN", "SHOT": "POST_SHOT_TURN", "GO": "LINE_TRACK",
    }[action]


@pytest.mark.parametrize("action,motion,counter,phase,source", MISSIONS)
def test_zero_execution_budget_disables_mission_without_starting(action, motion, counter, phase, source):
    parameter = MissionPhaseManager.EXECUTION_COUNTERS[action][1]
    manager = MissionPhaseManager(**{parameter: 0})
    assert manager.execution_limit_reached(action)
    assert not manager.start_special_action(action, 1)
    assert getattr(manager, counter) == 0
