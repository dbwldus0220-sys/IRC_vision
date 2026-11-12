"""Keep hurdle ownership from confirmed approach entry through detection loss."""

import pytest

from mission_control.motion_decision_planner import MotionDecisionConfig, MotionDecisionPlanner
from test_continuous_approach import clock
from test_hurdle_580_sequence import observe, start_far
from test_hurdle_approach_prequeue import start
from test_hurdle_positioning import HurdleHarness, hurdle
from test_motion_decision_planner import ball_info, goal_info, line_info


@pytest.mark.parametrize("phase", ["AUTO", "LINE_TRACK", "HURDLE_APPROACH"])
@pytest.mark.parametrize("depth", [.580001, .8, 1.0])
def test_approach_acquires_lock_without_entering_near_control(phase, depth):
    planner = MotionDecisionPlanner()
    decision = planner.plan(phase, {"line": line_info(), "hurdle": hurdle(depth)}, .1)
    assert decision.source == "hurdle" and decision.action == "STRAIGHT_3"
    assert planner.hurdle_lock_active
    assert not planner.hurdle_planner.fine_approach_active
    assert not decision.source_command.get("hurdle_positioning_active", False)
    assert decision.source_command["hurdle_stage"] == "RECOGNITION_APPROACH"
    assert not decision.source_command["hurdle_near_control"]


@pytest.mark.parametrize("changes", [
    {"detected": False}, {"confirmation_confirmed": False},
    {"confidence": .59}, {"depth_valid": False},
    {"depth_m": None}, {"depth_m": 0.}, {"depth_m": -1.},
    {"depth_m": float("nan")}, {"depth_m": float("inf")},
    {"depth_m": 1.000001},
])
def test_invalid_or_outside_approach_observation_cannot_acquire_lock(changes):
    planner = MotionDecisionPlanner()
    planner.plan("AUTO", {"line": line_info(), "hurdle": {**hurdle(.8), **changes}}, .1)
    assert not planner.hurdle_lock_active


def test_lock_respects_configured_control_range():
    planner = MotionDecisionPlanner(MotionDecisionConfig(hurdle_control_range_m=.9))
    planner.plan("AUTO", {"hurdle": hurdle(.95)}, .1)
    assert not planner.hurdle_lock_active
    planner.plan("AUTO", {"hurdle": hurdle(.9)}, .1)
    assert planner.hurdle_lock_active


@pytest.mark.parametrize("missing", [
    None,
    {"detected": False, "raw_detected": False},
    {"detected": False, "raw_detected": True, "confirmation_confirmed": False},
])
def test_missing_approach_target_keeps_lock_without_selecting_another_mission(missing):
    planner = MotionDecisionPlanner()
    planner.plan("AUTO", {"hurdle": hurdle(.8)}, .1)
    for _ in range(20):
        decision = planner.plan("AUTO", {
            "hurdle": missing, "line": line_info(),
            "ball": ball_info(), "goal": goal_info(),
        }, 1.)
        assert planner.hurdle_lock_active
        assert decision.source == "hurdle" and decision.action == "WAIT"
        assert not decision.valid
        assert not planner.hurdle_planner.fine_approach_active
    resumed = planner.plan("AUTO", {"hurdle": hurdle(.8)}, .1)
    assert resumed.source == "hurdle" and resumed.action == "STRAIGHT_3"


def test_approach_lock_preserves_580mm_camera_gate_and_completion_release():
    planner = MotionDecisionPlanner()
    planner.plan("AUTO", {"hurdle": hurdle(.8)}, .1)
    waiting = planner.plan("AUTO", {"hurdle": hurdle(.580)}, .1)
    assert waiting.reason == "waiting_for_hurdle_camera_zero_frame"
    assert planner.hurdle_planner.fine_approach_active
    assert waiting.source_command["hurdle_stage"] == "FINE_APPROACH"
    ready = planner.plan("HURDLE_POSITIONING", {
        "hurdle": hurdle(.580, hurdle_camera_pose="0"),
    }, .1)
    assert ready.action == "GO" and ready.valid
    planner.plan("LINE_TRACK", {"line": line_info(), "hurdle": hurdle(.8)}, .1)
    assert not planner.hurdle_lock_active
    assert planner.hurdle_ignore_until_clear


def test_node_holds_hurdle_after_far_motion_finishes_with_missing_target(clock):
    node = HurdleHarness()
    first = start_far(node, clock)
    assert node.planner.hurdle_lock_active
    assert not getattr(node, "hurdle_positioning_entry_pending", False)
    clock[0] += 3.
    node.receive_line()
    observe(node, clock, {"detected": False, "raw_detected": False})
    node.send_status(first["action"], first["command_id"], "SUCCEEDED", "line_forward_6")
    node.publish()
    latest = node.publisher.messages[-1]
    assert latest["source"] == "hurdle" and latest["action"] == "WAIT"
    assert not latest["valid"]
    assert node.planner.hurdle_lock_active
    assert not node.planner.hurdle_planner.fine_approach_active


def test_confirmation_during_line_gait_is_remembered_before_next_decision(clock):
    node, first, started, duration = start(clock, source="line")
    clock[0] = started + duration * .2
    observe(node, clock, hurdle(.8))
    assert node.planner.hurdle_lock_active
    assert not getattr(node, "hurdle_positioning_entry_pending", False)
    assert node.general_motion_gate.active_command_id == first["command_id"]
    assert node.queued_general_command_id is None
    clock[0] = started + duration
    node.receive_line()
    observe(node, clock, {"detected": False, "raw_detected": False})
    node.send_status(first["action"], first["command_id"], "SUCCEEDED", "line_forward_6")
    node.publish()
    latest = node.publisher.messages[-1]
    assert latest["source"] == "hurdle" and latest["action"] == "WAIT"
    assert node.planner.hurdle_lock_active


@pytest.mark.parametrize("blocked", [
    "ball", "goal", "special", "completed", "disabled", "ignore", "phase", "expired",
])
def test_callback_does_not_acquire_approach_lock_when_blocked(clock, blocked):
    node = HurdleHarness()
    if blocked in {"ball", "goal"}:
        setattr(node.planner, f"{blocked}_lock_active", True)
    elif blocked == "special":
        node.phase_manager.start_special_action("GO", 10)
    elif blocked == "completed":
        node.phase_manager.hurdles_completed = node.phase_manager.required_hurdles
    elif blocked == "disabled":
        node.planner.disable_completed_hurdle_missions()
    elif blocked == "ignore":
        node.planner.hurdle_ignore_until_clear = True
    elif blocked == "phase":
        node.phase_manager.set_phase("BALL_APPROACH")
    observe(node, clock, hurdle(.8), delay=.6 if blocked == "expired" else 0.)
    assert not node.planner.hurdle_lock_active
