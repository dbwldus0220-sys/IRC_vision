"""Diagnose queue rejection without changing candidate counting or publication."""

import json
from types import SimpleNamespace

import pytest

from mission_control.motion_decision_node import MotionDecisionNode as Node


@pytest.mark.parametrize("index", [1, 2])
@pytest.mark.parametrize("context", ["goal_walk", "line_to_goal"])
def test_candidate_changes_and_frame_shortage_are_distinct(monkeypatch, index, context):
    clock = [20.8]
    logs, published = [], []
    handoff = context == "line_to_goal"
    forward, recovery = "GOAL_CAMERA_90_FORWARD_2", "GOAL_CAMERA90_RECOVER_RIGHT_4"
    node = SimpleNamespace(
        mission_phase="LINE_TRACK_AFTER_PICKUP" if handoff else "GOAL_APPROACH",
        active_general_source="line" if handoff else "goal",
        active_special_command_id=None, queued_general_command_id=None,
        general_motion_gate=SimpleNamespace(locked=True, active_command_id=10,
                                           active_action="STRAIGHT" if handoff else forward),
        phase_manager=SimpleNamespace(completed_ball_index=index,
                                      grasp_result_for_ball=lambda _: "GRABBED"),
        latest_info={"goal": None}, latest_time={"goal": clock[0]},
        goal_forward_started_at=19., goal_forward_duration_sec=2.056,
        goal_forward_capture_after_ns=20_400_000_000,
        goal_forward_capture_count=0, goal_forward_capture_action=None,
        goal_forward_last_stamp_ns=None,
        active_line_motion_started_at=19., active_line_motion_duration_sec=3.084,
        post_ball_line_started_count=3, post_ball_goal_camera_ready_at=20.,
        post_ball_goal_preview=None, post_ball_goal_capture_count=0,
        post_ball_goal_last_stamp_ns=None, timeouts={"goal": 1.},
        get_logger=lambda: SimpleNamespace(info=logs.append),
        planner=SimpleNamespace(_is_detected_goal=lambda info: bool(info and info.get("detected")),
                                _plan_source=lambda source, info, dt: info["command"]),
    )
    node._fresh_observations = lambda now: (node.latest_info.copy(), {})
    monkeypatch.setattr(Node, "_current_ros_time_ns", lambda _: round(clock[0] * 1e9))

    def publish(self, decision, **kwargs):
        published.append(decision.action)
        self.queued_general_command_id = 11
        self.queued_general_action = decision.action

    monkeypatch.setattr(Node, "_publish_decision", publish)
    preview = Node._prepare_post_ball_goal_preview if handoff else Node._prepare_pending_goal_decision
    prefix = "post_ball_goal" if handoff else "goal_forward"

    def receive(action, **updates):
        clock[0] += .01
        node.latest_time["goal"] = clock[0]
        node.latest_info["goal"] = {
            "detected": True, "confirmation_confirmed": True,
            "confirmation_hits": 10, "confirmation_required_hits": 10,
            "rgb_stamp_ns": round(clock[0] * 1e9),
            "command": {"action": action, "valid": True, "reason": "test_geometry"},
            **updates,
        }
        preview(node, clock[0])

    receive(forward)
    receive(forward)
    assert not published and getattr(node, prefix + "_capture_count") == 2
    receive(recovery)
    changed = node.goal_prequeue_diagnostics[context]
    assert changed["reason"] == "candidate_changed"
    assert changed["previous_action"] == forward and changed["previous_frames"] == 2
    assert changed["frames"] == 1 and not published
    receive(recovery, detected=False, confirmation_confirmed=False, raw_detected=True,
            confirmation_hits=8)
    assert node.goal_prequeue_diagnostics[context]["reason"] == "goal_not_confirmed"
    assert node.goal_prequeue_diagnostics[context]["confirmation_hits"] == 8
    assert getattr(node, prefix + "_capture_count") == 0
    receive(recovery)
    preview(node, clock[0])
    assert getattr(node, prefix + "_capture_count") == 1 and not published
    receive(recovery)
    receive(recovery)
    assert published == [recovery]
    Node._log_goal_prequeue_end(node, context, 10, "SUCCEEDED")
    summary = json.loads(logs[-1].split("GOAL prequeue end: ", 1)[1])
    assert summary["queued_command_id"] == 11 and summary["ball_index"] == index
    assert any('"reason":"insufficient_candidate_frames"' in line for line in logs)


def test_diagnostic_throttling_does_not_hide_reason_changes(monkeypatch):
    logs = []
    node = SimpleNamespace(
        latest_info={}, latest_time={}, mission_phase="GOAL_APPROACH",
        phase_manager=SimpleNamespace(completed_ball_index=2),
        general_motion_gate=SimpleNamespace(active_command_id=10, active_action="GOAL_CAMERA_90_FORWARD_2"),
        get_logger=lambda: SimpleNamespace(info=logs.append),
    )
    monkeypatch.setattr(Node, "_current_ros_time_ns", lambda _: 1)
    Node._log_goal_prequeue(node, "goal_walk", "missing_or_stale_goal", 1.)
    Node._log_goal_prequeue(node, "goal_walk", "missing_or_stale_goal", 1.1)
    assert len(logs) == 1
    Node._log_goal_prequeue(node, "goal_walk", "goal_not_confirmed", 1.2)
    assert len(logs) == 2
    Node._log_goal_prequeue(node, "goal_walk", "goal_not_confirmed", 1.7)
    assert len(logs) == 3
