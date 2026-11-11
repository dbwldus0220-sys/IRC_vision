"""Replay pickup handoff and restored corner approach without robot execution."""

import math
from dataclasses import replace

import pytest

from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from test_line_corner_memory import CornerHarness, corner_info, receive_line
from test_line_offset_alignment import sample


@pytest.fixture
def clock(monkeypatch):
    now = [10.0]
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: now[0])
    monkeypatch.setattr(Node, "_current_ros_time_ns", lambda _: round(now[0] * 1e9))
    return now


def scene(clock, *, distance=.5, bearing=0., heading=-27.72, offset=-149.9, **changes):
    return sample(
        offset=offset, target_angle=-15.82,
        **{**corner_info(ground_heading_error_deg=heading,
                      corner_start_distance_m=distance,
                      corner_start_lateral_offset_m=math.tan(math.radians(bearing)) * distance,
                      corner_preview_raw_detected=True,
                      rgb_stamp_ns=round(clock[0] * 1e9)), "lateral_offset_px": offset, **changes},
    )


def node_for(phase="LINE_TRACK"):
    node = CornerHarness(phase=phase)
    node.line_corner_turn_distance_m = .15
    node.planner.config = replace(
        node.planner.config, line_offset_align_enter_px=100.)
    return node


def publish(node, clock, **kwargs):
    return node.publish_vision(line=scene(clock, **kwargs))[-1]


@pytest.mark.parametrize("phase", ["AUTO", "LINE_TRACK", "LINE_TRACK_AFTER_PICKUP"])
@pytest.mark.parametrize("bearing", [-15., 0., 15.])
def test_corner_bearing_does_not_override_normal_forward(clock, phase, bearing):
    node = node_for(phase)
    info = scene(clock, bearing=bearing)
    decision = node.planner.plan(phase, {"line": info}, .1)
    node.latest_time['line'] = clock[0]
    result = Node._apply_pending_line_corner(node, decision, info)
    assert decision.action == "STRAIGHT"
    assert result == decision


@pytest.mark.parametrize("bearing", [-28.49, 28.49])
def test_far_corner_does_not_add_a_stationary_turn_toward_start_point(clock, bearing):
    node = node_for()
    result = publish(node, clock, distance=.2793, bearing=bearing,
                     heading=10., offset=0.)
    assert result["valid"] and result["action"] == "STRAIGHT_1"
    assert result["reason"] == "line_corner_turn_too_far"
    assert Bridge.motion_id_for_action(result["action"]) == "line_forward_2"


@pytest.mark.parametrize("direction", ["LEFT", "RIGHT"])
@pytest.mark.parametrize("distance,expected", [(.15, None), (.1501, "STRAIGHT_1")])
def test_corner_distance_gate_waits_for_normal_three_sample_turn_confirmation(
    clock, direction, distance, expected,
):
    node = node_for()
    node.planner.line_planner.config = replace(
        node.planner.line_planner.config, direction_confirmation_frames=3)
    actions = []
    for _ in range(3):
        clock[0] += .05
        info = corner_info(
            direction, corner_start_distance_m=distance,
            corner_start_lateral_offset_m=.3,
            rgb_stamp_ns=round(clock[0] * 1e9),
        )
        node.latest_time["line"] = clock[0]
        decision = node.planner.plan("LINE_TRACK", {"line": info}, .1)
        result = Node._apply_pending_line_corner(node, decision, info)
        assert result.valid
        actions.append(result.action)
    assert actions == ["STRAIGHT", "STRAIGHT", expected or direction]
    assert Bridge.motion_id_for_action(direction) == f"line_recovery_{direction.lower()}_4"


@pytest.mark.parametrize("heading", [-57.76, -71.18, 60.])
def test_pickup_reacquisition_preserves_normal_line_decision(clock, heading):
    node = node_for("POST_BALL_LINE_ALIGN")
    info = scene(clock, distance=.14, bearing=25., heading=heading, offset=477.)
    expected = node.planner.plan("LINE_TRACK_AFTER_PICKUP", {"line": info}, .1)
    result = node.publish_vision(line=info)[-1]
    assert node.mission_phase != "POST_BALL_LINE_ALIGN"
    assert result["valid"] and result["action"] == expected.action
    assert result["reason"] == expected.reason
    assert result["source_command"]["alignment_reference"] == "ground_heading"


def test_approach_reobserves_distance_and_queues_corner_without_interrupting_gait(clock):
    node = node_for()
    first = publish(node, clock, distance=.5, heading=10., offset=0.)
    node.send_status(first["action"], first["command_id"], "RUNNING")
    clock[0] += .8
    near = scene(clock, distance=.14, heading=10., offset=0.)
    receive_line(node, clock, near)
    decision = node.planner.plan("LINE_TRACK", {"line": near}, .1)
    Node._publish_decision(node, decision, queue_while_locked=True)
    queued = node.publisher.messages[-1]
    assert queued["action"] == "RIGHT" and queued["reason"] == "line_tracking"
    assert node.general_motion_gate.active_command_id == first["command_id"]
    assert node.queued_general_command_id == queued["command_id"]
    node.send_status(first["action"], first["command_id"], "SUCCEEDED")
    node.send_status(queued["action"], queued["command_id"], "RUNNING")
    assert node.general_motion_gate.active_action == "RIGHT"


@pytest.mark.parametrize("updates", [
    {"heading_quality": .1}, {"corner_preview_held": True},
    {"corner_start_depth_valid": False}, {"rgb_stamp_ns": 1},
])
def test_ready_corner_still_requires_fresh_valid_measurement(clock, updates):
    node = node_for()
    info = {**scene(clock, distance=.39), **updates}
    result = node.publish_vision(line=info)[-1]
    assert not (result["valid"] and result["action"] == "RIGHT")


def test_correction_failure_blocks_even_a_ready_corner(clock):
    node = node_for()
    node.planner.line_offset_alignment_failed = True
    result = publish(node, clock, distance=.39)
    assert not result["valid"]


def test_missing_corner_target_preserves_normal_forward(clock):
    node = node_for()
    info = scene(clock)
    info.pop("corner_start_lateral_offset_m")
    decision = node.planner.plan("LINE_TRACK", {"line": info}, .1)
    node.latest_time['line'] = clock[0]
    result = Node._apply_pending_line_corner(node, decision, info)
    assert result == decision
    assert result.action == "STRAIGHT"


def test_reacquired_line_does_not_interrupt_running_pickup_search(clock):
    node = node_for("POST_BALL_LINE_ALIGN")
    command = node.publish_vision(line={"detected": False, "rgb_stamp_ns": round(clock[0]*1e9)})[-1]
    node.send_status(command["action"], command["command_id"], "RUNNING")
    clock[0] += .1
    messages = node.publish_vision(line=scene(clock, distance=.14))
    assert not any(m["valid"] for m in messages)
    assert node.mission_phase == "POST_BALL_LINE_ALIGN"
    assert node.general_motion_gate.active_command_id == command["command_id"]


@pytest.mark.parametrize("distance", [.35, .15])
def test_corner_point_alone_cannot_authorize_walking_without_heading(clock, distance):
    node = node_for()
    node.line_corner_turn_distance_m = Node.LINE_CORNER_TURN_DISTANCE_M
    assert node.line_corner_turn_distance_m == .15
    result = publish(
        node, clock, distance=distance, ground_projection_enabled=True,
        ground_projection_valid=False, ground_fit_reason="too_few_segment_points",
        ground_fit_input_point_count=2, ground_fit_segment="PRE_CORNER",
    )
    # A corner point without a usable image segment cannot authorize alignment
    # and must no longer be reclassified as line loss.
    assert not result["valid"] and result["action"] == "WAIT"
