"""Check offset-point steering independently of the local line slope."""

from dataclasses import replace

import pytest

from mission_control.motion_decision_planner import MotionDecisionConfig, MotionDecisionPlanner
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
from mission_control.motion_command_gate import normalize_general_action
from mission_control.motion_decision_node import MotionDecisionNode
from test_mission_phase_flow import line_info
from test_wait_refresh_and_fine_settle import LiveInputHarness, observe


def sample(angle=40.0, offset=200.0, **changes):
    data = {
        **line_info(heading=-15.0, offset=0.7),
        "lateral_offset_px": offset,
        "offset_reference_valid": True,
        "offset_reference_x_px": 710.0 + offset,
        "offset_reference_y_px": 590.4,
        "offset_reference_steering_deg": angle,
    }
    data.update(changes)
    return data


def planner(threshold=100.0):
    return MotionDecisionPlanner(MotionDecisionConfig(line_offset_align_enter_px=threshold))


@pytest.mark.parametrize("angle,direction,count,yaw", [
    (15., "RIGHT", 2, 15.), (29.999, "RIGHT", 2, 15.),
    (30., "RIGHT", 3, 30.), (40., "RIGHT", 3, 30.),
    (45., "RIGHT", 5, 45.), (65., "RIGHT", 7, 65.),
    (89., "RIGHT", 7, 65.), (-15., "LEFT", 1, 15.),
    (-30., "LEFT", 2, 30.), (-45., "LEFT", 3, 45.),
    (-89., "LEFT", 5, 75.),
])
def test_turn_count_uses_offset_point_angle_and_catalog(angle, direction, count, yaw):
    decision = planner().plan("LINE_TRACK", {"line": sample(angle)}, .1)
    action = f"LINE_OFFSET_TURN_{direction}_{count}"
    assert decision.valid and decision.action == action
    assert decision.source_command["turn_count"] == count
    assert decision.source_command["turn_angle_deg"] == yaw
    assert decision.source_command["linear_speed_mps"] == 0.
    assert decision.source_command["lateral_speed_mps"] == 0.
    assert normalize_general_action(action) == action
    assert MotionCommandBridgeNode.motion_id_for_action(action) == (
        f"post_ball_line_turn_{direction.lower()}_{count}")
    assert MotionDecisionNode._needs_correction_dwell(action)


def test_right_point_overrides_left_recovery_from_line_slope():
    data = sample()
    assert planner(-1.).plan("LINE_TRACK", {"line": data}, .1).action == "RECOVER_RIGHT_TURN_LEFT_4"
    assert planner().plan("LINE_TRACK", {"line": data}, .1).action == "LINE_OFFSET_TURN_RIGHT_3"


@pytest.mark.parametrize("angle,offset", [(0., 200.), (14.999, 200.), (40., 100.)])
def test_aligned_or_inside_limit_returns_to_normal_decision(angle, offset):
    data = sample(angle, offset)
    assert planner().plan("LINE_TRACK", {"line": data}, .1).action == (
        planner(-1.).plan("LINE_TRACK", {"line": data}, .1).action)


@pytest.mark.parametrize("changes", [
    {"offset_reference_valid": False}, {"offset_reference_steering_deg": None},
    {"offset_reference_steering_deg": float("nan")},
    {"offset_reference_steering_deg": 90.}, {"lateral_offset_px": None},
    {"heading_quality": 0.1}, {"detected": False},
])
def test_invalid_geometry_cannot_start_alignment(changes):
    decision = planner().plan("LINE_TRACK", {"line": sample(**changes)}, .1)
    assert not decision.valid


def test_default_applies_offset_alignment():
    decision = MotionDecisionPlanner().plan("LINE_TRACK", {"line": sample()}, .1)
    assert decision.valid and decision.action == "LINE_OFFSET_TURN_RIGHT_3"
    assert decision.source_command["offset_reference_steering_deg"] == 40.
    assert decision.source_command["offset_reference_turn_count"] == 3


def test_recorded_large_left_offset_uses_stationary_left_turn_by_default():
    data = sample(
        -75.8, -507.9, ground_heading_error_deg=31.99,
        filtered_heading_error_deg=49.3, filtered_lateral_offset_norm=-.795,
        turn_angle_deg=3.6,
    )
    assert planner(-1.).plan("LINE_TRACK", {"line": data}, .1).action == (
        "RECOVER_LEFT_TURN_RIGHT_4")
    decision = MotionDecisionPlanner().plan("LINE_TRACK", {"line": data}, .1)
    assert decision.valid and decision.action == "LINE_OFFSET_TURN_LEFT_5"
    assert decision.source_command["turn_angle_deg"] == 75.
    assert decision.source_command["linear_speed_mps"] == 0.


@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("offset", [99.9, 100., 100.1])
def test_default_pixel_limit_boundary(sign, offset):
    data = sample(sign * 40., sign * offset)
    decision = MotionDecisionPlanner().plan("LINE_TRACK", {"line": data}, .1)
    assert decision.valid
    assert decision.action.startswith("LINE_OFFSET_TURN_") == (offset > 100.)


@pytest.mark.parametrize("changes", [
    {"offset_reference_valid": False}, {"lateral_offset_px": None},
])
def test_enabled_default_stops_on_missing_alignment_geometry(changes):
    decision = MotionDecisionPlanner().plan("LINE_TRACK", {"line": sample(**changes)}, .1)
    assert not decision.valid
    assert decision.reason == "line_offset_alignment_invalid_reference"


@pytest.mark.parametrize("stamp", [None, True, 9_000_000_000, 10_000_000_000])
def test_alignment_refresh_rejects_frames_captured_before_boundary(monkeypatch, stamp):
    clock = [10.1]
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: clock[0])
    node = LiveInputHarness(phase="LINE_TRACK")
    node.line_offset_min_rgb_stamp_ns = 10_000_000_000
    observe(node, clock, "line", sample(rgb_stamp_ns=stamp), plan=False)
    assert node.latest_info["line"] is None
    observe(node, clock, "line", sample(rgb_stamp_ns=10_100_000_000), plan=False)
    assert node.latest_info["line"]["rgb_stamp_ns"] == 10_100_000_000


def test_alignment_dwell_sets_capture_boundary(monkeypatch):
    clock = [10.]
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: clock[0])
    monkeypatch.setattr(MotionDecisionNode, "_current_ros_time_ns", lambda self: 12_000_000_000)
    node = LiveInputHarness(phase="LINE_TRACK")
    node.correction_post_motion_dwell_until = 11.
    node.correction_post_motion_source = "line"
    node.line_offset_alignment_refresh_pending = True
    clock[0] = 12.
    MotionDecisionNode._publish_decision(node)
    assert node.line_offset_min_rgb_stamp_ns == 12_000_000_000
    assert not node.line_offset_alignment_refresh_pending


def test_turn_completion_waits_then_replans_new_frame(monkeypatch):
    clock = [10.]
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: clock[0])
    node = LiveInputHarness(phase="LINE_TRACK")
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = MotionDecisionNode.LINE_TURN_PRE_MOTION_SETTLE_SEC
    node.planner.config = replace(node.planner.config, line_offset_align_enter_px=100.)
    observe(node, clock, "line", sample())
    assert not node.publisher.messages
    clock[0] += 1.1
    observe(node, clock, "line", sample())
    command = node.publisher.messages[-1]
    assert command["action"] == "LINE_OFFSET_TURN_RIGHT_3"
    node.send_status(command["action"], command["command_id"], "RUNNING")
    node.send_status(command["action"], command["command_id"], "SUCCEEDED")
    deadline = node.correction_post_motion_dwell_until
    assert deadline > clock[0]
    before = len(node.publisher.messages)
    observe(node, clock, "line", sample())
    assert len(node.publisher.messages) == before
    clock[0] = deadline + .01
    MotionDecisionNode._publish_decision(node)
    assert node.latest_info["line"] is None
    observe(node, clock, "line", sample(0., 0., filtered_heading_error_deg=0.,
                                       ground_heading_error_deg=0.,
                                       filtered_lateral_offset_norm=0.))
    assert node.publisher.messages[-1]["action"] == "STRAIGHT"


@pytest.mark.parametrize("robot", [False, True])
@pytest.mark.parametrize("override", [None, "-1.0", "180.0"])
def test_launch_exposes_pixel_threshold(monkeypatch, tmp_path, robot, override):
    from launch_ros.actions import Node
    from test_full_system_launch import node_parameters
    if robot:
        from test_full_system_robot_launch import launch_description, default_context
    else:
        from test_full_system_launch import launch_description, launch_context as default_context
    description = launch_description(monkeypatch, tmp_path)
    context = default_context(description)
    assert context.launch_configurations["line_offset_align_enter_px"] == "100.0"
    if override is not None:
        context.launch_configurations["line_offset_align_enter_px"] = override
    node = next(e for e in description.entities if isinstance(e, Node)
                and e.node_executable == "motion_decision_node")
    expected = 100. if override is None else float(override)
    assert node_parameters(node, context)["line_offset_align_enter_px"] == expected
