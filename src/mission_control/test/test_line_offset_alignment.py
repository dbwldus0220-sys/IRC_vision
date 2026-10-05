"""Keep image offsets separate from physical heading correction."""

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
        **line_info(heading=20.0, offset=0.7),
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


@pytest.mark.parametrize("heading,direction,count,yaw", [
    (10.01, "RIGHT", 2, 15.), (29.999, "RIGHT", 2, 15.),
    (30., "RIGHT", 3, 30.), (45., "RIGHT", 5, 45.),
    (-10.01, "LEFT", 1, 15.), (-30., "LEFT", 2, 30.),
    (-45., "LEFT", 3, 45.),
])
@pytest.mark.parametrize("image_point_angle", [-80., 0., 80., None])
def test_turn_count_uses_heading_not_image_point(heading, direction, count, yaw, image_point_angle):
    data = sample(image_point_angle, ground_heading_error_deg=heading)
    decision = planner().plan("LINE_TRACK", {"line": data}, .1)
    action = f"LINE_HEADING_TURN_{direction}_{count}"
    assert decision.valid and decision.action == action
    assert decision.source_command["turn_angle_deg"] == yaw
    assert decision.source_command["alignment_reference"] == "ground_heading"
    assert decision.source_command["linear_speed_mps"] == 0.
    assert decision.source_command["lateral_speed_mps"] == 0.
    assert normalize_general_action(action) == action
    assert MotionCommandBridgeNode.motion_id_for_action(action) == (
        f"post_ball_line_turn_{direction.lower()}_{count}")
    assert MotionDecisionNode._needs_correction_dwell(action)


@pytest.mark.parametrize("phase", ["AUTO", "LINE_TRACK", "LINE_TRACK_AFTER_PICKUP"])
def test_recorded_aligned_left_offset_uses_short_forward(phase):
    data = sample(-50.675, -156.98, ground_heading_error_deg=-5.57,
                  filtered_heading_error_deg=6.7, heading_error_deg=6.7,
                  filtered_lateral_offset_norm=-.245, turn_angle_deg=12.1)
    decision = planner().plan(phase, {"line": data}, .1)
    assert decision.valid and decision.action == "STRAIGHT_1"
    assert decision.reason == "line_offset_short_forward"
    assert decision.source_command["target_heading_change_deg"] == 0.
    assert decision.source_command["recovery_side"] is None
    assert decision.source_command["turn_angle_deg"] is None
    assert decision.source_command["turn_count"] is None
    assert decision.source_command["offset_reference_turn_count"] == 0
    assert MotionCommandBridgeNode.motion_id_for_action(decision.action) == "line_forward_2"


@pytest.mark.parametrize("heading", [-10., 0., 10.])
@pytest.mark.parametrize("offset", [-500., -100.1, 100.1, 500.])
def test_aligned_offset_never_requests_stationary_yaw(heading, offset):
    data = sample(80., offset, ground_heading_error_deg=heading,
                  filtered_heading_error_deg=heading)
    decision = planner().plan("LINE_TRACK", {"line": data}, .1)
    assert decision.valid and decision.action == "STRAIGHT_1"


@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("offset", [99.9, 100., 100.1])
def test_pixel_limit_only_selects_short_forward_when_heading_aligned(sign, offset):
    data = sample(sign * 80., sign * offset, ground_heading_error_deg=0.,
                  filtered_heading_error_deg=0., filtered_lateral_offset_norm=0.)
    decision = planner().plan("LINE_TRACK", {"line": data}, .1)
    assert decision.valid
    assert decision.action == ("STRAIGHT_1" if offset > 100. else "STRAIGHT")


def test_disabled_policy_preserves_original_recovery():
    data = sample(-80.)
    assert planner(-1.).plan("LINE_TRACK", {"line": data}, .1).action == "RECOVER_RIGHT_TURN_RIGHT_4"


@pytest.mark.parametrize("changes", [
    {"ground_projection_valid": False}, {"ground_heading_error_deg": None},
    {"ground_heading_error_deg": float("nan")}, {"lateral_offset_px": None},
    {"heading_quality": .1}, {"detected": False},
])
def test_invalid_geometry_cannot_start_alignment(changes):
    assert not planner().plan("LINE_TRACK", {"line": sample(**changes)}, .1).valid


def test_raw_cross_line_still_vetoes_short_forward():
    data = sample(80., ground_heading_error_deg=5., heading_error_deg=55.)
    decision = planner().plan("LINE_TRACK", {"line": data}, .1)
    assert not decision.valid and decision.reason == "straight_heading_not_aligned"


def test_candidate_generation_preserves_base_planner_state():
    p = planner()
    base = p.line_planner
    base.previous_motion = "RIGHT"
    base.previous_angular_speed_rad_s = .2
    base.turn_candidate = "LEFT"
    base.turn_candidate_hits = 2
    result = p._line_offset_alignment(sample(), {"valid": True, "motion": "RIGHT"})
    assert result["motion"] == "LINE_HEADING_TURN_RIGHT_2"
    assert (base.previous_motion, base.previous_angular_speed_rad_s,
            base.turn_candidate, base.turn_candidate_hits) == ("RIGHT", .2, "LEFT", 2)


def test_recorded_stop_loop_finishes_settle_with_unchanged_geometry(monkeypatch):
    clock = [10.]
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: clock[0])
    node = LiveInputHarness(phase="AUTO")
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = 1.
    data = sample(67.6, 311.6, ground_heading_error_deg=16.43,
                  heading_error_deg=-5.76, filtered_heading_error_deg=-5.76,
                  filtered_lateral_offset_norm=.481, turn_angle_deg=40.6)
    for i in range(20):
        clock[0] = 10. + i * .1
        observe(node, clock, "line", data)
    commands = [m for m in node.publisher.messages if m["valid"]]
    assert len(commands) == 1
    assert commands[0]["action"] == "LINE_HEADING_TURN_RIGHT_2"
    assert commands[0]["source_command"]["turn_angle_deg"] == 15.


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
    assert command["action"] == "LINE_HEADING_TURN_RIGHT_2"
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
