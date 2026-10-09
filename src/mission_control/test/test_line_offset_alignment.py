"""Approach a visible ground target when the line is outside the pixel corridor."""

import math

import pytest

from mission_control.motion_decision_planner import MotionDecisionConfig, MotionDecisionPlanner
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
from mission_control.motion_decision_node import MotionDecisionNode
from test_mission_phase_flow import line_info
from test_wait_refresh_and_fine_settle import LiveInputHarness, observe


def sample(angle=40.0, offset=200.0, *, target_angle=20.0, **changes):
    data = {
        **line_info(heading=20.0, offset=0.7),
        "lateral_offset_px": offset, "offset_reference_valid": True,
        "offset_reference_x_px": 710.0 + offset,
        "offset_reference_y_px": 590.4, "offset_reference_steering_deg": angle,
        **changes,
    }
    heading = math.radians(data.get("ground_heading_error_deg") or 0.0)
    slope = math.tan(heading)
    intercept = math.tan(math.radians(target_angle)) * .5 - slope * .5
    data.setdefault("ground_lateral_offset_m", intercept * math.cos(heading))
    data.setdefault("ground_lookahead_distance_m", .5)
    data.setdefault("ground_line_points_m", [[slope*z+intercept, z] for z in (.35, .5, .7)])
    data.setdefault("ground_fit_segment", "PRE_CORNER" if data.get("corner_preview_confirmed") else "FULL_PATH")
    return data


def planner(threshold=100.0):
    return MotionDecisionPlanner(MotionDecisionConfig(line_offset_align_enter_px=threshold))


@pytest.mark.parametrize("offset", [-600., -100., 100., 600.])
def test_pixel_offset_does_not_replace_forward(offset):
    info = sample(offset=offset, ground_heading_error_deg=0.,
                  filtered_lateral_offset_norm=0., target_angle=0.)
    decision = planner().plan("LINE_TRACK", {"line": info}, .1)
    assert decision.valid and decision.action == "STRAIGHT"
    assert not decision.source_command["line_offset_alignment_active"]


@pytest.mark.parametrize("sign", [-1, 1])
def test_recorded_near_line_recovers_instead_of_stationary_alignment(sign):
    info = sample(offset=sign * 242.199, target_angle=sign * 18.8804,
                  ground_heading_error_deg=sign * 9.03,
                  filtered_lateral_offset_norm=sign * .378,
                  heading_error_deg=-sign * 8.3, turn_angle_deg=3.3)
    baseline = planner(-1.).plan("LINE_TRACK", {"line": info}, .1)
    decision = planner().plan("LINE_TRACK", {"line": info}, .1)
    side = "RIGHT" if sign > 0 else "LEFT"
    assert decision.valid and decision.action == baseline.action == f"RECOVER_{side}_TURN_{side}_4"
    assert not decision.source_command["line_offset_alignment_active"]


@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("offset", [99.9, 100., 100.1])
def test_far_recovery_guard_keeps_inclusive_pixel_threshold(sign, offset):
    info = sample(offset=sign*offset, target_angle=sign*25.,
                  ground_heading_error_deg=-sign*10., filtered_lateral_offset_norm=sign*.7)
    decision = planner().plan("LINE_TRACK", {"line": info}, .1)
    assert decision.valid
    if offset < 100.:
        assert decision.action.startswith("RECOVER_")
    else:
        assert decision.action == ("LINE_OFFSET_TURN_RIGHT_2" if sign > 0 else "LINE_OFFSET_TURN_LEFT_1")
        assert abs(decision.source_command["ground_lateral_offset_m"]) > .10


@pytest.mark.parametrize("target", [-10., 0., 10.])
def test_far_recovery_guard_allows_short_forward_when_target_aligned(target):
    info = sample(target_angle=target, ground_heading_error_deg=-35., heading_error_deg=55.)
    decision = planner().plan("LINE_TRACK", {"line": info}, .1)
    assert decision.valid and decision.action == "STRAIGHT_1"
    assert decision.reason == "line_offset_target_forward"


@pytest.mark.parametrize("heading", [-60., 60.])
def test_large_ground_heading_is_not_reversed_by_offset_target(heading):
    info = sample(ground_heading_error_deg=heading, target_angle=-heading)
    decision = planner().plan("LINE_TRACK", {"line": info}, .1)
    assert decision.valid
    assert decision.action.startswith("LINE_HEADING_TURN_RIGHT" if heading > 0 else "LINE_HEADING_TURN_LEFT")


def test_guard_does_not_latch_when_next_frame_selects_normal_recovery():
    p = planner()
    first = p.plan("LINE_TRACK", {"line": sample(target_angle=25., ground_heading_error_deg=-10.)}, .1)
    assert first.action == "LINE_OFFSET_TURN_RIGHT_2"
    result = p.plan("LINE_TRACK", {"line": sample(target_angle=18.88, ground_heading_error_deg=9.03,
                      filtered_lateral_offset_norm=.378)}, .1)
    assert result.action == "RECOVER_RIGHT_TURN_RIGHT_4"
    assert not p.line_offset_alignment_active


@pytest.mark.parametrize("changes", [
    {"ground_projection_valid": False}, {"ground_heading_error_deg": None},
    {"ground_heading_error_deg": float("nan")}, {"heading_quality": .1},
    {"ground_line_points_m": None}, {"ground_line_points_m": [[0., .5]]},
    {"ground_line_points_m": [[0., .5], [0., .5], [0., .5]]},
    {"ground_lateral_offset_m": None}, {"ground_lookahead_distance_m": -1.},
    {"corner_preview_confirmed": True, "ground_fit_segment": "FULL_PATH"},
])
def test_unusable_ground_cannot_authorize_far_recovery(changes):
    info = sample(target_angle=25., ground_heading_error_deg=-10.)
    info.update(changes)
    decision = planner().plan("LINE_TRACK", {"line": info}, .1)
    assert not decision.valid


def test_target_is_clamped_to_observed_segment_before_corner():
    info = sample(target_angle=20., ground_heading_error_deg=-10.,
                  ground_line_points_m=[[.18, .35], [.18, .4], [.18, .45]],
                  ground_lookahead_distance_m=.5,
                  corner_preview_confirmed=True, ground_fit_segment="PRE_CORNER")
    result = planner().plan("LINE_TRACK", {"line": info}, .1)
    assert result.source_command["target_forward_m"] == .45


@pytest.mark.parametrize('held,raw,ground_valid,expected', [
    (True, False, True, True), (False, True, True, False),
    (True, True, True, False), (True, False, False, False),
])
def test_far_guard_uses_full_path_only_when_corner_is_held(held, raw, ground_valid, expected):
    info = sample(target_angle=0., ground_heading_error_deg=-35.,
                  corner_preview_confirmed=True, corner_preview_held=held,
                  corner_preview_raw_detected=raw, ground_fit_segment='FULL_PATH',
                  ground_projection_valid=ground_valid)
    result = planner().plan('LINE_TRACK', {'line': info}, .1)
    assert result.valid is expected
    if expected:
        assert result.action == 'STRAIGHT_1'


def test_failed_guard_cannot_be_bypassed_by_normal_forward():
    p = planner()
    p.line_offset_alignment_failed = True
    decision = p.plan('LINE_TRACK', {'line': sample(ground_heading_error_deg=0.,
                       filtered_lateral_offset_norm=0., target_angle=0.)}, .1)
    assert not decision.valid and decision.reason == 'line_offset_motion_failed'


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



def test_alignment_failure_status_latches_until_other_mission(monkeypatch):
    clock = [10.]
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: clock[0])
    monkeypatch.setattr(MotionDecisionNode, "_current_ros_time_ns", lambda _: round(clock[0] * 1e9))
    node = LiveInputHarness(phase="LINE_TRACK")
    info = sample(target_angle=0., ground_heading_error_deg=-30.)
    observe(node, clock, "line", {**info, "rgb_stamp_ns": 10_010_000_000})
    command = node.publisher.messages[-1]
    assert command["action"] == "STRAIGHT_1"
    node.send_status(command["action"], command["command_id"], "RUNNING")
    node.send_status(command["action"], command["command_id"], "FAILED")
    assert node.planner.line_offset_alignment_failed
    valid_count = sum(m["valid"] for m in node.publisher.messages)
    for _ in range(3):
        observe(node, clock, "line", {**info, "rgb_stamp_ns": round((clock[0] + .01)*1e9)})
    assert sum(m["valid"] for m in node.publisher.messages) == valid_count
    assert node.planner.line_offset_alignment_failed
