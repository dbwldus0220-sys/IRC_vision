"""Replay steep line headings through planning, motion mapping and fresh-frame gates."""

import pytest

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
from mission_control.motion_command_gate import normalize_general_action
from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_decision_planner import MotionDecisionConfig, MotionDecisionPlanner
from test_motion_decision_planner import line_info
from test_wait_refresh_and_fine_settle import LiveInputHarness, observe


@pytest.mark.parametrize("heading,direction,count,yaw", [
    (45.001, "RIGHT", 5, 45.), (45.32, "RIGHT", 5, 45.),
    (64.9, "RIGHT", 5, 45.), (65., "RIGHT", 7, 65.),
    (-45.001, "LEFT", 3, 45.), (-60., "LEFT", 4, 60.),
])
@pytest.mark.parametrize("offset", [-.7, 0., .7])
def test_large_ground_heading_precedes_recovery_curve_and_offset_alignment(
    heading, direction, count, yaw, offset,
):
    planner = MotionDecisionPlanner(MotionDecisionConfig(line_offset_align_enter_px=100.))
    info = line_info(
        ground_heading_error_deg=heading, filtered_lateral_offset_norm=offset,
        turn_angle_deg=-heading, offset_reference_valid=True,
        lateral_offset_px=300., offset_reference_steering_deg=-heading,
    )
    decision = planner.plan("LINE_TRACK", {"line": info}, .1)
    action = f"LINE_HEADING_TURN_{direction}_{count}"
    assert decision.valid and decision.action == action
    assert decision.source_command["alignment_reference"] == "ground_heading"
    assert decision.source_command["turn_angle_deg"] == yaw <= abs(heading)
    assert decision.source_command["linear_speed_mps"] == 0.
    assert decision.source_command["lateral_speed_mps"] == 0.
    assert normalize_general_action(action) == action
    assert MotionCommandBridgeNode.motion_id_for_action(action) == (
        f"post_ball_line_turn_{direction.lower()}_{count}")
    assert MotionDecisionNode._needs_correction_dwell(action)


@pytest.mark.parametrize("heading", [-45., 37.54, 45.])
def test_threshold_is_strictly_over_45_and_first_screenshot_stays_recovery(heading):
    direction = "RIGHT" if heading > 0 else "LEFT"
    decision = MotionDecisionPlanner().plan("LINE_TRACK", {"line": line_info(
        ground_heading_error_deg=heading, filtered_lateral_offset_norm=.61,
    )}, .1)
    assert decision.action == f"RECOVER_RIGHT_TURN_{direction}_4"


@pytest.mark.parametrize("updates", [
    {"ground_projection_valid": False}, {"ground_heading_error_deg": None},
    {"heading_quality": .1}, {"detected": False},
])
def test_large_heading_does_not_bypass_geometry_or_quality_checks(updates):
    info = line_info(ground_heading_error_deg=55.)
    info.update(updates)
    decision = MotionDecisionPlanner().plan("LINE_TRACK", {"line": info}, .1)
    assert not decision.valid


def test_recorded_right_corner_can_be_seen_without_selecting_corner_motion():
    planner = MotionDecisionPlanner()
    corner = dict(corner_preview_confirmed=True, corner_direction="RIGHT",
                  corner_start_depth_valid=True, corner_preview_held=False)
    far = line_info(
        **corner, ground_heading_error_deg=2.72, filtered_heading_error_deg=-1.4,
        filtered_lateral_offset_norm=.057, lateral_offset_px=44.4,
        turn_angle_deg=70.2, corner_start_distance_m=.62,
    )
    # Even reliable right preview cannot bypass the minimum local heading of 5 deg.
    decision = planner.plan("LINE_TRACK", {"line": far}, .1)
    assert decision.valid and decision.action == "STRAIGHT"
    assert decision.reason == "turn_approach_pending"

    near = {**far, "ground_projection_valid": False, "ground_heading_error_deg": None,
            "filtered_heading_error_deg": 19.7, "corner_start_distance_m": .43}
    decision = planner.plan("LINE_TRACK", {"line": near}, .1)
    assert not decision.valid and decision.reason == "invalid_ground_line_geometry"

    steep = {**far, "ground_heading_error_deg": 75.83,
             "filtered_heading_error_deg": 63.2, "filtered_lateral_offset_norm": .639,
             "lateral_offset_px": 430.1, "offset_reference_valid": True,
             "offset_reference_steering_deg": 73.4, "turn_angle_deg": -13.7,
             "corner_start_distance_m": .1}
    decision = planner.plan("LINE_TRACK", {"line": steep}, .1)
    # Even a close confirmed corner does not override the earlier >45-deg branch.
    assert decision.valid and decision.action == "LINE_HEADING_TURN_RIGHT_7"
    assert decision.reason == "line_large_ground_heading"


@pytest.mark.parametrize("side", [-1, 1])
def test_raw_cross_line_blocks_forward_even_when_filtered_and_ground_angles_are_small(side):
    decision = MotionDecisionPlanner().plan("LINE_TRACK", {"line": line_info(
        ground_heading_error_deg=0., filtered_heading_error_deg=0.,
        heading_error_deg=side * 55.6,
    )}, .1)
    assert not decision.valid
    assert decision.reason == "straight_heading_not_aligned"


def test_stationary_turn_waits_before_execution_and_requires_post_dwell_capture(monkeypatch):
    clock = [10.]
    monkeypatch.setattr("mission_control.motion_decision_node.time.monotonic", lambda: clock[0])
    node = LiveInputHarness(phase="LINE_TRACK")
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = 1.
    info = line_info(ground_heading_error_deg=45.32)
    observe(node, clock, "line", info)
    assert not node.publisher.messages
    clock[0] += 1.1
    observe(node, clock, "line", info)
    command = node.publisher.messages[-1]
    assert command["action"] == "LINE_HEADING_TURN_RIGHT_5"
    node.send_status(command["action"], command["command_id"], "RUNNING")
    node.send_status(command["action"], command["command_id"], "SUCCEEDED")
    assert node.correction_post_motion_dwell_until >= clock[0] + 1.
    assert node.latest_info["line"] is None
    assert node.line_offset_alignment_refresh_pending
