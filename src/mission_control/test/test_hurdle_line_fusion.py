"""Tests for hurdle-owned line-intersection geometry."""

import pytest

from mission_control.hurdle_line_fusion import build_hurdle_path_reference
from mission_control.motion_decision_planner import MotionDecisionPlanner
from test_hurdle_positioning import hurdle, line_info


def test_route_line_is_extended_to_hurdle_lower_edge():
    hurdle = {
        "detected": True,
        "bbox": [500, 300, 900, 500],
        "image_width": 1280,
        "image_height": 720,
    }
    line = {
        "detected": True,
        "robot_center_x_px": 710.0,
        "center_points_px": [[800, 650], [800, 580], [800, 250], [800, 180]],
    }
    result = build_hurdle_path_reference(hurdle, line)
    assert result["path_reference_valid"] is True
    assert result["path_reference_point_px"] == [800, 500]
    assert result["path_offset_x_norm"] == pytest.approx(90 / 640)
    assert result["path_direction"] == "RIGHT"
    assert result["line_mode_allowed"] is False


def test_points_hidden_inside_hurdle_are_not_used_for_fit():
    hurdle = {
        "detected": True,
        "bbox": [500, 300, 900, 500],
        "image_width": 1280,
        "image_height": 720,
    }
    line = {"detected": True, "center_points_px": [[600, 450], [800, 430]]}
    result = build_hurdle_path_reference(hurdle, line)
    assert result["path_reference_valid"] is False
    assert result["path_reference_reason"] == "insufficient_unoccluded_line_points"


def test_consistent_points_rejected_across_occlusion_can_be_recovered():
    hurdle = {
        "detected": True,
        "bbox": [500, 300, 900, 500],
        "image_width": 1280,
        "image_height": 720,
    }
    line = {
        "detected": True,
        "robot_center_x_px": 710.0,
        "center_points_px": [[800, 650]],
        "path_rejected_points_px": [[800, 580], [800, 250]],
    }
    result = build_hurdle_path_reference(hurdle, line)
    assert result["path_reference_valid"] is True
    assert result["path_reference_point_px"] == [800, 500]
    assert len(result["path_support_points_px"]) == 3


@pytest.mark.parametrize("x", [200, 500, 900, 1100])
def test_diagnostic_line_heading_does_not_override_calibrated_hurdle_yaw(x):
    result = build_hurdle_path_reference(hurdle(.8), line_info(x=x, heading=25))
    assert result["line_heading_valid"]
    assert result["line_heading_deg"] == 25
    assert result["path_reference_valid"] == (400 <= x <= 1000)
    decision = MotionDecisionPlanner().plan("AUTO", {
        "hurdle": hurdle(.8), "line": line_info(x=x, heading=25),
    }, .1)
    assert decision.action == "STRAIGHT_1"
    assert "turn_count" not in decision.source_command


def test_filtered_image_heading_is_preferred_over_raw_and_ground_heading():
    line = {**line_info(heading=25), "heading_error_deg": -40,
            "ground_heading_error_deg": -60, "ground_fit_valid": True}
    result = build_hurdle_path_reference(hurdle(.6), line)
    assert result["line_heading_deg"] == 25
    line["filtered_heading_error_deg"] = None
    assert build_hurdle_path_reference(hurdle(.6), line)["line_heading_deg"] == -40


@pytest.mark.parametrize("updates", [
    {"detected": False}, {"heading_quality": .34}, {"geometry_quality": .34},
    {"filtered_heading_error_deg": None, "heading_error_deg": None},
    {"filtered_heading_error_deg": float("nan"), "heading_error_deg": float("inf")},
    {"filtered_heading_error_deg": 91},
])
def test_bad_line_heading_does_not_block_valid_hurdle_angle(updates):
    planner = MotionDecisionPlanner()
    info = hurdle(.8)
    decision = planner.plan("AUTO", {"hurdle": info, "line": {**line_info(), **updates}}, .1)
    assert decision.valid and decision.action == "STRAIGHT_1"
    resumed = planner.plan("AUTO", {"hurdle": info, "line": line_info(heading=-30)}, .1)
    assert resumed.valid and resumed.action == "STRAIGHT_1"
    assert "turn_count" not in resumed.source_command


def test_held_intersection_does_not_reuse_old_heading():
    planner = MotionDecisionPlanner()
    planner._hurdle_observation_with_path(hurdle(.8), line_info(heading=40), .1)
    info, reference = planner._hurdle_observation_with_path(hurdle(.8), None, .1)
    assert reference["path_reference_source"] == "held"
    assert not reference["line_heading_valid"]
    assert planner.hurdle_planner.plan(info, positioning=True).action == "STRAIGHT_1"
    info, reference = planner._hurdle_observation_with_path(
        hurdle(.8), line_info(x=1100, heading=-30), .1,
    )
    assert reference["path_reference_source"] == "held"
    assert reference["line_heading_reference_source"] == "fresh"
    assert planner.hurdle_planner.plan(info, positioning=True).action == "STRAIGHT_1"
