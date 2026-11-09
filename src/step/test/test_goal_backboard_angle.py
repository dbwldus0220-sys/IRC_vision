"""Backboard left/center/right depth and relative-yaw tests."""

import math
import time

import numpy as np
import pytest

from step.goal_analyzer import GoalAnalyzer
from test_goal_backboard_selection import detection, make_analyzer, send


def _geometry_analyzer() -> GoalAnalyzer:
    analyzer = object.__new__(GoalAnalyzer)
    analyzer.depth_timeout_sec = 0.7
    analyzer.depth_window_px = 3
    analyzer.max_valid_depth_m = 6.0
    analyzer.backboard_line_width_m = 0.155
    analyzer.backboard_sample_y_ratio = 0.18
    analyzer.backboard_edge_inset_ratio = 0.10
    analyzer.backboard_parallel_tolerance_deg = 3.0
    analyzer.camera_to_waist_depth_offset_m = 0.065
    analyzer.camera_to_waist_lateral_offset_m = 0.035
    analyzer.backboard_to_ring_center_forward_m = 0.10
    analyzer.waist_aim_deadband_deg = 1.0
    analyzer.fx = 600.0
    analyzer.cx = 640.0
    return analyzer


def test_line_depths_are_sampled_at_same_height_and_scaled_to_depth_image():
    analyzer = _geometry_analyzer()
    depth = np.full((50, 100), 1000, dtype=np.uint16)
    # RGB bbox [20, 20, 180, 80] at 200x100 maps to these depth pixels.
    for x, millimeters in ((18, 400), (50, 500), (82, 600)):
        depth[14:19, x - 2:x + 3] = millimeters
    analyzer.latest_depth_image = depth
    analyzer.latest_depth_time = time.monotonic()

    left, center, right, count, points = (
        analyzer._sample_backboard_line_depths(
            [20, 20, 180, 80],
            200,
            100,
        )
    )

    assert points == [[36, 31], [100, 31], [164, 31]]
    assert count == 3
    assert left == pytest.approx(0.4)
    assert center == pytest.approx(0.5)
    assert right == pytest.approx(0.6)


def test_known_155mm_line_recovers_positive_right_far_angle():
    analyzer = _geometry_analyzer()
    expected_angle = 20.0
    sampled_width = 0.155
    depth_delta = sampled_width * math.sin(math.radians(expected_angle))
    depths = (
        0.5 - depth_delta / 2.0,
        0.5,
        0.5 + depth_delta / 2.0,
    )

    angle, correction, direction, fit_error, measured_width = (
        analyzer._estimate_backboard_angle(
            [[560, 260], [640, 260], [720, 260]],
            depths,
        )
    )

    assert angle == pytest.approx(expected_angle)
    assert correction == pytest.approx(-expected_angle)
    assert direction == "RIGHT_FAR"
    assert fit_error == pytest.approx(0.0, abs=1e-12)
    assert measured_width is not None


def test_angle_requires_all_three_depth_samples():
    analyzer = _geometry_analyzer()
    result = analyzer._estimate_backboard_angle(
        [[560, 260], [640, 260], [720, 260]],
        (0.48, None, 0.52),
    )
    assert result == (None, None, "UNKNOWN", None, None)


def test_waist_aim_uses_65mm_forward_camera_offset():
    analyzer = _geometry_analyzer()

    waist_depth, waist_aim, direction = analyzer._waist_center_aim(
        0.43,
        -0.197,
    )

    assert waist_depth == pytest.approx(0.495)
    assert waist_aim == pytest.approx(
        math.degrees(math.atan2(-0.197 - 0.035, 0.495))
    )
    assert direction == "TURN_LEFT"


def test_ring_center_aim_uses_radius_and_board_yaw():
    analyzer = _geometry_analyzer()

    ring_x, ring_z, waist_x, waist_z, aim, direction = (
        analyzer._waist_ring_aim(
            0.5,
            0.0,
            30.0,
        )
    )

    assert ring_x == pytest.approx(0.10 * math.sin(math.radians(30.0)))
    assert ring_z == pytest.approx(
        0.5 - 0.10 * math.cos(math.radians(30.0))
    )
    assert waist_x == pytest.approx(ring_x - 0.035)
    assert waist_z == pytest.approx(ring_z + 0.065)
    assert aim == pytest.approx(math.degrees(math.atan2(waist_x, waist_z)))
    assert direction == "TURN_RIGHT"


def test_lateral_camera_mount_is_distance_dependent_not_fixed_angle():
    analyzer = _geometry_analyzer()

    _, _, waist_x, waist_z, aim, direction = analyzer._waist_ring_aim(
        0.5,
        0.0,
        0.0,
    )

    assert waist_x == pytest.approx(-0.035)
    assert waist_z == pytest.approx(0.5 - 0.10 + 0.065)
    assert aim == pytest.approx(math.degrees(math.atan2(-0.035, waist_z)))
    assert direction == "TURN_LEFT"


def test_goal_info_publishes_three_depths_and_relative_angle():
    analyzer, published = make_analyzer(depth_m=0.5)
    sampled_width = 0.155
    depth_delta = sampled_width * math.sin(math.radians(-15.0))
    points = [[576, 222], [640, 222], [704, 222]]
    analyzer._sample_backboard_line_depths = lambda *_args: (
        0.5 - depth_delta / 2.0,
        0.5,
        0.5 + depth_delta / 2.0,
        3,
        points,
    )

    info = send(analyzer, published, [detection()])

    assert info["left_depth_m"] == pytest.approx(0.5201)
    assert info["center_depth_m"] == pytest.approx(0.5)
    assert info["right_depth_m"] == pytest.approx(0.4799)
    assert info["backboard_depth_sample_count"] == 3
    assert info["backboard_sample_points_px"] == points
    assert info["backboard_angle_deg"] == pytest.approx(-15.0)
    assert info["parallel_correction_deg"] == pytest.approx(15.0)
    assert info["backboard_angle_direction"] == "LEFT_FAR"
    assert info["backboard_angle_valid"] is True
    assert info["backboard_line_width_m"] == pytest.approx(0.155)
    assert info["camera_to_waist_depth_offset_m"] == pytest.approx(0.065)
    assert info["camera_to_waist_lateral_offset_m"] == pytest.approx(0.035)
    assert info["waist_depth_m"] == pytest.approx(0.565)
    assert info["waist_aim_deg"] == pytest.approx(
        math.degrees(math.atan2(-0.035, 0.565)), abs=1e-3
    )
    assert info["waist_turn_direction"] == "TURN_LEFT"
    assert info["backboard_to_ring_center_forward_m"] == pytest.approx(0.10)
    assert info["ring_center_lateral_m"] == pytest.approx(
        0.10 * math.sin(math.radians(-15.0)), abs=1e-4
    )
    assert info["ring_center_depth_m"] == pytest.approx(
        0.5 - 0.10 * math.cos(math.radians(-15.0)), abs=1e-4
    )
    assert info["waist_ring_depth_m"] == pytest.approx(
        info["ring_center_depth_m"] + 0.065, abs=1e-4
    )
    assert info["waist_ring_lateral_m"] == pytest.approx(
        info["ring_center_lateral_m"] - 0.035, abs=1e-4
    )
    assert info["waist_ring_aim_deg"] < 0.0
    assert info["waist_ring_turn_direction"] == "TURN_LEFT"
