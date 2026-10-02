"""Tests for robot-center calibration and corner geometry."""

import json

import numpy as np
import pytest
import rclpy
from std_msgs.msg import String

from step.yolo_line_analyzer import analyze_ground_line
from step.yolo_line_analyzer import GROUND_PROJECTION_DEFAULTS
from step.yolo_line_analyzer import project_line_points_to_ground

from step.yolo_line_analyzer import calibrated_robot_center_x
from step.yolo_line_analyzer import offset_reference_geometry
from step.yolo_line_analyzer import ground_forward_distance_from_depth
from step.yolo_line_analyzer import LinePoint
from step.yolo_line_analyzer import YoloLineAnalyzer


def test_1280_image_center_is_shifted_70_pixels_right():
    assert calibrated_robot_center_x(1280, 70.0) == pytest.approx(710.0)


def test_center_calibration_is_clipped_inside_image():
    assert calibrated_robot_center_x(1280, -1000.0) == 0.0
    assert calibrated_robot_center_x(1280, 1000.0) == 1279.0


@pytest.mark.parametrize("dx,expected", [(-128.6, -45.0), (0.0, 0.0), (128.6, 45.0)])
def test_offset_reference_angle_uses_calibrated_bottom_center(dx, expected):
    result = offset_reference_geometry(710.0 + dx, 590.4, 710.0, 1280, 720)
    assert result["offset_reference_valid"]
    assert result["offset_reference_steering_deg"] == pytest.approx(expected)


@pytest.mark.parametrize("x,y", [(1280., 590.), (-1., 590.), (710., 719.),
                                   (float("nan"), 590.), (710., float("inf"))])
def test_offset_reference_rejects_invisible_or_invalid_points(x, y):
    result = offset_reference_geometry(x, y, 710., 1280, 720)
    assert not result["offset_reference_valid"]
    assert result["offset_reference_steering_deg"] is None
    json.dumps(result, allow_nan=False)


def _corner_geometry(points):
    return YoloLineAnalyzer._detect_corner_start_geometry(
        points,
        min_points=3,
        min_segment_length_px=20.0,
        straight_max_turn_delta_deg=15.0,
        min_turn_delta_deg=30.0,
        onset_deviation_deg=15.0,
        min_consistent_segments=1,
        min_consistency=0.75,
    )


@pytest.mark.parametrize(
    ("far_x", "expected_direction"),
    [(600.0, "RIGHT"), (400.0, "LEFT")],
)
def test_three_points_find_corner_at_middle_point(far_x, expected_direction):
    points = [
        LinePoint(500.0, 700.0, 0.9),
        LinePoint(500.0, 600.0, 0.9),
        LinePoint(far_x, 600.0, 0.9),
    ]
    result = _corner_geometry(points)
    assert result["detected"] is True
    assert result["direction"] == expected_direction
    assert result["start_index"] == 1


def test_slanted_straight_line_never_creates_corner_preview():
    points = [
        LinePoint(500.0 + index * 20.0, 700.0 - index * 70.0, 0.9)
        for index in range(7)
    ]
    result = _corner_geometry(points)
    assert result["detected"] is False
    assert result["state"] == "STRAIGHT"


def test_floor_forward_projection_removes_camera_pitch_slant():
    depth_m = (0.50 + 0.70) / (2.0 ** 0.5)
    camera_down_m = (-0.50 + 0.70) / (2.0 ** 0.5)
    y_px = 360.0 + 1000.0 * camera_down_m / depth_m
    lateral_m, forward_m = ground_forward_distance_from_depth(
        x_px=640.0,
        y_px=y_px,
        depth_m=depth_m,
        fx=1000.0,
        fy=1000.0,
        cx=640.0,
        cy=360.0,
        camera_pitch_down_deg=45.0,
        camera_height_m=0.70,
        reference_height_m=0.0,
        camera_forward_offset_m=0.0,
    )
    assert lateral_m == pytest.approx(0.0)
    assert forward_m == pytest.approx(0.50)


# Measured with the camera/head fixed; the last row really is Z = 1.00 m.
CALIBRATION_PIXELS = [
    (418, 649), (711, 647), (1016, 648),
    (506, 386), (709, 380), (922, 377),
    (548, 277), (714, 274), (894, 270),
    (569, 198), (713, 192), (868, 192),
]


def _pixel_points(pixels):
    return [LinePoint(float(u), float(v), 0.9) for u, v in pixels]


def _pixels_from_ground(ground):
    homography = np.asarray(
        GROUND_PROJECTION_DEFAULTS["ground_homography"],
    ).reshape(3, 3)
    homogeneous = np.column_stack((ground, np.ones(len(ground))))
    pixels = homogeneous @ np.linalg.inv(homography).T
    return _pixel_points(pixels[:, :2] / pixels[:, 2:3])


def test_measured_ground_calibration_accuracy():
    expected = np.asarray([
        (x, z) for z in (0.4, 0.6, 0.8, 1.0) for x in (-0.2, 0.0, 0.2)
    ])
    actual = project_line_points_to_ground(
        _pixel_points(CALIBRATION_PIXELS), 1280, 720, GROUND_PROJECTION_DEFAULTS,
    )
    error = np.linalg.norm(actual - expected, axis=1)
    assert np.mean(error) < 0.015
    assert np.max(error) < 0.025


def test_half_resolution_preserves_ground_coordinates():
    full = project_line_points_to_ground(
        _pixel_points(CALIBRATION_PIXELS), 1280, 720, GROUND_PROJECTION_DEFAULTS,
    )
    half = project_line_points_to_ground(
        _pixel_points(np.asarray(CALIBRATION_PIXELS) / 2),
        640, 360, GROUND_PROJECTION_DEFAULTS,
    )
    np.testing.assert_allclose(half, full, atol=1e-12)


def test_parallel_translation_preserves_heading_but_changes_steering():
    slope = np.tan(np.deg2rad(12.0))
    z = np.linspace(0.4, 1.0, 7)
    offsets, steering = [], []
    for intercept in (-0.12, 0.0, 0.12):
        points = _pixels_from_ground(np.column_stack((slope * z + intercept, z)))
        result = analyze_ground_line(points, 1280, 720)
        assert result["ground_projection_valid"] is True
        assert result["ground_heading_error_deg"] == pytest.approx(12.0)
        assert result["ground_lateral_offset_m"] == pytest.approx(
            intercept / np.sqrt(1 + slope ** 2), abs=1e-12,
        )
        assert result["ground_steering_angle_deg"] == pytest.approx(
            np.rad2deg(np.arctan2(slope * 0.5 + intercept, 0.5)),
        )
        offsets.append(result["ground_lateral_offset_m"])
        steering.append(result["ground_steering_angle_deg"])
    assert offsets[0] < offsets[1] < offsets[2]
    assert steering[0] < steering[1] < steering[2]
    assert steering[0] < 0 < steering[2]


def test_center_tape_matches_reference_results():
    result = analyze_ground_line(
        _pixel_points(CALIBRATION_PIXELS[1::3]), 1280, 720,
    )
    assert result["ground_heading_error_deg"] == pytest.approx(-0.326, abs=0.001)
    assert result["ground_lateral_offset_m"] == pytest.approx(0.0015, abs=0.0001)
    assert result["ground_steering_angle_deg"] == pytest.approx(-0.151, abs=0.001)
    assert result["ground_fit_rmse_m"] == pytest.approx(0.0016, abs=0.0001)
    assert result["ground_fit_point_count"] == 4
    assert result["ground_calibration_image_width"] == 1280
    assert result["ground_calibration_image_height"] == 720


def test_outlier_and_extrapolated_points_are_excluded():
    z = np.linspace(0.4, 1.0, 7)
    expected = np.column_stack((0.2 * z - 0.1, z))
    ground = np.vstack((expected, [0.5, 0.7], [0.0, 0.2], [0.0, 1.3]))
    result = analyze_ground_line(_pixels_from_ground(ground), 1280, 720)
    assert result["ground_projection_valid"] is True
    assert result["ground_fit_point_count"] == 7
    np.testing.assert_allclose(result["ground_line_points_m"], expected, atol=1e-12)
    assert result["ground_fit_rmse_m"] < 1e-12


@pytest.mark.parametrize("points,parameters,width,height", [
    ([], {}, 1280, 720),
    (_pixel_points(CALIBRATION_PIXELS[1:3]), {}, 1280, 720),
    (_pixel_points([(711, 647)] * 3), {}, 1280, 720),
    (_pixel_points(CALIBRATION_PIXELS), {"ground_projection_enabled": False}, 1280, 720),
    (_pixel_points(CALIBRATION_PIXELS), {}, 0, 720),
    (_pixel_points(CALIBRATION_PIXELS), {}, 1280, 0),
    (_pixel_points(CALIBRATION_PIXELS), {"ground_homography": [1, 2]}, 1280, 720),
    (_pixel_points(CALIBRATION_PIXELS), {"ground_homography": [0.0] * 9}, 1280, 720),
    (_pixel_points(CALIBRATION_PIXELS), {"ground_homography": [float('nan')] * 9}, 1280, 720),
    (_pixel_points(CALIBRATION_PIXELS), {"ground_lookahead_m": 0.0}, 1280, 720),
    (_pixel_points(CALIBRATION_PIXELS), {"ground_fit_min_points": 20}, 1280, 720),
])
def test_invalid_ground_fit_keeps_all_keys_with_null_numbers(
    points, parameters, width, height,
):
    result = analyze_ground_line(points, width, height, parameters)
    assert result.keys() == analyze_ground_line([], 1280, 720).keys()
    assert result["ground_projection_valid"] is False
    assert result["ground_line_points_m"] == []
    assert result["ground_coordinate_frame"] == "robot_x_right_z_forward"
    metadata = {
        "ground_projection_enabled", "ground_projection_valid",
        "ground_coordinate_frame", "ground_line_points_m",
        "ground_fit_segment", "ground_fit_input_point_count",
        "ground_fit_reason", "ground_fit_projected_point_count",
    }
    assert all(value is None for key, value in result.items() if key not in metadata)
    json.dumps(result, allow_nan=False)


def test_nonfinite_pixels_and_near_zero_denominator_are_discarded():
    points = _pixel_points(CALIBRATION_PIXELS[1::3])
    points.extend(_pixel_points([(float('nan'), 300), (700, float('inf'))]))
    h = np.asarray(GROUND_PROJECTION_DEFAULTS["ground_homography"]).reshape(3, 3)
    horizon_v = -(h[2, 0] * 700 + h[2, 2]) / h[2, 1]
    points.append(LinePoint(700, horizon_v, 0.9))
    result = analyze_ground_line(points, 1280, 720)
    assert result["ground_projection_valid"] is True
    assert result["ground_fit_point_count"] == 4
    json.dumps(result, allow_nan=False)


def test_published_diagnostics_preserve_legacy_fields_and_clear_after_loss(monkeypatch):
    rclpy.init()
    nodes = []
    try:
        outputs = []
        for enabled in (False, True):
            node = YoloLineAnalyzer()
            nodes.append(node)
            node.ground_projection_parameters["ground_projection_enabled"] = enabled
            published = []
            monkeypatch.setattr(node.publisher, "publish", lambda msg: published.append(
                json.loads(msg.data)))
            detections = [
                {"class_name": "line", "confidence": 0.9,
                 "bbox": [u - 5, v - 5, u + 5, v + 5], "center": [u, v]}
                for u, v in CALIBRATION_PIXELS[1::3]
            ]
            message = String(data=json.dumps({
                "image_width": 1280, "image_height": 720, "detections": detections,
                "stamp": {"sec": 123, "nanosec": 456},
            }))
            for _ in range(3):
                node._detections_callback(message)
            outputs.append(published[-1])
            geometry = published[-1]
            assert geometry["offset_reference_valid"]
            assert geometry["offset_reference_y_px"] == pytest.approx(720 * 0.82)
            assert geometry["offset_reference_x_px"] - geometry["robot_center_x_px"] == pytest.approx(
                geometry["lateral_offset_px"], abs=0.002)
            assert geometry["offset_reference_steering_deg"] is not None
            assert published[-1]["stamp"] == {"sec": 123, "nanosec": 456}
            node._detections_callback(String(data=json.dumps({"detections": []})))
            assert published[-1]["stamp"] is None
            assert published[-1]["ground_projection_valid"] is False
            assert published[-1]["ground_heading_error_deg"] is None
            assert published[-1]["offset_reference_valid"] is False
            assert published[-1]["offset_reference_steering_deg"] is None
            assert published[-1]["offset_reference_x_px"] is None
        assert outputs[1]["detected"] is True
        assert outputs[0]["ground_projection_valid"] is False
        assert outputs[1]["ground_projection_valid"] is True

        def legacy(output):
            return {
                key: value for key, value in output.items()
                if not key.startswith("ground_") and key != "processing_ms"
            }

        assert legacy(outputs[0]) == legacy(outputs[1])
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()
