"""Validate diagnostic hurdle yaw reconstruction for both camera poses."""

import math

import cv2
import numpy as np
import pytest

from step.hurdle_analyzer import fit_hurdle_yaw_from_depth_samples
from step.hurdle_analyzer import HurdleAnalyzer
from step.yolo26_detector import extract_hurdle_bar_centerline


FX = 600.0
FY = 600.0
CX = 640.0
CY = 360.0


def projected_bar_samples(yaw_deg: float, pitch_down_deg: float) -> list[dict]:
    """Project one horizontal robot-frame bar into aligned RGB-depth samples."""
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_down_deg)
    samples = []
    for index, robot_x in enumerate((-0.2, -0.1, 0.0, 0.1, 0.2), start=1):
        robot_forward = 1.0 + math.tan(yaw) * robot_x
        robot_y_down = -0.15
        camera_y_down = (
            robot_y_down * math.cos(pitch)
            - robot_forward * math.sin(pitch)
        )
        camera_forward = (
            robot_y_down * math.sin(pitch)
            + robot_forward * math.cos(pitch)
        )
        samples.append({
            "index": index,
            "x_px": CX + FX * robot_x / camera_forward,
            "y_px": CY + FY * camera_y_down / camera_forward,
            "depth_m": camera_forward,
        })
    return samples


@pytest.mark.parametrize("pitch", [0.0, 38.4, 45.0, 68.3])
@pytest.mark.parametrize("yaw", [-30.0, 0.0, 30.0])
def test_matching_pitch_recovers_robot_frame_hurdle_yaw(pitch, yaw):
    """A matching optical pitch recovers the known robot-frame yaw."""
    result = fit_hurdle_yaw_from_depth_samples(
        projected_bar_samples(yaw, pitch),
        fx=FX, fy=FY, cx=CX, cy=CY, pitch_down_deg=pitch,
    )
    assert result["angle_deg"] == pytest.approx(yaw, abs=1e-3)
    assert result["point_count"] == 5
    assert result["rmse_m"] == pytest.approx(0.0, abs=1e-4)


def test_wrong_zero_pitch_hypothesis_underestimates_45_degree_camera_yaw():
    """The wrong pitch reproduces the expected yaw underestimation."""
    samples = projected_bar_samples(30.0, 45.0)
    zero_pose = fit_hurdle_yaw_from_depth_samples(
        samples, fx=FX, fy=FY, cx=CX, cy=CY, pitch_down_deg=0.0,
    )
    camera_45_pose = fit_hurdle_yaw_from_depth_samples(
        samples, fx=FX, fy=FY, cx=CX, cy=CY, pitch_down_deg=45.0,
    )
    assert zero_pose["angle_deg"] == pytest.approx(22.208, abs=1e-3)
    assert camera_45_pose["angle_deg"] == pytest.approx(30.0, abs=1e-3)


def test_camera_yaw_mount_correction_is_visible_in_diagnostic_angle():
    samples = projected_bar_samples(30.0, 0.0)
    corrected = fit_hurdle_yaw_from_depth_samples(
        samples,
        fx=FX,
        fy=FY,
        cx=CX,
        cy=CY,
        pitch_down_deg=0.0,
        yaw_right_deg=5.0,
    )
    assert corrected["angle_deg"] == pytest.approx(25.0, abs=1e-3)


def test_analyzer_keeps_both_pose_estimates_separate_from_legacy_bbox_angle():
    """Navigation selects a pose estimate; the legacy field stays reproducible."""
    analyzer = object.__new__(HurdleAnalyzer)
    analyzer.min_confidence = 0.60
    analyzer.direction_deadband_norm = 0.04
    analyzer.max_valid_depth_m = 4.0
    analyzer.fx = FX
    analyzer.fy = FY
    analyzer.cx = CX
    analyzer.cy = CY
    analyzer.hurdle_camera_45_pitch_down_deg = 45.0
    analyzer.hurdle_camera_0_pitch_down_deg = 0.0
    analyzer.hurdle_camera_45_yaw_right_deg = 0.0
    analyzer.hurdle_camera_0_yaw_right_deg = 0.0
    analyzer.hurdle_camera_45_roll_clockwise_deg = 0.0
    analyzer.hurdle_camera_0_roll_clockwise_deg = 0.0
    analyzer._sample_depths = lambda *_args: (1.0, 1.0, 1.0, 5)
    centerline_samples = projected_bar_samples(30.0, 45.0)
    analyzer._sample_depth_points = lambda *_args: centerline_samples
    detection = {
        "confidence": 0.95,
        "bbox": [200, 100, 1000, 650],
        "center": [600, 375],
        "hurdle_bar_points_px": [[300, 500], [400, 450]],
        "hurdle_bar_image_angle_deg": 30.0,
    }

    candidate = analyzer._build_candidate(detection, 1280, 720)

    assert candidate is not None
    assert candidate.hurdle_angle_deg == pytest.approx(0.0)
    assert candidate.hurdle_angle_45_deg == pytest.approx(30.0)
    assert candidate.hurdle_angle_0_deg == pytest.approx(22.208)
    assert candidate.hurdle_angle_sample_source == "yellow_bar_centerline"


@pytest.mark.parametrize("angle_deg", [0.0, 30.0, 50.0])
def test_rgb_centerline_samples_follow_rotated_yellow_bar(angle_deg):
    """The five diagnostic pixels follow the bar instead of one bbox row."""
    image = np.full((480, 848, 3), (45, 45, 115), dtype=np.uint8)
    center = np.asarray([424.0, 260.0])
    direction = np.asarray([
        math.cos(math.radians(angle_deg)),
        -math.sin(math.radians(angle_deg)),
    ])
    start = np.rint(center - direction * 300.0).astype(int)
    end = np.rint(center + direction * 300.0).astype(int)
    cv2.line(image, tuple(start), tuple(end), (0, 220, 255), 24)

    result = extract_hurdle_bar_centerline(image, [90, 20, 760, 450])

    points = np.asarray(result["hurdle_bar_points_px"], dtype=float)
    assert points.shape == (5, 2)
    assert result["hurdle_bar_image_angle_deg"] == pytest.approx(
        angle_deg, abs=1.0,
    )
    assert result["hurdle_bar_elongation"] > 8.0
    perpendicular = np.asarray([-direction[1], direction[0]])
    distances = np.abs((points - center) @ perpendicular)
    assert np.max(distances) < 3.0
    if angle_deg > 0.0:
        assert len(set(points[:, 1])) == 5


@pytest.mark.parametrize(
    ("y_px", "expected_edge"),
    [(1, "TOP"), (478, "BOTTOM")],
)
def test_rgb_centerline_reports_image_edge_clipping(y_px, expected_edge):
    image = np.full((480, 848, 3), (45, 45, 115), dtype=np.uint8)
    cv2.line(image, (0, y_px), (847, y_px), (0, 220, 255), 20)

    result = extract_hurdle_bar_centerline(image, [0, 0, 848, 480])

    assert expected_edge in result["hurdle_bar_clipped_edges"]
    assert len(result["hurdle_bar_points_px"]) == 5
