"""Tests separating RGB ball detection from optional depth metadata."""

import threading
import time

import numpy as np
import pytest

from step.ball_analyzer import BallAnalyzer
from step.ball_analyzer import ball_path_heading_deg


def _analyzer_with_depth(depth_m, depth_valid):
    analyzer = object.__new__(BallAnalyzer)
    analyzer.min_confidence = 0.45
    analyzer.horizontal_deadband_px = 20
    analyzer.center_tolerance_px = 140
    analyzer.robot_center_offset_px = 70.0
    analyzer.detect_depth_m = 1.5
    analyzer.approach_depth_m = 0.9
    analyzer.pickup_ready_depth_m = 0.15
    analyzer.pickup_now_depth_m = 0.07
    analyzer.pickup_depth_tolerance_m = 0.02
    analyzer.pickup_center_tolerance_norm = 0.08
    analyzer.pickup_target_y_ratio = 0.82
    analyzer.pickup_y_tolerance_ratio = 0.12
    analyzer.fx = None
    analyzer.fy = None
    analyzer.cx = None
    analyzer.cy = None
    analyzer._sample_depth_m = (
        lambda _x, _y, _bbox=None: (depth_m, depth_valid)
    )
    return analyzer


def _raw_detection():
    return {
        "confidence": 0.83,
        "bbox": [680, 500, 736, 558],
        "center": [708, 529],
    }


def test_rgb_ball_remains_candidate_without_depth():
    """A strong RGB detection remains a ball with a NO_DEPTH state."""
    analyzer = _analyzer_with_depth(None, False)

    candidate = analyzer._build_candidate(_raw_detection(), 1280, 720)

    assert candidate is not None
    assert candidate.depth_valid is False
    assert candidate.offset_x_px == -2
    assert candidate.steering_angle_deg == pytest.approx(-0.603, abs=0.001)
    state = analyzer._state_for_candidate(candidate, 720)
    assert state[0] == "NO_DEPTH"


def test_ball_path_angle_uses_shifted_bottom_center_axis():
    """Use the line-calibrated +70 px axis and the image bottom origin."""
    centered = ball_path_heading_deg(
        target_x=710,
        target_y=500,
        image_width=1280,
        image_height=720,
        robot_center_offset_px=70.0,
    )
    image_midpoint = ball_path_heading_deg(
        target_x=640,
        target_y=500,
        image_width=1280,
        image_height=720,
        robot_center_offset_px=70.0,
    )

    assert centered == 0.0
    assert image_midpoint == pytest.approx(-17.726, abs=0.001)


def test_rgb_ball_remains_candidate_beyond_tracking_distance():
    """A distant RGB detection remains visible and is classified as FAR."""
    analyzer = _analyzer_with_depth(1.8, True)

    candidate = analyzer._build_candidate(_raw_detection(), 1280, 720)

    assert candidate is not None
    assert candidate.depth_m == 1.8
    state = analyzer._state_for_candidate(candidate, 720)
    assert state[0] == "FAR"


def test_ball_projection_reports_ground_plane_distance():
    """Expose pitch-corrected robot-floor range separately from optical Z."""
    analyzer = object.__new__(BallAnalyzer)
    analyzer.fx = 600.0
    analyzer.fy = 600.0
    analyzer.cx = 640.0
    analyzer.cy = 360.0
    analyzer.camera_pitch_down_deg = 45.0
    analyzer.camera_forward_offset_m = 0.0

    projection = analyzer._project_ball_position(640, 360, 1.0, True)

    assert projection[4] == pytest.approx(1.0)
    assert projection[5] == pytest.approx(2 ** -0.5)


def _depth_sampler(image):
    analyzer = object.__new__(BallAnalyzer)
    analyzer._depth_lock = threading.RLock()
    analyzer.latest_depth_image = image
    analyzer.latest_depth_time = time.monotonic()
    analyzer.depth_timeout_sec = 0.7
    analyzer.depth_window_px = 9
    analyzer.depth_bbox_inner_ratio = 0.70
    analyzer.depth_min_valid_pixels = 5
    analyzer.depth_hold_sec = 0.40
    analyzer.max_valid_depth_m = 4.0
    analyzer.last_valid_ball_depth_m = None
    analyzer.last_valid_ball_depth_time = None
    analyzer.last_valid_ball_depth_center = None
    return analyzer


def test_depth_falls_back_from_center_hole_to_inner_ball_bbox():
    """Recover distance from the ball ROI when its center has a depth hole."""
    image = np.zeros((720, 1280), dtype=np.uint16)
    image[500:560, 600:660] = 920
    image[525:534, 625:634] = 0
    analyzer = _depth_sampler(image)

    depth, valid = analyzer._sample_depth_m(
        629,
        529,
        [599, 499, 660, 561],
    )

    assert valid is True
    assert depth == np.float32(0.92)


def test_depth_briefly_holds_last_value_across_full_depth_hole():
    """Avoid STOP flicker for a short invalid stereo frame."""
    image = np.full((720, 1280), 910, dtype=np.uint16)
    analyzer = _depth_sampler(image)
    first_depth, first_valid = analyzer._sample_depth_m(
        629,
        529,
        [599, 499, 660, 561],
    )
    analyzer.latest_depth_image = np.zeros_like(image)

    held_depth, held_valid = analyzer._sample_depth_m(
        631,
        530,
        [601, 500, 662, 562],
    )

    assert first_valid is True
    assert first_depth == np.float32(0.91)
    assert held_valid is True
    assert held_depth == first_depth
