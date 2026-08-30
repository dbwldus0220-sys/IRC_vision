"""Unit tests for class-specific YOLO post-processing thresholds."""

import numpy as np

from step.yolo26_detector import LetterboxInfo
from step.yolo26_detector import Yolo26Detector


def test_ball_uses_lower_raw_threshold_without_lowering_other_classes():
    """Apply the relaxed raw threshold only to the ball class."""
    detector = object.__new__(Yolo26Detector)
    detector.max_detections = 300
    detector.class_names = ["line", "ball"]
    detector.confidence_threshold = 0.25
    detector.ball_confidence_threshold = 0.20
    detector.hurdle_confidence_threshold = 0.20
    predictions = np.asarray(
        [
            [10, 10, 20, 20, 0.21, 1],
            [30, 30, 40, 40, 0.21, 0],
            [50, 50, 60, 60, 0.19, 1],
        ],
        dtype=np.float32,
    )[None, ...]

    detections = detector._postprocess(
        predictions,
        LetterboxInfo(scale=1.0, pad_x=0.0, pad_y=0.0),
        (100, 100, 3),
    )

    assert len(detections) == 1
    assert detections[0].class_name == "ball"
    assert detections[0].confidence == np.float32(0.21)


def test_hurdle_uses_lower_raw_threshold_without_lowering_line():
    """Keep weak hurdle candidates for temporal and geometry checks."""
    detector = object.__new__(Yolo26Detector)
    detector.max_detections = 300
    detector.class_names = ["line", "hurdle"]
    detector.confidence_threshold = 0.25
    detector.ball_confidence_threshold = 0.20
    detector.hurdle_confidence_threshold = 0.20
    predictions = np.asarray(
        [
            [10, 10, 20, 20, 0.21, 1],
            [30, 30, 40, 40, 0.21, 0],
        ],
        dtype=np.float32,
    )[None, ...]

    detections = detector._postprocess(
        predictions,
        LetterboxInfo(scale=1.0, pad_x=0.0, pad_y=0.0),
        (100, 100, 3),
    )

    assert len(detections) == 1
    assert detections[0].class_name == "hurdle"


def test_raw_hurdle_stays_visible_without_depth_or_confirmation():
    """Do not hide a YOLO hurdle while analyzer metadata is pending."""
    detector = object.__new__(Yolo26Detector)
    detector.hurdle_control_range_m = 1.0

    assert detector._object_range_status("hurdle", None, None, None) == (
        True,
        False,
        None,
    )
    assert detector._object_range_status(
        "hurdle",
        None,
        None,
        {
            "detected": True,
            "depth_valid": False,
            "depth_m": None,
        },
    ) == (True, False, None)


def test_raw_ball_stays_visible_without_depth_or_confirmation():
    """Do not hide a YOLO ball while depth/temporal metadata is unavailable."""
    detector = object.__new__(Yolo26Detector)
    detector.ball_control_range_m = 1.5

    assert detector._object_range_status("ball", None, None, None) == (
        True,
        False,
        None,
    )
    assert detector._object_range_status(
        "ball",
        {
            "detected": False,
            "raw_detected": True,
            "depth_valid": False,
            "depth_m": None,
        },
        None,
        None,
    ) == (True, False, None)


def test_ball_depth_only_controls_motion_readiness():
    """Keep near and far balls visible while gating only control readiness."""
    detector = object.__new__(Yolo26Detector)
    detector.ball_control_range_m = 1.5

    near = detector._object_range_status(
        "ball",
        {"detected": True, "depth_valid": True, "depth_m": 1.2},
        None,
        None,
    )
    far = detector._object_range_status(
        "ball",
        {"detected": True, "depth_valid": True, "depth_m": 1.8},
        None,
        None,
    )

    assert near == (True, True, 1.2)
    assert far == (True, False, 1.8)
