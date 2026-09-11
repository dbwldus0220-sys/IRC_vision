"""Unit tests for class-specific YOLO post-processing thresholds."""

import time

import numpy as np

from step.yolo26_detector import DEFAULT_CLASS_NAMES
from step.yolo26_detector import LetterboxInfo
from step.yolo26_detector import Yolo26Detector


def test_default_class_names_include_grab_model_output():
    """Keep TensorRT/ONNX class ID 5 aligned with the six-class model."""
    assert DEFAULT_CLASS_NAMES == [
        "line",
        "ball",
        "goal",
        "backboard",
        "hurdle",
        "grab",
    ]


def test_grab_class_uses_reference_branch_threshold():
    """Publish class ID 5 at the detector's unchanged default threshold."""
    detector = object.__new__(Yolo26Detector)
    detector.max_detections = 300
    detector.class_names = DEFAULT_CLASS_NAMES.copy()
    detector.confidence_threshold = 0.25
    detector.ball_confidence_threshold = 0.20
    detector.hurdle_confidence_threshold = 0.60
    predictions = np.asarray(
        [
            [10, 10, 20, 20, 0.26, 5],
            [30, 30, 40, 40, 0.24, 5],
        ],
        dtype=np.float32,
    )[None, ...]

    detections = detector._postprocess(
        predictions,
        LetterboxInfo(scale=1.0, pad_x=0.0, pad_y=0.0),
        (100, 100, 3),
    )

    assert len(detections) == 1
    assert detections[0].class_id == 5
    assert detections[0].class_name == "grab"


def test_ball_uses_lower_raw_threshold_without_lowering_other_classes():
    """Apply the relaxed raw threshold only to the ball class."""
    detector = object.__new__(Yolo26Detector)
    detector.max_detections = 300
    detector.class_names = ["line", "ball"]
    detector.confidence_threshold = 0.25
    detector.ball_confidence_threshold = 0.20
    detector.hurdle_confidence_threshold = 0.60
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


def test_hurdle_uses_strict_raw_threshold_without_raising_line():
    """Reject weak hurdle boxes without changing another class threshold."""
    detector = object.__new__(Yolo26Detector)
    detector.max_detections = 300
    detector.class_names = ["line", "hurdle"]
    detector.confidence_threshold = 0.25
    detector.ball_confidence_threshold = 0.20
    detector.hurdle_confidence_threshold = 0.60
    predictions = np.asarray(
        [
            [10, 10, 20, 20, 0.61, 1],
            [30, 30, 40, 40, 0.26, 0],
            [50, 50, 60, 60, 0.59, 1],
        ],
        dtype=np.float32,
    )[None, ...]

    detections = detector._postprocess(
        predictions,
        LetterboxInfo(scale=1.0, pad_x=0.0, pad_y=0.0),
        (100, 100, 3),
    )

    assert len(detections) == 2
    assert detections[0].class_name == "hurdle"
    assert detections[1].class_name == "line"


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


def test_ball_raw_depth_controls_motion_readiness():
    """Keep near and far balls visible while gating by aligned Depth Z."""
    detector = object.__new__(Yolo26Detector)
    detector.ball_control_range_m = 1.5

    near = detector._object_range_status(
        "ball",
        {
            "detected": True,
            "depth_valid": True,
            "depth_m": 1.2,
            "ground_distance_m": 1.8,
        },
        None,
        None,
    )
    far = detector._object_range_status(
        "ball",
        {
            "detected": True,
            "depth_valid": True,
            "depth_m": 1.8,
            "ground_distance_m": 1.2,
        },
        None,
        None,
    )

    assert near == (True, True, 1.2)
    assert far == (True, False, 1.8)


def _detector_with_ball_info_stamp(*, info_stamp_ns, overlay_stamp_ns):
    detector = object.__new__(Yolo26Detector)
    detector.latest_ball_info = {"rgb_stamp_ns": info_stamp_ns}
    detector.latest_ball_info_time = time.monotonic()
    detector.ball_info_timeout_sec = 0.8
    detector.overlay_max_stamp_delta_sec = 0.05
    detector._overlay_rgb_stamp_ns = overlay_stamp_ns
    detector._ball_info_stamp_delta_ms = None
    return detector


def test_ball_overlay_accepts_info_within_rgb_stamp_tolerance():
    """Allow analyzer output close enough to the displayed RGB frame."""
    detector = _detector_with_ball_info_stamp(
        info_stamp_ns=1_000_000_000,
        overlay_stamp_ns=1_040_000_000,
    )

    assert detector._fresh_ball_info() == detector.latest_ball_info
    assert detector._ball_info_stamp_delta_ms == 40.0


def test_ball_overlay_rejects_info_outside_rgb_stamp_tolerance():
    """Do not draw a distance computed for an unrelated RGB frame."""
    detector = _detector_with_ball_info_stamp(
        info_stamp_ns=1_000_000_000,
        overlay_stamp_ns=1_051_000_000,
    )

    assert detector._fresh_ball_info() is None
    assert detector._ball_info_stamp_delta_ms == 51.0


def test_model_sha256_is_computed_without_changing_model_data(tmp_path):
    """Report a stable startup model identifier."""
    model_path = tmp_path / "model.onnx"
    model_path.write_bytes(b"STEP grasp model")

    assert Yolo26Detector._file_sha256(model_path) == (
        "40cd83a2cec1ead78101bb8d0510e8955d66cc5a7b74ee"
        "22d8ab713e80b6a058"
    )


def test_inference_timing_emits_one_bounded_summary():
    """Aggregate inference timing instead of logging every image."""

    class Logger:
        def __init__(self):
            self.infos = []

        def info(self, message):
            self.infos.append(message)

    detector = object.__new__(Yolo26Detector)
    detector.inference_timing_samples_ms = []
    detector.inference_timing_window_started = 0.0
    detector.inference_timing_last_log = 0.0
    logger = Logger()
    detector.get_logger = lambda: logger

    detector._record_inference_timing(10.0, 1.0)
    assert logger.infos == []
    detector._record_inference_timing(20.0, 2.0)

    assert len(logger.infos) == 1
    assert logger.infos[0].startswith("[YOLO_INFERENCE_TIMING]")
    assert "samples=2" in logger.infos[0]
    assert "avg_ms=15.000" in logger.infos[0]
    assert detector.inference_timing_samples_ms == []
