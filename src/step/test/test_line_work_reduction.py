"""Protect line-fit decisions and exact detector pixels during optimization."""

from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np
import pytest
from rclpy.node import Node

from step.yolo_line_analyzer import LinePoint, YoloLineAnalyzer, _unit_score
from step.yolo26_detector import Yolo26Detector


def make_offline_analyzer(cls=YoloLineAnalyzer):
    """Use real defaults and filters with no ROS communication or hardware."""
    parameters = {}

    def declare(name, default):
        parameters[name] = SimpleNamespace(value=default)
        return parameters[name]

    def fail(message):
        raise AssertionError(message)

    published = []
    logger = SimpleNamespace(info=lambda *_: None, warning=fail, error=fail)
    with patch.object(Node, "__init__", return_value=None), \
         patch.object(Node, "declare_parameter", side_effect=declare), \
         patch.object(Node, "get_parameter", side_effect=parameters.__getitem__), \
         patch.object(Node, "create_publisher", return_value=SimpleNamespace(publish=published.append)), \
         patch.object(Node, "create_subscription", return_value=None), \
         patch.object(Node, "get_logger", return_value=logger):
        node = cls()
    node.get_logger = lambda: logger
    return node, published


@pytest.mark.parametrize("value", [
    float("-inf"), -2., -0., 0., 1e-12, .4, 1. - 1e-12,
    1., 2., float("inf"), float("nan"),
])
def test_scalar_score_keeps_clip_boundaries_and_nonfinite_behavior(value):
    expected = float(np.clip(value, 0., 1.))
    actual = _unit_score(value)
    if np.isnan(expected):
        assert np.isnan(actual)
    else:
        assert actual == expected
        assert np.signbit(actual) == np.signbit(expected)


@pytest.mark.parametrize("mode", ["unchanged", "too_few_inliers", "last_rejection"])
def test_robust_fit_uses_solution_for_final_points(mode):
    analyzer = object.__new__(YoloLineAnalyzer)
    analyzer.min_points_for_fit = 2
    analyzer.min_fit_y_span_px = 0.
    analyzer.outlier_residual_px = 10.
    analyzer.max_fit_iterations = 1
    points = [LinePoint(20., y, .9) for y in (300., 200., 100.)]
    calls = []

    def fit(working):
        calls.append(list(working))
        residuals = {
            "unchanged": [1., 2., 3.],
            "too_few_inliers": [1., 20., 30.],
            "last_rejection": [1., 2., 30.],
        }[mode] if len(working) == 3 else [0., 0.]
        return float(len(working)), 20., np.asarray(residuals)

    analyzer._fit_once = fit
    result = analyzer._robust_fit(points)
    if mode == "last_rejection":
        assert calls == [points, points[:2]]
        assert result.point_count == 2
        assert result.slope == 2.
        assert result.rmse_px == 0.
    else:
        assert calls == [points]
        assert result.point_count == 3
        assert result.slope == 3.


@pytest.mark.parametrize("shape", [(720, 1280), (480, 848), (360, 640), (333, 517), (640, 360)])
@pytest.mark.parametrize("model_size", [(640, 640), (320, 256)])
def test_preprocess_preserves_every_model_input_bit(shape, model_size):
    detector = object.__new__(Yolo26Detector)
    detector.input_width, detector.input_height = model_size
    # Include all uint8 intensities and distinct B/G/R values.
    image = np.arange(shape[0] * shape[1] * 3, dtype=np.uint8).reshape(*shape, 3)
    original = image.copy()
    padded, expected_info = detector._letterbox(image)
    rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
    expected = np.ascontiguousarray(rgb.transpose(2, 0, 1)[None].astype(np.float32) / 255.0)

    blob, info = detector._preprocess(image)

    assert info == expected_info
    assert blob.dtype == np.float32 and blob.flags.c_contiguous
    assert blob.tobytes() == expected.tobytes()
    np.testing.assert_array_equal(image, original)


@pytest.mark.parametrize("shape", [(720, 1280), (480, 848), (360, 640)])
@pytest.mark.parametrize("panel_width,panel_y", [(410, 44), (390, 44), (410, 104)])
def test_panel_pixels_match_full_frame_overlay(shape, panel_width, panel_y):
    image = np.random.default_rng(16).integers(0, 256, (*shape, 3), dtype=np.uint8)
    expected = image.copy()
    x = shape[1] - panel_width - 12
    bottom = min(shape[0] - 8, panel_y + 526)
    overlay = expected.copy()
    cv2.rectangle(overlay, (x, panel_y), (x + panel_width, bottom), (20, 20, 20), -1)
    region = np.s_[panel_y:bottom, x:x + panel_width]
    expected[region] = cv2.addWeighted(overlay[region], .72, expected[region], .28, 0.)

    Yolo26Detector._draw_metrics_panel_background(image, x, panel_y, panel_width, bottom)

    np.testing.assert_array_equal(image, expected)
