"""Check ground diagnostic text and panel bounds at both camera resolutions."""

import numpy as np
import pytest

from step.yolo26_detector import Yolo26Detector


@pytest.mark.parametrize("width,height", [(1280, 720), (640, 360)])
@pytest.mark.parametrize("ground_valid", [False, True])
def test_ground_metrics_preserve_existing_rows_and_fit_panel(
    monkeypatch, width, height, ground_valid,
):
    detector = object.__new__(Yolo26Detector)
    detector.show_line_metrics = True
    info = {
        "detected": True,
        "filtered_heading_error_deg": 7.0,
        "filtered_lateral_offset_norm": 0.2,
        "ground_projection_valid": ground_valid,
        "ground_heading_error_deg": -0.326,
        "ground_lateral_offset_m": 0.0015,
        "ground_steering_angle_deg": -0.151,
        "ground_fit_rmse_m": 0.0016,
        "nearest_line_depth_m": 0.7,
        "nearest_line_ground_forward_distance_m": 0.5,
        "corner_preview_confirmed": True,
        "corner_direction": "RIGHT",
        "corner_start_distance_m": 0.8,
        "corner_approach_motion": "STRAIGHT",
    }
    monkeypatch.setattr(detector, "_fresh_line_info", lambda: info)
    monkeypatch.setattr(detector, "_fresh_motion_command", lambda: None)
    monkeypatch.setattr(detector, "_draw_line_path_geometry", lambda *args: None)
    texts, rectangles = [], []
    monkeypatch.setattr(
        "step.yolo26_detector.cv2.putText",
        lambda image, text, origin, *args: texts.append((text, origin)),
    )
    monkeypatch.setattr(
        "step.yolo26_detector.cv2.rectangle",
        lambda image, first, last, *args: rectangles.append((first, last)),
    )
    detector._draw_line_metrics(np.zeros((height, width, 3), dtype=np.uint8))
    rows = dict(text.split(":", 1) for text, _ in texts if ":" in text)
    assert rows["Image head  "].strip() == "+7.0deg"
    assert rows["Offset norm "].strip() == "+0.200"
    for label in ("Turn preview", "Quality     ", "Ref depth   ",
                  "Ref forward ", "Corner      ", "Corner dist ", "Approach    "):
        assert label in rows
    for label in ("Ground head ", "Ground off  ", "Steering    ", "Ground RMSE "):
        assert (rows[label].strip() != "N/A") == ground_valid
    (_, top), (_, bottom) = rectangles[0]
    assert bottom <= height - 8
    assert all(top < origin[1] < bottom for _, origin in texts)


def test_missing_line_still_shows_ground_metrics_as_unavailable(monkeypatch):
    detector = object.__new__(Yolo26Detector)
    detector.show_line_metrics = True
    monkeypatch.setattr(detector, "_fresh_line_info", lambda: None)
    monkeypatch.setattr(detector, "_fresh_motion_command", lambda: None)
    monkeypatch.setattr(detector, "_draw_line_path_geometry", lambda *args: None)
    texts = []
    monkeypatch.setattr(
        "step.yolo26_detector.cv2.putText",
        lambda image, text, *args: texts.append(text),
    )
    detector._draw_line_metrics(np.zeros((360, 640, 3), dtype=np.uint8))
    assert "State       : SEARCH" in texts
    assert "Ground head : N/A" in texts
    assert "Ground off  : N/A" in texts
    assert "Steering    : N/A" in texts
    assert "Ground RMSE : N/A" in texts
