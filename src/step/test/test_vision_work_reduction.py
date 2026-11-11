"""Protect published geometry while removing unused intermediate work."""

from dataclasses import asdict
import json
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from step.yolo26_detector import DEFAULT_CLASS_NAMES, Detection, Yolo26Detector
from step.yolo26_detector import extract_hurdle_bar_centerline
from test_goal_backboard_selection import detection, make_analyzer


def test_detection_serialization_preserves_fields_and_does_not_mutate_inputs():
    detector = object.__new__(Yolo26Detector)
    messages = []
    detector.detections_publisher = SimpleNamespace(publish=messages.append)
    detections = [
        Detection(index, name, 0.85, [10, 20, 30, 40], [20, 30])
        for index, name in enumerate(DEFAULT_CLASS_NAMES)
    ]
    expected = [asdict(item) for item in detections]
    message = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=12, nanosec=34), frame_id="camera",
        ),
        width=1280, height=720,
    )

    detector._publish_detections(message, detections)

    assert json.loads(messages[0].data) == {
        "stamp": {"sec": 12, "nanosec": 34}, "frame_id": "camera",
        "image_width": 1280, "image_height": 720, "detections": expected,
    }
    assert [asdict(item) for item in detections] == expected


def test_hurdle_component_bounds_preserve_crop_coordinates_and_selection():
    image = np.zeros((480, 848, 3), dtype=np.uint8)
    cv2.line(image, (180, 250), (700, 250), (0, 220, 255), 20)
    cv2.rectangle(image, (120, 90), (140, 110), (0, 220, 255), -1)
    cv2.rectangle(image, (520, 370), (550, 400), (0, 220, 255), -1)
    original = image.copy()

    full = extract_hurdle_bar_centerline(image, [0, 0, 848, 480])
    cropped = extract_hurdle_bar_centerline(image, [100, 60, 760, 440])

    assert cropped == full
    assert len(full["hurdle_bar_points_px"]) == 5
    assert full["hurdle_bar_image_angle_deg"] == pytest.approx(0.0)
    for x, y in full["hurdle_bar_points_px"]:
        assert 180 < x < 700
        assert y == 250
    np.testing.assert_array_equal(image, original)


@pytest.mark.parametrize("board_depth", [None, 0.6])
@pytest.mark.parametrize("missing", [None, "fx", "fy", "cx", "cy"])
def test_goal_camera_lateral_uses_each_depth_and_preserves_intrinsics_gate(
    board_depth, missing,
):
    analyzer, _ = make_analyzer(depth_m=0.5, robot_center_offset_px=96.0)
    analyzer._sample_backboard_line_depths = lambda *_: (
        0.6, board_depth, 0.6, 3, [[560, 222], [640, 222], [720, 222]],
    )
    if missing is not None:
        setattr(analyzer, missing, None)

    candidate = analyzer._build_candidate(detection(center_x=760), 1280, 720)

    if missing is not None:
        assert candidate.lateral_offset_m is None
        assert candidate.waist_aim_deg is None
        assert candidate.ring_center_lateral_m is None
    else:
        # Body alignment uses calibrated X=736; waist geometry uses optical X=640.
        assert candidate.lateral_offset_m == pytest.approx(0.02)
        expected_aim = np.degrees(np.arctan2(0.1 - 0.030, 0.5 + 0.10))
        assert candidate.waist_aim_deg == pytest.approx(expected_aim, abs=0.001)
        if board_depth is not None:
            assert candidate.ring_center_lateral_m == pytest.approx(0.12)
            assert candidate.ring_center_depth_m == pytest.approx(0.5)
        else:
            assert candidate.ring_center_lateral_m is None
