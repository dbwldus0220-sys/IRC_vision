from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "mission_control"))

from mock_mission_input_node import (
    MockScenarioError,
    build_mock_vision_input,
    should_publish,
)


def test_straight_scenario_builds_centered_line_input():
    topic, payload = build_mock_vision_input("straight")
    assert topic == "/vision/line_info"
    assert payload["detected"] is True
    assert payload["filtered_heading_error_deg"] == 0.0
    assert payload["filtered_lateral_offset_norm"] == 0.0
    assert payload["heading_quality"] == 1.0
    assert payload["geometry_quality"] == 1.0
    assert payload["detection_quality"] == 1.0


def test_unsupported_scenario_is_rejected():
    with pytest.raises(MockScenarioError):
        build_mock_vision_input("camera")


def test_module_has_no_camera_or_model_imports():
    source_path = (
        Path(__file__).resolve().parents[1]
        / "mission_control"
        / "mock_mission_input_node.py"
    )
    source = source_path.read_text(encoding="utf-8").lower()
    forbidden_imports = (
        "import cv2",
        "import pyrealsense",
        "import ultralytics",
        "from ultralytics",
        "import yolo",
    )
    assert all(item not in source for item in forbidden_imports)


def test_publish_once_setting_stops_after_first_publication():
    assert should_publish(True, False) is True
    assert should_publish(True, True) is False
    assert should_publish(False, False) is True
    assert should_publish(False, True) is True
