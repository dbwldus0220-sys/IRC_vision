"""Visible path coverage and sparse markers are distinct from executable fits."""

import math
import json

import pytest

from step.yolo_line_analyzer import LinePoint, YoloLineAnalyzer, visible_line_geometry


@pytest.mark.parametrize("count", [0, 1, 2])
def test_sparse_markers_survive_without_claiming_corner_geometry(count):
    points = [LinePoint(700, 650 - i * 100, .9) for i in range(count)]
    result = visible_line_geometry(points, 1280, 720, {}, None)
    assert len(result["visible_line_points_px"]) == count
    assert result["visible_line_confidences"] == [.9] * count
    assert result["corner_visible_outgoing_fraction"] is None


def test_corner_coverage_uses_length_not_marker_count():
    points = [LinePoint(500, 700, .9), LinePoint(500, 600, .9), LinePoint(900, 600, .9)]
    config = {"ground_projection_enabled": False}
    result = visible_line_geometry(points, 1280, 720, config, 1)
    assert result["corner_visible_outgoing_fraction"] == pytest.approx(.8)
    denser = points[:2] + [LinePoint(600, 600, .9), LinePoint(700, 600, .9)] + points[2:]
    assert visible_line_geometry(denser, 1280, 720, config, 1)["corner_visible_outgoing_fraction"] == pytest.approx(.8)


@pytest.mark.parametrize("angle,detected", [(0, False), (19.9, False), (20.1, True), (-20.1, True)])
def test_twenty_degree_threshold_keeps_straight_paths_out(angle, detected):
    points = [LinePoint(500, 700, .9), LinePoint(500, 600, .9),
              LinePoint(500 + 100 * math.sin(math.radians(angle)),
                        600 - 100 * math.cos(math.radians(angle)), .9)]
    result = YoloLineAnalyzer._detect_corner_start_geometry(
        points, min_points=3, min_segment_length_px=20., straight_max_turn_delta_deg=15.,
        min_turn_delta_deg=20., onset_deviation_deg=15., min_consistent_segments=1, min_consistency=.75,
    )
    assert result["detected"] == detected
    if detected:
        assert result["direction"] == ("RIGHT" if angle > 0 else "LEFT")


def test_analyzer_publishes_single_point_without_claiming_fitted_line(monkeypatch, tmp_path):
    import rclpy
    from std_msgs.msg import String
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    # An owned context isolates this replay from other tests' default ROS context.
    context = rclpy.context.Context()
    context.init(args=[], domain_id=201, initialize_logging=False)
    monkeypatch.setattr("rclpy.node.get_default_context", lambda: context)
    node = YoloLineAnalyzer()
    try:
        published = []
        monkeypatch.setattr(node.publisher, "publish", lambda m: published.append(json.loads(m.data)))
        assert node.corner_min_turn_delta_deg == 20.
        message = String(data=json.dumps({
            "image_width": 1280, "image_height": 720,
            "stamp": {"sec": 10, "nanosec": 0},
            "detections": [{"class_name": "line", "confidence": .9,
                            "center": [980, 550], "bbox": [970, 540, 990, 560]}],
        }))
        node._detections_callback(message)
        result = published[-1]
        assert not result["detected"] and not result["ground_projection_valid"]
        assert result["visible_line_points_px"] == [[980, 550]]
        assert result["visible_line_confidences"] == [.9]
        assert result["corner_visible_outgoing_fraction"] is None
        node._detections_callback(String(data=json.dumps({"detections": []})))
        assert published[-1]["visible_line_points_px"] == []
    finally:
        node.destroy_node()
        context.shutdown()


@pytest.mark.parametrize("action", ["LEFT", "RIGHT", "WAIT"])
def test_overlay_preserves_corner_names_and_explains_sparse_wait(monkeypatch, action):
    import numpy as np
    from step.yolo26_detector import Yolo26Detector
    detector = object.__new__(Yolo26Detector)
    detector.show_line_metrics = True
    metadata = ({"sparse_view": {"confirmed_frames": 2}} if action == "WAIT"
                else {"corner_state": "TURNING"})
    monkeypatch.setattr(detector, "_fresh_motion_command", lambda: {
        "source": "line", "action": action, "source_command": metadata})
    monkeypatch.setattr(detector, "_fresh_line_info", lambda: {
        "detected": False, "visible_line_points_px": [[980, 550]],
    })
    monkeypatch.setattr(detector, "_draw_line_path_geometry", lambda *args: None)
    labels = []
    monkeypatch.setattr(detector, "_draw_action_banner", lambda image, text, color: labels.append(text))
    detector._draw_line_metrics(np.zeros((720, 1280, 3), dtype=np.uint8))
    assert labels == ["WAIT / LINE CONFIRM" if action == "WAIT" else action]
