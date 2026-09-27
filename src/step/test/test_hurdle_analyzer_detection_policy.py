"""Tests for hurdle detection defaults without requiring ROS hardware."""

import importlib.util
import sys
from functools import lru_cache
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import patch


class FakeNode:
    """Provide the small rclpy Node surface used during analyzer setup."""

    def __init__(self, _name):
        """Create an empty parameter store."""
        self.parameters = {}

    def declare_parameter(self, name, value):
        """Record one declared parameter default."""
        self.parameters[name] = value

    def get_parameter(self, name):
        """Return one recorded parameter using the rclpy shape."""
        return SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, *_args):
        """Return a publisher substitute."""
        return SimpleNamespace(publish=lambda _message: None)

    def create_subscription(self, *_args, **_kwargs):
        """Return a subscription substitute."""
        return object()

    def get_logger(self):
        """Return a quiet logger substitute."""
        return SimpleNamespace(info=lambda _message: None)


@lru_cache(maxsize=1)
def load_hurdle_analyzer():
    """Load the analyzer with lightweight ROS interface substitutes."""
    cv_bridge = ModuleType('cv_bridge')
    cv_bridge.CvBridge = type('CvBridge', (), {})

    rclpy = ModuleType('rclpy')
    rclpy.callback_groups = ModuleType('rclpy.callback_groups')
    rclpy.callback_groups.MutuallyExclusiveCallbackGroup = type(
        'MutuallyExclusiveCallbackGroup',
        (),
        {},
    )
    rclpy.executors = ModuleType('rclpy.executors')
    rclpy.executors.ExternalShutdownException = type(
        'ExternalShutdownException',
        (Exception,),
        {},
    )
    rclpy.node = ModuleType('rclpy.node')
    rclpy.node.Node = FakeNode
    rclpy.qos = ModuleType('rclpy.qos')
    rclpy.qos.DurabilityPolicy = SimpleNamespace(VOLATILE='volatile')
    rclpy.qos.HistoryPolicy = SimpleNamespace(KEEP_LAST='keep_last')
    rclpy.qos.QoSProfile = lambda **kwargs: SimpleNamespace(**kwargs)
    rclpy.qos.ReliabilityPolicy = SimpleNamespace(
        BEST_EFFORT='best_effort',
        RELIABLE='reliable',
    )

    sensor_msgs = ModuleType('sensor_msgs')
    sensor_msgs.msg = ModuleType('sensor_msgs.msg')
    sensor_msgs.msg.CameraInfo = type('CameraInfo', (), {})
    sensor_msgs.msg.Image = type('Image', (), {})

    std_msgs = ModuleType('std_msgs')
    std_msgs.msg = ModuleType('std_msgs.msg')
    std_msgs.msg.String = type('String', (), {})

    module_name = 'step._hurdle_analyzer_detection_policy_test'
    analyzer_path = (
        Path(__file__).resolve().parents[1]
        / 'step'
        / 'hurdle_analyzer.py'
    )
    spec = importlib.util.spec_from_file_location(module_name, analyzer_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    substitutes = {
        'cv_bridge': cv_bridge,
        'rclpy': rclpy,
        'rclpy.callback_groups': rclpy.callback_groups,
        'rclpy.executors': rclpy.executors,
        'rclpy.node': rclpy.node,
        'rclpy.qos': rclpy.qos,
        'sensor_msgs': sensor_msgs,
        'sensor_msgs.msg': sensor_msgs.msg,
        'std_msgs': std_msgs,
        'std_msgs.msg': std_msgs.msg,
        module_name: module,
    }
    with patch.dict(sys.modules, substitutes):
        spec.loader.exec_module(module)
    return module.HurdleAnalyzer


def test_hurdle_detection_defaults_keep_go_policy_unchanged():
    """Detection defaults change while every GO threshold stays fixed."""
    analyzer = load_hurdle_analyzer()()

    assert analyzer.min_confidence == 0.60
    assert analyzer.head_down_trigger_bottom_distance_px == 120
    assert analyzer.detect_depth_m == 1.5
    assert analyzer.confirmation_filter.window_size == 40
    assert analyzer.confirmation_filter.required_hits == 15
    assert analyzer.confirmation_filter.max_missed_frames == 4

    assert analyzer.go_target_depth_m == 0.10
    assert analyzer.go_depth_tolerance_m == 0.10
    assert analyzer.go_angle_tolerance_deg == 8.0
    assert analyzer.go_confirmation_filter.window_size == 7
    assert analyzer.go_confirmation_filter.required_hits == 5


def test_hurdle_candidate_confidence_threshold_is_inclusive():
    """Accept confidence at 0.60 and reject a value just below it."""
    analyzer = load_hurdle_analyzer()()
    analyzer._sample_depths = lambda *_args: (None, None, None, 0)
    detection = {
        'confidence': 0.60,
        'bbox': [500, 250, 600, 350],
        'center': [550, 300],
    }

    accepted = analyzer._build_candidate(detection, 1280, 720)
    detection['confidence'] = 0.5999
    rejected = analyzer._build_candidate(detection, 1280, 720)

    assert accepted is not None
    assert accepted.confidence == 0.60
    assert rejected is None


def test_published_bottom_distance_uses_center_and_needs_no_depth():
    import json

    analyzer = load_hurdle_analyzer()()
    analyzer._sample_depths = lambda *_args: (None, None, None, 0)
    published = []
    analyzer._publish = published.append
    message = SimpleNamespace(data=json.dumps({
        "image_width": 640, "image_height": 480,
        "detections": [{
            "class_name": "hurdle", "confidence": 0.9,
            "bbox": [100, 350, 300, 408], "center": [200, 379],
        }],
    }))
    for _ in range(15):
        analyzer._detections_callback(message)
    info = published[-1]
    assert info.detected and not info.depth_valid
    assert info.bottom_distance_px == 100
    assert info.camera_bottom_gap_px is None
    analyzer._detections_callback(SimpleNamespace(data=json.dumps({"detections": []})))
    assert published[-1].bottom_distance_px is None


def test_far_confirmation_survives_approach_to_550mm():
    import json

    analyzer = load_hurdle_analyzer()()
    depth = [0.9]
    analyzer._sample_depths = lambda *_args: (depth[0], depth[0], depth[0], 5)
    published = []
    analyzer._publish = lambda info: published.append((info, dict(analyzer.confirmation_fields)))
    message = SimpleNamespace(data=json.dumps({
        "image_width": 640, "image_height": 480,
        "detections": [{
            "class_name": "hurdle", "confidence": 0.9,
            "bbox": [100, 300, 300, 360], "center": [200, 330],
        }],
    }))
    for _ in range(14):
        analyzer._detections_callback(message)
        assert not published[-1][0].detected
    analyzer._detections_callback(message)
    assert published[-1][0].detected
    assert published[-1][1]["confirmation_hits"] == 15
    for distance in (0.7, 0.551, 0.55):
        depth[0] = distance
        analyzer._detections_callback(message)
        info, confirmation = published[-1]
        assert info.detected and info.depth_m == distance
        assert confirmation["confirmation_confirmed"]
        assert confirmation["confirmation_hits"] > 15

    missing = SimpleNamespace(data=json.dumps({"detections": []}))
    for _ in range(4):
        analyzer._detections_callback(missing)
        assert not published[-1][0].detected
    analyzer._detections_callback(message)
    assert published[-1][1]["confirmation_confirmed"]
    for _ in range(5):
        analyzer._detections_callback(missing)
    analyzer._detections_callback(message)
    assert not published[-1][1]["confirmation_confirmed"]
    assert published[-1][1]["confirmation_hits"] == 1


def test_hurdle_bottom_trigger_uses_confirmed_rgb_center_without_depth():
    import json

    analyzer = load_hurdle_analyzer()()
    analyzer._sample_depths = lambda *_args: (None, None, None, 0)
    published = []
    analyzer._publish = published.append
    def observe(center_y):
        analyzer._detections_callback(SimpleNamespace(data=json.dumps({
            "image_width": 640, "image_height": 480,
            "detections": [{
                "class_name": "hurdle", "confidence": 0.9,
                "bbox": [100, center_y - 20, 300, center_y + 20],
                "center": [200, center_y],
            }],
        })))
        return published[-1]

    for _ in range(14):
        assert not observe(359).head_down_requested
    confirmed = observe(359)
    assert confirmed.detected and not confirmed.depth_valid
    assert confirmed.camera_center_offset_x_px == -120
    assert confirmed.bottom_distance_px == 120
    assert confirmed.head_down_requested
    assert not observe(358).head_down_requested
    assert observe(360).head_down_requested
    analyzer._detections_callback(SimpleNamespace(data=json.dumps({"detections": []})))
    assert not published[-1].head_down_requested
    assert published[-1].camera_center_offset_x_px is None
