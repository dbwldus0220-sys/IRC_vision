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

    def create_subscription(self, *_args):
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
    rclpy.executors = ModuleType('rclpy.executors')
    rclpy.executors.ExternalShutdownException = type(
        'ExternalShutdownException',
        (Exception,),
        {},
    )
    rclpy.node = ModuleType('rclpy.node')
    rclpy.node.Node = FakeNode
    rclpy.qos = ModuleType('rclpy.qos')
    rclpy.qos.qos_profile_sensor_data = object()

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

    assert analyzer.min_confidence == 0.50
    assert analyzer.confirmation_filter.window_size == 20
    assert analyzer.confirmation_filter.required_hits == 15
    assert analyzer.confirmation_filter.max_missed_frames == 4

    assert analyzer.detect_depth_m == 1.0
    assert analyzer.go_target_ground_gap_m == 0.10
    assert analyzer.go_ground_gap_tolerance_m == 0.10
    assert analyzer.go_max_camera_bottom_gap_m == 0.05
    assert analyzer.go_angle_tolerance_deg == 8.0
    assert analyzer.go_confirmation_filter.window_size == 7
    assert analyzer.go_confirmation_filter.required_hits == 5


def test_hurdle_candidate_confidence_threshold_is_inclusive():
    """Accept confidence at 0.50 and reject a value just below it."""
    analyzer = load_hurdle_analyzer()()
    analyzer._sample_depths = lambda *_args: (None, None, None, 0)
    detection = {
        'confidence': 0.50,
        'bbox': [500, 250, 600, 350],
        'center': [550, 300],
    }

    accepted = analyzer._build_candidate(detection, 1280, 720)
    detection['confidence'] = 0.4999
    rejected = analyzer._build_candidate(detection, 1280, 720)

    assert accepted is not None
    assert accepted.confidence == 0.50
    assert rejected is None
