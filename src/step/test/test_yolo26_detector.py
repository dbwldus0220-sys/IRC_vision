"""Unit tests for class-specific YOLO post-processing thresholds."""

import time

import numpy as np
import pytest

from step.yolo26_detector import DEFAULT_CLASS_NAMES
from step.yolo26_detector import Detection
from step.yolo26_detector import LetterboxInfo
from step.yolo26_detector import Yolo26Detector


@pytest.mark.parametrize('message', [
    'direct profile restore: Profile Velocity address=112 result=-1001',
    'failed to restore direct playback profiles: failed to restore direct playback profile',
])
def test_executor_fault_banner_identifies_profile_failure(message):
    assert Yolo26Detector._executor_fault_banner({
        'safety': {'latched': True, 'message': message,
                   'error_code': 'SDK_HARDWARE_NOT_READY'},
    }) == 'STOPPED: MOTOR PROFILE ERROR'


def test_executor_fault_banner_requires_latched_status():
    assert Yolo26Detector._executor_fault_banner(None) is None
    assert Yolo26Detector._executor_fault_banner({}) is None
    assert Yolo26Detector._executor_fault_banner({
        'safety': {'latched': False, 'error_code': 'SDK_HARDWARE_NOT_READY'},
    }) is None
    assert Yolo26Detector._executor_fault_banner({
        'safety': {'latched': True, 'error_code': 'EXECUTOR_HEARTBEAT_TIMEOUT'},
    }) == 'STOPPED: EXECUTOR_HEARTBEAT_TIMEOUT'


class _SubscriptionCountPublisher:
    """Minimal publisher double exposing only its subscriber count."""

    def __init__(self, count):
        """Store the simulated ROS subscription count."""
        self.count = count

    def get_subscription_count(self):
        """Return the simulated ROS subscription count."""
        return self.count


def test_annotated_image_copy_requires_an_actual_subscriber():
    """Keep the local window without an unused full-frame ROS copy."""
    detector = object.__new__(Yolo26Detector)
    detector.publish_annotated_image = True
    detector.annotated_publisher = _SubscriptionCountPublisher(0)
    assert detector._annotated_image_has_subscriber() is False

    detector.annotated_publisher = _SubscriptionCountPublisher(1)
    assert detector._annotated_image_has_subscriber() is True


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


@pytest.mark.parametrize('goal_info,expected', [
    (None, (True, False, None)),
    ({'detected': False, 'depth_valid': False}, (True, False, None)),
    ({'detected': False, 'depth_valid': True, 'depth_m': 0.4}, (True, False, 0.4)),
    ({'detected': True, 'depth_valid': False, 'depth_m': 0.4}, (True, False, None)),
    ({'detected': True, 'depth_valid': True, 'depth_m': 0.4}, (True, True, 0.4)),
    ({'detected': True, 'depth_valid': True, 'depth_m': 1.5}, (True, False, 1.5)),
])
def test_raw_backboard_is_visible_independently_of_goal_analysis(goal_info, expected):
    """Visibility must not imply confirmed, in-range control readiness."""
    detector = object.__new__(Yolo26Detector)
    detector.goal_control_range_m = 0.5
    assert detector._object_range_status('backboard', None, goal_info, None) == expected


def test_backboard_raw_display_does_not_change_goal_display_gate():
    detector = object.__new__(Yolo26Detector)
    detector.goal_tracking_range_m = 1.0
    detector.goal_control_range_m = 0.5
    assert detector._object_range_status('goal', None, None, None) == (False, False, None)


@pytest.mark.parametrize('metrics_mode', ['ball', 'line', 'goal', 'hurdle'])
def test_backboard_box_is_drawn_without_goal_or_depth(metrics_mode):
    """Exercise actual drawing with no analyzer or mission nodes running."""
    detector = object.__new__(Yolo26Detector)
    detector._active_metrics_mode = lambda: metrics_mode
    for method in (
        '_fresh_motion_command', '_fresh_ball_info', '_recent_ball_info',
        '_fresh_goal_info', '_fresh_hurdle_info', '_fresh_line_info',
        '_fresh_decision_debug',
    ):
        setattr(detector, method, lambda: None)
    for method in (
        '_draw_ball_metrics', '_draw_goal_metrics', '_draw_line_metrics',
        '_draw_hurdle_metrics', '_draw_grasp_verification_status',
    ):
        setattr(detector, method, lambda image: None)
    detector.active_provider = 'test'
    detector.smoothed_fps = 15.0
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    board = Detection(
        class_id=3, class_name='backboard', confidence=0.9,
        bbox=[100, 150, 300, 350], center=[200, 250],
    )

    annotated = detector._draw_detections(frame, [board])

    assert annotated[150, 100].any()
    assert annotated[250, 200].any()
    assert not frame.any()


def _detector_with_ball_info_stamp(*, info_stamp_ns, overlay_stamp_ns):
    detector = object.__new__(Yolo26Detector)
    detector.latest_ball_info = {"rgb_stamp_ns": info_stamp_ns}
    detector.latest_ball_info_time = time.monotonic()
    detector.ball_info_timeout_sec = 0.8
    detector.overlay_max_stamp_delta_sec = 0.05
    detector._overlay_rgb_stamp_ns = overlay_stamp_ns
    detector._ball_info_stamp_delta_ms = None
    return detector


@pytest.mark.parametrize('reference_x', [710, 736])
def test_goal_overlay_uses_analyzer_axis_while_confirmation_is_pending(reference_x):
    """The yellow line must agree with the analyzer, including custom offsets."""
    detector = object.__new__(Yolo26Detector)
    detector._fresh_goal_info = lambda: {
        'detected': False, 'robot_center_x_px': reference_x,
    }
    detector._fresh_motion_command = lambda: None
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    detector._draw_goal_metrics(frame)

    assert frame[650, reference_x].any()
    assert not frame[650, 640].any()
    assert frame[360, 640].any()


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


def test_recent_ball_info_remains_available_when_rgb_stamp_is_stale():
    """Expose analyzer status without reusing stale geometry."""
    detector = _detector_with_ball_info_stamp(
        info_stamp_ns=1_000_000_000,
        overlay_stamp_ns=1_080_000_000,
    )
    detector.latest_ball_info.update(
        {
            "confirmation_confirmed": True,
            "detected": True,
        }
    )

    assert detector._fresh_ball_info() is None
    assert detector._recent_ball_info() == detector.latest_ball_info
    assert detector._ball_info_stamp_delta_ms == 80.0


def test_recent_ball_info_rejects_expired_receipt():
    """Do not display analyzer status after its receipt timeout."""
    detector = _detector_with_ball_info_stamp(
        info_stamp_ns=1_000_000_000,
        overlay_stamp_ns=1_040_000_000,
    )
    detector.latest_ball_info_time = (
        time.monotonic() - detector.ball_info_timeout_sec - 0.01
    )

    assert detector._recent_ball_info() is None
    assert detector._fresh_ball_info() is None


def test_grasp_banner_only_appears_during_verification():
    """Hide grasp recognition outside the explicit checking window."""
    assert Yolo26Detector._grasp_verification_banner(None) is None
    assert Yolo26Detector._grasp_verification_banner(
        {"grasp_verification": {"active": False, "result": "GRABBED"}}
    ) is None


def test_grasp_banner_reports_the_mission_nodes_current_result():
    """Display waiting, success, and failure using distinct colors."""
    assert Yolo26Detector._grasp_verification_banner(
        {"grasp_verification": {"active": True, "result": "UNKNOWN"}}
    ) == ("GRASP CHECK: WAITING", (0, 165, 255))
    assert Yolo26Detector._grasp_verification_banner(
        {"grasp_verification": {"active": True, "result": "GRABBED"}}
    ) == ("GRASP CHECK: GRABBED", (0, 255, 0))
    assert Yolo26Detector._grasp_verification_banner(
        {
            "grasp_verification": {
                "active": True,
                "result": "NOT_GRABBED",
            }
        }
    ) == ("GRASP CHECK: NOT GRABBED", (0, 0, 255))


def test_grasp_debug_status_expires_instead_of_leaving_stale_banner():
    """Drop the overlay if motion_decision_node status stops arriving."""
    detector = object.__new__(Yolo26Detector)
    detector.latest_decision_debug = {
        "grasp_verification": {"active": True, "result": "GRABBED"}
    }
    detector.latest_decision_debug_time = time.monotonic()
    detector.decision_debug_timeout_sec = 0.5

    assert detector._fresh_decision_debug() == detector.latest_decision_debug
    detector.latest_decision_debug_time -= 0.51
    assert detector._fresh_decision_debug() is None


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


@pytest.mark.parametrize("direction,expected", [
    ("LEFT", "BALL LOST | SEARCH LEFT"),
    ("RIGHT", "BALL LOST | SEARCH RIGHT"),
    (None, "BALL LOST"),
])
def test_ball_lost_banner_shows_remembered_search_direction(direction, expected):
    assert Yolo26Detector._ball_lost_banner({
        "ball_tracking": {"lost": True, "last_direction": direction},
    }) == expected


@pytest.mark.parametrize("debug", [None, {}, {"ball_tracking": {"lost": False}}])
def test_ball_lost_banner_clears_without_current_loss(debug):
    assert Yolo26Detector._ball_lost_banner(debug) is None


def test_ball_top_loss_banner_shows_forward_search_before_side():
    assert Yolo26Detector._ball_lost_banner({
        "ball_tracking": {"lost": True, "last_direction": "LEFT",
                          "top_forward_pending": True},
    }) == "BALL LOST | SEARCH FORWARD"


@pytest.mark.parametrize('motion_id,expected', [
    ('pickup_fine_forward_0', 'PICKUP / FINE FORWARD'),
    ('pickup_pre_backward_camera_down', 'PICKUP / BACKWARD (2 CYCLES)'),
    ('goal_camera_90_backward_1', 'GOAL / BACKWARD (1 CYCLE)'),
    ('line_turn_right_2', 'LINE / IN-PLACE TURN RIGHT 2'),
    ('line_recovery_left_3', 'LINE RETURN LEFT 3'),
    ('post_ball_line_turn_left_2', 'LINE ALIGN / IN-PLACE TURN LEFT 2'),
    ('goal_camera_90_fine_forward_1', 'GOAL / FINE FORWARD 1'),
    ('post_ball_forward_8', 'POST BALL FORWARD 8'),
])
def test_running_submotion_banner_survives_navigation_command_timeout(monkeypatch, motion_id, expected):
    import json
    from std_msgs.msg import String
    detector = object.__new__(Yolo26Detector)
    monkeypatch.setattr(time, 'monotonic', lambda: 10.0)
    detector._motion_status_callback(String(data=json.dumps({
        'status': 'RUNNING', 'motion_id': motion_id,
        'command_id': 8, 'request_id': 8, 'action': 'PICKUP_NOW',
    })))
    monkeypatch.setattr(time, 'monotonic', lambda: 15.0)
    assert detector._running_motion_banner()[0] == expected
    detector._motion_status_callback(String(data=json.dumps({
        'status': 'REJECTED', 'motion_id': 'unrelated',
        'command_id': 9, 'request_id': 9,
    })))
    assert detector._running_motion_banner()[0] == expected
    detector._motion_status_callback(String(data=json.dumps({
        'status': 'SUCCEEDED', 'motion_id': motion_id,
        'command_id': 8, 'request_id': 8,
    })))
    assert detector._running_motion_banner() is None


def test_atomic_sequence_completion_clears_dwell_banner():
    import json
    from std_msgs.msg import String
    detector = object.__new__(Yolo26Detector)
    for status, motion_id in [('RUNNING', '__DWELL__'), ('SUCCEEDED', 'pickup_first_backward_turn_right')]:
        detector._motion_status_callback(String(data=json.dumps({
            'status': status, 'motion_id': motion_id,
            'command_id': 8, 'request_id': 8,
        })))
    assert detector._running_motion_banner() is None


def test_missing_terminal_status_does_not_leave_banner_forever(monkeypatch):
    detector = object.__new__(Yolo26Detector)
    detector.latest_running_motion = {'motion_id': 'pickup_fine_forward_0'}
    detector.latest_running_motion_time = 10.0
    monkeypatch.setattr(time, 'monotonic', lambda: 131.0)
    assert detector._running_motion_banner() is None


@pytest.mark.parametrize('metrics_mode', ['ball', 'goal', 'line', 'hurdle'])
@pytest.mark.parametrize('lost', [False, True])
def test_running_turn_is_drawn_in_every_metrics_mode_even_when_ball_is_lost(metrics_mode, lost):
    detector = object.__new__(Yolo26Detector)
    detector._active_metrics_mode = lambda: metrics_mode
    for method in (
        '_fresh_motion_command', '_fresh_ball_info', '_recent_ball_info',
        '_fresh_goal_info', '_fresh_hurdle_info', '_fresh_line_info',
    ):
        setattr(detector, method, lambda: None)
    detector._fresh_decision_debug = lambda: {'ball_tracking': {'lost': lost}}
    for method in (
        '_draw_ball_metrics', '_draw_goal_metrics', '_draw_line_metrics',
        '_draw_hurdle_metrics', '_draw_grasp_verification_status',
    ):
        setattr(detector, method, lambda image: None)
    detector.latest_running_motion = {'motion_id': 'line_turn_right_2'}
    detector.latest_running_motion_time = time.monotonic()
    detector.active_provider = 'test'
    detector.smoothed_fps = 15.0
    labels = []
    detector._draw_action_banner = lambda image, text, color: labels.append(text)
    detector._draw_detections(np.zeros((480, 640, 3), dtype=np.uint8), [])
    expected = 'LINE / IN-PLACE TURN RIGHT 2'
    assert labels[-1] == ('BALL LOST | ' + expected if lost else expected)
