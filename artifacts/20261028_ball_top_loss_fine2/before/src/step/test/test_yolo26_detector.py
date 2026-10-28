"""Unit tests for class-specific YOLO post-processing thresholds."""

import time

import numpy as np
import pytest

from step.yolo26_detector import DEFAULT_CLASS_NAMES
from step.yolo26_detector import Detection
from step.yolo26_detector import LetterboxInfo
from step.yolo26_detector import Yolo26Detector


@pytest.mark.parametrize("reason,label", [
    ("invalid_ground_line_geometry", "GROUND INVALID"),
    ("line_offset_target_invalid", "TARGET INVALID"),
    ("low_line_quality", "LOW QUALITY"),
])
def test_line_stop_banner_explains_navigation_rejection(reason, label):
    debug = {"source": "LINE", "decision": {"selected_action": "STOP", "reason": reason}}
    assert Yolo26Detector._line_stop_banner(debug) == f"STOP | {label}"
    debug["decision"]["selected_action"] = "RIGHT"
    assert Yolo26Detector._line_stop_banner(debug) is None
    debug["decision"]["selected_action"] = "STOP"
    debug["source"] = "BALL"
    assert Yolo26Detector._line_stop_banner(debug) is None


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


@pytest.mark.parametrize('delta_ms,accepted', [(200, True), (300, True), (301, False)])
def test_ball_overlay_relaxed_default_accepts_recorded_latency(delta_ms, accepted):
    detector = _detector_with_ball_info_stamp(
        info_stamp_ns=1_000_000_000,
        overlay_stamp_ns=1_000_000_000 + delta_ms * 1_000_000,
    )
    del detector.overlay_max_stamp_delta_sec
    assert (detector._fresh_ball_info() is not None) is accepted


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


@pytest.mark.parametrize("result", ["UNKNOWN", "NOT_GRABBED", "GRABBED"])
def test_grasp_banner_hides_all_provisional_votes(result):
    assert Yolo26Detector._grasp_verification_banner({
        "grasp_verification": {"active": True, "finalized": False, "result": result},
    }) is None


@pytest.mark.parametrize("result,label,color", [
    ("GRABBED", "GRASP CHECK: GRABBED", (0, 255, 0)),
    ("NOT_GRABBED", "GRASP CHECK: NOT GRABBED", (0, 0, 255)),
])
def test_grasp_banner_displays_only_final_result(result, label, color):
    assert Yolo26Detector._grasp_verification_banner({
        "grasp_verification": {"active": False, "finalized": True, "result": result},
    }) == (label, color)


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
    ("LEFT", "BALL LOST | WAIT"),
    ("RIGHT", "BALL LOST | WAIT"),
    (None, "BALL LOST | WAIT"),
])
def test_ball_lost_banner_does_not_claim_turn_from_memory_alone(direction, expected):
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
        "decision": {"selected_action": "BALL_LOST_FORWARD_2"},
    }) == "BALL LOST | SEARCH FORWARD"


@pytest.mark.parametrize('motion_id,expected', [
    ('pickup_fine_forward_0', 'PICKUP / FINE FORWARD'),
    ('__PICKUP_FINE_PRE_DWELL__', 'PICKUP / WAIT BEFORE FINE STEP'),
    ('pickup_pre_backward_camera_down', 'PICKUP / BACKWARD (2 CYCLES)'),
    ('pickup_retreat_2', 'PICKUP / BACKWARD (2 CYCLES)'),
    ('goal_camera_90_backward_1', 'GOAL / BACKWARD (1 CYCLE)'),
    ('pickup_lost_ball_backward_1', 'BALL REACQUIRE / BACKWARD (1 CYCLE)'),
    ('__TRANSITION_PRE_DWELL__', 'POSTURE / WAIT BEFORE TRANSITION'),
    ('__TRANSITION_POST_DWELL__', 'POSTURE / WAIT AFTER TRANSITION'),
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


@pytest.mark.parametrize('status_first', [False, True])
@pytest.mark.parametrize('source,motion_id,expected', [
    ('ball', 'ball_general_fine_forward_8', 'BALL / FINE FORWARD'),
    ('hurdle', 'ball_general_fine_forward_8', 'HURDLE / FINE FORWARD'),
    ('hurdle', 'pickup_fine_forward_0', 'HURDLE / FINE FORWARD'),
    ('hurdle', 'pickup_lost_ball_backward_1', 'HURDLE / BACKWARD (1 CYCLE)'),
    ('hurdle', 'line_forward_4', 'HURDLE / FORWARD 4'),
    ('hurdle', 'line_recovery_left_4', 'HURDLE / LINE RETURN LEFT 4'),
    ('hurdle', 'line_recovery_right_4', 'HURDLE / LINE RETURN RIGHT 4'),
])
def test_shared_fine_motion_label_keeps_command_owner(monkeypatch, status_first, source, motion_id, expected):
    import json
    from std_msgs.msg import String

    detector = object.__new__(Yolo26Detector)
    monkeypatch.setattr(time, 'monotonic', lambda: 10.0)
    command = String(data=json.dumps({
        'command_id': 8, 'source': source, 'action': 'STRAIGHT_0',
    }))
    status = String(data=json.dumps({
        'status': 'RUNNING', 'motion_id': motion_id,
        'command_id': 8, 'request_id': 8, 'action': 'STRAIGHT_0',
    }))
    callbacks = [
        (detector._motion_command_callback, command),
        (detector._motion_status_callback, status),
    ]
    if status_first:
        callbacks.reverse()
    for callback, message in callbacks:
        callback(message)
    # An unrelated command and an expired command display must not rename it.
    detector._motion_command_callback(String(data=json.dumps({
        'command_id': 9, 'source': 'line', 'action': 'WAIT',
    })))
    monkeypatch.setattr(time, 'monotonic', lambda: 15.0)
    detector._motion_status_callback(status)
    assert detector._running_motion_banner()[0] == expected


@pytest.mark.parametrize('depth,shown', [
    (0.55, False), (0.551, False), (0.67, False), (0.7, False),
    (0.700001, True), (1.0, True),
    (1.001, False), (None, False), (float('nan'), False),
])
def test_hurdle_recognition_label_preserves_line_driving(depth, shown):
    info = {
        'detected': True, 'confirmation_confirmed': True,
        'depth_valid': True, 'depth_m': depth,
    }
    label = Yolo26Detector._hurdle_line_tracking_label(info, 'line', 'LINE FORWARD 6')
    assert bool(label) is shown
    if shown:
        assert label == 'HURDLE / LINE_FORWARD 6'
    assert Yolo26Detector._hurdle_line_tracking_label(info, 'hurdle', 'FINE FORWARD') is None
    for key in ('detected', 'confirmation_confirmed', 'depth_valid'):
        assert Yolo26Detector._hurdle_line_tracking_label(
            {**info, key: False}, 'line', 'LINE FORWARD 6',
        ) is None
    assert Yolo26Detector._hurdle_line_tracking_label(None, 'line', 'LINE FORWARD 6') is None


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


@pytest.mark.parametrize("side", ["LEFT", "RIGHT"])
@pytest.mark.parametrize("turn,angle", [("LEFT", 30), ("RIGHT", 20)])
def test_line_recover_banner_uses_fixed_four_repeat_angle(side, turn, angle):
    banner = Yolo26Detector._action_banner("line", f"RECOVER_{side}_TURN_{turn}_4")
    assert banner is not None
    assert banner[0] == f"RECOVER {side} / TURN {turn} ({angle} DEG)"


@pytest.mark.parametrize("source,expected", [("line", "hurdle"), ("hurdle", "hurdle"), ("ball", "ball")])
def test_hurdle_metrics_show_during_line_approach_without_changing_owner(source, expected):
    detector = object.__new__(Yolo26Detector)
    detector.metrics_mode = "auto"
    detector.show_line_metrics = detector.show_ball_metrics = True
    detector.show_goal_metrics = detector.show_hurdle_metrics = True
    detector._fresh_motion_command = lambda: None
    detector.latest_running_motion = {"motion_id": "line_forward_6", "source": source}
    detector.latest_running_motion_time = time.monotonic()
    detector._fresh_hurdle_info = lambda: {"detected": True, "depth_m": 0.66}
    assert detector._active_metrics_mode() == expected
    assert detector.latest_running_motion["source"] == source


def test_hurdle_metrics_expose_actual_phase_and_rgb_head_trigger(monkeypatch):
    import cv2
    detector = object.__new__(Yolo26Detector)
    detector.show_hurdle_metrics = True
    detector._fresh_hurdle_info = lambda: {
        "detected": True, "center_x": 450, "center_y": 599,
        "camera_center_offset_x_px": -190, "bottom_distance_px": 120,
        "head_down_requested": True, "head_down_trigger_bottom_distance_px": 120,
        "confirmation_confirmed": True, "depth_valid": False,
    }
    detector._fresh_motion_command = lambda: None
    detector._fresh_line_info = lambda: None
    detector._fresh_decision_debug = lambda: {"source": "LINE", "phase": "AUTO"}
    detector._draw_line_path_geometry = lambda *args: None
    detector._draw_hurdle_path_reference = lambda *args: None
    rows = []
    original = cv2.putText
    def capture(image, text, origin, *args, **kwargs):
        rows.append((text, origin[1]))
        return original(image, text, origin, *args, **kwargs)
    monkeypatch.setattr(cv2, "putText", capture)
    detector._draw_hurdle_metrics(np.zeros((720, 1280, 3), dtype=np.uint8))
    assert any("Control" in text and "LINE" in text for text, _ in rows)
    assert any("Phase" in text and "AUTO" in text for text, _ in rows)
    assert any("Bottom dy" in text and "120px" in text for text, _ in rows)
    assert any("Head request" in text and "YES" in text for text, _ in rows)
    assert all(y > 96 for text, y in rows)


@pytest.mark.parametrize('action,reason,label', [
    ('LINE_LOST_TURN_LEFT', 'line_lost_turn_toward_last_seen_side', 'LINE LOST | SEARCH LEFT'),
    ('LINE_LOST_TURN_RIGHT', 'line_lost_turn_toward_last_seen_side', 'LINE LOST | SEARCH RIGHT'),
    ('LINE_LOST_TURN_LEFT_3', 'line_lost_turn_toward_last_seen_side', 'LINE LOST | SEARCH LEFT'),
    ('LINE_LOST_TURN_RIGHT_2', 'line_lost_turn_toward_last_seen_side', 'LINE LOST | SEARCH RIGHT'),
    ('WAIT', 'lost_search_turn_limit_reached', 'LINE LOST | SEARCH LIMIT'),
    ('WAIT', 'no_fresh_detected_target', 'LINE LOST | WAIT'),
])
def test_line_loss_banner_reports_selected_search_or_hold(action, reason, label):
    debug = {'source': 'LINE', 'fresh_vision': {'line': True},
             'line': {'line_detected': False},
             'decision': {'selected_action': action, 'reason': reason}}
    assert Yolo26Detector._line_lost_banner(debug) == label
    debug['fresh_vision']['line'] = False
    assert Yolo26Detector._line_lost_banner(debug) is None
    debug['fresh_vision']['line'] = True
    debug['line']['line_detected'] = True
    assert Yolo26Detector._line_lost_banner(debug) is None


@pytest.mark.parametrize('direction,count', [('LEFT', 1), ('LEFT', 3), ('RIGHT', 2), ('RIGHT', 5)])
@pytest.mark.parametrize('suffix', [False, True])
def test_running_line_search_keeps_line_lost_label(direction, count, suffix):
    detector = object.__new__(Yolo26Detector)
    detector.latest_running_motion = {
        'motion_id': f'line_search_{direction.lower()}_{count}',
        'action': f'LINE_LOST_TURN_{direction}' + (f'_{count}' if suffix else ''),
    }
    detector.latest_running_motion_time = time.monotonic()
    assert detector._running_motion_banner()[0] == f'LINE LOST | SEARCH {direction}'


def test_raw_grab_detection_does_not_appear_as_a_final_result():
    detector = object.__new__(Yolo26Detector)
    detector._active_metrics_mode = lambda: 'ball'
    for method in (
        '_fresh_motion_command', '_fresh_ball_info', '_recent_ball_info',
        '_fresh_goal_info', '_fresh_hurdle_info', '_fresh_line_info', '_fresh_decision_debug',
    ):
        setattr(detector, method, lambda: None)
    for method in ('_draw_ball_metrics', '_draw_grasp_verification_status'):
        setattr(detector, method, lambda image: None)
    detector.active_provider = 'test'
    detector.smoothed_fps = 15.
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    detection = Detection(class_id=5, class_name='grab', confidence=.95,
                          bbox=[100, 150, 300, 350], center=[200, 250])
    assert np.array_equal(detector._draw_detections(frame, [detection]),
                          detector._draw_detections(frame, []))


@pytest.mark.parametrize('phase,locked,visible', [
    ('AUTO', False, True), ('LINE_TRACK', False, True),
    ('HURDLE_POSITIONING_LOCK', True, False), ('GOAL_APPROACH_LOCK', True, False),
])
def test_line_loss_overlay_does_not_claim_another_missions_wait(phase, locked, visible):
    debug = {'source': 'NONE', 'phase': phase, 'execution': {'mission_locked': locked},
             'fresh_vision': {'line': True}, 'line': {'line_detected': False},
             'decision': {'selected_action': 'WAIT'}}
    assert (Yolo26Detector._line_lost_banner(debug) is not None) == visible


@pytest.mark.parametrize('action,reason,expected', [
    ('BALL_APPROACH_TURN_RIGHT_5', '', 'SEARCH RIGHT'),
    ('BALL_PICKUP_CAMERA_DOWN_TURN_LEFT_2', '', 'SEARCH LEFT'),
    ('BALL_PICKUP_FINE_SEARCH_RIGHT', '', 'SEARCH RIGHT'),
    ('BALL_PICKUP_FINE_SEARCH_BACKWARD', '', 'SEARCH BACKWARD'),
    ('BALL_PICKUP_INITIAL_SEARCH_FORWARD_4', '', 'SEARCH FORWARD'),
    ('WAIT', 'lost_search_turn_limit_reached', 'SEARCH LIMIT'),
    ('WAIT', 'ball_post_motion_dwell', 'WAIT'),
])
def test_ball_lost_banner_follows_selected_command(action, reason, expected):
    assert Yolo26Detector._ball_lost_banner({
        'ball_tracking': {'lost': True, 'last_direction': 'RIGHT'},
        'decision': {'selected_action': action, 'reason': reason},
    }) == f'BALL LOST | {expected}'
