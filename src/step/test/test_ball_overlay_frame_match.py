"""Keep delayed Ball geometry on its own RGB frame without delaying detection."""

from collections import deque
from types import SimpleNamespace

import numpy as np
import pytest

from step.yolo26_detector import Detection, Yolo26Detector


@pytest.fixture
def detector(monkeypatch):
    now = [10.]
    monkeypatch.setattr('step.yolo26_detector.time.monotonic', lambda: now[0])
    node = object.__new__(Yolo26Detector)
    node._overlay_frames = deque(maxlen=8)
    node.latest_ball_info = None
    node.latest_ball_info_time = None
    node.ball_info_timeout_sec = .8
    node.overlay_max_stamp_delta_sec = .05
    node._active_metrics_mode = lambda: 'ball'
    return node, now


def select(node, stamp):
    header = SimpleNamespace(stamp=SimpleNamespace(sec=stamp // 10**9, nanosec=stamp % 10**9))
    image = np.full((8, 8, 3), (stamp // 1_000_000) % 255, dtype=np.uint8)
    detections = [Detection(1, 'ball', .99, [1, 1, 5, 5], [3, 3])]
    return node._select_overlay_frame(image, detections, header)


def test_200ms_analyzer_delay_selects_the_matching_cached_rgb(detector):
    node, now = detector
    original = select(node, 10_000_000_000)
    for i in range(1, 6):
        now[0] = 10. + i * .033
        select(node, 10_000_000_000 + i * 33_000_000)
    node.latest_ball_info = {'rgb_stamp_ns': 10_000_000_000, 'detected': True}
    node.latest_ball_info_time = now[0]
    now[0] = 10.2
    matched = select(node, 10_200_000_000)
    assert matched[0] is original[0]
    assert matched[1] is original[1]
    assert matched[2] is original[2]
    node._overlay_rgb_stamp_ns = 10_000_000_000
    assert node._fresh_ball_info() == node.latest_ball_info
    assert node._overlay_display_lag_ms == 200.


def test_relaxed_tolerance_keeps_live_rgb_at_200ms_delay(detector):
    node, now = detector
    node.overlay_max_stamp_delta_sec = .30
    original = select(node, 10_000_000_000)
    node.latest_ball_info = {'rgb_stamp_ns': 10_000_000_000, 'detected': True}
    node.latest_ball_info_time = now[0]
    now[0] += .2
    current = select(node, 10_200_000_000)
    assert current[0] is not original[0]
    assert node._overlay_display_lag_ms == 0.
    node._overlay_rgb_stamp_ns = 10_200_000_000
    assert node._fresh_ball_info() == node.latest_ball_info


@pytest.mark.parametrize('gap', [.301, 1.])
def test_late_analysis_cannot_freeze_the_display(detector, gap):
    node, now = detector
    original = select(node, 10_000_000_000)
    now[0] += gap
    node.latest_ball_info = {'rgb_stamp_ns': 10_000_000_000}
    node.latest_ball_info_time = now[0]
    current = select(node, 10_000_000_000 + round(gap * 1e9))
    assert current[0] is not original[0]
    assert node._overlay_display_lag_ms == 0.


def test_line_view_does_not_delay_its_geometry_to_match_an_incidental_ball(detector):
    node, now = detector
    select(node, 10_000_000_000)
    node.latest_ball_info = {'rgb_stamp_ns': 10_000_000_000}
    node.latest_ball_info_time = now[0]
    node._active_metrics_mode = lambda: 'line'
    current = select(node, 10_200_000_000)
    assert current[2].stamp.nanosec == 200_000_000
    assert not node._overlay_frames


def test_cache_is_bounded_and_clock_reset_discards_old_rgb(detector):
    node, now = detector
    for i in range(20):
        now[0] += .01
        select(node, 10_000_000_000 + i * 10_000_000)
    assert len(node._overlay_frames) == 8
    select(node, 9_000_000_000)
    assert len(node._overlay_frames) == 1


def test_current_detections_publish_before_selecting_an_older_display_frame(detector):
    node, now = detector
    old = select(node, 10_000_000_000)
    node.latest_ball_info = {'rgb_stamp_ns': 10_000_000_000}
    node.latest_ball_info_time = now[0]
    current_header = SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=200_000_000))
    message = SimpleNamespace(header=current_header)
    events = []
    node.max_fps = 30.
    node.last_inference_time = 0.
    node.processing = False
    node.smoothed_fps = 0.
    node.display = False
    node.bridge = SimpleNamespace(
        imgmsg_to_cv2=lambda *a, **k: np.ones((8, 8, 3), dtype=np.uint8),
        cv2_to_imgmsg=lambda *a, **k: SimpleNamespace(),
    )
    node._preprocess = lambda image: (None, None)
    node._run_inference = lambda blob: None
    node._record_inference_timing = lambda *a: None
    node._postprocess = lambda *a: old[1]
    node._publish_detections = lambda msg, dets: events.append(('detection', msg.header))
    node._annotated_image_has_subscriber = lambda: True
    node._draw_detections = lambda image, dets: image.copy()
    node.annotated_publisher = SimpleNamespace(publish=lambda msg: events.append(('display', msg.header)))
    node.get_logger = lambda: SimpleNamespace(error=lambda msg: pytest.fail(msg))
    now[0] += .2
    node._image_callback(message)
    assert events == [('detection', current_header), ('display', old[2])]
    assert node._overlay_rgb_stamp_ns is None
