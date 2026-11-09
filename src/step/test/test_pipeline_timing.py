"""Timing is additive metadata and cannot authorize or reject control inputs."""

from dataclasses import fields
import json
from types import SimpleNamespace

import pytest

from step.pipeline_timing import (
    elapsed_ms, extend_trace, ball_pipeline_values, record_timing, record_ball_pipeline,
)


def trace():
    return dict(rgb_stamp_ns=1_000_000_000, detector_receive_ros_ns=1_050_000_000,
                detector_receive_mono_ns=10_000_000_000, detector_publish_mono_ns=10_030_000_000,
                ball_receive_mono_ns=10_050_000_000, ball_publish_mono_ns=10_060_000_000)


@pytest.mark.parametrize('start,end', [(None, 1), (True, 2), (2, 1), (0, 5), (1, '2')])
def test_invalid_clock_pairs_are_unknown_not_zero(start, end):
    assert elapsed_ms(start, end) is None


def test_stage_breakdown_separates_ros_capture_age_from_monotonic_handoffs():
    values = ball_pipeline_values(trace(), 10_100_000_000, 1_150_000_000, 10_105_000_000)
    assert values == dict(capture_to_detector=50., detector_work=30., detector_to_ball=20.,
                          ball_work=10., ball_to_decision=40., capture_to_decision=150.,
                          decision_callback=5.)


def test_trace_requires_same_frame_and_does_not_mutate_upstream_packet():
    original = trace()
    extended = extend_trace(original, original['rgb_stamp_ns'], ball_publish_mono_ns=11_000_000_000)
    assert original == trace()
    assert extended['ball_publish_mono_ns'] == 11_000_000_000
    assert extend_trace(original, 2_000_000_000) is None
    assert extend_trace({}, None) is None


def test_summary_is_bounded_and_aggregates_without_storing_frames():
    messages = []
    node = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=messages.append))
    for index, ms in enumerate((10., 20., 30.), 1):
        record_timing(node, '_timing', 'TIMING', {'work': ms}, {'rgb_stamp_ns': index}, index*10**9)
        if index < 3:
            assert not messages
    assert len(messages) == 1
    output = json.loads(messages[0].split('] ', 1)[1])
    assert output['samples'] == 3
    assert output['summary_ms']['work'] == dict(n=3, avg=20., max=30.)
    assert node._timing['metrics'] == {}


def test_first_confirmation_logs_immediately_without_logging_every_frame(monkeypatch):
    messages = []
    node = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=messages.append))
    now = [10_105_000_000]
    monkeypatch.setattr('step.pipeline_timing.time.monotonic_ns', lambda: now[0])
    payload = dict(rgb_stamp_ns=1_000_000_000, diagnostic_timing=trace(),
                   detected=True, confirmation_confirmed=True, confirmation_track_id=7,
                   confirmation_hits=10, raw_detected=True)
    record_ball_pipeline(node, payload, 10_100_000_000, 1_150_000_000, 'accepted')
    assert len(messages) == 1
    output = json.loads(messages[0].split('] ', 1)[1])
    assert output['confirmation_event'] and output['hits'] == 10
    assert output['latest_ms']['ball_to_decision'] == 40.
    record_ball_pipeline(node, payload, 10_100_000_000, 1_150_000_000, 'accepted')
    assert len(messages) == 1


def test_logging_failure_never_raises_into_control():
    def fail(_):
        raise RuntimeError('logger unavailable')
    node = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=fail))
    record_timing(node, '_timing', 'TIMING', {'work': 10.}, {}, 10**9, force=True)


def test_detector_and_ball_publications_add_only_diagnostic_metadata(monkeypatch):
    from step.yolo26_detector import Yolo26Detector
    from step.ball_analyzer import BallAnalyzer, BallInfo

    monkeypatch.setattr('step.pipeline_timing.time.monotonic_ns', lambda: 10_030_000_000)
    detector = object.__new__(Yolo26Detector)
    messages = []
    detector.detections_publisher = SimpleNamespace(publish=messages.append)
    image = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0),
                                                    frame_id='camera'), width=1280, height=720)
    detector._publish_detections(image, [])
    plain = json.loads(messages[-1].data)
    detector._diagnostic_frame_timing = {k: v for k, v in trace().items() if k.startswith('detector_receive') or k == 'rgb_stamp_ns'}
    detector._publish_detections(image, [])
    timed = json.loads(messages[-1].data)
    metadata = timed.pop('diagnostic_timing')
    assert timed == plain
    assert metadata['detector_publish_mono_ns'] == 10_030_000_000

    analyzer = object.__new__(BallAnalyzer)
    analyzer.publisher = SimpleNamespace(publish=messages.append)
    analyzer.confirmation_fields = dict(confirmation_hits=9, confirmation_confirmed=False)
    analyzer.pickup_confirmation_fields = {}
    values = {field.name: None for field in fields(BallInfo)}
    values.update(rgb_stamp_ns=1_000_000_000, detected=False, depth_valid=False, pickup_now=False)
    info = BallInfo(**values)
    analyzer._publish(info)
    plain = json.loads(messages[-1].data)
    analyzer._active_diagnostic_timing = extend_trace(metadata, 1_000_000_000,
                                                     ball_receive_mono_ns=10_050_000_000)
    monkeypatch.setattr('step.pipeline_timing.time.monotonic_ns', lambda: 10_060_000_000)
    analyzer._publish(info)
    timed = json.loads(messages[-1].data)
    metadata = timed.pop('diagnostic_timing')
    assert timed == plain
    assert metadata['ball_receive_mono_ns'] == 10_050_000_000
    assert metadata['ball_publish_mono_ns'] == 10_060_000_000
