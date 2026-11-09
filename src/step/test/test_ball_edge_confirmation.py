"""Bound partial-edge recovery without relaxing other confirmation consumers."""

import pytest
from step.temporal_confirmation import TemporalConfirmationFilter

FRAME = dict(image_width=1280, image_height=720)
PART = [500, 0, 600, 20]
FULL = [500, 0, 600, 80]


def candidate():
    f = TemporalConfirmationFilter(window_size=30, required_hits=10,
                                   max_missed_frames=10, edge_area_grace=True)
    for _ in range(3):
        result = f.update(True, bbox=PART, **FRAME)
    return f, result.track_id


def test_uncertain_edge_frame_does_not_count_and_next_consistent_frame_recovers():
    f, track = candidate()
    pending = f.update(True, bbox=FULL, **FRAME)
    assert pending.hit_count == 3 and not pending.confirmed
    assert pending.spatial_pending and not pending.matched_previous
    assert pending.track_id == track
    result = f.update(True, bbox=[502, 0, 602, 82], **FRAME)
    assert result.hit_count == 4 and result.track_id == track
    assert not result.confirmed and not result.spatial_pending
    for _ in range(5):
        assert not f.update(True, bbox=FULL, **FRAME).confirmed
    assert f.update(True, bbox=FULL, **FRAME).confirmed


@pytest.mark.parametrize('box', [[900, 0, 1000, 80], [500, 300, 600, 380]])
def test_spatial_jump_cannot_keep_old_count(box):
    f, track = candidate()
    result = f.update(True, bbox=box, **FRAME)
    assert result.hit_count == 1 and result.track_id != track
    assert not result.spatial_pending


def test_next_miss_discards_pending_history():
    f, track = candidate()
    f.update(True, bbox=FULL, **FRAME)
    result = f.update(False)
    assert result.hit_count == 0 and result.track_id != track
    assert not result.spatial_pending


def test_next_inconsistent_size_does_not_extend_grace():
    f, track = candidate()
    f.update(True, bbox=FULL, **FRAME)
    result = f.update(True, bbox=[500, 0, 600, 250], **FRAME)
    assert result.hit_count == 1 and result.track_id != track
    assert not result.spatial_pending


def test_close_ball_disables_grace_and_confirmed_tracks_keep_old_matching():
    f, track = candidate()
    result = f.update(True, bbox=FULL, allow_edge_recovery=False, **FRAME)
    assert result.hit_count == 1 and result.track_id != track
    f, track = candidate()
    for _ in range(7):
        f.update(True, bbox=PART, **FRAME)
    result = f.update(True, bbox=FULL, **FRAME)
    assert result.hit_count == 1 and result.track_id != track


def test_default_filter_and_non_edge_area_changes_still_reset_immediately():
    f = TemporalConfirmationFilter(window_size=30, required_hits=10)
    f.update(True, bbox=PART, **FRAME)
    result = f.update(True, bbox=FULL, **FRAME)
    assert result.hit_count == 1
    assert 'confirmation_track_id' not in result.as_dict()
    f, _ = candidate()
    f.reset()
    f.update(True, bbox=[500, 200, 600, 220], **FRAME)
    result = f.update(True, bbox=[500, 200, 600, 280], **FRAME)
    assert result.hit_count == 1 and not result.spatial_pending


@pytest.mark.parametrize('depth,valid,expected_hits', [(.86, True, 3), (.55, True, 1),
                                                   (None, False, 1)])
def test_analyzer_publishes_track_identity_and_limits_grace_to_far_ball(depth, valid, expected_hits):
    from dataclasses import fields, replace
    import json
    from types import SimpleNamespace
    from std_msgs.msg import String
    from step.ball_analyzer import BallInfo
    from test_ball_analyzer_depth_separation import _analyzer_with_depth

    analyzer = _analyzer_with_depth(depth, valid)
    analyzer._last_depth_age_sec = .05
    analyzer.ball_class_name = 'ball'
    analyzer.publish_empty_when_missing = True
    analyzer.confirmation_filter, _ = candidate()
    analyzer.confirmation_filter.reset()
    analyzer.pickup_confirmation_filter = TemporalConfirmationFilter(window_size=5, required_hits=3)
    analyzer._image_size_from_payload = lambda *_: (1280, 720)
    analyzer._get_depth_cache = lambda: SimpleNamespace(latest=lambda: None)
    empty = BallInfo(**{field.name: None for field in fields(BallInfo)})
    analyzer._empty_info = lambda note: replace(empty, detected=False, confidence=0.,
        depth_valid=False, pickup_ready=False, pickup_now=False, note=note)
    messages = []
    analyzer.publisher = SimpleNamespace(publish=messages.append)
    for bbox in (PART, PART, PART, FULL):
        analyzer._detections_callback(String(data=json.dumps({'detections': [{
            'class_name': 'ball', 'confidence': .9, 'bbox': bbox,
            'center': [(bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2],
        }]})))
    previous = json.loads(messages[-2].data)
    current = json.loads(messages[-1].data)
    assert current['confirmation_hits'] == expected_hits
    assert current['confirmation_spatial_pending'] is (expected_hits == 3)
    assert (previous['confirmation_track_id'] == current['confirmation_track_id']) is (expected_hits == 3)
    assert current['detected'] is False and current['pickup_now'] is False
