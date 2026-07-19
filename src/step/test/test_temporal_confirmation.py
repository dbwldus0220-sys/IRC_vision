"""Unit tests for reusable vision confirmation filtering."""

from step.temporal_confirmation import TemporalConfirmationFilter


FRAME = {"image_width": 1280, "image_height": 720}


def update(filter_, detected, bbox=None):
    """Update one frame using the standard test resolution."""
    return filter_.update(detected, bbox=bbox, **FRAME)


def test_three_of_five_consistent_hits_confirm_target():
    filter_ = TemporalConfirmationFilter(
        window_size=5,
        required_hits=3,
    )
    bbox = [500, 250, 600, 350]

    assert update(filter_, True, bbox).confirmed is False
    assert update(filter_, False).confirmed is False
    assert update(filter_, True, bbox).confirmed is False
    result = update(filter_, True, bbox)

    assert result.confirmed is True
    assert result.hit_count == 3


def test_spatial_jump_starts_a_new_candidate_history():
    filter_ = TemporalConfirmationFilter(
        window_size=5,
        required_hits=3,
        max_center_shift_norm=0.10,
    )

    update(filter_, True, [100, 100, 200, 200])
    update(filter_, True, [105, 100, 205, 200])
    jumped = update(filter_, True, [900, 450, 1000, 550])

    assert jumped.confirmed is False
    assert jumped.hit_count == 1


def test_excessive_size_change_starts_new_candidate_history():
    filter_ = TemporalConfirmationFilter(
        window_size=3,
        required_hits=2,
        min_area_ratio=0.5,
    )

    update(filter_, True, [500, 250, 600, 350])
    changed = update(filter_, True, [510, 260, 530, 280])

    assert changed.confirmed is False
    assert changed.hit_count == 1


def test_short_miss_keeps_history_for_fast_reacquisition():
    filter_ = TemporalConfirmationFilter(
        window_size=5,
        required_hits=3,
        max_missed_frames=2,
    )
    bbox = [500, 250, 600, 350]

    update(filter_, True, bbox)
    update(filter_, True, bbox)
    assert update(filter_, True, bbox).confirmed is True
    assert update(filter_, False).confirmed is False
    reacquired = update(filter_, True, bbox)

    assert reacquired.confirmed is True


def test_boolean_condition_uses_same_filter_without_bbox():
    filter_ = TemporalConfirmationFilter(
        window_size=3,
        required_hits=2,
        max_missed_frames=0,
        spatial_matching=False,
    )

    assert filter_.update(True).confirmed is False
    assert filter_.update(True).confirmed is True
    assert filter_.update(False).confirmed is False
