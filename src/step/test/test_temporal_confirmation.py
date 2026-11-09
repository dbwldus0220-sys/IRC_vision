"""Unit tests for reusable vision confirmation filtering."""

import ast
from pathlib import Path

from step.temporal_confirmation import TemporalConfirmationFilter


FRAME = {"image_width": 1280, "image_height": 720}


def test_ball_default_confirms_on_tenth_hit_without_changing_pickup_confirmation():
    # Use the node's actual defaults without starting ROS or camera subscriptions.
    source = Path(__file__).parents[1] / "step" / "ball_analyzer.py"
    declarations = {}
    for node in ast.walk(ast.parse(source.read_text())):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == "declare_parameter" and len(node.args) == 2
                and isinstance(node.args[0], ast.Constant)
                and isinstance(node.args[1], ast.Constant)):
            declarations[node.args[0].value] = node.args[1].value
    ball = TemporalConfirmationFilter(**{
        key: declarations[f"confirmation_{key}"]
        for key in ("window_size", "required_hits", "max_missed_frames",
                    "max_center_shift_norm", "min_area_ratio")
    })
    bbox = [500, 250, 600, 350]
    for _ in range(9):
        assert update(ball, True, bbox).confirmed is False
    assert update(ball, True, bbox).confirmed is True
    assert update(ball, False).confirmed is False
    assert update(ball, True, bbox).confirmed is True
    jumped = update(ball, True, [1000, 500, 1100, 600])
    assert jumped.hit_count == 1 and jumped.confirmed is False

    pickup = TemporalConfirmationFilter(
        window_size=declarations["pickup_confirmation_window_size"],
        required_hits=declarations["pickup_confirmation_required_hits"],
        max_missed_frames=0, spatial_matching=False,
    )
    assert pickup.update(True).confirmed is False
    assert pickup.update(True).confirmed is False
    assert pickup.update(True).confirmed is True
    assert pickup.update(False).confirmed is False


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


def test_hurdle_fifteen_of_forty_intermittent_hits_confirm_target():
    """Accumulate intermittent hits beyond the old 20-frame window."""
    filter_ = TemporalConfirmationFilter(
        window_size=40,
        required_hits=15,
        max_missed_frames=4,
    )
    bbox = [500, 250, 600, 350]

    for _ in range(14):
        assert update(filter_, True, bbox).confirmed is False
        assert update(filter_, False).confirmed is False
    for _ in range(3):
        assert update(filter_, False).confirmed is False

    result = update(filter_, True, bbox)

    assert result.confirmed is True
    assert result.hit_count == 15
    assert result.required_hits == 15
    assert result.window_size == 40


def test_hurdle_history_tolerates_four_misses_but_resets_on_fifth():
    """Keep history for four consecutive misses and reset on the fifth."""
    filter_ = TemporalConfirmationFilter(
        window_size=40,
        required_hits=15,
        max_missed_frames=4,
    )
    bbox = [500, 250, 600, 350]

    for _ in range(15):
        update(filter_, True, bbox)
    for missed_frames in range(1, 5):
        missed = update(filter_, False)
        assert missed.confirmed is False
        assert missed.missed_frames == missed_frames

    assert update(filter_, True, bbox).confirmed is True

    for _ in range(5):
        update(filter_, False)
    reacquired = update(filter_, True, bbox)

    assert reacquired.confirmed is False
    assert reacquired.hit_count == 1


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
