"""Unit tests for hardware-independent hurdle action decisions."""

import pytest

from step.hurdle_navigation_planner import HurdleNavigationPlanner


def hurdle_info(**overrides):
    """Create one valid hurdle analysis sample with optional changes."""
    sample = {
        "detected": True,
        "confidence": 0.9,
        "bearing_deg": 0.0,
        "offset_x_norm": 0.0,
        "depth_m": 0.8,
        "distance_m": 0.8,
        "depth_valid": True,
        "hurdle_angle_deg": 0.0,
    }
    sample.update(overrides)
    return sample


def test_centered_hurdle_at_target_depth_requests_sdk_motion():
    planner = HurdleNavigationPlanner()

    command = planner.plan(hurdle_info())

    assert command.valid is True
    assert command.action == "GO"
    assert command.go_now is True
    assert command.sdk_motion_requested is True


@pytest.mark.parametrize("depth", [0.7, 0.9])
def test_go_depth_tolerance_includes_boundary(depth):
    planner = HurdleNavigationPlanner()

    command = planner.plan(hurdle_info(depth_m=depth))

    assert command.action == "GO"


@pytest.mark.parametrize(
    ("offset", "expected"),
    [(0.2, "ALIGN_RIGHT"), (-0.2, "ALIGN_LEFT")],
)
def test_horizontal_offset_selects_alignment_direction(offset, expected):
    planner = HurdleNavigationPlanner()

    command = planner.plan(hurdle_info(offset_x_norm=offset))

    assert command.action == expected
    assert command.sdk_motion_requested is False


@pytest.mark.parametrize(
    ("depth", "expected"),
    [(1.0, "APPROACH_HURDLE"), (0.6, "RETREAT_HURDLE")],
)
def test_depth_outside_go_range_selects_adjustment(depth, expected):
    planner = HurdleNavigationPlanner()

    command = planner.plan(hurdle_info(depth_m=depth))

    assert command.action == expected


@pytest.mark.parametrize(
    ("sample", "reason"),
    [
        ({"detected": False}, "hurdle_not_detected"),
        (hurdle_info(confidence=0.1), "low_hurdle_confidence"),
        (
            hurdle_info(depth_m=None, depth_valid=False),
            "missing_valid_hurdle_depth",
        ),
    ],
)
def test_unsafe_input_produces_wait(sample, reason):
    planner = HurdleNavigationPlanner()

    command = planner.plan(sample)

    assert command.valid is False
    assert command.action == "WAIT"
    assert command.reason == reason
    assert command.sdk_motion_requested is False
