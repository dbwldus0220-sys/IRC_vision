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
        "depth_m": 0.1,
        "distance_m": 0.1,
        "ground_gap_m": 0.1,
        "camera_bottom_gap_m": 0.02,
        "depth_valid": True,
        "hurdle_angle_deg": 0.0,
        "bottom_distance_px": 200,
    }
    sample.update(overrides)
    return sample


@pytest.mark.parametrize("angle,action", [
    (-70.0, "ALIGN_LEFT"), (-69.999, "STRAIGHT_0"),
    (0.0, "STRAIGHT_0"), (69.999, "STRAIGHT_0"), (70.0, "ALIGN_RIGHT"),
])
@pytest.mark.parametrize("offset", [-0.5, 0.5])
def test_center_approach_ignores_parallel_angle_and_path_offset(angle, action, offset):
    decision = HurdleNavigationPlanner().plan(hurdle_info(
        depth_m=0.5, hurdle_angle_deg=60, bearing_deg=angle,
        path_reference_valid=True, path_offset_x_norm=offset,
    ), positioning=True)
    assert decision.valid and decision.action == action


def test_positioning_starts_fixed_sequence_without_old_jump_confirmation():
    decision = HurdleNavigationPlanner().plan(hurdle_info(
        depth_m=0.20, hurdle_angle_deg=30.0, go_now=False,
    ), positioning=True)
    assert decision.action == "GO"
    assert decision.go_now and decision.sdk_motion_requested
    assert decision.fine_sequence_requested


def test_parallel_hurdle_at_target_depth_requests_sdk_motion():
    planner = HurdleNavigationPlanner()

    command = planner.plan(hurdle_info())

    assert command.valid is True
    assert command.action == "GO"
    assert command.go_now is True
    assert command.sdk_motion_requested is True


def test_hurdle_waits_until_analyzer_confirms_go_condition():
    planner = HurdleNavigationPlanner()

    command = planner.plan(hurdle_info(go_now=False))

    assert command.action == "WAIT_GO_CONFIRMATION"
    assert command.sdk_motion_requested is False


@pytest.mark.parametrize("depth", [0.001, 0.2])
def test_go_depth_tolerance_includes_boundary(depth):
    planner = HurdleNavigationPlanner()

    command = planner.plan(hurdle_info(depth_m=depth))

    assert command.action == "GO"


@pytest.mark.parametrize(
    ("angle", "expected"),
    [(12.0, "ALIGN_LEFT"), (-12.0, "ALIGN_RIGHT")],
)
def test_hurdle_angle_selects_parallel_alignment_direction(angle, expected):
    planner = HurdleNavigationPlanner()

    command = planner.plan(hurdle_info(hurdle_angle_deg=angle))

    assert command.action == expected
    assert command.sdk_motion_requested is False


def test_horizontal_offset_does_not_affect_hurdle_action():
    planner = HurdleNavigationPlanner()

    command = planner.plan(hurdle_info(offset_x_norm=0.9))

    assert command.action == "GO"


@pytest.mark.parametrize(
    ("depth", "ground_gap", "expected"),
    [
        (0.45, 0.05, "STRAIGHT_0"),
        (0.05, 0.47, "GO"),
    ],
)
def test_raw_depth_selects_go_timing(
    depth,
    ground_gap,
    expected,
):
    planner = HurdleNavigationPlanner()

    command = planner.plan(
        hurdle_info(depth_m=depth, ground_gap_m=ground_gap)
    )

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


def test_missing_legacy_ground_gap_does_not_block_raw_depth_control():
    planner = HurdleNavigationPlanner()

    far = planner.plan(
        hurdle_info(
            depth_m=0.50,
            ground_gap_m=None,
            camera_bottom_gap_m=0.08,
        )
    )
    close = planner.plan(
        hurdle_info(
            depth_m=0.05,
            ground_gap_m=None,
            camera_bottom_gap_m=0.02,
        )
    )

    assert far.action == "STRAIGHT_0"
    assert far.go_now is False
    assert close.action == "GO"
    assert close.go_now is True


@pytest.mark.parametrize("alignment,far_action", [
    ({"hurdle_angle_deg": 12.0}, "ALIGN_LEFT"),
    ({"hurdle_angle_deg": -12.0}, "ALIGN_RIGHT"),
    ({"path_reference_valid": True, "path_offset_x_norm": 0.2}, "TURN_RIGHT"),
    ({"path_reference_valid": True, "path_offset_x_norm": -0.2}, "TURN_LEFT"),
])
@pytest.mark.parametrize("bottom", [99, 100, 101])
def test_close_pixel_boundary_replaces_both_turn_types_with_fine_approach(
    alignment, far_action, bottom,
):
    planner = HurdleNavigationPlanner()
    command = planner.plan(hurdle_info(
        depth_m=0.4, bottom_distance_px=bottom, **alignment,
    ))
    assert command.valid
    assert command.action == ("STRAIGHT_0" if bottom <= 100 else far_action)
    assert command.close_rotation_blocked == (bottom <= 100)
    assert not command.sdk_motion_requested


@pytest.mark.parametrize("depth,expected", [
    (0.15, "STRAIGHT_0"), (0.550, "STRAIGHT_0"),
    (0.550001, "STRAIGHT_0"), (0.7, "STRAIGHT_0"),
    (0.700001, "STRAIGHT"), (1.0, "STRAIGHT"),
])
def test_near_misaligned_hurdle_uses_depth_without_requesting_go(depth, expected):
    command = HurdleNavigationPlanner().plan(hurdle_info(
        depth_m=depth, bottom_distance_px=100, hurdle_angle_deg=12.0,
        go_now=True,
    ))
    assert command.action == expected
    assert command.valid and not command.go_now
    assert not command.sdk_motion_requested


def test_close_turn_block_survives_missing_depth_detection_and_pixel_increase():
    planner = HurdleNavigationPlanner()
    command = planner.plan(hurdle_info(
        bottom_distance_px=100, depth_valid=False, depth_m=None,
    ))
    assert command.action == "WAIT"
    assert not command.depth_fallback_requested
    assert planner.close_rotation_blocked
    assert planner.plan({"detected": False}).action == "WAIT"
    command = planner.plan(hurdle_info(
        bottom_distance_px=150, depth_m=0.4, hurdle_angle_deg=20.0,
    ))
    assert command.action == "STRAIGHT_0"
    assert command.close_rotation_blocked
    planner.reset()
    command = planner.plan(hurdle_info(
        bottom_distance_px=150, depth_m=0.4, hurdle_angle_deg=20.0,
    ))
    assert command.action == "ALIGN_LEFT"


@pytest.mark.parametrize("bottom", [None, -1, float('nan'), True])
def test_missing_or_invalid_pixel_gap_cannot_authorize_a_turn(bottom):
    command = HurdleNavigationPlanner().plan(hurdle_info(
        bottom_distance_px=bottom, hurdle_angle_deg=20.0,
    ))
    assert command.action == "WAIT"
    assert not command.valid


def test_near_hurdle_at_go_geometry_waits_for_confirmation():
    planner = HurdleNavigationPlanner()
    waiting = planner.plan(hurdle_info(bottom_distance_px=100, go_now=False))
    assert waiting.action == "WAIT_GO_CONFIRMATION"
    ready = planner.plan(hurdle_info(bottom_distance_px=100, go_now=True))
    assert ready.action == "GO" and ready.sdk_motion_requested


@pytest.mark.parametrize("depth_valid,depth", [
    (False, None), (False, 0.4), (True, None), (True, 0),
    (True, -1), (True, float('nan')), (True, float('inf')),
])
def test_close_invalid_depth_cannot_start_final_sequence(depth_valid, depth):
    command = HurdleNavigationPlanner().plan(hurdle_info(
        depth_valid=depth_valid, depth_m=depth, bottom_distance_px=100,
        hurdle_angle_deg=None,
    ))
    assert command.action == "WAIT" and not command.sdk_motion_requested
    assert command.to_dict()["depth_fallback_requested"] is False


def test_close_valid_depth_with_missing_angle_still_requests_fine45():
    command = HurdleNavigationPlanner().plan(hurdle_info(
        bottom_distance_px=100, hurdle_angle_deg=None, depth_m=0.4,
    ))
    assert command.action == "STRAIGHT_0" and command.valid
    assert not command.depth_fallback_requested


def test_no_depth_outside_close_pixels_still_waits():
    command = HurdleNavigationPlanner().plan(hurdle_info(
        bottom_distance_px=101, depth_valid=False, depth_m=None,
    ))
    assert command.action == "WAIT" and not command.valid
