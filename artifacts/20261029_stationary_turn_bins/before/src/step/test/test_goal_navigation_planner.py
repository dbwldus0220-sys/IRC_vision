"""Unit tests for hardware-independent goal navigation decisions."""

import pytest

from step.goal_navigation_planner import GoalNavigationPlanner


def goal_info(**overrides):
    """Create one valid backboard-based goal sample."""
    sample = {
        "detected": True,
        "confidence": 0.9,
        "depth_valid": True,
        "depth_m": 0.43,
        "ground_distance_m": 0.5,
        "distance_m": 0.5,
        "bearing_deg": 0.0,
        "offset_x_norm": 0.0,
        "offset_x_px": 0,
        "score_now": True,
    }
    sample.update(overrides)
    if "ground_distance_m" not in overrides and "depth_m" in overrides:
        sample["ground_distance_m"] = overrides["depth_m"]
    return sample


def test_goal_outside_tracking_range_waits():
    planner = GoalNavigationPlanner()

    command = planner.plan(goal_info(depth_m=2.01))

    assert command.valid is False
    assert command.action == "WAIT"
    assert command.reason == "goal_outside_control_range"


def test_goal_without_valid_depth_still_waits():
    planner = GoalNavigationPlanner()
    command = planner.plan(goal_info(depth_valid=False, depth_m=None))
    assert command.valid is False
    assert command.action == "WAIT"
    assert command.reason == "missing_valid_goal_depth"


def test_centered_goal_at_900mm_uses_four_cycle_forward():
    planner = GoalNavigationPlanner()

    command = planner.plan(goal_info(depth_m=0.90))

    assert command.valid is True
    assert command.action == "GOAL_CAMERA_90_FORWARD_2"
    assert command.reason == "approach_goal_by_depth"


def test_centered_goal_at_430mm_requests_score_motion():
    planner = GoalNavigationPlanner()

    command = planner.plan(goal_info(depth_m=0.43))

    assert command.action == "SHOT"
    assert command.sdk_motion_requested is True


def test_goal_decision_uses_raw_depth_not_ground_distance():
    planner = GoalNavigationPlanner()

    command = planner.plan(
        goal_info(depth_m=0.43, ground_distance_m=0.25)
    )

    assert command.action == "SHOT"
    assert command.depth_m == 0.43


def test_goal_waits_until_analyzer_confirms_score_condition():
    planner = GoalNavigationPlanner()

    command = planner.plan(goal_info(score_now=False))

    assert command.action == "WAIT_SCORE_CONFIRMATION"
    assert command.sdk_motion_requested is False


def test_misaligned_goal_crabs_before_scoring():
    planner = GoalNavigationPlanner()

    command = planner.plan(
        goal_info(offset_x_px=-91, offset_x_norm=-0.2)
    )

    assert command.action == "GOAL_CAMERA90_CRAB_LEFT"
    assert command.sdk_motion_requested is False


def test_centered_goal_below_scoring_range_retreats():
    planner = GoalNavigationPlanner()
    depth = (
        planner.config.score_target_depth_m
        - planner.config.score_depth_tolerance_m
        - 0.001
    )
    command = planner.plan(
        goal_info(depth_m=depth, distance_m=0.18, score_now=False)
    )
    assert command.valid is True
    assert command.action == "GOAL_CAMERA90_BACKWARD_1"
    assert command.reason == "retreat_goal_to_scoring_depth"
    assert command.sdk_motion_requested is False


def test_scoring_lower_boundary_is_inclusive_before_retreat():
    planner = GoalNavigationPlanner()
    boundary = (
        planner.config.score_target_depth_m
        - planner.config.score_depth_tolerance_m
    )
    at_boundary = planner.plan(goal_info(depth_m=boundary, score_now=False))
    below_boundary = planner.plan(
        goal_info(depth_m=boundary - 0.001, score_now=False)
    )
    assert at_boundary.action == "WAIT_SCORE_CONFIRMATION"
    assert below_boundary.action == "GOAL_CAMERA90_BACKWARD_1"


@pytest.mark.parametrize('depth', [0.39, 0.43, 0.47])
@pytest.mark.parametrize('offset_px', [-90, -70, 0, 70, 90])
def test_inclusive_shot_rectangle(depth, offset_px):
    command = GoalNavigationPlanner().plan(goal_info(depth_m=depth, offset_x_px=offset_px))
    assert command.action == 'SHOT'


@pytest.mark.parametrize('offset_px,action', [
    (-91, 'GOAL_CAMERA90_CRAB_LEFT'), (91, 'GOAL_CAMERA90_CRAB_RIGHT'),
])
def test_outside_pixel_bounds_never_shoots(offset_px, action):
    command = GoalNavigationPlanner().plan(goal_info(offset_x_px=offset_px))
    assert command.action == action
    assert command.score_now is False


@pytest.mark.parametrize('overrides', [
    {'depth_m': 0.389}, {'depth_m': 0.471}, {'offset_x_px': None},
    {'offset_x_px': float('nan')}, {'score_now': None}, {'score_now': False},
])
def test_invalid_or_unconfirmed_shot_is_blocked(overrides):
    assert GoalNavigationPlanner().plan(goal_info(**overrides)).action != 'SHOT'


@pytest.mark.parametrize('depth,action', [
    (2.0, 'GOAL_CAMERA_90_FORWARD'),
    (1.281, 'GOAL_CAMERA_90_FORWARD'),
    (1.280, 'GOAL_CAMERA_90_FORWARD_2'),
    (0.851, 'GOAL_CAMERA_90_FORWARD_2'),
    (0.850, 'GOAL_CAMERA90_FINE_FORWARD_3'),
    (0.501, 'GOAL_CAMERA90_FINE_FORWARD_3'),
    (0.500, 'GOAL_CAMERA90_FINE_FORWARD_1'),
    (0.470, 'WAIT_SCORE_CONFIRMATION'),
    (0.471, 'GOAL_CAMERA90_FINE_FORWARD_1'),
    (0.430, 'WAIT_SCORE_CONFIRMATION'),
    (0.410, 'WAIT_SCORE_CONFIRMATION'),
    (0.390, 'WAIT_SCORE_CONFIRMATION'),
    (0.389, 'GOAL_CAMERA90_BACKWARD_1'),
])
def test_depth_buckets_use_depth_z_and_include_upper_bound(depth, action):
    command = GoalNavigationPlanner().plan(goal_info(
        depth_m=depth, distance_m=0.2, ground_distance_m=0.1, score_now=False,
    ))
    assert command.action == action
    assert command.score_now is False


@pytest.mark.parametrize('depth', [None, 0.0, -1.0, float('nan'), float('inf')])
def test_invalid_depth_never_advances(depth):
    command = GoalNavigationPlanner().plan(goal_info(depth_m=depth))
    assert command.action == 'WAIT'
    assert command.valid is False


def test_too_close_and_off_center_retreats_before_lateral_alignment():
    command = GoalNavigationPlanner().plan(goal_info(
        depth_m=0.389, offset_x_px=200, score_now=False,
    ))
    assert command.action == 'GOAL_CAMERA90_BACKWARD_1'
    assert command.reason == 'retreat_goal_to_scoring_depth'


@pytest.mark.parametrize('offset_px', [-200, 200])
@pytest.mark.parametrize('depth,action', [
    (2.0, 'GOAL_CAMERA_90_FORWARD'),
    (1.280, 'GOAL_CAMERA_90_FORWARD_2'),
    (0.851, 'GOAL_CAMERA_90_FORWARD_2'),
    (0.850, 'GOAL_CAMERA90_FINE_FORWARD_3'),
    (0.501, 'GOAL_CAMERA90_FINE_FORWARD_3'),
    (0.500, 'GOAL_CAMERA90_FINE_FORWARD_1'),
    (0.471, 'GOAL_CAMERA90_FINE_FORWARD_1'),
])
def test_off_center_approach_uses_depth_bucket_without_crab(depth, action, offset_px):
    command = GoalNavigationPlanner().plan(goal_info(
        depth_m=depth, offset_x_px=offset_px, score_now=False,
    ))
    assert command.action == action
    assert command.is_centered is False
    assert command.depth_in_score_range is False


@pytest.mark.parametrize('depth', [0.39, 0.43, 0.47])
@pytest.mark.parametrize('offset_px,direction', [(-91, 'LEFT'), (91, 'RIGHT')])
def test_crab_is_reserved_for_inclusive_scoring_range(depth, offset_px, direction):
    command = GoalNavigationPlanner().plan(goal_info(
        depth_m=depth, offset_x_px=offset_px, score_now=False,
    ))
    assert command.action == f'GOAL_CAMERA90_CRAB_{direction}'


@pytest.mark.parametrize('depth', [0.471, 0.480, 0.500])
def test_fine_approach_overrides_stale_score_confirmation(depth):
    command = GoalNavigationPlanner().plan(goal_info(depth_m=depth, score_now=True))
    assert command.action == 'GOAL_CAMERA90_FINE_FORWARD_1'
    assert command.score_now is False
    assert command.sdk_motion_requested is False


@pytest.mark.parametrize("depth", [0.39, 0.43, 0.47])
@pytest.mark.parametrize("offset,action", [
    (-250, "GOAL_CAMERA90_TURN_LEFT_1"),
    (-140.001, "GOAL_CAMERA90_TURN_LEFT_1"),
    (-140, "GOAL_CAMERA90_CRAB_LEFT"),
    (-90.001, "GOAL_CAMERA90_CRAB_LEFT"),
    (-90, "SHOT"), (0, "SHOT"), (90, "SHOT"),
    (90.001, "GOAL_CAMERA90_CRAB_RIGHT"),
    (140, "GOAL_CAMERA90_CRAB_RIGHT"),
    (140.001, "GOAL_CAMERA90_TURN_RIGHT_2"),
    (250, "GOAL_CAMERA90_TURN_RIGHT_2"),
])
def test_scoring_alignment_pixel_boundaries(depth, offset, action):
    command = GoalNavigationPlanner().plan(goal_info(
        depth_m=depth, offset_x_px=offset, score_now=True,
    ))
    assert command.action == action
    assert command.is_centered is (action == "SHOT")
    assert command.score_now is (action == "SHOT")


@pytest.mark.parametrize("overrides", [
    {"detected": False}, {"confidence": 0.1}, {"depth_valid": False},
    {"depth_m": None}, {"offset_x_px": None}, {"offset_x_px": float("inf")},
])
def test_large_pixel_alignment_does_not_bypass_input_validation(overrides):
    sample = goal_info(offset_x_px=200)
    sample.update(overrides)
    command = GoalNavigationPlanner().plan(sample)
    assert command.action == "WAIT"
    assert command.valid is False
