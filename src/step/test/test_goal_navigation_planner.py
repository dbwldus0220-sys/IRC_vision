"""Unit tests for hardware-independent goal navigation decisions."""

import pytest

from step.goal_navigation_planner import GoalNavigationPlanner


def goal_info(**overrides):
    """Create one valid backboard-based goal sample."""
    sample = {
        "detected": True,
        "confidence": 0.9,
        "depth_valid": True,
        "depth_m": 0.795,
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


def test_centered_goal_farther_than_score_range_uses_fine_forward():
    planner = GoalNavigationPlanner()

    command = planner.plan(goal_info(depth_m=0.90))

    assert command.valid is True
    assert command.action == "GOAL_CAMERA90_FINE_FORWARD_2"
    assert command.reason == "approach_goal_by_depth"


def test_centered_goal_at_795mm_requests_score_motion():
    planner = GoalNavigationPlanner()

    command = planner.plan(goal_info(depth_m=0.795))

    assert command.action == "SHOT"
    assert command.sdk_motion_requested is True


def test_goal_decision_uses_raw_depth_not_ground_distance():
    planner = GoalNavigationPlanner()

    command = planner.plan(
        goal_info(depth_m=0.795, ground_distance_m=0.25)
    )

    assert command.action == "SHOT"
    assert command.depth_m == 0.795


def test_goal_waits_until_analyzer_confirms_score_condition():
    planner = GoalNavigationPlanner()

    command = planner.plan(goal_info(score_now=False))

    assert command.action == "WAIT_SCORE_CONFIRMATION"
    assert command.sdk_motion_requested is False


def test_misaligned_goal_crabs_before_scoring():
    planner = GoalNavigationPlanner()

    command = planner.plan(
        goal_info(offset_x_px=-41, offset_x_norm=-0.2)
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


@pytest.mark.parametrize('depth', [0.77, 0.795, 0.82])
@pytest.mark.parametrize('offset_px', [-40, 0, 100])
def test_inclusive_shot_rectangle(depth, offset_px):
    command = GoalNavigationPlanner().plan(goal_info(depth_m=depth, offset_x_px=offset_px))
    assert command.action == 'SHOT'


@pytest.mark.parametrize('offset_px,action', [
    (-41, 'GOAL_CAMERA90_CRAB_LEFT'), (101, 'GOAL_CAMERA90_CRAB_RIGHT'),
])
def test_outside_pixel_bounds_never_shoots(offset_px, action):
    command = GoalNavigationPlanner().plan(goal_info(offset_x_px=offset_px))
    assert command.action == action
    assert command.score_now is False


@pytest.mark.parametrize('overrides', [
    {'depth_m': 0.769}, {'depth_m': 0.821}, {'offset_x_px': None},
    {'offset_x_px': float('nan')}, {'score_now': None}, {'score_now': False},
])
def test_invalid_or_unconfirmed_shot_is_blocked(overrides):
    assert GoalNavigationPlanner().plan(goal_info(**overrides)).action != 'SHOT'


@pytest.mark.parametrize('depth,action', [
    (2.0, 'GOAL_CAMERA_90_FORWARD'),
    (1.281, 'GOAL_CAMERA_90_FORWARD'),
    (1.280, 'GOAL_CAMERA_90_FORWARD_2'),
    (1.026, 'GOAL_CAMERA_90_FORWARD_2'),
    (1.025, 'GOAL_CAMERA90_FINE_FORWARD_4'),
    (0.986, 'GOAL_CAMERA90_FINE_FORWARD_4'),
    (0.985, 'GOAL_CAMERA90_FINE_FORWARD_3'),
    (0.951, 'GOAL_CAMERA90_FINE_FORWARD_3'),
    (0.950, 'GOAL_CAMERA90_FINE_FORWARD_2'),
    (0.876, 'GOAL_CAMERA90_FINE_FORWARD_2'),
    (0.875, 'GOAL_CAMERA90_FINE_FORWARD_1'),
    (0.821, 'GOAL_CAMERA90_FINE_FORWARD_1'),
    (0.820, 'WAIT_SCORE_CONFIRMATION'),
    (0.770, 'WAIT_SCORE_CONFIRMATION'),
    (0.769, 'GOAL_CAMERA90_BACKWARD_1'),
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
        depth_m=0.769, offset_x_px=200, score_now=False,
    ))
    assert command.action == 'GOAL_CAMERA90_BACKWARD_1'
    assert command.reason == 'retreat_goal_to_scoring_depth'


@pytest.mark.parametrize('offset_px', [-200, 200])
@pytest.mark.parametrize('depth,action', [
    (2.0, 'GOAL_CAMERA_90_FORWARD'),
    (1.280, 'GOAL_CAMERA_90_FORWARD_2'),
    (1.025, 'GOAL_CAMERA90_FINE_FORWARD_4'),
    (0.985, 'GOAL_CAMERA90_FINE_FORWARD_3'),
    (0.950, 'GOAL_CAMERA90_FINE_FORWARD_2'),
    (0.875, 'GOAL_CAMERA90_FINE_FORWARD_1'),
    (0.821, 'GOAL_CAMERA90_FINE_FORWARD_1'),
])
def test_off_center_approach_uses_depth_bucket_without_crab(depth, action, offset_px):
    command = GoalNavigationPlanner().plan(goal_info(
        depth_m=depth, offset_x_px=offset_px, score_now=False,
    ))
    assert command.action == action
    assert command.is_centered is False
    assert command.depth_in_score_range is False


@pytest.mark.parametrize('depth', [0.77, 0.795, 0.82])
@pytest.mark.parametrize('offset_px,direction', [(-41, 'LEFT'), (101, 'RIGHT')])
def test_crab_is_reserved_for_inclusive_scoring_range(depth, offset_px, direction):
    command = GoalNavigationPlanner().plan(goal_info(
        depth_m=depth, offset_x_px=offset_px, score_now=False,
    ))
    assert command.action == f'GOAL_CAMERA90_CRAB_{direction}'
