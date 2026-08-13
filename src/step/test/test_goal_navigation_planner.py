"""Unit tests for hardware-independent goal navigation decisions."""

from step.goal_navigation_planner import GoalNavigationPlanner


def goal_info(**overrides):
    """Create one valid backboard-based goal sample."""
    sample = {
        "detected": True,
        "confidence": 0.9,
        "depth_valid": True,
        "depth_m": 0.5,
        "distance_m": 0.5,
        "bearing_deg": 0.0,
        "offset_x_norm": 0.0,
    }
    sample.update(overrides)
    return sample


def test_aligned_goal_outside_control_range_approaches():
    planner = GoalNavigationPlanner()

    command = planner.plan(goal_info(depth_m=0.51))

    assert command.valid is True
    assert command.action == "APPROACH_GOAL"
    assert command.reason == "goal_aligned_coarse_approach"


def test_misaligned_goal_outside_control_range_aligns_before_approach():
    planner = GoalNavigationPlanner()

    command = planner.plan(
        goal_info(depth_m=1.0, offset_x_norm=-0.2)
    )

    assert command.valid is True
    assert command.action == "ALIGN_LEFT"
    assert command.action != "APPROACH_GOAL"


def test_goal_without_valid_depth_still_waits():
    planner = GoalNavigationPlanner()

    command = planner.plan(
        goal_info(depth_valid=False, depth_m=None)
    )

    assert command.valid is False
    assert command.action == "WAIT"
    assert command.reason == "missing_valid_goal_depth"


def test_centered_goal_at_50cm_approaches():
    planner = GoalNavigationPlanner()

    command = planner.plan(goal_info(depth_m=0.50))

    assert command.valid is True
    assert command.action == "APPROACH_GOAL"


def test_centered_goal_at_25cm_requests_score_motion():
    planner = GoalNavigationPlanner()

    command = planner.plan(goal_info(depth_m=0.25))

    assert command.action == "SHOT"
    assert command.sdk_motion_requested is True


def test_goal_waits_until_analyzer_confirms_score_condition():
    planner = GoalNavigationPlanner()

    command = planner.plan(goal_info(depth_m=0.25, score_now=False))

    assert command.action == "WAIT_SCORE_CONFIRMATION"
    assert command.sdk_motion_requested is False


def test_misaligned_goal_at_25cm_aligns_before_scoring():
    planner = GoalNavigationPlanner()

    command = planner.plan(
        goal_info(depth_m=0.25, offset_x_norm=-0.2)
    )

    assert command.action == "ALIGN_LEFT"
    assert command.sdk_motion_requested is False


def test_centered_goal_below_scoring_range_retreats_with_geometry():
    planner = GoalNavigationPlanner()
    depth = (
        planner.config.score_target_depth_m
        - planner.config.score_depth_tolerance_m
        - 0.001
    )

    command = planner.plan(
        goal_info(
            depth_m=depth,
            distance_m=0.18,
            offset_x_norm=0.0,
            score_now=False,
        )
    )

    assert command.valid is True
    assert command.action == "RETREAT_GOAL"
    assert command.action not in {
        "APPROACH_GOAL",
        "ALIGN_LEFT",
        "ALIGN_RIGHT",
        "SHOT",
    }
    assert command.depth_m == depth
    assert command.distance_m == 0.18
    assert command.depth_error_m == (
        depth - planner.config.score_target_depth_m
    )
    assert command.is_centered is True
    assert command.depth_in_score_range is False


def test_scoring_lower_boundary_is_inclusive_before_retreat():
    planner = GoalNavigationPlanner()
    boundary = (
        planner.config.score_target_depth_m
        - planner.config.score_depth_tolerance_m
    )

    at_boundary = planner.plan(
        goal_info(depth_m=boundary, score_now=False)
    )
    below_boundary = planner.plan(
        goal_info(depth_m=boundary - 0.001, score_now=False)
    )

    assert at_boundary.action == "WAIT_SCORE_CONFIRMATION"
    assert at_boundary.depth_in_score_range is True
    assert below_boundary.action == "RETREAT_GOAL"
    assert below_boundary.depth_in_score_range is False
