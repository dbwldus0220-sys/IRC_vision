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


def test_goal_outside_50cm_control_range_waits():
    planner = GoalNavigationPlanner()

    command = planner.plan(goal_info(depth_m=0.51))

    assert command.valid is False
    assert command.action == "WAIT"
    assert command.reason == "goal_outside_control_range"


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
