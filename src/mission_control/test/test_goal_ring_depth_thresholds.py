"""Check ring approach boundaries independently of the measured board depth."""

import math

import pytest

from mission_control.motion_decision_planner import MotionDecisionConfig, MotionDecisionPlanner
from step.goal_navigation_planner import GoalNavigationPlanner
from step.goal_scoring import goal_shot_region
from test_goal_ring_alignment import sample


@pytest.mark.parametrize('depth,action', [
    (.349999, 'GOAL_CAMERA90_BACKWARD_1'),
    (.350, 'WAIT_SCORE_CONFIRMATION'),
    (.659999, 'WAIT_SCORE_CONFIRMATION'),
    (.660, 'GOAL_CAMERA90_FINE_FORWARD_1'),
    (.672999, 'GOAL_CAMERA90_FINE_FORWARD_1'),
    (.673, 'GOAL_CAMERA90_FINE_FORWARD_2'),
    (.692, 'GOAL_CAMERA90_FINE_FORWARD_3'),
    (.730, 'GOAL_CAMERA90_FINE_FORWARD_4'),
    (.745, 'GOAL_CAMERA90_FINE_FORWARD_5'),
    (.750, 'GOAL_CAMERA90_FINE_FORWARD_5'),
    (.750001, 'GOAL_CAMERA_90_FORWARD_2'),
    (1.9, 'GOAL_CAMERA_90_FORWARD_2'),
    (1.900001, 'WAIT'),
])
def test_shifted_ring_depth_controls_approach(depth, action):
    info = sample(depth, score_now=False)
    standalone = GoalNavigationPlanner().plan(info)
    integrated = MotionDecisionPlanner()._plan_source('goal', info, .1)
    assert standalone.action == integrated['action'] == action
    assert not standalone.score_now


def test_every_original_region_is_translated_by_exactly_100mm():
    old_bands = [(450, 530, 1), (530, 590, 2), (590, 630, 8),
                 (630, 660, 5), (660, 700, 6), (700, 760, 7)]
    for mm in range(300, 901):
        expected = next((region for low, high, region in old_bands if low <= mm < high), None)
        assert goal_shot_region((mm - 100) / 1000) == expected


def test_far_approach_steering_uses_shifted_target():
    command = GoalNavigationPlanner().plan(sample(
        .9, offset=100, score_now=False, lateral_offset_m=.1,
    ))
    assert command.depth_error_m == pytest.approx(.9 - .56)
    assert command.target_steering_angle_deg == pytest.approx(math.degrees(math.atan2(.1, .34)))


@pytest.mark.parametrize('board_limit', [1.0, 2.0])
def test_board_entry_limit_is_preserved_and_ring_limit_is_shifted(board_limit):
    planner = MotionDecisionPlanner(MotionDecisionConfig(goal_control_range_m=board_limit))
    assert planner.goal_planner.config.control_start_depth_m == pytest.approx(board_limit - .1)
    assert planner._goal_is_inside_control_range(sample(board_limit - .1, depth_m=board_limit))
    assert not planner._goal_is_inside_control_range(sample(board_limit - .1, depth_m=board_limit + .001))
