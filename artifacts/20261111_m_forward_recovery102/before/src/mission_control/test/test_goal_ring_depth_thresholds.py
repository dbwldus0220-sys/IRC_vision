"""Check board approach targeting separately from ring shot and fine bands."""

import math

import pytest

from mission_control.motion_decision_planner import MotionDecisionConfig, MotionDecisionPlanner
from step.goal_navigation_planner import GoalNavigationPlanner
from step.goal_scoring import goal_shot_region
from test_goal_ring_alignment import sample


@pytest.mark.parametrize('depth,action', [
    (.349999, 'GOAL_CAMERA90_BACKWARD_1'),
    (.350, 'WAIT_SCORE_CONFIRMATION'),
    (.599999, 'WAIT_SCORE_CONFIRMATION'),
    (.600, 'GOAL_CAMERA90_FINE_FORWARD_1'),
    (.612999, 'GOAL_CAMERA90_FINE_FORWARD_1'),
    (.613, 'GOAL_CAMERA90_FINE_FORWARD_2'),
    (.632, 'GOAL_CAMERA90_FINE_FORWARD_3'),
    (.670, 'GOAL_CAMERA90_FINE_FORWARD_4'),
    (.685, 'GOAL_CAMERA90_FINE_FORWARD_5'),
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


def test_remaining_regions_keep_depth_bounds_and_region_seven_is_removed():
    old_bands = [(450, 530, 1), (530, 590, 2), (590, 630, 8),
                 (630, 660, 5), (660, 700, 6)]
    for mm in range(300, 901):
        expected = next((region for low, high, region in old_bands if low <= mm < high), None)
        assert goal_shot_region((mm - 100) / 1000) == expected


def test_far_approach_steering_uses_measured_board_depth():
    command = GoalNavigationPlanner().plan(sample(
        .9, offset=100, score_now=False, lateral_offset_m=.1,
    ))
    assert command.depth_error_m == pytest.approx(.95 - .63)
    assert command.target_steering_angle_deg == pytest.approx(math.degrees(math.atan2(.1 + .63 * 40 / 900, .32)))


def test_far_approach_aims_at_board_630mm_and_minus_40px():
    command = GoalNavigationPlanner().plan(sample(
        .9, offset=-40, score_now=False, lateral_offset_m=-.63 * 40 / 900,
    ))
    assert command.depth_error_m == pytest.approx(.32)
    assert command.target_lateral_error_m == pytest.approx(0.)
    assert command.target_steering_angle_deg == pytest.approx(0.)
    assert command.action == 'GOAL_CAMERA_90_FORWARD_2'


@pytest.mark.parametrize('ring_depth', [.8, .9, 1.0])
@pytest.mark.parametrize('angle,action', [
    (0, 'GOAL_CAMERA_90_FORWARD_2'),
    (14.999, 'GOAL_CAMERA_90_FORWARD_2'),
    (15, 'GOAL_CAMERA90_RECOVER_RIGHT_4'),
    (-15, 'GOAL_CAMERA90_RECOVER_LEFT_4'),
    (50, 'GOAL_CAMERA90_RECOVER_RIGHT_4'),
    (-50, 'GOAL_CAMERA90_RECOVER_LEFT_4'),
    (50.001, 'GOAL_CAMERA90_TURN_RIGHT_5'),
    (-50.001, 'GOAL_CAMERA90_TURN_LEFT_3'),
])
def test_board_target_controls_far_steering_independently_of_ring(ring_depth, angle, action):
    board_depth = 1.0
    lateral = (board_depth - .63) * math.tan(math.radians(angle)) - .63 * 40 / 900
    info = sample(ring_depth, depth_m=board_depth, offset=lateral / board_depth * 900,
                  lateral_offset_m=lateral, score_now=False)
    standalone = GoalNavigationPlanner().plan(info)
    integrated = MotionDecisionPlanner()._plan_source('goal', info, .1)
    assert standalone.action == integrated['action'] == action
    assert standalone.depth_error_m == pytest.approx(.37)
    assert standalone.target_steering_angle_deg == pytest.approx(angle)
    assert integrated['target_forward_error_m'] == .37


def test_far_approach_bearing_fallback_uses_board_depth():
    info = sample(.9, depth_m=1.0, offset=100, score_now=False,
                  bearing_deg=math.degrees(math.atan(100 / 900)))
    info.pop('lateral_offset_m', None)
    command = GoalNavigationPlanner().plan(info)
    assert command.target_lateral_error_m == pytest.approx(1.0 * 100 / 900 + .63 * 40 / 900)


@pytest.mark.parametrize('board_depth', [None, True, float('nan'), float('inf'), 0, -1])
def test_far_approach_requires_valid_board_depth(board_depth):
    command = GoalNavigationPlanner().plan(sample(.9, depth_m=board_depth, score_now=False))
    assert command.action == 'WAIT'
    assert not command.valid


@pytest.mark.parametrize('offset,action', [
    (-40, 'GOAL_CAMERA90_FINE_FORWARD_2'),
    (500, 'GOAL_CAMERA90_TURN_RIGHT_3'),
    (-540, 'GOAL_CAMERA90_FINE_FORWARD_2'),
])
def test_fine_approach_turn_threshold_is_relative_to_minus_40px(offset, action):
    command = GoalNavigationPlanner().plan(sample(
        .63, offset=offset, score_now=True,
        bearing_deg=math.degrees(math.atan(offset / 900)),
    ))
    assert command.action == action
    assert command.shot_region is None
    assert not command.score_now


@pytest.mark.parametrize('board_limit', [1.0, 2.0])
def test_board_entry_limit_is_preserved_and_ring_limit_is_shifted(board_limit):
    planner = MotionDecisionPlanner(MotionDecisionConfig(goal_control_range_m=board_limit))
    assert planner.goal_planner.config.control_start_depth_m == pytest.approx(board_limit - .1)
    assert planner._goal_is_inside_control_range(sample(board_limit - .1, depth_m=board_limit))
    assert not planner._goal_is_inside_control_range(sample(board_limit - .1, depth_m=board_limit + .001))
