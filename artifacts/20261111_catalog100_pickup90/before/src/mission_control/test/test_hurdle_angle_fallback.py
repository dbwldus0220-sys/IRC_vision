"""Use calibrated hurdle yaw regardless of conflicting line observations."""

import pytest

from mission_control.motion_decision_planner import MotionDecisionPlanner
from step.hurdle_navigation_planner import HurdleNavigationPlanner
from test_hurdle_positioning import hurdle, line_info


def sample(angle=-15.2, depth=.56, **updates):
    return hurdle(depth, **{
        'hurdle_angle_45_deg': angle, 'depth_age_sec': .05, 'depth_sample_count': 5,
        'left_depth_m': .61, 'right_depth_m': .52, 'bottom_distance_px': 423,
        **updates,
    })


@pytest.mark.parametrize('recovery', [False, True])
@pytest.mark.parametrize('angle,direction,count', [
    (15., 'LEFT', 1), (29.999, 'LEFT', 1), (30., 'LEFT', 2),
    (45., 'LEFT', 3), (60., 'LEFT', 4), (75., 'LEFT', 5),
    (-15., 'RIGHT', 1), (-29.999, 'RIGHT', 1), (-30., 'RIGHT', 3),
    (-45., 'RIGHT', 5), (-60., 'RIGHT', 6), (-75., 'RIGHT', 7),
])
def test_calibrated_yaw_uses_stationary_turn_bins_with_correct_sign(recovery, angle, direction, count):
    planner = MotionDecisionPlanner()
    planner.hurdle_planner.recovery_active = recovery
    decision = planner.plan('HURDLE_POSITIONING', {'hurdle': sample(angle)}, .1)
    assert decision.valid and decision.action == f'ALIGN_{direction}'
    assert decision.source_command['turn_count'] == count
    assert decision.source_command['center_steering_deg'] == -angle
    assert decision.source_command['alignment_reference'] == 'hurdle_angle_45'
    assert decision.reason == ('hurdle_reacquired_hurdle_alignment' if recovery else 'hurdle_calibrated_angle_alignment')


@pytest.mark.parametrize('angle', [-14.999, -8., 0., 8., 14.999])
def test_alignment_tolerance_releases_recovery_into_forward(angle):
    planner = HurdleNavigationPlanner()
    planner.recovery_active = True
    decision = planner.plan(sample(angle), positioning=True)
    assert decision.valid and decision.action == 'STRAIGHT_1'
    assert not planner.recovery_active


@pytest.mark.parametrize('recovery', [False, True])
def test_calibrated_hurdle_yaw_wins_over_conflicting_line_angle(recovery):
    planner = MotionDecisionPlanner()
    planner.hurdle_planner.recovery_active = recovery
    decision = planner.plan('HURDLE_POSITIONING', {
        'hurdle': sample(-60), 'line': line_info(heading=-30),
    }, .1)
    assert decision.action == 'ALIGN_RIGHT'
    assert decision.source_command['turn_count'] == 6
    assert decision.source_command['center_steering_deg'] == 60
    assert decision.source_command['alignment_reference'] == 'hurdle_angle_45'


@pytest.mark.parametrize('updates', [
    {'hurdle_angle_45_deg': None}, {'hurdle_angle_45_deg': float('nan')},
    {'hurdle_angle_45_deg': 90}, {'depth_age_sec': .701}, {'depth_age_sec': -1},
    {'depth_age_sec': None}, {'hurdle_angle_fit_point_count': 1},
    {'confirmation_confirmed': False}, {'depth_valid': False},
])
def test_invalid_fallback_cannot_release_recovery(updates):
    planner = HurdleNavigationPlanner()
    planner.recovery_active = True
    decision = planner.plan(sample(**updates), positioning=True)
    assert not decision.valid and decision.action == 'WAIT'
    assert planner.recovery_active


@pytest.mark.parametrize('updates', [{'depth': .45}, {'bottom_distance_px': 100}])
def test_fallback_does_not_add_near_robot_recovery_turns(updates):
    planner = HurdleNavigationPlanner()
    planner.recovery_active = True
    decision = planner.plan(sample(**updates), positioning=True)
    assert not decision.valid and decision.reason == 'hurdle_turn_blocked_near_robot'


def test_lost_line_and_lost_hurdle_do_not_reuse_old_fallback_angle():
    planner = HurdleNavigationPlanner()
    assert planner.plan(sample(), positioning=True).action == 'ALIGN_RIGHT'
    decision = planner.plan({'detected': False, 'raw_detected': False}, positioning=True)
    assert not decision.valid
