"""The relaxed recovery deadband must hold across defaults and repeated decisions."""

import pytest

from step.line_navigation_planner import LineNavigationPlanner, NavigationConfig
from mission_control.motion_decision_planner import MotionDecisionConfig, MotionDecisionPlanner
from test_mission_phase_flow import line_info


@pytest.mark.parametrize('heading', [-19.999, -15., -10., -3., 0., 3., 10., 15., 19.999])
@pytest.mark.parametrize('offset', [-.7, -.3, 0., .3, .7])
@pytest.mark.parametrize('previous', ['STRAIGHT', 'RECOVER_LEFT_TURN_LEFT_4'])
def test_straight_line_below_twenty_cannot_bypass_recovery_deadband(heading, offset, previous):
    planner = LineNavigationPlanner()
    planner.previous_motion = previous
    for _ in range(5):
        command = planner.plan(line_info(heading, offset), .1)
        assert command.valid and command.motion == 'STRAIGHT'


@pytest.mark.parametrize('heading', [-20.001, -20., 20., 20.001])
@pytest.mark.parametrize('offset', [0., .1, .3, .7])
def test_twenty_is_inclusive_for_centered_and_away_heading(heading, offset):
    side = 'RIGHT' if heading > 0 else 'LEFT'
    offset = offset if heading > 0 else -offset
    planner = MotionDecisionPlanner()
    decision = planner.plan('LINE_TRACK', {'line': line_info(heading, offset)}, .1)
    assert decision.action == f'RECOVER_{side}_TURN_{side}_4'


def test_planner_defaults_both_use_twenty():
    for config in [NavigationConfig(), MotionDecisionConfig(), MotionDecisionPlanner().line_planner.config]:
        assert config.recovery_heading_turn_deg == 20.
        assert config.recovery_away_heading_turn_deg == 20.


@pytest.mark.parametrize('robot', [False, True])
def test_launch_passes_both_twenty_degree_defaults(monkeypatch, tmp_path, robot):
    from launch_ros.actions import Node
    from test_full_system_launch import node_parameters
    if robot:
        from test_full_system_robot_launch import launch_description, default_context
    else:
        from test_full_system_launch import launch_description, launch_context as default_context
    description = launch_description(monkeypatch, tmp_path)
    context = default_context(description)
    node = next(e for e in description.entities if isinstance(e, Node)
                and e.node_executable == 'motion_decision_node')
    parameters = node_parameters(node, context)
    assert parameters['recovery_heading_turn_deg'] == 20.
    assert parameters['recovery_away_heading_turn_deg'] == 20.
