"""Validate asymmetric far scoring zones on real GoalAnalyzer messages."""

import pytest

from step.goal_navigation_planner import GoalNavigationPlanner
from test_goal_backboard_selection import make_analyzer, send, detection


@pytest.mark.parametrize('depth,offset,region', [
    (.590, -80, 8), (.629, 0, 8), (.630, -80, 5), (.659, 0, 5),
    (.660, -80, 6), (.699, 0, 6), (.700, -70, 7), (.759, 0, 7),
])
def test_analyzer_confirms_new_region_and_publishes_calibration(depth, offset, region):
    analyzer, published = make_analyzer(depth_m=depth)
    for index in range(3):
        info = send(analyzer, published, [detection(center_x=640 + offset)])
        assert info['score_now'] is (index == 2)
        assert info['is_centered'] is True
        assert info['camera_fx_px'] == analyzer.fx
        command = GoalNavigationPlanner().plan(info)
        assert command.shot_region == region
        assert command.action == ('SHOT' if index == 2 else 'WAIT_SCORE_CONFIRMATION')


@pytest.mark.parametrize('depth,offset', [
    (.610, 1), (.610, -81), (.645, 1), (.680, -81),
    (.730, -71), (.730, 1), (.760, -40),
])
def test_analyzer_never_confirms_outside_new_region(depth, offset):
    analyzer, published = make_analyzer(depth_m=depth)
    for _ in range(5):
        info = send(analyzer, published, [detection(center_x=640 + offset)])
        assert not info['score_now']
        assert GoalNavigationPlanner().plan(info).action != 'SHOT'
