"""Verify ring measurements, score confirmation and depth-only region choice."""

import pytest
from step.goal_navigation_planner import GoalNavigationPlanner
from test_goal_backboard_selection import make_analyzer, detection, send


@pytest.mark.parametrize('ring_depth,region', [(.45, 1), (.53, 2), (.59, 8), (.63, 5), (.66, 6), (.70, 7)])
@pytest.mark.parametrize('offset', [-350, -121, -120, 0, 120, 121, 350])
def test_analyzer_confirms_ring_distance_instead_of_board_distance(ring_depth, region, offset):
    analyzer, published = make_analyzer(depth_m=ring_depth + .10)
    for frame in range(3):
        info = send(analyzer, published, [detection(center_x=640 + offset)])
        assert info['depth_m'] == pytest.approx(ring_depth + .10)
        assert info['ring_center_depth_m'] == pytest.approx(ring_depth)
        assert info['score_now'] is (frame == 2)
        command = GoalNavigationPlanner().plan(info)
        assert command.shot_region == region
        assert command.waist_ring_aim_deg == info['waist_ring_aim_deg']
        assert command.action == ('SHOT' if frame == 2 else 'WAIT_SCORE_CONFIRMATION')


@pytest.mark.parametrize('offset', [-350, -121, 121, 350])
def test_outside_pixel_window_confirms_shot_without_body_alignment(offset):
    analyzer, published = make_analyzer(depth_m=.76)
    for frame in range(5):
        info = send(analyzer, published, [detection(center_x=640 + offset)])
        assert not info['is_centered']
        assert info['score_now'] is (frame >= 2)
        command = GoalNavigationPlanner().plan(info)
        assert command.shot_region == 6
        assert command.action == ('SHOT' if frame >= 2 else 'WAIT_SCORE_CONFIRMATION')


def test_missing_plane_depth_does_not_fall_back_to_board_for_shot():
    analyzer, published = make_analyzer(depth_m=.66)
    analyzer._sample_backboard_line_depths = lambda *_: (None, .66, .66, 2, [[560, 222], [640, 222], [720, 222]])
    for _ in range(5):
        info = send(analyzer, published, [detection()])
        assert info['depth_valid']
        assert info['ring_center_depth_m'] is None
        assert not info['score_now']
        assert GoalNavigationPlanner().plan(info).action == 'WAIT'
