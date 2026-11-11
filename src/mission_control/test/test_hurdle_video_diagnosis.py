"""Reproduce the shown heading decisions and loss after hurdle completion."""

import pytest

from mission_control.motion_decision_planner import MotionDecisionPlanner
from test_hurdle_positioning import HurdleHarness, clock, enter_positioning, hurdle, line_info


@pytest.mark.parametrize('depth,heading,action,count', [
    (.86, -27.3, 'STRAIGHT_1', None),
    (.78, -14.4, 'STRAIGHT_1', None),
])
def test_old_screenshot_line_heading_no_longer_controls_hurdle_yaw(depth, heading, action, count):
    decision = MotionDecisionPlanner().plan('LINE_TRACK', {
        'hurdle': hurdle(depth), 'line': line_info(heading=heading),
    }, .1)
    assert decision.source == 'hurdle'
    assert decision.action == action
    assert decision.source_command.get('turn_count') == count
    assert decision.source_command['center_steering_deg'] == 0


def test_completed_hurdle_clears_search_side_but_new_line_can_restore_it(clock):
    node = HurdleHarness()
    go = enter_positioning(node, clock)
    node.send_status('GO', go['command_id'], 'RUNNING', motion_id='hurdle')
    node.planner.last_line_seen_direction = 'RIGHT'
    node.send_status('GO', go['command_id'], 'SUCCEEDED', motion_id='hurdle')
    assert node.mission_phase == 'LINE_TRACK'
    assert node.latest_info['line'] is None
    assert node.planner.last_line_seen_direction is None
    missing = {'detected': False}
    waiting = node.planner.plan('LINE_TRACK', {'line': missing}, .1)
    assert not waiting.valid
    assert waiting.reason == 'line_not_detected'
    node.planner.observe_line_for_search({**line_info(x=900), 'image_width': 1280})
    assert node.planner.last_line_seen_direction == 'RIGHT'
    search = node.planner.plan('LINE_TRACK', {'line': missing}, .1)
    assert search.valid and search.action == 'LINE_LOST_TURN_RIGHT'
