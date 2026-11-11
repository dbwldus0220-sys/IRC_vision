"""Count walking gaits across yaw corrections and reset each pickup's search."""

import json

import pytest
from std_msgs.msg import String

from mission_control.lost_search_turn_limiter import LostSearchTurnLimiter
from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from test_continuous_approach import clock
from test_motion_command_bridge import decoded_messages
from test_pickup_continuous_walk import forward_status
from test_three_line_goal_handoff import start, reserve_line, finish_rest
from test_wait_refresh_and_fine_settle import LiveInputHarness


@pytest.mark.parametrize('action', [
    'STRAIGHT', 'STRAIGHT_1', 'STRAIGHT_2', 'STRAIGHT_3', 'STRAIGHT_4',
    'LEFT', 'RIGHT', 'LINE_SPARSE_FORWARD',
    'RECOVER_LEFT_TURN_LEFT_4', 'RECOVER_RIGHT_TURN_RIGHT_4',
    'RECOVER_LEFT_TURN_RIGHT_4', 'RECOVER_RIGHT_TURN_LEFT_4',
])
def test_all_counted_commands_resolve_to_actual_forward_or_recovery(action):
    assert Node._post_pickup_counted_line_action(action)
    assert Node._post_pickup_line_motion_started({'motion_id': Bridge.motion_id_for_action(action)})


@pytest.mark.parametrize('action', [
    'LINE_HEADING_TURN_LEFT_4', 'LINE_OFFSET_TURN_RIGHT_1',
    'LINE_LOST_TURN_LEFT_1', 'LINE_SPARSE_TURN_RIGHT_1',
    'POST_BALL_LINE_TURN_LEFT_3', 'WAIT', 'STOP', 'PICKUP_NOW',
])
def test_stationary_or_non_line_commands_are_not_counted(action):
    assert not Node._post_pickup_counted_line_action(action)


@pytest.mark.parametrize('index', [1, 2])
def test_corrections_between_walks_do_not_raise_camera_or_finish_line_stage(clock, index):
    node, bridge, current = start(clock, index)
    # Reproduce the short yaw command that formerly consumed the third slot.
    for heading, expected_count, counted in [(60., 1, False), (0., 2, True), (-60., 2, False), (0., 3, True)]:
        command = reserve_line(node, clock, heading)
        assert command['command_id'] != current['command_id']
        metadata = command['source_command']
        assert ('post_pickup_line_index' in metadata) is counted
        assert ('goal_head_prepare_id' in metadata) is (expected_count == 3)
        bridge.navigation_command_callback(String(data=json.dumps(command)))
        forward_status(node, bridge, current, 'SUCCEEDED')
        finish_rest(bridge, clock)
        current = decoded_messages(bridge.executor_request_publisher)[-1]
        assert current['request_id'] == command['command_id']
        forward_status(node, bridge, current, 'RUNNING')
        forward_status(node, bridge, current, 'RUNNING')
        assert node.post_ball_line_started_count == expected_count
        assert node.mission_phase == 'LINE_TRACK_AFTER_PICKUP'
        if expected_count < 3:
            assert not node.camera_requests
            assert getattr(node, 'post_ball_goal_camera_request_id', None) is None
    assert len(node.post_ball_line_command_ids) == 3
    assert current['goal_head_prepare_id'] == command['command_id']
    assert current['motion_id'].startswith('line_forward_')
    forward_status(node, bridge, current, 'SUCCEEDED')
    assert node.mission_phase == 'GOAL_APPROACH'


@pytest.mark.parametrize('index', [1, 2])
@pytest.mark.parametrize('status', ['SUCCEEDED', 'FAILED', 'CANCELLED'])
def test_only_successful_pickup_resets_line_search_budget(clock, index, status):
    node = LiveInputHarness(phase='BALL_APPROACH')
    node.phase_manager.pickups_completed = index - 1
    node.phase_manager.pickups_executed = index - 1
    node.phase_manager.ball_sections_processed = index - 1
    node.phase_manager.shots_completed = index - 1
    node.phase_manager.shots_executed = index - 1
    limiter = node.lost_search_turn_limiter = LostSearchTurnLimiter(staged_sources=('line', 'ball', 'goal'))
    limiter.counts.update(line=3, ball=2, goal=1)
    limiter.angles['line'] = 45.
    limiter.corner_directions['line'] = 'LEFT'
    limiter.failed_sources.add('line')
    assert node.phase_manager.start_special_action('PICKUP_NOW', 10)
    node.send_status('PICKUP_NOW', 10, 'RUNNING', motion_id='pickup_400')
    assert node.phase_manager.record_active_pickup_grasp_result('GRABBED')
    node.send_status('PICKUP_NOW', 10, status, motion_id=(
        'pickup_first_backward_turn_right' if index == 1 else 'pickup_second_backward_turn_left'))
    assert limiter.counts['ball'] == 2 and limiter.counts['goal'] == 1
    if status != 'SUCCEEDED':
        assert limiter.counts['line'] == 3
        return
    assert limiter.counts['line'] == limiter.angles['line'] == 0
    assert 'line' not in limiter.failed_sources and 'line' not in limiter.corner_directions
    lost = {'line': {'detected': False, 'raw_detected': False}}
    expected = [('RIGHT', 5)] + [('RIGHT', 1)] * 3 + [('LEFT', 1)] * 6 if index == 1 else (
        [('LEFT', 3)] + [('LEFT', 1)] * 3 + [('RIGHT', 1)] * 6)
    for direction, count in expected:
        decision = limiter.filter(node.planner._plan_post_ball_line_align(lost['line']), lost)
        assert decision.valid and decision.action == f'POST_BALL_LINE_TURN_{direction}_{count}'
        limiter.record_published(decision, lost)
    decision = limiter.filter(node.planner._plan_post_ball_line_align(lost['line']), lost)
    assert not decision.valid and decision.reason == 'lost_search_turn_limit_reached'
