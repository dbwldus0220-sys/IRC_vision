"""Exercise both goal exits, stationary alignment and post-motion observations."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.motion_decision_planner import MotionDecisionPlanner
from test_continuous_approach import clock
from test_mission_phase_flow import line_info
from test_motion_command_bridge import FakeBridge, decoded_messages
from test_pickup_continuous_walk import forward_status
from test_post_ball_goal_handoff import receive
from test_post_shot_forward_prequeue import start_turn, turn_frames
from test_wait_refresh_and_fine_settle import LiveInputHarness


@pytest.mark.parametrize('heading,expected', [
    (-90., 'TURN_LEFT_6'), (-45., 'TURN_LEFT_3'), (-35.001, 'TURN_LEFT_2'),
    (-35., 'TURN_LEFT_2'), (-31., 'TURN_LEFT_2'),
    (-15., 'TURN_LEFT_1'), (-14.999, 'ALIGNED'),
    (0., 'ALIGNED'), (14.999, 'ALIGNED'),
    (15., 'TURN_RIGHT_1'), (31., 'TURN_RIGHT_3'),
    (35., 'TURN_RIGHT_3'), (35.001, 'TURN_RIGHT_3'), (45., 'TURN_RIGHT_5'),
    (90., 'TURN_RIGHT_9'),
])
@pytest.mark.parametrize('offset', [-1.29, 0., 1.29])
def test_heading_boundaries_preserve_post_shot_offset_policy(heading, expected, offset):
    result = MotionDecisionPlanner().plan(
        'POST_SHOT_LINE_ALIGN', {'line': line_info(heading, offset)}, .1,
    )
    assert result.valid
    assert result.action == 'POST_SHOT_LINE_' + expected
    if expected.startswith('TURN_'):
        assert result.source_command['turn_count'] == int(expected.rsplit('_', 1)[1])


@pytest.mark.parametrize('changes', [
    {'ground_heading_error_deg': None}, {'geometry_quality': .01},
    {'ground_heading_error_deg': float('nan')},
])
def test_invalid_geometry_does_not_start_turn(changes):
    result = MotionDecisionPlanner().plan(
        'POST_SHOT_LINE_ALIGN', {'line': {**line_info(31.), **changes}}, .1,
    )
    assert result.action == 'WAIT'
    assert not result.valid


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('region', [3, 4])
@pytest.mark.parametrize('heading', [-31., 31.])
def test_both_fixed_exits_can_enter_either_stationary_turn(clock, section, region, heading):
    node = LiveInputHarness(phase='POST_SHOT_TURN')
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = Node.LINE_TURN_PRE_MOTION_SETTLE_SEC
    node.phase_manager.ball_sections_processed = section
    node.phase_manager.last_completed_shot_region = region
    bridge = FakeBridge(advance_transition_dwells=False)
    receive(node, clock, 'line', line_info())
    Node._publish_decision(node)
    exit_command = node.publisher.messages[-1]
    assert exit_command['action'] == (
        'POST_SHOT_TURN_RIGHT_9' if section == 1 else 'POST_SHOT_TURN_LEFT_4'
    )
    bridge.navigation_command_callback(String(data=json.dumps(exit_command)))
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, request, 'RUNNING')
    clock[0] += 3.
    forward_status(node, bridge, request, 'SUCCEEDED')
    assert node.mission_phase == 'POST_SHOT_LINE_ALIGN'
    assert node.latest_info['line'] is None
    clock[0] = node.post_shot_dwell_until
    Node._publish_decision(node)
    clock[0] += .01
    receive(node, clock, 'line', line_info(heading, -1.29))
    Node._publish_decision(node)
    command = node.publisher.messages[-1]
    direction = 'LEFT' if heading < 0 else 'RIGHT'
    count = 2 if heading < 0 else 3
    assert command['action'] == f'POST_SHOT_LINE_TURN_{direction}_{count}'
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request['motion_id'] == f'post_ball_line_turn_{direction.lower()}_{count}'
    forward_status(node, bridge, request, 'RUNNING')
    assert node.active_line_motion_started_at == clock[0]


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('next_heading,expected', [
    (31., 'TURN_RIGHT_3'), (-31., 'TURN_LEFT_2'),
    (60., 'TURN_RIGHT_6'), (-60., 'TURN_LEFT_4'),
    (0., 'ALIGNED'),
])
def test_turn_without_forward_reservation_reobserves_and_continues(
    clock, section, next_heading, expected,
):
    node, bridge, request = start_turn(clock, section, 31.)
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = Node.LINE_TURN_PRE_MOTION_SETTLE_SEC
    turn_frames(node, clock, line_info(31.))
    assert node.queued_general_command_id is None
    forward_status(node, bridge, request, 'SUCCEEDED')
    assert node.mission_phase == 'POST_SHOT_LINE_ALIGN'
    assert not node.general_motion_gate.locked
    assert node.latest_info['line'] is None
    count = len(node.publisher.messages)
    clock[0] += .49
    receive(node, clock, 'line', line_info(next_heading))
    Node._publish_decision(node)
    assert len(node.publisher.messages) == count
    clock[0] = node.post_shot_dwell_until
    Node._publish_decision(node)
    assert node.latest_info['line'] is None
    clock[0] += .01
    receive(node, clock, 'line', line_info(next_heading))
    Node._publish_decision(node)
    command = node.publisher.messages[-1]
    assert command['action'] == ('POST_SHOT_FORWARD' if expected == 'ALIGNED'
                                 else 'POST_SHOT_LINE_' + expected)
    assert command['command_id'] != request['command_id']
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    next_request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert next_request['command_id'] == command['command_id']


@pytest.mark.parametrize('section', [1, 2])
def test_missing_line_after_turn_searches_in_the_course_exit_direction(clock, section):
    node, bridge, request = start_turn(clock, section, 31.)
    node.planner.post_ball_line_search_direction = 'RIGHT' if section == 1 else 'LEFT'
    forward_status(node, bridge, request, 'SUCCEEDED')
    clock[0] = node.post_shot_dwell_until
    Node._publish_decision(node)
    clock[0] += .01
    receive(node, clock, 'line', {'detected': False, 'raw_detected': False})
    Node._publish_decision(node)
    action = node.publisher.messages[-1]['action']
    assert action == ('POST_SHOT_LINE_TURN_RIGHT_5' if section == 1
                      else 'POST_SHOT_LINE_TURN_LEFT_3')


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'REJECTED', 'CANCELLED', 'UNSUPPORTED'])
def test_failed_turn_holds_without_replay(clock, status):
    node, _, request = start_turn(clock, 1, 31.)
    node.send_status(request['action'], request['command_id'], status)
    assert node.phase_manager.post_shot_failed
    clock[0] += 1.
    receive(node, clock, 'line', line_info(31.))
    Node._publish_decision(node)
    assert node.publisher.messages[-1]['action'] == 'WAIT'
    assert node.publisher.messages[-1]['reason'] == 'post_shot_motion_failed'


@pytest.mark.parametrize('section', [1, 2])
def test_lost_frame_clears_forward_evidence_before_reacquisition(clock, section):
    node, _, _ = start_turn(clock, section, 31.)
    clock[0] += node.active_line_motion_duration_sec * .70
    for _ in range(2):
        receive(node, clock, 'line', line_info())
        clock[0] += .05
    receive(node, clock, 'line', {'detected': False, 'raw_detected': False})
    assert node.active_line_motion_frames == []
    assert node.queued_general_command_id is None
    for _ in range(2):
        clock[0] += .05
        receive(node, clock, 'line', line_info())
    assert node.queued_general_command_id is None
    clock[0] += .05
    receive(node, clock, 'line', line_info())
    assert node.publisher.messages[-1]['action'] == 'POST_SHOT_FORWARD'
    assert node.queued_general_command_id is not None
