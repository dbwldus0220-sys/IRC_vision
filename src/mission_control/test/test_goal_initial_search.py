"""Camera-up goal acquisition uses exactly two completed, fresh-image scans."""

import json

import pytest
from std_msgs.msg import String

from mission_control.goal_initial_search import GoalInitialSearch
from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from test_mission_phase_flow import line_info, approaching_goal
from test_wait_refresh_and_fine_settle import LiveInputHarness


@pytest.fixture
def clock(monkeypatch):
    now = [10.]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: now[0])
    monkeypatch.setattr(Node, '_current_ros_time_ns', lambda _: round(now[0] * 1e9))
    return now


def receive(node, clock, source, info, **updates):
    clock[0] += .02
    payload = {**info, 'rgb_stamp_ns': round(clock[0] * 1e9), **updates}
    Node._info_callback(node, source)(String(data=json.dumps(payload)))


def goal_frame(node, clock, info=None, **updates):
    receive(node, clock, 'goal', {'detected': False, 'raw_detected': False} if info is None else info, **updates)
    before = len(node.publisher.messages)
    Node._publish_decision(node)
    return node.publisher.messages[before:]


def camera_ready(clock, side='RIGHT', status='SUCCEEDED'):
    node = LiveInputHarness(phase='LINE_TRACK_AFTER_PICKUP')
    node.phase_manager.pickups_completed = 1
    node.phase_manager.ball_grasp_results[1] = 'GRABBED'
    if side is not None:
        receive(node, clock, 'line', {**line_info(), 'image_width': 1280,
                'robot_center_x_px': 710., 'center_points_px': [[900 if side == 'RIGHT' else 680, 600]],
                'lateral_offset_px': -300 if side == 'RIGHT' else 300})
    node.phase_manager.set_phase('POST_BALL_GOAL_TRANSITION')
    Node._publish_decision(node)
    camera = node.publisher.messages[-1]
    assert camera['action'] == 'POST_BALL_GOAL_TRANSITION'
    node.send_status(camera['action'], camera['command_id'], 'RUNNING')
    clock[0] += .8
    node.send_status(camera['action'], camera['command_id'], status)
    return node


def finish_turn(node, clock, command, status='SUCCEEDED'):
    node.send_status(command['action'], command['command_id'], 'RUNNING')
    # A new no-goal frame during execution must not queue the second turn.
    assert not any(m['valid'] for m in goal_frame(node, clock))
    node.send_status(command['action'], command['command_id'], status)
    if status == 'SUCCEEDED':
        deadline = node.goal_post_motion_dwell_until
        assert deadline == pytest.approx(clock[0] + 1.)
        assert not any(m['valid'] for m in goal_frame(node, clock))
        clock[0] = deadline
        Node._publish_decision(node)
        assert node.latest_info['goal'] is None


@pytest.mark.parametrize('side,actions', [
    ('RIGHT', ['GOAL_CAMERA90_TURN_RIGHT_5', 'GOAL_CAMERA90_TURN_LEFT_6']),
    ('LEFT', ['GOAL_CAMERA90_TURN_LEFT_6', 'GOAL_CAMERA90_TURN_RIGHT_9']),
    (None, ['GOAL_CAMERA90_TURN_RIGHT_5', 'GOAL_CAMERA90_TURN_LEFT_6']),
])
def test_exact_two_scans_then_wait_without_loop(clock, side, actions):
    node = camera_ready(clock, side)
    assert node.mission_phase == 'GOAL_APPROACH'
    for index, action in enumerate(actions):
        command = goal_frame(node, clock)[-1]
        assert command['valid'] and command['action'] == action
        assert command['source_command']['goal_initial_search_step'] == index + 1
        assert Bridge.motion_id_for_action(action) == action.lower().replace('camera90', 'camera_90')
        finish_turn(node, clock, command)
        assert node.goal_initial_search.step == index + 1
        node.send_status(action, command['command_id'], 'SUCCEEDED')
        assert node.goal_initial_search.step == index + 1
    for _ in range(4):
        command = goal_frame(node, clock)[-1]
        assert not command['valid'] and command['reason'] == 'goal_initial_search_exhausted'


@pytest.mark.parametrize('completed_scans', [0, 1, 2])
def test_goal_acquisition_ends_scan_and_preserves_normal_approach(clock, completed_scans):
    node = camera_ready(clock)
    for _ in range(completed_scans):
        command = goal_frame(node, clock)[-1]
        finish_turn(node, clock, command)
    commands = goal_frame(node, clock, approaching_goal())
    assert not node.goal_initial_search.active
    assert commands[-1]['valid']
    assert not commands[-1]['source_command'].get('goal_initial_search')


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'REJECTED', 'CANCELLED'])
def test_scan_failure_does_not_start_other_side(clock, status):
    node = camera_ready(clock)
    command = goal_frame(node, clock)[-1]
    finish_turn(node, clock, command, status)
    for _ in range(3):
        command = goal_frame(node, clock)[-1]
        assert not command['valid'] and command['reason'] == 'goal_initial_search_motion_failed'


@pytest.mark.parametrize('stamp', [None, True, 1, 99_000_000_000])
def test_missing_old_or_future_goal_capture_cannot_start_scan(clock, stamp):
    node = camera_ready(clock)
    for _ in range(3):
        assert not any(m['valid'] for m in goal_frame(node, clock, rgb_stamp_ns=stamp))
    assert node.goal_initial_search.step == 0


def test_raw_goal_candidate_prevents_scan_while_confirming(clock):
    node = camera_ready(clock)
    command = goal_frame(node, clock, {'detected': False, 'raw_detected': True})[-1]
    assert not command['valid']
    assert command['reason'] == 'goal_initial_search_waiting_for_confirmation'
    assert node.goal_initial_search.step == 0


def test_camera_failure_never_arms_search(clock):
    node = camera_ready(clock, status='FAILED')
    commands = goal_frame(node, clock)
    assert not any(m['valid'] for m in commands)
    assert not node.goal_initial_search.active


def test_phase_override_cancels_search(clock):
    node = camera_ready(clock)
    Node._phase_callback(node, String(data='LINE_TRACK'))
    assert not node.goal_initial_search.active


def test_line_side_ignores_stale_and_low_quality_observations(clock):
    node = camera_ready(clock, 'LEFT')
    assert node.goal_initial_search.last_line_side == 'LEFT'
    node.goal_initial_search.cancel()
    node.phase_manager.set_phase('LINE_TRACK_AFTER_PICKUP')
    right = {**line_info(), 'lateral_offset_px': 300}
    receive(node, clock, 'line', right, rgb_stamp_ns=1)
    assert node.goal_initial_search.last_line_side == 'LEFT'
    receive(node, clock, 'line', {**right, 'heading_quality': .1})
    assert node.goal_initial_search.last_line_side == 'LEFT'


def test_new_camera_transition_rearms_two_scans():
    search = GoalInitialSearch()
    search.last_line_side = 'LEFT'
    search.arm()
    search.step = 2
    search.failed = True
    search.arm()
    assert search.step == 0 and not search.failed
    assert search.sequence == (('LEFT', 6), ('RIGHT', 9))
