"""Reacquire a normal ground fit with fresh, individually completed small turns."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_decision_planner import MotionDecisionConfig, MotionDecisionPlanner
from test_mission_phase_flow import line_info
from test_sparse_line_recovery import sparse_info
from test_wait_refresh_and_fine_settle import LiveInputHarness


def unconfirmed_sparse_info(**kwargs):
    """Isolate ordinary ground recovery; confirmed corners own their recovery."""
    return {**sparse_info(**kwargs), 'corner_preview_confirmed': False}


@pytest.fixture
def clock(monkeypatch):
    now = [10.0]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: now[0])
    monkeypatch.setattr(MotionDecisionNode, '_current_ros_time_ns', lambda self: int(now[0] * 1e9))
    return now


def node_for(phase='LINE_TRACK'):
    node = LiveInputHarness(phase=phase)
    # Exercise recovery outside the entry radius; near corners have priority.
    node.line_corner_turn_distance_m = 0.15
    if phase == 'POST_BALL_LINE_ALIGN':
        node.phase_manager.pickups_completed = 1
        node.phase_manager.ball_grasp_results[1] = 'GRABBED'
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = MotionDecisionNode.LINE_TURN_PRE_MOTION_SETTLE_SEC
    return node


def frame(node, clock, info=None, **updates):
    payload = {**(unconfirmed_sparse_info() if info is None else info),
               'rgb_stamp_ns': int(clock[0] * 1e9), **updates}
    before = len(node.publisher.messages)
    MotionDecisionNode._info_callback(node, 'line')(String(data=json.dumps(payload)))
    MotionDecisionNode._publish_decision(node)
    return node.publisher.messages[before:]


def start_turn(node, clock, info):
    start = clock[0]
    for elapsed in (0.0, 0.4, 0.8, 1.01):
        clock[0] = start + elapsed
        commands = frame(node, clock, info)
        if elapsed < 1.0:
            assert not any(command['valid'] for command in commands)
    command = commands[-1]
    assert command['valid'] and command['source_command']['invalid_ground_recovery']
    return command


def finish_turn(node, clock, command):
    node.send_status(command['action'], command['command_id'], 'RUNNING')
    clock[0] += 0.6
    node.send_status(command['action'], command['command_id'], 'SUCCEEDED')
    assert node.latest_info['line'] is None
    deadline = (getattr(node, 'post_ball_line_dwell_until', None)
                or node.correction_post_motion_dwell_until)
    assert deadline == clock[0] + 1.0
    clock[0] = deadline - 0.01
    assert frame(node, clock) == []
    clock[0] = deadline
    MotionDecisionNode._publish_decision(node)
    assert node.latest_info['line'] is None
    assert node.line_offset_min_rgb_stamp_ns == int(deadline * 1e9)
    clock[0] += 0.01


@pytest.mark.parametrize('phase', ['AUTO', 'LINE_TRACK', 'LINE_TRACK_AFTER_PICKUP'])
@pytest.mark.parametrize('side,count', [('RIGHT', 2), ('LEFT', 1)])
def test_visible_invalid_ground_turns_toward_last_seen_line(clock, phase, side, count):
    node = node_for(phase)
    sign = 1 if side == 'RIGHT' else -1
    info = {**unconfirmed_sparse_info(), 'filtered_lateral_offset_norm': sign * 0.3}
    command = start_turn(node, clock, info)
    assert command['action'] == f'LINE_HEADING_TURN_{side}_{count}'
    assert command['source_command']['direction_source'] == 'last_seen_line'
    assert command['source_command']['turn_angle_deg'] == 15.0
    assert MotionCommandBridgeNode.motion_id_for_action(command['action']) == f'post_ball_line_turn_{side.lower()}_{count}'
    assert node.latest_info['line']['ground_projection_valid'] is False
    assert not node.sparse_line_recovery.eligible


@pytest.mark.parametrize('section,side,count', [(1, 'RIGHT', 2), (2, 'LEFT', 1)])
def test_pickup_exit_direction_overrides_opposite_visible_line(clock, section, side, count):
    node = node_for('BALL_APPROACH')
    node.phase_manager.pickups_completed = section - 1
    assert node.phase_manager.start_special_action('PICKUP_NOW', 50)
    node.send_status('PICKUP_NOW', 50, 'RUNNING')
    node.send_status('PICKUP_NOW', 50, 'SUCCEEDED')
    assert node.mission_phase == 'POST_BALL_LINE_ALIGN'
    assert node.planner.post_ball_line_search_direction == side
    # Deliberately disagree with the pickup exit direction.
    info = {**unconfirmed_sparse_info(), 'filtered_lateral_offset_norm': -0.4 if side == 'RIGHT' else 0.4}
    command = start_turn(node, clock, info)
    assert command['action'] == f'POST_BALL_LINE_TURN_{side}_{count}'
    assert command['source_command']['direction_source'] == 'pickup_exit_turn'
    finish_turn(node, clock, command)
    repeated = start_turn(node, clock, info)
    assert repeated['action'] == command['action']
    assert node.mission_phase == 'POST_BALL_LINE_ALIGN'


@pytest.mark.parametrize('phase', ['LINE_TRACK', 'POST_BALL_LINE_ALIGN'])
def test_recovery_repeats_beyond_two_turns_until_normal_three_point_fit(clock, phase):
    node = node_for(phase)
    for _ in range(3):
        command = start_turn(node, clock, unconfirmed_sparse_info())
        # Replanning during execution cannot enqueue another turn.
        clock[0] += 0.1
        assert frame(node, clock) == []
        finish_turn(node, clock, command)
    normal = {**line_info(), 'ground_projection_enabled': True,
              'ground_fit_point_count': 3, 'ground_fit_reason': 'ok'}
    commands = frame(node, clock, normal)
    assert commands[-1]['action'].startswith('STRAIGHT')
    assert not commands[-1]['source_command'].get('invalid_ground_recovery', False)
    if phase == 'POST_BALL_LINE_ALIGN':
        assert node.mission_phase == 'LINE_TRACK_AFTER_PICKUP'


def test_normal_fit_cancels_turn_during_pre_motion_wait(clock):
    node = node_for()
    assert frame(node, clock) == []
    clock[0] += 0.4
    normal = {**line_info(), 'ground_projection_enabled': True, 'ground_fit_point_count': 3}
    command = frame(node, clock, normal)[-1]
    assert command['action'].startswith('STRAIGHT')
    assert not any(m['source_command'].get('invalid_ground_recovery') for m in node.publisher.messages)


@pytest.mark.parametrize('stamp', [None, True, 8_000_000_000, 12_000_000_000])
def test_missing_stale_or_future_capture_never_authorizes_recovery(clock, stamp):
    node = node_for()
    for _ in range(8):
        commands = frame(node, clock, rgb_stamp_ns=stamp)
        assert not any(c['valid'] for c in commands)
        clock[0] += 0.2


def test_expired_received_input_cancels_pending_recovery(clock):
    node = node_for()
    frame(node, clock)
    clock[0] += 1.1
    MotionDecisionNode._publish_decision(node)
    assert not any(c['valid'] for c in node.publisher.messages)
    assert node.pre_motion_settle_started_at is None


@pytest.mark.parametrize('phase', ['LINE_TRACK', 'POST_BALL_LINE_ALIGN'])
def test_even_valid_ground_captured_during_dwell_cannot_resume_motion(clock, phase):
    node = node_for(phase)
    command = start_turn(node, clock, unconfirmed_sparse_info())
    finish_turn(node, clock, command)
    normal = {**line_info(), 'ground_projection_enabled': True, 'ground_fit_point_count': 3}
    commands = frame(node, clock, normal, rgb_stamp_ns=node.line_offset_min_rgb_stamp_ns - 1)
    assert node.latest_info['line'] is None
    assert not any(c['valid'] for c in commands)
    assert frame(node, clock, normal)[-1]['action'].startswith('STRAIGHT')


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'REJECTED'])
def test_recovery_failure_cannot_be_bypassed_by_opposite_line_side(clock, status):
    node = node_for()
    command = start_turn(node, clock, unconfirmed_sparse_info())
    if status != 'REJECTED':
        node.send_status(command['action'], command['command_id'], 'RUNNING')
    node.send_status(command['action'], command['command_id'], status)
    assert node.line_ground_recovery_failed
    clock[0] += 0.1
    commands = frame(node, clock, filtered_lateral_offset_norm=-0.4)
    assert not any(c['valid'] for c in commands)
    assert not any(c['action'] == 'LINE_HEADING_TURN_LEFT_1' for c in commands)


@pytest.mark.parametrize('updates', [
    {'heading_quality': 0.1}, {'geometry_quality': None}, {'detection_quality': float('nan')},
    {'ground_projection_enabled': False}, {'ground_fit_reason': 'invalid_parameters'},
    {'ground_fit_reason': 'invalid_projection_or_fit'}, {'ground_fit_reason': 'nonfinite_fit'},
    {'filtered_lateral_offset_norm': None}, {'detected': 'true'},
])
def test_other_invalid_inputs_cannot_be_overridden(clock, updates):
    node = node_for()
    for _ in range(8):
        commands = frame(node, clock, **updates)
        assert not any(c['valid'] for c in commands)
        clock[0] += 0.2


def test_no_known_line_side_keeps_waiting(clock):
    node = node_for()
    for _ in range(8):
        commands = frame(node, clock, filtered_lateral_offset_norm=0.0)
        assert not any(c['valid'] for c in commands)
        clock[0] += 0.2


def test_nearest_visible_point_selects_side_instead_of_extrapolated_offset(clock):
    node = node_for()
    info = {**unconfirmed_sparse_info(), 'center_points_px': [[100, 600], [200, 400]], 'image_width': 1280}
    assert start_turn(node, clock, info)['action'] == 'LINE_HEADING_TURN_LEFT_1'


def test_centered_current_line_keeps_previously_seen_side(clock):
    node = node_for()
    node.planner.last_line_seen_direction = 'LEFT'
    info = {**unconfirmed_sparse_info(), 'filtered_lateral_offset_norm': 0.0}
    assert start_turn(node, clock, info)['action'] == 'LINE_HEADING_TURN_LEFT_1'


def test_valid_other_target_keeps_priority_over_line_recovery(clock):
    node = node_for('AUTO')
    from test_motion_decision_planner import goal_info
    node.latest_info['goal'] = goal_info(depth_m=0.9, bearing_deg=0.0)
    node.latest_time['goal'] = clock[0]
    frame(node, clock)
    assert node.last_selected_decision.source == 'goal'
    assert not node.last_selected_decision.source_command.get('invalid_ground_recovery')


def test_image_heading_mode_keeps_existing_policy(clock):
    node = node_for()
    node.planner = MotionDecisionPlanner(MotionDecisionConfig(line_heading_source='image'))
    frame(node, clock)
    assert not node.last_selected_decision.source_command.get('invalid_ground_recovery')


def test_pickup_recovery_waits_for_camera_instead_of_larger_blind_search(clock):
    node = node_for('POST_BALL_LINE_ALIGN')
    command = start_turn(node, clock, unconfirmed_sparse_info())
    finish_turn(node, clock, command)
    before = len(node.publisher.messages)
    for _ in range(8):
        clock[0] += 0.3
        MotionDecisionNode._publish_decision(node)
    assert not any(c['valid'] for c in node.publisher.messages[before:])
    assert node.last_selected_decision.reason == 'line_ground_recovery_waiting_for_fresh_full_fit'


@pytest.mark.parametrize('phase', ['LINE_TRACK', 'POST_BALL_LINE_ALIGN'])
@pytest.mark.parametrize('capture_offset', [-0.6, 0.6])
def test_delayed_or_future_normal_fit_cannot_end_recovery(clock, phase, capture_offset):
    node = node_for(phase)
    command = start_turn(node, clock, unconfirmed_sparse_info())
    finish_turn(node, clock, command)
    clock[0] += 2.0
    normal = {**line_info(), 'ground_projection_enabled': True, 'ground_fit_point_count': 3}
    commands = frame(node, clock, normal, rgb_stamp_ns=int((clock[0] + capture_offset) * 1e9))
    assert not any(c['valid'] for c in commands)
    assert node.line_ground_recovery_active
    assert frame(node, clock, normal)[-1]['action'].startswith('STRAIGHT')


def test_failed_recovery_does_not_start_different_search_when_line_disappears(clock):
    node = node_for()
    command = start_turn(node, clock, unconfirmed_sparse_info())
    node.send_status(command['action'], command['command_id'], 'REJECTED')
    for _ in range(8):
        clock[0] += 0.2
        commands = frame(node, clock, detected=False)
        assert not any(c['valid'] for c in commands)
    normal = {**line_info(), 'ground_projection_enabled': True, 'ground_fit_point_count': 3}
    assert frame(node, clock, normal)[-1]['action'].startswith('STRAIGHT')
    assert not node.line_ground_recovery_failed


def test_two_point_valid_flag_cannot_end_active_recovery(clock):
    node = node_for()
    command = start_turn(node, clock, unconfirmed_sparse_info())
    finish_turn(node, clock, command)
    commands = frame(node, clock, {**line_info(), 'ground_projection_enabled': True,
                                  'ground_fit_point_count': 2})
    assert not any(c['valid'] for c in commands)
    assert node.line_ground_recovery_active
