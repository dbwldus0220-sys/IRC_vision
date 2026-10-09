"""Exercise forward queueing and the timed post-pickup motion boundary."""

from dataclasses import replace
import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode
from test_mission_phase_flow import MissionFlowHarness, line_info
from test_line_corner_memory import CornerHarness, corner_info
from test_motion_command_bridge import FakeBridge, decoded_messages, executor_status


@pytest.fixture
def clock(monkeypatch):
    now = [10.]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: now[0])
    return now


def start_forward(phase):
    node = MissionFlowHarness(phase=phase)
    node.phase_manager.pickups_completed = 1
    node.phase_manager.ball_grasp_results[1] = 'GRABBED'
    first = node.publish_vision(line=line_info())[-1]
    assert first['action'] == 'STRAIGHT'
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    return node, first


def capture(node, clock):
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * .8
    node.active_line_motion_frames = [line_info() for _ in range(10)]
    MotionDecisionNode._prepare_pending_line_decision(node, clock[0])


@pytest.mark.parametrize('phase', ['AUTO', 'LINE_TRACK', 'LINE_TRACK_AFTER_PICKUP'])
def test_normal_line_queues_next_forward_before_current_finishes(clock, phase):
    node, first = start_forward(phase)
    capture(node, clock)
    queued = node.publisher.messages[-1]
    assert queued['command_id'] != first['command_id']
    assert queued['action'] == 'STRAIGHT'
    assert getattr(node, 'queued_general_command_id', None) == queued['command_id']
    assert node.general_motion_gate.active_command_id == first['command_id']
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    node.send_status(queued['action'], queued['command_id'], 'RUNNING')
    assert node.general_motion_gate.active_command_id == queued['command_id']
    assert getattr(node, 'queued_general_command_id', None) is None


@pytest.mark.parametrize('source,allowed', [('ball', True), ('goal', True), ('hurdle', False), ('finish', False)])
def test_timed_line_masks_only_the_same_targets_as_mission_selection(clock, source, allowed):
    node, _ = start_forward('LINE_TRACK_AFTER_PICKUP')
    observations = {source: {'detected': True, 'raw_detected': True}}
    assert MotionDecisionNode._line_only_prequeue_allowed(node, observations) is allowed


@pytest.mark.parametrize('remaining,allowed', [(5., True), (2.467, False), (.1, False), (-.1, False)])
def test_no_extra_queue_when_next_full_forward_will_not_fit(clock, remaining, allowed):
    node, _ = start_forward('LINE_TRACK_AFTER_PICKUP')
    current_end = node.active_line_motion_started_at + node.active_line_motion_duration_sec
    node.post_ball_line_run_until = current_end + remaining
    capture(node, clock)
    assert (getattr(node, 'queued_general_command_id', None) is not None) is allowed


def test_timed_stage_waits_for_already_queued_forward_before_transition(clock):
    node, first = start_forward('LINE_TRACK_AFTER_PICKUP')
    capture(node, clock)
    second = node.publisher.messages[-1]
    # A slow executor may finish after the nominal duration. Drain the single
    # accepted command; never change camera pose while that command is pending.
    clock[0] = 21.
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    MotionDecisionNode._publish_decision(node)
    assert node.mission_phase == 'LINE_TRACK_AFTER_PICKUP'
    node.send_status(second['action'], second['command_id'], 'RUNNING')
    capture(node, clock)
    assert len(node.publisher.messages) == 2
    node.send_status(second['action'], second['command_id'], 'SUCCEEDED')
    MotionDecisionNode._publish_decision(node)
    assert node.publisher.messages[-1]['action'] == 'POST_BALL_GOAL_TRANSITION'


@pytest.mark.parametrize('action', ['RIGHT', 'LINE_HEADING_TURN_RIGHT_2', 'RECOVER_LEFT_TURN_LEFT_4'])
def test_timed_queue_does_not_bypass_turn_boundary(clock, action):
    node, _ = start_forward('LINE_TRACK_AFTER_PICKUP')
    decision = replace(node.last_selected_decision, action=action)
    MotionDecisionNode._publish_decision(node, decision, queue_while_locked=True)
    assert len(node.publisher.messages) == 1
    assert getattr(node, 'queued_general_command_id', None) is None


def observe_walk_end(node, clock, info):
    """Feed distinct real callback captures during the last part of one gait."""
    started, duration = node.active_line_motion_started_at, node.active_line_motion_duration_sec
    assert started is not None
    for index in range(1, 26):
        clock[0] = started + duration * .8 * index / 25
        MotionDecisionNode._info_callback(node, 'line')(String(data=json.dumps({
            **info, 'rgb_stamp_ns': round(clock[0] * 1e9),
        })))
    assert clock[0] < started + duration


@pytest.mark.parametrize('initial,next_info,expected', [
    (line_info(), line_info(20., .4), 'RECOVER_RIGHT_TURN_RIGHT_4'),
    (line_info(20., .4), line_info(), 'STRAIGHT'),
    (line_info(-20., -.4), line_info(), 'STRAIGHT'),
    (line_info(20., .4), line_info(-20., -.4), 'RECOVER_LEFT_TURN_LEFT_4'),
    (line_info(), corner_info('LEFT'), 'LEFT'),
    (line_info(), corner_info('RIGHT'), 'RIGHT'),
    (line_info(), corner_info('RIGHT', ground_heading_error_deg=75.), 'RIGHT'),
    (corner_info('LEFT'), line_info(), 'STRAIGHT'),
    (corner_info('RIGHT'), line_info(), 'STRAIGHT'),
])
def test_walking_transition_is_reserved_and_dispatched_at_success_without_new_vision(
    clock, monkeypatch, initial, next_info, expected,
):
    monkeypatch.setattr(MotionDecisionNode, '_current_ros_time_ns', lambda _: round(clock[0] * 1e9))
    node, bridge = CornerHarness(phase='AUTO'), FakeBridge()
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = 1.
    first = node.publish_vision(line={**initial, 'rgb_stamp_ns': round(clock[0] * 1e9)})[-1]
    bridge.navigation_command_callback(String(data=json.dumps(first)))
    first_request = decoded_messages(bridge.executor_request_publisher)[-1]
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    observe_walk_end(node, clock, next_info)
    assert len(node.publisher.messages) == 2
    queued = node.publisher.messages[-1]
    assert queued['action'] == expected and queued['valid']
    assert node.general_motion_gate.active_command_id == first['command_id']
    bridge.navigation_command_callback(String(data=json.dumps(queued)))
    assert bridge.queued_request_deferred
    assert len(bridge.executor_request_publisher.messages) == 1
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec
    finished_at = clock[0]
    bridge.executor_status_callback(executor_status(**first_request, status='SUCCEEDED'))
    # The existing completion callback dispatches immediately, with no timer or RGB input.
    requests = decoded_messages(bridge.executor_request_publisher)
    assert len(requests) == 2 and requests[-1]['command_id'] == queued['command_id']
    assert clock[0] == finished_at
    assert bridge.transition_dwell_until is None
    MotionDecisionNode._motion_status_callback(node, bridge.motion_status_publisher.messages[-1])
    assert getattr(node, 'correction_post_motion_dwell_until', None) is None
    bridge.executor_status_callback(executor_status(**requests[-1], status='RUNNING'))
    MotionDecisionNode._motion_status_callback(node, bridge.motion_status_publisher.messages[-1])
    assert node.general_motion_gate.active_command_id == queued['command_id']
    assert node.active_line_motion_action == expected
    observe_walk_end(node, clock, line_info())
    assert len(node.publisher.messages) == 3
    assert node.publisher.messages[-1]['action'] == 'STRAIGHT'


@pytest.mark.parametrize('grasp,queues', [('NOT_GRABBED', True), ('UNKNOWN', True), ('GRABBED', False)])
def test_goal_only_blocks_forward_reservation_with_verified_ball(clock, grasp, queues):
    node, first = start_forward('AUTO')
    node.phase_manager.ball_sections_processed = 1
    node.phase_manager.ball_grasp_results[1] = grasp
    node.latest_info['goal'] = {'detected': True, 'raw_detected': True}
    node.latest_time['goal'] = clock[0]
    # This harness supplies observations directly; live capture freshness is tested separately.
    capture(node, clock)
    assert (getattr(node, 'queued_general_command_id', None) is not None) is queues
    if queues:
        assert node.publisher.messages[-1]['action'] == 'STRAIGHT'
        assert node.general_motion_gate.active_command_id == first['command_id']


@pytest.mark.parametrize('source', ['ball', 'hurdle', 'finish'])
def test_empty_pickup_still_respects_other_mission_reservations(clock, source):
    node, _ = start_forward('AUTO')
    node.phase_manager.ball_sections_processed = 1
    node.phase_manager.ball_grasp_results[1] = 'NOT_GRABBED'
    assert not MotionDecisionNode._line_only_prequeue_allowed(
        node, {source: {'detected': True, 'raw_detected': True}})


@pytest.mark.parametrize('updates', [
    {'ground_projection_valid': False}, {'heading_quality': .1}, {'detected': False},
    {'ground_heading_error_deg': 70.},
])
def test_bad_line_or_stationary_turn_cannot_be_reserved_as_walking(clock, updates):
    node, _ = start_forward('AUTO')
    observe_walk_end(node, clock, {**line_info(), **updates})
    assert getattr(node, 'queued_general_command_id', None) is None


@pytest.mark.parametrize('age', [-.1, .6])
def test_reservation_requires_fresh_camera_capture(clock, monkeypatch, age):
    node, _ = start_forward('AUTO')
    monkeypatch.setattr(MotionDecisionNode, '_current_ros_time_ns', lambda _: round(clock[0] * 1e9))
    info = {**line_info(), 'rgb_stamp_ns': round((clock[0] - age) * 1e9)}
    node.latest_info['line'] = info
    node.active_line_motion_frames = [info] * 10
    MotionDecisionNode._prepare_pending_line_decision(node, clock[0])
    assert getattr(node, 'queued_general_command_id', None) is None


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED'])
def test_failed_walking_drops_reserved_correction_in_node_and_bridge(clock, status):
    node, first = start_forward('AUTO')
    bridge = FakeBridge()
    bridge.navigation_command_callback(String(data=json.dumps(first)))
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    observe_walk_end(node, clock, line_info(20., .4))
    queued = node.publisher.messages[-1]
    assert queued['action'] == 'RECOVER_RIGHT_TURN_RIGHT_4'
    bridge.navigation_command_callback(String(data=json.dumps(queued)))
    bridge.executor_status_callback(executor_status(**request, status=status))
    MotionDecisionNode._motion_status_callback(node, bridge.motion_status_publisher.messages[-1])
    assert getattr(node, 'queued_general_command_id', None) is None
    assert bridge.queued_request_id is None
    assert len(bridge.executor_request_publisher.messages) == 1


def test_walking_capture_windows_match_active_recovery_catalogs():
    from pathlib import Path
    import yaml
    from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
    root = Path(__file__).resolve().parents[3]
    aliases = yaml.safe_load((root / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    catalog = {m['name']: m for m in json.loads((root / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
    for action in ['LEFT', 'RIGHT', 'RECOVER_LEFT_TURN_LEFT_4', 'RECOVER_RIGHT_TURN_RIGHT_4']:
        motion = catalog[aliases[MotionCommandBridgeNode.motion_id_for_action(action)]]
        actual = max(f['start_ms'] + f['time_ms'] for f in motion['frames']) * motion['repeat_count'] / motion['playback_speed'] / 1000
        assert MotionDecisionNode.LINE_MOTION_CAPTURE_CONFIG[action][0] == pytest.approx(actual)
