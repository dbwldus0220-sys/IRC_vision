"""Exercise forward queueing and the counted post-pickup motion boundary."""

from dataclasses import replace
import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode
from test_mission_phase_flow import MissionFlowHarness, line_info
from test_line_corner_memory import CornerHarness, corner_info
from test_line_offset_alignment import sample as offset_sample
from test_motion_command_bridge import FakeBridge, decoded_messages, executor_status


@pytest.fixture
def clock(monkeypatch):
    now = [10.]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: now[0])
    monkeypatch.setattr(MotionDecisionNode, '_current_ros_time_ns', lambda _: round(now[0] * 1e9))
    return now


def start_forward(phase):
    node = MissionFlowHarness(phase=phase)
    node.phase_manager.pickups_completed = 1
    node.phase_manager.ball_grasp_results[1] = 'GRABBED'
    first = node.publish_vision(line=line_info())[-1]
    assert first['action'] == 'STRAIGHT'
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    return node, first


def capture(node, clock, info=None):
    info = line_info() if info is None else info
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * .8
    node.active_line_motion_frames = [{**info, 'rgb_stamp_ns': round((clock[0] - .01 * index) * 1e9)}
                                      for index in reversed(range(10))]
    node.observations['line'] = node.active_line_motion_frames[-1]
    node.latest_time['line'] = clock[0]
    MotionDecisionNode._prepare_pending_line_decision(node, clock[0])


@pytest.mark.parametrize('phase', [
    'AUTO', 'LINE_TRACK', 'LINE_TRACK_AFTER_PICKUP',
    'BALL_SEARCH', 'GOAL_SEARCH', 'HURDLE_APPROACH',
])
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


@pytest.mark.parametrize('issued,allowed', [(1, True), (2, True), (3, False)])
def test_no_fourth_line_command_can_be_reserved(clock, issued, allowed):
    node, first = start_forward('LINE_TRACK_AFTER_PICKUP')
    node.post_ball_line_command_ids = [first['command_id'] + i for i in range(issued)]
    capture(node, clock)
    assert (getattr(node, 'queued_general_command_id', None) is not None) is allowed


@pytest.mark.parametrize('info,expected', [
    (line_info(), 'STRAIGHT'),
    (offset_sample(offset=149.6, ground_heading_error_deg=-6.63,
                   ground_lateral_offset_m=.1572), 'STRAIGHT_1'),
])
def test_counted_stage_drains_third_command_before_transition(clock, info, expected):
    node, current = start_forward('LINE_TRACK_AFTER_PICKUP')
    for ordinal in (2, 3):
        capture(node, clock, info)
        following = node.publisher.messages[-1]
        assert following['action'] == expected
        assert following['source_command']['post_pickup_line_index'] == ordinal
        clock[0] += 20.  # Elapsed time does not change the decision budget.
        node.send_status(current['action'], current['command_id'], 'SUCCEEDED')
        assert node.mission_phase == 'LINE_TRACK_AFTER_PICKUP'
        node.send_status(following['action'], following['command_id'], 'RUNNING')
        current = following
    capture(node, clock, info)
    assert len(node.publisher.messages) == 3
    node.send_status(current['action'], current['command_id'], 'SUCCEEDED')
    assert node.mission_phase == 'GOAL_APPROACH'
    assert node.publisher.messages[-1]['action'] == 'WAIT'


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


def right_gap_line():
    return offset_sample(offset=149.6, ground_heading_error_deg=-6.63,
                         ground_lateral_offset_m=.1572)


@pytest.mark.parametrize('phase', ['AUTO', 'LINE_TRACK'])
def test_gap_forward_without_reservation_releases_lock_and_reobserves(clock, phase):
    node = CornerHarness(phase=phase)
    first = node.publish_vision(line={**right_gap_line(), 'rgb_stamp_ns': round(clock[0] * 1e9)})[-1]
    assert first['action'] == 'STRAIGHT_1'
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    clock[0] += node.active_line_motion_duration_sec
    assert getattr(node, 'queued_general_command_id', None) is None
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    assert not node.general_motion_gate.locked
    assert not node.planner.line_offset_alignment_failed
    clock[0] += .01
    before = len(node.publisher.messages)
    node.publish_vision(line={**right_gap_line(), 'rgb_stamp_ns': round(clock[0] * 1e9)})
    following = node.publisher.messages[before:][-1]
    assert following['valid'] and following['action'] == 'STRAIGHT_1'
    assert following['command_id'] != first['command_id']


@pytest.mark.parametrize('initial,next_info,expected', [
    (line_info(), line_info(20., .4), 'RECOVER_RIGHT_TURN_RIGHT_4'),
    (line_info(20., .4), line_info(), 'STRAIGHT'),
    (line_info(-20., -.4), line_info(), 'STRAIGHT'),
    (line_info(), line_info(50., .4), 'RECOVER_RIGHT_TURN_RIGHT_4'),
    (line_info(), line_info(-50., -.4), 'RECOVER_LEFT_TURN_LEFT_4'),
    (line_info(50., .4), line_info(), 'STRAIGHT'),
    (line_info(-50., -.4), line_info(50., .4), 'RECOVER_RIGHT_TURN_RIGHT_4'),
    (line_info(20., .4), line_info(-20., -.4), 'RECOVER_LEFT_TURN_LEFT_4'),
    (line_info(), corner_info('LEFT', ground_heading_error_deg=-15.), 'LEFT'),
    (line_info(), corner_info('RIGHT', ground_heading_error_deg=15.), 'RIGHT'),
    (corner_info('LEFT'), line_info(), 'STRAIGHT'),
    (corner_info('RIGHT'), line_info(), 'STRAIGHT'),
    (line_info(), right_gap_line(), 'STRAIGHT_1'),
    (line_info(20., .4), right_gap_line(), 'STRAIGHT_1'),
    (line_info(-20., -.4), right_gap_line(), 'STRAIGHT_1'),
    (right_gap_line(), right_gap_line(), 'STRAIGHT_1'),
    (right_gap_line(), line_info(), 'STRAIGHT'),
    (right_gap_line(), line_info(20., .4), 'RECOVER_RIGHT_TURN_RIGHT_4'),
    (right_gap_line(), line_info(-20., -.4), 'RECOVER_LEFT_TURN_LEFT_4'),
])
@pytest.mark.parametrize('phase', ['AUTO', 'LINE_TRACK', 'BALL_SEARCH', 'GOAL_SEARCH', 'HURDLE_APPROACH'])
def test_walking_transition_is_sent_to_sdk_before_success_without_new_vision(
    clock, monkeypatch, initial, next_info, expected, phase,
):
    monkeypatch.setattr(MotionDecisionNode, '_current_ros_time_ns', lambda _: round(clock[0] * 1e9))
    node, bridge = CornerHarness(phase=phase), FakeBridge()
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = 1.
    first = node.publish_vision(line={**initial, 'rgb_stamp_ns': round(clock[0] * 1e9)})[-1]
    bridge.navigation_command_callback(String(data=json.dumps(first)))
    first_request = decoded_messages(bridge.executor_request_publisher)[-1]
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    observe_walk_end(node, clock, next_info)
    if first['action'].startswith('RECOVER_') and expected.startswith('RECOVER_'):
        assert len(node.publisher.messages) == 1
        clock[0] = (node.active_line_motion_started_at + node.active_line_motion_duration_sec
                    - MotionDecisionNode.LINE_RECOVERY_PREQUEUE_LEAD_SEC + .01)
        MotionDecisionNode._info_callback(node, 'line')(String(data=json.dumps({
            **next_info, 'rgb_stamp_ns': round(clock[0] * 1e9),
        })))
    assert len(node.publisher.messages) == 2
    queued = node.publisher.messages[-1]
    assert queued['action'] == expected and queued['valid']
    assert node.general_motion_gate.active_command_id == first['command_id']
    bridge.navigation_command_callback(String(data=json.dumps(queued)))
    assert not bridge.queued_request_deferred
    assert len(bridge.executor_request_publisher.messages) == 2
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec
    finished_at = clock[0]
    bridge.executor_status_callback(executor_status(**first_request, status='SUCCEEDED'))
    # Completion promotes the existing SDK reservation without a second start request.
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


def test_near_corner_cannot_prequeue_over_a_stationary_heading_correction(clock, monkeypatch):
    monkeypatch.setattr(MotionDecisionNode, '_current_ros_time_ns', lambda _: round(clock[0] * 1e9))
    node = CornerHarness(phase='AUTO')
    first = node.publish_vision(line={**line_info(), 'rgb_stamp_ns': round(clock[0] * 1e9)})[-1]
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    observe_walk_end(node, clock, corner_info('RIGHT', ground_heading_error_deg=75.))
    assert len(node.publisher.messages) == 1
    assert getattr(node, 'queued_general_command_id', None) is None
    assert node.general_motion_gate.active_command_id == first['command_id']


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
    node.observations['line'] = node.active_line_motion_frames[-1]
    node.latest_time['line'] = clock[0]
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
    # The SDK reservation was already sent; failure must not dispatch it again.
    assert len(bridge.executor_request_publisher.messages) == 2


def test_walking_capture_windows_match_active_recovery_catalogs():
    from pathlib import Path
    import yaml
    from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
    root = Path(__file__).resolve().parents[3]
    aliases = yaml.safe_load((root / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    catalog = {m['name']: m for m in json.loads((root / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
    for action, (duration, _) in MotionDecisionNode.LINE_MOTION_CAPTURE_CONFIG.items():
        motion_id = MotionCommandBridgeNode.motion_id_for_action(action)
        if motion_id is None:
            # Legacy recovery timing slots retain the active four-cycle gait.
            assert action.startswith('RECOVER_')
            motion_id = MotionCommandBridgeNode.motion_id_for_action(action.rsplit('_', 1)[0] + '_4')
        motion = catalog[aliases[motion_id]]
        actual = max(f['start_ms'] + f['time_ms'] for f in motion['frames']) * motion['repeat_count'] / motion['playback_speed'] / 1000
        assert duration == pytest.approx(actual), action
    for action, duration in MotionDecisionNode.GOAL_FORWARD_DURATION_SEC.items():
        motion = catalog[aliases[MotionCommandBridgeNode.motion_id_for_action(action)]]
        actual = max(f['start_ms'] + f['time_ms'] for f in motion['frames']) * motion['repeat_count'] / motion['playback_speed'] / 1000
        assert duration == pytest.approx(actual), action
    for action, duration in MotionDecisionNode.APPROACH_WALK_DURATION_SEC.items():
        motion_id = ('ball_camera_down_forward_4' if action == 'STRAIGHT'
                     else MotionCommandBridgeNode.motion_id_for_action(action))
        motion = catalog[aliases[motion_id]]
        actual = max(f['start_ms'] + f['time_ms'] for f in motion['frames']) * motion['repeat_count'] / motion['playback_speed'] / 1000
        assert duration == pytest.approx(actual), action
