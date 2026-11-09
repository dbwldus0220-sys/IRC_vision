"""Queue offset/search turns using fresh late images and defer physical dispatch."""

import json
from pathlib import Path

import pytest
import yaml
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.motion_decision_planner import MotionDecision
from mission_control.lost_search_turn_limiter import LostSearchTurnLimiter
from test_continuous_approach import clock, fresh
from test_mission_phase_flow import line_info
from test_line_offset_alignment import sample
from test_motion_command_bridge import FakeBridge, decoded_messages
from test_pickup_continuous_walk import forward_status
from test_wait_refresh_and_fine_settle import LiveInputHarness, observe


def start(clock, action='STRAIGHT', phase='LINE_TRACK'):
    node = LiveInputHarness(phase=phase)
    node.queued_general_command_id = None
    node.correction_post_motion_dwell_until = None
    node.planner.last_line_seen_direction = 'LEFT'
    if action == 'LINE_LOST_TURN_LEFT_1':
        node.lost_search_turn_limiter = LostSearchTurnLimiter(staged_sources=('line',))
        node.lost_search_turn_limiter.counts['line'] = 1
        node.lost_search_turn_limiter.angles['line'] = 45.
        node.lost_search_turn_limiter.first_directions['line'] = 'LEFT'
    initial = {'detected': False} if action.startswith('LINE_LOST_') else line_info(heading=-20, offset=-.2)
    fresh(node, clock, 'line', initial, plan=False)
    decision = MotionDecision(phase=phase, source='line', action=action, valid=True,
                              reason='test', sdk_motion_requested=False,
                              requires_ack=False, source_command={})
    Node._publish_decision(node, decision)
    if not node.publisher.messages or not node.publisher.messages[-1]['valid']:
        clock[0] += .6
        fresh(node, clock, 'line', initial, plan=False)
        Node._publish_decision(node, decision)
    command = node.publisher.messages[-1]
    assert command['action'] == action
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    if bridge.transition_dwell_until is not None:
        clock[0] = bridge.transition_dwell_until
        bridge._check_atomic_dwell()
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, request, 'RUNNING')
    assert node.active_line_motion_started_at is not None
    return node, bridge, request


def offset():
    return sample(offset=-200, target_angle=-35, ground_heading_error_deg=0)


def late(node, clock, info, count=3):
    clock[0] = max(clock[0], node.active_line_motion_started_at
                   + node.active_line_motion_duration_sec * .72)
    for _ in range(count):
        fresh(node, clock, 'line', info, plan=False)


@pytest.mark.parametrize('phase', ['AUTO', 'LINE_TRACK'])
@pytest.mark.parametrize('action,info,expected', [
    ('STRAIGHT', offset(), 'LINE_OFFSET_TURN_LEFT_2'),
    ('STRAIGHT', {'detected': False}, 'LINE_LOST_TURN_LEFT_3'),
    ('LINE_OFFSET_TURN_LEFT_2', line_info(), 'STRAIGHT'),
    ('LINE_OFFSET_TURN_LEFT_2', offset(), 'LINE_OFFSET_TURN_LEFT_2'),
    ('LINE_OFFSET_TURN_LEFT_6', line_info(), 'STRAIGHT'),
    ('LINE_OFFSET_TURN_LEFT_6', offset(), 'LINE_OFFSET_TURN_LEFT_2'),
    ('LINE_LOST_TURN_LEFT_3', line_info(), 'STRAIGHT'),
    ('LINE_LOST_TURN_LEFT_3', {'detected': False}, 'LINE_LOST_TURN_LEFT_1'),
])
def test_turn_reserves_once_and_starts_only_after_success(clock, phase, action, info, expected):
    node, bridge, request = start(clock, action, phase)
    before = len(node.publisher.messages)
    late(node, clock, info, 2)
    assert len(node.publisher.messages) == before
    late(node, clock, info, 1)
    command = node.publisher.messages[-1]
    assert command['action'] == expected
    assert command['source_command']['line_turn_prequeue']
    assert node.queued_general_command_id == command['command_id']
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    assert bridge.queued_request_deferred
    assert len(bridge.executor_request_publisher.messages) == 1
    late(node, clock, info, 3)
    assert node.publisher.messages[-1]['command_id'] == command['command_id']
    forward_status(node, bridge, request, 'SUCCEEDED')
    assert node.correction_post_motion_dwell_until is None
    assert bridge.transition_dwell_until == pytest.approx(clock[0] + .5)
    clock[0] += .499
    bridge._check_atomic_dwell()
    assert len(bridge.executor_request_publisher.messages) == 1
    clock[0] += .001
    bridge._check_atomic_dwell()
    next_request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert next_request['command_id'] == command['command_id']
    assert next_request['motion_id'] == bridge.motion_id_for_action(expected)
    assert bridge.transition_dwell_until is None
    forward_status(node, bridge, next_request, 'RUNNING')
    assert node.general_motion_gate.active_command_id == command['command_id']
    assert node.queued_general_command_id is None


@pytest.mark.parametrize('bad', ['early', 'old', 'future', 'missing_stamp', 'duplicate', 'low_quality'])
def test_invalid_frames_cannot_authorize_turn(clock, bad):
    node, _, _ = start(clock)
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * (.4 if bad == 'early' else .75)
    stamp = round(clock[0] * 1e9)
    for _ in range(4):
        info = offset()
        current = round((clock[0] + .01) * 1e9)
        if bad == 'old':
            current -= 1_000_000_000
        elif bad == 'future':
            current += 1_000_000_000
        elif bad == 'missing_stamp':
            current = None
        elif bad == 'duplicate':
            current = stamp
        elif bad == 'low_quality':
            info['heading_quality'] = .01
        observe(node, clock, 'line', {**info, 'rgb_stamp_ns': current}, plan=False)
    assert node.queued_general_command_id is None


@pytest.mark.parametrize('status', ['FAILED', 'CANCELLED', 'TIMEOUT'])
@pytest.mark.parametrize('action', ['STRAIGHT', 'LINE_OFFSET_TURN_LEFT_6'])
def test_failure_discards_deferred_turn(clock, status, action):
    node, bridge, request = start(clock, action)
    late(node, clock, offset())
    bridge.navigation_command_callback(String(data=json.dumps(node.publisher.messages[-1])))
    forward_status(node, bridge, request, status)
    assert len(bridge.executor_request_publisher.messages) == 1
    assert bridge.queued_request_id is None
    assert node.queued_general_command_id is None


def test_unreserved_turn_retains_stop_and_fresh_observation(clock):
    node, bridge, request = start(clock, 'LINE_OFFSET_TURN_LEFT_2')
    late(node, clock, offset(), 2)
    assert node.queued_general_command_id is None
    forward_status(node, bridge, request, 'SUCCEEDED')
    assert node.correction_post_motion_dwell_until == pytest.approx(clock[0] + .5)
    assert node.latest_info['line'] is None


def test_capture_does_not_start_on_preparation(clock):
    node, _, request = start(clock, 'LINE_OFFSET_TURN_LEFT_2')
    Node._discard_line_motion_capture(node)
    node.send_status(request['action'], request['command_id'], 'RUNNING', 'forward_to_turn_ready')
    assert node.active_line_motion_started_at is None
    node.send_status(request['action'], request['command_id'], 'RUNNING', request['motion_id'])
    assert node.active_line_motion_started_at is not None


def test_capture_durations_match_motion_catalog():
    root = Path(__file__).resolve().parents[3]
    aliases = yaml.safe_load((root / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    motions = {m['name']: m for m in json.loads((root / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
    for action, (seconds, _) in Node.LINE_TURN_CAPTURE_CONFIG.items():
        alias = FakeBridge.ACTION_TO_MOTION_ID[action]
        motion = motions[aliases[alias]]
        actual = max(f['start_ms'] + f['time_ms'] for f in motion['frames']) / 1000
        actual *= motion['repeat_count'] / motion['playback_speed']
        assert seconds == pytest.approx(actual), action


def test_source78_six_cycle_turn_uses_updated_capture_window(clock):
    node, _, _ = start(clock, 'LINE_OFFSET_TURN_LEFT_6')
    started = node.active_line_motion_started_at
    assert node.active_line_motion_duration_sec == pytest.approx(.355 * 6 / .95)
    clock[0] = started + .355 * 6 / .95 * .60
    for _ in range(3):
        fresh(node, clock, 'line', line_info(), plan=False)
    assert node.queued_general_command_id is None
    late(node, clock, line_info(), 3)
    assert node.queued_general_action == 'STRAIGHT'
    # The three fresh samples authorize a reservation before this gait completes.
    assert started + .355 * 6 / .95 * .70 <= clock[0] < started + .355 * 6 / .95


@pytest.mark.parametrize('action', ['LINE_OFFSET_TURN_LEFT_1', 'LINE_LOST_TURN_LEFT_1'])
def test_short_turn_can_use_recent_images_before_the_late_window(clock, action):
    node, _, _ = start(clock, action)
    started, duration = node.active_line_motion_started_at, node.active_line_motion_duration_sec
    for fraction in (.35, .55, .85):
        clock[0] = started + duration * fraction
        fresh(node, clock, 'line', line_info(), plan=False)
        if fraction < .7:
            assert node.queued_general_command_id is None
    assert node.queued_general_action == 'STRAIGHT'


@pytest.mark.parametrize('blocked', ['safety', 'executor', 'ball', 'special', 'search_limit'])
def test_turn_queue_keeps_execution_and_mission_guards(clock, blocked):
    node, _, _ = start(clock)
    info = offset()
    if blocked == 'safety':
        node.safety_interlock.observe_executor_status(error_code='SDK_INTERNAL_ERROR',
            message='test', action='STRAIGHT', command_id=1, event_id=None)
    elif blocked == 'executor':
        node.executor_auto_ready = False
    elif blocked == 'ball':
        node.ball_pickup_entry_pending = True
    elif blocked == 'special':
        node.phase_manager.active_special_command_id = 99
    else:
        node.lost_search_turn_limiter.failed_sources.add('line')
        info = {'detected': False}
    late(node, clock, info)
    assert node.queued_general_command_id is None


def test_changing_turn_candidate_requires_new_matching_frames(clock):
    node, _, _ = start(clock)
    late(node, clock, offset(), 2)
    late(node, clock, {'detected': False}, 2)
    assert node.queued_general_command_id is None
    late(node, clock, offset(), 2)
    assert node.queued_general_command_id is None
    late(node, clock, offset(), 1)
    assert node.queued_general_action == 'LINE_OFFSET_TURN_LEFT_2'
