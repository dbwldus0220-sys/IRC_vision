"""Reserve the post-shot forward during either goal's extra alignment turn."""

import json
from pathlib import Path

import pytest
import yaml
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_continuous_approach import clock
from test_post_ball_goal_handoff import receive
from test_mission_phase_flow import line_info
from test_wait_refresh_and_fine_settle import LiveInputHarness
from test_motion_command_bridge import FakeBridge, decoded_messages
from test_pickup_continuous_walk import forward_status


def start_turn(clock, section, heading=20.):
    node = LiveInputHarness(phase='POST_SHOT_LINE_ALIGN')
    node.phase_manager.ball_sections_processed = section
    node.queued_general_command_id = None
    node.queued_general_action = None
    node.queued_general_source = None
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = (
        'post_shot_default_turn_right_5' if section == 1 else 'post_shot_default_turn_left_3'
    )
    bridge.stationary_since = clock[0] - .5
    receive(node, clock, 'line', line_info(heading=heading))
    Node._publish_decision(node)
    command = node.publisher.messages[-1]
    assert command['action'].startswith('POST_SHOT_LINE_TURN_')
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, request, 'RUNNING')
    assert node.active_line_motion_started_at == clock[0]
    return node, bridge, request


def turn_frames(node, clock, sample=None, fractions=None):
    started = node.active_line_motion_started_at
    duration = node.active_line_motion_duration_sec
    if fractions is None:
        # Three 10 Hz observations, all within the 0.5 s freshness limit,
        # including for longer stationary turns.
        fractions = (.85 - .2 / duration, .85 - .1 / duration, .85)
    for fraction in fractions:
        clock[0] = started + duration * fraction
        receive(node, clock, 'line', sample or line_info())


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('heading', [20., 31., 35., -20., -35., 35.001, -35.001])
def test_both_goals_reserve_and_start_forward_without_post_turn_images(clock, section, heading):
    node, bridge, turn = start_turn(clock, section, heading)
    turn_frames(node, clock)
    reserved = node.publisher.messages[-1]
    assert reserved['action'] == 'POST_SHOT_FORWARD'
    assert reserved['source_command']['post_shot_forward_prequeue']
    assert node.mission_phase == 'POST_SHOT_LINE_ALIGN'
    assert node.queued_general_command_id == reserved['command_id']
    bridge.navigation_command_callback(String(data=json.dumps(reserved)))
    assert bridge.queued_request_deferred
    assert len(bridge.executor_request_publisher.messages) == 1
    clock[0] += .03
    receive(node, clock, 'line', line_info())
    assert node.publisher.messages[-1]['command_id'] == reserved['command_id']
    forward_status(node, bridge, turn, 'SUCCEEDED')
    assert node.mission_phase == 'POST_SHOT_FORWARD'
    assert node.post_shot_dwell_until is None
    assert bridge.transition_dwell_until == pytest.approx(clock[0] + (.3 if 'RIGHT' in turn['action'] else .5))
    clock[0] = bridge.transition_dwell_until
    bridge._check_atomic_dwell()
    forward = decoded_messages(bridge.executor_request_publisher)[-1]
    assert forward['command_id'] == reserved['command_id']
    assert forward['motion_id'] == 'line_forward_6'
    assert bridge.transition_dwell_until is None
    forward_status(node, bridge, turn, 'SUCCEEDED')  # Duplicate terminal cannot dispatch twice.
    assert len(bridge.executor_request_publisher.messages) == 2
    forward_status(node, bridge, forward, 'RUNNING')
    assert node.general_motion_gate.active_command_id == reserved['command_id']
    forward_status(node, bridge, forward, 'SUCCEEDED')
    assert node.mission_phase == ('AUTO' if section == 1 else 'LINE_TRACK')


@pytest.mark.parametrize('changes', [
    {'detected': False}, {'geometry_quality': .01},
    {'ground_heading_error_deg': None}, {'ground_heading_error_deg': 40.},
])
def test_unusable_or_unaligned_turn_frames_do_not_reserve(clock, changes):
    node, bridge, turn = start_turn(clock, 2)
    turn_frames(node, clock, {**line_info(), **changes})
    assert node.queued_general_command_id is None
    forward_status(node, bridge, turn, 'SUCCEEDED')
    assert node.post_shot_dwell_until == pytest.approx(clock[0] + (.3 if 'RIGHT' in turn['action'] else .5))
    assert len(bridge.executor_request_publisher.messages) == 1


@pytest.mark.parametrize('kind', ['early', 'duplicate', 'before_turn', 'stale'])
def test_only_distinct_recent_in_turn_frames_can_reserve(clock, kind):
    node, _, _ = start_turn(clock, 1)
    if kind == 'early':
        turn_frames(node, clock, fractions=(.1, .2, .3))
    else:
        started = clock[0]
        clock[0] += node.active_line_motion_duration_sec * .85
        for i in range(3):
            stamp = started - .1 if kind == 'before_turn' else clock[0] - 1. if kind == 'stale' else clock[0]
            Node._info_callback(node, 'line')(String(data=json.dumps({
                **line_info(), 'rgb_stamp_ns': round(stamp * 1e9),
            })))
    assert node.queued_general_command_id is None


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED'])
def test_turn_failure_discards_reserved_forward(clock, section, status):
    node, bridge, turn = start_turn(clock, section, -20.)
    turn_frames(node, clock)
    bridge.navigation_command_callback(String(data=json.dumps(node.publisher.messages[-1])))
    assert bridge.queued_request_deferred
    forward_status(node, bridge, turn, status)
    assert node.phase_manager.post_shot_failed
    assert node.queued_general_command_id is None
    assert bridge.queued_request_id is None
    assert len(bridge.executor_request_publisher.messages) == 1


@pytest.mark.parametrize('blocked,value', [
    ('executor_auto_ready', False), ('decision_started', False),
])
def test_reservation_retains_executor_and_start_gates(clock, blocked, value, monkeypatch):
    node, _, _ = start_turn(clock, 1)
    monkeypatch.setattr(Node, '_poll_start_enter', lambda _: None)
    setattr(node, blocked, value)
    turn_frames(node, clock)
    assert node.queued_general_command_id is None


@pytest.mark.parametrize('heading', [-60., -31., 31., 60.])
def test_actual_turn_running_starts_capture_after_preparation(clock, heading):
    node = LiveInputHarness(phase='POST_SHOT_LINE_ALIGN')
    receive(node, clock, 'line', line_info(heading=heading))
    Node._publish_decision(node)
    command = node.publisher.messages[-1]
    node.send_status(command['action'], command['command_id'], 'RUNNING', motion_id='fine_to_turn_ready_90')
    assert node.active_line_motion_started_at is None
    clock[0] += .5
    node.send_status(command['action'], command['command_id'], 'RUNNING',
                     motion_id=FakeBridge.ACTION_TO_MOTION_ID[command['action']])
    assert node.active_line_motion_started_at == clock[0]
    receive(node, clock, 'line', line_info())
    clock[0] += .01
    node.send_status(command['action'], command['command_id'], 'RUNNING',
                     motion_id=FakeBridge.ACTION_TO_MOTION_ID[command['action']])
    assert len(node.active_line_motion_frames) == 1


def test_capture_timings_match_runtime_turn_catalog():
    root = Path(__file__).resolve().parents[3]
    aliases = yaml.safe_load((root / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    catalog = {m['name']: m for m in json.loads((root / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
    for action, alias in FakeBridge.ACTION_TO_MOTION_ID.items():
        if not action.startswith('POST_SHOT_LINE_TURN_'):
            continue
        motion = catalog[aliases[alias]]
        duration = max(f['start_ms'] + f['time_ms'] for f in motion['frames']) / 1000
        duration *= motion['repeat_count'] / motion['playback_speed']
        assert Node.POST_SHOT_ALIGN_CAPTURE_CONFIG[action][0] == pytest.approx(duration)
