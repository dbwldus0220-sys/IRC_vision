"""Verify catalog64 and camera-down/goal sequencing without robot hardware."""

import json
from pathlib import Path
import sys

import pytest
import yaml

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
from mission_control.motion_decision_node import MotionDecisionNode
from test_mission_phase_flow import MissionFlowHarness, mark_next_ball_grabbed, score_ready_goal
from test_motion_command_bridge import (
    FakeBridge, navigation_message, decoded_messages, complete_active_motion,
    fine_alignment_message, string_message, executor_status,
)
from test_shot_start_posture import finish_sequence

ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261031_catalog64_goal_prepare'
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source


def catalog(path):
    motions = json.loads(path.read_text())['motions']
    assert len(motions) == len({m['name'] for m in motions})
    return {m['name']: m for m in motions}


@pytest.mark.parametrize('filename', ['robot_motions_pc.json', 'robot_motions_runtime.json'])
def test_selective_source64_import_preserves_other_motions(filename):
    names = json.loads((UPDATE / 'manifest.json').read_text())['motion_names']
    source = catalog(UPDATE / 'source64.json')
    before = catalog(UPDATE / 'before/artifacts' / filename)
    actual = catalog(ROOT / 'artifacts' / filename)
    assert actual.keys() == before.keys() | set(names)
    for name, motion in actual.items():
        expected = source[name] if name in names else before[name]
        if name in names and 'runtime' in filename:
            expected = runtime_motion_from_source(expected)
        assert motion == expected, name
    aliases = yaml.safe_load((ROOT / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    assert set(aliases.values()) <= actual.keys()
    assert aliases['goal_camera_90_crab_left'] == 'r미세왼옆꽃게90도'
    if 'runtime' in filename:
        for name in names:
            motion = actual[name]
            duration = max(f['start_ms'] + f['time_ms'] for f in motion['frames'])
            duration *= motion['repeat_count'] / motion['playback_speed']
            assert duration + motion['completion']['settle_timeout_ms'] < FakeBridge.DEFAULT_TIMEOUT_MS


@pytest.mark.parametrize('initial', [True, False])
@pytest.mark.parametrize('offset,count', [(-201, 2), (-200, 1), (-199, 1), (199, 1), (200, 1), (201, 2)])
def test_crab_selects_repeat_then_returns_to_same_checkpoint(initial, offset, count):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action='PICKUP_NOW'))
    if not initial:
        bridge.pickup_initial_align_waiting = False
        bridge.pickup_fine_align_waiting = True
        bridge.active_motion_id = bridge.FINE_ALIGN_MARKER
    side = 'left' if offset < 0 else 'right'
    bridge.last_physical_motion_id = f'pickup_crab_{side}_0'
    action = f'BALL_PICKUP_{"INITIAL_" if initial else ""}CRAB_{side.upper()}'
    payload = json.loads(fine_alignment_message(action).data)
    payload['source_command'] = {'offset_x_px': offset}
    bridge.navigation_command_callback(string_message(payload))
    expected = f'pickup_crab_{side}_0' + ('_2' if count == 2 else '')
    assert bridge.active_motion_id == expected
    complete_active_motion(bridge)
    if bridge.active_dwell_until is not None:
        bridge._check_atomic_dwell(bridge.active_dwell_until)
    if bridge.pickup_initial_align_dwell_until is not None:
        bridge._check_pickup_initial_align_dwell(bridge.pickup_initial_align_dwell_until)
    assert bridge.pickup_initial_align_waiting if initial else bridge.pickup_fine_align_waiting
    assert bridge.active_action == 'PICKUP_NOW'
    assert [m['motion_id'] for m in decoded_messages(bridge.executor_request_publisher)] == [expected]


@pytest.mark.parametrize('side,other', [('left', 'right'), ('right', 'left')])
def test_two_cycle_crab_keeps_transition_routes(side, other):
    bridge = FakeBridge()
    bridge.last_physical_motion_id = f'pickup_crab_{side}_0_2'
    assert bridge._transition_prepare_motion('pickup_fine_forward_0') == f'pickup_crab_{side}_to_fine_0'
    assert bridge._transition_prepare_motion('pickup_camera_down_turn_left_1') == f'pickup_crab_{side}_to_turn_0'
    assert bridge._transition_prepare_motion(f'pickup_crab_{other}_0_2') == f'pickup_crab_{side}_to_{other}_0'
    assert bridge._transition_prepare_motion(f'pickup_crab_{side}_0') is None


@pytest.mark.parametrize('previous,expected', [
    ('goal_camera_90_crab_left', 'goal_crab_left_to_fine_90'),
    ('goal_camera_90_crab_right', 'goal_crab_right_to_fine_90'),
    ('goal_camera_90_turn_left_1', 'goal_fine_prepare_90'),
    ('goal_camera_90_forward_4', 'goal_fine_prepare_90'),
    ('goal_camera_90_backward_1', 'goal_fine_prepare_90'),
    ('post_ball_camera_90', 'goal_fine_prepare_90'),
    ('goal_camera_90_fine_forward_3', None),
])
def test_prepare_only_uses_last_posture_and_never_shoots(previous, expected):
    bridge = FakeBridge()
    bridge.last_physical_motion_id = previous
    bridge.navigation_command_callback(navigation_message(action='GOAL_SHOT_PREPARE'))
    finish_sequence(bridge)
    assert [m['motion_id'] for m in decoded_messages(bridge.executor_request_publisher)] == ([expected] if expected else [])
    assert decoded_messages(bridge.motion_status_publisher)[-1]['status'] == 'SUCCEEDED'
    bridge.navigation_command_callback(navigation_message(action='SHOT', command_id=8001,
                                                          source_command={'shot_region': 4}))
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == 'goal_shot_region_4'
    assert bridge.active_pickup_sequence == ()


@pytest.fixture
def goal_node(monkeypatch):
    clock = [10.0]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: clock[0])
    node = MissionFlowHarness(phase='GOAL_APPROACH')
    mark_next_ball_grabbed(node)
    return node, clock


def prepare(node, clock):
    command = node.publish_vision(goal={**score_ready_goal(), 'depth_m': .48, 'offset_x_px': 30})[-1]
    assert command['action'] == 'GOAL_SHOT_PREPARE'
    assert command['source_command']['shot_region'] is None
    node.send_status(command['action'], command['command_id'], 'RUNNING')
    clock[0] = 12.0
    assert node.publish_vision(goal=score_ready_goal()) == []
    node.send_status(command['action'], command['command_id'], 'SUCCEEDED')
    assert node.goal_shot_observe_until == 13.0
    return command


def test_region_is_reselected_after_complete_preparation_and_one_second(goal_node):
    node, clock = goal_node
    prepare(node, clock)
    changed = {**score_ready_goal(), 'depth_m': .54, 'offset_x_px': -30}
    for t in (12.0, 12.5, 12.999):
        clock[0] = t
        assert all(c['action'] == 'WAIT' for c in node.publish_vision(goal=changed))
    clock[0] = 13.0
    command = node.publish_vision(goal=changed)[-1]
    assert command['action'] == 'SHOT'
    assert command['source_command']['shot_region'] == 3
    assert node.shots_completed == 0


@pytest.mark.parametrize('invalid', [None, {'detected': False},
    {**score_ready_goal(), 'depth_valid': False},
    {**score_ready_goal(), 'score_now': False},
    {**score_ready_goal(), 'depth_m': .65},
])
def test_old_region_cannot_authorize_shot_after_preparation(goal_node, invalid):
    node, clock = goal_node
    prepare(node, clock)
    clock[0] = 13.0
    assert all(c['action'] != 'SHOT' for c in node.publish_vision(goal=invalid))


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'REJECTED', 'CANCELLED'])
def test_failed_preparation_never_opens_region_window(goal_node, status):
    node, clock = goal_node
    command = node.publish_vision(goal=score_ready_goal())[-1]
    node.send_status(command['action'], command['command_id'], 'RUNNING')
    node.send_status(command['action'], command['command_id'], status)
    assert node.goal_shot_observe_until is None
    clock[0] = 20.0
    assert all(c['action'] != 'SHOT' for c in node.publish_vision(goal=score_ready_goal()))


@pytest.mark.parametrize('previous', ['goal_camera_90_crab_left', 'goal_camera_90_fine_forward_3'])
def test_real_decision_and_bridge_exchange_preparation_then_new_region(goal_node, previous):
    node, clock = goal_node
    bridge = FakeBridge()
    bridge.last_physical_motion_id = previous
    first = node.publish_vision(goal={**score_ready_goal(), 'depth_m': .48, 'offset_x_px': 30})[-1]
    bridge.navigation_command_callback(string_message(first))
    finish_sequence(bridge)
    for status in bridge.motion_status_publisher.messages:
        MotionDecisionNode._motion_status_callback(node, status)
    assert node.goal_shot_observe_until == 11.0
    assert not any(m['motion_id'].startswith('goal_shot')
                   for m in decoded_messages(bridge.executor_request_publisher))
    clock[0] = 11.0
    shot = node.publish_vision(goal={**score_ready_goal(), 'depth_m': .54, 'offset_x_px': -30})[-1]
    assert shot['action'] == 'SHOT'
    before = len(bridge.motion_status_publisher.messages)
    bridge.navigation_command_callback(string_message(shot))
    assert bridge.active_motion_id == 'goal_shot_region_3'
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    bridge.executor_status_callback(executor_status(**request, status='RUNNING'))
    finish_sequence(bridge)
    for status in bridge.motion_status_publisher.messages[before:]:
        MotionDecisionNode._motion_status_callback(node, status)
    assert node.shots_completed == 1
    assert node.mission_phase == 'POST_SHOT_TURN'


@pytest.mark.parametrize('previous', [
    'goal_fine_prepare_90', 'goal_crab_left_to_fine_90', 'goal_crab_right_to_fine_90',
])
@pytest.mark.parametrize('target,prepare', [
    ('goal_camera_90_crab_left', 'goal_fine_to_crab_left_90'),
    ('goal_camera_90_crab_right', 'goal_fine_to_crab_right_90'),
    ('goal_camera_90_turn_left_1', 'fine_to_turn_ready_90'),
    ('goal_camera_90_forward_4', 'fine_to_turn_ready_90'),
    ('line_recovery_left_4', 'fine_to_turn_ready_90'),
    ('goal_camera_90_fine_forward_3', None),
])
def test_out_of_zone_correction_retains_prepared_support_posture(previous, target, prepare):
    bridge = FakeBridge()
    bridge.last_physical_motion_id = previous
    assert bridge._transition_prepare_motion(target) == prepare
