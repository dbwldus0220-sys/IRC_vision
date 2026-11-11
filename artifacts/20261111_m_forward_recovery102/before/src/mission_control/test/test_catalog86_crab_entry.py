"""Source86 entry data and completion-gated recovery-to-crab transitions."""

import copy
import json
from pathlib import Path

import pytest
import yaml

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from test_motion_command_bridge import (
    FakeBridge, navigation_message, fine_alignment_message, decoded_messages,
    complete_active_motion, finish_correction_checkpoint,
    executor_status,
)
from test_continuous_approach import clock
from test_pickup_continuous_walk import (
    ball, start, forward_status, late_frames, reserved_request, finish_rest,
)
from test_wait_refresh_and_fine_settle import LiveInputHarness
from std_msgs.msg import String


ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261110_catalog86_crab_entry'
NAMES = {'r오뒤에서 오옆꽃게0도', 'r오뒤에서 오옆꽃게90도'}
RECOVERIES = [
    'line_recovery_left_4', 'line_recovery_left_6', 'line_recovery_right_4',
    'line_recovery_ball_first_left_4', 'line_recovery_ball_second_right_4',
]


@pytest.mark.parametrize('kind', ['pc', 'runtime'])
def test_only_requested_entries_are_replaced_with_source86(kind):
    relative = Path(f'artifacts/robot_motions_{kind}.json')
    before = json.loads((UPDATE / 'before' / relative).read_text())
    source = {motion['name']: motion for motion in
              json.loads((UPDATE / 'source_robot_motions.json').read_text())['motions']}
    expected = copy.deepcopy(before)
    for index, motion in enumerate(expected['motions']):
        if motion['name'] in NAMES:
            replacement = copy.deepcopy(source[motion['name']])
            if kind == 'runtime':
                replacement['completion']['position_tolerance_deg'] = 5.0
            expected['motions'][index] = replacement
    after = ROOT / 'artifacts/20261110_video_review_speed1/before' / relative
    assert json.loads(after.read_text()) == expected


@pytest.mark.parametrize('camera', [0, 90])
def test_left_and_right_use_same_source_transition(camera):
    aliases = yaml.safe_load((ROOT / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    identifiers = (['pickup_forward_to_crab_0', 'pickup_forward_to_crab_left_0'] if camera == 0
                   else ['goal_forward_to_crab_right_90', 'goal_forward_to_crab_left_90', 'r_forward_to_crab_90'])
    motions = {motion['name']: motion for motion in
               json.loads((ROOT / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
    for identifier in identifiers:
        assert aliases[identifier] == f'r오뒤에서 오옆꽃게{camera}도'
        motion = motions[aliases[identifier]]
        duration_ms = max(frame['start_ms'] + frame['time_ms'] for frame in motion['frames'])
        assert duration_ms == 165
        assert motion['repeat_count'] == 1
        assert motion['playback_speed'] == 1.0
        assert duration_ms + motion['completion']['settle_timeout_ms'] < Bridge.DEFAULT_TIMEOUT_MS


def crab_message(direction, count, command_id=8001):
    message = fine_alignment_message(f'BALL_PICKUP_INITIAL_CRAB_{direction}', command_id)
    payload = json.loads(message.data)
    payload['source_command'] = {'offset_x_px': (450. if count == 2 else 100.) * (1 if direction == 'RIGHT' else -1)}
    message.data = json.dumps(payload)
    return message


@pytest.mark.parametrize('previous', RECOVERIES)
@pytest.mark.parametrize('direction', ['LEFT', 'RIGHT'])
@pytest.mark.parametrize('count', [1, 2])
def test_recovery_prepares_once_before_crab_and_preserves_pickup_checkpoint(previous, direction, count):
    bridge = FakeBridge()
    bridge.last_physical_motion_id = previous
    bridge.navigation_command_callback(navigation_message(action='PICKUP_NOW'))
    bridge.navigation_command_callback(crab_message(direction, count))
    prepare = 'pickup_forward_to_crab_0' if direction == 'RIGHT' else 'pickup_forward_to_crab_left_0'
    crab = f'pickup_crab_{direction.lower()}_0' + ('_2' if count == 2 else '')
    assert bridge.active_motion_id == prepare
    assert bridge.pending_pickup_crab_motion_id == crab
    assert [request['motion_id'] for request in decoded_messages(bridge.executor_request_publisher)] == [prepare]
    complete_active_motion(bridge)
    assert bridge.active_motion_id == crab
    complete_active_motion(bridge)
    finish_correction_checkpoint(bridge)
    assert bridge.pickup_initial_align_waiting
    bridge.navigation_command_callback(crab_message(direction, count, 8002))
    assert bridge.active_motion_id == crab
    assert [request['motion_id'] for request in decoded_messages(bridge.executor_request_publisher)] == [prepare, crab, crab]


@pytest.mark.parametrize('previous', RECOVERIES)
@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'REJECTED', 'CANCELLED'])
def test_unsuccessful_preparation_never_starts_crab(previous, status):
    bridge = FakeBridge()
    bridge.last_physical_motion_id = previous
    bridge.navigation_command_callback(navigation_message(action='PICKUP_NOW'))
    bridge.navigation_command_callback(crab_message('RIGHT', 1))
    complete_active_motion(bridge, status=status, error_code='TEST_PREPARATION_FAILURE')
    assert [request['motion_id'] for request in decoded_messages(bridge.executor_request_publisher)] == ['pickup_forward_to_crab_0']
    assert not bridge.motion_in_progress
    assert bridge.pending_pickup_crab_motion_id is None
    assert bridge.pickup_queued_request is None
    assert bridge.queued_request_id is None
    assert bridge.pending_turn_request is None
    assert bridge.transition_pending_request is None


@pytest.mark.parametrize('previous', RECOVERIES)
@pytest.mark.parametrize('direction', ['left', 'right'])
def test_goal_recovery_entry_keeps_camera90_transition(previous, direction):
    bridge = FakeBridge()
    bridge.last_physical_motion_id = previous
    assert bridge._transition_prepare_motion(f'goal_camera_90_crab_{direction}') == f'goal_forward_to_crab_{direction}_90'


@pytest.mark.parametrize('index,recovery_side,identifier,name', [
    (1, 'LEFT', 'line_recovery_ball_first_left_4', 'p라인복귀좌회전공용'),
    (2, 'RIGHT', 'line_recovery_ball_second_right_4', 't라인복귀우공용'),
])
@pytest.mark.parametrize('direction', ['LEFT', 'RIGHT'])
def test_common_recovery_finishes_before_queued_pickup_prepares_crab(index, recovery_side, identifier, name, direction):
    aliases = yaml.safe_load((ROOT / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    assert aliases[identifier] == name
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        command_id=7999, action=f'BALL_APPROACH_RECOVER_{recovery_side}_4',
        source='ball', phase='BALL_APPROACH',
        mission_progress={'active_ball_mode_index': index},
    ))
    assert bridge.active_motion_id == identifier
    bridge.navigation_command_callback(navigation_message(action='PICKUP_NOW'))
    assert bridge.queued_request_deferred
    assert bridge.queued_command_id == 8000
    assert len(bridge.executor_request_publisher.messages) == 1
    complete_active_motion(bridge)
    assert bridge.last_physical_motion_id == identifier
    assert bridge.queued_request_id is None
    assert bridge.pickup_initial_align_waiting
    bridge.navigation_command_callback(crab_message(direction, 1))
    prepare_request = decoded_messages(bridge.executor_request_publisher)[-1]
    prepare = 'pickup_forward_to_crab_0' if direction == 'RIGHT' else 'pickup_forward_to_crab_left_0'
    assert prepare_request['motion_id'] == prepare
    bridge.navigation_command_callback(navigation_message(action='STRAIGHT', command_id=9000))
    assert decoded_messages(bridge.motion_status_publisher)[-1]['error_code'] == 'ATOMIC_SEQUENCE_LOCKED'
    assert bridge.queued_request_id is None
    complete_active_motion(bridge)
    crab = f'pickup_crab_{direction.lower()}_0'
    assert bridge.active_motion_id == crab
    bridge.executor_status_callback(executor_status(**prepare_request, status='SUCCEEDED'))
    assert bridge.active_motion_id == crab
    complete_active_motion(bridge)
    finish_correction_checkpoint(bridge)
    assert bridge.pickup_initial_align_waiting
    assert [request['motion_id'] for request in decoded_messages(bridge.executor_request_publisher)] == [identifier, prepare, crab]


@pytest.mark.parametrize('previous', RECOVERIES[-2:])
@pytest.mark.parametrize('direction', ['left', 'right'])
def test_common_recovery_preparation_then_crab_reservation_releases_normally(clock, previous, direction):
    node = LiveInputHarness(phase='BALL_APPROACH')
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = previous
    sample = ball(offset_x_px=-100 if direction == 'left' else 100)
    parent, preparation = start(node, bridge, clock, sample)
    assert preparation['motion_id'] in {'pickup_forward_to_crab_0', 'pickup_forward_to_crab_left_0'}
    assert node.pickup_walk_reservation.key is None
    assert bridge.pickup_queued_request is None
    forward_status(node, bridge, preparation, 'SUCCEEDED')
    finish_rest(bridge, clock)
    first = decoded_messages(bridge.executor_request_publisher)[-1]
    assert first['motion_id'] == f'pickup_crab_{direction}_0'
    forward_status(node, bridge, first, 'RUNNING')
    late_frames(node, clock, sample)
    queued_command = node.publisher.messages[-1]
    assert queued_command['source_command']['pickup_walk_prequeue']
    bridge.navigation_command_callback(String(data=json.dumps(queued_command)))
    second = reserved_request(bridge)
    assert second['command_id'] == parent['command_id']
    assert second['request_id'] != first['request_id']
    forward_status(node, bridge, first, 'SUCCEEDED')
    finish_rest(bridge, clock)
    forward_status(node, bridge, second, 'RUNNING')
    forward_status(node, bridge, second, 'SUCCEEDED')
    assert bridge.pickup_initial_align_waiting
    assert bridge.pickup_queued_request is None
    assert bridge.pending_pickup_crab_motion_id is None
    assert not node.pickup_walk_reservation.pending
    assert [request['motion_id'] for request in decoded_messages(bridge.executor_request_publisher)] == [preparation['motion_id'], first['motion_id'], first['motion_id']]
