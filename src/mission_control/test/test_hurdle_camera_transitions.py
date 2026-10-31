"""Match Hurdle support transitions to the executor's camera-down latch."""

import copy
import json
from pathlib import Path

import pytest
import yaml

from test_motion_command_bridge import (
    FakeBridge, decoded_messages, executor_status, navigation_message, string_message,
)
from test_transition_dwells import check_both_pauses, clock


ROOT = Path(__file__).resolve().parents[3]
UPDATE = ROOT / 'artifacts/20261031_hurdle_camera_transitions'


def camera_state(bridge, active):
    bridge.executor_heartbeat_callback(string_message({'hurdle_head_override_active': active}))


@pytest.mark.parametrize('previous', ['line_forward_4', 'stationary_turn_left', 'line_recovery_right_4'])
@pytest.mark.parametrize('action,target', [
    ('STRAIGHT_0', 'hurdle_fine_forward_5'), ('GO', 'hurdle_fine_forward_10'),
])
@pytest.mark.parametrize('down', [True, False])
def test_hurdle_entry_uses_current_camera_state(clock, previous, action, target, down):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = previous
    camera_state(bridge, down)
    bridge.navigation_command_callback(navigation_message(source='hurdle', action=action))
    prepare = 'hurdle_fine_prepare_0' if down else 'pickup_fine_prepare'
    check_both_pauses(bridge, clock, prepare, target)


@pytest.mark.parametrize('previous', ['hurdle_fine_forward_5', 'hurdle_fine_forward_10'])
@pytest.mark.parametrize('action,target', [
    ('ALIGN_LEFT', 'post_ball_line_turn_left_2'),
    ('ALIGN_RIGHT', 'post_ball_line_turn_right_2'),
    ('STRAIGHT', 'line_forward_4'),
])
@pytest.mark.parametrize('down', [True, False])
def test_hurdle_exit_uses_current_camera_state(clock, previous, action, target, down):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = previous
    camera_state(bridge, down)
    bridge.navigation_command_callback(navigation_message(
        source='hurdle', action=action, source_command={'turn_count': 2},
    ))
    check_both_pauses(bridge, clock, 'fine_to_turn_ready_0' if down else 'fine_to_turn_ready_45', target)


@pytest.mark.parametrize('active,age,expected', [
    (True, 1.0, True), (False, 1.0, False),
    (True, 1.001, None), (False, 1.001, None),
    (None, 0, None), ('false', 0, None), (0, 0, None), (1, 0, None),
])
def test_missing_stale_or_invalid_status_keeps_prior_fallback(clock, active, age, expected):
    bridge = FakeBridge()
    camera_state(bridge, active)
    clock[0] += age
    assert bridge._hurdle_camera_down_state() is expected
    bridge.last_physical_motion_id = 'line_forward_4'
    assert bridge._transition_prepare_motion('hurdle_fine_forward_5') == (
        'hurdle_fine_prepare_0' if expected is True else 'pickup_fine_prepare')
    bridge.last_physical_motion_id = 'hurdle_fine_forward_5'
    assert bridge._transition_prepare_motion('stationary_turn_left') == (
        'fine_to_turn_ready_45' if expected is False else 'fine_to_turn_ready_0')


def test_camera_latch_update_before_queued_turn_selects_zero_exit(clock):
    bridge = FakeBridge(advance_transition_dwells=False)
    camera_state(bridge, False)
    bridge.navigation_command_callback(navigation_message(source='hurdle', action='STRAIGHT_0', command_id=1))
    fine = decoded_messages(bridge.executor_request_publisher)[-1]
    bridge.navigation_command_callback(navigation_message(
        source='hurdle', action='ALIGN_LEFT', command_id=2, source_command={'turn_count': 2},
    ))
    assert bridge.queued_request_deferred
    # The pixel-triggered latch arrives while the previous gait is still running.
    camera_state(bridge, True)
    bridge.executor_status_callback(executor_status(**fine, status='SUCCEEDED'))
    check_both_pauses(bridge, clock, 'fine_to_turn_ready_0', 'post_ball_line_turn_left_2')


@pytest.mark.parametrize('down', [True, False])
@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED'])
@pytest.mark.parametrize('entry', [True, False])
def test_camera_selected_preparation_failure_never_sends_target(clock, down, status, entry):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = 'line_forward_4' if entry else 'hurdle_fine_forward_5'
    camera_state(bridge, down)
    bridge.navigation_command_callback(navigation_message(
        source='hurdle', action='STRAIGHT_0' if entry else 'ALIGN_LEFT',
        source_command={'turn_count': 2},
    ))
    clock[0] += .5
    bridge._check_atomic_dwell()
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    bridge.executor_status_callback(executor_status(**request, status=status, error_code='TEST_FAILURE'))
    clock[0] += 10
    bridge._check_atomic_dwell()
    assert decoded_messages(bridge.executor_request_publisher) == [request]
    assert not bridge.motion_in_progress


@pytest.mark.parametrize('filename', ['robot_motions_pc.json', 'robot_motions_runtime.json'])
def test_only_camera_zero_entry_data_added_with_once_only_speed_scaling(filename):
    before = json.loads((UPDATE / 'before/artifacts' / filename).read_text())
    snapshot = ROOT / 'artifacts/20261031_transition_table_revision/before'
    actual = json.loads((snapshot / 'artifacts' / filename).read_text())
    expected = copy.deepcopy(json.loads((UPDATE / 'source_requested_motion.json').read_text()))
    if 'runtime' in filename:
        expected['playback_speed'] = .95
        expected['completion']['position_tolerance_deg'] = 5.0
    assert actual == {**before, 'motions': [*before['motions'], expected]}
    alias_path = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
    old_aliases = yaml.safe_load((UPDATE / 'before' / alias_path).read_text())['motion_aliases']
    aliases = yaml.safe_load((snapshot / alias_path).read_text())['motion_aliases']
    assert aliases == {**old_aliases, 'hurdle_fine_prepare_0': expected['name']}
