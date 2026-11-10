"""Replay two-cycle hurdle walking, fresh-input gates, and the final reservation."""

import json
from pathlib import Path

import pytest
import yaml
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_continuous_approach import clock
from test_hurdle_positioning import HurdleHarness, hurdle
from test_motion_command_bridge import FakeBridge, decoded_messages, executor_status
from test_pickup_continuous_walk import forward_status

ROOT = Path(__file__).resolve().parents[3]
ALIASES = yaml.safe_load((ROOT / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
MOTIONS = {m['name']: m for m in json.loads((ROOT / 'artifacts/robot_motions_runtime.json').read_text())['motions']}


def duration(motion_id):
    motion = MOTIONS[ALIASES[motion_id]]
    return max(f['start_ms'] + f['time_ms'] for f in motion['frames']) / 1000 * motion['repeat_count'] / motion['playback_speed']


def observe(node, clock, depth=.60, delay=0., **updates):
    node.receive({**hurdle(depth), 'rgb_stamp_ns': round((clock[0] - delay) * 1e9), **updates})


def start(clock):
    node = HurdleHarness()
    node.pending_hurdle_final_sequence = None
    node.FINE_FORWARD_PRE_MOTION_SETTLE_SEC = Node.FINE_FORWARD_PRE_MOTION_SETTLE_SEC
    observe(node, clock)
    assert node.publish() == []
    clock[0] += .5
    observe(node, clock)
    command = node.publish()[-1]
    assert command['action'] == 'STRAIGHT_1' and command['source'] == 'hurdle'
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = 'line_forward_4'
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request['motion_id'] == 'line_forward_2'
    forward_status(node, bridge, request, 'RUNNING')
    return node, bridge, request, clock[0]


def finish_walk(node, bridge, request, clock, started):
    clock[0] = started + duration(request['motion_id'])
    forward_status(node, bridge, request, 'SUCCEEDED')
    assert not node.general_motion_gate.locked
    assert node.latest_info['hurdle'] is None
    clock[0] += .499
    assert node.publish() == []
    clock[0] += .001
    node.publish()  # Discard pre-settle inputs before the next decision.


@pytest.mark.parametrize('delay', [0., .20, .35])
def test_forward2_repeats_only_after_completion_settle_and_new_valid_frame(clock, delay):
    node, bridge, request, started = start(clock)
    for fraction in (.72, .82, .92):
        clock[0] = started + duration('line_forward_2') * fraction
        observe(node, clock, .53, delay)
        assert node.publish() == []
        assert getattr(node, 'queued_general_command_id', None) is None
        assert bridge.queued_request_id is None
    finish_walk(node, bridge, request, clock, started)
    assert not any(command['valid'] for command in node.publish())
    observe(node, clock, .53, delay)
    command = node.publish()[-1]
    assert command['action'] == 'STRAIGHT_1' and command['valid']
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    assert [r['motion_id'] for r in decoded_messages(bridge.executor_request_publisher)] == ['line_forward_2'] * 2


@pytest.mark.parametrize('depth', [.450, .449, .40])
@pytest.mark.parametrize('delay', [0., .20, .35])
def test_crossing_reserves_final_sequence_once_even_when_hurdle_disappears(clock, depth, delay):
    node, bridge, request, started = start(clock)
    clock[0] = started + duration('line_forward_2') * .8
    observe(node, clock, depth, delay)
    assert node.pending_hurdle_final_sequence['observed_during_action'] == 'STRAIGHT_1'
    observe(node, clock, detected=False, raw_detected=False)
    assert node.publish() == []
    assert getattr(node, 'queued_general_command_id', None) is None
    finish_walk(node, bridge, request, clock, started)
    go = node.publish()[-1]
    assert go['action'] == 'GO' and go['source_command']['hurdle_final_sequence_latched']
    bridge.navigation_command_callback(String(data=json.dumps(go)))
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == 'pickup_fine_prepare'
    # Preparation is required when the final fine gait follows ordinary walking.
    forwarded = len(bridge.motion_status_publisher.messages)
    for _ in range(20):
        if not bridge.motion_in_progress:
            break
        deadline = bridge.transition_dwell_until or bridge.active_dwell_until
        if deadline is not None:
            clock[0] = max(clock[0], deadline)
            bridge._check_atomic_dwell()
        else:
            current = decoded_messages(bridge.executor_request_publisher)[-1]
            bridge.executor_status_callback(executor_status(**current, status='RUNNING'))
            clock[0] += duration(current['motion_id'])
            bridge.executor_status_callback(executor_status(**current, status='SUCCEEDED'))
        for message in bridge.motion_status_publisher.messages[forwarded:]:
            Node._motion_status_callback(node, message)
        forwarded = len(bridge.motion_status_publisher.messages)
    assert not bridge.motion_in_progress
    assert node.mission_phase == 'LINE_TRACK' and node.phase_manager.hurdles_completed == 1
    ids = [r['motion_id'] for r in decoded_messages(bridge.executor_request_publisher)]
    assert ids == ['line_forward_2', 'pickup_fine_prepare', 'hurdle_fine_forward_10', 'hurdle']
    assert node.pending_hurdle_final_sequence is None
    assert node.active_special_command_id is None


@pytest.mark.parametrize('updates', [
    {'detected': False, 'raw_detected': True, 'confirmation_confirmed': False},
    {'detected': False, 'raw_detected': False},
    {'depth_valid': False}, {'depth_m': None}, {'depth_m': float('nan')},
])
def test_missing_invalid_or_unconfirmed_input_holds_then_recovers(clock, updates):
    node, bridge, request, started = start(clock)
    clock[0] = started + .8
    observe(node, clock, .44, **updates)
    assert node.pending_hurdle_final_sequence is None
    finish_walk(node, bridge, request, clock, started)
    observe(node, clock, .44, **updates)
    assert not any(command['valid'] for command in node.publish())
    observe(node, clock, .53)
    assert node.publish()[-1]['action'] == 'STRAIGHT_1'


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED'])
def test_failed_forward_discards_final_reservation_and_never_jumps_on_old_input(clock, status):
    node, bridge, request, started = start(clock)
    clock[0] = started + .8
    observe(node, clock, .44)
    assert node.pending_hurdle_final_sequence is not None
    forward_status(node, bridge, request, status, error_code='TEST_FAILURE')
    assert node.pending_hurdle_final_sequence is None
    observe(node, clock, detected=False, raw_detected=False)
    clock[0] += 1.
    assert not any(c['valid'] and c['action'] == 'GO' for c in node.publish())
    assert node.phase_manager.hurdles_completed == 0


def test_forward2_catalog_matches_existing_normal_walk():
    assert ALIASES['line_forward_2'] == 'p전진45(2회)'
    motion = MOTIONS[ALIASES['line_forward_2']]
    assert motion['repeat_count'] == 2
    assert motion['playback_speed'] == 1.
    assert duration('line_forward_2') == pytest.approx(1.028)
